#!/usr/bin/env python3
"""Test harness comparing Python vs Zig motion lookahead output.

Feeds identical move sequences through both paths and verifies the
velocity profiles match. This catches math discrepancies between the
Python LookAheadQueue and Zig LookAheadQueue before deploying to hardware.

Usage:
    python3 test/test_native_motion.py
    python3 -m pytest test/test_native_motion.py -v
"""

import math
import os
import sys

# Direct import of chelper without going through klippy package
# (klippy.py shadows the klippy package on import)
_klippy_dir = os.path.join(os.path.dirname(__file__), "..", "klippy")
sys.path.insert(0, _klippy_dir)

_chelper_mod = None


def get_zig_ffi():
    """Load the Zig motion engine library."""
    global _chelper_mod
    if _chelper_mod is None:
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "chelper",
            os.path.join(_klippy_dir, "chelper", "__init__.py"),
        )
        _chelper_mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(_chelper_mod)

    me_ffi, me_lib = _chelper_mod.get_motion_ffi()
    if me_lib is None:
        raise RuntimeError(
            "Zig motion engine not available. "
            "Build with: cd klippy/chelper/zig_engine && zig build"
        )
    return me_ffi, me_lib


# ── Python reference implementation (from toolhead.py) ──


class PyMove:
    """Minimal copy of toolhead.Move for testing."""

    def __init__(self, start_pos, end_pos, speed, max_velocity, max_accel,
                 max_accel_to_decel, junction_deviation):
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        self.accel = max_accel
        self.junction_deviation = junction_deviation
        self.timing_callbacks = []
        velocity = min(speed, max_velocity)
        self.is_kinematic_move = True
        self.axes_d = [end_pos[i] - start_pos[i] for i in (0, 1, 2, 3)]
        self.move_d = math.sqrt(sum([d * d for d in self.axes_d[:3]]))
        if self.move_d < 0.000000001:
            self.end_pos = (start_pos[0], start_pos[1], start_pos[2],
                            end_pos[3])
            self.axes_d[0] = self.axes_d[1] = self.axes_d[2] = 0.0
            self.move_d = abs(self.axes_d[3])
            inv_move_d = 0.0
            if self.move_d:
                inv_move_d = 1.0 / self.move_d
            self.accel = 99999999.9
            velocity = speed
            self.is_kinematic_move = False
        else:
            inv_move_d = 1.0 / self.move_d
        self.axes_r = [d * inv_move_d for d in self.axes_d]
        self.min_move_t = self.move_d / velocity
        self.max_start_v2 = 0.0
        self.max_cruise_v2 = velocity ** 2
        self.delta_v2 = 2.0 * self.move_d * self.accel
        self.max_smoothed_v2 = 0.0
        self.smooth_delta_v2 = 2.0 * self.move_d * max_accel_to_decel
        self.next_junction_v2 = 999999999.9
        self.start_v = self.cruise_v = self.end_v = 0.0
        self.accel_t = self.cruise_t = self.decel_t = 0.0

    def calc_junction(self, prev_move):
        if not self.is_kinematic_move or not prev_move.is_kinematic_move:
            return
        max_start_v2 = min(
            self.max_cruise_v2, prev_move.max_cruise_v2,
            prev_move.next_junction_v2,
            prev_move.max_start_v2 + prev_move.delta_v2,
        )
        axes_r = self.axes_r
        prev_axes_r = prev_move.axes_r
        junction_cos_theta = -(
            axes_r[0] * prev_axes_r[0] + axes_r[1] * prev_axes_r[1]
            + axes_r[2] * prev_axes_r[2]
        )
        sin_theta_d2 = math.sqrt(max(0.5 * (1.0 - junction_cos_theta), 0.0))
        cos_theta_d2 = math.sqrt(max(0.5 * (1.0 + junction_cos_theta), 0.0))
        one_minus_sin_theta_d2 = 1.0 - sin_theta_d2
        if one_minus_sin_theta_d2 > 0.0 and cos_theta_d2 > 0.0:
            R_jd = sin_theta_d2 / one_minus_sin_theta_d2
            move_jd_v2 = R_jd * self.junction_deviation * self.accel
            pmove_jd_v2 = R_jd * prev_move.junction_deviation * prev_move.accel
            quarter_tan_theta_d2 = 0.25 * sin_theta_d2 / cos_theta_d2
            move_centripetal_v2 = self.delta_v2 * quarter_tan_theta_d2
            pmove_centripetal_v2 = prev_move.delta_v2 * quarter_tan_theta_d2
            max_start_v2 = min(
                max_start_v2, move_jd_v2, pmove_jd_v2,
                move_centripetal_v2, pmove_centripetal_v2,
            )
        self.max_start_v2 = max_start_v2
        self.max_smoothed_v2 = min(
            max_start_v2, prev_move.max_smoothed_v2 + prev_move.smooth_delta_v2
        )

    def set_junction(self, start_v2, cruise_v2, end_v2):
        half_inv_accel = 0.5 / self.accel
        accel_d = (cruise_v2 - start_v2) * half_inv_accel
        decel_d = (cruise_v2 - end_v2) * half_inv_accel
        self.start_v = math.sqrt(start_v2)
        self.cruise_v = math.sqrt(cruise_v2)
        self.end_v = math.sqrt(end_v2)
        self.accel_t = accel_d / ((self.start_v + self.cruise_v) * 0.5)
        self.cruise_t = (self.move_d - accel_d - decel_d) / self.cruise_v
        self.decel_t = decel_d / ((self.end_v + self.cruise_v) * 0.5)


LOOKAHEAD_FLUSH_TIME = 0.250


def py_flush(moves):
    """Python LookAheadQueue.flush() — reference implementation."""
    # Add junctions
    for i in range(1, len(moves)):
        moves[i].calc_junction(moves[i - 1])
    # Backward pass (same as LookAheadQueue.flush with lazy=False)
    flush_count = len(moves)
    next_end_v2 = next_smoothed_v2 = peak_cruise_v2 = 0.0
    delayed = []
    for i in range(flush_count - 1, -1, -1):
        move = moves[i]
        reachable_start_v2 = next_end_v2 + move.delta_v2
        start_v2 = min(move.max_start_v2, reachable_start_v2)
        reachable_smoothed_v2 = next_smoothed_v2 + move.smooth_delta_v2
        smoothed_v2 = min(move.max_smoothed_v2, reachable_smoothed_v2)
        if smoothed_v2 < reachable_smoothed_v2:
            if (smoothed_v2 + move.smooth_delta_v2 > next_smoothed_v2
                    or delayed):
                peak_cruise_v2 = min(
                    move.max_cruise_v2,
                    (smoothed_v2 + reachable_smoothed_v2) * 0.5,
                )
                if delayed:
                    mc_v2 = peak_cruise_v2
                    for m, ms_v2, me_v2 in reversed(delayed):
                        mc_v2 = min(mc_v2, ms_v2)
                        m.set_junction(
                            min(ms_v2, mc_v2), mc_v2, min(me_v2, mc_v2)
                        )
                    del delayed[:]
            cruise_v2 = min(
                (start_v2 + reachable_start_v2) * 0.5,
                move.max_cruise_v2, peak_cruise_v2,
            )
            move.set_junction(
                min(start_v2, cruise_v2), cruise_v2,
                min(next_end_v2, cruise_v2),
            )
        else:
            delayed.append((move, start_v2, next_end_v2))
        next_end_v2 = start_v2
        next_smoothed_v2 = smoothed_v2


# ── Test helpers ──


def make_move_params(start_pos, end_pos, speed, max_velocity=500.0,
                     max_accel=3000.0, min_cruise_ratio=0.5,
                     square_corner_velocity=5.0):
    """Create parameters for both Python and Zig move creation."""
    max_accel_to_decel = max_accel * (1.0 - min_cruise_ratio)
    scv2 = square_corner_velocity ** 2
    junction_deviation = scv2 * (math.sqrt(2.0) - 1.0) / max_accel
    return {
        "start_pos": start_pos,
        "end_pos": end_pos,
        "speed": speed,
        "max_velocity": max_velocity,
        "max_accel": max_accel,
        "max_accel_to_decel": max_accel_to_decel,
        "junction_deviation": junction_deviation,
    }


def run_comparison(move_params_list, max_velocity=500.0, max_accel=3000.0,
                   min_cruise_ratio=0.5, square_corner_velocity=5.0,
                   tolerance=1e-9):
    """Run the same moves through Python and Zig, compare results."""
    me_ffi, me_lib = get_zig_ffi()

    # ── Python path ──
    py_moves = []
    for p in move_params_list:
        py_moves.append(PyMove(**p))
    py_flush(py_moves)
    py_results = []
    for m in py_moves:
        py_results.append({
            "start_v": m.start_v,
            "cruise_v": m.cruise_v,
            "end_v": m.end_v,
            "accel_t": m.accel_t,
            "cruise_t": m.cruise_t,
            "decel_t": m.decel_t,
        })

    # ── Zig path ──
    # Feed the SAME post-construction parameters that _native_move sends.
    # Create PyMove objects to get the actual computed values.
    eng = me_lib.motion_engine_create()
    me_lib.motion_engine_set_velocity_limits(
        eng, max_velocity, max_accel, square_corner_velocity, min_cruise_ratio
    )
    for i, p in enumerate(move_params_list):
        m = py_moves[i]  # Use the already-constructed Python Move
        me_lib.motion_engine_queue_move_ex(
            eng,
            m.start_pos[0], m.start_pos[1],
            m.start_pos[2], m.start_pos[3],
            m.end_pos[0], m.end_pos[1],
            m.end_pos[2], m.end_pos[3],
            p["speed"],
            m.accel,
            m.max_cruise_v2,
            m.delta_v2,
            m.smooth_delta_v2,
            m.next_junction_v2,
            1 if m.is_kinematic_move else 0,
        )
    results_buf = me_ffi.new("struct FlushedMoveResult[4096]")
    count = me_lib.motion_engine_flush_and_extract(
        eng, results_buf, len(move_params_list), 0
    )
    zig_results = []
    for i in range(count):
        r = results_buf[i]
        zig_results.append({
            "start_v": r.start_v,
            "cruise_v": r.cruise_v,
            "end_v": r.end_v,
            "accel_t": r.accel_t,
            "cruise_t": r.cruise_t,
            "decel_t": r.decel_t,
        })
    me_lib.motion_engine_destroy(eng)

    # ── Compare ──
    assert len(py_results) == len(zig_results), (
        f"Move count mismatch: Python={len(py_results)}, Zig={len(zig_results)}"
    )
    mismatches = []
    for i, (py, zig) in enumerate(zip(py_results, zig_results)):
        for key in py:
            diff = abs(py[key] - zig[key])
            if diff > tolerance:
                mismatches.append(
                    f"  Move {i} {key}: Python={py[key]:.12f} "
                    f"Zig={zig[key]:.12f} diff={diff:.2e}"
                )
    return py_results, zig_results, mismatches


# ── Test cases ──


def test_single_move():
    """Single move — should produce identical velocity profile."""
    params = [make_move_params([0, 0, 0, 0], [10, 0, 0, 0], 100.0)]
    py, zig, mismatches = run_comparison(params)
    assert not mismatches, "Single move mismatch:\n" + "\n".join(mismatches)
    assert py[0]["cruise_v"] > 0


def test_collinear_moves():
    """Collinear moves — junctions should allow high speed."""
    params = []
    for i in range(10):
        params.append(make_move_params(
            [i * 10, 0, 0, 0], [(i + 1) * 10, 0, 0, 0], 200.0
        ))
    py, zig, mismatches = run_comparison(params)
    assert not mismatches, "Collinear mismatch:\n" + "\n".join(mismatches)
    # Middle moves should have non-zero start velocity (junction optimization)
    assert py[5]["start_v"] > 0


def test_right_angle_corners():
    """90-degree corners — junctions should limit speed."""
    params = [
        make_move_params([0, 0, 0, 0], [50, 0, 0, 0], 200.0),
        make_move_params([50, 0, 0, 0], [50, 50, 0, 0], 200.0),
        make_move_params([50, 50, 0, 0], [0, 50, 0, 0], 200.0),
        make_move_params([0, 50, 0, 0], [0, 0, 0, 0], 200.0),
    ]
    py, zig, mismatches = run_comparison(params)
    assert not mismatches, "Corner mismatch:\n" + "\n".join(mismatches)
    # Junction speed at 90-degree corner should be much lower than cruise
    assert py[1]["start_v"] < py[0]["cruise_v"] * 0.5


def test_acute_angle():
    """Sharp 45-degree reversal."""
    params = [
        make_move_params([0, 0, 0, 0], [50, 50, 0, 0], 300.0),
        make_move_params([50, 50, 0, 0], [100, 0, 0, 0], 300.0),
    ]
    py, zig, mismatches = run_comparison(params)
    assert not mismatches, "Acute angle mismatch:\n" + "\n".join(mismatches)


def test_extrude_only():
    """Extrude-only moves (no XYZ movement)."""
    params = [
        make_move_params([10, 20, 30, 0], [10, 20, 30, 5], 50.0),
        make_move_params([10, 20, 30, 5], [10, 20, 30, 10], 50.0),
    ]
    py, zig, mismatches = run_comparison(params)
    assert not mismatches, "Extrude-only mismatch:\n" + "\n".join(mismatches)


def test_mixed_kinematic_and_extrude():
    """XYZ moves with extrusion."""
    params = [
        make_move_params([0, 0, 0, 0], [50, 0, 0, 2.5], 100.0),
        make_move_params([50, 0, 0, 2.5], [100, 0, 0, 5.0], 100.0),
        make_move_params([100, 0, 0, 5.0], [100, 50, 0, 7.5], 100.0),
    ]
    py, zig, mismatches = run_comparison(params)
    assert not mismatches, "Mixed move mismatch:\n" + "\n".join(mismatches)


def test_z_moves_with_limit():
    """Z-axis moves with lower speed/accel limits."""
    params = []
    for i in range(5):
        p = make_move_params(
            [50, 50, i * 10, 0], [50, 50, (i + 1) * 10, 0], 5.0,
            max_velocity=500.0, max_accel=3000.0,
        )
        params.append(p)
    py, zig, mismatches = run_comparison(params)
    assert not mismatches, "Z move mismatch:\n" + "\n".join(mismatches)


def test_many_short_segments():
    """Many short segments (circle approximation) — stresses lookahead."""
    import math as m
    params = []
    n = 100
    r = 50.0
    cx, cy = 90.0, 90.0
    for i in range(n):
        a0 = 2 * m.pi * i / n
        a1 = 2 * m.pi * (i + 1) / n
        params.append(make_move_params(
            [cx + r * m.cos(a0), cy + r * m.sin(a0), 0, 0],
            [cx + r * m.cos(a1), cy + r * m.sin(a1), 0, 0],
            200.0,
        ))
    py, zig, mismatches = run_comparison(params)
    assert not mismatches, (
        f"Circle mismatch ({len(mismatches)} diffs):\n"
        + "\n".join(mismatches[:20])
    )


def test_high_speed_zigzag():
    """High-speed zigzag pattern — the real-world stress case."""
    params = []
    pos = [10, 10, 0, 0]
    for i in range(50):
        x = 10 + (i % 2) * 160
        y = 10 + ((i // 2) % 160)
        end = [x, y, 0, 0]
        if pos[:3] != end[:3]:
            params.append(make_move_params(list(pos), end, 500.0))
            pos = end
    py, zig, mismatches = run_comparison(params)
    assert not mismatches, (
        f"Zigzag mismatch ({len(mismatches)} diffs):\n"
        + "\n".join(mismatches[:20])
    )


def test_different_accel_settings():
    """Various accel/velocity/scv combinations."""
    configs = [
        {"max_velocity": 100, "max_accel": 1000, "square_corner_velocity": 2},
        {"max_velocity": 800, "max_accel": 10000, "square_corner_velocity": 8},
        {"max_velocity": 300, "max_accel": 5000, "square_corner_velocity": 5},
    ]
    for cfg in configs:
        params = [
            make_move_params([0, 0, 0, 0], [30, 0, 0, 0], 200.0, **cfg),
            make_move_params([30, 0, 0, 0], [30, 30, 0, 0], 200.0, **cfg),
            make_move_params([30, 30, 0, 0], [0, 30, 0, 0], 200.0, **cfg),
        ]
        py, zig, mismatches = run_comparison(
            params, tolerance=1e-9, **cfg
        )
        assert not mismatches, (
            f"Config {cfg} mismatch:\n" + "\n".join(mismatches)
        )


# ── Main ──


def main():
    tests = [
        test_single_move,
        test_collinear_moves,
        test_right_angle_corners,
        test_acute_angle,
        test_extrude_only,
        test_mixed_kinematic_and_extrude,
        test_z_moves_with_limit,
        test_many_short_segments,
        test_high_speed_zigzag,
        test_different_accel_settings,
    ]
    passed = 0
    failed = 0
    for test in tests:
        name = test.__name__
        try:
            test()
            print(f"  PASS  {name}")
            passed += 1
        except Exception as e:
            print(f"  FAIL  {name}: {e}")
            failed += 1
    print(f"\n{passed} passed, {failed} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
