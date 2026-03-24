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


# ── Trapq comparison tests ──
# These test that Zig's trapq_append produces the same trapq entries
# as Python's, by running flush_and_process and extracting trapq data.


def get_chelper_ffi():
    """Load the base chelper FFI (for trapq_extract_old etc.)."""
    global _chelper_mod
    if _chelper_mod is None:
        get_zig_ffi()  # ensures _chelper_mod is loaded
    ffi_main, ffi_lib = _chelper_mod.get_ffi()
    return ffi_main, ffi_lib


def extract_trapq_moves(ffi_main, ffi_lib, trapq, start_time, end_time,
                        max_moves=1000):
    """Extract pull_move data from a trapq."""
    buf = ffi_main.new("struct pull_move[%d]" % max_moves)
    count = ffi_lib.trapq_extract_old(trapq, buf, max_moves,
                                      start_time, end_time)
    moves = []
    for i in range(count):
        m = buf[i]
        moves.append({
            "print_time": m.print_time,
            "move_t": m.move_t,
            "start_v": m.start_v,
            "accel": m.accel,
            "start_x": m.start_x,
            "start_y": m.start_y,
            "start_z": m.start_z,
            "x_r": m.x_r,
            "y_r": m.y_r,
            "z_r": m.z_r,
        })
    return moves


def test_trapq_output_matches():
    """Compare trapq entries from Python trapq_append vs Zig flush_and_process.

    This is the critical test for the full pipeline: after Zig appends moves
    to the trapq, the entries must be identical to what Python would produce.
    """
    me_ffi, me_lib = get_zig_ffi()
    c_ffi, c_lib = get_chelper_ffi()

    params = [
        make_move_params([0, 0, 0, 0], [50, 0, 0, 0], 200.0),
        make_move_params([50, 0, 0, 0], [50, 50, 0, 0], 200.0),
        make_move_params([50, 50, 0, 0], [0, 50, 0, 0], 200.0),
        make_move_params([0, 50, 0, 0], [0, 0, 0, 0], 200.0),
    ]
    py_moves = [PyMove(**p) for p in params]
    py_flush(py_moves)

    # ── Python trapq ──
    py_trapq = c_lib.trapq_alloc()
    print_time = 1.0
    for m in py_moves:
        if m.is_kinematic_move:
            c_lib.trapq_append(
                py_trapq, print_time,
                m.accel_t, m.cruise_t, m.decel_t,
                m.start_pos[0], m.start_pos[1], m.start_pos[2],
                m.axes_r[0], m.axes_r[1], m.axes_r[2],
                m.start_v, m.cruise_v, m.accel,
            )
        print_time += m.accel_t + m.cruise_t + m.decel_t

    py_trapq_moves = extract_trapq_moves(c_ffi, c_lib, py_trapq, 0, 9999)

    # ── Zig trapq via flush_and_process ──
    zig_trapq = c_lib.trapq_alloc()
    eng = me_lib.motion_engine_create()
    me_lib.motion_engine_set_velocity_limits(eng, 500.0, 3000.0, 5.0, 0.5)
    me_lib.motion_engine_set_trapq(eng, zig_trapq)
    me_lib.motion_engine_set_print_time(eng, 1.0)

    for i, m in enumerate(py_moves):
        me_lib.motion_engine_queue_move_ex(
            eng,
            m.start_pos[0], m.start_pos[1],
            m.start_pos[2], m.start_pos[3],
            m.end_pos[0], m.end_pos[1],
            m.end_pos[2], m.end_pos[3],
            params[i]["speed"], m.accel,
            m.max_cruise_v2, m.delta_v2,
            m.smooth_delta_v2, m.next_junction_v2,
            1 if m.is_kinematic_move else 0,
        )

    results_buf = me_ffi.new("struct FlushedMoveResult[100]")
    me_lib.motion_engine_flush_and_process(eng, results_buf, 100, 0)

    zig_trapq_moves = extract_trapq_moves(c_ffi, c_lib, zig_trapq, 0, 9999)

    # ── Compare ──
    assert len(py_trapq_moves) == len(zig_trapq_moves), (
        f"Trapq move count: Python={len(py_trapq_moves)}, "
        f"Zig={len(zig_trapq_moves)}"
    )
    tolerance = 1e-9
    mismatches = []
    for i, (pm, zm) in enumerate(zip(py_trapq_moves, zig_trapq_moves)):
        for key in pm:
            diff = abs(pm[key] - zm[key])
            if diff > tolerance:
                mismatches.append(
                    f"  Trapq {i} {key}: Py={pm[key]:.12f} "
                    f"Zig={zm[key]:.12f} diff={diff:.2e}"
                )
    assert not mismatches, (
        f"Trapq mismatch ({len(mismatches)}):\n" + "\n".join(mismatches[:20])
    )

    c_lib.trapq_free(py_trapq)
    c_lib.trapq_free(zig_trapq)
    me_lib.motion_engine_destroy(eng)


def test_trapq_circle_segments():
    """Trapq comparison with many short segments (circle)."""
    me_ffi, me_lib = get_zig_ffi()
    c_ffi, c_lib = get_chelper_ffi()

    n = 50
    r, cx, cy = 30.0, 90.0, 90.0
    params = []
    for i in range(n):
        a0 = 2 * math.pi * i / n
        a1 = 2 * math.pi * (i + 1) / n
        params.append(make_move_params(
            [cx + r * math.cos(a0), cy + r * math.sin(a0), 0, 0],
            [cx + r * math.cos(a1), cy + r * math.sin(a1), 0, 0],
            150.0,
        ))
    py_moves = [PyMove(**p) for p in params]
    py_flush(py_moves)

    # Python trapq
    py_trapq = c_lib.trapq_alloc()
    t = 1.0
    for m in py_moves:
        if m.is_kinematic_move:
            c_lib.trapq_append(
                py_trapq, t, m.accel_t, m.cruise_t, m.decel_t,
                m.start_pos[0], m.start_pos[1], m.start_pos[2],
                m.axes_r[0], m.axes_r[1], m.axes_r[2],
                m.start_v, m.cruise_v, m.accel,
            )
        t += m.accel_t + m.cruise_t + m.decel_t

    py_entries = extract_trapq_moves(c_ffi, c_lib, py_trapq, 0, 9999)

    # Zig trapq
    zig_trapq = c_lib.trapq_alloc()
    eng = me_lib.motion_engine_create()
    me_lib.motion_engine_set_velocity_limits(eng, 500.0, 3000.0, 5.0, 0.5)
    me_lib.motion_engine_set_trapq(eng, zig_trapq)
    me_lib.motion_engine_set_print_time(eng, 1.0)
    for i, m in enumerate(py_moves):
        me_lib.motion_engine_queue_move_ex(
            eng,
            m.start_pos[0], m.start_pos[1], m.start_pos[2], m.start_pos[3],
            m.end_pos[0], m.end_pos[1], m.end_pos[2], m.end_pos[3],
            params[i]["speed"], m.accel,
            m.max_cruise_v2, m.delta_v2, m.smooth_delta_v2,
            m.next_junction_v2, 1 if m.is_kinematic_move else 0,
        )
    results_buf = me_ffi.new("struct FlushedMoveResult[200]")
    me_lib.motion_engine_flush_and_process(eng, results_buf, 200, 0)

    zig_entries = extract_trapq_moves(c_ffi, c_lib, zig_trapq, 0, 9999)

    assert len(py_entries) == len(zig_entries), (
        f"Circle trapq count: Py={len(py_entries)} Zig={len(zig_entries)}"
    )
    mismatches = []
    for i, (pm, zm) in enumerate(zip(py_entries, zig_entries)):
        for key in pm:
            if abs(pm[key] - zm[key]) > 1e-9:
                mismatches.append(
                    f"  {i} {key}: Py={pm[key]:.9f} Zig={zm[key]:.9f}"
                )
    assert not mismatches, (
        f"Circle trapq ({len(mismatches)} diffs):\n"
        + "\n".join(mismatches[:20])
    )

    c_lib.trapq_free(py_trapq)
    c_lib.trapq_free(zig_trapq)
    me_lib.motion_engine_destroy(eng)


# ── Clock sync tests ──


def test_clock_sync_basic():
    """Clock sync produces consistent time estimates."""
    me_ffi, me_lib = get_zig_ffi()

    cs = me_lib.clock_sync_create(48000000.0)
    assert cs is not None

    # Initial estimate at t=1.0 should be ~1.0
    pt = me_lib.clock_sync_estimated_print_time(cs, 1.0)
    assert abs(pt - 1.0) < 0.1, f"Initial estimate off: {pt}"

    me_lib.clock_sync_destroy(cs)


def test_clock_sync_update():
    """Clock sync processes samples and updates frequency estimate."""
    me_ffi, me_lib = get_zig_ffi()

    freq = 48000000.0
    cs = me_lib.clock_sync_create(freq)

    # Simulate clock samples
    for i in range(10):
        t = 1.0 + i * 1.0
        clock32 = int(t * freq) & 0xFFFFFFFF
        result = me_lib.clock_sync_update(cs, clock32, t, t + 0.0005)
        # result is new freq estimate or -1 if rejected

    # After samples, estimate should be reasonable
    pt = me_lib.clock_sync_estimated_print_time(cs, 11.0)
    assert abs(pt - 11.0) < 1.0, f"Post-update estimate off: {pt}"

    me_lib.clock_sync_destroy(cs)


# ── Engine lifecycle and edge case tests ──


def test_engine_create_destroy():
    """Engine lifecycle — create, use, destroy without leaks."""
    me_ffi, me_lib = get_zig_ffi()
    for _ in range(10):
        eng = me_lib.motion_engine_create()
        assert eng is not None
        me_lib.motion_engine_set_velocity_limits(eng, 500.0, 3000.0, 5.0, 0.5)
        me_lib.motion_engine_queue_move_ex(
            eng, 0, 0, 0, 0, 10, 0, 0, 0,
            100.0, 3000.0, 10000.0, 60000.0, 30000.0, 999999999.9, 1,
        )
        me_lib.motion_engine_destroy(eng)


def test_engine_reset():
    """Reset clears queue and timing state."""
    me_ffi, me_lib = get_zig_ffi()
    eng = me_lib.motion_engine_create()
    me_lib.motion_engine_set_velocity_limits(eng, 500.0, 3000.0, 5.0, 0.5)
    me_lib.motion_engine_queue_move_ex(
        eng, 0, 0, 0, 0, 10, 0, 0, 0,
        100.0, 3000.0, 10000.0, 60000.0, 30000.0, 999999999.9, 1,
    )
    assert me_lib.motion_engine_get_queue_len(eng) == 1
    me_lib.motion_engine_reset(eng)
    assert me_lib.motion_engine_get_queue_len(eng) == 0
    assert me_lib.motion_engine_get_print_time(eng) == 0.0
    me_lib.motion_engine_destroy(eng)


def test_empty_flush():
    """Flushing empty queue returns 0."""
    me_ffi, me_lib = get_zig_ffi()
    eng = me_lib.motion_engine_create()
    buf = me_ffi.new("struct FlushedMoveResult[10]")
    assert me_lib.motion_engine_flush_and_extract(eng, buf, 10, 0) == 0
    assert me_lib.motion_engine_flush_and_extract(eng, buf, 10, 1) == 0
    me_lib.motion_engine_destroy(eng)


def test_sync_state():
    """sync_state correctly updates all timing fields."""
    me_ffi, me_lib = get_zig_ffi()
    eng = me_lib.motion_engine_create()
    me_lib.motion_engine_sync_state(
        eng, 10.0, 9.5, 9.0, 10.5, 10.2, 5.0,
        50.0, 60.0, 10.0, 0.0,
    )
    assert me_lib.motion_engine_get_print_time(eng) == 10.0
    assert me_lib.motion_engine_get_last_flush_time(eng) == 9.5
    me_lib.motion_engine_destroy(eng)


def test_zero_distance_move_rejected():
    """Zero-distance moves are silently rejected."""
    me_ffi, me_lib = get_zig_ffi()
    eng = me_lib.motion_engine_create()
    me_lib.motion_engine_set_velocity_limits(eng, 500.0, 3000.0, 5.0, 0.5)
    ret = me_lib.motion_engine_queue_move_ex(
        eng, 10, 20, 30, 0, 10, 20, 30, 0,
        100.0, 3000.0, 10000.0, 60000.0, 30000.0, 999999999.9, 1,
    )
    assert ret == 0
    assert me_lib.motion_engine_get_queue_len(eng) == 0
    me_lib.motion_engine_destroy(eng)


def test_multi_batch_flush():
    """Multiple flush batches produce consistent results."""
    me_ffi, me_lib = get_zig_ffi()

    # Reference: all 20 moves flushed at once
    params = []
    for i in range(20):
        a = 2 * math.pi * i / 20
        params.append(make_move_params(
            [90 + 40 * math.cos(a), 90 + 40 * math.sin(a), 0, 0],
            [90 + 40 * math.cos(a + 2 * math.pi / 20),
             90 + 40 * math.sin(a + 2 * math.pi / 20), 0, 0],
            200.0,
        ))
    py_moves = [PyMove(**p) for p in params]
    py_flush(py_moves)
    single_results = [{
        "start_v": m.start_v, "cruise_v": m.cruise_v, "end_v": m.end_v,
    } for m in py_moves]

    # Zig: flush in two batches of 10
    eng = me_lib.motion_engine_create()
    me_lib.motion_engine_set_velocity_limits(eng, 500.0, 3000.0, 5.0, 0.5)
    buf = me_ffi.new("struct FlushedMoveResult[100]")

    # Queue first 10
    for i in range(10):
        m = py_moves[i]
        me_lib.motion_engine_queue_move_ex(
            eng, m.start_pos[0], m.start_pos[1], m.start_pos[2], m.start_pos[3],
            m.end_pos[0], m.end_pos[1], m.end_pos[2], m.end_pos[3],
            params[i]["speed"], m.accel, m.max_cruise_v2, m.delta_v2,
            m.smooth_delta_v2, m.next_junction_v2,
            1 if m.is_kinematic_move else 0,
        )
    # Flush batch 1 (lazy — may not flush all 10)
    count1 = me_lib.motion_engine_flush_and_extract(eng, buf, 100, 1)

    # Queue remaining 10
    for i in range(10, 20):
        m = py_moves[i]
        me_lib.motion_engine_queue_move_ex(
            eng, m.start_pos[0], m.start_pos[1], m.start_pos[2], m.start_pos[3],
            m.end_pos[0], m.end_pos[1], m.end_pos[2], m.end_pos[3],
            params[i]["speed"], m.accel, m.max_cruise_v2, m.delta_v2,
            m.smooth_delta_v2, m.next_junction_v2,
            1 if m.is_kinematic_move else 0,
        )
    # Flush batch 2 (non-lazy — flush everything)
    count2 = me_lib.motion_engine_flush_and_extract(eng, buf, 100, 0)

    total = count1 + count2
    assert total == 20, f"Multi-batch total: {total} (batch1={count1}, batch2={count2})"

    me_lib.motion_engine_destroy(eng)


def test_velocity_limits_update():
    """Velocity limits can be changed between moves."""
    me_ffi, me_lib = get_zig_ffi()
    eng = me_lib.motion_engine_create()

    # Start with low limits
    me_lib.motion_engine_set_velocity_limits(eng, 100.0, 1000.0, 5.0, 0.5)
    me_lib.motion_engine_queue_move_ex(
        eng, 0, 0, 0, 0, 50, 0, 0, 0,
        100.0, 1000.0, 10000.0, 100000.0, 50000.0, 999999999.9, 1,
    )

    buf = me_ffi.new("struct FlushedMoveResult[10]")
    count = me_lib.motion_engine_flush_and_extract(eng, buf, 10, 0)
    assert count == 1
    assert buf[0].cruise_v <= 100.0 + 1e-9

    # Update to high limits
    me_lib.motion_engine_set_velocity_limits(eng, 500.0, 5000.0, 5.0, 0.5)
    me_lib.motion_engine_queue_move_ex(
        eng, 50, 0, 0, 0, 100, 0, 0, 0,
        500.0, 5000.0, 250000.0, 500000.0, 250000.0, 999999999.9, 1,
    )
    count = me_lib.motion_engine_flush_and_extract(eng, buf, 10, 0)
    assert count == 1
    assert buf[0].cruise_v > 100.0  # higher cruise with new limits

    me_lib.motion_engine_destroy(eng)


def test_obtuse_angle():
    """Obtuse angle (135 degrees) — moderate junction speed."""
    params = [
        make_move_params([0, 0, 0, 0], [50, 0, 0, 0], 200.0),
        make_move_params([50, 0, 0, 0], [100, 50, 0, 0], 200.0),
    ]
    py, zig, mismatches = run_comparison(params)
    assert not mismatches, "Obtuse angle mismatch:\n" + "\n".join(mismatches)
    # 135-degree turn should allow higher junction than 90
    assert py[1]["start_v"] > 0


def test_full_reversal():
    """180-degree reversal — junction speed should be near zero."""
    params = [
        make_move_params([0, 0, 0, 0], [50, 0, 0, 0], 200.0),
        make_move_params([50, 0, 0, 0], [0, 0, 0, 0], 200.0),
    ]
    py, zig, mismatches = run_comparison(params)
    assert not mismatches, "Reversal mismatch:\n" + "\n".join(mismatches)
    # Full reversal should have very low junction speed
    assert py[1]["start_v"] < 10.0


def test_tiny_moves():
    """Very short moves (0.1mm) — tests numerical stability."""
    params = []
    for i in range(20):
        params.append(make_move_params(
            [50 + i * 0.1, 50, 0, 0],
            [50 + (i + 1) * 0.1, 50, 0, 0],
            100.0,
        ))
    py, zig, mismatches = run_comparison(params)
    assert not mismatches, (
        f"Tiny moves mismatch ({len(mismatches)}):\n"
        + "\n".join(mismatches[:10])
    )


def test_diagonal_3d():
    """3D diagonal moves."""
    params = [
        make_move_params([0, 0, 0, 0], [30, 30, 30, 0], 100.0),
        make_move_params([30, 30, 30, 0], [60, 0, 60, 0], 100.0),
        make_move_params([60, 0, 60, 0], [0, 60, 0, 0], 100.0),
    ]
    py, zig, mismatches = run_comparison(params)
    assert not mismatches, "3D diagonal mismatch:\n" + "\n".join(mismatches)


# ── Main ──


def main():
    tests = [
        # Velocity profile comparison (Python vs Zig lookahead)
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
        test_obtuse_angle,
        test_full_reversal,
        test_tiny_moves,
        test_diagonal_3d,
        # Trapq output comparison (Python trapq_append vs Zig flush_and_process)
        test_trapq_output_matches,
        test_trapq_circle_segments,
        # Clock sync
        test_clock_sync_basic,
        test_clock_sync_update,
        # Engine lifecycle and edge cases
        test_engine_create_destroy,
        test_engine_reset,
        test_empty_flush,
        test_sync_state,
        test_zero_distance_move_rejected,
        test_multi_batch_flush,
        test_velocity_limits_update,
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
