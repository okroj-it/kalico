# Zig Native Motion Engine

## Preface

### The problem

Klipper splits 3D printer control between a host computer (Raspberry Pi) and
microcontrollers. The MCU firmware is C with microsecond-precision timing —
it's fast. But the host-side motion planning that feeds the MCUs runs in
Python, and on resource-constrained boards (Pi Zero, Pi 3, Pi 4 under load),
Python can't keep up with high-speed printers.

The symptoms are familiar to anyone pushing a Klipper printer past 500 mm/s:

- **"Timer too close" errors** — the host can't compute moves fast enough to
  keep the MCU's command buffer filled.
- **Print stalls** — the MCU runs out of queued steps and pauses mid-move.
- **Timing synchronization failures** — clock drift between host and MCU
  causes cascading errors.

The root cause is Python's interpreter overhead in the motion hot path. Every
single move goes through this pipeline:

1. `Move.__init__()` — ~20 floating point operations to construct the move
2. `LookAheadQueue.flush()` — O(n) backward traversal through all queued
   moves, doing trigonometry and velocity math at each step
3. `_process_moves()` — iterates resolved moves, calls `trapq_append` (C),
   `extruder.move()` (Python wrapping C), timing callbacks
4. `_advance_flush_time()` — calls `itersolve_generate_steps` (C) and
   `steppersync_flush` (C) for each stepper and MCU

Steps 3 and 4 are mostly thin Python wrappers around C functions that are
already fast. Steps 1 and 2 are pure Python float math — thousands of
interpreter-mediated operations per second. On a fast printer doing 1000+
moves/sec with a 250ms lookahead window, that's ~250 moves per flush batch,
each requiring ~20 float operations in the backward pass alone — 5000+ Python
float operations in a single flush call, all blocking the GIL.

Add Python's garbage collector (which the existing code in `reactor.py` already
tries to manually schedule around timing-critical sections) and the Global
Interpreter Lock (which serializes the clock sync thread with the motion
planning thread), and you get unpredictable latency spikes right where you
need deterministic timing.

### The approach

Rather than rewriting Kalico/Klipper from scratch, we surgically replace the
most expensive part of the motion pipeline with a native Zig module while
keeping everything else in Python. The key insight is that Klipper's existing
`chelper` already establishes the pattern: C code compiled into a shared
library, loaded by Python via CFFI. We follow the exact same pattern, but with
Zig instead of C.

Zig was chosen over Rust for this specific project because:

- **Zero-cost C interop via `@cImport`** — Zig directly imports the existing
  chelper C headers (`trapq.h`, `itersolve.h`, `stepcompress.h`) and calls
  their functions as native calls. No FFI bindings, no `unsafe` blocks, no
  wrapper code. The Zig module links the existing C sources directly.
- **No runtime** — no garbage collector, no async runtime, no hidden
  allocations. Pure computation with deterministic timing.
- **Cross-compilation is trivial** — `zig build -Dtarget=aarch64-linux-gnu`
  produces a Pi-ready binary on any host. No cross-toolchain setup.
- **Small binaries** — the complete `.so` including all chelper C code is ~300KB.

### What the native engine replaces

The native engine replaces `LookAheadQueue.flush()` — the O(n) backward
traversal that resolves junction speeds between queued moves. This is the
single most expensive operation in the motion pipeline: pure Python float math
over hundreds of moves per flush batch.

The native engine also handles move construction (`Move.__init__` equivalent)
and junction calculation (`calc_junction` equivalent) in Zig, eliminating
Python object creation and float math for every queued move.

After the native lookahead resolves velocity profiles, the results are passed
back to Python `Move` objects which then flow through the existing
`_process_moves()` pipeline unchanged. This means `trapq_append`,
`extruder.move()`, `itersolve_generate_steps`, `steppersync_flush`, timing
callbacks, and all reactor interactions work exactly as before.

### What stays in Python

Everything except the lookahead flush:

- **`_process_moves()`** — iterates resolved moves, calls trapq_append and
  extruder.move. These are thin C wrappers, not the bottleneck.
- **`_advance_flush_time()`** — calls itersolve and steppersync. Also thin C
  wrappers with complex state (clock sync, multi-MCU coordination).
- **`check_move()`** — kinematics validation. Must stay for plugin compat.
- **`drip_move()` (homing)** — temporarily disables native engine, uses Python
  path for the special drip timing that homing requires.
- **Config, gcode, all 156 extras, plugins, reactor, webhooks** — untouched.

### How it interops with Kalico without touching core code

The integration follows three principles:

**1. Opt-in via config, default off.** The native engine is enabled by a single
config option (`native_motion_engine: True` in `[danger_options]`). When
disabled, the code path is unchanged — zero overhead, zero risk.

**2. Python `Move` objects still exist, but only for validation and post-flush
processing.** Kinematics plugins implement `check_move()` which may call
`move.limit_speed()` to reduce velocity/acceleration. The extruder does the
same. So `toolhead.move()` still creates a Python `Move`, calls `check_move()`,
then passes the final post-validation parameters to the native engine's
lookahead queue. The Python `Move` is kept in a parallel queue so that after
the native flush resolves velocity profiles, the results can be applied back
to the Python Moves for `_process_moves()` to consume normally.

**3. Graceful fallback.** If the Zig library can't be loaded (missing binary,
build failure, unsupported architecture), the system automatically falls back
to the original Python path with a warning in the log.

### Design iterations

The design went through four iterations:

1. **Full native pipeline** — Zig handles everything from lookahead through
   steppersync_flush, calling C functions directly. Failed in practice because
   Klipper has many code paths that call `_advance_flush_time` (dwell,
   flush_handler, drip_move, stats), and overriding all of them led to state
   conflicts between Zig and Python timing state, especially with multi-MCU
   clock synchronization and homing.

2. **Native lookahead + Python MCU flush** — Zig handles lookahead + trapq +
   itersolve, Python handles steppersync_flush. Cleaner but still had state
   conflicts with dwell/drip_move calling Python `_advance_flush_time`.

3. **Native lookahead + result extraction** — Zig only resolves junction
   speeds, returns velocity profiles to Python, Python does everything else
   via the existing `_process_moves` pipeline. This is what shipped and is
   confirmed working on real hardware.

4. **Full native pipeline (planned)** — Now that the math is validated on
   hardware, the full pipeline can be re-attempted with proper handling of
   multi-MCU clocks, drip_move bypass, and all `_advance_flush_time` callers.

### Hardware validation

Tested on an Annex K3 printer:
- **Host**: Raspberry Pi 3 (aarch64)
- **MCUs**: 4 (Spider mainboard, Supernova XY controller, host MCU, Beacon probe)
- **Steppers**: 7 (X, X1, Y, Y1, Z, Z1, Z2 + extruder)
- **Kinematics**: Cartesian with AWD gantry

Results:
- G28 (homing all axes): works (uses Python drip_move path)
- Z-tilt adjustment: works
- Rapid travel moves (500 mm/s): works, `print_stall=0`
- 500-move stress test at F30000: works, `print_stall=0`
- Actual print with extrusion: works, correct dimensions

---

## Enabling

Add to your `printer.cfg`:

```ini
[danger_options]
native_motion_engine: True
```

When disabled (default), the original Python path is used with zero overhead.

## Building

### Pre-built binaries (no Zig required)

Pre-compiled binaries are included for common architectures:

```
klippy/chelper/zig_engine/prebuilt/
├── libmotion_engine-x86_64.so   (~298KB)
├── libmotion_engine-aarch64.so  (~302KB)  ← Raspberry Pi 4/5
└── libmotion_engine-armv7.so    (~1.1MB)  ← Raspberry Pi 3/Zero
```

The Python loader automatically detects your architecture and loads the
matching prebuilt binary. No Zig installation needed on the printer.

### Building from source

```bash
cd klippy/chelper/zig_engine
zig build -Doptimize=ReleaseFast
zig build test  # run tests
```

Requires Zig 0.16.0-dev or later.

### Loading priority

```
1. prebuilt/ binary for detected architecture  → use it (no Zig needed)
2. zig-out/ from a previous build              → use it
3. `zig build` from source                     → build and use
4. none available                              → fall back to Python
```

## Module structure

```
klippy/chelper/zig_engine/
├── build.zig          # Zig build — compiles Zig + links chelper C sources
├── build.zig.zon      # Package metadata, pins Zig version
├── prebuilt/          # Pre-compiled .so for x86_64, aarch64, armv7
└── src/
    ├── main.zig       # Entry point, forces symbol export
    ├── c.zig          # @cImport of chelper C headers
    ├── move.zig       # MoveData: construction, junction calc, velocity profiles
    ├── lookahead.zig  # LookAheadQueue: O(n) backward pass for velocity planning
    ├── clocksync.zig  # ClockSync: linear regression for MCU clock estimation
    └── engine.zig     # MotionEngine: orchestrator + C API exports
```

## Files modified in Kalico

| File | Change |
|---|---|
| `klippy/chelper/__init__.py` | CFFI definitions for motion engine, `get_motion_ffi()` loader |
| `klippy/toolhead.py` | `_init_native_engine()`, `_native_move()`, `_native_flush()`, `_native_flush_lookahead()`, `_native_flush_step_generation()` + delegation in `move()`, `_flush_lookahead()`, `flush_step_generation()`, `get_last_move_time()`, `limit_next_junction_speed()`, `drip_move()`, `_handle_shutdown()`, `_calc_junction_deviation()` |
| `klippy/extras/danger_options.py` | `native_motion_engine` boolean option (default False) |

No other Kalico files are modified. All kinematics, extras, plugins, config
parsing, gcode handling, reactor, and MCU communication code is untouched.

## Performance

### Who benefits

**Older/constrained hardware (Pi 3, Pi Zero, CM3) — biggest impact.** These
boards are where Python motion planning hits its ceiling first. The native
engine removes the most expensive computation from Python, leaving headroom
for everything else.

**Modern hardware (Pi 4, Pi 5, CM4) — extends the envelope.** Eliminates
unpredictable GC pauses and GIL contention that cause occasional stalls even
on fast hardware. Frees CPU for higher speeds, more steppers, heavier features.

### What's faster

The lookahead flush (junction speed resolution) runs in native code instead of
the Python interpreter. For a typical flush batch of 250 moves, this eliminates
~5000 Python float operations. Move construction and junction calculation also
run natively, eliminating per-move interpreter overhead.

### What's unchanged

`_process_moves` (trapq_append, extruder), `_advance_flush_time` (itersolve,
steppersync), and all reactor/timing interactions remain in Python. These are
thin C wrappers and not the bottleneck.

Real-world benchmarks on target hardware are needed to quantify the improvement
to buffer_time stability and maximum sustainable move rate.
