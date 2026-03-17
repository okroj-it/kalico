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
timing-critical motion pipeline with a native Zig module while keeping
everything else in Python. The key insight is that Klipper's existing `chelper`
already establishes the pattern: C code compiled into a shared library,
loaded by Python via CFFI. We follow the exact same pattern, but with Zig
instead of C, and covering a wider scope.

Zig was chosen over Rust for this specific project because:

- **Zero-cost C interop via `@cImport`** — Zig directly imports the existing
  chelper C headers (`trapq.h`, `itersolve.h`, `stepcompress.h`) and calls
  their functions as native calls. No FFI bindings, no `unsafe` blocks, no
  wrapper code. The Zig module links the existing C sources directly.
- **No runtime** — no garbage collector, no async runtime, no hidden
  allocations. The motion loop is pure computation with deterministic timing.
- **Cross-compilation is trivial** — `zig build -Dtarget=aarch64-linux-gnu`
  produces a Pi-ready binary on any host. No cross-toolchain setup.
- **Small binaries** — the complete `.so` including all chelper C code is ~300KB.

The design went through three iterations:

1. **First attempt**: Native engine handles only the lookahead flush, returns
   resolved velocity profiles to Python, Python runs `_process_moves`. This
   still created Python `Move` objects and kept a parallel Python queue.

2. **Second attempt**: Same as above but Python also handled step generation
   and MCU flush via callbacks. This was correct but still had per-move Python
   overhead for the forward pass in `_process_moves`.

3. **Final design**: The native engine holds direct C pointers to
   `stepper_kinematics` and `steppersync` structs, registered at startup. It
   calls `itersolve_generate_steps()` and `steppersync_flush()` directly —
   the same C functions that Python's `stepper.generate_steps()` and
   `mcu.flush_moves()` were wrapping. No Python in the motion loop at all.

### How it interops with Kalico without touching core code

The integration follows three principles:

**1. Opt-in via config, default off.** The native engine is enabled by a single
config option (`native_motion_engine: True` in `[danger_options]`). When
disabled, the code path is unchanged — zero overhead, zero risk.

**2. Python `Move` objects still exist, but only for validation.** Kinematics
plugins (`cartesian.py`, `delta.py`, etc.) implement `check_move()` which may
call `move.limit_speed()` to reduce velocity/acceleration for specific axes.
The extruder does the same for extrusion limits. These are Python plugin
methods that we can't (and shouldn't) bypass. So `toolhead.move()` still
creates a Python `Move`, calls `check_move()`, then passes the final
post-validation parameters to the native engine. The Python `Move` is
immediately discarded — it's not queued, not stored, not iterated later.

**3. C pointers, not Python callbacks.** The existing Python step generator
(`stepper.generate_steps()`) is a method that calls
`itersolve_generate_steps(sk, flush_time)` — a C function taking a C struct
pointer and a double. The MCU flush (`mcu.flush_moves()`) calls
`steppersync_flush(ss, clock, clear_history_clock)` — same pattern. The
extruder calls `trapq_append()` on its own trapq. All of these are C functions
operating on C data. The native engine receives the C pointers at startup and
calls these functions directly, bypassing the Python wrappers entirely.

### What this means in practice

Before (Python motion loop):
```
Python Move.__init__()                    ← interpreter overhead
Python LookAheadQueue.flush()             ← O(n) Python float math
Python _process_moves() loop:
  Python → CFFI → C trapq_append()        ← CFFI crossing overhead per move
  Python → CFFI → C trapq_append() (ext)  ← CFFI crossing overhead per move
Python _advance_flush_time():
  Python → CFFI → C itersolve_generate_steps()  ← CFFI per stepper
  Python → CFFI → C steppersync_flush()         ← CFFI per MCU
```

After (Zig motion loop):
```
Python check_move()                       ← still Python (kinematics plugins)
  ↓
Zig: build MoveData from parameters       ← native float math
Zig: LookAheadQueue.addMove + calcJunction ← native
Zig: LookAheadQueue.flush()               ← native O(n) backward pass
Zig: processMoves():
  Zig → C trapq_append()                  ← direct call, no CFFI
  Zig → C trapq_append() (extruder)       ← direct call, no CFFI
Zig: advanceFlushTime():
  Zig → C itersolve_generate_steps()      ← direct call per stepper
  Zig → C steppersync_flush()             ← direct call per MCU
```

No Python interpreter, no GIL, no GC, no CFFI crossing in the motion loop.

---

## Enabling

Add to your `printer.cfg`:

```ini
[danger_options]
native_motion_engine: True
```

When disabled (default), the original Python path is used with zero overhead.
If the Zig library can't be loaded (missing binary, build failure), the system
automatically falls back to Python with a warning in the log.

## What gets replaced

| Python code | What it does | Zig replacement |
|---|---|---|
| `Move.__init__` | Construct move with float math | `move.zig` `MoveData.init` |
| `Move.calc_junction` | Junction speed from dot product + trig | `move.zig` `calcJunction` |
| `Move.set_junction` | Velocity profile from junction speeds | `move.zig` `setJunction` |
| `LookAheadQueue.flush` | O(n) backward pass resolving speeds | `lookahead.zig` `flush` |
| `_process_moves` | trapq_append for XYZ + extruder | `engine.zig` `processMoves` |
| `_advance_flush_time` | itersolve + steppersync | `engine.zig` `advanceFlushTime` |
| `_advance_move_time` | Batch flush orchestration | `engine.zig` `advanceMoveTime` |

## What stays in Python

Everything that isn't in the per-move timing-critical path:

- **Config parsing** (`configfile.py`) — runs once at startup
- **Gcode parsing** (`gcode.py`) — fast enough, not timing-critical
- **Kinematics `check_move()`** — per-move validation, but just limit checks
- **All 156 extras** — probes, bed mesh, TMC drivers, fans, heaters, displays
- **Plugin system** — dynamic module loading via importlib
- **Webhooks/API** — status reporting, Moonraker interface
- **Reactor event loop** — Python for scheduling, pause, shutdown
- **`_check_pause` / `_flush_handler`** — timer-based buffer management
- **`drip_move` (homing)** — temporarily disables native engine, uses Python path

## Building

### Pre-built binaries (no Zig required)

Pre-compiled binaries are included for common architectures:

```
klippy/chelper/zig_engine/prebuilt/
├── libmotion_engine-x86_64.so   (~298KB)
├── libmotion_engine-aarch64.so  (~302KB)  ← Raspberry Pi 4/5
└── libmotion_engine-armv7.so    (~1.1MB)  ← Raspberry Pi 3/Zero
```

The Python loader (`chelper/__init__.py`) automatically detects your
architecture and loads the matching prebuilt binary. No Zig installation
needed on the printer.

### Building from source

```bash
cd klippy/chelper/zig_engine

# Debug build
zig build

# Release build
zig build -Doptimize=ReleaseFast

# Cross-compile for Raspberry Pi (aarch64)
zig build -Doptimize=ReleaseFast -Dtarget=aarch64-linux-gnu

# Run tests
zig build test
```

Requires Zig 0.16.0-dev or later (pinned in `build.zig.zon`).

### Loading priority

```
1. prebuilt/ binary for detected architecture  → use it (no Zig needed)
2. zig-out/ from a previous build              → use it
3. `zig build` from source                     → build and use
4. none available                              → fall back to Python
```

### CI

The GitHub Actions workflow (`.github/workflows/ci-motion-engine.yaml`) rebuilds
prebuilt binaries automatically when any file in `klippy/chelper/zig_engine/src/`,
`build.zig`, or `klippy/chelper/*.c`/`.h` changes. It runs tests and
cross-compiles for all three architectures.

## C API Reference

The library exposes a C ABI loaded by Python via CFFI.

### Lifecycle

```c
struct MotionEngine *motion_engine_create(void);
void motion_engine_destroy(struct MotionEngine *engine);
void motion_engine_reset(struct MotionEngine *engine);
```

### Hardware registration (called at klippy:connect)

```c
// XYZ kinematics trapq
void motion_engine_set_trapq(struct MotionEngine *engine, struct trapq *tq);

// Extruder trapq and pressure advance state
void motion_engine_set_extruder_trapq(struct MotionEngine *engine, struct trapq *tq);
void motion_engine_set_extruder_params(struct MotionEngine *engine,
    double pressure_advance, double use_pa_from_trapq, double instant_corner_v);

// Register stepper_kinematics pointers (for itersolve_generate_steps)
int motion_engine_add_stepper(struct MotionEngine *engine,
    struct stepper_kinematics *sk);

// Register steppersync pointers (for steppersync_flush)
int motion_engine_add_mcu(struct MotionEngine *engine,
    struct steppersync *ss, double mcu_freq);

// Update MCU frequency after clock recalibration
void motion_engine_update_mcu_freq(struct MotionEngine *engine,
    uint32_t index, double mcu_freq);
```

### Configuration

```c
void motion_engine_set_velocity_limits(struct MotionEngine *engine,
    double max_velocity, double max_accel,
    double square_corner_velocity, double min_cruise_ratio);
void motion_engine_set_position(struct MotionEngine *engine,
    double x, double y, double z, double e);
void motion_engine_set_print_time(struct MotionEngine *engine, double print_time);
void motion_engine_set_kin_flush_delay(struct MotionEngine *engine, double delay);
```

### Move processing

```c
// Queue a move with pre-applied velocity limits (from Python check_move)
int motion_engine_queue_move_ex(struct MotionEngine *engine,
    double start_x, double start_y, double start_z, double start_e,
    double end_x, double end_y, double end_z, double end_e,
    double speed, double accel,
    double max_cruise_v2, double delta_v2,
    double smooth_delta_v2, double next_junction_v2,
    int is_kinematic);
    // Returns: -1 error, 0 queued, 1 flush triggered

// Full flush — resolves all moves, generates steps, flushes MCUs
void motion_engine_flush(struct MotionEngine *engine);
void motion_engine_flush_step_generation(struct MotionEngine *engine);
```

### Status

```c
double motion_engine_get_print_time(const struct MotionEngine *engine);
double motion_engine_get_buffer_time(const struct MotionEngine *engine,
    double est_print_time);
uint32_t motion_engine_get_stall_count(const struct MotionEngine *engine);
uint32_t motion_engine_get_queue_len(const struct MotionEngine *engine);
```

### Clock Sync

```c
struct ClockSync *clock_sync_create(double mcu_freq);
void clock_sync_destroy(struct ClockSync *cs);
void clock_sync_set_freq(struct ClockSync *cs, double mcu_freq);
double clock_sync_update(struct ClockSync *cs,
    uint32_t clock32, double sent_time, double receive_time);
int64_t clock_sync_get_clock(const struct ClockSync *cs, double eventtime);
double clock_sync_estimated_print_time(const struct ClockSync *cs, double eventtime);
```

## Module structure

```
klippy/chelper/zig_engine/
├── build.zig          # Zig build — compiles Zig + links chelper C sources
├── build.zig.zon      # Package metadata, pins Zig version
├── prebuilt/          # Pre-compiled .so for x86_64, aarch64, armv7
└── src/
    ├── main.zig       # Entry point, forces symbol export
    ├── c.zig          # @cImport of chelper C headers (trapq, itersolve, etc.)
    ├── move.zig       # MoveData: construction, junction calc, velocity profiles
    ├── lookahead.zig  # LookAheadQueue: O(n) backward pass for velocity planning
    ├── clocksync.zig  # ClockSync: linear regression for MCU clock estimation
    └── engine.zig     # MotionEngine: orchestrator, C API exports, direct C calls
```

## How it links with existing code

The Zig module `@cImport`s the existing chelper C headers and compiles all
chelper C sources into the shared library. This means:

1. **Direct C function calls** — Zig calls `trapq_append`,
   `itersolve_generate_steps`, `steppersync_flush` as native function calls.
   No CFFI, no FFI, no wrapper overhead.

2. **Holds C struct pointers** — At startup, Python passes `stepper_kinematics*`
   and `steppersync*` pointers to the native engine. These are the same C
   structs that Python's `stepper.py` and `mcu.py` allocate via CFFI. The
   native engine stores them and uses them directly during flush.

3. **Same CFFI loading pattern** — Python loads `libmotion_engine.so` via CFFI
   exactly like it loads `c_helper.so`. The library contains both the new Zig
   code and all existing chelper C code.

4. **No modifications to chelper C sources** — The existing `.c` and `.h` files
   are compiled unmodified into the Zig shared library. The Zig code is purely
   additive.

## Files modified in Kalico

| File | Change |
|---|---|
| `klippy/chelper/__init__.py` | CFFI definitions for motion engine functions, `get_motion_ffi()` loader with prebuilt/build/fallback |
| `klippy/toolhead.py` | `_init_native_engine()`, `_native_register_hardware()`, `_native_move()`, `_native_flush_lookahead()`, `_native_flush_step_generation()` + delegation in `move()`, `_flush_lookahead()`, `flush_step_generation()`, `set_position()`, `get_last_move_time()`, `_calc_junction_deviation()`, `note_step_generation_scan_time()`, `limit_next_junction_speed()`, `drip_move()`, `_handle_shutdown()` |
| `klippy/extras/danger_options.py` | `native_motion_engine` boolean option (default False) |

No other Kalico files are modified. All kinematics, extras, plugins, config
parsing, gcode handling, reactor, and MCU communication code is untouched.

## Testing

```bash
cd klippy/chelper/zig_engine
zig build test
```

Tests cover:
- Move initialization (kinematic and extrude-only)
- Junction speed calculation (collinear and cornering moves)
- Velocity profile generation (set_junction)
- Look-ahead queue flush (single, collinear, and corner moves)
- Clock synchronization (sample processing, time estimation)
- Engine lifecycle (create, queue, flush, destroy)

## Performance expectations

The native module eliminates all Python from the motion loop:

| Operation | Before (Python) | After (Zig) |
|---|---|---|
| Move construction | ~20 Python float ops | Native float ops |
| Junction calculation | ~15 Python float ops + trig | Native |
| Lookahead flush (250 moves) | ~5000 Python float ops | Native O(n) |
| trapq_append per move | Python → CFFI → C | Zig → C (direct call) |
| itersolve per stepper | Python → CFFI → C | Zig → C (direct call) |
| steppersync per MCU | Python → CFFI → C | Zig → C (direct call) |
| GIL contention | Blocks clock sync thread | No GIL |
| GC pauses | Unpredictable, manually managed | None |

### Who benefits

**Older/constrained hardware (Pi 3, Pi Zero, CM3) — biggest impact.** These
boards are where Python motion planning hits its ceiling first. A Pi 3 running
a fast CoreXY at 500+ mm/s with input shaper enabled can easily saturate the
Python interpreter, causing "Timer too close" errors and print stalls. The
native engine removes that ceiling entirely — the host CPU spends near-zero
time on motion math, leaving headroom for everything else (web UI, camera
streaming, additional MCUs).

**Modern hardware (Pi 4, Pi 5, CM4) — extends the envelope.** These boards
rarely stall under normal conditions, but they still hit limits when pushing
extreme speeds (1000+ mm/s), running multiple MCUs, or using compute-heavy
features like high-frequency input shaper with many stepper motors. The native
engine reduces motion planning from the dominant CPU consumer to a rounding
error, freeing resources for features that would otherwise compete with the
motion loop. It also eliminates the unpredictable GC pauses and GIL contention
that can cause occasional one-off stalls even on fast hardware.

**In short:** older hardware goes from "can't keep up" to "works reliably."
Newer hardware goes from "works reliably" to "works reliably with headroom
to spare for higher speeds, more steppers, and heavier workloads."

The motion math itself (lookahead flush, junction calculation) runs 10-50x
faster in native code vs Python — this is well-established for compiled vs
interpreted float math. The end-to-end impact on print reliability depends on
how much of the host's time was spent in the motion loop vs other work (gcode
parsing, reactor, serial I/O). Real-world benchmarks on target hardware are
needed to quantify the actual improvement to buffer_time stability and maximum
sustainable move rate.
