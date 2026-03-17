// Kalico Native Motion Engine
//
// Replaces the timing-critical Python motion planning code with native Zig.
// Exposes a C ABI so Python can call via CFFI (same pattern as chelper).
//
// Components:
//   move.zig       - Move construction + junction calculation (replaces toolhead.Move)
//   lookahead.zig  - Look-ahead queue + flush (replaces toolhead.LookAheadQueue)
//   clocksync.zig  - Clock synchronization (replaces clocksync.ClockSync)
//   engine.zig     - Top-level orchestrator tying it all together

pub const move = @import("move.zig");
pub const lookahead = @import("lookahead.zig");
pub const clocksync = @import("clocksync.zig");
pub const engine = @import("engine.zig");
pub const c = @import("c.zig");

// C API is exposed via `export fn` in each module.
// Python loads libmotion_engine.so and calls these directly via CFFI.

comptime {
    // Force export of all C API functions by referencing the modules.
    _ = engine;
    _ = clocksync;
}

test {
    @import("std").testing.refAllDecls(@This());
}
