// Top-level motion engine orchestrator.
//
// This is the main C API that Python calls. It owns the lookahead queue and
// coordinates flushing moves into the existing C trapq/itersolve/stepcompress
// pipeline — calling C functions directly, not through Python.
//
// Replaces the full timing-critical pipeline:
//   - toolhead.Move (construction + junction calculation)
//   - toolhead.LookAheadQueue (velocity profile resolution)
//   - toolhead._process_moves (trapq_append + extruder trapq)
//   - toolhead._advance_flush_time (itersolve + steppersync)
//   - toolhead._advance_move_time (batch orchestration)
//
// Python only calls check_move() for kinematics validation, then passes
// final limits here. Everything else runs natively.

const std = @import("std");
const MoveData = @import("move.zig").MoveData;
const Pos = @import("move.zig").Pos;
const LookAheadQueue = @import("lookahead.zig").LookAheadQueue;
const c = @import("c.zig");

const STEPCOMPRESS_FLUSH_TIME: f64 = 0.050;
const SDS_CHECK_TIME: f64 = 0.001;
const MOVE_BATCH_TIME: f64 = 0.500;
const MAX_STEPPERS: usize = 64;
const MAX_MCUS: usize = 8;

/// Callback for rare Python-side notifications (timing_callbacks, active_callbacks).
/// Only called when Python registers interest — not in the hot path.
pub const NotifyCallback = *const fn (ctx: ?*anyopaque) callconv(.c) void;

/// MCU entry: holds steppersync pointer and clock conversion parameters.
/// Primary MCU: offset=0, freq=mcu_freq
/// Secondary MCUs: offset=adjusted_offset, freq=adjusted_freq (from clock_adj)
const McuEntry = struct {
    steppersync: *c.StepperSync,
    time_offset: f64,
    mcu_freq: f64,
};

/// Stepper entry: holds stepper_kinematics pointer for direct step generation.
const StepperEntry = struct {
    sk: *c.StepperKinematics,
};

pub const MotionEngine = struct {
    allocator: std.mem.Allocator,
    lookahead: LookAheadQueue,

    // Velocity limits
    max_velocity: f64,
    max_accel: f64,
    max_accel_to_decel: f64,
    junction_deviation: f64,
    square_corner_velocity: f64,

    // Print time tracking
    print_time: f64,
    last_flush_time: f64,
    min_restart_time: f64,
    need_flush_time: f64,
    step_gen_time: f64,
    clear_history_time: f64,
    kin_flush_delay: f64,

    // Position tracking
    commanded_pos: Pos,

    // Stall detection
    print_stall: u32,

    // XYZ kinematics trapq (owned by chelper, we reference it)
    trapq: ?*c.Trapq,

    // Extruder trapq + state (for direct extruder move processing)
    extruder_trapq: ?*c.Trapq,
    extruder_pressure_advance: f64,
    extruder_use_pa_from_trapq: f64,
    extruder_instant_corner_v: f64,

    // Direct C pointers for step generation (no Python callbacks)
    steppers: [MAX_STEPPERS]StepperEntry,
    stepper_count: usize,

    // Direct C pointers for MCU flush (no Python callbacks)
    mcus: [MAX_MCUS]McuEntry,
    mcu_count: usize,

    // Optional Python callback for post-flush notification
    // (called once per flush batch for timing_callbacks, active_callbacks, etc.)
    post_flush_cb: ?NotifyCallback,
    post_flush_ctx: ?*anyopaque,

    pub fn init(allocator: std.mem.Allocator) MotionEngine {
        return .{
            .allocator = allocator,
            .lookahead = LookAheadQueue.init(allocator),
            .max_velocity = 500.0,
            .max_accel = 3000.0,
            .max_accel_to_decel = 1500.0,
            .junction_deviation = 0.01,
            .square_corner_velocity = 5.0,
            .print_time = 0.0,
            .last_flush_time = 0.0,
            .min_restart_time = 0.0,
            .need_flush_time = 0.0,
            .step_gen_time = 0.0,
            .clear_history_time = 0.0,
            .kin_flush_delay = SDS_CHECK_TIME,
            .commanded_pos = .{ 0, 0, 0, 0 },
            .print_stall = 0,
            .trapq = null,
            .extruder_trapq = null,
            .extruder_pressure_advance = 0.0,
            .extruder_use_pa_from_trapq = 0.0,
            .extruder_instant_corner_v = 0.0,
            .steppers = undefined,
            .stepper_count = 0,
            .mcus = undefined,
            .mcu_count = 0,
            .post_flush_cb = null,
            .post_flush_ctx = null,
        };
    }

    pub fn deinit(self: *MotionEngine) void {
        self.lookahead.deinit();
    }

    pub fn reset(self: *MotionEngine) void {
        self.lookahead.reset();
        self.print_time = 0.0;
        self.last_flush_time = 0.0;
        self.min_restart_time = 0.0;
        self.need_flush_time = 0.0;
        self.step_gen_time = 0.0;
        self.clear_history_time = 0.0;
        self.print_stall = 0;
    }

    /// Process flushed moves: trapq_append for XYZ + extruder, all in native code.
    fn processMoves(self: *MotionEngine, moves: []MoveData) void {
        var next_move_time = self.print_time;

        for (moves) |*m| {
            // XYZ kinematics trapq
            if (m.is_kinematic_move) {
                if (self.trapq) |tq| {
                    c.trapq_append(
                        tq,
                        next_move_time,
                        m.accel_t,
                        m.cruise_t,
                        m.decel_t,
                        m.start_pos[0],
                        m.start_pos[1],
                        m.start_pos[2],
                        m.axes_r[0],
                        m.axes_r[1],
                        m.axes_r[2],
                        m.start_v,
                        m.cruise_v,
                        m.accel,
                    );
                }
            }

            // Extruder trapq (direct C call, replaces Python extruder.move())
            if (m.axes_d[3] != 0.0) {
                if (self.extruder_trapq) |etq| {
                    const axis_r = m.axes_r[3];
                    var pa = self.extruder_pressure_advance;
                    // Only apply pressure advance for forward extrusion with XY movement
                    if (axis_r <= 0.0 or (m.axes_d[0] == 0.0 and m.axes_d[1] == 0.0)) {
                        pa = 0.0;
                    }
                    c.trapq_append(
                        etq,
                        next_move_time,
                        m.accel_t,
                        m.cruise_t,
                        m.decel_t,
                        m.start_pos[3], // x = extruder position
                        0.0, // y = 0
                        0.0, // z = 0
                        1.0, // x_r = 1 (extruder movement)
                        pa, // y_r = pressure_advance
                        self.extruder_use_pa_from_trapq, // z_r = use_pa_from_trapq flag
                        m.start_v * axis_r,
                        m.cruise_v * axis_r,
                        m.accel * axis_r,
                    );
                }
            }

            next_move_time += m.accel_t + m.cruise_t + m.decel_t;
        }

        // Update timing
        self.need_flush_time = @max(self.need_flush_time, next_move_time + self.kin_flush_delay);
        self.step_gen_time = @max(self.step_gen_time, next_move_time + self.kin_flush_delay);
        self.advanceMoveTime(next_move_time);
    }

    /// Advance flush time: call itersolve_generate_steps + steppersync_flush directly.
    fn advanceFlushTime(self: *MotionEngine, flush_time: f64) void {
        const ft = @max(flush_time, self.last_flush_time);

        const sg_flush_want = @min(
            ft + STEPCOMPRESS_FLUSH_TIME,
            self.print_time - self.kin_flush_delay,
        );
        const sg_flush_time = @max(sg_flush_want, ft);

        // Generate steps for all steppers (direct C calls)
        for (self.steppers[0..self.stepper_count]) |entry| {
            _ = c.itersolve_generate_steps(entry.sk, sg_flush_time);
        }
        self.min_restart_time = @max(self.min_restart_time, sg_flush_time);

        // Finalize trapq moves
        const free_time = sg_flush_time - self.kin_flush_delay;
        if (self.trapq) |tq| {
            c.trapq_finalize_moves(tq, free_time, self.clear_history_time);
        }
        // Also finalize extruder trapq
        if (self.extruder_trapq) |etq| {
            c.trapq_finalize_moves(etq, free_time, self.clear_history_time);
        }

        // NOTE: steppersync_flush is NOT called here. Python handles MCU
        // flushing because it owns the clock synchronization state (clock_adj
        // for secondary MCUs, steppersync_set_time calibration). The native
        // engine handles the expensive parts (itersolve + trapq), Python
        // handles the MCU flush which is once per batch, not per move.

        self.last_flush_time = ft;
    }

    fn advanceMoveTime(self: *MotionEngine, next_print_time: f64) void {
        const pt_delay = self.kin_flush_delay + STEPCOMPRESS_FLUSH_TIME;
        var flush_time = @max(self.last_flush_time, self.print_time - pt_delay);
        self.print_time = @max(self.print_time, next_print_time);
        const want_flush_time = @max(flush_time, self.print_time - pt_delay);

        while (true) {
            flush_time = @min(flush_time + MOVE_BATCH_TIME, want_flush_time);
            self.advanceFlushTime(flush_time);
            if (flush_time >= want_flush_time) break;
        }
    }

    pub fn flushLookahead(self: *MotionEngine, lazy: bool) void {
        const flush_count = self.lookahead.flush(lazy);
        if (flush_count == 0) return;

        const moves = self.lookahead.getFlushedMoves(flush_count);
        self.processMoves(moves);
        self.lookahead.consumeMoves(flush_count);

        // Notify Python if it registered a post-flush callback
        if (self.post_flush_cb) |cb| {
            cb(self.post_flush_ctx);
        }
    }

    pub fn flushAll(self: *MotionEngine) void {
        self.flushLookahead(false);
    }

    pub fn flushStepGeneration(self: *MotionEngine) void {
        self.flushAll();
        self.advanceFlushTime(self.step_gen_time);
        self.min_restart_time = @max(self.min_restart_time, self.print_time);
    }

    pub fn updateJunctionDeviation(self: *MotionEngine) void {
        const scv2 = self.square_corner_velocity * self.square_corner_velocity;
        self.junction_deviation = scv2 * (@sqrt(2.0) - 1.0) / self.max_accel;
        self.max_accel_to_decel = self.max_accel * 0.5;
    }
};

/// Result struct for extracting resolved velocity profiles back to Python.
pub const FlushedMoveResult = extern struct {
    start_v: f64,
    cruise_v: f64,
    end_v: f64,
    accel_t: f64,
    cruise_t: f64,
    decel_t: f64,
    accel: f64,
};

// ── C API ──

export fn motion_engine_create() ?*MotionEngine {
    const allocator = std.heap.c_allocator;
    const eng = allocator.create(MotionEngine) catch return null;
    eng.* = MotionEngine.init(allocator);
    return eng;
}

export fn motion_engine_destroy(eng: ?*MotionEngine) void {
    if (eng) |e| {
        e.deinit();
        std.heap.c_allocator.destroy(e);
    }
}

export fn motion_engine_reset(eng: *MotionEngine) void {
    eng.reset();
}

export fn motion_engine_set_trapq(eng: *MotionEngine, trapq: ?*c.Trapq) void {
    eng.trapq = trapq;
}

export fn motion_engine_set_extruder_trapq(eng: *MotionEngine, trapq: ?*c.Trapq) void {
    eng.extruder_trapq = trapq;
}

export fn motion_engine_set_extruder_params(
    eng: *MotionEngine,
    pressure_advance: f64,
    use_pa_from_trapq: f64,
    instant_corner_v: f64,
) void {
    eng.extruder_pressure_advance = pressure_advance;
    eng.extruder_use_pa_from_trapq = use_pa_from_trapq;
    eng.extruder_instant_corner_v = instant_corner_v;
}

/// Register a stepper's kinematics pointer for direct step generation.
export fn motion_engine_add_stepper(eng: *MotionEngine, sk: *c.StepperKinematics) i32 {
    if (eng.stepper_count >= MAX_STEPPERS) return -1;
    eng.steppers[eng.stepper_count] = .{ .sk = sk };
    eng.stepper_count += 1;
    return 0;
}

/// Register an MCU's steppersync pointer for direct flush.
/// time_offset: 0 for primary MCU, adjusted_offset for secondary MCUs.
/// mcu_freq: mcu_freq for primary MCU, adjusted_freq for secondary MCUs.
export fn motion_engine_add_mcu(
    eng: *MotionEngine,
    ss: *c.StepperSync,
    time_offset: f64,
    mcu_freq: f64,
) i32 {
    if (eng.mcu_count >= MAX_MCUS) return -1;
    eng.mcus[eng.mcu_count] = .{
        .steppersync = ss,
        .time_offset = time_offset,
        .mcu_freq = mcu_freq,
    };
    eng.mcu_count += 1;
    return 0;
}

/// Update MCU clock calibration (called when clock sync recalibrates).
export fn motion_engine_update_mcu_clock(
    eng: *MotionEngine,
    index: u32,
    time_offset: f64,
    mcu_freq: f64,
) void {
    if (index < eng.mcu_count) {
        eng.mcus[index].time_offset = time_offset;
        eng.mcus[index].mcu_freq = mcu_freq;
    }
}

/// Set a post-flush callback (for Python to handle timing_callbacks etc.)
export fn motion_engine_set_post_flush_cb(
    eng: *MotionEngine,
    cb: ?NotifyCallback,
    ctx: ?*anyopaque,
) void {
    eng.post_flush_cb = cb;
    eng.post_flush_ctx = ctx;
}

export fn motion_engine_queue_move(
    eng: *MotionEngine,
    x: f64,
    y: f64,
    z: f64,
    e: f64,
    speed: f64,
) i32 {
    var m = MoveData.init(
        eng.commanded_pos,
        .{ x, y, z, e },
        speed,
        eng.max_velocity,
        eng.max_accel,
        eng.max_accel_to_decel,
        eng.junction_deviation,
    );
    if (m.move_d == 0.0) return 0;
    eng.commanded_pos = m.end_pos;
    const should_flush = eng.lookahead.addMove(m) catch return -1;
    if (should_flush) {
        eng.flushLookahead(true);
        return 1;
    }
    return 0;
}

/// Queue a move with post-check_move velocity limits applied by Python.
export fn motion_engine_queue_move_ex(
    eng: *MotionEngine,
    start_x: f64,
    start_y: f64,
    start_z: f64,
    start_e: f64,
    end_x: f64,
    end_y: f64,
    end_z: f64,
    end_e: f64,
    _: f64, // speed (unused — max_cruise_v2 encodes this)
    accel: f64,
    max_cruise_v2: f64,
    delta_v2: f64,
    smooth_delta_v2: f64,
    next_junction_v2: f64,
    is_kinematic: i32,
) i32 {
    const start_pos = Pos{ start_x, start_y, start_z, start_e };
    const end_pos = Pos{ end_x, end_y, end_z, end_e };

    var m: MoveData = undefined;
    m.start_pos = start_pos;
    m.end_pos = end_pos;
    m.axes_d = .{
        end_pos[0] - start_pos[0],
        end_pos[1] - start_pos[1],
        end_pos[2] - start_pos[2],
        end_pos[3] - start_pos[3],
    };
    m.move_d = @sqrt(m.axes_d[0] * m.axes_d[0] +
        m.axes_d[1] * m.axes_d[1] +
        m.axes_d[2] * m.axes_d[2]);
    if (m.move_d < 0.000000001) {
        m.move_d = @abs(m.axes_d[3]);
    }
    if (m.move_d == 0.0) return 0;

    const inv_move_d = 1.0 / m.move_d;
    m.axes_r = .{
        m.axes_d[0] * inv_move_d,
        m.axes_d[1] * inv_move_d,
        m.axes_d[2] * inv_move_d,
        m.axes_d[3] * inv_move_d,
    };
    m.accel = accel;
    m.max_start_v2 = 0.0;
    m.max_cruise_v2 = max_cruise_v2;
    m.delta_v2 = delta_v2;
    m.max_smoothed_v2 = 0.0;
    m.smooth_delta_v2 = smooth_delta_v2;
    m.next_junction_v2 = next_junction_v2;
    m.min_move_t = m.move_d / @sqrt(max_cruise_v2);
    m.junction_deviation = eng.junction_deviation;
    m.is_kinematic_move = is_kinematic != 0;
    m.start_v = 0.0;
    m.cruise_v = 0.0;
    m.end_v = 0.0;
    m.accel_t = 0.0;
    m.cruise_t = 0.0;
    m.decel_t = 0.0;

    eng.commanded_pos = end_pos;
    _ = eng.lookahead.addMove(m) catch return -1;
    // Don't auto-flush — Python controls flush timing via flush_and_extract
    return 0;
}

export fn motion_engine_flush(eng: *MotionEngine) void {
    eng.flushAll();
}

export fn motion_engine_flush_step_generation(eng: *MotionEngine) void {
    eng.flushStepGeneration();
}

/// Flush lookahead and extract results (for hybrid Python integration fallback).
export fn motion_engine_flush_and_extract(
    eng: *MotionEngine,
    results: [*]FlushedMoveResult,
    max_results: u32,
    lazy: i32,
) i32 {
    const flush_count = eng.lookahead.flush(lazy != 0);
    if (flush_count == 0) return 0;
    const moves = eng.lookahead.getFlushedMoves(flush_count);
    const count = @min(flush_count, @as(usize, max_results));
    for (0..count) |i| {
        results[i] = .{
            .start_v = moves[i].start_v,
            .cruise_v = moves[i].cruise_v,
            .end_v = moves[i].end_v,
            .accel_t = moves[i].accel_t,
            .cruise_t = moves[i].cruise_t,
            .decel_t = moves[i].decel_t,
            .accel = moves[i].accel,
        };
    }
    eng.lookahead.consumeMoves(flush_count);
    return @intCast(count);
}

export fn motion_engine_get_print_time(eng: *const MotionEngine) f64 {
    return eng.print_time;
}

export fn motion_engine_get_buffer_time(eng: *const MotionEngine, est_print_time: f64) f64 {
    return eng.print_time - est_print_time;
}

export fn motion_engine_set_position(eng: *MotionEngine, x: f64, y: f64, z: f64, e: f64) void {
    eng.commanded_pos = .{ x, y, z, e };
}

export fn motion_engine_set_velocity_limits(
    eng: *MotionEngine,
    max_velocity: f64,
    max_accel: f64,
    square_corner_velocity: f64,
    min_cruise_ratio: f64,
) void {
    eng.max_velocity = max_velocity;
    eng.max_accel = max_accel;
    eng.square_corner_velocity = square_corner_velocity;
    eng.max_accel_to_decel = max_accel * (1.0 - min_cruise_ratio);
    eng.updateJunctionDeviation();
}

export fn motion_engine_get_stall_count(eng: *const MotionEngine) u32 {
    return eng.print_stall;
}

export fn motion_engine_set_print_time(eng: *MotionEngine, print_time: f64) void {
    eng.print_time = print_time;
}

/// Sync all timing state from Python after a period where Python handled
/// moves directly (e.g. drip_move during homing).
export fn motion_engine_sync_state(
    eng: *MotionEngine,
    print_time: f64,
    last_flush_time: f64,
    min_restart_time: f64,
    need_flush_time: f64,
    step_gen_time: f64,
    clear_history_time: f64,
    pos_x: f64,
    pos_y: f64,
    pos_z: f64,
    pos_e: f64,
) void {
    eng.print_time = print_time;
    eng.last_flush_time = last_flush_time;
    eng.min_restart_time = min_restart_time;
    eng.need_flush_time = need_flush_time;
    eng.step_gen_time = step_gen_time;
    eng.clear_history_time = clear_history_time;
    eng.commanded_pos = .{ pos_x, pos_y, pos_z, pos_e };
    eng.lookahead.reset();
}

export fn motion_engine_set_kin_flush_delay(eng: *MotionEngine, delay: f64) void {
    eng.kin_flush_delay = delay;
}

export fn motion_engine_get_last_flush_time(eng: *const MotionEngine) f64 {
    return eng.last_flush_time;
}

export fn motion_engine_get_queue_len(eng: *const MotionEngine) u32 {
    return @intCast(eng.lookahead.len());
}

// Tests

test "engine create and destroy" {
    const allocator = std.testing.allocator;
    var eng = MotionEngine.init(allocator);
    defer eng.deinit();

    eng.max_velocity = 300.0;
    eng.max_accel = 5000.0;
    eng.updateJunctionDeviation();
    try std.testing.expect(eng.junction_deviation > 0);
}

test "engine queue and flush moves" {
    const allocator = std.testing.allocator;
    var eng = MotionEngine.init(allocator);
    defer eng.deinit();

    eng.max_velocity = 100.0;
    eng.max_accel = 3000.0;
    eng.max_accel_to_decel = 1500.0;
    eng.square_corner_velocity = 5.0;
    eng.updateJunctionDeviation();

    _ = try eng.lookahead.addMove(MoveData.init(.{ 0, 0, 0, 0 }, .{ 10, 0, 0, 0 }, 100.0, 100.0, 3000.0, 1500.0, eng.junction_deviation));
    _ = try eng.lookahead.addMove(MoveData.init(.{ 10, 0, 0, 0 }, .{ 20, 0, 0, 0 }, 100.0, 100.0, 3000.0, 1500.0, eng.junction_deviation));
    _ = try eng.lookahead.addMove(MoveData.init(.{ 20, 0, 0, 0 }, .{ 30, 0, 0, 0 }, 100.0, 100.0, 3000.0, 1500.0, eng.junction_deviation));

    try std.testing.expect(eng.lookahead.len() == 3);
    eng.flushAll();
    try std.testing.expect(eng.lookahead.len() == 0);
}
