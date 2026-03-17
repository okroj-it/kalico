// Native Move construction and junction calculation.
// Direct translation of klippy/toolhead.py Move class.

const std = @import("std");
const math = std.math;

/// 4-axis position (X, Y, Z, E)
pub const Pos = [4]f64;

/// A planned move with velocity profile.
/// Replaces Python's toolhead.Move class.
pub const MoveData = struct {
    start_pos: Pos,
    end_pos: Pos,
    axes_d: Pos, // delta per axis
    axes_r: Pos, // ratio per axis (normalized)
    move_d: f64, // total XYZ distance

    // Velocity planning
    accel: f64,
    max_start_v2: f64, // max start velocity squared
    max_cruise_v2: f64, // max cruise velocity squared
    delta_v2: f64, // max velocity^2 change in this move
    max_smoothed_v2: f64,
    smooth_delta_v2: f64,
    next_junction_v2: f64,
    min_move_t: f64, // minimum move time
    junction_deviation: f64,

    is_kinematic_move: bool,

    // Set by set_junction after lookahead
    start_v: f64,
    cruise_v: f64,
    end_v: f64,
    accel_t: f64,
    cruise_t: f64,
    decel_t: f64,

    /// Initialize a move from start/end positions and speed/accel limits.
    /// Direct translation of Python Move.__init__
    pub fn init(
        start_pos: Pos,
        end_pos: Pos,
        speed: f64,
        max_velocity: f64,
        max_accel: f64,
        max_accel_to_decel: f64,
        junction_deviation: f64,
    ) MoveData {
        var self: MoveData = undefined;
        self.start_pos = start_pos;
        self.end_pos = end_pos;
        self.accel = max_accel;
        self.junction_deviation = junction_deviation;
        self.max_start_v2 = 0.0;
        self.max_smoothed_v2 = 0.0;
        self.next_junction_v2 = 999999999.9;

        // Computed fields (set after junction)
        self.start_v = 0.0;
        self.cruise_v = 0.0;
        self.end_v = 0.0;
        self.accel_t = 0.0;
        self.cruise_t = 0.0;
        self.decel_t = 0.0;

        const velocity = @min(speed, max_velocity);

        self.is_kinematic_move = true;
        self.axes_d = .{
            end_pos[0] - start_pos[0],
            end_pos[1] - start_pos[1],
            end_pos[2] - start_pos[2],
            end_pos[3] - start_pos[3],
        };

        self.move_d = @sqrt(
            self.axes_d[0] * self.axes_d[0] +
                self.axes_d[1] * self.axes_d[1] +
                self.axes_d[2] * self.axes_d[2],
        );

        var actual_velocity = velocity;
        var inv_move_d: f64 = undefined;

        if (self.move_d < 0.000000001) {
            // Extrude only move
            self.end_pos = .{
                start_pos[0],
                start_pos[1],
                start_pos[2],
                end_pos[3],
            };
            self.axes_d[0] = 0.0;
            self.axes_d[1] = 0.0;
            self.axes_d[2] = 0.0;
            self.move_d = @abs(self.axes_d[3]);
            inv_move_d = if (self.move_d != 0.0) 1.0 / self.move_d else 0.0;
            self.accel = 99999999.9;
            actual_velocity = speed;
            self.is_kinematic_move = false;
        } else {
            inv_move_d = 1.0 / self.move_d;
        }

        self.axes_r = .{
            self.axes_d[0] * inv_move_d,
            self.axes_d[1] * inv_move_d,
            self.axes_d[2] * inv_move_d,
            self.axes_d[3] * inv_move_d,
        };

        self.min_move_t = self.move_d / actual_velocity;
        self.max_cruise_v2 = actual_velocity * actual_velocity;
        self.delta_v2 = 2.0 * self.move_d * self.accel;
        self.smooth_delta_v2 = 2.0 * self.move_d * max_accel_to_decel;

        return self;
    }

    /// Apply speed/accel limits (from kinematics check_move).
    pub fn limitSpeed(self: *MoveData, speed: f64, accel: f64) void {
        const speed2 = speed * speed;
        if (speed2 < self.max_cruise_v2) {
            self.max_cruise_v2 = speed2;
            self.min_move_t = self.move_d / speed;
        }
        self.accel = @min(self.accel, accel);
        self.delta_v2 = 2.0 * self.move_d * self.accel;
        self.smooth_delta_v2 = @min(self.smooth_delta_v2, self.delta_v2);
    }

    pub fn limitNextJunctionSpeed(self: *MoveData, speed: f64) void {
        self.next_junction_v2 = @min(self.next_junction_v2, speed * speed);
    }

    /// Calculate junction speed with previous move.
    /// Direct translation of Python Move.calc_junction
    pub fn calcJunction(self: *MoveData, prev: *const MoveData) void {
        if (!self.is_kinematic_move or !prev.is_kinematic_move) return;

        // Extruder junction calculation would be called separately by Python
        var max_start_v2 = @min(
            self.max_cruise_v2,
            prev.max_cruise_v2,
            prev.next_junction_v2,
            prev.max_start_v2 + prev.delta_v2,
        );

        // Approximated centripetal velocity
        const axes_r = self.axes_r;
        const prev_axes_r = prev.axes_r;
        const junction_cos_theta = -(axes_r[0] * prev_axes_r[0] +
            axes_r[1] * prev_axes_r[1] +
            axes_r[2] * prev_axes_r[2]);

        const sin_theta_d2 = @sqrt(@max(0.5 * (1.0 - junction_cos_theta), 0.0));
        const cos_theta_d2 = @sqrt(@max(0.5 * (1.0 + junction_cos_theta), 0.0));
        const one_minus_sin_theta_d2 = 1.0 - sin_theta_d2;

        if (one_minus_sin_theta_d2 > 0.0 and cos_theta_d2 > 0.0) {
            const r_jd = sin_theta_d2 / one_minus_sin_theta_d2;
            const move_jd_v2 = r_jd * self.junction_deviation * self.accel;
            const pmove_jd_v2 = r_jd * prev.junction_deviation * prev.accel;
            const quarter_tan_theta_d2 = 0.25 * sin_theta_d2 / cos_theta_d2;
            const move_centripetal_v2 = self.delta_v2 * quarter_tan_theta_d2;
            const pmove_centripetal_v2 = prev.delta_v2 * quarter_tan_theta_d2;
            max_start_v2 = @min(
                max_start_v2,
                move_jd_v2,
                pmove_jd_v2,
                move_centripetal_v2,
                pmove_centripetal_v2,
            );
        }

        self.max_start_v2 = max_start_v2;
        self.max_smoothed_v2 = @min(
            max_start_v2,
            prev.max_smoothed_v2 + prev.smooth_delta_v2,
        );
    }

    /// Calculate junction speed with previous move, with extruder junction limit.
    pub fn calcJunctionWithExtruder(self: *MoveData, prev: *const MoveData, extruder_v2: f64) void {
        if (!self.is_kinematic_move or !prev.is_kinematic_move) return;

        var max_start_v2 = @min(
            extruder_v2,
            self.max_cruise_v2,
            prev.max_cruise_v2,
            prev.next_junction_v2,
            prev.max_start_v2 + prev.delta_v2,
        );

        const axes_r = self.axes_r;
        const prev_axes_r = prev.axes_r;
        const junction_cos_theta = -(axes_r[0] * prev_axes_r[0] +
            axes_r[1] * prev_axes_r[1] +
            axes_r[2] * prev_axes_r[2]);

        const sin_theta_d2 = @sqrt(@max(0.5 * (1.0 - junction_cos_theta), 0.0));
        const cos_theta_d2 = @sqrt(@max(0.5 * (1.0 + junction_cos_theta), 0.0));
        const one_minus_sin_theta_d2 = 1.0 - sin_theta_d2;

        if (one_minus_sin_theta_d2 > 0.0 and cos_theta_d2 > 0.0) {
            const r_jd = sin_theta_d2 / one_minus_sin_theta_d2;
            const move_jd_v2 = r_jd * self.junction_deviation * self.accel;
            const pmove_jd_v2 = r_jd * prev.junction_deviation * prev.accel;
            const quarter_tan_theta_d2 = 0.25 * sin_theta_d2 / cos_theta_d2;
            const move_centripetal_v2 = self.delta_v2 * quarter_tan_theta_d2;
            const pmove_centripetal_v2 = prev.delta_v2 * quarter_tan_theta_d2;
            max_start_v2 = @min(
                max_start_v2,
                move_jd_v2,
                pmove_jd_v2,
                move_centripetal_v2,
                pmove_centripetal_v2,
            );
        }

        self.max_start_v2 = max_start_v2;
        self.max_smoothed_v2 = @min(
            max_start_v2,
            prev.max_smoothed_v2 + prev.smooth_delta_v2,
        );
    }

    /// Set the velocity profile after lookahead resolves junction speeds.
    /// Direct translation of Python Move.set_junction
    pub fn setJunction(self: *MoveData, start_v2: f64, cruise_v2: f64, end_v2: f64) void {
        const half_inv_accel = 0.5 / self.accel;
        const accel_d = (cruise_v2 - start_v2) * half_inv_accel;
        const decel_d = (cruise_v2 - end_v2) * half_inv_accel;
        const cruise_d = self.move_d - accel_d - decel_d;

        self.start_v = @sqrt(start_v2);
        self.cruise_v = @sqrt(cruise_v2);
        self.end_v = @sqrt(end_v2);

        self.accel_t = accel_d / ((self.start_v + self.cruise_v) * 0.5);
        self.cruise_t = cruise_d / self.cruise_v;
        self.decel_t = decel_d / ((self.end_v + self.cruise_v) * 0.5);
    }

    /// Total time for this move.
    pub fn totalTime(self: *const MoveData) f64 {
        return self.accel_t + self.cruise_t + self.decel_t;
    }
};

// Tests
test "move init basic" {
    const m = MoveData.init(
        .{ 0, 0, 0, 0 },
        .{ 10, 0, 0, 0 },
        100.0, // speed
        500.0, // max_velocity
        3000.0, // max_accel
        1500.0, // max_accel_to_decel
        0.01, // junction_deviation
    );
    try std.testing.expect(m.is_kinematic_move);
    try std.testing.expectApproxEqAbs(10.0, m.move_d, 1e-9);
    try std.testing.expectApproxEqAbs(0.1, m.min_move_t, 1e-9);
}

test "move init extrude only" {
    const m = MoveData.init(
        .{ 10, 20, 30, 0 },
        .{ 10, 20, 30, 5 },
        50.0,
        500.0,
        3000.0,
        1500.0,
        0.01,
    );
    try std.testing.expect(!m.is_kinematic_move);
    try std.testing.expectApproxEqAbs(5.0, m.move_d, 1e-9);
}

test "move junction calculation" {
    var m1 = MoveData.init(
        .{ 0, 0, 0, 0 },
        .{ 10, 0, 0, 0 },
        100.0,
        500.0,
        3000.0,
        1500.0,
        0.01,
    );
    m1.max_start_v2 = 100.0 * 100.0;

    var m2 = MoveData.init(
        .{ 10, 0, 0, 0 },
        .{ 20, 0, 0, 0 },
        100.0,
        500.0,
        3000.0,
        1500.0,
        0.01,
    );

    // Collinear moves — junction should allow high speed
    m2.calcJunction(&m1);
    try std.testing.expect(m2.max_start_v2 > 0);
}

test "move set_junction" {
    var m = MoveData.init(
        .{ 0, 0, 0, 0 },
        .{ 10, 0, 0, 0 },
        100.0,
        500.0,
        3000.0,
        1500.0,
        0.01,
    );
    m.setJunction(0.0, 100.0 * 100.0, 0.0);
    try std.testing.expect(m.accel_t > 0);
    try std.testing.expect(m.decel_t > 0);
    try std.testing.expectApproxEqAbs(100.0, m.cruise_v, 1e-9);
}
