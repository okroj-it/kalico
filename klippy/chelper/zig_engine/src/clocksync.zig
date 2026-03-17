// Native clock synchronization.
// Direct translation of klippy/clocksync.py ClockSync class.
//
// Maintains a linear regression model mapping system time to MCU clock ticks.
// This must be fast and jitter-free because estimated_print_time() is called
// in every timing-critical path.

const std = @import("std");

const RTT_AGE: f64 = 0.000010 / (60.0 * 60.0);
const DECAY: f64 = 1.0 / 30.0;
const TRANSMIT_EXTRA: f64 = 0.001;

pub const ClockEstimate = struct {
    sample_time: f64,
    clock: f64,
    freq: f64,
};

pub const ClockSync = struct {
    mcu_freq: f64,
    last_clock: u64,
    clock_est: ClockEstimate,

    // Minimum round-trip-time tracking
    min_half_rtt: f64,
    min_rtt_time: f64,

    // Linear regression state
    time_avg: f64,
    time_variance: f64,
    clock_avg: f64,
    clock_covariance: f64,
    prediction_variance: f64,
    last_prediction_time: f64,

    pub fn init(mcu_freq: f64) ClockSync {
        return .{
            .mcu_freq = mcu_freq,
            .last_clock = 0,
            .clock_est = .{ .sample_time = 0.0, .clock = 0.0, .freq = mcu_freq },
            .min_half_rtt = 999999999.9,
            .min_rtt_time = 0.0,
            .time_avg = 0.0,
            .time_variance = 0.0,
            .clock_avg = 0.0,
            .clock_covariance = 0.0,
            .prediction_variance = (0.001 * mcu_freq) * (0.001 * mcu_freq),
            .last_prediction_time = 0.0,
        };
    }

    /// Set the initial clock state after connecting.
    pub fn setInitial(self: *ClockSync, time: f64, clock: u64) void {
        self.last_clock = clock;
        self.clock_avg = @floatFromInt(clock);
        self.time_avg = time;
        self.clock_est = .{
            .sample_time = time,
            .clock = self.clock_avg,
            .freq = self.mcu_freq,
        };
        self.prediction_variance = (0.001 * self.mcu_freq) * (0.001 * self.mcu_freq);
    }

    /// Process a clock sample from the MCU.
    /// Direct translation of Python ClockSync._handle_clock
    ///
    /// Returns the updated frequency estimate, or null if sample was rejected.
    pub fn update(
        self: *ClockSync,
        clock32: u32,
        sent_time: f64,
        receive_time: f64,
    ) ?f64 {
        if (sent_time == 0.0) return null;

        // Extend clock to 64-bit
        const last_clock = self.last_clock;
        const clock_delta = (@as(u64, clock32) -% @as(u32, @truncate(last_clock))) & 0xFFFFFFFF;
        const clock = last_clock +% clock_delta;
        self.last_clock = clock;

        const clock_f: f64 = @floatFromInt(clock);

        // Check if this is the best RTT so far
        const half_rtt = 0.5 * (receive_time - sent_time);
        const aged_rtt = (sent_time - self.min_rtt_time) * RTT_AGE;
        if (half_rtt < self.min_half_rtt + aged_rtt) {
            self.min_half_rtt = half_rtt;
            self.min_rtt_time = sent_time;
        }

        // Filter out extreme outliers
        const exp_clock = (sent_time - self.time_avg) * self.clock_est.freq + self.clock_avg;
        const clock_diff = clock_f - exp_clock;
        const clock_diff2 = clock_diff * clock_diff;
        const freq_threshold = 0.000500 * self.mcu_freq;

        if (clock_diff2 > 25.0 * self.prediction_variance and
            clock_diff2 > freq_threshold * freq_threshold)
        {
            if (clock_f > exp_clock and
                sent_time < self.last_prediction_time + 10.0)
            {
                // Ignore this sample
                return null;
            }
            // Reset prediction variance
            self.prediction_variance = (0.001 * self.mcu_freq) * (0.001 * self.mcu_freq);
        } else {
            self.last_prediction_time = sent_time;
            self.prediction_variance = (1.0 - DECAY) *
                (self.prediction_variance + clock_diff2 * DECAY);
        }

        // Update linear regression
        const diff_sent_time = sent_time - self.time_avg;
        self.time_avg += DECAY * diff_sent_time;
        self.time_variance = (1.0 - DECAY) *
            (self.time_variance + diff_sent_time * diff_sent_time * DECAY);

        const diff_clock = clock_f - self.clock_avg;
        self.clock_avg += DECAY * diff_clock;
        self.clock_covariance = (1.0 - DECAY) *
            (self.clock_covariance + diff_sent_time * diff_clock * DECAY);

        // Compute new frequency from regression
        const new_freq = if (self.time_variance > 0.0)
            self.clock_covariance / self.time_variance
        else
            self.mcu_freq;

        self.clock_est = .{
            .sample_time = self.time_avg + self.min_half_rtt,
            .clock = self.clock_avg,
            .freq = new_freq,
        };

        return new_freq;
    }

    // Clock/time conversions

    pub fn printTimeToClock(self: *const ClockSync, print_time: f64) i64 {
        return @intFromFloat(print_time * self.mcu_freq);
    }

    pub fn clockToPrintTime(self: *const ClockSync, clock: i64) f64 {
        return @as(f64, @floatFromInt(clock)) / self.mcu_freq;
    }

    pub fn getClock(self: *const ClockSync, eventtime: f64) i64 {
        const est = self.clock_est;
        return @intFromFloat(est.clock + (eventtime - est.sample_time) * est.freq);
    }

    pub fn estimatedPrintTime(self: *const ClockSync, eventtime: f64) f64 {
        return self.clockToPrintTime(self.getClock(eventtime));
    }
};

// ── C API ──

export fn clock_sync_create(mcu_freq: f64) ?*ClockSync {
    const allocator = std.heap.c_allocator;
    const cs = allocator.create(ClockSync) catch return null;
    cs.* = ClockSync.init(mcu_freq);
    return cs;
}

export fn clock_sync_destroy(cs: ?*ClockSync) void {
    if (cs) |ptr| {
        std.heap.c_allocator.destroy(ptr);
    }
}

export fn clock_sync_set_freq(cs: *ClockSync, mcu_freq: f64) void {
    cs.mcu_freq = mcu_freq;
}

export fn clock_sync_update(
    cs: *ClockSync,
    clock32: u32,
    sent_time: f64,
    receive_time: f64,
) f64 {
    return cs.update(clock32, sent_time, receive_time) orelse -1.0;
}

export fn clock_sync_get_clock(cs: *const ClockSync, eventtime: f64) i64 {
    return cs.getClock(eventtime);
}

export fn clock_sync_estimated_print_time(cs: *const ClockSync, eventtime: f64) f64 {
    return cs.estimatedPrintTime(eventtime);
}

// Tests

test "clock_sync basic" {
    var cs = ClockSync.init(48000000.0);
    cs.setInitial(1000.0, 48000000000);

    // Simulate a clock sample with ~0.5ms RTT
    const sent = 1001.0;
    const recv = 1001.0005;
    const clock32: u32 = @truncate(48000000000 + 48000000);

    const freq = cs.update(clock32, sent, recv);
    try std.testing.expect(freq != null);
}

test "clock_sync estimated_print_time" {
    var cs = ClockSync.init(48000000.0);
    cs.setInitial(0.0, 0);

    const pt = cs.estimatedPrintTime(1.0);
    // At t=1.0s with 48MHz clock, print_time should be ~1.0
    try std.testing.expectApproxEqAbs(1.0, pt, 0.01);
}
