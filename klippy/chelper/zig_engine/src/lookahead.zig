// Native look-ahead queue with velocity planning.
// Direct translation of klippy/toolhead.py LookAheadQueue class.
//
// The look-ahead algorithm traverses the move queue backwards to determine
// maximum junction speeds, then forwards to set velocity profiles.
// This is the core algorithm that must run fast to avoid timing stalls.

const std = @import("std");
const MoveData = @import("move.zig").MoveData;

pub const LOOKAHEAD_FLUSH_TIME: f64 = 0.250;
const MAX_QUEUE_SIZE: usize = 4096;

pub const LookAheadQueue = struct {
    queue: std.ArrayList(MoveData),
    allocator: std.mem.Allocator,
    junction_flush: f64,

    pub fn init(allocator: std.mem.Allocator) LookAheadQueue {
        return .{
            .queue = .empty,
            .allocator = allocator,
            .junction_flush = LOOKAHEAD_FLUSH_TIME,
        };
    }

    pub fn deinit(self: *LookAheadQueue) void {
        self.queue.deinit(self.allocator);
    }

    pub fn reset(self: *LookAheadQueue) void {
        self.queue.clearRetainingCapacity();
        self.junction_flush = LOOKAHEAD_FLUSH_TIME;
    }

    pub fn setFlushTime(self: *LookAheadQueue, flush_time: f64) void {
        self.junction_flush = flush_time;
    }

    pub fn getLast(self: *LookAheadQueue) ?*MoveData {
        if (self.queue.items.len == 0) return null;
        return &self.queue.items[self.queue.items.len - 1];
    }

    pub fn len(self: *const LookAheadQueue) usize {
        return self.queue.items.len;
    }

    /// Add a move and calculate its junction with the previous move.
    /// Returns true if the queue should be flushed (enough time accumulated).
    pub fn addMove(self: *LookAheadQueue, m: MoveData) !bool {
        try self.queue.append(self.allocator, m);
        const items = self.queue.items;
        if (items.len > 1) {
            items[items.len - 1].calcJunction(&items[items.len - 2]);
            self.junction_flush -= m.min_move_t;
            if (self.junction_flush <= 0.0) {
                return true; // Signal: time to flush
            }
        }
        return false;
    }

    /// Add a move with extruder junction calculation.
    pub fn addMoveWithExtruder(self: *LookAheadQueue, m: MoveData, extruder_v2: f64) !bool {
        try self.queue.append(self.allocator, m);
        const items = self.queue.items;
        if (items.len > 1) {
            items[items.len - 1].calcJunctionWithExtruder(&items[items.len - 2], extruder_v2);
            self.junction_flush -= m.min_move_t;
            if (self.junction_flush <= 0.0) {
                return true;
            }
        }
        return false;
    }

    /// Flush the look-ahead queue: resolve junction speeds and return
    /// moves ready to be processed.
    ///
    /// When lazy=true, only flushes up to the point where the velocity
    /// profile is fully determined (optimizes for streaming).
    ///
    /// Returns the number of moves flushed (0 means nothing ready).
    /// Direct translation of Python LookAheadQueue.flush
    pub fn flush(self: *LookAheadQueue, lazy: bool) usize {
        self.junction_flush = LOOKAHEAD_FLUSH_TIME;
        var update_flush_count = lazy;
        const items = self.queue.items;
        var flush_count = items.len;

        if (flush_count == 0) return 0;

        // Traverse queue from last to first move and determine maximum
        // junction speed assuming the robot comes to a complete stop
        // after the last move.
        var next_end_v2: f64 = 0.0;
        var next_smoothed_v2: f64 = 0.0;
        var peak_cruise_v2: f64 = 0.0;

        // Delayed moves: (index, start_v2, end_v2)
        var delayed_buf: [MAX_QUEUE_SIZE]DelayedEntry = undefined;
        var delayed_count: usize = 0;

        var i: usize = flush_count;
        while (i > 0) {
            i -= 1;
            const m = &items[i];

            const reachable_start_v2 = next_end_v2 + m.delta_v2;
            const start_v2 = @min(m.max_start_v2, reachable_start_v2);
            const reachable_smoothed_v2 = next_smoothed_v2 + m.smooth_delta_v2;
            const smoothed_v2 = @min(m.max_smoothed_v2, reachable_smoothed_v2);

            if (smoothed_v2 < reachable_smoothed_v2) {
                // It's possible for this move to accelerate
                if (smoothed_v2 + m.smooth_delta_v2 > next_smoothed_v2 or delayed_count > 0) {
                    if (update_flush_count and peak_cruise_v2 != 0.0) {
                        flush_count = i;
                        update_flush_count = false;
                    }
                    peak_cruise_v2 = @min(
                        m.max_cruise_v2,
                        (smoothed_v2 + reachable_smoothed_v2) * 0.5,
                    );
                    if (delayed_count > 0) {
                        // Propagate peak_cruise_v2 to delayed moves
                        if (!update_flush_count and i < flush_count) {
                            var mc_v2 = peak_cruise_v2;
                            var d: usize = delayed_count;
                            while (d > 0) {
                                d -= 1;
                                const entry = &delayed_buf[d];
                                mc_v2 = @min(mc_v2, entry.start_v2);
                                items[entry.index].setJunction(
                                    @min(entry.start_v2, mc_v2),
                                    mc_v2,
                                    @min(entry.end_v2, mc_v2),
                                );
                            }
                        }
                        delayed_count = 0;
                    }
                }
                if (!update_flush_count and i < flush_count) {
                    const cruise_v2 = @min(
                        (start_v2 + reachable_start_v2) * 0.5,
                        m.max_cruise_v2,
                        peak_cruise_v2,
                    );
                    m.setJunction(
                        @min(start_v2, cruise_v2),
                        cruise_v2,
                        @min(next_end_v2, cruise_v2),
                    );
                }
            } else {
                // Delay calculating this move until peak_cruise_v2 is known
                if (delayed_count < MAX_QUEUE_SIZE) {
                    delayed_buf[delayed_count] = .{
                        .index = i,
                        .start_v2 = start_v2,
                        .end_v2 = next_end_v2,
                    };
                    delayed_count += 1;
                }
            }
            next_end_v2 = start_v2;
            next_smoothed_v2 = smoothed_v2;
        }

        if (update_flush_count or flush_count == 0) {
            return 0;
        }

        return flush_count;
    }

    /// Remove the first `count` moves from the queue after they've been processed.
    pub fn consumeMoves(self: *LookAheadQueue, count: usize) void {
        if (count >= self.queue.items.len) {
            self.queue.clearRetainingCapacity();
        } else {
            // Shift remaining items to front
            const items = self.queue.items;
            const remaining = items.len - count;
            std.mem.copyForwards(MoveData, items[0..remaining], items[count..items.len]);
            self.queue.shrinkRetainingCapacity(remaining);
        }
    }

    /// Get the flushed moves slice (first `count` items).
    pub fn getFlushedMoves(self: *LookAheadQueue, count: usize) []MoveData {
        return self.queue.items[0..count];
    }
};

const DelayedEntry = struct {
    index: usize,
    start_v2: f64,
    end_v2: f64,
};

// Tests
test "lookahead basic flush" {
    const allocator = std.testing.allocator;
    var laq = LookAheadQueue.init(allocator);
    defer laq.deinit();

    const m = MoveData.init(
        .{ 0, 0, 0, 0 },
        .{ 10, 0, 0, 0 },
        100.0,
        500.0,
        3000.0,
        1500.0,
        0.01,
    );
    _ = try laq.addMove(m);

    const flushed = laq.flush(false);
    try std.testing.expect(flushed == 1);

    const moves = laq.getFlushedMoves(flushed);
    try std.testing.expect(moves[0].cruise_v > 0);
}

test "lookahead collinear moves" {
    const allocator = std.testing.allocator;
    var laq = LookAheadQueue.init(allocator);
    defer laq.deinit();

    var i: usize = 0;
    while (i < 5) : (i += 1) {
        const x_start: f64 = @floatFromInt(i * 10);
        const x_end: f64 = @floatFromInt((i + 1) * 10);
        const m = MoveData.init(
            .{ x_start, 0, 0, 0 },
            .{ x_end, 0, 0, 0 },
            100.0,
            500.0,
            3000.0,
            1500.0,
            0.01,
        );
        _ = try laq.addMove(m);
    }

    const flushed = laq.flush(false);
    try std.testing.expect(flushed == 5);

    const moves = laq.getFlushedMoves(flushed);
    try std.testing.expect(moves[1].start_v > 0);
}

test "lookahead corner moves" {
    const allocator = std.testing.allocator;
    var laq = LookAheadQueue.init(allocator);
    defer laq.deinit();

    const m1 = MoveData.init(
        .{ 0, 0, 0, 0 },
        .{ 10, 0, 0, 0 },
        100.0,
        500.0,
        3000.0,
        1500.0,
        0.01,
    );
    _ = try laq.addMove(m1);

    const m2 = MoveData.init(
        .{ 10, 0, 0, 0 },
        .{ 10, 10, 0, 0 },
        100.0,
        500.0,
        3000.0,
        1500.0,
        0.01,
    );
    _ = try laq.addMove(m2);

    const flushed = laq.flush(false);
    try std.testing.expect(flushed == 2);

    const moves = laq.getFlushedMoves(flushed);
    try std.testing.expect(moves[1].start_v < 50.0);
}
