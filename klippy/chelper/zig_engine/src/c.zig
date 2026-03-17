// C interop: imports existing chelper headers so we can call trapq, itersolve, etc.

pub const c = @cImport({
    @cInclude("trapq.h");
    @cInclude("itersolve.h");
    @cInclude("stepcompress.h");
    @cInclude("pyhelper.h");
    @cInclude("pollreactor.h");
    @cInclude("serialqueue.h");
});

// Re-export commonly used types
pub const Trapq = c.struct_trapq;
pub const CMove = c.struct_move;
pub const Coord = c.struct_coord;
pub const PullMove = c.struct_pull_move;
pub const StepperKinematics = c.struct_stepper_kinematics;
pub const StepperSync = c.struct_steppersync;

// Re-export commonly used functions
pub const trapq_alloc = c.trapq_alloc;
pub const trapq_free = c.trapq_free;
pub const trapq_append = c.trapq_append;
pub const trapq_finalize_moves = c.trapq_finalize_moves;
pub const trapq_set_position = c.trapq_set_position;
pub const get_monotonic = c.get_monotonic;
pub const itersolve_generate_steps = c.itersolve_generate_steps;
pub const itersolve_check_active = c.itersolve_check_active;
pub const steppersync_flush = c.steppersync_flush;
