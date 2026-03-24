# Wrapper around C helper code
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import os

import cffi

######################################################################
# c_helper.so compiling
######################################################################

GCC_CMD = "gcc"
COMPILE_ARGS = (
    "-Wall -g -O2 -shared -fPIC"
    " -flto -fwhole-program -fno-use-linker-plugin"
    " -o %s %s"
)
SSE_FLAGS = "-mfpmath=sse -msse2"
SOURCE_FILES = [
    "pyhelper.c",
    "serialqueue.c",
    "stepcompress.c",
    "itersolve.c",
    "trapq.c",
    "pollreactor.c",
    "msgblock.c",
    "trdispatch.c",
    "kin_cartesian.c",
    "kin_corexy.c",
    "kin_corexz.c",
    "kin_delta.c",
    "kin_deltesian.c",
    "kin_polar.c",
    "kin_rotary_delta.c",
    "kin_winch.c",
    "kin_extruder.c",
    "kin_shaper.c",
    "kin_idex.c",
]
DEST_LIB = "c_helper.so"
OTHER_FILES = [
    "list.h",
    "serialqueue.h",
    "stepcompress.h",
    "itersolve.h",
    "pyhelper.h",
    "trapq.h",
    "pollreactor.h",
    "msgblock.h",
]

defs_stepcompress = """
    struct pull_history_steps {
        uint64_t first_clock, last_clock;
        int64_t start_position;
        int step_count, interval, add;
    };

    struct stepcompress *stepcompress_alloc(uint32_t oid);
    void stepcompress_fill(struct stepcompress *sc, uint32_t max_error
        , int32_t queue_step_msgtag, int32_t set_next_step_dir_msgtag);
    void stepcompress_set_invert_sdir(struct stepcompress *sc
        , uint32_t invert_sdir);
    void stepcompress_free(struct stepcompress *sc);
    int stepcompress_reset(struct stepcompress *sc, uint64_t last_step_clock);
    int stepcompress_set_last_position(struct stepcompress *sc
        , uint64_t clock, int64_t last_position);
    int64_t stepcompress_find_past_position(struct stepcompress *sc
        , uint64_t clock);
    int stepcompress_queue_msg(struct stepcompress *sc
        , uint32_t *data, int len);
    int stepcompress_queue_mq_msg(struct stepcompress *sc, uint64_t req_clock
        , uint32_t *data, int len);
    int stepcompress_extract_old(struct stepcompress *sc
        , struct pull_history_steps *p, int max
        , uint64_t start_clock, uint64_t end_clock);

    struct steppersync *steppersync_alloc(struct serialqueue *sq
        , struct stepcompress **sc_list, int sc_num, int move_num);
    void steppersync_free(struct steppersync *ss);
    void steppersync_set_time(struct steppersync *ss
        , double time_offset, double mcu_freq);
    int steppersync_flush(struct steppersync *ss, uint64_t move_clock
        , uint64_t clear_history_clock);
"""

defs_itersolve = """
    int32_t itersolve_generate_steps(struct stepper_kinematics *sk
        , double flush_time);
    double itersolve_check_active(struct stepper_kinematics *sk
        , double flush_time);
    int32_t itersolve_is_active_axis(struct stepper_kinematics *sk, char axis);
    void itersolve_set_trapq(struct stepper_kinematics *sk, struct trapq *tq);
    void itersolve_set_stepcompress(struct stepper_kinematics *sk
        , struct stepcompress *sc, double step_dist);
    double itersolve_calc_position_from_coord(struct stepper_kinematics *sk
        , double x, double y, double z);
    void itersolve_set_position(struct stepper_kinematics *sk
        , double x, double y, double z);
    double itersolve_get_commanded_pos(struct stepper_kinematics *sk);
"""

defs_trapq = """
    struct pull_move {
        double print_time, move_t;
        double start_v, accel;
        double start_x, start_y, start_z;
        double x_r, y_r, z_r;
    };

    struct trapq *trapq_alloc(void);
    void trapq_free(struct trapq *tq);
    void trapq_append(struct trapq *tq, double print_time
        , double accel_t, double cruise_t, double decel_t
        , double start_pos_x, double start_pos_y, double start_pos_z
        , double axes_r_x, double axes_r_y, double axes_r_z
        , double start_v, double cruise_v, double accel);
    void trapq_finalize_moves(struct trapq *tq, double print_time
        , double clear_history_time);
    void trapq_set_position(struct trapq *tq, double print_time
        , double pos_x, double pos_y, double pos_z);
    int trapq_extract_old(struct trapq *tq, struct pull_move *p, int max
        , double start_time, double end_time);
"""

defs_kin_cartesian = """
    struct stepper_kinematics *cartesian_stepper_alloc(char axis);
"""

defs_kin_corexy = """
    struct stepper_kinematics *corexy_stepper_alloc(char type);
"""

defs_kin_corexz = """
    struct stepper_kinematics *corexz_stepper_alloc(char type);
"""

defs_kin_delta = """
    struct stepper_kinematics *delta_stepper_alloc(double arm2
        , double tower_x, double tower_y);
"""

defs_kin_deltesian = """
    struct stepper_kinematics *deltesian_stepper_alloc(double arm2
        , double arm_x);
"""

defs_kin_polar = """
    struct stepper_kinematics *polar_stepper_alloc(char type);
"""

defs_kin_rotary_delta = """
    struct stepper_kinematics *rotary_delta_stepper_alloc(
        double shoulder_radius, double shoulder_height
        , double angle, double upper_arm, double lower_arm);
"""

defs_kin_winch = """
    struct stepper_kinematics *winch_stepper_alloc(double anchor_x
        , double anchor_y, double anchor_z);
"""

defs_kin_extruder = """
    struct stepper_kinematics *extruder_stepper_alloc(void);
    void extruder_set_pressure_advance(struct stepper_kinematics *sk
        , double pressure_advance, double smooth_time);
"""

defs_kin_shaper = """
    double input_shaper_get_step_generation_window(
        struct stepper_kinematics *sk);
    int input_shaper_set_shaper_params(struct stepper_kinematics *sk, char axis
        , int n, double a[], double t[]);
    int input_shaper_set_sk(struct stepper_kinematics *sk
        , struct stepper_kinematics *orig_sk);
    struct stepper_kinematics * input_shaper_alloc(void);
"""

defs_kin_idex = """
    void dual_carriage_set_sk(struct stepper_kinematics *sk
        , struct stepper_kinematics *orig_sk);
    int dual_carriage_set_transform(struct stepper_kinematics *sk
        , char axis, double scale, double offs);
    struct stepper_kinematics * dual_carriage_alloc(void);
"""

defs_serialqueue = """
    #define MESSAGE_MAX 64
    struct pull_queue_message {
        uint8_t msg[MESSAGE_MAX];
        int len;
        double sent_time, receive_time;
        uint64_t notify_id;
    };

    struct serialqueue *serialqueue_alloc(int serial_fd, char serial_fd_type
        , int client_id);
    void serialqueue_exit(struct serialqueue *sq);
    void serialqueue_free(struct serialqueue *sq);
    struct command_queue *serialqueue_alloc_commandqueue(void);
    void serialqueue_free_commandqueue(struct command_queue *cq);
    void serialqueue_send(struct serialqueue *sq, struct command_queue *cq
        , uint8_t *msg, int len, uint64_t min_clock, uint64_t req_clock
        , uint64_t notify_id);
    void serialqueue_pull(struct serialqueue *sq
        , struct pull_queue_message *pqm);
    void serialqueue_set_wire_frequency(struct serialqueue *sq
        , double frequency);
    void serialqueue_set_receive_window(struct serialqueue *sq
        , int receive_window);
    void serialqueue_set_clock_est(struct serialqueue *sq, double est_freq
        , double conv_time, uint64_t conv_clock, uint64_t last_clock);
    void serialqueue_get_stats(struct serialqueue *sq, char *buf, int len);
    int serialqueue_extract_old(struct serialqueue *sq, int sentq
        , struct pull_queue_message *q, int max);
"""

defs_trdispatch = """
    void trdispatch_start(struct trdispatch *td, uint32_t dispatch_reason);
    void trdispatch_stop(struct trdispatch *td);
    struct trdispatch *trdispatch_alloc(void);
    struct trdispatch_mcu *trdispatch_mcu_alloc(struct trdispatch *td
        , struct serialqueue *sq, struct command_queue *cq, uint32_t trsync_oid
        , uint32_t set_timeout_msgtag, uint32_t trigger_msgtag
        , uint32_t state_msgtag);
    void trdispatch_mcu_setup(struct trdispatch_mcu *tdm
        , uint64_t last_status_clock, uint64_t expire_clock
        , uint64_t expire_ticks, uint64_t min_extend_ticks);
"""

defs_pyhelper = """
    void set_python_logging_callback(void (*func)(const char *));
    double get_monotonic(void);
"""

defs_std = """
    void free(void*);
"""

defs_motion_engine = """
    struct MotionEngine *motion_engine_create(void);
    void motion_engine_destroy(struct MotionEngine *engine);
    void motion_engine_reset(struct MotionEngine *engine);
    void motion_engine_set_trapq(struct MotionEngine *engine
        , struct trapq *tq);
    int motion_engine_queue_move(struct MotionEngine *engine
        , double x, double y, double z, double e, double speed);
    int motion_engine_queue_move_ex(struct MotionEngine *engine
        , double start_x, double start_y, double start_z, double start_e
        , double end_x, double end_y, double end_z, double end_e
        , double speed, double accel
        , double max_cruise_v2, double delta_v2
        , double smooth_delta_v2, double next_junction_v2
        , int is_kinematic);
    void motion_engine_flush(struct MotionEngine *engine);
    void motion_engine_flush_step_generation(struct MotionEngine *engine);
    double motion_engine_get_print_time(const struct MotionEngine *engine);
    double motion_engine_get_buffer_time(const struct MotionEngine *engine
        , double est_print_time);
    void motion_engine_set_position(struct MotionEngine *engine
        , double x, double y, double z, double e);
    void motion_engine_set_velocity_limits(struct MotionEngine *engine
        , double max_velocity, double max_accel
        , double square_corner_velocity, double min_cruise_ratio);
    uint32_t motion_engine_get_stall_count(const struct MotionEngine *engine);
    void motion_engine_set_print_time(struct MotionEngine *engine
        , double print_time);
    void motion_engine_sync_state(struct MotionEngine *engine
        , double print_time, double last_flush_time
        , double min_restart_time, double need_flush_time
        , double step_gen_time, double clear_history_time
        , double pos_x, double pos_y, double pos_z, double pos_e);
    void motion_engine_set_kin_flush_delay(struct MotionEngine *engine
        , double delay);
    double motion_engine_get_last_flush_time(const struct MotionEngine *engine);
    uint32_t motion_engine_get_queue_len(const struct MotionEngine *engine);

    void motion_engine_set_extruder_trapq(struct MotionEngine *engine
        , struct trapq *tq);
    void motion_engine_set_extruder_params(struct MotionEngine *engine
        , double pressure_advance, double use_pa_from_trapq
        , double instant_corner_v);
    int motion_engine_add_stepper(struct MotionEngine *engine
        , struct stepper_kinematics *sk);
    int motion_engine_add_mcu(struct MotionEngine *engine
        , struct steppersync *ss, double time_offset, double mcu_freq);
    void motion_engine_update_mcu_clock(struct MotionEngine *engine
        , uint32_t index, double time_offset, double mcu_freq);
    void motion_engine_set_post_flush_cb(struct MotionEngine *engine
        , void (*cb)(void *), void *ctx);

    struct FlushedMoveResult {
        double start_v, cruise_v, end_v;
        double accel_t, cruise_t, decel_t;
        double accel;
    };
    int motion_engine_flush_and_extract(struct MotionEngine *engine
        , struct FlushedMoveResult *results, uint32_t max_results, int lazy);
    int motion_engine_flush_and_process(struct MotionEngine *engine
        , struct FlushedMoveResult *results, uint32_t max_results, int lazy);
    int motion_engine_generate_steps(struct MotionEngine *engine
        , double sg_flush_time);
    void motion_engine_finalize_trapqs(struct MotionEngine *engine
        , double free_time, double clear_history_time);

    struct ClockSync *clock_sync_create(double mcu_freq);
    void clock_sync_destroy(struct ClockSync *cs);
    void clock_sync_set_freq(struct ClockSync *cs, double mcu_freq);
    double clock_sync_update(struct ClockSync *cs
        , uint32_t clock32, double sent_time, double receive_time);
    int64_t clock_sync_get_clock(const struct ClockSync *cs, double eventtime);
    double clock_sync_estimated_print_time(const struct ClockSync *cs
        , double eventtime);
"""

defs_all = [
    defs_pyhelper,
    defs_serialqueue,
    defs_std,
    defs_stepcompress,
    defs_itersolve,
    defs_trapq,
    defs_trdispatch,
    defs_kin_cartesian,
    defs_kin_corexy,
    defs_kin_corexz,
    defs_kin_delta,
    defs_kin_deltesian,
    defs_kin_polar,
    defs_kin_rotary_delta,
    defs_kin_winch,
    defs_kin_extruder,
    defs_kin_shaper,
    defs_kin_idex,
]


# Update filenames to an absolute path
def get_abs_files(srcdir, filelist):
    return [os.path.join(srcdir, fname) for fname in filelist]


# Return the list of file modification times
def get_mtimes(filelist):
    out = []
    for filename in filelist:
        try:
            t = os.path.getmtime(filename)
        except os.error:
            continue
        out.append(t)
    return out


# Check if the code needs to be compiled
def check_build_code(sources, target):
    src_times = get_mtimes(sources)
    obj_times = get_mtimes([target])
    return not obj_times or max(src_times) > min(obj_times)


# Check if the current gcc version supports a particular command-line option
def check_gcc_option(option):
    cmd = "%s %s -S -o /dev/null -xc /dev/null > /dev/null 2>&1" % (
        GCC_CMD,
        option,
    )
    res = os.system(cmd)
    return res == 0


# Check if the current gcc version supports a particular command-line option
def do_build_code(cmd):
    res = os.system(cmd)
    if res:
        msg = "Unable to build C code module (error=%s)" % (res,)
        logging.error(msg)
        raise Exception(msg)


FFI_main = None
FFI_lib = None
pyhelper_logging_callback = None


# Hepler invoked from C errorf() code to log errors
def logging_callback(msg):
    logging.error(FFI_main.string(msg))


# Return the Foreign Function Interface api to the caller
def get_ffi():
    global FFI_main, FFI_lib, pyhelper_logging_callback
    if FFI_lib is None:
        srcdir = os.path.dirname(os.path.realpath(__file__))
        srcfiles = get_abs_files(srcdir, SOURCE_FILES)
        ofiles = get_abs_files(srcdir, OTHER_FILES)
        destlib = get_abs_files(srcdir, [DEST_LIB])[0]
        if check_build_code(srcfiles + ofiles + [__file__], destlib):
            if check_gcc_option(SSE_FLAGS):
                cmd = "%s %s %s" % (GCC_CMD, SSE_FLAGS, COMPILE_ARGS)
            else:
                cmd = "%s %s" % (GCC_CMD, COMPILE_ARGS)
            logging.info("Building C code module %s", DEST_LIB)
            do_build_code(cmd % (destlib, " ".join(srcfiles)))
        FFI_main = cffi.FFI()
        for d in defs_all:
            FFI_main.cdef(d)
        FFI_lib = FFI_main.dlopen(destlib)
        # Setup error logging
        pyhelper_logging_callback = FFI_main.callback(
            "void func(const char *)", logging_callback
        )
        FFI_lib.set_python_logging_callback(pyhelper_logging_callback)
    return FFI_main, FFI_lib


######################################################################
# Native motion engine (Zig)
######################################################################

ZIG_ENGINE_DIR = "zig_engine"
ZIG_ENGINE_LIB = "libmotion_engine.so"

ME_FFI_main = None
ME_FFI_lib = None


def _detect_arch():
    """Detect CPU architecture for prebuilt binary selection."""
    import platform

    machine = platform.machine().lower()
    if machine in ("x86_64", "amd64"):
        return "x86_64"
    elif machine in ("aarch64", "arm64"):
        return "aarch64"
    elif machine.startswith("arm"):
        return "armv7"
    return None


def get_motion_ffi():
    """Load the Zig native motion engine library.
    Tries prebuilt binaries first, then falls back to building from source.
    Returns (ffi_main, ffi_lib) or (None, None) if not available.
    """
    global ME_FFI_main, ME_FFI_lib
    if ME_FFI_lib is not None:
        return ME_FFI_main, ME_FFI_lib
    srcdir = os.path.dirname(os.path.realpath(__file__))
    zigdir = os.path.join(srcdir, ZIG_ENGINE_DIR)
    destlib = None
    # 1. Try prebuilt binary for this architecture
    arch = _detect_arch()
    if arch is not None:
        prebuilt = os.path.join(
            zigdir, "prebuilt", "libmotion_engine-%s.so" % arch
        )
        if os.path.exists(prebuilt):
            destlib = prebuilt
            logging.info(
                "Using prebuilt native motion engine for %s", arch
            )
    # 2. Try zig-out from a previous build
    if destlib is None:
        built = os.path.join(zigdir, "zig-out", "lib", ZIG_ENGINE_LIB)
        if os.path.exists(built):
            destlib = built
    # 3. Try to build from source
    if destlib is None:
        import subprocess

        logging.info("Building Zig motion engine %s", ZIG_ENGINE_LIB)
        try:
            subprocess.check_call(
                ["zig", "build", "-Doptimize=ReleaseFast"],
                cwd=zigdir,
                timeout=120,
            )
        except (subprocess.CalledProcessError, FileNotFoundError) as e:
            logging.warning(
                "Failed to build Zig motion engine: %s. "
                "Falling back to Python motion planning.",
                e,
            )
            return None, None
        built = os.path.join(zigdir, "zig-out", "lib", ZIG_ENGINE_LIB)
        if os.path.exists(built):
            destlib = built
    if destlib is None:
        logging.warning("Zig motion engine library not found")
        return None, None
    # Reuse the same FFI instance as get_ffi() so struct types match
    # (CFFI rejects pointers across different FFI instances)
    ffi_main, _ = get_ffi()
    ffi_main.cdef(defs_motion_engine)
    ME_FFI_main = ffi_main
    ME_FFI_lib = ffi_main.dlopen(destlib)
    logging.info("Loaded Zig native motion engine from %s", destlib)
    return ME_FFI_main, ME_FFI_lib


######################################################################
# hub-ctrl hub power controller
######################################################################

HC_COMPILE_CMD = "gcc -Wall -g -O2 -o %s %s -lusb"
HC_SOURCE_FILES = ["hub-ctrl.c"]
HC_SOURCE_DIR = "../../lib/hub-ctrl"
HC_TARGET = "hub-ctrl"
HC_CMD = "sudo %s/hub-ctrl -h 0 -P 2 -p %d"


def run_hub_ctrl(enable_power):
    srcdir = os.path.dirname(os.path.realpath(__file__))
    hubdir = os.path.join(srcdir, HC_SOURCE_DIR)
    srcfiles = get_abs_files(hubdir, HC_SOURCE_FILES)
    destlib = get_abs_files(hubdir, [HC_TARGET])[0]
    if check_build_code(srcfiles, destlib):
        logging.info("Building C code module %s", HC_TARGET)
        do_build_code(HC_COMPILE_CMD % (destlib, " ".join(srcfiles)))
    os.system(HC_CMD % (hubdir, enable_power))


if __name__ == "__main__":
    get_ffi()
