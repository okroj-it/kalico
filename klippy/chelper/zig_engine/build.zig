const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const chelper_path = b.path("..");

    const c_sources: []const []const u8 = &.{
        "trapq.c",
        "itersolve.c",
        "stepcompress.c",
        "serialqueue.c",
        "pollreactor.c",
        "msgblock.c",
        "pyhelper.c",
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
    };

    const c_flags: []const []const u8 = &.{ "-Wall", "-O2" };

    // Shared library: libmotion_engine.so
    const lib_module = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });
    lib_module.addIncludePath(chelper_path);
    lib_module.linkSystemLibrary("c", .{});
    lib_module.addCSourceFiles(.{
        .root = chelper_path,
        .files = c_sources,
        .flags = c_flags,
        .language = null,
    });

    const lib = b.addLibrary(.{
        .linkage = .dynamic,
        .name = "motion_engine",
        .root_module = lib_module,
    });

    b.installArtifact(lib);

    // Tests
    const test_module = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });
    test_module.addIncludePath(chelper_path);
    test_module.linkSystemLibrary("c", .{});
    test_module.addCSourceFiles(.{
        .root = chelper_path,
        .files = c_sources,
        .flags = c_flags,
        .language = null,
    });

    const tests = b.addTest(.{
        .root_module = test_module,
    });

    const run_tests = b.addRunArtifact(tests);
    const test_step = b.step("test", "Run motion engine tests");
    test_step.dependOn(&run_tests.step);
}
