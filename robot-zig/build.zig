const std = @import("std");
const builtin = @import("builtin");

// Although this function looks imperative, note that its job is to
// declaratively construct a build graph that will be executed by an external
// runner.
pub fn build(b: *std.Build) void {
    // Standard target options allows the person running `zig build` to choose
    // what target to build for. Here we do not override the defaults, which
    // means any target is allowed, and the default is native. Other options
    // for restricting supported target set are available.

    //const native_target = b.standardTargetOptions(.{});
    const cross_compile = true; //native_target.result.isDarwinLibC();

    const target = if (cross_compile)
        // cross compile
        b.standardTargetOptions(.{
            .default_target = .{ .cpu_arch = .aarch64, .os_tag = .linux, .abi = .gnu },
        })
    else
        b.standardTargetOptions(.{});

    // Standard optimization options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall. Here we do not
    // set a preferred release mode, allowing the user to decide how to optimize.
    const optimize = b.standardOptimizeOption(.{});

    // This creates a "module", which represents a collection of source files alongside
    // some compilation options, such as optimization mode and linked system libraries.
    // Every executable or library we compile will be based on one or more modules.
    const lib_mod = b.createModule(.{
        // `root_source_file` is the Zig "entry point" of the module. If a module
        // only contains e.g. external object files, you can make this `null`.
        // In this case the main source file is merely a path, however, in more
        // complicated build scripts, this could be a generated file.
        .root_source_file = b.path("src/robot.zig"),
        .target = target,
        .optimize = optimize,
    });

    const web_mod = b.createModule(.{
        .root_source_file = b.path("src/web.zig"),
        .target = target,
        .optimize = optimize,
    });

    const speech_mod = b.createModule(.{
        // `root_source_file` is the Zig "entry point" of the module. If a module
        // only contains e.g. external object files, you can make this `null`.
        // In this case the main source file is merely a path, however, in more
        // complicated build scripts, this could be a generated file.
        .root_source_file = b.path("src/piper_speech.zig"),
        .target = target,
        .optimize = optimize,
    });
    //speech_mod.addSystemIncludePath(b.path("orca/include"));
    //speech_mod.addLibraryPath(b.path("orca/lib/raspberry-pi/cortex-a53"));
    speech_mod.addLibraryPath(b.path("orca/lib/raspberry-pi/cortex-a53-aarch64"));
    // For speech synthesis, we will link against the Orca library.
    //speech_mod.linkSystemLibrary("pv_orca", .{ .needed = true });
    //speech_mod.linkSystemLibrary("asound", .{ .needed = true });
    //speech_mod.linkSystemLibrary("pulse-simple", .{ .needed = true });

    // We will also create a module for our other entry point, 'main.zig'.
    const exe_mod = b.createModule(.{
        // `root_source_file` is the Zig "entry point" of the module. If a module
        // only contains e.g. external object files, you can make this `null`.
        // In this case the main source file is merely a path, however, in more
        // complicated build scripts, this could be a generated file.
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });

    const types_mod = b.createModule(.{
        // `root_source_file` is the Zig "entry point" of the module. If a module
        // only contains e.g. external object files, you can make this `null`.
        // In this case the main source file is merely a path, however, in more
        // complicated build scripts, this could be a generated file.
        .root_source_file = b.path("src/types.zig"),
        .target = target,
        .optimize = optimize,
    });

    // Conditional Compilation Flags
    if (cross_compile) {
        std.debug.print("Cross-compiling for Raspberry Pi\n", .{});
        // Tweak the include path for cross-compiling for RPi (the default)
        lib_mod.addSystemIncludePath(.{ .cwd_relative = "/Users/scott/dev/Personal/Robot/rpi-sysroot/usr/include" });
        //lib_mod.addSystemIncludePath(.{ .cwd_relative = "/Users/scott/dev/Personal/Robot/rpi-sysroot/usr/include/arm-linux-gnueabihf" });
        lib_mod.addSystemIncludePath(.{ .cwd_relative = "/Users/scott/dev/Personal/Robot/rpi-sysroot/usr/include/aarch64-linux-gnu" });
        lib_mod.addLibraryPath(.{ .cwd_relative = "/Users/scott/dev/Personal/Robot/rpi-sysroot/usr/lib" });

        web_mod.addSystemIncludePath(.{ .cwd_relative = "/Users/scott/dev/Personal/Robot/rpi-sysroot/usr/include" });
        //web_mod.addSystemIncludePath(.{ .cwd_relative = "/Users/scott/dev/Personal/Robot/rpi-sysroot/usr/include/arm-linux-gnueabihf" });
        web_mod.addSystemIncludePath(.{ .cwd_relative = "/Users/scott/dev/Personal/Robot/rpi-sysroot/usr/include/aarch64-linux-gnu" });
        web_mod.addLibraryPath(.{ .cwd_relative = "/Users/scott/dev/Personal/Robot/rpi-sysroot/usr/lib" });

        speech_mod.addSystemIncludePath(.{ .cwd_relative = "/Users/scott/dev/Personal/Robot/rpi-sysroot/usr/include" });
        speech_mod.addLibraryPath(.{ .cwd_relative = "/Users/scott/dev/Personal/Robot/rpi-sysroot/usr/lib" });
        //speech_mod.addLibraryPath(.{ .cwd_relative = "/Users/scott/dev/Personal/Robot/rpi-sysroot/usr/lib/arm-linux-gnueabihf" });
        speech_mod.addLibraryPath(.{ .cwd_relative = "/Users/scott/dev/Personal/Robot/rpi-sysroot/usr/lib/aarch64-linux-gnu" });

        exe_mod.addSystemIncludePath(.{ .cwd_relative = "/Users/scott/dev/Personal/Robot/rpi-sysroot/usr/include" });
        // exe_mod.addLibraryPath(b.path("orca/lib/raspberry-pi/cortex-a53"));
        //exe_mod.addLibraryPath(.{ .cwd_relative = "/Users/scott/dev/Personal/Robot/rpi-sysroot/usr/lib/arm-linux-gnueabihf" });
        exe_mod.addLibraryPath(.{ .cwd_relative = "/Users/scott/dev/Personal/Robot/rpi-sysroot/usr/lib/aarch64-linux-gnu" });
        // exe_mod.linkSystemLibrary("rt", .{ .needed = true });
        // exe_mod.linkSystemLibrary("pv_orca", .{ .needed = true });
        // exe_mod.linkSystemLibrary("asound", .{ .needed = true });
    } else {
        std.debug.print("Compiling for native platform.\n", .{});
        // TODO: stubs for pigpio
        // exe_mod.addLibraryPath(b.path("orca/lib/macos/x86_64"));
    }

    // Modules can depend on one another using the `std.Build.Module.addImport` function.
    // This is what allows Zig source code to use `@import("foo")` where 'foo' is not a
    // file path. In this case, we set up `exe_mod` to import `lib_mod`.
    exe_mod.addImport("robot_lib", lib_mod);
    exe_mod.addImport("robot_web", web_mod);
    exe_mod.addImport("speech", speech_mod);
    exe_mod.addImport("types", types_mod);
    lib_mod.addImport("types", types_mod);
    web_mod.addImport("types", types_mod);
    web_mod.addImport("speech", speech_mod);

    // Now, we will create a static library based on the module we created above.
    // This creates a `std.Build.Step.Compile`, which is the build step responsible
    // for actually invoking the compiler.
    const lib = b.addLibrary(.{
        .linkage = .static,
        .name = "robot_lib",
        .root_module = lib_mod,
    });

    // This declares intent for the library to be installed into the standard
    // location when the user invokes the "install" step (the default step when
    // running `zig build`).
    b.installArtifact(lib);

    const web = b.addLibrary(.{
        .linkage = .static,
        .name = "robot_web",
        .root_module = web_mod,
    });
    b.installArtifact(web);

    const speech = b.addLibrary(.{
        .linkage = .static,
        .name = "speech",
        .root_module = speech_mod,
    });
    b.installArtifact(speech);

    // This creates another `std.Build.Step.Compile`, but this one builds an executable
    // rather than a static library.
    const exe = b.addExecutable(.{
        .name = "robot",
        .root_module = exe_mod,
    });

    ////  SWP -- Using http.zig for a web control panel   ////
    ////  Possibly even web VR stuff for Occulus control  ////
    const httpz = b.dependency("httpz", .{
        .target = target,
        .optimize = optimize,
    });
    exe.root_module.addImport("httpz", httpz.module("httpz"));
    web.root_module.addImport("httpz", httpz.module("httpz"));

    lib.linkSystemLibrary("pigpio");
    lib.linkLibC();
    exe.linkLibC();

    // This declares intent for the executable to be installed into the
    // standard location when the user invokes the "install" step (the default
    // step when running `zig build`).
    b.installArtifact(exe);

    // This *creates* a Run step in the build graph, to be executed when another
    // step is evaluated that depends on it. The next line below will establish
    // such a dependency.
    const run_cmd = b.addRunArtifact(exe);

    // By making the run step depend on the install step, it will be run from the
    // installation directory rather than directly from within the cache directory.
    // This is not necessary, however, if the application depends on other installed
    // files, this ensures they will be present and in the expected location.
    run_cmd.step.dependOn(b.getInstallStep());

    // This allows the user to pass arguments to the application in the build
    // command itself, like this: `zig build run -- arg1 arg2 etc`
    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    // This creates a build step. It will be visible in the `zig build --help` menu,
    // and can be selected like this: `zig build run`
    // This will evaluate the `run` step rather than the default, which is "install".
    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);

    // Creates a step for unit testing. This only builds the test executable
    // but does not run it.
    const lib_unit_tests = b.addTest(.{
        .root_module = lib_mod,
    });

    const run_lib_unit_tests = b.addRunArtifact(lib_unit_tests);

    const exe_unit_tests = b.addTest(.{
        .root_module = exe_mod,
    });

    const run_exe_unit_tests = b.addRunArtifact(exe_unit_tests);

    // Similar to creating the run step earlier, this exposes a `test` step to
    // the `zig build --help` menu, providing a way for the user to request
    // running the unit tests.
    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_lib_unit_tests.step);
    test_step.dependOn(&run_exe_unit_tests.step);

    // Clean step to remove build artifacts
    const clean_step = b.step("clean", "Clean build artifacts");
    const clean_cmd = b.addSystemCommand(&.{ "rm", "-rf", ".zig-cache", "zig-out" });
    clean_step.dependOn(&clean_cmd.step);
}
