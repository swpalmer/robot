const std = @import("std");
const os = std.os;
const c = @cImport({
    @cInclude("unistd.h");
});

var piper: std.process.Child = undefined;
var aplay: std.process.Child = undefined;

pub fn init() !void {
    const allocator = std.heap.page_allocator;

    var pipe_fds: [2]i32 = undefined;
    if (c.pipe(&pipe_fds) != 0) return error.PipeFailed;

    const pipe_read = pipe_fds[0];
    const pipe_write = pipe_fds[1];

    // Start Piper
    piper = std.process.Child.init(&.{
        "./piper/piper",
        "--model",
        "piper/en_GB-cori-medium.onnx",
        "--output_raw",
    }, allocator);
    piper.stdin_behavior = .Pipe;
    piper.stdout_behavior = .Pipe;
    piper.stderr_behavior = .Inherit; // ✅ Print Piper's logs to the Zig process console
    piper.stdout = std.fs.File{ .handle = pipe_write };

    // Start Aplay
    aplay = std.process.Child.init(&.{ "aplay", "-r", "22050", "-f", "S16_LE", "-t", "raw", "-" }, allocator);
    aplay.stdin_behavior = .Pipe;
    aplay.stdout_behavior = .Inherit; // ✅ Print aplay's logs to the Zig process console
    aplay.stderr_behavior = .Inherit; // ✅ Print aplay's logs to the Zig process console
    aplay.stdin = std.fs.File{ .handle = pipe_read };

    try aplay.spawn();
    try piper.spawn();

    // Close our side of the pipe to avoid hanging
    // _ = c.close(pipe_read);
    // _ = c.close(pipe_write);

    const piper_in = piper.stdin.?;
    try piper_in.writeAll("Hello!\n");
}

pub fn deinit() !void {
    // Nothing to do here, as the processes will be cleaned up by the OS.
    // If you need to terminate them, you can use `piper.kill()` and `aplay.kill()`.
    const piper_in = piper.stdin.?;
    _ = c.close(piper_in.handle);
    _ = try piper.wait();
    _ = try aplay.wait();
}

pub fn speak(text: [:0]const u8) !void {
    // This function should send the text to Piper for speech synthesis.
    // You can use the `piper.stdin` to write the text.
    // Example:
    const piper_in = piper.stdin.?;
    try piper_in.writeAll(text);
    try piper_in.writeAll("\n");
}
