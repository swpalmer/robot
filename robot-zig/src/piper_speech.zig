const std = @import("std");
const os = std.os;
const c = @cImport({
    @cInclude("unistd.h");
});

pub const ProcPipe = struct {
    allocator: std.mem.Allocator,
    proc1_stdin_file: *std.fs.File,
    proc1_stdin_writer: std.io.AnyWriter,
    parent_stdin_write_fd: std.posix.fd_t, // Store file descriptor for closing
    child_pid: std.posix.pid_t, // Store PID for waiting

    pub fn init(allocator: std.mem.Allocator, cmdline_1: std.ArrayList([]const u8), cmdline_2: std.ArrayList([]const u8)) !@This() {
        // This is the pipe form the parent process to the first child process
        const word_pipe = try std.posix.pipe();

        // This is pipe form child 1 to child 2
        const intermediate_pipe = try std.posix.pipe();

        // Call runPipe with the dynamically built commands
        const pid = try std.posix.fork();
        switch (pid) {
            // Child for second command (will fork again to get proc for first command)
            0 => {
                try runPipe(allocator, word_pipe, intermediate_pipe, cmdline_1, cmdline_2);
                unreachable;
            },
            else => {
                // This is the original Parent process continuing to run
                //std.time.sleep(std.time.ns_per_s * 5); // give child process time to start
                // Parent does not need the read end of the pipe to proc1 stdin
                std.posix.close(word_pipe[0]);
                // Parent does not need either end of the proc1->proc2 pipe
                std.posix.close(intermediate_pipe[0]);
                std.posix.close(intermediate_pipe[1]);
                std.debug.print("Parent process closed intermediate pipe it isn't using\n", .{});
                std.debug.print("Child process is {d}\n", .{pid});

                // Create Zig I/O writer from the file descriptor
                // that the parent will use for direct communication with the pipeline.
                //
                const proc1_pipe_file = try allocator.create(std.fs.File);
                proc1_pipe_file.* = std.fs.File{ .handle = word_pipe[1] };
                //const proc1_stdin_writer = proc1_stdin_file.writer().any();

                // These lines work and the output can be seen in the conole in ALL CAPS
                // try proc1_stdin_writer.writeAll("Hello?\nAre you there?\n"); // <-- fails
                // try proc1_stdin_writer.writeAll("Can you hear me?\n"); // <-- fails
                // try proc1_stdin_writer.writeAll("We haven't got much time.\n"); // <-- fails

                var voiceBox = @This(){
                    .allocator = allocator,
                    .proc1_stdin_file = proc1_pipe_file,
                    .proc1_stdin_writer = proc1_pipe_file.writer().any(),
                    .parent_stdin_write_fd = word_pipe[1],
                    .child_pid = pid,
                };
                // These lines work and the output can be seen in the conole in ALL CAPS
                try voiceBox.proc1_stdin_writer.writeAll("Hello?\nAre you there?\n"); // <-- fails
                try voiceBox.sendData("Can you hear me?\n"); // <-- fails
                //try proc1_stdin_writer.writeAll("We haven't got much time.\n"); // <-- fails
                return voiceBox;
            },
        }
    }

    // This runs in the first child process
    // (Which is the process that will run the SECOND command)
    fn runPipe(allocator: std.mem.Allocator, word_fd: [2]i32, pfd: [2]i32, first_command: std.ArrayList([]const u8), second_command: std.ArrayList([]const u8)) !void {
        // const allocator = std.heap.page_allocator;
        const pid = try std.posix.fork(); // fork again to get a second child process

        switch (pid) {
            0 => {
                // Child process for the first command
                try std.posix.dup2(word_fd[0], std.posix.STDIN_FILENO); // Redirect stdin from the pipe
                std.posix.close(word_fd[0]); // Close after replacing stdin
                std.posix.close(word_fd[1]); // We dont need the write end of the pipe
                // Child process for the first command
                try std.posix.dup2(pfd[1], std.posix.STDOUT_FILENO); // Redirect stdout to the pipe
                std.posix.close(pfd[0]); // Close unused read end
                std.posix.close(pfd[1]); // Close after replacing stdout
                // Execute the first command passed from main
                std.process.execve(allocator, first_command.items, null) catch {
                    std.debug.print("Failed to execute first command: {s}\n", .{first_command.items[0]});
                };
            },
            else => {
                // Child process for the second command
                try std.posix.dup2(pfd[0], std.posix.STDIN_FILENO); // Redirect stdin from the pipe
                std.posix.close(pfd[0]); // Close after replacing stdin
                std.posix.close(pfd[1]); // Close unused write end
                // Execute the second command passed from main
                std.process.execve(allocator, second_command.items, null) catch {
                    std.debug.print("Failed to execute second command: {s}\n", .{second_command.items[0]});
                };
                const ret = std.posix.waitpid(-1, 0);
                if (ret.pid != -1) {
                    std.debug.print("Process {d} exited with {d}\n", .{ ret.pid, ret.status });
                }
            },
        }
    }

    /// Sends data to the stdin of the first process in the pipeline.
    /// This method allows the parent to provide input to the chained processes.
    ///
    /// Parameters:
    ///   data: The byte slice to write to the process's stdin.
    ///
    /// Returns:
    ///   An error if writing fails.
    pub fn sendData(self: *@This(), data: []const u8) !void {
        try self.proc1_stdin_writer.writeAll(data);
        // Note: AnyWriter doesn't have flush(), data is sent immediately
    }

    pub fn terminate(self: *@This()) !void {
        std.debug.print("Terminating process pipeline...\n", .{});
        std.posix.close(self.parent_stdin_write_fd);
        _ = std.posix.waitpid(self.child_pid, 0);
    }
};

var voicePipe: *ProcPipe = undefined;
pub fn init(allocator: std.mem.Allocator) !void {
    std.debug.print("Initializing process pipeline...\n", .{});

    // Create command arrays for the commands you want to run
    var first_command = std.ArrayList([]const u8).init(allocator);
    defer first_command.deinit();
    var second_command = std.ArrayList([]const u8).init(allocator);
    defer second_command.deinit();

    // test processes - cat to pass input on verbatim
    try first_command.append("cat");

    // try first_command.append("ls");
    // try first_command.append("-al");
    // try first_command.append("/");

    try second_command.append("tr");
    try second_command.append("a-z");
    try second_command.append("A-Z");

    // Real sub processes for piper text to speech
    // try first_command.append("/home/pi/piper/piper");
    // try first_command.append("-m");
    // try first_command.append("/home/pi/voices/en_GB-cori-high.onnx");
    // try first_command.append("--output_raw");

    // try second_command.append("aplay");
    // try second_command.append("-t");
    // try second_command.append("raw");
    // try second_command.append("-f");
    // try second_command.append("S16_LE");
    // try second_command.append("-r");
    // try second_command.append("22050");

    voicePipe = try allocator.create(ProcPipe);
    voicePipe.* = try ProcPipe.init(allocator, first_command, second_command);
    std.debug.print("TTS Pipeline created successfully!\n", .{});

    std.time.sleep(std.time.ns_per_s * 5);

    // Test sending multiple strings to see if pipeline works despite panic
    try voicePipe.sendData("First test message\n");
    try voicePipe.sendData("Second test message\n");
    try voicePipe.sendData("Third test message\n");

    std.debug.print("Multiple test messages sent to pipeline!\n", .{});

    // Processes run asynchronously - no need to wait
}

pub fn close() !void {
    // Ensure the pipeline is terminated and all associated resources (processes, pipes)
    // are cleaned up when the `main` function exits, even if an error occurs.
    try voicePipe.terminate();
    //try voicePipe.deinit();
}

pub fn speak(text: [:0]const u8) !void {
    // This function should send the text to Piper for speech synthesis.
    // You can use the `piper.stdin` to write the text.
    // Example:
    try voicePipe.sendData(text);
    // if it wasn't newline terminated shall we send a newline?
    if (!std.mem.endsWith(u8, text, "\n")) {
        try voicePipe.sendData("\n");
    }
}
