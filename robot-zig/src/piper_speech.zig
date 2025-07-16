const std = @import("std");
const os = std.os;
const c = @cImport({
    @cInclude("unistd.h");
});

var piper: ?std.process.Child = null;
var aplay: ?std.process.Child = null;

pub fn init() !void {
    const allocator = std.heap.page_allocator;

    //var piper_pipe_fds: [2]i32 = undefined;
    //if (c.pipe(&pipe_fds) != 0) return error.PipeFailed;

    var pipe_fds: [2]i32 = undefined;
    if (c.pipe(&pipe_fds) != 0) return error.PipeFailed;

    const pipe_read = pipe_fds[0];
    const pipe_write = pipe_fds[1];

    // Start Piper
    var piper_proc = std.process.Child.init(&.{
        //     "./piper/piper",
        "cat", // For testing, replace with Piper command when ready
        //     "--model",
        //     "piper/en_GB-cori-medium.onnx",
        //     "--output_raw",
    }, allocator);

    piper_proc.stdin_behavior = .Pipe;
    piper_proc.stdout_behavior = .Pipe;
    piper_proc.stderr_behavior = .Inherit; // ✅ Print Piper's logs to the Zig process console
    //piper.stdin = std.fs.File{ .handle = piper_pipe_fds[1] }; // Write to Piper's stdin
    piper_proc.stdout = std.fs.File{ .handle = pipe_write };

    // Start Aplay
    var aplay_proc = std.process.Child.init(&.{
        // "aplay",
        "cat", // For testing, replace with Aplay command when ready
        //"-r", "22050", "-f", "S16_LE", "-t", "raw", "-"
    }, allocator);

    aplay_proc.stdin_behavior = .Pipe;
    aplay_proc.stdout_behavior = .Inherit; // ✅ Print aplay's logs to the Zig process console
    aplay_proc.stderr_behavior = .Inherit; // ✅ Print aplay's logs to the Zig process console
    aplay_proc.stdin = std.fs.File{ .handle = pipe_read };

    try aplay_proc.spawn();
    aplay = aplay_proc;
    try piper_proc.spawn();
    piper = piper_proc;

    // Close our side of the pipe to avoid hanging
    // _ = c.close(pipe_read);
    // _ = c.close(pipe_write);

    const piper_in = piper_proc.stdin.?;
    try piper_in.writeAll("Hello!\n");
    std.debug.print("Hello\n", .{});
}

pub fn close() !void {
    defer {
        std.debug.print("Exiting TTS close().\n", .{});
    }
    std.debug.print("Closing Piper and Aplay processes...\n", .{});
    // Nothing to do here, as the processes will be cleaned up by the OS.
    // If you need to terminate them, you can use `piper.kill()` and `aplay.kill()`.
    if (piper) |piper_proc| {
        if (piper_proc.stdin) |piper_in| {
            std.debug.print("Closing Piper stdin...\n", .{});
            piper_in.close();
            //_ = c.close(piper_in.handle);
            std.debug.print("Waiting for piper process to exit...\n", .{});
            _ = try piper_proc.wait();
        }
    }
    if (aplay) |aplay_proc| {
        std.debug.print("Waiting for aplay process to exit...\n", .{});
        _ = try aplay_proc.wait();
    }
}

pub fn speak(text: [:0]const u8) !void {
    // This function should send the text to Piper for speech synthesis.
    // You can use the `piper.stdin` to write the text.
    // Example:
    if (piper) |piper_proc| {
        if (piper_proc.stdin) |piper_in| {
            try piper_in.writeAll(text);
            try piper_in.writeAll("\n");
        }
    }
}

/////////// Gemini Code
///
const ChildProcess = std.process.Child;
const io = std.io;
const posix = std.posix; // For fork, execve, dup2, pipe, close

/// Represents a pipeline of two child processes,
/// where the stdout of the first is piped to the stdin of the second.
/// The parent can write to the first process's stdin, and the second process's
/// stdout will be inherited from the parent.
pub const ProcessPipeline = struct {
    allocator: std.mem.Allocator,
    proc1_stdin_writer: io.AnyWriter,
    parent_stdin_write_fd: posix.fd_t, // Store file descriptor for closing
    proc1_pid: posix.pid_t, // Store PID for waiting
    proc2_pid: posix.pid_t, // Store PID for waiting

    /// Helper function to convert a Zig slice of string slices
    /// into a C-style null-terminated array of null-terminated C string pointers.
    /// This is necessary for functions like `execvpeZ`.
    /// The allocated memory is managed by the provided allocator.
    fn convertToCArgv(a: std.mem.Allocator, zig_argv_slice: []const []const u8) ![]?*const u8 {
        var c_argv_list = std.ArrayList(?*const u8).init(a);
        // No defer c_argv_list.deinit() here, as toOwnedSlice transfers ownership.
        // The caller of convertToCArgv is responsible for deiniting the returned slice.

        for (zig_argv_slice) |arg| {
            // Allocate space for the string plus a null terminator
            const c_str_buf = try a.alloc(u8, arg.len + 1);
            // Copy the Zig string into the new buffer using copyForward
            std.mem.copyForwards(u8, c_str_buf[0..arg.len], arg);
            // Add the null terminator
            c_str_buf[arg.len] = 0;
            // Append the pointer to the null-terminated C string, casting to *const u8
            // The ArrayList's type (?*const u8) will handle the optional conversion implicitly.
            try c_argv_list.append(@ptrCast(c_str_buf.ptr)); // Simplified cast
        }
        try c_argv_list.append(null); // Null-terminate the array of pointers

        // Return an owned slice, which the caller is responsible for freeing
        return c_argv_list.toOwnedSlice();
    }

    /// Initializes and launches the two child processes,
    /// setting up the necessary pipes for communication using POSIX APIs.
    ///
    /// Parameters:
    ///   allocator: The allocator to use for memory management.
    ///   proc1_argv: Command line arguments for the first process (e.g., &[_][]const u8{"cat"}).
    ///   proc2_argv: Command line arguments for the second process (e.g., &[_][]const u8{"tr", "a-z", "A-Z"}).
    ///
    /// Returns:
    ///   An initialized ProcessPipeline instance on success.
    ///   An error if process spawning or pipe creation fails.
    pub fn init(
        allocator: std.mem.Allocator,
        proc1_argv: []const []const u8,
        proc2_argv: []const []const u8,
    ) !@This() {
        // File descriptors for standard I/O (0: stdin, 1: stdout)
        const STDIN_FILENO: posix.fd_t = 0;
        const STDOUT_FILENO: posix.fd_t = 1;
        // STDERR_FILENO is not explicitly needed as stderr is inherited.

        // Use an ArenaAllocator for temporary C string conversions.
        // This simplifies cleanup as all allocations from this arena are freed at once.
        var arena = std.heap.ArenaAllocator.init(allocator);
        // Don't defer arena.deinit() here - we'll do it after child processes finish
        const arena_allocator = arena.allocator();

        // Convert Zig argv slices to C-style null-terminated argv arrays
        const c_proc1_argv_ptr = try convertToCArgv(arena_allocator, proc1_argv);
        const c_proc2_argv_ptr = try convertToCArgv(arena_allocator, proc2_argv);

        // 1. Pipe for parent -> proc1_stdin
        // parent_to_proc1_pipe_fds[0] is the read end (for proc1's stdin)
        // parent_to_proc1_pipe_fds[1] is the write end (for parent to write to)
        const parent_to_proc1_pipe_fds = try posix.pipe();
        const proc1_stdin_read_fd = parent_to_proc1_pipe_fds[0];
        const parent_stdin_write_fd = parent_to_proc1_pipe_fds[1];

        // 2. Pipe for proc1_stdout -> proc2_stdin (the chaining pipe)
        // proc1_to_proc2_pipe_fds[0] is the read end (for proc2's stdin)
        // proc1_to_proc2_pipe_fds[1] is the write end (for proc1's stdout)
        const proc1_to_proc2_pipe_fds = try posix.pipe();
        const proc2_stdin_read_fd = proc1_to_proc2_pipe_fds[0];
        const proc1_stdout_write_fd = proc1_to_proc2_pipe_fds[1];

        // --- Spawn Process 1 ---
        std.debug.print("About to fork process 1...\n", .{});
        const proc1_pid = try posix.fork();
        if (proc1_pid == 0) {
            // Child process 1
            std.debug.print("Child process 1: Setting up file descriptors...\n", .{});
            // Redirect stdin to the read end of parent_to_proc1_pipe
            try posix.dup2(proc1_stdin_read_fd, STDIN_FILENO);
            // Redirect stdout to the write end of proc1_to_proc2_pipe
            try posix.dup2(proc1_stdout_write_fd, STDOUT_FILENO);
            // Inherit stderr (no redirection needed for stderr)

            std.debug.print("Child process 1: Closing pipe file descriptors...\n", .{});
            // Close all pipe ends in the child that are not its standard I/O
            std.debug.print("Child process 1: DEBUG LINE 1\n", .{});
            posix.close(proc1_stdin_read_fd);
            std.debug.print("Child process 1: DEBUG LINE 2\n", .{});
            posix.close(parent_stdin_write_fd); // Parent's write end
            std.debug.print("Child process 1: DEBUG LINE 3\n", .{});
            posix.close(proc2_stdin_read_fd); // Read end of the middle pipe
            std.debug.print("Child process 1: DEBUG LINE 4\n", .{});
            posix.close(proc1_stdout_write_fd);
            std.debug.print("Child process 1: DEBUG LINE 5\n", .{});
            // Close the other end of the middle pipe (write end)
            posix.close(proc1_to_proc2_pipe_fds[0]);
            std.debug.print("Child process 1: DEBUG LINE 6\n", .{});
            // Don't close proc1_to_proc2_pipe_fds[1] again - it's already closed above as proc1_stdout_write_fd
            std.debug.print("Child process 1: DEBUG LINE 7\n", .{});

            // Execute the first program using posix.execvpeZ
            // c_proc1_argv_ptr[0].? is a pointer to the first argument (executable path)
            // c_proc1_argv_ptr is the null-terminated array of argument pointers
            std.debug.print("Child process 1: DEBUG LINE 8 - About to start execution\n", .{});
            std.debug.print("Child process 1: About to execute command: {s}\n", .{c_proc1_argv_ptr[0].?});

            std.debug.print("Child process 1: DEBUG LINE 9 - Checking args\n", .{});
            // Debug: Check if strings are valid
            var i: usize = 0;
            while (c_proc1_argv_ptr[i]) |arg| : (i += 1) {
                std.debug.print("Child process 1: arg[{}] = {s}\n", .{ i, arg });
            }
            std.debug.print("Child process 1: Total args: {}\n", .{i});

            std.debug.print("Child process 1: DEBUG LINE 10 - About to call execvpeZ\n", .{});
            const err = posix.execvpeZ(@ptrCast(c_proc1_argv_ptr[0].?), @ptrCast(c_proc1_argv_ptr), @ptrCast(std.os.environ.ptr));
            // execvpeZ only returns if an error occurs
            std.debug.print("Error executing proc1 (execvpeZ failed): {s}\n", .{@errorName(err)});
            posix.exit(1); // Exit child process on error
        }
        std.debug.print("Process 1 forked with PID: {}\n", .{proc1_pid});

        // Parent process after forking proc1
        // Close the child's read end of its stdin pipe in the parent
        posix.close(proc1_stdin_read_fd);
        // Close the child's write end of its stdout pipe in the parent
        posix.close(proc1_stdout_write_fd);

        // --- Spawn Process 2 ---
        std.debug.print("About to fork process 2...\n", .{});
        const proc2_pid = try posix.fork();
        if (proc2_pid == 0) {
            // Child process 2
            std.debug.print("Child process 2: Setting up file descriptors...\n", .{});
            // Redirect stdin to the read end of proc1_to_proc2_pipe
            try posix.dup2(proc2_stdin_read_fd, STDIN_FILENO);
            // Redirect stdout to the parent's stdout (inherit it)
            // Inherit stderr (no redirection needed for stderr)

            std.debug.print("Child process 2: Closing pipe file descriptors...\n", .{});
            // Close all pipe ends in the child that are not its standard I/O
            std.debug.print("Child process 2: DEBUG LINE 1\n", .{});
            posix.close(parent_stdin_write_fd); // Parent's write end
            std.debug.print("Child process 2: DEBUG LINE 2\n", .{});
            posix.close(proc2_stdin_read_fd); // Close after dup2 (this is proc1_to_proc2_pipe_fds[0])
            std.debug.print("Child process 2: DEBUG LINE 3\n", .{});
            // Don't close proc1_to_proc2_pipe_fds[0] again - it's already closed above
            std.debug.print("Child process 2: DEBUG LINE 4\n", .{});
            posix.close(proc1_to_proc2_pipe_fds[1]); // Write end of the middle pipe
            std.debug.print("Child process 2: DEBUG LINE 5\n", .{});

            // Execute the second program using posix.execvpeZ
            std.debug.print("Child process 2: DEBUG LINE 6 - About to start execution\n", .{});
            std.debug.print("Child process 2: About to execute command: {s}\n", .{c_proc2_argv_ptr[0].?});

            std.debug.print("Child process 2: DEBUG LINE 7 - Checking args\n", .{});
            // Debug: Check if strings are valid
            var i: usize = 0;
            while (c_proc2_argv_ptr[i]) |arg| : (i += 1) {
                std.debug.print("Child process 2: arg[{}] = {s}\n", .{ i, arg });
            }
            std.debug.print("Child process 2: Total args: {}\n", .{i});

            std.debug.print("Child process 2: DEBUG LINE 8 - About to call execvpeZ\n", .{});
            const err = posix.execvpeZ(@ptrCast(c_proc2_argv_ptr[0].?), @ptrCast(c_proc2_argv_ptr), @ptrCast(std.os.environ.ptr));
            // execvpeZ only returns if an error occurs
            std.debug.print("Error executing proc2 (execvpeZ failed): {s}\n", .{@errorName(err)});
            posix.exit(1); // Exit child process on error
        }
        std.debug.print("Process 2 forked with PID: {}\n", .{proc2_pid});

        // Parent process after forking proc2
        // Close the child's read end of its stdin pipe in the parent
        posix.close(proc2_stdin_read_fd);
        // Close the unused write end of the middle pipe in the parent
        posix.close(proc1_to_proc2_pipe_fds[1]);
        // Close the unused read end of the middle pipe in the parent
        posix.close(proc1_to_proc2_pipe_fds[0]);

        // Create Zig I/O writer from the file descriptor
        // that the parent will use for direct communication with the pipeline.
        const proc1_stdin_file = std.fs.File{ .handle = parent_stdin_write_fd };
        const proc1_stdin_writer = proc1_stdin_file.writer().any();

        // Don't wait for child processes here - let them run asynchronously
        // The arena allocator will be cleaned up when the function returns
        // but the child processes have their own memory space
        defer arena.deinit();

        return @This(){
            .allocator = allocator,
            .proc1_stdin_writer = proc1_stdin_writer,
            .parent_stdin_write_fd = parent_stdin_write_fd,
            .proc1_pid = proc1_pid,
            .proc2_pid = proc2_pid,
        };
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
        try self.proc1_stdin_writer.flush(); // Ensure data is immediately sent to the pipe
    }

    /// Terminates the pipeline by closing the input to the first process,
    /// waiting for both processes to exit, and closing the remaining pipe handles.
    /// This is crucial for proper resource cleanup and to allow the child processes
    /// to finish their work.
    pub fn terminate(self: *@This()) void {
        // Close the parent's writer to proc1's stdin.
        // This sends an EOF (End-Of-File) signal to the first process,
        // indicating that no more input will be provided. This allows it to finish
        // processing its input and subsequently close its stdout, which then signals
        // EOF to the second process.
        posix.close(self.parent_stdin_write_fd);

        // Wait for both child processes to terminate.
        // It's important to wait for them to ensure resources are cleaned up
        // and to prevent "zombie" processes.
        _ = posix.waitpid(self.proc1_pid, 0);
        _ = posix.waitpid(self.proc2_pid, 0);
    }
};

var pipeline: ProcessPipeline = undefined;

pub fn init2(allocator: std.mem.Allocator) !void {
    std.debug.print("Initializing process pipeline...\n", .{});

    // Simple test without complex pipeline - just try to exec a single command
    const pid = try posix.fork();
    if (pid == 0) {
        // Child process - try to exec a simple command
        std.debug.print("Child: About to exec /bin/echo\n", .{});

        // Create simple C-style argv array manually
        const arg0 = "/bin/echo";
        const arg1 = "test";

        // Allocate and create null-terminated strings
        const c_arg0 = try allocator.dupeZ(u8, arg0);
        defer allocator.free(c_arg0);
        const c_arg1 = try allocator.dupeZ(u8, arg1);
        defer allocator.free(c_arg1);

        // Create argv array
        const argv = [_]?*const u8{ @ptrCast(c_arg0.ptr), @ptrCast(c_arg1.ptr), null };

        std.debug.print("Child: About to call execvpeZ\n", .{});
        const err = posix.execvpeZ(@ptrCast(argv[0].?), @ptrCast(&argv), @ptrCast(std.os.environ.ptr));
        std.debug.print("Child: execvpeZ failed: {s}\n", .{@errorName(err)});
        posix.exit(1);
    } else {
        // Parent process
        std.debug.print("Parent: Waiting for child {}\n", .{pid});
        _ = posix.waitpid(pid, 0);
        std.debug.print("Parent: Child finished\n", .{});
    }
}

pub fn close2() !void {
    // In the simplified test version, there's no pipeline to clean up
    // The child process has already finished by the time this is called
    std.debug.print("close2: No cleanup needed for simplified test\n", .{});
}
