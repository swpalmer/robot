// Interface to the local Llama model
const std = @import("std");

var allocator: std.mem.Allocator = undefined;
pub fn init(
    alloc: std.mem.Allocator,
) !void {
    allocator = alloc;
    // Initialize the Llama model here
}

pub fn deinit() !void {
    // Deinitialize the Llama model here
}

// Sends a prompt to the robot using the Llama model
// consumes the response and speaks it out loud
// with the robot's voice
pub fn tellRobot(prompt: []const u8) !void {
    // Tell the robot using the Llama model here
    //
    // POST /api/generate
    // Parameters
    //
    //  model: (required) the model name
    //  prompt: the prompt to generate a response for
    //  suffix: the text after the model response
    //  images: (optional) a list of base64-encoded images (for multimodal models such as llava)
    //  think: (for thinking models) should the model think before responding?
    //
    //
    try converse2(prompt, undefined);
}

pub fn showRobot() !void {
    // Show the robot an image using the Llama model here
}

//url http://localhost:11434/api/generate -d '{
//  "model": "llama3.2",
//  "prompt": "Why is the sky blue?"
//}'

// Tell the robot something, including showing it images
// and stream it's response to the TTS engine.
//  - tellRobot() and sjowRobot() call this
fn converse(prompt: []const u8, imageBase64: []const u8) !void {
    // Generate a response using the Llama model here
    _ = prompt;
    _ = imageBase64;

    // Create a HTTP client
    var client = std.http.Client{ .allocator = allocator };
    defer client.deinit();

    // Allocate a buffer for server headers
    var buf: [4096]u8 = undefined;

    const payload =
        \\ {
        \\  "model": "llama2:7b",
        \\  "format": "json",
        \\  "stream": false,
        \\  "prompt": "You are a curious autonomous robot named Sam. Your responses should be brief and to the point. Introduce yourself."
        \\ }
    ;

    // Start the HTTP request
    const uri = try std.Uri.parse("http://localhost:11434/api/generate");
    var req = try client.open(.POST, uri, .{ .server_header_buffer = &buf });
    defer req.deinit();

    req.transfer_encoding = .{ .content_length = payload.len };
    // Send the HTTP request headers
    try req.send();
    // send the body of the request
    var wtr = req.writer();
    try wtr.writeAll(payload);
    // Finish the body of a request
    try req.finish();
    std.debug.print("Request Body:\n{s}\n", .{payload});

    // Waits for a response from the server and parses any headers that are sent
    try req.wait();

    // Read the body
    var bbuffer: [65536]u8 = undefined;
    const hlength = req.response.parser.header_bytes_len;
    _ = try req.readAll(&bbuffer);
    const blength = req.response.content_length orelse return error.NoBodyLength; // We trust

    std.debug.print("Headers...\n", .{});
    var iter = req.response.iterateHeaders();
    while (iter.next()) |header| {
        std.debug.print("{s}: {s}\n", .{ header.name, header.value });
    }

    // the Content-Length returned by the server…
    std.debug.print("------\nstatus={d}\nheader bytes: {d}\nbody content bytes: {d}\n", .{ req.response.status, hlength, blength });
    // if response was not ok - return now
    if (req.response.status != .ok)
        return;

    var rdr = req.reader();
    //const body: []u8 = try allocator.alloc(u8, blength);
    //defer allocator.free(body);
    //const read_size = try rdr.readAtLeast(body, blength);
    //std.debug.print("read_size: {d}\n", .{read_size});
    const body = try rdr.readAllAlloc(allocator, 65536);
    defer allocator.free(body);

    std.debug.print("Body:\n{s}\n", .{body});

    // Parse JSON
    //const json_blob = try std.json.parseFromSlice(Result, allocator, body, .{ .ignore_unknown_fields = true });
}

const Result = struct {
    model: []const u8,
    created_at: []const u8,
    response: []const u8,
    done: bool,
    done_reason: []const u8,
    context: []const u32,
    total_duration: u64,
    load_duration: u64,
    prompt_eval_count: u32,
    prompt_eval_duration: u64,
    eval_count: u32,
    eval_duration: u64,
};

fn converse2(prompt: []const u8, imageBase64: []const u8) !void {
    _ = prompt;
    _ = imageBase64;

    // will have to add
    //   format: "json"
    // and deal with
    //   stream: true
    // for better user experience
    const payload =
        \\ {
        \\  "model": "deepseek-r1:1.5b",
        \\  "stream": false,
        \\  "prompt": "You are a curious autonomous robot named Sam. Your responses should be brief and to the point. Introduce yourself."
        \\ }
    ;

    // Create a HTTP client
    var client = std.http.Client{ .allocator = allocator };
    defer client.deinit();

    //We can set up any headers we want
    const headers = &[_]std.http.Header{
        .{ .name = "X-Custom-Header", .value = "application" },
        // if we wanted to do a post request with JSON payload we would add
        // .{ .name = "Content-Type", .value = "application/json" },
    };

    const response = try post(&client, "http://localhost:11434/api/generate", headers, payload);

    // .ignore_unknown_fields will just omit any fields the server returns that are not in our type
    // otherwise an unknown field causes an error
    const result = try std.json.parseFromSlice(Result, allocator, response.items, .{ .ignore_unknown_fields = true });

    std.debug.print("Response: {s}\n", .{result.value.response});
}

fn post(client: *std.http.Client, url: []const u8, headers: []const std.http.Header, payload: []const u8) !std.ArrayList(u8) {
    var response_body = std.ArrayList(u8).init(allocator);
    const response = try client.fetch(.{
        .method = .POST,
        .location = .{ .url = url },
        .extra_headers = headers, //put these here instead of .headers
        .response_storage = .{ .dynamic = &response_body }, // this allows us to get a response of unknown size
        // if we were doing a post request we would include the payload here
        .payload = payload,
    });

    std.debug.print("Response Status: {d}\n Response Body:{s}\n", .{ response.status, response_body.items });
    return response_body;
}
