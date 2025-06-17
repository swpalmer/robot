// websocket for realtime control
// using https://github.com/karlseguin/websocket.zig
// To add to build.zig.zon
// use:
//  zig fetch --save "git+https://github.com/karlseguin/websocket.zig#master"
// That will add the websocket.zig dependency to your project's build.zon file
// in the .dependencies section:
//   .dependencies = .{
//     .httpz = .{
//         .url = "git+https://github.com/karlseguin/http.zig?ref=master#8ecf3a330ab1bed8495604d444e549b94f08bc0f",
//         .hash = "httpz-0.0.0-PNVzrOy2BgBA2lU1zixuVrv0UUkSnVrBUIlIHl1XV0XV",
//     },
//     .websocket = .{
//         .url = "git+https://github.com/karlseguin/websocket.zig?ref=master#10b0e7be2158ff22733f1e59c1ae0bace5bf3a0c",
//         .hash = "websocket-0.1.0-ZPISdbhVAwCL-zrOIFLKLr6CHtwV92raAnObudtbYQjJ",
//     },
//   },
const std = @import("std");
const httpz = @import("httpz");
const types = @import("types");
const tts = @import("speech");
const websocket = httpz.websocket;

const PIDValues = types.PIDValues;
const SpeakParams = struct { text: []u8 };
const DriveParams = struct { x: f32, y: f32 };

// This is the context that will be available to every request handler.
pub const WebContext = struct {
    pid: *PIDValues,
    upright: *PIDValues,
    falling: *PIDValues,
    flag_ptr: *volatile bool,
    x: *f32,
    y: *f32,
    pub const WebsocketHandler = WSClient;
};

pub fn webserver(context: *WebContext) !httpz.Server(*WebContext) {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const allocator = gpa.allocator();

    var server = try httpz.Server(*WebContext).init(allocator, .{ .address = "0.0.0.0", .port = 40607 }, context);

    var router = try server.router(.{});
    // Main page
    router.get("/robot/*", robotPage, .{});

    // APIs
    router.get("/api/pidCurrent", getCurrentPID, .{});
    router.post("/api/pidUpright", postUprightPID, .{});
    router.post("/api/pidFalling", postFallingPID, .{});
    router.post("/api/speak", postSpeak, .{});
    router.post("/api/drive", postDrive, .{});
    router.post("/api/reset", postReset, .{});

    // WebScoket for steering control
    router.get("/ws", ws, .{});

    //blocks
    //try server.listen();

    // non blocking...
    _ = try server.listenInNewThread();
    return server;
}

const robot_prefix = "/robot/";

fn robotPage(context: *WebContext, req: *httpz.Request, res: *httpz.Response) !void {
    _ = context;
    // req.url;
    std.debug.print("Request for robot page: {s}\n", .{req.url.path});

    res.status = 200;
    //try res.json(.{ .id = "Hello", .number = 42 }, .{});
    //res.body = try std.fmt.allocPrint(res.arena, @embedFile("index.html"), .{ @embedFile("script.js"), context.pid.Kp, context.pid.Ki, context.pid.Kd, context.upright.Kp, context.upright.Ki, context.upright.Kd, context.falling.Kp, context.falling.Ki, context.falling.Kd });

    if (std.mem.eql(u8, req.url.path, "/robot/") or std.mem.eql(u8, req.url.path, "/robot")) {
        // Redirect to the main page
        res.status = 301;
        res.header("Location", "/robot/index.html");
        return;
    }

    if (!std.mem.startsWith(u8, req.url.path, robot_prefix)) {
        res.status = 404;
        res.body = "Not Found";
        return;
    }
    // Strip "/robot/" and prepend "www/"
    var file_path_buf: [256]u8 = undefined;
    const rel_path = req.url.path[robot_prefix.len..];
    const file_path = try std.fmt.bufPrint(&file_path_buf, "www/{s}", .{rel_path});
    //const file_path = try std.fmt.bufPrint(&file_path_buf, "www/{s}", .{req.url.path});

    var file = try std.fs.cwd().openFile(file_path, .{});
    defer file.close();
    const file_content = try file.readToEndAlloc(res.arena, 65536);
    res.body = file_content;
    res.status = 200;
    // Set the content type based on the file extension
    if (std.mem.endsWith(u8, file_path, ".html")) {
        res.header("Content-Type", "text/html; charset=utf-8");
    } else if (std.mem.endsWith(u8, file_path, ".js")) {
        res.header("Content-Type", "application/javascript; charset=utf-8");
    } else if (std.mem.endsWith(u8, file_path, ".css")) {
        res.header("Content-Type", "text/css; charset=utf-8");
    } else if (std.mem.endsWith(u8, file_path, ".png")) {
        res.header("Content-Type", "image/png");
    } else if (std.mem.endsWith(u8, file_path, ".jpg") or std.mem.endsWith(u8, file_path, ".jpeg")) {
        res.header("Content-Type", "image/jpeg");
    } else if (std.mem.endsWith(u8, file_path, ".gif")) {
        res.header("Content-Type", "image/gif");
    } else if (std.mem.endsWith(u8, file_path, ".svg")) {
        res.header("Content-Type", "image/svg+xml");
    } else if (std.mem.endsWith(u8, file_path, ".ico")) {
        res.header("Content-Type", "image/x-icon");
    } else if (std.mem.endsWith(u8, file_path, ".webp")) {
        res.header("Content-Type", "image/webp");
    } else if (std.mem.endsWith(u8, file_path, ".woff") or std.mem.endsWith(u8, file_path, ".woff2")) {
        res.header("Content-Type", "font/woff");
    } else if (std.mem.endsWith(u8, file_path, ".ttf")) {
        res.header("Content-Type", "font/ttf");
    } else if (std.mem.endsWith(u8, file_path, ".otf")) {
        res.header("Content-Type", "font/otf");
    } else if (std.mem.endsWith(u8, file_path, ".json")) {
        res.header("Content-Type", "application/json; charset=utf-8");
    } else {
        // Default to plain text if no specific type is matched
        res.header("Content-Type", "text/plain; charset=utf-8");
    }
}

fn getCurrentPID(context: *WebContext, req: *httpz.Request, res: *httpz.Response) !void {
    _ = req;
    res.status = 200;

    // Return the current PID values
    try res.json(.{
        .Kp = context.pid.Kp,
        .Ki = context.pid.Ki,
        .Kd = context.pid.Kd,
        .upright = .{
            .Kp = context.upright.Kp,
            .Ki = context.upright.Ki,
            .Kd = context.upright.Kd,
        },
        .falling = .{
            .Kp = context.falling.Kp,
            .Ki = context.falling.Ki,
            .Kd = context.falling.Kd,
        },
    }, .{});
}
// This is the handler for the /pidUpright endpoint

fn postUprightPID(context: *WebContext, req: *httpz.Request, res: *httpz.Response) !void {
    //_ = req;
    res.status = 200;

    if (try req.json(PIDValues)) |requestedPidValues| {
        // This isn't atomic... perhaps there is a better way... (E.g. copy from a secondary PIDValues object in main the loop)
        context.upright.Kp = requestedPidValues.Kp;
        context.upright.Ki = requestedPidValues.Ki;
        context.upright.Kd = requestedPidValues.Kd;
    }

    // use this a signal that we can try balancing again
    context.flag_ptr.* = false;

    try res.json(.{ .Kp = context.pid.Kp, .Ki = context.pid.Ki, .Kd = context.pid.Kd }, .{});
}

fn postFallingPID(context: *WebContext, req: *httpz.Request, res: *httpz.Response) !void {
    //_ = req;
    res.status = 200;

    if (try req.json(PIDValues)) |requestedPidValues| {
        // This isn't atomic... perhaps there is a better way... (E.g. copy from a secondary PIDValues object in main the loop)
        context.falling.Kp = requestedPidValues.Kp;
        context.falling.Ki = requestedPidValues.Ki;
        context.falling.Kd = requestedPidValues.Kd;
    }

    // use this a signal that we can try balancing again
    context.flag_ptr.* = false;

    try res.json(.{ .Kp = context.pid.Kp, .Ki = context.pid.Ki, .Kd = context.pid.Kd }, .{});
}

fn postSpeak(_: *WebContext, req: *httpz.Request, res: *httpz.Response) !void {
    res.status = 200;

    if (try req.json(SpeakParams)) |requestedSpeech| {
        // Allocate a null-terminated string in the response arena
        const text_len = requestedSpeech.text.len;
        std.debug.print("Was asked to say: {s} [len: {}]\n", .{ requestedSpeech.text, text_len });
        var nt_text = try res.arena.alloc(u8, text_len + 1);
        std.mem.copyForwards(u8, nt_text[0..text_len], requestedSpeech.text);
        nt_text[text_len] = 0; // null terminator
        try tts.speak(nt_text[0..text_len :0]);
    }

    // _ = req;
    // try tts.playMonoSine44100Hz();

    try res.json(.{}, .{});
}

fn postDrive(context: *WebContext, req: *httpz.Request, res: *httpz.Response) !void {
    if (try req.json(DriveParams)) |joystick| {
        std.debug.print("Drive control: x:{d} y:{d}\n", .{ joystick.x, joystick.y });
        context.x.* = joystick.x;
        context.y.* = joystick.y;
    }
    try res.json(.{}, .{});
    res.status = 200;
}

fn postReset(context: *WebContext, req: *httpz.Request, res: *httpz.Response) !void {
    _ = req;
    context.x.* = 0;
    context.y.* = 0;
    // Reset PID values to defaults
    context.pid.Kp = 0.0;
    context.pid.Ki = 0.0;
    context.pid.Kd = 0.0;
    context.upright.Kp = 14.0;
    context.upright.Ki = 0.0;
    context.upright.Kd = 5.0;
    context.falling.Kp = 100.0;
    context.falling.Ki = 0.0;
    context.falling.Kd = 15.0;
    // use this a signal that we can try balancing again
    context.flag_ptr.* = false;

    res.status = 200;
    try res.json(.{}, .{});
}
const WSClient = struct {
    user_id: u32,
    conn: *websocket.Conn,

    const Context = struct {
        user_id: u32,
    };

    // context is any abitrary data that you want, you'll pass it to upgradeWebsocket
    pub fn init(conn: *websocket.Conn, ctx: *const Context) !WSClient {
        return .{
            .conn = conn,
            .user_id = ctx.user_id,
        };
    }

    // at this point, it's safe to write to conn
    pub fn afterInit(self: *WSClient) !void {
        return self.conn.write("welcome!");
    }

    pub fn clientMessage(self: *WSClient, data: []const u8) !void {
        // echo back to client
        return self.conn.write(data);
    }
};

fn ws(_: *WebContext, req: *httpz.Request, res: *httpz.Response) !void {
    // Could do authentication or anything else before upgrading the connection
    // The context is any arbitrary data you want to pass to Client.init.
    const ctx = WSClient.Context{ .user_id = 9001 };

    // The first parameter, WSClient, ***MUST*** be the same as Handler.WebSocketHandler
    // I'm sorry about the awkwardness of that.
    // It's undefined behavior if they don't match, and it _will_ behave weirdly/crash.
    if (try httpz.upgradeWebsocket(WSClient, req, res, &ctx) == false) {
        res.status = 500;
        res.body = "invalid websocket";
    }
    // unsafe to use req or res at this point!
}
