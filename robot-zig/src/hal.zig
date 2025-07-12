const std = @import("std");
const c = @cImport({
    @cInclude("pigpio.h");
    @cInclude("gpiod.h");
    @cInclude("sys/ioctl.h");
});

const linux = @cImport({
    @cInclude("linux/i2c.h");
    @cInclude("linux/i2c-dev.h");
});
const File = std.fs.File;
pub const Mode = enum { PIGPIO, GPIOD };
pub const Direction = enum { INPUT, OUTPUT };
pub const Pull = enum { NONE, DOWN, UP };
pub const Edge = enum { NONE, RISING, FALLING, BOTH };
pub const AltMode = enum { INPUT, OUTPUT, ALT0, ALT1, ALT2, ALT3, ALT4, ALT5 };

// Hardware GPIO pins
const pwm_gpio = [4]c_uint{ 12, 13, 18, 19 }; // GPIO 18 & 19 are also used for PCM
var pwmFreq: u32 = 20000; // default 20kHz
var pwm_period_ns: u32 = 1_000_000_000 / 20000;

// Pi 5
// Hardware PWM for motor power control
const PWM_CHIP_PATH = "/sys/class/pwm/pwmchip0";
const PWM_EXPORT_PATH = PWM_CHIP_PATH ++ "/export";
const PWM_PIN = 0; // PWM pin number on pwmchip0 (adjust as needed)

const PWMx_PIN_PATH = PWM_CHIP_PATH ++ "/pwm{d}";
const PWMx_PERIOD_PATH = PWMx_PIN_PATH ++ "/period";
const PWMx_DUTY_CYCLE_PATH = PWMx_PIN_PATH ++ "/duty_cycle";
const PWMx_ENABLE_PATH = PWMx_PIN_PATH ++ "/enable";

// const PWM0_PIN_PATH = PWM_CHIP_PATH ++ "/pwm0"; // pin numberhere too
// const PWM0_PERIOD_PATH = PWM0_PIN_PATH ++ "/period";
// const PWM0_DUTY_CYCLE_PATH = PWM0_PIN_PATH ++ "/duty_cycle";
// const PWM0_ENABLE_PATH = PWM0_PIN_PATH ++ "/enable";

// const PWM1_PIN_PATH = PWM_CHIP_PATH ++ "/pwm1"; // pin number
// const PWM1_PERIOD_PATH = PWM1_PIN_PATH ++ "/period";
// const PWM1_DUTY_CYCLE_PATH = PWM1_PIN_PATH ++ "/duty_cycle";
// const PWM1_ENABLE_PATH = PWM1_PIN_PATH ++ "/enable";
// These files are kept open for performance reasons
const MAX_PWM_CHANNEL = 3;
var pwmDutyCycleFiles: [MAX_PWM_CHANNEL + 1]?File = .{ null, null, null, null };
var pwm0DutyCycleFile: ?File = null;
var pwm1DutyCycleFile: ?File = null;
// gpiod
var chip: ?*c.gpiod_chip = null;
var lines: [32]?*c.gpiod_line = undefined;

// Is this right? do we need an area?
//const allocator = std.heap.c_allocator;
//var scratch: [256]u8 = undefined;
//var scratchPad = std.heap.FixedBufferAllocator.init(&scratch);

pub const HAL = struct {
    // Release resources
    close: *const fn () void,

    // GPIO
    gpioRead: *const fn (pin: u8) anyerror!u8,
    gpioWrite: *const fn (pin: u8, value: u8) anyerror!void,
    gpioSetDirection: *const fn (pin: u8, direction: Direction) anyerror!void,
    gpioSetPull: *const fn (pin: u8, pull: Pull) anyerror!void,
    gpioSetAltMode: *const fn (pin: u8, mode: AltMode) anyerror!void,
    // TODO
    // gpioAlert(pin:u8, edge:u8, callback: fn(gpio:u8, level:u8,tick: u32, userdata: ?*anyopaque) void)
    // PWM
    pwmSetDutyCycle: *const fn (channel: u8, duty_cycle: f32) anyerror!void,
    pwmSetFrequency: *const fn (channel: u8, frequency: u32) anyerror!void,
    pwmEnable: *const fn (channel: u8) anyerror!void,
    pwmDisable: *const fn (channel: u8) anyerror!void,
    // I2C
    i2cOpen: *const fn (bus: u8, address: u8) anyerror!File,
    i2cClose: *const fn (device: *const File) anyerror!void,
    i2cWrite: *const fn (device: *const File, register: u8, buffer: []u8) anyerror!void,
    i2cWriteByte: *const fn (device: *const File, register: u8, datum: u8) anyerror!void,
    i2cRead: *const fn (device: *const File, register: u8, buffer: []u8) anyerror!void,
    i2cReadByte: *const fn (device: *const File, register: u8) anyerror!u8,
    // SPI
    spiOpen: *const fn (bus: u8, device: u8) anyerror!File,
    spiWrite: *const fn (device: *File, buffer: []u8) anyerror!void,
    spiRead: *const fn (device: *File, buffer: []u8) anyerror!void,
    spiClose: *const fn (device: *File) anyerror!void,
};

var hal: ?HAL = null;

pub fn initialize(mode: Mode) anyerror!HAL {
    if (hal) |tmp| {
        return tmp;
    }
    hal = switch (mode) {
        .GPIOD => .{
            .close = closePiGPIO,
            .i2cOpen = openI2C,
            .i2cClose = closeI2C,
            .i2cRead = readI2C,
            .i2cReadByte = readI2CByte,
            .i2cWrite = writeI2C,
            .i2cWriteByte = writeI2CByte,
            .gpioRead = gpiodRead, // TODO: PI5 version
            .gpioWrite = gpiodWrite, // TODO: PI5 version
            .gpioSetDirection = gpiodSetDirection, // TODO: PI5 version
            .gpioSetPull = gpiodSetPullUpDown, // TODO: PI5 version
            .gpioSetAltMode = noopSetAltMode,
            .pwmSetDutyCycle = pi5PWMSetDutyCycle,
            .pwmSetFrequency = pi5PWMSetFrequency,
            .pwmEnable = pi5PWMEnable,
            .pwmDisable = pi5PWMDisable,
            .spiOpen = dummyOpen,
            .spiWrite = dummyWrite,
            .spiRead = dummyRead,
            .spiClose = dummyClose,
        },
        .PIGPIO => .{
            .close = closeGPIOD,
            .i2cOpen = openI2C,
            .i2cClose = closeI2C,
            .i2cRead = readI2C,
            .i2cReadByte = readI2CByte,
            .i2cWrite = writeI2C,
            .i2cWriteByte = writeI2CByte,
            .gpioRead = pigpioRead,
            .gpioWrite = pigpioWrite,
            .gpioSetDirection = pigpioSetDirection,
            .gpioSetPull = pigpioSetPullUpDown,
            .gpioSetAltMode = pigpioSetAltMode,
            .pwmSetDutyCycle = pigpioPwmSetDutyCycle,
            .pwmSetFrequency = pigpioPwmSetFrequency,
            .pwmEnable = pigpioPwmEnable,
            .pwmDisable = pigpioPwmDisable,
            .spiOpen = dummyOpen,
            .spiWrite = dummyWrite,
            .spiRead = dummyRead,
            .spiClose = dummyClose,
        },
    };
    switch (mode) {
        .GPIOD => {
            gpiodInit();
        },
        .PIGPIO => {
            try pigpioInit();
        },
    }

    return hal.?;
}

fn gpiodInit() void {
    chip = c.gpiod_chip_open_by_number(0); // "/dev/gpiochip0"
    //Sclose();
    // enable PWM

}

fn pigpioInit() !void {
    const pigpioVer = c.gpioInitialise();
    if (pigpioVer < 0) {
        std.debug.print("gpioInitialise() failed: {}\n", .{pigpioVer});
        return error.GPIO_Initialize_Failed;
    }
    _ = c.atexit(&cleanup_at_exit);
    std.debug.print("PIGPIO Version: {}\n", .{pigpioVer});
    std.debug.print("Hardware rev: {}\n", .{c.gpioHardwareRevision()});
    std.debug.print("GPIO Version: {}\n", .{c.gpioVersion()});
    std.debug.print("cfg = {}\n", .{c.gpioCfgGetInternals()});
}
pub export fn cleanup_at_exit() void {
    c.gpioTerminate();
}

fn closePiGPIO() void {
    closeCommon();
    c.gpioTerminate();
}

fn closeGPIOD() void {
    closeCommon();
    c.gpiod_chip_close(chip);
}

fn closeCommon() void {
    if (hal) |h| {
        h.pwmSetDutyCycle(0, 0) catch {};
        h.pwmSetDutyCycle(1, 0) catch {};
        h.pwmDisable(0) catch {};
        h.pwmDisable(1) catch {};
    }
}

// GPIO access
// Pi 2-4 using pigpio
fn pigpioRead(pin: u8) !u8 {
    const value = c.gpioRead(pin);
    if (value < 0)
        return error.GPIOReadFailed;
    return @intCast(value);
}

fn pigpioWrite(pin: u8, value: u8) !void {
    if (c.gpioWrite(pin, @intCast(value)) < 0)
        return error.GPIOWriteFailed;
}
fn pigpioSetDirection(pin: u8, direction: Direction) !void {
    // Setup the pin for input
    _ = c.gpioSetMode(pin, switch (direction) {
        Direction.INPUT => c.PI_INPUT,
        Direction.OUTPUT => c.PI_OUTPUT,
    });
}
fn pigpioSetPullUpDown(pin: u8, pud: Pull) !void {
    // Optional: enable pull-down/up if floating
    // Optional: enable pull-down/up if floating
    _ = c.gpioSetPullUpDown(pin, switch (pud) {
        Pull.DOWN => c.PI_PUD_DOWN,
        Pull.UP => c.PI_PUD_UP,
        Pull.NONE => c.PI_PUD_OFF,
    });
}
fn pigpioSetAltMode(pin: u8, mode: AltMode) !void {
    _ = c.gpioSetMode(pin, switch (mode) {
        AltMode.INPUT => c.PI_INPUT,
        AltMode.OUTPUT => c.PI_OUTPUT,
        AltMode.ALT0 => c.PI_ALT0,
        AltMode.ALT1 => c.PI_ALT1,
        AltMode.ALT2 => c.PI_ALT2,
        AltMode.ALT3 => c.PI_ALT3,
        AltMode.ALT4 => c.PI_ALT4,
        AltMode.ALT5 => c.PI_ALT5,
    });
}
// PWM (Hardware)
fn pigpioPwmSetFrequency(channel: u8, frequency: u32) !void {
    _ = channel;
    pwmFreq = frequency;
}
fn pigpioPwmSetDutyCycle(channel: u8, power: f32) !void {
    const gpio: c_uint = pwm_gpio[channel];
    const duty: c_uint = @as(c_uint, @intFromFloat(power * c.PI_HW_PWM_RANGE));
    _ = c.gpioHardwarePWM(gpio, pwmFreq, duty);
}
fn pigpioPwmEnable(channel: u8) !void { // No-op
    _ = channel;
}
fn pigpioPwmDisable(channel: u8) !void { // No-op
    _ = channel;
}

//============================
// Pi5 version using libgpiod
//============================
fn acquireGpioLine(gpio: u8) !?*c.gpiod_line {
    var line = lines[gpio];
    if (lines[gpio]) |prevLine| {
        line = prevLine;
    } else {
        line = c.gpiod_chip_get_line(chip, gpio); // e.g. motor direction
        if (line == null) return error.LineRequestFailed;
        lines[gpio] = line;
    }
    return line;
}
fn releaseGpioLine(gpio: u8) !void {
    if (lines[gpio]) |prevLine| {
        _ = c.gpiod_line_release(prevLine);
        lines[gpio] = null;
    }
}
fn gpiodSetDirection(gpio: u8, dir: Direction) !void {
    const line = try acquireGpioLine(gpio);
    var config: c.struct_gpiod_line_request_config = .{
        .consumer = "The Robot",
        .request_type = if (dir == Direction.OUTPUT) c.GPIOD_LINE_REQUEST_DIRECTION_OUTPUT else c.GPIOD_LINE_REQUEST_DIRECTION_INPUT,
        .flags = 0,
    };

    // set default level low
    if (c.gpiod_line_request(line, &config, 0) != 0)
        return error.RequestFailed;
}
fn gpiodRead(gpio: u8) !u8 {
    const line = try acquireGpioLine(gpio);
    return @intCast(c.gpiod_line_get_value(line));
}
fn gpiodWrite(gpio: u8, level: u8) !void {
    const line = try acquireGpioLine(gpio);
    _ = c.gpiod_line_set_value(line, level);
}
fn gpiodSetPullUpDown(gpio: u8, pud: Pull) !void {
    const line = try acquireGpioLine(gpio);
    const ret = c.gpiod_line_set_flags(line, switch (pud) {
        Pull.NONE => c.GPIOD_LINE_REQUEST_FLAG_BIAS_DISABLE,
        Pull.DOWN => c.GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN,
        Pull.UP => c.GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP,
    });
    if (ret != 0) {
        std.debug.print("Failed to set pull-up/down: {}\n", .{std.c._errno().*});
        return error.RequestFailed;
    }
}
fn noopSetAltMode(gpio: u8, mode: AltMode) !void {
    _ = gpio;
    _ = mode;
    std.debug.print("WARNING gpioSetAltMode ignored for gpiod HAL\n", .{});
}
fn getDutyCycleSysFile(channel: u8) !File {
    if (channel > MAX_PWM_CHANNEL) {
        return error.InvalidChannel;
    }
    if (pwmDutyCycleFiles[channel]) |f| {
        return f;
    }
    var buf: [128]u8 = undefined;
    const path = try std.fmt.bufPrint(&buf, PWMx_DUTY_CYCLE_PATH, .{channel});

    const file = std.fs.cwd().openFile(path, .{ .mode = .write_only }) catch |err| {
        std.debug.print("Failed to open {s}: {}\n", .{ path, err });
        return error.OpenFailed;
    };
    pwmDutyCycleFiles[channel] = file;
    return file;
}
// PWM via /sys/class/pwm/pwmchip0
fn setPWMValue(path: []const u8, value: u32) !void {
    const file = std.fs.cwd().openFile(path, .{ .mode = .write_only }) catch |err| {
        std.debug.print("Failed to open {s}: {}\n", .{ path, err });
        return error.OpenFailed;
    };
    defer file.close();
    var buf: [16]u8 = undefined;
    const str_value = try std.fmt.bufPrint(&buf, "{d}", .{value});
    //const str_value = try std.fmt.allocPrint(allocator, "{d}", .{value});
    //defer allocator.free(str_value);
    try file.writeAll(str_value);
}
fn setPWMFileValue(file: ?File, value: u32) !void {
    var buf: [16]u8 = undefined;
    const str_value = try std.fmt.bufPrint(&buf, "{d}", .{value});
    try file.?.writeAll(str_value);
}

fn exportPWMIfNeeded(channel: u8) !void {
    var buf: [128]u8 = undefined;
    const path = try std.fmt.bufPrint(&buf, PWMx_PIN_PATH, .{channel});
    // Check if the PWM pin is already exported
    var pwm_dir = std.fs.cwd().openDir(path, .{}) catch {
        std.debug.print("Exporting PWM pin for channel {d}...\n", .{channel});
        // Export the pin by writing to the export file
        try setPWMValue(PWM_EXPORT_PATH, channel);
        std.debug.print("PWM pin exported successfully for channel {d}.\n", .{channel});
        return;
    };
    defer pwm_dir.close();
    std.debug.print("PWM pin already exported for channel {d}.\n", .{channel});
}
fn pi5PWMSetFrequency(channel: u8, frequency: u32) !void {
    try exportPWMIfNeeded(channel);
    pwmFreq = frequency;
    pwm_period_ns = 1_000_000_000 / frequency; // Period in nanoseconds (1 second = 1 billion ns)
    var buf: [128]u8 = undefined;
    const path = try std.fmt.bufPrint(&buf, PWMx_PERIOD_PATH, .{channel});
    try setPWMValue(path, pwm_period_ns);
}
fn pi5PWMSetDutyCycle(channel: u8, power: f32) !void {
    const duty_cycle: u32 = @intFromFloat(@as(f32, @floatFromInt(pwm_period_ns)) * @abs(power));
    try setPWMFileValue(try getDutyCycleSysFile(channel), duty_cycle);
}
fn pi5PWMEnable(channel: u8) !void {
    var buf: [128]u8 = undefined;
    const path = try std.fmt.bufPrint(&buf, PWMx_ENABLE_PATH, .{channel});
    try setPWMValue(path, 1); // Enable PWM
}
fn pi5PWMDisable(channel: u8) !void {
    var buf: [128]u8 = undefined;
    const path = try std.fmt.bufPrint(&buf, PWMx_ENABLE_PATH, .{channel});
    try setPWMValue(path, 0); // Disable PWM
}

// Open I2C device
fn openI2C(bus: u8, addr: u8) !File {
    var buf: [16]u8 = undefined;
    const path = try std.fmt.bufPrint(&buf, "/dev/i2c-{}", .{bus});
    const fd = try std.fs.cwd().openFile(path, .{ .mode = .read_write });

    if (c.ioctl(fd.handle, linux.I2C_SLAVE, addr) < 0)
        return error.I2CSlaveSetFailed;
    return fd;
}

fn closeI2C(file: *const File) !void {
    file.close();
}

// Function to write data to I2C device
fn writeI2C(file: *const File, register: u8, data: []const u8) !void {
    // First, write the register address (the register offset)
    //
    const combined_len = 1 + data.len;
    if (combined_len > 256)
        return error.BufferTooLarge;
    var temp_buffer: [256]u8 = undefined;
    var combined_buffer = temp_buffer[0..combined_len];
    combined_buffer[0] = register;
    std.mem.copyForwards(u8, combined_buffer[1..], data);
    try file.writeAll(combined_buffer);
}
fn writeI2CByte(file: *const File, register: u8, datum: u8) !void {
    const registerData: [2]u8 = .{ register, datum };
    try file.writeAll(&registerData);
}

// Function to read data from I2C device
fn readI2C(file: *const File, register: u8, buffer: []u8) !void {
    const registerData = [_]u8{register};
    try file.writeAll(&registerData);
    const readCount = try file.readAll(buffer);
    if (readCount != buffer.len) {
        std.debug.print("Reading {d} bytes from register offset 0x{x:2} from I2C (setting register offset)\n", .{ buffer.len, register });
        std.debug.print("Read failed {d}\n", .{readCount});
        return error.ReadFailed;
    }
}

fn readI2CByte(file: *const File, register: u8) !u8 {
    var buffer = [_]u8{0};
    try readI2C(file, register, &buffer);
    return buffer[0];
}

// SPI - ToDo
fn dummyOpen(bus: u8, device: u8) !File {
    _ = bus;
    _ = device;
    return error.NotImplemented;
}

fn dummyWrite(file: *const File, data: []const u8) !void {
    _ = file;
    _ = data;
}
fn dummyRead(file: *const File, buffer: []u8) !void {
    _ = file;
    _ = buffer;
}
fn dummyClose(file: *const File) !void {
    _ = file;
}
