//! By convention, root.zig is the root source file when making a library. If
//! you are making an executable, the convention is to delete this file and
//! start with main.zig instead.
const std = @import("std");
const testing = std.testing;

const c = @cImport({
    @cInclude("stdlib.h");
    @cInclude("pigpio.h");
});

// IMU
// New version uses  BMI 160 IMU
const bmi160 = @import("bmi160.zig");
var lmuData: bmi160.NormalizedSensorData = undefined;

// Old version used the  LSM9DS1 Gyroscope/Accelerometer, Magnetoscope //
///////////////////////////////////////////////////
//const lsm9ds1 = @import("lsm9ds1");

// Motor control
// PWM via two ZS-X11H modules for the main drive wheels
const pwm_gpio = [4]c_uint{ 12, 13, 18, 19 }; // GPIO 18 & 19 are also used for PCM
const direction_gpio = [2]c_uint{ 26, 27 };

// 16-channel I2C Servo control board PCA9685
// TODO

const Raw3 = struct { x: i16, y: i16, z: i16 };
const Vec3 = struct {
    x: f32,
    y: f32,
    z: f32,
    fn dot(self: Vec3, other: Vec3) f32 {
        return self.x * other.x + self.y * other.y + self.z * other.z;
    }
    fn normalize(self: *Vec3) Vec3 {
        const length = std.math.sqrt(self.dot(self));
        if (length == 0)
            return self;
        return Vec3{ .x = self.x / length, .y = self.y / length, .z = self.z / length };
    }
    // TODO: crossProduct
};

var temp: u16 = 0;

var screen = [64]u16{
    0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 0, 0, 0, 0, 0, //
    0, 0, 0, 0, 0, 0, 0, 0,
};
// Reinterpret as a pointer to an array of 128 u8 for the writeAll call
const bytes: *[128]u8 = @ptrCast(&screen);

pub fn initialize() !void {
    const pigpioVer = c.gpioInitialise();
    if (pigpioVer < 0) {
        std.debug.print("gpioInitialise() failed: {}\n", .{pigpioVer});
        return error.GPIO_Initialize_Failed;
    }
    _ = c.atexit(&cleanup);
    std.debug.print("PIGPIO Version: {}\n", .{pigpioVer});
    std.debug.print("Hardware rev: {}\n", .{c.gpioHardwareRevision()});
    std.debug.print("GPIO Version: {}\n", .{c.gpioVersion()});
    std.debug.print("cfg = {}\n", .{c.gpioCfgGetInternals()});

    try setMode(24, c.PI_INPUT);

    // Configure HW PWM pins
    try setMode(12, c.PI_ALT0); // ALT0 is HW PWM
    try setMode(13, c.PI_ALT0);
    try setMode(18, c.PI_ALT0);
    try setMode(19, c.PI_ALT0);

    // Configure outputs for direction control
    try setMode(direction_gpio[0], c.PI_OUTPUT);
    try setMode(direction_gpio[1], c.PI_OUTPUT);
    try setPullUpDown(direction_gpio[0], c.PI_PUD_DOWN);
    try setPullUpDown(direction_gpio[1], c.PI_PUD_DOWN);

    const duty_cycle = c.gpioGetPWMdutycycle(12);
    std.debug.print("PWM dutycycle for GPIO 12 (PWM0) is {}\n", .{duty_cycle});
    const real_range = c.gpioGetPWMrealRange(12);
    std.debug.print("Real PWM range for GPIO 12 is {}\n", .{real_range});

    try initializeIMU();

    clearScreen();
}

pub fn initializeIMU() !void {
    bmi160.init() catch {
        std.debug.print("BMI160 init failed.\n", .{});
        return error.BMI160_Initialize_Failed;
    };
}

pub export fn cleanup() void {
    std.debug.print("Exiting...\n", .{});
    bmi160.close();
    c.gpioTerminate();
    //std.os.tcsetattr(std.io.getStdIn().handle, std.os.TermiosHow.TCSANOW, std.heap.page_allocator); // Restore mode on exit

}

// fn setTerminalRawMode(fd: std.os.fd_t) !std.mem.Allocator {
//     var termios = try std.os.tcgetattr(fd);
//     termios.c_lflag &= ~std.os.Lflag.ICANON; // Disable canonical mode
//     termios.c_lflag &= ~std.os.Lflag.ECHO;   // Disable echo
//     termios.c_cc[std.os.SpecialControlChar.VMIN] = 1;  // Minimum number of bytes
//     termios.c_cc[std.os.SpecialControlChar.VTIME] = 0; // No timeout
//
//     try std.os.tcsetattr(fd, std.os.TermiosHow.TCSANOW, &termios);
//     return std.heap.page_allocator;
// }

pub fn readSensors() !void {
    lmuData = try bmi160.readSensorDataNormalized();

    // TODO: Laser Ranging

    // TODO: Sonar

    // TODO: Microphone
}

// comp-time struct conversion (requires matching fields)
fn convertStruct(To: type, from: anytype) To {
    var result: To = undefined;
    inline for (@typeInfo(To).@"struct".fields) |field| {
        @field(result, field.name) = @field(from, field.name);
    }
    return result;
}

pub fn acceleration() Vec3 {
    return convertStruct(Vec3, lmuData.accel);
}

pub fn orientation() Vec3 {
    return convertStruct(Vec3, lmuData.gyro);
}

pub fn tilt_angle_yz() f32 {
    return std.math.atan2(lmuData.accel.y, lmuData.accel.z) * (180.0 / std.math.pi);
}

pub fn dumpSensors() void {
    std.debug.print("\x1b[3AAccel.X: {d: >7.4}g, Gyro.X: {d: >7.4}°/s\n", .{ lmuData.accel.x, lmuData.gyro.x });
    std.debug.print("Accel.Y: {d: >7.4}g, Gyro.Y: {d: >7.4}°/s\n", .{ lmuData.accel.y, lmuData.gyro.y });
    std.debug.print("Accel.Z: {d: >7.4}g, Gyro.Z: {d: >7.4}°/s   Tilt: {d: >6.2}°\n", .{ lmuData.accel.z, lmuData.gyro.z, tilt_angle_yz() });
}

fn setMode(gpio: c_uint, mode: c_uint) !void {
    const ret = c.gpioSetMode(gpio, mode);
    switch (ret) {
        c.PI_BAD_GPIO => {
            std.debug.print("gpioSetMode: BAD_GPIO for gpio = {}, mode = {}\n", .{ gpio, mode });
        },
        c.PI_BAD_MODE => {
            std.debug.print("gpioSetMode: BAD_MODE for gpio = {}. mode = {}\n", .{ gpio, mode });
        },
        else => {},
    }
}

fn setPullUpDown(gpio: c_uint, updown: c_uint) !void {
    const ret = c.gpioSetPullUpDown(gpio, updown);
    if (ret != 0) {
        std.debug.print("Failed to set pull up-down for GPIO {} to {}\n", .{ gpio, updown });
    }
}

// velocity here is actually power output, we aren't measuring and compensating for actual wheel speed yet.
pub fn driveMotor(left_right: bool, velocity: f32) bool {
    const channel: u8 = if (left_right) 0 else 1;
    const direction: bool = if (left_right) (velocity >= 0) else !(velocity >= 0);
    _ = c.gpioWrite(direction_gpio[channel], if (direction) 0 else 1);
    const power = if (velocity < 0.0) -velocity else velocity;
    return hw_pwm(channel, power);
}

pub fn hw_pwm(channel: u8, power: f32) bool {
    if (power > 1.0 or power < 0.0) {
        std.debug.print("power out of range: {}\n", .{power});
        return false;
    }
    if (channel > 3) {
        std.debug.print("Channel out of range: {}\n", .{channel});
        return false;
    }
    const gpio: c_uint = pwm_gpio[channel];
    const duty: c_uint = @as(c_uint, @intFromFloat(power * c.PI_HW_PWM_RANGE));
    const ret: c_int = c.gpioHardwarePWM(gpio, 20000, duty);
    switch (ret) {
        c.PI_BAD_GPIO => {
            std.debug.print("hw_pwm: BAD_GPIO\n", .{});
            return false;
        },
        c.PI_NOT_HPWM_GPIO => {
            std.debug.print("hw_pwm: NOT_HPWM_GPIO\n", .{});
            return false;
        },
        c.PI_BAD_HPWM_DUTY => {
            std.debug.print("hw_pwm: BAD_HPWM_DUTY\n", .{});
            return false;
        },
        c.PI_BAD_HPWM_FREQ => {
            std.debug.print("hw_pwm: BAD_HPWM_FREQ\n", .{});
            return false;
        },
        c.PI_HPWM_ILLEGAL => {
            std.debug.print("hw_pwm: HPWM_ILLEGAL\n", .{});
            return false;
        },
        else => {},
    }
    return true;
}

pub fn sleepMillis(millis: u32) void {
    std.time.sleep(@as(u64, millis) * 1_000_000);
}

pub export fn plot(x: u8, y: u8, colour: u16) void {
    screen[y * 8 + x] = colour;
}

pub fn clearScreen() void {
    @memset(&screen, 0);
}

pub fn flip() void {
    const frameBuffer = std.fs.openFileAbsolute("/dev/fb1", .{ .mode = .write_only }) catch unreachable;
    defer frameBuffer.close();
    frameBuffer.writeAll(bytes) catch unreachable;
}

// For Speed measurement, we need to use the hardware timer
// and the GPIO interrupt to measure the time between pulses.

// TODO: pick the right GPIO pin for speed measurement
const SPEED_PIN: c_uint = 24; // GPIO pin for speed measurement

pub fn setupSpeedMeasurement() !void {
    // Setup the pin for input
    _ = c.gpioSetMode(SPEED_PIN, c.PI_INPUT);

    // Optional: enable pull-down/up if floating
    _ = c.gpioSetPullUpDown(SPEED_PIN, c.PI_PUD_DOWN);

    // Register the callback
    _ = c.gpioSetAlertFunc(SPEED_PIN, speed_callback);
}

// Use a callback to count edges
var last_tick: u32 = 0;
var pulse_interval_us: u32 = 0;
var current_rpm: f32 = 0.0;
const pulses_per_rev: u32 = 6; // Adjust as needed TODO double check this value

fn speed_callback(gpio: i32, level: i32, tick: u32, userdata: ?*anyopaque) callconv(.C) void {
    _ = gpio; // Unused parameter
    _ = userdata; // Unused parameter

    if (level != 1) return; // Rising edge only

    if (last_tick != 0) {
        const delta = tick - last_tick;
        pulse_interval_us = delta;

        // Convert pulse interval to RPM:
        // 1e6 µs/sec, 60 sec/min, divide by pulses per revolution
        if (pulse_interval_us != 0) {
            const freq_hz = 1_000_000.0 / @as(f32, @floatFromInt(pulse_interval_us));
            current_rpm = freq_hz * 60.0 / @as(f32, @floatFromInt(pulses_per_rev));
        }
    }

    last_tick = tick;
}
pub fn getCurrentRPM() f32 {
    return current_rpm;
}
