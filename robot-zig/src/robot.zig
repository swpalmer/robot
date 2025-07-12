// Robot Control Layer
// Providing access to Robot peripherals
// Including sensors, switches, motors, and actuators.
const std = @import("std");
const halMod = @import("hal");
const bmi160 = @import("bmi160.zig");
const types = @import("types");
const HAL = halMod.HAL;
const AltMode = halMod.AltMode;
const Direction = halMod.Direction;
const Mode = halMod.Mode;
const Pull = halMod.Pull;
const halMode = Mode.GPIOD;
var hal: HAL = undefined;
const Vec3 = types.Vec3;

// IMU
pub var lmuData: types.NormalizedSensorData = undefined;

// Motor control
// PWM via two ZS-X11H modules for the main drive wheels
const MOTOR_CHAN_A: u8 = 0;
const MOTOR_CHAN_B: u8 = 1;
const pwm_gpio = [4]c_uint{ 12, 13, 18, 19 }; // GPIO 18 & 19 are also used for PCM
const direction_gpio = [2]u8{ 26, 27 };

pub fn initialize() !void {
    hal = try halMod.initialize(halMode);
    try initialize_w_hal();
}

fn initialize_w_hal() !void {
    try hal.gpioSetDirection(24, Direction.INPUT);

    // Configure HW PWM pins
    try hal.gpioSetAltMode(12, AltMode.ALT0); // ALT0 is HW PWM
    try hal.gpioSetAltMode(13, AltMode.ALT0);
    try hal.gpioSetAltMode(18, AltMode.ALT0);
    try hal.gpioSetAltMode(19, AltMode.ALT0);

    // Configure outputs for direction control
    try hal.gpioSetDirection(direction_gpio[0], Direction.OUTPUT);
    try hal.gpioSetDirection(direction_gpio[1], Direction.OUTPUT);
    try hal.gpioSetPull(direction_gpio[0], Pull.DOWN);
    try hal.gpioSetPull(direction_gpio[1], Pull.DOWN);

    //const duty_cycle = c.gpioGetPWMdutycycle(12);
    //std.debug.print("PWM dutycycle for GPIO 12 (PWM0) is {}\n", .{duty_cycle});
    //const real_range = c.gpioGetPWMrealRange(12);
    //std.debug.print("Real PWM range for GPIO 12 is {}\n", .{real_range});
    try initializePWM();
    try initializeIMU();

    //clearScreen();
    // Setup GPIO, PWM, I2C etc
}

fn initializePWM() !void {
    try hal.gpioSetDirection(12, Direction.OUTPUT);
    try hal.gpioSetDirection(13, Direction.OUTPUT);
    try hal.gpioSetDirection(18, Direction.OUTPUT);
    try hal.gpioSetDirection(19, Direction.OUTPUT);
    std.debug.print("PWM channel exported.\n", .{});
    try hal.pwmSetFrequency(MOTOR_CHAN_A, 20000);
    std.debug.print("PWM frequency set to 20 kHz.\n", .{});
    try hal.pwmSetFrequency(MOTOR_CHAN_B, 20000);
    // Set PWM duty cycle to 50% (half of the period)
    // 20 kHz period is 50us
    //const duty_cycle = 25000; // 50% duty cycle is 25us
    const duty_cycle = 0; // 50% duty cycle is 25us
    try hal.pwmSetDutyCycle(MOTOR_CHAN_A, duty_cycle);
    try hal.pwmSetDutyCycle(MOTOR_CHAN_B, duty_cycle);
    try hal.pwmEnable(MOTOR_CHAN_A);
    try hal.pwmEnable(MOTOR_CHAN_B);
}

pub fn initializeIMU() !void {
    bmi160.init(hal) catch {
        std.debug.print("BMI160 init failed.\n", .{});
        return error.BMI160_Initialize_Failed;
    };
}

pub fn cleanup() void {
    bmi160.close();
    hal.close();
}

fn clearScreen() void {
    std.debug.print("clearScreen", .{});
}

pub fn readSensors() !void {
    lmuData = try bmi160.readSensorDataNormalized();
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

// comp-time struct conversion (requires matching fields)
fn convertStruct(To: type, from: anytype) To {
    var result: To = undefined;
    inline for (@typeInfo(To).@"struct".fields) |field| {
        @field(result, field.name) = @field(from, field.name);
    }
    return result;
}

pub fn dumpSensors() void {
    std.debug.print("\x1b[3AAccel.X: {d: >7.4}g, Gyro.X: {d: >7.4}°/s\n", .{ lmuData.accel.x, lmuData.gyro.x });
    std.debug.print("Accel.Y: {d: >7.4}g, Gyro.Y: {d: >7.4}°/s\n", .{ lmuData.accel.y, lmuData.gyro.y });
    std.debug.print("Accel.Z: {d: >7.4}g, Gyro.Z: {d: >7.4}°/s   Tilt: {d: >6.2}°\n", .{ lmuData.accel.z, lmuData.gyro.z, tilt_angle_yz() });
}

pub fn driveMotor(left_right: bool, velocity: f32) bool {
    const channel: u8 = if (left_right) MOTOR_CHAN_A else MOTOR_CHAN_B;
    const direction: bool = if (left_right) (velocity >= 0) else !(velocity >= 0);
    hal.gpioWrite(direction_gpio[channel], if (direction) 0 else 1) catch |err| {
        std.debug.print("GPIO write failed: {}\n", .{err});
        return false;
    };
    const power = if (velocity < 0.0) -velocity else velocity;
    hal.pwmSetDutyCycle(channel, power) catch |err| {
        std.debug.print("PWM set duty cycle failed: {}\n", .{err});
        return false;
    };
    return true;
}
