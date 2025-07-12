// BMI160 IMU

const std = @import("std");
const types = @import("types");
const halMod = @import("hal");
const HAL = halMod.HAL;
const c = @cImport({
    @cInclude("pigpio.h");
});

// BMI160 I2C address (068 or 069 depending on AD0 pin)
pub const BMI160_ADDR: c_uint = 0x68;

// BMI160 register addresses
pub const REG_CHIP_ID = 0x00;
pub const REG_PMU_STATUS = 0x03;
pub const REG_DATA = 0x0C; // Gyro X LSB starts here
pub const REG_ACC_CONF = 0x40;
pub const REG_ACC_RANGE = 0x41;
pub const REG_GYR_CONF = 0x42;
pub const REG_GYR_RANGE = 0x43;
pub const REG_CMD = 0x7E;

pub const CHIP_ID: u8 = 0xD1;
pub const ACCEL_NORMAL_MODE: u8 = 0x11;
pub const GYRO_NORMAL_MODE: u8 = 0x15;

pub const ACCEL_RANGE_CONFIG = 0x03; // ±2g
pub const GYRO_RANGE_CONFIG = 0x02; // ±500°/s
const ACCEL_RANGE = 2.0;
const GYRO_RANGE = 500.0;

var bmi160I2C: ?std.fs.File = null;

var pitch: f32 = 0;
var roll: f32 = 0;
var yaw: f32 = 0;

const RawSensorData = packed struct {
    gyro_x: i16,
    gyro_y: i16,
    gyro_z: i16,
    accel_x: i16,
    accel_y: i16,
    accel_z: i16,
};

pub const NormalizedSensorData = types.NormalizedSensorData;

const stdout = std.io.getStdOut().writer();
var i2c: HAL = undefined;

pub fn init(hal: HAL) !void {
    i2c = hal; // TODO: migrate to using the HAL

    bmi160I2C = i2c.i2cOpen(1, BMI160_ADDR) catch |err| {
        std.debug.print("Initialize HAL library before calling bmi160.init() :{}\n", .{err});
        return error.I2COpenFailed;
    };

    const handle = &bmi160I2C.?;

    // Check chip ID, should be 0xD1
    const chip_id: u8 = try i2c.i2cReadByte(handle, REG_CHIP_ID);

    try stdout.print("Chip ID: {x}\n", .{chip_id});

    if (chip_id != CHIP_ID) {
        std.debug.print("BMI 160, unexpected chip ID.  Wrong address??\n", .{});
        return error.InvalidChipID;
    }

    // Send reset command
    try i2c.i2cWriteByte(handle, REG_CMD, 0xB6);
    std.time.sleep(std.time.ns_per_ms * 100); // wait 100ms

    // Set accelerometer config: ODR 100Hz, normal mode
    try i2c.i2cWriteByte(handle, REG_ACC_CONF, 0x2a); // 400 Hz, normal mode
    try i2c.i2cWriteByte(handle, REG_ACC_RANGE, ACCEL_RANGE_CONFIG); // ±2g

    // Set gyro config: ODR 100Hz, normal mode
    try i2c.i2cWriteByte(handle, REG_GYR_CONF, 0x2a); // 400 Hz, normal mode
    //try i2c.i2cWriteByte(handle, REG_GYR_RANGE, 0x00); // ±2000°/s
    //try i2c.i2cWriteByte(handle, REG_GYR_RANGE, 0x01); // ±1000°/s
    try i2c.i2cWriteByte(handle, REG_GYR_RANGE, GYRO_RANGE_CONFIG); // ±500°/s
    //try i2c.i2cWriteByte(handle, REG_GYR_RANGE, 0x03); // ±250°/s
    //try i2c.i2cWriteByte(handle, REG_GYR_RANGE, 0x04); // ±125°/s

    std.time.sleep(std.time.ns_per_ms * 100); // wait for mode to settle

    // Enable Gyroscope
    try i2c.i2cWriteByte(handle, REG_CMD, GYRO_NORMAL_MODE);
    std.time.sleep(20 * std.time.ns_per_ms); // Wait at least 10ms for it to enter normal mode (sometimes can be up to 80ms)

    const GYRO_MODE_NORMAL: u8 = 0x01;
    var gyro_mode: u8 = 0;
    while (gyro_mode != GYRO_MODE_NORMAL) {
        std.time.sleep(10 * std.time.ns_per_ms); // Wait at least 10ms for it to enter normal mode (sometimes can be up to 80ms)
        const pmu_status = try i2c.i2cReadByte(handle, REG_PMU_STATUS);
        gyro_mode = @intCast((pmu_status >> 2) & 0b11);
        std.debug.print("pmu_status = {b}, gyro_mode = {b:2}\n", .{ pmu_status, gyro_mode });
    }

    // Enable Accelerometer
    try i2c.i2cWriteByte(handle, REG_CMD, ACCEL_NORMAL_MODE);

    // Let's verify that the accelerometer is operational
    const ACCEL_MODE_NORMAL: u8 = 0x01;
    var accel_mode: u8 = 0;
    while (accel_mode != ACCEL_MODE_NORMAL) {
        std.time.sleep(10 * std.time.ns_per_ms); // Wait 10ms
        const pmu_status = try i2c.i2cReadByte(handle, REG_PMU_STATUS);
        // Bits 4–5: Accel mode (00 = suspend, 01 = normal, 10 = low-power)
        accel_mode = @intCast((pmu_status >> 4) & 0b11);
        std.debug.print("pmu_status = {b}, accel_mode = {b:2}\n", .{ pmu_status, accel_mode });
    }
}

pub fn test_loop() !void {
    while (true) {
        const data = try readSensorDataNormalized(bmi160I2C.?);
        try stdout.print("Accel X:{d}  Y: {d}  Z:{d}\n Gyro X:{d}  Y:{d}  Z:{d}\n", .{ data.accel.x, data.accel.y, data.accel.z, data.gyro.x, data.gyro.y, data.gyro.z });

        std.time.sleep(std.time.ns_per_ms * 10); // 100 Hz loop
    }
}

pub fn close() void {
    if (bmi160I2C) |file| {
        i2c.i2cClose(&file) catch |err| {
            std.debug.print("Error closing I2C device: {}\n", .{err});
        };
    }
    bmi160I2C = null;
}

fn read_i16(lo: u8, hi: u8) i16 {
    return @bitCast((@as(u16, hi) << 8) | lo);
}

fn normalizeAccel(raw: i16) f32 {
    const scale: f32 = ACCEL_RANGE / 32768.0; // ±2g
    return @as(f32, @floatFromInt(raw)) * scale;
}

fn normalizeGyro(raw: i16) f32 {
    const scale: f32 = GYRO_RANGE / 32768.0; // ±500°/s
    return @as(f32, @floatFromInt(raw)) * scale;
}

var tick: u64 = 0;

pub fn readSensorDataNormalized() !NormalizedSensorData {
    var buffer: [12]u8 = undefined;

    try i2c.i2cRead(&bmi160I2C.?, REG_DATA, &buffer);

    const raw = @as(*align(1) const RawSensorData, @ptrCast(&buffer)).*;

    //std.debug.print("\x1b[4A{}\nAccel.x: {:>6},  Gyro.x: {:>6}\nAccel.y: {:>6},  Gyro.y: {:>6}\nAccel.z: {:>6},  Gyro.z: {:>6}\n", .{ tick, raw.accel_x, raw.gyro_x, raw.accel_y, raw.gyro_y, raw.accel_z, raw.gyro_z });
    tick += 1;

    return NormalizedSensorData{
        .gyro = .{
            .x = normalizeGyro(raw.gyro_x),
            .y = normalizeGyro(raw.gyro_y),
            .z = normalizeGyro(raw.gyro_z),
        },
        .accel = .{
            .x = normalizeAccel(raw.accel_x),
            .y = normalizeAccel(raw.accel_y),
            .z = normalizeAccel(raw.accel_z),
        },
    };
}
