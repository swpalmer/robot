// LSM9DS1 registers and configuration
//
const std = @import("std");
const testing = std.testing;

const c = @cImport({
    @cInclude("stdlib.h");
    @cInclude("pigpio.h");
});

// IMU
// LSM9DS1 Gyroscope/Accelerometer, Magnetoscope //
///////////////////////////////////////////////////
const ACCEL_GYRO_ADDR: c_uint = 0x6A;
const MAG_ADDR: c_uint = 0x1C;

const LSM9DS1_REG_CTRL_REG4: c_uint = 0x1e;
const LSM9DS1_REG_CTRL_REG1_G: c_uint = 0x10;
const LSM9DS1_REG_CTRL_REG5_XL: c_uint = 0x1f;
const LSM9DS1_REG_CTRL_REG6_XL: c_uint = 0x20;
const LSM9DS1_REG_CTRL_REG8: c_uint = 0x22;
const LSM9DS1_REG_CTRL_REG1_M: c_uint = 0x20;
const LSM9DS1_REG_CTRL_REG2_M: c_uint = 0x21;
const LSM9DS1_REG_CTRL_REG3_M: c_uint = 0x22;
const LSM9DS1_REG_CTRL_REG4_M: c_uint = 0x23;
const LSM9DS1_REG_CTRL_REG5_M: c_uint = 0x24;
// outputs
const LSM9DS1_REG_OUT_X_L_G: c_uint = 0x18;
const LSM9DS1_REG_OUT_X_H_G: c_uint = 0x19;
const LSM9DS1_REG_OUT_Y_L_G: c_uint = 0x1a;
const LSM9DS1_REG_OUT_Y_H_G: c_uint = 0x1b;
const LSM9DS1_REG_OUT_Z_L_G: c_uint = 0x1c;
const LSM9DS1_REG_OUT_Z_H_G: c_uint = 0x1d;
const LSM9DS1_REG_OUT_X_L_XL: c_uint = 0x28;
const LSM9DS1_REG_OUT_X_H_XL: c_uint = 0x29;
const LSM9DS1_REG_OUT_Y_L_XL: c_uint = 0x2a;
const LSM9DS1_REG_OUT_Y_H_XL: c_uint = 0x2b;
const LSM9DS1_REG_OUT_Z_L_XL: c_uint = 0x2c;
const LSM9DS1_REG_OUT_Z_H_XL: c_uint = 0x2d;

// Accelerometer/Gyroscope register values
const LSM9DS1_CTRL_REG4_CONFIG: c_uint = 0x38; // Zen_G | Yen_g | Xen_G
// OLD const LSM9DS1_CTRL_REG1_G_CONFIG: c_uint = 0x82; // ODR = 238 Hz, BW = 50 Hz
const LSM9DS1_CTRL_REG1_G_CONFIG: c_uint = 0x20; // ODR = 119 Hz, BW = 17 Hz, FS +/245 dps (8.75 mdps/LSB)
const LSM9DS1_CTRL_REG5_XL_CONFIG: c_uint = 0x38; // Zen_XL | Yen_XL | Xen_XL
const LSM9DS1_CTRL_REG6_XL_CONFIG: c_uint = 0x93; //ODR = 238 Hz, FS_XL = +/- 4g, BW = 105 Hz
const LSM9DS1_CTRL_REG8_CONFIG: c_uint = 0x44; // BDU = 1, IF_ADDR_INC = 1
// Magnetometer register values
const LSM9DS1_CTRL_REG1_M_CONFIG: c_uint = 0xd0; // Temp comp enabled, OpMode High perf, ODR 10Hz
const LSM9DS1_CTRL_REG2_M_CONFIG: c_uint = 0x00; // Full scale +/- 3 gauss
const LSM9DS1_CTRL_REG3_M_CONFIG: c_uint = 0x00; // I2C enabled
const LSM9DS1_CTRL_REG4_M_CONFIG: c_uint = 0x08; // Z-axis op mode = high perf, LSb at lower addr
const LSM9DS1_CTRL_REG5_M_CONFIG: c_uint = 0x40; // Block Data Update

var i2cGyroHandle: c_uint = 0;
var i2cMagHandle: c_uint = 0;

const Raw3 = struct { x: i16, y: i16, z: i16 };
const Vec3 = struct { x: f32, y: f32, z: f32 };
var accel_raw = Raw3{ .x = 0, .y = 0, .z = 0 };
var accel = Vec3{ .x = 0.0, .y = 0.0, .z = 0.0 };

var gyro_raw = Raw3{ .x = 0, .y = 0, .z = 0 };
var gyro = Vec3{ .x = 0, .y = 0, .z = 0 };

var mag_x: i16 = 0;
var mag_y: i16 = 0;
var mag_z: i16 = 0;

var temp: u16 = 0;

pub fn initialize() bool {
    const pigpioVer = c.gpioInitialise();
    if (pigpioVer < 0) {
        std.debug.print("gpioInitialise() failed: {}\n", .{pigpioVer});
        return false;
    }
    _ = c.atexit(&cleanup);
    std.debug.print("PIGPIO Version: {}\n", .{pigpioVer});
    std.debug.print("Hardware rev: {}\n", .{c.gpioHardwareRevision()});
    std.debug.print("GPIO Version: {}\n", .{c.gpioVersion()});
    std.debug.print("cfg = {}\n", .{c.gpioCfgGetInternals()});

    var handle = c.i2cOpen(1, ACCEL_GYRO_ADDR, 0);
    if (handle < 0) {
        std.debug.print("Failed to open I2C for gyroscope: {}\n", .{handle});
        return false;
    }
    i2cGyroHandle = @bitCast(handle);
    handle = c.i2cOpen(1, MAG_ADDR, 0);
    if (handle < 0) {
        std.debug.print("Failed to open I2C for magnetometer: {}\n", .{i2cMagHandle});
        return false;
    }
    i2cMagHandle = @bitCast(handle);

    // Configure Accelerometer and Gyroscope
    _ = c.i2cWriteByteData(i2cGyroHandle, LSM9DS1_REG_CTRL_REG4, LSM9DS1_CTRL_REG4_CONFIG);
    sleepMillis(1);
    _ = c.i2cWriteByteData(i2cGyroHandle, LSM9DS1_REG_CTRL_REG1_G, LSM9DS1_CTRL_REG1_G_CONFIG);
    sleepMillis(1);
    _ = c.i2cWriteByteData(i2cGyroHandle, LSM9DS1_REG_CTRL_REG5_XL, LSM9DS1_CTRL_REG5_XL_CONFIG);
    sleepMillis(1);
    _ = c.i2cWriteByteData(i2cGyroHandle, LSM9DS1_REG_CTRL_REG6_XL, LSM9DS1_CTRL_REG6_XL_CONFIG);
    sleepMillis(1);
    _ = c.i2cWriteByteData(i2cGyroHandle, LSM9DS1_REG_CTRL_REG8, LSM9DS1_CTRL_REG8_CONFIG);
    sleepMillis(1);
    // Configure Magnetometer
    _ = c.i2cWriteByteData(i2cMagHandle, LSM9DS1_REG_CTRL_REG1_M, LSM9DS1_CTRL_REG1_M_CONFIG);
    sleepMillis(1);
    _ = c.i2cWriteByteData(i2cMagHandle, LSM9DS1_REG_CTRL_REG2_M, LSM9DS1_CTRL_REG2_M_CONFIG);
    sleepMillis(1);
    _ = c.i2cWriteByteData(i2cMagHandle, LSM9DS1_REG_CTRL_REG3_M, LSM9DS1_CTRL_REG3_M_CONFIG);
    sleepMillis(1);
    _ = c.i2cWriteByteData(i2cMagHandle, LSM9DS1_REG_CTRL_REG4_M, LSM9DS1_CTRL_REG4_M_CONFIG);
    sleepMillis(1);
    _ = c.i2cWriteByteData(i2cMagHandle, LSM9DS1_REG_CTRL_REG5_M, LSM9DS1_CTRL_REG5_M_CONFIG);
    sleepMillis(1);

    return true;
}

export fn cleanup() void {
    std.debug.print("Exiting...\n", .{});
    const hGyro: c_int = @bitCast(i2cGyroHandle);
    const hMag: c_int = @bitCast(i2cMagHandle);
    if (hGyro >= 0) {
        _ = c.i2cClose(i2cGyroHandle);
    }
    if (hMag >= 0) {
        _ = c.i2cClose(i2cMagHandle);
    }
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

pub fn readSensors() void {
    // Accelerometer
    accel_raw.x = @truncate(c.i2cReadWordData(i2cGyroHandle, LSM9DS1_REG_OUT_X_L_XL));
    accel_raw.y = @truncate(c.i2cReadWordData(i2cGyroHandle, LSM9DS1_REG_OUT_Y_L_XL));
    accel_raw.z = @truncate(c.i2cReadWordData(i2cGyroHandle, LSM9DS1_REG_OUT_Z_L_XL));
    // full scale is configured as +/- 4g, so 1g is 1/8192 of the 16-bit value
    accel.x = @as(f32, @floatFromInt(accel_raw.x)) / 8192.0;
    accel.y = @as(f32, @floatFromInt(accel_raw.y)) / 8192.0;
    accel.z = @as(f32, @floatFromInt(accel_raw.z)) / 8192.0;

    // Gyroscope
    gyro_raw.x = @truncate(c.i2cReadWordData(i2cGyroHandle, LSM9DS1_REG_OUT_X_L_G));
    gyro_raw.y = @truncate(c.i2cReadWordData(i2cGyroHandle, LSM9DS1_REG_OUT_Y_L_G));
    gyro_raw.z = @truncate(c.i2cReadWordData(i2cGyroHandle, LSM9DS1_REG_OUT_Z_L_G));
    // to degrees per second (with LSM9DS1 configured as +/-245 dps full scale)
    gyro.x = @as(f32, @floatFromInt(gyro_raw.x)) * 8.75 / 1000.0;
    gyro.y = @as(f32, @floatFromInt(gyro_raw.y)) * 8.75 / 1000.0;
    gyro.z = @as(f32, @floatFromInt(gyro_raw.z)) * 8.75 / 1000.0;

}

pub fn acceleration() Vec3 {
    return accel;
}

pub fn orientation() Vec3 {
    return gyro;
}

pub fn dumpSensors() void {
    std.debug.print("\x1b[2AAccel.X: {:6}, raw: {:6}\n", .{ accel.x, accel.raw_x });
    std.debug.print("Accel.Y: {:6}, raw: {:6}\n", .{ accel.y, accel.raw_y });
}


pub fn sleepMillis(millis: u32) void {
    std.time.sleep(@as(u64, millis) * 1_000_000);
}

