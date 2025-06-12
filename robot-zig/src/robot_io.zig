pub extern var frontPlane: [8][8]u8;
pub extern var topPlane: [8][8]u8;
pub extern var sidePlane: [8][8]u8;
pub const struct_BotControl = extern struct {
    i2cGyroHandle: c_int = @import("std").mem.zeroes(c_int),
    i2cMagHandle: c_int = @import("std").mem.zeroes(c_int),
    raw_accel_X: i16 = @import("std").mem.zeroes(i16),
    raw_accel_Y: i16 = @import("std").mem.zeroes(i16),
    raw_accel_Z: i16 = @import("std").mem.zeroes(i16),
    raw_gyro_X: i16 = @import("std").mem.zeroes(i16),
    raw_gyro_Y: i16 = @import("std").mem.zeroes(i16),
    raw_gyro_Z: i16 = @import("std").mem.zeroes(i16),
    raw_mag_X: i16 = @import("std").mem.zeroes(i16),
    raw_mag_Y: i16 = @import("std").mem.zeroes(i16),
    raw_mag_Z: i16 = @import("std").mem.zeroes(i16),
    min_accel_X: i16 = @import("std").mem.zeroes(i16),
    min_accel_Y: i16 = @import("std").mem.zeroes(i16),
    min_accel_Z: i16 = @import("std").mem.zeroes(i16),
    min_gyro_X: i16 = @import("std").mem.zeroes(i16),
    min_gyro_Y: i16 = @import("std").mem.zeroes(i16),
    min_gyro_Z: i16 = @import("std").mem.zeroes(i16),
    min_mag_X: i16 = @import("std").mem.zeroes(i16),
    min_mag_Y: i16 = @import("std").mem.zeroes(i16),
    min_mag_Z: i16 = @import("std").mem.zeroes(i16),
    max_accel_X: i16 = @import("std").mem.zeroes(i16),
    max_accel_Y: i16 = @import("std").mem.zeroes(i16),
    max_accel_Z: i16 = @import("std").mem.zeroes(i16),
    max_gyro_X: i16 = @import("std").mem.zeroes(i16),
    max_gyro_Y: i16 = @import("std").mem.zeroes(i16),
    max_gyro_Z: i16 = @import("std").mem.zeroes(i16),
    max_mag_X: i16 = @import("std").mem.zeroes(i16),
    max_mag_Y: i16 = @import("std").mem.zeroes(i16),
    max_mag_Z: i16 = @import("std").mem.zeroes(i16),
    raw_temp: i16 = @import("std").mem.zeroes(i16),
    gyro_X: f32 = @import("std").mem.zeroes(f32),
    gyro_Y: f32 = @import("std").mem.zeroes(f32),
    gyro_Z: f32 = @import("std").mem.zeroes(f32),
    accel_X: f32 = @import("std").mem.zeroes(f32),
    accel_Y: f32 = @import("std").mem.zeroes(f32),
    accel_Z: f32 = @import("std").mem.zeroes(f32),
    mag_X: f32 = @import("std").mem.zeroes(f32),
    mag_Y: f32 = @import("std").mem.zeroes(f32),
    mag_Z: f32 = @import("std").mem.zeroes(f32),
};
pub export fn initContext() bool {
    _ = memset(@as(?*anyopaque, @ptrCast(&botCtrls)), @as(c_int, 0), @sizeOf(struct_BotControl));
    botCtrls.i2cGyroHandle = -@as(c_int, 1);
    botCtrls.i2cMagHandle = -@as(c_int, 1);
    _ = printf("Hardware rev: %x\n", gpioHardwareRevision());
    _ = printf("GPIO Version: %d\n", gpioVersion());
    var pigpioVer: c_int = gpioInitialise();
    _ = &pigpioVer;
    if (pigpioVer < @as(c_int, 0)) {
        _ = printf("gpioInitialize failed: %0x\n", pigpioVer);
        return @as(c_int, 0) != 0;
    }
    _ = printf("PIGPIO Version: %d\n", pigpioVer);
    var cfg: c_int = @as(c_int, @bitCast(gpioCfgGetInternals()));
    _ = &cfg;
    _ = printf("cfg = %0x\n", cfg);
    var gpio: c_int = 24;
    _ = &gpio;
    var ret: c_int = gpioSetMode(@as(c_uint, @bitCast(gpio)), @as(c_uint, @bitCast(@as(c_int, 0))));
    _ = &ret;
    while (true) {
        switch (ret) {
            @as(c_int, -3) => {
                _ = printf("BAD_GPIO: Failed to set mode for %d to %d\n", gpio, @as(c_int, 0));
                return @as(c_int, 0) != 0;
            },
            @as(c_int, -4) => {
                _ = printf("BAD_MODE: Failed to set mode for %d to %d\n", gpio, @as(c_int, 0));
                return @as(c_int, 0) != 0;
            },
            else => {},
        }
        break;
    }
    ret = gpioSetMode(@as(c_uint, @bitCast(@as(c_int, 12))), @as(c_uint, @bitCast(@as(c_int, 4))));
    if (ret != @as(c_int, 0)) return @as(c_int, 0) != 0;
    ret = gpioSetMode(@as(c_uint, @bitCast(@as(c_int, 13))), @as(c_uint, @bitCast(@as(c_int, 4))));
    if (ret != @as(c_int, 0)) return @as(c_int, 0) != 0;
    ret = gpioSetMode(@as(c_uint, @bitCast(@as(c_int, 18))), @as(c_uint, @bitCast(@as(c_int, 4))));
    if (ret != @as(c_int, 0)) return @as(c_int, 0) != 0;
    ret = gpioSetMode(@as(c_uint, @bitCast(@as(c_int, 19))), @as(c_uint, @bitCast(@as(c_int, 4))));
    if (ret != @as(c_int, 0)) return @as(c_int, 0) != 0;
    ret = gpioSetMode(@as(c_uint, @bitCast(direction_gpio[@as(c_uint, @intCast(@as(c_int, 0)))])), @as(c_uint, @bitCast(@as(c_int, 1))));
    ret = gpioSetMode(@as(c_uint, @bitCast(direction_gpio[@as(c_uint, @intCast(@as(c_int, 1)))])), @as(c_uint, @bitCast(@as(c_int, 1))));
    ret = gpioSetPullUpDown(@as(c_uint, @bitCast(direction_gpio[@as(c_uint, @intCast(@as(c_int, 0)))])), @as(c_uint, @bitCast(@as(c_int, 1))));
    _ = gpioSetPullUpDown(@as(c_uint, @bitCast(direction_gpio[@as(c_uint, @intCast(@as(c_int, 1)))])), @as(c_uint, @bitCast(@as(c_int, 1))));
    if (ret != @as(c_int, 0)) {
        _ = printf("Failed to set pull-down for direction control\n");
        return @as(c_int, 0) != 0;
    }
    var dutyCycle: c_int = gpioGetPWMdutycycle(@as(c_uint, @bitCast(@as(c_int, 12))));
    _ = &dutyCycle;
    _ = printf("PWM dutycycle for GPIO 12 (PWM0) is %d\n", dutyCycle);
    var real_range: c_int = gpioGetPWMrealRange(@as(c_uint, @bitCast(@as(c_int, 12))));
    _ = &real_range;
    _ = printf("Real PWM range for GPIO 12 is %d\n", real_range);
    botCtrls.i2cGyroHandle = i2cOpen(@as(c_uint, @bitCast(@as(c_int, 1))), @as(c_uint, @bitCast(@as(c_int, 106))), @as(c_uint, @bitCast(@as(c_int, 0))));
    if (botCtrls.i2cGyroHandle < @as(c_int, 0)) {
        _ = printf("Failed to open I2C for accelerometer and gyro: %d\n", botCtrls.i2cGyroHandle);
        return @as(c_int, 0) != 0;
    }
    botCtrls.i2cMagHandle = i2cOpen(@as(c_uint, @bitCast(@as(c_int, 1))), @as(c_uint, @bitCast(@as(c_int, 28))), @as(c_uint, @bitCast(@as(c_int, 0))));
    if (botCtrls.i2cMagHandle < @as(c_int, 0)) {
        _ = printf("Failed to open I2C for magnetometer: %d\n", botCtrls.i2cMagHandle);
        return @as(c_int, 0) != 0;
    }
    _ = i2cWriteByteData(@as(c_uint, @bitCast(botCtrls.i2cGyroHandle)), @as(c_uint, @bitCast(@as(c_int, 30))), @as(c_uint, @bitCast(@as(c_int, 56))));
    sleepmillis(@as(c_int, 1));
    _ = i2cWriteByteData(@as(c_uint, @bitCast(botCtrls.i2cGyroHandle)), @as(c_uint, @bitCast(@as(c_int, 16))), @as(c_uint, @bitCast(@as(c_int, 130))));
    sleepmillis(@as(c_int, 1));
    _ = i2cWriteByteData(@as(c_uint, @bitCast(botCtrls.i2cGyroHandle)), @as(c_uint, @bitCast(@as(c_int, 31))), @as(c_uint, @bitCast(@as(c_int, 56))));
    sleepmillis(@as(c_int, 1));
    _ = i2cWriteByteData(@as(c_uint, @bitCast(botCtrls.i2cGyroHandle)), @as(c_uint, @bitCast(@as(c_int, 32))), @as(c_uint, @bitCast(@as(c_int, 147))));
    sleepmillis(@as(c_int, 1));
    _ = i2cWriteByteData(@as(c_uint, @bitCast(botCtrls.i2cGyroHandle)), @as(c_uint, @bitCast(@as(c_int, 34))), @as(c_uint, @bitCast(@as(c_int, 68))));
    sleepmillis(@as(c_int, 1));
    _ = i2cWriteByteData(@as(c_uint, @bitCast(botCtrls.i2cMagHandle)), @as(c_uint, @bitCast(@as(c_int, 32))), @as(c_uint, @bitCast(@as(c_int, 208))));
    sleepmillis(@as(c_int, 1));
    _ = i2cWriteByteData(@as(c_uint, @bitCast(botCtrls.i2cMagHandle)), @as(c_uint, @bitCast(@as(c_int, 33))), @as(c_uint, @bitCast(@as(c_int, 0))));
    sleepmillis(@as(c_int, 1));
    _ = i2cWriteByteData(@as(c_uint, @bitCast(botCtrls.i2cMagHandle)), @as(c_uint, @bitCast(@as(c_int, 34))), @as(c_uint, @bitCast(@as(c_int, 0))));
    sleepmillis(@as(c_int, 1));
    _ = i2cWriteByteData(@as(c_uint, @bitCast(botCtrls.i2cMagHandle)), @as(c_uint, @bitCast(@as(c_int, 35))), @as(c_uint, @bitCast(@as(c_int, 8))));
    sleepmillis(@as(c_int, 1));
    _ = i2cWriteByteData(@as(c_uint, @bitCast(botCtrls.i2cMagHandle)), @as(c_uint, @bitCast(@as(c_int, 36))), @as(c_uint, @bitCast(@as(c_int, 64))));
    sleepmillis(@as(c_int, 1));
    clearScreen(@as([*c][8]u8, @ptrCast(@alignCast(&frontPlane))));
    clearScreen(@as([*c][8]u8, @ptrCast(@alignCast(&topPlane))));
    clearScreen(@as([*c][8]u8, @ptrCast(@alignCast(&sidePlane))));
    return @as(c_int, 1) != 0;
}
pub export fn closeContext() void {
    if (botCtrls.i2cGyroHandle >= @as(c_int, 0)) {
        _ = i2cClose(@as(c_uint, @bitCast(botCtrls.i2cGyroHandle)));
    }
    if (botCtrls.i2cMagHandle >= @as(c_int, 0)) {
        _ = i2cClose(@as(c_uint, @bitCast(botCtrls.i2cMagHandle)));
    }
    gpioTerminate();
}
pub export fn readAccel() void {
    botCtrls.raw_accel_X = @as(i16, @bitCast(@as(c_short, @truncate(i2cReadWordData(@as(c_uint, @bitCast(botCtrls.i2cGyroHandle)), @as(c_uint, @bitCast(@as(c_int, 40))))))));
    botCtrls.raw_accel_Y = @as(i16, @bitCast(@as(c_short, @truncate(i2cReadWordData(@as(c_uint, @bitCast(botCtrls.i2cGyroHandle)), @as(c_uint, @bitCast(@as(c_int, 42))))))));
    botCtrls.raw_accel_Z = @as(i16, @bitCast(@as(c_short, @truncate(i2cReadWordData(@as(c_uint, @bitCast(botCtrls.i2cGyroHandle)), @as(c_uint, @bitCast(@as(c_int, 44))))))));
    if (@as(c_int, @bitCast(@as(c_int, botCtrls.raw_accel_X))) < @as(c_int, @bitCast(@as(c_int, botCtrls.min_accel_X)))) {
        botCtrls.min_accel_X = botCtrls.raw_accel_X;
    } else if (@as(c_int, @bitCast(@as(c_int, botCtrls.raw_accel_X))) > @as(c_int, @bitCast(@as(c_int, botCtrls.max_accel_X)))) {
        botCtrls.max_accel_X = botCtrls.raw_accel_X;
    }
    botCtrls.accel_X = if (@as(f64, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.raw_accel_X))))) < 0.0) -(@as(f32, @floatFromInt(botCtrls.raw_accel_X)) / @as(f32, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.min_accel_X)))))) else @as(f32, @floatFromInt(botCtrls.raw_accel_X)) / @as(f32, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.max_accel_X)))));
    if (@as(c_int, @bitCast(@as(c_int, botCtrls.raw_accel_Y))) < @as(c_int, @bitCast(@as(c_int, botCtrls.min_accel_Y)))) {
        botCtrls.min_accel_Y = botCtrls.raw_accel_Y;
    } else if (@as(c_int, @bitCast(@as(c_int, botCtrls.raw_accel_Y))) > @as(c_int, @bitCast(@as(c_int, botCtrls.max_accel_Y)))) {
        botCtrls.max_accel_Y = botCtrls.raw_accel_Y;
    }
    botCtrls.accel_Y = if (@as(f64, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.raw_accel_Y))))) < 0.0) -(@as(f32, @floatFromInt(botCtrls.raw_accel_Y)) / @as(f32, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.min_accel_Y)))))) else @as(f32, @floatFromInt(botCtrls.raw_accel_Y)) / @as(f32, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.max_accel_Y)))));
    if (@as(c_int, @bitCast(@as(c_int, botCtrls.raw_accel_Z))) < @as(c_int, @bitCast(@as(c_int, botCtrls.min_accel_Z)))) {
        botCtrls.min_accel_Z = botCtrls.raw_accel_Z;
    } else if (@as(c_int, @bitCast(@as(c_int, botCtrls.raw_accel_Z))) > @as(c_int, @bitCast(@as(c_int, botCtrls.max_accel_Z)))) {
        botCtrls.max_accel_Z = botCtrls.raw_accel_Z;
    }
    botCtrls.accel_Z = if (@as(f64, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.raw_accel_Z))))) < 0.0) -(@as(f32, @floatFromInt(botCtrls.raw_accel_Z)) / @as(f32, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.min_accel_Z)))))) else @as(f32, @floatFromInt(botCtrls.raw_accel_Z)) / @as(f32, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.max_accel_Z)))));
}
pub export fn readGyro() void {
    botCtrls.raw_gyro_X = @as(i16, @bitCast(@as(c_short, @truncate(i2cReadWordData(@as(c_uint, @bitCast(botCtrls.i2cGyroHandle)), @as(c_uint, @bitCast(@as(c_int, 24))))))));
    botCtrls.raw_gyro_Y = @as(i16, @bitCast(@as(c_short, @truncate(i2cReadWordData(@as(c_uint, @bitCast(botCtrls.i2cGyroHandle)), @as(c_uint, @bitCast(@as(c_int, 26))))))));
    botCtrls.raw_gyro_Z = @as(i16, @bitCast(@as(c_short, @truncate(i2cReadWordData(@as(c_uint, @bitCast(botCtrls.i2cGyroHandle)), @as(c_uint, @bitCast(@as(c_int, 28))))))));
    if (@as(c_int, @bitCast(@as(c_int, botCtrls.raw_gyro_X))) < @as(c_int, @bitCast(@as(c_int, botCtrls.min_gyro_X)))) {
        botCtrls.min_gyro_X = botCtrls.raw_gyro_X;
    } else if (@as(c_int, @bitCast(@as(c_int, botCtrls.raw_gyro_X))) > @as(c_int, @bitCast(@as(c_int, botCtrls.max_gyro_X)))) {
        botCtrls.max_gyro_X = botCtrls.raw_gyro_X;
    }
    botCtrls.gyro_X = if (@as(f64, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.raw_gyro_X))))) < 0.0) -(@as(f32, @floatFromInt(botCtrls.raw_gyro_X)) / @as(f32, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.min_gyro_X)))))) else @as(f32, @floatFromInt(botCtrls.raw_gyro_X)) / @as(f32, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.max_gyro_X)))));
    if (@as(c_int, @bitCast(@as(c_int, botCtrls.raw_gyro_Y))) < @as(c_int, @bitCast(@as(c_int, botCtrls.min_gyro_Y)))) {
        botCtrls.min_gyro_Y = botCtrls.raw_gyro_Y;
    } else if (@as(c_int, @bitCast(@as(c_int, botCtrls.raw_gyro_Y))) > @as(c_int, @bitCast(@as(c_int, botCtrls.max_gyro_Y)))) {
        botCtrls.max_gyro_Y = botCtrls.raw_gyro_Y;
    }
    botCtrls.gyro_Y = if (@as(f64, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.raw_gyro_Y))))) < 0.0) -(@as(f32, @floatFromInt(botCtrls.raw_gyro_Y)) / @as(f32, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.min_gyro_Y)))))) else @as(f32, @floatFromInt(botCtrls.raw_gyro_Y)) / @as(f32, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.max_gyro_Y)))));
    if (@as(c_int, @bitCast(@as(c_int, botCtrls.raw_gyro_Z))) < @as(c_int, @bitCast(@as(c_int, botCtrls.min_gyro_Z)))) {
        botCtrls.min_gyro_Z = botCtrls.raw_gyro_Z;
    } else if (@as(c_int, @bitCast(@as(c_int, botCtrls.raw_gyro_Z))) > @as(c_int, @bitCast(@as(c_int, botCtrls.max_gyro_Z)))) {
        botCtrls.max_gyro_Z = botCtrls.raw_gyro_Z;
    }
    botCtrls.gyro_Z = if (@as(f64, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.raw_gyro_Z))))) < 0.0) -(@as(f32, @floatFromInt(botCtrls.raw_gyro_Z)) / @as(f32, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.min_gyro_Z)))))) else @as(f32, @floatFromInt(botCtrls.raw_gyro_Z)) / @as(f32, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.max_gyro_Z)))));
}
pub export fn readMag() void {
    botCtrls.raw_mag_X = @as(i16, @bitCast(@as(c_short, @truncate(i2cReadWordData(@as(c_uint, @bitCast(botCtrls.i2cMagHandle)), @as(c_uint, @bitCast(@as(c_int, 40))))))));
    botCtrls.raw_mag_Y = @as(i16, @bitCast(@as(c_short, @truncate(i2cReadWordData(@as(c_uint, @bitCast(botCtrls.i2cMagHandle)), @as(c_uint, @bitCast(@as(c_int, 42))))))));
    botCtrls.raw_mag_Z = @as(i16, @bitCast(@as(c_short, @truncate(i2cReadWordData(@as(c_uint, @bitCast(botCtrls.i2cMagHandle)), @as(c_uint, @bitCast(@as(c_int, 44))))))));
    if (@as(c_int, @bitCast(@as(c_int, botCtrls.raw_mag_X))) < @as(c_int, @bitCast(@as(c_int, botCtrls.min_mag_X)))) {
        botCtrls.min_mag_X = botCtrls.raw_mag_X;
    } else if (@as(c_int, @bitCast(@as(c_int, botCtrls.raw_mag_X))) > @as(c_int, @bitCast(@as(c_int, botCtrls.max_mag_X)))) {
        botCtrls.max_mag_X = botCtrls.raw_mag_X;
    }
    botCtrls.mag_X = if (@as(f64, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.raw_mag_X))))) < 0.0) -(@as(f32, @floatFromInt(botCtrls.raw_mag_X)) / @as(f32, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.min_mag_X)))))) else @as(f32, @floatFromInt(botCtrls.raw_mag_X)) / @as(f32, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.max_mag_X)))));
    if (@as(c_int, @bitCast(@as(c_int, botCtrls.raw_mag_Y))) < @as(c_int, @bitCast(@as(c_int, botCtrls.min_mag_Y)))) {
        botCtrls.min_mag_Y = botCtrls.raw_mag_Y;
    } else if (@as(c_int, @bitCast(@as(c_int, botCtrls.raw_mag_Y))) > @as(c_int, @bitCast(@as(c_int, botCtrls.max_mag_Y)))) {
        botCtrls.max_mag_Y = botCtrls.raw_mag_Y;
    }
    botCtrls.mag_Y = if (@as(f64, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.raw_mag_Y))))) < 0.0) -(@as(f32, @floatFromInt(botCtrls.raw_mag_Y)) / @as(f32, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.min_mag_Y)))))) else @as(f32, @floatFromInt(botCtrls.raw_mag_Y)) / @as(f32, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.max_mag_Y)))));
    if (@as(c_int, @bitCast(@as(c_int, botCtrls.raw_mag_Z))) < @as(c_int, @bitCast(@as(c_int, botCtrls.min_mag_Z)))) {
        botCtrls.min_mag_Z = botCtrls.raw_mag_Z;
    } else if (@as(c_int, @bitCast(@as(c_int, botCtrls.raw_mag_Z))) > @as(c_int, @bitCast(@as(c_int, botCtrls.max_mag_Z)))) {
        botCtrls.max_mag_Z = botCtrls.raw_mag_Z;
    }
    botCtrls.mag_Z = if (@as(f64, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.raw_mag_Z))))) < 0.0) -(@as(f32, @floatFromInt(botCtrls.raw_mag_Z)) / @as(f32, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.min_mag_Z)))))) else @as(f32, @floatFromInt(botCtrls.raw_mag_Z)) / @as(f32, @floatFromInt(@as(c_int, @bitCast(@as(c_int, botCtrls.max_mag_Z)))));
}
pub export fn readMagOffsetX() i16 {
    return @as(i16, @bitCast(@as(c_short, @truncate(i2cReadWordData(@as(c_uint, @bitCast(botCtrls.i2cMagHandle)), @as(c_uint, @bitCast(@as(c_int, 5))))))));
}
pub export fn readMagOffsetY() i16 {
    return @as(i16, @bitCast(@as(c_short, @truncate(i2cReadWordData(@as(c_uint, @bitCast(botCtrls.i2cMagHandle)), @as(c_uint, @bitCast(@as(c_int, 7))))))));
}
pub export fn readMagOffsetZ() i16 {
    return @as(i16, @bitCast(@as(c_short, @truncate(i2cReadWordData(@as(c_uint, @bitCast(botCtrls.i2cMagHandle)), @as(c_uint, @bitCast(@as(c_int, 9))))))));
}
pub export fn readTemp() void {
    botCtrls.raw_temp = @as(i16, @bitCast(@as(c_short, @truncate(i2cReadWordData(@as(c_uint, @bitCast(botCtrls.i2cGyroHandle)), @as(c_uint, @bitCast(@as(c_int, 21))))))));
}
pub export fn readSensors() void {
    readAccel();
    readGyro();
    readMag();
    readTemp();
}
pub export fn hw_pwm(arg_channel: c_uint, arg_power: f64) bool {
    var channel = arg_channel;
    var power = arg_power;
    if ((power > 1.0) or (power < -1.0)) {
        _ = printf("power out of range: %f\n", power);
        return @as(c_int, 0) != 0;
    }
    if (channel > @as(c_uint, @bitCast(@as(c_int, 3)))) {
        _ = printf("channel out of range: %d\n", channel);
        return @as(c_int, 0) != 0;
    }
    var gpio: c_int = 12;
    while (true) {
        switch (channel) {
            @as(c_uint, @bitCast(@as(c_int, 0))) => break,
            @as(c_uint, @bitCast(@as(c_int, 1))) => {
                gpio = 13;
                break;
            },
            @as(c_uint, @bitCast(@as(c_int, 2))) => {
                gpio = 18;
                break;
            },
            @as(c_uint, @bitCast(@as(c_int, 3))) => {
                gpio = 19;
                break;
            },
            else => return @as(c_int, 0) != 0,
        }
        break;
    }
    var duty: c_int = @as(c_int, @intFromFloat(power * @as(f64, @floatFromInt(@as(c_int, 1000000)))));
    var ret: c_int = gpioHardwarePWM(@as(c_uint, @bitCast(gpio)), @as(c_uint, @bitCast(@as(c_int, 20000))), @as(c_uint, @bitCast(duty)));
    while (true) {
        switch (ret) {
            @as(c_int, -3) => {
                _ = printf("gpioHardwarePWM -> PI_BAD_GPIO\n");
                return @as(c_int, 0) != 0;
            },
            @as(c_int, -95) => {
                _ = printf("gpioHardwarePWM -> PI_NOT_HPWM_GPIO\n");
                return @as(c_int, 0) != 0;
            },
            @as(c_int, -97) => {
                _ = printf("gpioHardwarePWM -> PI_BAD_HPWM_DUTY\n");
                return @as(c_int, 0) != 0;
            },
            @as(c_int, -96) => {
                _ = printf("gpioHardwarePWM -> PI_BAD_HPWM_FREQ\n");
                return @as(c_int, 0) != 0;
            },
            @as(c_int, -100) => {
                _ = printf("gpioHardwarePWM -> PI_HPWM_ILLEGAL\n");
                return @as(c_int, 0) != 0;
            },
            else => {},
        }
        break;
    }
    return @as(c_int, 1) != 0;
}
pub export fn driveMotor(arg_leftRight: bool, arg_velocity: f64) bool {
    var leftRight = arg_leftRight;
    var velocity = arg_velocity;
    var channel: c_int = if (@as(c_int, @intFromBool(leftRight)) != 0) @as(c_int, 0) else @as(c_int, 1);
    var direction: bool = velocity >= @as(f64, @floatFromInt(@as(c_int, 0)));
    var motorDir: c_int = channel ^ @as(c_int, @intFromBool(direction));
    _ = gpioWrite(@as(c_uint, @bitCast(direction_gpio[@as(c_uint, @intCast(channel))])), @as(c_uint, @bitCast(motorDir)));
    var power: f64 = if (velocity < 0.0) -velocity else velocity;
    _ = &power;
    return hw_pwm(@as(c_uint, @bitCast(channel)), power);
}
pub extern fn sleepmillis(millis: c_int) void;
pub extern fn seconds(...) c_long;
pub export fn clearScreen(arg_screen: [*c][8]u8) void {
    var screen = arg_screen;
    {
        var y: c_int = 0;
        while (y < @as(c_int, 8)) : (y += 1) {
            {
                var x: c_int = 0;
                while (x < @as(c_int, 8)) : (x += 1) {
                    (blk: {
                        const tmp = x;
                        if (tmp >= 0) break :blk screen + @as(usize, @intCast(tmp)) else break :blk screen - ~@as(usize, @bitCast(@as(isize, @intCast(tmp)) +% -1));
                    }).*[@as(c_uint, @intCast(y))] = '.';
                }
            }
        }
    }
}
pub export fn printScreen(arg_screen: [*c][8]u8) void {
    var screen = arg_screen;
    {
        var y: c_int = 0;
        while (y < @as(c_int, 8)) : (y += 1) {
            {
                var x: c_int = 0;
                while (x < @as(c_int, 8)) : (x += 1) {
                    _ = printf("%c", @as(c_int, @bitCast(@as(c_uint, (blk: {
                        const tmp = x;
                        if (tmp >= 0) break :blk screen + @as(usize, @intCast(tmp)) else break :blk screen - ~@as(usize, @bitCast(@as(isize, @intCast(tmp)) +% -1));
                    }).*[@as(c_uint, @intCast(y))]))));
                }
            }
            _ = printf("\n");
        }
    }
}
pub export fn renderSlope(arg_screen: [*c][8]u8, arg_angle: f32, arg_c: u8) void {
    var screen = arg_screen;
    var angle = arg_angle;
    var c = arg_c;
    var cosA: f32 = @as(f32, @floatCast(cos(@as(f64, @floatCast(angle)))));
    var sinA: f32 = @as(f32, @floatCast(sin(@as(f64, @floatCast(angle)))));
    var sx: c_int = 0;
    var ex: c_int = 7;
    var sy: c_int = toInt(@as(f32, @floatFromInt(@as(c_int, 4))) + (sinA * @as(f32, @floatFromInt(@as(c_int, 4)))));
    var ey: c_int = toInt(@as(f32, @floatFromInt(@as(c_int, 4))) - (sinA * @as(f32, @floatFromInt(@as(c_int, 4)))));
    sy = min(@as(c_int, 7), max(@as(c_int, 0), sy));
    ey = min(@as(c_int, 7), max(@as(c_int, 0), ey));
    renderLine(screen, sx, sy, ex, ey, c);
}
pub export fn printCube() void {
    var topBackEdge: [*c]u8 = @as([*c]u8, @ptrCast(@volatileCast(@constCast("         _______________\n"))));
    _ = &topBackEdge;
    var eightToEdge: [*c]u8 = @as([*c]u8, @ptrCast(@volatileCast(@constCast("        /"))));
    _ = &eightToEdge;
    _ = printf(topBackEdge);
    {
        var y: c_int = 0;
        while (y < @as(c_int, 8)) : (y += 1) {
            _ = printf(eightToEdge);
            eightToEdge += 1;
            {
                var x: c_int = 0;
                while (x < @as(c_int, 8)) : (x += 1) {
                    _ = printf("%c", @as(c_int, @bitCast(@as(c_uint, topPlane[@as(c_uint, @intCast(x))][@as(c_uint, @intCast(y))]))));
                    if (x < @as(c_int, 7)) {
                        _ = printf(" ");
                    }
                }
            }
            _ = printf("/");
            {
                var x: c_int = @as(c_int, 8) - y;
                var r: c_int = 0;
                while (x < @as(c_int, 8)) : (_ = blk: {
                    x += 1;
                    break :blk blk_1: {
                        const ref = &r;
                        const tmp = ref.*;
                        ref.* += 1;
                        break :blk_1 tmp;
                    };
                }) {
                    _ = printf("%c", @as(c_int, @bitCast(@as(c_uint, sidePlane[@as(c_uint, @intCast(x))][@as(c_uint, @intCast(r))]))));
                }
            }
            _ = printf("|\n");
        }
    }
    _ = printf("*---------------*");
    {
        var d: c_int = 0;
        while (d < @as(c_int, 8)) : (d += 1) {
            _ = printf("%c", @as(c_int, @bitCast(@as(c_uint, sidePlane[@as(c_uint, @intCast(d))][@as(c_uint, @intCast(d))]))));
        }
    }
    _ = printf("|\n");
    {
        var y: c_int = 0;
        while (y < @as(c_int, 8)) : (y += 1) {
            _ = printf("|");
            {
                var x: c_int = 0;
                while (x < @as(c_int, 8)) : (x += 1) {
                    _ = printf("%c", @as(c_int, @bitCast(@as(c_uint, frontPlane[@as(c_uint, @intCast(x))][@as(c_uint, @intCast(y))]))));
                    if (x < @as(c_int, 7)) {
                        _ = printf(" ");
                    }
                }
            }
            _ = printf("|");
            {
                var x: c_int = 0;
                var r: c_int = y + @as(c_int, 1);
                while (r < @as(c_int, 8)) : (_ = blk: {
                    x += 1;
                    break :blk blk_1: {
                        const ref = &r;
                        const tmp = ref.*;
                        ref.* += 1;
                        break :blk_1 tmp;
                    };
                }) {
                    _ = printf("%c", @as(c_int, @bitCast(@as(c_uint, sidePlane[@as(c_uint, @intCast(x))][@as(c_uint, @intCast(r))]))));
                }
            }
            _ = printf("/\n");
        }
    }
    _ = printf(" ---------------*\n");
}
pub export fn dumpPins() void {
    {
        var gpio: c_int = 0;
        while (gpio <= @as(c_int, 31)) : (gpio += 1) {
            var mode: c_int = gpioGetMode(@as(c_uint, @bitCast(gpio)));
            if (mode == -@as(c_int, 3)) {
                _ = printf("BAD_GPIO: Failed to get mode for %d\n", gpio);
            } else {
                _ = printf("GPIO-%d mode is %d\n", gpio, mode);
            }
        }
    }
    {
        var gpio: c_int = 0;
        while (gpio <= @as(c_int, 31)) : (gpio += 1) {
            var level: c_int = gpioRead(@as(c_uint, @bitCast(gpio)));
            if (level == -@as(c_int, 3)) {
                _ = printf("BAD_GPIO: Failed to read GPIO-%d\n", gpio);
            } else {
                _ = printf("GPIO-%d : %d\n", gpio, level);
            }
        }
    }
}
pub extern fn min(a: c_int, b: c_int) c_int;
pub extern fn max(a: c_int, b: c_int) c_int;
pub extern fn toInt(f: f32) c_int;
pub export fn renderLine(arg_screen: [*c][8]u8, arg_sx: c_int, arg_sy: c_int, arg_ex: c_int, arg_ey: c_int, arg_color: u8) void {
    var screen = arg_screen;
    var sx = arg_sx;
    var sy = arg_sy;
    var ex = arg_ex;
    var ey = arg_ey;
    var color = arg_color;
    var dx: c_int = ex - sx;
    var dy: c_int = ey - sy;
    if (abs(dy) > abs(dx)) {
        var fdx: f32 = @as(f32, @floatFromInt(dx)) / @as(f32, @floatFromInt(dy));
        {
            var i: c_int = 0;
            while (i <= dy) : (i += 1) {
                var y: c_int = sy + i;
                var x: c_int = toInt(@as(f32, @floatFromInt(sx)) + (@as(f32, @floatFromInt(i)) * fdx));
                (blk: {
                    const tmp = x;
                    if (tmp >= 0) break :blk screen + @as(usize, @intCast(tmp)) else break :blk screen - ~@as(usize, @bitCast(@as(isize, @intCast(tmp)) +% -1));
                }).*[@as(c_uint, @intCast(y))] = color;
            }
        }
    } else {
        var fdy: f32 = @as(f32, @floatFromInt(dy)) / @as(f32, @floatFromInt(dx));
        {
            var i: c_int = 0;
            while (i <= dx) : (i += 1) {
                var x: c_int = sx + i;
                var y: c_int = toInt(@as(f32, @floatFromInt(sy)) + (@as(f32, @floatFromInt(i)) * fdy));
                (blk: {
                    const tmp = x;
                    if (tmp >= 0) break :blk screen + @as(usize, @intCast(tmp)) else break :blk screen - ~@as(usize, @bitCast(@as(isize, @intCast(tmp)) +% -1));
                }).*[@as(c_uint, @intCast(y))] = color;
            }
        }
    }
}
pub export fn printFront() void {
    printScreen(@as([*c][8]u8, @ptrCast(@alignCast(&frontPlane))));
}
pub export var botCtrls: struct_BotControl = @import("std").mem.zeroes(struct_BotControl);
pub export var direction_gpio: [2]c_int = [2]c_int{
    26,
    27,
};

pub const ACCEL_GYRO_ADDR = @as(c_int, 0x6A);
pub const MAG_ADDR = @as(c_int, 0x1c);
pub const LSM9DS1_REG_ACT_THS = @as(c_int, 0x04);
pub const LSM9DS1_REG_ACT_DUR = @as(c_int, 0x05);
pub const LSM9DS1_REG_INT_GEN_CFG_XL = @as(c_int, 0x06);
pub const LSM9DS1_REG_INT_GEN_THS_X_XL = @as(c_int, 0x07);
pub const LSM9DS1_REG_INT_GEN_THS_Y_XL = @as(c_int, 0x08);
pub const LSM9DS1_REG_INT_GEN_THS_Z_XL = @as(c_int, 0x09);
pub const LSM9DS1_REG_INT_GEN_DUR_XL = @as(c_int, 0x0A);
pub const LSM9DS1_REG_REFERENCE_G = @as(c_int, 0x0B);
pub const LSM9DS1_REG_INT1_CTRL = @as(c_int, 0x0C);
pub const LSM9DS1_REG_INT2_CTRL = @as(c_int, 0x0D);
pub const LSM9DS1_REG_WHO_AM_I = @as(c_int, 0x0F);
pub const LSM9DS1_REG_CTRL_REG1_G = @as(c_int, 0x10);
pub const LSM9DS1_REG_CTRL_REG2_G = @as(c_int, 0x11);
pub const LSM9DS1_REG_CTRL_REG3_G = @as(c_int, 0x12);
pub const LSM9DS1_REG_ORIENT_CFG_G = @as(c_int, 0x13);
pub const LSM9DS1_REG_INT_GEN_SRC_G = @as(c_int, 0x14);
pub const LSM9DS1_REG_OUT_TEMP_L = @as(c_int, 0x15);
pub const LSM9DS1_REG_OUT_TEMP_H = @as(c_int, 0x16);
pub const LSM9DS1_REG_STATUS_REG = @as(c_int, 0x17);
pub const LSM9DS1_REG_OUT_X_L_G = @as(c_int, 0x18);
pub const LSM9DS1_REG_OUT_X_H_G = @as(c_int, 0x19);
pub const LSM9DS1_REG_OUT_Y_L_G = @as(c_int, 0x1A);
pub const LSM9DS1_REG_OUT_Y_H_G = @as(c_int, 0x1B);
pub const LSM9DS1_REG_OUT_Z_L_G = @as(c_int, 0x1C);
pub const LSM9DS1_REG_OUT_Z_H_G = @as(c_int, 0x1D);
pub const LSM9DS1_REG_CTRL_REG4 = @as(c_int, 0x1E);
pub const LSM9DS1_REG_CTRL_REG5_XL = @as(c_int, 0x1F);
pub const LSM9DS1_REG_CTRL_REG6_XL = @as(c_int, 0x20);
pub const LSM9DS1_REG_CTRL_REG7_XL = @as(c_int, 0x21);
pub const LSM9DS1_REG_CTRL_REG8 = @as(c_int, 0x22);
pub const LSM9DS1_REG_CTRL_REG9 = @as(c_int, 0x23);
pub const LSM9DS1_REG_CTRL_REG10 = @as(c_int, 0x24);
pub const LSM9DS1_REG_INT_GEN_SRC_XL = @as(c_int, 0x26);
pub const LSM9DS1_REG_OUT_X_L_XL = @as(c_int, 0x28);
pub const LSM9DS1_REG_OUT_X_H_XL = @as(c_int, 0x29);
pub const LSM9DS1_REG_OUT_Y_L_XL = @as(c_int, 0x2A);
pub const LSM9DS1_REG_OUT_Y_H_XL = @as(c_int, 0x2B);
pub const LSM9DS1_REG_OUT_Z_L_XL = @as(c_int, 0x2C);
pub const LSM9DS1_REG_OUT_Z_H_XL = @as(c_int, 0x2D);
pub const LSM9DS1_REG_FIFO_CTRL = @as(c_int, 0x2E);
pub const LSM9DS1_REG_FIFO_SRC = @as(c_int, 0x2F);
pub const LSM9DS1_REG_INT_GEN_CFG_G = @as(c_int, 0x30);
pub const LSM9DS1_REG_INT_GEN_THS_XH_G = @as(c_int, 0x31);
pub const LSM9DS1_REG_INT_GEN_THS_XL_G = @as(c_int, 0x32);
pub const LSM9DS1_REG_INT_GEN_THS_YH_G = @as(c_int, 0x33);
pub const LSM9DS1_REG_INT_GEN_THS_YL_G = @as(c_int, 0x34);
pub const LSM9DS1_REG_INT_GEN_THS_ZH_G = @as(c_int, 0x35);
pub const LSM9DS1_REG_INT_GEN_THS_ZL_G = @as(c_int, 0x36);
pub const LSM9DS1_REG_INT_GEN_DUR_G = @as(c_int, 0x37);
pub const LSM9DS1_REG_OFFSET_X_REG_L_M = @as(c_int, 0x05);
pub const LSM9DS1_REG_OFFSET_X_REG_H_M = @as(c_int, 0x06);
pub const LSM9DS1_REG_OFFSET_Y_REG_L_M = @as(c_int, 0x07);
pub const LSM9DS1_REG_OFFSET_Y_REG_H_M = @as(c_int, 0x08);
pub const LSM9DS1_REG_OFFSET_Z_REG_L_M = @as(c_int, 0x09);
pub const LSM9DS1_REG_OFFSET_Z_REG_H_M = @as(c_int, 0x0A);
pub const LSM9DS1_REG_CTRL_REG1_M = @as(c_int, 0x20);
pub const LSM9DS1_REG_CTRL_REG2_M = @as(c_int, 0x21);
pub const LSM9DS1_REG_CTRL_REG3_M = @as(c_int, 0x22);
pub const LSM9DS1_REG_CTRL_REG4_M = @as(c_int, 0x23);
pub const LSM9DS1_REG_CTRL_REG5_M = @as(c_int, 0x24);
pub const LSM9DS1_REG_STATUS_REG_M = @as(c_int, 0x27);
pub const LSM9DS1_REG_OUT_X_L_M = @as(c_int, 0x28);
pub const LSM9DS1_REG_OUT_X_H_M = @as(c_int, 0x29);
pub const LSM9DS1_REG_OUT_Y_L_M = @as(c_int, 0x2A);
pub const LSM9DS1_REG_OUT_Y_H_M = @as(c_int, 0x2B);
pub const LSM9DS1_REG_OUT_Z_L_M = @as(c_int, 0x2C);
pub const LSM9DS1_REG_OUT_Z_H_M = @as(c_int, 0x2D);
pub const LSM9DS1_REG_INT_CFG_M = @as(c_int, 0x30);
pub const LSM9DS1_REG_INT_SRC_M = @as(c_int, 0x31);
pub const LSM9DS1_REG_INT_THS_L_M = @as(c_int, 0x32);
pub const LSM9DS1_REG_INT_THS_H_M = @as(c_int, 0x33);
pub const LSM9DS1_CTRL_REG4_CONFIG = @as(c_int, 0x38);
pub const LSM9DS1_CTRL_REG1_G_CONFIG = @as(c_int, 0x82);
pub const LSM9DS1_CTRL_REG5_XL_CONFIG = @as(c_int, 0x38);
pub const LSM9DS1_CTRL_REG6_XL_CONFIG = @as(c_int, 0x93);
pub const LSM9DS1_CTRL_REG8_CONFIG = @as(c_int, 0x44);
pub const LSM9DS1_CTRL_REG1_M_CONFIG = @as(c_int, 0xD0);
pub const LSM9DS1_CTRL_REG2_M_CONFIG = @as(c_int, 0x00);
pub const LSM9DS1_CTRL_REG3_M_CONFIG = @as(c_int, 0x00);
pub const LSM9DS1_CTRL_REG4_M_CONFIG = @as(c_int, 0x08);
pub const LSM9DS1_CTRL_REG5_M_CONFIG = @as(c_int, 0x40);

pub const BotControl = struct_BotControl;
