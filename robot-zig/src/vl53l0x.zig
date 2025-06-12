// VL53L0X Laser Ranging Sensor //
//////////////////////////////////

const std = @import("std");
const c = @import({
    @cInclude("pigpio.h");
});

const VL53L0X_DEFAULT_ADDR: c_uint = 0x52;

pub fn init() !void {
    // TODO
}

// TODO
