// BMM150 geomagnetic sensor driver
//

// BMM150 I2C address
//pub const BMM150_ADDR: c_uint = 0x10; // If CSB and SDO are low
pub const I2C_ADDRESS: u8 = 0x13; // If CSB and SDO are high (floating)
pub const CHIP_ID: u8 = 0x32; // Expected Chip ID for BMM150
pub const READY_STATUS_MASK: u8 = 0x01;

pub const REG_CHIP_ID = 0x40;
pub const REG_DATA_X_LSB = 0x42;
pub const REG_DATA_Y_LSB = 0x44;
pub const REG_DATA_Z_LSB = 0x46;
pub const REG_RHALL_LSB_DATA_READY_STATUS = 0x48;
pub const REG_RHALL_MSB = 0x49;
pub const REG_POWER_CONTROL = 0x4B;
pub const REG_OP_MODE = 0x4C;
pub const REG_REP_XY = 0x51;
pub const REG_REP_Z = 0x52;

// Trim registers
const REG_DIG_X1 = 0x5D;
const REG_DIG_Y1 = 0x5E;
const REG_DIG_Z4_LSB = 0x62;
const REG_DIG_Z4_MSB = 0x63;
const REG_DIG_X2 = 0x64;
const REG_DIG_Y2 = 0x65;
const REG_DIG_Z2_LSB = 0x68;
const REG_DIG_Z2_MSB = 0x69;
const REG_DIG_Z1_LSB = 0x6A;
const REG_DIG_Z1_MSB = 0x6B;
const REG_DIG_XYZ1_LSB = 0x6C;
const REG_DIG_XYZ1_MSB = 0x6D;
const REG_DIG_Z3_LSB = 0x6E;
const REG_DIG_Z3_MSB = 0x6F;
const REG_DIG_XY2_LSB = 0x70;
const REG_DIG_XY2_MSB = 0x71;

const OVERFLOW_ADCVAL_XYAXES_FLIP = -4096;
const OVERFLOW_ADCVAL_ZAXIS_HALL = -16384;
pub const OVERFLOW_OUTPUT = -32768;
const POSITIVE_SATURATION_Z = 32767;
const NEGATIVE_SATURATION_Z = -32768;

const std = @import("std");
const types = @import("types");
const halMod = @import("hal");
const HAL = halMod.HAL;

var bmm150I2C: ?std.fs.File = null;
var hal: HAL = undefined;

pub const IntVec3 = struct {
    x: i16,
    y: i16,
    z: i16,
};

// trim registers
var dig_x1: i8 = undefined;
var dig_y1: i8 = undefined;
var dig_x2: i8 = undefined;
var dig_y2: i8 = undefined;
var dig_z1: u16 = undefined;
var dig_z2: i16 = undefined;
var dig_z3: i16 = undefined;
var dig_z4: i16 = undefined;
var dig_xy1: u8 = undefined;
var dig_xy2: i8 = undefined;
var dig_xyz1: u16 = undefined;

// BMM150 sensor initialization
pub fn init(hardware_abstraction_layer: HAL) !void {
    hal = hardware_abstraction_layer;
    bmm150I2C = try hal.i2cOpen(1, I2C_ADDRESS);
    const handle = &bmm150I2C.?;
    // Initialize BMM150 sensor

    // Wake up the sensor
    // The sensor starts in Suspend mode. Set the power control bit to 1
    // to enable the digital circuitry. This puts the sensor into Sleep mode.
    try hal.i2cWriteByte(handle, REG_POWER_CONTROL, 0x01);
    std.time.sleep(std.time.ns_per_ms * 3); // allow time for the sensor to power up

    // Verify Sensor Identity (this field reads as 0 if the sensor is in Suspend mode)
    const chip_id: u8 = try hal.i2cReadByte(handle, REG_CHIP_ID);
    if (chip_id != CHIP_ID) {
        std.debug.print("BMM150: Unexpected Chip ID: {x}\n", .{chip_id});
        return error.InvalidChipID;
    }

    // Update Trim Values
    try read_trim_values(handle);

    // Configure Measurement Repetitions
    // It's best to configure the measurement repetitions while the sensor is in Sleep mode.
    // We will use the "Regular" preset values recommended by the datasheet.
    // For XY-axis: N_XY = 4 repetitions. Register value = (2 * N_XY) + 1 = 9
    try hal.i2cWriteByte(handle, REG_REP_XY, 0x09);
    // For Z-axis: N_Z = 7 repetitions. Register value = (2 * N_Z) + 1 = 15
    try hal.i2cWriteByte(handle, REG_REP_Z, 0x0F);

    // Set Operation Mode and Data Rate
    // This write operation takes the sensor from Sleep mode to Normal mode
    // and starts the continuous measurements.
    // OpMode = Normal (00), Output Data Rate (ODR) = 10Hz (000)
    try hal.i2cWriteByte(handle, REG_OP_MODE, 0x00);

    std.debug.print("BMM150: Sensor initialized successfully\n", .{});
}

pub fn close() void {
    hal.i2cClose(bmm150I2C.?);
}

fn read_trim_values(handle: *std.fs.File) !void {
    var trim_x1y1: [2]u8 = undefined;
    try hal.i2cRead(handle, REG_DIG_X1, &trim_x1y1);
    var trim_xyz_data: [4]u8 = undefined;
    try hal.i2cRead(handle, REG_DIG_Z4_LSB, &trim_xyz_data);
    var trim_xy1xy2: [10]u8 = undefined;
    try hal.i2cRead(handle, REG_DIG_Z2_LSB, &trim_xy1xy2);
    std.debug.print("Trim Values:\ntrim_x1y1{x}\ntrim_xyz_data{x}\ntrim_xy1xy2{x}\n", .{ trim_x1y1, trim_xyz_data, trim_xy1xy2 });
    // trim data which is read is updated in the device structure
    dig_x1 = @bitCast(trim_x1y1[0]);
    dig_y1 = @bitCast(trim_x1y1[1]);
    dig_x2 = @bitCast(trim_xyz_data[2]);
    dig_y2 = @bitCast(trim_xyz_data[3]);
    dig_z1 = @as(u16, trim_xy1xy2[3]) << 8 | trim_xy1xy2[2];
    dig_z2 = @as(i16, trim_xy1xy2[1]) << 8 | trim_xy1xy2[0];
    dig_z3 = @as(i16, trim_xy1xy2[7]) << 8 | trim_xy1xy2[6];
    dig_z4 = @as(i16, trim_xyz_data[1]) << 8 | trim_xyz_data[0];
    dig_xy1 = trim_xy1xy2[9];
    dig_xy2 = @bitCast(trim_xy1xy2[8]);
    dig_xyz1 = @as(u16, trim_xy1xy2[5] & 0x7f) << 8 | trim_xy1xy2[4];

    std.debug.print("dig_x1: {x}\n", .{dig_x1});
    std.debug.print("dig_y1: {x}\n", .{dig_y1});
    std.debug.print("dig_x2: {x}\n", .{dig_x2});
    std.debug.print("dig_y2: {x}\n", .{dig_y2});
    std.debug.print("dig_z1: {x}\n", .{dig_z1});
    std.debug.print("dig_z2: {x}\n", .{dig_z2});
    std.debug.print("dig_z3: {x}\n", .{dig_z3});
    std.debug.print("dig_z4: {x}\n", .{dig_z4});
    std.debug.print("dig_xy1: {x}\n", .{dig_xy1});
    std.debug.print("dig_xy2: {x}\n", .{dig_xy2});
    std.debug.print("dig_xyz1: {x}\n", .{dig_xyz1});
}

// BMM150 sensor read
pub fn read() !IntVec3 {
    const handle = &bmm150I2C.?;
    // Read BMM150 sensor data

    // Read Raw Data Registers
    var buffer: [8]u8 = undefined; // Read from 0x42 up to and including 0x49
    // must read in one operation to ensure data integrity
    try hal.i2cRead(handle, REG_DATA_X_LSB, &buffer);
    // After the read the Data Ready Status bit is cleared
    // 1. Check Ready Status
    const status: u8 = buffer[REG_RHALL_LSB_DATA_READY_STATUS - REG_RHALL_LSB_DATA_READY_STATUS];
    if ((status & READY_STATUS_MASK) == 0) {
        std.debug.print("BMM150: Sensor not ready\n\n", .{});
        return error.SensorNotReady;
    }

    //std.debug.print("BMM150: Sensor data: {x}\n", .{buffer});
    const raw_datax = (@as(i16, buffer[1]) << 8 | buffer[0]) >> 3; // raw x 13 bits
    const raw_datay = (@as(i16, buffer[3]) << 8 | buffer[2]) >> 3; // raw y 13 bits
    const raw_dataz = (@as(i16, buffer[5]) << 8 | buffer[4]) >> 1; // raw z 15 bits
    const raw_data_r = (@as(u16, buffer[7]) << 6 | (buffer[6] >> 2)); // raw rhall 10 bits

    //std.debug.print("BMM150: raw_datax: {x}\n", .{raw_datax});
    //std.debug.print("BMM150: raw_datay: {x}\n", .{raw_datay});
    //std.debug.print("BMM150: raw_dataz: {x}\n", .{raw_dataz});

    const x: i16 = (compensate_x(raw_datax, raw_data_r));
    //std.debug.print("BMM150: compensated_x: {x}\n", .{x});
    const y: i16 = (compensate_y(raw_datay, raw_data_r));
    //std.debug.print("BMM150: compensated_y: {x}\n", .{y});
    const z: i16 = (compensate_z(raw_dataz, raw_data_r));
    //std.debug.print("BMM150: compensated_z: {x}\n", .{z});

    // if (x < 0) x = x + 360;
    // if (y < 0) y = y + 360;
    // if (z < 0) z = z + 360;

    return IntVec3{
        .x = x,
        .y = y,
        .z = z,
    };
}

fn compensate_x(mag_data_x: i16, data_rhall: u16) i16 {
    var retval: i16 = undefined;
    if (mag_data_x != OVERFLOW_ADCVAL_XYAXES_FLIP) {
        const process_comp_x0: u16 = if (data_rhall != 0)
            // availability of valid data
            data_rhall
        else if (dig_xyz1 != 0)
            dig_xyz1
        else
            0;

        if (process_comp_x0 != 0) {
            const process_comp_x1: i32 = @as(i32, dig_xyz1) *% 16384;
            //std.debug.print("BMM150: process_comp_x1: {x}\n", .{process_comp_x1});
            const process_comp_x2: u16 = @as(u16, @bitCast(@as(i16, @truncate(@divTrunc(process_comp_x1, process_comp_x0))))) - 0x4000;
            //std.debug.print("BMM150: process_comp_x2: {x}\n", .{process_comp_x2});
            retval = @bitCast(process_comp_x2);
            const process_comp_x3: i32 = @as(i32, retval) * @as(i32, retval);
            //std.debug.print("BMM150: process_comp_x3: {x}\n", .{process_comp_x3});
            const process_comp_x4: i32 = @as(i32, dig_xy2) *% @divTrunc(process_comp_x3, 128);
            //std.debug.print("BMM150: process_comp_x4: {x}\n", .{process_comp_x4});
            const process_comp_x5: i32 = @as(i32, @as(i16, dig_xy1) *% 128);
            //std.debug.print("BMM150: process_comp_x5: {x}\n", .{process_comp_x5});
            const process_comp_x6: i32 = @as(i32, retval) *% process_comp_x5;
            //std.debug.print("BMM150: process_comp_x6: {x}\n", .{process_comp_x6});
            const process_comp_x7: i32 = @divTrunc(process_comp_x4 +% process_comp_x6, 512) +% 0x100000;
            //std.debug.print("BMM150: process_comp_x7: {x}\n", .{process_comp_x7});
            const process_comp_x8: i32 = @intCast(@as(i16, dig_x2) +% 0xA0);
            //std.debug.print("BMM150: process_comp_x8: {x}\n", .{process_comp_x8});
            const process_comp_x9: i32 = @divTrunc(process_comp_x7 *% process_comp_x8, 4096);
            //std.debug.print("BMM150: process_comp_x9: {x}\n", .{process_comp_x9});
            const process_comp_x10: i32 = @as(i32, mag_data_x) *% process_comp_x9;
            //std.debug.print("BMM150: process_comp_x10: {x}\n", .{process_comp_x10});
            retval = @truncate(@divTrunc(process_comp_x10, 8192));
            retval = @divTrunc(retval +% @as(i16, dig_x1) *% 8, 16);
        } else {
            retval = OVERFLOW_OUTPUT;
        }
    } else {
        retval = OVERFLOW_OUTPUT;
    }
    return retval;
}

fn compensate_y(mag_data_y: i16, data_rhall: u16) i16 {
    var retval: i16 = undefined;
    if (mag_data_y != OVERFLOW_ADCVAL_XYAXES_FLIP) {
        const process_comp_y0: u16 = if (data_rhall != 0)
            // availability of valid data
            data_rhall
        else if (dig_xyz1 != 0)
            dig_xyz1
        else
            0;

        const process_comp_y1: i32 = @divTrunc(@as(i32, dig_xyz1) *% 16384, process_comp_y0);
        const process_comp_y2: u16 = @as(u16, @truncate(@as(u32, @bitCast(process_comp_y1)))) -% 0x4000;
        retval = @bitCast(process_comp_y2);
        const process_comp_y3: i32 = @as(i32, retval) *% @as(i32, retval);
        const process_comp_y4: i32 = @as(i32, dig_xy2) *% @divTrunc(process_comp_y3, 128);
        const process_comp_y5: i32 = @as(i32, @as(i16, dig_xy1) *% 128);
        const process_comp_y6: i32 = @divTrunc(process_comp_y4 +% (@as(i32, retval) * process_comp_y5), 512);
        const process_comp_y7: i32 = @as(i32, @as(i16, dig_y2) +% @as(i16, 0xA0));
        const process_comp_y8: i32 = @divTrunc(process_comp_y6 +% (@as(i32, 0x100000) *% process_comp_y7), 4096);
        const process_comp_y9: i32 = @as(i32, mag_data_y) *% process_comp_y8;
        retval = @truncate(@divTrunc(process_comp_y9, 8192));
        retval = @divTrunc(retval +% (@as(i16, dig_y1) *% 8), 16);
    } else {
        retval = OVERFLOW_OUTPUT;
    }
    return retval;
}

fn compensate_z(mag_data_z: i16, data_rhall: u16) i16 {
    var retval: i32 = undefined;
    if (mag_data_z != OVERFLOW_ADCVAL_ZAXIS_HALL) {
        if (dig_z2 != 0 and dig_z1 != 0 and data_rhall != 0 and dig_xyz1 != 0) {
            // processing compensation equations
            const process_comp_z0: i16 = @as(i16, @bitCast(data_rhall)) -% @as(i16, @bitCast(dig_xyz1));
            const process_comp_z1 = @as(i32, dig_z3) *% @divTrunc(@as(i32, process_comp_z0), 4);
            const process_comp_z2 = (@as(i32, mag_data_z) -% dig_z4) *% 32768;
            const process_comp_z3 = @as(i32, dig_z1) *% (@as(i16, @bitCast(data_rhall)) *% 2);
            const process_comp_z4 = @as(i16, @truncate(@divTrunc(process_comp_z3 +% 32768, 65536)));
            retval = (@divTrunc((process_comp_z2 -% process_comp_z1), (dig_z2 +% process_comp_z4)));

            // Saturate result to +/- 2 micro-tesla
            if (retval > POSITIVE_SATURATION_Z) {
                retval = POSITIVE_SATURATION_Z;
            } else if (retval < NEGATIVE_SATURATION_Z) {
                retval = NEGATIVE_SATURATION_Z;
            }
            // Conversion of LSB to micro-tesla
            retval = @divTrunc(retval, 16);
        } else {
            retval = OVERFLOW_OUTPUT;
        }
    } else {
        retval = OVERFLOW_OUTPUT;
    }
    return @truncate(retval);
}
