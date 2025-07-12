const std = @import("std");

pub const Raw3 = struct { x: i16, y: i16, z: i16 };
pub const Vec3 = struct {
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

pub const NormalizedSensorData = struct {
    gyro: struct { x: f32, y: f32, z: f32 },
    accel: struct { x: f32, y: f32, z: f32 },
};

pub const PID_Ks = struct { Kp: f32, Ki: f32, Kd: f32 };
pub const BalanceState = enum {
    Stable, // basicly upright, very minor adjustments needed
    Fine, // upright but starting to tilt
    Moderate, // Mild disturbance - still upright
    Falling,
    Fall, // Too far gone to recover safely, just kill the motor power and fall over.
};

pub const PIDTwiddleState = struct {
    params: PID_Ks,
    deltas: PID_Ks,
    best_cost: f32,
    current_cost: f32 = 0.0,
    num_samples: u16 = 0,
    step: usize = 0,
    direction: i8 = 1, // +1 or -1
    state: enum {
        Init,
        TryIncrease,
        TryDecrease,
        Done,
    } = .Init,
};
