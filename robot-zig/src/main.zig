//! By convention, main.zig is where your main function lives in the case that
//! you are building an executable. If you are making a library, the convention
//! is to delete this file and start with root.zig instead.

const std = @import("std");
/// This imports the separate module containing `robot.zig`. Take a look in `build.zig` for details.
const lib = @import("robot_lib");
const web = @import("robot_web");
//const httpz = @import("httpz");
const tts = @import("speech");

const types = @import("types");
const BalanceState = types.BalanceState;
const PIDTwiddleState = types.PIDTwiddleState;

const PID_Ks = types.PID_Ks;
const MOTOR_DONT_BOTHER: f32 = 0.0002; // Motor lower than this, not really needing to move at all.
const MOTOR_MIN: f32 = 0.0004; // to overcome internal friction, etc.
const MOTOR_MAX: f32 = 0.30; // limiting motor power %
const SAMPLE_PERIOD_US = 4000; // 4ms
const DRIVE_MOTOR_POWER_SCALE_FACTOR: f32 = 0.0002;
const MAX_VELOCITY_DELTA: f32 = 0.6;
const TILT_BIAS: f32 = 3.6; // 3.5° bias to properly balance with current physical config

//var pid = PID{ .kp = 5.0, .ki = 0, .kd = 0.5 };
// var pid = PID{ .kp = 1.0, .ki = 0, .kd = 1.5 }; flops on the ground

// pid values currently in use (may be a linear interpolation between the values below)
var pid = PID{ .K = .{ .Kp = 15.0, .Ki = 0, .Kd = 2.0 } };

var pidKs_stable = PID_Ks{ .Kp = 14.3, .Ki = 0.0, .Kd = 4.1 };
var pidKs_fine = PID_Ks{ .Kp = 20.5, .Ki = 0.0, .Kd = 6.7 };
var pidKs_moderate = PID_Ks{ .Kp = 50.5, .Ki = 0.0, .Kd = 15.5 };
var pidKs_falling = PID_Ks{ .Kp = 115.0, .Ki = 0.0, .Kd = 20.0 };

// TWIDDLE
var twiddle_Stable = PIDTwiddleState{ .params = PID_Ks{ .Kp = 14.3, .Ki = 0.0, .Kd = 4.1 }, .deltas = PID_Ks{ .Kp = 0.1, .Ki = 0.0, .Kd = 0.1 }, .best_cost = 100000.0 };
var twiddle_Fine = PIDTwiddleState{ .params = PID_Ks{ .Kp = 20.2, .Ki = 0.0, .Kd = 6.7 }, .deltas = PID_Ks{ .Kp = 0.1, .Ki = 0.0, .Kd = 0.1 }, .best_cost = 100000.0 };
var twiddle_Moderate = PIDTwiddleState{ .params = PID_Ks{ .Kp = 50.0, .Ki = 0.0, .Kd = 15.0 }, .deltas = PID_Ks{ .Kp = 0.1, .Ki = 0.0, .Kd = 0.1 }, .best_cost = 100000.0 };
var twiddle_Falling = PIDTwiddleState{ .params = PID_Ks{ .Kp = 115.0, .Ki = 0.0, .Kd = 20.0 }, .deltas = PID_Ks{ .Kp = 0.1, .Ki = 0.0, .Kd = 0.1 }, .best_cost = 100000.0 };
const twiddle = false; // set to true to enable twiddling

const fine_threshold = 1.0; // up to this many degrees, we are stable and need minial power to remain balanced
const moderate_threshold = 5.0; // use more aggressive correction after tilt of this many degrees
const falling_threshold = 10.0; // Need aggressive correction after tilt of this many degrees
const unstable_threshold = 22.0; // If tilt is more than this, we are falling over, cut the motors

var joytick_x: f32 = 0.0; // steering control
var joytick_y: f32 = 0.0; // throttle control

var angle: f32 = 0.0; // filtered tilt angle
const alpha: f32 = 0.975;
var target_angle: f32 = 0.0;
var prev_gyro_rate_dps: f32 = 0.0;
var tilt_mode: BalanceState = BalanceState.Fine;
var last_tilt_mode: i8 = 0;

const webContext = if (twiddle) web.WebContext{
    .pid = &pid.K,
    .stable = &twiddle_Stable.params,
    .fine = &twiddle_Fine.params,
    .moderate = &twiddle_Moderate.params,
    .falling = &twiddle_Falling.params,
    .state = &tilt_mode,
    .flag_ptr = &in_sensor_debug,
    .x = &joytick_x,
    .y = &joytick_y,
} else web.WebContext{
    .pid = &pid.K,
    .stable = &pidKs_stable,
    .fine = &pidKs_fine,
    .moderate = &pidKs_moderate,
    .falling = &pidKs_falling,
    .state = &tilt_mode,
    .flag_ptr = &in_sensor_debug,
    .x = &joytick_x,
    .y = &joytick_y,
};

pub fn main() !void {
    std.debug.print("Robots are cool.\n", .{});
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const allocator = gpa.allocator();

    // stdout is for the actual output of your application, for example if you
    // are implementing gzip, then only the compressed bytes should be sent to
    // stdout, not any debugging messages.
    const stdout_file = std.io.getStdOut().writer();
    var bw = std.io.bufferedWriter(stdout_file);
    const stdout = bw.writer();

    try stdout.print("Initializing text-to-speech...\n", .{});
    try bw.flush(); // Don't forget to flush!
    try tts.init();
    defer tts.deinit() catch unreachable;

    try stdout.print("Setting up internal web server...\n", .{});
    try bw.flush();

    // start webserver
    const server = try web.webserver(allocator, &webContext);
    defer {
        server.stop();
        server.deinit();

        // ensure motors are off
        main_drive_motors(0, 0);
    }

    try stdout.print("For now, I'm just going to try to stand up straight.\n", .{});
    try bw.flush();

    try lib.initialize();
    defer lib.cleanup();

    try stdout.print("\n\n\n", .{});
    try bw.flush();

    while (true) {
        try balance();

        // ensure motors are off
        main_drive_motors(0, 0);

        try sensor_debug_loop();
    }
}

var in_sensor_debug: bool = false;
fn sensor_debug_loop() !void {
    in_sensor_debug = true;
    const flag_ptr: *volatile bool = &in_sensor_debug;
    while (flag_ptr.*) {
        const now_us = std.time.microTimestamp();
        try lib.readSensors();
        lib.dumpSensors();
        const ellapsed = std.time.microTimestamp() - now_us;
        if (ellapsed < SAMPLE_PERIOD_US) {
            std.time.sleep(@intCast(1000 * (SAMPLE_PERIOD_US - ellapsed))); // sleep time is ns
        } else {
            std.debug.print("Compute time in balance loop was {}us\n", .{ellapsed});
        }
    }
}

fn calibrateGyro(samples: usize) f32 {
    var sum: f32 = 0.0;
    for (0..samples) |_| {
        lib.readSensors() catch break;
        const sample = lib.orientation().x; // deg/sec
        sum += sample;
        sleepMicroseconds(500); // adjust for your ODR
    }
    return sum / @as(f32, @floatFromInt(samples));
}

fn balance() !void {
    var current_motor_power: f32 = 0.0;
    var last_time = std.time.microTimestamp();

    // calibration
    var gyro_bias: f32 = 0.0;
    var not_calibrated = true;
    while (not_calibrated) {
        gyro_bias = calibrateGyro(250);
        std.debug.print("Gyro bias: {d}\n", .{gyro_bias});
        if (gyro_bias != 0.0) {
            not_calibrated = false;
        } else {
            try lib.initializeIMU();
            std.debug.print("Gyro calibration failed. Retinitializing...\n", .{});
        }
    }

    //const runtime = 600; // seconds
    //const startTime = std.time.timestamp(); // second
    //while (std.time.timestamp() - startTime < runtime) {

    while (true) {
        const now_us = std.time.microTimestamp();
        const delta_us = @max(now_us - last_time, 1); // minimum 1 µs
        last_time = now_us;

        const dt: f32 = @as(f32, @floatFromInt(delta_us)) / 1000000.0;

        lib.readSensors() catch break;
        //plotXY();

        // Goal is for Y acceleration to be zero indicating no tilt
        // as the x/y plane should be perpendicular to gravity vector
        // meaning the vector in the yz-plane has no y component.
        // We want no forward or backward acceleration (for now)
        // This will be different when we want the robot to be in motion.
        target_angle = 0.0;

        // rate of tilt
        const gyro_rate_dps = lib.orientation().x - gyro_bias; // deg/sec

        const throttle_bias = joytick_y / 30.0;
        const steering_bias = joytick_x / 100.0;

        // Compute angle from accelerometer
        // (pitch or tilt in the yz-plane)
        const pitch = lib.tilt_angle_yz() + TILT_BIAS + throttle_bias;

        // Complementary filter
        // Weighted average of:
        //   predicted angle (previous angle + rate_of_chnage * dt)
        //   measured angle
        const gyro_rate_avg = (prev_gyro_rate_dps + gyro_rate_dps) * 0.5;
        prev_gyro_rate_dps = gyro_rate_dps;
        angle = alpha * (angle + gyro_rate_avg * dt) + (1.0 - alpha) * pitch;

        // Compute control
        const angle_error = target_angle - angle;

        // Based on the absolute angle we will adjust the PID gains so it is more agressive when severely tilted
        const angleAbs = @abs(angle);
        const gyroAbs = @abs(gyro_rate_avg);
        tilt_mode = calcBalanceState(angleAbs, gyroAbs); // default to Fine control

        // TWIDDLE to fine tune PID gains
        if (twiddle) {
            const twiddle_pick: ?*PIDTwiddleState = switch (tilt_mode) {
                BalanceState.Stable => &twiddle_Stable,
                BalanceState.Fine => &twiddle_Fine,
                BalanceState.Moderate => &twiddle_Moderate,
                BalanceState.Falling => &twiddle_Falling,
                else => null, // no twiddling for Falling or Fall states
            };
            if (twiddle_pick) |twiddle_state| {
                twiddle_state.current_cost += compute_cost(angle_error, dt);
                twiddle_state.num_samples += 1;
                if (twiddle_state.num_samples >= 600) { // approx 3 seconds of samples
                    // after threshold
                    std.debug.print("Twiddling cost for {} state is {} after {} samples.\n", .{ tilt_mode, twiddle_state.current_cost, twiddle_state.num_samples });
                    twiddle_update(twiddle_state);
                    pid.K = twiddle_state.params;
                }
            }
        } else {
            switch (tilt_mode) {
                BalanceState.Stable => {
                    // Fine control, use normal PID gains
                    pid.K = pidKs_stable;
                },
                BalanceState.Fine => {
                    const gain_factor = std.math.clamp(angleAbs / (moderate_threshold - fine_threshold), 0.0, 1.0);
                    pid.K = lerp(pidKs_fine, pidKs_moderate, gain_factor);
                },
                BalanceState.Moderate => {
                    // Moderate disturbance, use more agressive PID gains
                    const gain_factor = std.math.clamp(angleAbs / (falling_threshold - moderate_threshold), 0.0, 1.0);
                    pid.K = lerp(pidKs_moderate, pidKs_falling, gain_factor);
                },
                BalanceState.Falling => {
                    // Falling over, use very agressive PID gains
                    pid.K = pidKs_falling;
                },
                BalanceState.Fall => {
                    // Too far gone to recover, just cut the motors
                    main_drive_motors(0, 0);
                    std.debug.print("Falling over! Cutting motors.\n", .{});
                    return; // exit balance loop
                },
                // else => {
                //     std.debug.print("Unknown tilt mode: {}\n", .{tilt_mode});
                //     pid.K = pidKs_fine; // fallback to conservative PID gains
                // },
            }
        }

        const output = pid.update(angle_error, gyro_rate_dps, dt) * DRIVE_MOTOR_POWER_SCALE_FACTOR;
        const clamped_output = std.math.clamp(output, -MOTOR_MAX, MOTOR_MAX);

        var motor_power = clamped_output;
        //const motor_power = limit_rate(current_motor_power, clamped_output, dt);

        // min drive threshold
        if (@abs(motor_power) < MOTOR_MIN and @abs(motor_power) > MOTOR_DONT_BOTHER) {
            motor_power = sign(motor_power) * MOTOR_MIN;
        }

        const steering_power = steering_bias * (4 * MOTOR_MIN + @abs(motor_power) * 0.10); // steering bias is a fraction of motor power

        current_motor_power = motor_power;

        const left_power = motor_power + steering_power;
        const right_power = motor_power - steering_power;

        // simulated braking with brief reverse pulse
        const simulated_braking = false; // set to true to enable simulated braking
        const min_drive = 0.002;
        const braking_threshold = 3;
        const brake_strength = 0.0017;
        if (simulated_braking and @abs(motor_power) < min_drive and @abs(gyro_rate_dps) > braking_threshold) {
            // apply short reverse pulse
            const brake_power = -brake_strength * sign(gyro_rate_dps);
            main_drive_motors(brake_power, brake_power);
        } else {
            main_drive_motors(left_power, right_power);
        }

        // debug print
        //std.debug.print("Current gyro rate: {d: >7.4}°/s\n", .{gyro_rate_dps});
        const msg = "{s}𝚫t = {d: >7.6}s,   error = {d: >7.4}, da/dt = {d: >8.4}°/s   P = {d: >8.4},   I = {d: >8.4},   D = {d: >8.4},   output = {d: >7.4}{s}\x1b[39m\x1b[A\n";
        const text_colour = switch (tilt_mode) {
            BalanceState.Stable, BalanceState.Fine => "\x1b[32m",
            BalanceState.Moderate => "\x1b[33m",
            BalanceState.Falling => "\x1b[31m",
            else => "",
        };
        const is_clamped = if (clamped_output != output) " motor output clamped" else ""; //clamped_output
        std.debug.print(msg, .{ text_colour, dt, angle_error, gyro_rate_avg, pid.P, pid.I, pid.D, motor_power, is_clamped });

        // Pacing - SAMPLE_PERIOD_MS per iteration
        const ellapsed = std.time.microTimestamp() - now_us;
        if (ellapsed < SAMPLE_PERIOD_US) {
            sleepMicroseconds(@intCast(SAMPLE_PERIOD_US - ellapsed)); // sleep time is ns
        } else {
            std.debug.print("Compute time in balance loop was {}us\n", .{ellapsed});
        }
    }
    std.debug.print("Exited balance loop.", .{});
}

fn lerp(a: PID_Ks, b: PID_Ks, t: f32) PID_Ks {
    return .{
        .Kp = a.Kp + (b.Kp - a.Kp) * t,
        .Ki = a.Ki + (b.Ki - a.Ki) * t,
        .Kd = a.Kd + (b.Kd - a.Kd) * t,
    };
}

fn calcBalanceState(angleAbs: f32, gyroAbs: f32) BalanceState {
    if (angleAbs > unstable_threshold) {
        // We are falling over, cut the motors
        return BalanceState.Fall;
    }
    if (angleAbs > falling_threshold or gyroAbs > 80.0) {
        // We are falling over, cut the motors
        return BalanceState.Falling;
    } else if (angleAbs > moderate_threshold) {
        // Emergency recovery mode
        return BalanceState.Moderate;
    } else if (abs(angle) > fine_threshold) {
        // Mild disturbance - still upright
        return BalanceState.Fine;
    } else { // within stable threshold
        // Steady-state fine control
        return BalanceState.Stable;
    }
}

// TWIDDLE

fn compute_cost(angle_error: f32, dt: f32) f32 {
    return angle_error * angle_error * dt;
}
fn twiddle_update(t: *PIDTwiddleState) void {
    const current_cost = t.current_cost;
    const i = t.step % 3;
    var param_ptr: *f32 = undefined;
    var delta_ptr: *f32 = undefined;
    if (i == 0) {
        param_ptr = &t.params.Kp;
        delta_ptr = &t.deltas.Kp;
    } else if (i == 1) {
        param_ptr = &t.params.Ki;
        delta_ptr = &t.deltas.Ki;
    } else {
        param_ptr = &t.params.Kd;
        delta_ptr = &t.deltas.Kd;
    }

    switch (t.state) {
        .Init => {
            param_ptr.* += delta_ptr.*;
            t.state = .TryIncrease;
        },
        .TryIncrease => {
            if (current_cost < t.best_cost) {
                t.best_cost = current_cost;
                delta_ptr.* *= 1.1;
                t.step += 1;
                t.state = .Init;
            } else {
                param_ptr.* -= 2 * delta_ptr.*;
                t.state = .TryDecrease;
            }
        },
        .TryDecrease => {
            if (current_cost < t.best_cost) {
                t.best_cost = current_cost;
                delta_ptr.* *= 1.1;
            } else {
                param_ptr.* += delta_ptr.*;
                delta_ptr.* *= 0.9;
            }
            t.step += 1;
            t.state = .Init;
        },
        else => {},
    }
    // reset for next iteration
    t.num_samples = 0;
    t.current_cost = 0.0;
}

// utils

fn sleepMicroseconds(useconds: u64) void {
    std.time.sleep(1000 * useconds);
}

fn sign(v: f32) f32 {
    if (v == 0)
        return 0;
    if (v > 0)
        return 1;
    return -1;
}

fn main_drive_motors(left: f32, right: f32) void {
    _ = lib.driveMotor(false, left);
    _ = lib.driveMotor(true, right);
}

fn limit_rate(current: f32, target: f32, dt: f32) f32 {
    const delta = target - current;
    const clamped_delta = std.math.clamp(delta, -MAX_VELOCITY_DELTA * dt, MAX_VELOCITY_DELTA * dt);
    if (clamped_delta != delta) {
        std.debug.print("Clampled output delta {d} to {d}\n", .{ delta, clamped_delta });
    }
    return current + clamped_delta;
}

// PID stuff
const PID = struct {
    K: PID_Ks,
    // Kp: f32 = 0,
    // Ki: f32 = 0, // was 0.01
    // Kd: f32 = 0, // was 0.005

    P: f32 = 0,
    I: f32 = 0,
    D: f32 = 0,
    prev_error: f32 = 0,
    integral: f32 = 0,

    pub fn update(self: *PID, tilt_error: f32, gyro_rate: f32, dt: f32) f32 {
        // SWP tweak:
        if (@abs(tilt_error) < 1)
            self.integral = 0;

        self.integral += tilt_error * dt;
        const derivative = gyro_rate;
        //const derivative = (tilt_error - self.prev_error) / dt;
        //self.prev_error = tilt_error;

        self.P = self.K.Kp * tilt_error;
        self.I = self.K.Ki * self.integral;
        self.D = -self.K.Kd * derivative;

        const output = self.P + self.I + self.D;

        return output;
    }
};

fn abs(value: anytype) @TypeOf(value) {
    if (value < 0) return -value;
    return value;
}

// 8x8 LED Matrix fun

fn showFace() void {
    const w: u16 = 0xffff;
    const r: u16 = 0b1111100000000000;
    const g: u16 = 0b0000011111100000;
    const b: u16 = 0b0000000000011111;
    const m: u16 = 0b1111100000011111;
    const frames = [10][64]u16{
        [64]u16{
            r, r, w, w, w, w, g, g, //
            r, w, 0, 0, 0, 0, w, g, //
            w, 0, w, 0, 0, w, 0, w, //
            w, 0, 0, 0, 0, 0, 0, w, //
            w, 0, w, 0, 0, w, 0, w, //
            w, 0, 0, w, w, 0, 0, w, //
            b, w, 0, 0, 0, 0, w, m, //
            b, b, w, w, w, w, m, m,
        },
        [64]u16{
            0, 0, w, w, w, w, 0, 0, //
            0, w, 0, 0, 0, 0, w, 0, //
            w, 0, w, 0, 0, w, 0, w, //
            w, 0, 0, 0, 0, 0, 0, w, //
            w, 0, w, w, 0, w, 0, w, //
            w, 0, 0, 0, w, 0, 0, w, //
            0, w, 0, 0, 0, 0, w, 0, //
            0, 0, w, w, w, w, 0, 0,
        },
        [64]u16{
            0, 0, w, w, w, w, 0, 0, //
            0, w, 0, 0, 0, 0, w, 0, //
            w, 0, w, 0, 0, w, 0, w, //
            w, 0, 0, 0, 0, 0, 0, w, //
            w, 0, w, w, w, w, 0, w, //
            w, 0, 0, 0, 0, 0, 0, w, //
            0, w, 0, 0, 0, 0, w, 0, //
            0, 0, w, w, w, w, 0, 0,
        },
        [64]u16{
            0, 0, w, w, w, w, 0, 0, //
            0, w, 0, 0, 0, 0, w, 0, //
            w, 0, w, 0, 0, w, 0, w, //
            w, 0, 0, 0, 0, 0, 0, w, //
            w, 0, 0, w, w, w, 0, w, //
            w, 0, w, 0, 0, 0, 0, w, //
            0, w, 0, 0, 0, 0, w, 0, //
            0, 0, w, w, w, w, 0, 0,
        },
        [64]u16{
            0, 0, w, w, w, w, 0, 0, //
            0, w, 0, 0, 0, 0, w, 0, //
            w, 0, w, 0, 0, w, 0, w, //
            w, 0, 0, 0, 0, 0, 0, w, //
            w, 0, 0, w, w, 0, 0, w, //
            w, 0, w, 0, 0, w, 0, w, //
            0, w, 0, 0, 0, 0, w, 0, //
            0, 0, w, w, w, w, 0, 0,
        },
        [64]u16{
            0, 0, w, w, w, w, 0, 0, //
            0, w, 0, 0, 0, 0, w, 0, //
            w, 0, w, 0, 0, w, 0, w, //
            w, 0, 0, 0, 0, 0, 0, w, //
            w, 0, 0, w, w, 0, 0, w, //
            w, 0, 0, 0, 0, w, 0, w, //
            0, w, 0, 0, 0, 0, w, 0, //
            0, 0, w, w, w, w, 0, 0,
        },
        [64]u16{
            0, 0, w, w, w, w, 0, 0, //
            0, w, 0, 0, 0, 0, w, 0, //
            w, 0, w, 0, 0, w, 0, w, //
            w, 0, 0, 0, 0, 0, 0, w, //
            w, 0, 0, w, w, 0, 0, w, //
            w, 0, 0, 0, 0, 0, 0, w, //
            0, w, 0, 0, 0, 0, w, 0, //
            0, 0, w, w, w, w, 0, 0,
        },
        [64]u16{
            0, 0, w, w, w, w, 0, 0, //
            0, w, 0, 0, 0, 0, w, 0, //
            w, 0, w, 0, 0, w, 0, w, //
            w, 0, 0, 0, 0, 0, 0, w, //
            w, 0, 0, w, w, 0, 0, w, //
            w, 0, 0, w, w, 0, 0, w, //
            0, w, 0, 0, 0, 0, w, 0, //
            0, 0, w, w, w, w, 0, 0,
        },
        [64]u16{
            0, 0, w, w, w, w, 0, 0, //
            0, w, 0, 0, 0, 0, w, 0, //
            w, 0, w, 0, 0, w, 0, w, //
            w, 0, 0, 0, 0, 0, 0, w, //
            w, 0, w, w, w, 0, 0, w, //
            w, 0, 0, w, w, 0, 0, w, //
            0, w, 0, 0, 0, 0, w, 0, //
            0, 0, w, w, w, w, 0, 0,
        },
        [64]u16{
            0, 0, w, w, w, w, 0, 0, //
            0, w, 0, 0, 0, 0, w, 0, //
            w, 0, w, 0, 0, w, 0, w, //
            w, 0, 0, 0, 0, 0, 0, w, //
            w, 0, w, w, w, w, 0, w, //
            w, 0, 0, w, w, 0, 0, w, //
            0, w, 0, 0, 0, 0, w, 0, //
            0, 0, w, w, w, w, 0, 0,
        },
    };

    var frame: u16 = 0;
    while (frame < 10) {
        var i: u8 = 0;
        while (i < 64) {
            const c = frames[frame][i];
            const x = @as(u8, i & 7);
            const y = @as(u8, i >> 3);
            lib.plot(x, y, c);
            i += 1;
        }
        lib.flip();
        lib.sleepMillis(500);
        frame += 1;
    }
    lib.clearScreen();
}

fn plotXY() void {
    const accel = lib.acceleration();
    // full scale is set to +/- 4g
    // so 1 g = 8192.  We will treat 1g on X or Y axis as "fully sideways"
    const fx = std.math.clamp(accel.x * 3.5, -3.5, 3.5);
    const fy = std.math.clamp(accel.y * 3.5, -3.5, 3.5);
    var ix = @as(i16, @intFromFloat(fx));
    var iy = @as(i16, @intFromFloat(fy));
    const px = abs(fx - @as(f32, @floatFromInt(ix))); // partial
    const py = abs(fy - @as(f32, @floatFromInt(iy))); // partial
    // x: green, y: red
    const cx: u16 = @as(u16, @intFromFloat(0b11111 * (1 - px) + 0.5)) << 11;
    const cy: u16 = @as(u16, @intFromFloat(0b111111 * (1 - py) + 0.5)) << 5;
    const cxp: u16 = @as(u16, @intFromFloat(0b11111 * px + 0.5)) << 11;
    const cyp: u16 = @as(u16, @intFromFloat(0b111111 * py + 0.5)) << 5;
    ix = std.math.clamp(ix, -3, 3);
    iy = std.math.clamp(iy, -3, 3);
    lib.clearScreen();
    const x: u8 = @intCast(3 + ix);
    const y: u8 = @intCast(3 + iy);
    var i: u8 = 0;
    while (i < 8) {
        lib.plot(x, i, cx);
        lib.plot(x + 1, i, cxp);
        lib.plot(i, y, cy);
        lib.plot(i, y + 1, cyp);
        i += 1;
    }
    lib.plot(x, y, cx + cy);
    lib.plot(x + 1, y + 1, cxp + cyp);
    lib.plot(x, y + 1, cx + cyp);
    lib.plot(x + 1, y, cxp + cy);
    lib.flip();
}
