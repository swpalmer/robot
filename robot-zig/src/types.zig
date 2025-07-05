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
