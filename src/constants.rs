//! constants.rs

use nalgebra as na;
// use std::sync::LazyLock;

pub const DT: f64 = 0.1;                    // Simulation global timestep
pub const PI: f64 = std::f64::consts::PI;
pub const UGV_L: f64 = 0.5;                 // UGV Wheelbase length
pub const VG_MAX: f64 = 3.0;
pub const PHI_G_MAX: f64 = 5.0 * PI / 12.0;
pub const VA_MIN: f64 = 10.0;
pub const VA_MAX: f64 = 20.0;
pub const OMEGA_A_MAX: f64 = PI / 6.0;

pub const N_STATES: usize = 6;              // [xg, yg, thetag, xa, ya, thetaa]
pub const N_INPUTS: usize = 4;              // [vg, phig, va, omega_a]
pub const N_MEASUREMENTS: usize = 5;        // [bearing_g, range, bearing_a, xa, ya]

/// Type aliases for state-space system
pub type SystemState = na::SVector<f64, N_STATES>;
pub type StateTransition = na::SMatrix<f64, N_STATES, N_STATES>;
pub type ControlMatrix = na::SMatrix<f64, N_STATES, N_INPUTS>;
pub type ControlInput = na::SVector<f64, N_INPUTS>;

pub type ObservationState = na::SVector<f64, N_MEASUREMENTS>;
pub type ObsSensativity = na::SMatrix<f64, N_MEASUREMENTS, N_STATES>;

pub type StateCov = na::SMatrix<f64, N_STATES, N_STATES>;
pub type MeasCov = na::SMatrix<f64, N_MEASUREMENTS, N_MEASUREMENTS>;

// Ground truth process & measurement noise covariance matrix generators
pub fn q_true() -> StateCov {
    let mut q: StateCov = StateCov::zeros();
    q[(0,0)] = 0.01;
    q[(1,1)] = 0.01;
    q[(2,2)] = 0.001;
    q[(3,3)] = 0.01;
    q[(4,4)] = 0.01;
    q[(5,5)] = 0.001;

    return q;
}

pub fn r_true() -> MeasCov {
    let mut r: MeasCov = MeasCov::zeros();
    r[(0,0)] = 0.0225;
    r[(1,1)] = 64.0;
    r[(2,2)] = 0.04;
    r[(3,3)] = 36.0;
    r[(4,4)] = 36.0;

    return r;
}

// Helper to wrap angles to interval [-pi, pi]
pub fn wrap_to_pi(angle: f64) -> f64 {
    let mut a: f64 = angle % (2.0 * PI);

    if a > PI {
        a -= 2.0 * PI;
    } else if a < -PI {
        a += 2.0 * PI;
    }

    return a;
}
