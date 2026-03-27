//! constants.rs

use nalgebra as na;

/// Simulation global timestep
pub const DT: f64 = 0.1;

pub const PI: f64 = std::f64::consts::PI;

/// UGV Wheelbase length
pub const UGV_L: f64 = 0.5;

/// Type aliases for state-space system
pub type SystemState = na::SVector<f64, 6>;
pub type StateTransition = na::SMatrix<f64, 6, 6>;
pub type ControlMatrix = na::SMatrix<f64, 6, 4>;
pub type ControlInput = na::SVector<f64, 4>;

pub type ObservationState = na::SVector<f64, 5>;
pub type ObsSensativity = na::SMatrix<f64, 5, 6>;

pub type StateCov = na::SMatrix<f64, 6, 6>;
pub type MeasCov = na::SMatrix<f64, 5, 5>;

pub const Q_TRUE: StateCov = StateCov::from_diagonal(&na::SVector<f64,6>::from_row_slice(&[
    0.001, 0.001, 0.01, 0.001, 0.001, 0.01
]));
pub const R_TRUE: MeasCov = MeasCov::from_diagonal(&na::SVector<f64,5>::from_row_slice(&[
    0.0225, 64.0, 0.04, 36.0, 36.0
]));

#[derive(Debug, Clone)]
pub struct TruthParams {
    pub q_true: StateCov,
    pub r_true: MeasCov,
}

impl Default for TruthParams {
    fn default() -> Self {
        Self {
            q_true: Q_TRUE,
            r_true: R_TRUE,
        }
    }
}
