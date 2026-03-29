//! system.rs

use crate::constants::{SystemState, ControlInput, ObservationState, ObsSensativity, UGV_L, PI, DT};
use crate::utils::{DynamicsModel, wrap_to_pi};

/// Sensor model h(x)
/// y = [ heading_ugv, range, heading_uav, ξa, ηa ]^T
pub fn sensor_model(x: &SystemState) -> ObservationState {
    let dx: f64 = x[3] - x[0];
    let dy: f64 = x[4] - x[1];

    let heading_ugv: f64 = wrap_to_pi(dy.atan2(dx) - x[2]);
    let heading_uav: f64 = wrap_to_pi((-dy.atan2(-dx) - x[5]));
    let range: f64 = (dx * dx + dy * dy).sqrt();

    ObservationState::from_row_slice(&[
        heading_ugv,
        range,
        heading_uav,
        x[3],
        x[4],
    ])
}

/// Measurement Model Jacobian
pub fn measurement_jacobian(x: &SystemState) -> ObsSensativity {
    let dx: f64 = x[3] - x[0];
    let dy: f64 = x[4] - x[1];
    let r2: f64 = dx * dx + dy * dy;
    let r: f64 = r2.sqrt();

    if r < 1e-8 {
        return ObsSensativity::zeros();
    }

    let c1: f64 = dx / r2;
    let c2: f64 = dy / r2;

    let H: ObsSensativity = ObsSensativity::from_row_slice(&[
        -c2, c1, -1.0, c2, -c1, 0.0,
        -dx/r, -dy/r, 0.0, dx/r, dy/r, 0.0,
        c2, -c1, 0.0, -c2, c1, -1.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    ]);

    return H;
}

pub fn propagate_dynamics(x0: &SystemState, u: &ControlInput, t0: f64) -> SystemState {
    use ode_solvers::{Dopri5, System};

    let mut dynamics: DynamicsModel = crate::utils::DynamicsModel::new(u);
    let mut stepper: Dopri5::new(dynamics, t0, DT, 1e-6, x0.as_slice().to_vec(), 1e-6, 1e-6);
    let _ = stepper.integrate();

    let x = stepper.y_out().last().unwrap();
    return SystemState::from_row_slice(x);
}