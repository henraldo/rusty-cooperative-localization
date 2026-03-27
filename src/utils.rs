//! utils.rs

use core::f64;

use crate::constants::{ControlInput, SystemState, UGV_L, PI};
use ode_solvers::{System, Vector6};

/// Wrap angle to [-π, π]
pub fn wrap_to_pi(angle: f64) -> f64 {
    let tau = 2.0 * PI;
    return (angle + PI).rem_euclid(tau) - PI;
}

/// sec(θ)
pub fn sec(theta: f64) -> f64 {
    let c = theta.cos();
    if c.abs() < f64::EPSILON * 10.0 {
        return f64::NAN;
    } else {
        return 1.0 / c;
    }
}

#[derive(Debug)]
pub struct DynamicsModel {
    pub u: ControlInput,
}

impl DynamicsModel {
    pub fn new(control_vec: ControlInput) -> Self {
        Self { u: control_vec }
    }
}

impl System<f64, Vector6<f64>> for DynamicsModel {
    /// rhs = right-hand-side = f(t, x)
    fn rhs(&self, _t: f64, x: &Vector6<f64>, dxdt: &mut Vector6<f64>) {
        let x_na = SystemState::from_row_slice(x.as_slice());

        let mut dxdt_na = SystemState::zeros();
        dxdt_na[0] = self.u[0] * x_na[2].cos();
        dxdt_na[1] = self.u[0] * x_na[2].sin();
        dxdt_na[2] = (self.u[0] / UGV_L) * self.u[1].tan();
        dxdt_na[3] = self.u[2] * x_na[5].cos();
        dxdt_na[4] = self.u[2] * x_na[5].sin();
        dxdt_na[5] = self.u[3];

        dxdt.copy_from_slice(dxdt_na.as_slice());
    }

}
