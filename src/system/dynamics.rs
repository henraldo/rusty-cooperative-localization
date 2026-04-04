// src/system/dynamics.rs
use crate::constants::*;

pub fn f(x: &SystemState, u: &ControlInput) -> SystemState {
    let vg: f64 = u[0];
    let phi_g: f64 = u[1];
    let va: f64 = u[2];
    let omega_a: f64 = u[3];

    let theta_g: f64 = x[2];
    let theta_a: f64 = x[5];

    let mut dx: SystemState = SystemState::zeros();
    dx[0] = vg * theta_g.cos();
    dx[1] = vg * theta_g.sin();
    dx[2] = (vg / UGV_L) * phi_g.tan();
    dx[3] = va * theta_a.cos();
    dx[4] = va * theta_a.sin();
    dx[5] = omega_a;

    return dx;
}

pub fn h(x: &SystemState) -> ObservationState {
    let xg: f64 = x[0];
    let yg: f64 = x[1];
    let theta_g: f64 = x[2];

    let xa: f64 = x[3];
    let ya: f64 = x[4];
    let theta_a: f64 = x[5];

    let dx: f64 = xa - xg;
    let dy: f64 = ya - yg;
    let range: f64 = ((dx * dx) + (dy * dy)).sqrt();

    let mut y: ObservationState = ObservationState::zeros();
    y[0] = wrap_to_pi(dy.atan2(dx) - theta_g);
    y[1] = range;
    y[2] = wrap_to_pi((-dy).atan2(-dx) - theta_a);
    y[3] = xa;
    y[4] = ya;

    return y;
}

pub fn rk4_step_integrator(x: &SystemState, u: &ControlInput, dt: f64) -> SystemState {
    let k1: SystemState = f(x, u);
    let k2: SystemState = f(&(x + 0.5 * dt * k1), u);
    let k3: SystemState = f(&(x + 0.5 * dt * k2), u);
    let k4: SystemState = f(&(x + dt * k3), u);

    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}
