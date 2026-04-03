// src/system/model.rs
use nalgebra as na;
use rand::prelude::*;
use rand_distr::StandardNormal;
use crate::constants::*;
use crate::system::dynamics::*;

pub struct SystemModel;

impl SystemModel {
    pub fn generate_ground_truth(total_time: f64, u: &ControlInput, x0: &SystemState) -> (Vec<f64>, Vec<SystemState>, Vec<ObservationState>) {
        let num_steps = (total_time / DT).round() as usize;

        let mut times: Vec<f64> = Vec::with_capacity(num_steps + 1);
        let mut x_truth:Vec<SystemState> = Vec::with_capacity(num_steps + 1);
        let mut y_truth:Vec<ObservationState> = Vec::with_capacity(num_steps + 1);

        let mut x: SystemState = *x0;
        let mut y: ObservationState = h(&x);
        let mut rng: ThreadRng = thread_rng();

        let q: StateCov = q_true();
        let r: MeasCov = r_true();
        let svx: StateCov = q.cholesky().expect("Q must be positive semi-definite").l();
        let svy: MeasCov = r.cholesky().expect("R must be positive semi-definite").l();

        // Generate trajectory and add AWGN
        let z_q: SystemState = na::SVector::from_iterator(
            (0..N_STATES).map(|_| rng.sample(StandardNormal))
        );
        let z_r: ObservationState = na::SVector::from_iterator(
            (0..N_MEASUREMENTS).map(|_| rng.sample(StandardNormal))
        );

        times.push(0.0);
        x_truth.push(x + (svx * z_q));
        y_truth.push(y + (svy * z_r));

        for step in 1..=num_steps {
            x = rk4_step_integrator(&x, u, DT);
            x[2] = wrap_to_pi(x[2]);
            x[5] = wrap_to_pi(x[5]);

            y = h(&x);

            let z_q: SystemState = na::SVector::from_iterator(
                (0..N_STATES).map(|_| rng.sample(StandardNormal))
            );
            let z_r: ObservationState = na::SVector::from_iterator(
                (0..N_MEASUREMENTS).map(|_| rng.sample(StandardNormal))
            );

            times.push(step as f64 * DT);
            x_truth.push(x + (svx * z_q));
            y_truth.push(y + (svy * z_r));
        }

        return (times, x_truth, y_truth);
    }
}
