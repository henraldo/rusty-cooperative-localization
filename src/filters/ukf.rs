// src/filters/ukf.rs
use nalgebra as na;
use crate::constants::*;
use crate::system::dynamics::*;
use super::Estimator;

pub struct UKF {
    xhat: SystemState,
    p: StateCov,
    q: StateCov,
    r: MeasCov,
    ey: ObservationState,
    alpha: f64, beta: f64, kappa: f64,
}

impl UKF {
    pub fn new(
        x0: SystemState,
        p0: StateCov,
        q_filter: StateCov,
        r_filter: MeasCov,
        a: f64,
        b: f64,
        k: f64
    ) -> Self {
        UKF {
            xhat: x0,
            p: p0,
            q: q_filter,
            r: r_filter,
            ey: ObservationState::zeros(),
            alpha: a,
            beta: b,
            kappa: k,
        }
    }

    fn safe_cholesky(p: &StateCov) -> StateCov {
        match p.cholesky() {
            Some(c) => c.l(),
            None => {
                // Regularization: add small diagonal jitter when P is not PSD
                let mut p_reg = *p;
                let eps = 1e-6 * p.norm();  // adaptive epsilon
                for i in 0..N_STATES {
                    p_reg[(i, i)] += eps;
                }
                p_reg.cholesky()
                    .expect("Cholesky failed even after regularization")
                    .l()
            }
        }
    }

    fn sigma_points(&self) -> (na::DMatrix<f64>, Vec<f64>, Vec<f64>) {
        let n = N_STATES as f64;
        let lambda = self.alpha * self.alpha * (n + self.kappa) - n;
        let gamma = (n + lambda).sqrt();

        let mut sigma_points = na::DMatrix::<f64>::zeros(N_STATES, 2 * N_STATES + 1);
        sigma_points.column_mut(0).copy_from(&self.xhat);

        let svp = self.p.cholesky().unwrap().l();
        for i in 0..N_STATES {
            let col = gamma * svp.column(i);
            sigma_points.column_mut(i + 1).copy_from(&(self.xhat + col));
            sigma_points.column_mut(i + 1 + N_STATES).copy_from(&(self.xhat - col));
        }

        let mut w_m = vec![lambda / (n + lambda); 2 * N_STATES + 1];
        let mut w_c = w_m.clone();
        w_m[0] = lambda / (n + lambda);
        w_c[0] = w_m[0] + (1.0 - (self.alpha * self.alpha) + self.beta);

        return (sigma_points, w_m, w_c);
    }
}

impl Estimator for UKF {
    fn predict(&mut self, u: &ControlInput) {
        let (sigma_points, w_m, w_c) = self.sigma_points();
        let mut xhat_m = SystemState::zeros();
        let mut p_m = StateCov::zeros();

        // Propagate sigma points
        let mut sp_x = vec![SystemState::zeros(); 2 * N_STATES + 1];
        for i in 0..(2 * N_STATES + 1) {
            sp_x[i] = rk4_step_integrator(
                &sigma_points.column(i).into_owned().fixed_resize::<N_STATES, 1>(0.0), u, DT
            );
            xhat_m += w_m[i] * sp_x[i];
        }
        xhat_m[2] = wrap_to_pi(xhat_m[2]);
        xhat_m[5] = wrap_to_pi(xhat_m[5]);

        for i in 0..(2 * N_STATES + 1) {
            let d_xx = sp_x[i] - xhat_m;
            p_m += w_c[i] * (d_xx * d_xx.transpose());
        }
        self.p = p_m + self.q;
    }

    fn correct(&mut self, z: &ObservationState) {
        let (sigma_points, w_m, w_c) = self.sigma_points();
        let mut yhat = ObservationState::zeros();
        let mut p_yy = MeasCov::zeros();
        let mut p_xy = na::SMatrix::<f64, N_STATES, N_MEASUREMENTS>::zeros();

        let mut y_sig = vec![ObservationState::zeros(); 2 * N_STATES + 1];
        for i in 0..(2 * N_STATES + 1) {
            y_sig[i] = h(&sigma_points.column(i).into_owned().fixed_resize::<N_STATES, 1>(0.0));
            yhat += w_m[i] * y_sig[i];
        }
        yhat[0] = wrap_to_pi(yhat[0]);
        yhat[2] = wrap_to_pi(yhat[2]);

        for i in 0..(2 * N_STATES + 1) {
            let dy = y_sig[i] - yhat;
            let dx = sigma_points.column(i).into_owned().fixed_resize::<N_STATES, 1>(0.0) - self.xhat;
            p_yy += w_c[i] * (dy * dy.transpose());
            p_xy += w_c[i] * (dx * dy.transpose());
        }
        p_yy += self.r;

        let k = p_xy * p_yy.try_inverse().unwrap();
        self.ey = z - yhat;
        self.ey[0] = wrap_to_pi(self.ey[0]);
        self.ey[2] = wrap_to_pi(self.ey[2]);

        self.xhat += k * self.ey;
        self.xhat[2] = wrap_to_pi(self.xhat[2]);
        self.xhat[5] = wrap_to_pi(self.xhat[5]);

        self.p -= k * p_yy * k.transpose();
    }

    fn get_estimated_state(&self) -> SystemState { self.xhat }

    fn get_measurement_residuals(&self) -> ObservationState { self.ey }

    fn get_covariance_diagonal(&self) -> SystemState {
        let mut diag = SystemState::zeros();
        for i in 0..N_STATES {
            diag[i] = self.p[(i,i)];
        }
        return diag;
    }
}
