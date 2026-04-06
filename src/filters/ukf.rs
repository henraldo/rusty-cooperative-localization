// src/filters/ukf.rs
use nalgebra as na;
use crate::constants::*;
use crate::system::dynamics::*;
use super::Estimator;

pub type SigmaPoints = na::SMatrix<f64, N_STATES, {2 * N_STATES + 1}>;
pub type SigmaPointsY = na::SMatrix<f64, N_MEASUREMENTS, {2 * N_STATES + 1}>;
pub type Weights = na::SVector<f64, {2 * N_STATES + 1}>;

pub struct UKF {
    xhat: SystemState, ey: ObservationState, sp_x: SigmaPoints,
    P: StateCov, Q: StateCov, R: MeasCov,
    weights_mean: Weights, weights_covar: Weights,
    alpha: f64, beta: f64, kappa: f64,
    L: usize,
}

impl UKF {
    /// Instantiates Unscented Kalman Filter object
    ///
    /// * `x0` - inital states
    /// * `p0` - initial filter covariance
    /// * `q_filter` - filter process noise covariance
    /// * `r_filter` - filter measurement noise covariance
    /// * `a` - alpha parameter for unscented transform
    /// * `b` - beta parameter for unscented transform
    /// * `k` - kappa parameter for unscented transform
    pub fn new(
        x0: SystemState,
        p0: StateCov,
        q_filter: StateCov,
        r_filter: MeasCov,
        a: f64,
        b: f64,
        k: f64,
    ) -> Self {
        UKF {
            xhat: x0,
            ey: ObservationState::zeros(),
            sp_x: SigmaPoints::zeros(),
            P: p0,
            Q: q_filter,
            R: r_filter,
            weights_mean: Weights::zeros(),
            weights_covar: Weights::zeros(),
            alpha: a,
            beta: b,
            kappa: k,
            L: (2 * N_STATES) + 1,
        }
    }

    /// Calculates filter lambda scaling value from UKF configuration
    fn calculate_lambda(&self) -> f64 {
        let n: f64 = N_STATES as f64;
        return (self.alpha * self.alpha) * (n + self.kappa) - n;
    }

    /// Updates mean and covariance weighting values
    fn update_weights(&mut self) {
        let n: f64 = N_STATES as f64;
        let lambda: f64 = self.calculate_lambda();

        self.weights_mean[0] = lambda / (n + lambda);
        self.weights_covar[0] = self.weights_mean[0] + (1.0 - (self.alpha * self.alpha) + self.beta);
        let w: f64 = 1.0 / (2.0 * (n + lambda));

        for i in 1..self.L as usize {
            self.weights_mean[i] = w;
            self.weights_covar[i] = w;
        }
    }

    /// Generates new set of Sigma points from the latest state estimates and filter covariance
    fn compute_sigma_points(&mut self) {
        let n: f64 = N_STATES as f64;
        self.update_weights();

        let gamma: f64 = (n + self.calculate_lambda()).sqrt();
        let svp: StateCov = self.P.cholesky().expect("P must be PSD").l();

        self.sp_x.column_mut(0).copy_from(&self.xhat);
        for i in 0..N_STATES {
            let col: SystemState = gamma * svp.column(i);

            self.sp_x.column_mut(i + 1).copy_from(&(self.xhat + col));
            self.sp_x[(2, i)] = wrap_to_pi(self.sp_x[(2, i)]);
            self.sp_x[(5, i)] = wrap_to_pi(self.sp_x[(5, i)]);

            self.sp_x.column_mut(i + 1 + N_STATES).copy_from(&(self.xhat - col));
            self.sp_x[(2, i + 1 + N_STATES)] = wrap_to_pi(self.sp_x[(2, i + 1 + N_STATES)]);
            self.sp_x[(5, i + 1 + N_STATES)] = wrap_to_pi(self.sp_x[(5, i + 1 + N_STATES)]);
        }
    }
}

impl Estimator for UKF {

    /// Predict states and covariance for next measurement update
    ///
    /// * `u` - control input vector
    fn predict(&mut self, u: &ControlInput) {
        self.compute_sigma_points();

        for i in 0..self.L {
            let mut xhat_m: SystemState = rk4_step_integrator(&self.xhat, u, DT);
            xhat_m[2] = wrap_to_pi(xhat_m[2]);
            xhat_m[5] = wrap_to_pi(xhat_m[5]);

            self.sp_x.column_mut(i).copy_from(&xhat_m);
        }

        self.xhat = self.sp_x * self.weights_mean;
        self.xhat[2] = wrap_to_pi(self.xhat[2]);
        self.xhat[5] = wrap_to_pi(self.xhat[5]);

        self.P = StateCov::zeros();
        for j in 0..self.L {
            let mut d_xx: SystemState = self.sp_x.column(j) - self.xhat;
            d_xx[2] = wrap_to_pi(d_xx[2]);
            d_xx[5] = wrap_to_pi(d_xx[5]);

            self.P += self.weights_covar[j] * (d_xx * d_xx.transpose());
        }
        self.P += self.Q;
    }

    /// Corrects state predictions from latest measurements
    ///
    /// * `z` - latest measurements
    fn correct(&mut self, z: &ObservationState) {
        let mut sp_y: SigmaPointsY = SigmaPointsY::zeros();

        let mut x: SystemState = SystemState::zeros();
        let mut yhat: ObservationState;
        for i in 0..self.L {
            x.copy_from(&self.sp_x.column(i));
            yhat = h(&x);
            sp_y.set_column(i, &yhat);
        }

        yhat = sp_y * self.weights_mean;
        yhat[0] = wrap_to_pi(yhat[0]);
        yhat[2] = wrap_to_pi(yhat[2]);

        self.ey = z - yhat;
        self.ey[0] = wrap_to_pi(self.ey[0]);
        self.ey[2] = wrap_to_pi(self.ey[2]);

        let mut Pyy: MeasCov = MeasCov::zeros();
        let mut d_yy: ObservationState = ObservationState::zeros();
        for j in 0..self.L {
            d_yy.copy_from(&sp_y.column(j));
            d_yy -= yhat;
            d_yy[0] = wrap_to_pi(d_yy[0]);
            d_yy[2] = wrap_to_pi(d_yy[2]);

            Pyy += self.weights_covar[j] * (d_yy * d_yy.transpose());
        }
        Pyy += self.R;

        let mut Pxy = na::SMatrix::<f64, N_STATES, N_MEASUREMENTS>::zeros();
        let mut dx: SystemState = SystemState::zeros();
        let mut dy: ObservationState = ObservationState::zeros();
        for i in 0..self.L {
            dx.copy_from(&self.sp_x.column(i));
            dx -= self.xhat;
            dx[2] = wrap_to_pi(dx[2]);
            dx[5] = wrap_to_pi(dx[5]);

            dy.copy_from(&sp_y.column(i));
            dy -= yhat;
            dy[0] = wrap_to_pi(dy[0]);
            dy[2] = wrap_to_pi(dy[2]);

            Pxy += self.weights_covar[i] * (dx * dy.transpose());
        }

        let K = Pxy * Pyy.transpose();
        self.xhat += K * self.ey;
        self.P -= K * Pyy * K.transpose();

        self.xhat[2] = wrap_to_pi(self.xhat[2]);
        self.xhat[5] = wrap_to_pi(self.xhat[5]);

    }

    /// Returns last computed state estimates
    fn get_estimated_state(&self) -> SystemState { self.xhat }

    /// Returns last computed measurement errors
    fn get_measurement_residuals(&self) -> ObservationState { self.ey }

    /// Returns last computed filter covariance diagonal
    fn get_covariance_diagonal(&self) -> SystemState {
        let mut diag = SystemState::zeros();
        for i in 0..N_STATES {
            diag[i] = self.P[(i,i)];
        }
        diag
    }
}