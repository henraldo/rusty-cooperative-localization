// src/filters/ekf.rs
use crate::constants::*;
use crate::system::dynamics::*;
use super::Estimator;

pub struct EKF {
    xhat: SystemState,
    P: StateCov,
    Q: StateCov,
    R: MeasCov,
    ey: ObservationState,
    omega: StateCov,
}

impl EKF {
    /// Instantiates Extended Kalman Filter object
    ///
    /// * `x0` - inital states
    /// * `p0` - initial filter covariance
    /// * `q_filter` - filter process noise covariance
    /// * `r_filter` - filter measurement noise covariance
    pub fn new(x0: SystemState, p0: StateCov, q_filter: StateCov, r_filter: MeasCov) -> Self {
        EKF {
            xhat: x0,
            P: p0,
            Q: q_filter,
            R: r_filter,
            ey: ObservationState::zeros(),
            omega: omega(),
        }
    }

    /// Returns linearized observation sensativity matrix via calculating Jacobian of nonlinear dynamics model
    ///
    /// * `u` - control input vector
    fn jacobian_f(&self, u: &ControlInput) -> StateTransition {
        let mut F = StateTransition::zeros();
        let theta_g = self.xhat[2];
        let theta_a = self.xhat[5];
        let vg = u[0];
        let va = u[2];

        F[(0,2)] = -vg * theta_g.sin();
        F[(1,2)] = vg * theta_g.cos();
        F[(3,5)] = -va * theta_a.sin();
        F[(4,5)] = va * theta_a.cos();

        return F;
    }

    /// Returns linearized observation sensativity matrix via calculating Jacobian of nonlinear measurment model
    fn jacobian_h(&self) -> ObsSensativity {
        let mut H: ObsSensativity = ObsSensativity::zeros();
        let xg = self.xhat[0]; let yg = self.xhat[1];
        let xa = self.xhat[3]; let ya = self.xhat[4];
        let dx = xa - xg; let dy = ya - yg;
        let r2 = (dx * dx) + (dy * dy);
        let r = r2.sqrt();

        if r2 < 1e-12 { return H; }

        // UGV bearing row
        let d_bear_g_dx = -dy / r2;
        let d_bear_g_dy = dx / r2;
        H[(0,0)] = d_bear_g_dx;
        H[(0,1)] = d_bear_g_dy;
        H[(0,2)] = -1.0;
        H[(0,3)] = -d_bear_g_dx;
        H[(0,4)] = -d_bear_g_dy;

        // range row
        let d_range_dx = -dx / r;
        let d_range_dy = -dy /r;
        H[(1,0)] = d_range_dx;
        H[(1,1)] = d_range_dy;
        H[(1,3)] = -d_range_dx;
        H[(1,4)] = -d_range_dy;

        // UAV bearing row
        let d_bear_a_dx = dy / r2;
        let d_bear_a_dy = -dx / r2;
        H[(2,0)] = d_bear_a_dx;
        H[(2,1)] = d_bear_a_dy;
        H[(2,3)] = -d_bear_a_dx;
        H[(2,4)] = -d_bear_a_dy;
        H[(2,5)] = -1.0;

        // UAV position rows
        H[(3,3)] = 1.0;
        H[(4,4)] = 1.0;

        return H;
    }
}

impl Estimator for EKF {

    /// Predict states and covariance for next measurement update
    ///
    /// * `u` - control input vector
    fn predict(&mut self, u: &ControlInput) {
        // Propagate nonlinear system dynamics model
        self.xhat = rk4_step_integrator(&self.xhat, u, DT);

        // Linearize and predict covariance
        let F: StateTransition = self.jacobian_f(u);
        self.P = F * self.P * F.transpose() + (self.omega * self.Q * self.omega.transpose());
    }

    /// Corrects state predictions from latest measurements
    ///
    /// * `z` - latest measurements
    fn correct(&mut self, z: &ObservationState) {
        let H: ObsSensativity = self.jacobian_h();
        let yhat: ObservationState = h(&self.xhat);

        self.ey = z - yhat;
        self.ey[0] = wrap_to_pi(self.ey[0]);
        self.ey[2] = wrap_to_pi(self.ey[2]);

        let S = H * self.P * H.transpose() + self.R;
        let K = self.P * H.transpose() * S.try_inverse().expect("S invertable");

        self.xhat += K * self.ey;
        self.xhat[2] = wrap_to_pi(self.xhat[2]);
        self.xhat[5] = wrap_to_pi(self.xhat[5]);

        let I: StateCov = StateCov::identity();
        self.P = (I - K * H) * self.P * (I - K * H).transpose() + K * self.R * K.transpose();
    }

    /// Returns last computed state estimates
    fn get_estimated_state(&self) -> SystemState { self.xhat }

    /// Returns last computed measurement errors
    fn get_measurement_residuals(&self) -> ObservationState { self.ey }

    /// Returns last computed filter covariance diagonal
    fn get_covariance_diagonal(&self) -> SystemState {
        let mut diag: SystemState = SystemState::zeros();
        for i in 0..N_STATES {
            diag[i] = self.P[(i,i)];
        }
        return diag;
    }
}
