// src/filters/ekf.rs
use crate::constants::*;
use crate::system::dynamics::*;
use super::Estimator;

pub struct EKF {
    xhat: SystemState,
    p: StateCov,
    q: StateCov,
    r: MeasCov,
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
            p: p0,
            q: q_filter,
            r: r_filter,
            ey: ObservationState::zeros(),
            omega: omega(),
        }
    }

    fn jacobian_f(&self, u: &ControlInput) -> StateTransition {
        let mut f = StateTransition::zeros();
        let theta_g = self.xhat[2];
        let theta_a = self.xhat[5];
        let vg = u[0];
        let va = u[2];

        f[(0,2)] = -vg * theta_g.sin();
        f[(1,2)] = vg * theta_g.cos();
        f[(3,5)] = -va * theta_a.sin();
        f[(4,5)] = va * theta_a.cos();

        return f;
    }

    fn jacobian_h(&self) -> ObsSensativity {
        let mut h = ObsSensativity::zeros();
        let xg = self.xhat[0]; let yg = self.xhat[1];
        let xa = self.xhat[3]; let ya = self.xhat[4];
        let dx = xa - xg; let dy = ya - yg;
        let r2 = (dx * dx) + (dy * dy);
        let r = r2.sqrt();

        if r2 < 1e-12 { return h; }

        // UGV bearing row
        let d_bear_g_dx = -dy / r2;
        let d_bear_g_dy = dx / r2;
        h[(0,0)] = d_bear_g_dx;
        h[(0,1)] = d_bear_g_dy;
        h[(0,2)] = -1.0;
        h[(0,3)] = -d_bear_g_dx;
        h[(0,4)] = -d_bear_g_dy;

        // range row
        let d_range_dx = -dx / r;
        let d_range_dy = -dy /r;
        h[(1,0)] = d_range_dx;
        h[(1,1)] = d_range_dy;
        h[(1,3)] = -d_range_dx;
        h[(1,4)] = -d_range_dy;

        // UAV bearing row
        let d_bear_a_dx = dy / r2;
        let d_bear_a_dy = -dx / r2;
        h[(2,0)] = d_bear_a_dx;
        h[(2,1)] = d_bear_a_dy;
        h[(2,3)] = -d_bear_a_dx;
        h[(2,4)] = -d_bear_a_dy;
        h[(2,5)] = -1.0;

        // UAV position rows
        h[(3,3)] = 1.0;
        h[(4,4)] = 1.0;

        return h;
    }
}

impl Estimator for EKF {

    // Predict states and covariance for next measurement update
    fn predict(&mut self, u: &ControlInput) {
        // Propagate nonlinear system dynamics model
        self.xhat = rk4_step_integrator(&self.xhat, u, DT);

        // Linearize and predict covariance
        let f = self.jacobian_f(u);
        self.p = f * self.p * f.transpose() + (self.omega * self.q * self.omega.transpose());
    }

    // Corrects predictions from latest measurements
    fn correct(&mut self, z: &ObservationState) {
        let h_til = self.jacobian_h();
        let yhat = h(&self.xhat);

        self.ey = z - yhat;
        self.ey[0] = wrap_to_pi(self.ey[0]);
        self.ey[2] = wrap_to_pi(self.ey[2]);

        let s = h_til * self.p * h_til.transpose() + self.r;
        let k = self.p * h_til.transpose() * s.try_inverse().expect("S invertable");

        self.xhat += k * self.ey;
        self.xhat[2] = wrap_to_pi(self.xhat[2]);
        self.xhat[5] = wrap_to_pi(self.xhat[5]);

        let i = StateCov::identity();
        self.p = (i - k * h_til) * self.p * (i - k * h_til).transpose() + k * self.r * k.transpose();
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
