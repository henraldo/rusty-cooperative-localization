// src/filters/estimator.rs
use crate::constants::*;

pub trait Estimator {
    fn predict(&mut self, u: &ControlInput);
    fn correct(&mut self, z: &ObservationState);
    fn get_estimated_state(&self) -> SystemState;
    fn get_covariance_diagonal(&self) -> SystemState;
    fn get_measurement_residuals(&self) -> ObservationState;
}
