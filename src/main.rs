mod constants;
mod system;
mod data;
mod filters;

use crate::constants::*;
use crate::filters::{Estimator, EKF, UKF};
use std::env;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    let filter_type: &str = args.get(1).map(|s| s.as_str()).unwrap_or("ekf");

    println!("Beginning UAV-UGV Cooperative Localization Simulation...");
    let x0: SystemState = constants::SystemState::from_row_slice(&[10.0, 0.0, constants::PI/2.0, -60.0, 0.0, -constants::PI/2.0]);
    let u0: ControlInput = constants::ControlInput::from_row_slice(&[2.0, -constants::PI/18.0, 12.0, constants::PI/25.0]);

    // Initial covariance
    let mut p0: StateCov = StateCov::zeros();
    p0[(0,0)] = 0.9; p0[(1,1)] = 0.9; p0[(2,2)] = 0.1;
    p0[(3,3)] = 1.5; p0[(4,4)] = 1.5; p0[(5,5)] = 0.1;

    // UKF settings
    let alpha: f64 = 0.001;
    let beta: f64 = 2.0;
    let kappa: f64 = 0.0;

    let mut filter: Box<dyn Estimator> = match filter_type {
        "ukf" => Box::new(UKF::new(x0, p0, q_true(), r_true(), alpha, beta, kappa)),
        _ => Box::new(EKF::new(x0, p0, q_true(), r_true())),
    };

    // Ground truth generation
    let (times, x_truth, y_truth) = system::SystemModel::generate_ground_truth(500.0, &u0, &x0);

    // Run Filter Simulation
    let mut recorder: data::DataRecorder = data::DataRecorder::new("simulation_output");
    for (i, &t) in times.iter().enumerate() {
        if i > 0 {
            filter.predict(&u0);
            filter.correct(&y_truth[i])
        }

        recorder.record(
            t,
            x_truth[i],
            y_truth[i],
            filter.get_estimated_state(),
            filter.get_measurement_residuals(),
            filter.get_covariance_diagonal()
        );
    }

    recorder.save("run", filter_type)?;
    println!("Simulation complete! Check for file in simulation_output/ folder.");
    Ok(())

}
