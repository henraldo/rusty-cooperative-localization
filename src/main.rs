mod constants;
mod system;
mod data;
mod filters;

use crate::constants::*;
use crate::filters::{Estimator, EKF, UKF};
use std::env;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    let filter_type = args.get(1).map(|s| s.as_str()).unwrap_or("ekf");

    println!("Beginning UAV-UGV Cooperative Localization Simulation...");
    let x0 = constants::SystemState::from_row_slice(&[10.0, 0.0, constants::PI/2.0, -60.0, 0.0, -constants::PI/2.0]);
    let u0 = constants::ControlInput::from_row_slice(&[2.0, -constants::PI/18.0, 12.0, constants::PI/25.0]);

    // Initial covariance (scaled like your main.cpp)
    let mut p0 = StateCov::zeros();
    p0[(0,0)] = 0.01; p0[(1,1)] = 0.01; p0[(2,2)] = 0.1;
    p0[(3,3)] = 0.01; p0[(4,4)] = 0.01; p0[(5,5)] = 0.1;

    // UKF settings
    let alpha: f64 = 0.001;
    let beta: f64 = 2.0;
    let kappa: f64 = 0.0;

    // let mut filter = filters::EKF::new(x0, p0, q_true(), r_true());
    let mut filter: Box<dyn Estimator> = match filter_type {
        "ukf" => Box::new(UKF::new(x0, p0, q_true(), r_true(), alpha, beta, kappa)),
        _ => Box::new(EKF::new(x0, p0, q_true(), r_true())),
    };

    // Full ground-truth generation
    let (times, x_truth, y_truth) = system::SystemModel::generate_ground_truth(500.0, &u0, &x0);

    // test record ground truth
    let mut recorder = data::DataRecorder::new("simulation_output");

    for (i, &t) in times.iter().enumerate() {
        if i > 0 {
            filter.predict(&u0);
            filter.correct(&y_truth[i])
        }

        recorder.record(t, x_truth[i], y_truth[i]);

        if i % 500 == 0 {
            let xhat = filter.get_estimated_state();
            println!("t={:.1}s  x_hat = [{:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}]",
                     t, xhat[0], xhat[1], xhat[2], xhat[3], xhat[4], xhat[5]);
        }
    }

    recorder.save_truth_data("ukf_run")?;
    println!("Simulation complete! Check the new simulation_output/ folder.");
    Ok(())

}
