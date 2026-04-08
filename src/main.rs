mod constants;
mod system;
mod data;
mod filters;
mod plotting;

use crate::constants::*;
use crate::filters::{Estimator, EKF, UKF};
use clap::Parser;

#[derive(Parser)]
struct Args {
    #[arg(short, long, default_value = "ekf")]
    filter: String,
    #[arg(short, long, default_value = "run1")]
    run_name: String,
    #[arg(short, long, default_value_t = 500.0)]
    time: f64,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Args = Args::parse();

    println!("Beginning UAV-UGV Cooperative Localization Simulation...");

    // Initial states and control inputs
    let mut x0: SystemState = SystemState::from_row_slice(&[10.0, 0.0, PI/2.0, -60.0, 0.0, -PI/2.0]);
    let u0: ControlInput = ControlInput::from_row_slice(&[2.0, -PI/18.0, 12.0, PI/25.0]);

    // Perturbation to initial states
    let dx0: SystemState = SystemState::from_row_slice(&[-0.01, 0.05, 0.0, -0.5, 0.5, 0.0]);

    // Initial covariance
    let mut p0: StateCov = StateCov::zeros();
    p0[(0,0)] = 0.9; p0[(1,1)] = 0.9; p0[(2,2)] = 0.1;
    p0[(3,3)] = 1.5; p0[(4,4)] = 1.5; p0[(5,5)] = 0.1;

    // UKF settings
    let alpha: f64 = 0.001;
    let beta: f64 = 2.0;
    let kappa: f64 = 0.0;

    let mut filter: Box<dyn Estimator> = match args.filter.as_str() {
        "ukf" => Box::new(UKF::new(x0, p0, q_true(), r_true(), alpha, beta, kappa)),
        _ => Box::new(EKF::new(x0, p0, q_true(), r_true())),
    };

    // Ground truth generation
    x0 += dx0;
    let (times, x_truth, y_truth) = system::SystemModel::generate_ground_truth(args.time, &u0, &x0);

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

    if let Err(e) = crate::plotting::save_plots(
        &recorder.sim_data, args.filter.as_str(), args.run_name.as_str()
    ) {
        eprintln!("Warning: could not save plot: {}", e);
    }

    recorder.save(args.run_name.as_str(), args.filter.as_str())?;
    println!("Simulation complete! Check for file in simulation_output/ folder.");
    Ok(())

}
