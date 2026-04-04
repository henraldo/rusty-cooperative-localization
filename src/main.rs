mod constants;
mod system;
mod data;
mod filters;

use crate::constants::*;
use crate::filters::Estimator;
use tokio::sync::mpsc;
use tokio::time::{interval, Duration};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Beginning UAV-UGV Cooperative Localization Simulation...");
    let x0 = constants::SystemState::from_row_slice(&[10.0, 0.0, constants::PI/2.0, -60.0, 0.0, -constants::PI/2.0]);
    let u0 = constants::ControlInput::from_row_slice(&[2.0, -constants::PI/18.0, 12.0, constants::PI/25.0]);

    // Initial covariance & filter instantiation
    let mut p0 = StateCov::zeros();
    p0[(0,0)] = 0.9; p0[(1,1)] = 0.9; p0[(2,2)] = 0.1;
    p0[(3,3)] = 1.5; p0[(4,4)] = 1.5; p0[(5,5)] = 0.1;

    let mut filter = filters::EKF::new(x0, p0, q_true(), r_true());

    // Setup channel for async sensor input (time + x_truth + y_meas)
    let (tx, mut rx) = mpsc::channel::<(f64, SystemState, ObservationState)>(1024);

    // ** Sim Task: runs fast (50 Hz replay) **
    let sim_handle = tokio::spawn(async move {
        let (times, x_truth, y_truth) = system::SystemModel::generate_ground_truth(500.0, &u0, &x0);

        for (i, &t) in times.iter().enumerate() {
            if i > 0 {
                // Sim replay runs "faster" - 50 Hz replay --> sleep 20ms
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
            tx.send((t, x_truth[i], y_truth[i])).await.unwrap();
        }
        println!("Sim task finished.");
    });

    // ** Filter Task: runs at slower rate (10 Hz update rate) **
    let mut recorder = data::DataRecorder::new("simulation_output");
    let mut filter_interval = interval(Duration::from_millis(100));

    println!("Filter task started — processing at 10 Hz...");

    loop {
        tokio::select! {
            Some((t, x_true, y_true)) = rx.recv() => {
                filter.predict(&u0);
                filter.correct(&y_true);

                recorder.record(t, x_true, y_true);
            }

             // Filter rate tick
            _ = filter_interval.tick() => {
                let xhat = filter.get_estimated_state();
            }
        }

        if rx.is_closed() && rx.try_recv().is_err() {
            break;
        }
    }


    // for (i, &t) in times.iter().enumerate() {
    //     if i > 0 {
    //         filter.predict(&u0);
    //         filter.correct(&y_truth[i])
    //     }

    //     recorder.record(t, x_truth[i], y_truth[i]);

    //     // Quick console peek
    //     if i % 500 == 0 {
    //         let xhat = filter.get_estimated_state();
    //         println!("t={:.1}s  x_hat = [{:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2}]",
    //                  t, xhat[0], xhat[1], xhat[2], xhat[3], xhat[4], xhat[5]);
    //     }
    // }

    recorder.save_truth_data("ekf_run")?;

    // Wait for sim to finish cleanly
    let _ = sim_handle.await;
    println!("Simulation complete! Check the new simulation_output/ folder.");
    Ok(())

}
