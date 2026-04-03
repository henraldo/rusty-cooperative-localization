//! src/main.rs
mod constants;
mod system;
// mod utils;
// mod system;

fn main() {
    println!("Rust project ready! Dynamics + measurement models loaded.");
    let x0 = constants::SystemState::from_row_slice(&[10.0, 0.0, constants::PI/2.0, -60.0, 0.0, -constants::PI/2.0]);
    let u0 = constants::ControlInput::from_row_slice(&[2.0, -constants::PI/18.0, 12.0, constants::PI/25.0]);

    // Full ground-truth generation (exactly your C++ call)
    let (times, x_truth, y_truth) = system::SystemModel::generate_ground_truth(500.0, &u0, &x0);

    println!("Generated {} steps of ground truth", times.len());
    println!("Final x_truth: {:?}", x_truth.last().unwrap());
    println!("Final y_truth (first 3 elements): {:?}", &y_truth.last().unwrap().as_slice()[0..3]);
}