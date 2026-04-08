// src/plotting/mod.rs
use plotly::{
    common::{Mode, Title},
    layout::{Axis, Layout, Legend},
    Plot, Scatter,
};
use crate::constants::*;
use crate::data::recorder::SimulationData;
use std::path::*;

pub fn save_plots(
    records: &[SimulationData],
    filter_type: &str,
    run_name: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let subdir: String = format!("{}_{}", filter_type, run_name);
    let dir: PathBuf = Path::new("simulation_output").join(&subdir);
    std::fs::create_dir_all(&dir)?;

    // Trajectory plot
    let mut trajectory: Plot = Plot::new();

    // Ground truth
    let ugv_true_x: Vec<f64> = records.iter().map(|r: &SimulationData| r.x_truth[0]).collect();
    let ugv_true_y: Vec<f64> = records.iter().map(|r: &SimulationData| r.x_truth[1]).collect();
    let uav_true_x: Vec<f64> = records.iter().map(|r: &SimulationData| r.x_truth[3]).collect();
    let uav_true_y: Vec<f64> = records.iter().map(|r: &SimulationData| r.x_truth[4]).collect();

    let ugv_true = Scatter::new(ugv_true_x, ugv_true_y)
        .mode(Mode::Lines)
        .name("UGV Truth")
        .line(plotly::common::Line::new().color("#F54927"));

    let uav_true = Scatter::new(uav_true_x, uav_true_y)
        .mode(Mode::Lines)
        .name("UAV Truth")
        .line(plotly::common::Line::new().color("#308DC7"));

    // Estimates
    let xhat0: Vec<f64> = records.iter().map(|r: &SimulationData| r.xhat[0]).collect();
    let xhat1: Vec<f64> = records.iter().map(|r: &SimulationData| r.xhat[1]).collect();
    let xhat3: Vec<f64> = records.iter().map(|r: &SimulationData| r.xhat[3]).collect();
    let xhat4: Vec<f64> = records.iter().map(|r: &SimulationData| r.xhat[4]).collect();

    let ugv_est = Scatter::new(xhat0, xhat1)
        .mode(Mode::Lines)
        .name("UGV Truth")
        .line(plotly::common::Line::new().color("#89B52B").dash(plotly::common::DashType::Dash));

    let uav_est = Scatter::new(xhat3, xhat4)
        .mode(Mode::Lines)
        .name("UAV Truth")
        .line(plotly::common::Line::new().color("#B0135F").dash(plotly::common::DashType::Dash));

    trajectory.add_trace(ugv_true);
    trajectory.add_trace(uav_true);
    trajectory.add_trace(ugv_est);
    trajectory.add_trace(uav_est);

    let traj_layout = Layout::new()
        .title(Title::new().text(&format!("UAV-UGV Cooperative Localization — {} ({})", filter_type.to_uppercase(), run_name)))
        .x_axis(Axis::new().title("East (m)"))
        .y_axis(Axis::new().title("North (m)"))
        .legend(Legend::new().x(0.0).y(1.0));

    trajectory.set_layout(traj_layout);
    let traject_file = dir.join("trajectories.html");
    trajectory.write_html(traject_file.to_str().unwrap());
    println!("Saved interactive trajectory plot -> {}", traject_file.display());

    // Measurement Residuals
    let mut residuals: Plot = Plot::new();


    Ok(())
}