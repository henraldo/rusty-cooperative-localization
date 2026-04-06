// src/plotting/mod.rs
use plotters::prelude::*;
use plotters::style::RGBColor;
// use crate::constants::*;
use crate::data::recorder::SimulationData;

pub fn save_plots(
    records: &[SimulationData],
    filter_type: &str,
    run_name: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let subdir = format!("{}_{}", filter_type, run_name);
    let dir = std::path::Path::new("simulation_output").join(&subdir);
    std::fs::create_dir_all(&dir)?;

    let filename = dir.join("trajectories.png");

    let root = BitMapBackend::new(filename.to_str().unwrap(), (1200, 800))
        .into_drawing_area();
    root.fill(&WHITE)?;

    let mut chart = ChartBuilder::on(&root)
        .caption(
            format!("UAV-UGV Cooperative Localization — {} ({})", filter_type.to_uppercase(), run_name),
            ("sans-serif", 30),
        )
        .margin(10)
        .x_label_area_size(40)
        .y_label_area_size(60)
        .build_cartesian_2d(-70.0..140.0, -110.0..110.0)?;

    chart.configure_mesh().draw()?;

    // Ground truth
    let ugv_true: Vec<(f64, f64)> = records.iter().map(|r| (r.x_truth[0], r.x_truth[1])).collect();
    let uav_true: Vec<(f64, f64)> = records.iter().map(|r| (r.x_truth[3], r.x_truth[4])).collect();

    chart.draw_series(LineSeries::new(ugv_true, &RED))
        .unwrap()
        .label("UGV True")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], RED));

    chart.draw_series(LineSeries::new(uav_true, &BLUE))
        .unwrap()
        .label("UAV True")
        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], BLUE));

    // Estimates
    let ugv_hat: Vec<(f64, f64)> = records.iter().map(|r| (r.xhat[0], r.xhat[1])).collect();
    let uav_hat: Vec<(f64, f64)> = records.iter().map(|r| (r.xhat[3], r.xhat[4])).collect();

    let ugv_est_color = RGBColor(139, 100, 0);
    let uav_est_color = RGBColor(0, 100, 139);

    chart.draw_series(LineSeries::new(ugv_hat, &ugv_est_color))
        .unwrap()
        .label("UGV Est")
        .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], ugv_est_color));

    chart.draw_series(LineSeries::new(uav_hat, &uav_est_color))
        .unwrap()
        .label("UAV Est")
        .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], uav_est_color));

    chart.configure_series_labels()
        .background_style(WHITE.mix(0.8))
        .border_style(BLACK)
        .draw()?;

    root.present()?;
    println!("Saved trajectory plot -> {}", filename.display());

    Ok(())
}