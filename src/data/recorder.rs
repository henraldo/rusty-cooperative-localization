// src/data/recorder.rs
use std::fs;
use std::path::Path;
use csv::WriterBuilder;
use crate::constants::*;

#[derive(Debug)]
pub struct SimulationData {
    pub time: f64,
    pub x_truth: SystemState,
    pub y_truth: ObservationState,
    pub xhat: SystemState,
    pub ey: ObservationState,
    pub cov_diag: SystemState,
}

pub struct DataRecorder {
    pub sim_data: Vec<SimulationData>,
    output_dir: String,
}

impl DataRecorder {
    pub fn new(output_dir: &str) -> Self {
        DataRecorder {
            sim_data: Vec::new(),
            output_dir: output_dir.to_string(),
        }
    }

    /// Records timestep
    ///
    /// * `time` - current simulation time
    /// * `x_truth` - ground truth state
    /// * `y_truth` - measurement truth
    /// * `xhat` - state estimates
    /// * `ey` - measurement residuals
    /// * `cov_diag` - covariance diagonal
    pub fn record(
        &mut self,
        time: f64,
        x_truth: SystemState,
        y_truth: ObservationState,
        xhat: SystemState,
        ey: ObservationState,
        cov_diag: SystemState,
    ) {
        self.sim_data.push(SimulationData { time, x_truth, y_truth, xhat, ey, cov_diag });
    }

    /// Writes simulation data to CSV
    /// * `run_name` - prefix for csv file name
    pub fn save(&self, run_name: &str, filter_type: &str) -> Result<(), Box<dyn std::error::Error>> {
        let subdir = format!("{}_{}", filter_type, run_name);
        let dir = Path::new(&self.output_dir).join(&subdir);
        fs::create_dir_all(&dir)?;

        let filename: std::path::PathBuf = dir.join("simulation_data.csv");
        let mut wtr = WriterBuilder::new().has_headers(true).from_path(filename)?;

        wtr.write_record(&[
            "time",
            "east_g", "north_g", "heading_g", "east_a", "north_a", "heading_a",
            "bearing_g", "range", "bearing_a", "xa_meas", "ya_meas",
            "xg_hat", "yg_hat", "heading_g_hat", "xa_hat", "ya_hat", "heading_a_hat",
            "e_bearing_g", "e_range", "e_bearing_a", "e_xa_meas", "e_ya_meas",
            "p_xg", "p_yg", "p_heading_g", "p_xa", "p_ya", "p_heading_a"
        ])?;

        for r in &self.sim_data {
            let mut row = vec![r.time.to_string()];

            // ground truth states
            for i in 0..N_STATES {
                row.push(r.x_truth[i].to_string());
            }

            // measurements truth
            for i in 0..N_MEASUREMENTS {
                row.push(r.y_truth[i].to_string());
            }

            // estimated states
            for i in 0..N_STATES {
                row.push(r.xhat[i].to_string());
            }

            // measurement residuals
            for i in 0..N_MEASUREMENTS {
                row.push(r.ey[i].to_string());
            }

            // covariance diagonal
            for i in 0..N_STATES {
                row.push(r.cov_diag[i].to_string());
            }

            wtr.write_record(&row)?;
        }

        wtr.flush()?;
        println!("Saved data to CSV -> {}/{}_simulation_data.csv", self.output_dir, run_name);
        Ok(())
    }
}
