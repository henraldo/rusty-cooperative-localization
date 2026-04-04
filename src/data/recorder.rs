// src/data/recorder.rs
use std::fs;
use std::path::Path;
use csv::WriterBuilder;
use nalgebra as na;
use crate::constants::*;
use crate::system::model::SystemModel;

#[derive(Debug)]
pub struct TruthData {
    pub time: f64,
    pub x_truth: SystemState,
    pub y_truth: ObservationState,
}

pub struct DataRecorder {
    truth_data: Vec<TruthData>,
    output_dir: String,
}

impl DataRecorder {
    pub fn new(output_dir: &str) -> Self {
        DataRecorder {
            truth_data: Vec::new(),
            output_dir: output_dir.to_string(),
        }
    }

    /// records timestep
    pub fn record(&mut self, time: f64, x_truth: SystemState, y_truth: ObservationState) {
        self.truth_data.push(TruthData { time, x_truth, y_truth });
    }

    /// Writes truth data to CSV
    pub fn save_truth_data(&self, run_name: &str) -> Result<(), Box<dyn std::error::Error>> {
        let dir = Path::new(&self.output_dir);
        fs::create_dir_all(dir)?;

        let filename = dir.join(format!("{}_truth_data.csv", run_name));
        let mut wtr = WriterBuilder::new().has_headers(true).from_path(filename)?;

        wtr.write_record(&[
            "time",
            "east_g", "north_g", "heading_g", "east_a", "north_a", "heading_a",  // state
            "bearing_g", "range", "bearing_a", "xa_meas", "ya_meas"  // measurements
        ])?;

        for r in &self.truth_data {
            let mut row = vec![r.time.to_string()];
            for i in 0..N_STATES {
                row.push(r.x_truth[i].to_string());
            }

            for i in 0..N_MEASUREMENTS {
                row.push(r.y_truth[i].to_string());
            }
            wtr.write_record(&row)?;
        }

        wtr.flush()?;
        println!("Saved data to CSV -> {}/{}_truth_data.csv", self.output_dir, run_name);
        Ok(())
    }
}
