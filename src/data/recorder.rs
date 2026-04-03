// src/data/recorder.rs
use std::fs;
use std::path::Path;
use csv::WriterBuilder;
use nalgebra as na;
use crate::constants::*;
use crate::system::model::SystemModel;

#[derive(Debug)]
pub struct SimulationHistory {
    pub time: f64,
    pub x_truth: SystemState,
    pub y_truth: ObservationState,
}

pub struct DataRecorder {
    time_history: Vec<SimulationHistory>,
    output_dir: String,
}

impl DataRecorder {
    pub fn new(output_dir: &str) -> Self {
        DataRecorder {
            time_history: Vec::new(),
            output_dir: output_dir.to_string(),
        }
    }

    pub fn record(&mut self, time: f64, x_true: SystemState, y_true: ObservationState) {
        self.time_history.push(SimulationHistory { time, x_true, y_true });
    }

    pub fn save_data(&self, run_name: &str) -> Result<(), Box<dyn std::error::Error>> {
        let dir = Path::new(&self.output_dir);
        fs::create_dir_all(dir)?;

        let filename = dir.join(format!("{}_simulation_data.csv", run_name));
        let mut wtr = WriterBuilder::new().has_headers(true).from_path(filename)?;

        wtr.write_record(&[
            "time",
            "xg", "yg", "thetag", "xa", "ya", "thetaa",  // state
            "bearing_g", "range", "bearing_a", "xa_meas", "ya_meas"  // measurements
        ])?;

        for r in &self.time_history {
            let mut row = vec![r.time.to_string()];
            for i in 0..N_STATES {
                row.push(r.x_true[i].to_string());
            }

            for i in 0..N_MEASUREMENTS {
                row.push(r.y_true[i].to_string());
            }
            wtr.write_record(&row)?;
        }

        wtr.flush()?;
        println!("Saved data to CSV -> {}/{}_simulation_data.csv", self.output_dir, run_name);
        Ok(())
    }
}
