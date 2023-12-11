#![no_std]

use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ManualControl {
    ShortenLeft,
    LengthenLeft,
    StopLeft,
    ShortenRight,
    LengthenRight,
    StopRight,
}

#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Calibrate {
    MarkLeft,
    Finish { y_offset: f32, x_offset: f32 },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Cmd {
    Manual(ManualControl),
    Calibrate(Calibrate),
    // Tell the bradipograph that it's currently in this position.
    SetPos(f32, f32),
    // Temporary, to test calibration
    MoveTo(f32, f32),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Calibration {
    pub claw_distance: f32,
    pub spool_radius: f32,
    pub max_hang: f32,
    // Commented out for now, because until bleps supports larger responses
    // we need to keep the size down...
    //pub min_angle_deg: f32,
    //pub steps_per_revolution: f32,
}

impl From<bradipous_geom::Config> for Calibration {
    fn from(c: bradipous_geom::Config) -> Self {
        Calibration {
            claw_distance: c.claw_distance as f32,
            spool_radius: c.spool_radius as f32,
            max_hang: c.max_hang as f32,
            //min_angle_deg: c.min_angle.degrees() as f32,
            //steps_per_revolution: c.steps_per_revolution as f32,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Position {
    pub x: f32,
    pub y: f32,
    pub left: u32,
    pub right: u32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum CalibrationStatus {
    Uncalibrated,
    Calibrated(Calibration),
    //CalibratedAndPositioned(Calibration, Position),
}
