#![no_std]

use bradipous_curves::Curve;
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
pub struct Calibration {
    pub claw_distance_cm: f32,
    pub left_arm_cm: f32,
    pub right_arm_cm: f32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[allow(clippy::large_enum_variant)]
pub enum Cmd {
    Manual(ManualControl),
    Calibrate(Calibration),
    // Tell the bradipograph that it's currently in this position.
    SetPos(f32, f32),
    // Temporary, to test calibration
    MoveTo(f32, f32),
    Draw(Curve<32>),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Config {
    pub claw_distance: f32,
    pub spool_radius: f32,
    pub max_hang: f32,
    pub min_angle_deg: f32,
    pub steps_per_revolution: f32,
}

impl From<bradipous_geom::Config> for Config {
    fn from(c: bradipous_geom::Config) -> Self {
        Config {
            claw_distance: c.claw_distance as f32,
            spool_radius: c.spool_radius as f32,
            max_hang: c.max_hang as f32,
            min_angle_deg: c.min_angle.degrees() as f32,
            steps_per_revolution: c.steps_per_revolution as f32,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Position {
    pub x: f32,
    pub y: f32,
    pub left_arm_length: f32,
    pub right_arm_length: f32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum CalibrationStatus {
    Uncalibrated,
    Calibrated(Config),
    CalibratedAndPositioned(Config, Position),
}
