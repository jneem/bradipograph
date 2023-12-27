#![cfg_attr(not(feature = "std"), no_std)]

use bradipous_geom::{Angle, StepperPositions};
use serde::{Deserialize, Serialize};

mod stepper;
pub use stepper::StepperSegment;

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
pub enum Cmd {
    Calibrate(Calibration),
    SetMaxHang(f32),
    SetMinAngleDegrees(f32),
    Segment(StepperSegment),
    PenUp,
    PenDown,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct State {
    pub claw_distance: f32,
    pub spool_radius: f32,
    pub max_hang: f32,
    pub min_angle_deg: f32,
    pub steps_per_revolution: f32,

    pub position: StepperPositions,
    pub pen_down: bool,
}

impl State {
    pub fn geom(&self) -> bradipous_geom::Config {
        bradipous_geom::ConfigBuilder::default()
            .with_max_hang(self.max_hang.into())
            .with_spool_radius(self.spool_radius.into())
            .with_claw_distance(self.claw_distance.into())
            .with_min_angle(Angle::from_degrees(self.min_angle_deg.into()))
            .with_steps_per_revolution(self.steps_per_revolution.into())
            .build()
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum CalibrationStatus {
    Uncalibrated,
    Calibrated(State),
}
