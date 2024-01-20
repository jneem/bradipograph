#![no_std]

use serde::{Deserialize, Serialize};

mod stepper;
use bradipo_geom::{Angle, ArmLengths, Len, StepperPositions};
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
    pub claw_distance: Len,
    pub arm_lengths: ArmLengths,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Cmd {
    Calibrate(Calibration),
    SetMaxHang(Len),
    SetMinAngle(Angle),
    Segment(StepperSegment),
    PenUp,
    PenDown,
}

// TODO: maybe separated the "derived" part of the config from the original
// config, so that we don't need to list the fields here.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct State {
    pub claw_distance: Len,
    pub spool_radius: Len,
    pub max_hang: Len,
    pub min_angle: Angle,
    pub steps_per_revolution: f32,

    pub position: StepperPositions,
    pub pen_down: bool,
}

impl State {
    pub fn geom(&self) -> bradipo_geom::Config {
        bradipo_geom::ConfigBuilder::default()
            .with_max_hang(self.max_hang)
            .with_spool_radius(self.spool_radius)
            .with_claw_distance(self.claw_distance)
            .with_min_angle(self.min_angle)
            .with_steps_per_revolution(self.steps_per_revolution)
            .build()
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum CalibrationStatus {
    Uncalibrated,
    Calibrated(State),
}
