#![cfg_attr(not(feature = "std"), no_std)]

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

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct State {
    pub geom: bradipo_geom::ConfigBuilder,
    pub position: StepperPositions,
    pub pen_down: bool,
}

impl State {
    pub fn geom(&self) -> bradipo_geom::Config {
        self.geom.build()
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum CalibrationStatus {
    Uncalibrated,
    Calibrated(State),
}
