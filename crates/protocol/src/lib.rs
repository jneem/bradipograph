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
pub enum Calibration {
    MarkLeft,
    Finish { y_offset: f32, x_offset: f32 },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Cmd {
    Manual(ManualControl),
    Calibrate(Calibration),
    // Temporary, to test calibration
    MoveTo(f32, f32),
}
