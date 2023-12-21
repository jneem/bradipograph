#![cfg_attr(not(feature = "std"), no_std)]

use bradipous_geom::StepperPositions;
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
    Calibrate(Calibration),
    Segment(StepperSegment),
    PenUp,
    PenDown,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StepperSegment {
    pub left_steps: i32,
    pub right_steps: i32,
    pub start_steps_per_sec: u16,
    pub end_steps_per_sec: u16,
}

impl StepperSegment {
    /// If a segment is long and the initial and final velocities are both small, the step
    /// iterator will just move along it very slowly. This is probably not what we want,
    /// though: we want to accelerate to a higher maximum speed, and then decelerate at
    /// the end.
    ///
    /// This method splits a single segment into two: an acceleration segment and a deceleration
    /// segment.
    #[cfg(feature = "std")]
    pub fn split(
        &self,
        max_steps_per_sec: u16,
        max_steps_per_sec_per_sec: u32,
    ) -> Option<(StepperSegment, StepperSegment)> {
        let start_v = self.start_steps_per_sec as i32;
        let end_v = self.end_steps_per_sec as i32;
        let max_v = max_steps_per_sec as i32;
        let max_steps_per_sec_per_sec = max_steps_per_sec_per_sec as i32;

        let square = |x| x * x;
        // How many steps would it take to get from the initial to the final velocity?
        let steps = (square(start_v) - square(end_v)).abs() / (2 * max_steps_per_sec_per_sec);

        let max_steps = self.left_steps.abs().max(self.right_steps.abs());
        if max_steps == 0 {
            // Avoid dividing by zero.
            return None;
        }
        if steps <= max_steps / 2 && (start_v <= max_v * 3 / 4 || end_v <= max_v * 3 / 4) {
            // 2 * max_steps * self.accel.steps_per_s_per_s is the total amount that the squared
            // velocity can change while traversing this segment. Some of that change must be used
            // up in getting from the initial velocity to the final velocity. And then the peak energy
            // as we traverse the segment is half of what's left (since we need to go up and then down).
            let max_energy =
                max_steps * max_steps_per_sec_per_sec - (square(start_v) - square(end_v)).abs() / 2;
            let max_velocity = ((max_energy as f64).sqrt() as i32).min(max_v);

            // How may steps will it take to get from the max velocity back down to the final velocity?
            let decel_steps =
                (square(max_velocity) - square(end_v)) / (2 * max_steps_per_sec_per_sec);

            let decel_seg = StepperSegment {
                left_steps: self.left_steps * decel_steps / max_steps,
                right_steps: self.right_steps * decel_steps / max_steps,
                start_steps_per_sec: max_velocity as u16,
                end_steps_per_sec: self.end_steps_per_sec,
            };
            let accel_seg = StepperSegment {
                left_steps: self.left_steps - decel_seg.left_steps,
                right_steps: self.right_steps - decel_seg.right_steps,
                start_steps_per_sec: self.start_steps_per_sec,
                end_steps_per_sec: decel_seg.start_steps_per_sec,
            };
            Some((accel_seg, decel_seg))
        } else {
            None
        }
    }
}

// TODO: clarify the relationship between this config and bradipous_geom::Config.
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

impl From<Config> for bradipous_geom::Config {
    fn from(c: Config) -> Self {
        bradipous_geom::ConfigBuilder::default()
            .with_max_hang(c.max_hang as f64)
            .with_spool_radius(c.spool_radius as f64)
            .with_claw_distance(c.claw_distance as f64)
            .build()
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum CalibrationStatus {
    Uncalibrated,
    Calibrated(Config, StepperPositions),
}
