//! Basic geometry of the bradipograph, including conversion from Cartesian
//! coordinates to arm lengths and stepper steps.
//!
//! We call the marker location the "head," and the two bits of string are
//! the "arms," which go from the head to the two "claws."
//!
//! This crate supports `no_std` and uses `libm` to allow for running in
//! embedded contexts. (It does use floating point, however, and so it will
//! be slow on the esp32-c3.)

#![cfg_attr(not(feature = "std"), no_std)]

use core::f64::consts::PI;

#[cfg(feature = "std")]
use kurbo::Vec2;

use kurbo::{Point, Rect};
use libm::{sqrt, tan};

fn square(x: f64) -> f64 {
    x * x
}

#[derive(Copy, Clone, Debug, PartialEq, PartialOrd, serde::Serialize, serde::Deserialize)]
pub struct Angle {
    radians: f64,
}

impl Angle {
    pub fn from_degrees(degrees: f64) -> Angle {
        Angle {
            radians: degrees * core::f64::consts::PI / 180.0,
        }
    }

    pub fn from_radians(radians: f64) -> Angle {
        Angle { radians }
    }

    pub fn radians(&self) -> f64 {
        self.radians
    }

    pub fn degrees(&self) -> f64 {
        self.radians * 180.0 / core::f64::consts::PI
    }
}

/// Lengths of the arms, from the head to the claws.
///
/// Left and right are measured from the point of view of the person
/// looking at the bradipograph.
///
/// Measured in centimeters.
#[derive(Debug)]
pub struct ArmLengths {
    pub left: f64,
    pub right: f64,
}

/// Angles of the rotors (the things that the string winds around).
///
/// A zero angle corresponds to an arm length of zero, and positive
/// angles correspond to longer arms. Since the arms will always have
/// positive length, the angles will always be positive also.
///
/// The "left" rotor is the one attached to the left arm, even though
/// it's actually the rotor on the right.
pub struct RotorAngles {
    pub left: Angle,
    pub right: Angle,
}

impl RotorAngles {
    pub fn to_point(&self) -> Point {
        Point {
            x: self.left.radians(),
            y: self.right.radians(),
        }
    }
}

/// The position of the stepper motors, measured in number of steps.
///
/// A zero value corresponds to an arm length of zero, and increasing
/// values correspond to longer arms.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct StepperPositions {
    pub left: u32,
    pub right: u32,
}

pub struct ConfigBuilder {
    min_angle: Angle,
    max_hang: f64,
    claw_distance: f64,
    spool_radius: f64,
    steps_per_revolution: f64,
    side_inset: f64,
}

impl Default for ConfigBuilder {
    fn default() -> Self {
        Self {
            min_angle: Angle::from_degrees(10.0),
            max_hang: 100.0,
            claw_distance: 100.0,
            spool_radius: 2.0,
            steps_per_revolution: 2036.0,
            side_inset: 10.0,
        }
    }
}

impl ConfigBuilder {
    pub fn build(&self) -> Config {
        Config {
            claw_distance: self.claw_distance,
            spool_radius: self.spool_radius,
            min_angle: self.min_angle,
            max_hang: self.max_hang,
            steps_per_revolution: self.steps_per_revolution,
            hang_offset: self.claw_distance * tan(self.min_angle.radians()),
            side_inset: self.side_inset,
        }
    }

    pub fn with_max_hang(&mut self, max_hang: f64) -> &mut Self {
        self.max_hang = max_hang;
        self
    }

    pub fn with_spool_radius(&mut self, spool_radius: f64) -> &mut Self {
        self.spool_radius = spool_radius;
        self
    }

    pub fn with_claw_distance(&mut self, claw_distance: f64) -> &mut Self {
        self.claw_distance = claw_distance;
        self
    }

    pub fn with_min_angle(&mut self, angle: Angle) -> &mut Self {
        self.min_angle = angle;
        self
    }

    pub fn with_steps_per_revolution(&mut self, steps: f64) -> &mut Self {
        self.steps_per_revolution = steps;
        self
    }
}

/// The geometric configuration of a bradipograph.
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct Config {
    /// The horizontal distance between claws, in centimeters. (We assume
    /// that the claws are aligned vertically.)
    pub claw_distance: f64,
    /// The radius of the spools that the string winds around.
    pub spool_radius: f64,
    /// The bradipous can't lift itself all the way up to its claws, because
    /// there would be too much tension on the arms. The min angle controls
    /// how high it can get: when dangling directly below the left claw, measure
    /// the angle between the right arm and a horizontal line. This angle
    /// is not allowed to be smaller than `min_angle`.
    ///
    /// In particular, a min angle of zero would mean that the head can come
    /// all the way up to the claws. 10 to 15 degrees seems to be a reasonable
    /// value. If the stepper motors are struggling near the top of the picture,
    /// increase the min angle.
    pub min_angle: Angle,
    /// How far below the claws is the head allowed to hang?
    pub max_hang: f64,
    /// How many stepper motor steps does it take for the spools to make
    /// one revolution?
    pub steps_per_revolution: f64,

    // Our smallest hang, determined by min_angle and claw_distance. We
    // offset our publicly-visible coordinates by this amount, so that
    // our (0, 0) coordinate is in the middle of the two claws, hanging
    // below them by `hang_offset`.
    pub hang_offset: f64,

    // We can't effectively draw right under the claws. How much horizontal
    // buffer space should we give?
    pub side_inset: f64,
}

impl Config {
    pub fn arm_lengths(&self, p: &Point) -> ArmLengths {
        let x = p.x;
        let y = p.y + self.hang_offset;
        ArmLengths {
            left: sqrt(square(self.claw_distance / 2.0 + x) + square(y)),
            right: sqrt(square(self.claw_distance / 2.0 - x) + square(y)),
        }
    }

    pub fn spool_circumference(&self) -> f64 {
        self.spool_radius * 2.0 * core::f64::consts::PI
    }

    pub fn steps_to_point(&self, steps: &StepperPositions) -> Point {
        let angles = RotorAngles {
            left: Angle::from_radians(
                f64::from(steps.left) * (2.0 * PI) / self.steps_per_revolution,
            ),
            right: Angle::from_radians(
                f64::from(steps.right) * (2.0 * PI) / self.steps_per_revolution,
            ),
        };
        let arm_lengths = self.rotor_angles_to_arm_lengths(&angles);
        self.arm_lengths_to_point(&arm_lengths)
    }

    pub fn arm_lengths_to_point(&self, lengths: &ArmLengths) -> Point {
        let x = (square(lengths.left) - square(lengths.right)) / (2.0 * self.claw_distance);
        let y = sqrt(square(lengths.left) - square(self.claw_distance / 2.0 + x));
        Point::new(x, y - self.hang_offset)
    }

    pub fn rotor_angles(&self, lengths: &ArmLengths) -> RotorAngles {
        RotorAngles {
            left: Angle::from_radians(lengths.left / self.spool_radius),
            right: Angle::from_radians(lengths.right / self.spool_radius),
        }
    }

    pub fn rotor_angles_to_arm_lengths(&self, angles: &RotorAngles) -> ArmLengths {
        ArmLengths {
            left: angles.left.radians() * self.spool_radius,
            right: angles.right.radians() * self.spool_radius,
        }
    }

    pub fn stepper_steps(&self, angles: &RotorAngles) -> StepperPositions {
        StepperPositions {
            left: libm::round(angles.left.radians() / (2.0 * PI) * self.steps_per_revolution)
                as u32,
            right: libm::round(angles.right.radians() / (2.0 * PI) * self.steps_per_revolution)
                as u32,
        }
    }

    pub fn point_to_steps(&self, p: &Point) -> StepperPositions {
        let arms = self.arm_lengths(p);
        let angles = self.rotor_angles(&arms);
        self.stepper_steps(&angles)
    }

    pub fn draw_box(&self) -> Rect {
        Rect::new(
            -self.claw_distance / 2.0 + self.side_inset,
            0.0,
            self.claw_distance / 2.0 - self.side_inset,
            self.max_hang - self.hang_offset,
        )
    }
}

#[cfg(feature = "std")]
impl bradipous_planner::Transform for Config {
    fn f(&self, input: Point) -> Point {
        self.rotor_angles(&self.arm_lengths(&input)).to_point()
    }

    fn df(&self, input: Point, direction: Vec2) -> Vec2 {
        let Point { x, y } = input;
        let f = self.f(input);
        let d = self.claw_distance / 2.0;
        let r = self.spool_radius;
        let h = self.hang_offset;
        let v = direction;
        Vec2 {
            x: (v.x * (x + d) + v.y * (y + h)) / (r * r * f.x),
            y: (v.x * (x - d) + v.y * (y + h)) / (r * r * f.y),
        }
    }

    fn ddf(&self, input: Point, v: Vec2, w: Vec2) -> Vec2 {
        let Point { x, y } = input;
        let f = self.f(input);
        let r = self.spool_radius;
        let r2f2 = Point {
            x: square(r) * square(f.x),
            y: square(r) * square(f.y),
        };
        let r3f3 = Point {
            x: r2f2.x * r * f.x,
            y: r2f2.y * r * f.y,
        };
        let d = self.claw_distance / 2.0;
        let h = self.hang_offset;

        let fx_xx = (r2f2.x - square(x + d)) / (r * r3f3.x);
        let fx_yy = (r2f2.x - square(y + h)) / (r * r3f3.x);
        let fx_xy = -(x + d) * (y + h) / (r * r3f3.x);

        let fy_xx = (r2f2.y - square(x - d)) / (r * r3f3.y);
        let fy_yy = (r2f2.y - square(y + h)) / (r * r3f3.y);
        let fy_xy = -(x - d) * (y + h) / (r * r3f3.y);

        Vec2 {
            x: fx_xx * v.x * w.x + fx_xy * (v.x * w.y + v.y * w.x) + fx_yy * v.y * w.y,
            y: fy_xx * v.x * w.x + fy_xy * (v.x * w.y + v.y * w.x) + fy_yy * v.y * w.y,
        }
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;
    use bradipous_planner::Transform;
    use proptest::prelude::*;

    impl Arbitrary for Config {
        type Parameters = ();
        type Strategy = BoxedStrategy<Config>;

        fn arbitrary_with(_: ()) -> Self::Strategy {
            (1.0..10.0f64, 1.0..10.0f64, 1.0..10.0f64)
                .prop_map(|(r, d, h)| Config {
                    claw_distance: d,
                    spool_radius: r,
                    min_angle: Angle::from_degrees(10.0),
                    max_hang: 100.0,
                    steps_per_revolution: 2036.0,
                    hang_offset: h,
                    side_inset: d / 10.0,
                })
                .boxed()
        }
    }

    proptest! {
        // Check that our formula for the gradient is correct by comparing it to the difference quotient.
        #[test]
        fn test_df(cfg: Config, x in -10.0..10.0f64, y in 1.0..100.0f64, vx in -1.0..1.0f64, vy in -1.0..1.0f64) {
            let p = Point { x, y };
            let v = Vec2 { x: vx, y: vy };
            let fp = cfg.f(p);
            let dfp = cfg.df(p, v);

            let approx_dfp = (cfg.f(p + v * 1e-2) - fp) * 1e2;

            assert!((dfp.x - approx_dfp.x).abs() < 1e-2);
            assert!((dfp.y - approx_dfp.y).abs() < 1e-2);
        }

        // Check that our formula for the second derivative is correct by comparing it to the difference quotient
        // of the gradient.
        #[test]
        fn test_ddf(cfg: Config, x in -10.0..10.0f64, y in 1.0..100.0f64, vx in -1.0..1.0f64, vy in -1.0..1.0f64, wx in -1.0..1.0f64, wy in -1.0..1.0f64) {
            let p = Point { x, y };
            let v = Vec2 { x: vx, y: vy };
            let w = Vec2 { x: wx, y: wy };

            let dfp = cfg.df(p, v);
            let ddfp = cfg.ddf(p, v, w);

            let approx_ddfp = dbg!(cfg.df(p + w * 1e-2, v) - dfp) * 1e2;
            assert!((ddfp.x - approx_ddfp.x).abs() < 1e-2);
            assert!((ddfp.y - approx_ddfp.y).abs() < 1e-2);
        }

        // Check that arm_lengths and arm_lengths_to_point are inverses.
        #[test]
        fn test_arm_length_inverse(cfg: Config, x in -10.0..10.0f64, y in 1.0..100.0f64) {
            let p = Point {x,y};
            let arms = cfg.arm_lengths(&p);
            let q = cfg.arm_lengths_to_point(&arms);
            assert!((p.x - q.x).abs() < 1e-3);
            assert!((p.y - q.y).abs() < 1e-3);
        }

        // Check that point_to_steps and steps_to_point are inverses, more or less. They won't be exact
        // inverses because steps are integer-valued, but after a round-trip to ensure that the point
        // is an integer number of steps then they should be inverses.
        #[test]
        fn test_point_step_inverse(cfg: Config, x in -10.0..10.0f64, y in 1.0..100.0f64) {
            let p = Point {x,y};
            let steps = cfg.point_to_steps(&p);
            let p = cfg.steps_to_point(&steps);
            let steps = cfg.point_to_steps(&p);
            let q = cfg.steps_to_point(&steps);
            assert!((p.x - q.x).abs() < 1e-2);
            assert!((p.y - q.y).abs() < 1e-2);
        }
    }
}
