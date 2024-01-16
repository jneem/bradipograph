//! Basic geometry of the bradipograph, including conversion from Cartesian
//! coordinates to arm lengths and stepper steps.
//!
//! We call the marker location the "head," and the two bits of string are
//! the "arms," which go from the head to the two "claws."
//!
//! This crate supports `no_std` and uses `libm` to allow for running in
//! embedded contexts. (It does use floating point, however, and so it will
//! be slow on the esp32-c3.)
//!
//! It might be nice to use a "units" crate. uom seems to be the most mature,
//! but it doesn't support using libm in place of std.

#![cfg_attr(not(feature = "std"), no_std)]

use core::f32::consts::PI;
use libm::{sqrtf, tanf};

pub type Angle = euclid::Angle<f32>;
pub type Point = euclid::Point2D<f32, Cm>;

pub struct Steps;
pub struct Cm;

pub type Len = euclid::Length<f32, Cm>;

fn square<T: core::ops::Mul<T> + Copy>(x: T) -> <T as core::ops::Mul<T>>::Output {
    x * x
}

pub trait LenExt {
    fn cm(self) -> Len;
}

impl LenExt for f32 {
    fn cm(self) -> Len {
        Len::new(self)
    }
}

pub trait FromKurbo {
    type Input;
    fn from_kurbo(p: Self::Input) -> Self;
}

#[cfg(feature = "kurbo")]
impl FromKurbo for Point {
    type Input = kurbo::Point;
    fn from_kurbo(p: kurbo::Point) -> Self {
        Point::new(p.x as f32, p.y as f32)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct LeftRight<T> {
    pub left: T,
    pub right: T,
}

/// Lengths of the arms, from the head to the claws.
///
/// Left and right are measured from the point of view of the person
/// looking at the bradipograph.
///
/// Measured in centimeters.
pub type ArmLengths = LeftRight<Len>;

/// Angles of the rotors (the things that the string winds around).
///
/// A zero angle corresponds to an arm length of zero, and positive
/// angles correspond to longer arms. Since the arms will always have
/// positive length, the angles will always be positive also.
///
/// The "left" rotor is the one attached to the left arm, even though
/// it's actually the rotor on the right.
pub type RotorAngles = LeftRight<Angle>;

impl RotorAngles {
    #[cfg(feature = "kurbo")]
    pub fn to_kurbo_point(&self) -> kurbo::Point {
        kurbo::Point {
            x: self.left.get().into(),
            y: self.right.get().into(),
        }
    }
}

/// The position of the stepper motors, measured in number of steps.
///
/// A zero value corresponds to an arm length of zero, and increasing
/// values correspond to longer arms.
pub type StepperPositions = LeftRight<u32>;

pub struct ConfigBuilder {
    min_angle: Angle,
    max_hang: Len,
    claw_distance: Len,
    spool_radius: Len,
    steps_per_revolution: f32,
    side_inset: Len,
}

impl Default for ConfigBuilder {
    fn default() -> Self {
        Self {
            min_angle: Angle::degrees(10.0),
            max_hang: 100.0.cm(),
            claw_distance: 100.0.cm(),
            spool_radius: 1.3.cm(),
            steps_per_revolution: 2036.0,
            side_inset: 10.0.cm(),
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
            hang_offset: self.claw_distance * tanf(self.min_angle.get()),
            side_inset: self.side_inset,
        }
    }

    pub fn with_max_hang(&mut self, max_hang: Len) -> &mut Self {
        self.max_hang = max_hang;
        self
    }

    pub fn with_spool_radius(&mut self, spool_radius: Len) -> &mut Self {
        self.spool_radius = spool_radius;
        self
    }

    pub fn with_claw_distance(&mut self, claw_distance: Len) -> &mut Self {
        self.claw_distance = claw_distance;
        self
    }

    pub fn with_min_angle(&mut self, angle: Angle) -> &mut Self {
        self.min_angle = angle;
        self
    }

    pub fn with_steps_per_revolution(&mut self, steps: f32) -> &mut Self {
        self.steps_per_revolution = steps;
        self
    }
}

/// The geometric configuration of a bradipograph.
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct Config {
    /// The horizontal distance between claws, in centimeters. (We assume
    /// that the claws are aligned vertically.)
    pub claw_distance: Len,
    /// The radius of the spools that the string winds around.
    pub spool_radius: Len,
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
    pub max_hang: Len,
    /// How many stepper motor steps does it take for the spools to make
    /// one revolution?
    pub steps_per_revolution: f32,

    // Our smallest hang, determined by min_angle and claw_distance. We
    // offset our publicly-visible coordinates by this amount, so that
    // our (0, 0) coordinate is in the middle of the two claws, hanging
    // below them by `hang_offset`.
    pub hang_offset: Len,

    // We can't effectively draw right under the claws. How much horizontal
    // buffer space should we give?
    pub side_inset: Len,
}

impl Config {
    fn recalculate_hang_offset(&mut self) {
        self.hang_offset = self.claw_distance * tanf(self.min_angle.get());
    }

    pub fn set_min_angle(&mut self, angle: Angle) {
        self.min_angle = angle;
        self.recalculate_hang_offset();
    }

    pub fn set_claw_distance(&mut self, d: Len) {
        self.claw_distance = d;
        self.recalculate_hang_offset();
    }

    pub fn point_to_arm_lengths(&self, p: &Point) -> ArmLengths {
        let x = p.x;
        let y = p.y + self.hang_offset.get();
        let d = self.claw_distance.get();
        ArmLengths {
            left: sqrtf(square(d / 2.0 + x) + square(y)).cm(),
            right: sqrtf(square(d / 2.0 - x) + square(y)).cm(),
        }
    }

    pub fn spool_circumference(&self) -> Len {
        self.spool_radius * 2.0 * core::f32::consts::PI
    }

    pub fn steps_to_point(&self, steps: &StepperPositions) -> Point {
        let angles = RotorAngles {
            left: Angle::radians(steps.left as f32 * (2.0 * PI) / self.steps_per_revolution),
            right: Angle::radians(steps.right as f32 * (2.0 * PI) / self.steps_per_revolution),
        };
        let arm_lengths = self.rotor_angles_to_arm_lengths(&angles);
        self.arm_lengths_to_point(&arm_lengths)
    }

    pub fn arm_lengths_to_point(&self, lengths: &ArmLengths) -> Point {
        let d = self.claw_distance.get();
        let l = lengths.left.get();
        let r = lengths.right.get();
        let x = (square(l) - square(r)) / (2.0 * d);
        let y = sqrtf(square(l) - square(d / 2.0 + x));
        Point::new(x, y - self.hang_offset.get())
    }

    pub fn arm_lengths_to_rotor_angles(&self, lengths: &ArmLengths) -> RotorAngles {
        RotorAngles {
            left: Angle::radians((lengths.left / self.spool_radius).get()),
            right: Angle::radians((lengths.right / self.spool_radius).get()),
        }
    }

    pub fn rotor_angles_to_arm_lengths(&self, angles: &RotorAngles) -> ArmLengths {
        ArmLengths {
            left: self.spool_radius * angles.left.get(),
            right: self.spool_radius * angles.right.get(),
        }
    }

    pub fn rotor_angles_to_stepper_steps(&self, angles: &RotorAngles) -> StepperPositions {
        StepperPositions {
            left: libm::roundf(angles.left.get() / (2.0 * PI) * self.steps_per_revolution) as u32,
            right: libm::roundf(angles.right.get() / (2.0 * PI) * self.steps_per_revolution) as u32,
        }
    }

    pub fn point_to_steps(&self, p: &Point) -> StepperPositions {
        let arms = self.point_to_arm_lengths(p);
        let angles = self.arm_lengths_to_rotor_angles(&arms);
        self.rotor_angles_to_stepper_steps(&angles)
    }

    #[cfg(feature = "kurbo")]
    pub fn draw_box(&self) -> kurbo::Rect {
        let w = self.claw_distance.get() as f64 / 2.0 - self.side_inset.get() as f64;
        let h = self.max_hang.get() as f64 - self.hang_offset.get() as f64;
        kurbo::Rect::new(-w, 0.0, w, h)
    }
}

#[cfg(feature = "kurbo")]
impl bradipous_planner::Transform for Config {
    fn f(&self, input: kurbo::Point) -> kurbo::Point {
        self.arm_lengths_to_rotor_angles(&self.point_to_arm_lengths(&Point::from_kurbo(input)))
            .to_kurbo_point()
    }

    fn df(&self, input: kurbo::Point, direction: kurbo::Vec2) -> kurbo::Vec2 {
        let kurbo::Point { x, y } = input;
        let f = self.f(input);
        let d = self.claw_distance.get() as f64 / 2.0;
        let r = self.spool_radius.get() as f64;
        let h = self.hang_offset.get() as f64;
        let v = direction;
        kurbo::Vec2 {
            x: (v.x * (x + d) + v.y * (y + h)) / (r * r * f.x),
            y: (v.x * (x - d) + v.y * (y + h)) / (r * r * f.y),
        }
    }

    fn ddf(&self, input: kurbo::Point, v: kurbo::Vec2, w: kurbo::Vec2) -> kurbo::Vec2 {
        let kurbo::Point { x, y } = input;
        let f = self.f(input);
        let r = self.spool_radius.get() as f64;
        let r2f2 = kurbo::Point {
            x: square(r) * square(f.x),
            y: square(r) * square(f.y),
        };
        let r3f3 = kurbo::Point {
            x: r2f2.x * r * f.x,
            y: r2f2.y * r * f.y,
        };
        let d = self.claw_distance.get() as f64 / 2.0;
        let h = self.hang_offset.get() as f64;

        let fx_xx = (r2f2.x - square(x + d)) / (r * r3f3.x);
        let fx_yy = (r2f2.x - square(y + h)) / (r * r3f3.x);
        let fx_xy = -(x + d) * (y + h) / (r * r3f3.x);

        let fy_xx = (r2f2.y - square(x - d)) / (r * r3f3.y);
        let fy_yy = (r2f2.y - square(y + h)) / (r * r3f3.y);
        let fy_xy = -(x - d) * (y + h) / (r * r3f3.y);

        kurbo::Vec2 {
            x: fx_xx * v.x * w.x + fx_xy * (v.x * w.y + v.y * w.x) + fx_yy * v.y * w.y,
            y: fy_xx * v.x * w.x + fy_xy * (v.x * w.y + v.y * w.x) + fy_yy * v.y * w.y,
        }
    }
}

#[cfg(all(test, feature = "kurbo"))]
mod tests {
    use super::*;
    use bradipous_planner::Transform;
    use proptest::prelude::*;

    impl Arbitrary for Config {
        type Parameters = ();
        type Strategy = BoxedStrategy<Config>;

        fn arbitrary_with(_: ()) -> Self::Strategy {
            (1.0..10.0f32, 1.0..10.0f32, 1.0..10.0f32)
                .prop_map(|(r, d, h)| Config {
                    claw_distance: d.cm(),
                    spool_radius: r.cm(),
                    min_angle: Angle::degrees(10.0),
                    max_hang: 100.0.cm(),
                    steps_per_revolution: 2036.0,
                    hang_offset: h.cm(),
                    side_inset: d.cm() / 10.0,
                })
                .boxed()
        }
    }

    proptest! {
        // Check that our formula for the gradient is correct by comparing it to the difference quotient.
        #[test]
        fn test_df(cfg: Config, x in -10.0..10.0f64, y in 1.0..100.0f64, vx in -1.0..1.0f64, vy in -1.0..1.0f64) {
            let p = kurbo::Point { x, y };
            let v = kurbo::Vec2 { x: vx, y: vy };
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
            let p = kurbo::Point { x, y };
            let v = kurbo::Vec2 { x: vx, y: vy };
            let w = kurbo::Vec2 { x: wx, y: wy };

            let dfp = cfg.df(p, v);
            let ddfp = cfg.ddf(p, v, w);

            let approx_ddfp = dbg!(cfg.df(p + w * 1e-2, v) - dfp) * 1e2;
            assert!((ddfp.x - approx_ddfp.x).abs() < 1e-2);
            assert!((ddfp.y - approx_ddfp.y).abs() < 1e-2);
        }

        // Check that arm_lengths and arm_lengths_to_point are inverses.
        #[test]
        fn test_arm_length_inverse(cfg: Config, x in -10.0..10.0f32, y in 1.0..100.0f32) {
            let p = Point::new(x,y);
            let arms = cfg.point_to_arm_lengths(&p);
            let q = cfg.arm_lengths_to_point(&arms);
            assert!((p.x - q.x).abs() < 1e-2);
            assert!((p.y - q.y).abs() < 1e-2);
        }

        // Check that point_to_steps and steps_to_point are inverses, more or less. They won't be exact
        // inverses because steps are integer-valued, but after a round-trip to ensure that the point
        // is an integer number of steps then they should be inverses.
        #[test]
        fn test_point_step_inverse(cfg: Config, x in -10.0..10.0f32, y in 1.0..100.0f32) {
            let p = Point::new(x,y);
            let steps = cfg.point_to_steps(&p);
            let p = cfg.steps_to_point(&steps);
            let steps = cfg.point_to_steps(&p);
            let q = cfg.steps_to_point(&steps);
            assert!((p.x - q.x).abs() < 1e-2);
            assert!((p.y - q.y).abs() < 1e-2);
        }
    }
}
