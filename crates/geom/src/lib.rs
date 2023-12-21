#![cfg_attr(not(feature = "std"), no_std)]

use core::f64::consts::PI;

use bradipous_planner::Transform;
use kurbo::{Point, Rect, Vec2};
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

#[derive(Debug)]
pub struct ArmLengths {
    pub left: f64,
    pub right: f64,
}

pub struct RotorAngles {
    pub left: Angle,
    pub right: Angle,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct StepperPositions {
    pub left: u32,
    pub right: u32,
}

impl RotorAngles {
    pub fn to_point(&self) -> Point {
        Point {
            x: self.left.radians(),
            y: self.right.radians(),
        }
    }
}

// TODO: might be fun to add some typestate to make sure things are initialized properly...
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

    /// Hanging calibration works like this: the claws are positioned an unknown
    /// distance apart (perfectly horizontal from one another). We manually
    /// position the head at a known position relative to the left claw, and
    /// then move it to the symmetric position relative to the right claw.
    /// We record the change in the arm lengths when moving from one of these
    /// positions to the other (by symmetry, it should be the same for both
    /// arms).
    ///
    /// `hang` is the distance that the head was hanging below the claws, `x_offset`
    /// is the distance that the head was to the side of the claws, and
    /// `arm_change` is the change in arm lengths when moving from one hanging
    /// position to the other.
    pub fn with_hanging_calibration(
        &mut self,
        hang: f64,
        x_offset: f64,
        arm_change: f64,
    ) -> &mut Self {
        let a = sqrt(square(hang) + square(x_offset));
        self.claw_distance =
            x_offset + sqrt(square(x_offset) + square(arm_change) + 2.0 * arm_change * a);
        self
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
}

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct Config {
    pub claw_distance: f64,
    pub spool_radius: f64,
    pub min_angle: Angle,
    pub max_hang: f64,
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
    pub fn draw_box(&self) -> Rect {
        Rect {
            x0: -self.claw_distance / 2.0,
            x1: self.claw_distance / 2.0,
            y0: 0.0,
            y1: self.max_hang,
        }
    }

    pub fn arm_lengths(&self, p: &Point) -> ArmLengths {
        let x = p.x;
        let y = p.y + self.hang_offset;
        ArmLengths {
            left: sqrt(square(self.claw_distance / 2.0 + x) + square(y)),
            right: sqrt(square(self.claw_distance / 2.0 - x) + square(y)),
        }
    }

    pub fn steps_to_point(&self, steps: &StepperPositions) -> Point {
        let arm_lengths = ArmLengths {
            left: f64::from(steps.left) * (2.0 * PI) / self.steps_per_revolution,
            right: f64::from(steps.right) * (2.0 * PI) / self.steps_per_revolution,
        };
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

    pub fn bbox(&self) -> Rect {
        Rect::new(
            -self.claw_distance / 2.0 + self.side_inset,
            0.0,
            self.claw_distance / 2.0 - self.side_inset,
            self.max_hang,
        )
    }
}

impl Transform for Config {
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
            dbg!(p, q, arms);
            assert!((p.x - q.x).abs() < 1e-3);
            assert!((p.y - q.y).abs() < 1e-3);
        }
    }
}
