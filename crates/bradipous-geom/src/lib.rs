#![cfg_attr(not(feature = "std"), no_std)]

use libm::{sqrtf, tanf};

fn square(x: f32) -> f32 {
    x * x
}

#[derive(Copy, Clone, Debug, PartialEq, PartialOrd)]
pub struct Angle {
    radians: f32,
}

impl Angle {
    pub fn from_degrees(degrees: f32) -> Angle {
        Angle {
            radians: degrees * core::f32::consts::PI / 180.0,
        }
    }

    pub fn from_radians(radians: f32) -> Angle {
        Angle { radians }
    }

    pub fn radians(&self) -> f32 {
        self.radians
    }
}

pub struct Rect {
    // Left edge
    pub x0: f32,
    // Top edge (we're y-down)
    pub y0: f32,
    // Right edge
    pub x1: f32,
    // Bottom edge
    pub y1: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct Point {
    pub x: f32,
    pub y: f32,
}

impl Point {
    pub fn to_arm_lengths(&self, config: &Config) -> ArmLengths {
        config.arm_lengths(self)
    }

    // TODO: implement motion_planner::Transform for this transformation
    pub fn to_rotor_angles(&self, config: &Config) -> RotorAngles {
        config.rotor_angles(&self.to_arm_lengths(config))
    }

    pub fn to_vec(&self) -> Vec2 {
        Vec2 {
            x: self.x,
            y: self.y,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl core::ops::Sub<Point> for Point {
    type Output = Vec2;

    fn sub(self, rhs: Point) -> Vec2 {
        Vec2 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl core::ops::Mul<f32> for Vec2 {
    type Output = Vec2;

    fn mul(self, factor: f32) -> Vec2 {
        Vec2 {
            x: self.x * factor,
            y: self.y * factor,
        }
    }
}

impl core::ops::Add<Vec2> for Point {
    type Output = Point;

    fn add(self, rhs: Vec2) -> Point {
        Point {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl core::ops::Add<Vec2> for Vec2 {
    type Output = Vec2;

    fn add(self, rhs: Vec2) -> Vec2 {
        Vec2 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}
impl core::ops::Sub<Vec2> for Vec2 {
    type Output = Vec2;

    fn sub(self, rhs: Vec2) -> Vec2 {
        Vec2 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

pub struct ArmLengths {
    pub left: f32,
    pub right: f32,
}

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

// TODO: might be fun to add some typestate to make sure things are initialized properly...
pub struct ConfigBuilder {
    min_angle: Angle,
    max_hang: f32,
    claw_distance: f32,
    spool_radius: f32,
}

impl Default for ConfigBuilder {
    fn default() -> Self {
        Self {
            min_angle: Angle::from_degrees(10.0),
            max_hang: 100.0,
            claw_distance: 100.0,
            spool_radius: 2.0,
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
            hang_offset: self.claw_distance * tanf(self.min_angle.radians()),
        }
    }

    /// Hanging calibration works like this: the claws are positioned an unknown
    /// distance apart (perfectly horizontal from one another). We manually
    /// position the head directly below one claw and measure the distance to
    /// that claw. Then we manually position the head the exact same distance
    /// below the other claw. We record the change in the arm lengths when moving
    /// from one of these positions to the other (by symmetry, it should be the
    /// same for both arms).
    ///
    /// `hang` is the distance that the head was hanging below the claws, and
    /// `arm_change` is the change in arm lengths when moving from one hanging
    /// position to the other.
    pub fn with_hanging_calibration(&mut self, hang: f32, arm_change: f32) -> &mut Self {
        self.claw_distance = sqrtf(square(arm_change) + 2.0 * arm_change * hang);
        self
    }

    pub fn with_max_hang(&mut self, max_hang: f32) -> &mut Self {
        self.max_hang = max_hang;
        self
    }

    pub fn with_spool_radius(&mut self, spool_radius: f32) -> &mut Self {
        self.spool_radius = spool_radius;
        self
    }
}

#[derive(Clone, Debug)]
pub struct Config {
    pub claw_distance: f32,
    pub spool_radius: f32,
    pub min_angle: Angle,
    pub max_hang: f32,

    // Our smallest hang, determined by min_angle and claw_distance. We
    // offset our publicly-visible coordinates by this amount, so that
    // our (0, 0) coordinate is in the middle of the two claws, hanging
    // below them by `hang_offset`.
    hang_offset: f32,
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
            left: sqrtf(square(self.claw_distance / 2.0 + x) + square(y)),
            right: sqrtf(square(self.claw_distance / 2.0 - x) + square(y)),
        }
    }

    pub fn rotor_angles(&self, lengths: &ArmLengths) -> RotorAngles {
        RotorAngles {
            left: Angle::from_radians(lengths.left / self.spool_radius),
            right: Angle::from_radians(lengths.right / self.spool_radius),
        }
    }
}

pub trait Transform {
    fn f(&self, input: Point) -> Point;
    fn df(&self, input: Point, direction: Vec2) -> Vec2;
    fn ddf(&self, input: Point, u: Vec2, v: Vec2) -> Vec2;
}

impl Transform for Config {
    fn f(&self, input: Point) -> Point {
        input.to_rotor_angles(self).to_point()
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
            (1.0..10.0f32, 1.0..10.0f32, 1.0..10.0f32)
                .prop_map(|(r, d, h)| Config {
                    claw_distance: d,
                    spool_radius: r,
                    min_angle: Angle::from_degrees(10.0),
                    max_hang: 100.0,
                    hang_offset: h,
                })
                .boxed()
        }
    }

    proptest! {
        // Check that our formula for the gradient is correct by comparing it to the difference quotient.
        #[test]
        fn test_df(cfg: Config, x in -10.0..10.0f32, y in 1.0..100.0f32, vx in -1.0..1.0f32, vy in -1.0..1.0f32) {
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
        fn test_ddf(cfg: Config, x in -10.0..10.0f32, y in 1.0..100.0f32, vx in -1.0..1.0f32, vy in -1.0..1.0f32, wx in -1.0..1.0f32, wy in -1.0..1.0f32) {
            let p = Point { x, y };
            let v = Vec2 { x: vx, y: vy };
            let w = Vec2 { x: wx, y: wy };

            let dfp = cfg.df(p, v);
            let ddfp = cfg.ddf(p, v, w);

            let approx_ddfp = dbg!(cfg.df(p + w * 1e-2, v) - dfp) * 1e2;
            assert!((ddfp.x - approx_ddfp.x).abs() < 1e-2);
            assert!((ddfp.y - approx_ddfp.y).abs() < 1e-2);
        }
    }
}
