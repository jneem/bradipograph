use bradipous_geom::{Angle, FromKurbo as _, Point, RotorAngles, StepperPositions};
use bradipous_planner::{MotionCurve, PlannerConfig};
use bradipous_protocol::{Cmd, StepperSegment};
use kurbo::{BezPath, ParamCurve as _, PathSeg};

pub struct Simulation {
    pub geom: bradipous_geom::Config,
    pub max_steps_per_sec: u16,
    pub max_steps_per_sec_per_sec: u32,
    pub pen_down: bool,
    pub position: StepperPositions,
    pub steps_per_sec: u16,
}

impl Simulation {
    fn min_steps_per_sec(&self) -> u16 {
        (self.max_steps_per_sec_per_sec as f64 * 2.0).sqrt() as u16
    }

    fn mv(&mut self, to: StepperPositions, to_steps_per_sec: u16, cmds: &mut Vec<Cmd>) {
        let to_steps_per_sec = to_steps_per_sec.max(self.min_steps_per_sec());
        if self.position == to {
            return;
        }
        let right_steps = to.right as i32 - self.position.right as i32;
        let left_steps = to.left as i32 - self.position.left as i32;
        let seg = StepperSegment {
            left_steps,
            right_steps,
            start_steps_per_sec: self.steps_per_sec.max(self.min_steps_per_sec()),
            end_steps_per_sec: to_steps_per_sec,
            steps_per_sec_per_sec: self.max_steps_per_sec_per_sec,
        };
        if let Some((accel, decel)) = seg.split(self.max_steps_per_sec) {
            if accel.left_steps != 0 || accel.right_steps != 0 {
                cmds.push(Cmd::Segment(accel));
            }
            if decel.left_steps != 0 || decel.right_steps != 0 {
                cmds.push(Cmd::Segment(decel));
            }
        } else {
            cmds.push(Cmd::Segment(seg));
        }
        self.position = to;
        self.steps_per_sec = to_steps_per_sec;
    }

    pub fn move_to(&mut self, p: Point) -> Vec<Cmd> {
        self.move_to_angles(
            self.geom
                .arm_lengths_to_rotor_angles(&self.geom.point_to_arm_lengths(&p)),
        )
    }

    pub fn move_to_angles(&mut self, angles: RotorAngles) -> Vec<Cmd> {
        let mut ret = Vec::new();
        let to_steps = self.geom.rotor_angles_to_stepper_steps(&angles);

        if to_steps != self.position && self.pen_down {
            ret.push(Cmd::PenUp);
            self.pen_down = false;
        }

        self.mv(to_steps, 0, &mut ret);

        ret
    }

    pub fn draw_to_angles(&mut self, angles: RotorAngles, energy: f32) -> Vec<Cmd> {
        let mut ret = Vec::new();
        let to_steps = self.geom.rotor_angles_to_stepper_steps(&angles);

        // Energy is the square of the velocity, measured in radians per second.
        let end_steps_per_s = (energy.sqrt() * self.steps_per_radian()) as u16;

        if to_steps != self.position && !self.pen_down {
            ret.push(Cmd::PenDown);
            self.pen_down = true;
        }

        self.mv(to_steps, end_steps_per_s, &mut ret);
        ret
    }

    fn steps_per_radian(&self) -> f32 {
        self.geom.steps_per_revolution / 2.0 * std::f32::consts::PI
    }

    pub fn draw_path(&mut self, path: &BezPath, accuracy: f32) -> Vec<Cmd> {
        // The planner's coordinates are in radians.
        let max_rads_per_sec = self.max_steps_per_sec as f64 / self.steps_per_radian() as f64;
        let max_rads_per_sec_per_sec =
            self.max_steps_per_sec_per_sec as f64 / self.steps_per_radian() as f64;
        let planner_config = PlannerConfig {
            max_energy: max_rads_per_sec * max_rads_per_sec,
            max_acceleration: max_rads_per_sec_per_sec,
            accuracy: accuracy as f64,
        };

        let mut smooth_parts = bradipous_planner::smoother::SmoothParts::new(path.segments());
        let mut ret = Vec::new();
        while let Some(part) = smooth_parts.next_part() {
            let part: Vec<PathSeg> = part.collect();
            let draw_start = part[0].eval(0.0);
            ret.extend(self.move_to(Point::from_kurbo(draw_start)));

            let plan = MotionCurve::plan_one(&part, &planner_config, &self.geom);

            for (angs, energy) in plan.points.iter().zip(&plan.energies) {
                let angles = RotorAngles {
                    left: Angle::radians(angs.x as f32),
                    right: Angle::radians(angs.y as f32),
                };
                ret.extend(self.draw_to_angles(angles, *energy as f32));
            }
        }

        if self.pen_down {
            ret.push(Cmd::PenUp);
            self.pen_down = false;
        }

        ret
    }
}
