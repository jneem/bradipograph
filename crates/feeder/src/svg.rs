use std::path::Path;

use bradipous_geom::{Angle, RotorAngles};
use bradipous_planner::{MotionCurve, PlannerConfig};
use bradipous_protocol::{Cmd, StepperSegment};
use kurbo::{Affine, BezPath, ParamCurve as _, PathSeg, Point, Rect, Shape};
use usvg::{tiny_skia_path::PathSegment, TreeParsing};

use crate::mv;

const MAX_STEPS_PER_SEC: u32 = 400;
// what fraction of a second does it take to reach max velocity
const MAX_VELOCITY_PER_SEC: u32 = 2;

pub fn plan(
    p: &BezPath,
    initial_position: &Point,
    config: &bradipous_geom::Config,
) -> anyhow::Result<Vec<Cmd>> {
    let mut ret = Vec::new();

    let max_accel_steps = MAX_STEPS_PER_SEC * MAX_VELOCITY_PER_SEC;
    let min_steps_per_s = (max_accel_steps as f64 * 2.0).sqrt() as u16;
    let steps_per_radian = config.steps_per_revolution / 2.0 * std::f64::consts::PI;

    // The planner's coordinates are in radians.
    let max_rads_per_sec = MAX_STEPS_PER_SEC as f64 / steps_per_radian;
    let planner_config = PlannerConfig {
        max_energy: max_rads_per_sec * max_rads_per_sec,
        max_acceleration: max_rads_per_sec * MAX_VELOCITY_PER_SEC as f64,
        accuracy: 0.05,
    };

    let mut pos = *initial_position;
    let mut steps;
    let mut smooth_parts = bradipous_planner::smoother::SmoothParts::new(p.segments());

    let mut pen_is_up = true;
    while let Some(part) = smooth_parts.next_part() {
        let part: Vec<PathSeg> = part.collect();
        let draw_start = part[0].eval(0.0);
        let mut move_to = mv(config, pos, draw_start);
        if !pen_is_up && !move_to.is_empty() {
            ret.push(Cmd::PenUp);
            pen_is_up = true;
        }
        ret.append(&mut move_to);
        steps = config.point_to_steps(&draw_start);

        let mut last_steps_per_s = min_steps_per_s;
        let plan = MotionCurve::plan_one(&part, &planner_config, config);

        if pen_is_up {
            ret.push(Cmd::PenDown);
            pen_is_up = false;
        }

        for (angs, energy) in plan.points.iter().zip(&plan.energies) {
            let end_steps_per_s = ((energy.sqrt() * steps_per_radian) as u16).max(min_steps_per_s);
            let angles = RotorAngles {
                left: Angle::from_radians(angs.x),
                right: Angle::from_radians(angs.y),
            };
            let target_steps = config.stepper_steps(&angles);
            if target_steps == steps {
                // Filter out tiny segments.
                continue;
            }

            let right_steps = target_steps.right as i32 - steps.right as i32;
            let left_steps = target_steps.left as i32 - steps.left as i32;
            let seg = StepperSegment {
                left_steps,
                right_steps,
                start_steps_per_sec: last_steps_per_s,
                end_steps_per_sec: end_steps_per_s,
                steps_per_sec_per_sec: max_accel_steps,
            };
            if let Some((accel, decel)) = seg.split(MAX_STEPS_PER_SEC as u16) {
                ret.push(Cmd::Segment(accel));
                ret.push(Cmd::Segment(decel));
            } else {
                ret.push(Cmd::Segment(seg));
            }
            last_steps_per_s = end_steps_per_s;
            steps = target_steps;
        }

        pos = config.steps_to_point(&steps);
    }

    if !pen_is_up {
        ret.push(Cmd::PenUp);
    }

    Ok(ret)
}

pub fn transform(path: &mut BezPath, config: &bradipous_geom::Config) {
    let bbox = path.bounding_box();

    let w = config.claw_distance - 20.0;
    let target_bbox = Rect::new(-w / 2.0, 0.0, w / 2.0, config.max_hang);
    let scale = (target_bbox.height() / bbox.height()).min(target_bbox.width() / bbox.width());

    let transform = Affine::translate(-bbox.center().to_vec2())
        .then_scale(scale)
        .then_translate(target_bbox.center().to_vec2());

    path.apply_affine(transform);
}

pub fn load_svg(path: &Path) -> anyhow::Result<BezPath> {
    // TODO: apparently git master usvg supports text-to-path?
    let data = std::fs::read(path)?;
    let opt = usvg::Options::default();
    let tree = usvg::Tree::from_data(&data, &opt)?;
    let mut ret = BezPath::new();

    let cvt = |pt: usvg::tiny_skia_path::Point| kurbo::Point::new(pt.x as f64, pt.y as f64);

    for node in tree.root.descendants() {
        let mut bez = BezPath::new();
        if let usvg::NodeKind::Path(p) = &*node.borrow() {
            for seg in p.data.segments() {
                match seg {
                    PathSegment::MoveTo(mut pt) => {
                        p.transform.map_point(&mut pt);
                        bez.move_to(cvt(pt));
                    }
                    PathSegment::LineTo(mut pt) => {
                        p.transform.map_point(&mut pt);
                        bez.line_to(cvt(pt));
                    }
                    PathSegment::QuadTo(mut pt1, mut pt2) => {
                        p.transform.map_point(&mut pt1);
                        p.transform.map_point(&mut pt2);
                        bez.quad_to(cvt(pt1), cvt(pt2));
                    }
                    PathSegment::CubicTo(mut pt1, mut pt2, mut pt3) => {
                        p.transform.map_point(&mut pt1);
                        p.transform.map_point(&mut pt2);
                        p.transform.map_point(&mut pt3);
                        bez.curve_to(cvt(pt1), cvt(pt2), cvt(pt3));
                    }
                    PathSegment::Close => bez.close_path(),
                }
            }
        }
        ret.extend(bez);
    }
    Ok(ret)
}
