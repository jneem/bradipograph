use std::{
    fs::File,
    path::{Path, PathBuf},
};

use bradipous_planner::{MotionCurve, PlannerConfig, Transform};
use clap::Parser as _;
use kurbo::{Affine, BezPath, Point, Rect, Shape, Vec2};
use usvg::{tiny_skia_path::PathSegment, TreeParsing};

#[derive(clap::Parser)]
struct Args {
    path: PathBuf,

    #[arg(long, default_value_t = 0.01)]
    accuracy: f64,

    #[arg(long, default_value_t = std::f64::consts::PI * 8.)]
    max_acceleration: f64,

    #[arg(long, default_value_t = std::f64::consts::PI)]
    max_velocity: f64,

    #[arg(long)]
    output: PathBuf,
}

pub fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    let mut paths = load_svg(&args.path)?;

    let bbox = paths
        .iter()
        .map(Shape::bounding_box)
        .reduce(|b1, b2| b1.union(b2))
        .unwrap();

    let target_bbox = BrachioTransform::input_range();
    let scale = (target_bbox.height() / bbox.height()).min(target_bbox.width() / bbox.width());

    let transform = Affine::translate(-bbox.center().to_vec2())
        // SVG has positive y pointing down; we have positive y pointing up.
        .then_scale_non_uniform(1.0, -1.0)
        .then_scale(scale)
        .then_translate(target_bbox.center().to_vec2());

    for path in &mut paths {
        path.apply_affine(transform);
    }

    let config = PlannerConfig {
        max_energy: args.max_velocity * args.max_velocity,
        max_acceleration: args.max_acceleration,
        accuracy: args.accuracy,
    };

    let plans = paths
        .iter_mut()
        .map(|p| MotionCurve::<1024>::plan(p, &config, &BrachioTransform))
        .collect::<Vec<_>>();
    let out = File::create(&args.output)?;

    serde_json::to_writer(out, &plans)?;

    Ok(())
}

fn load_svg(path: &Path) -> anyhow::Result<Vec<BezPath>> {
    // TODO: apparently git master usvg supports text-to-path?
    let data = std::fs::read(path)?;
    let opt = usvg::Options::default();
    let tree = usvg::Tree::from_data(&data, &opt)?;
    let mut ret = Vec::new();

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
        if !bez.is_empty() {
            ret.push(bez);
        }
    }
    Ok(ret)
}

#[derive(Clone, Copy)]
pub struct BrachioTransform;

impl BrachioTransform {
    pub fn input_range() -> Rect {
        Rect::new(-1.0, 3.0 / 8.0, 1.0, 13.0 / 8.0)
    }
}

impl Transform for BrachioTransform {
    fn f(&self, input: Point) -> Point {
        let theta = input.to_vec2().atan2();
        let r2 = input.to_vec2().hypot2();
        let elbow = (1.0 - r2 / 2.0).clamp(-1.0, 1.0).asin();
        let shoulder = 3.0 * std::f64::consts::PI / 4.0 - theta + elbow / 2.0;
        Point::new(elbow, shoulder)
    }

    fn df(&self, input: Point, direction: Vec2) -> Vec2 {
        let r = input.to_vec2().hypot();
        let r2 = input.to_vec2().hypot2();
        let Point { x, y } = input;
        let elbow_x = -x / (r * (1. - r2 / 4.).sqrt());
        let elbow_y = -y / (r * (1. - r2 / 4.).sqrt());
        let shoulder_x = y / r2 + elbow_x / 2.;
        let shoulder_y = -x / r2 + elbow_y / 2.;

        let Vec2 { x: dx, y: dy } = direction;
        Vec2::new(
            elbow_x * dx + elbow_y * dy,
            shoulder_x * dx + shoulder_y * dy,
        )
    }

    fn ddf(&self, input: Point, u: Vec2, v: Vec2) -> Vec2 {
        let r = input.to_vec2().hypot();
        let r2 = input.to_vec2().hypot2();
        let Point { x, y } = input;

        let denom = r2 * r * (1. - r2 / 4.).powf(1.5);
        let elbow_xx = (x * x * (1. - r2 / 2.) - r2 * (1. - r2 / 4.)) / denom;
        let elbow_yy = (y * y * (1. - r2 / 2.) - r2 * (1. - r2 / 4.)) / denom;
        let elbow_xy = x * y * (1. - r2 / 2.) / denom;

        let shoulder_xx = -2. * x * y / (r2 * r2) + elbow_xx / 2.;
        let shoulder_yy = -2. * x * y / (r2 * r2) + elbow_yy / 2.;
        let shoulder_xy = (x * x - y * y) / (r2 * r2) + elbow_xy / 2.;

        Vec2::new(
            elbow_xx * u.x * v.x + elbow_xy * (u.x * v.y + u.y * v.x) + elbow_yy * u.y * v.y,
            shoulder_xx * u.x * v.x
                + shoulder_xy * (u.x * v.y + u.y * v.x)
                + shoulder_yy * u.y * v.y,
        )
    }
}
