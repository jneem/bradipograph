use std::{
    fs::File,
    path::{Path, PathBuf},
};

use bradipous_curves::Curve;
use bradipous_geom::ConfigBuilder;
use bradipous_planner::{MotionCurve, PlannerConfig};
use clap::Parser as _;
use kurbo::{Affine, BezPath, Rect, Shape};
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

    let geom_config = ConfigBuilder::default()
        .with_claw_distance(100.0)
        .with_max_hang(50.0)
        .build();
    let target_bbox = geom_config.draw_box();
    let scale = (target_bbox.height() / bbox.height()).min(target_bbox.width() / bbox.width());

    let transform = Affine::translate(-bbox.center().to_vec2())
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

    let mut curve = Curve::<1024>::default();
    for p in paths.iter() {
        curve.extend(p.elements()).unwrap();
    }
    let m = MotionCurve::<1024>::plan(&curve, &config, &geom_config);
    let out = File::create(&args.output)?;

    serde_json::to_writer(out, &m)?;

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
