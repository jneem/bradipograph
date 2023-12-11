use std::path::Path;

use kurbo::{Affine, BezPath, Rect, Shape};
use usvg::{tiny_skia_path::PathSegment, TreeParsing};

pub fn transform(paths: &mut [BezPath], config: &bradipous_geom::Config) {
    let bbox = paths
        .iter()
        .map(Shape::bounding_box)
        .reduce(|b1, b2| b1.union(b2))
        .unwrap();

    let w = config.claw_distance - 20.0;
    let target_bbox = Rect::new(-w / 2.0, 0.0, w / 2.0, config.max_hang);
    let scale = (target_bbox.height() / bbox.height()).min(target_bbox.width() / bbox.width());

    let transform = Affine::translate(-bbox.center().to_vec2())
        .then_scale(scale)
        .then_translate(target_bbox.center().to_vec2());

    for path in paths {
        path.apply_affine(transform);
    }
}

pub fn load_svg(path: &Path) -> anyhow::Result<Vec<BezPath>> {
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
