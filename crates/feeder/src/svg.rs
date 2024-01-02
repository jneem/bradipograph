use std::path::Path;

use kurbo::{Affine, BezPath, Rect, Shape};
use usvg::{tiny_skia_path::PathSegment, TreeParsing};

pub fn transform(path: &mut BezPath, target_rect: &Rect) {
    let bbox = path.bounding_box();

    let scale = (target_rect.height() / bbox.height()).min(target_rect.width() / bbox.width());

    let transform = Affine::translate(-bbox.center().to_vec2())
        .then_scale(scale)
        .then_translate(target_rect.center().to_vec2());

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
            let transform = match &*node.parent().unwrap().borrow() {
                usvg::NodeKind::Group(b) => b.abs_transform,
                _ => {
                    eprintln!("Hm. Expected a path to have a group as its parent");
                    usvg::Transform::identity()
                }
            };
            let path = p.data.as_ref().clone().transform(transform).unwrap();
            for seg in path.segments() {
                match seg {
                    PathSegment::MoveTo(pt) => {
                        bez.move_to(cvt(pt));
                    }
                    PathSegment::LineTo(pt) => {
                        bez.line_to(cvt(pt));
                    }
                    PathSegment::QuadTo(pt1, pt2) => {
                        bez.quad_to(cvt(pt1), cvt(pt2));
                    }
                    PathSegment::CubicTo(pt1, pt2, pt3) => {
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
