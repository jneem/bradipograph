use std::path::Path;

use geo::{coordinate_position::CoordPos, dimensions::Dimensions, Relate as _};
use geo_booleanop::boolean::{sweep_event::SweepEvent, BoundingBox, Operation};
use geo_types::{Coord, LineString, Polygon};
use kurbo::{Affine, BezPath, Point, Rect, Shape};
use usvg::{tiny_skia_path::PathSegment, TreeParsing};

mod connect_edges;

pub fn transform(path: &mut BezPath, target_rect: &Rect) {
    let bbox = path.bounding_box();

    let scale = (target_rect.height() / bbox.height()).min(target_rect.width() / bbox.width());

    let transform = Affine::translate(-bbox.center().to_vec2())
        .then_scale(scale)
        .then_translate(target_rect.center().to_vec2());

    path.apply_affine(transform);
}

pub fn to_polylines(path: &BezPath, tolerance: f64) -> Vec<LineString<f64>> {
    let mut polylines = Vec::new();
    let mut cur_polyline = Vec::new();
    path.flatten(tolerance, |el| match el {
        kurbo::PathEl::MoveTo(p) => {
            if !cur_polyline.is_empty() {
                polylines.push(cur_polyline.iter().cloned().collect::<LineString<f64>>());
                cur_polyline.clear();
            }
            cur_polyline.push(Coord { x: p.x, y: p.y });
        }
        kurbo::PathEl::LineTo(p) => {
            cur_polyline.push(Coord { x: p.x, y: p.y });
        }
        kurbo::PathEl::ClosePath => {
            if let Some(start) = cur_polyline.first().cloned() {
                cur_polyline.push(start);
                polylines.push(cur_polyline.iter().cloned().collect::<LineString<f64>>());
                cur_polyline.clear();
            } else {
                eprintln!("warning: closing empty path");
            }
        }
        _ => unreachable!(),
    });
    if !cur_polyline.is_empty() {
        polylines.push(cur_polyline.iter().cloned().collect::<LineString<f64>>());
    }
    polylines
}

pub fn to_polygon(path: &BezPath, tolerance: f64) -> Polygon<f64> {
    // Note that Polygon is supposed to satisfy a bunch of properties, and we aren't testing any of them.
    // This causes a panic with the Tweag logo.
    // We should have some routines for, e.g., splitting a path into simple loops.
    let mut polylines = to_polylines(path, tolerance).into_iter();
    if let Some(first) = polylines.next() {
        Polygon::new(first, polylines.collect())
    } else {
        panic!("empty!")
    }
}

pub fn clip_polyline(polyline: &LineString<f64>, poly: &Polygon<f64>) -> Vec<LineString<f64>> {
    let mut sbbox = BoundingBox {
        min: Coord {
            x: std::f64::INFINITY,
            y: std::f64::INFINITY,
        },
        max: Coord {
            x: std::f64::NEG_INFINITY,
            y: std::f64::NEG_INFINITY,
        },
    };
    let mut cbbox = sbbox;

    let mut event_queue = connect_edges::fill_queue_lines(
        polyline,
        &[poly.clone()],
        &mut sbbox,
        &mut cbbox,
        Operation::Intersection,
    );

    if sbbox.min.x > cbbox.max.x
        || cbbox.min.x > sbbox.max.x
        || sbbox.min.y > cbbox.max.y
        || cbbox.min.y > sbbox.max.y
    {
        panic!();
    }

    let sorted_events = geo_booleanop::boolean::subdivide_segments::subdivide(
        &mut event_queue,
        &sbbox,
        &cbbox,
        Operation::Intersection,
    );

    //dbg!(&sorted_events);
    let contours = connect_edges::connect_edges_to_events(&sorted_events);
    dbg!(contours.len());

    fn both_subject(ev: &SweepEvent<f64>) -> bool {
        ev.is_subject && ev.get_other_event().map_or(false, |o| o.is_subject)
    }

    let mut contours = contours.into_iter();
    let mut ret = Vec::new();
    let mut cur = contours
        .next()
        .unwrap()
        .into_iter()
        .map(|ev| ev.point)
        .collect::<Vec<_>>();
    for next in contours {
        let Some(first_on_line) = next.iter().position(both_subject) else {
            continue;
        };
        let last_end = cur.last().unwrap();
        let next_start = &next[first_on_line];

        let join = geo_types::Line::new(*last_end, next_start.point);
        let next_points = next.into_iter().map(|ev| ev.point);
        if poly
            .relate(&join)
            .get(CoordPos::Outside, CoordPos::OnBoundary)
            == Dimensions::Empty
        {
            cur.extend(next_points);
        } else {
            ret.push(LineString::new(cur));
            cur = next_points.collect();
        }
    }
    if !cur.is_empty() {
        ret.push(LineString::new(cur));
    }

    ret
}

pub fn clip_path(path: &BezPath, poly: &Polygon<f64>, tolerance: f64) -> Vec<LineString<f64>> {
    let lines = to_polylines(path, tolerance);

    lines
        .into_iter()
        .flat_map(|line| clip_polyline(&line, poly))
        .collect()
}

pub struct Zigzag {
    pub row_height: f64,
    pub row_gap: f64,
    pub row_offset: f64,
    pub zig: f64,
    pub zag: f64,
}

impl Default for Zigzag {
    fn default() -> Zigzag {
        Zigzag {
            row_height: 3.0,
            row_gap: -0.3,
            row_offset: 0.6,
            zig: 2.0,
            zag: -0.5,
        }
    }
}

impl Zigzag {
    pub fn with_row_height(&mut self, rh: f64) -> &mut Self {
        self.row_height = rh;
        self
    }

    pub fn with_row_gap(&mut self, rg: f64) -> &mut Self {
        self.row_gap = rg;
        self
    }

    pub fn with_row_offset(&mut self, ro: f64) -> &mut Self {
        self.row_offset = ro;
        self
    }

    pub fn with_zig(&mut self, zig: f64) -> &mut Self {
        self.zig = zig;
        self
    }

    pub fn with_zag(&mut self, zag: f64) -> &mut Self {
        self.zag = zag;
        self
    }

    pub fn points(&self, rect: &Rect) -> Vec<Point> {
        assert!(self.zig + self.zag > 0.0);
        let row_count = (rect.height() / self.row_height / 2.0).ceil() as u64;
        (0..row_count)
            .flat_map(|i| {
                let y = rect.y0 + 2.0 * self.row_height * i as f64;
                self.row(y, rect.x0 - 2.0 * i as f64 * self.row_offset, rect.x1)
                    .into_iter()
                    .chain(
                        self.row(
                            y + self.row_height,
                            rect.x0 - (2.0 * i as f64 + 1.0) * self.row_offset,
                            rect.x1,
                        )
                        .into_iter()
                        .rev(),
                    )
            })
            .collect()
    }

    fn row(&self, y: f64, x_start: f64, x_end: f64) -> Vec<Point> {
        let x_step = self.zig + self.zag;
        let count = ((x_end - x_start) / x_step).ceil() as u64;
        (0..count)
            .flat_map(|i| {
                let x = x_start + i as f64 * x_step;
                [
                    Point::new(x, y + self.row_gap / 2.0),
                    Point::new(x + self.zig, y + self.row_height - self.row_gap / 2.0),
                ]
                .into_iter()
            })
            .collect()
    }
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
