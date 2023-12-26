use std::iter::Peekable;

use kurbo::{PathSeg, Vec2};

pub struct SmoothParts<I: Iterator> {
    segments: Peekable<I>,
}

impl<I: Iterator<Item = PathSeg>> SmoothParts<I> {
    pub fn new(iter: I) -> Self {
        Self {
            segments: iter.peekable(),
        }
    }
}

pub struct SmoothPart<'a, I: Iterator> {
    last_tangent: Option<Vec2>,
    segments: &'a mut Peekable<I>,
}

impl<I: Iterator<Item = PathSeg>> SmoothParts<I> {
    pub fn next_part(&mut self) -> Option<SmoothPart<'_, I>> {
        self.segments.peek().is_some().then_some(SmoothPart {
            last_tangent: None,
            segments: &mut self.segments,
        })
    }
}

impl<'a, I: Iterator<Item = PathSeg>> Iterator for SmoothPart<'a, I> {
    type Item = PathSeg;

    fn next(&mut self) -> Option<Self::Item> {
        const SMOOTHNESS_THRESHOLD: f64 = 5e-2;

        let ret = self.segments.peek()?;
        if let Some(prev_tangent) = self.last_tangent {
            let mut is_corner = false;
            let start_tangent = match ret {
                PathSeg::Line(l) => l.p1 - l.p0,
                PathSeg::Quad(q) => q.p1 - q.p0,
                PathSeg::Cubic(c) => c.p1 - c.p0,
            };

            // If one of the tangents is close to zero, say that they aren't parallel. This
            // might be a false positive, but it saves us from having to consider the next
            // derivative.
            is_corner |=
                prev_tangent.length_squared() < 1e-3 || start_tangent.length_squared() < 1e-3;

            // If the tangents aren't approximately parallel, it's a corner.
            is_corner |= prev_tangent.cross(start_tangent).abs()
                > SMOOTHNESS_THRESHOLD * prev_tangent.length() * start_tangent.length();

            // If the tangents are pointing in opposite directions, it's a corner.
            is_corner |= prev_tangent.dot(start_tangent) <= 0.;

            if is_corner {
                return None;
            }
        }

        self.last_tangent = Some(match ret {
            PathSeg::Line(l) => l.p1 - l.p0,
            PathSeg::Quad(q) => q.p2 - q.p1,
            PathSeg::Cubic(c) => c.p3 - c.p2,
        });
        self.segments.next()
    }
}

#[cfg(test)]
mod tests {
    use kurbo::{BezPath, Line, Rect, Shape as _};

    use super::*;

    #[test]
    fn square() {
        let p: BezPath = Rect::new(0., 0., 10., 10.).path_elements(0.01).collect();
        let mut parts = SmoothParts::new(p.segments());
        let p0 = parts.next_part().unwrap().collect::<Vec<_>>();
        assert_eq!(p0, vec![PathSeg::Line(Line::new((0., 0.), (10., 0.)))]);

        let p1 = parts.next_part().unwrap().collect::<Vec<_>>();
        assert_eq!(p1, vec![PathSeg::Line(Line::new((10., 0.), (10., 10.)))]);

        let p2 = parts.next_part().unwrap().collect::<Vec<_>>();
        assert_eq!(p2, vec![PathSeg::Line(Line::new((10., 10.), (0., 10.)))]);

        let p3 = parts.next_part().unwrap().collect::<Vec<_>>();
        assert_eq!(p3, vec![PathSeg::Line(Line::new((0., 10.), (0., 0.)))]);
    }
}
