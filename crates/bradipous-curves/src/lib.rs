#![cfg_attr(not(feature = "std"), no_std)]
#![allow(clippy::result_unit_err)]

use heapless::Vec;
use kurbo::{PathEl, PathSeg, Vec2};

#[derive(Copy, Clone, Debug)]
pub struct Pt {
    pub x: f32,
    pub y: f32,
}

impl Pt {
    pub fn to_kurbo(self) -> kurbo::Point {
        kurbo::Point::new(self.x as f64, self.y as f64)
    }
}

impl From<kurbo::Point> for Pt {
    fn from(p: kurbo::Point) -> Pt {
        Pt {
            x: p.x as f32,
            y: p.y as f32,
        }
    }
}

impl From<Pt> for kurbo::Point {
    fn from(p: Pt) -> kurbo::Point {
        kurbo::Point {
            x: p.x as f64,
            y: p.y as f64,
        }
    }
}

impl From<Pt> for () {
    fn from(_p: Pt) {}
}

#[derive(Copy, Clone, Debug)]
pub enum Inst {
    MoveTo,
    LineTo,
    QuadTo,
    CurveTo,
}

impl Inst {
    pub fn pts(&self) -> usize {
        match self {
            Inst::MoveTo | Inst::LineTo => 1,
            Inst::QuadTo => 2,
            Inst::CurveTo => 3,
        }
    }

    pub fn is_draw(&self) -> bool {
        !matches!(self, Inst::MoveTo)
    }
}

impl From<Inst> for () {
    fn from(_p: Inst) {}
}

#[derive(Clone, Copy, Debug)]
pub struct CurveIdx {
    /// Index into the array of instructions.
    pub inst_idx: u16,
    /// Index into the array of points.
    ///
    /// This points to the first point that is needed for interpreting the first
    /// instruction, even though we sometimes think of that point as coming "before"
    /// the instruction. For example, if the instruction is `LineTo` then this is
    /// the index of the line's initial point.
    pub pt_idx: u16,
}

impl From<CurveIdx> for () {
    fn from(_p: CurveIdx) {}
}

#[derive(Clone, Debug, Default)]
pub struct Curve<const CAP: usize> {
    pub pts: Vec<Pt, CAP>,
    // We could have different capacities for points and instructions. Since the
    // number of points per instruction is variable, they aren't necessarily the
    // same length. Probably `pts` will tend to be longer, though, and there isn't
    // much benefit to saving capacity on `insts` since each `inst` is only one byte.
    pub insts: Vec<Inst, CAP>,
    // Indices giving the start indices of smooth segments.
    //
    // The zero index is not included.
    pub smooth_boundaries: Vec<CurveIdx, CAP>,
}

impl<const CAP: usize> Curve<CAP> {
    pub fn extend(&mut self, path: &kurbo::BezPath) -> Result<(), ()> {
        let PathEl::MoveTo(start_point) = path.elements().first().ok_or(())? else {
            panic!("invalid bez path");
        };

        for el in path.elements() {
            let el = match *el {
                PathEl::ClosePath => PathEl::MoveTo(*start_point),
                x => x,
            };
            self.push_el(&el)?;
        }

        Ok(())
    }

    pub fn push_el_unchecked(&mut self, el: &PathEl) -> Result<(), ()> {
        match *el {
            PathEl::MoveTo(p) => {
                if !self.pts.is_empty() {
                    self.insts.push(Inst::MoveTo)?;
                }
                self.pts.push(p.into())?;
            }
            PathEl::LineTo(p) => {
                self.pts.push(p.into())?;
                self.insts.push(Inst::LineTo)?;
            }
            PathEl::QuadTo(p, q) => {
                self.pts.push(p.into())?;
                self.pts.push(q.into())?;
                self.insts.push(Inst::QuadTo)?;
            }
            PathEl::CurveTo(p, q, r) => {
                self.pts.push(p.into())?;
                self.pts.push(q.into())?;
                self.pts.push(r.into())?;
                self.insts.push(Inst::CurveTo)?;
            }
            PathEl::ClosePath => {
                self.pts.push(self.pts[0])?;
                self.insts.push(Inst::LineTo)?;
            }
        }
        Ok(())
    }

    fn last_tangent(&self) -> Option<Vec2> {
        if self.pts.len() < 2
            || self
                .insts
                .last()
                .map_or(true, |inst| matches!(inst, Inst::MoveTo))
        {
            None
        } else {
            Some(self.pts[self.pts.len() - 1].to_kurbo() - self.pts[self.pts.len() - 2].to_kurbo())
        }
    }

    pub fn push_el(&mut self, el: &PathEl) -> Result<(), ()> {
        const SMOOTHNESS_THRESHOLD: f64 = 1e-2;

        let mut is_corner = false;

        // When transitioning from moving to drawing (i.e., when the pen has to go up or down),
        // always start a new segment.
        if let Some(last_inst) = self.insts.last() {
            is_corner |= last_inst.is_draw() == matches!(el, PathEl::MoveTo(_));
        }

        if let Some((prev_tangent, p)) = self.last_tangent().zip(self.pts.last()) {
            let start_tangent = match el {
                PathEl::MoveTo(q)
                | PathEl::LineTo(q)
                | PathEl::QuadTo(q, _)
                | PathEl::CurveTo(q, ..) => *q - p.to_kurbo(),
                PathEl::ClosePath => self.pts[0].to_kurbo() - p.to_kurbo(),
            };
            // If one of the tangents is close to zero, say that they aren't parallel. This
            // might be a false positive, but it saves us from having to consider the next
            // derivative.
            is_corner |=
                prev_tangent.length_squared() < 1e-3 || start_tangent.length_squared() < 1e-3;

            // If the tangents aren't approximately parallel, it's a corner.
            is_corner |= libm::fabs(prev_tangent.cross(start_tangent))
                > SMOOTHNESS_THRESHOLD * prev_tangent.length() * start_tangent.length();

            // If the tangents are pointing in opposite directions, it's a corner.
            is_corner |= prev_tangent.dot(start_tangent) <= 0.;
        }

        if is_corner {
            self.smooth_boundaries.push(CurveIdx {
                inst_idx: self.insts.len() as u16,
                pt_idx: self.pts.len() as u16 - 1,
            })?;
        }
        self.push_el_unchecked(el)
    }

    pub fn subcurves(&self) -> impl Iterator<Item = CurveRef<'_>> {
        Subcurves {
            curve: self,
            last_idx: CurveIdx {
                inst_idx: 0,
                pt_idx: 0,
            },
            iter: self.smooth_boundaries.iter(),
        }
    }
}

#[derive(Debug)]
struct Subcurves<'a, const CAP: usize> {
    curve: &'a Curve<CAP>,
    last_idx: CurveIdx,
    iter: core::slice::Iter<'a, CurveIdx>,
}

impl<'a, const CAP: usize> Iterator for Subcurves<'a, CAP> {
    type Item = CurveRef<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        let end_idx = self.iter.next().copied().or_else(|| {
            (self.last_idx.inst_idx < self.curve.insts.len() as u16).then(|| CurveIdx {
                inst_idx: self.curve.insts.len() as u16,
                pt_idx: self.curve.pts.len() as u16 - 1,
            })
        })?;

        let ret = CurveRef {
            pts: &self.curve.pts[self.last_idx.pt_idx as usize..=end_idx.pt_idx as usize],
            insts: &self.curve.insts[self.last_idx.inst_idx as usize..end_idx.inst_idx as usize],
        };

        self.last_idx = end_idx;
        Some(ret)
    }
}

#[derive(Copy, Clone, Debug)]
pub struct CurveRef<'a> {
    pub pts: &'a [Pt],
    pub insts: &'a [Inst],
}

impl<'a> CurveRef<'a> {
    pub fn segments(&self) -> impl Iterator<Item = PathSeg> + 'a {
        CurveSegments { curve: *self }
    }
}

// Iterator over segments in a curve.
struct CurveSegments<'a> {
    curve: CurveRef<'a>,
}

impl<'a> Iterator for CurveSegments<'a> {
    type Item = PathSeg;

    fn next(&mut self) -> Option<PathSeg> {
        if let Some((inst, insts)) = self.curve.insts.split_first() {
            let n_pts = inst.pts();
            if self.curve.pts.len() < n_pts + 1 {
                return None;
            }

            let pts = self.curve.pts;
            let seg = match inst {
                Inst::MoveTo | Inst::LineTo => PathSeg::Line(kurbo::Line::new(pts[0], pts[1])),
                Inst::QuadTo => PathSeg::Quad(kurbo::QuadBez::new(pts[0], pts[1], pts[2])),
                Inst::CurveTo => {
                    PathSeg::Cubic(kurbo::CubicBez::new(pts[0], pts[1], pts[2], pts[3]))
                }
            };

            self.curve.pts = &self.curve.pts[n_pts..];
            self.curve.insts = insts;
            Some(seg)
        } else {
            None
        }
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;
    use kurbo::Line;

    #[test]
    fn basic() {
        let mut c = Curve::<1024>::default();
        c.push_el(&PathEl::MoveTo(kurbo::Point::new(0.0, 0.0)))
            .unwrap();
        c.push_el(&PathEl::LineTo(kurbo::Point::new(10.0, 0.0)))
            .unwrap();
        c.push_el(&PathEl::LineTo(kurbo::Point::new(10.0, 10.0)))
            .unwrap();
        c.push_el(&PathEl::LineTo(kurbo::Point::new(0.0, 10.0)))
            .unwrap();
        c.push_el(&PathEl::LineTo(kurbo::Point::new(0.0, 0.0)))
            .unwrap();

        let mut cs = c.subcurves();

        let mut segs = cs.next().unwrap().segments();
        assert_eq!(
            segs.next().unwrap(),
            PathSeg::Line(Line::new((0.0, 0.0), (10.0, 0.0)))
        );
        assert!(segs.next().is_none());

        let mut segs = cs.next().unwrap().segments();
        assert_eq!(
            segs.next().unwrap(),
            PathSeg::Line(Line::new((10.0, 0.0), (10.0, 10.0)))
        );
        assert!(segs.next().is_none());

        let mut segs = cs.next().unwrap().segments();
        assert_eq!(
            segs.next().unwrap(),
            PathSeg::Line(Line::new((10.0, 10.0), (0.0, 10.0)))
        );
        assert!(segs.next().is_none());

        let mut segs = cs.next().unwrap().segments();
        assert_eq!(
            segs.next().unwrap(),
            PathSeg::Line(Line::new((0.0, 10.0), (0.0, 0.0)))
        );
        assert!(segs.next().is_none());

        assert!(cs.next().is_none());
    }
}
