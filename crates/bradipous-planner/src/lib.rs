#![cfg_attr(not(feature = "std"), no_std)]

use arclen::{bendiness, inv_bendiness};
use heapless::Vec;
use kurbo::{
    BezPath, CubicBez, Line, ParamCurve, ParamCurveArclen, ParamCurveCurvature, PathEl, PathSeg,
    Point, QuadBez, Vec2,
};

mod arclen;

#[derive(Clone, Debug, serde::Serialize)]
pub struct MotionCurve<const CAP: usize> {
    pub points: Vec<Point, CAP>,
    pub energies: Vec<f64, CAP>,
}

#[derive(Clone, Debug)]
pub struct PlannerConfig {
    pub max_energy: f64,
    pub max_acceleration: f64,
    pub accuracy: f64,
}

// TODO: right now, if we run out of capacity then we panic. Be more robust.
impl<const CAP: usize> MotionCurve<CAP> {
    pub fn plan(path: &mut BezPath, config: &PlannerConfig, transform: &impl Transform) -> Self {
        let mut ret = MotionCurve {
            points: Vec::new(),
            energies: Vec::new(),
        };

        assert!(!path.is_empty());
        let mut start = 1;
        let PathEl::MoveTo(mut start_point) = path.elements()[0] else {
            panic!("invalid bez");
        };
        if matches!(path.elements().last(), Some(PathEl::ClosePath)) {
            *path.elements_mut().last_mut().unwrap() = PathEl::LineTo(start_point);
        }
        let mut segments = path.segments().peekable();
        while start < path.elements().len() {
            let take = TakeWhileSmooth::new(&mut segments, config.accuracy);
            let smooth_count = take.count();
            assert!(smooth_count > 0);

            let end = start + smooth_count;
            let subpath = Subpath {
                path: &path.elements()[start..end],
                start: start_point,
            };

            ret.append_smooth_path(subpath, config, transform);
            start = end;
            start_point = subpath.path.last().unwrap().end_point().unwrap();
        }

        ret
    }

    fn append_smooth_path(
        &mut self,
        path: Subpath<'_>,
        config: &PlannerConfig,
        transform: &impl Transform,
    ) {
        let transformed_path = TransformedPath { path, transform };

        let plan = Energizer::<CAP>::plan(
            &transformed_path,
            // FIXME: get rid of this magic 10.0 constant
            config.accuracy * 10.0,
            config.max_energy,
            config.max_acceleration,
        );

        self.points
            .extend(plan.time.iter().map(|t| transformed_path.eval(*t)));
        self.energies.extend_from_slice(&plan.energy);
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Subpath<'a> {
    path: &'a [PathEl],
    start: Point,
}

impl<'a> Subpath<'a> {
    pub fn get_seg(&self, idx: usize) -> PathSeg {
        let start = if idx == 0 {
            self.start
        } else {
            match &self.path[idx - 1] {
                PathEl::LineTo(p) => *p,
                PathEl::QuadTo(_, p) => *p,
                PathEl::CurveTo(_, _, p) => *p,
                _ => panic!("invalid bez path"),
            }
        };

        match self.path[idx] {
            PathEl::LineTo(p) => PathSeg::Line(Line::new(start, p)),
            PathEl::QuadTo(p, q) => PathSeg::Quad(QuadBez::new(start, p, q)),
            PathEl::CurveTo(p, q, r) => PathSeg::Cubic(CubicBez::new(start, p, q, r)),
            // Note: we need to ensure that our paths don't have ClosePath.
            _ => panic!("invalid bez path"),
        }
    }

    pub fn segments(&self) -> impl Iterator<Item = PathSeg> + '_ {
        kurbo::segments(
            core::iter::once(PathEl::MoveTo(self.start)).chain(self.path.iter().cloned()),
        )
    }
}

struct TakeWhileSmooth<'a, I: Iterator> {
    segs: &'a mut core::iter::Peekable<I>,
    prev_tangent: Option<Vec2>,
    accuracy: f64,
}

impl<'a, I: Iterator<Item = PathSeg>> Iterator for TakeWhileSmooth<'a, I> {
    type Item = PathSeg;

    fn next(&mut self) -> Option<PathSeg> {
        let next_seg = self.segs.peek()?;
        let (start_tangent, end_tangent) = match next_seg {
            kurbo::PathSeg::Line(ell) => (ell.p1 - ell.p0, ell.p1 - ell.p0),
            kurbo::PathSeg::Quad(q) => (q.p1 - q.p0, q.p2 - q.p1),
            kurbo::PathSeg::Cubic(c) => (c.p1 - c.p0, c.p2 - c.p1),
        };

        if let Some(prev_tangent) = self.prev_tangent {
            // If one of the tangents is close to zero, say that they aren't parallel. This
            // might be a false positive, but it saves us from having to consider the next
            // derivative.
            let mut is_corner =
                prev_tangent.length_squared() < 1e-3 || start_tangent.length_squared() < 1e-3;

            // If the tangents aren't approximately parallel, it's a corner.
            is_corner |= prev_tangent.cross(start_tangent).abs()
                < self.accuracy * prev_tangent.length() * start_tangent.length();

            // If the tangents are pointing in opposite directions, it's a corner.
            is_corner |= prev_tangent.dot(start_tangent) <= 0.;

            if is_corner {
                return None;
            }
        }
        self.prev_tangent = Some(end_tangent);
        self.segs.next()
    }
}

impl<'a, I: Iterator<Item = PathSeg>> TakeWhileSmooth<'a, I> {
    pub fn new(iter: &'a mut core::iter::Peekable<I>, accuracy: f64) -> Self {
        Self {
            segs: iter,
            prev_tangent: None,
            accuracy,
        }
    }
}

pub trait Transform: Clone {
    fn f(&self, input: Point) -> Point;
    fn df(&self, input: Point, direction: Vec2) -> Vec2;
    fn ddf(&self, input: Point, u: Vec2, v: Vec2) -> Vec2;
}

pub struct TransformedCurve<T> {
    pub transform: T,
    pub curve: CubicBez,
}

pub struct TransformedPath<'a, T> {
    pub path: Subpath<'a>,
    pub transform: &'a T,
}

impl<'a, T: Transform + Clone> TransformedPath<'a, T> {
    fn seg_time(&self, t: PathTime) -> (PathSeg, f64) {
        (self.path.get_seg(t.idx), t.t)
    }

    pub fn eval(&self, t: PathTime) -> Point {
        let (seg, t) = self.seg_time(t);
        self.transform.f(seg.eval(t))
    }

    pub fn curvature(&self, t: PathTime) -> f64 {
        let (seg, t) = self.seg_time(t);
        let transformed = TransformedCurve {
            transform: self.transform.clone(),
            curve: seg.to_cubic(),
        };
        transformed.curvature(t)
    }
}

#[derive(Debug)]
pub struct Energizer<const CAP: usize> {
    pub emax: f64,
    pub amax: f64,
    pub increments: Vec<f64, CAP>,
    pub time: Vec<PathTime, CAP>,
    pub curvature: Vec<f64, CAP>,
    pub energy: Vec<f64, CAP>,
}

fn square(x: f64) -> f64 {
    x * x
}

impl<const CAP: usize> Energizer<CAP> {
    pub fn plan<T: Transform + Clone>(
        path: &TransformedPath<T>,
        increment: f64,
        emax: f64,
        amax: f64,
    ) -> Self {
        let mut ret = Self::init(path, increment, emax, amax);
        ret.forward_pass();
        ret.backward_pass();
        ret
    }

    pub fn init<T: Transform + Clone>(
        path: &TransformedPath<T>,
        increment: f64,
        emax: f64,
        amax: f64,
    ) -> Self {
        let mut time = Vec::new();
        let mut increments = Vec::new();
        discretize_time(path, increment, &mut time, &mut increments);
        let curvature: Vec<_, CAP> = time.iter().map(|t| path.curvature(*t)).collect();
        let energy = curvature
            .iter()
            .map(|kappa| emax.min(2. * amax / kappa.abs()))
            .collect();

        Self {
            emax,
            amax,
            increments,
            time,
            curvature,
            energy,
        }
    }

    // TODO: we're starting from zero energy here, but we should support building up a path
    // without constantly restarting from zero.
    pub fn forward_pass(&mut self) {
        *self.energy.first_mut().unwrap() = 0.;
        let mut last = 0.;
        for ((energy_bound, curvature), increment) in self
            .energy
            .iter_mut()
            .zip(&self.curvature)
            .skip(1)
            .zip(&self.increments)
        {
            let deriv_bound = (square(self.amax) - square(last * curvature) / 4.).sqrt();
            *energy_bound = energy_bound.min(last + increment * deriv_bound);
            last = *energy_bound;
        }
    }

    pub fn backward_pass(&mut self) {
        *self.energy.last_mut().unwrap() = 0.;
        let mut last = 0.;
        let energies = self.energy.iter_mut().rev();
        let curvatures = self.curvature.iter().rev();
        let increments = self.increments.iter().rev();
        for ((energy_bound, curvature), increment) in
            energies.zip(curvatures).skip(1).zip(increments)
        {
            let deriv_bound = (square(self.amax) - square(last * curvature) / 4.).sqrt();
            *energy_bound = energy_bound.min(last + increment * deriv_bound);
            last = *energy_bound;
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct PathTime {
    pub idx: usize,
    pub t: f64,
}

impl PathTime {
    fn new(idx: usize, t: f64) -> Self {
        Self { idx, t }
    }
}

pub fn discretize_time<T: Transform + Clone, const CAP: usize>(
    path: &TransformedPath<T>,
    increment: f64,
    times: &mut Vec<PathTime, CAP>,
    increments: &mut Vec<f64, CAP>,
) {
    times.push(PathTime::new(0, 0.0));

    let mut bend_accumulated = 0.0;
    let mut arclen_accumulated = 0.0;
    for (idx, seg) in path.path.segments().enumerate() {
        let transformed = TransformedCurve {
            curve: seg.to_cubic(),
            transform: path.transform.clone(),
        };
        let mut prev_t = 0.0;
        loop {
            let subseg = transformed.subsegment(prev_t..1.0);
            let t_step = inv_bendiness(&subseg, increment - bend_accumulated);

            // t_step was computed for the subsegment, which got reparametrized. Put
            // it back in the original coordinates.
            let t = prev_t + t_step * (1.0 - prev_t);

            // This should be close to `increment` most of the time, but near the
            // end of the segment it could be less.
            let bend_step = bendiness(&transformed.subsegment(prev_t..t));

            bend_accumulated += bend_step;
            arclen_accumulated += transformed.subsegment(prev_t..t).arclen(1e-6);
            if bend_accumulated >= 0.99 * increment {
                times.push(PathTime::new(idx, t));
                increments.push(arclen_accumulated);
                bend_accumulated = 0.0;
                arclen_accumulated = 0.0;
            }

            // t_step = 1.0 is basically equivalent to t = 1.0, but more numerically reliable.
            if t_step >= 1.0 || prev_t >= 1.0 {
                break;
            }
            prev_t = t;
        }
    }

    if arclen_accumulated > 1e-3 {
        let idx = path.path.segments().count() - 1;
        times.push(PathTime::new(idx, 1.0));
        increments.push(arclen_accumulated);
    }
}
