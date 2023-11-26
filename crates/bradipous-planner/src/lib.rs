#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;
use alloc::vec::Vec;

use arclen::{bendiness, inv_bendiness};
use kurbo::{
    BezPath, CubicBez, ParamCurve, ParamCurveArclen, ParamCurveCurvature, PathSeg, Point, Vec2,
};

mod arclen;

#[derive(Clone, Debug, serde::Serialize)]
pub struct MotionCurve {
    pub points: Vec<Point>,
    pub energies: Vec<f64>,
}

#[derive(Clone, Debug)]
pub struct PlannerConfig {
    pub max_energy: f64,
    pub max_acceleration: f64,
    pub accuracy: f64,
}

impl MotionCurve {
    pub fn plan(path: &BezPath, config: &PlannerConfig, transform: &impl Transform) -> Self {
        let paths = split_one_at_corners(path, config.accuracy);

        Self::from_paths(paths.iter(), config, transform)
    }

    fn from_paths<'a>(
        paths: impl Iterator<Item = &'a BezPath>,
        config: &PlannerConfig,
        transform: &impl Transform,
    ) -> Self {
        let mut ret = MotionCurve {
            points: Vec::new(),
            energies: Vec::new(),
        };

        for p in paths {
            ret.append_smooth_path(p, config, transform);
        }
        ret
    }

    fn append_smooth_path(
        &mut self,
        path: &BezPath,
        config: &PlannerConfig,
        transform: &impl Transform,
    ) {
        let transformed_path = TransformedPath { path, transform };

        let plan = Energizer::plan(
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

pub struct MotionPlan {
    pub curves: Vec<MotionCurve>,
}

impl MotionPlan {}

pub fn split_one_at_corners(path: &BezPath, accuracy: f64) -> Vec<BezPath> {
    let mut ret = Vec::new();

    let mut cur = BezPath::new();

    // path must start with a MoveTo; add it to our curve
    cur.push(path.iter().next().unwrap());

    let mut prev_tangent = None::<Vec2>;
    for seg in path.segments() {
        let (start_tangent, end_tangent) = match seg {
            kurbo::PathSeg::Line(ell) => (ell.p1 - ell.p0, ell.p1 - ell.p0),
            kurbo::PathSeg::Quad(q) => (q.p1 - q.p0, q.p2 - q.p1),
            kurbo::PathSeg::Cubic(c) => (c.p1 - c.p0, c.p2 - c.p1),
        };

        if let Some(prev_tangent) = prev_tangent {
            // If one of the tangents is close to zero, say that they aren't parallel. This
            // might be a false positive, but it saves us from having to consider the next
            // derivative.
            let mut is_corner =
                prev_tangent.length_squared() < 1e-3 || start_tangent.length_squared() < 1e-3;

            // If the tangents aren't approximately parallel, it's a corner.
            is_corner |= prev_tangent.cross(start_tangent).abs()
                < accuracy * prev_tangent.length() * start_tangent.length();

            // If the tangents are pointing in opposite directions, it's a corner.
            is_corner |= prev_tangent.dot(start_tangent) <= 0.;

            if is_corner {
                ret.push(cur);
                cur = BezPath::new();
                cur.push(kurbo::PathEl::MoveTo(seg.start()));
            }
        }

        cur.push(seg.as_path_el());
        prev_tangent = Some(end_tangent);
    }

    ret.push(cur);
    ret
}

pub fn split_at_corners<'a>(
    paths: impl IntoIterator<Item = &'a BezPath>,
    accuracy: f64,
) -> Vec<BezPath> {
    let mut ret = Vec::new();
    for path in paths {
        ret.append(&mut split_one_at_corners(path, accuracy));
    }
    ret
}

pub trait Transform: Clone {
    fn f(&self, input: &Point) -> Point;
    fn df(&self, input: &Point, direction: &Vec2) -> Vec2;
    fn ddf(&self, input: &Point, u: &Vec2, v: &Vec2) -> Vec2;
}

pub struct TransformedCurve<T> {
    pub transform: T,
    pub curve: CubicBez,
}

pub struct TransformedPath<'a, T> {
    pub path: &'a BezPath,
    pub transform: &'a T,
}

impl<'a, T: Transform + Clone> TransformedPath<'a, T> {
    fn seg_time(&self, t: PathTime) -> (PathSeg, f64) {
        (self.path.get_seg(t.idx + 1).unwrap(), t.t)
    }

    pub fn eval(&self, t: PathTime) -> Point {
        let (seg, t) = self.seg_time(t);
        self.transform.f(&seg.eval(t))
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
pub struct Energizer {
    pub emax: f64,
    pub amax: f64,
    pub increments: Vec<f64>,
    pub time: Vec<PathTime>,
    pub curvature: Vec<f64>,
    pub energy: Vec<f64>,
}

fn square(x: f64) -> f64 {
    x * x
}

impl Energizer {
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
        let (time, increments) = discretize_time(path, increment);
        let curvature: Vec<_> = time.iter().map(|t| path.curvature(*t)).collect();
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

pub fn discretize_time<T: Transform + Clone>(
    path: &TransformedPath<T>,
    increment: f64,
) -> (Vec<PathTime>, Vec<f64>) {
    // let accuracy = increment / 3.0;
    let mut times = Vec::new();
    let mut increments = Vec::new();
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
            if t_step >= 1.0 {
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

    (times, increments)
}
