#![cfg_attr(not(feature = "std"), no_std)]

use arclen::{bendiness, inv_bendiness};
use bradipous_curves::{Curve, CurveRef};
use heapless::Vec;
use kurbo::{
    CubicBez, Line, ParamCurve, ParamCurveArclen, ParamCurveCurvature, PathEl, PathSeg, Point,
    QuadBez, Vec2,
};
use libm::{fabs, sqrt};

mod arclen;

#[derive(Clone, Copy, Debug)]
pub struct Velocity {
    // This is allowed to go up to 2^14 (about 16k)
    pub steps_per_s: u16,
}

impl Velocity {
    pub fn from_steps_per_second(steps_per_s: u16) -> Self {
        Self { steps_per_s }
    }
}

#[cfg(test)]
impl proptest::arbitrary::Arbitrary for Velocity {
    type Parameters = ();
    type Strategy = proptest::strategy::BoxedStrategy<Velocity>;

    fn arbitrary_with(_args: Self::Parameters) -> Self::Strategy {
        use proptest::strategy::Strategy;

        (0u16..(1 << 14))
            .prop_map(|v| Velocity { steps_per_s: v })
            .boxed()
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Accel {
    // This can go up to 2^18.
    pub steps_per_s_per_s: u32,
}

#[cfg(test)]
impl proptest::arbitrary::Arbitrary for Accel {
    type Parameters = ();
    type Strategy = proptest::strategy::BoxedStrategy<Accel>;

    fn arbitrary_with(_args: Self::Parameters) -> Self::Strategy {
        use proptest::strategy::Strategy;

        (0u32..(1 << 18))
            .prop_map(|v| Accel {
                steps_per_s_per_s: v,
            })
            .boxed()
    }
}

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
    pub fn plan<const C: usize>(
        path: &Curve<C>,
        config: &PlannerConfig,
        transform: &impl Transform,
    ) -> Option<Self> {
        let mut ret = MotionCurve {
            points: Vec::new(),
            energies: Vec::new(),
        };

        for seg in path.subcurves() {
            ret.append_smooth_path(seg, config, transform)?;
        }

        Some(ret)
    }

    pub fn plan_one(
        path: CurveRef<'_>,
        config: &PlannerConfig,
        transform: &impl Transform,
    ) -> Option<Self> {
        let mut ret = MotionCurve {
            points: Vec::new(),
            energies: Vec::new(),
        };

        ret.append_smooth_path(path, config, transform)?;

        Some(ret)
    }

    fn append_smooth_path(
        &mut self,
        path: CurveRef<'_>,
        config: &PlannerConfig,
        transform: &impl Transform,
    ) -> Option<()> {
        let transformed_path = TransformedPath { path, transform };

        let plan = Energizer::<CAP>::plan(
            &transformed_path,
            // FIXME: get rid of this magic 10.0 constant
            config.accuracy * 10.0,
            config.max_energy,
            config.max_acceleration,
        );

        // TODO: panics if not enough capacity
        self.points
            .extend(transformed_path.evals(plan.time.iter().copied()));
        self.energies.extend_from_slice(&plan.energy).ok()?;
        Some(())
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
    pub path: CurveRef<'a>,
    pub transform: &'a T,
}

impl<'a, T: Transform + Clone> TransformedPath<'a, T> {
    fn seg_times<'b>(
        &self,
        ts: impl Iterator<Item = PathTime> + 'b,
    ) -> impl Iterator<Item = (PathSeg, f64)> + 'b
    where
        'a: 'b,
    {
        let mut segs = self.path.segments();
        let mut seg_idx = 0;
        let mut seg = segs.next();
        ts.scan((), move |_, t| {
            while t.idx > seg_idx {
                seg_idx += 1;
                seg = segs.next();
            }

            assert!(t.idx == seg_idx);
            Some((seg.unwrap(), t.t))
        })
    }

    pub fn evals<'b>(
        &self,
        ts: impl Iterator<Item = PathTime> + 'b,
    ) -> impl Iterator<Item = Point> + 'b
    where
        'a: 'b,
    {
        self.seg_times(ts)
            .map(|(seg, t)| self.transform.f(seg.eval(t)))
    }

    pub fn curvatures<'b>(
        &self,
        ts: impl Iterator<Item = PathTime> + 'b,
    ) -> impl Iterator<Item = f64> + 'b
    where
        'a: 'b,
    {
        self.seg_times(ts).map(|(seg, t)| {
            let transformed = TransformedCurve {
                transform: self.transform.clone(),
                curve: seg.to_cubic(),
            };
            transformed.curvature(t)
        })
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
        let curvature: Vec<_, CAP> = path.curvatures(time.iter().cloned()).collect();
        let energy = curvature
            .iter()
            .map(|kappa| emax.min(2. * amax / fabs(*kappa)))
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
            let deriv_bound = sqrt(square(self.amax) - square(last * curvature) / 4.);
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
            let deriv_bound = sqrt(square(self.amax) - square(last * curvature) / 4.);
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
) -> Option<()> {
    times.push(PathTime::new(0, 0.0)).ok()?;

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
                times.push(PathTime::new(idx, t)).ok()?;
                increments.push(arclen_accumulated).ok()?;
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
        times.push(PathTime::new(idx, 1.0)).ok()?;
        increments.push(arclen_accumulated).ok()?;
    }

    Some(())
}
