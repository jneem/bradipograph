use kurbo::{
    common::{GAUSS_LEGENDRE_COEFFS_16, GAUSS_LEGENDRE_COEFFS_24, GAUSS_LEGENDRE_COEFFS_8},
    CubicBez, ParamCurve, ParamCurveArclen, ParamCurveCurvature, ParamCurveDeriv,
};

use crate::{Transform, TransformedCurve};

pub struct DerivTransformedCurve<T> {
    curve: CubicBez,
    transform: T,
}

pub struct DerivDerivTransformedCurve<T> {
    curve: CubicBez,
    transform: T,
}

impl<T: Transform + Clone> ParamCurve for TransformedCurve<T> {
    fn eval(&self, t: f64) -> kurbo::Point {
        self.transform.f(self.curve.eval(t))
    }

    fn subsegment(&self, range: core::ops::Range<f64>) -> Self {
        Self {
            curve: self.curve.subsegment(range),
            transform: self.transform.clone(),
        }
    }
}

impl<T: Transform + Clone> ParamCurve for DerivTransformedCurve<T> {
    fn eval(&self, t: f64) -> kurbo::Point {
        let pt = self.curve.eval(t);
        let v = self.curve.deriv().eval(t).to_vec2();
        self.transform.df(pt, v).to_point()
    }

    fn subsegment(&self, range: core::ops::Range<f64>) -> Self {
        Self {
            curve: self.curve.subsegment(range),
            transform: self.transform.clone(),
        }
    }
}

impl<T: Transform + Clone> ParamCurve for DerivDerivTransformedCurve<T> {
    fn eval(&self, t: f64) -> kurbo::Point {
        let pt = self.curve.eval(t);
        let v = self.curve.deriv().eval(t).to_vec2();
        let w = self.curve.deriv().deriv().eval(t).to_vec2();

        (self.transform.ddf(pt, v, v) + self.transform.df(pt, w)).to_point()
    }

    fn subsegment(&self, range: core::ops::Range<f64>) -> Self {
        Self {
            curve: self.curve.subsegment(range),
            transform: self.transform.clone(),
        }
    }
}

impl<T: Transform + Clone> ParamCurveDeriv for TransformedCurve<T> {
    type DerivResult = DerivTransformedCurve<T>;

    fn deriv(&self) -> Self::DerivResult {
        DerivTransformedCurve {
            curve: self.curve,
            transform: self.transform.clone(),
        }
    }
}

impl<T: Transform + Clone> ParamCurveArclen for TransformedCurve<T> {
    fn arclen(&self, accuracy: f64) -> f64 {
        arclen_rec(&self.curve, &self.transform.clone(), accuracy, 0)
    }
}

impl<T: Transform + Clone> ParamCurveDeriv for DerivTransformedCurve<T> {
    type DerivResult = DerivDerivTransformedCurve<T>;

    fn deriv(&self) -> Self::DerivResult {
        DerivDerivTransformedCurve {
            curve: self.curve,
            transform: self.transform.clone(),
        }
    }
}

impl<T: Transform + Clone> ParamCurveCurvature for TransformedCurve<T> {}

fn arclen_rec<T: Transform + Clone>(
    c: &CubicBez,
    transform: &T,
    accuracy: f64,
    depth: usize,
) -> f64 {
    let d03 = c.p3 - c.p0;
    let d01 = c.p1 - c.p0;
    let d12 = c.p2 - c.p1;
    let d23 = c.p3 - c.p2;
    let lp_lc = d01.hypot() + d12.hypot() + d23.hypot() - d03.hypot();
    let dd1 = d12 - d01;
    let dd2 = d23 - d12;
    // It might be faster to do direct multiplies, the data dependencies would be shorter.
    // The following values don't have the factor of 3 for first deriv
    let dm = 0.25 * (d01 + d23) + 0.5 * d12; // first derivative at midpoint
    let dm1 = 0.5 * (dd2 + dd1); // second derivative at midpoint
    let dm2 = 0.25 * (dd2 - dd1); // 0.5 * (third derivative at midpoint)

    let est = GAUSS_LEGENDRE_COEFFS_8
        .iter()
        .map(|&(wi, xi)| {
            wi * {
                let d_norm2 = (dm + dm1 * xi + dm2 * (xi * xi)).hypot2();
                let dd_norm2 = (dm1 + dm2 * (2.0 * xi)).hypot2();
                dd_norm2 / d_norm2
            }
        })
        .sum::<f64>();
    let transformed = TransformedCurve {
        transform: transform.clone(),
        curve: *c,
    };
    let est_gauss8_error = 4. * (est.powi(3) * 2.5e-6).min(3e-2) * lp_lc;
    if est_gauss8_error < accuracy {
        return transformed.gauss_arclen(GAUSS_LEGENDRE_COEFFS_8);
    }
    let est_gauss16_error = 4. * (est.powi(6) * 1.5e-11).min(9e-3) * lp_lc;
    if est_gauss16_error < accuracy {
        return transformed.gauss_arclen(GAUSS_LEGENDRE_COEFFS_16);
    }
    let est_gauss24_error = 4. * (est.powi(9) * 3.5e-16).min(3.5e-3) * lp_lc;
    if est_gauss24_error < accuracy || depth >= 20 {
        return transformed.gauss_arclen(GAUSS_LEGENDRE_COEFFS_24);
    }
    let (c0, c1) = c.subdivide();
    arclen_rec(&c0, transform, accuracy * 0.5, depth + 1)
        + arclen_rec(&c1, transform, accuracy * 0.5, depth + 1)
}

pub fn bendiness<C: ParamCurveCurvature + ParamCurveDeriv>(c: &C) -> f64
where
    C::DerivResult: ParamCurveDeriv,
{
    let d = c.deriv();
    GAUSS_LEGENDRE_COEFFS_16
        .iter()
        .map(|(wi, xi)| {
            let t = 0.5 * (xi + 1.0); // Transform the interval from [-1, 1] to [0, 1]
            wi * d.eval(t).to_vec2().hypot() * c.curvature(t).abs().sqrt()
        })
        .sum::<f64>()
        * 0.5
}

pub fn inv_bendiness<C: ParamCurve + ParamCurveCurvature + ParamCurveDeriv>(c: &C, bend: f64) -> f64
where
    C::DerivResult: ParamCurveDeriv,
{
    if bend <= 0.0 {
        return 0.0;
    }

    let total_bendiness = bendiness(c);
    if bend >= total_bendiness {
        return 1.0;
    }

    let mut t_last = 0.0;
    let mut bend_last = 0.0;

    // In principle, f(t) should compute the bendiness from 0 to t, but since this is
    // an integral from 0 to t it's inefficient to keep recomputing it. Instead, we remember
    // the most recent evaluation (say, s) and then when asked to evaluate at t we just integrate
    // from s to t.
    let f = |t: f64| {
        let (range, dir) = if t > t_last {
            (t_last..t, 1.0)
        } else {
            (t..t_last, -1.0)
        };
        let db = bendiness(&c.subsegment(range));
        bend_last += db * dir;
        t_last = t;
        bend_last - bend
    };
    kurbo::common::solve_itp(f, 0.0, 1.0, 1e-4, 1, 0.2, -bend, total_bendiness - bend)
}
