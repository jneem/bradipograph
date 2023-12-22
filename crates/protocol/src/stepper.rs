use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StepperSegment {
    pub left_steps: i32,
    pub right_steps: i32,
    pub start_steps_per_sec: u16,
    pub end_steps_per_sec: u16,
    pub steps_per_sec_per_sec: u32,
}

impl StepperSegment {
    /// If a segment is long and the initial and final velocities are both small, the step
    /// iterator will just move along it very slowly. This is probably not what we want,
    /// though: we want to accelerate to a higher maximum speed, and then decelerate at
    /// the end.
    ///
    /// This method splits a single segment into two: an acceleration segment and a deceleration
    /// segment.
    #[cfg(feature = "std")]
    pub fn split(&self, max_steps_per_sec: u16) -> Option<(StepperSegment, StepperSegment)> {
        let start_v = self.start_steps_per_sec as f64;
        let end_v = self.end_steps_per_sec as f64;
        let max_v = max_steps_per_sec as f64;
        let max_steps_per_sec_per_sec = self.steps_per_sec_per_sec as f64;

        let square = |x| x * x;
        // How many steps would it take to get from the initial to the final velocity?
        let steps = (square(start_v) - square(end_v)).abs() / (2.0 * max_steps_per_sec_per_sec);

        let max_steps = self.left_steps.abs().max(self.right_steps.abs()) as f64;
        if max_steps == 0.0 {
            // Avoid dividing by zero.
            return None;
        }
        if steps <= max_steps / 2.0 && end_v <= max_v * 0.75 {
            // 2 * max_steps * self.accel.steps_per_s_per_s is the total amount that the squared
            // velocity can change while traversing this segment. Some of that change must be used
            // up in getting from the initial velocity to the final velocity. And then the peak energy
            // as we traverse the segment is half of what's left (since we need to go up and then down).
            let max_energy = max_steps * max_steps_per_sec_per_sec
                - (square(start_v) - square(end_v)).abs() / 2.0;
            let max_velocity = max_energy.sqrt().min(max_v);

            // How may steps will it take to get from the max velocity back down to the final velocity?
            let decel_steps =
                (square(max_velocity) - square(end_v)) / (2.0 * max_steps_per_sec_per_sec);

            let decel_seg = StepperSegment {
                left_steps: (self.left_steps as f64 * decel_steps / max_steps).round() as i32,
                right_steps: (self.right_steps as f64 * decel_steps / max_steps).round() as i32,
                start_steps_per_sec: max_velocity as u16,
                end_steps_per_sec: self.end_steps_per_sec,
                steps_per_sec_per_sec: self.steps_per_sec_per_sec,
            };
            let accel_seg = StepperSegment {
                left_steps: self.left_steps - decel_seg.left_steps,
                right_steps: self.right_steps - decel_seg.right_steps,
                start_steps_per_sec: self.start_steps_per_sec,
                end_steps_per_sec: decel_seg.start_steps_per_sec,
                steps_per_sec_per_sec: self.steps_per_sec_per_sec,
            };
            Some((accel_seg, decel_seg))
        } else {
            None
        }
    }

    pub fn iter_steps(&self) -> StepIter {
        let left = self.left_steps.unsigned_abs();
        let right = self.right_steps.unsigned_abs();

        // If either left or right doesn't move, the reciprocal is 1/0. By replacing it by MAX_PARAM,
        // we ensure that the this stepper never wins the "minimum" test that would mark it as the
        // next stepper to move.
        let recip_left_steps = MAX_PARAM.checked_div(left).unwrap_or(MAX_PARAM);
        let recip_right_steps = MAX_PARAM.checked_div(right).unwrap_or(MAX_PARAM);

        // There should be a minimum number of steps. If we say 16, it means that this can
        // be at most 2^19.
        let param_per_steps = MAX_PARAM.saturating_div(left.max(right));

        // We impose a minimum velocity so that we don't wait forever until the first step.
        // A sensible value would be sqrt(accel / 2), since that's the velocity after a
        // single step. We mostly rely on the caller to set the minimum velocity, though.
        //
        // The effect of the saturating_mul here is that the velocity will get truncated
        // if (1) it's fast and (2) there are very few steps. This truncation is probably
        // fine, because it will be over soon anyway.
        let start_param_per_t = (self.start_steps_per_sec as u32)
            .max(16)
            .saturating_mul(param_per_steps);

        let end_param_per_t = (self.end_steps_per_sec as u32)
            .max(16)
            .saturating_mul(param_per_steps);

        let param_per_t_per_t = self.steps_per_sec_per_sec.saturating_mul(param_per_steps);

        StepIter {
            left: 0,
            right: 0,
            max_left: left,
            max_right: right,
            param: 0,
            recip_left_steps,
            recip_right_steps,
            param_per_t: start_param_per_t,
            param_per_t_min: start_param_per_t.min(end_param_per_t),
            param_per_t_max: start_param_per_t.max(end_param_per_t),
            param_per_t_per_t,
            debounce: recip_right_steps.min(recip_left_steps) / 16,
            speeding_up: end_param_per_t > start_param_per_t,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Tick {
    pub left: bool,
    pub right: bool,
    pub delay_us: u32,
}

const MAX_PARAM: u32 = 1 << 24;

#[derive(Debug)]
pub struct StepIter {
    // These can't actually go all the way up to u32::MAX. Up to 2^24 is probably ok, but I haven't checked carefully.
    max_left: u32,
    max_right: u32,

    // Current position.
    left: u32,
    right: u32,

    // Current parametrization, as a fraction of the distance from
    // the start position to the end position. This maxes out at 2^24.
    param: u32,

    // Current speed, in param per unit time. This goes up to 2^32 - 1, meaning
    // that the fastest we can get through the whole parameter space is about 1/256
    // of a second.
    param_per_t: u32,

    // Measured in the same units as `param`, if two steps happen closer than this
    // we make them happen at the same time.
    debounce: u32,

    // These are relative to MAX_PARAM, in that (modulo rounding errors) recip_left_steps * left_steps = MAX_PARAM.
    recip_left_steps: u32,
    recip_right_steps: u32,

    param_per_t_min: u32,
    param_per_t_max: u32,
    param_per_t_per_t: u32,

    speeding_up: bool,
}

impl Iterator for StepIter {
    type Item = Tick;

    fn next(&mut self) -> Option<Tick> {
        if self.left >= self.max_left && self.right >= self.max_right {
            return None;
        }

        let next_left = self.left + 1;
        let next_param_left = if next_left > self.max_left {
            MAX_PARAM
        } else {
            next_left * self.recip_left_steps
        };
        let next_right = self.right + 1;
        let next_param_right = if next_right > self.max_right {
            MAX_PARAM
        } else {
            next_right * self.recip_right_steps
        };

        let next_param = next_param_right.min(next_param_left);
        let max_next_param = next_param_left.max(next_param_right);
        let next_param = if max_next_param <= next_param + self.debounce {
            max_next_param
        } else {
            next_param
        };

        // dt is (next_param - self.param) / self.param_per_t, but this is often smaller
        // than 1, so we expand the precision first.
        let dt = (((next_param - self.param) as u64) << 32) / self.param_per_t as u64;
        let dparam_per_t = ((self.param_per_t_per_t as u64 * dt) >> 32) as u32;

        self.param_per_t = if self.speeding_up {
            self.param_per_t
                .saturating_add(dparam_per_t)
                .min(self.param_per_t_max)
        } else {
            self.param_per_t
                .saturating_sub(dparam_per_t)
                .max(self.param_per_t_min)
        };

        let mut ret = Tick {
            left: false,
            right: false,
            delay_us: ((dt * 1_000_000) >> 32) as u32,
        };
        if next_param_left <= next_param && self.left < self.max_left {
            self.left += 1;
            ret.left = true;
        }
        if next_param_right <= next_param && self.right < self.max_right {
            self.right += 1;
            ret.right = true;
        }

        self.param = next_param;
        Some(ret)
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;
    use proptest::prelude::*;

    impl Arbitrary for StepperSegment {
        type Parameters = ();
        type Strategy = BoxedStrategy<StepperSegment>;

        fn arbitrary_with(_: ()) -> Self::Strategy {
            let n = 1 << 20;
            (-n..n, -n..n, 1u16..1000, 1u16..1000, 100u32..1000)
                .prop_map(
                    |(left_steps, right_steps, start_steps_per_sec, end_steps_per_sec, a)| {
                        StepperSegment {
                            left_steps,
                            right_steps,
                            start_steps_per_sec,
                            end_steps_per_sec,
                            steps_per_sec_per_sec: a,
                        }
                    },
                )
                .boxed()
        }
    }

    fn left_delays(seg: &StepperSegment) -> Vec<u32> {
        let mut ret = Vec::new();
        let mut last_t = 0;
        for step in seg.iter_steps() {
            last_t += step.delay_us;
            if step.left {
                ret.push(last_t);
                last_t = 0;
            }
        }
        ret
    }

    proptest! {
        #[test]
        fn test_step_count(seg: StepperSegment) {
            let left_counts: u32 = seg.iter_steps().map(|t| u32::from(t.left)).sum();
            let right_counts: u32 = seg.iter_steps().map(|t| u32::from(t.right)).sum();
            assert_eq!(left_counts, seg.left_steps.unsigned_abs());
            assert_eq!(right_counts, seg.right_steps.unsigned_abs());
        }

        // Ensure that when moving at a constant velocity, the delays between
        // ticks are approximately as expected.
        #[test]
        fn expected_velocities(mut seg: StepperSegment) {
            seg.start_steps_per_sec = 500;
            seg.end_steps_per_sec = 500;
            let delays = left_delays(&seg);

            let left = seg.left_steps.unsigned_abs();
            let right = seg.right_steps.unsigned_abs();
            let left_ratio = if left > right {
                1.0
            } else {
                left as f64 / right as f64
            };
            let left_steps_per_second = seg.start_steps_per_sec as f64 * left_ratio;
            let us_per_step = 1_000_000.0 / left_steps_per_second;

            let mean = delays.iter().sum::<u32>() as f64 / delays.len() as f64;
            let mean_deviation = delays.iter().map(|&d| (d as f64 - us_per_step).abs()).sum::<f64>() / delays.len() as f64;
            // We calculate the max deviation around the mean instead of the ideal value: discretization
            // can give some bias to the mean, and we don't want to count that error twice.
            let max_deviation = delays.iter().map(|&d| (d as f64 - mean).abs()).max_by(|x, y| x.total_cmp(y)).unwrap();
            assert!(mean_deviation < us_per_step / 16.0);
            assert!(max_deviation < us_per_step / 10.0);
        }

        #[test]
        fn successful_split_is_nonempty(seg: StepperSegment, max_v in 100u16..1000) {
            let max_v = seg.start_steps_per_sec.max(seg.end_steps_per_sec).max(max_v);
            if let Some((a, d)) = seg.split(max_v) {
                assert!(a.left_steps != 0 || a.right_steps != 0);
                assert!(d.left_steps != 0 || d.right_steps != 0);
            }
        }

        // This test is a little poorly defined because we don't have a great handle on
        // the errors introduced by the discrete approximation scheme. We introduce various
        // kinds of slack to make up.
        #[test]
        fn split_has_the_right_final_velocity(seg: StepperSegment, max_v in 100u16..1000) {
            let left = seg.left_steps.unsigned_abs();
            let right = seg.right_steps.unsigned_abs();
            if left < 16 {
                return Ok(());
            }
            let max_v = seg.start_steps_per_sec.max(seg.end_steps_per_sec).max(max_v);
            let Some((_, d)) = seg.split(max_v) else {
                return Ok(());
            };
            let delays = left_delays(&d);
            let Some(&last_delay) = delays.last() else {
                return Ok(());
            };

            let left_ratio = if left > right {
                1.0
            } else {
                left as f64 / right as f64
            };
            let final_velocity = 1_000_000.0 / last_delay as f64;
            let target_velocity = seg.end_steps_per_sec.max(16) as f64 * left_ratio;
            let final_energy = final_velocity * final_velocity;
            let target_energy = target_velocity * target_velocity;
            dbg!(final_velocity, target_velocity, final_energy, target_energy);
            assert!(final_velocity >= 0.9 * target_velocity - 50.0);

            // The energy changes by about 2 * steps_per_sec_per_sec on each step, so adding that
            // slack means the number of deceleration steps is allowed to be off by about 8.
            assert!(final_energy <= 1.1 * target_energy + 16.0 * seg.steps_per_sec_per_sec as f64);
        }
    }
}
