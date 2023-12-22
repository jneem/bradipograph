use libm::sqrtf;

#[cfg_attr(test, derive(proptest_derive::Arbitrary))]
#[derive(Clone, Copy, Debug)]
pub struct Position {
    pub left: u16,
    pub right: u16,
}

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

#[cfg_attr(test, derive(proptest_derive::Arbitrary))]
#[derive(Clone, Copy, Debug)]
pub struct Segment {
    pub steps: Position,
    /// The initial velocity. This specifies the velocity of the faster
    /// motor; the velocity of the slower motor will be always be some
    /// fixed fraction of the faster one.
    pub start_velocity: Velocity,
    /// The final velocity.
    pub end_velocity: Velocity,
    pub accel: Accel,
}

#[derive(Clone, Copy, Debug)]
pub struct Tick {
    pub left: bool,
    pub right: bool,
    pub delay_us: u32,
}

const MAX_PARAM: u32 = 1 << 24;

impl Segment {
    /// If a segment is long and the initial and final velocities are both small, the step
    /// iterator will just move along it very slowly. This is probably not what we want,
    /// though: we want to accelerate to a higher maximum speed, and then decelerate at
    /// the end.
    ///
    /// This method splits a single segment into two: an acceleration segment and a deceleration
    /// segment.
    pub fn split(&self, max_velocity: Velocity) -> Option<(Segment, Segment)> {
        let start_v = self.start_velocity.steps_per_s as u32;
        let end_v = self.end_velocity.steps_per_s as u32;
        let max_v = max_velocity.steps_per_s as u32;

        let square = |x| x * x;
        // How many steps would it take to get from the initial to the final velocity?
        let steps = square(start_v).abs_diff(square(end_v)) / (2 * self.accel.steps_per_s_per_s);

        let max_steps = self.steps.left.max(self.steps.right) as u32;
        if steps <= max_steps / 2 && (start_v <= max_v * 3 / 4 || end_v <= max_v * 3 / 4) {
            // 2 * max_steps * self.accel.steps_per_s_per_s is the total amount that the squared
            // velocity can change while traversing this segment. Some of that change must be used
            // up in getting from the initial velocity to the final velocity. And then the peak energy
            // as we traverse the segment is half of what's left (since we need to go up and then down).
            let max_energy = max_steps * self.accel.steps_per_s_per_s
                - square(start_v).abs_diff(square(end_v)) / 2;
            let max_velocity = (sqrtf(max_energy as f32) as u32).min(max_v);

            // How may steps will it take to get from the max velocity back down to the final velocity?
            let decel_steps =
                (square(max_velocity) - square(end_v)) / (2 * self.accel.steps_per_s_per_s);

            let decel_seg = Segment {
                steps: Position {
                    // FIXME: don't crash on an empty segment
                    left: (self.steps.left as u32 * decel_steps / max_steps) as u16,
                    right: (self.steps.right as u32 * decel_steps / max_steps) as u16,
                },
                start_velocity: Velocity::from_steps_per_second(max_velocity as u16),
                end_velocity: self.end_velocity,
                accel: self.accel,
            };
            let accel_seg = Segment {
                steps: Position {
                    left: self.steps.left - decel_seg.steps.left,
                    right: self.steps.right - decel_seg.steps.right,
                },
                start_velocity: self.start_velocity,
                end_velocity: Velocity::from_steps_per_second(max_velocity as u16),
                accel: self.accel,
            };
            Some((accel_seg, decel_seg))
        } else {
            None
        }
    }

    pub fn iter_steps(&self) -> StepIter {
        // If either left or right doesn't move, the reciprocal is 1/0. By replacing it by MAX_PARAM,
        // we ensure that the this stepper never wins the "minimum" test that would mark it as the
        // next stepper to move.
        let recip_left_steps = MAX_PARAM
            .checked_div(self.steps.left as u32)
            .unwrap_or(MAX_PARAM);
        let recip_right_steps = MAX_PARAM
            .checked_div(self.steps.right as u32)
            .unwrap_or(MAX_PARAM);

        // There should be a minimum number of steps. If we say 16, it means that this can
        // be at most 2^19.
        // FIXME: don't crash on an empty segment
        let param_per_steps = MAX_PARAM / (self.steps.left.max(self.steps.right) as u32);

        // We impose a minimum velocity so that we don't wait forever until the first step.
        // A sensible value would be sqrt(accel / 2), since that's the velocity after a
        // single step. We mostly rely on the caller to set the minimum velocity, though.
        //
        // The effect of the saturating_mul here is that the velocity will get truncated
        // if (1) it's fast and (2) there are very few steps. This truncation is probably
        // fine, because it will be over soon anyway.
        let start_param_per_t = (self.start_velocity.steps_per_s as u32)
            .max(16)
            .saturating_mul(param_per_steps);

        let end_param_per_t = (self.end_velocity.steps_per_s as u32)
            .max(16)
            .saturating_mul(param_per_steps);

        let param_per_t_per_t = self.accel.steps_per_s_per_s.saturating_mul(param_per_steps);

        StepIter {
            pos: Position { left: 0, right: 0 },
            max_pos: self.steps,
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

#[derive(Debug)]
pub struct StepIter {
    max_pos: Position,
    // Current position.
    pos: Position,

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
        if self.pos.left >= self.max_pos.left && self.pos.right >= self.max_pos.right {
            return None;
        }

        let next_left = self.pos.left + 1;
        let next_param_left = if next_left > self.max_pos.left {
            MAX_PARAM
        } else {
            next_left as u32 * self.recip_left_steps
        };
        let next_right = self.pos.right + 1;
        let next_param_right = if next_right > self.max_pos.right {
            MAX_PARAM
        } else {
            next_right as u32 * self.recip_right_steps
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
        if next_param_left <= next_param && self.pos.left < self.max_pos.left {
            self.pos.left += 1;
            ret.left = true;
        }
        if next_param_right <= next_param && self.pos.right < self.max_pos.right {
            self.pos.right += 1;
            ret.right = true;
        }

        self.param = next_param;
        Some(ret)
    }
}
