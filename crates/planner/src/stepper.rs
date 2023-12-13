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

#[derive(Clone, Copy, Debug)]
pub struct Accel {
    // This can go up to 2^18.
    pub steps_per_s_per_s: u32,
}

#[derive(Clone, Copy, Debug)]
pub struct Segment {
    pub steps: Position,
    pub start_velocity: Velocity,
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
    pub fn iter_steps(&self) -> StepIter {
        // If either left or right doesn't move, the reciprocal is 1/0. But we can safely
        // replace it by 1, because in this case it will only ever get multiplied by zero.
        let recip_left_steps = MAX_PARAM.checked_div(self.steps.left as u32).unwrap_or(1);
        let recip_right_steps = MAX_PARAM.checked_div(self.steps.right as u32).unwrap_or(1);

        // There should be a minimum number of steps. If we say 16, it means that this can
        // be at most 2^19.
        let param_per_steps = MAX_PARAM / (self.steps.left.max(self.steps.right) as u32);
        // TODO: convert from seconds to mebi-microseconds
        //let s_per_t = U32F32::from(1_000_000u32) / U32F32::from(1024u32 * 1024);

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
            debounce: recip_right_steps.min(recip_left_steps) / 8,
            speeding_up: end_param_per_t > start_param_per_t,
        }
    }
}

// Our internal time units here are mebi-microseconds, i.e. 10^20 microseconds.
// Our public interface uses microseconds.
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
        if self.param == MAX_PARAM {
            return None;
        }

        let next_left = (self.pos.left + 1).min(self.max_pos.left);
        let next_param_left = next_left as u32 * self.recip_left_steps;
        let next_right = (self.pos.right + 1).min(self.max_pos.right);
        let next_param_right = next_right as u32 * self.recip_right_steps;

        let next_param = match (
            next_right >= self.max_pos.right,
            next_left >= self.max_pos.left,
        ) {
            (true, true) => MAX_PARAM,
            (true, false) => next_param_left,
            (false, true) => next_param_right,
            (false, false) => next_param_left.min(next_param_right),
        };
        let max_next_param = next_param_left.max(next_param_right).max(next_param);
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

        // TODO: do we also need to check that self.pos.left < self.max_pos.left?
        if next_param_left <= next_param {
            self.pos.left += 1;
        }
        if next_param_right <= next_param {
            self.pos.right += 1;
        }

        self.param = next_param;
        Some(Tick {
            left: next_param_left <= next_param,
            right: next_param_right <= next_param,
            delay_us: ((dt * 1_000_000) >> 32) as u32,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // TODO: add a property test checking that the number of steps is always correct.
    #[test]
    fn basic() {
        let seg = Segment {
            steps: Position {
                left: 20,
                right: 19,
            },
            start_velocity: Velocity { steps_per_s: 64 },
            end_velocity: Velocity { steps_per_s: 500 },
            accel: Accel {
                steps_per_s_per_s: 8 * 500,
            },
        };
    }
}
