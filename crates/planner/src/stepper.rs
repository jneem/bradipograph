use fixed::{
    traits::ToFixed as _,
    types::{U16F16, U32F32},
};

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

impl Segment {
    pub fn iter_steps(&self) -> StepIter {
        // If either left or right doesn't move, the reciprocal is 1/0. But we can safely
        // replace it by 1, because in this case it will only ever get multiplied by zero.
        let recip_left_steps = U32F32::from_num(self.steps.left)
            .checked_recip()
            .unwrap_or(1u8.into());
        let recip_right_steps = U32F32::from_num(self.steps.right)
            .checked_recip()
            .unwrap_or(1u8.into());

        let steps_per_param = U16F16::from_num(self.steps.left.max(self.steps.right));
        let param_per_steps = U16F16::from_num(steps_per_param.recip());
        let s_per_t = U16F16::from_num(U32F32::from(1_000_000u32) / U32F32::from(1024u32 * 1024));

        // TODO: re-evaluate the minimum velocity
        let start_param_per_t =
            U16F16::from(self.start_velocity.steps_per_s.max(2)) * s_per_t * param_per_steps;
        let end_param_per_t =
            U16F16::from(self.end_velocity.steps_per_s.max(2)) * s_per_t * param_per_steps;
        let param_per_t_per_t = U32F32::from(self.accel.steps_per_s_per_s)
            * U32F32::from(param_per_steps * s_per_t * s_per_t);

        StepIter {
            pos: Position { left: 0, right: 0 },
            max_pos: self.steps.clone(),
            param: 0i8.to_fixed(),
            recip_left_steps,
            recip_right_steps,
            t_per_param: start_param_per_t.recip(),
            t_per_param_min: start_param_per_t.max(end_param_per_t).recip(),
            t_per_param_max: start_param_per_t.min(end_param_per_t).recip(),
            param_per_t_per_t: U16F16::from_num(param_per_t_per_t),
            speeding_up: end_param_per_t < start_param_per_t,
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
    // the start position to the end position.
    param: U16F16,

    // Current reciprocal speed
    t_per_param: U16F16,

    // TODO: this is the reciprocal of an integer, and we're losing precision by not having more fractional bits.
    // It would probably be ok to have this as a U1F31, and then multiply it by an integer to get a U16F16 or so...
    // but `fixed` doesn't make it so convenient to mix precisions like this.
    recip_left_steps: U32F32,
    recip_right_steps: U32F32,

    t_per_param_min: U16F16,
    t_per_param_max: U16F16,
    param_per_t_per_t: U16F16,

    speeding_up: bool,
}

impl Iterator for StepIter {
    type Item = Tick;

    fn next(&mut self) -> Option<Tick> {
        if self.param == 1 {
            return None;
        }

        // FIXME: this effectively restricts us to 2^16 steps
        let next_left = (self.pos.left + 1).min(self.max_pos.left);
        let next_param_left = U16F16::from_num(U32F32::from(next_left) * self.recip_left_steps);
        let next_right = (self.pos.right + 1).min(self.max_pos.right);
        let next_param_right = U16F16::from_num(U32F32::from(next_right) * self.recip_right_steps);

        let next_param = match (
            next_right == self.max_pos.right,
            next_left == self.max_pos.left,
        ) {
            (true, true) => U16F16::from(1u8),
            (true, false) => next_param_left,
            (false, true) => next_param_right,
            // TODO: should have some wiggle room if they're very close to each other.
            (false, false) => next_param_left.min(next_param_right),
        };

        let dt = (next_param - self.param) * self.t_per_param;

        if self.speeding_up {
            self.t_per_param -= dt * self.param_per_t_per_t * self.t_per_param;
        } else {
            self.t_per_param += dt * self.param_per_t_per_t * self.t_per_param;
        }
        self.t_per_param = self
            .t_per_param
            .clamp(self.t_per_param_min, self.t_per_param_max);

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
            delay_us: dt.to_bits() << 4,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic() {
        let seg = Segment {
            steps: Position {
                left: 1024,
                right: 512,
            },
            start_velocity: Velocity { steps_per_s: 0 },
            end_velocity: Velocity { steps_per_s: 100 },
            accel: Accel {
                steps_per_s_per_s: 1000,
            },
        };

        let ticks: Vec<_> = seg.iter_steps().collect();
        dbg!(ticks.len());
    }
}
