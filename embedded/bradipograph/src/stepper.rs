use embedded_hal::digital::v2::{OutputPin, PinState};
use esp32c3_hal::gpio::{AnyPin, Output, PushPull};

/// These directions are from the point of view of a person looking at the
/// tip of the spinny part of the motor. The shaft, maybe? Anyway, if the
/// bradipograph is hanging on the wall and you're standing back and watching it
/// draw, these are the directions from your point of view.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    Clockwise,
    CounterClockwise,
}

pub type AnyOutput = AnyPin<Output<PushPull>>;

pub struct Stepper {
    pub state: u8,
    pub pins: [AnyOutput; 4],
}

impl Stepper {
    pub fn new(
        p0: impl Into<AnyOutput>,
        p1: impl Into<AnyOutput>,
        p2: impl Into<AnyOutput>,
        p3: impl Into<AnyOutput>,
    ) -> Self {
        let pins = [p0.into(), p1.into(), p2.into(), p3.into()];
        let mut ret = Self { state: 0, pins };
        ret.apply_state();
        ret
    }

    fn apply_state(&mut self) {
        for (idx, p) in self.pins.iter_mut().enumerate() {
            let idx = idx as u8;
            p.set_state(PinState::from(
                idx == self.state || idx == (self.state + 1) % 4,
            ))
            .unwrap();
        }
    }

    pub fn step(&mut self, dir: Direction) {
        let inc = if dir == Direction::Clockwise { 1 } else { 3 };
        self.state = (self.state + inc) % 4;
        self.apply_state();
    }
}
