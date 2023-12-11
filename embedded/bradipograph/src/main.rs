#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use bradipous_protocol::{Calibrate, Cmd, ManualControl};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Timer};
use esp32c3_hal::{
    clock::ClockControl,
    embassy,
    peripherals::Peripherals,
    prelude::*,
    rmt::{Channel0, TxChannelCreator},
    systimer::{self, SystemTimer},
    timer::TimerGroup,
    Rmt, Rng, IO,
};
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::EspWifiInitFor;
use espilepsy::Color;
use futures::future::Either;
use kurbo::Point;
use stepper::{Direction, Stepper};

mod ble;
mod stepper;

use ble::{ble_task, CmdChannel, STATUS};

pub type Channel<T, const N: usize> = embassy_sync::channel::Channel<CriticalSectionRawMutex, T, N>;
pub type Sender<T, const N: usize> =
    embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, T, N>;
pub type Receiver<T, const N: usize> =
    embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, T, N>;

static CMD_CHANNEL: CmdChannel = CmdChannel::new();
static LED_CHANNEL: espilepsy::CmdChannel<CriticalSectionRawMutex> = Channel::new();

#[derive(Default)]
struct StepperPosition {
    left: i32,
    right: i32,
}

fn now() -> u64 {
    SystemTimer::now()
}

fn soon() -> u64 {
    SystemTimer::now() + SystemTimer::TICKS_PER_SECOND
}

async fn calibrated_control(
    cmds: Receiver<Cmd, 64>,
    left: &mut Stepper,
    right: &mut Stepper,
    config: bradipous_geom::Config,
    init_pos: Point,
) {
    let mut point = init_pos;
    let mut steps = config.point_to_steps(&point);
    println!("init_pos {init_pos:?}, init_steps {steps:?}");
    loop {
        match cmds.receive().await {
            Cmd::MoveTo(x, y) => {
                // TODO: validate that the point is within range
                let p = Point::new(x as f64, y as f64);
                let target_steps = config.point_to_steps(&p);
                println!("move to (x, y), steps {target_steps:?}");
                let mut right_count = target_steps.right.abs_diff(steps.right);
                let mut left_count = target_steps.left.abs_diff(steps.left);
                let right_dir = if target_steps.right > steps.right {
                    Direction::Clockwise
                } else {
                    Direction::CounterClockwise
                };
                let left_dir = if target_steps.left > steps.left {
                    Direction::CounterClockwise
                } else {
                    Direction::Clockwise
                };

                while right_count != 0 || left_count != 0 {
                    if left_count > 0 {
                        left.step(left_dir);
                        left_count -= 1;
                    }
                    if right_count > 0 {
                        right.step(right_dir);
                        right_count -= 1;
                    }
                    Timer::after_millis(2).await;
                }

                steps = target_steps;
                point = p;
            }
            Cmd::SetPos(x, y) => {
                point = Point::new(x as f64, y as f64);
                steps = config.point_to_steps(&point);
            }
            _ => {
                println!("unexpected cmd in calibrated mode");
                continue;
            }
        }

        STATUS.lock(|status| {
            status.borrow_mut().position = Some(bradipous_protocol::Position {
                x: point.x as f32,
                y: point.y as f32,
                left: steps.left,
                right: steps.right,
            })
        })
    }
}

async fn manual_control(
    cmds: Receiver<Cmd, 64>,
    left: &mut Stepper,
    right: &mut Stepper,
) -> (bradipous_geom::Config, kurbo::Point) {
    #[derive(Default, Debug)]
    struct StepperState {
        dir: Option<Direction>,
        timeout: u64,
    }

    #[derive(Default, Debug)]
    struct State {
        left: StepperState,
        right: StepperState,
    }

    impl State {
        fn apply(&mut self, cmd: ManualControl) {
            // embassy-time's instant::now seems unreliable
            let timeout = soon();
            match cmd {
                ManualControl::ShortenLeft => {
                    self.left.dir = Some(Direction::Clockwise);
                    self.left.timeout = timeout;
                }
                ManualControl::LengthenLeft => {
                    self.left.dir = Some(Direction::CounterClockwise);
                    self.left.timeout = timeout;
                }
                ManualControl::ShortenRight => {
                    self.right.dir = Some(Direction::CounterClockwise);
                    self.right.timeout = timeout;
                }
                ManualControl::LengthenRight => {
                    self.right.dir = Some(Direction::Clockwise);
                    self.right.timeout = timeout;
                }
                ManualControl::StopLeft => {
                    self.left.dir = None;
                }
                ManualControl::StopRight => {
                    self.right.dir = None;
                }
            }
        }

        fn is_idle(&self) -> bool {
            self.left.dir.is_none() && self.right.dir.is_none()
        }
    }

    let mut state = State::default();
    let mut position = StepperPosition::default();

    let mut pending_cmd = Some(cmds.receive().await);
    let (y_offset, x_offset) = 'calibrate: loop {
        loop {
            let cmd = match pending_cmd.take() {
                Some(c) => c,
                None => cmds.receive().await,
            };

            match cmd {
                Cmd::Manual(cmd) => {
                    state.apply(cmd);
                }
                Cmd::Calibrate(Calibrate::MarkLeft) => {
                    position.left = 0;
                    position.right = 0;
                }
                Cmd::Calibrate(Calibrate::Finish { y_offset, x_offset }) => {
                    break 'calibrate (y_offset, x_offset);
                }
                _ => println!("unexpected command in manual mode"),
            }
            if !state.is_idle() {
                break;
            }
        }

        while !state.is_idle() {
            if let Some(dir) = state.left.dir {
                left.step(dir);
                position.left += dir.to_i32();
            }
            if let Some(dir) = state.right.dir {
                right.step(dir);
                position.right += dir.to_i32();
            }

            let now = now();
            if now >= state.left.timeout {
                state.left.dir = None;
            }
            if now >= state.right.timeout {
                state.right.dir = None;
            }

            match futures::future::select(cmds.receive(), Timer::after_millis(2)).await {
                Either::Left((Cmd::Manual(cmd), _)) => state.apply(cmd),
                Either::Left((other, _)) => {
                    // Force us to idle (which will break the `while` loop)
                    state = State::default();
                    pending_cmd = Some(other);
                }
                Either::Right(_) => {}
            }
        }
    };

    let radius = 1.3;
    let circumference = radius * 2.0 * core::f64::consts::PI;
    let steps_per_rev = 2036.0;

    let distance_steps = ((position.left.abs() + position.right.abs()) / 2) as f64;
    let distance_cm = distance_steps / steps_per_rev * circumference;

    let config = bradipous_geom::ConfigBuilder::default()
        .with_spool_radius(radius)
        .with_max_hang(50.0)
        .with_hanging_calibration(y_offset as f64, x_offset as f64, distance_cm)
        .build();

    println!(
        "calibrated {distance_steps} steps, {distance_cm} cm, claw distance {}",
        config.claw_distance
    );

    // The current position should be y_offset cm below the right claw and x_offset cm
    // to its left.
    let pos = kurbo::Point::new(
        config.claw_distance / 2.0 - x_offset as f64,
        y_offset as f64 - config.hang_offset,
    );
    println!("current position {pos:?}");
    let steps = config.point_to_steps(&pos);

    STATUS.lock(|status| {
        let mut status = status.borrow_mut();
        status.calibration = Some(config.clone().into());
        status.position = Some(bradipous_protocol::Position {
            x: pos.x as f32,
            y: pos.y as f32,
            left: steps.left,
            right: steps.right,
        });
    });

    (config, pos)
}

#[embassy_executor::task]
async fn led_task(
    rmt_channel: Channel0<0>,
    recv: espilepsy::CmdReceiver<'static, CriticalSectionRawMutex>,
) {
    espilepsy::task(rmt_channel, recv).await;
}

#[main]
async fn main(spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    embassy::init(&clocks, TimerGroup::new(peripherals.TIMG0, &clocks).timer0);

    let init = esp_wifi::initialize(
        EspWifiInitFor::Ble,
        systimer::SystemTimer::new(peripherals.SYSTIMER).alarm0,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    spawner.must_spawn(ble_task(init, peripherals.BT, CMD_CHANNEL.sender()));

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();
    let rmt_channel = rmt
        .channel0
        .configure(
            io.pins.gpio7,
            esp32c3_hal::rmt::TxChannelConfig {
                clk_divider: 1,
                ..Default::default()
            },
        )
        .unwrap();

    spawner.must_spawn(led_task(rmt_channel, LED_CHANNEL.receiver()));

    LED_CHANNEL
        .send(espilepsy::Cmd::Blinky {
            color0: Color { r: 0, g: 0, b: 0 },
            color1: Color { r: 32, g: 0, b: 0 },
            period: Duration::from_secs(1),
        })
        .await;

    let mut right = Stepper::new(
        io.pins.gpio0.into_push_pull_output(),
        io.pins.gpio1.into_push_pull_output(),
        io.pins.gpio2.into_push_pull_output(),
        io.pins.gpio3.into_push_pull_output(),
    );
    let mut left = Stepper::new(
        io.pins.gpio4.into_push_pull_output(),
        io.pins.gpio5.into_push_pull_output(),
        io.pins.gpio8.into_push_pull_output(),
        io.pins.gpio10.into_push_pull_output(),
    );

    let (config, pos) = manual_control(CMD_CHANNEL.receiver(), &mut left, &mut right).await;
    calibrated_control(CMD_CHANNEL.receiver(), &mut left, &mut right, config, pos).await;
}
