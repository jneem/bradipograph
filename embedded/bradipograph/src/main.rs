#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ble::{ble_task, CmdChannel, CmdReceiver, ManualControl};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal::digital::v2::PinState;
use esp32c3_hal::{
    clock::ClockControl,
    embassy,
    peripherals::Peripherals,
    prelude::*,
    systimer::{self, SystemTimer},
    timer::TimerGroup,
    Rng, IO,
};
use esp_backtrace as _;
use futures::{future::Either, FutureExt};

mod ble;
mod stepper;

use esp_println::println;
use esp_wifi::{ble::controller::asynch::BleConnector, EspWifiInitFor};
use stepper::{Direction, Stepper};

macro_rules! singleton {
    ($val:expr, $T:ty) => {{
        static STATIC_CELL: ::static_cell::StaticCell<$T> = ::static_cell::StaticCell::new();
        STATIC_CELL.init($val)
    }};
    ($val:expr) => {{
        static STATIC_CELL: ::static_cell::StaticCell<_> = ::static_cell::StaticCell::new();
        STATIC_CELL.init($val)
    }};
}

pub type Channel<T, const N: usize> = embassy_sync::channel::Channel<CriticalSectionRawMutex, T, N>;
pub type Sender<T, const N: usize> =
    embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, T, N>;
pub type Receiver<T, const N: usize> =
    embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, T, N>;

static CMD_CHANNEL: CmdChannel = CmdChannel::new();
static STEPPER_CHANNEL: Channel<StepperCmd, 4> = Channel::new();

struct StepperCmd {
    left: bool,
    dir: Direction,
}

fn now() -> u64 {
    SystemTimer::now()
}

fn soon() -> u64 {
    SystemTimer::now() + SystemTimer::TICKS_PER_SECOND
}

async fn stepper_task(cmds: CmdReceiver, mut left: Stepper, mut right: Stepper) {
    struct StepperState {
        dir: Option<Direction>,
        timeout: u64,
    }

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
                    self.left.dir = Some(Direction::Up);
                    self.left.timeout = timeout;
                }
                ManualControl::LengthenLeft => {
                    self.left.dir = Some(Direction::Down);
                    self.left.timeout = timeout;
                }
                ManualControl::ShortenRight => {
                    self.right.dir = Some(Direction::Up);
                    self.right.timeout = timeout;
                }
                ManualControl::LengthenRight => {
                    self.right.dir = Some(Direction::Down);
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

    let mut state = State {
        left: StepperState {
            dir: None,
            timeout: 0,
        },
        right: StepperState {
            dir: None,
            timeout: 0,
        },
    };

    loop {
        println!("steppers idle");
        while state.is_idle() {
            state.apply(cmds.receive().await);
        }

        println!("steppers moving");
        while !state.is_idle() {
            if let Some(dir) = state.left.dir {
                left.step(dir);
            }
            if let Some(dir) = state.right.dir {
                right.step(dir);
            }

            let now = now();
            if now >= state.left.timeout {
                if state.left.dir.is_some() {
                    println!(
                        "marking left as idle, now {now}, timeout {}",
                        state.left.timeout
                    );
                }
                state.left.dir = None;
            }
            if now >= state.right.timeout {
                if state.right.dir.is_some() {
                    println!(
                        "marking right as idle, now {now}, timeout {}",
                        state.right.timeout
                    );
                }
                state.right.dir = None;
            }

            match futures::future::select(cmds.receive(), Timer::after_millis(4)).await {
                Either::Left((cmd, _)) => state.apply(cmd),
                Either::Right(_) => {}
            }
        }
    }
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
    let left = Stepper::new(
        io.pins.gpio0.into_push_pull_output(),
        io.pins.gpio1.into_push_pull_output(),
        io.pins.gpio2.into_push_pull_output(),
        io.pins.gpio3.into_push_pull_output(),
    );
    let right = Stepper::new(
        io.pins.gpio4.into_push_pull_output(),
        io.pins.gpio5.into_push_pull_output(),
        io.pins.gpio8.into_push_pull_output(),
        io.pins.gpio10.into_push_pull_output(),
    );

    stepper_task(CMD_CHANNEL.receiver(), left, right).await;

    loop {
        // for _ in 0..2038 / 4 {
        //     stepper.step(Direction::Up);
        //     Timer::after_millis(2).await;
        // }
        // for _ in 0..2038 / 4 {
        //     stepper.step(Direction::Down);
        //     Timer::after_millis(2).await;
        // }
    }
}
