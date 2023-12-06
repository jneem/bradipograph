#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ble::{ble_task, Calibration, Cmd, CmdChannel, CmdReceiver, ManualControl};
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Timer};
use embedded_hal::digital::v2::PinState;
use esp32c3_hal::{
    clock::{ClockControl, Clocks},
    embassy,
    gpio::{GpioPin, Output, PushPull},
    peripherals::Peripherals,
    prelude::*,
    rmt::{Channel0, TxChannelCreator},
    systimer::{self, SystemTimer},
    timer::TimerGroup,
    Rmt, Rng, IO,
};
use esp_backtrace as _;
use espilepsy::Color;
use futures::{future::Either, FutureExt};

mod ble;
mod stepper;

use esp_println::println;
use esp_wifi::{ble::controller::asynch::BleConnector, EspWifiInitFor};
use stepper::{Direction, Stepper};

pub type Channel<T, const N: usize> = embassy_sync::channel::Channel<CriticalSectionRawMutex, T, N>;
pub type Sender<T, const N: usize> =
    embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, T, N>;
pub type Receiver<T, const N: usize> =
    embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, T, N>;

static CMD_CHANNEL: CmdChannel = CmdChannel::new();
static STEPPER_CHANNEL: Channel<StepperCmd, 4> = Channel::new();
static LED_CHANNEL: espilepsy::CmdChannel<CriticalSectionRawMutex> = Channel::new();

struct StepperPosition {
    left: i32,
    right: i32,
}

static POSITION: Mutex<CriticalSectionRawMutex, StepperPosition> =
    Mutex::new(StepperPosition { left: 0, right: 0 });

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

static MANUAL_CONTROL_CHANNEL: Channel<Option<ManualControl>, 16> = Channel::new();
#[embassy_executor::task]
async fn manual_control(
    cmds: Receiver<Option<ManualControl>, 16>,
    mut left: Stepper,
    mut right: Stepper,
) {
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
        while state.is_idle() {
            match cmds.receive().await {
                Some(cmd) => state.apply(cmd),
                None => break,
            }
        }

        while !state.is_idle() {
            if let Some(dir) = state.left.dir {
                left.step(dir);
            }
            if let Some(dir) = state.right.dir {
                right.step(dir);
            }

            let now = now();
            if now >= state.left.timeout {
                state.left.dir = None;
            }
            if now >= state.right.timeout {
                state.right.dir = None;
            }

            match futures::future::select(cmds.receive(), Timer::after_millis(4)).await {
                Either::Left((Some(cmd), _)) => state.apply(cmd),
                Either::Left((None, _)) => break,
                Either::Right(_) => {}
            }
        }
    }
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

    spawner.must_spawn(manual_control(
        MANUAL_CONTROL_CHANNEL.receiver(),
        left,
        right,
    ));

    loop {
        match CMD_CHANNEL.receive().await {
            Cmd::Manual(m) => {
                MANUAL_CONTROL_CHANNEL.send(Some(m)).await;
            }
            Cmd::Calibrate(Calibration::MarkLeft) => {
                let mut lock = POSITION.lock().await;
                lock.left = 0;
                lock.right = 0;
            }
            Cmd::Calibrate(Calibration::Finish { y_offset, x_offset }) => {
                MANUAL_CONTROL_CHANNEL.send(None).await;
            } // TODO: how best to manage ownership of the steppers between the calibration
              // mode and the other mode?
        }
    }
}
