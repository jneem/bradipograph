#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::Cell;

use bradipous_geom::{Config, ConfigBuilder};
use bradipous_planner::stepper::{Accel, Velocity};
use bradipous_protocol::{Calibration, Cmd};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Timer};
use embedded_storage::{ReadStorage, Storage};
use esp32c3_hal::{
    clock::ClockControl,
    embassy,
    peripherals::Peripherals,
    prelude::*,
    rmt::{Channel as RmtChannel, TxChannelCreator},
    systimer::{self},
    timer::TimerGroup,
    Rmt, Rng, IO,
};
use esp_backtrace as _;
use esp_println::println;
use esp_storage::FlashStorage;
use esp_wifi::EspWifiInitFor;
use espilepsy::Color;
use kurbo::Point;
use stepper::{Direction, Stepper};

mod ble;
mod stepper;

use ble::{ble_task, CmdChannel};

pub type Channel<T, const N: usize> = embassy_sync::channel::Channel<CriticalSectionRawMutex, T, N>;
pub type Sender<T, const N: usize> =
    embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, T, N>;
pub type Receiver<T, const N: usize> =
    embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, T, N>;
pub type Mutex<T> = embassy_sync::blocking_mutex::Mutex<CriticalSectionRawMutex, T>;

static CMD_CHANNEL: CmdChannel = CmdChannel::new();
static LED_CHANNEL: espilepsy::CmdChannel<CriticalSectionRawMutex> = Channel::new();

const FLASH_ADDR: u32 = 0x110000;

const MAX_STEPS_PER_SEC: u32 = 500;
// what fraction of a second does it take to reach max velocity
const MAX_VELOCITY_PER_SEC: u32 = 8;
const MAX_ACCEL: u32 = MAX_STEPS_PER_SEC * MAX_VELOCITY_PER_SEC;

static GLOBAL: GlobalState = GlobalState {
    config: Mutex::new(Cell::new(None)),
    position: Mutex::new(Cell::new(None)),
};

struct GlobalState {
    config: Mutex<Cell<Option<Config>>>,
    position: Mutex<Cell<Option<Point>>>,
}

impl GlobalState {
    pub fn config(&self) -> Option<Config> {
        self.config.lock(|c| c.get())
    }

    pub fn position(&self) -> Option<Point> {
        self.position.lock(|p| p.get())
    }

    pub fn set_config(&self, config: Config) {
        self.config.lock(|c| c.set(Some(config)));
        self.write_to_flash();
    }

    pub fn set_position(&self, pos: Point) {
        self.position.lock(|p| p.set(Some(pos)));
        self.write_to_flash();
    }

    pub fn write_to_flash(&self) {
        if let (Some(config), Some(position)) = (self.config(), self.position()) {
            let mut buf = [0u8; 256];
            buf[0..4].copy_from_slice(b"brad");
            let mut flash = FlashStorage::new();
            let data = FlashData { config, position };
            let _ = postcard::to_slice(&data, &mut buf[4..]).unwrap();
            flash.write(FLASH_ADDR, &buf).unwrap();
        }
    }

    pub fn read_from_flash(&self) -> Option<(Config, Point)> {
        let mut buf = [0u8; 256];
        let mut flash = FlashStorage::new();
        flash.read(FLASH_ADDR, &mut buf).unwrap();

        if &buf[0..4] == b"brad" {
            let data = postcard::from_bytes::<FlashData>(&buf[4..]).ok()?;
            self.set_config(data.config);
            self.set_position(data.position);
            Some((data.config, data.position))
        } else {
            None
        }
    }
}

fn apply_calibration(global: &GlobalState, calib: Calibration) {
    let radius = 1.3;

    let config = ConfigBuilder::default()
        .with_claw_distance(calib.claw_distance_cm as f64)
        .with_spool_radius(radius)
        .with_max_hang(30.0)
        .build();

    global.set_config(config);
}

async fn calibrated_control(cmds: Receiver<Cmd, 64>, left: &mut Stepper, right: &mut Stepper) {
    let maybe_point = GLOBAL.position();
    let maybe_config = GLOBAL.config();
    let mut maybe_steps = maybe_point
        .zip(maybe_config)
        .map(|(p, c)| c.point_to_steps(&p));
    loop {
        match cmds.receive().await {
            Cmd::MoveTo(x, y) => {
                let Some((config, steps)) = maybe_config.zip(maybe_steps) else {
                    println!("cannot move without a calibration and a position");
                    continue;
                };

                // TODO: validate that the point is within range
                let p = Point::new(x as f64, y as f64);
                let target_steps = config.point_to_steps(&p);
                println!("move to (x, y), steps {target_steps:?}");
                let right_count = target_steps.right.abs_diff(steps.right);
                let left_count = target_steps.left.abs_diff(steps.left);

                let max_count = right_count.max(left_count);
                let steps_to_full_v = MAX_STEPS_PER_SEC / MAX_VELOCITY_PER_SEC;

                let decel_steps = steps_to_full_v.min(max_count / 2);
                let decel = bradipous_planner::stepper::Position {
                    left: (left_count * decel_steps / max_count) as u16,
                    right: (right_count * decel_steps / max_count) as u16,
                };
                let accel = bradipous_planner::stepper::Position {
                    left: left_count as u16 - decel.left,
                    right: right_count as u16 - decel.right,
                };

                let max_velocity = libm::sqrtf((decel_steps * MAX_ACCEL) as f32) as u32;
                let max_velocity = Velocity {
                    steps_per_s: max_velocity.min(MAX_STEPS_PER_SEC) as u16,
                };
                let min_velocity = Velocity {
                    steps_per_s: libm::sqrtf(MAX_ACCEL as f32 / 2.0) as u16,
                };

                let accel_seg = bradipous_planner::stepper::Segment {
                    steps: accel,
                    start_velocity: min_velocity,
                    end_velocity: max_velocity,
                    accel: Accel {
                        steps_per_s_per_s: MAX_ACCEL,
                    },
                };
                println!("accel segment {:?}", accel_seg.iter_steps());
                for tick in accel_seg.iter_steps().take(10) {
                    println!("tick: {tick:?}");
                }
                let decel_seg = bradipous_planner::stepper::Segment {
                    steps: decel,
                    start_velocity: max_velocity,
                    end_velocity: min_velocity,
                    accel: Accel {
                        steps_per_s_per_s: MAX_ACCEL,
                    },
                };

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

                for tick in accel_seg.iter_steps().chain(decel_seg.iter_steps()) {
                    if tick.left {
                        left.step(left_dir);
                    }
                    if tick.right {
                        right.step(right_dir);
                    }
                    Timer::after_micros(tick.delay_us.into()).await;
                }

                maybe_steps = Some(target_steps);
                GLOBAL.set_position(p);
            }
            Cmd::SetPos(x, y) => {
                let Some(config) = maybe_config else {
                    println!("cannot set position before calibration");
                    continue;
                };
                let p = Point::new(x as f64, y as f64);
                GLOBAL.set_position(p);
                maybe_steps = Some(config.point_to_steps(&p));
            }
            Cmd::Calibrate(calibration) => {
                apply_calibration(&GLOBAL, calibration);
                maybe_steps = None;
            }
            _ => {
                println!("unexpected cmd in calibrated mode");
                continue;
            }
        }
    }
}

#[embassy_executor::task]
async fn led_task(
    rmt_channel: RmtChannel<0>,
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

    let mut buf = [0u8; 256];
    let mut flash = FlashStorage::new();
    flash.read(FLASH_ADDR, &mut buf).unwrap();

    GLOBAL.read_from_flash();
    calibrated_control(CMD_CHANNEL.receiver(), &mut left, &mut right).await;
}

#[derive(serde::Serialize, serde::Deserialize, Debug)]
struct FlashData {
    config: Config,
    position: Point,
}
