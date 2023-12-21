#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::Cell;

use bradipous_geom::{Config, ConfigBuilder, StepperPositions};
use bradipous_planner::stepper::{Accel, Velocity};
use bradipous_protocol::{Calibration, Cmd, StepperSegment};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Timer};
use embedded_storage::{ReadStorage, Storage};
use esp32c3_hal::{
    clock::{ClockControl, Clocks},
    embassy,
    ledc::{self, timer, LSGlobalClkSource, LowSpeed, LEDC},
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
use servo::Servo;
use stepper::{Direction, Stepper};

mod ble;
mod servo;
mod stepper;

macro_rules! singleton {
    ($val:expr, $T:ty) => {{
        static STATIC_CELL: ::static_cell::StaticCell<$T> = ::static_cell::StaticCell::new();
        STATIC_CELL.init($val)
    }};
}

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

// TODO: these should be part of the config
const MAX_STEPS_PER_SEC: u32 = 300;
// what fraction of a second does it take to reach max velocity
const MAX_VELOCITY_PER_SEC: u32 = 8;
const MAX_ACCEL: u32 = MAX_STEPS_PER_SEC * MAX_VELOCITY_PER_SEC;

static GLOBAL: GlobalState = GlobalState {
    config: Mutex::new(Cell::new(None)),
    position: Mutex::new(Cell::new(None)),
};

struct GlobalState {
    config: Mutex<Cell<Option<Config>>>,
    position: Mutex<Cell<Option<StepperPositions>>>,
}

impl GlobalState {
    pub fn config(&self) -> Option<Config> {
        self.config.lock(|c| c.get())
    }

    pub fn position(&self) -> Option<StepperPositions> {
        self.position.lock(|p| p.get())
    }

    pub fn set_config(&self, config: Config) {
        self.config.lock(|c| c.set(Some(config)));
        self.write_to_flash();
    }

    pub fn set_position(&self, pos: StepperPositions) {
        self.position.lock(|p| p.set(Some(pos)));
        self.write_to_flash();
    }

    pub fn write_to_flash(&self) {
        if let (Some(config), Some(position)) = (self.config(), self.position()) {
            let mut buf = [0u8; 256];
            buf[0..5].copy_from_slice(b"bradi");
            let mut flash = FlashStorage::new();
            let data = FlashData { config, position };
            let _ = postcard::to_slice(&data, &mut buf[5..]).unwrap();
            flash.write(FLASH_ADDR, &buf).unwrap();
        }
    }

    pub fn read_from_flash(&self) -> Option<(Config, StepperPositions)> {
        let mut buf = [0u8; 256];
        let mut flash = FlashStorage::new();
        flash.read(FLASH_ADDR, &mut buf).unwrap();

        if &buf[0..5] == b"bradi" {
            let data = postcard::from_bytes::<FlashData>(&buf[5..]).ok()?;
            self.set_config(data.config);
            self.set_position(data.position);
            Some((data.config, data.position))
        } else {
            None
        }
    }
}

fn apply_calibration(global: &GlobalState, calib: Calibration) -> Config {
    let radius = 1.4;

    let config = ConfigBuilder::default()
        .with_claw_distance(calib.claw_distance_cm as f64)
        .with_spool_radius(radius)
        .with_max_hang(30.0)
        .build();

    let pos = StepperPositions {
        left: (calib.left_arm_cm as f64 * config.steps_per_revolution) as u32,
        right: (calib.right_arm_cm as f64 * config.steps_per_revolution) as u32,
    };

    global.set_config(config);
    global.set_position(pos);
    config
}

async fn move_seg(seg: StepperSegment, left: &mut Stepper, right: &mut Stepper) {
    // TODO: unify protocol::StepperSegment and planner::stepper::Segment
    let seg2 = bradipous_planner::stepper::Segment {
        steps: bradipous_planner::stepper::Position {
            left: seg.left_steps.unsigned_abs() as u16,
            right: seg.right_steps.unsigned_abs() as u16,
        },
        start_velocity: Velocity::from_steps_per_second(seg.start_steps_per_sec),
        end_velocity: Velocity::from_steps_per_second(seg.end_steps_per_sec),
        accel: Accel {
            steps_per_s_per_s: MAX_ACCEL,
        },
    };

    let right_dir = if seg.right_steps > 0 {
        Direction::Clockwise
    } else {
        Direction::CounterClockwise
    };
    let left_dir = if seg.left_steps > 0 {
        Direction::CounterClockwise
    } else {
        Direction::Clockwise
    };

    for tick in seg2.iter_steps() {
        Timer::after_micros(tick.delay_us.into()).await;
        if tick.left {
            left.step(left_dir);
        }
        if tick.right {
            right.step(right_dir);
        }
    }
}

async fn calibrated_control(
    cmds: Receiver<Cmd, 64>,
    left: &mut Stepper,
    right: &mut Stepper,
    servo: &mut Servo,
) {
    let mut maybe_steps = GLOBAL.position();
    loop {
        match cmds.receive().await {
            Cmd::Segment(seg) => {
                let Some(mut steps) = maybe_steps else {
                    println!("cannot move without a position");
                    continue;
                };

                steps.left = steps.left.checked_add_signed(seg.left_steps).unwrap();
                steps.right = steps.right.checked_add_signed(seg.right_steps).unwrap();
                maybe_steps = Some(steps);

                move_seg(seg, left, right).await;

                GLOBAL.set_position(steps);
            }
            Cmd::Calibrate(calibration) => {
                let config = apply_calibration(&GLOBAL, calibration);

                let circumference = config.spool_radius * core::f64::consts::PI * 2.0;

                maybe_steps = Some(StepperPositions {
                    left: (calibration.left_arm_cm as f64 / circumference
                        * config.steps_per_revolution) as u32,
                    right: (calibration.right_arm_cm as f64 / circumference
                        * config.steps_per_revolution) as u32,
                });
            }
            Cmd::PenUp => {
                servo.set_angle(90);
                Timer::after_millis(50).await;
            }
            Cmd::PenDown => {
                servo.set_angle(180);
                Timer::after_millis(50).await;
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
    let clocks = singleton!(ClockControl::max(system.clock_control).freeze(), Clocks<'_>);
    embassy::init(clocks, TimerGroup::new(peripherals.TIMG0, clocks).timer0);

    let init = esp_wifi::initialize(
        EspWifiInitFor::Ble,
        systimer::SystemTimer::new(peripherals.SYSTIMER).alarm0,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        clocks,
    )
    .unwrap();

    spawner.must_spawn(ble_task(init, peripherals.BT, CMD_CHANNEL.sender()));

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), clocks).unwrap();
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

    let ledc = singleton!(LEDC::new(peripherals.LEDC, clocks), LEDC<'static>);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let lstimer0 = singleton!(
        ledc.get_timer::<LowSpeed>(timer::Number::Timer2),
        ledc::timer::Timer<LowSpeed>
    );
    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: 50u32.Hz(),
        })
        .unwrap();
    let mut servo_channel = ledc.get_channel::<LowSpeed, _>(
        ledc::channel::Number::Channel0,
        io.pins.gpio6.into_push_pull_output(),
    );
    servo_channel
        .configure(ledc::channel::config::Config {
            timer: lstimer0,
            duty_pct: 3,
            pin_config: ledc::channel::config::PinConfig::PushPull,
        })
        .unwrap();
    let mut servo = Servo::new(servo_channel);
    servo.set_angle(90);

    let mut buf = [0u8; 256];
    let mut flash = FlashStorage::new();
    flash.read(FLASH_ADDR, &mut buf).unwrap();

    GLOBAL.read_from_flash();
    calibrated_control(CMD_CHANNEL.receiver(), &mut left, &mut right, &mut servo).await;
}

#[derive(serde::Serialize, serde::Deserialize, Debug)]
struct FlashData {
    config: Config,
    position: StepperPositions,
}
