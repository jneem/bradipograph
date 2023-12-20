#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::Cell;

use bradipous_geom::{ArmLengths, Config, ConfigBuilder, StepperPositions};
use bradipous_planner::{
    stepper::{Accel, Velocity},
    MotionCurve, PlannerConfig,
};
use bradipous_protocol::{Calibration, Cmd};
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
use kurbo::Point;
use libm::sqrtf;
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

const MAX_STEPS_PER_SEC: u32 = 300;
const MAX_REVS_PER_SEC: f64 = MAX_STEPS_PER_SEC as f64 / 2036.0;
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

fn apply_calibration(global: &GlobalState, calib: Calibration) -> Config {
    let radius = 1.4;

    let config = ConfigBuilder::default()
        .with_claw_distance(calib.claw_distance_cm as f64)
        .with_spool_radius(radius)
        .with_max_hang(30.0)
        .build();

    let pos = config.arm_lengths_to_point(&ArmLengths {
        left: calib.left_arm_cm as f64,
        right: calib.right_arm_cm as f64,
    });

    global.set_config(config);
    global.set_position(pos);
    config
}

#[allow(clippy::too_many_arguments)]
async fn move_to(
    config: &Config,
    from: StepperPositions,
    x: f32,
    y: f32,
    start_velocity: Velocity,
    end_velocity: Velocity,
    left: &mut Stepper,
    right: &mut Stepper,
) -> (StepperPositions, Point) {
    let p = Point::new(x as f64, y as f64);
    let target_steps = config.point_to_steps(&p);
    println!("move to (x, y), steps {from:?} -> {target_steps:?}");
    let right_count = target_steps.right.abs_diff(from.right);
    let left_count = target_steps.left.abs_diff(from.left);
    let max_velocity = Velocity {
        steps_per_s: MAX_STEPS_PER_SEC as u16,
    };
    let seg = bradipous_planner::stepper::Segment {
        steps: bradipous_planner::stepper::Position {
            left: left_count as u16,
            right: right_count as u16,
        },
        start_velocity,
        end_velocity,
        accel: Accel {
            steps_per_s_per_s: MAX_ACCEL,
        },
    };
    let (accel, decel) = seg.split(max_velocity).unwrap();

    let right_dir = if target_steps.right > from.right {
        Direction::Clockwise
    } else {
        Direction::CounterClockwise
    };
    let left_dir = if target_steps.left > from.left {
        Direction::CounterClockwise
    } else {
        Direction::Clockwise
    };

    for tick in accel.iter_steps().chain(decel.iter_steps()) {
        if tick.left {
            left.step(left_dir);
        }
        if tick.right {
            right.step(right_dir);
        }
        Timer::after_micros(tick.delay_us.into()).await;
    }

    (target_steps, p)
}

async fn calibrated_control(
    cmds: Receiver<Cmd, 64>,
    left: &mut Stepper,
    right: &mut Stepper,
    servo: &mut Servo,
) {
    let min_velocity = Velocity {
        steps_per_s: libm::sqrtf(2.0 * MAX_ACCEL as f32) as u16,
    };
    let maybe_point = GLOBAL.position();
    let mut maybe_config = GLOBAL.config();
    let mut maybe_steps = maybe_point
        .zip(maybe_config)
        .map(|(p, c)| c.point_to_steps(&p));
    'cmd: loop {
        match cmds.receive().await {
            Cmd::Draw(curve) => {
                let Some((config, mut steps)) = maybe_config.zip(maybe_steps) else {
                    println!("cannot move without a calibration and a position");
                    continue;
                };
                let planner = PlannerConfig {
                    max_energy: MAX_REVS_PER_SEC * MAX_REVS_PER_SEC,
                    max_acceleration: MAX_REVS_PER_SEC / 16.0,
                    accuracy: 0.05,
                };
                let bbox = config.bbox();
                for p in &curve.pts {
                    if !bbox.contains(Point::from(*p)) {
                        println!("point ({}, {}) is out of bounds", p.x, p.y);
                        continue 'cmd;
                    }
                }
                let mut final_point = None;
                let mut pen_is_up = true;
                for subcurve in curve.subcurves() {
                    let mut last_velocity = min_velocity;
                    let Some(m) = MotionCurve::<1024>::plan(&curve, &planner, &config) else {
                        println!("error planning the curve");
                        continue;
                    };
                    let move_first = subcurve.insts.first().map_or(false, |i| !i.is_draw());
                    for (i, (pt, energy)) in m.points.iter().zip(&m.energies).enumerate() {
                        if i == 0 && move_first {
                            // Set pen up before moving
                            servo.set_angle(90);
                            pen_is_up = true;
                            Timer::after_millis(50).await;
                        } else if pen_is_up {
                            // Pen is up, but it should be down.
                            servo.set_angle(180);
                            pen_is_up = false;
                            Timer::after_millis(50).await;
                        }

                        let end_velocity = Velocity {
                            steps_per_s: sqrtf(*energy as f32) as u16,
                        };
                        let (next_steps, p) = move_to(
                            &config,
                            steps,
                            pt.x as f32,
                            pt.y as f32,
                            last_velocity,
                            end_velocity,
                            left,
                            right,
                        )
                        .await;
                        last_velocity = end_velocity;
                        steps = next_steps;
                        final_point = Some(p);
                    }
                }
                maybe_steps = Some(steps);
                if let Some(p) = final_point {
                    GLOBAL.set_position(p);
                }
            }
            Cmd::MoveTo(x, y) => {
                let Some((config, steps)) = maybe_config.zip(maybe_steps) else {
                    println!("cannot move without a calibration and a position");
                    continue;
                };

                let (steps, p) = move_to(
                    &config,
                    steps,
                    x,
                    y,
                    min_velocity,
                    min_velocity,
                    left,
                    right,
                )
                .await;
                maybe_steps = Some(steps);
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
                let config = apply_calibration(&GLOBAL, calibration);
                maybe_config = Some(config);

                let circumference = config.spool_radius * core::f64::consts::PI * 2.0;

                maybe_steps = Some(StepperPositions {
                    left: (calibration.left_arm_cm as f64 / circumference
                        * config.steps_per_revolution) as u32,
                    right: (calibration.right_arm_cm as f64 / circumference
                        * config.steps_per_revolution) as u32,
                });
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
    position: Point,
}
