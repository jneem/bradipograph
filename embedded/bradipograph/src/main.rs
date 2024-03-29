#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::Cell;

use bradipo_geom::{Config, ConfigBuilder, LenExt as _, StepperPositions};
use bradipo_protocol::{Calibration, Cmd, StepperSegment};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Timer;
use embedded_storage::{ReadStorage, Storage};
use esp32c3_hal::{
    clock::{ClockControl, Clocks},
    embassy,
    ledc::{self, timer, LSGlobalClkSource, LowSpeed, LEDC},
    peripherals::Peripherals,
    prelude::*,
    systimer::{self},
    timer::TimerGroup,
    Rng, IO,
};
use esp_backtrace as _;
use esp_println::println;
use esp_storage::FlashStorage;
use esp_wifi::EspWifiInitFor;
use heapless::spsc::Queue;
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

use ble::{ble_task, CmdConsumer};

pub type Channel<T, const N: usize> = embassy_sync::channel::Channel<CriticalSectionRawMutex, T, N>;
pub type Sender<T, const N: usize> =
    embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, T, N>;
pub type Receiver<T, const N: usize> =
    embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, T, N>;
pub type Mutex<T> = embassy_sync::blocking_mutex::Mutex<CriticalSectionRawMutex, T>;

const FLASH_ADDR: u32 = 0x110000;

static GLOBAL: GlobalState = GlobalState {
    config: Mutex::new(Cell::new(None)),
    position: Mutex::new(Cell::new(None)),
    pen_down: Mutex::new(Cell::new(false)),
};

struct GlobalState {
    config: Mutex<Cell<Option<Config>>>,
    position: Mutex<Cell<Option<StepperPositions>>>,
    pen_down: Mutex<Cell<bool>>,
}

impl GlobalState {
    pub fn config(&self) -> Option<Config> {
        self.config.lock(|c| c.get())
    }

    pub fn position(&self) -> Option<StepperPositions> {
        self.position.lock(|p| p.get())
    }

    pub fn pen_down(&self) -> bool {
        self.pen_down.lock(|p| p.get())
    }

    pub fn set_config(&self, config: Config) {
        self.config.lock(|c| c.set(Some(config)));
    }

    pub fn modify_config(&self, f: impl FnOnce(&mut Config)) {
        self.config.lock(|cell| {
            if let Some(mut c) = cell.get() {
                f(&mut c);
                cell.set(Some(c));
            }
        });
    }

    pub fn set_position(&self, pos: StepperPositions) {
        self.position.lock(|p| p.set(Some(pos)));
    }

    pub fn set_pen_down(&self, down: bool) {
        self.pen_down.lock(|p| p.set(down));
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

fn apply_calibration(global: &GlobalState, calib: Calibration) -> (Config, StepperPositions) {
    let radius = 1.35.cm();

    let config = if let Some(mut prev_config) = global.config() {
        prev_config.claw_distance = calib.claw_distance;
        // TODO: make these configurable, and don't override them here
        prev_config.spool_radius = radius;
        prev_config.steps_per_revolution = 2036.0;
        prev_config
    } else {
        ConfigBuilder::default()
            .with_claw_distance(calib.claw_distance)
            .with_spool_radius(radius)
            .with_max_hang(30.0.cm())
            .build()
    };

    let angles = config.arm_lengths_to_rotor_angles(&calib.arm_lengths);
    let pos = config.rotor_angles_to_stepper_steps(&angles);

    global.set_config(config);
    global.set_position(pos);
    global.write_to_flash();
    (config, pos)
}

async fn move_seg(seg: StepperSegment, left: &mut Stepper, right: &mut Stepper) {
    // This is for "underarm" orientation.
    let right_dir = if seg.right_steps > 0 {
        Direction::CounterClockwise
    } else {
        Direction::Clockwise
    };
    let left_dir = if seg.left_steps > 0 {
        Direction::Clockwise
    } else {
        Direction::CounterClockwise
    };

    for tick in seg.iter_steps() {
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
    mut cmds: CmdConsumer,
    left: &mut Stepper,
    right: &mut Stepper,
    servo: &mut Servo,
) {
    let mut maybe_steps = GLOBAL.position();
    loop {
        let mut written = false;
        let cmd = loop {
            if let Some(cmd) = cmds.dequeue() {
                break cmd;
            }
            if !written {
                // If we get here, our command queue is empty so let's take the
                // opportunity to write our current state to flash.
                GLOBAL.write_to_flash();
                written = true;
            }
            Timer::after_millis(2).await
        };
        match cmd {
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
                let (_, pos) = apply_calibration(&GLOBAL, calibration);

                maybe_steps = Some(pos);
            }
            Cmd::SetMinAngle(deg) => {
                GLOBAL.modify_config(|c| c.min_angle = deg);
                GLOBAL.write_to_flash();
            }
            Cmd::SetMaxHang(h) => {
                GLOBAL.modify_config(|c| c.max_hang = h);
                GLOBAL.write_to_flash();
            }
            Cmd::PenUp => {
                servo.set_angle(90).await;
                GLOBAL.set_pen_down(false);
                GLOBAL.write_to_flash();
                Timer::after_millis(500).await;
            }
            Cmd::PenDown => {
                servo.set_angle(0).await;
                GLOBAL.set_pen_down(true);
                GLOBAL.write_to_flash();
                Timer::after_millis(500).await;
            }
        }
    }
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

    let cmd_queue = singleton!(Queue::new(), Queue<Cmd, 64>);
    let (cmd_tx, cmd_rx) = cmd_queue.split();
    spawner.must_spawn(ble_task(init, peripherals.BT, cmd_tx));

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut right = Stepper::new(
        io.pins.gpio0.into_push_pull_output(),
        io.pins.gpio1.into_push_pull_output(),
        io.pins.gpio2.into_push_pull_output(),
        io.pins.gpio3.into_push_pull_output(),
    );
    let mut left = Stepper::new(
        io.pins.gpio4.into_push_pull_output(),
        io.pins.gpio5.into_push_pull_output(),
        io.pins.gpio6.into_push_pull_output(),
        io.pins.gpio7.into_push_pull_output(),
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
        io.pins.gpio10.into_push_pull_output(),
    );
    servo_channel
        .configure(ledc::channel::config::Config {
            timer: lstimer0,
            duty_pct: 3,
            pin_config: ledc::channel::config::PinConfig::PushPull,
        })
        .unwrap();
    let mut servo = Servo::new(servo_channel);
    servo.set_angle(90).await;

    let mut buf = [0u8; 256];
    let mut flash = FlashStorage::new();
    flash.read(FLASH_ADDR, &mut buf).unwrap();

    GLOBAL.read_from_flash();
    calibrated_control(cmd_rx, &mut left, &mut right, &mut servo).await;
}

#[derive(serde::Serialize, serde::Deserialize, Debug)]
struct FlashData {
    config: Config,
    position: StepperPositions,
}
