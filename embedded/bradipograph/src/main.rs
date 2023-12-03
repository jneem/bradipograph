#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_hal::digital::v2::PinState;
use esp32c3_hal::{
    clock::ClockControl, embassy, peripherals::Peripherals, prelude::*, systimer,
    timer::TimerGroup, IO,
};
use esp_backtrace as _;

mod stepper;

use esp_println::println;
use stepper::{Direction, Stepper};

#[main]
async fn main(_spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    embassy::init(&clocks, TimerGroup::new(peripherals.TIMG0, &clocks).timer0);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut stepper = Stepper::new(
        io.pins.gpio0.into_push_pull_output(),
        io.pins.gpio1.into_push_pull_output(),
        io.pins.gpio2.into_push_pull_output(),
        io.pins.gpio3.into_push_pull_output(),
    );

    loop {
        for _ in 0..2038 / 4 {
            stepper.step(Direction::Up);
            Timer::after_millis(2).await;
        }
        for _ in 0..2038 / 4 {
            stepper.step(Direction::Down);
            Timer::after_millis(2).await;
        }
    }
}
