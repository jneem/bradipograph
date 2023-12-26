use embassy_time::Timer;
use esp32c3_hal::{
    gpio::{GpioPin, Output, PushPull},
    ledc::{
        channel::{Channel, ChannelHW as _},
        LowSpeed,
    },
};

pub struct Servo {
    channel: Channel<'static, LowSpeed, GpioPin<Output<PushPull>, 6>>,
    current_duty: u16,
}

impl Servo {
    pub fn new(ch: Channel<'static, LowSpeed, GpioPin<Output<PushPull>, 6>>) -> Self {
        Self {
            channel: ch,
            current_duty: Servo::duty_for_angle(90),
        }
    }

    pub fn duty_for_angle(deg: u8) -> u16 {
        let duty_exp = 14;
        let duty_range: u32 = 1 << duty_exp;
        (4 * duty_range / 100 + deg as u32 * 8 * duty_range / (100 * 180)) as u16
    }

    pub async fn set_angle(&mut self, deg: u8) {
        // 0 degrees is 4% duty; 180 degrees is 12% duty.
        let duty = Self::duty_for_angle(deg);

        // One pwm cycle is 20ms. Do the movement over 16 cycles (320ms).
        let diff = duty.abs_diff(self.current_duty);
        if diff < 16 {
            self.channel.set_duty_hw(duty as u32);
        } else {
            self.channel.start_duty_fade_hw(
                self.current_duty as u32,
                duty > self.current_duty,
                16,
                1,
                diff / 16,
            );
            Timer::after_millis(320).await;
            while self.channel.is_duty_fade_running_hw() {
                Timer::after_millis(1).await;
            }

            // This should be basically the duty already, but compensate for any rounding errors.
            self.channel.set_duty_hw(duty as u32);
        };
        self.current_duty = duty;
    }
}
