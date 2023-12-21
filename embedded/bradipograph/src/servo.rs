use esp32c3_hal::{
    gpio::{GpioPin, Output, PushPull},
    ledc::{
        channel::{Channel, ChannelHW as _},
        LowSpeed,
    },
};

pub struct Servo {
    channel: Channel<'static, LowSpeed, GpioPin<Output<PushPull>, 6>>,
}

impl Servo {
    pub fn new(ch: Channel<'static, LowSpeed, GpioPin<Output<PushPull>, 6>>) -> Self {
        Self { channel: ch }
    }

    pub fn set_angle(&mut self, deg: u8) {
        let duty_exp = 14;
        let duty_range: u32 = 1 << duty_exp;
        // 0 degrees is 4% duty; 180 degrees is 12% duty.
        let duty = 4 * duty_range / 100 + deg as u32 * 8 * duty_range / (100 * 180);
        esp_println::println!("degrees {deg}, duty {duty}");
        self.channel.set_duty_hw(duty);
    }
}
