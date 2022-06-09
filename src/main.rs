//! Demonstrate the use of a blocking `Delay` using the SYST (sysclock) timer.

#![deny(unsafe_code)]
#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use crate::hal::{pac, prelude::*};
use cortex_m_rt::entry;
use mpu6050_dmp::{
    address::Address, quaternion::Quaternion, sensor::Mpu6050, yaw_pitch_roll::YawPitchRoll,
};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal as hal;

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        rtt_init_print!();

        // Set up the LED. On the Nucleo-446RE it's connected to pin PA5.
        let mut led = dp.GPIOC.split().pc13.into_push_pull_output();

        // Set up the system clock. We want to run at 48MHz for this one.
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

        let _delay_tim5 = dp.TIM5.delay_ms(&clocks);

        let gpiob = dp.GPIOB.split();
        let scl = gpiob
            .pb8
            .into_alternate()
            .internal_pull_up(true)
            .set_open_drain();
        let sda = gpiob
            .pb9
            .into_alternate()
            .internal_pull_up(true)
            .set_open_drain();
        // let i2c = I2c::new(dp.I2C1, (scl, sda), 400.kHz(), &clocks);
        // or
        let i2c = dp.I2C1.i2c((scl, sda), 400.kHz(), &clocks);

        // Create a delay abstraction based on SysTick
        let mut delay = cp.SYST.delay(&clocks);

        let mut sensor = Mpu6050::new(i2c, Address::default()).unwrap();
        rprintln!("spawned");

        sensor.initialize_dmp(&mut delay).unwrap();

        loop {
            let len = sensor.get_fifo_count().unwrap();
            if len >= 28 {
                let mut buf = [0; 28];
                let buf = sensor.read_fifo(&mut buf).unwrap();
                let quat = Quaternion::from_bytes(&buf[..16]).unwrap();
                let ypr = YawPitchRoll::from(quat);
                rprintln!("{:.5?}; {:.5?}; {:.5?};", ypr.yaw, ypr.pitch, ypr.roll);
                led.toggle();
            }
        }
    }

    loop {}
}
