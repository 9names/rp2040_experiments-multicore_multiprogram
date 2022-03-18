//! Core1 flash fifo echo
//!
//! This program will increment the value received from the fifo, and then send it back.
//! It will turn the LED on if the value received is odd,
//! and it will turn it off if the value received is even.
//!
//! It is intended to be built, linked and objcopied into a regular rp2040 project
//! for running a task entirely independently and with Embedded Rust's regular safety
//! guarantees, as well as tooling and libraries
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;

use rp2040_hal as hal;

use hal::{pac, sio::Sio};

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut sio = Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.gpio25.into_push_pull_output();
    sio.fifo.drain();
    loop {
        let read = sio.fifo.read_blocking();
        if read & 1 == 1 {
            let _ = led_pin.set_high();
        } else {
            let _ = led_pin.set_low();
        }
        sio.fifo.write_blocking(read + 1000);
    }
}

// End of file
