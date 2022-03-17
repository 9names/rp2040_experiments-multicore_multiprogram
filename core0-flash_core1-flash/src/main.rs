//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".core1"]
#[used]
pub static CORE1: [u8; 652] = *include_bytes!("../../core1-blinky-flash/core1.bin");

fn boot_core1(psm: &mut pac::PSM, ppb: &mut pac::PPB, sio: &mut bsp::hal::Sio) {
    let vector_table = 0x10020000;
    let stack_ptr = 0x20030000 - 0x8;
    let reset_vector = 0x10020004 as *const u32;
    let entry = unsafe { reset_vector.read_volatile() };

    //let (psm, ppb, sio) = (&mut pac.PSM, &mut pac.PPB, &mut sio);
    // Reset the core
    psm.frce_off.modify(|_, w| w.proc1().set_bit());
    while !psm.frce_off.read().proc1().bit_is_set() {
        cortex_m::asm::nop();
    }
    // Drain the fifo before letting the core out of reset
    sio.fifo.drain();
    psm.frce_off.modify(|_, w| w.proc1().clear_bit());
    // We should get a 0 back from core1
    info!("core1 let out of reset");
    let read = sio.fifo.read_blocking();
    if read != 0 {
        info!("core1 sent something unexpected");
    }
    // After reset, core 1 is waiting to receive commands over FIFO.
    // This is the sequence to have it jump to some code.
    let cmd_seq = [
        0,
        0,
        1,
        vector_table as usize,
        stack_ptr as usize,
        entry as usize,
    ];

    let mut seq = 0;
    let mut fails = 0;
    loop {
        // info!("core1 start loop");
        let cmd = cmd_seq[seq] as u32;
        info!("sending {:X} to core1", cmd);
        // Always drain the READ FIFO (from core 1) before sending a 0
        if cmd == 0 {
            sio.fifo.drain();
            // Execute a SEV as core 1 may be waiting for FIFO space via WFE
            cortex_m::asm::sev();
        }
        sio.fifo.write_blocking(cmd);
        let response = sio.fifo.read_blocking();
        info!("core1 sent {:X} in response", response);
        if cmd == response {
            // Move to next state on correct response (echo-d value)
            seq += 1;
        } else {
            // otherwise start over
            seq = 0;
            fails += 1;
            if fails > 16 {
                info!("failed write");
                break;
            }
        }
        if seq >= cmd_seq.len() {
            info!("seq >= cmd_seq.len()");
            break;
        }
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // let pins = bsp::Pins::new(
    //     pac.IO_BANK0,
    //     pac.PADS_BANK0,
    //     sio.gpio_bank0,
    //     &mut pac.RESETS,
    // );

    //let mut led_pin = pins.led.into_push_pull_output();

    // Set the pins to their default state
    let pins = bsp::hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.gpio25.into_push_pull_output();
    led_pin.set_high();

    let SIO2: bsp::hal::pac::SIO = unsafe { core::mem::transmute(()) };
    let mut sio2 = Sio::new(SIO2);
    boot_core1(&mut pac.PSM, &mut pac.PPB, &mut sio2);
    let read = sio.fifo.read();
    if let Some(read) = read {
        info!("Core1 send {}", read);
    }
    info!("Program start");
    sio.fifo.write_blocking(b'a' as u32);
    sio.fifo.write_blocking(b'a' as u32);
    let mut count: u32 = 0;
    loop {
        cortex_m::asm::sev();
        sio.fifo.write(count);
        count = count.wrapping_add(1);
        let read = sio.fifo.read();
        if let Some(read) = read {
            info!("Core1 send {}", read);
        }
        info!("on!");
        //led_pin.set_high().unwrap();
        delay.delay_ms(500);
        info!("off!");
        //led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

// End of file
