//! Launch a precompiled program from RAM on core1
//!
//! This will copy the program `core1-blinky-flash` into some unused memory,
//! then launch that code on core1
//!
//! This program will then send an incrementing number over the multicore fifo.
//!
//! `core1-blinky-flash` will read from the multicore fifo, sets the LED if the number was odd,
//! and then writes that value + 1000 back to the fifo

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_time::fixed_point::FixedPoint;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    entry, pac,
    sio::Sio,
    watchdog::Watchdog,
};

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[used]
pub static CORE1: [u8; include_bytes!("../../core1-blinky-ram/core1ram.bin").len()] =
    *include_bytes!("../../core1-blinky-ram/core1ram.bin");

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

    unsafe {
        bsp::hal::rom_data::memcpy(
            0x20020000 as *mut u8,
            &CORE1[0] as *const u8,
            CORE1.len() as u32,
        );
    }

    let mut mc = bsp::hal::multicore::Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    let core1_program_started = unsafe { core1.bootload(0x20020000 as *const u32) };
    info!("Core 1 started okay? {:?}", core1_program_started.is_ok());

    let read = sio.fifo.read();
    if let Some(read) = read {
        info!("Core1 sent {} after start", read);
    }

    info!("Main loop start");
    let mut count: u32 = 0;
    sio.fifo.drain();
    loop {
        info!("Core0 sends {}", count);
        sio.fifo.write(count);
        cortex_m::asm::sev();
        delay.delay_ms(1);
        while sio.fifo.is_read_ready() {
            let read = sio.fifo.read();
            if let Some(read) = read {
                info!("Core1 replies {}", read);
                if read & 1 == 1 {
                    // LED should be turned on by the other core
                    info!("on!");
                } else {
                    // LED should be turned off by the other core
                    info!("off!");
                }
            }
        }
        count = count.wrapping_add(1);

        delay.delay_ms(500);
    }
}

// End of file
