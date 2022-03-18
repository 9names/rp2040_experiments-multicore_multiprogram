//! Launch a precompiled program from RAM on core1
//!
//! This will copy the program `core1-blinky-flash` into some unused memory,
//! then launch that code on core1
//! This program returns reads from the multicore fifo, and then writes that value + 1
//! Yes, `core1-blinky-ram` is poorly named. I should make it flash something...

#![no_std]
#![no_main]

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
    entry, pac,
    sio::Sio,
    watchdog::Watchdog,
};

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[used]
pub static CORE1: [u8; 652] = *include_bytes!("../../core1-blinky-ram/core1ram.bin");

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
    let prog = 0x20020000 as *const usize;
    // Stack pointer is u32 at offset 0 of the vector table
    let stack_ptr = unsafe { prog.read_volatile() };
    info!("Core 1 stack pointer: {:X}", stack_ptr);
    let core1_program_started = unsafe { core1.bootload(0x20020000) };
    info!("Core 1 started okay? {:?}", core1_program_started.is_ok());

    let read = sio.fifo.read();
    if let Some(read) = read {
        info!("Core1 sent {} after start", read);
    }

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // Set the pins to their default state
    let pins = bsp::hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.gpio25.into_push_pull_output();
    let _ = led_pin.set_high();

    info!("Main loop start");
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
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        info!("off!");
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

// End of file
