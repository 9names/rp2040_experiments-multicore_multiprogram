//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
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
    entry,
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};



/// Errors for multicore operations.
#[derive(Debug)]
pub enum Error {
    /// Operation is invalid on this core.
    InvalidCore,
    /// Core was unresposive to commands.
    Unresponsive,
    /// The vector table did not meet cortex-m vector table alignment requirements
    /// - Vector table must be 32 word (128 byte) aligned
    InvalidVectorTableAlignment,
    /// Vector table is not in SRAM, XIP SRAM or Flash
    InvalidProgramLocation,
    /// Invalid stack pointer
    InvalidStackPointer,
    /// Invalid program entry address - is before the start of the program
    InvalidEntryAddressBelow,
    /// Invalid program entry address - program in SRAM but entry point is not
    InvalidEntryAddressAboveRAM,
    /// Invalid program entry address - program in XIP SRAM but entry point is not
    InvalidEntryAddressAboveXIPRAM,
    /// Invalid program entry address - program in Flash but entry point is not
    InvalidEntryAddressAboveFLASH,
}

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[used]
pub static CORE1: [u8; 652] = *include_bytes!("../../core1-blinky-ram/core1ram.bin");

fn boot_core1(psm: &mut pac::PSM, ppb: &mut pac::PPB, sio: &mut bsp::hal::Sio) -> Result<(), Error> {
    let vector_table = 0x20020000;
    let stack_ptr = 0x20030000 - 0x08;
    let reset_vector = 0x20020004 as *const u32;
    
    
    let entry = unsafe { reset_vector.read_volatile() };
    info!("vector: {:X}, stack: {:X}, reset: {:X}, entry: {:X}", vector_table, stack_ptr, reset_vector, entry);
    let prog2 = vector_table as *const usize;
    // Stack pointer is u32 at offset 0 of the vector table
    let stack_ptr2 = unsafe { prog2.read_volatile() };
    // Reset vector is u32 at offset 1 of the vector table
    let reset_vector2 = unsafe { prog2.offset(1).read_volatile() };

    info!("vector: {:X}, stack: {:X}, reset: {:X}", vector_table, stack_ptr2, reset_vector2);

    validate_bootstrap_payload(vector_table, stack_ptr, entry as usize)?;

    info!("We validated okay");
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
    Ok(())
}

/// Perform some basic validation of the program payload
///
/// Validation performed:
/// - Check `vector_table_addr` is correctly aligned
/// - Check that `vector_table_addr` is in a valid memory type (SRAM, XIP_SRAM, Flash)
/// - Check `entry_addr` is after `vector_table_addr`
/// - Check that `entry_addr` is not beyond the end of the memory type
/// (SRAM, XIP_SRAM, Flash) that the `vector_table_addr` is in
fn validate_bootstrap_payload(
    vector_table_addr: usize,
    stack_addr: usize,
    entry_addr: usize,
) -> Result<(), Error> {
    if vector_table_addr & 0x80 != 0 {
        // Vector table was not 32 word (128 byte) aligned
        return Err(Error::InvalidVectorTableAlignment);
    }
    if stack_addr & 0x4 != 0 {
        // Stack pointer must be 4 byte aligned - invalid vector table?
        return Err(Error::InvalidStackPointer);
    }
    if entry_addr <= vector_table_addr {
        // Reset vector pointed to before program to bootload
        return Err(Error::InvalidEntryAddressBelow);
    }

    // Since we didn't pass in the size of the program we don't really know where it ends
    // We can still check that we're within the memory segment the program is loaded into
    if vector_table_addr & 0x2000_0000 == 0x2000_0000 {
        // Program is in RAM
        if entry_addr >= 0x2003_F000 {
            // Reset vector pointed off the end of RAM
            return Err(Error::InvalidEntryAddressAboveRAM);
        }
    } else if vector_table_addr & 0x15000000 == 0x15000000 {
        // Program is in XIP RAM
        if entry_addr >= 0x15004000 {
            // Reset vector pointed off the end of XIP RAM
            return Err(Error::InvalidEntryAddressAboveXIPRAM);
        }
    } else if vector_table_addr & 0x1000_0000 == 0x10000000 {
        // Program is in Flash
        if entry_addr >= 0x1100_0000 {
            // Reset vector pointed off the end of Flash
            return Err(Error::InvalidEntryAddressAboveFLASH);
        }
    } else {
        return Err(Error::InvalidProgramLocation);
    }

    // If we haven't hit any of the previous guard clauses,
    // we have validated successfully
    Ok(())
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

    unsafe {
        bsp::hal::rom_data::memcpy(
            0x20020000 as *mut u8,
            &CORE1[0] as *const u8,
            CORE1.len() as u32,
        );
    }
    // Ensure the write of the program to memory happens before we keep going
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

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

    let mut mc = bsp::hal::multicore::Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio2);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let core1_program_started = unsafe { core1.bootload(0x20020000) };
    info!("Core 1 started okay? {:?}", core1_program_started.is_ok());

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
