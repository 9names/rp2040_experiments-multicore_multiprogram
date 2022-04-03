# Multicore multiprogram experiments

These are some basic test programs aimed to prove code in https://github.com/9names/rp-hal/tree/multicore-bootload is functional.

core1-blinky-flash and core1-blinky-ram are just regular RP2040 programs, with a regular vector table but without a bootloader.
These are converted to plain binaries using cargo objcopy.

core0-flash_core1-flash and core0-flash_core1-flash will take the flash or ram binaries produced above (respectively)
and either keep them in program flash like a regular program or load them into the top of SRAM, before instructing core1 to boot them.

## To build:

cd core0-flash_core1_flash
```console
./run.sh
```
will build `core1-blinky-flash` then launch `core0-flash_core1_flash` via `probe-run`

cd core0-flash_core1_ram
```console
./run.sh
```
will build `core1-blinky-ram` then launch `core0-flash_core1_flash` via `probe-run`
