MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 64K - 0x100

    /* Reserve some RAM for core1's program */
    /* CORE1_PROG : ORIGIN = 0x10020000, LENGTH = 64K */ 

    RAM   : ORIGIN = 0x20000000, LENGTH = 64K
}

EXTERN(BOOT2_FIRMWARE)

SECTIONS {
    /* ### Boot loader */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;
