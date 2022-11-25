MEMORY
{
    /* The origin is shifted to accommodate the AT91Bootstrap bootloader */
    SRAM : ORIGIN = 0x23f00000, LENGTH = 10240K
}

_top_of_memory = 0x210000; /* Top of memory */
_sram_start = 0x200000;  /* Start of SRAM */
