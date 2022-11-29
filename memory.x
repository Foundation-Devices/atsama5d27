MEMORY
{
    /*
     * This address is within on-board DDR memory space.
     * DDR controller must be initialized by AT91Bootstrap before loading.
     */
    DDR_MEM : ORIGIN = 0x23f00000, LENGTH = 10240K
}

_top_of_memory = 0x210000; /* Top of SRAM memory */
_sram_start = 0x200000;  /* Start of SRAM */
