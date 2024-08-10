MEMORY
{
    /*
     * This address is within on-board DDR memory space.
     * DDR controller must be initialized by AT91Bootstrap before loading.
     */
    DDR_MEM : ORIGIN = 0x20000000, LENGTH = 128M
}

_top_of_memory = 0x210000; /* Top of SRAM memory */
_sram_start = 0x200000;  /* Start of SRAM */

RUST_STACK_SIZE = 0x500;
IRQ_STACK_SIZE = 0x60;
FIQ_STACK_SIZE = 0x60;
SYS_STACK_SIZE = 0x40;
ABT_STACK_SIZE = 0x40;
UND_STACK_SIZE = 0x40;
