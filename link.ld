INCLUDE memory.x
ENTRY(reset);

SECTIONS
{
    . = ALIGN(4);
  .text : {
    _stext = .;
    *(.text);    /* Place .text section first so that the reset vector is always placed at the load address */
    *(.text.*);  /* Place other code sections next */
    *(.rodata)                 /* read-only data (constants) */
    *(.rodata*)
    . = ALIGN(4);
    *(.glue_7)
    . = ALIGN(4);
    *(.eh_frame)
    . = ALIGN(4);
     _etext = . ;
  } > DDR_MEM

  . = ALIGN(4);
  .dummy : {
    _edummy = .;
  } > DDR_MEM

  .data : AT (LOADADDR(.dummy)) {
    _sdata = .;
    *(.vectors)
    *(.data)
    *(.data.*)
    _edata = .;
  } > DDR_MEM

  /* collect all uninitialized .bss sections */
  .bss (NOLOAD) : {
    . = ALIGN(4);
    _sbss = .;
    *(.bss)
    *(.bss.*)
    _ebss = .;
  } > DDR_MEM

    /* ## .got */
    /* Dynamic relocations are unsupported. This section is only used to detect relocatable code in
     the input files and raise an error if relocatable code is found */
    .got (NOLOAD) :
    {
        KEEP(*(.got .got.*));
    }

    /* ## Discarded sections */
    /DISCARD/ :
    {
        /* Unused exception related info that only wastes space */
        *(.ARM.exidx);
        *(.ARM.exidx.*);
        *(.ARM.extab.*);
    }

    . = ALIGN(8);
  .stack (NOLOAD) :
  {
    . += IRQ_STACK_SIZE;
    . = ALIGN(8);
    _irqstack = .;

    . += FIQ_STACK_SIZE;
    . = ALIGN(8);
    _fiqstack = .;

    . += ABT_STACK_SIZE;
    . = ALIGN(8);
    _abtstack = .;

    . += UND_STACK_SIZE;
    . = ALIGN(8);
    _undstack = .;

    . += SYS_STACK_SIZE;
    . = ALIGN(8);
    _sysstack = .;
  } >DDR_MEM
}

_romsize = _edata - _stext;
_sramsize = _ebss - _stext;

end = .;  /* define a global symbol marking the end of application */
