.section start
  .text

  .globl reset
  .align 4

reset:

/* Exception vectors (should be a branch to be detected as a valid code by the rom */
_exception_vectors:
  b   reset_vector    /* reset */
  b   undef_vector   /* Undefined Instruction */
  b   swi_vector     /* Software Interrupt */
  b   pabt_vector    /* Prefetch Abort */
  b   dabt_vector    /* Data Abort */
.word    _romsize  /* Size of the binary for ROMCode loading */
  b   irq_vector  /* IRQ : read the AIC */
  b   fiq_vector      /* FIQ */

undef_vector:
  b   undef_vector
swi_vector:
  b   swi_vector
pabt_vector:
  b   pabt_vector
dabt_vector:
  subs  pc, r14, #4  /* return */
  nop
rsvd_vector:
  b   rsvd_vector
irq_vector:
  b   irq_vector
fiq_vector:
  b   fiq_vector
reset_vector:

/*
 * We must configure VBAR for the reset vectors to be at
 * the start of SRAM (0x200000)
 */
mrc p15, 0, r2, c12, c0, 0 /* Read VBAR into R2 */
ldr  r2, =_sram_start
mcr  p15, 0, r2, c12, c0, 0

/* Init the stack */
_init_stack:

ldr r0, =_top_of_memory

# Set the stack for IRQ mode
msr cpsr_c, #0xd2
mov sp, r0

# Set the stack for FIQ mode
msr cpsr_c, #0xd1
mov sp, r0

# Set the stack for Abort mode
msr cpsr_c, #0xd7
mov sp, r0

# Set the stack for Undefined Instruction mode
msr cpsr_c, #0xdb
mov sp, r0

# Back to Supervisor mode, (IRQ and FIQ both masked, Arm instruction
# set) set the stack for Supervisor mode
msr cpsr_c, #0xd3
mov sp, r0

/* Clear Abort condition if it is pending with help of the abort handler */
ldr  r1, =(0x100)
mrs  r0, cpsr
eor  r0, r0, r1
msr  cpsr_x, r0
nop
eor  r0, r0, r1
msr  cpsr_x, r0

/* Branch to the Rust _entry function */
_branch_main:
    BL _entry
    B .

.align
_lp_data:
        .word _edummy
        .word _sdata
        .word _edata

_lp_bss:
  .word _sbss
  .word _ebss
