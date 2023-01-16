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
  mrc p15, 0, r0, c6, c0
  bkpt
  b dabt_vector
rsvd_vector:
  b   rsvd_vector
irq_vector:
/*
------------------------------------------------------------------------------
 Handles incoming interrupt requests by branching to the corresponding
 handler, as defined in the AIC. Supports interrupt nesting.
------------------------------------------------------------------------------
*/
  sub     lr, lr, #4
  stmfd   sp!, {{lr}}
  mrs     lr, SPSR
  stmfd   sp!, {{r0, lr}}

  /* Write in the IVR to support Protect Mode */

  ldr     lr, =(0xFC020000)
  ldr     r0, [r14, #0x10]
  str     lr, [r14, #0x10]
  /* Dummy read to force AIC_IVR write completion */
  ldr     lr, [r14, #0x04]

  /* Branch to interrupt handler in Supervisor mode */

  msr     CPSR_c, #0x13
  stmfd   sp!, {{r1-r3, r4, r12, lr}}

  /* Check for 8-byte alignment and save lr plus a */
  /* word to indicate the stack adjustment used (0 or 4) */

  and     r1, sp, #4
  sub     sp, sp, r1
  stmfd   sp!, {{r1, lr}}

  blx     r0

  ldmia   sp!, {{r1, lr}}
  add     sp, sp, r1

  ldmia   sp!, {{r1-r3, r4, r12, lr}}
  msr     CPSR_c, #0x12 | 0x80 | 0x40

  /* Acknowledge interrupt */

  ldr     lr, =(0xFC020000)
  str     lr, [r14, #0x38]

  /* Restore interrupt context and branch back to calling code */

  ldmia   sp!, {{r0, lr}}
  msr     SPSR_cxsf, lr
  ldmia   sp!, {{pc}}^

fiq_vector:
  sub     lr, lr, #4
  stmfd   sp!, {{lr}}
  # mrs     lr, SPSR
  stmfd   sp!, {{r0}}

  /* Write in the IVR to support Protect Mode */

  ldr     lr, =(0xF803C000)
  ldr     r0, [r14, #0x10]
  str     lr, [r14, #0x10]

  /* Branch to interrupt handler in Supervisor mode */

  msr     CPSR_c, #0x13
  stmfd   sp!, {{r1-r3, r4, r12, lr}}

  blx      r0

  ldmia   sp!, {{r1-r3, r4, r12, lr}}
  msr     CPSR_c, #0x11 | 0x80 | 0x40

  /* Acknowledge interrupt */
  ldr     lr, =(0xF803C000)
  str     lr, [r14, #0x38]

  /* Restore interrupt context and branch back to calling code */

  ldmia   sp!, {{r0}}
  # msr     SPSR_cxsf, lr
  ldmia   sp!, {{pc}}^

reset_vector:

/*
 * We must configure VBAR for the reset vectors to be at
 * the start of SRAM (0x200000)
 */
mrc p15, 0, r2, c12, c0, 0 /* Read VBAR into R2 */
ldr  r2, =_exception_vectors
mcr  p15, 0, r2, c12, c0, 0

/* Init the stack */
_init_stack:

/* Set up the fast interrupt stack pointer */
mrs     r0, CPSR
bic     r0, r0, #0x1F
orr     r0, r0, #0x11
msr     CPSR_c, r0
ldr     sp, =_fiqstack
bic     sp, sp, #0x7

/* Set up the normal interrupt stack pointer */
bic     r0, r0, #0x1F
orr     r0, r0, #0x12
msr     CPSR_c, r0
ldr     sp, =_irqstack
bic     sp, sp, #0x7

/* Set up the abort mode stack pointer */
bic     r0, r0, #0x1F
orr     r0, r0, #0x17
msr     CPSR_c, r0
ldr     sp, =_abtstack
bic     sp, sp, #0x7

/* Set up the undefined mode stack pointer */
bic     r0, r0, #0x1F
orr     r0, r0, #0x1B
msr     CPSR_c, r0
ldr     sp, =_undstack
bic     sp, sp, #0x7

/* Set up the system mode stack pointer */
bic     r0, r0, #0x1F
orr     r0, r0, #0x1F
msr     CPSR_c, r0
ldr     sp, =_sysstack
bic     sp, sp, #0x7

/* Set up the supervisor mode stack pointer */
bic     r0, r0, #0x1F
orr     r0, r0, #0x13
msr     CPSR_c, r0
ldr     sp, =_top_of_memory
bic     sp, sp, #0x7

/* Branch to the Rust _entry function */
_branch_main:
    BL _entry
    B .
