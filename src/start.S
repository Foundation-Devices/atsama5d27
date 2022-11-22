.section .text.start
.global start

start:

# Set the stack pointer
ldr r0, =_stack_start
mov sp, r0

# Run entrypoint
BL _entry
B .
