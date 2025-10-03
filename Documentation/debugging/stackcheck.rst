====================================
Stack Overflow Check
====================================

Overview
--------

Currently NuttX supports three types of stack overflow detection:
    1. Stack Overflow Software Check During Function Call
    2. Stack Overflow Software Check During Context Switching
    3. Stack Overflow Hardware Check
    4. Stack Canary Check

The software stack detection during function call includes two implementation ideas:
    1. Implemented by coloring the stack memory
    2. Implemented by comparing the sp and sl registers

The software stack detection during context switching includes two implementation ideas:
    1. Implemented by coloring the stack memory
    2. Implemented by checking the bottom memory of the stack and the sp register

Support
-------

Software and hardware stack overflow detection implementation,
currently only implemented on ARM Cortex-M (32-bit) series chips
Stack Canary Check is available on all platforms

Stack Overflow Software Check During Function Call
--------------------------------------------------

1. Memory Coloring Implementation Principle
    1. Before using the stack, Thread will refresh the stack area to 0xdeadbeef
    2. When Thread is running, it will overwrite 0xdeadbeef
    3. up_check_tcbstack() detects 0xdeadbeef to get the stack peak value

    Usage:
        Enable CONFIG_STACK_COLORATION

2. Compare sp and sl
    When compiling the program, keep r10 and use r10 as stackbase::
    '''
    ARCHOPTIMIZATION += -finstrument-functions -ffixed-r10

    Each function will automatically add the following when entering and exiting:
    __cyg_profile_func_enter
    __cyg_profile_func_exit

    Usage:
        Enable CONFIG_ARMV8M_STACKCHECK or CONFIG_ARMV7M_STACKCHECK

Stack Overflow Software Check During Context Switching
------------------------------------------------------

1. Determine by detecting the number of bytes specified at the bottom of the stack.
2. Check if the sp register is out of bounds.

Usage:
    Set the detection length by STACKCHECK_MARGIN

Stack Overflow Hardware Check
-----------------------------

1. Set MSPLIM PSPLIM when context switching
2. Each time sp is operated, the hardware automatically compares sp and PSPLIM. If sp is lower than PSPLIM, crash

Usage:
    Enable CONFIG_ARMV8M_STACKCHECK_HARDWARE

Stack Canary Check
-----------------------------

1. Add a canary value to the stack
2. When the thread is running, the canary value is overwritten
3. When the thread is running, the canary value is compared with the original value
4. If the value is different, it means that the stack is overflowed

Usage:
    Enable CONFIG_STACK_CANARIES
