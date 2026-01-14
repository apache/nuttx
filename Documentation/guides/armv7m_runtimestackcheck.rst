===============================
ARMv7-M Run Time Stack Checking
===============================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/ARMv7-M+Run+Time+Stack+Checking

Overview
========

Nuttx supports facilities to verify the dynamically allocated stacks and fixed 
stacks used by the tasks and interrupt context running under Nuttx. There are 2
types of stack checking that can be used together or separately.

1. The Stack Monitor
2. Per function Call (ARMV7 Only)

The Stack Monitor
-----------------

The use of the Stack Monitor application requires that 
``CONFIG_STACK_COLORATION`` be enabled. This compile time option enables the 
writing of a know pattern ``STACK_COLOR`` to the stack memory at creation time. 
In the case of the idle task and interrupt, this is done in the code that runs 
just after reset at startup. This is known as stack coloring.

Once the pattern has been established, the functions ``up_check_stack`` and 
friends are used to perform the stack checking by finding the lowest word of 
the pattern in the allocated stack.

The Stack Monitor is enabled with ``CONFIG_SYSTEM_STACKMONITOR`` which will 
enable a daemon that will periodically run and check the stack penetration of 
the tasks running on the system.

The stack monitor is good to help size and check the usage of tasks. However it 
is not really useful to detect, certain kinds, nor the cause of a stack overrun.

The reason for this is because a corrupted stack may not be eaten away at. It 
may have overruns where the stack pointer is set way below the stack bottom in 
a function call as it allocates local variables on the stack. The code in the 
function call can then corrupt the memory below the stack bottom, restores the 
stack pointer and returns to the caller without actually overwriting the 
coloring at the base of the stack.

This brings us the the next method of stack checking.


Per function Call
-----------------
This method of stack checking leverages the profiler hook mechanism supported 
by the compiler. Once enabled using ``CONFIG_ARMV7M_STACKCHECK``, one register 
is set aside (``R10`` is the default) and the value of the base of stack is 
saved there (``rBS``). Then every function call will have a preamble and a
postamble code added to it. The preamble (``__cyg_profile_func_enter``) checks 
the current stack pointer, minus a margin of 64 bytes (with an additional 136 
bytes for the ``FP`` registers) against the value in the reserved register 
``rBS``. If the computed value lies below the value in ``rBS`` a hard fault is 
generated. The postamble code (``__cyg_profile_func_exit``)  just returns to 
the caller.

The rationale for subtracting the margin can be viewed two ways. If the 
configuration is not using a separate ISR stack, then the space reserved will 
accommodate the context save of the CPU and optionally the FPU registers to 
service an interrupt on the users stack. If the configuration is using a 
separate ISR stack, some the 64 bytes will accommodate the transition to the 
interrupt stack and the remaining 60-200 bytes are just margin. Either way 
stacks should always be allocated with at least 200 bytes of margin.

Because of the reserved register rBS contains the current context's stack base, 
and rBS is not updated on the entry to an ISR, it is not possible to check the 
stack penetration for an interrupt with Per function Call stack checking.

One thing to consider is the impact on code size and speed this method of stack 
checking will have. Each function will have two additional call and return 
instructions added to it. In the execution path of each function, there will be 
an added set of instructions to perform the preamble and postamble 
functionality. In a call tree that is nested several layers deep, this can add 
up. In one particular use, we saw an increase of 30% to 35% additional CPU 
utilization required to support per function call stack checking.

Is this just a debugging tool? One could imagine that in a mission critical 
application, this might be part of a release build if the code size and speed 
impact can be tolerated.

As of commit 4942867 Register R11 will contain the value that the stack pointer 
would hit that caused the fault. This can be used to calculate the stack size 
needed for that task that faulted. To do so, take the difference of R10-R11 and 
round it up by 12 bytes (The round up is to make up for the 8 byte stack 
alignment and 4 byte decrease that may happen in the stack allocation) Then add 
that amount to the failing tasks stack size.

Details for Support of Per Call Stack Checking
----------------------------------------------

Currently only ARMV7 derivatives support Per Call Stack Checking. Support 
requires the following components:

The start function must establish the value in ``rBS`` (``R10`` by default see 
below). Yet to do this the start function must NOT have the preamble and 
postamble code added to it. This is accomplished with the use the following 
gcc attribute:

.. code-block:: c

    #ifdef CONFIG_ARMV7M_STACKCHECK
    /* we need to get r10 set before we can allow instrumentation calls */
    
    void __start(void) __attribute__ ((no_instrument_function));
    #endif

...

.. code-block:: c

    void __start(void)
    {
      const uint32_t *src;
      uint32_t *dest;
    
    #ifdef CONFIG_ARMV7M_STACKCHECK
    
      /* Set the stack limit before we attempt to call any functions */
    
      __asm__ volatile ("sub r10, sp, %0" : : "r" (CONFIG_IDLETHREAD_STACKSIZE -64) : );
    #endif

The minus 64 is setting the limit 64 bytes above the bottom of the stack. Note: 
This may be adding another 64 bytes of margin

For the creation of a task's context the following code is needed to set up 
``rBS``

.. code-block:: c

    void up_initial_state(struct tcb_s *tcb)
    {
      struct xcptcontext *xcp = &tcb->xcp;
    
      /* Initialize the initial exception register context structure */
    
      memset(xcp, 0, sizeof(struct xcptcontext));

      /* Save the initial stack pointer */

      xcp->regs[REG_SP]      = (uint32_t)tcb->adj_stack_ptr;

    #ifdef CONFIG_ARMV7M_STACKCHECK
      /* Set the stack limit value */

      xcp->regs[REG_R10]     = (uint32_t)tcb->stack_alloc_ptr + 64;
    #endif

And finally up_stackcheck.c needs to be included in the build and the compiler 
flags set to reserve ``R10`` and enable the instrumentation.

This is done for a given architecture in nuttx/arch/arm/src/<arch>/Make.defs:

.. code-block:: makefile

    ifeq ($(CONFIG_ARMV7M_STACKCHECK),y)
    CMN_CSRCS += up_stackcheck.c
    endif

The compiler flags are added in the nuttx/arch/arm/src/armv7-m/Toolchain.defs

.. code-block:: makefile

    # enable precise stack overflow tracking
    ifeq ($(CONFIG_ARMV7M_STACKCHECK),y)
    INSTRUMENTATIONDEFINES   = -finstrument-functions -ffixed-r10
    endif

Other Considerations
--------------------

If using the export build feature of Nuttx: For the runtime stack checking both 
the Application and Nuttx need to be built with the 
``CONFIG_ARMV7M_STACKCHECK`` option set the same state, enabled or disabled. 
Any mismatch will created either compile time or runtime issues.