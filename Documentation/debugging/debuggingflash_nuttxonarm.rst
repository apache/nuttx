===================================================================
Debugging / flashing NuttX on ARM with hardware debugger (JTAG/SWD)
===================================================================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/pages/viewpage.action?pageId=139629444


NOTE: If you experience the issues described on this page, you can enable the 
configuration option below to resolve it.

.. code-block:: makefile

    CONFIG_STM32_DISABLE_IDLE_SLEEP_DURING_DEBUG=y

What's the problem?
===================

On some architectures (like ARM Cortex-M3) Idle thread causes the core to stop 
using WFI (Wait For Interrupt) assembly instruction. This effectively stops 
clocking of the core, which is resumed only by some enabled interrupt. This 
causes hardware debuggers to believe that they were disconnected from the 
target, as they lose connection with the now stopped core. For example OpenOCD 
shows errors like these the moment you start the target:

.. code-block:: console

    Error: jtag status contains invalid mode value - communication failure
    Polling target failed, GDB will be halted. Polling again in 100ms
    Error: jtag status contains invalid mode value - communication failure
    Polling target failed, GDB will be halted. Polling again in 300ms
    Error: jtag status contains invalid mode value - communication failure
    Polling target failed, GDB will be halted. Polling again in 700ms
    Error: jtag status contains invalid mode value - communication failure
    Polling target failed, GDB will be halted. Polling again in 1500ms
    Error: jtag status contains invalid mode value - communication failure
    Polling target failed, GDB will be halted. Polling again in 3100ms
    Error: jtag status contains invalid mode value - communication failure
    Polling target failed, GDB will be halted. Polling again in 6300ms
    Error: jtag status contains invalid mode value - communication failure
    Polling target failed, GDB will be halted. Polling again in 6300ms


This makes debugging the code impossible and flashing the chip is much harder -
you have to connect to the chip at the right moment (when it's not disabled 
due to WFI) - the chances of doing that are inverse proportional to the load 
of your system (if your chip spends 99% of time in Idle mode, you have 1% 
chance of connecting and halting it).

Solution
========

Some ARM cores that support disabling of clocking after WFI instruction have 
special configuration options to make debugging possible. One example is STM32 
family - with it's ``DBGMCU->CR`` register it's possible to keep the core 
clocked during power-down modes. If your chip supports such configuration you 
should put it in some early stage of initialization, like in 
``stm32_boardinitialize()`` function. The following code demonstrates the 
change for STM32:

.. code-block:: c

    uint32_t cr = getreg32(STM32_DBGMCU_CR);
    cr |= DBGMCU_CR_STANDBY | DBGMCU_CR_STOP | DBGMCU_CR_SLEEP;
    putreg32(cr, STM32_DBGMCU_CR);

If your chip doesn't provide such options there is no other way than not using 
WFI instruction in up_idle() function.

It should be noted that such modification should be done only for development 
stage, as keeping the core clocked during power-down modes contradicts the 
major purpose of using them - reducing power usage.

In rare cases that you still have problems with connecting to the target 
(especially after power cycle), you should try connecting and halting the chip 
under reset (this is supported by new versions of OpenOCD), by holding the 
reset button while starting OpenOCD or by configuring OpenOCD to do that for 
you.

Work-around
-----------

If you keep the RESET button pressed and run OpenOCD command to connected to 
it, then it will connect sucessful. After connecting you need to keep the 
reset button pressed until you open the telnet connection 
(telnet 127.0.0.1 4444) and execute "reset halt":

.. code-block:: console

    > reset halt
    timed out while waiting for target halted
    TARGET: stm32f1x.cpu - Not halted
      
    in procedure 'reset'
    target state: halted
    target halted due to debug-request, current mode: Thread
    xPSR: 0x01000000 pc: 0x080003d0 msp: 0x20001278

Then release the RESET boot and it will reset correctly.

This work-around was tested on viewtool-stm32f107 board and bypassed the above 
error reported by OpenOCD. The SWD programmer was a STLink-V2 and this was 
the command to connect:

.. code-block:: console

    openocd -f interface/stlink-v2.cfg -f target/stm32f1x_stlink.cfg

The OpenOCD version used was: Open On-Chip Debugger 0.8.0-dev-00307-g215c41c 
(git commit 215c41c)
