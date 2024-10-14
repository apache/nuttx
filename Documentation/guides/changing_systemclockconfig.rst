=======================================
Changing the System Clock Configuration
=======================================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/Changing+the+System+Clock+Configuration


Question
========
`Is an STM32 configuration booting with the internal 16 MHz clock, then 
switching later (on command) to an external 25 MHz xtal doable? I don't think 
so, but would you mind confirming that?`

Answer
======

Of course, that is what always happens: The STM32 boots using an internal clock 
and switches to the external crystal source after booting. But I assume that 
you mean MUCH later on, after initialization.

Yes that can be done too. There are only a few issues and things to be aware of:

Custom Clock Configuration
--------------------------

The ``configs/vsn/`` configuration does something like you say. It skips the 
initial clock configuration by defining 
``CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=y``. Then the normal clock 
configuration logic in ``arch/arm/src/stm32/stm32_rcc.c`` is not executed.
Instead, the "custom" clock initialization at ``confgs/vsn/src/sysclock.c``
is called:

.. code-block:: c

    void stm32_clockconfig(void)
    {
      /* Make sure that we are starting in the reset state */

      rcc_reset();

    #if defined(CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG)

      /* Invoke Board Custom Clock Configuration */

      stm32_board_clockconfig();

    #else

      /* Invoke standard, fixed clock configuration based on definitions in board.h */

      stm32_stdclockconfig();

    #endif

      /* Enable peripheral clocking */

      rcc_enableperipherals();
    }

Doing things that way, you can have complete control over when the crystal 
clock source is used. The initial "custom" clock configuration can use an 
internal source, then other custom clock configuration logic can change the 
clock source later.

NOTE: Since this original writing, the VSN configuration has been retired and 
is no long in at config/vsn. The retired code can still be found in the 
`Obsoleted repository <https://bitbucket.org/patacongo/obsoleted/src/master/nuttx/configs/vsn>`_.

Peripheral Clocks
-----------------

The peripheral clock used by many devices to set up things like the SPI 
frequency and UART bard rates. Currently, those peripheral clock frequencies 
are hardcoded in the board.h header file. So you have two options:

1. **Fixed Peripheral Clocking**. Ideally, you would like to keep the peripheral 
   clock frequencies the same in either case. Then life is simple. You could 
   probably use an internal RC clock source as input to a PLL and set up 
   dividers so that you get the same peripheral clocks. Then, I think, from 
   the standpoint of the peripherals, nothing happened.

2. **Variable Peripheral Clocking**. You can make the peripheral clocking 
   variable. I had to do this for the SAMA5Dx family. Look at 
   ``boards/arm/stm32/sama5d4-ek/include/board_sdram.h`` for example. Notice 
   that the frequencies are not constants, but function calls:

.. code-block:: c

    #define BOARD_MAINCK_FREQUENCY     BOARD_MAINOSC_FREQUENCY
    #define BOARD_PLLA_FREQUENCY       (sam_pllack_frequency(BOARD_MAINOSC_FREQUENCY))
    #define BOARD_PLLADIV2_FREQUENCY   (sam_plladiv2_frequency(BOARD_MAINOSC_FREQUENCY))
    #define BOARD_PCK_FREQUENCY        (sam_pck_frequency(BOARD_MAINOSC_FREQUENCY))
    #define BOARD_MCK_FREQUENCY        (sam_mck_frequency(BOARD_MAINOSC_FREQUENCY))

Given that I know that XTAL oscillator frequency I can derive the frequency of 
other clocks. This turns out to be more work than you would think, however, 
because there are probably C pre-processor tests that will now fail. Like:

.. code-block:: c

    #if BOARD_MCK_FREQUENCY > 16000000
    ... do something ...
    #endif

Such logic would have to be converted from a compile time decision to a 
run-time decision, perhaps like this:

.. code-block:: c

    if (BOARD_MCK_FREQUENCY > 16000000)
    {
      ... do something ...
    }

The SAMA5D4-EK case was intended for the case where the software is running out 
of SDRAM and the clocking cannot be reconfigured. Rather, it must derive the 
clocking as it was left by the bootloader. But you could do something like what 
was done for the SAMA5D4-EK when you change the frequency too. You could also 
make the peripheral clocks variable.

Reinitializing Peripherals
--------------------------

Variable Peripheral Clocking
----------------------------

If you did something like what was done for the SAMA5D4-EK when you change the 
frequency, then the peripheral clocks would be variable. The main problem would
then be that you would have to re-initialize the peripherals when the 
peripheral clocking changes. If, for example, the UART was initialized at 
the initial peripheral clock, then you would have to recalculate the BAUD 
divisor if the peripheral clock changes.

But this is not really be a big issue. You can force the UARTs to recalculate 
the BAUD divisor with TERMIOS ioctl calls. You could use the setfrequency() 
methods to recalculate I2C and SPI BAUD divisors. But there are also memory 
card frequencies and more.

Systick Timer
-------------

If the CPU frequency changes, you would have to change the Systick timer 
configuration: It is always driven by the CPU clock

up_mdelay
---------

up_mdelay() provides a low level timing loop and must be re-calibrated for 
anything that causes change in the rate of execution of that timing loop. 
This calibration is not critical and fairly large errors in the calibration 
are tolerable. Hopefully, you could keep the execution rate close enough that
up_mdelay() would not be grossly in error.

Power Management
----------------

This is also the same kind of thing that you would have to do if you wanted to 
switch clocking for power management reasons. NuttX does have a power 
management system and perhaps making use of the power management system 
to manage system clocking changes might be possible. For example, when the 
clocking changes, you could force some power management state change. That 
state change would notify all drivers and, in response, the drivers could 
recalculate their frequency related settings.

Here is some Power Management documentation:

.. toctree::
  :maxdepth: 1

  /components/drivers/special/power/pm/index.rst