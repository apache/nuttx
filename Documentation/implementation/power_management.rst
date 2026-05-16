.. power-management:

================
Power Management
================

Power Management (PM) Subsystem
===============================

I would expect that the logic that performs clocking adjustments would utilize
the NuttX Power Management (PM) subsystem. That subsystem basically implements
a random walk. It collects and monitors system utilization information from
devices drivers. This utilization information comes from critical device
drivers via calls to::

  void pm_activity(int domain, int priority);

This function is called by a device driver to indicate that it is performing
meaningful activities (non-idle). This increments an activity counter and will
prevent entering reduced power states.

For a system driver by a human interface, ``pm_activity()`` might be called
with touchscreen in puts and/or keypad inputs.
As long as the human is interacting with the device it remains non-idle.

When there is no further activity, the activity counter will decrease
and when it crosses a threshold, it will initiate a change to a lower power
usage state.

There are many actions that systems may take when the reduced power state
is entered. They may, for example, dim that backlight on a display or enable
MCU-specific reduced power consumptions modes. And for the purposes
of this discussion, they may also reduce the clocking to the CPU.

Alternative way to conserve power is using :ref:`tickless` with some
(dis)advantages you need to consider to know if it fits your needs.


.. _dynamic-clocking:

Dynamic Clocking
================

Most configurations are created with fixed clock configurations.
One strategy for power management is to use variable, dynamic clocking
where the CPU clock frequency is lowered during periods of inactivity.

Clock Update Function
---------------------

Variable clocking is not difficult to achieve but most ports are created
with fixed clock configuration using settings defined in the ``board.h``
header file. For example, Kinetis has::

  void kinetis_clockconfig(void)

In order to make clocking variable a few things are needed:

* The initial board settings should still be used at power up,
  but there needs to be a new clock initialization function,
  perhaps ``kinetis_clock_update()`` that takes a structure containing
  all of the clock settings.
* The existing ``kinetis_clockconfig()`` would need to be gutted,
  most of the logic being moved to ``kinetis_clock_update()``.
  ``kinetis_clockconfig()`` would just create the structure using the constant
  settings from the ``board.h`` header file then call
  ``kinetis_clock_update()`` to establish the initial clock configuration.

.. note:: Additional complexities may be associated with changing the clocking
   other than just controlling dividers, PLLs, and clock sources. For example,
   FLASH wait states. When increasing the clocking, the FLASH wait states must
   be increased BEFORE reconfiguring the faster clocking.
   But when reducing the clocking, FLASH wait states cannot be reduced until
   AFTER reconfiguring the slower clocking.

Handling Driver Dependencies
----------------------------

The first part of the problem was modifying the clock configuration logic
to support variable configurations.
The second part of the problem is that now all of the devices that depend
on clocking such as the timer, the system timer, all MCU-specific timers,
UARTs, and perhaps other device need to be notified of the change
in clock frequency.

* The system timer, the SysTick in Cortex-M MCUs, needs to be notified
  of the of the clock frequency change so that it can re-calculate
  the system timer tick interrupt so that there is no disruption in timing.
* The same may be true of other MCU-specific timers. They may also need to
  recalculate certain clocking. An MCU-specific timer may, for example,
  be providing system time in Tickless mode.
* If serial devices are used, such as for a serial console, then
  serial drivers must be notified of the clock change so that they can
  recalculate their BAUD settings without lost of serial communications.
* Other devices such as SPI, I2C, I2S, SDIO, USB, etc. may also need to
  be notified of the change in the clock configuration to behave correctly
  across the clocking change.

Full functionality is not normally expected in low power consumption states.
So the most typical behavior for drivers in response to reduced power
consumption state changes is simply to shut themselves down (turn off
the peripheral, disable clocking to the peripheral, and other actions
as appropriate).

The above discussion only applies to peripherals that you expect
to continue normal operation in the reduced power state.
You may, as an example, want to keep timing accuracy throughout
the low power state or you may want to preserve serial debug output
in the reduced power state.


Driver PM Callback Functions
----------------------------

The NuttX PM subsystem provides the glue that integrates both parts
of the clock management problem.

The first part is managed by PM activity monitor and PM state changes.
The PM subsystem integrates the second part of the problem using PM driver
callback functions. The timing sensitive device drivers can be notified
of the change in clocking using the driver callbacks that are a port
of NuttX Power Management (PM) functionality.

Each timing sensitive device driver needs to register for a PM callback and,
on each callback, it must recalculate its frequency settings using the current
clock configuration.

To receive this callbacks, the driver logic needs to provide functions
with prototypes like:

.. code-block:: c

  /* Power management callback function prototypes */

  #ifdef CONFIG_PM
  static void xyz_pm_notify(struct pm_callback_s *cb, int domain,
                            enum pm_state_e pmstate);
  static int xyz_pm_prepare(struct pm_callback_s *cb, int domain,
                            enum pm_state_e pmstate);
  #endif

Which are used to notify a callback vtable like:

.. code-block:: c

  /* Power management callback function vtable */

  #ifdef CONFIG_PM
  static struct pm_callback_s g_xyz_pmcb =
  {
    .notify  = xyz_pm_notify,
    .prepare = xyz_pm_prepare,
  };
  #endif

The ``prepare()`` callback method gives the driver some advance notification
of the pending PM state change.

Some drivers may need to put the device in a safe state in the ``prepare()``
callback.
A serial driver, for example, may want to disable RX and TX so that garbage
is not generated or received during the clock change.

The ``notify()`` callback method signifies the actually state change
and it is in this method that the driver logic should recalculate its timing
parameters based on the new clock settings and also resume an operations
that were disabled by the ``prepare()`` callback.

The driver registers its PM callback vtable by calling pm_register() like:

.. code-block:: c

  /* Register to receive power management callbacks */

  int ret = pm_register(&g_xyz_pmcb);

Dynamic Clock Frequency Utility Functions
-----------------------------------------

Most timers and drivers in the system us the constant frequency values defined
in the ``board.h`` header file. In order to have variable time, the drivers
must be able to determine the current clock frequencies dynamically,
not via a fixed, constant definitions.
This means that there must be clock functions that derive various clock
firequencies in the clock distribution knowing only the input frequency
(i.e., crystal frequency of internal clock selection).

When the driver receives the PM ``notify()`` informing it that the clocking
may have changed, it must call these functions to get the new clock settings
instead of using the constant settings in board.h.

There is something similar in the source tree now for the case where NuttX
is started by a bootloader. In that case, the clock configuration is
set up by the bootloader, usually u-boot, and each driver that needs to know
clock frequencies must query the clock configuration to determine
the frequency.

For the SAMA5D4-EK, that is still handled by definitions in the ``board.h``
header file. See, for example, ``boards/arm/sama5/sama5d4-ek/include/*.h``.
``board.h`` includes another file that defines the clock setup based
on the configuration. If the SAMA5D4-EK is running out of SDRAM,
then it must have been started by a bootloader and the definitions appear in
``boards/ar/sama5/sama5d4-ek/include/board_sdram.h``.
In that case, the macros are redefined so that they map to a function call
to determine the clock frequency:

.. code-block:: c

  #define BOARD_MAINCK_FREQUENCY     BOARD_MAINOSC_FREQUENCY
  #define BOARD_PLLA_FREQUENCY       (sam_pllack_frequency(BOARD_MAINOSC_FREQUENCY))
  #define BOARD_PLLADIV2_FREQUENCY   (sam_plladiv2_frequency(BOARD_MAINOSC_FREQUENCY))
  #define BOARD_PCK_FREQUENCY        (sam_pck_frequency(BOARD_MAINOSC_FREQUENCY))
  #define BOARD_MCK_FREQUENCY        (sam_mck_frequency(BOARD_MAINOSC_FREQUENCY))

Something like this could also be done for other architectures to support
dynamic clock management.

You would probably have to do something similar if change the clocking.
``SysTick`` and each device would need a callback from the PM subsystem and each
would have to re-calculate is BAUD based on the new clock settings using
such functions calls to get the dynamic clock frequency.
