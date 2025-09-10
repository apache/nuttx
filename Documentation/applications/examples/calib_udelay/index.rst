=============================================
``calib_udelay`` Calibration tool for udelay
=============================================

This tool is used for calibrating the configuration option
``CONFIG_BOARD_LOOPSPERMSEC``. This option is used by NuttX to perform
busy-waiting (i.e., spinning in a loop) when a very basic busy-wait sleep is
needed in board logic. This is also sometimes used when the timer-based sleep
functions do not have a low enough resolution for shorter timings (i.e. system
tick every 1ms but you want to sleep for 100us).

When porting NuttX a new board, this example program is very useful to get a
calibrated value for ``CONFIG_BOARD_LOOPSPERMSEC``.

.. note::

   If you are testing any drivers and have unexpected issues with them, make
   sure that this configuration option has been calibrated. It can cause
   bad/incorrect timings in drivers if not calibrated.

Here is the example output from running the application:

.. code-block:: console

   nsh> calib_udelay

   Calibrating timer for main calibration...
   Performing main calibration for udelay.This will take approx. 17.280 seconds.
   Calibration slope for udelay:
     Y = m*X + b, where
       X is loop iterations,
       Y is time in nanoseconds,
       b is base overhead,
       m is nanoseconds per loop iteration.

     m = 5.33333333 nsec/iter
     b = -199999.99999995 nsec

     Correlation coefficient, R² = 1.0000

   Without overhead, 0.18750000 iterations per nanosecond and 187500.00 iterations per millis.

   Recommended setting for CONFIG_BOARD_LOOPSPERMSEC:
      CONFIG_BOARD_LOOPSPERMSEC=187500

You can simply copy paste the value from the console output and use it as the
value for your board by setting it in the Kconfig menu.

The program is run without any arguments. Configuration options for how the
program runs (taking more measurements, etc.) can be seen in its Kconfig menu.
Press ``h`` with the configuration option highlighted under your cursor to read
the help text about what each option does.
