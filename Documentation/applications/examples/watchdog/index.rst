``watchdog`` Watchdog Timer
===========================

A simple test of a watchdog timer driver. Initializes starts the watchdog timer.
It pings the watchdog timer for a period of time then lets the watchdog timer
expire... resetting the CPU is successful. This example can ONLY be built as an
NSH built-in function.

This test depends on these specific Watchdog/NSH configurations settings (your
specific watchdog hardware settings might require additional settings).

- ``CONFIG_WATCHDOG`` – Enables watchdog timer support support.
- ``CONFIG_NSH_BUILTIN_APPS`` – Build the watchdog time test as an NSH built-in
  function.

Specific configuration options for this example include:

- ``CONFIG_EXAMPLES_WATCHDOG_DEVPATH`` – The path to the Watchdog device. Default:
  ``/dev/watchdog0``.
- ``CONFIG_EXAMPLES_WATCHDOG_PINGTIME`` – Time in milliseconds that the example
  will ping the watchdog before letting the watchdog expire. Default: ``5000``
  milliseconds.
- ``CONFIG_EXAMPLES_WATCHDOG_PINGDELAY`` – Time delay between pings in
  milliseconds. Default: ``500`` milliseconds.
- ``CONFIG_EXAMPLES_WATCHDOG_TIMEOUT`` – The watchdog timeout value in
  milliseconds before the watchdog timer expires. Default: ``2000`` milliseconds.
