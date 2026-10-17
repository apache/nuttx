===================================================
``pmstress`` Power Management Current Staircase
===================================================

``pmstress`` walks the power management states in a fixed, timed sequence,
so that a logging power supply can record the current of each state with
no debugger or console attached. The sequence is: active (busy), idle and
standby, each for ``CONFIG_EXAMPLES_PMSTRESS_STEP_SECS``, then the deepest
state until power off. A 1 s full-speed burst comes before each step, so
the current trace shows where each step starts.

``pmstress`` first turns the user LEDs off and prints the schedule. Then
it waits ``CONFIG_EXAMPLES_PMSTRESS_DISCONNECT_SECS``, so that you can
disconnect the debugger and the console. A debugger keeps power domains on.

On the rp23xx, ``pmstress suspend [<seconds>]`` (default 10) suspends to
RAM with the ``BOARDIOC_RP23XX_SUSPEND`` boardctl() command. It sets an RTC
alarm on ``/dev/rtc0`` with ``RTC_SET_RELATIVE`` as the wake, and prints the
wake source after the resume. This needs ``CONFIG_RP23XX_PM_SUSPEND``,
``CONFIG_BOARDCTL_IOCTL``, ``CONFIG_RTC_DRIVER`` and ``CONFIG_RTC_ALARM``.
See :doc:`/platforms/arm/rp23xx/index`.

Configuration
=============

- ``CONFIG_EXAMPLES_PMSTRESS`` -- Enable the example. Needs ``CONFIG_PM``.
- ``CONFIG_EXAMPLES_PMSTRESS_STEP_SECS`` -- Seconds to hold each state.
  Default 30.
- ``CONFIG_EXAMPLES_PMSTRESS_DISCONNECT_SECS`` -- Seconds to wait before the
  run starts. Default 30.
- ``CONFIG_EXAMPLES_PMSTRESS_LEDPATH`` -- User LED device that is turned off
  during the run. Default ``/dev/userleds``. Needs ``CONFIG_USERLED_LOWER``.
- ``CONFIG_EXAMPLES_PMSTRESS_PROGNAME``, ``CONFIG_EXAMPLES_PMSTRESS_PRIORITY``
  and ``CONFIG_EXAMPLES_PMSTRESS_STACKSIZE`` -- Program name, priority and
  stack size.
