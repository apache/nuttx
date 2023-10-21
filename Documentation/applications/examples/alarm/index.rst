``alarm`` RTC Alarm
===================

A simple example that tests the alarm IOCTLs of the RTC driver.

Dependencies:

- ``CONFIG_RTC_DRIVER`` –  RTC driver must be initialized to allow user space
  access to the RTC.
- ``CONFIG_RTC_ALARM`` – Support for RTC alarms must be enabled.

Configuration:

- ``CONFIG_EXAMPLES_ALARM`` – Enable the RTC driver alarm test.
- ``CONFIG_EXAMPLES_ALARM_PROGNAME`` –  This is the name of the program that will
  be used when the NSH ELF program is installed.
- ``CONFIG_EXAMPLES_ALARM_PRIORITY`` – Alarm daemon priority.
- ``CONFIG_EXAMPLES_ALARM_STACKSIZE`` – Alarm daemon stack size.
- ``CONFIG_EXAMPLES_ALARM_DEVPATH`` – RTC device path (``/dev/rtc0``).
- ``CONFIG_EXAMPLES_ALARM_SIGNO`` – Alarm signal.
