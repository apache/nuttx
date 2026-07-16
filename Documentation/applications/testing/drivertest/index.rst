======================================
``drivertest`` cmocka driver test
======================================

``drivertest`` is a collection of applications that exercise NuttX driver
interfaces with the :doc:`cmocka </applications/testing/cmocka/index>` test
framework.  It is not a single test runner.  Each test is registered as a
separate application, such as ``cmocka_driver_rtc`` or
``cmocka_driver_watchdog``, when its configuration dependencies are met.

Configuration and use
=====================

Enable ``CONFIG_TESTING_CMOCKA`` and ``CONFIG_TESTING_DRIVER_TEST``.  The
selected board configuration determines which driver test applications are
built.  ``CONFIG_TESTING_DRIVER_TEST_PRIORITY`` and
``CONFIG_TESTING_DRIVER_TEST_STACKSIZE`` set their common task priority and
stack size.  The simple cmocka self-test and the oneshot test also require
``CONFIG_TESTING_DRIVER_TEST_SIMPLE`` and ``CONFIG_TESTING_ONESHOT_TEST``,
respectively.

Run an enabled test from NSH by entering its application name.  Some tests
require real or simulated devices and may change device state, wait for an
interrupt, or reset the board.  Check the test-specific behavior before adding
one to an unattended test run.

Current test coverage
=====================

The available applications are grouped below.  Only applications whose
dependencies are enabled are included in a given NuttX image.

* Basic and storage tests: ``cmocka_driver_simple`` and
  ``cmocka_driver_block``.
* Time-related tests: ``cmocka_driver_rtc``, ``cmocka_driver_timer``,
  ``cmocka_driver_oneshot``, ``cmocka_posix_timer``, and
  ``cmocka_driver_watchdog``.
* Peripheral and bus tests: ``cmocka_driver_pwm``, ``cmocka_driver_adc``,
  ``cmocka_driver_i2c_spi``, ``cmocka_driver_i2c_write``,
  ``cmocka_driver_i2c_read``, ``cmocka_driver_gpio``,
  ``cmocka_driver_relay``, ``cmocka_driver_uart``, and
  ``cmocka_driver_audio``.
* Display and input tests: ``cmocka_driver_framebuffer``,
  ``cmocka_driver_lcd``, and ``cmocka_driver_touchpanel``.
* Power-management tests: ``cmocka_driver_cpufreq``,
  ``cmocka_driver_regulator``, ``cmocka_driver_pm``,
  ``cmocka_driver_pm_smp``, and ``cmocka_driver_pm_runtime``.
* Arm MPS2 AN500 interrupt tests: ``cmocka_driver_mps2``,
  ``cmocka_driver_mps2_zerointerrupt``, and
  ``cmocka_driver_mps2_isr_signal``.

Watchdog tests
==============

``cmocka_driver_watchdog`` is built when ``CONFIG_WATCHDOG`` is enabled with
at least one of ``CONFIG_BOARDCTL_RESET_CAUSE`` or
``CONFIG_WATCHDOG_TIMEOUT_NOTIFIER``.

With ``CONFIG_BOARDCTL_RESET_CAUSE``, the ``-r`` option selects a hardware
watchdog test case:

* ``-r 0`` feeds the watchdog for a configured interval and then stops
  feeding it.  The watchdog is expected to reset the board.
* ``-r 1`` verifies that the watchdog resets the board while interrupts are
  disabled.
* ``-r 2`` verifies that the watchdog resets the board when an interrupt-level
  watchdog callback does not return.
* ``-r 3`` exercises the watchdog status, keepalive, capture, and stop
  interfaces.  This case is not included on Armv7-A TrustZone builds.

Cases 0 through 2 intentionally reset the target and use the reset cause from
the preceding boot to check the result.  They must therefore be run with a
board that reports reset causes correctly and should not be treated as normal
non-destructive unit tests.  The application accepts ``-d`` for the device
path, ``-o`` for the timeout, ``-t`` for the total ping interval, ``-l`` for
the delay between pings, ``-a`` for the status tolerance, and ``-g`` to skip
the status check.

With ``CONFIG_WATCHDOG_TIMEOUT_NOTIFIER``, the application also runs two
non-destructive notifier tests.  They verify callback priority ordering,
duplicate registration, repeated delivery, unregister behavior, the selected
automonitor action and payload, and concurrent register/notify/unregister
operation.  These notifier tests do not require
``CONFIG_BOARDCTL_RESET_CAUSE`` or a watchdog device node.
