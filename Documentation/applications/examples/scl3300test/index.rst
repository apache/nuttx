===================================================
``scl3300test`` SCL3300 inclinometer test program
===================================================

This program exercises the :doc:`Murata SCL3300 inclinometer driver
</os/drivers/special/sensors/scl3300>` through its uORB topics and ioctls.
Use it to check a new board or wiring. It reports PASS or FAIL for each
test and returns ``EXIT_SUCCESS`` only if every test passes.

Configuration
=============

``CONFIG_EXAMPLES_SCL3300TEST``
  Enable the program. Depends on ``CONFIG_SENSORS_SCL3300``.

``CONFIG_EXAMPLES_SCL3300TEST_PROGNAME``
  Program name, ``scl3300test`` by default.

``CONFIG_EXAMPLES_SCL3300TEST_PRIORITY`` and ``CONFIG_EXAMPLES_SCL3300TEST_STACKSIZE``
  Task priority (100) and stack size (4096).

The ``stm32f4discovery:scl3300`` configuration has everything else the
program needs. Add ``CONFIG_EXAMPLES_SCL3300TEST=y`` to it.

Usage
=====

.. code-block:: console

   nsh> scl3300test [-n devno] <command>

``-n devno`` selects the topic instance: ``sensor_inclinometer<devno>``
and ``sensor_accel<devno>``. The default is 0.

============ ================================================================
Command      Test
============ ================================================================
``whoami``   ``SNIOC_WHO_AM_I`` must return 0xC1.
``selftest`` ``SNIOC_SELFTEST`` must succeed. Keep the device at rest.
``info``     Prints ``SNIOC_GET_INFO`` for each topic.
``read``     Reads 200 inclinometer samples at 10 ms and prints the mean and
             standard deviation of each axis.
``modes``    Selects Modes 1, 2, 3, 4 and 1 again with
             ``SNIOC_SET_OPERATIONAL_MODE``. Prints the info and the noise in
             each. Then checks that mode 5 is rejected with ``EINVAL``.
``power``    Selects Mode 4 and ``SCL3300_POWER_DOWN_IDLE``. Then opens and
             closes the topic 100 times, reading one sample each time, so
             the device powers down and wakes up 100 times. With the
             accelerometer topic enabled, it also checks that the mode
             survives (the accelerometer resolution depends on the mode).
             Restores Mode 1 and ``SCL3300_POWER_ALWAYS_ON``.
``reset``    ``SNIOC_RESET``, then the ``read`` test.
``all``      All of the above, in this order.
============ ================================================================

Run the tests with the device at rest. Vibration shows up as noise and can
fail the self-test.

Example
=======

Output on an STM32F4Discovery with the sensor lying flat (shortened):

.. code-block:: console

   nsh> scl3300test all
   WHOAMI: 0xc1 -> PASS
   SELFTEST: 0 -> PASS
   INFO inclinometer: Murata SCL3300 v1, 1.20 mA, range 90.0000, resolution 0.0054931, interval 500..2147483647 us
   INFO accel: Murata SCL3300 v1, 1.20 mA, range 11.7828, resolution 0.0016365, interval 500..2147483647 us
   READ:
   ...
   MODE 1:
   ...
   MODE 3:
   INFO inclinometer: Murata SCL3300 v1, 1.20 mA, range 90.0000, resolution 0.0054931, interval 500..2147483647 us
   INFO accel: Murata SCL3300 v1, 1.20 mA, range 9.8190, resolution 0.0008182, interval 500..2147483647 us
     200/200 samples, temperature 25.25 C
     X: mean   -1.5023 deg, stddev 0.00801 deg
     Y: mean    0.3013 deg, stddev 0.00545 deg
     Z: mean   88.4676 deg, stddev 0.00814 deg
   MODE 4:
   ...
   MODE 1:
   INFO inclinometer: Murata SCL3300 v1, 1.20 mA, range 90.0000, resolution 0.0054931, interval 500..2147483647 us
   INFO accel: Murata SCL3300 v1, 1.20 mA, range 11.7828, resolution 0.0016365, interval 500..2147483647 us
     200/200 samples, temperature 25.31 C
     X: mean   -1.4989 deg, stddev 0.01770 deg
     Y: mean    0.2962 deg, stddev 0.01329 deg
     Z: mean   88.4719 deg, stddev 0.01715 deg
   MODE 5: rejected -> PASS
   POWER: 100 power-down/wake-up cycles
   POWER: 0 failures -> PASS
   RESET: 0 -> PASS
   READ:
   ...
   ALL: PASS

The noise is lowest in Mode 4 and highest in Mode 2, following the
datasheet.
