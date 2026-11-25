=======================================
Hynitron CST92xx Capacitive Touchscreen
=======================================

The Hynitron CST92xx family (CST9217, CST9220) are I2C capacitive
multi-touch controllers found on small round and square AMOLED panels,
for example the Waveshare ESP32-S3-Touch-AMOLED-2.16.  They use 16-bit
register addresses and live at the 7-bit I2C address ``0x5a``.

The driver is a touchscreen lower half for the common touchscreen upper
half (see :doc:`../touchscreen`).  Once registered the device appears as
``/dev/inputN`` and applications read ``struct touch_sample_s`` samples.

- Header: ``include/nuttx/input/cst92xx.h``
- Driver: ``drivers/input/cst92xx.c``

Configuration
=============

- ``CONFIG_INPUT=y``
- ``CONFIG_INPUT_TOUCHSCREEN=y``
- ``CONFIG_INPUT_CST92XX=y`` (selects ``CONFIG_SCHED_HPWORK``)
- ``CONFIG_INPUT_CST92XX_MAX_POINTS``: number of contacts reported per
  sample (default 1).  Values above 1 enable multi-touch; readers must
  then read ``SIZEOF_TOUCH_SAMPLE_S(maxpoint)`` bytes per sample.
- ``CONFIG_INPUT_CST92XX_NBUFFERS``: samples buffered per open file.
- ``CONFIG_INPUT_CST92XX_POLLMODE`` / ``CONFIG_INPUT_CST92XX_POLL_PERIOD``:
  poll the controller while the device is open instead of using INT.

Board Support
=============

The board provides an initialized I2C master and a persistent
``struct cst92xx_config_s``:

- ``address``, ``frequency``: I2C address (``CST92XX_I2C_ADDRESS``) and
  bus speed.
- ``xres``, ``yres``: output resolution.  If zero, the resolution read
  from the controller firmware is used.
- ``flags``: any of ``TOUCH_FLAG_SWAPXY``, ``TOUCH_FLAG_MIRRORX`` and
  ``TOUCH_FLAG_MIRRORY``.  The driver swaps first and then mirrors
  against the output resolution.
- ``attach``: attach the driver interrupt handler to the INT GPIO
  (active low, falling edge).  Required unless polled mode is used.
- ``enable``: enable or disable the INT GPIO interrupt.  Required unless
  polled mode is used.
- ``reset``: drive the RST pin (``true`` holds the controller in reset).
  Optional.

Then call ``cst92xx_register(i2c, &config, minor)`` during board
bring-up:

.. code-block:: c

   static const struct cst92xx_config_s g_cst92xx_config =
   {
     .address   = CST92XX_I2C_ADDRESS,
     .frequency = 400000,
     .xres      = 480,
     .yres      = 480,
     .attach    = board_cst92xx_attach,
     .enable    = board_cst92xx_enable,
     .reset     = board_cst92xx_reset,
   };

   ret = cst92xx_register(i2c, &g_cst92xx_config, 0);

Operation
=========

``cst92xx_register()`` pulses RST (if provided), switches the controller
into its information mode to verify the firmware check code and chip
type (``0x9217`` or ``0x9220``) and to read the native resolution, then
returns it to normal reporting mode, registers ``/dev/inputN`` and
attaches the interrupt.  The interrupt is enabled while at least one
file is open.

Each INT edge queues a worker on the high-priority work queue.  The
worker reads the touch report at ``0xd000`` and acknowledges it by
writing ``0xab`` back to that address.  Reports without the ``0xab``
marker, or still holding the previous acknowledge, are ignored.
Contacts are tracked by their hardware ID and reported as:

- first contact: ``TOUCH_DOWN | TOUCH_ID_VALID | TOUCH_POS_VALID``
- continued contact: ``TOUCH_MOVE | TOUCH_ID_VALID | TOUCH_POS_VALID``
- lost contact: ``TOUCH_UP | TOUCH_ID_VALID | TOUCH_POS_VALID`` at the
  last reported position

When the controller signals that the panel is covered (palm detect) all
contacts are released.

Testing
=======

``apps/examples/touchscreen`` (``tc``) reads single-point samples from
``CONFIG_EXAMPLES_TOUCHSCREEN_DEVPATH`` (default ``/dev/input0``) and
works with ``CONFIG_INPUT_CST92XX_MAX_POINTS=1``.  With more points a
sample carries one point per active or just-released contact, so its
size varies with ``npoints``.
