==========================================
``npzero`` nPZero G1S power-saving example
==========================================

This example uses the nPZero G1S driver
(:doc:`/os/drivers/special/power/npzero`) with the sensors of the nPZero G1S
Development Kit (:doc:`/platforms/arm/stm32l0/boards/npzero-g1s-dk/index`),
like the example of the Nanopower driver API:

- a LIS2DW12 accelerometer on peripheral 3 (SPI, PMOD 3), read every 5 s; it
  wakes the host up when the X acceleration is above about 61 mg,
- an AS6212 temperature sensor on peripheral 4 (I2C, PMOD 4), read every 30 s;
  it wakes the host up when the temperature is above 25 C or below 10 C.

The host is also woken up by the global time-out, after 300 s.

At each boot of the host, the example:

#. reads and shows the ID of the nPZero and the wake-up reason (STA1 and
   STA2), with the last value of each peripheral that triggered it and the
   ADC values when an ADC channel triggered it,
#. sends the configuration (``NPZEROIOC_CONFIGURE``),
#. waits ``CONFIG_EXAMPLES_NPZERO_IDLE_DELAY`` milliseconds, which leaves time
   to connect a debugger,
#. sends the Idle command (``NPZEROIOC_IDLE``): the nPZero switches the host
   off until the next wake-up.

Usage::

  npzero [-s]

``-s`` shows the ID and the status only, without configuring the nPZero nor
entering Idle state.

The AS6212 board shares the I2C bus with the host, and an unpowered AS6212
board blocks the bus (see *Shared I2C bus* in
:doc:`/os/drivers/special/power/npzero`). So the example keeps both sensors
powered while the host is on (``pswint``), and the ``npzero-g1s-dk`` board
powers them after a reset of the nPZero
(``CONFIG_NPZERO_G1S_DK_PMOD_POWER``). Without this, ``/dev/npzero0`` is not
registered (``npzero_register()`` fails with ``-EAGAIN``) and the example
fails to open it. Do not plug the sensor boards while the host is running.

Configuration
=============

- ``CONFIG_EXAMPLES_NPZERO``: enables the example. It needs ``CONFIG_NPZERO``.
- ``CONFIG_EXAMPLES_NPZERO_DEVPATH``: path of the device, ``/dev/npzero0`` by
  default.
- ``CONFIG_EXAMPLES_NPZERO_IDLE_DELAY``: delay before the Idle command, in
  milliseconds (1000 by default).

The example can be the entry point of the system
(``CONFIG_INIT_ENTRYPOINT="npzero_main"``), as in the ``npzero-g1s-dk:npzero``
configuration, since the host restarts at each wake-up.
