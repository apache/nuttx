======================
Remote Control Devices
======================

NuttX provides a Remote Control (RC) framework for infrared style input and
output devices. The userspace interface follows the LIRC model and exposes
each device as a character node such as ``/dev/lirc0``.

Like many NuttX drivers, the RC subsystem is split into two parts:

#. a generic upper half in ``drivers/rc/lirc_dev.c``, and
#. a lower half supplied by platform or device specific code.

The upper half handles the character driver registration, per-open buffering,
``poll()`` support, and the common ``ioctl()`` interface. The lower half is
responsible for the hardware specific work such as receiving pulse timings,
transmitting IR data, or reporting decoded scancodes.

Files related to the RC framework are located in:

- ``include/nuttx/lirc.h``
- ``include/nuttx/rc/lirc_dev.h``
- ``drivers/rc/lirc_dev.c``

Application Programming Interface
=================================

Applications should include the following header files:

.. code-block:: c

  #include <nuttx/lirc.h>
  #include <sys/ioctl.h>

Each RC device is registered as a POSIX character device in ``/dev``.
Applications open the device with the standard ``open()`` call and then use
``read()``, ``write()``, ``poll()``, and ``ioctl()`` just like other character
drivers.

The RC framework supports three kinds of lower-half devices:

- ``LIRC_DRIVER_SCANCODE`` for devices that report decoded scancodes
- ``LIRC_DRIVER_IR_RAW`` for devices that report raw pulse/space timings
- ``LIRC_DRIVER_IR_RAW_TX`` for transmit-only raw devices

Raw Pulse/Space Format
======================

Raw IR timing is transferred in the LIRC ``mode2`` format. Each sample is a
32-bit ``unsigned int`` value. The upper 8 bits describe the sample type and
the lower 24 bits carry the payload, usually a duration in microseconds.

The helpers in ``include/nuttx/lirc.h`` are intended to build and inspect
these values:

- ``LIRC_PULSE(usec)``
- ``LIRC_SPACE(usec)``
- ``LIRC_FREQUENCY(value)``
- ``LIRC_TIMEOUT(usec)``
- ``LIRC_IS_PULSE(sample)``
- ``LIRC_IS_SPACE(sample)``
- ``LIRC_VALUE(sample)``

Typical raw transmit data looks like this:

.. code-block:: c

  unsigned int txbuf[] =
  {
    LIRC_PULSE(9000),
    LIRC_SPACE(4500),
    LIRC_PULSE(560),
    LIRC_SPACE(560),
    LIRC_PULSE(560),
  };

For raw transmit, ``write()`` expects an odd number of ``unsigned int``
samples. The upper half forwards the pulse/space sequence to the lower-half
``tx_ir()`` callback.

IOCTL Commands
==============

The RC upper half supports the standard LIRC ``ioctl()`` commands defined in
``include/nuttx/lirc.h``. Commonly used commands include:

- ``LIRC_GET_FEATURES``
- ``LIRC_GET_SEND_MODE``
- ``LIRC_GET_REC_MODE``
- ``LIRC_SET_SEND_MODE``
- ``LIRC_SET_REC_MODE``
- ``LIRC_GET_REC_RESOLUTION``
- ``LIRC_SET_SEND_CARRIER``
- ``LIRC_SET_SEND_DUTY_CYCLE``
- ``LIRC_SET_REC_TIMEOUT``

Support for a specific command depends on the lower-half capabilities. In
practice, applications usually start by querying ``LIRC_GET_FEATURES`` and
then only use the operations that the device advertises.

Lower-Half Registration
=======================

Platform code registers an RC lower half by filling ``struct lirc_lowerhalf_s``
and calling ``lirc_register()``.

.. code-block:: c

  int ret = lirc_register(lower, devno);

The lower half provides a ``struct lirc_ops_s`` callback table. Depending on
the hardware, it may implement open/close callbacks, raw transmit via
``tx_ir()``, scancode transmit via ``tx_scancode()``, carrier and duty-cycle
configuration, or receive timeout handling.

Testing
-------

The ``irtest`` application in ``nuttx-apps/system/irtest`` can be used to
exercise RC devices from NSH. A typical validation sequence is:

#. open ``/dev/lircN``
#. query ``LIRC_GET_FEATURES``
#. query or set send / receive mode
#. write raw mode2 samples on transmit-capable devices
#. read back mode2 samples from receive-capable devices

Espressif RMT LIRC Adapter
==========================

Espressif targets may provide an arch-local LIRC adapter built on top of the
RMT peripheral:

- ``arch/xtensa/src/common/espressif/esp_lirc.c``
- ``arch/risc-v/src/common/espressif/esp_lirc.c``

These adapters expose the RMT peripheral through the RC framework as
``/dev/lircN`` devices while keeping the hardware specific RMT implementation
in the Espressif lower-half driver.
