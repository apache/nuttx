===============================
Two Wire Interface in AVR DA/DB
===============================

Two Wire Interface is AVR peripheral capable of supporting both I\ :sup:`2`\ C and SMBus.

Usage
=====


Pointer to initialized ``struct i2c_master_s`` can be obtained using

.. code-block:: c

   FAR struct i2c_master_s *i2c;
   i2c = avrdx_initialize_twi(0);


This function will initialize the peripheral based on configuration specified
via Kconfig. It may be called multiple times, only first call will perform
the initialization - subsequent calls will only return the pointer.

Deinitialization is currently not supported.

The parameter denotes which peripheral is to be initialized. With current chips,
permitted values are ``0`` and ``1``.
It is ignored if the chip has only single TWI interface.

Configuration options
=====================

Pin selection
-------------

This option makes it possible to choose which I/O pins will be connected
to the peripheral.

Mode selection
--------------

The peripheral supports Standard, Fast and Fast Plus modes.

Quick commands
--------------

SMBus permits transactions that contain no data. Instead, the R/nW bit
is interpreted as a single-bit datum by the target device. If this is enabled,
I\ :sup:`2`\ C messages with zero length are interpreted as quick commands. If not,
zero-length messages are not permitted and the transmission is rejected
with ``EINVAL``

Forbid NOSTART
--------------

NuttX upper half of the I\ :sup:`2`\ C driver permits ``I2C_M_NOSTART``
flag for a message, indicating that the message is not a standalone entity
but rather a continuation of previous message. Since not all drivers need
this and program memory space is not unlimited on the chip, this configuration
option can be used to remove support for such messages.

Transmission that submits message with this flag will be rejected
with ``EINVAL`` if this configuration option is set. Enable this only
if you know internals of I\ :sup:`2`\ C drivers used by your application.

Limitations
===========

Mode and addressing
-------------------

Currently, only master mode is supported.

Only 7-bit addressing is supported. The peripheral does not support 10-bit
addressing directly and the driver software does currently have no support
for that either. I\ :sup:`2`\ C messages requesting 10-bit address
are rejected with ``ENOTSUP``.

Message limitations
-------------------

Maximum message count for single transmission is 127.

The driver does not support ``I2C_M_NOSTOP`` flag for last submitted message.
As per rules specified in ``include/nuttx/i2c/i2c_master.h``, the I\ :sup:`2`\ C
driver lower half is supposed to leave the bus occupied if the last
of the submitted messages has this ``NOSTOP`` flag set. This driver
is not capable of doing that.

Also not supported is ``I2C_M_NOSTART`` flag for first submitted message.
First message within the transfer always starts with ``START`` condition.

Messages with zero length (which are only permitted based on configuration
- see above) may not specify ``I2C_M_NOSTART`` nor ``I2C_M_NOSTOP``.
(Such a message is interpreted as a quick command and quick commands
must end with a ``STOP`` condition according to the chip's documentation.)

Inactivity timeout
------------------

The driver configures timeout for bus inactivity during initialization.
This timeout is used by the hardware to switch its internal state
from "bus is busy" or "bus is in unknown state" to "bus is idle."
This transition otherwise only happens after a ``STOP`` condition is detected
or when forced by software.

This timeout remains in effect during normal TWI operation as well.
Such a timeout is defined for SMBus but not for I\ :sup:`2`\ C; nevertheless,
the driver does not distinguish between the two and the timeout is always
in place.
