==============
Device Drivers
==============

NuttX supports a variety of device drivers, which can be broadly
divided in three classes:

.. toctree::
  :maxdepth: 1

  character/index.rst
  block/index.rst
  special/index.rst
  thermal/index.rst

.. note::
  Device driver support depends on the *in-memory*, *pseudo*
  file system that is enabled by default.

Lower-half and upper-half
=========================

Drivers in NuttX generally work in two distinct layers:

  * An *upper half* which registers itself to NuttX using
    a call such as :c:func:`register_driver` or
    :c:func:`register_blockdriver` and implements the corresponding
    high-level interface (`read`, `write`, `close`, etc.).
    implements the interface. This *upper half* calls into
    the *lower half* via callbacks.
  * A "lower half" which is typically hardware-specific. This is
    usually implemented at the architecture or board level.

Details about drivers implementation can be found in
:doc:`../../implementation/drivers_design` and :doc:`../../implementation/device_drivers`.

Subdirectories of ``nuttx/drivers``
===================================

* ``1wire/`` :doc:`character/1wire`

  1wire device drivers.

* ``analog/`` :doc:`character/analog/index`

  This directory holds implementations of analog device drivers.
  This includes drivers for Analog to Digital Conversion (ADC) as
  well as drivers for Digital to Analog Conversion (DAC).

* ``audio/`` :doc:`special/audio`

  Audio device drivers.

* ``bch/`` :doc:`character/bch`

  Contains logic that may be used to convert a block driver into
  a character driver.  This is the complementary conversion as that
  performed by loop.c.

* ``can/`` :doc:`character/can`

  This is the CAN drivers and logic support.

* ``clk/``:doc:`special/clk`

  Clock management (CLK) device drivers.

* ``contactless/`` :doc:`character/contactless`

  Contactless devices are related to wireless devices.  They are not
  communication devices with other similar peers, but couplers/interfaces
  to contactless cards and tags.

* ``crypto/`` :doc:`character/crypto/index`

  Contains crypto drivers and support logic, including the
  ``/dev/urandom`` device.

* ``devicetree/`` :doc:`special/devicetree`

  Device Tree support.

* ``dma/`` :doc:`special/dma`

  DMA drivers support.

* ``eeprom/`` :doc:`block/eeprom`

  An EEPROM is a form of Memory Technology Device (see ``drivers/mtd``).
  EEPROMs are non-volatile memory like FLASH, but differ in underlying
  memory technology and differ in usage in many respects: They may not
  be organized into blocks (at least from the standpoint of the user)
  and it is not necessary to erase the EEPROM memory before re-writing
  it.  In addition, EEPROMs tend to be much smaller than FLASH parts,
  usually only a few kilobytes vs megabytes for FLASH.  EEPROM tends to
  be used to retain a small amount of device configuration information;
  FLASH tends to be used for program or massive data storage. For these
  reasons, it may not be convenient to use the more complex MTD
  interface but instead use the simple character interface provided by
  the EEPROM drivers.

* ``efuse/`` :doc:`character/efuse`

  EFUSE drivers support.

* ``i2c/`` :doc:`special/i2c`

  I2C drivers and support logic.

* ``i2s/`` :doc:`character/i2s`

  I2S drivers and support logic.


* ``i3c/`` :doc:`special/i3c`

  I3C drivers and support logic.

* ``input/`` :doc:`character/input/index`

  This directory holds implementations of human input device (HID) drivers.
  This includes such things as mouse, touchscreen, joystick,
  keyboard and keypad drivers.

  Note that USB HID devices are treated differently.  These can be found under
  ``usbdev/`` or ``usbhost/``.

* ``ioexpander/`` :doc:`special/ioexpander`

  IO Expander drivers.

* ``ipcc/`` :doc:`character/ipcc`

  IPCC (Inter Processor Communication Controller) driver.

* ``lcd/`` :doc:`special/lcd`

  Drivers for parallel and serial LCD and OLED type devices.

* ``leds/`` :doc:`character/leds/index`

  Various LED-related drivers including discrete as well as PWM- driven LEDs.

* ``loop/`` :doc:`character/loop`

  Supports the standard loop device that can be used to export a
  file (or character device) as a block device.

  See ``losetup()`` and ``loteardown()`` in ``include/nuttx/fs/fs.h``.

* ``math/`` :doc:`character/math`

  MATH Acceleration drivers.

* ``misc/`` :doc:`character/nullzero` :doc:`special/rwbuffer` :doc:`block/ramdisk`

  Various drivers that don't fit elsewhere.

* ``mmcsd/`` :doc:`special/sdio` :doc:`special/mmcsd`

  Support for MMC/SD block drivers.  MMC/SD block drivers based on
  SPI and SDIO/MCI interfaces are supported.

* ``modem/`` :doc:`character/modem`

  Modem Support.

* ``motor/`` :doc:`character/motor/index`

  Motor control drivers.

* ``mtd/`` :doc:`special/mtd`

  Memory Technology Device (MTD) drivers.  Some simple drivers for
  memory technologies like FLASH, EEPROM, NVRAM, etc.

  (Note: This is a simple memory interface and should not be
  confused with the "real" MTD developed at infradead.org.  This
  logic is unrelated; I just used the name MTD because I am not
  aware of any other common way to refer to this class of devices).

* ``net/`` :doc:`special/net/index`

  Network interface drivers.

* ``notes/`` :doc:`character/note`

  Note Driver Support.

* ``pinctrl/`` :doc:`special/pinctrl`

  Configure and manage pin.

* ``pipes/`` :doc:`special/pipes`

  FIFO and named pipe drivers.
  Standard interfaces are declared in ``include/unistd.h``

* ``power/`` :doc:`special/power/index`

  Various drivers related to power management.

* ``rc/`` :doc:`character/rc`

  Remote Control Device Support.

* ``regmap/`` :doc:`special/regmap`

  Regmap Subsystems Support.

* ``reset/`` :doc:`special/reset`

  Reset Driver Support.

* ``rf/`` :doc:`character/rf`

  RF Device Support.

* ``rptun/`` :doc:`special/rptun`

  Remote Proc Tunnel Driver Support.

* ``segger/`` :doc:`special/segger`

  Segger RTT drivers.

* ``sensors/`` :doc:`special/sensors`

  Drivers for various sensors.  A sensor driver differs little from
  other types of drivers other than they are use to provide measurements
  of things in environment like temperature, orientation, acceleration,
  altitude, direction, position, etc.

  DACs might fit this definition of a sensor driver as well since they
  measure and convert voltage levels.  DACs, however, are retained in
  the ``analog/`` sub-directory.

* ``serial/``:doc:`character/serial`

  Front-end character drivers for chip-specific UARTs.
  This provide some TTY-like functionality and are commonly used (but
  not required for) the NuttX system console.

* ``spi/`` :doc:`special/spi`

  SPI drivers and support logic.

* ``syslog/`` :doc:`special/syslog`

  System logging devices.

* ``timers/`` :doc:`character/timers/index`

  Includes support for various timer devices.

* ``usbdev/`` :doc:`special/usbdev`

  USB device drivers.

* ``usbhost/`` :doc:`special/usbhost`

  USB host drivers.

* ``usbmisc/`` :doc:`special/usbmisc`

  USB Miscellaneous drivers.

* ``usbmonitor/`` :doc:`special/usbmonitor`

  USB Monitor support.

* ``usrsock/`` :doc:`special/usrsock`

  Usrsock Driver Support.

* ``video/`` :doc:`special/video`

  Video-related drivers.

* ``virtio/`` :doc:`special/virtio`

  Virtio Device Support.

* ``wireless/`` :doc:`special/wireless`

  Drivers for various wireless devices.

Skeleton Files
==============

Skeleton files are "empty" frameworks for NuttX drivers.  They are provided to
give you a good starting point if you want to create a new NuttX driver.
The following skeleton files are available:

* ``drivers/lcd/skeleton.c`` Skeleton LCD driver
* ``drivers/mtd/skeleton.c`` Skeleton memory technology device drivers
* ``drivers/net/skeleton.c`` Skeleton network/Ethernet drivers
* ``drivers/usbhost/usbhost_skeleton.c`` Skeleton USB host class driver

Drivers Early Initialization
============================

To initialize drivers early in the boot process, the :c:func:`drivers_early_initialize`
function is introduced. This is particularly beneficial for certain drivers,
such as SEGGER SystemView, or others that require initialization before the
system is fully operational.

It is important to note that during this early initialization phase,
system resources are not yet available for use. This includes memory allocation,
file systems, and any other system resources.
