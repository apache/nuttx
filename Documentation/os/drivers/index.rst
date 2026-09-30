==============
Device Drivers
==============

NuttX supports a variety of device drivers, which can be broadly
divided in three classes:

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
:doc:`/os/drivers/drivers_design` and :doc:`/os/drivers/device_drivers`.

Subdirectories of ``nuttx/drivers``
===================================

* ``1wire/`` :doc:`/os/drivers/character/1wire`

  1wire device drivers.

* ``aie/`` :doc:`/os/drivers/character/aie`

  Upper-half character driver for hardware AI / NPU engines.
  See ``include/nuttx/aie/ai_engine.h``.

* ``analog/`` :doc:`/os/drivers/character/analog/index`

  This directory holds implementations of analog device drivers.
  This includes drivers for Analog to Digital Conversion (ADC) as
  well as drivers for Digital to Analog Conversion (DAC).

* ``audio/`` :doc:`/os/drivers/special/audio`

  Audio device drivers.

* ``bch/`` :doc:`/os/drivers/character/bch`

  Contains logic that may be used to convert a block driver into
  a character driver.  This is the complementary conversion as that
  performed by loop.c.

* ``can/`` :doc:`/os/drivers/character/can`

  This is the CAN drivers and logic support.

* ``clk/``:doc:`/os/drivers/special/clk`

  Clock management (CLK) device drivers.

* ``contactless/`` :doc:`/os/drivers/character/contactless`

  Contactless devices are related to wireless devices.  They are not
  communication devices with other similar peers, but couplers/interfaces
  to contactless cards and tags.

* ``crypto/`` :doc:`/os/drivers/character/crypto/index`

  Contains crypto drivers and support logic, including the
  ``/dev/urandom`` device.

* ``devicetree/`` :doc:`/os/drivers/special/devicetree`

  Device Tree support.

* ``dma/`` :doc:`/os/drivers/special/dma`

  DMA drivers support.

* ``eeprom/`` :doc:`/os/drivers/character/eeprom`

  EEPROM support as character drivers. Support as Memory Technology Device
  (MTD) is located in the ``mtd/`` directory.

* ``efuse/`` :doc:`/os/drivers/character/efuse`

  EFUSE drivers support.

* ``i2c/`` :doc:`/os/drivers/special/i2c`

  I2C drivers and support logic.

* ``i2s/`` :doc:`/os/drivers/character/i2s`

  I2S drivers and support logic.

* ``i3c/`` :doc:`/os/drivers/special/i3c`

  I3C drivers and support logic.

* ``input/`` :doc:`/os/drivers/character/input/index`

  This directory holds implementations of human input device (HID) drivers.
  This includes such things as mouse, touchscreen, joystick,
  keyboard and keypad drivers.

  Note that USB HID devices are treated differently.  These can be found under
  ``usbdev/`` or ``usbhost/``.

* ``ioexpander/`` :doc:`/os/drivers/special/ioexpander`

  IO Expander drivers.

* ``ipcc/`` :doc:`/os/drivers/character/ipcc`

  IPCC (Inter Processor Communication Controller) driver.

* ``lcd/`` :doc:`/os/drivers/special/lcd`

  Drivers for parallel and serial LCD and OLED type devices.

* ``leds/`` :doc:`character/leds/index`

  Various LED-related drivers including discrete as well as PWM- driven LEDs.

* ``loop/`` :doc:`/os/drivers/character/loop`

  Supports the standard loop device that can be used to export a
  file (or character device) as a block device.

  See ``losetup()`` and ``loteardown()`` in ``include/nuttx/fs/fs.h``.

* ``math/`` :doc:`/os/drivers/character/math`

  MATH Acceleration drivers.

* ``misc/`` :doc:`/os/drivers/character/nullzero` :doc:`/os/drivers/special/rwbuffer` :doc:`/os/drivers/block/ramdisk`

  Various drivers that don't fit elsewhere.

* ``mmcsd/`` :doc:`/os/drivers/special/sdio` :doc:`/os/drivers/special/mmcsd`

  Support for MMC/SD block drivers.  MMC/SD block drivers based on
  SPI and SDIO/MCI interfaces are supported.

* ``modem/`` :doc:`/os/drivers/character/modem`

  Modem Support.

* ``motor/`` :doc:`/os/drivers/character/motor/index`

  Motor control drivers.

* ``mtd/`` :doc:`/os/drivers/special/mtd/index`

  Memory Technology Device (MTD) drivers.  Some simple drivers for
  memory technologies like FLASH, EEPROM, NVRAM, etc.

  (Note: This is a simple memory interface and should not be
  confused with the "real" MTD developed at infradead.org.  This
  logic is unrelated; I just used the name MTD because I am not
  aware of any other common way to refer to this class of devices).

* ``net/`` :doc:`/os/drivers/special/net/index`

  Network interface drivers.

* ``notes/`` :doc:`/os/drivers/character/note`

  Note Driver Support.

* ``pinctrl/`` :doc:`/os/drivers/special/pinctrl`

  Configure and manage pin.

* ``pipes/`` :doc:`/os/drivers/special/pipes`

  FIFO and named pipe drivers.
  Standard interfaces are declared in ``include/unistd.h``

* ``power/`` :doc:`/os/drivers/special/power/index`

  Various drivers related to power management.

* ``rc/`` :doc:`/os/drivers/character/rc`

  Remote Control Device Support.

* ``regmap/`` :doc:`/os/drivers/special/regmap`

  Regmap Subsystems Support.

* ``reset/`` :doc:`/os/drivers/special/reset`

  Reset Driver Support.

* ``rf/`` :doc:`/os/drivers/character/rf`

  RF Device Support.

* ``rptun/`` :doc:`/os/drivers/special/rptun/index`

  Remote Proc Tunnel Driver Support.

* ``segger/`` :doc:`/os/drivers/special/segger`

  Segger RTT drivers.

* ``sensors/`` :doc:`/os/drivers/special/sensors`

  Drivers for various sensors.  A sensor driver differs little from
  other types of drivers other than they are use to provide measurements
  of things in environment like temperature, orientation, acceleration,
  altitude, direction, position, etc.

  DACs might fit this definition of a sensor driver as well since they
  measure and convert voltage levels.  DACs, however, are retained in
  the ``analog/`` sub-directory.

* ``serial/``:doc:`/os/drivers/character/serial`

  Front-end character drivers for chip-specific UARTs.
  This provide some TTY-like functionality and are commonly used (but
  not required for) the NuttX system console.

* ``spi/`` :doc:`/os/drivers/special/spi`

  SPI drivers and support logic.

* ``syslog/`` :doc:`/os/drivers/special/syslog`

  System logging devices.

* ``timers/`` :doc:`/os/drivers/character/timers/index`

  Includes support for various timer devices.

* ``usbdev/`` :doc:`/os/drivers/special/usbdev`

  USB device drivers.

* ``usbhost/`` :doc:`/os/drivers/special/usbhost`

  USB host drivers.

* ``usbmisc/`` :doc:`/os/drivers/special/usbmisc`

  USB Miscellaneous drivers.

* ``usbmonitor/`` :doc:`/os/drivers/special/usbmonitor`

  USB Monitor support.

* ``usrsock/`` :doc:`/os/drivers/special/usrsock`

  Usrsock Driver Support.

* ``video/`` :doc:`/os/drivers/special/video`

  Video-related drivers.

* ``virtio/`` :doc:`/os/drivers/special/virtio/index`

  Virtio Device Support.

* ``wireless/`` :doc:`/os/drivers/special/wireless`

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

.. toctree::
  :maxdepth: 1

  character/index.rst
  block/index.rst
  special/index.rst
  thermal/index.rst

.. toctree::
   :maxdepth: 1

   drivers_design.rst
   device_drivers.rst
   device_nodes.rst
   ioctl.rst
   usb.rst
