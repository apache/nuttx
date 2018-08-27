README
^^^^^^

This directory contains various device drivers -- both block and
character drivers as well as other more specialized drivers.

Contents:
  - Files in this directory
  - Subdirectories of this directory
  - Skeleton files

Files in this directory
^^^^^^^^^^^^^^^^^^^^^^^

dev_null.c, dev_urandom, and dev_zero.c
  These files provide the standard /dev/null, /dev/urandom, and /dev/zero
  devices.  See include/nuttx/drivers/driers.h for prototypes of functions
  that should be called if you want to register these devices
  (devnull_register(), devurandom_register(), and devzero_register()).

ramdisk.c
  Can be used to set up a block of memory or (read-only) FLASH as
  a block driver that can be mounted as a files system.  See
  include/nuttx/drivers/ramdisk.h.

rwbuffer.c
  A facility that can be use by any block driver in-order to add
  writing buffering and read-ahead buffering.

Subdirectories of this directory:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

analog/
  This directory holds implementations of analog device drivers.
  This includes drivers for Analog to Digital Conversion (ADC) as
  well as drivers for Digital to Analog Conversion (DAC).
  See include/nuttx/analog/*.h for registration information.

audio/
  Audio device drivers.  See include/nuttx/audio/audio.h for interface
  definitions.  See also the audio subsystem at nuttx/audio/.

bch/
  Contains logic that may be used to convert a block driver into
  a character driver.  This is the complementary conversion as that
  performed by loop.c.  See include/nuttx/fs/fs.h for registration
  information.

can/
  This is the CAN drivers and logic support.  See include/nuttx/can/can.h
  for usage information.

contactless/
  Contactless devices are related to wireless devices.  They are not
  communication devices with other similar peers, but couplers/interfaces
  to contactless cards and tags.

crypto/
  Contains crypto drivers and support logic.

eeprom/
  An EEPROM is a form of Memory Technology Device (see drivers/mtd).
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

i2c/
  I2C drivers and support logic.  See include/nuttx/i2c/i2c_master.h

i2s/
  I2S drivers and support logic.  See include/nuttx/audio/i2s.h

input/
  This directory holds implementations of human input device (HID)
  drivers.  This includes such things as mouse, touchscreen, joystick,
  keyboard and keypad drivers.  See include/nuttx/input/*.h for
  registration information.

  Note that USB HID devices are treated differently.  These can be
  found under usbdev/ or usbhost/.

lcd/
  Drivers for parallel and serial LCD and OLED type devices.  These
  drivers support interfaces as defined in include/nuttx/lcd/lcd.h

leds/
  Various LED-related drivers including discrete as well as PWM-
  driven LEDs.

loop/
  Supports the standard loop device that can be used to export a
  file (or character device) as a block device.  See losetup() and
  loteardown() in include/nuttx/fs/fs.h.

mmcsd/
  Support for MMC/SD block drivers.  MMC/SD block drivers based on
  SPI and SDIO/MCI interfaces are supported.  See include/nuttx/mmcsd.h
  and include/nuttx/sdio.h for further information.

mtd/
  Memory Technology Device (MTD) drivers.  Some simple drivers for
  memory technologies like FLASH, EEPROM, NVRAM, etc.  See
  include/nuttx/mtd/mtd.h

  (Note: This is a simple memory interface and should not be
  confused with the "real" MTD developed at infradead.org.  This
  logic is unrelated; I just used the name MTD because I am not
  aware of any other common way to refer to this class of devices).

net/
  Network interface drivers.  See also include/nuttx/net/net.h

pipes/
  FIFO and named pipe drivers.  Standard interfaces are declared
  in include/unistd.h

power/
  Power management (PM) driver interfaces.  These interfaces are used
  to manage power usage of a platform by monitoring driver activity
  and by placing drivers into reduce power usage modes when the
  drivers are not active.

pwm/
  Provides the "upper half" of a pulse width modulation (PWM) driver.
  The "lower half" of the PWM driver is provided by device-specific
  logic.  See include/nuttx/drivers/pwm.h for usage information.

sensors/
  Drivers for various sensors.  A sensor driver differs little from
  other types of drivers other than they are use to provide measuresments
  of things in envionment like temperatore, orientation, acceleration,
  altitude, direction, position, etc.

  DACs might fit this definition of a sensor driver as well since they
  measure and convert voltage levels.  DACs, however, are retained in
  the analog/ sub-directory.

serial/
  Front-end character drivers for chip-specific UARTs.  This provide
  some TTY-like functionality and are commonly used (but not required for)
  the NuttX system console.  See also include/nuttx/serial/serial.h

spi/
  SPI drivers and support logic.  See include/nuttx/spi/spi.h

syslog/
  System logging devices. See include/syslog.h and include/nuttx/syslog/syslog.h

timers/
  Includes support for various timer devices including:

  - An "upper half" for a generic timer driver.  See
    include/nuttx/timers/timer.h for more information.

  - An "upper half" for a generic watchdog driver.  See
    include/nuttx/timers/watchdog.h for more information.

  - RTC drivers

usbdev/
  USB device drivers.  See also include/nuttx/usb/usbdev.h

usbhost/
  USB host drivers.  See also include/nuttx/usb/usbhost.h

video/
  Video-related drivers.  See inlude/nuttx/video/.

wireless/
  Drivers for various wireless devices.

Skeleton Files
^^^^^^^^^^^^^^

Skeleton files a "empty" frameworks for NuttX drivers.  They are provided to
give you a good starting point if you want to create a new NuttX driver.
The following skeleton files are available:

  drivers/lcd/skeleton.c -- Skeleton LCD driver
  drivers/mtd/skeleton.c -- Skeleton memory technology device drivers
  drivers/net/skeleton.c -- Skeleton network/Ethernet drivers
  drivers/usbhost/usbhost_skeleton.c -- Skeleton USB host class driver
