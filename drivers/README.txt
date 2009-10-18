README
^^^^^^

This directory contains various device drivers -- both block and
character drivers as well as other more specialized drivers.

Files in this directory:
^^^^^^^^^^^^^^^^^^^^^^^

can.c
	An unfinished CAN driver.

dev_null.c and dev_zero.c
	These files provide the standard /dev/null and /dev/zero devices.
 	See include/nuttx/fs.h for functions that should be called if you
	want to register these devices (devnull_register() and
	devzero_register()).

loop.c
	Supports the standard loop device that can be used to export a
	file (or character device) as a block device.  See losetup() and
	loteardown() in	include/nuttx/fs.h.

ramdisk.c
	Can be used to set up a block of memory or (read-only) FLASH as
	a block driver that can be mounted as a files system.  See
	include/nuttx/ramdisk.h.

Subdirectories of this directory:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

bch/
	Contains logic that may be used to convert a block driver into
	a character driver.  This is the complementary conversion as that
	performed by loop.c.  See include/nuttx/fs.h for registration
	information.

mmcsd/
	Support for MMC/SD block drivers.  At present, only SPI-based
	MMC/SD is supported. See include/nuttx/mmcsd.h.

net/
	Network interface drivers.  See also include/nuttx/net.h

pipes/
	FIFO and named pipe drivers.  Standard interfaces are declared
	in include/unistd.h

serial/
	Front-ends character drivers for chip-specific UARTs.  This provide
	some TTY-like functionality and are commonly used (but not required for)
	the NuttX system console.  See include/nuttx/serial.h

usbdev/
	USB device drivers.  See include/nuttx/usb*.h

