============
Auto-Mounter
============

General Description
===================
NuttX implements an auto-mounter than can make working with SD cards or other
removable media easier. With the auto-mounter, the file system will be
automatically mounted when media is inserted and automatically unmounted when
the media is removed.

The auto is enable by selecting in the NuttX configuration::

  CONFIG_FS_AUTOMOUNTER=y


WARNING: SD cards should never be removed without first unmounting them. This
is to avoid data and possible corruption of the file system. Certainly this is
the case if you are writing to the SD card at the time of the removal. If you
use the SD card for read-only access, however, then I cannot think of any
reason why removing the card without mounting would be harmful.

For applications that write to the removable media, the automatic unmount is
still beneficial (as opposed to leaving a broken mount in place) although
should not be relied upon for a proper solution.

Board-Specific Support
======================

Like many components of NuttX, the auto-mounter has a upper-half/lower-half
architecture:

* **Upper half** The upper half is the file ``fs/fs_automount.c``. This upper
  half performs the basic automount activities. It responds to media
  insertion and removal events by mounting and unmounting the file system on
  the media. This includes logic to handle unmount retries: The unmount cannot
  be performed while applications have open files on the media. In this case,
  the auto-mounter will periodically retry the unmount until all of the
  applications close there references to files on the non-existent media.

* **Lower Half** The lower half is defined by a standard interface. That
  interface definition is in the header file ``include/nuttx/fs/automount.h``.
  The lower half interface provides: (1) mount information including file
  system type, block driver path, and mount point path, (2) mount and unmount
  retry delays, and (3) and callbacks to attach to and management the media
  insertion / removal interrupts.

Example Implementation
======================

There is an example implementation of this lower half interface at
``boards/arm/sama5/sama5d4-ek/src/sam_automount.c``. The ``boards/arm/sama5/sama5d4-ek/Kconfig``
as the board-specific configuration for the auto-mounter. You can see
the configuration settings in the ``boards/arm/sama5/sama5d4-ek/configs/nsh/defconfig``
and ``boards/arm/sama5/sama5d4-ek/configs/nxwm/defconfig`` configuration files::

  CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT=y
  CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_FSTYPE="vfat"
  CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_BLKDEV="/dev/mmcsd0"
  CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_MOUNTPOINT="/mnt/sdcard"
  CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_DDELAY=1000
  CONFIG_SAMA5D4EK_HSMCI0_AUTOMOUNT_UDELAY=2000

These setting determine the values in the lower half interface. The interrupt is
provided by a PIO pin defined in ``boards/arm/sama5/sama5d4-ek/src/sama5e4-ek.h`` and
the implementation of the interface and callbacks is in
``boards/arm/sama5/sama5d4-ek/src/sam_automount.c``.


