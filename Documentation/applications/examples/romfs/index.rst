``romfs`` File System
=====================

This example exercises the romfs filesystem. Configuration options include:

- ``CONFIG_EXAMPLES_ROMFS_RAMDEVNO`` – The minor device number to use for the ROM
  disk. The default is ``1`` (meaning ``/dev/ram1``).
- ``CONFIG_EXAMPLES_ROMFS_SECTORSIZE`` – The ROM disk sector size to use. Default
  is ``64``.
- ``CONFIG_EXAMPLES_ROMFS_MOUNTPOINT`` – The location to mount the ROM disk.
  Default: ``/usr/local/share``.
