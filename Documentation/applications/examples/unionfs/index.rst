=============================
``unionfs`` Union File System
=============================

This is at trivial test of the Union File System. See
``nuttx/fs/unionfs/README.txt``. Dependencies:

- ``CONFIG_DISABLE_MOUNTPOINT``          – Mountpoint support must not be
  disabled.
- ``CONFIG_FS_ROMFS``                    – ROMFS support is required.
- ``CONFIG_FS_UNIONFS``                  – Union File System support is required.

Configuration options. Use the defaults if you are unsure of what you are doing:

- ``CONFIG_EXAMPLES_UNIONFS``            – Enables the example.
- ``CONFIG_EXAMPLES_UNIONFS_MOUNTPT``    – Mountpoint path for the Union File
  System.
- ``CONFIG_EXAMPLES_UNIONFS_TMPA``       – Temporary mount point for file system
  ``1``.
- ``CONFIG_EXAMPLES_UNIONFS_TMPB``       – Temporary mount point for file system
  ``2``.
- ``CONFIG_EXAMPLES_UNIONFS_RAMDEVNO_A`` – ROMFS file system ``1`` RAM disk device
  number.
- ``CONFIG_EXAMPLES_UNIONFS_RAMDEVNO_B`` – ROMFS file system ``2`` RAM disk device
  number.
- ``CONFIG_EXAMPLES_UNIONFS_SECTORSIZE`` – ROM disk sector size.

See :doc:`/platforms/sim/sim/boards/sim/index` page for a walk-through of the
output of this text.
