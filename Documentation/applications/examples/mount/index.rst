==========================
``mount`` Mount Filesystem
==========================

This contains a simple test of filesystem mountpoints.

- ``CONFIG_EXAMPLES_MOUNT_DEVNAME`` – The name of the user-provided block device
  to mount. If ``CONFIG_EXAMPLES_MOUNT_DEVNAME`` is not provided, then a RAM disk
  will be configured.

- ``CONFIG_EXAMPLES_MOUNT_NSECTORS`` – The number of sectors in the RAM disk
  used when ``CONFIG_EXAMPLES_MOUNT_DEVNAME`` is not defined.

- ``CONFIG_EXAMPLES_MOUNT_SECTORSIZE`` – The size of each sectors in the RAM disk
  used when ``CONFIG_EXAMPLES_MOUNT_DEVNAME`` is not defined.

- ``CONFIG_EXAMPLES_MOUNT_RAMDEVNO`` – The RAM device minor number used to mount
  the RAM disk used when ``CONFIG_EXAMPLES_MOUNT_DEVNAME`` is not defined. The
  default is zero (meaning that ``/dev/ram0`` will be used).
