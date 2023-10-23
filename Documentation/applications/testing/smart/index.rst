===========================
``smart`` SMART File System
===========================

This is a test of the SMART file system that derives from ``testing/nxffs``.

- ``CONFIG_TESTING_SMART`` – Enable the SMART file system example.

- ``CONFIG_TESTING_SMART_ARCHINIT`` – The default is to use the RAM MTD device at
  ``drivers/mtd/rammtd.c``. But an architecture-specific MTD driver can be used
  instead by defining ``CONFIG_TESTING_SMART_ARCHINIT``. In this case, the
  initialization logic will call ``smart_archinitialize()`` to obtain the MTD
  driver instance.

- ``CONFIG_TESTING_SMART_NEBLOCKS`` – When ``CONFIG_TESTING_SMART_ARCHINIT`` is not
  defined, this test will use the RAM MTD device at ``drivers/mtd/rammtd.c`` to
  simulate FLASH. In this case, this value must be provided to give the number
  of erase blocks in MTD RAM device. The size of the allocated RAM drive will
  be: ``CONFIG_RAMMTD_ERASESIZE * CONFIG_TESTING_SMART_NEBLOCKS``.

- ``CONFIG_TESTING_SMART_MAXNAME`` – Determines the maximum size of names used in
  the filesystem.

- ``CONFIG_TESTING_SMART_MAXFILE`` – Determines the maximum size of a file.
- ``CONFIG_TESTING_SMART_MAXIO`` – Max I/O, default ``347``.
- ``CONFIG_TESTING_SMART_MAXOPEN`` – Max open files.
- ``CONFIG_TESTING_SMART_MOUNTPT`` – SMART mountpoint.
- ``CONFIG_TESTING_SMART_NLOOPS`` – Number of test loops. default ``100``.
- ``CONFIG_TESTING_SMART_VERBOSE`` – Verbose output.
