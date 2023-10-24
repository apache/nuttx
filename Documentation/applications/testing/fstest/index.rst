===================================
``fstest`` Generic file system test
===================================

This is a generic file system test that derives from ``testing/nxffs``. It was
created to test the tmpfs file system, but should work with any file system
provided that all initialization has already been performed prior to starting
the test.

This test a a general test for any file system, but includes some specific hooks
for the SPIFFS file system.

- ``CONFIG_TESTING_FSTEST`` – Enable the file system example.
- ``CONFIG_TESTING_FSTEST_MAXNAME`` – Determines the maximum size of names used in
  the filesystem.
- ``CONFIG_TESTING_FSTEST_MAXFILE`` – Determines the maximum size of a file.
- ``CONFIG_TESTING_FSTEST_MAXIO`` – Max I/O, default ``347``.
- ``CONFIG_TESTING_FSTEST_MAXOPEN`` – Max open files.
- ``CONFIG_TESTING_FSTEST_MOUNTPT`` – Path where the file system is mounted.
- ``CONFIG_TESTING_FSTEST_NLOOPS`` – Number of test loops. default ``100``.
- ``CONFIG_TESTING_FSTEST_VERBOSE`` – Verbose output.

EXAMPLE::

  fstest -m /mnt -n 10 – Test /mnt 10 times
  fstest -h            – Get help message
  fstest               – Test path define by `CONFIG_TESTING_FSTEST_MOUNTPT`
                         `CONFIG_TESTING_FSTEST_NLOOPS` times

