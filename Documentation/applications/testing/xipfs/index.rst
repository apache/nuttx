==================================
``xipfs`` XIPFS File System Test
==================================

Exercises the :doc:`XIPFS </components/filesystem/xipfs>` file system: the
routine VFS paths, the write-once rules, both flavours of mapping, and then
the two properties that are easy to get wrong and quiet when they are --
release of execute-in-place pins, and power-loss atomicity of the metadata
commit.

Usage::

    xipfs_test [<section>]

With no argument every section runs. The power-loss sweeps take minutes while
everything else takes seconds, hence the selector::

    nsh> xipfs_test xip
    XIPFS test suite on /mnt/xipfs (/dev/rpflash)
    -- XIP mapping --
      PASS  create module image
      PASS  media is memory mapped
      PASS  strict XIP mmap succeeds
      PASS  mapping points into flash
      PASS  mapped bytes match file contents
      PASS  mapping does not cost RAM proportional to the file
      PASS  mapping takes a pin
      PASS  munmap
      PASS  munmap drops the pin
    ==== 9 passed, 0 failed ====

Sections
========

``basic``
  Create, read back, stat, readdir, unlink, and that all of it survives a
  remount.

``writeonce``
  That reopening a closed file for writing, appending, and seeking during a
  write are refused, and that a sequential write still works afterwards.

``strict``
  That a mapping past the end of a file is refused and a valid one is not.

``xip``
  That a mapping lands inside the media window rather than the heap, that the
  mapped bytes are the file's, that it costs no RAM proportional to the file,
  and that it takes and releases a pin.

``multimap``
  That N mappings of one file produce N pins on one extent and alias the same
  address.

``crosstask``
  That one task dying does not release another task's pin, and that the last
  holder does.

``teardown``
  That a task killed without calling ``munmap`` still has its pin released.
  This is the one that matters for a module that faults.

``dirs``
  That directories behave like a real tree: an empty one exists and survives
  a remount, ``readdir`` reports each level, ``rmdir`` refuses a directory
  that still holds something and succeeds once emptied, and a path through a
  missing directory or through a file fails rather than being created.

``defrag``
  That compaction relocates extents, keeps their contents, survives a
  remount, distinguishes an open file from a pinned one, and never relocates
  a pinned extent.

``powerloss``, ``powertorn``, ``unlink``, ``defraglos``
  Fail the Nth flash operation, remount, and check the volume is consistent
  and every committed file byte-for-byte intact -- sweeping N across create,
  unlink and compaction. The ``torn`` variant leaves the failing write
  partially programmed rather than cleanly refused, which is what a real
  power loss mid-program leaves behind. These need
  ``CONFIG_FS_XIPFS_FAULT_INJECT``.

Configuration
=============

``CONFIG_TESTING_FS_XIPFS``
  Enable the test. Requires ``CONFIG_FS_XIPFS``.

``CONFIG_TESTING_FS_XIPFS_MOUNTPT``
  The volume to test. The suite remounts it, so nothing else should be using
  it.

``CONFIG_TESTING_FS_XIPFS_MTD``
  The MTD device backing that volume, needed for the remounts.

.. note::

   The suite writes and erases the volume continuously, the power-loss
   sections most of all. Point it at a volume whose contents do not matter.
