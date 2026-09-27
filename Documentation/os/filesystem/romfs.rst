=====
ROMFS
=====

A read-only file system, laid out so that files *can* be used **in place**
-- mapped and read straight from where they are stored, without being
copied into RAM.  That is the usual reason to choose it for the files a
system ships with rather than the files it writes.

Enabled with ``CONFIG_FS_ROMFS``, which needs mount point support
(``!CONFIG_DISABLE_MOUNTPOINT``).  The code is in ``fs/romfs/``.

Whether you actually get in-place access is decided at mount time, and not
by ROMFS.  It asks the block driver underneath for a ``BIOC_XIPBASE``
address:

* if the driver answers, ROMFS reads the media directly and
  ``mmap()`` on a file returns a pointer into it;
* if it does not, ROMFS allocates a sector buffer and reads through it like
  any other file system, and ``mmap()`` fails with ``-ENOTTY``.

Memory-mapped flash and the RAM/ROM disk driver answer; a file system on an
SD card does not.

Why read-only is the point
==========================

Nothing in a ROMFS image can be changed after it is built, and that buys
several things at once:

* **Little RAM for the files themselves.**  In the in-place case above,
  reading a ROMFS file is reading flash, so a 200 KB file costs no RAM for
  its contents.  Two caches still do cost some:
  ``CONFIG_FS_ROMFS_CACHE_NODE``, on unless the build is a small one, reads
  every directory node into RAM at mount so that lookups do not have to
  walk the media; and ``CONFIG_FS_ROMFS_CACHE_FILE_NSECTORS`` gives each
  open file a sector cache, one sector by default.
* **Execute in place.**  Where the media can be mapped, a program in ROMFS
  can be run without being loaded first, which is what makes it a sensible
  home for built-in applications.
* **Nothing to corrupt.**  A read-only file system cannot be left
  inconsistent by a power cut, so it needs no journal, no checking at mount
  time and no wear levelling.

Where it is used
================

Two arrangements are common, and both appear throughout the board ports:

* Mounted at ``/etc`` to hold a startup script, so the shell has something
  to run at boot without a writable file system existing at all.  See
  :doc:`/guides/filesystem/etcromfs`.
* Holding the binaries that :doc:`/os/binfmt/index` loads, so programs live
  in flash rather than being linked into the kernel image.

If the files have to change at runtime, ROMFS is the wrong file system;
:doc:`/os/filesystem/index` lists the writable ones.
