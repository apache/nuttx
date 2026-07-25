=========================================
``xipfs`` XIPFS Compaction and Block Map
=========================================

Compacts a :doc:`XIPFS </components/filesystem/xipfs>` volume and reports how
its erase blocks are laid out.

Because a XIPFS file is a single contiguous extent, an allocation can fail
for lack of a contiguous run while plenty of free space remains. Compaction
coalesces the holes that deletes leave behind. It is never automatic: an
allocation simply fails with ``ENOSPC`` and the caller decides whether
compacting is worth its cost in erases. This command is that decision made
by hand.

Usage::

    xipfs [-n] [-t <ms>] [<mountpoint>]

``-n``
  Survey and report only; move nothing.

``-t <ms>``
  Give the pass a time budget. It stops at the first extent boundary after
  the budget is spent, which is always a consistent layout.

``<mountpoint>``
  The volume to act on. Defaults to ``CONFIG_SYSTEM_XIPFS_MOUNTPOINT``.

The compaction itself is asked for through a descriptor for the mountpoint
directory, so no file inside the volume is open while it runs and one pass
can reach every extent. Without ``-n`` the command surveys, compacts, and
surveys again, so the map it prints last is the one on the media::

    nsh> xipfs -n
    /mnt/xipfs: 250 blocks of 4096 bytes (1000 KB)

      file                             start  blocks    bytes  pin
      xipmod                               0       1      268    0

         0 |#.................................................|
        50 |..................................................|

      '.' free   '#' in use   'P' pinned by a live mapping

      used 1, free 249, largest free run 249 blocks (996 KB)
      1 free run, fragmentation 0%

The fragmentation figure is the share of free space a single allocation
cannot reach, ``(free - largest_run) / free``. At 0% the largest possible
file already fits however scattered the map looks, which is the question a
caller facing ``ENOSPC`` actually has.

Files are listed by their path relative to the mount: the command walks the
volume's directories, which hold no blocks of their own and so do not appear
in the map.

A ``P`` marks an extent held by a live execute-in-place mapping. Compaction
skips those, because relocating them would move code that is executing, so
they bound what any pass can reclaim.

Configuration
=============

``CONFIG_SYSTEM_XIPFS``
  Enable the command. Requires ``CONFIG_FS_XIPFS``.

``CONFIG_SYSTEM_XIPFS_MOUNTPOINT``
  The volume used when none is named on the command line.
