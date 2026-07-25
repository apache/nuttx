======================================
XIPFS Contiguous Execute-In-Place FS
======================================

XIPFS stores each file as a single physically **contiguous**, erase-block
aligned extent on memory-mapped flash, and exposes a true execute-in-place
``mmap`` that returns a direct pointer into that flash rather than a RAM
copy.

It exists for one purpose: letting a NOMMU target run downloadable modules
out of NOR flash so that their read-only text and rodata never consume RAM.
Only a running module's writable segment is copied.

Configuration
=============

``CONFIG_FS_XIPFS``
  Enable the filesystem.  Requires ``CONFIG_MTD``.

``CONFIG_FS_XIPFS_FAULT_INJECT``
  Test-only.  Adds a countdown to the flash write and erase paths so a test
  can fail an arbitrary operation and then remount, modelling a power loss
  at that exact point.  Do not enable in production.

XIPFS mounts on an **MTD** device::

  mount -t xipfs /dev/rammtd /mnt/xipfs

Pass ``autoformat`` as the mount data to format an unrecognised volume
rather than failing with ``EFTYPE``.

For XIP mappings to be possible at all, the underlying MTD driver must
implement the ``BIOC_XIPBASE`` ioctl, returning the directly addressable
base of the media.  ``rammtd`` does, which makes it a usable stand-in for
memory-mapped NOR during development.  A driver that does not is still
perfectly usable as a filesystem; it simply cannot serve XIP mappings.

Usage model
===========

XIPFS is **not** a general purpose read-write filesystem, and does not
pretend to be.  Files are **write once**:

* A file is created, its size is declared, it is written sequentially, and
  it is closed.
* Thereafter it is immutable until it is deleted.  Reopening for writing,
  appending, and seeking during a write are all refused.

Declaring the size up front with ``ftruncate()`` is the preferred path,
because it lets the filesystem reserve exactly the right contiguous extent::

  fd = open("/mnt/xipfs/module.bin", O_WRONLY | O_CREAT | O_TRUNC, 0644);
  ftruncate(fd, module_size);
  write(fd, module_data, module_size);
  close(fd);

If the size is not declared, the largest available free run is reserved and
trimmed back at close.  That keeps ordinary sequential writes -- including
shell redirection, and a file arriving over a serial transfer -- working, at
the cost of temporarily reserving more space than needed.

Such a reservation is erased **lazily**, one block ahead of the writer,
rather than in full when it is taken.  Erasing it up front would make the
cost of writing a small file proportional to the free space instead of to
the file: on an empty 1 MB volume that was roughly 250 sector erases, about
13 seconds on an RP2350, with interrupts disabled for each one.  Long enough
that the far end of a serial transfer gives up.  The blocks the trim at
close hands back were never erased at all.

An exact reservation from ``ftruncate()`` is already the right size, so
there is nothing to defer and it is still erased when taken.

Directories
-----------

Directories are real: they are records in the metadata generation, so an
empty one exists, survives a remount, and is removed by ``rmdir``. What they
are **not** is objects in the data region. A directory costs no data block
and no erase; it costs one entry out of the volume's fixed supply, and
``statfs`` reports that supply as ``f_files``/``f_ffree``.

That placement is deliberate, and it is what keeps the power-safety story in
one piece: ``mkdir`` and ``rmdir`` add or remove a record and commit a single
generation, exactly as ``open(O_CREAT)`` and ``unlink`` do. There is no
multi-object update to journal and no orphan to collect at mount.

Each entry carries its own identity and the identity of the directory that
holds it; the root is implicit and owns identity zero. A name is therefore
one path component, and ``XIPFS_NAME_MAX`` bounds a component rather than a
whole path -- which is what ``statfs`` reports as ``f_namelen``.

Mount rebuilds the tree from those records and checks that it *is* a tree:
identities must be unique, names must be unique within a directory, every
parent must name a live directory, and following parents must reach the
root. A cycle on the medium would otherwise hang a path walk rather than
merely giving a wrong answer.

``.`` and ``..`` are refused as components with ``EINVAL`` rather than
interpreted. An entry stored under either name could never be reached again,
and the VFS has already resolved the ones that were meant navigationally.

Unlike a filesystem that synthesises directories from names, a path through
something that is not a directory fails rather than being created: writing
to ``bin/hello`` when ``bin`` does not exist gives ``ENOENT``, and when
``bin`` is a file, ``ENOTDIR``. Create the directories first.

Execute-in-place mapping
========================

A normal ``mmap()`` returns a direct flash pointer when it can and falls
back to the VFS RAM copy when it cannot.  That fallback is convenient for
data readers but fatal for a module loader, which would silently lose the
entire benefit of executing in place.

Loaders must therefore pass ``MAP_XIP_STRICT``::

  addr = mmap(NULL, len, PROT_READ | PROT_EXEC,
              MAP_SHARED | MAP_XIP_STRICT, fd, 0);

With that flag, a mapping that cannot be resolved in place fails with
``ENXIO`` instead of being copied.  The usual response is to defragment and
retry, or to refuse the load.

Pinning
-------

A mapping that aliases flash takes a **pin** on the extent, and the pin
lives on the extent rather than on the file descriptor.  Three running
instances of one module therefore produce a pin count of three, and the
extent becomes movable again only when the last of them goes away.

An extent with a non-zero pin count is never relocated or erased by the
defragmenter, which is what allows a module to keep executing from flash
while the filesystem is being compacted around it.

Pins are released on ``munmap`` **or** on task teardown.  A module that
faults or is killed without unmapping still drops its pin, because
``mm_map_destroy()`` walks the dying task's mapping list and invokes each
mapping's unmap callback.  Nothing relies on the application behaving well.

Defragmentation
===============

Because files are created at a known size and never grow, a file can never
fragment internally.  The only thing that fragments is free space, through
the holes deletes leave behind.

Compaction is therefore **manual, best effort and pin aware**.  It is never
invoked from inside a failing allocation: allocation simply returns
``ENOSPC`` and the caller decides whether compacting is worth it.

::

  struct xipfs_defrag_arg_s arg = { .max_ms = 50 };
  int fd = open("/mnt/xipfs", O_RDONLY | O_DIRECTORY);
  ioctl(fd, XIPFSIOC_DEFRAG, &arg);
  /* arg.result.largest_free_run says whether a retry can now succeed */

The pass is a loop of atomic relocations.  Each one copies an extent into
free space, commits new metadata naming the new location, and only then
erases the vacated blocks, so every iteration boundary is a fully
consistent layout.  Any reason for stopping -- a time budget, a pinned
extent in the way, a media error, a power cut -- lands on one of those
boundaries.  Nothing is ever left half moved.

The result reports what was achieved and why it stopped:

``XIPFS_DEFRAG_DONE``
  Nothing left to compact.
``XIPFS_DEFRAG_BLOCKED_PINS``
  A live XIP mapping is in the way.  Resolve by unloading a module;
  ``XIPFSIOC_LISTPINNED`` reports which.
``XIPFS_DEFRAG_BLOCKED_OPEN``
  A merely open file is in the way.  Resolve by closing a descriptor.
``XIPFS_DEFRAG_BLOCKED_RAM``
  Transient resource shortage.
``XIPFS_DEFRAG_TIME_BUDGET``
  The caller's ``max_ms`` was reached.
``XIPFS_DEFRAG_ERROR``
  A media error stopped the pass cleanly.

.. note::

   Issue the volume commands -- ``XIPFSIOC_DEFRAG``, ``XIPFSIOC_LISTPINNED``
   and ``XIPFSIOC_FAULTINJECT`` -- on a descriptor for the **mountpoint
   directory**, obtained with ``open(mountpoint, O_RDONLY | O_DIRECTORY)``.
   They reach the file system through its ``ioctldir`` method.

   A descriptor for a file inside the volume is also accepted, but it holds
   that file open for the duration, and an open extent cannot be relocated.
   A pass asked for that way is therefore obstructed by the act of asking,
   and normally reports ``XIPFS_DEFRAG_BLOCKED_OPEN`` with the caller's own
   file as the obstruction.

   ``XIPFSIOC_EXTENTINFO``, ``XIPFSIOC_PIN`` and ``XIPFSIOC_UNPIN`` name a
   file rather than the volume, so those do take a descriptor for the file
   itself.

The ``xipfs`` command
---------------------

``apps/system/xipfs`` wraps the ioctl and reports the layout::

  xipfs [-n] [-t <ms>] [<mountpoint>]

``-n`` surveys and prints without moving anything: the per-file table, a
block map, and how much of the free space a single allocation can reach.
``-t`` passes a time budget to the pass.  Without ``-n`` it surveys,
compacts, and surveys again, so the final map is the one on the media::

  /mnt/xipfs: 250 blocks of 4096 bytes (1000 KB)

     0 |#####.............................................|

  '.' free   '#' in use   'P' pinned by a live mapping

  used 5, free 245, largest free run 245 blocks (980 KB)
  1 free run, fragmentation 0%

The fragmentation figure is the share of free space that a single
allocation cannot reach, ``(free - largest_run) / free``.  At 0% the
largest possible file already fits however scattered the map looks, which
is the question a caller facing ``ENOSPC`` actually has.

Power-loss behaviour
====================

Metadata is a ping-pong ring of whole generations.  Each commit erases the
next ring slot, writes the directory body, and finally writes the header
carrying the sequence number and a CRC over header plus body.

That final header write is the commit point, and it is a single read/write
block program -- the smallest unit the media can tear at.  A power loss
before it leaves a slot whose header is still erased, so the generation
does not exist.  A power loss during it leaves a header failing its own
CRC.  Either way mount falls back to the previous generation, which is
untouched because it lives in a different slot.

Every state change follows the same shape: write the new data, flip the
metadata reference, then erase the old.  The reference flip is always the
commit point, and the old copy is never erased before the new reference is
durable.

On-media layout
===============

::

  block 0      superblock copy A
  block 1      superblock copy B
  block 2..5   metadata ring (4 generations, ping-pong).  A generation is
               a header plus one 64-byte record per file and per directory
  block 6..    data region, contiguous erase-block aligned extents.  Only
               files live here; a directory is a record and nothing more

Limitations
===========

* A directory costs one of the volume's fixed number of entries, since it is
  a metadata record. On a 4 KiB sector with 256-byte pages that supply is 60
  entries, files and directories together.
* No random writes, appends, growth, or rename.
* A file occupies a whole number of erase blocks, so a small file consumes
  a full block.  This is accepted deliberately: it means no block ever
  holds a mix of live and dead data, which is what makes delete, relocate
  and erase clean operations and shrinks the defrag staging buffer to a
  single program page.
* Erase slicing, erase-suspend and running the erase routine from RAM are
  not implemented.  On a single NOR die without read-while-write these are
  needed to bound interrupt latency; they belong in
  ``fs/xipfs/xipfs_flash.c``, through which every media access already
  funnels.
