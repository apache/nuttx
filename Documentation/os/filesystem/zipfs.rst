=====
ZipFS
=====

Zipfs is a read only file system that mounts a zip file as a NuttX file system through the NuttX VFS interface.
This allows users to read files while decompressing them, without requiring additional storage space.

CONFIG
======

.. code-block:: bash

    CONFIG_FS_ZIPFS=y
    CONFIG_LIB_ZLIB=y

Example
=======

1. ``./tools/configure.sh sim:zipfs`` build sim platform with zipfs support.

2. ``make`` build NuttX.

3. ``./nuttx`` run NuttX.

4. ``nsh> mount -t hostfs -o /home/<your host name>/work /host`` mount host directory to ``/host``.

5. ``nsh> mount -t zipfs -o /host/test.zip /zip`` mount zip file to ``/zipfs``.

6. Use cat/ls command to test.

.. code-block:: bash

    nsh> ls /zip
    /zip:
     a/1
     a/2
    nsh> cat /zip/a/1
    this is zipfs test 1
    nsh> cat /zip/a/2
    this is zipfs test 2

Write Mode (wzipfs)
===================

The ``wzipfs`` mount type provides a write-only streaming ZIP filesystem
backed by a regular file, FIFO, or socket.  It is compiled independently
of the read-only ``zipfs`` driver via ``CONFIG_FS_ZIPFS_WRITE``.

Mount
-----

Syntax::

    mount -t wzipfs -o src=<backing>[,method=store|deflate] <mountpoint>

Behavior:

- If ``<backing>`` does not exist or has zero size, wzipfs streams a new
  ZIP archive into it.
- If ``<backing>`` is a regular file with ``st_size > 0``, mount fails with
  ``-EEXIST``.  Remove or truncate the backing file before mounting to
  start a fresh archive.
- FIFO and socket targets are always accepted and stream from offset 0.

Compression
-----------

Selected at mount time via ``method=``:

- ``method=store`` -- no compression (default; always available).
- ``method=deflate`` -- DEFLATE compression; requires
  ``CONFIG_FS_ZIPFS_WRITE_DEFLATE=y``.  Without that config,
  ``method=deflate`` returns ``-ENOTSUP``.

The method applies to every entry written during the mount.

Supported Operations
--------------------

- ``open`` -- create a new entry.  Only one entry may be open at a time;
  a second ``open`` blocks until the first ``close`` returns.
- ``write`` -- stream data into the current entry.  Entry header is
  emitted lazily on the first write.
- ``close`` -- finalise the current entry.  Empty files become a
  zero-byte STORE entry.
- ``mkdir`` -- create a directory entry (trailing ``/``, ``AE_IFDIR``).
- ``umount`` -- flush the central directory and close the backing.

Unsupported Operations
----------------------

- ``read``, ``readdir``, ``opendir``, ``stat`` -- wzipfs is write-only.
  To list or read entries, unmount and mount the same backing as
  read-only ``zipfs``.
- ``unlink``, ``rmdir``, ``rename``, ``truncate`` -- ZIP entries are
  immutable once committed.  Modifying a committed entry requires
  rewriting the whole archive (every subsequent LFH+data shifted, plus
  CD rebuilt); on embedded flash this cost is unacceptable and real-world
  use cases are rare.  Applications that need this should decompress,
  modify, and rebuild from scratch.
- Creating encrypted entries -- libarchive supports WinZip AES-128/256
  but requires a secure passphrase delivery mechanism (NuttX kernel
  lacks a keyring subsystem; passing passphrases via mount options is
  an anti-pattern).  Applications needing encryption should encrypt
  payload bytes before writing.

RAM Budget
----------

- ~64 KB libarchive writer workspace
- ~5 KB wzipfs mount structure
- +~260 KB zlib workspace if ``method=deflate`` (only when selected)

Devices with less than 128 KB heap cannot use wzipfs with DEFLATE.

Known Limitations
-----------------

- **No crash safety**: power loss during a write session leaves an
  incomplete ZIP.  The central directory is written only on clean
  ``umount``.
- **Single writer**: one file descriptor at a time.  A second ``open``
  blocks until the first closes.

Usage Examples
--------------

Write to a regular file::

    : > /data/out.zip
    mount -t wzipfs -o src=/data/out.zip /mnt/zip
    echo hello > /mnt/zip/hello.txt
    umount /mnt/zip

Stream to a FIFO (pipe to another process)::

    mkfifo /dev/zipstream
    mount -t wzipfs -o src=/dev/zipstream /mnt/zip
    echo data > /mnt/zip/data.txt
    umount /mnt/zip

DEFLATE compression::

    mount -t wzipfs -o src=/data/out.zip,method=deflate /mnt/zip
