=================
NuttX File System
=================

**Overview**. NuttX includes an optional, scalable file system.
This file-system may be omitted altogether; NuttX does not depend
on the presence of any file system.

**Pseudo Root File System**. A simple *in-memory*, *pseudo* file
system can be enabled by default. This is an *in-memory* file
system because it does not require any storage medium or block
driver support. Rather, file system contents are generated
on-the-fly as referenced via standard file system operations
(open, close, read, write, etc.). In this sense, the file system
is *pseudo* file system (in the same sense that the Linux
``/proc`` file system is also referred to as a pseudo file
system).

Any user supplied data or logic can be accessed via the
pseudo-file system. Built in support is provided for character and
block `drivers <#DeviceDrivers>`__ in the ``/dev`` pseudo file
system directory.

**Mounted File Systems** The simple in-memory file system can be
extended my mounting block devices that provide access to true
file systems backed up via some mass storage device. NuttX
supports the standard ``mount()`` command that allows a block
driver to be bound to a mountpoint within the pseudo file system
and to a file system. At present, NuttX supports the standard VFAT
and ROMFS file systems, a special, wear-leveling NuttX FLASH File
System (NXFFS), as well as a Network File System client (NFS
version 3, UDP).

**Comparison to Linux** From a programming perspective, the NuttX
file system appears very similar to a Linux file system. However,
there is a fundamental difference: The NuttX root file system is a
pseudo file system and true file systems may be mounted in the
pseudo file system. In the typical Linux installation by
comparison, the Linux root file system is a true file system and
pseudo file systems may be mounted in the true, root file system.
The approach selected by NuttX is intended to support greater
scalability from the very tiny platform to the moderate platform.

Partition Table
===============

Text based Partition Table
----------------------------------

**Summary**

TXTABLE - A text based partition table stored in last eraseblock
(or in romdisk for backup).

1. The 1st line must be "Magic+Version", current is "TXTABLE0".
#. The 2nd and remaining lines are partition entries(min: one)
   in format: "%s %zx %zx"(means name, size and offset (byte)(in hex)).
#. Size or offset can be default zero(means zero(for 1st entry) or
   caculate(for others)), and will be caculated by the parser refs to previous
   and next entries.
#. The last eraseblock will be registered as pseudo partition named "txtable".
   If the last eraseblock included by the last real partition, it will be
   excluded from.

To avoid problems of PTABLE: In case of multiple NuttX binary,
partition table maybe out of sync.

And it's easier:

1. Text format with simple rules(name + size + offset).
#. Size or offset can be default(caculated refs to previous
   and next entries).
#. Support backup table(eg. /etc/txtable.txt in ROMFS)

Size / Offset can be automatically calculated, case:

1. The offset of the first entry is zero, and the offset of other entries
   is zero: automatic calculation;
#. The size of the last entry is zero: fill to the end of the entire Flash
   (keep the last eraseblock); the size of other entries is zero:
   automatically calculated(next.offset - current.offset);
#. Typical case 1: The size of all entries is
   zero (calculated automatically), and the offset is non-zero;
#. Typical case 2: The size and offset of a certain entry are all zero,
   but the size and offset of two adjacent entries are all non-zero;

**Examples**

Both size and offset of "partition6" are zero,
gap exists between "partition7" and "data",
and not reserve last eraseblock.

* txtable.txt

  ::

    TXTABLE0
    partition1 0x6C000 0x4000
    partition2 0x10000 0x70000
    partition3 0x80000 0x80000
    partition4 0x80000 0x100000
    partition5 0x280000 0x180000
    partition6 0 0
    partition7 0x10000 0x480000
    data 0 0x500000



* Parsed

  | Reserved last eraseblock, and gap between partition7 and data is kept.
  | Format: name, offset, size

  ::

    /dev/partition1   offset 0x00004000, size 0x0006c000
    /dev/partition2   offset 0x00070000, size 0x00010000
    /dev/partition3   offset 0x00080000, size 0x00080000
    /dev/partition4   offset 0x00100000, size 0x00080000
    /dev/partition5   offset 0x00180000, size 0x00280000
    /dev/partition6   offset 0x00400000, size 0x00080000
    /dev/partition7   offset 0x00480000, size 0x00010000
    /dev/data         offset 0x00500000, size 0x00aff000
    /dev/txtable      offset 0x00fff000, size 0x00001000

More than one not set size or offset

* txtable.txt

  ::

    TXTABLE0
    partition1 0 0x4000
    partition2 0 0x70000
    partition3 0 0x80000
    partition4 0x80000 0x100000
    partition5 0x280000 0
    partition6 0 0
    partition7 0x10000 0x480000
    data 0 0x500000

* Parsed

  | Size of partition[2,3,4,6] and data are caculated, and gap between
    partition7 and data is kept.

  ::

    /dev/partition1   offset 0x00004000, size 0x0006c000
    /dev/partition2   offset 0x00070000, size 0x00010000
    /dev/partition3   offset 0x00080000, size 0x00080000
    /dev/partition4   offset 0x00100000, size 0x00080000
    /dev/partition5   offset 0x00180000, size 0x00280000
    /dev/partition6   offset 0x00400000, size 0x00080000
    /dev/partition7   offset 0x00480000, size 0x00010000
    /dev/data         offset 0x00500000, size 0x00aff000
    /dev/txtable      offset 0x00fff000, size 0x00001000

Only one partition entry, and size not spec

* txtable.txt

  ::

    TXTABLE0
    partition1 0x0 0x4000

* Parsed

  | The last eraseblock was kept, and size is correct.

  ::

    /dev/partition1   offset 0x00004000, size 0x00ffb000
    /dev/txtable      offset 0x00fff000, size 0x00001000

Blank line && New line delim

* txtable.txt

  | New line: CR + LF / LF.
  | Additional char/string after "%s %zx %zx".

  ::

    TXTABLE0
    partition1 0x6C000 0x4000
    partition2 0 0x70000
    partition3 0 0x80000
    partition4 0 0x100000
    partition5 0x280000 0x180000
    partition6 0x80000 0x400000   # String between "%s %zx %zx" and "LF" will be ignored.
    partition7 0x10000 0x480000   # Comments: This is the 7th partition.
    data 0 0x500000
    
    
    
    EOF

* Parsed

  | Blank lines are ignored, and new line of both "LF" or "CRLF" are parsed.
    String between "%s %zx %zx" and "LF" will be ignored(eg. CR, or some comments).

  ::

    /dev/partition1   offset 0x00004000, size 0x0006c000
    /dev/partition2   offset 0x00070000, size 0x00010000
    /dev/partition3   offset 0x00080000, size 0x00080000
    /dev/partition4   offset 0x00100000, size 0x00080000
    /dev/partition5   offset 0x00180000, size 0x00280000
    /dev/partition6   offset 0x00400000, size 0x00080000
    /dev/partition7   offset 0x00480000, size 0x00010000
    /dev/data         offset 0x00500000, size 0x00aff000
    /dev/txtable      offset 0x00fff000, size 0x00001000

ZipFS
=====

Zipfs is a read only file system that mounts a zip file as a NuttX file system through the NuttX VFS interface.
This allows users to read files while decompressing them, without requiring additional storage space.

CONFIG
------

.. code-block:: c

    CONFIG_FS_ZIPFS=y
    CONFIG_LIB_ZLIB=y

Example
-------

1. `./tools/configure.sh sim:zipfs` build sim platform with zipfs support.

2. `make` build NuttX.

3. `./nuttx` run NuttX.

4. `nsh> mount -t hostfs -o /home/<your host name>/work /host` mount host directory to /host.

5. `nsh> mount -t zipfs -o /host/test.zip /zip` mount zip file to /zipfs.

6. Use cat/ls command to test.

.. code-block:: c

    nsh> ls /zip
    /zip:
     a/1
     a/2
    nsh> cat /zip/a/1
    this is zipfs test 1
    nsh> cat /zip/a/2
    this is zipfs test 2

