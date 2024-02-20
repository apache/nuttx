===
FAT
===

File Allocation Table (FAT) is a very simple file system designed by Microsoft
and, as the name suggests, it uses a table to track clusters on a storage
volume. There have been multiple versions of FAT, like ``FAT12``, ``FAT16``,
and lastly ``FAT32`` to consider for growing data volumes. Even though FAT
has been superseded by more recent file systems in most Windows computers,
current Windows editions, macOS, Linux, Unix as well as Apache NuttX support
FAT. It is still used for floppy disks, USB flash drives as well as for
storage media in smaller, portable devices like digital cameras.

FAT uses a magic number ``0x4D44``, and has been named as ``vfat`` in the
codebase, due to the implementation being actually VFAT, which is an extension
that allows for long file names. This page contains information about the FAT
file system from the perspective of the implementation of FAT that exists in
NuttX.

FAT Structure
=============

The number beside ``FAT`` (eg. ``16`` in ``FAT16``) represents the number of
bits used for each entry in the allocation table.  A FAT volume has multiple
sections:

* **Master Boot Record**: Master Boot Record (MBR) or Boot Sector contains 
  information needed by the file system to access the volume, including the
  volume's layout and file system structure, akin to a superblock in Linux
  file systems.
* **Allocation Table Region**: Stores the file allocation table, as well as
  its copy which acts as a backup. The file allocation table keeps a track of
  all the clusters in the volume, and thus has one entry for each cluster.
* **Root Directory**: It stores a directory table containing entries 
  describing the files and directories stored on the volume. Each entry
  contains metadata about file system objects.

.. NOTE::
  Root Directory exists as a separate section if the volume is for ``FAT12``
  or ``FAT16``, right after the Allocation Table Region and has a fixed upper
  limit to the number of entries. ``FAT32`` does not have a dedicated region
  for the Root Directory, but has it incorporated into the following Data
  Region, and thus ``FAT32`` does not have any hard coded upper limit to the
  number of entries.

* **Data Region**: This region stores the actual data of the files as well as
  directory data. Data Region is divided into multiple clusters, which are
  numbered sequentially and have corresponding entries in the allocation
  table. Cluster sizes can vary depending on the volume size as well as on
  the type of the FAT file system.

The first entry in the Allocation Table is for the volume's FAT ID while the
second entry indicates that the cluster is reserved. Given that ``FAT32`` does
not have a dedicated Root Directory section, in ``FAT32`` volume, the third
entry points to the root directory.

File names can be either `short <https://en.wikipedia.org/wiki/8.3_filename>`_
(also known as 8.3 filename or SFN), or long (LFN), till a maximum length
limit.

Master Boot Record (MBR)
========================

The Boot Record may be one of two types. One is a much older type which does
not contain partitions, and one more recent with them. One distinguishing
feature of FAT is that all MBRs contain ``0x55AA`` at an offset of 510 from
the start, which can be used to determine the type of FAT being used in the
block device.

For the older type, the MBR exists at the start of the drive. It does not
contain any partitions.

The newer type has a partition table at an offset of 446 from the start of
the drive. It allows for 4 primary partitions. It has FAT Boot Records (FBRs)
at the start of every partition, which, for most parts, are identical in
structure to the older MBRs.

Partition Table Entries
=======================

The partition table contains information for 4 primary partitions, each
partition having entries in the table of 16 bytes each. These have various
information about the partition including, but not limited to, type of the
partition, starting sector, and partition size.

Allocation Table
================

The allocation table follows the boot record, and has an entry for each
cluster available in the data region. Each entry has a size defined by the
file system type. ``FAT12`` has 12 byte long entries, ``FAT16`` has 16 byte
entries and ``FAT32`` has 32 byte entries.

File data is made up of a linked list (or chain) of clusters. Entries in the
allocation table, which correspond to actual clusters, contain the cluster
number of the next cluster that appears in the file. The last cluster of a
file has a value in their allocation table entry such that all of its bits
are set (``0xFFF`` for FAT12, ``0xFFFF`` for FAT16, ``0xFFFFFFFF`` for FAT32).

The starting cluster (head of the file chain) is pointed to by the directory
entry of that file.

Directory Entires
=================

A directory is basically a file which has a table that contains directory
entries that contain the metadata about the files and subdirectories in it.
Each directory entry describes a file or a subdirectory inside the directory.

Root directory in FAT12/16 have a dedicated region, separate from the data
area. In FAT32, root directory is just like a regular directory in the data
region (ie. without any dedicated region) except for:

* Its starting cluster is denoted in the boot record.
* It is not pointed to by any other directory.
* It has no entries corresponding to ``.`` and ``..`` (all non-root
  directories have both of these entries).

Directory entries are 32 bytes long, and start with an 11 bytes long SFN. The
rest of the directory entry contain information like the file attributes,
timestamps of creation as well as access and write, cluster number and
file size, to name a few.

8.3 filename
------------

The first byte of a directory's 8.3 filename (and hence first byte of the
entire directory entry) has a special meaning. It has 3 possible values:

* ``0xe5`` : Denotes that the current directory entry is empty.
* ``0x00`` : Denotes that this entry, as well as all following entries, are
  empty.
* ``0x05`` : Actual value is ``0xe5``.

The rest of the 7 + 3 byte of the directory entry are for the name (with
extension).

Files
=====

The starting cluster of data in a file is pointed by the directory entry of
the file.

File allocation is very primitive in FAT, and the first available location in
the volume is given to a file.

File attributes
--------------------

File attributes are denoted by a bit flag of the size of a single byte.
The file flags in FAT, with their bit representation, are as follows:

.. list-table:: File Attributes
  :header-rows: 1

  * - Attribute Macro
    - Bit representation
    - Hex value
  * - ``FATATTR_READONLY``
    - ``00000001``
    - 0x1
  * - ``FATATTR_HIDDEN``
    - ``00000010``
    - 0x2
  * - ``FATATTR_SYSTEM``
    - ``00000100``
    - 0x4
  * - ``FATATTR_VOLUMEID``
    - ``00001000``
    - 0x8
  * - ``FATATTR_DIRECTORY``
    - ``00010000``
    - 0x10
  * - ``FATATTR_ARCHIVE``
    - ``00100000``
    - 0x20

Out of these, FAT exposes a user to ``FATATTR_READONLY``, ``FATATTR_HIDDEN``,
``FATATTR_SYSTEM`` and ``FATATTR_ARCHIVE`` to the user.

Implementation
==============

The Apache NuttX implementation of VFAT can be found in:

* ``fs/fat`` directory.
* ``include/nuttx/fs/fat.h`` header file.