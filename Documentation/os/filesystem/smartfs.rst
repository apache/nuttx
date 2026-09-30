=======
SMARTFS
=======

This page contains information about the implementation of the NuttX
Sector Mapped Allocation for Really Tiny (SMART) FLASH file system, SMARTFS.

Features
========

This implementation is a full-feature file system from the perspective of
file and directory access (i.e. not considering low-level details like the
lack of bad block management).  The SMART File System was designed specifically
for small SPI based FLASH parts (1-8 Mbyte for example), though this is not
a limitation.  It can certainly be used for any size FLASH and can work with
any MTD device by binding it with the SMART MTD layer and has been tested with
devices as large as 128MByte (using a 2048 byte sector size with 65534 sectors).
The FS includes support for:

- Multiple open files from different threads.
- Open for read/write access with seek capability.
- Appending to end of files in either write, append or read/write open modes.
- Directory support.
- Support for multiple mount points on a single volume / partition (see details
  below).
- Selectable FLASH Wear leveling algorithm
- Selectable CRC-8 or CRC-16 error detection for sector data
- Reduced RAM model for FLASH geometries with large number of sectors (16K-64K)

General operation
=================

The SMART File System divides the FLASH device or partition into equal
sized sectors which are allocated and "released" as needed to perform file
read/write and directory management operations.  Sectors are then "chained"
together to build files and directories.  The operations are split into two
layers:

1.  The MTD block layer (nuttx/drivers/mtd/smart.c).  This layer manages
    all low-level FLASH access operations including sector allocations,
    logical to physical sector mapping, erase operations, etc.
2.  The FS layer (nuttx/fs/smart/smartfs_smart.c).  This layer manages
    high-level file and directory creation, read/write, deletion, sector
    chaining, etc.

SMART MTD Block layer
=====================

The SMART MTD block layer divides the erase blocks of the FLASH device into
"sectors".  Sectors have both physical and logical number assignments.
The physicl sector number represents the actual offset from the beginning
of the device, while the logical sector number is assigned as needed.
A physical sector can have any logical sector assignment, and as files
are created, modified and destroyed, the logical sector number assignment
for a given physical sector will change over time.  The logical sector
number is saved in the physical sector header as the first 2 bytes, and
the MTD layer maintains an in-memory map of the logical to physical mapping.
Only physical sectors that are in use will have a logical assignment.

Also contained in the sector header is a flags byte and a sequence number.
When a sector is allocated, the COMMITTED flag will be "set" (changed from
erase state to non-erase state) to indicate the sector data is valid.  When
a sector's data needs to be deleted, the RELEASED flag will be "set" to
indicate the sector is no longer in use.  This is done because the erase
block containing the sector cannot necessarily be erased until all sectors
in that block have been "released".  This allows sectors in the erase
block to remain active while others are inactive until a "garbage collection"
operation is needed on the volume to reclaim released sectors.

The sequence number is used when a logical sector's data needs to be
updated with new information.  When this happens, a new physical sector
will be allocated which has a duplicate logical sector number but a
higher sequence number.  This allows maintaining flash consistency in the
event of a power failure by writing new data prior to releasing the old.
In the event of a power failure causing duplicate logical sector numbers,
the sector with the higher sequence number will win, and the older logical
sector will be released.

The SMART MTD block layer reserves some logical sector numbers for internal
use, including::

    Sector 0:     The Format Sector.  Has a format signature, format version, etc.
                  Also contains wear leveling information if enabled.
    Sector 1-2:   Additional wear-leveling info storage if needed.
    Sector 3:     The 1st (or only) Root Directory entry
    Sector 4-10:  Additional root directories when Multi-Mount points are supported.
    Sector 11-12: Reserved

To perform allocations, the SMART MTD block layer searches each erase block
on the device to identify the one with the most free sectors.  Free sectors
are those that have all bytes in the "erased state", meaning they have not
been previously allocated/released since the last block erase.  Not all
sectors on the device can be allocated ... the SMART MTD block driver must
reserve at least one erase-block worth of unused sectors to perform
garbage collection, which will be performed automatically when no free
sectors are available.  When wear leveling is enabled, the allocator also takes
into account the erase block erasure status to maintain level wearing.

Garbage collection is performed by identifying the erase block with the most
"released" sectors (those that were previously allocated but no longer being
used) and moving all still-active sectors to a different erase block.  Then
the now "vacant" erase block is erased, thus changing a group of released
sectors into free sectors.  This may occur several times depending on the
number of released sectors on the volume such that better "wear leveling"
is achieved.

Standard MTD block layer functions are provided for block read, block write,
etc. so that system utilities such as the "dd" command can be used,
however, all SMART operations are performed using SMART specific ioctl
codes to perform sector allocate, sector release, sector write, etc.

A couple of config items that the SMART MTD layer can take advantage of
in the underlying MTD drivers is SUBSECTOR_ERASE and BYTE_WRITE.  Most
flash devices have a 32K to 128K Erase block size, but some of them
have a smaller erase size available also.  Vendors have different names
for the smaller erase size; In the NuttX MTD layer it is called
SUBSECTOR_ERASE.  For FLASH devices that support the smaller erase size,
this configuration item can be added to the underlying MTD driver, and
SMART will use it.  As of the writing of this page, only the
drivers/mtd/m25px.c driver had support for SUBSECTOR_ERASE.

The BYTE_WRITE config option enables use of the underlying MTD driver's
ability to write data a byte or a few bytes at a time vs. a full page
at at time (which is typically 256 bytes).  For FLASH devices that support
byte write mode, support for this config item can be added to the MTD
driver.  Enabling and supporting this feature reduces the traffic on the
SPI bus considerably because SMARTFS performs many operations that affect
only a few bytes on the device.  Without BYTE_WRITE, the code must
perform a full page read-modify-write operation on a 256 or even 512
byte page.

Wear Leveling
=============

When wear leveling is enabled, the code automatically writes data across
the entire FLASH device in a manner that causes each erase block to be
worn (i.e. erased) evenly.  This is accomplished by maintaining a 4-bit
wear level count for each erase block and forcing less worn blocks to be
used for writing new data.  The code maintains each block's erase count
to be within 16 erases of each other, though through testing, the span
so far was never greater than 10 erases of each other.

As the data in a block is modified repeatedly, the erase count will
increase.  When the wear level reaches a value of 8 or higher, and the block
needs to be erased (because the data in it has been modified, etc.) the code
will select an erase block with the lowest wear count and relocate it to
this block (with the higher wear count).  The idea being that a block with
the lowest wear count contains more "static" data and should require fewer
additional erase operations.  This relocation process will continue on the
block (only when it needs to be erased again).

When the wear level of all erase blocks has increased to a level of
SMART_WEAR_MIN_LEVEL (currently set to 5), then the wear level counts
will all be reduced by this value.  This keeps the wear counts normalized
so they fit in a 4-bit value.  Note that theoretically, it *IS* possible to
write data to the flash in a manner that causes the wear count of a single
erase block to increment beyond it's maximum value of 15.  This would have
to be a very, very, very specific and un-predictable write sequence though
as data is always spread out across the sectors and relocated dynamically.
In the extremely rare event this does occur, the code will automatically
cap the maximum wear level at 15 an increment an "uneven wear count"
variable to indicate the number times this event has occurred.  So far, I
have not been able to get the wear count above 10 though my testing.

The wear level status bits are saved in the format sector (logical sector
number zero) with overflow saved in the reserved logical sectors one and
two.  Additionally, the uneven wear count (and total block erases if
PROCFS is enabled) are stored in the format sector.  When the PROCFS file
system is enabled and a SMARTFS volume is mounted, the SMART block driver
details and / or wear level details can be viewed with a command such as::

     cat /proc/fs/smartfs/smart0/status
        Format version:    1
        Name Len:          16
        Total Sectors:     2048
        Sector Size:       512
        Format Sector:     1487
        Dir Sector:        8
        Free Sectors:      67
        Released Sectors:  572
        Unused Sectors:    817
        Block Erases:      5680
        Sectors Per Block: 8
        Sector Utilization:98%
        Uneven Wear Count: 0

     cat /proc/fs/smartfs/smart0/erasemap
        DDDCGCCDDCDCCDCBDCCDDGBBDBCDCCDDDCDDDDCCDDCCCGCGDCCDBCDDGBDBDCDD
        BCCCDDCCDDDCBCCDGCCCBDDCCGBBCBCCGDCCDCBDBCCCDCDDCDDGCDCGDCBCDBDG
        BCDDCDCBGCCCDDCGBCCGBCCBDDBDDCGDCDDDCGCDDBCDCBDDBCDCGDDCCBCGBCCC
        GCBCCGCCCDDDBGCCCCGDCCCCCDCDDGBBDACABDBBABCAABCCCDAACBADADDDAECB

Enabling wear leveling can increase the total number of block erases on the
device in favor of even wearing (erasing).  This is caused by writing /
moving sectors that otherwise don't need to be written to move static data
to the more highly worn blocks.  This additional write requirement is known
as write amplification.  To get an idea of the amount of write amplification
incurred by enabling wear leveling, I conducted the smart_test example using
four different configurations (wear, no wear, CRC-8, no CRC) and the results
are shown below.  This was done on a 1M Byte simulated FLASH with 4K erase
block size, 512 sectors per byte.  The smart_test creates a 700K file and
then performs 20,000 random seek, write, verify tests.  The seek write forces
a multitude of sector relocation operations (with or without CRC enabled),
causing a boatload of block erases.

Enabling wear leveling actually decreased the number of erase operations
with CRC enabled or disabled.  This is only a single test point based one
testing method ... results will likely vary based on the method the data
is written, the amount of static vs. dynamic data, the amount of free space
on the volume, and the volume geometry (erase block size, sector size, etc.).

The results of the tests are::

    Case                          Total Block erases
    ================================================
    No wear leveling     CRC-8         6632
    Wear leveling        CRC-8         5585

    No wear leveling     no CRC        6658
    Wear leveling        no CRC        5398

Reduced RAM model
=================

On devices with a larger number of logical sectors (i.e. a lot of erase
blocks with a small selected sector size), the RAM requirement can become
fairly significant.  This is caused by the in-memory sector map which
keeps track of the logical to physical mapping of all sectors.  This is
a RAM array which is 2 * totalsectors in size.  For a device with 64K
sectors, this means 128K of RAM is required just for the sector map, not
counting RAM for read/write buffers, erase block management, etc.

So a reduced RAM model has been added which only keeps track of which
logical sectors have been used (a table which is totalsectors / 8 in size)
and a configurable sized sector map cache.  Each entry in the sector map
cache is 6 bytes (logical sector, physical sector and cache entry age).
ON DEVICES WITH SMALLER TOTAL SECTOR COUNT, ENABLING THIS OPTION COULD
ACTUALLY INCREASE THE RAM FOOTPRINT INSTEAD OF REDUCE IT.

The sector map cache size should be selected to balance the desired RAM
usage and the file system performance.  When a logical to physical sector
mapping is not found in the cache, the code must perform a physical search
of the FLASH to find the requested logical sector.  This involves reading
the 5-byte header from each sector on the device until the sector is
found.  Performing a full read, seek or open for append on a large file
can cause the sector map cache to flush completely if the file is larger
than (cache entries * sector size).  For example, in a configuration with
256 cache entries and a 512 byte sector size, a full read, seek or open for
append on a 128K file will flush the cache.

An additional RAM savings is realized on FLASH parts that contain 16 or
fewer logical sectors per erase block by packing the free and released
sector counts into a single byte (plus a little extra for 16 sectors per
erase block).  A device with a 64K erase block size can benefit from this
savings by selecting a 4096 or 8192 byte logical sector size, for example.

SMART FS Layer
==============

This layer interfaces with the SMART MTD block layer to allocate / release
logical sectors, create and destroy sector chains, and perform directory and
file I/O operations.  Each directory and file on the volume is represented
as a chain or "linked list" of logical sectors.  Thus the actual physical
sectors that a give file or directory uses does not need to be contiguous
and in fact can (and will) move around over time.  To manage the sector
chains, the SMARTFS layer adds a "chain header" after the sector's "sector
header".  This is a 5-byte header which contains the chain type (file or
directory), a "next logical sector" entry and the count of bytes actually
used within the sector.

Files are stored in directories, which are sector chains that have a
specific data format to track file names and "first" logical sector
numbers.  Each file in the directory has a fixed-size "directory entry"
that has bits to indicate if it is still active or has been deleted, file
permission bits, first sector number, date (utc stamp), and filename.  The
filename length is set from the CONFIG_SMARTFS_NAMLEN config value at the
time the mksmartfs command is executed.  Changes to the
CONFIG_SMARTFS_NAMLEN parameter will not be reflected on the volume
unless it is reformatted.  The same is true of the sector size parameter.

Subdirectories are supported by creating a new sector chain (of type
directory) and creating a standard directory entry for it in it's parent
directory.  Then files and additional sub-directories can be added to
that directory chain.  As such, each directory on the volume will occupy
a minimum of one sector on the device.  Subdirectories can be deleted
only if they are "empty" (i.e they reference no active entries).  There
are no provision made for performing a recursive directory delete.

New files and subdirectories can be added to a directory without needing
to copy and release the original directory sector.  This is done by
writing only the new entry data to the sector and ignoring the "bytes
used" field of the chain header for directories.  Updates (modifying
existing data) or appending to a sector for regular files requires copying
the file data to a new sector and releasing the old one.

SMARTFS organization
====================

The following example assumes 2 logical blocks per FLASH erase block.  The
actual relationship is determined by the FLASH geometry reported by the MTD
driver::

  ERASE LOGICAL                   Sectors begin with a sector header.  Sectors may
  BLOCK SECTOR      CONTENTS      be marked as "released," pending garbage collection
    n   2*n     --+---------------+
       Sector Hdr |LLLLLLLLLLLLLLL| Logical sector number (2 bytes)
                  |QQQQQQQQQQQQQQQ| Sequence number (2 bytes)
                  |SSSSSSSSSSSSSSS| Status bits (1 byte)
                  +---------------+
           FS Hdr |TTTTTTTTTTTTTTT| Sector Type (dir or file) (1 byte)
                  |NNNNNNNNNNNNNNN| Number of next logical sector in chain
                  |UUUUUUUUUUUUUUU| Number of bytes used in this sector
                  |               |
                  |               |
                  | (Sector Data) |
                  |               |
                  |               |
        2*n+1   --+---------------+
       Sector Hdr |LLLLLLLLLLLLLLL| Logical sector number (2 bytes)
                  |QQQQQQQQQQQQQQQ| Sequence number (2 bytes)
                  |SSSSSSSSSSSSSSS| Status bits (1 byte)
                  +---------------+
           FS Hdr |TTTTTTTTTTTTTTT| Sector Type (dir or file) (1 byte)
                  |NNNNNNNNNNNNNNN| Number of next logical sector in chain
                  |UUUUUUUUUUUUUUU| Number of bytes used in this sector
                  |               |
                  |               |
                  | (Sector Data) |
                  |               |
                  |               |
   n+1  2*(n+1) --+---------------+
       Sector Hdr |LLLLLLLLLLLLLLL| Logical sector number (2 bytes)
                  |QQQQQQQQQQQQQQQ| Sequence number (2 bytes)
                  |SSSSSSSSSSSSSSS| Status bits (1 byte)
                  +---------------+
           FS Hdr |TTTTTTTTTTTTTTT| Sector Type (dir or file) (1 byte)
                  |NNNNNNNNNNNNNNN| Number of next logical sector in chain
                  |UUUUUUUUUUUUUUU| Number of bytes used in this sector
                  |               |
                  |               |
                  | (Sector Data) |
                  |               |
                  |               |
                --+---------------+

Headers
=======
``SECTOR HEADER``
    Each sector contains a header (currently 5 bytes) for identifying the
    status of the sector.  The header contains the sector's logical sector
    number mapping, an incrementing sequence number to manage changes to
    logical sector data, and sector flags (committed, released, version, etc.).
    At the block level, there is no notion of sector chaining, only
    allocated sectors within erase blocks.

``FORMAT HEADER``
    Contains information regarding the format on the volume, including
    a format signature, formatted block size, name length within the directory
    chains, etc.

``CHAIN HEADER``
    The file system header (next 5 bytes) tracks file and directory sector
    chains and actual sector usage (number of bytes that are valid in the
    sector).  Also indicates the type of chain (file or directory).

Multiple Mount Points
=====================

Typically, a volume contains a single root directory entry (logical sector
number 1) and all files and subdirectories are "children" of that root
directory.  This is a traditional scheme and allows the volume to
be mounted in a single location within the VFS.  As a configuration
option, when the volume is formatted via the mksmartfs command, multiple
root directory entries can be created instead.  The number of entries to
be created is an added parameter to the mksmartfs command in this
configuration.

When this option has been enabled in the configuration and specified
during the format, then the volume will have multiple root directories
and can support a mount point in the VFS for each.  In this mode,
the device entries reported in the /dev directory will have a directory
number postfixed to the name, such as::

    /dev/smart0d1
    /dev/smart0d2
    /dev/smart1p1d1
    /dev/smart1p2d2
    etc.

Each device entry can then be mounted at different locations, such as::

    /dev/smart0d1 --> /usr
    /dev/smart0d2 --> /home
    etc.

Using multiple mount points is slightly different from using partitions
on the volume in that each mount point has the potential to use the
entire space on the volume vs. having a pre-allocated reservation of
space defined by the partition sizes.  Also, all files and directories
of all mount-points will be physically "mixed in" with data from the
other mount-points (though files from one will never logically "appear"
in the others).  Each directory structure is isolated from the others,
they simply share the same physical media for storage.

SMARTFS Limitations
===================

This implementation has several limitations that you should be aware
before opting to use SMARTFS:

1. There is currently no FLASH bad-block management code.  The reason for
   this is that the FS was geared for Serial NOR FLASH parts.  To use
   SMARTFS with a NAND FLASH, bad block management would need to be added,
   along with a few minor changes to eliminate single bit writes to release
   a sector, etc.

2. The implementation can support CRC-8 or CRC-16 error detection, and can
   relocate a failed write operation to a new sector.  However with no bad
   block management implementation, the code will continue it attempts at
   using failing block / sector, reducing efficiency and possibly successfully
   saving data in a block with questionable integrity.

3. The released-sector garbage collection process occurs only during a write
   when there are no free FLASH sectors.  Thus, occasionally, file writing
   may take a long time.  This typically isn't noticeable unless the volume
   is very full and multiple copy / erase cycles must be performed to
   complete the garbage collection.

4. The total number of logical sectors on the device must be 65534 or less.
   The number of logical sectors is based on the total device / partition
   size and the selected sector size.  For larger flash parts, a larger
   sector size would need to be used to meet this requirement. Creating a
   geometry which results in 65536 sectors (a 32MByte FLASH with 512 byte
   logical sector, for example) will cause the code to automatically reduce
   the total sector count to 65534, thus "wasting" the last two logical
   sectors on the device (they will never be used).

   This restriction exists because:

   a. The logical sector number is a 16-bit field (i.e. 65535 is the max).
   b. Logical sector number 65535 (0xFFFF) is reserved as this is typically
      the "erased state" of the FLASH.

ioctls
======

``BIOC_LLFORMAT``
    Performs a SMART low-level format on the volume.  This erases the volume
    and writes the FORMAT HEADER to the first physical sector on the volume.

``BIOC_GETFORMAT``
    Returns information about the format found on the volume during the
    "scan" operation which is performed when the volume is mounted.

``BIOC_ALLOCSECT``
    Allocates a logical sector on the device.

``BIOC_FREESECT``
    Frees a logical sector that had been previously allocated.  This
    causes the sector to be marked as "released" and possibly causes the
    erase block to be erased if it is the last active sector in the
    it's erase block.

``BIOC_READSECT``
    Reads data from a logical sector.  This uses a structure to identify
    the offset and count of data to be read.

``BIOC_WRITESECT``
    Writes data to a logical sector.  This uses a structure to identify
    the offset and count of data to be written.  May cause a logical
    sector to be physically relocated and may cause garbage collection
    if needed when moving data to a new physical sector.

Things to Do
============

- Add file permission checking to open / read / write routines.
- Add reporting of actual FLASH usage for directories (each directory
  occupies one or more physical sectors, yet the size is reported as
  zero for directories).




Using SmartFS
=============

.. warning:: This section is a copy-paste from old wiki documentation.
             It needs review and probably an update.

What is SmartFS
---------------

SmartFS stands for **Sector Mapped Allocation for Really Tiny (SMART) flash**.
It is a filesystem that has been designed to work primary with small,
serial NOR type flash parts that are 1M byte to 16M byte in size
(though this is not a limitation).
The filesystem operates by segmenting the flash (or flash partition)
into "logical sectors" of equal size and then managing them (allocating,
mapping, chaining, releasing, etc.) to build files and directories.

SmartFS Code Layering
---------------------

The system consists of two layers built on top of a standard NuttX MTD driver
layer (with it's associated hardware abstraction layer).

The code directly above the NuttX MTD driver is the SMART MTD layer.
This interfaces with the MTD (flash driver) layer and handles low-level media
operations such as logical sector allocation, freeing and management,
erase block management, low-level formatting, wear leveling, etc.

On top of the SMART MTD layer is the Smart Filesystem code.
The SmartFS code uses the logical sector services of the SMART MTD layer
to provide file and directory level management, such as creating new files,
chaining logical sectors together to create files, creating directories
and file / directory search and management routines.

+------------+------------------------------------+
| Smart FS   | fs / smartfs / *                   |
+============+====================================+
| SMART MTD  | ``drivers/mtd/smart.c``            |
+------------+---------+---------+---------+------+
| MTD Driver | m25px   | sst25   | filemtd | etc. |
+------------+---------+---------+---------+------+
| HW Driver  | spi dev | spi dev | VFS     | …    |
+------------+---------+---------+---------+------+

Example SmartFS Device Setup
----------------------------

Setting up a device for use with SmartFS is typically done in the config
specific source initialization files and would look something like::

  int board_app_initialize(uintptr_t arg)
  {
    FAR struct spi_dev_s *spi;
    FAR struct mtd_dev_s *mtd;
    int minor = 0;

    /* Initialize the SPI bus #3 with an M25P FLASH driver */

    spi = stm32_spibus_initialize(3);
    mtd = m25p_initialize(spi);

    /* Initialize SMART MTD to work with M25P FLASH device */

    smart_initialize(minor, mtd, NULL);
  }

Upon successful initialization of the code above,
the NuttX Virtual File System (VFS) will contain a new entry
called ``/dev/smart0`` to represent the SMART MTD device.
Note that this is not a filesystem, but rather a raw block device
(which may or may not already be formatted for use with SmartFS).

To use the ``/dev/smart0`` as a filesystem, it must be initialized
and mounted to the VFS as follows::

  nsh> mksmartfs /dev/smart0
  nsh> mount -t smartfs /dev/smart0 /mnt

Details of Operation
--------------------

Pages, Blocks, Sectors and things that FLASH
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

There are a number of companies that manufacture FLASH parts, and typically
they use varying terminology in their data sheets when referring
to device geometry.
All devices generally have different region sizes for programming, erasing
and reading and usually use terms like page, erase block, sector
and/or sub-sector.

To avoid confusion, NuttX uses the following terminology:

* **Page or Block:** The smallest area that a device can program
  with a single command. This is typically 256 bytes.
* **Erase Block:** The smallest area that a device can erase.
  If multiple erase block sizes are supported (e.g. 4K, 32K and 64K), then
  the smallest of those sizes, though for larger parts (8M Byte - 32M Byte),
  sometimes it is better to use the larger erase block size to keep RAM usage
  to a minimum.
* **Sector:** Same as Erase Block.
* **Logical Sector:** A size used by a specific filesystem for managing
  a region independently from the actual device specified blocks / sectors.
  SmartFS (for example) can use any power of 2 value from 256 through 32768.

Given that each manufacturer and each part has varying geometry sizes,
SmartFS uses a Logical Sector whose size is determined when the device
is formatted (defaulting to ``CONFIG_MTD_SMART_SECTOR_SIZE``).
The SmartFS code then performs all operations using logical sectors and maps
physical accesses to the device based on it's reported geometry.
This allows filesystem performance tuning (total sectors, minimum allocation
size, overhead waste, etc.) independently from the device's erase block size,
etc. Each logical sector contains a 10-byte header (5 for MTD layer,
5 for FS layer) for format management.

An example 128K Byte Flash with 32K and 4K Erase Block sizes and 256 byte page
read/write sizes:

+---------------------+-----------------------------------------------------------------------------------------------------+
| Bulk Erase          |  Entire Device                                                                                      |
+=====================+========================================+===============+===============+============================+
| Sector Erase        | 32K                                    | 32K           | 32K           | 32K                        |
+---------------------+-----------------------------+-----+----+----+-----+----+----+-----+----+----+-----+-----------------+
| Sector Erase        | 4K                          | ... | 4K | 4K | ... | 4K | 4K | ... | 4K | 4K | ... | 4K              |
+---------------------+-----+-----+-----+-----+-----+-----+----+----+-----+----+----+-----+----+----+-----+-----+-----+-----+
| Page read/write     | 256 | 256 | ... | 256 | 256 | ...                                                 | 256 | ... | 256 |
+---------------------+-----+-----+-----+-----+-----+-----------------------------------------------------+-----+-----+-----+
| Logical Sector (FS) | 512       | ... | 512       | ...                                                                   |
+---------------------+-----------+-----+-----------+-----------------------------------------------------------------------+


In the example above, a SMART MTD logical sector size of 512 bytes was chosen.
On the 128K Byte FLASH represented in the table above,
this means there would be:

* 128K / 512 = 256 Logical Sectors Total.
* 4K / 512 = 8 Logical Sectors per Erase Block.
* 512 / 256 = 2 MTD Read/Write Blocks per SMART Logical Sector.
* 10 / 512 = 1.95% Overhead for logical sector headers.
* A maximum of about 250 files / directories supported
  (considering format overhead).
* Allocations to files in 512 byte chunks
  (a zero-length file consumes 512 bytes).

Choosing a SMART Logical sector size of 256 bytes instead would give
the following results:

* 128K / 256 = 512 Logical Sectors Total.
* 4K / 256 = 16 Logical Sectors per Erase Block.
* 256 / 256 = 1 MTD Read/Write Block per SMART Logical Sector.
* 10 / 256 = 3.9% Overhead for logical sector headers.
* A maximum of about 506 files / directories supported
  (considering format overhead).
* Allocations to files in 256 byte chunks
  (a zero-length file consumes 256 bytes).

Checking Your MTD Geometry
^^^^^^^^^^^^^^^^^^^^^^^^^^

Most of the NuttX MTD drivers (in the ``drivers/mtd directory``) have been
tested to work with SmartFS / SMART MTD layer and to report block / erase
block sizes correctly.
However there may be one or more drivers that have not been validated.
When configuring SmartFS for the first time using a new / unknown MTD driver,
validate the reported geometries are sane based on the description above.

Additionally, care must be taken when selecting the SMART MTD logical sector
size. The selected size is represented using 3 bits in the logical sector
status byte and stored on each sector.
This means valid values for the logical sector are:

* 256, 512.
* 1024, 2048.
* 4096, 8192.
* 16384, 32768.

As shown in the table / example above, the logical sector size selection
controls the maximum number of files and minimum file size on the volume.
There are some limitations to the value selected imposed by the SMART MTD
code. The logical sector size must be selected such that:

* The total number of sectors on the device / partition fits in a 16-bit word.
  This means 65536 or less total sectors (65536 is supported, though
  the topmost 2 sectors will never be used).
* The logical sector size cannot be smaller than the MTD device's
  block/page size.
* The logical sector size cannot be larger than the MTD devices's
  reported erase block size.
* The total RAM used by the SMART MTD layer is dependent on the logical
  sector size (when not using Minimize RAM Config option).
  The selected logical sector size must not create a RAM requirement greater
  than the available RAM.
* The number of logical sectors per MTD device erase block cannot be greater
  than 256, and 4 - 128 is the best choice.
* If wear-leveling is enabled, then the following condition must be met:

  ``Total MTD Erase Blocks / 2 < (Logical Sector Size - 36) * 3``.

Selecting a logical sector size that creates 256 logical sectors per erase
block will create some wasted space on the device.
The SMART MTD layer uses a 1-byte variable for the "free sector count"
and the "released sector count".
This means it can only track up to 255 logical sectors per erase block.
When the sectPerEraseBlk == 256, the last logical sector in each erase
block will never be used, thus causing wasted space on the device.

RAM Usage Calculation
^^^^^^^^^^^^^^^^^^^^^

Efficient management of the filesystem requires building and maintaining RAM
resident status information of the SMART MTD logical sector structure
on the physical device.
During the ``smart_initialize()`` function, the code performs a device scan
(``smart_scan()`` routine) to perform this action.
Tasks performed by the smart_scan are:

* Detect if the format sector exists (logical sector zero with "SMRT" tag).
* Determine the logical sector size that was used to format the device.
* Locate the root directory logical sectors (logical sector 3-11).
* Count the number of free sectors on the device and per erase block.
* Count the number of released sectors on the device and per erase block.

If ``CONFIG_MTD_SMART_MINIMIZE_RAM`` is **not** set:

* Build a map of logical sector to physical sector numbers.

If ``CONFIG_MTD_SMART_WEAR_LEVEL`` is set:

* Allocate wear-level RAM and read leveling info from the format sector.

The amount of RAM consumed is dependent on the config settings
(``MINIMIZE_RAM``, ``WEAR_LEVEL``, etc.).
Rough calculation details are presented for both settings
of ``CONFIG_MTD_SMART_MINIMIZE_RAM``:

When ``CONFIG_MTD_SMART_MINIMIZE_RAM`` is **not** set

+---------------------------+------------------------+-------------------+--------------------+
| Item                      | RAM Requirement        | 1MB / 256 Logical | 8MB / 1024 Logical |
+===========================+========================+===================+====================+
| dev struct                | 368-380 (approx)       | 376               | 376                |
+---------------------------+------------------------+-------------------+--------------------+
| Logical sector map        | total_sectors * 2      | 8192              | 16384              |
+---------------------------+------------------------+-------------------+--------------------+
| Erase block free count    | total erase blocks     | 16                | 128                |
+---------------------------+------------------------+-------------------+--------------------+
| Erase block release count | total erase blocks     | 16                |  128               |
+---------------------------+------------------------+-------------------+--------------------+
| Wear status               | total erase blocks / 2 | 8                 |  64                |
+---------------------------+------------------------+-------------------+--------------------+
| MTD sector R/W buffer     | logical sector size    | 256               | 1024               |
+---------------------------+------------------------+-------------------+--------------------+
| FS sector R/W buffer      | logical sector size    | 256               | 1024               |
+---------------------------+------------------------+-------------------+--------------------+
| Total                                              | **9,176**         | **19,128**         |
+----------------------------------------------------+-------------------+--------------------+

On larger volumes, the RAM requirement increases significantly because
of the logical sector to physical sector mapping table.
To help keep RAM requirement under control, setting
the ``CONFIG_MTD_SMART_MINIMIZE_RAM`` option eliminates this sector-to-sector
map and replaces it with a sector-to-sector cache.
The cache size is user defined via the
``CONFIG_MTD_SMART_SECTOR_CACHE_SIZE`` option.
Using this RAM reduction mode trades off RAM usage for performance.

The cache always contains the format and root-directory logical to physical
mapping entries, and then stores additional mappings of recently used
logical sectors. When a logical sector is requested that is not contained
in the cache, then the MTD device is scanned front-to-back until
it's physical location on the device is located.
Additionally, if the number of logical sectors per erase block is 16 or less,
then the "free count" and "release count" can be packed into a single byte
per erase block.

When ``CONFIG_MTD_SMART_MINIMIZE_RAM=y`` and
``CONFIG_MTD_SMART_SECTOR_CACHE_SIZE=64``:

+---------------------------+-------------------------+-------------------+--------------------+
| Item                      | RAM Requirement         | 1MB / 256 Logical | 8MB / 1024 Logical |
+===========================+=========================+===================+====================+
| dev struct                | 368-400 (approx)        | 392               | 392                |
+---------------------------+-------------------------+-------------------+--------------------+
| Logical sector            | cache cache entries * 6 | 384               | 384                |
+---------------------------+-------------------------+-------------------+--------------------+
| Free sector bitmap        | total sectors / 8       | 512               | 1024               |
+---------------------------+-------------------------+-------------------+--------------------+
| Erase block free count    | total erase blocks      | 16                | 128                |
+---------------------------+-------------------------+-------------------+--------------------+
| Erase block release count | total erase blocks      | 16                | 128                |
+---------------------------+-------------------------+-------------------+--------------------+
| Wear status               | total erase blocks / 2  | 8                 | 64                 |
+---------------------------+-------------------------+-------------------+--------------------+
| MTD sector R/W buffer     | logical sector size     | 256               | 1024               |
+---------------------------+-------------------------+-------------------+--------------------+
| FS sector R/W buffer      | logical sector size     | 256               | 1024               |
+---------------------------+-------------------------+-------------------+--------------------+
| Total                                               | **1,832**         | **4,168**          |
+-----------------------------------------------------+-------------------+--------------------+

Partitions and Mount Points
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Creating partitions on the MTD flash / media provides the benefits
of physically isolating one filesystem from another, as well as a mechanism
for creating multiple mount points within the VFS.
There are some pros and cons to consider when deciding to use partitions
with SmartFS:

Partitions - PROS:

* Physically separate filesystems ... possible errors in one
  doesn't affect the other.
* Smaller partitions on a large device can use smaller logical sector size
  (good if files are small).
* Partitions could be used for redundancy.
* Multiple mount-points within the VFS can be performed if needed.

Partitions - CONS:

* Partition sizes must be understood ahead of time.
* Directory structure / files cannot span across partitions.
* Additional overhead is needed for sector management (one
  erase block + 4 pages/blocks reserved per partition
  vs. one+4 for entire device).
* Wear leveling (if enabled) will be limited to operate only within
  each partition and cannot span multiple partitions globally.

When using partitions with SmartFS, a partition name should be specified
during invocation of the ``smart_initialize()`` routine.
Though not a rule, a suggested standard is to provide partition names
like ``p1``, ``p2``, etc. Then the SMART MTD device entries
in the ``/dev`` directory will take the form::

  /dev/smart0p1
  /dev/smart0p2
  etc.

Prior to the addition of partitions in NuttX, the SmartFS implementation
had already implemented a feature which allows multiple VFS mount-points
to a single SMART MTD device.
This gives the appearance of multiple SmartFS filesystems, though in reality
it simply is a single filesystem with multiple logical root-directories.
Each of the root-directories on the filesystem will be logically isolated
from the others, and after the 'mksmartfs' each will have it's own
``/dev/smart*`` entry.

Multi-Root Directory PROS:

* Supports multiple mount points within the NuttX VFS.
* Each mount-point "appears" to be it's own partition ... each directory
  structure is isolated from the others.
* Any mount-point / directory structure can occupy as little or as much
  of the filesystem as needed.
* Mount-points can be enabled within partitions.
* Sector management overhead (reserved erase blocks, etc.) are shared
  by all mount-points.
* All mount points share RAM structures.
* Wear leveling is performed evenly over all mount-points.

Multi-Root Directory CONS:

* The logical directories are not physically isolated.
  Files, directories and individual sectors are all intermixed
  on the physical device.
* Any error in the filesystem can effect all mount-points / root directories.

This feature must be enabled specifically using the
``CONFIG_SMARTFS_MULTI_ROOT_DIRS=y`` option.
Setting this option will cause the SMART MTD ``/dev`` entries to be appended
with a directory number, such as ``d1``, ``d2``, etc.
Prior to creating a SmartFS filesystem on the raw device,
a single entry will be identified, such as::

  /dev/smart0d1
  /dev/smart1p1d1    (device with partitions and multi-root directories)

Then after executing the ``mksmartfs`` command, additional entries
will appear (each of which can be mounted to a different VFS location)::

  nsh> ls /dev
   ...
   /dev/ram0
   /dev/smart0d1
   /dev/zero
  nsh> mksmartfs /dev/smart0d1 3
  nsh> ls /dev
   ...
   /dev/ram0
   /dev/smart0d1
   /dev/smart0d2
   /dev/smart0d3
   /dev/zero
  nsh> mount -t smartfs /dev/smart0d1 /data
  nsh> mount -t smartfs /dev/smart0d2 /apps
  nsh> mount -t smartfs /dev/smart0d3 /recover

The ProcFS Interface
^^^^^^^^^^^^^^^^^^^^

When the PROCFS interface is enabled, each mounted SmartFS device
will appear under::

  /proc/fs/smartfs/smart#

The pseudo files reported for each entry will depend on the configured
options. Entries that can currently appear are:

+-------------+-------------------------+---------------------------+
| Entry       | ``CONFIG_MTD_SMART_*``  | Meaning                   |
+=============+=========================+===========================+
| status      |                         | Report volume status      |
|             |                         | including geometry.       |
+-------------+-------------------------+---------------------------+
| debuglevel  |                         | Write ASCII '0' - '2'     |
|             |                         | to set debug print level. |
+-------------+-------------------------+---------------------------+
| erasemap    | WEAR_LEVEL=y            | Report map (A-N) of erase |
|             |                         | block erasures.           |
+-------------+-------------------------+---------------------------+
| mem         | ALLOC_DEBUG=y           | Print report of all SMART |
|             |                         | MTD memory allocs.        |
+-------------+-------------------------+---------------------------+

Example ``procfs`` usage::

  nsh> mount -t smartfs /dev/smart0 /mnt
  nsh> mount -t procfs /proc
  nsh> cat /proc/fs/smartfs/smart0/status

  Format version:    1
  Name Len:          16
  Total Sectors:     4096
  Sector Size:       256
  Format Sector:     0
  Dir Sector:        256
  Free Sectors:      4078
  Released Sectors:  0
  Unused Sectors:    0
  Block Erases:      0
  Sectors Per Block: 256
  Sector Utilization:100%
  Uneven Wear Count: 0

  nsh> cat /proc/fs/smartfs/smart0/erasemap
  BBACAACA
  AABAAAAA

  nsh> 

When Things Don't Work
^^^^^^^^^^^^^^^^^^^^^^

The SmartFS code and MTD layer have been pretty well tested and used
in production products. If things aren't working for you, there are
a couple of places to start debugging first.

Check FLASH MTD Driver
~~~~~~~~~~~~~~~~~~~~~~

Ensure the geometry of the FLASH MTD driver is following the NuttX standard
for block / erase block geometry sizes. Most of them do, but via simple
inspection of the code (as of version 7.16, July 12, 1016), the drivers
that are likely to have improper Geometry reporting (and thus incompatible
with SmartFS) are::

    s25fl1.c
    sst39vf.c

When configuring to use SmartFS, check the reported geometry of the MTD
driver you are using. If the ``geo.blocksize`` is reported to be the same
as the ``geo.sectorsize``, then there is likely an issue with the MTD driver
implementation.
But it is likely to be a reporting problem of the ``geo.blocksize`` that
is incorrect AND possibly the starting address calculations in the
``_bwrite`` / ``_bread`` routines may be incorrect.
These are BLOCK read / write operations and calculations need to be
performed using the block size (typically 256), not the sector size
(4K, 32K, etc.).

Check CONFIG Options
~~~~~~~~~~~~~~~~~~~~

Double check all of the CONFIG options for both the MTD driver
and the SMART MTD layer:

* ``CONFIG_MTD_SMART_SECTOR_SIZE``: Ensure it's not smaller than the block
  size or larger than erase block size.
* ``CONFIG_MTD_BYTE_WRITE``: If enabled, ensure the FLASH actually supports
  this mode (single byte programming).
* ``CONFIG_MTD_XXX_SECTOR512``: Ensure this is not set. This should only
  be use with FAT volumes.
* ``CONFIG_MTD_XXX_MANUFACTURER``: Double check this with the data
  sheet / MTD driver code.
* ``CONFIG_MTD_XXX_MEMORY_TYPE``: Double check this with the data sheet.

Check Writability to the Part
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Ensure the FLASH can be written successfully.
If the WP pin is pull active (i.e. the part is write protected),
then the SMART MTD layer will not be able to write any data.
Additionally validate there are no individual protected sectors on the device.


SmartFS Internals
=================

.. warning:: This section is a copy-paste from old wiki documentation.
             It needs review and probably an update.
 
General Structure
-----------------

As described in Using SmartFS, the code is divided into a SMART MTD layer
and a filesystem layer. The MTD layer divides the flash or partition into
equal sized logical sectors, allocates, deallocates and moves them around
as needed to fulfill requests from the filesystem layer.

The filesystem layer uses the logical sectors to store directory and file
information, and to create chains of logical sector numbers to build larger
files (and directories).

The diagram below depicts a portion of a SmartFS device showing the erase
blocks, logical sectors and sector assignments. In the diagram, the top row
of numbers is the absolute sector number within the device.
The bottom row with numbers represents the logical sector number assigned
to each absolute sector.
Using 5 bytes from the header in each sector, the MTD layer manages
the assignments of the logical sector numbers.

The filesystem layer receives the logical sector numbers reported from MTD
layer and assigns them to specific files and/or directories.
As a file grows in size, additional logical sector numbers are requested
and "chained" together. In the diagram, three files are depicted
(files ``a``, ``b`` and ``c``) with varying lengths.
The files shown are as follows:

* File ``a``: Has data in 3 sectors, ``a0-a2``, logical sectors ``12-14``.
* File ``b``: Has data in 4 sectors, ``b0-b3``, logical sectors ``15-18``.
* File ``c``: Has data in a single sector, ``c0``, logical sector ``19``.

+----------------------------------------------------------------------------------------------------+
| Device or partition                                                                                |
+===================+===================+===================+===================+====================+
| EB                | EB                |  EB               | EB                | ...                |
+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+-----+
| 0  | 1  | 2  | 3  | 4  | 5  | 6  | 7  | 8  | 9  | 10 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | ... |
+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+-----+
| LS | LS | LS | LS | LS | LS | LS | LS | LS | LS | LS | LS | LS | LS | LS | LS | LS | LS | LS | ... |
+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+-----+
| FS | b0 | -- | -- | RS | b1 | -- | -- | a0 | b2 | -- | -- | a1 | b3 | -- | -- | a2 | c0 | -- | ... |
+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+-----+
| 0  | 15 |    |    | 3  | 16 |    |    | 12 | 17 |    |    | 13 | 18 |    |    | 14 | 19 |    |     |
+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+----+-----+

* EB = Erase Block.
* LS = Logical Sector.
* FS = Format Sector.
* RS = Root-directory Sector.
* -- = Free Sector.

The filesystem layer uses last 5-bytes of each sector's header to save
the logical sector number of the next logical sector in the chain.
When a file is first created, it's name and beginning logical sector number
are recorded in the Root-directory Sector (or a sub-directory
sector as needed).

The filesystem layer uses the recorded logical sector numbers
in the directory sectors and header sector-chain information to perform
logical sector allocate, read/write and sector release requests to carry out
all typical file system operations.
The SMART MTD layer then performs all the logical to physical mapping,
wear-leveling, sector relocation and block erase operations, etc.
When a sector needs to be physically relocated, it will retain
it's logical sector number, preventing the need to update file sector-chain
information, etc.
The MTD layer will simply update the logical to physical map assignments.

When things change
^^^^^^^^^^^^^^^^^^

Writing data to the filesystem and then reading it back is great.
But at some point you might actually want to change or delete something.
When this happens, the existing data on the flash has to be modified.
As you probably already know, data on a flash can't simply be re-written,
it must be erased in large chunks (erase blocks) that are typically 4K,
32K or 64K in size. When erasing this large a chunk of the flash,
it is highly likely that there will be data in that erase block
which should NOT be erased.

When data in a logical sector on the flash needs to be modified or deleted,
SmartFS simply marks that logical sector as "released" without actually
touching the data. This is done using the characteristic of NOR flash
that allows multiple writes to a given address.
This feature allows any bit of any byte to be changed from a ``1`` state
to a ``0`` state, regardless of whether that byte had previously been written
with other bits set to a ``0`` state.
As long as there is no attempt to change any bits from a ``0`` to a ``1``,
each address can be programmed multiple times.

As shown in the diagrams below, the SMART MTD layer uses the 5th byte
of the logical sector header as a status byte.
The most significant bit of this status byte (``0x80``) indicates
if the sector has been allocated (contains valid data) while the next bit
(``0x40``) indicates if the sector has been "released" (contains data
that is no longer valid). When a sector is first allocated, the
``RELEASE`` bit is held at a ``1`` state, meaning the data
has not been released.
If the data in that sector needs to be changed, deleted or relocated,
the SMART MTD code will simply re-program the status byte,
changing the ``RELEASE`` bit to a ``0`` state.
At that point, all data in that logical sector becomes invalid.

Logical Sector MTD Header
^^^^^^^^^^^^^^^^^^^^^^^^^

+------------------------------------------------------------------------+------------+
| MTD Header (5 bytes)                                                   |  FS Header |
+=======================+=======+=====+==================================+============+
| Logical Sector Number | Seq # | CRC | Status                           | 5 Bytes    |
+----------+------------+-------+-----+----+----+----+---------+---------+------------+
| LSB      | MSB        |       |     | CB | RB | CE | SS(2-0) | FV(1-0) |            |
+----------+------------+-------+-----+----+----+----+---------+---------+------------+

* CB: Commit Bit.
* RB: Release Bit.
* CE: CRC Enable Bit.
* SS: Sector Size Bits.
* FV: Format Version Bits.
