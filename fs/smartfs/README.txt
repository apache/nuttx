SMARTFS README
^^^^^^^^^^^^^^

This README file contains information about the implemenation of the NuttX
Sector Mapped Allocation for Really Tiny (SMART) FLASH file system, SMARTFS.

Contents:

  Features
  General operation
  SMARTFS organization
  Headers
  Multiple mount points
  SMARTFS Limitations
  ioctls
  Things to Do

Features
========

  This implementation is a full-feature file system from the perspective of
  file and directory access (i.e. not considering low-level details like the
  lack of wear-leveling, etc.).  The SMART File System was designed specifically
  for small SPI based FLASH parts (1-8 Mbyte for example), though this is not
  a limitation.  It can certainly be used for any size FLASH and can work with
  any MTD device by binding it with the SMART MTD layer.  The FS includes
  support for:
    - Multiple open files from different threads.
    - Open for read/write access with seek capability.
    - Appending to end of files in either write, append or read/write
      open modes.
    - Directory support.
    - Support for multiple mount points on a single volume / partition (see
      details below).

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
  When a sector is allocated, the COMMITED flag will be "set" (changed from
  erase state to non-erase state) to indicate the sector data is valid.  When
  a sector's data needs to be deleted, the RELEASED flag will be "set" to
  indicate the sector is no longer in use.  This is done because the erase
  block continaing the sector cannot necessarly be erased until all sectors
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
  use, including

    Sector 0:    The Format Sector.  Has a format signture, format version, etc.
    Sector 1:    The 1st (or only) Root Directory entry
    Sector 2-8:  Additional root directories when Multi-Mount points are supported.
    Sector 9-11: Reserved (maybe for sector wear-leveling, etc.)

  To perform allocations, the SMART MTD block layer searches each erase block
  on the device to identify the one with the most free sectors.  Free sectors
  are those that have all bytes in the "erased state", meaning they have not
  been previously allocated/released since the last block erase.  Not all
  sectors on the device can be allocated ... the SMART MTD block driver must
  reserve at least one erase-block worth of unused sectors to perform
  garbage collection, which will be performed automatically when no free
  sectors are available.

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
  SMART will use it.  As of the writing of this README, only the
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

  SMART FS Layer
  ==============

  This layer interfaces with the SMART MTD block layer to allocate / release
  logical sectors, create and destroy sector chains, and perform directory and
  file I/O operations.  Each directory and file on the volume is represented
  as a chain or "linked list" of logical sectors.  Thus the actual physical
  sectors that a give file or directory uses does not need to be contigous
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
  driver.

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
  SECTOR HEADER:
    Each sector contains a header (currently 5 bytes) for identifying the
    status of the sector.  The header contains the sector's logical sector
    number mapping, an incrementing sequence number to manage changes to
    logical sector data, and sector flags (committed, released, version, etc.).
    At the block level, there is no notion of sector chaining, only
    allocated sectors within erase blocks.

  FORMAT HEADER:
    Contains information regarding the format on the volume, including
    a format signature, formatted block size, name length within the directory
    chains, etc.

  CHAIN HEADER:
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
  number postfixed to the name, such as:

    /dev/smart0d1
    /dev/smart0d2
    /dev/smart1p1d1
    /dev/smart1p2d2
    etc.

  Each device entry can then be mounted at different locations, such as:

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

1. No wear leveling has been implemented.  The allocation scheme has a
   bit of inherent wear-leveling since it automatically distributes
   sector allocations across the device, but no provisions exist to
   guarantee equal wearing.

2. There is no CRC or checksum calculations performed on the data stored
   to FLASH, so no error detection has been implemented.  This could be
   added by "stealing" one of the sequence number bytes in the sector
   header and incrementing the sector version number.

3. There is currently no FLASH bad-block management code.  The reason for
   this is that the FS was geared for Serial NOR FLASH parts.  To use
   SMARTFS with a NAND FLASH, bad block management would need to be added.

4. The released-sector garbage collection process occurs only during a write
   when there are no free FLASH sectors.  Thus, occasionally, file writing
   may take a long time.  This typically isn't noticable unless the volume
   is very full and multiple copy / erase cycles must be performed to
   complete the garbage collection.

5. The total number of logical sectors on the device must be less than 65534.
   The number of logical sectors is based on the total device / partition
   size and the selected sector size.  For larger flash parts, a larger
   sector size would need to be used to meet this requirement. This
   restriction exists because:

   a. The logical sector number is a 16-bit field (i.e. 65535 is the max).
   b. The SMART MTD layer reserves 1 logical sector for a format sector.
   c. Logical sector number 65535 (0xFFFF) is reerved as this is typically
      the "erased state" of the FLASH.

ioctls
======

  BIOC_LLFORMAT
    Performs a SMART low-level format on the volume.  This erases the volume
    and writes the FORMAT HEADER to the first physical sector on the volume.

  BIOC_GETFORMAT
    Returns information about the format found on the volume during the
    "scan" operation which is performed when the volume is mounted.

  BIOC_ALLOCSECT
    Allocates a logical sector on the device.

  BIOC_FREESECT
    Frees a logical sector that had been previously allocated.  This
    causes the sector to be marked as "released" and possibly causes the
    erase block to be erased if it is the last active sector in the
    it's erase block.

  BIOC_READSECT
    Reads data from a logial sector.  This uses a structure to identify
    the offset and count of data to be read.

  BIOC_WRITESECT
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
- Add sector aging to provide some degree of wear-leveling.
- Possibly steal a byte from the sector header's sequence number and
  implement a sector data verification scheme using a 1-byte CRC.


