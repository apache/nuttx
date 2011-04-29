General NXFFS organization
==========================

The following example assumes 4 logical blocks per FLASH erase block.  The
actual relationship is determined by the FLASH geometry reported by the MTD
driver.

ERASE LOGICAL                   Inodes begin with a inode header.  inode may
BLOCK BLOCK       CONTENTS      be marked as "deleted," pending clean-up.
  n   4*n     --+--------------+
                |BBBBBBBBBBBBBB| Logic block header
                |IIIIIIIIIIIIII| Inodes begin with a inode header
                |DDDDDDDDDDDDDD| Data block containing inode data block
                | (Inode Data) |
      4*n+1   --+--------------+
                |BBBBBBBBBBBBBB| Logic block header
                |DDDDDDDDDDDDDD| Inodes may consist of multiple data blocks
                | (Inode Data) |
                |IIIIIIIIIIIIII| Next inode header
                |              | Possibly a few unused bytes at the end of a block
      4*n+2   --+--------------+
                |BBBBBBBBBBBBBB| Logic block header
                |DDDDDDDDDDDDDD|
                | (Inode Data) |
      4*n+3   --+--------------+
                |BBBBBBBBBBBBBB| Logic block header
                |IIIIIIIIIIIIII| Next inode header
                |DDDDDDDDDDDDDD|
                | (Inode Data) |
 n+1  4*(n+1) --+--------------+
                |BBBBBBBBBBBBBB| Logic block header
                |              | All FLASH is unused after the end of the final
                |              | inode.
              --+--------------+

General operation
=================

  Inodes are written starting at the beginning of FLASH.  As inodes are
  deleted, they are marked as deleted but not removed.  As new inodes are
  written, allocations  proceed to toward the end of the FLASH -- thus,
  supporting wear leveling by using all FLASH blocks equally.

  When the FLASH becomes full (no more space at the end of the FLASH), a
  clean-up operation must be performed:  All inodes marked deleted are
  finally removed and the remaining inodes are packed at the beginning of
  the FLASH.  Allocations then continue at the freed FLASH memory at the
  end of the FLASH.

Headers
=======
  BLOCK HEADER:
    The block header is used to determine if the block has every been
    formatted and also indicates bad blocks which should never be used.

  INODE HEADER:
    Each inode begins with an inode header that contains, among other things,
    the name of the inode, the offset to the first data block, and the
    length of the inode data.

    At present, the only kind of inode support is a file.  So for now, the
    term file and inode are interchangeable.

  INODE DATA HEADER:
    Inode data is enclosed in a data header.  For a given inode, there
    is at most one inode data block per logical block.  If the inode data
    spans more than one logical block, then the inode data may be enclosed
    in multiple data blocks, one per logical block.

NXFFS Limitations
=================

This implementation is very simple as, as a result, has several limitations
that you should be aware before opting to use NXFFS:

1. Since the files are contiguous in FLASH and since allocations always
   proceed toward the end of the FLASH, there can only be one file opened
   for writing at a time.  Multiple files may be opened for reading.

2. Files may not be increased in size after they have been closed.  The
   O_APPEND open flag is not supported.

3. Files are always written sequential.  Seeking within a file opened for
   writing will not work.

4. There are no directories, however, '/' may be used within a file name
   string providing some illusion of directories.

5. Files may be opened for reading or for writing, but not both: The O_RDWR
   open flag is not supported.

6. The clean-up process occurs only during a write when the free FLASH
   memory at the end of the FLASH is exhausted.  Thus, occasionally, file
   writing may take a long time.

7. Another limitation is that there can be only a single NXFFS volume
   mounted at any time.  This has to do with the fact that we bind to
   an MTD driver (instead of a block driver) and bypass all of the normal
   mount operations.

