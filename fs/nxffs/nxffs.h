/****************************************************************************
 * fs/nxffs/nxffs.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * References: Linux/Documentation/filesystems/romfs.txt
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __FS_NXFFS_NXFFS_H
#define __FS_NXFFS_NXFFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

#include <nuttx/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
#ifndef CONFIG_NXFFS_ERASEDSTATE
#  define CONFIG_NXFFS_ERASEDSTATE 0xff
#endif

#if CONFIG_NXFFS_ERASEDSTATE != 0xff && CONFIG_NXFFS_ERASEDSTATE != 0x00
#  error "CONFIG_NXFFS_ERASEDSTATE must be either 0x00 or 0xff"
#endif

/* NXFFS Definitions ********************************************************/
/* General NXFFS organization.  The following example assumes 4 logical
 * blocks per FLASH erase block.  The actual relationship is determined by
 * the FLASH geometry reported by the MTD driver.
 *
 * ERASE LOGICAL                   Inodes begin with a inode header.  inode may
 * BLOCK BLOCK       CONTENTS      be marked as "deleted," pending clean-up.
 *   n   4*n     --+--------------+
 *                 |BBBBBBBBBBBBBB| Logic block header
 *                 |IIIIIIIIIIIIII| Inodes begin with a inode header
 *                 |DDDDDDDDDDDDD| Data block containing inode data block
 *                 | (Inode Data) |
 *       4*n+1   --+--------------+
 *                 |BBBBBBBBBBBBBB| Logic block header
 *                 |DDDDDDDDDDDDDD| Inodes may consist of multiple data blocks
 *                 | (Inode Data) |
 *                 |IIIIIIIIIIIIII| Next inode header
 *                 |             | Possibly a few unused bytes at the end of a block
 *       4*n+2   --+--------------+
 *                 |BBBBBBBBBBBBBB| Logic block header
 *                 |DDDDDDDDDDDDDD|
 *                 | (Inode Data) |
 *       4*n+3   --+--------------+
 *                 |BBBBBBBBBBBBBB| Logic block header
 *                 |IIIIIIIIIIIIII| Next inode header
 *                 |DDDDDDDDDDDDDD|
 *                 | (Inode Data) |
 *  n+1  4*(n+1) --+--------------+
 *                 |BBBBBBBBBBBBBB| Logic block header
 *                 |              | All FLASH is unused after the end of the final
 *                 |              | inode.
 *               --+--------------+
 *
 * General operation:
 *   Inodes are written starting at the beginning of FLASH.  As inodes are
 *   deleted, they are marked as deleted but not removed.  As new inodes are
 *   written, allocations  proceed to toward the end of the FLASH -- thus,
 *   supporting wear leveling by using all FLASH blocks equally.
 *
 *   When the FLASH becomes full (no more space at the end of the FLASH), a
 *   clean-up operation must be performed:  All inodes marked deleted are
 *   finally removed and the remaining inodes are packed at the beginning of
 *   the FLASH.  Allocations then continue at the freed FLASH memory at the
 *   end of the FLASH.
 *
 * BLOCK HEADER:
 *   The block header is used to determine if the block has every been
 *   formatted and also indicates bad blocks which should never be used.
 *
 * INODE HEADER:
 *   Each inode begins with an inode header that contains, among other things,
 *   the name of the inode, the offset to the first data block, and the
 *   length of the inode data.
 *
 *   At present, the only kind of inode support is a file.  So for now, the
 *   term file and inode are interchangeable.
 *
 * INODE DATA HEADER:
 *   Inode data is enclosed in a data header.  For a given inode, there
 *   is at most one inode data block per logical block.  If the inode data
 *   spans more than one logical block, then the inode data may be enclosed
 *   in multiple data blocks, one per logical block.
 *
 * NXFFS Limitations:
 * 1. Since the files are contiguous in FLASH and since allocations always
 *    proceed toward the end of the FLASH, there can only be one file opened
 *    for writing at a time.  Multiple files may be opened for reading.
 * 2. Files may not be increased in size after they have been closed.  The
 *    O_APPEND open flag is not supported.
 * 3. Files are always written sequential.  Seeking within a file opened for
 *    writing will not work.
 * 4. There are no directories, however, '/' may be used within a file name
 *    string providing some illusion of directories.
 * 5. Files may be opened for reading or for writing, but not both: The O_RDWR
 *    open flag is not supported.
 * 6. The clean-up process occurs only during a write when the free FLASH
 *    memory at the end of the FLASH is exhausted.  Thus, occasionally, file
 *    writing may take a long time.
 *
 */

/* Values for logical block state.  Basically, there are only two, perhaps
 * three, states:
 *
 * BLOCK_STATE_GOOD - The block is not known to be bad.
 * BLOCK_STATE_BAD  - An error was found on the block and it is marked bad.
 * Other values     - The block is bad and has an invalid state.
 */

#define BLOCK_STATE_GOOD          CONFIG_NXFFS_ERASEDSTATE
#define BLOCK_STATE_BAD           0xaa

/* Values for NXFFS inode state.  Similar there are 2 (maybe 3) inode states:
 *
 * INODE_STATE_FILE    - The inode is a valid usuable, file
 * INODE_STATE_DELETED - The inode has been deleted.
 */

#define INODE_STATE_FILE          CONFIG_NXFFS_ERASEDSTATE
#define INODE_STATE_DELETED       0x55

/* Number of bytes in an the NXFFS magic sequences */

#define NXFFS_MAGICSIZE	          4

/* Internal definitions *****************************************************/
/* If we encounter this number of erased bytes, we assume that all of the
 * flash beyond this point is erased.
 */

#define NXFFS_NERASED         128

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure defines each packed block on the FLASH media */

struct nxffs_block_s
{
  uint8_t                magic[4];  /* 0-3: Magic number for valid block */
  uint8_t                state;     /* 4: Block state: See BLOCK_STATE_* */
};
#define SIZEOF_NXFFS_BLOCK_HDR 5

/* This structure defines each packed NXFFS inode header on the FLASH media */

struct nxffs_inode_s
{
  uint8_t                magic[4];  /* 0-3: Magic number for valid inode */
  uint8_t                state;     /* 4: Inode state: See INODE_STATE_* */
  uint8_t                noffset;   /* 5: Offset to the file name from the header */
  uint8_t                doffset;   /* 5: Offset to data from file header */
  uint8_t                utc[4];    /* 7-9: Creation time */
  uint8_t                crc[4];    /* 10-13: CRC32 */
  uint8_t                datlen[4]; /* 14-17: Length of data in bytes */
                                    /* 18-: Variable length file name follows */
};
#define SIZEOF_NXFFS_INODE_HDR 18

/* This structure defines each packed NXFFS data header on the FLASH media */

struct nxffs_data_s
{
  uint8_t                magic[4];  /* 0-3: Magic number for valid data */
  uint8_t                crc[4];    /* 4-7: CRC32 */
  uint8_t                datlen[4]; /* 8-11: Length of data in bytes */
                                    /* 12-: Variable length data follows */
};
#define SIZEOF_NXFFS_DATA_HDR 12

/* This is an in-memory representation of the NXFFS inode as extracted from
 * FLASH and with additional state information.
 */

struct nxffs_entry_s
{
  off_t                  hoffset;   /* Offset to the inode on the media */
  off_t                  doffset;   /* Offset to the data on the media */
  FAR char              *name;      /* inode name */
  uint32_t               utc;       /* Time stamp */
  uint32_t               datlen;    /* Length of inode data */
};

/* This structure represents the overall state of on NXFFS instance. */

struct nxffs_volume_s
{
  FAR struct mtd_dev_s  *mtd;       /* Supports FLASH access */
  sem_t                  exclsem;   /* Used to assure thread-safe access */
  struct mtd_geometry_s  geo;       /* Device geometry */
  uint8_t                wrbusy: 1; /* 1: Volume open for writing */
  uint8_t                blkper;    /* R/W blocks per erase block */
  uint8_t                ncached;   /* Number of blocks in cache */
  uint16_t               iooffset;  /* Next offset in read/write access (in ioblock) */
  off_t                  inoffset;  /* Offset to the first valid inode header */
  off_t                  froffset;  /* Offset to the first free byte */
  off_t                  ioblock;   /* Current block number being accessed */
  off_t                  cblock;    /* Starting block number in cache */
  FAR uint8_t           *cache;     /* Allocated erase block */
};

/* This structure describes the state of one open file.  This structure
 * is protected by the volume semaphore.
 */

struct nxffs_ofile_s
{
  struct nxffs_ofile_s  *flink;     /* Supports a singly linked list */
  int16_t                crefs;     /* Reference count */
  mode_t                 mode;      /* Open mode */
  struct nxffs_entry_s   entry;     /* Describes the NXFFS inode entry */
};

/* A file opened for writing require some additional information */

struct nxffs_wrfile_s
{
  /* The following fields provide the common open file information. */

  struct nxffs_ofile_s   ofile;

  /* The following fields are required to support the current write
   * operation.  Note that the size of the current block can be determined
   * from (wroffset - dathdr - SIZEOF_NXFFS_DATA_HDR).  Basic write
   * operation:
   *
   * 1. Inode header location determined (but not yet written).
   * 2. Block header location determined (but not yet written).
   * 3. Check FLASH memory to make sure that it is erased.
   * 4. As data is written, wrlen is updated and the data is written to FLASH.
   * 5. If the end of the FLASH block is encountered, the data block CRC is
   *    calculated and the block header is also written to flash.
   * 6. When the file is closed, the final, partial data block is written to
   *    FLASH in the same way.  The final file size is determined, the header
   *    CRC is calculated, and the inode header is written to FLASH, completing
   *    the write operation.
   */

  uint16_t              wrlen;      /* Number of bytes written in data block */
  off_t                 dathdr;     /* FLASH offset to the current data header */
};

/* This structure describes the state of the blocks on the NXFFS volume */

struct nxffs_blkstats_s
{
  off_t                 nblocks;    /* Total number of FLASH blocks */
  off_t                 ngood;      /* Number of good FLASH blocks found */
  off_t                 nbad;       /* Number of well-formatted FLASH blocks marked as bad */
  off_t                 nunformat;  /* Number of unformatted FLASH blocks */
  off_t                 ncorrupt;   /* Number of blocks with correupted format info */
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* The magic number that appears that the beginning of each NXFFS (logical)
 * block
 */

extern const uint8_t g_blockmagic[NXFFS_MAGICSIZE];

/* The magic number that appears that the beginning of each NXFFS inode */

extern const uint8_t g_inodemagic[NXFFS_MAGICSIZE];

/* The magic number that appears that the beginning of each NXFFS inode
 * data block.
 */

extern const uint8_t g_datamagic[NXFFS_MAGICSIZE];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_limits
 *
 * Description:
 *   Recalculate file system limits:  (1) the FLASH offset to the first,
 *   valid inode, and (2) the FLASH offset to the first, unused byte after
 *   the last inode (invalid or not).
 *
 *   The first, lower limit must be recalculated: (1) initially, (2)
 *   whenever the first inode is deleted, or (3) whenever inode is moved
 *   as part of the clean-up operation.
 *
 *   The second, upper limit must be (1) incremented whenever new file
 *   data is written, or (2) recalculated as part of the clean-up operation.
 *
 * Input Parameters:
 *   volume - Identifies the NXFFS volume
 *
 * Returned Value:
 *   Zero on success. Otherwise, a negated error is returned indicating the
 *   nature of the failure.
 *
 * Defined in nxffs_initialize.c
 *
 ****************************************************************************/

extern int nxffs_limits(FAR struct nxffs_volume_s *volume);

/****************************************************************************
 * Name: nxffs_rdle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Values:
 *   A uint16_t representing the whole 16-bit integer value
 *
 * Defined in nxffs_util.c
 *
 ****************************************************************************/

extern uint16_t nxffs_rdle16(const uint8_t *val);

/****************************************************************************
 * Name: nxffs_rdle32
 *
 * Description:
 *   Get a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Values:
 *   A uint32_t representing the whole 32-bit integer value
 *
 * Defined in nxffs_util.c
 *
 ****************************************************************************/

extern uint32_t nxffs_rdle32(const uint8_t *val);

/****************************************************************************
 * Name: nxffs_rdcache
 *
 * Description:
 *   Read one or more logical blocks into the volume cache memory.
 *
 * Input Parameters:
 *   volume - Describes the current volume
 *   block  - The first logical block to read
 *   nblocks - The number of logical blocks to be read.
 *
 * Returned Value:
 *   Negated errnos are returned only in the case of MTD reported failures.
 *   Nothing in the volume data itself will generate errors.
 *
 * Defined in nxffs_cache.c
 *
 ****************************************************************************/

extern int nxffs_rdcache(FAR struct nxffs_volume_s *volume, off_t block,
                         uint8_t nblocks);

/****************************************************************************
 * Name: nxffs_wrcache
 *
 * Description:
 *   Write one or more logical blocks from the volume cache memory.
 *
 * Input Parameters:
 *   volume - Describes the current volume
 *   block  - The first logical block to write
 *   nblocks - The number of logical blocks to be write.
 *
 * Returned Value:
 *   Negated errnos are returned only in the case of MTD reported failures.
 *   Nothing in the volume data itself will generate errors.
 *
 * Defined in nxffs_cache.c
 *
 ****************************************************************************/

extern int nxffs_wrcache(FAR struct nxffs_volume_s *volume, off_t block,
                         uint8_t nblocks);

/****************************************************************************
 * Name: nxffs_ioseek
 *
 * Description:
 *   Seek to a position in FLASH memory.  This simply sets up the offsets
 *   and pointer values.  This is a necessary step prior to using
 *   nxffs_getc().
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   offset - The physical offset in bytes from the beginning of the FLASH
 *     in bytes.
 *
 * Defined in nxffs_cache.c
 *
 ****************************************************************************/

extern void nxffs_ioseek(FAR struct nxffs_volume_s *volume, off_t offset);

/****************************************************************************
 * Name: nxffs_getc
 *
 * Description:
 *   Get the next byte from FLASH.  This function allows the data in the
 *   formatted FLASH blocks to be read as a continuous byte stream, skipping
 *   over bad blocks and block headers as necessary.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume.  The paramters ioblock and iooffset
 *     in the volume structure determine the behavior of nxffs_getc().
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno indicating the
 *   nature of the failure.
 *   
 * Defined in nxffs_cache.c
 *
 ****************************************************************************/

extern int nxffs_getc(FAR struct nxffs_volume_s *volume);

/****************************************************************************
 * Name: nxffs_rddata
 *
 * Description:
 *   Read a sequence of data bytes from the FLASH memory.  This function
 *   allows the data in the formatted FLASH blocks to be read as a continuous\
 *   byte stream, skipping over bad blocks and block headers as necessary.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume.  The paramters ioblock and iooffset
 *     in the volume structure determine the behavior of nxffs_getc().
 *
 * Returned Value:
 *   The number of bytes read is returned on success.  Otherwise, a negated
 *   errno indicating the nature of the failure.
 *
 * Defined in nxffs_cache.c
 *
 ****************************************************************************/

extern ssize_t nxffs_rddata(FAR struct nxffs_volume_s *volume,
                            FAR uint8_t *buffer, size_t buflen);

/****************************************************************************
 * Name: nxffs_freeentry
 *
 * Description:
 *   The inode values returned by nxffs_nextentry() include allocated memory
 *   (specifically, the file name string).  This function should be called
 *   to dispose of that memory when the inode entry is no longer needed.
 *
 *   Note that the nxffs_entry_s containing structure is not freed.  The
 *   caller may call kfree upon return of this function if necessary to
 *   free the entry container.
 *
 * Input parameters:
 *   entry  - The entry to be freed.
 *
 * Returned Value:
 *   None
 *
 * Defined in nxffs_inode.c
 *
 ****************************************************************************/

extern void nxffs_freeentry(FAR struct nxffs_entry_s *entry);

/****************************************************************************
 * Name: nxffs_nextentry
 *
 * Description:
 *   Search for the next valid inode starting at the provided FLASH offset.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume.
 *   offset - The FLASH memory offset to begin searching.
 *   entry  - A pointer to memory provided by the caller in which to return
 *     the inode description.
 *  
 * Returned Value:
 *   Zero is returned on success. Otherwise, a negated errno is returned
 *   that indicates the nature of the failure.
 *
 * Defined in nxffs_inode.c
 *
 ****************************************************************************/

extern int nxffs_nextentry(FAR struct nxffs_volume_s *volume, off_t offset,
                           FAR struct nxffs_entry_s *entry);

/****************************************************************************
 * Name: nxffs_findinode
 *
 * Description:
 *   Search for an inode with the provided name starting with the first
 *   valid inode and proceeding to the end FLASH or until the matching
 *   inode is found.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   name   - The name of the inode to find
 *   entry  - The location to return information about the inode.
 *
 * Returned Value:
 *   Zero is returned on success. Otherwise, a negated errno is returned
 *   that indicates the nature of the failure.
 *
 * Defined in nxffs_inode.c
 *
 ****************************************************************************/

extern int nxffs_findinode(FAR struct nxffs_volume_s *volume,
                           FAR const char *name,
                           FAR struct nxffs_entry_s *entry);

/****************************************************************************
 * Name: nxffs_verifyblock
 *
 * Description:
 *   Assure the the provided (logical) block number is in the block cache
 *   and that it has a valid block header (i.e., proper magic and
 *   marked good)
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   block - The (logical) block number to load and verify.
 *
 * Returned Values:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned indicating the nature of the failure.
 *
 * Defined in nxffs_block.c
 *
 ****************************************************************************/

extern int nxffs_verifyblock(FAR struct nxffs_volume_s *volume, off_t block);

/****************************************************************************
 * Name: nxffs_validblock
 *
 * Description:
 *   Find the next valid (logical) block in the volume.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   block  - On entry, this provides the starting block number.  If the
 *     function is succesfful, then this memory location will hold the
 *     block number of the next valid block on return.
 *
 *  Returned Value:
 *    Zero on success otherwise a negated errno value indicating the nature
 *    of the failure.
 *
 * Defined in nxffs_block.c
 *
 ****************************************************************************/

extern int nxffs_validblock(struct nxffs_volume_s *volume, off_t *block);

/****************************************************************************
 * Name: nxffs_blockstats
 *
 * Description:
 *   Analyze the NXFFS volume.  This operation must be performed when the
 *   volume is first mounted in order to detect if the volume has been
 *   formatted and contains a usable NXFFS file system.
 *
 * Input Parameters:
 *   volume - Describes the current NXFFS volume.
 *   stats  - On return, will hold nformation describing the state of the
 *     volume.
 *
 * Returned Value:
 *   Negated errnos are returned only in the case of MTD reported failures.
 *   Nothing in the volume data itself will generate errors.
 *
 * Defined in nxffs_blockstats.c
 *
 ****************************************************************************/

extern int nxffs_blockstats(FAR struct nxffs_volume_s *volume,
                            FAR struct nxffs_blkstats_s *stats);

/****************************************************************************
 * Name: nxffs_reformat
 *
 * Description:
 *   Erase and reformat the entire volume.  Verify each block and mark
 *   improperly erased blocks as bad.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume to be reformatted.
 *
 * Returned Value:
 *   Zero on success or a negated errno on a failure.  Failures will be
 *   returned n the case of MTD reported failures o.
 *   Nothing in the volume data itself will generate errors.
 *
 * Defined in nxffs_reformat.c
 *
 ****************************************************************************/

extern int nxffs_reformat(FAR struct nxffs_volume_s *volume);

/****************************************************************************
 * Name: nxffs_findofile
 *
 * Description:
 *   Search the list of already opened files to see if the inode of this
 *   name is one of the opened files.
 *
 * Input Parameters:
 *   name - The name of the inode to check.
 *
 * Returned Value:
 *   If an inode of this name is found in the list of opened inodes, then
 *   a reference to the open file structure is returned.  NULL is returned
 *   otherwise.
 *
 * Defined in nxffs_open.c
 *
 ****************************************************************************/

extern FAR struct nxffs_ofile_s *nxffs_findofile(FAR const char *name);

/****************************************************************************
 * Name: nxffs_rminode
 *
 * Description:
 *   Remove an inode from FLASH.  This is the internal implementation of
 *   the file system unlinke operation.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume.
 *   name - the name of the inode to be deleted.
 *
 * Returned Value:
 *   Zero is returned if the inode is successfully deleted.  Otherwise, a
 *   negated errno value is returned indicating the nature of the failure.
 *
 ****************************************************************************/

extern int nxffs_rminode(FAR struct nxffs_volume_s *volume, FAR const char *name);

/****************************************************************************
 * Standard mountpoint operation methods
 *
 * Description:
 *   See include/nuttx/fs.h
 *
 * - nxffs_open() and nxffs_close() are defined in nxffs_open.c
 * - nxffs_ioctl() is defined in nxffs_ioctl.c
 * - nxffs_opendir(), nxffs_readdir(), and nxffs_rewindir() are defined in
 *   nxffs_dir.c
 * - nxffs_stat() and nxffs_statfs() are defined in nxffs_stat.c
 * - nxffs_unlink() is defined nxffs_unlink.c
 *
 ****************************************************************************/

struct file;        /* Forward references */
struct inode;
struct fs_dirent_s;
struct statfs;
struct stat;

extern int nxffs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode);
extern int nxffs_close(FAR struct file *filep);
extern ssize_t nxffs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
extern ssize_t nxffs_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
extern int nxffs_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
extern int nxffs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                         FAR struct fs_dirent_s *dir);
extern int nxffs_readdir(FAR struct inode *mountpt, FAR struct fs_dirent_s *dir);
extern int nxffs_rewinddir(FAR struct inode *mountpt, FAR struct fs_dirent_s *dir);
extern int nxffs_bind(FAR struct inode *blkdriver, FAR const void *data,
                      FAR void **handle);
extern int nxffs_unbind(FAR void *handle, FAR struct inode **blkdriver);
extern int nxffs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf);
extern int nxffs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                      FAR struct stat *buf);
extern int nxffs_unlink(FAR struct inode *mountpt, FAR const char *relpath);

#endif /* __FS_NXFFS_NXFFS_H */


