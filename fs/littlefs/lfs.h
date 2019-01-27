/****************************************************************************
 * fs/littlefs/lfs.h
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *
 * Ported by:
 *
 *   Copyright (C) 2019 Pinecone Inc. All rights reserved.
 *   Author: lihaichen <li8303@163.com>
 *
 * This port derives from ARM mbed logic which has a compatible 3-clause
 * BSD license:
 *
 *   Copyright (c) 2017, Arm Limited. All rights reserved.
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
 * 3. Neither the names ARM, NuttX nor the names of its contributors may be
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

#ifndef __FS_LITTLEFS_LFS_H
#define __FS_LITTLEFS_LFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Version info */

/* Software library version
 * Major (top-nibble), incremented on backwards incompatible changes
 * Minor (bottom-nibble), incremented on feature additions
 */

#define LFS_VERSION 0x00010007
#define LFS_VERSION_MAJOR (0xffff & (LFS_VERSION >> 16))
#define LFS_VERSION_MINOR (0xffff & (LFS_VERSION >>  0))

/* Version of On-disk data structures
 * Major (top-nibble), incremented on backwards incompatible changes
 * Minor (bottom-nibble), incremented on feature additions
 */

#define LFS_DISK_VERSION 0x00010001
#define LFS_DISK_VERSION_MAJOR (0xffff & (LFS_DISK_VERSION >> 16))
#define LFS_DISK_VERSION_MINOR (0xffff & (LFS_DISK_VERSION >>  0))

/* Max name size in bytes */

#ifndef LFS_NAME_MAX
#  define LFS_NAME_MAX NAME_MAX
#endif

/* Max file size in bytes */

#ifndef LFS_FILE_MAX
#  define LFS_FILE_MAX 2147483647
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef uint32_t lfs_size_t;
typedef uint32_t lfs_off_t;

typedef int32_t  lfs_ssize_t;
typedef int32_t  lfs_soff_t;

typedef uint32_t lfs_block_t;

/* Possible error codes, these are negative to allow
 * valid positive return values
 */

enum lfs_error_e
{
  LFS_ERR_OK       = 0,    /* No error */
  LFS_ERR_IO       = -5,   /* Error during device operation */
  LFS_ERR_CORRUPT  = -52,  /* Corrupted */
  LFS_ERR_NOENT    = -2,   /* No directory entry */
  LFS_ERR_EXIST    = -17,  /* Entry already exists */
  LFS_ERR_NOTDIR   = -20,  /* Entry is not a dir */
  LFS_ERR_ISDIR    = -21,  /* Entry is a dir */
  LFS_ERR_NOTEMPTY = -39,  /* Dir is not empty */
  LFS_ERR_BADF     = -9,   /* Bad file number */
  LFS_ERR_FBIG     = -27,  /* File too large */
  LFS_ERR_INVAL    = -22,  /* Invalid parameter */
  LFS_ERR_NOSPC    = -28,  /* No space left on device */
  LFS_ERR_NOMEM    = -12,  /* No more memory available */
};

/* File types */

enum lfs_type_e
{
  LFS_TYPE_REG        = 0x11,
  LFS_TYPE_DIR        = 0x22,
  LFS_TYPE_SUPERBLOCK = 0x2e,
};

/* File open flags */

enum lfs_open_flags_e
{
  /* open flags */

  LFS_O_RDONLY = 1,        /* Open a file as read only */
  LFS_O_WRONLY = 2,        /* Open a file as write only */
  LFS_O_RDWR   = 3,        /* Open a file as read and write */
  LFS_O_CREAT  = 0x0100,   /* Create a file if it does not exist */
  LFS_O_EXCL   = 0x0200,   /* Fail if a file already exists */
  LFS_O_TRUNC  = 0x0400,   /* Truncate the existing file to zero size */
  LFS_O_APPEND = 0x0800,   /* Move to end of file on every write */

  /* internally used flags */

  LFS_F_DIRTY   = 0x10000, /* File does not match storage */
  LFS_F_WRITING = 0x20000, /* File has been written since last flush */
  LFS_F_READING = 0x40000, /* File has been read since last flush */
  LFS_F_ERRED   = 0x80000, /* An error occured during write */
};

/* File seek flags */

enum lfs_whence_flags_e
{
  LFS_SEEK_SET = 0,   /* Seek relative to an absolute position */
  LFS_SEEK_CUR = 1,   /* Seek relative to the current file position */
  LFS_SEEK_END = 2,   /* Seek relative to the end of the file */
};

/* Configuration provided during initialization of the littlefs */

struct lfs_config_s
{
  /* Opaque user provided context that can be used to pass
   * information to the block device operations
   */

  FAR void *context;

  /* Read a region in a block. Negative error codes are propagated
   * to the user.
   */

  CODE int (*read)(FAR const struct lfs_config_s *c, lfs_block_t block,
                   lfs_off_t off, FAR void *buffer, lfs_size_t size);

  /* Program a region in a block. The block must have previously
   * been erased. Negative error codes are propagated to the user.
   * May return LFS_ERR_CORRUPT if the block should be considered bad.
   */

  CODE int (*prog)(FAR const struct lfs_config_s *c, lfs_block_t block,
                   lfs_off_t off, const void *buffer, lfs_size_t size);

  /* Erase a block. A block must be erased before being programmed.
   * The state of an erased block is undefined. Negative error codes
   * are propagated to the user.
   * May return LFS_ERR_CORRUPT if the block should be considered bad.
   */

  CODE int (*erase)(FAR const struct lfs_config_s *c, lfs_block_t block);

  /* Sync the state of the underlying block device. Negative error codes
   * are propagated to the user.
   */

  CODE int (*sync)(FAR const struct lfs_config_s *c);

  /* Minimum size of a block read. This determines the size of read buffers.
   * This may be larger than the physical read size to improve performance
   * by caching more of the block device.
   */

  lfs_size_t read_size;

  /* Minimum size of a block program. This determines the size of program
   * buffers. This may be larger than the physical program size to improve
   * performance by caching more of the block device.
   * Must be a multiple of the read size.
   */

  lfs_size_t prog_size;

  /* Size of an erasable block. This does not impact ram consumption and
   * may be larger than the physical erase size. However, this should be
   * kept small as each file currently takes up an entire block.
   * Must be a multiple of the program size.
   */

  lfs_size_t block_size;

  /* Number of erasable blocks on the device. */

  lfs_size_t block_count;

  /* Number of blocks to lookahead during block allocation. A larger
   * lookahead reduces the number of passes required to allocate a block.
   * The lookahead buffer requires only 1 bit per block so it can be quite
   * large with little ram impact. Should be a multiple of 32.
   */

  lfs_size_t lookahead;

  /* Optional, statically allocated read buffer. Must be read sized. */

  FAR void *read_buffer;

  /* Optional, statically allocated program buffer. Must be program sized. */

  FAR void *prog_buffer;

  /* Optional, statically allocated lookahead buffer. Must be 1 bit per
   * lookahead block.
   */

  FAR void *lookahead_buffer;

  /* Optional, statically allocated buffer for files. Must be program sized.
   * If enabled, only one file may be opened at a time.
   */

  FAR void *file_buffer;
};

/* Optional configuration provided during lfs_file_opencfg */

struct lfs_file_config_s
{
  /* Optional, statically allocated buffer for files. Must be program sized.
   * If NULL, malloc will be used by default.
   */

  FAR void *buffer;
};

/* File info structure */

struct lfs_info_s
{
  /* Type of the file, either LFS_TYPE_REG or LFS_TYPE_DIR */

  uint8_t type;

  /* Size of the file, only valid for REG files */

  lfs_size_t size;

  /* Name of the file stored as a null-terminated string */

  char name[LFS_NAME_MAX+1];
};


/* littlefs data structures */

typedef struct lfs_entry_s
{
  lfs_off_t off;

  struct lfs_disk_entry_s
  {
    uint8_t type;
    uint8_t elen;
    uint8_t alen;
    uint8_t nlen;
    union
    {
      struct
      {
        lfs_block_t head;
        lfs_size_t size;
      } file;
      lfs_block_t dir[2];
    } u;
 } d;
} lfs_entry_t;

typedef struct lfs_cache_s
{
  lfs_block_t block;
  lfs_off_t off;
  FAR uint8_t *buffer;
} lfs_cache_t;

typedef struct lfs_file_s
{
  FAR struct lfs_file_s *next;
  lfs_block_t pair[2];
  lfs_off_t poff;

  lfs_block_t head;
  lfs_size_t size;

  FAR const struct lfs_file_config_s *cfg;
  uint32_t flags;
  lfs_off_t pos;
  lfs_block_t block;
  lfs_off_t off;
  lfs_cache_t cache;
} lfs_file_t;

typedef struct lfs_dir_s
{
  FAR struct lfs_dir_s *next;
  lfs_block_t pair[2];
  lfs_off_t off;

  lfs_block_t head[2];
  lfs_off_t pos;

  struct lfs_disk_dir_s
  {
    uint32_t rev;
    lfs_size_t size;
    lfs_block_t tail[2];
  } d;
} lfs_dir_t;

typedef struct lfs_superblock_s
{
  lfs_off_t off;

  struct lfs_disk_superblock_s
  {
    uint8_t type;
    uint8_t elen;
    uint8_t alen;
    uint8_t nlen;
    lfs_block_t root[2];
    uint32_t block_size;
    uint32_t block_count;
    uint32_t version;
    char magic[8];
  } d;
} lfs_superblock_t;

typedef struct lfs_free_s
{
  lfs_block_t off;
  lfs_block_t size;
  lfs_block_t i;
  lfs_block_t ack;
  FAR uint32_t *buffer;
} lfs_free_t;

/* The littlefs type */

typedef struct lfs_s
{
  FAR const struct lfs_config_s *cfg;

  lfs_block_t root[2];
  FAR lfs_file_t *files;
  lfs_dir_t *dirs;

  lfs_cache_t rcache;
  lfs_cache_t pcache;

  lfs_free_t free;
  bool deorphaned;
  bool moving;
} lfs_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Filesystem functions */

/* Format a block device with the littlefs
 *
 * Requires a littlefs object and config struct. This clobbers the littlefs
 * object, and does not leave the filesystem mounted. The config struct must
 * be zeroed for defaults and backwards compatibility.
 *
 * Returns a negative error code on failure.
 */

int lfs_format(FAR lfs_t *lfs, FAR const struct lfs_config_s *config);

/* Mounts a littlefs
 *
 * Requires a littlefs object and config struct. Multiple filesystems
 * may be mounted simultaneously with multiple littlefs objects. Both
 * lfs and config must be allocated while mounted. The config struct must
 * be zeroed for defaults and backwards compatibility.
 *
 * Returns a negative error code on failure.
 */

int lfs_mount(FAR lfs_t *lfs, FAR const struct lfs_config_s *config);

/* Unmounts a littlefs
 *
 * Does nothing besides releasing any allocated resources.
 * Returns a negative error code on failure.
 */

int lfs_unmount(FAR lfs_t *lfs);

/* General operations */

/* Removes a file or directory
 *
 * If removing a directory, the directory must be empty.
 * Returns a negative error code on failure.
 */

int lfs_remove(FAR lfs_t *lfs, FAR const char *path);

/* Rename or move a file or directory
 *
 * If the destination exists, it must match the source in type.
 * If the destination is a directory, the directory must be empty.
 *
 * Returns a negative error code on failure.
 */

int lfs_rename(FAR lfs_t *lfs, FAR const char *oldpath, FAR
               const char *newpath);

/* Find info about a file or directory
 *
 * Fills out the info structure, based on the specified file or directory.
 * Returns a negative error code on failure.
 */

int lfs_stat(FAR lfs_t *lfs, FAR const char *path,
             FAR struct lfs_info_s *info);

/* File operations */

/* Open a file
 *
 * The mode that the file is opened in is determined by the flags, which
 * are values from the enum lfs_open_flags that are bitwise-ored together.
 *
 * Returns a negative error code on failure.
 */

int lfs_file_open(FAR lfs_t *lfs, FAR lfs_file_t *file,
                  FAR const char *path, int flags);

/* Open a file with extra configuration
 *
 * The mode that the file is opened in is determined by the flags, which
 * are values from the enum lfs_open_flags that are bitwise-ored together.
 *
 * The config struct provides additional config options per file as described
 * above. The config struct must be allocated while the file is open, and the
 * config struct must be zeroed for defaults and backwards compatibility.
 *
 * Returns a negative error code on failure.
 */

int lfs_file_opencfg(FAR lfs_t *lfs, FAR lfs_file_t *file,
                     FAR const char *path, int flags,
                     FAR const struct lfs_file_config_s *config);

/* Close a file
 *
 * Any pending writes are written out to storage as though
 * sync had been called and releases any allocated resources.
 *
 * Returns a negative error code on failure.
 */

int lfs_file_close(FAR lfs_t *lfs, FAR lfs_file_t *file);

/* Synchronize a file on storage
 *
 * Any pending writes are written out to storage.
 * Returns a negative error code on failure.
 */

int lfs_file_sync(FAR lfs_t *lfs, FAR lfs_file_t *file);

/* Read data from file
 *
 * Takes a buffer and size indicating where to store the read data.
 * Returns the number of bytes read, or a negative error code on failure.
 */

lfs_ssize_t lfs_file_read(FAR lfs_t *lfs, FAR lfs_file_t *file,
                          FAR void *buffer, lfs_size_t size);

/* Write data to file
 *
 * Takes a buffer and size indicating the data to write. The file will not
 * actually be updated on the storage until either sync or close is called.
 *
 * Returns the number of bytes written, or a negative error code on failure.
 */

lfs_ssize_t lfs_file_write(FAR lfs_t *lfs, FAR lfs_file_t *file,
                           FAR const void *buffer, lfs_size_t size);

/* Change the position of the file
 *
 * The change in position is determined by the offset and whence flag.
 * Returns the old position of the file, or a negative error code on failure.
 */

lfs_soff_t lfs_file_seek(FAR lfs_t *lfs, FAR lfs_file_t *file,
                         lfs_soff_t off, int whence);

/* Truncates the size of the file to the specified size
 *
 * Returns a negative error code on failure.
 */

int lfs_file_truncate(FAR FAR lfs_t *lfs, FAR lfs_file_t *file, lfs_off_t size);

/* Return the position of the file
 *
 * Equivalent to lfs_file_seek(lfs, file, 0, LFS_SEEK_CUR)
 * Returns the position of the file, or a negative error code on failure.
 */

lfs_soff_t lfs_file_tell(FAR lfs_t *lfs, FAR lfs_file_t *file);

/* Change the position of the file to the beginning of the file
 *
 * Equivalent to lfs_file_seek(lfs, file, 0, LFS_SEEK_CUR)
 * Returns a negative error code on failure.
 */

int lfs_file_rewind(FAR lfs_t *lfs, FAR lfs_file_t *file);

/* Return the size of the file
 *
 * Similar to lfs_file_seek(lfs, file, 0, LFS_SEEK_END)
 * Returns the size of the file, or a negative error code on failure.
 */

lfs_soff_t lfs_file_size(FAR lfs_t *lfs, FAR lfs_file_t *file);

/* Directory operations */

/* Create a directory
 *
 * Returns a negative error code on failure.
 */

int lfs_mkdir(FAR lfs_t *lfs, FAR const char *path);

/* Open a directory
 *
 * Once open a directory can be used with read to iterate over files.
 * Returns a negative error code on failure.
 */

int lfs_dir_open(FAR lfs_t *lfs, FAR lfs_dir_t *dir, FAR const char *path);

/* Close a directory
 *
 * Releases any allocated resources.
 * Returns a negative error code on failure.
 */

int lfs_dir_close(FAR lfs_t *lfs, FAR lfs_dir_t *dir);

/* Read an entry in the directory
 *
 * Fills out the info structure, based on the specified file or directory.
 * Returns a negative error code on failure.
 */

int lfs_dir_read(FAR lfs_t *lfs, FAR lfs_dir_t *dir,
                 FAR struct lfs_info_s *info);

/* Change the position of the directory
 *
 * The new off must be a value previous returned from tell and specifies
 * an absolute offset in the directory seek.
 *
 * Returns a negative error code on failure.
 */

int lfs_dir_seek(FAR lfs_t *lfs, FAR lfs_dir_t *dir, lfs_off_t off);

/* Return the position of the directory
 *
 * The returned offset is only meant to be consumed by seek and may not make
 * sense, but does indicate the current position in the directory iteration.
 *
 * Returns the position of the directory, or a negative error code on failure.
 */

lfs_soff_t lfs_dir_tell(FAR lfs_t *lfs, FAR lfs_dir_t *dir);

/* Change the position of the directory to the beginning of the directory
 *
 * Returns a negative error code on failure.
 */

int lfs_dir_rewind(FAR lfs_t *lfs, FAR lfs_dir_t *dir);

/* Miscellaneous littlefs specific operations */

/* Traverse through all blocks in use by the filesystem
 *
 * The provided callback will be called with each block address that is
 * currently in use by the filesystem. This can be used to determine which
 * blocks are in use or how much of the storage is available.
 *
 * Returns a negative error code on failure.
 */

int lfs_traverse(FAR lfs_t *lfs, CODE int (*cb)(FAR void *, lfs_block_t),
                 FAR void *data);

/* Prunes any recoverable errors that may have occured in the filesystem
 *
 * Not needed to be called by user unless an operation is interrupted
 * but the filesystem is still mounted. This is already called on first
 * allocation.
 *
 * Returns a negative error code on failure.
 */

int lfs_deorphan(FAR lfs_t *lfs);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __FS_LITTLEFS_LFS_H */
