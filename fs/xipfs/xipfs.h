/****************************************************************************
 * fs/xipfs/xipfs.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __FS_XIPFS_XIPFS_H
#define __FS_XIPFS_XIPFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/statfs.h>
#include <sys/types.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/compiler.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/xipfs.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mutex.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* On-media identification */

#define XIPFS_SBLK_MAGIC    XIPFS_MAGIC /* "XIPF", see sys/statfs.h */
#define XIPFS_GEN_MAGIC     0x5847454e  /* "XGEN"                    */
#define XIPFS_VERSION       2

/* Volume geometry.
 *
 * Block 0 and block 1 hold the two superblock copies.  The next
 * XIPFS_META_NBLOCKS blocks form the metadata ring.  Everything after that
 * is the data region, allocated in whole erase blocks.
 */

#define XIPFS_SBLK_A        0
#define XIPFS_SBLK_B        1
#define XIPFS_META_START    2
#define XIPFS_META_NBLOCKS  4
#define XIPFS_DATA_START    (XIPFS_META_START + XIPFS_META_NBLOCKS)

/* The smallest volume that can hold the superblocks, the metadata ring and
 * at least one data block.
 */

#define XIPFS_MIN_BLOCKS    (XIPFS_DATA_START + 1)

/* Directory entry size */

#define XIPFS_DIRENT_SIZE   64

/* Dirent flags */

#define XIPFS_DIRENT_DIR    (1 << 0)  /* The entry is a directory */

/* The root directory is implicit: it owns id 0 and has no record of its
 * own, so it always exists and cannot be removed.
 */

#define XIPFS_ROOT_ID       0

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* On-media superblock.  Written once by mkfs into both block 0 and block 1.
 * The CRC covers every byte preceding the crc field.
 */

begin_packed_struct struct xipfs_sblk_s
{
  uint32_t magic;
  uint32_t version;
  uint32_t erasesize;
  uint32_t blocksize;
  uint32_t nblocks;         /* Total erase blocks in the volume */
  uint32_t meta_start;      /* First block of the metadata ring */
  uint32_t meta_nblocks;    /* Length of the metadata ring      */
  uint32_t data_start;      /* First block of the data region   */
  uint32_t data_nblocks;    /* Length of the data region        */
  uint32_t crc;
} end_packed_struct;

/* On-media directory entry.  One per file and one per directory, stored in
 * the body of a metadata generation.
 *
 * Directories are records here and nowhere else.  They are not objects in
 * the data region, which is what keeps the power-safety story intact: mkdir
 * and rmdir add or remove a record and commit one generation, exactly as
 * create and unlink do, so there is never a multi-object update to journal
 * or an orphan to collect at mount.
 *
 * 'name' is one path component, not a path.  Depth comes from 'parent', so
 * XIPFS_NAME_MAX bounds a component, which is what statfs reports it as.
 *
 * Padded to XIPFS_DIRENT_SIZE so that a whole number of dirents fits in a
 * read/write block, which keeps the body write loop aligned without any
 * straddling entries.
 */

begin_packed_struct struct xipfs_dirent_s
{
  char     name[XIPFS_NAME_MAX + 1];  /* 32 bytes, one component          */
  uint32_t size;                      /* File size in bytes; 0 for a dir  */
  uint32_t start_block;               /* Extent block; 0 for a directory  */
  uint32_t nblocks;                   /* Extent length; 0 for a directory */
  uint32_t flags;                     /* XIPFS_DIRENT_DIR                 */
  uint16_t id;                        /* Identity, unique, never 0        */
  uint16_t parent;                    /* Containing directory's id        */
  uint8_t  reserved[12];              /* Pad to XIPFS_DIRENT_SIZE         */
} end_packed_struct;

/* The padding above is maintained by hand, and the body read loop indexes
 * the generation at a fixed XIPFS_DIRENT_SIZE stride.  A field added without
 * taking it out of 'reserved' would therefore not fail loudly: it would
 * reparse every existing volume at the wrong offset.
 */

static_assert(sizeof(struct xipfs_dirent_s) == XIPFS_DIRENT_SIZE,
              "xipfs dirent layout mismatch");

/* On-media metadata generation header.
 *
 * This lives in the first read/write block of a metadata ring block; the
 * dirent array follows in the blocks after it.  The body is written first
 * and the header last, so the single header write IS the commit point: a
 * torn body leaves no valid header, and a torn header fails its own CRC.
 * Either way mount falls back to the previous generation.
 */

begin_packed_struct struct xipfs_genhdr_s
{
  uint32_t magic;
  uint32_t seq;             /* Monotonic generation number */
  uint32_t nentries;        /* Number of dirents in body   */
  uint32_t crc;             /* CRC over header + body      */
} end_packed_struct;

/* In-RAM directory entry.  One per file and one per directory; a directory
 * carries no extent, so its start_block, nblocks and size are all zero and
 * everything that accounts for blocks has to skip it.
 *
 * For a file this is also the object the pin count lives on -- deliberately
 * NOT the fd and NOT the open file struct, so that N mappings of one module
 * produce a pin count of N and the extent only becomes movable when the
 * last one goes away.
 */

struct xipfs_extent_s
{
  FAR struct xipfs_extent_s *flink;

  /* Back pointer to the owning mount.  The task teardown unmap path can
   * only reach the extent, and must not consult the current task's group,
   * so the extent has to be able to name its own filesystem.
   */

  FAR struct xipfs_mount_s *fs;

  /* One path component, not a path; depth comes from 'parent' */

  char     name[XIPFS_NAME_MAX + 1];

  uint16_t id;              /* Identity, unique, never 0              */
  uint16_t parent;          /* Containing directory; XIPFS_ROOT_ID    */
  uint32_t size;
  uint32_t start_block;
  uint32_t nblocks;
  uint32_t pincount;        /* Mappings that alias flash (true XIP)   */
  uint32_t openrefs;        /* Open file descriptors                  */
  bool     isdir;           /* A directory record, with no extent     */
  bool     writing;         /* Create-time write still in progress    */
  bool     unlinked;        /* Detached from directory, awaiting free */
};

/* In-RAM mount state */

struct xipfs_mount_s
{
  FAR struct inode      *driver;
  FAR struct mtd_dev_s  *mtd;
  struct mtd_geometry_s  geo;

  /* Direct address of the media, from BIOC_XIPBASE.  NULL when the
   * underlying driver cannot expose one, in which case XIP mappings are
   * impossible and strict requests fail with -ENXIO.
   */

  FAR uint8_t *xipbase;

  uint32_t meta_start;
  uint32_t meta_nblocks;
  uint32_t meta_slot;       /* Ring slot holding the live generation */
  uint32_t meta_seq;        /* Sequence number of that generation    */
  uint32_t data_start;
  uint32_t data_nblocks;

  FAR unsigned long *bitmap; /* Free bitmap over the data region      */
  size_t   bitmapsize;

  /* Staging buffer for defrag relocation.  Reserved once at bind time so
   * that a relocation can never fail part way through because a transient
   * allocation did not succeed.
   */

  FAR uint8_t *stage;

  /* Scratch for building and verifying a metadata generation.  Separate
   * from 'stage' because defrag commits a generation while its relocation
   * copy is in flight.
   */

  FAR uint8_t *metabuf;

  uint32_t entries_per_blk; /* Dirents in one read/write block */
  uint32_t max_entries;     /* Dirents that fit in one ring block */

  FAR struct xipfs_extent_s *extents;
  uint32_t nextents;

  /* Single guard over extent metadata, the allocator and the pin counts.
   * The map path holds it across "range check -> resolve -> take pin" and
   * defrag holds it across "observe pincount == 0 -> relocate", so a new
   * mapping cannot slip in between the compactor's check and its move.
   */

  rmutex_t lock;

  bool     unmounted;

#ifdef CONFIG_FS_XIPFS_FAULT_INJECT
  bool     media_dead;
  int32_t  fault_countdown; /* Negative disables injection */
  uint8_t  fault_mode;      /* XIPFS_FAULT_CLEAN or XIPFS_FAULT_TORN */
#endif
};

/* Per open file state, hung off filep->f_priv */

struct xipfs_file_s
{
  FAR struct xipfs_extent_s *ext;

  /* Create-time write state.  NOR programs whole pages, so bytes are
   * accumulated here and flushed a page at a time.
   */

  FAR uint8_t *wrbuf;
  uint32_t wrpos;           /* Bytes buffered but not yet programmed  */
  uint32_t written;         /* Bytes already programmed to flash      */
  uint32_t erased;          /* Blocks of the extent already erased    */
  bool     writing;
  bool     greedy;          /* Extent was over-reserved, trim at close */
};

/* Directory enumeration state */

/* An open directory: which directory it is, and how far the walk has got */

struct xipfs_dir_s
{
  struct fs_dirent_s base;
  uint16_t dirid;
  uint32_t index;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN const struct mountpt_operations g_xipfs_operations;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* xipfs_flash.c ************************************************************/

int  xipfs_flash_probe(FAR struct xipfs_mount_s *fs);
int  xipfs_flash_read(FAR struct xipfs_mount_s *fs, uint32_t block,
                      uint32_t offset, FAR void *buffer, size_t nbytes);
int  xipfs_flash_write(FAR struct xipfs_mount_s *fs, uint32_t block,
                       uint32_t offset, FAR const void *buffer,
                       size_t nbytes);
int  xipfs_flash_erase(FAR struct xipfs_mount_s *fs, uint32_t block,
                       uint32_t nblocks);

/* Direct address of a block, or NULL when the media is not memory mapped */

FAR uint8_t *xipfs_flash_addr(FAR struct xipfs_mount_s *fs, uint32_t block);

/* xipfs_meta.c *************************************************************/

int  xipfs_meta_format(FAR struct xipfs_mount_s *fs);
int  xipfs_meta_mount(FAR struct xipfs_mount_s *fs);
int  xipfs_meta_commit(FAR struct xipfs_mount_s *fs);
void xipfs_meta_freeextents(FAR struct xipfs_mount_s *fs);

FAR struct xipfs_extent_s *xipfs_meta_find(FAR struct xipfs_mount_s *fs,
                                           uint16_t parent,
                                           FAR const char *name,
                                           size_t namelen);
FAR struct xipfs_extent_s *xipfs_meta_add(FAR struct xipfs_mount_s *fs,
                                          uint16_t parent,
                                          FAR const char *name,
                                          size_t namelen, bool isdir,
                                          uint32_t start_block,
                                          uint32_t nblocks, uint32_t size);
bool xipfs_meta_haschildren(FAR struct xipfs_mount_s *fs, uint16_t dirid);
void xipfs_meta_path(FAR struct xipfs_mount_s *fs,
                     FAR struct xipfs_extent_s *ext,
                     FAR char *buf, size_t buflen);
void xipfs_meta_detach(FAR struct xipfs_mount_s *fs,
                       FAR struct xipfs_extent_s *ext);

/* xipfs_alloc.c ************************************************************/

int      xipfs_alloc_init(FAR struct xipfs_mount_s *fs);
void     xipfs_alloc_rebuild(FAR struct xipfs_mount_s *fs);
int      xipfs_alloc(FAR struct xipfs_mount_s *fs, uint32_t nblocks,
                     FAR uint32_t *start_block);
void     xipfs_free(FAR struct xipfs_mount_s *fs, uint32_t start_block,
                    uint32_t nblocks);
uint32_t xipfs_alloc_largestrun(FAR struct xipfs_mount_s *fs);
uint32_t xipfs_alloc_freeblocks(FAR struct xipfs_mount_s *fs);

/* xipfs_mmap.c *************************************************************/

int  xipfs_mmap(FAR struct file *filep, FAR struct mm_map_entry_s *map);

/* xipfs_defrag.c ***********************************************************/

int  xipfs_defrag(FAR struct xipfs_mount_s *fs, uint32_t max_ms,
                  FAR struct xipfs_defrag_result_s *result);
int  xipfs_listpinned(FAR struct xipfs_mount_s *fs,
                      FAR struct xipfs_pinned_entry_s *entries,
                      size_t nentries, FAR size_t *count);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline int xipfs_lock(FAR struct xipfs_mount_s *fs)
{
  return nxrmutex_lock(&fs->lock);
}

static inline void xipfs_unlock(FAR struct xipfs_mount_s *fs)
{
  nxrmutex_unlock(&fs->lock);
}

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __FS_XIPFS_XIPFS_H */
