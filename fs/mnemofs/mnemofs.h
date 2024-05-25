/****************************************************************************
 * fs/mnemofs/mnemofs.h
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

#ifndef __FS_MNEMOFS_MNEMOFS_H
#define __FS_MNEMOFS_MNEMOFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/fs/fs.h>
#include <nuttx/list.h>
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MFS_JRNL_MAGIC  "-mfs!j!-"

#define MFS_BLK2PG(sb, blk)     ((blk) << (sb)->log_pg_in_blk)
#define MFS_PG2BLK(sb, pg)      ((pg) >> (sb)->log_pg_in_blk)
#define MFS_PG2BLKPGOFF(sb, pg) ((pg) % (1 << (sb)->log_pg_in_blk))

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef uint32_t  mfs_t;

struct mfs_ctz_store_s
{
  mfs_t pg_e;
  mfs_t idx_e;
};

struct mfs_jrnl_info
{
  struct list_node            list;
  mfs_t                       depth;
  FAR struct mfs_ctz_store_s *path;
  struct mfs_ctz_store_s      new;
};

struct mfs_jrnl_state
{
  uint8_t     n_blks; /* Does not consider master blocks. */
  FAR mfs_t   *idxarr;
  mfs_t       s_off; /* Writeable area start offset */
  mfs_t       w_off; /* Read pointer */
  mfs_t       r_off; /* Write pointer */
};

struct mfs_sb_info
{
  mutex_t               fs_lock;
  FAR struct inode      *drv;
  struct mtd_geometry_s geo;
  mfs_t                 pg_sz;         /* In bytes */
  mfs_t                 blk_sz;        /* In bytes */
  uint8_t               log_blk_sz;
  mfs_t                 n_blks;
  uint8_t               log_pg_sz;
  uint16_t              pg_in_blk;
  uint8_t               log_pg_in_blk;
  uint8_t               j_nblks;
  mfs_t                 master_node;
  struct mfs_jrnl_state j_state; /* Journal State */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mfs_blkremsz
 *
 * Description:
 *   Given a page and a page offset, it returns the bytes left in the entire
 *   block after the offset location.
 *
 * Input Parameters:
 *   sb    - Superblock instance of the device.
 *   pg    - Page number.
 *   pgoff - Page offset.
 *
 * Returned Value:
 *   Bytes left in the block.
 *
 ****************************************************************************/

static mfs_t inline mfs_blkremsz(FAR const struct mfs_sb_info * const sb,
                                  mfs_t pg, mfs_t pgoff)
{
  return sb->blk_sz - (MFS_PG2BLKPGOFF(sb, pg) * sb->pg_sz + pgoff);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* mnemofs_journal.c */

/****************************************************************************
 * Name: mfs_jrnl_fmt
 *
 * Description:
 *   Formats a journal into a NAND flash device.
 *
 * Input Parameters:
 *   sb - Superblock instance of the device.
 *
 * Returned Value:
 *   0        - OK
 *   -ENOMEM  - No memory left
 *
 * Assumptions/Limitations:
 *   This is used as part of the whole format process for the device.
 *
 ****************************************************************************/

int mfs_jrnl_fmt(FAR struct mfs_sb_info * const sb);

/****************************************************************************
 * Name: mfs_jrnl_init
 *
 * Description:
 *   Reads a journal from the NAND device and initializes it's in-memory
 *   metadata.
 *
 * Input Parameters:
 *   sb     - Superblock instance of the device.
 *   blk    - The block number of the very first block of the journal.
 *   m_node - Populates it with the most recently written master node.
 *
 * Returned Value:
 *   0        - OK
 *   -EINVAL  - Invalid first block
 *   -ENOMEM  - No memory left
 *
 * Assumptions/Limitations:
 *   This is used as part of the whole format process for the device.
 *
 ****************************************************************************/

int mfs_jrnl_init(FAR struct mfs_sb_info * const sb, mfs_t blk,
                  mfs_t *m_node);

/****************************************************************************
 * Name: mfs_jrnl_init
 *
 * Description:
 *   Reset the state of the journal's read pointer.
 *
 * Input Parameters:
 *   sb - Superblock instance of the device.
 *
 * Assumptions/Limitations:
 *   This assumes all read traversals over the journal is done in from start
 *   to the end, and this is called at the start of every such traversal.
 *
 ****************************************************************************/

void mfs_jrnl_statereset(FAR struct mfs_sb_info * const sb);

/****************************************************************************
 * Name: mfs_jrnl_radv
 *
 * Description:
 *   Read and advance the read pointer of the journal by one log.
 *
 * Input Parameters:
 *   sb   - Superblock instance of the device.
 *   info - If not null, it will be populated by the log that is read.
 *
 * Returned Value:
 *   0        - OK
 *   -ENOMEM  - No memory left
 *   -EINVAL  - Invalid current state.
 *
 * Assumptions/Limitations:
 *   This assumes all read traversals over the journal is done in from start
 *   to the end, and this is called at the start of every such traversal.
 *
 ****************************************************************************/

int mfs_jrnl_radv(FAR struct mfs_sb_info * const sb,
                  FAR struct mfs_jrnl_info * info);

/****************************************************************************
 * Name: mfs_jrnl_wadv
 *
 * Description:
 *   Write log at current journal state and advance the write pointer of the
 *   journal by one log.
 *
 * Input Parameters:
 *   sb   - Superblock instance of the device.
 *   info - Log to be written. Needs to be non-NULL.
 *
 * Returned Value:
 *   0        - OK
 *   -ENOMEM  - No memory left
 *   -EINVAL  - Invalid current state.
 *
 ****************************************************************************/

int mfs_jrnl_wadv(FAR struct mfs_sb_info * const sb,
                  FAR const struct mfs_jrnl_info * const info);

/****************************************************************************
 * Name: mfs_jrnl_nwadv
 *
 * Description:
 *   Write n logs at current journal state and advance the write pointer of
 *   the journal by n logs.
 *
 * Input Parameters:
 *   sb   - Superblock instance of the device.
 *   list - List of logs to be written. The value of n is the length of this
 *          list.
 *
 * Returned Value:
 *   0        - OK
 *   -ENOMEM  - No memory left
 *   -EINVAL  - Invalid current state.
 *
 ****************************************************************************/

int mfs_jrnl_nwadv(FAR struct mfs_sb_info * const sb,
                  FAR struct list_node list);

/****************************************************************************
 * Name: mfs_jrml_updatectz
 *
 * Description:
 *   Given a CTZ list from on-flash location, this traverses through the
 *   entire journal and finally provided the updated position of the list,
 *   which is yet to be commited (hence in the journal).
 *
 * Input Parameters:
 *   sb    - Superblock instance of the device.
 *   path  - CTZ array representation of the path.
 *   depth - The depth of the fs object in the path.
 *
 ****************************************************************************/

void mfs_jrml_updatectz(FAR struct mfs_sb_info * const sb,
                        FAR struct mfs_ctz_store_s * const path,
                        const mfs_t depth);

/* mnemofs_blkalloc.c */

/****************************************************************************
 * Name: mfs_get_blk
 *
 * Description:
 *   Returns a block location on request.
 *
 * Input Parameters:
 *   sb - Superblock instance of the device.
 *
 * Returned Value:
 *   0        - OK
 *   -ENOMEM  - No memory left
 *
 * Assumptions/Limitations:
 *   This assumes that the entire block is as good as being completely
 *   written to by the caller. No pages inside this block will be allocated
 *   to anybody else till this block is not erased.
 *
 ****************************************************************************/

mfs_t mfs_get_blk(FAR struct mfs_sb_info * const sb);

/* mnemofs_rw.c */

/****************************************************************************
 * Name: mfs_write_data
 *
 * Description:
 *   Writes an array starting from a particular page and its page offset.
 *
 * Input Parameters:
 *   sb       - Superblock instance of the device.
 *   data     - Array.
 *   datalen  - Length of the array.
 *   pg       - Page number to start writing.
 *   pgoff    - Offset inside page to start writing from.
 *
 * Returned Value:
 *   Number of bytes written.
 *
 * Assumptions/Limitations:
 *   If the remaining size of the page is not enough to hold the entire data,
 *   it will write it in the next page, and so on.
 *
 ****************************************************************************/

ssize_t mfs_write_data(FAR const struct mfs_sb_info * const sb,
                      char *data, uint64_t datalen, uint32_t page,
                      uint8_t pgoff);

/****************************************************************************
 * Name: mfs_read_data
 *
 * Description:
 *   Reads an array starting from a particular page and its page offset.
 *
 * Input Parameters:
 *   sb       - Superblock instance of the device.
 *   data     - Array.
 *   datalen  - Length of the array.
 *   pg       - Page number to start reading.
 *   pgoff    - Offset inside page to start reading from.
 *
 * Returned Value:
 *   Number of bytes read.
 *
 * Assumptions/Limitations:
 *   If the remaining size of the page is not enough to read the entire data,
 *   it will read from the next page as well, and so on.
 *
 ****************************************************************************/

ssize_t mfs_read_data(FAR const struct mfs_sb_info * const sb,
                      char *data, uint64_t datalen, uint32_t pg,
                      uint8_t pgoff);

/****************************************************************************
 * Name: mfs_read_page
 *
 * Description:
 *   Reads an array starting from a particular page and its page offset to
 *   either the end of the array or the end of the page, whichever occurs
 *   first.
 *
 * Input Parameters:
 *   sb       - Superblock instance of the device.
 *   data     - Array.
 *   data     - Length of the array..
 *   pg       - Page number to start reading.
 *   pgoff    - Offset inside page to start reading from.
 *
 * Returned Value:
 *   Number of bytes read.
 *
 ****************************************************************************/

ssize_t mfs_read_page(FAR const struct mfs_sb_info * const sb,
                      char *data, uint64_t datalen, uint32_t pg,
                      uint8_t pgoff);

/* mnemofs_util.c */

/****************************************************************************
 * Name: mfs_arrhash
 *
 * Description:
 *   Returns an 8-bit hash of an entire array of data.
 *
 * Input Parameters:
 *   arr - Data array.
 *   len - Length of the array.
 *
 * Returned Value:
 *   16-bit hash of the array.
 *
 ****************************************************************************/

char mfs_arrhash(FAR const char *arr, ssize_t len);

/****************************************************************************
 * Name: mfs_ser_8
 *
 * Description:
 *   Serialize a 8 bit type into output.
 *
 * Input Parameters:
 *   n   - 8 bit to serialize
 *   out - Output array where to serialize.
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

char *mfs_ser_8(const char n, char * const out);

/****************************************************************************
 * Name: mfs_deser_8
 *
 * Description:
 *   Deserialize a 8 bit type from input.
 *
 * Input Parameters:
 *   in - Input array from where to deserialize.
 *   n  - 8 bit to deserialize
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

const char *mfs_deser_8(const char * const in, char *n);

/****************************************************************************
 * Name: mfs_ser_str
 *
 * Description:
 *   Serialize a string into output.
 *
 * Input Parameters:
 *   str - String to serialize
 *   len - Length of string
 *   out - Output array where to serialize.
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

char *mfs_ser_str(const char * const str, const mfs_t len, char * const out);

/****************************************************************************
 * Name: mfs_deser_str
 *
 * Description:
 *   Deserialize a string from intput.
 *
 * Input Parameters:
 *   in  - Intput array from where to deserialize.
 *   str - String to deserialize
 *   len - Length of string
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

const char *mfs_deser_str(const char * const in, char * const str,
                          const mfs_t len);

/****************************************************************************
 * Name: mfs_ser_mfs
 *
 * Description:
 *   Serialize a mfs_t type into output.
 *
 * Input Parameters:
 *   n   - mfs_t to serialize
 *   out - Output array where to serialize.
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

char *mfs_ser_mfs(const mfs_t n, char * const out);

/****************************************************************************
 * Name: mfs_deser_mfs
 *
 * Description:
 *   Deserialize a mfs_t type from input..
 *
 * Input Parameters:
 *   in - Input array from where to deserialize.
 *   n  - mfs_t to deserialize
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

const char *mfs_deser_mfs(const char * const in, mfs_t * const n);

/****************************************************************************
 * Name: mfs_ser_ctz_store
 *
 * Description:
 *   Serialize a mfs_ctz_store_s type into output.
 *
 * Input Parameters:
 *   x   - mfs_ctz_store_s to serialize
 *   out - Output array where to serialize.
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

char *mfs_ser_ctz_store(const struct mfs_ctz_store_s * const x,
                        char * const out);

/****************************************************************************
 * Name: mfs_deser_ctz_store
 *
 * Description:
 *   Deserialize a mfs_ctz_store_s type into output.
 *
 * Input Parameters:
 *   in - Input array from where to deserialize.
 *   x  - mfs_ctz_store_s to deserialize
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

const char *mfs_deser_ctz_store(const char * const in,
                                struct mfs_ctz_store_s * const x);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __FS_MNEMOFS_MNEMOFS_H */