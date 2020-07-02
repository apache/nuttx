/****************************************************************************
 * fs/cromfs/cromfs.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __FS_CROMFS_CROMFS_H
#define __FS_CROMFS_CROMFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Maximum size of an offset.  This should normally be size_t since this is
 * an in-memory file system.  However, uint32_t is 32-bits on most 32-bit
 * target machines but 64-bits on 64-host machines.  We restrict offsets to
 * 32-bits for commonality (limiting the size of the CROMFS image to 4Gb).
 *
 * REVISIT: What about small memory systems where the size_t is only 16-bits?
 *
 * Similarly, the NuttX mode_t is only 16-bits so uint16_t is explicitly used
 * for NuttX file modes.
 */

/* This structure describes the CROMFS volume.  It provides most of the
 * information needed for statfs() including:
 *
 *   f_type     - Type of filesystem
 *                Return cv_magic (CROMFS_MAGIC)
 *   f_namelen  - Maximum length of filenames
 *                Return NAME_MAX
 *   f_bsize    - Optimal block size for transfers
 *                Return cv_bsize, the block size used when file system was
 *                compressed
 *   f_blocks   - Total data blocks in the file system
 *              - Return cv_nblocks
 *   f_bfree    - Free blocks in the file system
 *                Return 0
 *   f_bavail   - Free blocks avail to non-superuser
 *                Return 0
 *   f_files    - Total file nodes in the file system
 *              - Return cv_nnodes
 *   f_ffree    - Free file nodes in the file system
 *                Return 0
 *
 * The volume header is followed immediately by the root directory node.  An
 * offset to that node is used to permit future variable length data (such as
 * a volume name) which may intervene.
 */

struct cromfs_volume_s
{
  uint32_t cv_magic;     /* Must be first.  Must be CROMFS_MAGIC */
  uint16_t cv_nnodes;    /* Total number of nodes in-use */
  uint16_t cv_nblocks;   /* Total number of data blocks in-use */
  uint32_t cv_root;      /* Offset to the first node in the root file system */
  uint32_t cv_fsize;     /* Size of the compressed file system image */
  uint32_t cv_bsize;     /* Optimal block size for transfers */
};

/* This describes one node in the CROMFS file system. It holds node meta data
 * that provides the information that will be return by stat() or fstat()
 * and also provides the information needed by the CROMFS file system to
 * access the node data.
 *
 * Relationship to struct stat:
 *
 *   st_mode    - File type, attributes, and access mode bits
 *                Return cn_mode from the node structure
 *   st_size    - Size of file/directory, in bytes
 *                Return cn_size from the node structure
 *   st_blksize - Block size used for filesystem I/O
 *                Return cv_bsize from the volume header.
 *   st_blocks  - Number of blocks allocated
 *                Return (cn_size + (cv_bsize - 1)) / cv_bsize
 *   st_atime   - Time of last access
 *                Return 0
 *   st_mtime   - Time of last modification
 *                Return 0
 *   st_ctime   - Time of last status change
 *                Return 0
 */

struct cromfs_node_s
{
  uint16_t cn_mode;      /* File type, attributes, and access mode bits */
  uint16_t cn_pad;       /* Not used */
  uint32_t cn_name;      /* Offset from the beginning of the volume header to the
                          * node name string.  NUL-terminated. */
  uint32_t cn_size;      /* Size of the uncompressed data (in bytes) */
  uint32_t cn_peer;      /* Offset to next node in this directory (for readdir()) */
  union
  {
    uint32_t cn_child;   /* Offset to first node in sub-directory (directories only) */
    uint32_t cn_link;    /* Offset to an arbitrary node (for hard link) */
    uint32_t cn_blocks;  /* Offset to first block of compressed data (for read) */
  } u;
};

#endif /* __FS_CROMFS_CROMFS_H */
