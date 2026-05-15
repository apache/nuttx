/****************************************************************************
 * fs/mnemofs/mnemofs.h
 *
 * SPDX-License-Identifier: Apache-2.0 or BSD-3-Clause
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
 * Alternatively, the contents of this file may be used under the terms of
 * the BSD-3-Clause license:
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2024 Saurav Pal
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __FS_MNEMOFS_MNEMOFS_H
#define __FS_MNEMOFS_MNEMOFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <dirent.h>
#include <debug.h>
#include <nuttx/compiler.h>
#include <nuttx/fs/fs.h>
#include <nuttx/list.h>
#include <nuttx/mutex.h>
#include <nuttx/mtd/mtd.h>

#include <stdbool.h>
#include <stdint.h>
#include <sys/stat.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

typedef uint32_t mfs_t;
typedef uint64_t mfs_time_t;

#define MFS_DIRENT_MAGIC 131
#define MFS_DIR_MAGIC 197
#define MFS_NAME_MAX 30
#define MFS_NAME_LEN (MFS_NAME_MAX + 1)
#define MFS_LOCATION_INVALID 0
#define MFS_DIRENT_OFF_INVALID UINT32_MAX
#define MFS_FORMAT_VERSION_MAJOR 1

#define MFS_PAGE_SIZE(sb) ((sb)->geo.blocksize)
#define MFS_BLOCK_SIZE(sb) ((sb)->geo.erasesize)
#define MFS_BLOCK_COUNT(sb) ((sb)->geo.neraseblocks)
#define MFS_PAGES_PER_BLOCK(sb) ((sb)->pagesperblk)
#define MFS_PAGE_COUNT(sb) \
  (MFS_BLOCK_COUNT(sb) * MFS_PAGES_PER_BLOCK(sb))
#define MFS_BLOCK_TO_PAGE(sb, blk) \
  ((blk) * MFS_PAGES_PER_BLOCK(sb))
#define MFS_DIRENTS_PER_PAGE(sb) \
  (MFS_PAGE_SIZE(sb) / sizeof(struct mfs_direntry_s))

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct mfs_direntry_s
{
  uint8_t    magic;
  uint16_t   mode;
  mfs_t      oldloc;
  mfs_t      newloc;
  uint16_t   namehash;
  char       name[MFS_NAME_MAX];
  mfs_time_t ctime;
  mfs_time_t mtime;
  union
  {
    uint32_t size;
    uint32_t reserved;
  } meta;
  uint8_t    checksum;
} end_packed_struct;

begin_packed_struct struct mfs_dirmeta_s
{
  uint8_t magic;
  mfs_t   parent;
  uint8_t checksum;
} end_packed_struct;

begin_packed_struct struct mfs_superblock_s
{
  uint32_t magic;
  uint32_t npages;
  uint32_t pagesize;
  uint32_t pagesperblock;
  uint32_t nblocks;
  mfs_t    rootdir;
  uint16_t version;
} end_packed_struct;

struct mfs_dirloc_s
{
  mfs_t page;
  mfs_t offset;
};

struct mfs_sb_s
{
  FAR struct inode      *driver;
  FAR struct mtd_dev_s  *mtd;
  struct mtd_geometry_s  geo;
  uint8_t                erasestate;
  FAR uint8_t           *rwbuf;
  mfs_t                  rwpage;
  bool                   rwvalid;
  bool                   rwdirty;
  FAR uint8_t           *freepages;
  FAR uint8_t           *delpages;
  size_t                 bitmapsize;
  mfs_t                  nextpage;
  mfs_t                  pagesperblk;
  mfs_t                  superblock;
  mfs_t                  rootdir;
  uint16_t               version;
  mutex_t                lock;
  struct list_node       ofiles;
};

struct mfs_dirfs_s
{
  struct fs_dirent_s     dir;
  mfs_t                  location;
  struct mfs_direntry_s  direntry;
  struct mfs_dirloc_s    nextloc;
};

struct mfs_ctz_s
{
  mfs_t page;
  mfs_t index;
};

struct mfs_ofd_common_s
{
  struct mfs_direntry_s direntry;
  struct mfs_dirloc_s   direntloc;
  FAR char             *relpath;
  mfs_t                 fileloc;
  off_t                 pos;
  unsigned int          refs;
};

struct mfs_ofd_s
{
  struct list_node         entry;
  FAR struct mfs_sb_s     *sb;
  FAR struct mfs_ofd_common_s *common;
};

static_assert(sizeof(struct mfs_direntry_s) == 64,
              "mnemofs direntry layout mismatch");
static_assert(sizeof(struct mfs_dirmeta_s) == 6,
              "mnemofs dirmeta layout mismatch");
static_assert(sizeof(struct mfs_superblock_s) == 26,
              "mnemofs superblock layout mismatch");

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int mfs_is_bad_block(FAR const struct mfs_sb_s *sb, mfs_t blk);
int mfs_rwbuf_sync(FAR struct mfs_sb_s *sb);
void mfs_rwbuf_invalidate(FAR struct mfs_sb_s *sb);
int mfs_rwbuf_prepare_write(FAR struct mfs_sb_s *sb);
void mfs_rwbuf_discard_page(FAR struct mfs_sb_s *sb, mfs_t page);
void mfs_rwbuf_discard_block(FAR struct mfs_sb_s *sb, mfs_t block);
ssize_t mfs_write_page(FAR struct mfs_sb_s *sb, mfs_t page,
                       FAR const uint8_t *buffer);
ssize_t mfs_read_page(FAR struct mfs_sb_s *sb, mfs_t page,
                      FAR uint8_t *buffer);
int mfs_erase_blocks(FAR struct mfs_sb_s *sb, mfs_t startblk,
                     size_t nblocks);
int mfs_update_rootdir(FAR struct mfs_sb_s *sb, mfs_t rootdir);

int mfs_alloc_init(FAR struct mfs_sb_s *sb);
void mfs_alloc_uninit(FAR struct mfs_sb_s *sb);
int mfs_alloc_page(FAR struct mfs_sb_s *sb, FAR mfs_t *page);
int mfs_alloc_block(FAR struct mfs_sb_s *sb, FAR mfs_t *block);
int mfs_release_page(FAR struct mfs_sb_s *sb, mfs_t page);
int mfs_report_page_deleted(FAR struct mfs_sb_s *sb, mfs_t page);
int mfs_report_block_deleted(FAR struct mfs_sb_s *sb, mfs_t block);

mfs_t mfs_ctz_unit_data_area(FAR struct mfs_sb_s *sb, mfs_t index);
int mfs_ctz_traverse(FAR struct mfs_sb_s *sb, mfs_t startidx,
                     mfs_t endidx, mfs_t startpage, FAR mfs_t *endpage);
int mfs_ctz_index_from_off(FAR struct mfs_sb_s *sb, mfs_t offset,
                           FAR mfs_t *index, FAR mfs_t *pageoff);
int mfs_ctz_fill_next_ptrs(FAR struct mfs_sb_s *sb, mfs_t page,
                           mfs_t index, FAR uint8_t *buffer);

mfs_t mfs_page_to_block(FAR const struct mfs_sb_s *sb, mfs_t page);

int mfs_dirent_traverse(FAR struct mfs_sb_s *sb, FAR const char *relpath,
                        FAR mfs_t *location,
                        FAR struct mfs_direntry_s *direntry,
                        FAR struct mfs_dirloc_s *dirloc);
int mfs_dir_create_file(FAR struct mfs_sb_s *sb, FAR const char *relpath,
                        mode_t mode, FAR mfs_t *location,
                        FAR struct mfs_direntry_s *direntry,
                        FAR struct mfs_dirloc_s *dirloc);
int mfs_dir_update_file(FAR struct mfs_sb_s *sb, mfs_t oldloc, mfs_t newloc,
                        FAR const struct mfs_dirloc_s *dirloc,
                        FAR struct mfs_direntry_s *direntry,
                        off_t length);
int mfs_dir_truncate_file(FAR struct mfs_sb_s *sb, mfs_t location,
                          FAR const struct mfs_dirloc_s *dirloc,
                          FAR struct mfs_direntry_s *direntry,
                          off_t length);

bool mfs_file_is_open(FAR const struct mfs_sb_s *sb, mfs_t fileloc);

int mfs_file_open(FAR struct mfs_sb_s *sb, FAR struct file *filep,
                  FAR const char *relpath, int oflags, mode_t mode);
int mfs_file_close(FAR struct mfs_sb_s *sb, FAR struct file *filep);
ssize_t mfs_file_read(FAR struct mfs_sb_s *sb, FAR struct file *filep,
                      FAR char *buffer, size_t buflen);
ssize_t mfs_file_write(FAR struct mfs_sb_s *sb, FAR struct file *filep,
                       FAR const char *buffer, size_t buflen);
off_t mfs_file_seek(FAR struct mfs_sb_s *sb, FAR struct file *filep,
                    off_t offset, int whence);
int mfs_file_ioctl(FAR struct mfs_sb_s *sb, FAR struct file *filep,
                   int cmd, unsigned long arg);
int mfs_file_truncate(FAR struct mfs_sb_s *sb, FAR struct file *filep,
                      off_t length);
int mfs_file_sync(FAR struct mfs_sb_s *sb, FAR struct file *filep);
int mfs_file_dup(FAR struct mfs_sb_s *sb, FAR const struct file *oldp,
                 FAR struct file *newp);
int mfs_file_fstat(FAR struct mfs_sb_s *sb, FAR const struct file *filep,
                   FAR struct stat *buf);

int mfs_dir_opendir(FAR struct mfs_sb_s *sb, FAR const char *relpath,
                    FAR struct fs_dirent_s **dir);
int mfs_dir_closedir(FAR struct mfs_sb_s *sb,
                     FAR struct fs_dirent_s *dir);
int mfs_dir_readdir(FAR struct mfs_sb_s *sb, FAR struct fs_dirent_s *dir,
                    FAR struct dirent *entry);
int mfs_dir_rewinddir(FAR struct mfs_sb_s *sb,
                      FAR struct fs_dirent_s *dir);
int mfs_dir_unlink(FAR struct mfs_sb_s *sb, FAR const char *relpath);
int mfs_dir_mkdir(FAR struct mfs_sb_s *sb, FAR const char *relpath,
                  mode_t mode);
int mfs_dir_rmdir(FAR struct mfs_sb_s *sb, FAR const char *relpath);
int mfs_dir_rename(FAR struct mfs_sb_s *sb, FAR const char *oldrelpath,
                   FAR const char *newrelpath);
int mfs_dir_stat(FAR struct mfs_sb_s *sb, FAR const char *relpath,
                 FAR struct stat *buf);

extern const struct mountpt_operations g_mnemofs_operations;

#endif /* __FS_MNEMOFS_MNEMOFS_H */
