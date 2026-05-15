/****************************************************************************
 * fs/mnemofs/mnemofs_dirent.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/kmalloc.h>

#include <errno.h>
#include <string.h>
#include <sys/stat.h>

#include "fs_heap.h"
#include "mnemofs.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static FAR const char *mfs_direntry_name(FAR const char *relpath);
static size_t mfs_direntry_namelen(
              FAR const struct mfs_direntry_s *direntry);
static bool mfs_direntry_name_equal(
  FAR const struct mfs_direntry_s *direntry,
  FAR const char *name);
static uint32_t mfs_deterministic_hash(FAR const uint8_t *arr, size_t sz);
static uint8_t mfs_folded_hash8(FAR const uint8_t *arr, size_t sz);
static uint8_t mfs_dirmeta_checksum(FAR const struct mfs_dirmeta_s *dirmeta);
static bool mfs_dirmeta_is_valid(FAR const struct mfs_dirmeta_s *dirmeta);
static void mfs_dirmeta_init(FAR struct mfs_dirmeta_s *dirmeta,
                             mfs_t parent);
static int mfs_dirmeta_read(FAR struct mfs_sb_s *sb, mfs_t directory,
                            FAR struct mfs_dirmeta_s *dirmeta);
static int mfs_dirmeta_write(FAR struct mfs_sb_s *sb, mfs_t directory,
                             mfs_t parent);
static FAR const char *mfs_skip_slashes(FAR const char *path);
static int mfs_next_path_component(FAR const char *path, FAR char *name,
                                   FAR const char **next);
static FAR char *mfs_dir_pathdup(FAR const char *relpath);
static void mfs_dirloc_invalidate(FAR struct mfs_dirloc_s *dirloc);
static bool mfs_dirloc_is_valid(FAR const struct mfs_sb_s *sb, mfs_t block,
                                FAR const struct mfs_dirloc_s *dirloc);
static void mfs_dirloc_start(FAR const struct mfs_sb_s *sb, mfs_t block,
                             FAR struct mfs_dirloc_s *dirloc);
static int mfs_dirloc_next(FAR const struct mfs_sb_s *sb, mfs_t block,
                           FAR struct mfs_dirloc_s *dirloc);
static int mfs_direntry_read(FAR struct mfs_sb_s *sb,
                             FAR const struct mfs_dirloc_s *dirloc,
                             FAR struct mfs_direntry_s *direntry);
static void mfs_root_direntry(FAR const struct mfs_sb_s *sb,
                              FAR struct mfs_direntry_s *direntry);
static void mfs_stat_from_direntry(FAR const struct mfs_sb_s *sb,
                                   mfs_t location,
                                   FAR const struct mfs_direntry_s *direntry,
                                   FAR struct stat *buf);
static void mfs_direntry_to_dirent(FAR const struct mfs_direntry_s *direntry,
                                   FAR struct dirent *entry);
static int mfs_dirloc_parent_block(FAR const struct mfs_sb_s *sb,
                                   FAR const struct mfs_dirloc_s *dirloc,
                                   FAR mfs_t *block);
static int mfs_dir_follow(FAR struct mfs_sb_s *sb, mfs_t directory,
                          FAR const struct mfs_dirloc_s *startloc,
                          FAR const struct mfs_direntry_s *startentry,
                          FAR struct mfs_direntry_s *latest,
                          FAR struct mfs_dirloc_s *latestloc);
static int mfs_dir_lookup(FAR struct mfs_sb_s *sb, mfs_t directory,
                          FAR const char *name, FAR mfs_t *location,
                          FAR struct mfs_direntry_s *direntry,
                          FAR struct mfs_dirloc_s *dirloc);
static int mfs_dir_lookup_location(FAR struct mfs_sb_s *sb, mfs_t directory,
                                   mfs_t location,
                                   FAR struct mfs_direntry_s *direntry,
                                   FAR struct mfs_dirloc_s *dirloc);
static int mfs_path_parent_dir(FAR struct mfs_sb_s *sb,
                               FAR const char *relpath,
                               FAR mfs_t *directory);
static int mfs_parent_dir(FAR struct mfs_sb_s *sb, mfs_t location,
                          FAR const struct mfs_dirloc_s *dirloc,
                          FAR mfs_t *directory);
static int mfs_same_directory(FAR struct mfs_sb_s *sb,
                              FAR const char *newrelpath,
                              mfs_t location,
                              FAR const struct mfs_dirloc_s *dirloc,
                              FAR mfs_t *directory);
static uint16_t mfs_direntry_namehash(FAR const char *name, size_t namelen);
static uint8_t mfs_direntry_checksum(
               FAR const struct mfs_direntry_s *direntry);
static bool mfs_direntry_is_valid(FAR const struct mfs_direntry_s *direntry);
static int mfs_direntry_init(FAR struct mfs_direntry_s *direntry,
                             mode_t mode, FAR const char *relpath);
static int mfs_direntry_set_name(FAR struct mfs_direntry_s *direntry,
                                 FAR const char *relpath);
static void mfs_direntry_seal(FAR struct mfs_direntry_s *direntry);
static int mfs_direntry_add_noflush(
  FAR struct mfs_sb_s *sb, mfs_t directory,
  FAR const struct mfs_direntry_s *direntry);
static int mfs_dir_reparent_children(FAR struct mfs_sb_s *sb,
                                     mfs_t directory, mfs_t parent);
static int mfs_dir_refresh_open_files(FAR struct mfs_sb_s *sb,
                                      mfs_t olddir, mfs_t newdir);
static int mfs_direntry_flush(FAR struct mfs_sb_s *sb, mfs_t directory,
                              FAR const struct mfs_direntry_s *direntry);
static int mfs_direntry_add(FAR struct mfs_sb_s *sb, mfs_t directory,
                            FAR const struct mfs_direntry_s *direntry);
static int mfs_dir_is_empty(FAR struct mfs_sb_s *sb, mfs_t directory);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mfs_direntry_name
 *
 * Description:
 * Return a pointer to the final path component in relpath.
 *
 * Input Parameters:
 *   relpath - The relative path to inspect.
 *
 * Returned Value:
 * A pointer to the last path component is returned on success. NULL is
 * returned if relpath is NULL.
 *
 ****************************************************************************/

static FAR const char *mfs_direntry_name(FAR const char *relpath)
{
  FAR const char *name;

  if (relpath == NULL)
    {
      return NULL;
    }

  name = strrchr(relpath, '/');
  return name == NULL ? relpath : name + 1;
}

/****************************************************************************
 * Name: mfs_direntry_namelen
 *
 * Description:
 * Return the bounded length of the name stored in a direntry.
 *
 * Input Parameters:
 *   direntry - The direntry whose name length is requested.
 *
 * Returned Value:
 * The string length of the direntry name is returned. Zero is returned
 * if direntry is NULL.
 *
 ****************************************************************************/

static size_t mfs_direntry_namelen(FAR const struct mfs_direntry_s *direntry)
{
  if (direntry == NULL)
    {
      return 0;
    }

  return strnlen(direntry->name, sizeof(direntry->name));
}

/****************************************************************************
 * Name: mfs_direntry_name_equal
 *
 * Description:
 * Compare the stored direntry name with a caller-provided name.
 *
 * Input Parameters:
 *   direntry - The direntry to compare.
 *   name - The candidate name.
 *
 * Returned Value:
 * true is returned if both names are valid and equal. false is returned
 * if either input is invalid or the names differ.
 *
 ****************************************************************************/

static bool mfs_direntry_name_equal(
  FAR const struct mfs_direntry_s *direntry,
  FAR const char *name)
{
  size_t dirnamelen;
  size_t namelen;

  if (direntry == NULL || name == NULL)
    {
      return false;
    }

  namelen = strnlen(name, MFS_NAME_LEN);
  if (namelen == 0 || namelen > MFS_NAME_MAX)
    {
      return false;
    }

  dirnamelen = mfs_direntry_namelen(direntry);
  return dirnamelen == namelen &&
         memcmp(direntry->name, name, namelen) == 0;
}

/****************************************************************************
 * Name: mfs_deterministic_hash
 *
 * Description:
 * Compute a stable 32-bit hash over a byte array.
 *
 * Input Parameters:
 *   arr - The byte array to hash.
 *   sz - The number of bytes in arr.
 *
 * Returned Value:
 * The computed hash value is returned. Zero is returned if arr is NULL.
 *
 ****************************************************************************/

static uint32_t mfs_deterministic_hash(FAR const uint8_t *arr, size_t sz)
{
  uint32_t distance;
  uint32_t hash;
  uint32_t product;
  uint32_t reverse;
  uint32_t shift_amount;
  uint32_t term1;
  uint32_t term2;
  size_t midpoint;
  size_t reverse_shift;
  size_t i;

  if (arr == NULL)
    {
      return 0;
    }

  midpoint = sz / 2;
  hash = 0;

  for (i = 0; i < sz; i++)
    {
      term1 = arr[i] + (uint32_t)i;
      reverse = arr[sz - i - 1];
      reverse_shift = sz - i - 1;
      term2 = reverse_shift >= 32 ? 0 : reverse >> reverse_shift;
      product = term1 * term2;
      distance = (uint32_t)(midpoint > i ? midpoint - i : i - midpoint);
      shift_amount = i % 32;
      hash += (product ^ distance) << shift_amount;
    }

  return hash;
}

/****************************************************************************
 * Name: mfs_folded_hash8
 *
 * Description:
 * Fold the deterministic 32-bit hash down to one byte.
 *
 * Input Parameters:
 *   arr - The byte array to hash.
 *   sz - The number of bytes in arr.
 *
 * Returned Value:
 * The folded 8-bit hash value is returned.
 *
 ****************************************************************************/

static uint8_t mfs_folded_hash8(FAR const uint8_t *arr, size_t sz)
{
  return (uint8_t)(mfs_deterministic_hash(arr, sz) %
                   ((uint32_t)UINT8_MAX + 1));
}

/****************************************************************************
 * Name: mfs_dirmeta_checksum
 *
 * Description:
 * Compute the checksum stored in a directory metadata record.
 *
 * Input Parameters:
 *   dirmeta - The directory metadata record to checksum.
 *
 * Returned Value:
 * The computed checksum value is returned.
 *
 ****************************************************************************/

static uint8_t mfs_dirmeta_checksum(FAR const struct mfs_dirmeta_s *dirmeta)
{
  return mfs_folded_hash8((FAR const uint8_t *)dirmeta,
                          sizeof(*dirmeta) - 1);
}

/****************************************************************************
 * Name: mfs_dirmeta_is_valid
 *
 * Description:
 * Verify the magic and checksum stored in a directory metadata record.
 *
 * Input Parameters:
 *   dirmeta - The directory metadata record to validate.
 *
 * Returned Value:
 * true is returned if dirmeta contains a valid directory metadata
 * record. false is returned otherwise.
 *
 ****************************************************************************/

static bool mfs_dirmeta_is_valid(FAR const struct mfs_dirmeta_s *dirmeta)
{
  if (dirmeta == NULL)
    {
      return false;
    }

  if (dirmeta->magic != MFS_DIR_MAGIC)
    {
      return false;
    }

  if (dirmeta->checksum != mfs_dirmeta_checksum(dirmeta))
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: mfs_dirmeta_init
 *
 * Description:
 * Initialize one in-memory directory metadata record.
 *
 * Input Parameters:
 *   dirmeta - The metadata record to initialize.
 *   parent - The parent directory block number to store.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

static void mfs_dirmeta_init(FAR struct mfs_dirmeta_s *dirmeta,
                             mfs_t parent)
{
  if (dirmeta == NULL)
    {
      return;
    }

  memset(dirmeta, 0, sizeof(*dirmeta));
  dirmeta->magic    = MFS_DIR_MAGIC;
  dirmeta->parent   = parent;
  dirmeta->checksum = mfs_dirmeta_checksum(dirmeta);
}

/****************************************************************************
 * Name: mfs_dirmeta_read
 *
 * Description:
 * Read directory metadata from the first page of a directory block.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   directory - The directory block number.
 *   dirmeta - The location to receive the decoded metadata.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mfs_dirmeta_read(FAR struct mfs_sb_s *sb, mfs_t directory,
                            FAR struct mfs_dirmeta_s *dirmeta)
{
  mfs_t page;
  ssize_t nread;

  if (sb == NULL || dirmeta == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  if (directory == MFS_LOCATION_INVALID || directory >= MFS_BLOCK_COUNT(sb))
    {
      ferr("invalid directory block\n");
      return -EINVAL;
    }

  page = MFS_BLOCK_TO_PAGE(sb, directory);
  nread = mfs_read_page(sb, page, sb->rwbuf);
  if (nread < 0)
    {
      ferr("mfs_read_page failed: %zd\n", nread);
      return nread;
    }

  if (nread != 1)
    {
      ferr("short read: %zd\n", nread);
      return -EIO;
    }

  memcpy(dirmeta, sb->rwbuf, sizeof(*dirmeta));
  if (!mfs_dirmeta_is_valid(dirmeta))
    {
      ferr("invalid directory metadata\n");
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: mfs_dirmeta_write
 *
 * Description:
 * Write directory metadata into the first page of a directory block.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   directory - The directory block number.
 *   parent - The parent directory block number.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mfs_dirmeta_write(FAR struct mfs_sb_s *sb, mfs_t directory,
                             mfs_t parent)
{
  struct mfs_dirmeta_s dirmeta;
  mfs_t page;
  int ret;
  ssize_t nwritten;

  if (sb == NULL)
    {
      ferr("invalid sb\n");
      return -EINVAL;
    }

  if (directory == MFS_LOCATION_INVALID || directory >= MFS_BLOCK_COUNT(sb))
    {
      ferr("invalid directory block\n");
      return -EINVAL;
    }

  mfs_dirmeta_init(&dirmeta, parent);
  ret = mfs_rwbuf_prepare_write(sb);
  if (ret < 0)
    {
      ferr("mfs_rwbuf_prepare_write failed: %d\n", ret);
      return ret;
    }

  memset(sb->rwbuf, 0, MFS_PAGE_SIZE(sb));
  memcpy(sb->rwbuf, &dirmeta, sizeof(dirmeta));

  page = MFS_BLOCK_TO_PAGE(sb, directory);
  nwritten = mfs_write_page(sb, page, sb->rwbuf);
  if (nwritten < 0)
    {
      ferr("mfs_write_page failed: %zd\n", nwritten);
      return nwritten;
    }

  if (nwritten != 1)
    {
      ferr("short write: %zd\n", nwritten);
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: mfs_skip_slashes
 *
 * Description:
 * Advance past any leading slash characters in path.
 *
 * Input Parameters:
 *   path - The path pointer to normalize.
 *
 * Returned Value:
 * A pointer to the first non-slash character is returned. If path is
 * NULL, then NULL is returned.
 *
 ****************************************************************************/

static FAR const char *mfs_skip_slashes(FAR const char *path)
{
  while (path != NULL && *path == '/')
    {
      path++;
    }

  return path;
}

/****************************************************************************
 * Name: mfs_next_path_component
 *
 * Description:
 * Decode one path component and advance next to the following component.
 *
 * Input Parameters:
 *   path - The path string to parse.
 *   name - The buffer that receives the decoded component.
 *   next - The location that receives the pointer to the remaining path.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned
 * if the inputs are invalid or the component exceeds MFS_NAME_MAX.
 *
 ****************************************************************************/

static int mfs_next_path_component(FAR const char *path, FAR char *name,
                                   FAR const char **next)
{
  size_t len;

  if (path == NULL || name == NULL || next == NULL)
    {
      return -EINVAL;
    }

  memset(name, 0, MFS_NAME_LEN);
  path = mfs_skip_slashes(path);
  if (path == NULL)
    {
      return -EINVAL;
    }

  if (*path == '\0')
    {
      * next = path;
      return OK;
    }

  len = strcspn(path, "/");
  if (len > MFS_NAME_MAX)
    {
      return -ENAMETOOLONG;
    }

  memcpy(name, path, len);
  name[len] = '\0';
  * next = mfs_skip_slashes(path + len);
  return OK;
}

/****************************************************************************
 * Name: mfs_dir_pathdup
 *
 * Description:
 * Duplicate a directory path string for an open directory handle.
 *
 * Input Parameters:
 *   relpath - The relative path to duplicate.
 *
 * Returned Value:
 * A heap-allocated copy of relpath is returned on success. If relpath is
 * empty, then a copy of "/" is returned. NULL is returned on allocation
 * failure.
 *
 ****************************************************************************/

static FAR char *mfs_dir_pathdup(FAR const char *relpath)
{
  FAR const char *src;
  FAR char *dst;
  size_t len;

  src = relpath;
  if (src == NULL || *src == '\0')
    {
      src = "/";
    }

  len = strlen(src) + 1;
  dst = fs_heap_malloc(len);
  if (dst == NULL)
    {
      return NULL;
    }

  memcpy(dst, src, len);
  return dst;
}

/****************************************************************************
 * Name: mfs_dirloc_invalidate
 *
 * Description:
 * Mark a dirloc as invalid.
 *
 * Input Parameters:
 *   dirloc - The dirloc to invalidate.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

static void mfs_dirloc_invalidate(FAR struct mfs_dirloc_s *dirloc)
{
  if (dirloc == NULL)
    {
      return;
    }

  dirloc->page   = MFS_LOCATION_INVALID;
  dirloc->offset = MFS_DIRENT_OFF_INVALID;
}

/****************************************************************************
 * Name: mfs_dirloc_is_valid
 *
 * Description:
 * Check whether a dirloc refers to a valid slot inside one directory
 * block.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   block - The directory block that should own dirloc.
 *   dirloc - The dirloc to validate.
 *
 * Returned Value:
 * true is returned if dirloc references a valid direntry slot inside the
 * directory block. false is returned otherwise.
 *
 ****************************************************************************/

static bool mfs_dirloc_is_valid(FAR const struct mfs_sb_s *sb, mfs_t block,
                                FAR const struct mfs_dirloc_s *dirloc)
{
  mfs_t firstpage;
  mfs_t lastpage;

  if (sb == NULL || dirloc == NULL)
    {
      return false;
    }

  if (block == MFS_LOCATION_INVALID || block >= MFS_BLOCK_COUNT(sb))
    {
      return false;
    }

  if (dirloc->page == MFS_LOCATION_INVALID ||
      dirloc->offset == MFS_DIRENT_OFF_INVALID)
    {
      return false;
    }

  if (dirloc->offset + sizeof(struct mfs_direntry_s) > MFS_PAGE_SIZE(sb))
    {
      return false;
    }

  firstpage = MFS_BLOCK_TO_PAGE(sb, block) + 1;
  lastpage  = MFS_BLOCK_TO_PAGE(sb, block) + MFS_PAGES_PER_BLOCK(sb) - 1;
  return firstpage <= lastpage &&
         dirloc->page >= firstpage && dirloc->page <= lastpage;
}

/****************************************************************************
 * Name: mfs_dirloc_start
 *
 * Description:
 * Initialize a dirloc so it points at the first direntry slot in a
 * directory block.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   block - The directory block to scan.
 *   dirloc - The dirloc to initialize.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

static void mfs_dirloc_start(FAR const struct mfs_sb_s *sb, mfs_t block,
                             FAR struct mfs_dirloc_s *dirloc)
{
  if (dirloc == NULL)
    {
      return;
    }

  if (sb == NULL || MFS_PAGES_PER_BLOCK(sb) < 2)
    {
      mfs_dirloc_invalidate(dirloc);
      return;
    }

  dirloc->page   = MFS_BLOCK_TO_PAGE(sb, block) + 1;
  dirloc->offset = 0;
}

/****************************************************************************
 * Name: mfs_dirloc_next
 *
 * Description:
 * Advance a dirloc to the next direntry slot within the same directory
 * block.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   block - The directory block being scanned.
 *   dirloc - The dirloc to advance.
 *
 * Returned Value:
 * Zero (OK) is returned on success. -ENOENT is returned if dirloc is
 * already at the end of the directory block or otherwise invalid.
 *
 ****************************************************************************/

static int mfs_dirloc_next(FAR const struct mfs_sb_s *sb, mfs_t block,
                           FAR struct mfs_dirloc_s *dirloc)
{
  mfs_t nextoff;
  mfs_t lastpage;

  if (!mfs_dirloc_is_valid(sb, block, dirloc))
    {
      return -ENOENT;
    }

  nextoff = dirloc->offset + sizeof(struct mfs_direntry_s);
  if (nextoff + sizeof(struct mfs_direntry_s) <= MFS_PAGE_SIZE(sb))
    {
      dirloc->offset = nextoff;
      return OK;
    }

  lastpage = MFS_BLOCK_TO_PAGE(sb, block) + MFS_PAGES_PER_BLOCK(sb) - 1;
  if (dirloc->page >= lastpage)
    {
      mfs_dirloc_invalidate(dirloc);
      return -ENOENT;
    }

  dirloc->page++;
  dirloc->offset = 0;
  return OK;
}

/****************************************************************************
 * Name: mfs_direntry_read
 *
 * Description:
 * Read one direntry record from the location identified by dirloc.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   dirloc - The direntry location to read.
 *   direntry - The location that receives the decoded direntry.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned if
 * dirloc is invalid or the underlying page read fails.
 *
 ****************************************************************************/

static int mfs_direntry_read(FAR struct mfs_sb_s *sb,
                             FAR const struct mfs_dirloc_s *dirloc,
                             FAR struct mfs_direntry_s *direntry)
{
  ssize_t nread;

  if (sb == NULL || dirloc == NULL || direntry == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  if (dirloc->page == MFS_LOCATION_INVALID ||
      dirloc->offset + sizeof(*direntry) > MFS_PAGE_SIZE(sb))
    {
      ferr("invalid dirloc\n");
      return -EINVAL;
    }

  nread = mfs_read_page(sb, dirloc->page, sb->rwbuf);
  if (nread < 0)
    {
      ferr("mfs_read_page failed: %zd\n", nread);
      return nread;
    }

  if (nread != 1)
    {
      ferr("short read: %zd\n", nread);
      return -EIO;
    }

  memcpy(direntry, sb->rwbuf + dirloc->offset, sizeof(*direntry));
  return OK;
}

/****************************************************************************
 * Name: mfs_page_is_erased
 *
 * Description:
 * Check whether a page buffer is still entirely in the device erase
 * state.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   pagebuf - The page buffer to inspect.
 *
 * Returned Value:
 * true is returned if every byte matches sb->erasestate. false is
 * returned otherwise.
 *
 ****************************************************************************/

static bool mfs_page_is_erased(FAR const struct mfs_sb_s *sb,
                               FAR const uint8_t *pagebuf)
{
  size_t i;

  if (sb == NULL || pagebuf == NULL)
    {
      return false;
    }

  for (i = 0; i < MFS_PAGE_SIZE(sb); i++)
    {
      if (pagebuf[i] != sb->erasestate)
        {
          return false;
        }
    }

  return true;
}

/****************************************************************************
 * Name: mfs_root_direntry
 *
 * Description:
 * Build the synthetic direntry that represents the mounted root
 * directory.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   direntry - The location that receives the synthetic root direntry.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

static void mfs_root_direntry(FAR const struct mfs_sb_s *sb,
                              FAR struct mfs_direntry_s *direntry)
{
  memset(direntry, 0, sizeof(*direntry));
  direntry->magic     = MFS_DIRENT_MAGIC;
  direntry->mode      = S_IFDIR | S_IRWXU | S_IRGRP | S_IXGRP |
                        S_IROTH | S_IXOTH;
  direntry->newloc    = sb->rootdir;
  direntry->meta.size = MFS_BLOCK_SIZE(sb);
  mfs_direntry_seal(direntry);
}

/****************************************************************************
 * Name: mfs_stat_from_direntry
 *
 * Description:
 * Populate a struct stat from one live direntry record.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   location - The live object location.
 *   direntry - The direntry that describes the object.
 *   buf - The location that receives the converted stat data.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

static void mfs_stat_from_direntry(FAR const struct mfs_sb_s *sb,
                                   mfs_t location,
                                   FAR const struct mfs_direntry_s *direntry,
                                   FAR struct stat *buf)
{
  memset(buf, 0, sizeof(*buf));
  buf->st_ino         = location;
  buf->st_mode        = (mode_t)direntry->mode;
  buf->st_nlink       = 1;
  buf->st_size        = S_ISDIR(direntry->mode) ?
                        MFS_BLOCK_SIZE(sb) : direntry->meta.size;
  buf->st_atim.tv_sec = direntry->mtime;
  buf->st_mtim.tv_sec = direntry->mtime;
  buf->st_ctim.tv_sec = direntry->ctime;
  buf->st_blksize     = MFS_PAGE_SIZE(sb);
  buf->st_blocks      = (buf->st_size + buf->st_blksize - 1) /
                        buf->st_blksize;
}

/****************************************************************************
 * Name: mfs_direntry_to_dirent
 *
 * Description:
 * Convert one direntry record into a POSIX dirent.
 *
 * Input Parameters:
 *   direntry - The direntry to convert.
 *   entry - The location that receives the dirent view.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

static void mfs_direntry_to_dirent(FAR const struct mfs_direntry_s *direntry,
                                   FAR struct dirent *entry)
{
  size_t namelen;

  memset(entry, 0, sizeof(*entry));
  namelen = mfs_direntry_namelen(direntry);
  if (namelen >= sizeof(entry->d_name))
    {
      namelen = sizeof(entry->d_name) - 1;
    }

  memcpy(entry->d_name, direntry->name, namelen);
  entry->d_name[namelen] = '\0';
  entry->d_type = S_ISDIR(direntry->mode) ? DTYPE_DIRECTORY : DTYPE_FILE;
}

/****************************************************************************
 * Name: mfs_dirloc_parent_block
 *
 * Description:
 * Derive the parent directory block number from a direntry location.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   dirloc - The direntry location inside the parent directory block.
 *   block - The location that receives the parent block number.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned if
 * the inputs are invalid or dirloc does not map to a valid block.
 *
 ****************************************************************************/

static int mfs_dirloc_parent_block(FAR const struct mfs_sb_s *sb,
                                   FAR const struct mfs_dirloc_s *dirloc,
                                   FAR mfs_t *block)
{
  if (sb == NULL || dirloc == NULL || block == NULL)
    {
      return -EINVAL;
    }

  * block = mfs_page_to_block(sb, dirloc->page);
  if (*block == MFS_LOCATION_INVALID)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: mfs_dir_follow
 *
 * Description:
 * Follow one directory-entry history chain from its first record to the
 * latest reachable record.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   directory - The directory block that owns the chain.
 *   startloc - The location of the first record in the chain.
 *   startentry - The first record in the chain.
 *   latest - The location to receive the latest record contents.
 *   latestloc - The location to receive the latest record address.
 *
 * Returned Value:
 * Zero (OK) is returned on success. -ENOENT is returned when the chain
 * does not lead to a live location. A negated errno value is returned on
 * other failures.
 *
 ****************************************************************************/

static int mfs_dir_follow(FAR struct mfs_sb_s *sb, mfs_t directory,
                          FAR const struct mfs_dirloc_s *startloc,
                          FAR const struct mfs_direntry_s *startentry,
                          FAR struct mfs_direntry_s *latest,
                          FAR struct mfs_dirloc_s *latestloc)
{
  struct mfs_dirloc_s scanloc;
  struct mfs_direntry_s current;
  struct mfs_direntry_s candidate;
  int ret;

  if (sb == NULL || startloc == NULL || startentry == NULL ||
      latest == NULL || latestloc == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  memcpy(&current, startentry, sizeof(current));
  memcpy(latest, startentry, sizeof(*latest));
  memcpy(latestloc, startloc, sizeof(*latestloc));

  /* A tombstone ends this history chain.  Later direntries with
   * oldloc == MFS_LOCATION_INVALID are fresh heads, not successors.
   */

  if (current.newloc == MFS_LOCATION_INVALID)
    {
      return -ENOENT;
    }

  scanloc = *startloc;
  ret = mfs_dirloc_next(sb, directory, &scanloc);
  while (ret == OK)
    {
      ret = mfs_direntry_read(sb, &scanloc, &candidate);
      if (ret < 0)
        {
          ferr("mfs_direntry_read failed: %d\n", ret);
          return ret;
        }

      if (mfs_direntry_is_valid(&candidate) &&
          candidate.oldloc == current.newloc)
        {
          memcpy(&current, &candidate, sizeof(current));
          memcpy(latest, &candidate, sizeof(*latest));
          memcpy(latestloc, &scanloc, sizeof(*latestloc));

          if (current.newloc == MFS_LOCATION_INVALID)
            {
              return -ENOENT;
            }
        }

      ret = mfs_dirloc_next(sb, directory, &scanloc);
    }

  return OK;
}

/****************************************************************************
 * Name: mfs_dir_lookup
 *
 * Description:
 * Locate the latest live direntry for one name inside a directory block.
 * The search scans only chain starting points and then follows each chain
 * according to the oldloc/newloc rule.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   directory - The directory block number.
 *   name - The entry name to search for.
 *   location - The location to receive the latest object location.
 *   direntry - The location to receive the latest direntry contents.
 *   dirloc - The location to receive the latest direntry address.
 *
 * Returned Value:
 * Zero (OK) is returned on success. -ENOENT is returned if the named
 * entry is not found. A negated errno value is returned on other
 * failures.
 *
 ****************************************************************************/

static int mfs_dir_lookup(FAR struct mfs_sb_s *sb, mfs_t directory,
                          FAR const char *name, FAR mfs_t *location,
                          FAR struct mfs_direntry_s *direntry,
                          FAR struct mfs_dirloc_s *dirloc)
{
  struct mfs_dirloc_s scanloc;
  struct mfs_direntry_s candidate;
  struct mfs_direntry_s latest;
  struct mfs_dirloc_s latestloc;
  uint16_t namehash;
  int ret;

  if (sb == NULL || name == NULL || location == NULL ||
      direntry == NULL || dirloc == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  if (directory == MFS_LOCATION_INVALID || directory >= MFS_BLOCK_COUNT(sb))
    {
      return -ENOENT;
    }

  namehash = mfs_direntry_namehash(name, strlen(name));
  mfs_dirloc_start(sb, directory, &scanloc);

  while (mfs_dirloc_is_valid(sb, directory, &scanloc))
    {
      ret = mfs_direntry_read(sb, &scanloc, &candidate);
      if (ret < 0)
        {
          ferr("mfs_direntry_read failed: %d\n", ret);
          return ret;
        }

      if (mfs_direntry_is_valid(&candidate) &&
          candidate.oldloc == MFS_LOCATION_INVALID)
        {
          ret = mfs_dir_follow(sb, directory, &scanloc, &candidate,
                               &latest, &latestloc);
          if (ret < 0 && ret != -ENOENT)
            {
              ferr("mfs_dir_follow failed: %d\n", ret);
            }

          if (ret == OK &&
              latest.namehash == namehash &&
              mfs_direntry_name_equal(&latest, name))
            {
              * location = latest.newloc;
              memcpy(direntry, &latest, sizeof(*direntry));
              memcpy(dirloc, &latestloc, sizeof(*dirloc));
              return OK;
            }
        }

      ret = mfs_dirloc_next(sb, directory, &scanloc);
      if (ret < 0)
        {
          break;
        }
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: mfs_dir_lookup_location
 *
 * Description:
 * Locate the latest live direntry whose resolved location matches the
 * requested location.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   directory - The directory block number to search.
 *   location - The live object location to match.
 *   direntry - The location that receives the latest direntry contents.
 *   dirloc - The location that receives the latest direntry address.
 *
 * Returned Value:
 * Zero (OK) is returned on success. -ENOENT is returned if no live
 * direntry resolves to location. A negated errno value is returned on
 * other failures.
 *
 ****************************************************************************/

static int mfs_dir_lookup_location(FAR struct mfs_sb_s *sb, mfs_t directory,
                                   mfs_t location,
                                   FAR struct mfs_direntry_s *direntry,
                                   FAR struct mfs_dirloc_s *dirloc)
{
  struct mfs_dirloc_s scanloc;
  struct mfs_direntry_s candidate;
  struct mfs_direntry_s latest;
  struct mfs_dirloc_s latestloc;
  int ret;

  if (sb == NULL || direntry == NULL || dirloc == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  if (directory == MFS_LOCATION_INVALID ||
      directory >= MFS_BLOCK_COUNT(sb) ||
      location == MFS_LOCATION_INVALID)
    {
      return -ENOENT;
    }

  mfs_dirloc_start(sb, directory, &scanloc);
  while (mfs_dirloc_is_valid(sb, directory, &scanloc))
    {
      ret = mfs_direntry_read(sb, &scanloc, &candidate);
      if (ret < 0)
        {
          ferr("mfs_direntry_read failed: %d\n", ret);
          return ret;
        }

      if (mfs_direntry_is_valid(&candidate) &&
          candidate.oldloc == MFS_LOCATION_INVALID)
        {
          ret = mfs_dir_follow(sb, directory, &scanloc, &candidate,
                               &latest, &latestloc);
          if (ret == OK && latest.newloc == location)
            {
              memcpy(direntry, &latest, sizeof(*direntry));
              memcpy(dirloc, &latestloc, sizeof(*dirloc));
              return OK;
            }

          if (ret < 0 && ret != -ENOENT)
            {
              ferr("mfs_dir_follow failed: %d\n", ret);
              return ret;
            }
        }

      ret = mfs_dirloc_next(sb, directory, &scanloc);
      if (ret < 0)
        {
          break;
        }
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: mfs_path_parent_dir
 *
 * Description:
 * Resolve the parent directory block for a relative path.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   relpath - The relative path whose parent is requested.
 *   directory - The location to receive the parent directory block number.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mfs_path_parent_dir(FAR struct mfs_sb_s *sb,
                               FAR const char *relpath,
                               FAR mfs_t *directory)
{
  FAR const char *path;
  FAR const char *next;
  struct mfs_direntry_s direntry;
  struct mfs_dirloc_s dirloc;
  char component[MFS_NAME_LEN];
  mfs_t current;
  int ret;

  if (sb == NULL || relpath == NULL || directory == NULL)
    {
      return -EINVAL;
    }

  memset(component, 0, sizeof(component));
  path = mfs_skip_slashes(relpath);
  if (path == NULL || *path == '\0')
    {
      return -EINVAL;
    }

  current = sb->rootdir;
  for (; ; )
    {
      ret = mfs_next_path_component(path, component, &next);
      if (ret < 0)
        {
          return ret;
        }

      if (component[0] == '\0')
        {
          return -EINVAL;
        }

      if (*next == '\0')
        {
          * directory = current;
          return OK;
        }

      ret = mfs_dir_lookup(sb, current, component, &current,
                           &direntry, &dirloc);
      if (ret < 0)
        {
          return ret;
        }

      if (!S_ISDIR(direntry.mode))
        {
          return -ENOTDIR;
        }

      path = next;
    }
}

/****************************************************************************
 * Name: mfs_parent_dir
 *
 * Description:
 * Resolve the parent directory block for a live file system object. If
 * the direntry location is known, then the parent directory can be
 * derived directly from that page. Otherwise the function falls back to
 * reading the directory metadata stored in the object's own block.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   location - The live object location.
 *   dirloc - Optional latest direntry location for that object.
 *   directory - The location to receive the parent directory block number.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mfs_parent_dir(FAR struct mfs_sb_s *sb, mfs_t location,
                          FAR const struct mfs_dirloc_s *dirloc,
                          FAR mfs_t *directory)
{
  struct mfs_dirmeta_s dirmeta;
  int ret;

  if (sb == NULL || directory == NULL)
    {
      return -EINVAL;
    }

  if (location == sb->rootdir)
    {
      return -EBUSY;
    }

  if (dirloc != NULL)
    {
      ret = mfs_dirloc_parent_block(sb, dirloc, directory);
      if (ret == OK)
        {
          return OK;
        }
    }

  ret = mfs_dirmeta_read(sb, location, &dirmeta);
  if (ret < 0)
    {
      return ret;
    }

  if (dirmeta.parent == MFS_LOCATION_INVALID ||
      dirmeta.parent >= MFS_BLOCK_COUNT(sb))
    {
      return -EIO;
    }

  * directory = dirmeta.parent;
  return OK;
}

/****************************************************************************
 * Name: mfs_same_directory
 *
 * Description:
 * Verify that location and newrelpath belong to the same parent
 * directory.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   newrelpath - The replacement path to check.
 *   location - The live object location being examined.
 *   dirloc - Optional latest direntry location for location.
 *   directory - The location that receives the shared parent directory.
 *
 * Returned Value:
 * Zero (OK) is returned if both objects share the same parent directory.
 * -EXDEV is returned if the parent directories differ. A negated errno
 * value is returned on other failures.
 *
 ****************************************************************************/

static int mfs_same_directory(FAR struct mfs_sb_s *sb,
                              FAR const char *newrelpath,
                              mfs_t location,
                              FAR const struct mfs_dirloc_s *dirloc,
                              FAR mfs_t *directory)
{
  mfs_t olddir;
  mfs_t newdir;
  int ret;

  if (sb == NULL || newrelpath == NULL || directory == NULL)
    {
      return -EINVAL;
    }

  ret = mfs_parent_dir(sb, location, dirloc, &olddir);
  if (ret < 0)
    {
      return ret;
    }

  ret = mfs_path_parent_dir(sb, newrelpath, &newdir);
  if (ret < 0)
    {
      return ret;
    }

  if (olddir != newdir)
    {
      return -EXDEV;
    }

  * directory = olddir;
  return OK;
}

/****************************************************************************
 * Name: mfs_direntry_namehash
 *
 * Description:
 * Compute the 16-bit name hash stored in direntry records.
 *
 * Input Parameters:
 *   name - The name to hash.
 *   namelen - The number of characters in name to hash.
 *
 * Returned Value:
 * The computed 16-bit hash value is returned. Zero is returned if name
 * is NULL.
 *
 ****************************************************************************/

static uint16_t mfs_direntry_namehash(FAR const char *name, size_t namelen)
{
  FAR const uint8_t *arr;
  uint32_t ret;
  uint32_t idx_mult;
  uint32_t elem;
  size_t i;

  if (name == NULL)
    {
      return 0;
    }

  arr = (FAR const uint8_t *)name;
  ret = 0;

  for (i = 0; i < namelen; i++)
    {
      idx_mult = ((uint32_t)(i + 1) % UINT16_MAX) + 1;
      elem = ((uint32_t)arr[i] + 1) % ((uint32_t)UINT16_MAX + 1);
      elem = (uint32_t)(((uint64_t)elem * idx_mult * idx_mult) %
                        ((uint32_t)UINT16_MAX + 1));
      ret = (ret + elem) % ((uint32_t)UINT16_MAX + 1);
    }

  return (uint16_t)ret;
}

/****************************************************************************
 * Name: mfs_direntry_checksum
 *
 * Description:
 * Compute the checksum stored in one direntry record.
 *
 * Input Parameters:
 *   direntry - The direntry to checksum.
 *
 * Returned Value:
 * The computed checksum value is returned.
 *
 ****************************************************************************/

static uint8_t
mfs_direntry_checksum(FAR const struct mfs_direntry_s *direntry)
{
  return mfs_folded_hash8((FAR const uint8_t *)direntry,
                          sizeof(*direntry) - 1);
}

/****************************************************************************
 * Name: mfs_direntry_is_valid
 *
 * Description:
 * Verify the checksum, mode, and stored name fields of a direntry
 * record.
 *
 * Input Parameters:
 *   direntry - The direntry to validate.
 *
 * Returned Value:
 * true is returned if direntry contains a valid record. false is
 * returned otherwise.
 *
 ****************************************************************************/

static bool mfs_direntry_is_valid(FAR const struct mfs_direntry_s *direntry)
{
  size_t namelen;

  if (direntry == NULL)
    {
      return false;
    }

  if (direntry->magic != MFS_DIRENT_MAGIC)
    {
      return false;
    }

  if (direntry->checksum != mfs_direntry_checksum(direntry))
    {
      return false;
    }

  if (!S_ISDIR(direntry->mode) && !S_ISREG(direntry->mode))
    {
      return false;
    }

  namelen = mfs_direntry_namelen(direntry);
  if (namelen == 0)
    {
      return false;
    }

  if (direntry->namehash != mfs_direntry_namehash(direntry->name, namelen))
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: mfs_direntry_init
 *
 * Description:
 * Initialize a brand-new direntry record from one relative path and mode.
 *
 * Input Parameters:
 *   direntry - The direntry to initialize.
 *   mode - The file type and permission bits to store.
 *   relpath - The relative path whose final component becomes the name.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned if
 * the inputs are invalid or the final path component is too long.
 *
 ****************************************************************************/

static int mfs_direntry_init(FAR struct mfs_direntry_s *direntry,
                             mode_t mode, FAR const char *relpath)
{
  FAR const char *name;
  size_t namelen;

  if (direntry == NULL)
    {
      return -EINVAL;
    }

  if (!S_ISDIR(mode) && !S_ISREG(mode))
    {
      return -EINVAL;
    }

  name = mfs_direntry_name(relpath);
  if (name == NULL || *name == '\0')
    {
      return -EINVAL;
    }

  namelen = strnlen(name, MFS_NAME_LEN);
  if (namelen == 0 || namelen > MFS_NAME_MAX)
    {
      return -ENAMETOOLONG;
    }

  memset(direntry, 0, sizeof(*direntry));
  direntry->magic = MFS_DIRENT_MAGIC;
  direntry->mode  = (uint16_t)mode;
  memcpy(direntry->name, name, namelen);
  direntry->namehash = mfs_direntry_namehash(direntry->name, namelen);
  mfs_direntry_seal(direntry);
  return OK;
}

/****************************************************************************
 * Name: mfs_direntry_set_name
 *
 * Description:
 * Replace the stored direntry name with the final component of relpath.
 *
 * Input Parameters:
 *   direntry - The direntry to update.
 *   relpath - The relative path whose final component becomes the new
 *   name.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned if
 * the inputs are invalid or the final path component is too long.
 *
 ****************************************************************************/

static int mfs_direntry_set_name(FAR struct mfs_direntry_s *direntry,
                                 FAR const char *relpath)
{
  FAR const char *name;
  size_t namelen;

  if (direntry == NULL)
    {
      return -EINVAL;
    }

  name = mfs_direntry_name(relpath);
  if (name == NULL || *name == '\0')
    {
      return -EINVAL;
    }

  namelen = strnlen(name, MFS_NAME_LEN);
  if (namelen == 0 || namelen > MFS_NAME_MAX)
    {
      return -ENAMETOOLONG;
    }

  memset(direntry->name, 0, sizeof(direntry->name));
  memcpy(direntry->name, name, namelen);
  direntry->namehash = mfs_direntry_namehash(direntry->name, namelen);
  mfs_direntry_seal(direntry);
  return OK;
}

/****************************************************************************
 * Name: mfs_direntry_seal
 *
 * Description:
 * Refresh the checksum field after a direntry record is modified.
 *
 * Input Parameters:
 *   direntry - The direntry to seal.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

static void mfs_direntry_seal(FAR struct mfs_direntry_s *direntry)
{
  if (direntry == NULL)
    {
      return;
    }

  direntry->checksum = mfs_direntry_checksum(direntry);
}

/****************************************************************************
 * Name: mfs_direntry_add_noflush
 *
 * Description:
 * Append one new direntry record into the first blank page found in the
 * directory block. Directory metadata occupies the first page, so the
 * scan begins at the second page. Even though legacy images may contain
 * multiple packed direntries per page, new writes only consume fully
 * erased pages so one NAND page is never programmed twice.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   directory - The directory block number.
 *   direntry - The new direntry record to store.
 *
 * Returned Value:
 * Zero (OK) is returned on success. -ENOSPC is returned when the
 * directory block has no blank page left for a new direntry. A negated
 * errno value is returned on other failures.
 *
 ****************************************************************************/

static int mfs_direntry_add_noflush(
  FAR struct mfs_sb_s *sb, mfs_t directory,
  FAR const struct mfs_direntry_s *direntry)
{
  struct mfs_direntry_s candidate;
  mfs_t page;
  mfs_t endpage;
  size_t offset;
  ssize_t nread;
  ssize_t nwritten;
  bool occupied;

  if (sb == NULL || direntry == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  if (directory == MFS_LOCATION_INVALID || directory >= MFS_BLOCK_COUNT(sb))
    {
      ferr("invalid directory block\n");
      return -EINVAL;
    }

  if (!mfs_direntry_is_valid(direntry))
    {
      ferr("invalid direntry\n");
      return -EINVAL;
    }

  page    = MFS_BLOCK_TO_PAGE(sb, directory) + 1;
  endpage = MFS_BLOCK_TO_PAGE(sb, directory) + MFS_PAGES_PER_BLOCK(sb);
  for (; page < endpage; page++)
    {
      nread = mfs_read_page(sb, page, sb->rwbuf);
      if (nread < 0)
        {
          ferr("mfs_read_page failed: %zd\n", nread);
          return nread;
        }

      if (nread != 1)
        {
          ferr("short read: %zd\n", nread);
          return -EIO;
        }

      occupied = false;
      for (offset = 0;
           offset + sizeof(candidate) <= MFS_PAGE_SIZE(sb);
           offset += sizeof(candidate))
        {
          memcpy(&candidate, sb->rwbuf + offset, sizeof(candidate));
          if (mfs_direntry_is_valid(&candidate))
            {
              occupied = true;
              break;
            }
        }

      if (occupied || !mfs_page_is_erased(sb, sb->rwbuf))
        {
          continue;
        }

      memcpy(sb->rwbuf, direntry, sizeof(*direntry));
      nwritten = mfs_write_page(sb, page, sb->rwbuf);
      if (nwritten < 0)
        {
          ferr("mfs_write_page failed: %zd\n", nwritten);
          return nwritten;
        }

      if (nwritten != 1)
        {
          ferr("short write: %zd\n", nwritten);
          return -EIO;
        }

      return OK;
    }

  ferr("directory full\n");
  return -ENOSPC;
}

/****************************************************************************
 * Name: mfs_dir_reparent_children
 *
 * Description:
 * Rewrite the metadata page of each live child directory so its stored
 * parent block number becomes parent.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   directory - The directory block whose live children are scanned.
 *   parent - The parent block number to store into child metadata.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned if
 * scanning or rewriting a child directory metadata page fails.
 *
 ****************************************************************************/

static int mfs_dir_reparent_children(FAR struct mfs_sb_s *sb,
                                     mfs_t directory, mfs_t parent)
{
  struct mfs_dirloc_s scanloc;
  struct mfs_direntry_s candidate;
  struct mfs_direntry_s latest;
  struct mfs_dirloc_s latestloc;
  int ret;

  if (sb == NULL)
    {
      ferr("invalid sb\n");
      return -EINVAL;
    }

  if (directory == MFS_LOCATION_INVALID ||
      directory >= MFS_BLOCK_COUNT(sb) ||
      parent == MFS_LOCATION_INVALID ||
      parent >= MFS_BLOCK_COUNT(sb))
    {
      ferr("invalid directory or parent\n");
      return -EINVAL;
    }

  mfs_dirloc_start(sb, directory, &scanloc);
  while (mfs_dirloc_is_valid(sb, directory, &scanloc))
    {
      ret = mfs_direntry_read(sb, &scanloc, &candidate);
      if (ret < 0)
        {
          ferr("mfs_direntry_read failed: %d\n", ret);
          return ret;
        }

      if (mfs_direntry_is_valid(&candidate) &&
          candidate.oldloc == MFS_LOCATION_INVALID)
        {
          ret = mfs_dir_follow(sb, directory, &scanloc, &candidate,
                               &latest, &latestloc);
          if (ret == OK && S_ISDIR(latest.mode))
            {
              ret = mfs_dirmeta_write(sb, latest.newloc, parent);
              if (ret < 0)
                {
                  ferr("mfs_dirmeta_write failed: %d\n", ret);
                  return ret;
                }
            }
          else if (ret < 0 && ret != -ENOENT)
            {
              ferr("mfs_dir_follow failed: %d\n", ret);
              return ret;
            }
        }

      ret = mfs_dirloc_next(sb, directory, &scanloc);
      if (ret < 0)
        {
          break;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: mfs_dir_refresh_open_files
 *
 * Description:
 * Update open regular-file descriptors after a directory block has been
 * relocated.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   olddir - The original directory block number.
 *   newdir - The replacement directory block number.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned if
 * scanning the replacement directory fails.
 *
 ****************************************************************************/

static int mfs_dir_refresh_open_files(FAR struct mfs_sb_s *sb,
                                      mfs_t olddir, mfs_t newdir)
{
  struct mfs_dirloc_s scanloc;
  struct mfs_direntry_s candidate;
  struct mfs_direntry_s latest;
  struct mfs_dirloc_s latestloc;
  FAR struct list_node *node;
  FAR struct mfs_ofd_s *ofd;
  int ret;

  if (sb == NULL)
    {
      ferr("invalid sb\n");
      return -EINVAL;
    }

  if (olddir == MFS_LOCATION_INVALID || olddir >= MFS_BLOCK_COUNT(sb) ||
      newdir == MFS_LOCATION_INVALID || newdir >= MFS_BLOCK_COUNT(sb))
    {
      ferr("invalid directory block\n");
      return -EINVAL;
    }

  mfs_dirloc_start(sb, newdir, &scanloc);
  while (mfs_dirloc_is_valid(sb, newdir, &scanloc))
    {
      ret = mfs_direntry_read(sb, &scanloc, &candidate);
      if (ret < 0)
        {
          ferr("mfs_direntry_read failed: %d\n", ret);
          return ret;
        }

      if (mfs_direntry_is_valid(&candidate) &&
          candidate.oldloc == MFS_LOCATION_INVALID)
        {
          ret = mfs_dir_follow(sb, newdir, &scanloc, &candidate,
                               &latest, &latestloc);
          if (ret == OK && S_ISREG(latest.mode))
            {
              list_for_every(&sb->ofiles, node)
                {
                  ofd = list_entry(node, struct mfs_ofd_s, entry);
                  if (ofd->common == NULL ||
                      ofd->common->direntloc.page == MFS_LOCATION_INVALID ||
                      mfs_page_to_block(sb,
                                        ofd->common->direntloc.page) !=
                                        olddir ||
                      (ofd->common->fileloc != latest.oldloc &&
                       ofd->common->fileloc != latest.newloc))
                    {
                      continue;
                    }

                  memcpy(&ofd->common->direntry, &latest,
                         sizeof(ofd->common->direntry));
                  memcpy(&ofd->common->direntloc, &latestloc,
                         sizeof(ofd->common->direntloc));
                  ofd->common->fileloc = latest.newloc;
                }
            }
          else if (ret < 0 && ret != -ENOENT)
            {
              ferr("mfs_dir_follow failed: %d\n", ret);
              return ret;
            }
        }

      ret = mfs_dirloc_next(sb, newdir, &scanloc);
      if (ret < 0)
        {
          break;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: mfs_direntry_flush
 *
 * Description:
 * Relocate a full directory block into a freshly allocated block and then
 * append direntry into the new copy.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   directory - The original directory block number.
 *   direntry - The new direntry record to append.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned if
 * allocation, copying, or parent bookkeeping fails.
 *
 ****************************************************************************/

static int mfs_direntry_flush(FAR struct mfs_sb_s *sb, mfs_t directory,
                              FAR const struct mfs_direntry_s *direntry)
{
  struct mfs_dirloc_s scanloc;
  struct mfs_direntry_s candidate;
  struct mfs_direntry_s latest;
  struct mfs_direntry_s copied;
  struct mfs_direntry_s parententry;
  struct mfs_dirloc_s latestloc;
  struct mfs_dirmeta_s dirmeta;
  mfs_t newdir;
  mfs_t parent;
  int ret;
  int cleanup;
  bool moved_root;
  bool children_reparented;

  if (sb == NULL || direntry == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  if (directory == MFS_LOCATION_INVALID ||
      directory >= MFS_BLOCK_COUNT(sb) ||
      !mfs_direntry_is_valid(direntry))
    {
      ferr("invalid directory or direntry\n");
      return -EINVAL;
    }

  moved_root = directory == sb->rootdir;
  if (moved_root)
    {
      parent = directory;
    }
  else
    {
      ret = mfs_dirmeta_read(sb, directory, &dirmeta);
      if (ret < 0)
        {
          ferr("mfs_dirmeta_read failed: %d\n", ret);
          return ret;
        }

      parent = dirmeta.parent;
    }

  ret = mfs_alloc_block(sb, &newdir);
  if (ret < 0)
    {
      ferr("mfs_alloc_block failed: %d\n", ret);
      return ret;
    }

  children_reparented = false;
  if (!moved_root)
    {
      ret = mfs_dirmeta_write(sb, newdir, parent);
      if (ret < 0)
        {
          ferr("mfs_dirmeta_write failed: %d\n", ret);
          goto errout_with_newdir;
        }
    }

  mfs_dirloc_start(sb, directory, &scanloc);
  while (mfs_dirloc_is_valid(sb, directory, &scanloc))
    {
      ret = mfs_direntry_read(sb, &scanloc, &candidate);
      if (ret < 0)
        {
          ferr("mfs_direntry_read failed: %d\n", ret);
          goto errout_with_newdir;
        }

      if (mfs_direntry_is_valid(&candidate) &&
          candidate.oldloc == MFS_LOCATION_INVALID)
        {
          ret = mfs_dir_follow(sb, directory, &scanloc, &candidate,
                               &latest, &latestloc);
          if (ret == OK)
            {
              memcpy(&copied, &latest, sizeof(copied));
              copied.oldloc = MFS_LOCATION_INVALID;
              mfs_direntry_seal(&copied);
              ret = mfs_direntry_add_noflush(sb, newdir, &copied);
              if (ret < 0)
                {
                  ferr("copy direntry add failed: %d\n", ret);
                  goto errout_with_newdir;
                }
            }
          else if (ret != -ENOENT)
            {
              ferr("mfs_dir_follow failed: %d\n", ret);
              goto errout_with_newdir;
            }
        }

      ret = mfs_dirloc_next(sb, directory, &scanloc);
      if (ret < 0)
        {
          break;
        }
    }

  ret = mfs_direntry_add_noflush(sb, newdir, direntry);
  if (ret < 0)
    {
      ferr("append to new directory failed: %d\n", ret);
      goto errout_with_newdir;
    }

  ret = mfs_dir_reparent_children(sb, newdir, newdir);
  if (ret < 0)
    {
      ferr("mfs_dir_reparent_children failed: %d\n", ret);
      cleanup = mfs_dir_reparent_children(sb, newdir, directory);
      if (cleanup < 0)
        {
          ferr("child reparent rollback failed: %d\n", cleanup);
          ret = cleanup;
        }

      goto errout_with_newdir;
    }

  children_reparented = true;
  if (moved_root)
    {
      ret = mfs_update_rootdir(sb, newdir);
      if (ret < 0)
        {
          ferr("mfs_update_rootdir failed: %d\n", ret);
          goto errout_with_children;
        }
    }
  else
    {
      ret = mfs_dir_lookup_location(sb, parent, directory, &parententry,
                                    &latestloc);
      if (ret < 0)
        {
          ferr("mfs_dir_lookup_location failed: %d\n", ret);
          goto errout_with_children;
        }

      parententry.oldloc = directory;
      parententry.newloc = newdir;
      mfs_direntry_seal(&parententry);

      ret = mfs_direntry_add(sb, parent, &parententry);
      if (ret < 0)
        {
          ferr("parent update direntry add failed: %d\n", ret);
          goto errout_with_children;
        }
    }

  ret = mfs_dir_refresh_open_files(sb, directory, newdir);
  if (ret < 0)
    {
      ferr("mfs_dir_refresh_open_files failed: %d\n", ret);
      return ret;
    }

  ret = mfs_report_block_deleted(sb, directory);
  if (ret < 0)
    {
      ferr("mfs_report_block_deleted failed: %d\n", ret);
    }

  return ret;

errout_with_children:
  if (children_reparented)
    {
      cleanup = mfs_dir_reparent_children(sb, newdir, directory);
      if (cleanup < 0)
        {
          ferr("child reparent rollback failed: %d\n", cleanup);
          ret = cleanup;
        }
    }

errout_with_newdir:
  cleanup = mfs_report_block_deleted(sb, newdir);
  if (cleanup < 0)
    {
      ferr("newdir cleanup failed: %d\n", cleanup);
    }

  return cleanup < 0 ? cleanup : ret;
}

/****************************************************************************
 * Name: mfs_direntry_add
 *
 * Description:
 * Append one direntry record to a directory, relocating the directory
 * block if it is already full.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   directory - The directory block number.
 *   direntry - The new direntry record to append.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mfs_direntry_add(FAR struct mfs_sb_s *sb, mfs_t directory,
                            FAR const struct mfs_direntry_s *direntry)
{
  int ret;

  ret = mfs_direntry_add_noflush(sb, directory, direntry);
  if (ret != -ENOSPC)
    {
      if (ret < 0)
        {
          ferr("mfs_direntry_add_noflush failed: %d\n", ret);
        }

      return ret;
    }

  ret = mfs_direntry_flush(sb, directory, direntry);
  if (ret < 0)
    {
      ferr("mfs_direntry_flush failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: mfs_dir_is_empty
 *
 * Description:
 * Determine whether a directory contains any live entry chains.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   directory - The directory block number.
 *
 * Returned Value:
 * Zero (OK) is returned if the directory is empty. -ENOTEMPTY is
 * returned if a live entry exists. A negated errno value is returned on
 * other failures.
 *
 ****************************************************************************/

static int mfs_dir_is_empty(FAR struct mfs_sb_s *sb, mfs_t directory)
{
  struct mfs_dirloc_s scanloc;
  struct mfs_direntry_s candidate;
  struct mfs_direntry_s latest;
  struct mfs_dirloc_s latestloc;
  int ret;

  if (sb == NULL)
    {
      return -EINVAL;
    }

  if (directory == MFS_LOCATION_INVALID || directory >= MFS_BLOCK_COUNT(sb))
    {
      return -EINVAL;
    }

  mfs_dirloc_start(sb, directory, &scanloc);
  while (mfs_dirloc_is_valid(sb, directory, &scanloc))
    {
      ret = mfs_direntry_read(sb, &scanloc, &candidate);
      if (ret < 0)
        {
          return ret;
        }

      if (mfs_direntry_is_valid(&candidate) &&
          candidate.oldloc == MFS_LOCATION_INVALID)
        {
          ret = mfs_dir_follow(sb, directory, &scanloc, &candidate,
                               &latest, &latestloc);
          if (ret == OK)
            {
              return -ENOTEMPTY;
            }

          if (ret != -ENOENT)
            {
              return ret;
            }
        }

      ret = mfs_dirloc_next(sb, directory, &scanloc);
      if (ret < 0)
        {
          break;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mfs_dirent_traverse
 *
 * Description:
 * Resolve a relative path to its latest live direntry and location.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   relpath - The relative path to resolve.
 *   location - The location to receive the resolved object location.
 *   direntry - The location to receive the resolved direntry contents.
 *   dirloc - The location to receive the resolved direntry address.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

int mfs_dirent_traverse(FAR struct mfs_sb_s *sb, FAR const char *relpath,
                        FAR mfs_t *location,
                        FAR struct mfs_direntry_s *direntry,
                        FAR struct mfs_dirloc_s *dirloc)
{
  FAR const char *path;
  FAR const char *next;
  char component[MFS_NAME_LEN];
  mfs_t current;
  int ret;

  if (sb == NULL || location == NULL || direntry == NULL || dirloc == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  memset(component, 0, sizeof(component));
  if (relpath == NULL)
    {
      ferr("relpath is NULL\n");
      return -EINVAL;
    }

  path = mfs_skip_slashes(relpath);
  if (path == NULL || *path == '\0')
    {
      * location = sb->rootdir;
      mfs_root_direntry(sb, direntry);
      mfs_dirloc_invalidate(dirloc);
      return OK;
    }

  current = sb->rootdir;
  for (; ; )
    {
      ret = mfs_next_path_component(path, component, &next);
      if (ret < 0)
        {
          ferr("mfs_next_path_component failed: %d\n", ret);
          return ret;
        }

      if (component[0] == '\0')
        {
          ferr("empty path component\n");
          return -EINVAL;
        }

      ret = mfs_dir_lookup(sb, current, component, &current,
                           direntry, dirloc);
      if (ret < 0)
        {
          if (ret != -ENOENT)
            {
              ferr("mfs_dir_lookup failed: %d\n", ret);
            }

          return ret;
        }

      if (*next == '\0')
        {
          * location = current;
          return OK;
        }

      if (!S_ISDIR(direntry->mode))
        {
          ferr("non-directory in path: %s\n", component);
          return -ENOTDIR;
        }

      path = next;
    }
}

/****************************************************************************
 * Name: mfs_dir_create_file
 *
 * Description:
 * Create a new empty regular file. The function allocates a data page,
 * emits the initial direntry record, and then resolves the freshly
 * created path back to its live state.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   relpath - The relative path of the file to create.
 *   mode - The requested file mode.
 *   location - The location to receive the created file location.
 *   direntry - The location to receive the created direntry contents.
 *   dirloc - The location to receive the created direntry address.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

int mfs_dir_create_file(FAR struct mfs_sb_s *sb, FAR const char *relpath,
                        mode_t mode, FAR mfs_t *location,
                        FAR struct mfs_direntry_s *direntry,
                        FAR struct mfs_dirloc_s *dirloc)
{
  struct mfs_direntry_s newentry;
  struct mfs_direntry_s existing;
  struct mfs_dirloc_s existingloc;
  mfs_t directory;
  mfs_t page;
  int ret;
  int cleanup;

  if (sb == NULL || relpath == NULL || location == NULL ||
      direntry == NULL || dirloc == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  ret = mfs_path_parent_dir(sb, relpath, &directory);
  if (ret < 0)
    {
      ferr("mfs_path_parent_dir failed: %d\n", ret);
      return ret;
    }

  ret = mfs_dir_lookup(sb, directory, mfs_direntry_name(relpath), &page,
                       &existing, &existingloc);
  if (ret == OK)
    {
      ferr("file already exists: %s\n", relpath);
      return -EEXIST;
    }

  if (ret != -ENOENT)
    {
      ferr("mfs_dir_lookup failed: %d\n", ret);
      return ret;
    }

  ret = mfs_direntry_init(&newentry, (mode & ~S_IFMT) | S_IFREG, relpath);
  if (ret < 0)
    {
      ferr("mfs_direntry_init failed: %d\n", ret);
      return ret;
    }

  ret = mfs_alloc_page(sb, &page);
  if (ret < 0)
    {
      ferr("mfs_alloc_page failed: %d\n", ret);
      return ret;
    }

  newentry.newloc    = page;
  newentry.meta.size = 0;
  mfs_direntry_seal(&newentry);

  ret = mfs_direntry_add(sb, directory, &newentry);
  if (ret < 0)
    {
      ferr("mfs_direntry_add failed: %d\n", ret);
      goto errout_with_page;
    }

  ret = mfs_dirent_traverse(sb, relpath, location, direntry, dirloc);
  if (ret < 0)
    {
      ferr("verify traverse failed: %d\n", ret);
      goto errout_with_page;
    }

  finfo("created file %s\n", relpath);
  return OK;

errout_with_page:
  cleanup = mfs_release_page(sb, page);
  if (cleanup < 0)
    {
      ferr("page cleanup failed: %d\n", cleanup);
    }

  return cleanup < 0 ? cleanup : ret;
}

/****************************************************************************
 * Name: mfs_dir_update_file
 *
 * Description:
 * Emit a new direntry record that updates a regular file's stored
 * location and size.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   oldloc - The current file location.
 *   newloc - The revised file location.
 *   dirloc - The latest direntry location for the file.
 *   direntry - The latest direntry contents. Updated in-place on success.
 *   length - The new stored file size.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

int mfs_dir_update_file(FAR struct mfs_sb_s *sb, mfs_t oldloc, mfs_t newloc,
                        FAR const struct mfs_dirloc_s *dirloc,
                        FAR struct mfs_direntry_s *direntry,
                        off_t length)
{
  struct mfs_direntry_s newentry;
  mfs_t directory;
  int ret;

  if (sb == NULL || dirloc == NULL || direntry == NULL || length < 0)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  if (!S_ISREG(direntry->mode))
    {
      ferr("direntry is not regular file\n");
      return -EINVAL;
    }

  if (oldloc == MFS_LOCATION_INVALID || newloc == MFS_LOCATION_INVALID)
    {
      ferr("invalid file location\n");
      return -EINVAL;
    }

  if ((uint64_t)length > UINT32_MAX)
    {
      ferr("file length too large\n");
      return -EFBIG;
    }

  if (oldloc == newloc && direntry->meta.size == (uint32_t)length)
    {
      return OK;
    }

  ret = mfs_parent_dir(sb, oldloc, dirloc, &directory);
  if (ret < 0)
    {
      ferr("mfs_parent_dir failed: %d\n", ret);
      return ret;
    }

  memcpy(&newentry, direntry, sizeof(newentry));
  newentry.oldloc    = oldloc;
  newentry.newloc    = newloc;
  newentry.meta.size = (uint32_t)length;
  mfs_direntry_seal(&newentry);

  ret = mfs_direntry_add(sb, directory, &newentry);
  if (ret < 0)
    {
      ferr("mfs_direntry_add failed: %d\n", ret);
      return ret;
    }

  memcpy(direntry, &newentry, sizeof(*direntry));
  return OK;
}

/****************************************************************************
 * Name: mfs_dir_truncate_file
 *
 * Description:
 * Emit a new direntry record that updates the stored size of a regular
 * file without moving the file's current location.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   location - The current file location.
 *   dirloc - The latest direntry location for the file.
 *   direntry - The latest direntry contents. Updated in-place on success.
 *   length - The new stored file size.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

int mfs_dir_truncate_file(FAR struct mfs_sb_s *sb, mfs_t location,
                          FAR const struct mfs_dirloc_s *dirloc,
                          FAR struct mfs_direntry_s *direntry,
                          off_t length)
{
  return mfs_dir_update_file(sb, location, location, dirloc, direntry,
                             length);
}

/****************************************************************************
 * Name: mfs_dir_opendir
 *
 * Description:
 * Open a directory stream positioned at the first direntry slot in the
 * resolved directory block.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   relpath - The relative path of the directory to open.
 *   dir - The location to receive the opened directory stream.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

int mfs_dir_opendir(FAR struct mfs_sb_s *sb, FAR const char *relpath,
                    FAR struct fs_dirent_s **dir)
{
  FAR struct mfs_dirfs_s *mfsdir;
  struct mfs_dirloc_s dirloc;
  int ret;

  if (sb == NULL || dir == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  mfsdir = fs_heap_zalloc(sizeof(*mfsdir));
  if (mfsdir == NULL)
    {
      ferr("fs_heap_zalloc failed\n");
      return -ENOMEM;
    }

  ret = mfs_dirent_traverse(sb, relpath, &mfsdir->location,
                            &mfsdir->direntry, &dirloc);
  if (ret < 0)
    {
      ferr("mfs_dirent_traverse failed: %d\n", ret);
      goto errout_with_mfsdir;
    }

  if (!S_ISDIR(mfsdir->direntry.mode))
    {
      ferr("path is not directory\n");
      ret = -ENOTDIR;
      goto errout_with_mfsdir;
    }

  mfsdir->dir.fd_path = mfs_dir_pathdup(relpath);
  if (mfsdir->dir.fd_path == NULL)
    {
      ferr("mfs_dir_pathdup failed\n");
      ret = -ENOMEM;
      goto errout_with_mfsdir;
    }

  mfs_dirloc_start(sb, mfsdir->location, &mfsdir->nextloc);
  * dir = &mfsdir->dir;
  return OK;

errout_with_mfsdir:
  fs_heap_free(mfsdir);
  return ret;
}

/****************************************************************************
 * Name: mfs_dir_closedir
 *
 * Description:
 * Release one directory stream previously opened by mfs_dir_opendir().
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   dir - The directory stream to release.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * invalid input.
 *
 ****************************************************************************/

int mfs_dir_closedir(FAR struct mfs_sb_s *sb, FAR struct fs_dirent_s *dir)
{
  if (sb == NULL || dir == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  fs_heap_free(dir);
  return OK;
}

/****************************************************************************
 * Name: mfs_dir_readdir
 *
 * Description:
 * Return the next live directory entry from an open directory stream.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   dir - The directory stream to advance.
 *   entry - The location to receive the translated dirent record.
 *
 * Returned Value:
 * Zero (OK) is returned on success. -ENOENT is returned when the end of
 * the directory is reached. A negated errno value is returned on other
 * failures.
 *
 ****************************************************************************/

int mfs_dir_readdir(FAR struct mfs_sb_s *sb, FAR struct fs_dirent_s *dir,
                    FAR struct dirent *entry)
{
  FAR struct mfs_dirfs_s *mfsdir;
  struct mfs_dirloc_s marked;
  struct mfs_dirloc_s nextloc;
  struct mfs_direntry_s candidate;
  struct mfs_direntry_s latest;
  struct mfs_dirloc_s latestloc;
  int ret;

  if (sb == NULL || dir == NULL || entry == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  mfsdir = (FAR struct mfs_dirfs_s *)dir;
  while (mfs_dirloc_is_valid(sb, mfsdir->location, &mfsdir->nextloc))
    {
      marked = mfsdir->nextloc;
      nextloc = marked;
      ret = mfs_dirloc_next(sb, mfsdir->location, &nextloc);
      if (ret < 0)
        {
          mfs_dirloc_invalidate(&nextloc);
        }

      ret = mfs_direntry_read(sb, &marked, &candidate);
      if (ret < 0)
        {
          ferr("mfs_direntry_read failed: %d\n", ret);
          return ret;
        }

      mfsdir->nextloc = nextloc;
      if (!mfs_direntry_is_valid(&candidate) ||
          candidate.oldloc != MFS_LOCATION_INVALID)
        {
          continue;
        }

      ret = mfs_dir_follow(sb, mfsdir->location, &marked, &candidate,
                           &latest, &latestloc);
      if (ret == -ENOENT)
        {
          continue;
        }

      if (ret < 0)
        {
          ferr("mfs_dir_follow failed: %d\n", ret);
          return ret;
        }

      memcpy(&mfsdir->direntry, &latest, sizeof(mfsdir->direntry));
      mfs_direntry_to_dirent(&latest, entry);
      return OK;
    }

  memset(entry, 0, sizeof(*entry));
  return -ENOENT;
}

/****************************************************************************
 * Name: mfs_dir_rewinddir
 *
 * Description:
 * Reset an open directory stream so the next read begins at the first
 * direntry slot again.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   dir - The directory stream to rewind.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * invalid input.
 *
 ****************************************************************************/

int mfs_dir_rewinddir(FAR struct mfs_sb_s *sb, FAR struct fs_dirent_s *dir)
{
  FAR struct mfs_dirfs_s *mfsdir;

  if (sb == NULL || dir == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  mfsdir = (FAR struct mfs_dirfs_s *)dir;
  mfs_dirloc_start(sb, mfsdir->location, &mfsdir->nextloc);
  return OK;
}

/****************************************************************************
 * Name: mfs_dir_unlink
 *
 * Description:
 * Remove a regular file by appending a tombstone direntry and then
 * reporting the file page as deleted.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   relpath - The relative path of the file to remove.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

int mfs_dir_unlink(FAR struct mfs_sb_s *sb, FAR const char *relpath)
{
  struct mfs_direntry_s direntry;
  struct mfs_dirloc_s dirloc;
  mfs_t directory;
  mfs_t location;
  int ret;

  if (sb == NULL || relpath == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  ret = mfs_dirent_traverse(sb, relpath, &location, &direntry, &dirloc);
  if (ret < 0)
    {
      ferr("mfs_dirent_traverse failed: %d\n", ret);
      return ret;
    }

  if (S_ISDIR(direntry.mode))
    {
      ferr("path is directory: %s\n", relpath);
      return -EISDIR;
    }

  if (mfs_file_is_open(sb, location))
    {
      ferr("file is open: %s\n", relpath);
      return -EBUSY;
    }

  ret = mfs_parent_dir(sb, location, &dirloc, &directory);
  if (ret < 0)
    {
      ferr("mfs_parent_dir failed: %d\n", ret);
      return ret;
    }

  direntry.oldloc = location;
  direntry.newloc = MFS_LOCATION_INVALID;
  mfs_direntry_seal(&direntry);

  ret = mfs_direntry_add(sb, directory, &direntry);
  if (ret < 0)
    {
      ferr("mfs_direntry_add failed: %d\n", ret);
      return ret;
    }

  ret = mfs_report_page_deleted(sb, location);
  if (ret < 0)
    {
      ferr("mfs_report_page_deleted failed: %d\n", ret);
      return ret;
    }

  finfo("removed file %s\n", relpath);
  return OK;
}

/****************************************************************************
 * Name: mfs_dir_mkdir
 *
 * Description:
 * Create a new directory block, initialize its metadata page, and emit
 * the initial direntry record in the parent directory.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   relpath - The relative path of the directory to create.
 *   mode - The requested directory mode.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

int mfs_dir_mkdir(FAR struct mfs_sb_s *sb, FAR const char *relpath,
                  mode_t mode)
{
  struct mfs_direntry_s direntry;
  struct mfs_direntry_s existing;
  struct mfs_dirloc_s existingloc;
  mfs_t directory;
  mfs_t block;
  int ret;
  int cleanup;

  if (sb == NULL || relpath == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  ret = mfs_path_parent_dir(sb, relpath, &directory);
  if (ret < 0)
    {
      ferr("mfs_path_parent_dir failed: %d\n", ret);
      return ret;
    }

  ret = mfs_dir_lookup(sb, directory, mfs_direntry_name(relpath), &block,
                       &existing, &existingloc);
  if (ret == OK)
    {
      ferr("directory already exists: %s\n", relpath);
      return -EEXIST;
    }

  if (ret != -ENOENT)
    {
      ferr("mfs_dir_lookup failed: %d\n", ret);
      return ret;
    }

  ret = mfs_direntry_init(&direntry, (mode & ~S_IFMT) | S_IFDIR, relpath);
  if (ret < 0)
    {
      ferr("mfs_direntry_init failed: %d\n", ret);
      return ret;
    }

  ret = mfs_alloc_block(sb, &block);
  if (ret < 0)
    {
      ferr("mfs_alloc_block failed: %d\n", ret);
      return ret;
    }

  ret = mfs_dirmeta_write(sb, block, directory);
  if (ret < 0)
    {
      ferr("mfs_dirmeta_write failed: %d\n", ret);
      goto errout_with_block;
    }

  direntry.newloc    = block;
  direntry.meta.size = MFS_BLOCK_SIZE(sb);
  mfs_direntry_seal(&direntry);

  ret = mfs_direntry_add(sb, directory, &direntry);
  if (ret < 0)
    {
      ferr("mfs_direntry_add failed: %d\n", ret);
      goto errout_with_block;
    }

  finfo("created dir %s\n", relpath);
  return OK;

errout_with_block:
  cleanup = mfs_report_block_deleted(sb, block);
  if (cleanup < 0)
    {
      ferr("block cleanup failed: %d\n", cleanup);
    }

  return cleanup < 0 ? cleanup : ret;
}

/****************************************************************************
 * Name: mfs_dir_rmdir
 *
 * Description:
 * Remove an empty directory by appending a tombstone direntry and then
 * reporting the directory block as deleted.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   relpath - The relative path of the directory to remove.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

int mfs_dir_rmdir(FAR struct mfs_sb_s *sb, FAR const char *relpath)
{
  struct mfs_direntry_s direntry;
  struct mfs_dirloc_s dirloc;
  mfs_t directory;
  mfs_t location;
  int ret;

  if (sb == NULL || relpath == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  ret = mfs_dirent_traverse(sb, relpath, &location, &direntry, &dirloc);
  if (ret < 0)
    {
      ferr("mfs_dirent_traverse failed: %d\n", ret);
      return ret;
    }

  if (!S_ISDIR(direntry.mode))
    {
      ferr("path is not directory: %s\n", relpath);
      return -ENOTDIR;
    }

  ret = mfs_dir_is_empty(sb, location);
  if (ret < 0)
    {
      ferr("mfs_dir_is_empty failed: %d\n", ret);
      return ret;
    }

  ret = mfs_parent_dir(sb, location, &dirloc, &directory);
  if (ret < 0)
    {
      ferr("mfs_parent_dir failed: %d\n", ret);
      return ret;
    }

  direntry.oldloc = location;
  direntry.newloc = MFS_LOCATION_INVALID;
  mfs_direntry_seal(&direntry);

  ret = mfs_direntry_add(sb, directory, &direntry);
  if (ret < 0)
    {
      ferr("mfs_direntry_add failed: %d\n", ret);
      return ret;
    }

  ret = mfs_report_block_deleted(sb, location);
  if (ret < 0)
    {
      ferr("mfs_report_block_deleted failed: %d\n", ret);
      return ret;
    }

  finfo("removed dir %s\n", relpath);
  return OK;
}

/****************************************************************************
 * Name: mfs_dir_rename
 *
 * Description:
 * Rename a file system object within the same parent directory by
 * appending a new direntry record with the revised name.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   oldrelpath - The current relative path.
 *   newrelpath - The replacement relative path.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

int mfs_dir_rename(FAR struct mfs_sb_s *sb, FAR const char *oldrelpath,
                   FAR const char *newrelpath)
{
  struct mfs_direntry_s direntry;
  struct mfs_dirloc_s dirloc;
  mfs_t directory;
  mfs_t location;
  int ret;

  if (sb == NULL || oldrelpath == NULL || newrelpath == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  ret = mfs_dirent_traverse(sb, oldrelpath, &location, &direntry, &dirloc);
  if (ret < 0)
    {
      ferr("mfs_dirent_traverse failed: %d\n", ret);
      return ret;
    }

  if (S_ISREG(direntry.mode) && mfs_file_is_open(sb, location))
    {
      ferr("file is open: %s\n", oldrelpath);
      return -EBUSY;
    }

  ret = mfs_same_directory(sb, newrelpath, location, &dirloc, &directory);
  if (ret < 0)
    {
      ferr("mfs_same_directory failed: %d\n", ret);
      return ret;
    }

  ret = mfs_direntry_set_name(&direntry, newrelpath);
  if (ret < 0)
    {
      ferr("mfs_direntry_set_name failed: %d\n", ret);
      return ret;
    }

  direntry.oldloc = location;
  direntry.newloc = location;
  mfs_direntry_seal(&direntry);
  ret = mfs_direntry_add(sb, directory, &direntry);
  if (ret < 0)
    {
      ferr("mfs_direntry_add failed: %d\n", ret);
      return ret;
    }

  finfo("renamed %s -> %s\n", oldrelpath, newrelpath);
  return OK;
}

/****************************************************************************
 * Name: mfs_dir_stat
 *
 * Description:
 * Resolve one path and translate its live direntry into a struct stat.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   relpath - The relative path to inspect.
 *   buf - The location to receive the translated stat data.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

int mfs_dir_stat(FAR struct mfs_sb_s *sb, FAR const char *relpath,
                 FAR struct stat *buf)
{
  struct mfs_direntry_s direntry;
  struct mfs_dirloc_s dirloc;
  mfs_t location;
  int ret;

  if (sb == NULL || relpath == NULL || buf == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  ret = mfs_dirent_traverse(sb, relpath, &location, &direntry, &dirloc);
  if (ret < 0)
    {
      ferr("mfs_dirent_traverse failed: %d\n", ret);
      return ret;
    }

  mfs_stat_from_direntry(sb, location, &direntry, buf);
  return OK;
}
