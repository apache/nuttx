/****************************************************************************
 * fs/mnemofs/mnemofs_file.c
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

#include <fcntl.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>

#include <errno.h>
#include <limits.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/param.h>

#include "inode/inode.h"
#include "mnemofs.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static FAR struct mfs_ofd_s *mfs_file_ofd(FAR const struct mfs_sb_s *sb,
                                          FAR const struct file *filep);
static void mfs_file_stat(FAR const struct mfs_sb_s *sb,
                          FAR const struct mfs_ofd_common_s *com,
                          FAR struct stat *buf);
static int mfs_file_bounds_from_size(FAR struct mfs_sb_s *sb, mfs_t filesize,
                                     FAR mfs_t *eofidx,
                                     FAR mfs_t *eofpgoff,
                                     FAR mfs_t *finalidx);
static mfs_t mfs_file_unit_off(FAR struct mfs_sb_s *sb, mfs_t index);
static int mfs_file_unit_page(FAR struct mfs_sb_s *sb, mfs_t headpage,
                              mfs_t headidx, mfs_t targetidx,
                              FAR mfs_t *targetpage);
static int mfs_file_apply_range(FAR struct mfs_sb_s *sb, mfs_t headpage,
                                mfs_t headidx, mfs_t startidx,
                                mfs_t endidx,
                                int (*action)(FAR struct mfs_sb_s *,
                                              mfs_t));
static int mfs_file_reset_empty(FAR struct mfs_sb_s *sb,
                                FAR mfs_t *fileloc,
                                FAR const struct mfs_dirloc_s *dirloc,
                                FAR struct mfs_direntry_s *direntry);
static ssize_t mfs_file_read_from_off(FAR struct mfs_sb_s *sb,
                                      FAR struct mfs_ofd_s *ofd,
                                      mfs_t offset, FAR char *buf,
                                      size_t buflen);
static ssize_t mfs_file_write_from_off(FAR struct mfs_sb_s *sb,
                                       FAR struct mfs_ofd_s *ofd,
                                       mfs_t offset,
                                       FAR const char *buf, size_t buflen,
                                       FAR mfs_t *newloc,
                                       FAR mfs_t *newsize);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mfs_file_ofd
 *
 * Description:
 * Return the open-file descriptor state associated with filep for this
 * mounted file system.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   filep - The VFS file structure to inspect.
 *
 * Returned Value:
 * A pointer to the matching open-file descriptor is returned on success.
 * NULL is returned if no matching descriptor exists.
 *
 ****************************************************************************/

static FAR struct mfs_ofd_s *mfs_file_ofd(FAR const struct mfs_sb_s *sb,
                                          FAR const struct file *filep)
{
  FAR struct mfs_ofd_s *ofd;

  if (sb == NULL || filep == NULL)
    {
      return NULL;
    }

  ofd = filep->f_priv;
  if (ofd == NULL || ofd->sb != sb)
    {
      return NULL;
    }

  return ofd;
}

/****************************************************************************
 * Name: mfs_file_stat
 *
 * Description:
 * Populate a struct stat from one open-file descriptor snapshot.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   com - The shared open-file descriptor state.
 *   buf - The location to receive the translated stat data.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

static void mfs_file_stat(FAR const struct mfs_sb_s *sb,
                          FAR const struct mfs_ofd_common_s *com,
                          FAR struct stat *buf)
{
  memset(buf, 0, sizeof(*buf));
  buf->st_ino         = com->fileloc;
  buf->st_mode        = (mode_t)com->direntry.mode;
  buf->st_nlink       = 1;
  buf->st_size        = com->direntry.meta.size;
  buf->st_atim.tv_sec = com->direntry.mtime;
  buf->st_mtim.tv_sec = com->direntry.mtime;
  buf->st_ctim.tv_sec = com->direntry.ctime;
  buf->st_blksize     = MFS_PAGE_SIZE(sb);
  buf->st_blocks      = (buf->st_size + buf->st_blksize - 1) /
                        buf->st_blksize;
}

/****************************************************************************
 * Name: mfs_file_bounds_from_size
 *
 * Description:
 * Convert a stored file size into CTZ end-of-file indices used by the
 * file read and write paths.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   filesize - The file size to decode.
 *   eofidx - The location to receive the CTZ unit index at EOF.
 *   eofpgoff - The location to receive the byte offset inside that unit.
 *   finalidx - Optional location to receive the final allocated unit index.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mfs_file_bounds_from_size(FAR struct mfs_sb_s *sb, mfs_t filesize,
                                     FAR mfs_t *eofidx,
                                     FAR mfs_t *eofpgoff,
                                     FAR mfs_t *finalidx)
{
  int ret;

  if (sb == NULL || eofidx == NULL || eofpgoff == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  ret = mfs_ctz_index_from_off(sb, filesize, eofidx, eofpgoff);
  if (ret < 0)
    {
      ferr("mfs_ctz_index_from_off failed: %d\n", ret);
      return ret;
    }

  if (finalidx != NULL)
    {
      if (filesize == 0 || *eofpgoff != 0)
        {
          * finalidx = *eofidx;
        }
      else
        {
          DEBUGASSERT(*eofidx > 0);
          * finalidx = *eofidx - 1;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: mfs_file_unit_off
 *
 * Description:
 * Return the logical file offset where CTZ unit index begins.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   index - The CTZ unit index to translate.
 *
 * Returned Value:
 * The logical file offset of the unit is returned.
 *
 ****************************************************************************/

static mfs_t mfs_file_unit_off(FAR struct mfs_sb_s *sb, mfs_t index)
{
  mfs_t off;
  mfs_t idx;
  mfs_t datasz;

  DEBUGASSERT(sb != NULL);

  off = 0;
  for (idx = 0; idx < index; idx++)
    {
      datasz = mfs_ctz_unit_data_area(sb, idx);
      DEBUGASSERT(datasz > 0);
      off += datasz;
    }

  return off;
}

/****************************************************************************
 * Name: mfs_file_unit_page
 *
 * Description:
 * Resolve the page that stores targetidx within one CTZ file chain.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   headpage - The current head page of the file chain.
 *   headidx - The index of headpage.
 *   targetidx - The CTZ unit index to resolve.
 *   targetpage - The location to receive the resolved page.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * invalid input or traversal failure.
 *
 ****************************************************************************/

static int mfs_file_unit_page(FAR struct mfs_sb_s *sb, mfs_t headpage,
                              mfs_t headidx, mfs_t targetidx,
                              FAR mfs_t *targetpage)
{
  int ret;

  if (sb == NULL || targetpage == NULL || headpage == MFS_LOCATION_INVALID ||
      targetidx > headidx)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  ret = mfs_ctz_traverse(sb, headidx, targetidx, headpage, targetpage);
  if (ret < 0)
    {
      ferr("mfs_ctz_traverse failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: mfs_file_apply_range
 *
 * Description:
 * Resolve each CTZ unit in an inclusive index range and apply action to
 * its page.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   headpage - The current head page of the file chain.
 *   headidx - The index of headpage.
 *   startidx - The first CTZ unit index in the range.
 *   endidx - The last CTZ unit index in the range.
 *   action - The callback to invoke for each resolved page.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * invalid input, traversal failure, or callback failure.
 *
 ****************************************************************************/

static int mfs_file_apply_range(FAR struct mfs_sb_s *sb, mfs_t headpage,
                                mfs_t headidx, mfs_t startidx,
                                mfs_t endidx,
                                int (*action)(FAR struct mfs_sb_s *,
                                              mfs_t))
{
  mfs_t idx;
  mfs_t page;
  int ret;

  if (sb == NULL || action == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  if (startidx > endidx)
    {
      return OK;
    }

  if (endidx > headidx)
    {
      ferr("invalid range\n");
      return -EINVAL;
    }

  idx = startidx;
  while (true)
    {
      ret = mfs_file_unit_page(sb, headpage, headidx, idx, &page);
      if (ret < 0)
        {
          ferr("mfs_file_unit_page failed: %d\n", ret);
          return ret;
        }

      ret = action(sb, page);
      if (ret < 0)
        {
          ferr("range action failed: %d\n", ret);
          return ret;
        }

      if (idx == endidx)
        {
          return OK;
        }

      idx++;
    }
}

/****************************************************************************
 * Name: mfs_file_reset_empty
 *
 * Description:
 * Replace one regular file with a fresh empty single-page image and mark
 * the old committed pages obsolete.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   fileloc - The current file head location. Updated on success.
 *   dirloc - The latest known direntry location for the file.
 *   direntry - The latest direntry contents. Updated in-place on success.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mfs_file_reset_empty(FAR struct mfs_sb_s *sb,
                                FAR mfs_t *fileloc,
                                FAR const struct mfs_dirloc_s *dirloc,
                                FAR struct mfs_direntry_s *direntry)
{
  FAR uint8_t *pagebuf;
  mfs_t oldloc;
  mfs_t oldsize;
  mfs_t old_eof_idx;
  mfs_t old_eof_pgoff;
  mfs_t old_final_idx;
  mfs_t newloc;
  ssize_t nwritten;
  int ret;
  int cleanup;

  if (sb == NULL || fileloc == NULL || dirloc == NULL || direntry == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  if (!S_ISREG(direntry->mode))
    {
      ferr("direntry is not regular file\n");
      return -EINVAL;
    }

  oldloc  = *fileloc;
  oldsize = direntry->meta.size;
  if (oldloc == MFS_LOCATION_INVALID)
    {
      ferr("invalid file location\n");
      return -EINVAL;
    }

  if (oldsize == 0)
    {
      return OK;
    }

  ret = mfs_file_bounds_from_size(sb, oldsize, &old_eof_idx, &old_eof_pgoff,
                                  &old_final_idx);
  if (ret < 0)
    {
      ferr("old bounds decode failed: %d\n", ret);
      return ret;
    }

  UNUSED(old_eof_idx);
  UNUSED(old_eof_pgoff);

  pagebuf = kmm_zalloc(MFS_PAGE_SIZE(sb));
  if (pagebuf == NULL)
    {
      ferr("kmm_zalloc failed\n");
      return -ENOMEM;
    }

  newloc = MFS_LOCATION_INVALID;
  ret = mfs_alloc_page(sb, &newloc);
  if (ret < 0)
    {
      ferr("mfs_alloc_page failed: %d\n", ret);
      goto errout_with_buf;
    }

  nwritten = mfs_write_page(sb, newloc, pagebuf);
  if (nwritten < 0)
    {
      ferr("mfs_write_page failed: %zd\n", nwritten);
      ret = nwritten;
      goto errout_with_page;
    }

  if (nwritten != 1)
    {
      ferr("short write: %zd\n", nwritten);
      ret = -EIO;
      goto errout_with_page;
    }

  ret = mfs_dir_update_file(sb, oldloc, newloc, dirloc, direntry, 0);
  if (ret < 0)
    {
      ferr("mfs_dir_update_file failed: %d\n", ret);
      goto errout_with_page;
    }

  * fileloc = newloc;
  kmm_free(pagebuf);

  ret = mfs_file_apply_range(sb, oldloc, old_final_idx, 0, old_final_idx,
                             mfs_report_page_deleted);
  if (ret < 0)
    {
      ferr("old page cleanup failed: %d\n", ret);
    }

  return ret;

errout_with_page:
  cleanup = mfs_release_page(sb, newloc);
  if (cleanup < 0)
    {
      ferr("page cleanup failed: %d\n", cleanup);
      ret = cleanup;
    }

errout_with_buf:
  kmm_free(pagebuf);
  return ret;
}

/****************************************************************************
 * Name: mfs_file_read_from_off
 *
 * Description:
 * Read file data starting at one logical offset without updating the
 * caller's open-file position.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   ofd - The open-file descriptor state.
 *   offset - The logical file offset to start reading from.
 *   buf - The caller's buffer.
 *   buflen - The number of bytes requested.
 *
 * Returned Value:
 * A non-negative byte count is returned on success. A negated errno
 * value is returned on failure.
 *
 ****************************************************************************/

static ssize_t mfs_file_read_from_off(FAR struct mfs_sb_s *sb,
                                      FAR struct mfs_ofd_s *ofd,
                                      mfs_t offset, FAR char *buf,
                                      size_t buflen)
{
  mfs_t filesize;
  mfs_t final_idx;
  mfs_t eof_idx;
  mfs_t eof_pgoff;
  mfs_t s_idx;
  mfs_t s_pgoff;
  mfs_t e_idx;
  mfs_t e_pgoff;
  mfs_t c_idx;
  mfs_t c_pgoff;
  mfs_t c_pg;
  mfs_t readlen;
  mfs_t off;
  mfs_t datasz;
  mfs_t rd_sz;
  ssize_t nread;
  int ret;

  if (sb == NULL || ofd == NULL || ofd->sb != sb || ofd->common == NULL ||
      (buf == NULL && buflen != 0))
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  filesize = ofd->common->direntry.meta.size;
  if (buflen == 0 || offset >= filesize)
    {
      return 0;
    }

  readlen = MIN((mfs_t)buflen, filesize - offset);
  ret = mfs_file_bounds_from_size(sb, filesize, &eof_idx, &eof_pgoff,
                                  &final_idx);
  if (ret < 0)
    {
      ferr("mfs_file_bounds_from_size failed: %d\n", ret);
      return ret;
    }

  ret = mfs_ctz_index_from_off(sb, offset, &s_idx, &s_pgoff);
  if (ret < 0)
    {
      ferr("start offset decode failed: %d\n", ret);
      return ret;
    }

  ret = mfs_ctz_index_from_off(sb, offset + readlen - 1, &e_idx, &e_pgoff);
  if (ret < 0)
    {
      ferr("end offset decode failed: %d\n", ret);
      return ret;
    }

  UNUSED(eof_idx);
  UNUSED(eof_pgoff);
  UNUSED(e_pgoff);

  c_idx   = s_idx;
  c_pgoff = s_pgoff;
  off     = 0;
  while (off < readlen)
    {
      datasz = mfs_ctz_unit_data_area(sb, c_idx);
      if (datasz == 0 || c_pgoff > datasz)
        {
          ferr("invalid data area\n");
          return -EINVAL;
        }

      rd_sz = MIN(datasz - c_pgoff, readlen - off);
      ret = mfs_file_unit_page(sb, ofd->common->fileloc, final_idx, c_idx,
                               &c_pg);
      if (ret < 0)
        {
          ferr("mfs_file_unit_page failed: %d\n", ret);
          return ret;
        }

      nread = mfs_read_page(sb, c_pg, sb->rwbuf);
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

      memcpy(buf + off, sb->rwbuf + c_pgoff, rd_sz);
      off += rd_sz;

      if (c_idx == e_idx)
        {
          break;
        }

      c_idx++;
      c_pgoff = 0;
    }

  return off;
}

/****************************************************************************
 * Name: mfs_file_write_from_off
 *
 * Description:
 * Materialize a rewritten CTZ file image for a write at one logical
 * offset without updating directory state yet.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   ofd - The open-file descriptor state.
 *   offset - The logical file offset to start writing at.
 *   buffer - The source bytes to write.
 *   buflen - The number of bytes to write.
 *   newloc - The location to receive the new file head page.
 *   newsize - The location to receive the resulting file size.
 *
 * Returned Value:
 * A non-negative byte count is returned on success. A negated errno
 * value is returned on failure.
 *
 ****************************************************************************/

static ssize_t mfs_file_write_from_off(FAR struct mfs_sb_s *sb,
                                       FAR struct mfs_ofd_s *ofd,
                                       mfs_t offset,
                                       FAR const char *buffer, size_t buflen,
                                       FAR mfs_t *newloc,
                                       FAR mfs_t *newsize)
{
  FAR uint8_t *pagebuf;
  mfs_t oldsize;
  mfs_t writeend;
  mfs_t filesize;
  mfs_t old_eof_idx;
  mfs_t old_eof_pgoff;
  mfs_t old_final_idx;
  mfs_t new_eof_idx;
  mfs_t new_eof_pgoff;
  mfs_t new_final_idx;
  mfs_t first_new_idx;
  mfs_t s_idx;
  mfs_t s_pgoff;
  mfs_t prev_page;
  mfs_t last_new_page;
  mfs_t last_new_idx;
  mfs_t cur_idx;
  mfs_t cur_page;
  mfs_t cur_old_page;
  mfs_t unit_off;
  mfs_t unit_end;
  mfs_t copy_start;
  mfs_t copy_end;
  mfs_t datasz;
  ssize_t nread;
  ssize_t nwritten;
  int ret;
  int cleanup;

  if (sb == NULL || ofd == NULL || ofd->sb != sb || ofd->common == NULL ||
      newloc == NULL || newsize == NULL ||
      (buffer == NULL && buflen != 0))
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  * newloc = MFS_LOCATION_INVALID;
  * newsize = ofd->common->direntry.meta.size;

  if (buflen == 0)
    {
      * newloc = ofd->common->fileloc;
      * newsize = ofd->common->direntry.meta.size;
      return 0;
    }

  if ((uint64_t)offset + (uint64_t)buflen > UINT32_MAX)
    {
      ferr("write would overflow file size\n");
      return -EFBIG;
    }

  oldsize  = ofd->common->direntry.meta.size;
  writeend = offset + (mfs_t)buflen;
  filesize = MAX(oldsize, writeend);

  ret = mfs_ctz_index_from_off(sb, offset, &s_idx, &s_pgoff);
  if (ret < 0)
    {
      ferr("start offset decode failed: %d\n", ret);
      return ret;
    }

  ret = mfs_file_bounds_from_size(sb, oldsize, &old_eof_idx, &old_eof_pgoff,
                                  &old_final_idx);
  if (ret < 0)
    {
      ferr("old bounds decode failed: %d\n", ret);
      return ret;
    }

  ret = mfs_file_bounds_from_size(sb, filesize, &new_eof_idx, &new_eof_pgoff,
                                  &new_final_idx);
  if (ret < 0)
    {
      ferr("new bounds decode failed: %d\n", ret);
      return ret;
    }

  UNUSED(old_eof_pgoff);
  UNUSED(new_eof_idx);
  UNUSED(new_eof_pgoff);
  UNUSED(s_pgoff);

  first_new_idx = MIN(s_idx, old_eof_idx);
  prev_page     = MFS_LOCATION_INVALID;
  last_new_page = MFS_LOCATION_INVALID;
  last_new_idx  = MFS_LOCATION_INVALID;
  unit_off      = mfs_file_unit_off(sb, first_new_idx);

  if (first_new_idx > 0)
    {
      ret = mfs_file_unit_page(sb, ofd->common->fileloc, old_final_idx,
                               first_new_idx - 1, &prev_page);
      if (ret < 0)
        {
          ferr("mfs_file_unit_page failed: %d\n", ret);
          return ret;
        }
    }

  pagebuf = kmm_malloc(MFS_PAGE_SIZE(sb));
  if (pagebuf == NULL)
    {
      ferr("kmm_malloc failed\n");
      return -ENOMEM;
    }

  cur_idx = first_new_idx;
  while (true)
    {
      datasz = mfs_ctz_unit_data_area(sb, cur_idx);
      if (datasz == 0)
        {
          ferr("invalid data area\n");
          ret = -EINVAL;
          goto errout;
        }

      unit_end = unit_off + datasz;
      memset(pagebuf, 0, MFS_PAGE_SIZE(sb));

      if (cur_idx > 0)
        {
          ret = mfs_ctz_fill_next_ptrs(sb, prev_page, cur_idx - 1, pagebuf);
          if (ret < 0)
            {
              ferr("mfs_ctz_fill_next_ptrs failed: %d\n", ret);
              goto errout;
            }
        }

      if (cur_idx <= old_final_idx)
        {
          ret = mfs_file_unit_page(sb, ofd->common->fileloc, old_final_idx,
                                   cur_idx, &cur_old_page);
          if (ret < 0)
            {
              ferr("mfs_file_unit_page failed: %d\n", ret);
              goto errout;
            }

          nread = mfs_read_page(sb, cur_old_page, sb->rwbuf);
          if (nread < 0)
            {
              ferr("mfs_read_page failed: %zd\n", nread);
              ret = nread;
              goto errout;
            }

          if (nread != 1)
            {
              ferr("short read: %zd\n", nread);
              ret = -EIO;
              goto errout;
            }
        }

      if (cur_idx <= old_final_idx)
        {
          copy_start = unit_off;
          copy_end   = MIN(unit_end, MIN(offset, oldsize));
          if (copy_start < copy_end)
            {
              memcpy(pagebuf + (copy_start - unit_off), sb->rwbuf,
                     copy_end - copy_start);
            }
        }

      copy_start = MAX(unit_off, offset);
      copy_end   = MIN(unit_end, writeend);
      if (copy_start < copy_end)
        {
          memcpy(pagebuf + (copy_start - unit_off),
                 buffer + (copy_start - offset), copy_end - copy_start);
        }

      if (cur_idx <= old_final_idx)
        {
          copy_start = MAX(unit_off, writeend);
          copy_end   = MIN(unit_end, oldsize);
          if (copy_start < copy_end)
            {
              memcpy(pagebuf + (copy_start - unit_off),
                     sb->rwbuf + (copy_start - unit_off),
                     copy_end - copy_start);
            }
        }

      ret = mfs_alloc_page(sb, &cur_page);
      if (ret < 0)
        {
          ferr("mfs_alloc_page failed: %d\n", ret);
          goto errout;
        }

      nwritten = mfs_write_page(sb, cur_page, pagebuf);
      if (nwritten < 0)
        {
          ferr("mfs_write_page failed: %zd\n", nwritten);
          ret = nwritten;
          cleanup = mfs_release_page(sb, cur_page);
          if (cleanup < 0)
            {
              ferr("mfs_release_page failed: %d\n", cleanup);
              ret = cleanup;
            }

          goto errout;
        }

      if (nwritten != 1)
        {
          ferr("short write: %zd\n", nwritten);
          ret = -EIO;
          cleanup = mfs_release_page(sb, cur_page);
          if (cleanup < 0)
            {
              ferr("mfs_release_page failed: %d\n", cleanup);
              ret = cleanup;
            }

          goto errout;
        }

      prev_page     = cur_page;
      last_new_page = cur_page;
      last_new_idx  = cur_idx;

      if (cur_idx == new_final_idx)
        {
          break;
        }

      unit_off = unit_end;
      cur_idx++;
    }

  kmm_free(pagebuf);
  * newloc = last_new_page;
  * newsize = filesize;
  return (ssize_t)buflen;

errout:
  kmm_free(pagebuf);
  if (last_new_page != MFS_LOCATION_INVALID)
    {
      cleanup = mfs_file_apply_range(sb, last_new_page, last_new_idx,
                                     first_new_idx, last_new_idx,
                                     mfs_release_page);
      if (cleanup < 0)
        {
          ferr("rollback release failed: %d\n", cleanup);
          return cleanup;
        }
    }

  ferr("failed: %d\n", ret);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mfs_file_is_open
 *
 * Description:
 * Check whether a live file location already has an open descriptor.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   fileloc - The file location to test.
 *
 * Returned Value:
 * true is returned if the file is already open. false is returned
 * otherwise.
 *
 ****************************************************************************/

bool mfs_file_is_open(FAR const struct mfs_sb_s *sb, mfs_t fileloc)
{
  FAR struct list_node *node;
  FAR struct mfs_ofd_s *ofd;

  if (sb == NULL || fileloc == MFS_LOCATION_INVALID)
    {
      return false;
    }

  list_for_every(&sb->ofiles, node)
    {
      ofd = list_entry(node, struct mfs_ofd_s, entry);
      if (ofd->common != NULL && ofd->common->fileloc == fileloc)
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: mfs_file_open
 *
 * Description:
 * Open a regular file. If O_CREAT is present and the path is absent,
 * then an initial direntry record and page allocation are performed
 * before the open-file state is created.
 *
 * Input Parameters:
 *   filep - The file structure to initialize.
 *   relpath - The relative path of the file.
 *   oflags - Open flags.
 *   mode - Create mode used with O_CREAT.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

int mfs_file_open(FAR struct mfs_sb_s *sb, FAR struct file *filep,
                  FAR const char *relpath, int oflags, mode_t mode)
{
  FAR struct mfs_ofd_s *ofd;
  FAR struct mfs_ofd_common_s *common;
  FAR char *pathdup;
  struct mfs_direntry_s direntry;
  struct mfs_dirloc_s dirloc;
  mfs_t location;
  int ret;

  if (sb == NULL || filep == NULL || relpath == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  if ((oflags & O_CREAT) != 0)
    {
      ret = mfs_dirent_traverse(sb, relpath, &location, &direntry, &dirloc);
      if (ret == OK)
        {
          if (!S_ISREG(direntry.mode))
            {
              ferr("path is directory: %s\n", relpath);
              return -EISDIR;
            }

          if ((oflags & O_EXCL) != 0)
            {
              ferr("file exists with O_EXCL: %s\n", relpath);
              return -EEXIST;
            }
        }
      else
        {
          if (ret != -ENOENT)
            {
              ferr("mfs_dirent_traverse failed: %d\n", ret);
              return ret;
            }

          if ((mode & S_IFMT) == S_IFDIR)
            {
              ferr("create requested directory mode: %s\n", relpath);
              return -EISDIR;
            }

          ret = mfs_dir_create_file(sb, relpath, mode, &location,
                                    &direntry, &dirloc);
          if (ret < 0)
            {
              ferr("mfs_dir_create_file failed: %d\n", ret);
              return ret;
            }
        }
    }
  else
    {
      ret = mfs_dirent_traverse(sb, relpath, &location, &direntry, &dirloc);
      if (ret < 0)
        {
          ferr("mfs_dirent_traverse failed: %d\n", ret);
          return ret;
        }

      if (!S_ISREG(direntry.mode))
        {
          ferr("path is directory: %s\n", relpath);
          return -EISDIR;
        }
    }

  common = kmm_zalloc(sizeof(*common));
  if (common == NULL)
    {
      ferr("common allocation failed\n");
      return -ENOMEM;
    }

  ofd = kmm_zalloc(sizeof(*ofd));
  if (ofd == NULL)
    {
      ferr("ofd allocation failed\n");
      ret = -ENOMEM;
      goto errout_with_common;
    }

  pathdup = kmm_malloc(strlen(relpath) + 1);
  if (pathdup == NULL)
    {
      ferr("path allocation failed\n");
      ret = -ENOMEM;
      goto errout_with_ofd;
    }

  strlcpy(pathdup, relpath, strlen(relpath) + 1);

  if ((oflags & O_TRUNC) != 0 && (oflags & O_ACCMODE) != O_RDONLY)
    {
      ret = mfs_file_reset_empty(sb, &location, &dirloc, &direntry);
      if (ret < 0)
        {
          ferr("mfs_file_reset_empty failed: %d\n", ret);
          goto errout_with_pathdup;
        }
    }

  memcpy(&common->direntry, &direntry, sizeof(common->direntry));
  memcpy(&common->direntloc, &dirloc, sizeof(common->direntloc));
  common->relpath = pathdup;
  common->fileloc = location;
  common->refs    = 1;

  ofd->sb     = sb;
  ofd->common = common;
  list_clear_node(&ofd->entry);
  list_add_tail(&sb->ofiles, &ofd->entry);
  filep->f_priv = ofd;
  filep->f_pos  = 0;
  return OK;

errout_with_pathdup:
  kmm_free(pathdup);
errout_with_ofd:
  kmm_free(ofd);
errout_with_common:
  kmm_free(common);
  return ret;
}

/****************************************************************************
 * Name: mfs_file_close
 *
 * Description:
 * Close one open-file descriptor, releasing shared state when the last
 * reference disappears.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   filep - The VFS file structure to close.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * invalid input.
 *
 ****************************************************************************/

int mfs_file_close(FAR struct mfs_sb_s *sb, FAR struct file *filep)
{
  FAR struct mfs_ofd_s *ofd;

  ofd = mfs_file_ofd(sb, filep);
  if (ofd == NULL || ofd->common == NULL)
    {
      ferr("invalid file handle\n");
      return -EINVAL;
    }

  list_delete(&ofd->entry);
  ofd->common->refs--;
  if (ofd->common->refs == 0)
    {
      kmm_free(ofd->common->relpath);
      kmm_free(ofd->common);
    }

  kmm_free(ofd);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: mfs_file_read
 *
 * Description:
 * Read from an open regular file at its current file position and advance
 * that position by the number of bytes read.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   filep - The VFS file structure to read from.
 *   buffer - The caller's read buffer.
 *   buflen - The number of bytes requested.
 *
 * Returned Value:
 * A non-negative byte count is returned on success. A negated errno
 * value is returned on failure.
 *
 ****************************************************************************/

ssize_t mfs_file_read(FAR struct mfs_sb_s *sb, FAR struct file *filep,
                      FAR char *buffer, size_t buflen)
{
  FAR struct mfs_ofd_s *ofd;
  ssize_t ret;

  ofd = mfs_file_ofd(sb, filep);
  if (ofd == NULL || ofd->common == NULL || (buffer == NULL && buflen != 0))
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  if (ofd->common->pos < 0)
    {
      ferr("negative file position\n");
      return -EINVAL;
    }

  if ((uint64_t)ofd->common->pos > UINT32_MAX)
    {
      return 0;
    }

  ret = mfs_file_read_from_off(sb, ofd, (mfs_t)ofd->common->pos, buffer,
                               buflen);
  if (ret < 0)
    {
      ferr("mfs_file_read_from_off failed: %zd\n", ret);
    }

  if (ret > 0)
    {
      ofd->common->pos += ret;
      filep->f_pos = ofd->common->pos;
    }

  return ret;
}

/****************************************************************************
 * Name: mfs_file_write
 *
 * Description:
 * Write to an open regular file, update directory state to the new CTZ
 * chain, and advance the current file position.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   filep - The VFS file structure to write to.
 *   buffer - The source bytes to write.
 *   buflen - The number of bytes to write.
 *
 * Returned Value:
 * A non-negative byte count is returned on success. A negated errno
 * value is returned on failure.
 *
 ****************************************************************************/

ssize_t mfs_file_write(FAR struct mfs_sb_s *sb, FAR struct file *filep,
                       FAR const char *buffer, size_t buflen)
{
  FAR struct mfs_ofd_s *ofd;
  off_t offset;
  mfs_t writeoff;
  mfs_t newloc;
  mfs_t newsize;
  mfs_t oldloc;
  mfs_t oldsize;
  mfs_t s_idx;
  mfs_t s_pgoff;
  mfs_t old_eof_idx;
  mfs_t old_eof_pgoff;
  mfs_t old_final_idx;
  mfs_t new_eof_idx;
  mfs_t new_eof_pgoff;
  mfs_t new_final_idx;
  mfs_t first_new_idx;
  int cleanupret;
  int updateret;
  ssize_t ret;

  ofd = mfs_file_ofd(sb, filep);
  if (ofd == NULL || ofd->common == NULL || (buffer == NULL && buflen != 0))
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  offset = ofd->common->pos;
  if ((filep->f_oflags & O_APPEND) != 0)
    {
      offset = ofd->common->direntry.meta.size;
    }

  if (offset < 0)
    {
      ferr("negative file position\n");
      return -EINVAL;
    }

  if ((uint64_t)offset > UINT32_MAX)
    {
      ferr("file offset too large\n");
      return -EFBIG;
    }

  writeoff = (mfs_t)offset;
  oldloc = ofd->common->fileloc;
  oldsize = ofd->common->direntry.meta.size;

  ret = mfs_file_write_from_off(sb, ofd, writeoff, buffer, buflen, &newloc,
                                &newsize);
  if (ret <= 0)
    {
      if (ret < 0)
        {
          ferr("mfs_file_write_from_off failed: %zd\n", ret);
        }

      return ret;
    }

  if (newloc == MFS_LOCATION_INVALID)
    {
      ferr("write returned invalid new location\n");
      ret = -EIO;
      goto errout;
    }

  updateret = mfs_dir_update_file(sb, oldloc, newloc,
                                  &ofd->common->direntloc,
                                  &ofd->common->direntry, (off_t)newsize);
  if (updateret < 0)
    {
      ferr("mfs_dir_update_file failed: %d\n", updateret);
      ret = updateret;
      goto errout_with_newpages;
    }

  ofd->common->fileloc = newloc;
  ofd->common->pos = offset + ret;
  filep->f_pos = ofd->common->pos;

  cleanupret = mfs_ctz_index_from_off(sb, writeoff, &s_idx, &s_pgoff);
  if (cleanupret < 0)
    {
      ferr("start offset decode failed: %d\n", cleanupret);
      ret = cleanupret;
      goto errout;
    }

  cleanupret = mfs_file_bounds_from_size(sb, oldsize, &old_eof_idx,
                                         &old_eof_pgoff, &old_final_idx);
  if (cleanupret < 0)
    {
      ferr("old bounds decode failed: %d\n", cleanupret);
      ret = cleanupret;
      goto errout;
    }

  UNUSED(s_pgoff);
  UNUSED(old_eof_pgoff);
  first_new_idx = MIN(s_idx, old_eof_idx);
  cleanupret = mfs_file_apply_range(sb, oldloc, old_final_idx,
                                    first_new_idx, old_final_idx,
                                    mfs_report_page_deleted);
  if (cleanupret < 0)
    {
      ferr("old page cleanup failed: %d\n", cleanupret);
      ret = cleanupret;
    }

  return ret;

errout_with_newpages:
  cleanupret = mfs_ctz_index_from_off(sb, writeoff, &s_idx, &s_pgoff);
  if (cleanupret >= 0)
    {
      cleanupret = mfs_file_bounds_from_size(sb, newsize,
                                             &new_eof_idx,
                                             &new_eof_pgoff,
                                             &new_final_idx);
    }

  if (cleanupret >= 0)
    {
      cleanupret = mfs_file_bounds_from_size(sb, oldsize,
                                             &old_eof_idx,
                                             &old_eof_pgoff,
                                             &old_final_idx);
    }

  if (cleanupret >= 0)
    {
      UNUSED(s_pgoff);
      UNUSED(old_eof_pgoff);
      UNUSED(new_eof_idx);
      UNUSED(new_eof_pgoff);
      first_new_idx = MIN(s_idx, old_eof_idx);
      cleanupret = mfs_file_apply_range(sb, newloc, new_final_idx,
                                        first_new_idx, new_final_idx,
                                        mfs_release_page);
    }

  if (cleanupret < 0)
    {
      ferr("rollback cleanup failed: %d\n", cleanupret);
    }

  return cleanupret < 0 ? cleanupret : ret;

errout:
  ferr("failed: %zd\n", ret);
  return ret;
}

/****************************************************************************
 * Name: mfs_file_seek
 *
 * Description:
 * Update the current logical file position for an open file.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   filep - The VFS file structure to reposition.
 *   offset - The seek offset argument.
 *   whence - The seek base selector.
 *
 * Returned Value:
 * The new file position is returned on success. A negated errno value is
 * returned on failure.
 *
 ****************************************************************************/

off_t mfs_file_seek(FAR struct mfs_sb_s *sb, FAR struct file *filep,
                    off_t offset, int whence)
{
  FAR struct mfs_ofd_s *ofd;
  off_t nextpos;

  ofd = mfs_file_ofd(sb, filep);
  if (ofd == NULL || ofd->common == NULL)
    {
      ferr("invalid file handle\n");
      return -EINVAL;
    }

  if (whence == SEEK_SET)
    {
      nextpos = offset;
    }
  else if (whence == SEEK_CUR)
    {
      nextpos = ofd->common->pos + offset;
    }
  else if (whence == SEEK_END)
    {
      nextpos = ofd->common->direntry.meta.size + offset;
    }
  else
    {
      ferr("invalid whence\n");
      return -EINVAL;
    }

  if (nextpos < 0)
    {
      ferr("negative target position\n");
      return -EINVAL;
    }

  ofd->common->pos = nextpos;
  filep->f_pos = nextpos;
  return nextpos;
}

/****************************************************************************
 * Name: mfs_file_ioctl
 *
 * Description:
 * Handle file-specific ioctl requests for an open mnemofs file.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   filep - The VFS file structure to operate on.
 *   cmd - The ioctl command.
 *   arg - The ioctl argument.
 *
 * Returned Value:
 * Zero (OK) or another command-specific value is returned on success. A
 * negated errno value is returned on failure.
 *
 ****************************************************************************/

int mfs_file_ioctl(FAR struct mfs_sb_s *sb, FAR struct file *filep,
                   int cmd, unsigned long arg)
{
  FAR struct mfs_ofd_s *ofd;
  FAR char *path;
  size_t len;
  int ret;

  ofd = mfs_file_ofd(sb, filep);
  if (ofd == NULL || ofd->common == NULL)
    {
      ferr("invalid file handle\n");
      return -EINVAL;
    }

  switch (cmd)
    {
      case FIOC_FILEPATH:
        {
          path = (FAR char *)(uintptr_t)arg;
          if (path == NULL || ofd->common->relpath == NULL)
            {
              ferr("invalid filepath buffer\n");
              return -EINVAL;
            }

          ret = inode_getpath(filep->f_inode, path, PATH_MAX);
          if (ret < 0)
            {
              ferr("inode_getpath failed: %d\n", ret);
              return ret;
            }

          len = strlen(path);
          if (len > 0 && path[len - 1] != '/')
            {
              path[len++] = '/';
              path[len]   = '\0';
            }

          if (strlcat(path, ofd->common->relpath, PATH_MAX) >= PATH_MAX)
            {
              ferr("path too long\n");
              return -ENAMETOOLONG;
            }

          return OK;
        }

      case BIOC_FLUSH:
        {
          if (sb->mtd == NULL)
            {
              ferr("missing mtd\n");
              return -ENODEV;
            }

          ret = MTD_IOCTL(sb->mtd, BIOC_FLUSH, arg);
          if (ret < 0)
            {
              ferr("BIOC_FLUSH failed: %d\n", ret);
            }

          return ret;
        }

      default:
        {
          ferr("unsupported ioctl: %d\n", cmd);
          return -ENOTTY;
        }
    }
}

/****************************************************************************
 * Name: mfs_file_truncate
 *
 * Description:
 * Update a file's stored length by emitting a new direntry record that
 * points to the same file location with a different size.
 *
 * Input Parameters:
 *   filep - The open file instance.
 *   length - The new stored file size.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

int mfs_file_truncate(FAR struct mfs_sb_s *sb, FAR struct file *filep,
                      off_t length)
{
  FAR struct mfs_ofd_s *ofd;
  int ret;

  ofd = mfs_file_ofd(sb, filep);
  if (ofd == NULL || ofd->common == NULL || length < 0)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  if (length == 0)
    {
      ret = mfs_file_reset_empty(sb, &ofd->common->fileloc,
                                 &ofd->common->direntloc,
                                 &ofd->common->direntry);
    }
  else
    {
      ret = mfs_dir_truncate_file(sb, ofd->common->fileloc,
                                  &ofd->common->direntloc,
                                  &ofd->common->direntry, length);
    }

  if (ret < 0)
    {
      ferr("truncate failed: %d\n", ret);
      return ret;
    }

  if (ofd->common->pos > length)
    {
      ofd->common->pos = length;
      filep->f_pos = length;
    }

  return OK;
}

/****************************************************************************
 * Name: mfs_file_sync
 *
 * Description:
 * Validate one open-file descriptor for sync. mnemofs updates metadata
 * eagerly, so no extra flush work is needed here.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   filep - The VFS file structure to sync.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * invalid input.
 *
 ****************************************************************************/

int mfs_file_sync(FAR struct mfs_sb_s *sb, FAR struct file *filep)
{
  FAR struct mfs_ofd_s *ofd;

  ofd = mfs_file_ofd(sb, filep);
  if (ofd == NULL || ofd->common == NULL)
    {
      ferr("invalid file handle\n");
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: mfs_file_dup
 *
 * Description:
 * Duplicate one open-file descriptor so both VFS file structures share
 * the same underlying open-file state.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   oldp - The existing VFS file structure.
 *   newp - The new VFS file structure to initialize.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

int mfs_file_dup(FAR struct mfs_sb_s *sb, FAR const struct file *oldp,
                 FAR struct file *newp)
{
  FAR struct mfs_ofd_s *oldofd;
  FAR struct mfs_ofd_s *newofd;

  oldofd = mfs_file_ofd(sb, oldp);
  if (oldofd == NULL || oldofd->common == NULL || newp == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  newofd = kmm_zalloc(sizeof(*newofd));
  if (newofd == NULL)
    {
      ferr("ofd allocation failed\n");
      return -ENOMEM;
    }

  newofd->sb     = oldofd->sb;
  newofd->common = oldofd->common;
  newofd->common->refs++;
  list_clear_node(&newofd->entry);
  list_add_tail(&newofd->sb->ofiles, &newofd->entry);
  newp->f_priv = newofd;
  newp->f_pos  = newofd->common->pos;
  return OK;
}

/****************************************************************************
 * Name: mfs_file_fstat
 *
 * Description:
 * Return file status information for one open regular file.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   filep - The VFS file structure to inspect.
 *   buf - The location to receive the translated stat data.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

int mfs_file_fstat(FAR struct mfs_sb_s *sb, FAR const struct file *filep,
                   FAR struct stat *buf)
{
  FAR struct mfs_ofd_s *ofd;

  ofd = mfs_file_ofd(sb, filep);
  if (ofd == NULL || ofd->common == NULL || buf == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  mfs_file_stat(sb, ofd->common, buf);
  return OK;
}
