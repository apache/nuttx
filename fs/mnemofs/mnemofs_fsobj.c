/****************************************************************************
 * fs/mnemofs/mnemofs_fsobj.c
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
 * In mnemofs, all the FS object methods (ie. methods in this file),
 * interface directly with the LRU. To these methods, only the methods
 * exposed by the LRU are visible, nothing else. The LRU will give them the
 * most updated data, which includes data from the flash, the updates from
 * the journal and the LRU deltas as well.
 *
 * TODO: The above menetioned concept.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/kmalloc.h>
#include <sys/stat.h>

#include "mnemofs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static mfs_t      nobjs_in_path(FAR const char * relpath);
static const char *next_child(FAR const char *relpath);
static FAR char   *mfs_ser_dirent(FAR const struct mfs_dirent_s * const x,
                                  FAR char * const out);
static FAR const char *mfs_deser_dirent(FAR const char * const in,
                                        FAR struct mfs_dirent_s * const x);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nobjs_in_path
 *
 * Description:
 *   Count number of file system objects in path. This includes root in the
 *   count.
 *
 * Input Parameters:
 *   relpath   - Relative Path.
 *
 * Returned Value:
 *   The number of file system objects in the path including the root.
 *
 ****************************************************************************/

static mfs_t nobjs_in_path(FAR const char *relpath)
{
  mfs_t count;

  if (*relpath == 0)
    {
      return 1;
    }

  count = 2;

  while (*relpath == 0)
    {
      if (*relpath == '/')
        {
          count++;
        }

      relpath++;
    }

  return count;
}

/****************************************************************************
 * Name: nobjs_in_path
 *
 * Description:
 *   Give the pointer to next child that appears in the path.
 *
 * Input Parameters:
 *   relpath   - Relative Path.
 *
 * Returned Value:
 *   The pointer to the next child. This is not allocated, but points to the
 *   inside relpath itself.
 *
 ****************************************************************************/

static const char *next_child(FAR const char *relpath)
{
  while (*relpath != 0)
    {
      if (*relpath == '/')
        {
          return relpath + 1;
        }

      relpath++;
    }

  return NULL;
}

/****************************************************************************
 * Name: mfs_ser_dirent
 *
 * Description:
 *   Serialize a direntry.
 *
 * Input Parameters:
 *   x    - Direntry.
 *   out  - Buffer to populate.
 *
 * Returned Value:
 *   Pointer to after the end of serialized content in out.
 *
 ****************************************************************************/

static FAR char *mfs_ser_dirent(FAR const struct mfs_dirent_s * const x,
                                FAR char * const out)
{
  FAR char *o = out;

  o = mfs_ser_8(x->name_hash, o);
  o = mfs_ser_mfs(x->sz, o);
  o = mfs_ser_mfs((mfs_t) x->mode, o);
  o = mfs_ser_timespec(&x->st_atim, o);
  o = mfs_ser_timespec(&x->st_mtim, o);
  o = mfs_ser_timespec(&x->st_ctim, o);
  o = mfs_ser_ctz(&x->ctz, o);
  o = mfs_ser_8(x->namelen, o);
  o = mfs_ser_str(x->name, x->namelen, o);

  return o;
}

/****************************************************************************
 * Name: mfs_deser_dirent
 *
 * Description:
 *   Deserialize a direntry.
 *
 * Input Parameters:
 *   in  - Buffer.
 *   x   - Direntry to populate.
 *
 * Returned Value:
 *   Pointer to after the end of deserialized content in in.
 *
 ****************************************************************************/

static FAR const char *mfs_deser_dirent(FAR const char * const in,
                                        FAR struct mfs_dirent_s * const x)
{
  FAR const char *i = in;

  i = mfs_deser_8(i, &x->name_hash);
  i = mfs_deser_mfs(i, &x->sz);
  i = mfs_deser_mfs(i, (mfs_t *) &x->mode);
  i = mfs_deser_timespec(i, &x->st_atim);
  i = mfs_deser_timespec(i, &x->st_mtim);
  i = mfs_deser_timespec(i, &x->st_ctim);
  i = mfs_deser_ctz(i, &x->ctz);
  i = mfs_deser_8(i, &x->namelen);
  i = mfs_deser_str(i, x->name, x->namelen);

  return i;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR const char * mfs_path2childname(FAR const char *relpath)
{
  FAR const char *last = relpath + strlen(relpath) - 1;

  while (last >= relpath && *last != '/')
    {
      last--;
    }

  return last + 1;
}

mfs_t mfs_get_fsz(FAR struct mfs_sb_s * const sb,
                  FAR const struct mfs_path_s * const path,
                  const mfs_t depth)
{
  mfs_t sz;

  if (depth == 0)
    {
      /* Master node. */

      return 0;
    }
  else if (depth == 1)
    {
      sz = MFS_MN(sb).root_sz;
      mfs_lru_updatedsz(sb, path, depth, &sz);

/* Journal updated to the root creates a new master node entry. TODO
 * this and moving of the journal.
 */

      finfo("File size got as %u for root.", sz);
      return sz;
    }

  return path[depth - 1].sz;
}

int mfs_get_patharr(FAR struct mfs_sb_s *const sb,
                    FAR const char *relpath, FAR struct mfs_path_s **path,
                    FAR mfs_t *depth)
{
  int                     ret;
  bool                    found;
  bool                    isfile;
  mfs_t                   sz;
  mfs_t                   idx;
  uint8_t                 hash;
  const mfs_t             len     = nobjs_in_path(relpath);
  FAR const char          *next;
  struct mfs_pitr_s       pitr;
  FAR struct mfs_dirent_s *dirent;

  ret     = 0;
  idx     = 0;
  isfile  = false;
  found   = true;

  *path = kmm_zalloc(len * sizeof(struct mfs_ctz_s));
  if (predict_false(*path == NULL))
    {
      return -ENOMEM;
    }

  *depth           = len;
  (*path)[idx].ctz = MFS_MN(sb).root_ctz;
  (*path)[idx].off = 0;
  (*path)[idx].sz  = mfs_get_fsz(sb, *path, 1);

  finfo("Path \"%s\" depth %u.", relpath, len);

  if (len == 1)
    {
      ret |= MFS_P_EXIST | MFS_P_ISDIR | MFS_ISDIR;
      goto errout;
    }
  else if (len == 2)
    {
      ret |= MFS_P_EXIST | MFS_P_ISDIR;
    }

  for (idx = 1; idx < len; idx++)
    {
      mfs_pitr_init(sb, *path, idx - 1, &pitr, false);

      if (predict_false(idx == 1))
        {
          next = relpath;
        }
      else
        {
          next  = next_child(relpath);
        }

      sz    = next - relpath;
      hash  = mfs_arrhash(relpath, sz);

      for (; ; )
        {
          mfs_pitr_readdirent(sb, &pitr, &dirent);
          if (predict_false(dirent == NULL))
            {
              found = false;
              ret |= MFS_NEXIST;
              break;
            }

          if (dirent->name_hash == hash &&
              !strncmp(dirent->name, relpath, sz))
            {
              (*path)[idx - 1].ctz = dirent->ctz;
              (*path)[idx - 1].off = pitr.c_off;
              (*path)[idx - 1].sz  = dirent->sz;

              mfs_free_dirent(dirent);

              if (len >= 2 && idx == len - 2)
                {
                  ret |= MFS_P_EXIST;
                  ret |= (S_ISDIR(dirent->mode) ? MFS_P_ISDIR :
                                                (MFS_FINPATH | MFS_NEXIST));
                }
              else if (idx == len - 1)
                {
                  ret |= (S_ISDIR(dirent->mode) ? MFS_ISDIR : MFS_ISFILE);
                }
              else
                {
                  ret |= (S_ISDIR(dirent->mode) ? 0 :
                                                (MFS_FINPATH | MFS_NEXIST));
                }

              break;
            }

          if (!S_ISDIR(dirent->mode))
            {
              isfile = true;
            }

          mfs_pitr_adv_dirent(&pitr, dirent);

          mfs_free_dirent(dirent);

          if (isfile)
            {
              /* At max, only the last element is allowed to be a file. */

              break;
            }
        }

      mfs_pitr_free(&pitr);

      if (!found)
        {
          break;
        }

      relpath = next;
      finfo("Next path \"%s\"", relpath);
    }

errout:
  finfo("Path array for \"%s\", returned flags: %u.", relpath, ret);
  return ret;
}

void mfs_free_patharr(FAR struct mfs_path_s *path)
{
  kmm_free(path);
}

bool mfs_obj_isempty(FAR struct mfs_sb_s * const sb,
                     FAR struct mfs_pitr_s * const pitr)
{
  FAR struct mfs_dirent_s *dirent = NULL;
  bool ret;

  mfs_pitr_readdirent(sb, pitr, &dirent);
  ret = (dirent->sz == 0);
  mfs_free_dirent(dirent);

  return ret;
}

void mfs_pitr_init(FAR struct mfs_sb_s * const sb,
                   FAR const struct mfs_path_s * const path,
                   const mfs_t depth, FAR struct mfs_pitr_s *pitr,
                   bool child)
{
  const uint8_t diff = child ? 1 : 0;
  mfs_t p_depth;

  if (predict_false(depth < diff))
    {
      return;
    }

  p_depth = depth - diff;

  if (p_depth == 0)
    {
      /* Master Node */

      pitr->sz = 0;
      memset(&pitr->p, 0, sizeof(pitr->p));
    }
  else
    {
      pitr->p.ctz = path[p_depth - 1].ctz;
      pitr->sz    = path[p_depth - 1].sz;
    }

  pitr->depth = p_depth;
  pitr->c_off  = 0;

  finfo("Pitr initialized for parent CTZ at (%u, %u) at"
        " depth %u for file size %u.",
        pitr->p.ctz.idx_e, pitr->p.ctz.pg_e, pitr->depth, pitr->sz);
}

void mfs_pitr_free(FAR struct mfs_pitr_s * const pitr)
{
  finfo("Pitr for CTZ (%u, %u) at depth %u freed.",
        pitr->p.ctz.idx_e, pitr->p.ctz.pg_e, pitr->depth);

  memset(pitr, 0, sizeof(*pitr));
}

void mfs_pitr_adv(FAR struct mfs_sb_s * const sb,
                  FAR struct mfs_pitr_s * const pitr)
{
  FAR struct mfs_dirent_s *dirent;

  mfs_pitr_readdirent(sb, pitr, &dirent);
  mfs_pitr_adv_dirent(pitr, dirent);
  mfs_free_dirent(dirent);

  finfo("Pitr for CTZ (%u, %u) advanced to offset %u.",
        pitr->p.ctz.idx_e, pitr->p.ctz.pg_e, pitr->c_off);
}

void mfs_pitr_adv_dirent(FAR struct mfs_pitr_s * const pitr,
                         FAR const struct mfs_dirent_s * const dirent)
{
  pitr->c_off += MFS_DIRENTSZ(dirent);

  finfo("Pitr for CTZ (%u, %u) advanced to offset %u using dirent.",
        pitr->p.ctz.idx_e, pitr->p.ctz.pg_e, pitr->c_off);
}

void mfs_pitr_adv_off(FAR struct mfs_pitr_s * const pitr,
                      const mfs_t off)
{
  pitr->c_off += off;

  finfo("Pitr for CTZ (%u, %u) advanced to offset %u by offset %u.",
        pitr->p.ctz.idx_e, pitr->p.ctz.pg_e, pitr->c_off, off);
}

void mfs_pitr_adv_tochild(FAR struct mfs_pitr_s * const pitr,
                          FAR const struct mfs_path_s * const path,
                          const mfs_t depth)
{
  if (pitr->depth == 0)
    {
      return;
    }

  pitr->c_off = path[depth - 1].off;

  finfo("Pitr for CTZ (%u, %u) at depth %u advanced to offset %u.",
        pitr->p.ctz.idx_e, pitr->p.ctz.pg_e, depth, path[depth - 1].off);
}

void mfs_pitr_reset(FAR struct mfs_pitr_s * const pitr)
{
  pitr->c_off = 0;

  finfo("Pitr for CTZ (%u, %u) reset.",
        pitr->p.ctz.idx_e, pitr->p.ctz.pg_e);
}

void mfs_pitr_sync(FAR struct mfs_sb_s * const sb,
                   FAR struct mfs_pitr_s * const pitr,
                   FAR const struct mfs_path_s * const path,
                   const mfs_t depth)
{
  finfo("Pitr for CTZ (%u, %u) syncing...",
        pitr->p.ctz.idx_e, pitr->p.ctz.pg_e);

  /* TODO: mfs_jrnl_updatectz(sb, &pitr->p.ctz, pitr->depth); */

  pitr->sz = mfs_get_fsz(sb, path, depth - 1);

  DEBUGASSERT(depth == pitr->depth);

  finfo("New location is (%u, %u).",
        pitr->p.ctz.idx_e, pitr->p.ctz.pg_e);
}

int mfs_pitr_readdirent(FAR struct mfs_sb_s * const sb,
                        FAR struct mfs_pitr_s * const pitr,
                        FAR struct mfs_dirent_s **dirent)
{
  int                   ret               = OK;
  mfs_t                 sz                = 0;
  const mfs_t           len               = sizeof(struct mfs_dirent_s)
                                            + NAME_MAX + 1;
  char                  dirent_data[len];
  struct mfs_dirent_s   *dirent_rd        = NULL;

  /* No harm in reading more. The namelen will stop all the included
   * unnecessary characters during deserialization, and if there are none,
   * the rest will be just empty.
   */

  memset(dirent_data, 0, len);

  *dirent = NULL;

  dirent_rd = kmm_zalloc(len);
  if (predict_false(dirent_rd == NULL))
    {
      goto errout;
    }

  if (pitr->depth == 0)
    {
      dirent_rd->ctz       = MFS_MN(sb).root_ctz;
      dirent_rd->mode      = MFS_MN(sb).root_mode;
      dirent_rd->st_atim   = MFS_MN(sb).root_st_atim;
      dirent_rd->st_ctim   = MFS_MN(sb).root_st_ctim;
      dirent_rd->st_mtim   = MFS_MN(sb).root_st_mtim;
      dirent_rd->mode      = MFS_MN(sb).root_mode;
      dirent_rd->sz        = MFS_MN(sb).root_sz;
      dirent_rd->namelen   = 1;
      dirent_rd->name_hash = 0;

      memcpy(dirent_rd->name, "", 2);
    }
  else
    {
      mfs_lru_rdfromoff(sb, pitr->c_off, &pitr->p, pitr->depth,
                        dirent_data, len);
      mfs_deser_dirent(dirent_data, dirent_rd);

      finfo("Got direntry with name %s", dirent_rd->name);
    }

  sz = MFS_DIRENTSZ(dirent_rd);
  if (dirent_rd->namelen == 0)
    {
      /* Not found direntry. */

      goto errout_with_dirent_rd;
    }

  *dirent = kmm_zalloc(sz);
  if (predict_false(*dirent == NULL))
    {
      ret = -ENOMEM;
      goto errout_with_dirent_rd;
    }

  memcpy(*dirent, dirent_rd, sz);

errout_with_dirent_rd:
  kmm_free(dirent_rd);

  if (ret < 0)
    {
      finfo("Direntry could not be allocated.");
    }
  else if (*dirent == NULL)
    {
      finfo("No direntry found.");
    }
  else
    {
      finfo("Reading direntry done for %s.", (*dirent)->name);
    }

errout:
  return ret;
}

void mfs_free_dirent(FAR struct mfs_dirent_s *dirent)
{
  kmm_free(dirent);

  finfo("Dirent freed.");
}

bool mfs_searchfopen(FAR const struct mfs_sb_s * const sb,
                     FAR const struct mfs_path_s * const path,
                     const mfs_t depth)
{
  FAR struct mfs_ofd_s *ofd = NULL;

  list_for_every_entry(&sb->of, ofd, struct mfs_ofd_s, list)
    {
      if (ofd->com->depth != depth)
        {
          continue;
        }

      /* TODO: Ensure when an LRU's delta is flushed to the journal, the
       * new location is updated in the LRU AND the open files, if it is
       * open.
       */

      if (ofd->com->path[depth - 1].off       == path[depth - 1].off &&
          ofd->com->path[depth - 1].ctz.pg_e  == path[depth - 1].ctz.pg_e &&
          ofd->com->path[depth - 1].ctz.idx_e == path[depth - 1].ctz.idx_e)
        {
          return true;
        }
    }

  return false;
}

int mfs_pitr_appendnew(FAR struct mfs_sb_s * const sb,
                       FAR struct mfs_path_s * const path,
                       const mfs_t depth,
                       FAR const struct mfs_pitr_s * const pitr,
                       FAR const char * const child_name,
                       const mode_t mode)
{
  int                 ret     = OK;
  mfs_t               len;
  mfs_t               pg;
  struct mfs_dirent_s *dirent = NULL;
  struct timespec     ts;

  len    = strlen(child_name);

  dirent = kmm_zalloc(sizeof(*dirent) + len);
  if (predict_false(dirent == NULL))
    {
      return -ENOMEM;
    }

  clock_gettime(CLOCK_REALTIME, &ts);

  /* TODO: Confirm if creation for directory in VFS gives true for
   * S_ISDIR().
   */

  pg     = mfs_ba_getpg(sb);
  if (predict_false(pg == 0))
    {
      return -ENOSPC;
    }

  dirent->sz        = 0;
  dirent->mode      = mode;
  dirent->ctz.pg_e  = pg;
  dirent->ctz.idx_e = 0;
  dirent->st_atim   = ts;
  dirent->st_mtim   = ts;
  dirent->st_ctim   = ts;
  dirent->namelen   = len;
  strncpy(dirent->name, child_name, len);

  ret    = mfs_pitr_appenddirent(sb, path, depth, pitr, dirent);
  finfo("New direntry %p with name \"%s\" appended to CTZ (%u, %u) "
        "at offset %u depth %u.",
        dirent, dirent->name, pitr->p.ctz.idx_e, pitr->p.ctz.pg_e,
        pitr->c_off, pitr->depth);

  mfs_free_dirent(dirent);

  return ret;
}

int mfs_pitr_appenddirent(FAR struct mfs_sb_s * const sb,
                          FAR struct mfs_path_s * const path,
                          const mfs_t depth,
                          FAR const struct mfs_pitr_s * const pitr,
                          FAR const struct mfs_dirent_s * const dirent)
{
  int                     ret               = OK;
  const mfs_t             len               = MFS_DIRENTSZ(dirent);
  char                    dirent_data[len];
  struct mfs_pitr_s       p_pitr;
  FAR struct mfs_dirent_s *p_dirent         = NULL;

  memset(dirent_data, 0, len);
  mfs_ser_dirent(dirent, dirent_data);

  mfs_pitr_init(sb, path, depth - 1, &p_pitr, true);
  mfs_pitr_adv_tochild(&p_pitr, path, depth - 1);
  mfs_pitr_readdirent(sb, &p_pitr, &p_dirent);

  ret = mfs_lru_wr(sb, p_dirent->sz, len, p_dirent->sz, path, depth - 1,
                  dirent_data);

  mfs_free_dirent(p_dirent);
  mfs_pitr_free(&p_pitr);

  finfo("Appended direntry for \"%s\" at depth %u.", dirent->name, depth);

  return ret;
}

int mfs_pitr_rmdirent(FAR struct mfs_sb_s * const sb,
                      FAR struct mfs_path_s * const path,
                      const mfs_t depth,
                      FAR struct mfs_pitr_s * const pitr,
                      FAR const struct mfs_dirent_s * const dirent)
{
  FAR struct mfs_dirent_s *p_dirent = NULL;
  struct mfs_pitr_s p_pitr;
  int ret = OK;

  mfs_pitr_init(sb, path, depth - 1, &p_pitr, true);
  mfs_pitr_adv_tochild(&p_pitr, path, depth - 1);
  mfs_pitr_readdirent(sb, &p_pitr, &p_dirent);

  ret = mfs_lru_del(sb, pitr->c_off, MFS_DIRENTSZ(dirent), p_dirent->sz,
                    path, depth);

  mfs_free_dirent(p_dirent);
  mfs_pitr_free(&p_pitr);

  return ret;
}

int mfs_pitr_rm(FAR struct mfs_sb_s * const sb,
                FAR struct mfs_path_s * const path,
                const mfs_t depth)
{
  int                     ret     = OK;
  struct mfs_pitr_s       pitr;
  FAR struct mfs_dirent_s *dirent = NULL;

  mfs_pitr_init(sb, path, depth, &pitr, true);
  mfs_pitr_readdirent(sb, &pitr, &dirent);
  ret = mfs_pitr_rmdirent(sb, path, depth, &pitr, dirent);
  mfs_free_dirent(dirent);
  mfs_pitr_free(&pitr);

  return ret;
}
