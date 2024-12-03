/****************************************************************************
 * fs/mnemofs/mnemofs_fsobj.c
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
#include <sys/param.h>
#include <sys/stat.h>

#include "mnemofs.h"
#include "fs_heap.h"

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
static const char *last_child(FAR const char *relpath);
static FAR char   *mfs_ser_dirent(FAR const struct mfs_dirent_s * const x,
                                  FAR char * const out);

static FAR const char *mfs_deser_dirent(FAR const char * const in,
                                        FAR struct mfs_dirent_s * const x);

static int search_ctz_by_name(FAR const struct mfs_sb_s * const sb,
                              FAR struct mfs_path_s * const path,
                              const mfs_t depth, FAR const char * const name,
                              const mfs_t namelen, FAR mfs_t *off,
                              FAR struct mfs_dirent_s **dirent);

/****************************************************************************
 * Private Data
 ****************************************************************************/

const struct mfs_path_s empty_fsobj =
{
  0
};

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

  /* If mount point is "/hi", then operations on "/hi/bye" and "/hi/hello/a"
   * give respective relpaths as "bye" and "hello/a". Since mnemofs counts
   * the root as 1 FS object, these respectively contain 2 and 3 FS objects
   * in the path. The last FS object might be a file or a directory, but
   * everything else is a directory. The number of FS objects in the path
   * is the depth of the FS object the path refers to.
   */

  if (*relpath == 0)
    {
      return 1;
    }

  count = 2;

  while (*relpath != 0)
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
 * Name: next_child
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

static const char *next_child(FAR const char *path)
{
  mfs_t           inc = 0;
  FAR const char *tmp = path;

  MFS_EXTRA_LOG("NEXT_CHILD", "Requested string is \"%s\" (%p),", path,
                path);

  while (*path != 0)
    {
      if (*path == '/')
        {
          MFS_EXTRA_LOG("NEXT_CHILD", "Length is %" PRIu32, inc);
          DEBUGASSERT(inc == path - tmp);
          return path + 1;
        }

      path++;
      inc++;
    }

  MFS_EXTRA_LOG("NEXT_CHILD", "Length is %" PRIu32, inc);
  DEBUGASSERT(inc == path - tmp);
  MFS_EXTRA_LOG("NEXT_CHILD", "Last FS Object in string.");
  return path;
}

/****************************************************************************
 * Name: next_child
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

static const char *last_child(FAR const char *relpath)
{
  const mfs_t len = strlen(relpath);
  mfs_t i;

  for (i = len - 1; i > 0; i--)
    {
      if (relpath[i - 1] == '/')
        {
          return relpath + i;
        }
    }

  return relpath;
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

  o = mfs_ser_16(x->name_hash, o);
  o = mfs_ser_16(x->mode, o);
  o = mfs_ser_mfs(x->sz, o);
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

  i = mfs_deser_16(i, &x->name_hash);
  i = mfs_deser_16(i, &x->mode);
  i = mfs_deser_mfs(i, &x->sz);
  i = mfs_deser_timespec(i, &x->st_atim);
  i = mfs_deser_timespec(i, &x->st_mtim);
  i = mfs_deser_timespec(i, &x->st_ctim);
  i = mfs_deser_ctz(i, &x->ctz);
  i = mfs_deser_8(i, &x->namelen);
  i = mfs_deser_str(i, x->name, x->namelen);

  return i;
}

int pitr_traverse(FAR struct mfs_sb_s *sb, FAR struct mfs_path_s *path,
                  mfs_t depth, FAR mfs_t *cap)
{
  int                      ret    = OK;
  mfs_t                    i;
  mfs_t                    pg;
  struct mfs_pitr_s        pitr;
  struct mfs_ctz_s         ctz;
  FAR struct mfs_dirent_s *dirent = NULL;

  /* TODO: Double traversal can be made faster into a single traversal. */

  ctz = path[depth - 1].ctz;

  if (ctz.idx_e == 0 && ctz.pg_e == 0)
    {
      /* Not a valid one. TODO: Does this happens? */

      goto errout;
    }

  for (i = ctz.idx_e; i > 0; i--)
    {
      mfs_ba_markusedpg(sb, pg);

      pg = mfs_ctz_travel(sb, i, pg, i - 1);
      if (pg == 0)
        {
          break;
        }
    }

  memset(path + depth, 0, *cap - depth);

  if (depth == *cap)
    {
      *cap = (*cap * 3) / 2; /* Don't want to double it for memory. */

      path = fs_heap_realloc(path, (*cap) * sizeof(struct mfs_path_s));
      if (predict_false(path == NULL))
        {
          ret = -ENOMEM;
          goto errout;
        }
    }

  mfs_pitr_init(sb, path, depth, &pitr, true);

  while (true)
    {
      mfs_pitr_readdirent(sb, path, &pitr, &dirent);
      if (dirent == NULL)
        {
          break;
        }

      if (S_ISDIR(dirent->mode))
        {
          path[(depth + 1) - 1].ctz = dirent->ctz;

          ret = pitr_traverse(sb, path, depth + 1, cap);
          if (predict_false(ret < 0))
            {
              mfs_free_dirent(dirent);
              goto errout;
            }
        }

      mfs_pitr_adv_bydirent(&pitr, dirent);
      mfs_free_dirent(dirent);
    }

errout:
  return ret;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR const char *mfs_path2childname(FAR const char *relpath)
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
      sz = MFS_MN(sb).root_sz; /* Updated size. */

      /* Journal updated to the root creates a new master node entry. TODO
       * this and moving of the journal.
       */

      finfo("File size got as %u for root.", sz);
      return sz;
    }

  return path[depth - 1].sz;
}

bool mfs_obj_isempty(FAR struct mfs_sb_s * const sb,
                     FAR struct mfs_path_s *path,
                     FAR struct mfs_pitr_s * const pitr)
{
  bool                     ret;
  FAR struct mfs_dirent_s *dirent = NULL;

  mfs_pitr_readdirent(sb, path, pitr, &dirent);
  ret = (dirent->sz == 0);
  mfs_free_dirent(dirent);

  return ret;
}

void mfs_free_dirent(FAR struct mfs_dirent_s *dirent)
{
  fs_heap_free(dirent);

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

      if (mfs_path_eq(&ofd->com->path[depth - 1], &path[depth - 1]))
        {
          return true;
        }
    }

  return false;
}

int mfs_pitr_rmdirent(FAR struct mfs_sb_s * const sb,
                      FAR struct mfs_path_s * const path,
                      const mfs_t depth,
                      FAR struct mfs_pitr_s * const pitr,
                      FAR const struct mfs_dirent_s * const dirent)
{
  int                      ret      = OK;
  struct mfs_pitr_s        p_pitr;
  FAR struct mfs_dirent_s *p_dirent = NULL;

  mfs_pitr_init(sb, path, depth - 1, &p_pitr, true);
  mfs_pitr_adv_tochild(&p_pitr, path);
  mfs_pitr_readdirent(sb, path, &p_pitr, &p_dirent);

  ret = mfs_lru_del(sb, pitr->c_off, MFS_DIRENTSZ(dirent), path, depth);

  mfs_free_dirent(p_dirent);
  mfs_pitr_free(&p_pitr);

  return ret;
}

int mfs_pitr_rm(FAR struct mfs_sb_s * const sb,
                FAR struct mfs_path_s * const path,
                const mfs_t depth, bool rm_child)
{
  int                      ret    = OK;
  struct mfs_pitr_s        pitr;
  FAR struct mfs_dirent_s *dirent = NULL;

  /* TODO: MFS_LOG */

  mfs_pitr_init(sb, path, depth, &pitr, true);
  mfs_pitr_readdirent(sb, path, &pitr, &dirent);

  ret = mfs_pitr_rmdirent(sb, path, depth, &pitr, dirent);
  if (predict_false(ret < 0))
    {
      goto errout;
    }

  if (rm_child)
    {
      ret = mfs_lru_del(sb, 0, path[depth - 1].sz, path, depth);
      if (predict_false(ret < 0))
        {
          goto errout;
        }
    }

errout:
  mfs_free_dirent(dirent);
  mfs_pitr_free(&pitr);

  return ret;
}

int mfs_pitr_init(FAR const struct mfs_sb_s * const sb,
                  FAR const struct mfs_path_s * const path,
                  const mfs_t depth, FAR struct mfs_pitr_s * const pitr,
                  bool child)
{
  /* Ensure updated CTZ location from the journal before this. */

  int         ret     = OK;
  const int   diff    = child ? 1 : 0;
  const mfs_t p_depth = depth - diff;

  if (predict_false(depth < diff))
    {
      ret = -EINVAL;
      goto errout;
    }

  pitr->c_off = 0;
  pitr->depth = p_depth;

  if (predict_true(p_depth != 0))
    {
      pitr->p = path[p_depth - 1];
    }
  else
    {
      /* 0 or gabage value is fine for master node, not required. */

      pitr->p.ctz.idx_e = 0;
      pitr->p.ctz.pg_e  = 0;
      pitr->p.off       = 0;
      pitr->p.sz        = 1; /* For 1 traversal to get root. */
    }

  finfo("Pitr initialized at depth %u, with CTZ (%u, %u) and size %u.",
        p_depth, pitr->p.ctz.idx_e, pitr->p.ctz.pg_e, pitr->p.sz);

errout:
  return ret;
}

void mfs_pitr_free(FAR const struct mfs_pitr_s * const pitr)
{
  MFS_EXTRA_LOG("MFS_PITR_FREE", "Parent iterator at %p freed.", pitr);
  MFS_EXTRA_LOG_PITR(pitr);
}

void mfs_pitr_adv_off(FAR struct mfs_pitr_s * const pitr,
                      const mfs_t off)
{
  pitr->c_off += off;

  finfo("Pitr at depth %u with CTZ (%u, %u) advanced by %u to %u offset.",
        pitr->depth, pitr->p.ctz.idx_e, pitr->p.ctz.pg_e, off, pitr->c_off);
}

void mfs_pitr_adv_bydirent(FAR struct mfs_pitr_s * const pitr,
                           FAR const struct mfs_dirent_s * const dirent)
{
  mfs_pitr_adv_off(pitr, MFS_DIRENTSZ(dirent));

  finfo("Pitr at depth %u with CTZ (%u, %u) advanced by %u to %u offset.",
        pitr->depth, pitr->p.ctz.idx_e, pitr->p.ctz.pg_e,
        MFS_DIRENTSZ(dirent), pitr->c_off);
}

void mfs_pitr_adv_tochild(FAR struct mfs_pitr_s * const pitr,
                          FAR const struct mfs_path_s * const path)
{
  /* (pitr->depth + 1) - 1 is the child's index. */

  MFS_EXTRA_LOG("MFS_PITR_ADV_TOCHILD", "Advance pitr to child's offset.");
  MFS_EXTRA_LOG_PITR(pitr);
  MFS_EXTRA_LOG("MFS_PITR_ADV_TOCHILD", "New offset %" PRIu32, pitr->c_off);
  pitr->c_off = path[pitr->depth].off;
}

int mfs_pitr_readdirent(FAR const struct mfs_sb_s * const sb,
                        FAR struct mfs_path_s *path,
                        FAR struct mfs_pitr_s * const pitr,
                        FAR struct mfs_dirent_s **dirent)
{
  int                      ret      = OK;
  mfs_t                    sz;
  const mfs_t              len      = sizeof(struct mfs_dirent_s) \
                                      + NAME_MAX + 1;
  char                     rd[len];
  FAR struct mfs_dirent_s *d        = NULL;
  FAR struct mfs_dirent_s *tmp      = NULL;

  if (dirent == NULL)
    {
      return ret;
    }

  *dirent = NULL;
  memset(rd, 0, len);

  d = fs_heap_zalloc(len);
  if (predict_false(d == NULL))
    {
      ret = -ENOMEM;
      goto errout;
    }
  else if (pitr->c_off >= pitr->p.sz)
    {
      goto errout_with_d;
    }
  else if (pitr->depth == 0)
    {
      d->name[0]   = 0;
      d->namelen   = 0;
      d->ctz       = MFS_MN(sb).root_ctz;
      d->mode      = MFS_MN(sb).root_mode;
      d->name_hash = mfs_hash(d->name, d->namelen);
      d->st_atim   = MFS_MN(sb).root_st_atim;
      d->st_ctim   = MFS_MN(sb).root_st_ctim;
      d->st_mtim   = MFS_MN(sb).root_st_mtim;
      d->sz        = MFS_MN(sb).root_sz;

      pitr->c_off  = 1; /* To prevent infinite loop. */
    }
  else
    {
      ret = mfs_lru_rdfromoff(sb, pitr->c_off, path, pitr->depth, rd, len);
      if (predict_false(ret < 0))
        {
          goto errout_with_d;
        }

      mfs_deser_dirent(rd, d);
    }

  sz  = MFS_DIRENTSZ(d);
  tmp = fs_heap_realloc(d, sz);
  if (predict_true(tmp != NULL))
    {
      d = tmp;
    }

  *dirent = d;
  DEBUGASSERT(pitr->depth == 0 || strcmp(d->name, ""));
  finfo("Read direntry at %u offset, %u depth for CTZ (%u, %u). " \
        "Direntry name: \"%.*s\" with name length %u and size %u.",
        pitr->c_off, pitr->depth, pitr->p.ctz.idx_e, pitr->p.ctz.pg_e,
        d->namelen, d->name, d->namelen, d->sz);

  return ret;

errout_with_d:
  fs_heap_free(d);

  if (ret < 0)
    {
      finfo("Direntry could not be allocated.");
    }
  else if (*dirent == NULL)
    {
      finfo("No direntry found.");
    }

errout:
  return ret;
}

int mfs_pitr_adv(FAR struct mfs_sb_s * const sb,
                 FAR struct mfs_path_s *path,
                 FAR struct mfs_pitr_s * const pitr)
{
  int                      ret    = OK;
  FAR struct mfs_dirent_s *dirent;

  ret = mfs_pitr_readdirent(sb, path, pitr, &dirent);
  if (predict_false(ret < 0))
    {
      goto errout;
    }

  mfs_pitr_adv_bydirent(pitr, dirent);
  mfs_free_dirent(dirent);

  finfo("Pitr for CTZ (%u, %u) advanced to offset %u.",
        pitr->p.ctz.idx_e, pitr->p.ctz.pg_e, pitr->c_off);

errout:
  return ret;
}

static int search_ctz_by_name(FAR const struct mfs_sb_s * const sb,
                              FAR struct mfs_path_s * const path,
                              const mfs_t depth, FAR const char * const name,
                              const mfs_t namelen, FAR mfs_t *off,
                              FAR struct mfs_dirent_s **dirent)
{
  /* NOTE: depth is of the parent here. */

  /* Applies LRU updates. */

  int                      ret        = OK;
  bool                     found      = false;
  uint16_t                 name_hash;
  struct mfs_pitr_s        pitr;
  FAR struct mfs_dirent_s *nd;

  *dirent = NULL;

  if (depth == 0)
    {
      DEBUGASSERT(namelen == 0);

      nd = fs_heap_zalloc(sizeof(struct mfs_dirent_s));
      if (predict_false(nd == NULL))
        {
          ret = -ENOMEM;
          goto errout;
        }

      *off          = 0;
      nd->namelen   = namelen;
      nd->ctz       = MFS_MN(sb).root_ctz;
      nd->mode      = MFS_MN(sb).root_mode;
      nd->name_hash = mfs_hash(nd->name, nd->namelen);
      nd->st_atim   = MFS_MN(sb).root_st_atim;
      nd->st_ctim   = MFS_MN(sb).root_st_ctim;
      nd->st_mtim   = MFS_MN(sb).root_st_mtim;
      nd->sz        = MFS_MN(sb).root_sz;

      *dirent = nd;
      goto errout;
    }

  name_hash = mfs_hash(name, namelen);

  ret = mfs_lru_getupdatedinfo(sb, path, depth);
  if (predict_false(ret < 0))
    {
      goto errout;
    }

  ret = mfs_pitr_init(sb, path, depth, &pitr, false);
  if (predict_false(ret < 0))
    {
      goto errout;
    }

  for (; ; )
    {
      /* Readdirent takes care of LRU updates. */

      ret = mfs_pitr_readdirent(sb, path, &pitr, &nd);
      if (predict_false(ret < 0 || nd == NULL))
        {
          ret = -ENONET;
          goto errout;
        }

      if (nd->name_hash == name_hash &&
          !strncmp(nd->name, name, MIN(nd->namelen, namelen)))
        {
          found           = true;
          path[depth].sz  = nd->sz;
          path[depth].ctz = nd->ctz;
          path[depth].off = pitr.c_off;
          *off            = pitr.c_off;
          break;
        }

      mfs_pitr_adv_bydirent(&pitr, nd);
      mfs_free_dirent(nd);
    }

errout:
  if (found)
    {
      finfo("Searched \"%.*s\" direntry inside CTZ (%u, %u) at depth %u,"
            " size %u.", namelen, name, path[depth - 1].ctz.idx_e,
            path[depth - 1].ctz.pg_e, depth, path[depth - 1].sz);
      *dirent = nd;
    }
  else
    {
      ret = -ENOENT;
      finfo("Can not find requested direntry in parent. Ret: %d.", ret);
    }

  return ret;
}

int mfs_get_patharr(FAR const struct mfs_sb_s * const sb,
                    FAR const char *relpath, FAR struct mfs_path_s **path,
                    FAR mfs_t *depth)
{
  int                      ret       = OK;
  int                      ret_flags = 0;
  mfs_t                    i;
  mfs_t                    sz;
  mfs_t                    off;
  mfs_t                    n_objs;
  mfs_t                    name_len;
  FAR const char          *cur       = NULL;
  FAR const char          *next      = NULL;
  struct mfs_ctz_s         ctz;
  FAR struct mfs_path_s   *np        = NULL;
  FAR struct mfs_dirent_s *dirent    = NULL;

  MFS_LOG("MFS_GET_PATHARR", "Entry.");
  MFS_EXTRA_LOG("MFS_GET_PATHARR", "Relpath is \"%s\".", relpath);

  *path  = NULL;
  MFS_EXTRA_LOG("MFS_GET_PATHARR", "Path is %p.", path);

  n_objs = nobjs_in_path(relpath);
  MFS_EXTRA_LOG("MFS_GET_PATHARR", "There are %" PRIu32 " objects in path.",
                n_objs);

  np     = fs_heap_zalloc(n_objs * sizeof(struct mfs_path_s));
  if (predict_false(np == NULL))
    {
      MFS_EXTRA_LOG("MFS_GET_PATHARR", "Could not allocate Path array.");
      ret = -ENOMEM;
      goto errout;
    }
  else
    {
      MFS_EXTRA_LOG("MFS_GET_PATHARR", "Path array is allocated at %p.", np);
    }

  DEBUGASSERT(*cur != '/'); /* Relpath should not start with a '/' */

  ctz       = MFS_MN(sb).root_ctz;
  sz        = MFS_MN(sb).root_sz;
  np[0].sz  = sz;
  np[0].ctz = ctz;
  np[0].off = 0;
  cur       = relpath;
  next      = next_child(cur);

  DEBUGASSERT(*next != 0 || n_objs == 2);

  name_len  = *next == 0 ? next - cur : next - cur - 1;

  MFS_EXTRA_LOG("MFS_GET_PATHARR", "Root Master Node.");
  MFS_EXTRA_LOG("MFS_GET_PATHARR", "\tCTZ is (%" PRIu32 ", %" PRIu32 ")",
                MFS_MN(sb).root_ctz.idx_e, MFS_MN(sb).root_ctz.pg_e);
  MFS_EXTRA_LOG("MFS_GET_PATHARR", "\tSize is %" PRIu32, sz);

  if (predict_false(n_objs == 1))
    {
      MFS_EXTRA_LOG("MFS_GET_PATHARR", "There is only one object (root).");
      ret_flags |= MFS_ISDIR | MFS_EXIST;

      /* This will not go into the loop. */
    }
  else if (predict_false(n_objs == 2))
    {
      MFS_EXTRA_LOG("MFS_GET_PATHARR", "There are only 2 objects.");
      ret_flags |= MFS_P_EXIST | MFS_P_ISDIR;
    }

  /* MFS_MN(sb).root_* is always up to date, no need for journal update. */

  for (i = 1; i < n_objs; i++)
    {
      MFS_EXTRA_LOG("MFS_GET_PATHARR", "Looking at depth %" PRIu32, i);

      /* np[i] is the fs object at depth i + 1. */

      /* Need to update journal for every level in the path as, for eg., the
       * child can be deleted, etc. Same goes for LRU, which is taken care of
       * by search_ctz_by_name function.
       */

      MFS_EXTRA_LOG("MFS_GET_PATHARR", "Current String is \"%.*s\" (%p)",
                    name_len, cur, cur);
      MFS_EXTRA_LOG("MFS_GET_PATHARR", "Name length is %" PRIu32, name_len);
      MFS_EXTRA_LOG("MFS_GET_PATHARR", "Next String is \"%s\"", next);

      ret = search_ctz_by_name(sb, np, i, cur, name_len, &off, &dirent);
      if (predict_false(ret < 0))
        {
          MFS_EXTRA_LOG("MFS_GET_PATHARR", "Could not find CTZ.");
          goto errout_with_ret_flags;
        }
      else
        {
          MFS_EXTRA_LOG("MFS_GET_PATHARR", "Found CTZ.");
          MFS_EXTRA_LOG("MFS_GET_PATHARR", "New Offset is %" PRIu32, off);
          MFS_EXTRA_LOG_DIRENT(dirent);
        }

      if (i < n_objs - 2 && !S_ISDIR(dirent->mode))
        {
          MFS_EXTRA_LOG("MFS_GET_PATHARR", "Depth %" PRIu32 " contains file",
                        i);
          ret_flags |= MFS_FINPATH;
          goto errout_with_ret_flags;
        }
      else if (i == n_objs - 2)
        {
          MFS_EXTRA_LOG("MFS_GET_PATHARR", "Parent exists.");
          ret_flags |= MFS_P_EXIST;
          if (S_ISDIR(dirent->mode))
            {
              MFS_EXTRA_LOG("MFS_GET_PATHARR", "Parent is a directory.");
              ret_flags |= MFS_P_ISDIR;
            }
          else
            {
              MFS_EXTRA_LOG("MFS_GET_PATHARR", "Parent is a file.");
              ret_flags |= MFS_FINPATH;
              goto errout_with_ret_flags;
            }
        }
      else if (i == n_objs - 1)
        {
          MFS_EXTRA_LOG("MFS_GET_PATHARR", "Child exists.");
          ret_flags |= MFS_EXIST;
          if (S_ISDIR(dirent->mode))
            {
              MFS_EXTRA_LOG("MFS_GET_PATHARR", "Child is a directory.");
              ret_flags |= MFS_ISDIR;
            }
          else
            {
              MFS_EXTRA_LOG("MFS_GET_PATHARR", "Child is a file.");
              ret_flags |= MFS_ISFILE;
            }
        }
      else
        {
          /* OK */
        }

      np[i].ctz = dirent->ctz;
      np[i].off = off;
      np[i].sz  = dirent->sz;

      ctz       = dirent->ctz;

      mfs_free_dirent(dirent);

      cur       = next;
      next      = next_child(cur);
      name_len  = *next == 0 ? next - cur : next - cur - 1;

      DEBUGASSERT(cur != next);
    }

  ret    = ret_flags;
  *depth = n_objs;
  *path  = np;

  MFS_EXTRA_LOG("MFS_GET_PATHARR", "Child is a file.");

  MFS_LOG("MKDIR", "Exit | Flags: %u, Depth %u.", ret, n_objs);
  return ret;

errout_with_ret_flags:
  ret    = ret_flags;
  *depth = n_objs;
  *path  = np;

  /* mfs_free_patharr(np) : All callers will free np (ie. path) when done
   * with it.
   */

errout:
  MFS_LOG("MKDIR", "Exit | Flags: %u, Depth %u.", ret, n_objs);
  return ret;
}

void mfs_free_patharr(FAR struct mfs_path_s *path)
{
  MFS_EXTRA_LOG("MFS_FREE_PATHARR", "Path array at %p freed.", path);
  fs_heap_free(path);
}

void mfs_pitr_reset(FAR struct mfs_pitr_s * const pitr)
{
  pitr->c_off = 0;

  finfo("Pitr for CTZ (%u, %u) reset.",
        pitr->p.ctz.idx_e, pitr->p.ctz.pg_e);
}

int mfs_pitr_appenddirent(FAR struct mfs_sb_s * const sb,
                          FAR struct mfs_path_s * const path,
                          const mfs_t depth,
                          FAR struct mfs_pitr_s * const pitr,
                          FAR const struct mfs_dirent_s * const dirent)
{
  /* Depth is depth of the child to be appended. */

  int         ret     = OK;
  const mfs_t len     = MFS_DIRENTSZ(dirent);
  char        wd[len];

  if (pitr->depth == 0)
    {
      /* Root is the only child of the master node. */

      ret = -EINVAL;
      goto errout;
    }

  /* TODO: If the parent directory is newly formed (ie. size is 0), then
   * allocate space for it. This can be done better. Just allocate page when
   * its created and added first to LRU, and then add a check to ensure it
   * doesn't get re-allocated when written. A field like "new" would be
   * helpful in the  LRU node for this.
   */

  memset(wd, 0, len);

  mfs_ser_dirent(dirent, wd);
  ret = mfs_lru_wr(sb, pitr->p.sz, len, path, pitr->depth, wd);
  if (predict_false(ret < 0))
    {
      goto errout;
    }

errout:
  return ret;
}

int mfs_pitr_appendnew(FAR struct mfs_sb_s * const sb,
                       FAR struct mfs_path_s * const path, const mfs_t depth,
                       FAR struct mfs_pitr_s * const pitr,
                       FAR const char * const relpath, const mode_t mode)
{
  /* Depth is depth of the child to be appended. */

  int                      ret  = OK;
  FAR const char          *cur  = last_child(relpath);
  FAR const char          *next = next_child(cur);
  const mfs_t              len  = *next == 0 ? next - cur : next - cur - 1;
  struct timespec          ts;
  FAR struct mfs_dirent_s *d    = NULL;

  DEBUGASSERT(depth > 0);

  d = fs_heap_zalloc(sizeof(struct mfs_dirent_s) + len);
  if (predict_false(d == NULL))
    {
      ret = -ENOMEM;
      goto errout;
    }

  clock_gettime(CLOCK_REALTIME, &ts);

  d->ctz       = empty_fsobj.ctz;
  d->mode      = mode;
  d->st_atim   = ts;
  d->st_mtim   = ts;
  d->st_ctim   = ts;
  d->namelen   = len;
  strncpy(d->name, cur, d->namelen);
  d->name_hash = mfs_hash(d->name, d->namelen);
  d->sz        = 0;

  /* Add the new direntry in this path. */

  path[depth - 1].ctz = d->ctz;
  path[depth - 1].off = pitr->p.sz;
  path[depth - 1].sz  = d->sz;

  ret = mfs_pitr_appenddirent(sb, path, depth, pitr, d);
  if (predict_false(ret < 0))
    {
      goto errout_with_d;
    }

  finfo("Direntry appended to Pitr with %u depth, and CTZ (%u, %u). " \
        "Direntry name: \"%.*s\" with name length %u at offset %u.",
        pitr->depth, pitr->p.ctz.idx_e, pitr->p.ctz.pg_e, d->namelen,
        d->name, d->namelen, path[depth - 1].off);

errout_with_d:
  mfs_free_dirent(d);

errout:
  return ret;
}

/* Only for initialization of the block allocator. */

int mfs_pitr_traversefs(FAR struct mfs_sb_s * sb, const struct mfs_ctz_s ctz,
                        int type)
{
  /* type takes in MFS_ISFILE & MFS_ISDIR. */

  int                    ret      = OK;
  mfs_t                  capacity;
  FAR struct mfs_path_s *path     = NULL;

  capacity = MFS_TRAVERSE_INITSZ;
  path = fs_heap_zalloc(capacity * sizeof(struct mfs_path_s));
  if (predict_false(path == NULL))
    {
      ret = -ENOMEM;
      goto errout;
    }

  path[0].off = 0;
  path[0].ctz = MFS_MN(sb).root_ctz;
  path[0].sz  = MFS_MN(sb).root_sz;

  ret = pitr_traverse(sb, path, 1, &capacity);

  mfs_free_patharr(path);

errout:
  return ret;
}
