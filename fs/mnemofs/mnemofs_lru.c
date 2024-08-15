/****************************************************************************
 * fs/mnemofs/mnemofs_lru.c
 * LRU cache of mnemofs.
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
 * LRU (Least Recently Used) cache takes in all the changes the user wants
 * to do to the on-flash storage, and stores them in memory. When a
 * significant amount of changes are accumulated, the LRU writes the new
 * information to the flash.
 *
 * LRU is a kernel list of nodes. Each node represents a CTZ list. Each node
 * contains a kernel list of changes requested for the CTZ list, called as
 * deltas.
 *
 * When LRU is full the last node is flushed (it can be explicitly flushed as
 * well) and all the changes are written at once on the flash, and the new
 * location is noted down in the journal, and an entry for the location
 * update is added to the LRU for the parent.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/kmalloc.h>
#include <nuttx/list.h>
#include <sys/param.h>

#include "mnemofs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum
{
  MFS_LRU_UPD,
  MFS_LRU_DEL,
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void lru_nodesearch(FAR const struct mfs_sb_s * const sb,
                           FAR const struct mfs_path_s * const path,
                           const mfs_t depth, FAR struct mfs_node_s **node);
static bool lru_islrufull(FAR struct mfs_sb_s * const sb);
static bool lru_isnodefull(FAR struct mfs_node_s *node);
static int  lru_nodeflush(FAR struct mfs_sb_s * const sb,
                          FAR struct mfs_path_s * const path,
                          const mfs_t depth, FAR struct mfs_node_s *node,
                          bool clean_node);
static int  lru_wrtooff(FAR struct mfs_sb_s * const sb, const mfs_t data_off,
                        mfs_t bytes, int op,
                        FAR struct mfs_path_s * const path,
                        const mfs_t depth, FAR const char *buf);
static int  lru_updatesz(FAR struct mfs_sb_s * sb,
                         FAR struct mfs_path_s * const path,
                         const mfs_t depth, const mfs_t new_sz);

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
 * Name: lru_nodesearch
 *
 * Description:
 *   Searches a node by the `path` and `depth` in the LRU.
 *
 * Input Parameters:
 *   sb    - Superblock instance of the device.
 *   path  - CTZ representation of the relpath.
 *   depth - Depth of `path`.
 *   node  - To populate with the node corresponding to the CTZ.
 *
 ****************************************************************************/

static void lru_nodesearch(FAR const struct mfs_sb_s * const sb,
                           FAR const struct mfs_path_s * const path,
                           const mfs_t depth, FAR struct mfs_node_s **node)
{
  bool                   found = false;
  mfs_t                  i;
  FAR struct mfs_node_s *n;

  *node = NULL;

  list_for_every_entry(&MFS_LRU(sb), n, struct mfs_node_s, list)
    {
      /* We need this loop to specifically check the parents in case these
       * entries are all new, and have not been allocated any pages for
       * being stored in the flash. Also we know that the root (depth 1) will
       * be at least common in their paths.
       */

      DEBUGASSERT(depth > 0);

      if (n->depth != depth)
        {
          continue;
        }

      found = true;
      for (i = n->depth; i >= 1 && found; i--)
        {
          if (path[i - 1].ctz.idx_e == 0 && path[i - 1].ctz.pg_e == 0 &&
              mfs_ctz_eq(&n->path[i - 1].ctz, &path[i - 1].ctz) &&
              n->path[i - 1].off == path[i - 1].off)
            {
              /* OK */
            }
          else if (path[i - 1].ctz.pg_e != 0 &&
                  mfs_ctz_eq(&n->path[i - 1].ctz, &path[i - 1].ctz))
            {
              /* OK */
            }
          else
            {
              found = false;
            }
        }

      if (found)
        {
          *node = n;
          break;
        }
    }

  if (found)
    {
      finfo("Node search ended with match of node %p at depth %u"
            " for CTZ of %u size with range [%u, %u).", n, n->depth, n->sz,
            n->range_min, n->range_max);
    }
  else
    {
      finfo("Node search ended without match.");
      *node = NULL;
    }
}

/****************************************************************************
 * Name: lru_islrufull
 *
 * Description:
 *   Check whether the number of nodes in the LRU has reaches its limit.
 *
 * Input Parameters:
 *   sb - Superblock instance of the device.
 *
 * Returned Value:
 *  true   - LRU is full
 *  false  - LRU is not full.
 *
 ****************************************************************************/

static bool lru_islrufull(FAR struct mfs_sb_s * const sb)
{
  return sb->n_lru == CONFIG_MNEMOFS_NLRU;
}

/****************************************************************************
 * Name: lru_isnodefull
 *
 * Description:
 *   Check whether the number of deltas in an LRU node has reaches its limit.
 *
 * Input Parameters:
 *   node - LRU node.
 *
 * Returned Value:
 *  true   - LRU node is full
 *  false  - LRU node is not full.
 *
 ****************************************************************************/

static bool lru_isnodefull(FAR struct mfs_node_s *node)
{
  return node->n_list == CONFIG_MNEMOFS_NLRUDELTA;
}

/****************************************************************************
 * Name: lru_free_delta
 *
 * Description:
 *   Free a node's delta.
 *
 * Input Parameters:
 *   delta - LRU delta.
 *
 ****************************************************************************/

static void lru_free_delta(FAR struct mfs_delta_s *delta)
{
  kmm_free(delta->upd);
  kmm_free(delta);
}

/****************************************************************************
 * Name: lru_nodeflush
 *
 * Description:
 *   Clear out the deltas in a node by writing them to the flash, and adding
 *   a log about it to the journal.
 *
 * Input Parameters:
 *   sb          - Superblock instance of the device.
 *   path        - CTZ representation of the relpath.
 *   depth       - Depth of `path`.
 *   node        - LRU node to flush.
 *   clean_node  - To remove node out of LRU (true), or just clear the
 *                 deltas (false).
 *
 * Returned Value:
 *  0   - OK
 *  < 0 - Error
 *
 ****************************************************************************/

static int lru_nodeflush(FAR struct mfs_sb_s * const sb,
                         FAR struct mfs_path_s * const path,
                         const mfs_t depth, FAR struct mfs_node_s *node,
                         bool clean_node)
{
  int                     ret   = OK;
  FAR struct mfs_delta_s *delta = NULL;
  FAR struct mfs_delta_s *tmp   = NULL;

  if (predict_false(node == NULL))
    {
      return -ENOMEM;
    }

  /* TODO: Implement effct of clean_node. */

  ret = mfs_ctz_wrtnode(sb, node);
  if (predict_false(ret < 0))
    {
      goto errout;
    }

  /* Reset node stats. */

  node->range_max = 0;
  node->range_min = UINT32_MAX;

  /* Free deltas after flush. */

  list_for_every_entry_safe(&node->list, delta, tmp, struct mfs_delta_s,
                            list)
    {
      list_delete_init(&delta->list);
      lru_free_delta(delta);
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: lru_wrtooff
 *
 * Description:
 *   Write to offset in LRU.
 *
 * Input Parameters:
 *   sb       - Superblock instance of the device.
 *   data_off - Offset into the data in the CTZ skip list.
 *   bytes    - Number of bytes to write.
 *   ctz_sz   - Size of the CTZ skip list.
 *   op       - Operation (MFS_LRU_UPD or MFS_LRU_DEL).
 *   path     - CTZ representation of the path.
 *   depth    - Depth of path.
 *   buf      - Buffer containing data.
 *
 * Returned Value:
 *  0   - OK
 *  < 0 - Error
 *
 ****************************************************************************/

static int lru_wrtooff(FAR struct mfs_sb_s * const sb, const mfs_t data_off,
                       mfs_t bytes, int op,
                       FAR struct mfs_path_s * const path, const mfs_t depth,
                       FAR const char *buf)
{
  int                     ret       = OK;
  bool                    found     = true;
  mfs_t                   old_sz;
  FAR struct mfs_node_s  *node      = NULL;
  FAR struct mfs_node_s  *last_node = NULL;
  FAR struct mfs_delta_s *delta     = NULL;

  lru_nodesearch(sb, path, depth, &node);

  if (node == NULL)
    {
      node = kmm_zalloc(sizeof(*node));
      if (predict_false(node == NULL))
        {
          found = false;
          ret = -ENOMEM;
          goto errout;
        }

      node->path = kmm_zalloc(depth * sizeof(struct mfs_path_s));
      if (predict_false(node->path == NULL))
        {
          found = false;
          ret = -ENOMEM;
          goto errout_with_node;
        }

      node->sz        = path[depth - 1].sz;
      node->depth     = depth;
      node->n_list    = 0;
      node->range_max = 0;
      node->range_min = UINT32_MAX;
      list_initialize(&node->delta);
      memcpy(node->path, path, depth * sizeof(struct mfs_path_s));
      found = false;

      finfo("Node not found. Allocated at %p.", node);
    }

  if (!found)
    {
      if (lru_islrufull(sb))
        {
          finfo("LRU is full, need to flush a node.");
          last_node = list_container_of(list_peek_tail(&MFS_LRU(sb)),
                                        struct mfs_node_s, list);
          list_delete_init(&last_node->list);
          list_add_tail(&MFS_LRU(sb), &node->list);
          finfo("LRU flushing node complete, now only %u nodes", sb->n_lru);
        }
      else
        {
          list_add_tail(&MFS_LRU(sb), &node->list);
          sb->n_lru++;
          finfo("Node inserted into LRU, and it now %u node(s).", sb->n_lru);
        }
    }
  else if (found && lru_isnodefull(node))
    {
      /* Node flush writes to the flash and journal. */

      ret = lru_nodeflush(sb, path, depth, node, false);
      if (predict_false(ret < 0))
        {
          goto errout_with_node;
        }
    }

  /* Add delta to node. */

  finfo("Adding delta to the node.");
  delta = kmm_zalloc(sizeof(*delta));
  if (predict_false(delta == NULL))
    {
      ret = -ENOMEM;
      goto errout_with_node;
    }

  finfo("Delta allocated.");

  if (op == MFS_LRU_UPD)
    {
      delta->upd = kmm_zalloc(bytes);
      if (predict_false(delta->upd == NULL))
        {
          ret = -ENOMEM;
          goto errout_with_delta;
        }

      finfo("Delta is of the update type, has %u bytes at offset %u.",
            bytes, data_off);
    }

  delta->n_b = bytes;
  delta->off = data_off;
  list_add_tail(&node->delta, &delta->list);
  if (op == MFS_LRU_UPD)
    {
      memcpy(delta->upd, buf, bytes);
    }

  node->n_list++;
  node->range_min                = MIN(node->range_min, data_off);
  node->range_max                = MAX(node->range_max, data_off + bytes);

  old_sz                         = node->sz;
  node->sz                       = MAX(node->range_max, path[depth - 1].sz);
  node->path[node->depth - 1].sz = node->sz;

  if (old_sz != node->sz)
    {
      ret = lru_updatesz(sb, node->path, node->depth, node->sz);
      if (predict_false(ret < 0))
        {
          goto errout_with_delta;
        }
    }

  finfo("Delta attached to node %p. Now there are %lu nodes and the node has"
        " %lu deltas. Node with range [%u, %u).", node,
        list_length(&MFS_LRU(sb)), list_length(&node->delta),
        node->range_min, node->range_max);

  return ret;

errout_with_delta:
  list_delete(&delta->list);
  kmm_free(delta);

errout_with_node:
  if (!found && node != NULL)
    {
      list_delete(&node->list);
      kmm_free(node);
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: lru_updatesz
 *
 * Description:
 *   Updates size of an fs object in its parent.
 *
 * Input Parameters:
 *   sb     - Superblock instance of the device.
 *   path   - CTZ representation of the path.
 *   depth  - Depth of path.
 *   new_sz - New size
 *
 * Returned Value:
 *  0   - OK
 *  < 0 - Error
 *
 * Assumptions/Limitations:
 *   Adds an entry for the target's parent to update the child's size, and
 *   updates the size in path of everyone that has this child.
 *
 ****************************************************************************/

static int lru_updatesz(FAR struct mfs_sb_s * sb,
                        FAR struct mfs_path_s * const path,
                        const mfs_t depth, const mfs_t new_sz)
{
  int ret = OK;
  struct mfs_node_s *n = NULL;
  mfs_t i;
  bool found;
  char buf[sizeof(mfs_t)];

  DEBUGASSERT(depth > 0);

  list_for_every_entry(&MFS_LRU(sb), n, struct mfs_node_s, list)
    {
      if (n->depth < depth)
        {
          continue;
        }

      found = false;
      for (i = depth; i >= 1; i--)
        {
          if (path[i - 1].ctz.idx_e == 0 && path[i - 1].ctz.pg_e == 0 &&
              mfs_ctz_eq(&n->path[i - 1].ctz, &path[i - 1].ctz) &&
              n->path[i - 1].off == path[i - 1].off)
            {
              found = true;
            }
          else if (path[i - 1].ctz.pg_e != 0 &&
                  mfs_ctz_eq(&n->path[i - 1].ctz, &path[i - 1].ctz))
            {
              found = true;
            }
          else
            {
              break;
            }
        }

      if (found)
        {
          n->path[depth - 1].sz = new_sz;
        }
    }

  if (depth == 1)
    {
      MFS_MN(sb).root_sz = new_sz;
      goto errout;
    }

  memset(buf, 0, sizeof(mfs_t));
  mfs_ser_mfs(new_sz, buf);

  /* This function will be used by mfs_lru_wr itself, but given that if
   * there is no change in size, this won't cause an infinite loop, this
   * should be fine.
   */

  ret = mfs_lru_wr(sb, path[depth - 2].off + offsetof(struct mfs_dirent_s,
                   sz), sizeof(mfs_t), path, depth, buf);
  if (predict_false(ret < 0))
    {
      goto errout;
    }

  path[depth - 1].sz = new_sz;

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mfs_lru_ctzflush(FAR struct mfs_sb_s * const sb,
                     FAR struct mfs_path_s * const path, const mfs_t depth)
{
  struct mfs_node_s *node = NULL;

  lru_nodesearch(sb, path, depth, &node);
  if (node == NULL)
    {
      return OK;
    }

  return lru_nodeflush(sb, path, depth, node, true);
}

int mfs_lru_del(FAR struct mfs_sb_s * const sb, const mfs_t data_off,
                mfs_t bytes, FAR struct mfs_path_s * const path,
                const mfs_t depth)
{
  return lru_wrtooff(sb, data_off, bytes, MFS_LRU_DEL, path, depth, NULL);
}

int mfs_lru_wr(FAR struct mfs_sb_s * const sb, const mfs_t data_off,
               mfs_t bytes, FAR struct mfs_path_s * const path,
               const mfs_t depth, FAR const char *buf)
{
  return lru_wrtooff(sb, data_off, bytes, MFS_LRU_UPD, path, depth, buf);
}

void mfs_lru_init(FAR struct mfs_sb_s * const sb)
{
  list_initialize(&MFS_LRU(sb));
  sb->n_lru = 0;

  finfo("LRU Initialized\n");
}

int mfs_lru_rdfromoff(FAR const struct mfs_sb_s * const sb,
                      const mfs_t data_off,
                      FAR struct mfs_path_s * const path, const mfs_t depth,
                      FAR char *buf, const mfs_t buflen)
{
  /* Requires updated path from the journal. */

  int                     ret       = OK;
  mfs_t                   upper;
  mfs_t                   lower;
  mfs_t                   rem_sz;
  mfs_t                   upper_og;
  mfs_t                   upper_upd;
  mfs_t                   lower_upd;
  FAR char               *tmp;
  struct mfs_ctz_s        ctz;
  FAR struct mfs_node_s  *node      = NULL;
  FAR struct mfs_delta_s *delta     = NULL;

  lru_nodesearch(sb, path, depth, &node);
  if (node == NULL)
    {
      goto errout;
    }

  /* Node is NOT supposed to be freed by the caller, it's a reference to
   * the actual node in the LRU and freeing it could break the entire LRU.
   */

  tmp      = buf;
  ctz      = node->path[node->depth - 1].ctz;
  lower    = data_off;
  upper_og = lower + buflen;
  upper    = upper_og;
  rem_sz   = buflen;

  while (rem_sz > 0)
    {
      mfs_ctz_rdfromoff(sb, ctz, lower, rem_sz, tmp);

      list_for_every_entry(&node->delta, delta, struct mfs_delta_s, list)
        {
          if (delta->upd == NULL)
            {
              /* Delete */

              lower_upd = MAX(lower, delta->off);
              upper_upd = MIN(upper, delta->off + delta->n_b);

              if (lower_upd >= upper_upd)
                {
                  /* Outside range */
                }
              else
                {
                  memmove(tmp + (lower - lower_upd),
                          tmp + (upper_upd - lower), upper - upper_upd);

                  upper -= (upper_upd - lower_upd);
                }
            }
          else
            {
              /* Update */

              lower_upd = MAX(lower, delta->off);
              upper_upd = MIN(upper, delta->off + delta->n_b);

              if (lower_upd >= upper_upd)
                {
                  /* Outside range */
                }
              else
                {
                  memcpy(tmp + (lower_upd - lower),
                         delta->upd + (lower_upd - delta->off),
                         upper_upd - lower_upd);
                }
            }
        }

      tmp    += upper - lower;
      rem_sz -= upper - lower;
      lower   = upper;
      upper   = upper_og;
    }

errout:
  return ret;
}

int mfs_lru_updatedinfo(FAR const struct mfs_sb_s * const sb,
                        FAR struct mfs_path_s * const path,
                        const mfs_t depth)
{
  int                    ret  = OK;
  bool                   found;
  mfs_t                  i;
  FAR struct mfs_node_s *node = NULL;

  DEBUGASSERT(depth > 0);

  list_for_every_entry(&MFS_LRU(sb), node, struct mfs_node_s, list)
    {
      /* TODO: When a directory is newly created, and still in the LRU, its
       * CTZ is (0, 0), and this can match others as well if at same depth,
       * so, in these cases, match the parents, and so on up.
       */

      DEBUGASSERT(node->depth > 0);

      if (node->depth > depth)
        {
          continue;
        }

      /* We need this loop to specifically check the parents in case these
       * entries are all new, and have not been allocated any pages for
       * being stored in the flash. Also we know that the root (depth 1) will
       * be at least common in their paths.
       */

      found = true;
      for (i = node->depth; i >= 1 && found; i--)
        {
          if (path[i - 1].ctz.idx_e == 0 && path[i - 1].ctz.pg_e == 0 &&
              mfs_ctz_eq(&node->path[i - 1].ctz, &path[i - 1].ctz) &&
              node->path[i - 1].off == path[i - 1].off)
            {
              /* OK */
            }
          else if (path[i - 1].ctz.pg_e != 0 &&
                  mfs_ctz_eq(&node->path[i - 1].ctz, &path[i - 1].ctz))
            {
              /* OK */
            }
          else
            {
              found = false;
            }
        }

      if (found)
        {
          path[node->depth - 1].sz = node->path[node->depth - 1].sz;
        }
    }

  return ret;
}

int mfs_lru_updatectz(FAR struct mfs_sb_s * sb,
                      FAR struct mfs_path_s * const path, const mfs_t depth,
                      const struct mfs_ctz_s new_ctz)
{
  int                    ret                            = OK;
  char                   buf[sizeof(struct mfs_ctz_s)];
  FAR struct mfs_node_s *node                           = NULL;

  list_for_every_entry(&MFS_LRU(sb), node, struct mfs_node_s, list)
    {
      if (node->depth >= depth &&
          mfs_ctz_eq(&node->path[depth - 1].ctz, &path[depth - 1].ctz) &&
          node->path[depth - 1].sz == path[depth - 1].sz)
        {
          node->path[depth - 1].ctz = new_ctz;
        }
    }

  memset(buf, 0, sizeof(struct mfs_ctz_s));
  mfs_ser_ctz(&new_ctz, buf);

  ret = mfs_lru_wr(sb, path[depth - 1].off + offsetof(struct mfs_dirent_s,
                   ctz), sizeof(struct mfs_ctz_s), path, depth, buf);
  if (predict_false(ret < 0))
    {
      goto errout;
    }

  path[depth - 1].ctz = new_ctz;

errout:
  return ret;
}
