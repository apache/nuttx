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

static void lru_nodesearch(FAR struct mfs_sb_s * const sb,
                           FAR const struct mfs_path_s * const path,
                           const mfs_t depth, FAR struct mfs_node_s **node);
static bool lru_islrufull(FAR struct mfs_sb_s * const sb);
static bool lru_isnodefull(FAR struct mfs_node_s *node);
static int lru_nodeflush(FAR struct mfs_sb_s * const sb,
                         FAR struct mfs_path_s * const path,
                         const mfs_t depth, const mfs_t ctz_sz,
                         FAR struct mfs_node_s *node, bool clean_node);

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

static void lru_nodesearch(FAR struct mfs_sb_s * const sb,
                           FAR const struct mfs_path_s * const path,
                           const mfs_t depth, FAR struct mfs_node_s **node)
{
  bool                  found;
  mfs_t                 i;
  FAR struct mfs_node_s *n;

  *node = NULL;
  list_for_every_entry(&MFS_LRU(sb), n, struct mfs_node_s, list)
    {
      if (n == NULL)
        {
          break;
        }

      found = true;
      if (n->depth != depth)
        {
          continue;
        }

      if (depth != 0)
        {
          for (i = depth - 1; i + 1 > 0; i--) /* i + 1 prevents underflow. */
            {
              if (n->path[i].off       == path[i].off &&
                  n->path[i].ctz.idx_e == path[i].ctz.idx_e &&
                  n->path[i].ctz.pg_e  == path[i].ctz.pg_e)
                {
                  /* OK */
                }
              else
                {
                  found = false;
                  break;
                }
            }
        }

      if (found)
        {
          finfo("Node search ended with match of node %p at depth %u"
                " for CTZ of %u original size with range [%u, %u).", n,
                n->depth, n->sz, n->range_min, n->range_max);
          *node = n;
          return;
        }
    }

  *node = NULL;
  finfo("Node search ended without match.");
}

/****************************************************************************
 * Name: lru_islrufull
 *
 * Description:
 *   Check whether the number of nodes in the LRU has reaches its limit.
 *
 * Input Parameters:
 *   sb    - Superblock instance of the device.
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
                         const mfs_t depth, const mfs_t ctz_sz,
                         FAR struct mfs_node_s *node, bool clean_node)
{
  int                    ret    = OK;
  FAR struct mfs_delta_s *delta = NULL;
  FAR struct mfs_delta_s *tmp   = NULL;

  if (predict_false(node == NULL))
    {
      return -ENOMEM;
    }

  ret = mfs_ctz_nwrtooff(sb, node, path, depth, ctz_sz,
                        &path[depth - 1].ctz);
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

static int lru_wrtooff(FAR struct mfs_sb_s * const sb,
                       const mfs_t data_off, mfs_t bytes, mfs_t ctz_sz,
                       int op, FAR struct mfs_path_s * const path,
                       const mfs_t depth, FAR const char *buf)
{
  int                    ret        = OK;
  bool                   found      = true;
  FAR struct mfs_node_s  *node      = NULL;
  FAR struct mfs_node_s  *last_node = NULL;
  FAR struct mfs_delta_s *delta     = NULL;

  lru_nodesearch(sb, path, depth, &node);

  if (node == NULL)
    {
      node = kmm_zalloc(sizeof(*node) + (depth * sizeof(struct mfs_path_s)));
      if (predict_false(node == NULL))
        {
          ret = -ENOMEM;
          goto errout;
        }

      node->sz        = ctz_sz;
      node->depth     = depth;
      node->n_list    = 0;
      node->range_max = 0;
      node->range_min = UINT32_MAX;
      list_initialize(&node->delta);
      memcpy(node->path, path, depth * sizeof(struct mfs_path_s));
      found = false;

      finfo("Node not found, allocated, ready to be inserted into LRU.");
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

      ret = lru_nodeflush(sb, path, depth, ctz_sz, node, false);
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

      finfo("Delta is of the update type, has %u bytes of updates." , bytes);
    }

  delta->n_b = bytes;
  delta->off = data_off;
  list_add_tail(&node->delta, &delta->list);
  if (op == MFS_LRU_UPD)
    {
      memcpy(delta->upd, buf, bytes);
    }

  node->n_list++;
  node->range_min = MIN(node->range_min, data_off);
  node->range_max = MAX(node->range_max, data_off + bytes);

  finfo("Delta attached to node. Now there are %lu nodes and the node has"
        " %lu deltas. Node with range [%u, %u).", list_length(&MFS_LRU(sb)),
        list_length(&node->delta), node->range_min, node->range_max);

  return ret;

errout_with_delta:
  kmm_free(delta);

errout_with_node:
  if (!found && node != NULL)
    {
      kmm_free(node);
    }

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mfs_lru_ctzflush(FAR struct mfs_sb_s * const sb,
                     FAR struct mfs_path_s * const path, const mfs_t depth,
                     const mfs_t ctz_sz)
{
  struct mfs_node_s *node = NULL;

  lru_nodesearch(sb, path, depth, &node);
  if (node == NULL)
    {
      return OK;
    }

  return lru_nodeflush(sb, path, depth, ctz_sz, node, true);
}

int mfs_lru_del(FAR struct mfs_sb_s * const sb, const mfs_t data_off,
                mfs_t bytes, mfs_t ctz_sz,
                FAR struct mfs_path_s * const path, const mfs_t depth)
{
  return lru_wrtooff(sb, data_off, bytes, ctz_sz, MFS_LRU_DEL, path, depth,
                    NULL);
}

int mfs_lru_wr(FAR struct mfs_sb_s * const sb, const mfs_t data_off,
               mfs_t bytes, mfs_t ctz_sz, FAR struct mfs_path_s * const path,
               const mfs_t depth, FAR const char *buf)
{
  return lru_wrtooff(sb, data_off, bytes, ctz_sz, MFS_LRU_UPD, path, depth,
                    buf);
}

int mfs_lru_rdfromoff(FAR struct mfs_sb_s * const sb, const mfs_t data_off,
                      FAR struct mfs_path_s * const path, const mfs_t depth,
                      FAR char *buf, const mfs_t buflen)
{
  int                    ret = OK;
  mfs_t                  s;
  mfs_t                  e;
  mfs_t                  end;
  mfs_t                  start;
  mfs_t                  del_b;
  FAR struct mfs_node_s  *node      = NULL;
  FAR struct mfs_delta_s *delta     = NULL;

  finfo("Reading from offset %u, %u bytes", data_off, buflen);

  ret = mfs_ctz_rdfromoff(sb, data_off, path, depth, buf, buflen);
  if (predict_false(ret < 0))
    {
      goto errout;
    }

  lru_nodesearch(sb, path, depth, &node);
  if (node == NULL)
    {
      goto errout;
    }

  start = data_off;
  end   = data_off + buflen;
  del_b = 0;

  do
    {
      list_for_every_entry(&node->delta, delta, struct mfs_delta_s, list)
        {
          if (delta->upd)
            {
              s = MAX(delta->off, start);
              e = MIN(delta->off + delta->n_b, end);

              if (s >= e)
                {
                  continue;
                }

              memcpy(buf + (s - data_off), delta->upd + (s - delta->off),
                    e - s);
            }
          else
            {
              s = MAX(delta->off, start);
              e = MIN(delta->off + delta->n_b, end);

              if (s >= end)
                {
                  continue;
                }

              if (e <= start)
                {
                  del_b += delta->n_b;
                  s = data_off + delta->n_b;
                  e = end;
                  memmove(buf, buf + delta->n_b,
                          buflen - delta->n_b);
                  e = data_off + buflen - delta->n_b;
                }
              else
                {
                  del_b += e - s;
                  memmove(buf + s, buf + e, buflen - (e - s));
                  e = data_off + buflen - (e - s);
                }
            }

          start = s;
          end   = e;
        }

      start = end;
      end   = data_off + buflen;
    }
  while (start != end);

errout:
  return ret;
}

void mfs_lru_init(FAR struct mfs_sb_s * const sb)
{
  list_initialize(&MFS_LRU(sb));
  sb->n_lru = 0;

  finfo("LRU Initialized\n");
}

void mfs_lru_updatedsz(FAR struct mfs_sb_s * const sb,
                       FAR const struct mfs_path_s * const path,
                       const mfs_t depth, mfs_t *n_sz)
{
  mfs_t                  o_sz;
  FAR struct mfs_node_s  *node  = NULL;
  FAR struct mfs_delta_s *delta = NULL;

  if (depth == 0)
    {
      /* Master node */

      o_sz = 0;
    }

  if (depth == 1)
    {
      /* Root node. */

      o_sz = MFS_MN(sb).root_sz;
    }
  else
    {
      o_sz = path[depth - 1].sz;
    }

  *n_sz = o_sz;

  lru_nodesearch(sb, path, depth, &node);
  if (node == NULL)
    {
      finfo("No updates for file size, with depth %u.", depth);
      return;
    }

  list_for_every_entry(&node->delta, delta, struct mfs_delta_s, list)
    {
      finfo("depth %u %u %u %p", depth, delta->n_b, delta->off, delta->upd);
      if (delta->upd == NULL)
        {
          *n_sz -= MIN((*n_sz) - delta->off, delta->n_b);
        }
      else
        {
          *n_sz = MAX(*n_sz, delta->off + delta->n_b);
        }
    }

  finfo("Updated file size is %u with depth %u.", *n_sz, depth);
}
