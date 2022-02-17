/****************************************************************************
 * drivers/clk/clk.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk_provider.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/list.h>
#include <nuttx/mutex.h>

#include <debug.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CLK_PROCFS_LINELEN                  80

/****************************************************************************
 * Private Datas
 ****************************************************************************/

static mutex_t g_clk_list_lock            = MUTEX_INITIALIZER;

static struct list_node g_clk_root_list
                            = LIST_INITIAL_VALUE(g_clk_root_list);
static struct list_node g_clk_orphan_list
                            = LIST_INITIAL_VALUE(g_clk_orphan_list);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void clk_list_lock(void);
static void clk_list_unlock(void);

static int clk_fetch_parent_index(FAR struct clk_s *clk,
                                  FAR struct clk_s *parent);
static void clk_init_parent(FAR struct clk_s *clk);
static void clk_reparent(FAR struct clk_s *clk, FAR struct clk_s *parent);

static uint32_t clk_recalc(FAR struct clk_s *clk, uint32_t parent_rate);
static void __clk_recalc_rate(FAR struct clk_s *clk);

static void clk_calc_subtree(FAR struct clk_s *clk, uint32_t new_rate,
                             FAR struct clk_s *new_parent,
                             uint8_t p_index);
static FAR struct clk_s *clk_calc_new_rates(FAR struct clk_s *clk,
                                            uint32_t rate);
static void clk_change_rate(FAR struct clk_s *clk,
                            uint32_t best_parent_rate);

static uint32_t __clk_get_rate(FAR struct clk_s *clk);
static uint32_t __clk_round_rate(FAR struct clk_s *clk, uint32_t rate);
static int __clk_enable(FAR struct clk_s *clk);
static int __clk_disable(FAR struct clk_s *clk);

static struct clk_s *__clk_lookup(FAR const char *name,
                                  FAR struct clk_s *clk);
static int __clk_register(FAR struct clk_s *clk);

static void clk_disable_unused_subtree(FAR struct clk_s *clk);

/* File system methods */

#if !defined(CONFIG_FS_PROCFS_EXCLUDE_CLK) && defined(CONFIG_FS_PROCFS)

static int clk_procfs_open(FAR struct file *filep, FAR const char *relpath,
                           int oflags, mode_t mode);
static int clk_procfs_close(FAR struct file *filep);
static ssize_t clk_procfs_read(FAR struct file *filep, FAR char *buffer,
                               size_t buflen);
static int clk_procfs_dup(FAR const struct file *oldp,
                          FAR struct file *newp);
static int clk_procfs_stat(const char *relpath, struct stat *buf);

#endif /* !defined(CONFIG_FS_PROCFS_EXCLUDE_CLK) && defined(CONFIG_FS_PROCFS) */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if !defined(CONFIG_FS_PROCFS_EXCLUDE_CLK) && defined(CONFIG_FS_PROCFS)

const struct procfs_operations clk_procfsoperations =
{
  clk_procfs_open,       /* open */
  clk_procfs_close,      /* close */
  clk_procfs_read,       /* read */
  NULL,                  /* write */

  clk_procfs_dup,        /* dup */

  NULL,                  /* opendir */
  NULL,                  /* closedir */
  NULL,                  /* readdir */
  NULL,                  /* rewinddir */

  clk_procfs_stat,       /* stat */
};

#endif /* !defined(CONFIG_FS_PROCFS_EXCLUDE_CLK) && defined(CONFIG_FS_PROCFS) */

/****************************************************************************
 * Private Function
 ****************************************************************************/

#if !defined(CONFIG_FS_PROCFS_EXCLUDE_CLK) && defined(CONFIG_FS_PROCFS)

static int clk_procfs_open(FAR struct file *filep, FAR const char *relpath,
                           int oflags, mode_t mode)
{
  FAR struct procfs_file_s *priv;

  if ((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0)
    {
      return -EACCES;
    }

  priv = kmm_zalloc(sizeof(struct procfs_file_s));
  if (!priv)
    {
      return -ENOMEM;
    }

  filep->f_priv = priv;
  return OK;
}

static int clk_procfs_close(FAR struct file *filep)
{
  FAR struct procfs_file_s *priv = filep->f_priv;

  kmm_free(priv);
  filep->f_priv = NULL;
  return OK;
}

static size_t clk_procfs_printf(FAR char *buffer, size_t buflen,
                                off_t *pos, FAR const char *fmt,
                                ...)
{
  char tmp[CLK_PROCFS_LINELEN];
  size_t tmplen;
  va_list ap;

  va_start(ap, fmt);
  tmplen = vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);

  return procfs_memcpy(tmp, tmplen, buffer, buflen, pos);
}

static size_t clk_procfs_show_subtree(FAR FAR struct clk_s *clk, int level,
                                      FAR char *buffer, size_t buflen,
                                      off_t *pos)
{
  FAR struct clk_s *child;
  size_t oldlen = buflen;
  size_t ret;

  if (strchr(clk_get_name(clk), '/'))
    {
      clk_list_unlock();
    }

  ret = clk_procfs_printf(buffer, buflen, pos, "%*s%-*s %11d %11u %11d\n",
                          level * 2, "", 40 - level * 2, clk_get_name(clk),
                          clk_is_enabled(clk), clk_get_rate(clk),
                          clk_get_phase(clk));
  buffer += ret;
  buflen -= ret;

  if (strchr(clk_get_name(clk), '/'))
    {
      clk_list_lock();
    }

  if (buflen > 0)
    {
      list_for_every_entry(&clk->children, child, struct clk_s, node)
        {
          ret = clk_procfs_show_subtree(child, level + 1,
                                        buffer, buflen, pos);
          buffer += ret;
          buflen -= ret;

          if (buflen == 0)
            {
              break; /* No enough space, return */
            }
        }
    }

  return oldlen - buflen;
}

static size_t clk_procfs_showtree(FAR char *buffer,
                                  size_t buflen, off_t *pos)
{
  FAR struct clk_s *clk;
  size_t oldlen = buflen;
  size_t ret;

  clk_list_lock();

  list_for_every_entry(&g_clk_root_list, clk, struct clk_s, node)
    {
      ret = clk_procfs_show_subtree(clk, 0, buffer, buflen, pos);
      buffer += ret;
      buflen -= ret;

      if (buflen == 0)
        {
          goto out; /* No enough space, return */
        }
    }

  list_for_every_entry(&g_clk_orphan_list, clk, struct clk_s, node)
    {
      ret = clk_procfs_show_subtree(clk, 0, buffer, buflen, pos);
      buffer += ret;
      buflen -= ret;

      if (buflen == 0)
        {
          goto out; /* No enough space, return */
        }
    }

out:
  clk_list_unlock();
  return oldlen - buflen;
}

static ssize_t clk_procfs_read(FAR struct file *filep,
                               FAR char *buffer, size_t buflen)
{
  off_t pos = filep->f_pos;
  size_t oldlen = buflen;
  size_t ret;

  ret = clk_procfs_printf(buffer, buflen, &pos,
                         "%8s%44s%12s%12s\n",
                         "clock", "enable_cnt", "rate", "phase");
  buffer += ret;
  buflen -= ret;

  if (buflen > 0)
    {
      ret = clk_procfs_showtree(buffer, buflen, &pos);
      buffer += ret;
      buflen -= ret;
    }

  filep->f_pos += oldlen - buflen;
  return oldlen - buflen;
}

static int clk_procfs_dup(FAR const struct file *oldp,
                          FAR struct file *newp)
{
  FAR struct procfs_file_s *oldpriv;
  FAR struct procfs_file_s *newpriv;

  oldpriv = oldp->f_priv;
  DEBUGASSERT(oldpriv);

  newpriv = kmm_zalloc(sizeof(struct procfs_file_s));
  if (!newpriv)
    {
      return -ENOMEM;
    }

  memcpy(newpriv, oldpriv, sizeof(struct procfs_file_s));
  newp->f_priv = newpriv;
  return OK;
}

static int clk_procfs_stat(const char *relpath, struct stat *buf)
{
  /* File/directory size, access block size */

  buf->st_mode    = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;
  buf->st_size    = 0;
  buf->st_blksize = 0;
  buf->st_blocks  = 0;
  return OK;
}

#endif /* !defined(CONFIG_FS_PROCFS_EXCLUDE_CLK) && defined(CONFIG_FS_PROCFS) */

static void clk_list_lock(void)
{
  nxmutex_lock(&g_clk_list_lock);
}

static void clk_list_unlock(void)
{
  nxmutex_unlock(&g_clk_list_lock);
}

static int clk_fetch_parent_index(FAR struct clk_s *clk,
                                  FAR struct clk_s *parent)
{
  int i;

  if (!parent)
    {
      return -EINVAL;
    }

  for (i = 0; i < clk->num_parents; i++)
    {
      if (!strcmp(clk->parent_names[i], parent->name))
        {
          return i;
        }
    }

  return -EINVAL;
}

static void clk_reparent(FAR struct clk_s *clk, FAR struct clk_s *parent)
{
  list_delete(&clk->node);

  if (parent)
    {
      if (parent->new_child == clk)
        {
          parent->new_child = NULL;
        }
      list_add_head(&parent->children, &clk->node);
    }

  clk->parent = parent;
}

static uint32_t clk_recalc(FAR struct clk_s *clk, uint32_t parent_rate)
{
  if (clk->ops->recalc_rate)
    {
      return clk->ops->recalc_rate(clk, parent_rate);
    }

  return parent_rate;
}

static void __clk_recalc_rate(FAR struct clk_s *clk)
{
  uint32_t parent_rate = 0;
  FAR struct clk_s *child;

  if (clk->parent)
    {
      parent_rate = __clk_get_rate(clk->parent);
    }

  clk->rate = clk_recalc(clk, parent_rate);

  list_for_every_entry(&clk->children, child, struct clk_s, node)
    {
      __clk_recalc_rate(child);
    }
}

static void clk_calc_subtree(FAR struct clk_s *clk, uint32_t new_rate,
                             FAR struct clk_s *new_parent, uint8_t p_index)
{
  FAR struct clk_s *child;

  clk->new_rate = new_rate;
  clk->new_parent = new_parent;
  clk->new_parent_index = p_index;

  clk->new_child = NULL;
  if (new_parent && new_parent != clk->parent)
    {
      new_parent->new_child = clk;
    }

  list_for_every_entry(&clk->children, child, struct clk_s, node)
    {
      child->new_rate = clk_recalc(child, new_rate);
      clk_calc_subtree(child, child->new_rate, NULL, 0);
    }
}

static FAR struct clk_s *clk_calc_new_rates(FAR struct clk_s *clk,
                                      uint32_t rate)
{
  FAR struct clk_s *top = clk;
  FAR struct clk_s *old_parent;
  FAR struct clk_s *parent;
  uint32_t best_parent_rate = 0;
  uint32_t new_rate = 0;
  int p_index = 0;

  if (!clk)
    {
      return NULL;
    }

  parent = old_parent = clk->parent;
  if (parent)
    {
      best_parent_rate = __clk_get_rate(parent);
    }

  if (clk->ops->determine_rate)
    {
      new_rate = clk->ops->determine_rate(clk, rate,
                                          &best_parent_rate, &parent);
    }
  else if (clk->ops->round_rate)
    {
      new_rate = clk->ops->round_rate(clk, rate, &best_parent_rate);
    }
  else if (!parent || !(clk->flags & CLK_SET_RATE_PARENT))
    {
      clk->new_rate = clk->rate;
      return NULL;
    }
  else
    {
      top = clk_calc_new_rates(parent, rate);
      new_rate = parent->new_rate;
      goto out;
    }

  if (parent)
    {
      p_index = clk_fetch_parent_index(clk, parent);
      if (p_index < 0)
        {
          return NULL;
        }
    }

  if ((clk->flags & CLK_SET_RATE_PARENT) && parent &&
      best_parent_rate != __clk_get_rate(parent))
    {
      top = clk_calc_new_rates(parent, best_parent_rate);
    }

out:
  clk_calc_subtree(clk, new_rate, parent, p_index);
  return top;
}

static void clk_change_rate(FAR struct clk_s *clk, uint32_t best_parent_rate)
{
  FAR struct clk_s *child;
  FAR struct clk_s *old_parent;
  bool skip_set_rate = false;

  list_for_every_entry(&clk->children, child, struct clk_s, node)
    {
      if (child->new_parent && child->new_parent != clk)
        {
          continue;
        }
      if (child->new_rate > __clk_get_rate(child))
        {
          clk_change_rate(child, clk->new_rate);
        }
    }

  old_parent = clk->parent;

  if (clk->new_parent && clk->new_parent != clk->parent)
    {
      if (clk->enable_count)
        {
          clk_enable(clk->new_parent);
          clk_enable(clk);
        }

      clk_reparent(clk, clk->new_parent);

      if (clk->ops->set_rate_and_parent)
        {
          skip_set_rate = true;
          clk->ops->set_rate_and_parent(clk, clk->new_rate, best_parent_rate,
                                        clk->new_parent_index);
        }
      else if (clk->ops->set_parent)
        {
          clk->ops->set_parent(clk, clk->new_parent_index);
        }

      if (clk->enable_count)
        {
          clk_disable(clk);
          clk_disable(old_parent);
        }
    }

  if (!skip_set_rate && clk->ops->set_rate)
    {
      clk->ops->set_rate(clk, clk->new_rate, best_parent_rate);
    }

  clk->rate = clk->new_rate;

  list_for_every_entry(&clk->children, child, struct clk_s, node)
    {
      if (child->new_parent && child->new_parent != clk)
        {
          continue;
        }
      if (child->new_rate != __clk_get_rate(child))
        {
          clk_change_rate(child, clk->new_rate);
        }
    }

  if (clk->new_child && clk->new_child->new_rate !=
      __clk_get_rate(clk->new_child))
    {
      clk_change_rate(clk->new_child, clk->new_rate);
    }
}

static struct clk_s *__clk_lookup(FAR const char *name,
                                  FAR struct clk_s *clk)
{
  FAR struct clk_s *child;
  FAR struct clk_s *ret;

  if (!strcmp(clk->name, name))
    {
      return clk;
    }

  list_for_every_entry(&clk->children, child, struct clk_s, node)
    {
      ret = __clk_lookup(name, child);
      if (ret)
        {
          return ret;
        }
    }

  return NULL;
}

static uint32_t __clk_get_rate(FAR struct clk_s *clk)
{
  uint32_t parent_rate;

  if (!clk)
    {
      return 0;
    }

  if (clk->rate == 0)
    {
      parent_rate = __clk_get_rate(clk->parent);
      clk->rate = clk_recalc(clk, parent_rate);
    }

  return clk->rate;
}

static uint32_t __clk_round_rate(FAR struct clk_s *clk, uint32_t rate)
{
  uint32_t parent_rate = 0;
  FAR struct clk_s *parent;

  if (!clk)
    {
      return 0;
    }

  parent = clk->parent;
  if (parent)
    {
      parent_rate = __clk_get_rate(parent);
    }

  if (clk->ops->determine_rate)
    {
      return clk->ops->determine_rate(clk, rate, &parent_rate, &parent);
    }
  else if (clk->ops->round_rate)
    {
      return clk->ops->round_rate(clk, rate, &parent_rate);
    }
  else if (clk->flags & CLK_SET_RATE_PARENT)
    {
      return __clk_round_rate(clk->parent, rate);
    }
  else
    {
      return __clk_get_rate(clk);
    }
}

static int __clk_enable(FAR struct clk_s *clk)
{
  int ret = 0;

  if (!clk)
    {
      return 0;
    }

  if (clk->enable_count == 0)
    {
      ret = __clk_enable(clk->parent);
      if (ret < 0)
        {
          return ret;
        }

      if (clk->ops->enable)
        {
          ret = clk->ops->enable(clk);
          if (ret < 0)
            {
              __clk_disable(clk->parent);
              return ret;
            }
        }
    }

  return ++clk->enable_count;
}

static int __clk_disable(FAR struct clk_s *clk)
{
  if (!clk || clk->enable_count == 0)
    {
      return 0;
    }

  if (clk->flags & CLK_IS_CRITICAL)
    {
      return 0;
    }

  if (--clk->enable_count == 0)
    {
      if (clk->ops->disable)
        {
          clk->ops->disable(clk);
        }

      if (clk->parent)
        {
          __clk_disable(clk->parent);
        }
    }

  return clk->enable_count + 1;
}

static void clk_init_parent(FAR struct clk_s *clk)
{
  uint8_t index;

  if (!clk->num_parents)
    {
      return;
    }

  if (clk->num_parents == 1)
    {
      clk->parent = clk_get(clk->parent_names[0]);
      return;
    }

  if (!clk->ops->get_parent)
    {
      return;
    };

  index = clk->ops->get_parent(clk);
  clk->parent = clk_get_parent_by_index(clk, index);
}

static int __clk_register(FAR struct clk_s *clk)
{
  FAR struct clk_s *orphan;
  FAR struct clk_s *temp;
  uint8_t i;

  if (!clk)
    {
      return -EINVAL;
    }

  if (clk->ops->set_rate &&
    !((clk->ops->round_rate || clk->ops->determine_rate) &&
      clk->ops->recalc_rate))
    {
      return -EINVAL;
    }

  if (clk->ops->set_parent && !clk->ops->get_parent)
    {
      return -EINVAL;
    }

  if (clk->ops->set_rate_and_parent &&
      !(clk->ops->set_parent && clk->ops->set_rate))
    {
      return -EINVAL;
    }

  clk_init_parent(clk);

  clk_list_lock();

  if (clk->parent)
    {
      list_add_head(&clk->parent->children, &clk->node);
    }
  else if (!clk->num_parents)
    {
      list_add_head(&g_clk_root_list, &clk->node);
    }
  else
    {
      list_add_head(&g_clk_orphan_list, &clk->node);
    }

  list_for_every_entry_safe(&g_clk_orphan_list, orphan,
                            temp, struct clk_s, node)
    {
      if (orphan->num_parents && orphan->ops->get_parent)
        {
          i = orphan->ops->get_parent(orphan);
          if (!strcmp(clk->name, orphan->parent_names[i]))
            {
              clk_reparent(orphan, clk);
            }
        }
      else if (orphan->num_parents)
        {
          for (i = 0; i < orphan->num_parents; i++)
            {
              if (!strcmp(clk->name, orphan->parent_names[i]))
                {
                  clk_reparent(orphan, clk);
                  break;
                }
            }
        }
    }

  clk_list_unlock();
  return 0;
}

static void clk_disable_unused_subtree(FAR struct clk_s *clk)
{
  FAR struct clk_s *child = NULL;

  list_for_every_entry(&clk->children, child, struct clk_s, node)
    {
      clk_disable_unused_subtree(child);
    }

  if (clk->enable_count)
    {
      return;
    }

  if (clk_is_enabled(clk))
    {
      if (clk->flags & CLK_IS_CRITICAL)
        {
          __clk_enable(clk);
        }
      else if (clk->ops->disable)
        {
          clk->ops->disable(clk);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void clk_disable_unused(void)
{
  FAR struct clk_s *root_clk = NULL;

  clk_list_lock();

  list_for_every_entry(&g_clk_root_list, root_clk, struct clk_s, node)
    {
      clk_disable_unused_subtree(root_clk);
    }

  list_for_every_entry(&g_clk_orphan_list, root_clk, struct clk_s, node)
    {
      clk_disable_unused_subtree(root_clk);
    }

  clk_list_unlock();
}

int clk_disable(FAR struct clk_s *clk)
{
  return __clk_disable(clk);
}

int clk_enable(FAR struct clk_s *clk)
{
  return __clk_enable(clk);
}

uint32_t clk_round_rate(FAR struct clk_s *clk, uint32_t rate)
{
  return __clk_round_rate(clk, rate);
}

int clk_set_rate(FAR struct clk_s *clk, uint32_t rate)
{
  uint32_t parent_rate;
  FAR struct clk_s *top;
  int ret = 0;

  if (!clk)
    {
      return 0;
    }

  if (rate == __clk_get_rate(clk))
    {
      goto out;
    }

  if ((clk->flags & CLK_SET_RATE_GATE) && clk->enable_count)
    {
      ret = -EBUSY;
      goto out;
    }

  top = clk_calc_new_rates(clk, rate);
  if (!top)
    {
      ret = -EINVAL;
      goto out;
    }

  if (top->new_parent)
    {
      parent_rate = __clk_get_rate(top->new_parent);
    }
  else if (top->parent)
    {
      parent_rate = __clk_get_rate(top->parent);
    }
  else
    {
      parent_rate = 0;
    }

  clk_change_rate(top, parent_rate);

out:
  return ret;
}

int clk_set_rates(FAR const struct clk_rate_s *rates)
{
  FAR struct clk_s *clk;
  int ret;

  if (!rates)
    {
      return 0;
    }

  while (rates->name)
    {
      clk = clk_get(rates->name);
      if (!clk)
        {
          return -EINVAL;
        }

      ret = clk_set_rate(clk, rates->rate);
      if (ret < 0)
        {
          return ret;
        }

      rates++;
    }

  return 0;
}

int clk_set_phase(FAR struct clk_s *clk, int degrees)
{
  int ret = -EINVAL;

  if (!clk)
    {
      return 0;
    }

  degrees %= 360;
  if (degrees < 0)
    {
      degrees += 360;
    }

  if (clk->ops->set_phase)
    {
      ret = clk->ops->set_phase(clk, degrees);
    }

  return ret;
}

int clk_get_phase(FAR struct clk_s *clk)
{
  if (!clk || !clk->ops->get_phase)
    {
      return 0;
    }

  return clk->ops->get_phase(clk);
}

FAR const char *clk_get_name(FAR const struct clk_s *clk)
{
  return !clk ? NULL : clk->name;
}

int clk_is_enabled(FAR struct clk_s *clk)
{
  if (!clk)
    {
      return 0;
    }

  /* when hardware .is_enabled missing, used software counter */

  if (!clk->ops->is_enabled)
    {
      return clk->enable_count;
    }

  return clk->ops->is_enabled(clk);
}

FAR struct clk_s *clk_get(FAR const char *name)
{
  FAR struct clk_s *root_clk = NULL;
  FAR struct clk_s *ret = NULL;

  if (!name)
    {
      return NULL;
    }

  clk_list_lock();

  list_for_every_entry(&g_clk_root_list, root_clk, struct clk_s, node)
    {
      ret = __clk_lookup(name, root_clk);
      if (ret)
        {
          goto out;
        }
    }

  list_for_every_entry(&g_clk_orphan_list, root_clk, struct clk_s, node)
    {
      ret = __clk_lookup(name, root_clk);
      if (ret)
        {
          goto out;
        }
    }

out:
  clk_list_unlock();

#ifdef CONFIG_CLK_RPMSG
  if (ret == NULL)
    {
      ret = clk_register_rpmsg(name, CLK_GET_RATE_NOCACHE);
    }
#endif

  return ret;
}

int clk_set_parent(FAR struct clk_s *clk, FAR struct clk_s *parent)
{
  FAR struct clk_s *old_parent = NULL;
  int ret = 0;
  int index = 0;

  if (!clk)
    {
      return 0;
    }

  if (clk->num_parents > 1 && !clk->ops->set_parent)
    {
      return -ENOSYS;
    }

  if (clk->parent == parent)
    {
      goto out;
    }

  if ((clk->flags & CLK_SET_PARENT_GATE) && clk->enable_count)
    {
      ret = -EBUSY;
      goto out;
    }

  if (parent)
    {
      index = clk_fetch_parent_index(clk, parent);
      if (index < 0)
        {
          ret = index;
          goto out;
        }
    }

  old_parent = clk->parent;

  if (clk->enable_count)
    {
      clk_enable(parent);
      clk_enable(clk);
    }

  clk_reparent(clk, parent);

  if (parent && clk->ops->set_parent)
    {
      ret = clk->ops->set_parent(clk, index);
    }

  if (ret < 0)
    {
      clk_reparent(clk, old_parent);

      if (clk->enable_count)
        {
          clk_disable(clk);
          clk_disable(parent);
        }

      goto out;
    }

  if (clk->enable_count)
    {
      clk_disable(clk);
      clk_disable(old_parent);
    }

  __clk_recalc_rate(clk);

out:
  return ret;
}

FAR struct clk_s *clk_get_parent_by_index(FAR struct clk_s *clk,
                                          uint8_t index)
{
  if (!clk || index >= clk->num_parents)
    {
      return NULL;
    }

  return clk_get(clk->parent_names[index]);
}

FAR struct clk_s *clk_get_parent(FAR struct clk_s *clk)
{
  return !clk ? NULL : clk->parent;
}

uint32_t clk_get_rate(FAR struct clk_s *clk)
{
  if (!clk)
    {
      return 0;
    }

  if (clk->flags & CLK_GET_RATE_NOCACHE)
    {
      __clk_recalc_rate(clk);
    }

  return __clk_get_rate(clk);
}

FAR struct clk_s *clk_register(FAR const char *name,
                               FAR const char * const *parent_names,
                               uint8_t num_parents, uint8_t flags,
                               FAR const struct clk_ops_s *ops,
                               FAR void *private_data, size_t private_size)
{
  FAR struct clk_s *clk;
  size_t off;
  size_t len;
  int i;

  off = len = sizeof(struct clk_s) + num_parents * sizeof(char *);
  if (!(flags & CLK_PARENT_NAME_IS_STATIC))
    {
      for (i = 0; i < num_parents; i++)
        {
          len += strlen(parent_names[i]) + 1;
        }
    }

  len += private_size;

  if (flags & CLK_NAME_IS_STATIC)
    {
      clk = kmm_zalloc(len);
      if (!clk)
        {
          return NULL;
        }

      clk->name = name;
    }
  else
    {
      clk = kmm_zalloc(len + strlen(name) + 1);
      if (!clk)
        {
          return NULL;
        }

      clk->name = (char *)clk + len;
      strcpy((char *)clk->name, name);
    }

  clk->ops = ops;
  clk->num_parents = num_parents;
  clk->flags = flags;

  clk->private_data = (char *)clk + off;
  memcpy(clk->private_data, private_data, private_size);
  off += private_size;

  for (i = 0; i < num_parents; i++)
    {
      if (flags & CLK_PARENT_NAME_IS_STATIC)
        {
          clk->parent_names[i] = parent_names[i];
        }
      else
        {
          clk->parent_names[i] = (char *)clk + off;
          strcpy((char *)clk->parent_names[i], parent_names[i]);
          off += strlen(parent_names[i]) + 1;
        }
    }

  list_initialize(&clk->node);
  list_initialize(&clk->children);

  if (!__clk_register(clk))
    {
      return clk;
    }

  kmm_free(clk);
  return NULL;
}
