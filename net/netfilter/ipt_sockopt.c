/****************************************************************************
 * net/netfilter/ipt_sockopt.c
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

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/kmalloc.h>

#include "netfilter/iptables.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SWAP(a,b,t)    do { t = a; a = b; b = t; } while (0)

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Structure to store all info we need, including table data and
 * init/apply functions.
 */

struct ipt_table_s
{
  FAR struct ipt_replace *repl;
  FAR struct ipt_replace *(*init_func)(void);
  FAR int (*apply_func)(FAR const struct ipt_replace *);
};

/* Following structs represent the layout of an entry with standard/error
 * target (without matches). Mainly used to simplify initialization (entry
 * creation), not suggested to use under other situations, because there
 * might be matches between entry and target in data from user space.
 */

struct ipt_standard_entry_s
{
  struct ipt_entry entry;
  struct xt_standard_target target;
};

struct ipt_error_entry_s
{
  struct ipt_entry entry;
  struct xt_error_target target;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ipt_table_s g_tables[] =
{
#ifdef CONFIG_NET_NAT
  {NULL, ipt_nat_init, ipt_nat_apply},
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipt_table_init
 *
 * Description:
 *   Try initialize the table data if not initialized.
 *
 ****************************************************************************/

static void ipt_table_init(FAR struct ipt_table_s *table)
{
  if (table->repl == NULL && table->init_func != NULL)
    {
      table->repl = table->init_func();
    }
}

/****************************************************************************
 * Name: ipt_table
 *
 * Description:
 *   Find table data by table name.
 *
 ****************************************************************************/

static FAR struct ipt_table_s *ipt_table(FAR const char *name)
{
  int i;
  for (i = 0; i < ARRAY_SIZE(g_tables); i++)
    {
      ipt_table_init(&g_tables[i]);
      if (g_tables[i].repl != NULL &&
          strcmp(g_tables[i].repl->name, name) == 0)
        {
          return &g_tables[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: ipt_table_repl
 *
 * Description:
 *   Find table data by table name.
 *
 ****************************************************************************/

static FAR struct ipt_replace *ipt_table_repl(FAR const char *name)
{
  FAR struct ipt_table_s *table = ipt_table(name);
  if (table)
    {
      return table->repl;
    }

  return NULL;
}

/****************************************************************************
 * Name: get_info
 *
 * Description:
 *   Fill table info into ipt_getinfo structure.
 *
 * Input Parameters:
 *   get, len - The parameters from getsockopt.
 *
 ****************************************************************************/

static int get_info(FAR struct ipt_getinfo *get, FAR socklen_t *len)
{
  FAR struct ipt_replace *repl;

  if (*len != sizeof(*get))
    {
      return -EINVAL;
    }

  repl = ipt_table_repl(get->name);
  if (repl == NULL)
    {
      return -ENOENT;
    }

  get->valid_hooks = repl->valid_hooks;
  memcpy(get->hook_entry, repl->hook_entry, sizeof(get->hook_entry));
  memcpy(get->underflow, repl->underflow, sizeof(get->underflow));
  get->num_entries = repl->num_entries;
  get->size = repl->size;

  return OK;
}

/****************************************************************************
 * Name: get_entries
 *
 * Description:
 *   Fill entry info into ipt_get_entries structure.
 *
 * Input Parameters:
 *   get, len - The parameters from getsockopt.
 *
 ****************************************************************************/

static int get_entries(FAR struct ipt_get_entries *get, FAR socklen_t *len)
{
  FAR struct ipt_replace *repl;

  if (*len < sizeof(*get) || *len != sizeof(*get) + get->size)
    {
      return -EINVAL;
    }

  repl = ipt_table_repl(get->name);
  if (repl == NULL)
    {
      return -ENOENT;
    }

  if (get->size != repl->size)
    {
      return -EAGAIN;
    }

  memcpy(get->entrytable, repl->entries, get->size);

  return OK;
}

/****************************************************************************
 * Name: check_replace
 *
 * Description:
 *   Check whether an ipt_replace structure from user space is valid.
 *
 * Input Parameters:
 *   repl - The ipt_replace structure to check.
 *
 * Returned Value:
 *   OK if repl is valid, otherwise -EINVAL.
 *
 ****************************************************************************/

static int check_replace(FAR const struct ipt_replace *repl)
{
  FAR struct ipt_entry *entry;
  unsigned int entry_count = 0;

  ipt_entry_for_every(entry, repl->entries, repl->size)
    {
      entry_count++;
    }

  if (entry_count != repl->num_entries)
    {
      return -EINVAL;
    }

  /* May add more checks later. */

  return OK;
}

/****************************************************************************
 * Name: replace_entries
 *
 * Description:
 *   Apply replace data to kernel tables.
 *
 * Input Parameters:
 *   repl, len - The parameters from setsockopt.
 *
 ****************************************************************************/

static int replace_entries(FAR const struct ipt_replace *repl, socklen_t len)
{
  int ret;
  FAR struct ipt_replace *new_repl;
  FAR struct ipt_table_s *table = ipt_table(repl->name);

  if (table == NULL || table->repl == NULL)
    {
      return -ENOENT;
    }

  if (len < sizeof(*repl) || len != sizeof(*repl) + repl->size ||
      repl->valid_hooks != table->repl->valid_hooks)
    {
      return -EINVAL;
    }

  /* Check replace struct before applying it. */

  ret = check_replace(repl);
  if (ret != OK)
    {
      return ret;
    }

  new_repl = kmm_malloc(sizeof(*new_repl) + repl->size);
  if (new_repl == NULL)
    {
      return -ENOMEM;
    }

  /* Try to apply the config in replace data. */

  ret = table->apply_func(repl);

  /* If successfully applied, save data into kernel space. */

  if (ret == OK)
    {
      FAR struct ipt_replace *tmp;
      memcpy(new_repl, repl, sizeof(*repl) + repl->size);
      SWAP(table->repl, new_repl, tmp);
    }

  kmm_free(new_repl);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipt_alloc_table
 *
 * Description:
 *   Allocate an initial table info with valid_hooks specified.
 *   Will generate a default entry with standard target for each valid hook,
 *   and an entry with error target at the end of entry table.
 *
 * Input Parameters:
 *   table       - The name of the table.
 *   valid_hooks - The valid_hooks of the table, it's a bitmask.
 *
 * Returned Value:
 *   Newly generated ipt_replace structure.
 *
 ****************************************************************************/

FAR struct ipt_replace *ipt_alloc_table(FAR const char *table,
                                        unsigned int valid_hooks)
{
  FAR struct ipt_replace *repl;
  FAR struct ipt_standard_entry_s *entry;
  FAR struct ipt_error_entry_s *error_entry;
  size_t entry_size;
  unsigned int hook;
  unsigned int num_hooks;

  if (valid_hooks == 0 || valid_hooks > (1 << NF_INET_NUMHOOKS))
    {
      return NULL;
    }

  num_hooks = popcount(valid_hooks);

  /* There will be num_hooks entries with standard target (ACCEPT). */

  entry_size = num_hooks * sizeof(struct ipt_standard_entry_s);

  /* An error target as final entry. */

  entry_size += sizeof(struct ipt_error_entry_s);

  repl = kmm_zalloc(sizeof(*repl) + entry_size);
  if (repl == NULL)
    {
      return NULL;
    }

  strcpy(repl->name, table);
  repl->valid_hooks = valid_hooks;
  repl->num_entries = num_hooks + 1;
  repl->size = entry_size;

  entry = (FAR struct ipt_standard_entry_s *)(repl + 1);

  for (hook = 0; hook < NF_INET_NUMHOOKS; hook++)
    {
      if ((valid_hooks >> hook & 0x01) == 0)
        {
          continue;
        }

      repl->hook_entry[hook] = (uintptr_t)entry - (uintptr_t)(repl + 1);
      repl->underflow[hook] = repl->hook_entry[hook];

      entry->target.verdict = -NF_ACCEPT - 1;
      IPT_FILL_ENTRY(entry, XT_STANDARD_TARGET);

      entry++;
    }

  error_entry = (FAR struct ipt_error_entry_s *)entry;
  strcpy(error_entry->target.errorname, XT_ERROR_TARGET);
  IPT_FILL_ENTRY(error_entry, XT_ERROR_TARGET);

  return repl;
}

/****************************************************************************
 * Name: ipt_setsockopt
 *
 * Description:
 *   setsockopt function of iptables.
 *
 ****************************************************************************/

int ipt_setsockopt(FAR struct socket *psock, int option,
                   FAR const void *value, socklen_t value_len)
{
  switch (option)
    {
      case IPT_SO_SET_REPLACE:
        return replace_entries(value, value_len);

      default:
        return -ENOPROTOOPT;
    }
}

/****************************************************************************
 * Name: ipt_getsockopt
 *
 * Description:
 *   getsockopt function of iptables.
 *
 ****************************************************************************/

int ipt_getsockopt(FAR struct socket *psock, int option,
                   FAR void *value, FAR socklen_t *value_len)
{
  switch (option)
    {
      case IPT_SO_GET_INFO:
        return get_info(value, value_len);

      case IPT_SO_GET_ENTRIES:
        return get_entries(value, value_len);

      default:
        return -ENOPROTOOPT;
    }
}
