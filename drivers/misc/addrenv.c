/****************************************************************************
 * drivers/misc/addrenv.c
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

#include <nuttx/config.h>
#include <nuttx/list.h>
#include <nuttx/kmalloc.h>

#include <string.h>

#include <nuttx/drivers/addrenv.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct simple_addrenv_node_s
{
  struct list_node node;
  FAR const struct simple_addrenv_s *addrenv;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct list_node g_addrenv_list = LIST_INITIAL_VALUE(g_addrenv_list);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void simple_addrenv_initialize(FAR const struct simple_addrenv_s *addrenv)
{
  FAR struct simple_addrenv_node_s *node;

  if (addrenv != NULL)
    {
      node = kmm_malloc(sizeof(*node));
      if (node != NULL)
        {
          node->addrenv = addrenv;
          list_add_tail(&g_addrenv_list, &node->node);
        }
    }
}

FAR void *up_addrenv_pa_to_va(uintptr_t pa)
{
  FAR struct simple_addrenv_node_s *node;
  FAR const struct simple_addrenv_s *addrenv;
  uint32_t i;

  list_for_every_entry(&g_addrenv_list, node,
                       struct simple_addrenv_node_s, node)
    {
      addrenv = node->addrenv;
      for (i = 0; addrenv[i].size; i++)
        {
          if (pa - addrenv[i].pa < addrenv[i].size)
            {
              return (FAR void *)(addrenv[i].va + pa - addrenv[i].pa);
            }
        }
    }

  return (FAR void *)pa;
}

uintptr_t up_addrenv_va_to_pa(FAR void *va_)
{
  FAR struct simple_addrenv_node_s *node;
  FAR const struct simple_addrenv_s *addrenv;
  uintptr_t va = (uintptr_t)va_;
  uint32_t i;

  list_for_every_entry(&g_addrenv_list, node,
                       struct simple_addrenv_node_s, node)
    {
      addrenv = node->addrenv;
      for (i = 0; addrenv[i].size; i++)
        {
          uintptr_t tmp = addrenv[i].va;
          if (va - tmp < addrenv[i].size)
            {
              return addrenv[i].pa + (va - tmp);
            }
        }
    }

  return va;
}
