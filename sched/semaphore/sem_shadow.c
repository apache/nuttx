/****************************************************************************
 * sched/semaphore/sem_shadow.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>

#include "semaphore/semaphore.h"

#ifdef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SHADOW_BUCKETS 64
#define SHADOW_BUCKET(paddr) (((paddr) / CONFIG_MM_PGSIZE) % SHADOW_BUCKETS)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sem_shadow_s
{
  FAR struct sem_shadow_s *flink;
  uintptr_t paddr;
  int refs;
  sem_t sem;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct sem_shadow_s *g_shadows[SHADOW_BUCKETS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR struct sem_shadow_s *shadow_find(uintptr_t paddr)
{
  FAR struct sem_shadow_s *shadow = g_shadows[SHADOW_BUCKET(paddr)];

  while (shadow != NULL && shadow->paddr != paddr)
    {
      shadow = shadow->flink;
    }

  return shadow;
}

static bool shadow_unlink(FAR struct sem_shadow_s *shadow)
{
  FAR struct sem_shadow_s **curr = &g_shadows[SHADOW_BUCKET(shadow->paddr)];

  while (*curr != shadow)
    {
      curr = &(*curr)->flink;
    }

  *curr         = shadow->flink;
  shadow->paddr = 0;
  return shadow->refs == 0;
}

static void shadow_free(FAR struct sem_shadow_s *shadow)
{
  nxsem_destroyholder(&shadow->sem);
  kmm_free(shadow);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

bool nxsem_shadowed(FAR const sem_t *sem)
{
  uintptr_t vaddr = (uintptr_t)sem;

  return up_addrenv_user_vaddr(vaddr)
#ifdef CONFIG_MM_KMAP
         || (vaddr >= CONFIG_ARCH_KMAP_VBASE &&
             vaddr - CONFIG_ARCH_KMAP_VBASE <
             CONFIG_ARCH_KMAP_NPAGES * CONFIG_MM_PGSIZE)
#endif
#ifdef CONFIG_ARCH_PGPOOL_MAPPING
         || (vaddr >= CONFIG_ARCH_PGPOOL_VBASE &&
             vaddr - CONFIG_ARCH_PGPOOL_VBASE < CONFIG_ARCH_PGPOOL_SIZE)
#endif
         ;
}

FAR sem_t *nxsem_resolve(FAR sem_t *sem)
{
  FAR struct sem_shadow_s *shadow;
  FAR struct sem_shadow_s *fresh = NULL;
  irqstate_t flags;
  uintptr_t paddr;

  if (sem == NULL || !nxsem_shadowed(sem) ||
      (paddr = up_addrenv_va_to_pa(sem)) == 0)
    {
      return sem;
    }

  for (; ; )
    {
      flags  = enter_critical_section();
      shadow = shadow_find(paddr);

      if (shadow == NULL && fresh != NULL)
        {
          shadow        = fresh;
          fresh         = NULL;
          shadow->paddr = paddr;
          shadow->sem.val.semcount = sem->val.semcount;
          shadow->sem.flags        = sem->flags;
#ifdef CONFIG_PRIORITY_PROTECT
          shadow->sem.ceiling      = sem->ceiling;
#endif
#ifdef CONFIG_CUSTOM_SEMAPHORE_MAXVALUE
          shadow->sem.maxvalue     = sem->maxvalue;
#endif
          shadow->flink = g_shadows[SHADOW_BUCKET(paddr)];
          g_shadows[SHADOW_BUCKET(paddr)] = shadow;
        }

      if (shadow != NULL)
        {
          shadow->refs++;
        }

      leave_critical_section(flags);

      if (shadow != NULL || up_interrupt_context())
        {
          break;
        }

      fresh = kmm_zalloc(sizeof(*fresh));
      if (fresh == NULL)
        {
          break;
        }
    }

  kmm_free(fresh);
  return shadow != NULL ? &shadow->sem : sem;
}

void nxsem_unresolve(FAR sem_t *usem, FAR sem_t *sem)
{
  FAR struct sem_shadow_s *shadow;
  irqstate_t flags;
  bool last;

  if (usem == sem)
    {
      return;
    }

  shadow = container_of(sem, struct sem_shadow_s, sem);
  flags  = enter_critical_section();

  if (shadow->paddr != 0)
    {
      usem->val.semcount = sem->val.semcount;
      usem->flags        = sem->flags;
#ifdef CONFIG_PRIORITY_PROTECT
      usem->ceiling      = sem->ceiling;
#endif
#ifdef CONFIG_CUSTOM_SEMAPHORE_MAXVALUE
      usem->maxvalue     = sem->maxvalue;
#endif
    }

  last = --shadow->refs == 0 && shadow->paddr == 0;
  leave_critical_section(flags);

  if (last)
    {
      shadow_free(shadow);
    }
}

void nxsem_drop(FAR sem_t *sem)
{
  FAR struct sem_shadow_s *shadow;
  irqstate_t flags;
  uintptr_t paddr;
  bool unused = false;

  if (!nxsem_shadowed(sem) || (paddr = up_addrenv_va_to_pa(sem)) == 0)
    {
      return;
    }

  flags  = enter_critical_section();
  shadow = shadow_find(paddr);

  if (shadow != NULL && dq_empty(&shadow->sem.waitlist))
    {
      unused = shadow_unlink(shadow);
    }

  leave_critical_section(flags);

  if (unused)
    {
      shadow_free(shadow);
    }
}

void nxsem_release_shadows(uintptr_t paddr, size_t size)
{
  FAR struct sem_shadow_s *shadow;
  FAR struct sem_shadow_s *freed = NULL;
  FAR struct sem_shadow_s *next;
  irqstate_t flags;
  uintptr_t page;

  flags = enter_critical_section();

  for (page = paddr; page - paddr < size; page += CONFIG_MM_PGSIZE)
    {
      for (shadow = g_shadows[SHADOW_BUCKET(page)]; shadow; shadow = next)
        {
          next = shadow->flink;

          if (shadow->paddr - page < CONFIG_MM_PGSIZE &&
              shadow_unlink(shadow))
            {
              shadow->flink = freed;
              freed         = shadow;
            }
        }
    }

  leave_critical_section(flags);

  while ((shadow = freed) != NULL)
    {
      freed = shadow->flink;
      shadow_free(shadow);
    }
}

#endif /* CONFIG_BUILD_KERNEL */
