/****************************************************************************
 * arch/sim/src/sim/posix/sim_hostmemory.c
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
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdatomic.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/queue.h>

#ifdef __APPLE__
#include <malloc/malloc.h>
#else
#include <malloc.h>
#endif

#include "sim_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static atomic_int g_aordblks;
static atomic_int g_uordblks;

/* Record memory allocated for text sections by sys/queue.h */

struct textheap_s
{
  void *p;
  size_t size;
  TAILQ_ENTRY(textheap_s) entry;
};

static TAILQ_HEAD(, textheap_s) g_textheap_list =
  TAILQ_HEAD_INITIALIZER(g_textheap_list);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern uint64_t up_irq_save(void);
extern void up_irq_restore(uint64_t flags);

/****************************************************************************
 * Name: up_textheap_memalign
 *
 * Description:
 *   Allocate memory for text sections with the specified alignment.
 *
 ****************************************************************************/

void *up_textheap_memalign(size_t align, size_t size)
{
  uint64_t flags;
  void *p;

  /* host_allocheap (mmap) returns memory aligned to the page size, which
   * is always a multiple of the alignment (4/8) for text section. So, we
   * don't need to do anything here.
   */

  p = host_allocheap(size);

  flags = up_irq_save();

  /* Record the allocated memory to a global list */

  if (p)
    {
      struct textheap_s *node = malloc(sizeof(struct textheap_s));
      if (node)
        {
          node->p = p;
          node->size = size;
          TAILQ_INSERT_TAIL(&g_textheap_list, node, entry);
        }
    }

  up_irq_restore(flags);

  return p;
}

/****************************************************************************
 * Name: up_textheap_free
 *
 * Description:
 *   Free memory allocated for text sections.
 *
 ****************************************************************************/

void up_textheap_free(void *p)
{
  struct textheap_s *node;
  uint64_t flags = up_irq_save();

  /* Remove the memory from the global list */

  TAILQ_FOREACH(node, &g_textheap_list, entry)
    {
      if (node->p == p)
        {
          TAILQ_REMOVE(&g_textheap_list, node, entry);
          free(node);
          break;
        }
    }

  up_irq_restore(flags);
  host_freeheap(p);
}

/****************************************************************************
 * Name: up_textheap_heapmember
 *
 * Description:
 *   Test if memory is from text heap.
 *
 ****************************************************************************/

bool up_textheap_heapmember(void *p)
{
  struct textheap_s *node;
  uint64_t flags = up_irq_save();

  /* Traverse the global list to find the memory */

  TAILQ_FOREACH(node, &g_textheap_list, entry)
    {
      if (node->p == p)
        {
          up_irq_restore(flags);
          return true;
        }
    }

  up_irq_restore(flags);

  return false;
}

/****************************************************************************
 * Name: host_allocheap
 *
 * Description:
 *   Allocate executable memory for heap.
 *
 ****************************************************************************/

void *host_allocheap(size_t sz)
{
  uint64_t flags = up_irq_save();
  void *p;

#if defined(CONFIG_HOST_MACOS) && defined(CONFIG_HOST_ARM64)
  /* see: https://developer.apple.com/forums/thread/672804 */

  p = mmap(NULL, sz, PROT_READ | PROT_WRITE,
           MAP_ANON | MAP_SHARED, -1, 0);
#else
  p = mmap(NULL, sz, PROT_READ | PROT_WRITE | PROT_EXEC,
           MAP_ANON | MAP_PRIVATE, -1, 0);
#endif

  up_irq_restore(flags);

  if (p == MAP_FAILED)
    {
      return NULL;
    }

  return p;
}

/****************************************************************************
 * Name: host_freeheap
 *
 * Description:
 *   Free a executable memory block.
 *
 ****************************************************************************/

void host_freeheap(void *mem)
{
  uint64_t flags = up_irq_save();
  munmap(mem, 0);
  up_irq_restore(flags);
}

void *host_allocshmem(const char *name, size_t size, int master)
{
  uint64_t flags = up_irq_save();
  void *mem;
  int oflag;
  int ret;
  int fd;

  oflag = O_RDWR;
  if (master)
    {
      oflag |= O_CREAT | O_TRUNC;
    }

  fd = shm_open(name, oflag, S_IRUSR | S_IWUSR);
  if (fd < 0)
    {
      up_irq_restore(flags);
      return NULL;
    }

  if (!master)
    {
      /* Avoid the second slave instance open successfully */

      shm_unlink(name);
    }

  ret = ftruncate(fd, size);
  if (ret < 0)
    {
      up_irq_restore(flags);
      close(fd);
      return NULL;
    }

  mem = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd); /* Don't need keep fd any more once the memory get mapped */
  up_irq_restore(flags);
  if (mem == MAP_FAILED)
    {
      return NULL;
    }

  return mem;
}

void host_freeshmem(void *mem)
{
  uint64_t flags = up_irq_save();
  munmap(mem, 0);
  up_irq_restore(flags);
}

size_t host_mallocsize(void *mem)
{
#ifdef __APPLE__
  return malloc_size(mem);
#else
  return malloc_usable_size(mem);
#endif
}

void *host_memalign(size_t alignment, size_t size)
{
  uint64_t flags = up_irq_save();
  void *p;
  int error;

  if (alignment < sizeof(void *))
    {
      alignment = sizeof(void *);
    }

  error = posix_memalign(&p, alignment, size);
  if (error != 0)
    {
      up_irq_restore(flags);
      return NULL;
    }

  size = host_mallocsize(p);
  g_aordblks += 1;
  g_uordblks += size;

  up_irq_restore(flags);

  return p;
}

void host_free(void *mem)
{
  size_t size;
  uint64_t flags;

  if (mem == NULL)
    {
      return;
    }

  flags = up_irq_save();
  size = host_mallocsize(mem);
  g_aordblks -= 1;
  g_uordblks -= size;
  free(mem);
  up_irq_restore(flags);
}

void *host_realloc(void *oldmem, size_t size)
{
  uint64_t flags;
  size_t oldsize;
  void *mem;

  if (size == 0)
    {
      host_free(oldmem);
      return NULL;
    }
  else if (oldmem == NULL)
    {
      return host_memalign(sizeof(void *), size);
    }

  flags = up_irq_save();

  oldsize = host_mallocsize(oldmem);
  mem = realloc(oldmem, size);
  if (mem == NULL)
    {
      up_irq_restore(flags);
      return NULL;
    }

  size = host_mallocsize(mem);
  g_uordblks -= oldsize;
  g_uordblks += size;

  up_irq_restore(flags);

  return mem;
}

void host_mallinfo(int *aordblks, int *uordblks)
{
  *aordblks = g_aordblks;
  *uordblks = g_uordblks;
}
