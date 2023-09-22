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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_allocheap
 *
 * Description:
 *   Allocate executable memory for heap.
 *
 ****************************************************************************/

void *host_allocheap(size_t sz)
{
  void *p;

#if defined(CONFIG_HOST_MACOS) && defined(CONFIG_HOST_ARM64)
  /* see: https://developer.apple.com/forums/thread/672804 */

  p = host_uninterruptible(mmap, NULL, sz, PROT_READ | PROT_WRITE,
                           MAP_ANON | MAP_SHARED, -1, 0);
#else
  p = host_uninterruptible(mmap, NULL, sz,
                           PROT_READ | PROT_WRITE | PROT_EXEC,
                           MAP_ANON | MAP_PRIVATE, -1, 0);
#endif

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
  host_uninterruptible(munmap, mem, 0);
}

void *host_allocshmem(const char *name, size_t size, int master)
{
  void *mem;
  int oflag;
  int ret;
  int fd;

  oflag = O_RDWR;
  if (master)
    {
      oflag |= O_CREAT | O_TRUNC;
    }

  fd = host_uninterruptible(shm_open, name, oflag, S_IRUSR | S_IWUSR);
  if (fd < 0)
    {
      return NULL;
    }

  if (!master)
    {
      /* Avoid the second slave instance open successfully */

      host_unlinkshmem(name);
    }

  ret = host_uninterruptible(ftruncate, fd, size);
  if (ret < 0)
    {
      host_uninterruptible(close, fd);
      return NULL;
    }

  mem = host_uninterruptible(mmap, NULL, size,
                             PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  host_uninterruptible(close, fd); /* Don't need keep fd any more once the memory get mapped */
  if (mem == MAP_FAILED)
    {
      return NULL;
    }

  return mem;
}

void host_freeshmem(void *mem)
{
  host_uninterruptible(munmap, mem, 0);
}

int host_unlinkshmem(const char *name)
{
  return host_uninterruptible(shm_unlink, name);
}

size_t host_mallocsize(void *mem)
{
#ifdef __APPLE__
  return host_uninterruptible(malloc_size, mem);
#else
  return host_uninterruptible(malloc_usable_size, mem);
#endif
}

void *host_memalign(size_t alignment, size_t size)
{
  void *p;
  int error;

  if (alignment < sizeof(void *))
    {
      alignment = sizeof(void *);
    }

  error = host_uninterruptible(posix_memalign, &p, alignment, size);
  if (error != 0)
    {
      return NULL;
    }

  size = host_mallocsize(p);
  atomic_fetch_add(&g_aordblks, 1);
  atomic_fetch_add(&g_uordblks, size);

  return p;
}

void host_free(void *mem)
{
  size_t size;

  if (mem == NULL)
    {
      return;
    }

  size = host_mallocsize(mem);
  atomic_fetch_sub(&g_aordblks, 1);
  atomic_fetch_sub(&g_uordblks, size);
  host_uninterruptible_no_return(free, mem);
}

void *host_realloc(void *oldmem, size_t size)
{
  size_t oldsize;
  void *mem;

  if (oldmem == NULL)
    {
      return host_memalign(sizeof(void *), size);
    }

  oldsize = host_mallocsize(oldmem);
  mem = host_uninterruptible(realloc, oldmem, size);
  if (mem == NULL)
    {
      return NULL;
    }

  size = host_mallocsize(mem);
  atomic_fetch_add(&g_uordblks, size - oldsize);

  return mem;
}

void host_mallinfo(int *aordblks, int *uordblks)
{
  *aordblks = atomic_load(&g_aordblks);
  *uordblks = atomic_load(&g_uordblks);
}
