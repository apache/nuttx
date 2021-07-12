/****************************************************************************
 * arch/sim/src/sim/up_hostmemory.c
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

#ifdef __APPLE__
#include <malloc/malloc.h>
#else
#include <malloc.h>
#endif

#include "up_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static atomic_int g_aordblks;
static atomic_int g_uordblks;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_alloc_heap
 *
 * Description:
 *   Allocate executable memory for heap.
 *
 ****************************************************************************/

void *host_alloc_heap(size_t sz)
{
  void *p;

  p = mmap(NULL, sz, PROT_READ | PROT_WRITE | PROT_EXEC,
           MAP_ANON | MAP_PRIVATE, -1, 0);
  if (p == MAP_FAILED)
    {
      return NULL;
    }

  return p;
}

void *host_alloc_shmem(const char *name, size_t size, int master)
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

  while (1)
    {
      fd = shm_open(name, oflag, S_IRUSR | S_IWUSR);
      if (fd >= 0)
        {
          if (!master)
            {
              /* Avoid the second slave instance open successfully */

              shm_unlink(name);
            }
          break;
        }

      if (master || errno != ENOENT)
        {
          return NULL;
        }

      /* Master isn't ready, sleep and try again */

      usleep(1000);
    }

  ret = ftruncate(fd, size);
  if (ret < 0)
    {
      close(fd);
      return NULL;
    }

  mem = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd); /* Don't need keep fd any more once the memory get mapped */
  if (mem == MAP_FAILED)
    {
      return NULL;
    }

  return mem;
}

void host_free_shmem(void *mem)
{
  munmap(mem, 0);
}

size_t host_malloc_size(void *mem)
{
#ifdef __APPLE__
  return malloc_size(mem);
#else
  return malloc_usable_size(mem);
#endif
}

void *host_memalign(size_t alignment, size_t size)
{
  void *p;
  int error;

  error = posix_memalign(&p, alignment, size);
  if (error != 0)
    {
      return NULL;
    }

  size = host_malloc_size(p);
  g_aordblks += 1;
  g_uordblks += size;

  return p;
}

void host_free(void *mem)
{
  size_t size;

  if (mem == NULL)
    {
      return;
    }

  size = host_malloc_size(mem);
  g_aordblks -= 1;
  g_uordblks -= size;
  free(mem);
}

void *host_realloc(void *oldmem, size_t size)
{
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

  oldsize = host_malloc_size(oldmem);
  mem = realloc(oldmem, size);
  if (mem == NULL)
    {
      return NULL;
    }

  size = host_malloc_size(mem);
  g_uordblks -= oldsize;
  g_uordblks += size;

  return mem;
}

void host_mallinfo(int *aordblks, int *uordblks)
{
  *aordblks = g_aordblks;
  *uordblks = g_uordblks;
}
