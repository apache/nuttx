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

#include <sys/mman.h>
#include <sys/stat.h>

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
