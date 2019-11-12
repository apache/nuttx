/****************************************************************************
 * arch/sim/src/sim/up_shmem.c
 *
 *   Copyright (C) 2019 Xiaomi Inc. All rights reserved.
 *   Author: Chao An <anchao@pinecone.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#define _GNU_SOURCE 1

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <sys/mman.h>
#include <sys/stat.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void *shmem_open(const char *name, size_t size, int master)
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

void shmem_close(void *mem)
{
  munmap(mem, 0);
}
