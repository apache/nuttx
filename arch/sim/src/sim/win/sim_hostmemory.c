/****************************************************************************
 * arch/sim/src/sim/win/sim_hostmemory.c
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

#include <windows.h>

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
  return _aligned_malloc(sz, 8);
}

void *host_allocshmem(const char *name, size_t size, int master)
{
  HANDLE handle;
  void *mem;

  handle = CreateFileMapping(INVALID_HANDLE_VALUE, NULL,
                             PAGE_READWRITE, 0, 0, name);
  if (handle == NULL)
    {
      return NULL;
    }

  mem = MapViewOfFile(handle, FILE_MAP_ALL_ACCESS, 0, 0, size);
  CloseHandle(handle);

  return mem;
}

void host_freeshmem(void *mem)
{
  UnmapViewOfFile(mem);
}

size_t host_mallocsize(void *mem)
{
  return _msize(mem);
}

void *host_memalign(size_t alignment, size_t size)
{
  return _aligned_malloc(size, alignment);
}

void host_free(void *mem)
{
  _aligned_free(mem);
}

void *host_realloc(void *oldmem, size_t size)
{
  return _aligned_realloc(oldmem, size, 8);
}

void host_mallinfo(int *aordblks, int *uordblks)
{
}
