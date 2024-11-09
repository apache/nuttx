/****************************************************************************
 * libs/libc/misc/lib_pathbuffer.c
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
#include <nuttx/mutex.h>
#include <nuttx/lib/lib.h>

#include <stdlib.h>
#include <string.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pathbuffer_s
{
  mutex_t lock;              /* Lock for the buffer */
  unsigned long free_bitmap; /* Bitmap of free buffer */
  char buffer[CONFIG_LIBC_MAX_PATHBUFFER][PATH_MAX];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct pathbuffer_s g_pathbuffer =
{
  NXMUTEX_INITIALIZER,
  (1u << CONFIG_LIBC_MAX_PATHBUFFER) - 1,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_get_pathbuffer
 *
 * Description:
 *   The lib_get_pathbuffer() function returns a pointer to a temporary
 *   buffer.  The buffer is allocated from a pool of pre-allocated buffers
 *   and if the pool is exhausted, a new buffer is allocated through
 *   kmm_malloc(). The size of the buffer is PATH_MAX, and must freed by
 *   calling lib_put_pathbuffer().
 *
 * Returned Value:
 *   On success, lib_get_pathbuffer() returns a pointer to a temporary
 *   buffer.  On failure, NULL is returned.
 *
 ****************************************************************************/

FAR char *lib_get_pathbuffer(void)
{
  int index;

  /* Try to find a free buffer */

  nxmutex_lock(&g_pathbuffer.lock);
  index = ffsl(g_pathbuffer.free_bitmap) - 1;
  if (index >= 0 && index < CONFIG_LIBC_MAX_PATHBUFFER)
    {
      g_pathbuffer.free_bitmap &= ~(1u << index);
      nxmutex_unlock(&g_pathbuffer.lock);
      return g_pathbuffer.buffer[index];
    }

  nxmutex_unlock(&g_pathbuffer.lock);

  /* If no free buffer is found, allocate a new one if
   * CONFIG_LIBC_PATHBUFFER_MALLOC is enabled
   */

#ifdef CONFIG_LIBC_PATHBUFFER_MALLOC
  return lib_malloc(PATH_MAX);
#else
  return NULL;
#endif
}

/****************************************************************************
 * Name: lib_put_pathbuffer
 *
 * Description:
 *   The lib_put_pathbuffer() function frees a temporary buffer that was
 *   allocated by lib_get_pathbuffer(). If the buffer was allocated
 *   dynamically, it is freed by calling kmm_free(). Otherwise, the buffer
 *   is marked as free in the pool of pre-allocated buffers.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lib_put_pathbuffer(FAR char *buffer)
{
  int index;

  nxmutex_lock(&g_pathbuffer.lock);
  index = (buffer - &g_pathbuffer.buffer[0][0]) / PATH_MAX;

  if (index >= 0 && index < CONFIG_LIBC_MAX_PATHBUFFER)
    {
      /* Mark the corresponding bit as free */

      g_pathbuffer.free_bitmap |= 1u << index;
      nxmutex_unlock(&g_pathbuffer.lock);
      return;
    }

  nxmutex_unlock(&g_pathbuffer.lock);

  /* Free the buffer if it was dynamically allocated */

#ifdef CONFIG_LIBC_PATHBUFFER_MALLOC
  lib_free(buffer);
#endif
}
