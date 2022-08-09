/****************************************************************************
 * libs/libc/audio/lib_buffer.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/audio/audio.h>
#include <nuttx/usb/audio.h>

#include "libc.h"

#if defined(CONFIG_AUDIO)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: apb_semtake
 *
 *       Take an Audio Pipeline Buffer.
 *
 ****************************************************************************/

static void apb_semtake(FAR struct ap_buffer_s *apb)
{
  int ret;

  /* Take the semaphore (perhaps waiting) */

  while (_SEM_WAIT(&apb->sem) < 0)
    {
      /* The only case that an error should occr here is if
       * the wait was awakened by a signal.
       */

      DEBUGASSERT(_SEM_ERRNO(ret) == EINTR || _SEM_ERRNO(ret) == ECANCELED);
      UNUSED(ret);
    }
}

/****************************************************************************
 * Name: apb_semgive
 ****************************************************************************/

#define apb_semgive(b) _SEM_POST(&b->sem)

/****************************************************************************
 * Name: apb_alloc
 *
 * Allocate an Audio Pipeline Buffer for use in the Audio sub-system.  This
 * will perform the actual allocate based on buffer data format, number of
 * channels, etc. and prepare the buffer for consumption.
 *
 ****************************************************************************/

int apb_alloc(FAR struct audio_buf_desc_s *bufdesc)
{
  uint32_t            bufsize;
  int                 ret;
  struct ap_buffer_s  *apb;

  DEBUGASSERT(bufdesc->u.pbuffer != NULL);

  /* Perform a user mode allocation */

  bufsize = sizeof(struct ap_buffer_s) + bufdesc->numbytes;
  apb = lib_umalloc(bufsize);
  *bufdesc->u.pbuffer = apb;

  /* Test if the allocation was successful or not */

  if (*bufdesc->u.pbuffer == NULL)
    {
      ret = -ENOMEM;
    }
  else
    {
      /* Populate the buffer contents */

      memset(apb, 0, bufsize);
      apb->i.channels = 1;
      apb->crefs      = 1;
      apb->nmaxbytes  = bufdesc->numbytes;
      apb->nbytes     = 0;
      apb->flags      = 0;
      apb->samp       = (FAR uint8_t *)(apb + 1);
#ifdef CONFIG_AUDIO_MULTI_SESSION
      apb->session    = bufdesc->session;
#endif

      _SEM_INIT(&apb->sem, 0, 1);
      ret = sizeof(struct audio_buf_desc_s);
    }

  return ret;
}

/****************************************************************************
 * Name: apb_free
 *
 * Free's a previously allocated or referenced Audio Pipeline Buffer
 *
 ****************************************************************************/

void apb_free(FAR struct ap_buffer_s *apb)
{
  int refcount;

  /* Perform a reference count decrement and possibly release the memory */

  apb_semtake(apb);
  refcount = apb->crefs--;
  apb_semgive(apb);

  if (refcount <= 1)
    {
      audinfo("Freeing %p\n", apb);
      _SEM_DESTROY(&apb->sem);
      lib_ufree(apb);
    }
}

/****************************************************************************
 * Name: apb_reference
 *
 * Claim a reference to an Audio Pipeline Buffer.  Each call to apb_reference
 * will increment the reference count and must have a matching apb_free
 * call.  When the refcount decrements to zero, the buffer will be freed.
 *
 ****************************************************************************/

void apb_reference(FAR struct ap_buffer_s *apb)
{
  /* Do we need any thread protection here?  Almost certainly... */

  apb_semtake(apb);
  apb->crefs++;
  apb_semgive(apb);
}

#endif /* CONFIG_AUDIO */
