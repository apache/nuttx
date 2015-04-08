/****************************************************************************
 * libc/audio/lib_buffer.c
 *
 *   Copyright (C) 2013 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/audio/audio.h>
#include <nuttx/usb/audio.h>

#include "lib_internal.h"

#if defined(CONFIG_AUDIO)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

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
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&apb->sem) != 0)
    {
      /* The only case that an error should occr here is if
       * the wait was awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: apb_semgive
 ****************************************************************************/

#define apb_semgive(b) sem_post(&b->sem)

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

  DEBUGASSERT(bufdesc->u.ppBuffer != NULL);

  /* Perform a user mode allocation */

  bufsize = sizeof(struct ap_buffer_s) + bufdesc->numbytes;
  apb = lib_umalloc(bufsize);
  *bufdesc->u.ppBuffer = apb;

  /* Test if the allocation was successful or not */

  if (*bufdesc->u.ppBuffer == NULL)
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
#ifdef CONFIG_AUDIO_MULTI_SESSION
      apb->session    = bufdesc->session;
#endif

      sem_init(&apb->sem, 0, 1);
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
      audvdbg("Freeing %p\n", apb);
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
  /* Do we need any thread protection here?  Almost certaily... */

  apb_semtake(apb);
  apb->crefs++;
  apb_semgive(apb);
}

#endif /* CONFIG_AUDIO */
