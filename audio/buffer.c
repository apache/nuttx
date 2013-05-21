/****************************************************************************
 * audio/buffer.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/audio/audio.h>
#include <nuttx/usb/audio.h>

#if defined(CONFIG_AUDIO)

/****************************************************************************
 * Preprocessor Definitions
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

static void apb_semtake(sem_t *sem)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(sem) != 0)
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

#define apb_semgive(s) sem_post(s)

/****************************************************************************
 * Name: apb_alloc
 *
 * Allocate an Audio Pipeline Buffer for use in the Audio sub-system.  This
 * will perform the actual allocate based on buffer data format, number of
 * channels, etc. and prepare the buffer for consumption.
 *
 ****************************************************************************/

FAR struct ap_buffer_s *apb_alloc(int type, int sampleCount)
{
  /* TODO:  Implement the alloc logic */

  return NULL;
}

/****************************************************************************
 * Name: apb_prepare
 *
 * Prepare an AP Buffer for use in the Audio Pipeline.
 *
 ****************************************************************************/

void apb_prepare(FAR struct ap_buffer_s *apb, int8_t allocmode, uint8_t format,
    uint8_t subformat, apb_samp_t maxsamples)
{
  /* Perform a reference count decrement and possibly release the memory */

}

/****************************************************************************
 * Name: apb_free
 *
 * Free's a previously allocated or referenced Audio Pipeline Buffer
 *
 ****************************************************************************/

void apb_free(FAR struct ap_buffer_s *apb)
{
  /* Perform a reference count decrement and possibly release the memory */

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
  /* TODO:  Implement the reference logic */
}

#endif /* CONFIG_AUDIO */

