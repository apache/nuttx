/****************************************************************************
 * libs/libc/modlib/modlib_iobuffer.c
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <debug.h>
#include <errno.h>

#include <nuttx/module.h>
#include <nuttx/lib/modlib.h>

#include "libc.h"
#include "modlib/modlib.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modlib_allocbuffer
 *
 * Description:
 *   Perform the initial allocation of the I/O buffer, if it has not already
 *   been allocated.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_allocbuffer(FAR struct mod_loadinfo_s *loadinfo)
{
  /* Has a buffer been allocated> */

  if (!loadinfo->iobuffer)
    {
      /* No.. allocate one now */

      loadinfo->iobuffer = (FAR uint8_t *)lib_malloc(CONFIG_MODLIB_BUFFERSIZE);
      if (!loadinfo->iobuffer)
        {
          berr("ERROR: Failed to allocate an I/O buffer\n");
          return -ENOMEM;
        }

      loadinfo->buflen = CONFIG_MODLIB_BUFFERSIZE;
    }

  return OK;
}

/****************************************************************************
 * Name: modlib_reallocbuffer
 *
 * Description:
 *   Increase the size of I/O buffer by the specified buffer increment.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int modlib_reallocbuffer(FAR struct mod_loadinfo_s *loadinfo, size_t increment)
{
  FAR void *buffer;
  size_t newsize;

  /* Get the new size of the allocation */

  newsize = loadinfo->buflen + increment;

  /* And perform the reallocation */

   buffer = lib_realloc((FAR void *)loadinfo->iobuffer, newsize);
   if (!buffer)
    {
      berr("ERROR: Failed to reallocate the I/O buffer\n");
      return -ENOMEM;
    }

  /* Save the new buffer info */

  loadinfo->iobuffer = buffer;
  loadinfo->buflen   = newsize;
  return OK;
}
