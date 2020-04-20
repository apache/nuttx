/****************************************************************************
 * libs/libc/modlib/modlib_iobuffer.c
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

#include <debug.h>
#include <errno.h>

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

      loadinfo->iobuffer = (FAR uint8_t *)
                           lib_malloc(CONFIG_MODLIB_BUFFERSIZE);
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

int modlib_reallocbuffer(FAR struct mod_loadinfo_s *loadinfo,
                         size_t increment)
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
