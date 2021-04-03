/****************************************************************************
 * libs/libc/misc/lib_streamsem.c
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

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void stream_semtake(FAR struct streamlist *list)
{
  int ret;

  /* Take the semaphore (perhaps waiting) */

  while ((ret = _SEM_WAIT(&list->sl_sem)) < 0)
    {
      /* The only case that an error should occr here is if
       * the wait was awakened by a signal.
       */

      DEBUGASSERT(_SEM_ERRNO(ret) == EINTR || _SEM_ERRNO(ret) == ECANCELED);
      UNUSED(ret);
    }
}

void stream_semgive(FAR struct streamlist *list)
{
  _SEM_POST(&list->sl_sem);
}
