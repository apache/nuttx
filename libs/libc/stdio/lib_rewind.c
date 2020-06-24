/****************************************************************************
 * libs/libc/stdio/lib_rewind.c
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

#include <stdio.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rewind
 ****************************************************************************/

void rewind(FAR FILE *stream)
{
  /* Verify that we were provided with a stream */

  if (!stream)
    {
      set_errno(EBADF);
      return;
    }

  lib_take_semaphore(stream);
  (void) fseek(stream, 0L, SEEK_SET);
  stream->fs_flags &= ~__FS_FLAG_ERROR;
  lib_give_semaphore(stream);
}
