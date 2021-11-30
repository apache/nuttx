/****************************************************************************
 * libs/libc/stdio/lib_fsetpos.c
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
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fsetpos
 *
 * Description:
 *   fsetpos() function is an alternate interfaces equivalent to fseek()
 *   (with whence set to  SEEK_SET).  It sets the current value of the file
 *   offset to value in the location referenced by pos.  On some non-UNIX
 *   systems an fpos_t object may be a complex object and fsetpos may be the
 *   only way to portably reposition a stream.
 *
 * Returned Value:
 *   Zero on success; -1 on failure with errno set appropriately.
 *
 ****************************************************************************/

int fsetpos(FAR FILE *stream, FAR fpos_t *pos)
{
#ifdef CONFIG_DEBUG_FEATURES
  if (!stream || !pos)
    {
      set_errno(EINVAL);
      return ERROR;
    }
#endif

  return fseeko(stream, *pos, SEEK_SET);
}
