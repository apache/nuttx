/****************************************************************************
 * libs/libc/stdio/lib_setbuffer.c
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
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setbuffer()
 *
 * Description:
 *  The setbuffer() function is a wrapper around setvbuf() which
 *  enables full buffering on a buffer allocated by the caller, assuming
 *  buffer is not a null pointer.
 *
 * Input Parameters:
 *   stream - the stream to flush
 *   buffer - The user allocated buffer to use.
 *            If NULL, buffering is disabled.
 *   size   - size of the user buffer.
 *
 ****************************************************************************/

void setbuffer(FAR FILE *stream, FAR char *buffer, size_t size)
{
#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  setvbuf(stream, buffer, buffer ? _IOFBF : _IONBF, size);
#endif
}
