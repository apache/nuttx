/****************************************************************************
 * libs/libc/stdio/lib_setbuf.c
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
#include <assert.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setbuf
 *
 * Description:
 *   Except that it returns no value, the function call setbuf(stream, buf)
 *   is equivalent to:
 *
 *     setvbuf(stream, buf, _IOFBF, BUFSIZ)
 *
 *   if buf is not a null pointer, or to:
 *
 *     setvbuf(stream, buf, _IONBF, BUFSIZ)
 *
 *   if buf is a null pointer.
 *
 * Input Parameters:
 *   stream - The stream whose buffer will be modified
 *   buffer - If non-NULL, this is user allocated buffer of size BUFSIZ. If
 *            NULL, setvbuf will disable buffering
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void setbuf(FAR FILE *stream, FAR char *buf)
{
#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  int mode;

  DEBUGASSERT(stream != NULL);

  mode = (buf != NULL) ? _IOFBF : _IONBF;
  setvbuf(stream, buf, mode, BUFSIZ);

#else
  DEBUGASSERT(stream != NULL);
#endif
}
