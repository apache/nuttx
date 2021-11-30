/****************************************************************************
 * libs/libc/stdio/lib_fseek.c
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

#include <stdio.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fseek
 *
 * Description:
 *   The fseek() function sets the file position indicator for the stream
 *   pointed to by stream. The new position, measured in bytes, is obtained
 *   by adding offset bytes to the position specified by whence. If whence is
 *   set to SEEK_SET, SEEK_CUR, or SEEK_END, the offset is relative to the
 *   start of the file, the current position indicator, or end-of-file,
 *   respectively. A successful call to the fseek() function clears the
 *   end-of-file indicator for the stream and undoes any effects of the
 *   ungetc(3) function on the same stream.
 *
 * Returned Value:
 *   Zero on success; -1 on failure with errno set appropriately.
 *
 ****************************************************************************/

int fseek(FAR FILE *stream, long int offset, int whence)
{
  return fseeko(stream, offset, whence);
}
