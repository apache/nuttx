/****************************************************************************
 * libs/libc/stdio/lib_feof.c
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
#include <errno.h>

#include <nuttx/fs/fs.h>

#ifdef CONFIG_FILE_STREAM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: feof
 *
 * Description:
 *   The feof() function shall test if the currently file pointer for the
 *   stream is at the end of file.
 *
 * Returned Value:
 *  This function will return non-zero if the file pointer is positioned
 *  at the end of file.
 *
 ****************************************************************************/

int feof(FILE *stream)
{
  /* If the end-of-file condition is encountered by any of the C-buffered
   * I/O functions that perform read operations, they should set the
   * __FS_FLAG_EOF in the fs_flags field of struct file_struct.
   */

  return (stream->fs_flags & __FS_FLAG_EOF) != 0;
}

#endif /* CONFIG_FILE_STREAM */
