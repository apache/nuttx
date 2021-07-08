/****************************************************************************
 * libs/libc/termios/lib_ttyname.c
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

#include <errno.h>
#include <limits.h>
#include <unistd.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ttyname
 *
 * Description:
 *   The ttyname() function shall return a pointer to a string containing
 *   a null-terminated pathname of the terminal associated with file
 *   descriptor fildes. The application shall not modify the string returned.
 *   The returned pointer might be invalidated or the string content might
 *   be overwritten by a subsequent call to ttyname(). The returned pointer
 *   and the string content might also be invalidated if the calling thread
 *   is terminated.
 *
 * Input Parameters:
 *   fd - The 'fd' argument is an open file descriptor associated with
 *        a terminal.
 *
 * Returned Value:
 *   Upon successful completion, ttyname() shall return a pointer to
 *   a string. Otherwise, a null pointer shall be returned and errno
 *   set to indicate the error.
 *
 ****************************************************************************/

FAR char *ttyname(int fd)
{
  static char name[TTY_NAME_MAX];
  int ret;

  ret = ttyname_r(fd, name, TTY_NAME_MAX);
  if (ret != 0)
    {
      set_errno(ret);
      return NULL;
    }

  return name;
}
