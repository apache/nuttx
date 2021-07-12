/****************************************************************************
 * libs/libc/termios/lib_ttynamer.c
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
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ttyname_r
 *
 * Description:
 *   The ttyname_r() function shall store the null-terminated pathname of
 *   the terminal associated with the file descriptor fildes in the
 *   character array referenced by name. The array is namesize characters
 *   long and should have space for the name and the terminating null
 *   character. The maximum length of the terminal name shall be
 *   {TTY_NAME_MAX}.
 *
 * Input Parameters:
 *   fd - The 'fd' argument is an open file descriptor associated with
 *        a terminal.
 *   buf - Caller provided buffer to hold tty name.
 *   buflen - The size of the caller-provided buffer.
 *
 * Returned Value:
 *   If successful, the ttyname_r() function shall return zero.
 *   Otherwise, an error number shall be returned to indicate the error.
 *
 ****************************************************************************/

int ttyname_r(int fd, FAR char *buf, size_t buflen)
{
  if (!isatty(fd))
    {
      return ENOTTY;
    }

  if (buflen >= TTY_NAME_MAX)
    {
      return fcntl(fd, F_GETPATH, buf) < 0 ? get_errno() : 0;
    }
  else
    {
      char name[TTY_NAME_MAX];

      if (fcntl(fd, F_GETPATH, name) < 0)
        {
          return get_errno();
        }

      if (strlen(name) >= buflen)
        {
          return ERANGE;
        }

      strcpy(buf, name);
      return OK;
    }
}
