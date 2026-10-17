/****************************************************************************
 * libs/libc/stdio/lib_popen.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <sys/wait.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "libc.h"

/* The kernel side of libc does not run commands */

#ifndef __KERNEL__

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The cookie of a stream from popen() */

struct popen_s
{
  int fd;       /* The pipe to the command */
  pid_t pid;    /* The process ID of the shell */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static ssize_t popen_read(FAR void *cookie, FAR char *buf, size_t size)
{
  FAR struct popen_s *pp = cookie;

  return read(pp->fd, buf, size);
}

static ssize_t popen_write(FAR void *cookie, FAR const char *buf,
                           size_t size)
{
  FAR struct popen_s *pp = cookie;

  return write(pp->fd, buf, size);
}

static off_t popen_seek(FAR void *cookie, FAR off_t *offset, int whence)
{
  set_errno(ESPIPE);
  return -1;
}

static int popen_close(FAR void *cookie)
{
  FAR struct popen_s *pp = cookie;
  int ret;

  ret = close(pp->fd);
  lib_free(pp);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: popen
 *
 * Description:
 *   Run the command "sh -c command" in a new process, with a pipe between
 *   the caller and the command.  With mode "r", the caller reads the
 *   standard output of the command from the returned stream.  With mode
 *   "w", the caller writes the standard input of the command to the
 *   returned stream.  The command does not get the streams of earlier
 *   popen() calls.
 *
 * Input Parameters:
 *   command - The command string for the shell.
 *   mode    - "r" or "w".  An "e" is accepted and ignored: the descriptor
 *             is always close-on-exec.
 *
 * Returned Value:
 *   A stream on success.  On failure, NULL is returned and errno is set.
 *
 ****************************************************************************/

FAR FILE *popen(FAR const char *command, FAR const char *mode)
{
  cookie_io_functions_t io =
    {
      popen_read, popen_write, popen_seek, popen_close
    };

  FAR struct popen_s *pp;
  FAR FILE *stream;
  int oflag;
  int errcode;

  if (mode == NULL || (mode[0] != 'r' && mode[0] != 'w') ||
      strchr(mode, '+') != NULL)
    {
      set_errno(EINVAL);
      return NULL;
    }

  oflag = mode[0] == 'r' ? O_RDONLY : O_WRONLY;

  pp = lib_malloc(sizeof(*pp));
  if (pp == NULL)
    {
      set_errno(ENOMEM);
      return NULL;
    }

  pp->fd = dpopen(command, oflag | O_CLOEXEC, &pp->pid);
  if (pp->fd < 0)
    {
      lib_free(pp);
      return NULL;
    }

  stream = fopencookie(pp, mode[0] == 'r' ? "r" : "w", io);
  if (stream == NULL)
    {
      errcode = get_errno();
      dpclose(pp->fd, pp->pid);
      lib_free(pp);
      set_errno(errcode);
    }

  return stream;
}

/****************************************************************************
 * Name: pclose
 *
 * Description:
 *   Close a stream from popen() and wait for the command to terminate.
 *
 * Input Parameters:
 *   stream - The stream from popen().
 *
 * Returned Value:
 *   The termination status of the shell, as from waitpid().  On failure,
 *   -1 is returned and errno is set.
 *
 ****************************************************************************/

int pclose(FAR FILE *stream)
{
  FAR struct popen_s *pp;
  pid_t pid;
  int status;
  int ret;

  if (stream == NULL || stream->fs_iofunc.close != popen_close)
    {
      set_errno(EINVAL);
      return -1;
    }

  /* fclose() frees the cookie, so keep the process ID */

  pp  = stream->fs_cookie;
  pid = pp->pid;
  fclose(stream);

  do
    {
      ret = waitpid(pid, &status, 0);
    }
  while (ret < 0 && get_errno() == EINTR);

  return ret < 0 ? -1 : status;
}

#endif /* __KERNEL__ */
