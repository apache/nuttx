/****************************************************************************
 * libs/libc/unistd/lib_pipe.c
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

#include <errno.h>
#include <unistd.h>

#include <nuttx/drivers/drivers.h>

#if defined(CONFIG_PIPES) && CONFIG_DEV_PIPE_SIZE > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pipe
 *
 * Description:
 *   pipe() creates a pair of file descriptors, pointing to a pipe inode,
 *   and  places them in the array pointed to by 'fd'. fd[0] is for reading,
 *   fd[1] is for writing.
 *
 * Input Parameters:
 *   fd[2] - The user provided array in which to catch the pipe file
 *   descriptors
 *
 * Returned Value:
 *   0 is returned on success; otherwise, -1 is returned with errno set
 *   appropriately.
 *
 ****************************************************************************/

int pipe(int fd[2])
{
  int ret;

  ret = nx_pipe(fd, CONFIG_DEV_PIPE_SIZE, 0);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

#endif /* CONFIG_PIPES && CONFIG_DEV_PIPE_SIZE > 0 */
