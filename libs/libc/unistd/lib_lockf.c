/****************************************************************************
 * libs/libc/unistd/lib_lockf.c
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

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lockf
 *
 * Description:
 *   lockf() is a function that allows a process to apply or remove an
 *   advisory lock on an open file. The lock can be either a shared (read)
 *   lock or an exclusive (write) lock. This lock is advisory, meaning that
 *   it is not enforced by the system and it is up to cooperating processes
 *   to honor the lock.
 *
 * Input Parameters:
 *   fd  - File descriptor of the open file.
 *   cmd - Specifies the type of lock operation to be performed.
 *   It can be one of the following:
 *     F_LOCK: Request an exclusive (write) lock. If the lock is not
 *             available, the call may block until it can be acquired.
 *     F_TLOCK: Try to request an exclusive (write) lock. If the lock is not
 *              available, the call will not block and will return
 *              immediately.
 *     F_ULOCK: Unlock an existing lock.
 *     F_TEST: Test the lock. Returns 0 if the file is unlocked or locked by
 *             the calling process, or -1 with errno set to EACCES if another
 *             process holds the lock.
 *   len - Length of the locked region, relative to the current file
 *         position.
 *
 * Returned Value:
 *   The returned value of lockf() depends on the success or failure of the
 *   operation. On success, the return value is 0. On failure, the return
 *   value is -1, and the errno variable is set to indicate the specific
 *   error.
 *
 ****************************************************************************/

int lockf(int fd, int cmd, off_t len)
{
  struct flock lock;

  lock.l_whence = SEEK_CUR;
  lock.l_start = 0;
  lock.l_len = len;

  switch (cmd)
    {
      case F_LOCK:
        {
          lock.l_type = F_WRLCK;
          return fcntl(fd, F_SETLKW, &lock);
        }

      case F_TLOCK:
        {
          lock.l_type = F_WRLCK;
          return fcntl(fd, F_SETLK, &lock);
        }

      case F_ULOCK:
        {
          lock.l_type = F_UNLCK;
          return fcntl(fd, F_SETLK, &lock);
        }

      case F_TEST:
        {
          lock.l_type = F_RDLCK;
          if (fcntl(fd, F_GETLK, &lock) < 0)
            {
              return ERROR;
            }

          /* Check result */

          if (lock.l_type == F_UNLCK || lock.l_pid == getpid())
            {
              return OK;
            }

          set_errno(EACCES);
          return ERROR;
        }
    }

  set_errno(EINVAL);
  return ERROR;
}
