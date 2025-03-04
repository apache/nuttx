/****************************************************************************
 * libs/libc/unistd/lib_flock.c
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

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: flock
 *
 * Description:
 *   flock() is a function that allows a process to apply or remove an
 *   advisory lock on an open file. The lock can be either a shared (read)
 *   lock or an exclusive (write) lock. This lock is advisory, meaning that
 *   it is not enforced by the system and it is up to cooperating processes
 *   to honor the lock.
 *
 * Input Parameters:
 *   fd  - File descriptor of the open file.
 *   cmd - Specifies the type of lock operation to be performed.
 *   It can be one of the following:
 *     LOCK_SH: Request a shared (read) lock. If the lock is not available.
 *     LOCK_EX: Request an exclusive (write) lock. If the lock is not
 *              available.
 *     LOCK_UN: Unlock an existing lock.
 *     LOCK_NB: Do not block when locking.
 *
 * Returned Value:
 *   The returned value of flock() depends on the success or failure of the
 *   operation. On success, the return value is 0. On failure, the return
 *   value is -1, and the errno variable is set to indicate the specific
 *   error.
 *
 ****************************************************************************/

int flock(int fd, int cmd)
{
  struct flock lock;

  lock.l_whence = SEEK_SET;
  lock.l_start = 0;
  lock.l_len = 0;

  if ((cmd & LOCK_SH) != 0)
    {
      lock.l_type = F_RDLCK;
    }
  else if ((cmd & LOCK_EX) != 0)
    {
      lock.l_type = F_WRLCK;
    }
  else if ((cmd & LOCK_UN) != 0)
    {
      lock.l_type = F_UNLCK;
    }
  else
    {
      set_errno(EINVAL);
      return ERROR;
    }

  if ((cmd & LOCK_NB) != 0)
    {
      return fcntl(fd, F_SETLK, &lock);
    }
  else
    {
      return fcntl(fd, F_SETLKW, &lock);
    }
}
