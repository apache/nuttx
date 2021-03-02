/****************************************************************************
 * libs/libc/stdlib/lib_unlockpt.c
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

#include <sys/ioctl.h>
#include <stdlib.h>

#ifdef CONFIG_PSEUDOTERM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: unlockpt
 *
 * Description:
 *   The unlockpt() function unlocks the slave pseudoterminal device
 *   corresponding to the master pseudoterminal referred to by fd.
 *   unlockpt() must be called before opening the slave side of a
 *   pseudoterminal.
 *
 * Returned Value:
 *   When successful, unlockpt() returns 0. Otherwise, it returns -1 and
 *   sets errno appropriately.
 *
 *     EBADF - The fd argument is not a file descriptor open for writing.
 *     EINVAL - The fd argument is not associated with a master
 *       pseudoterminal
 *
 ****************************************************************************/

int unlockpt(int fd)
{
  return ioctl(fd, TIOCSPTLCK, 0);
}

#endif /* CONFIG_PSEUDOTERM */
