/****************************************************************************
 * libs/libc/unistd/lib_futimens.c
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

#include <sys/stat.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: futimens
 *
 * Description:
 *   futimens() update the timestamps of a file with nanosecond precision.
 *   This contrasts with the historical utime(2) and utimes(2), which permit
 *   only second and microsecond precision, respectively, when setting file
 *   timestamps.
 *
 * Input Parameters:
 *   fd  - Specifies the fd to be modified
 *   times - Specifies the time value to set
 *
 * Returned Value:
 *   On success, futimens() return 0.
 *   On error, -1 is returned and errno is set to indicate the error.
 *
 ****************************************************************************/

int futimens(int fd, const struct timespec times[2])
{
  set_errno(ENOTSUP);
  return ERROR;
}
