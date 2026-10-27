/****************************************************************************
 * libs/libc/stdlib/lib_grantpt.c
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

#include <sys/ioctl.h>
#include <stdint.h>
#include <stdlib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: grantpt
 *
 * Description:
 *   Give the slave pseudoterminal of fd its owner and mode.  NuttX creates
 *   the slave with them already, so this only checks that fd is a
 *   pseudoterminal.
 *
 * Returned Value:
 *   0 on success.  On failure, -1 with errno set by ioctl(): EBADF if fd
 *   is not open, ENOTTY if fd is not a pseudoterminal.
 *
 ****************************************************************************/

int grantpt(int fd)
{
  int ptyno;

  return ioctl(fd, TIOCGPTN, (unsigned long)((uintptr_t)&ptyno)) < 0 ?
         -1 : 0;
}
