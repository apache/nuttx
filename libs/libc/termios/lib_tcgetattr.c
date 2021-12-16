/****************************************************************************
 * libs/libc/termios/lib_tcgetattr.c
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

#include <termios.h>
#include <errno.h>

#include <nuttx/serial/tioctl.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcgetattr
 *
 * Description:
 *   The tcgetattr() function gets the parameters associated with the
 *   terminal referred to by 'fd' and stores them in the termios structure
 *   referenced by 'termiosp'.
 *
 * Input Parameters:
 *   fd - The 'fd' argument is an open file descriptor associated with a
 *        terminal.
 *   termiosp - The termiosp argument is a pointer to a termios structure.
 *
 * Returned Value:
 *   Upon successful completion, 0 is returned. Otherwise, -1 is returned and
 *   errno is set to indicate the error.
 *   The following errors may be reported:
 *
 *   - EBADF: The 'fd' argument is not a valid file descriptor.
 *   - ENOTTY: The file associated with 'fd' is not a terminal.
 *
 ****************************************************************************/

int tcgetattr(int fd, FAR struct termios *termiosp)
{
  return ioctl(fd, TCGETS, (unsigned long)(uintptr_t)termiosp);
}
