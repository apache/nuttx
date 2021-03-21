/****************************************************************************
 * libs/libc/termios/lib_isatty.c
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

/*   Source:
 *     https://github.com/raggi/apue.2e/blob/master/termios/isatty.c
 *     based on Advanced Programming in the UNIX Environment
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <termios.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: isatty
 *
 * Description:
 *   Return a non-zero (true) value if the file descriptor, fd, corresponds
 *   to a TTYY.
 *
 *   NOTE: NuttX, of course, does not have true TTYs in the sense that this
 *   function is intended.  In this implementation, the function simply
 *   returns true if the file descriptor is associated with a driver that
 *   responds wo tcgetattr() without an error -- that it, the driver supports
 *   the NuttX TCGETS ioctl command.
 *
 *   Of course, that can only be true if CONFIG_SERIAL_TERMIOS=y.
 *
 ****************************************************************************/

int isatty(int fd)
{
  struct termios ts;
  return (tcgetattr(fd, &ts) >= 0);
}
