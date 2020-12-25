/****************************************************************************
 * libs/libc/termios/lib_tcflush.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Alan Carvalho de Assis
 *
 *   Source:
 *     https://github.com/raggi/apue.2e/blob/master/termios/isatty.c
 *     based on Advanced Programming in the UNIX Environment
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

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
