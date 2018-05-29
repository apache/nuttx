/****************************************************************************
 * libs/libc/termios/lib_tcflow.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <nuttx/config.h>

#include <sys/ioctl.h>

#include <termios.h>
#include <errno.h>

#include <nuttx/serial/tioctl.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcflow
 *
 * Description:
 *   The tcflow() function will suspend or restart transmission or reception
 *   of data on the object referred to by 'fd', depending on the value of
 *   'action'.
 *
 * Input Parameters:
 *  fd     - An open file descriptor associated with a terminal.
 *  action - Describes the action to be performed:
 *           TCOOFF  - Output will be suspended.
 *           TCOON   - Suspended output will be restarted.
 *           TCIOFF  - The system will transmit a STOP character, which is
 *                     intended to cause the terminal device to stop
 *                     transmitting data to the system.
 *           TCION   - The system will transmit a START character, which is
 *                     intended to cause the terminal device to start
 *                     transmitting data to the system.
 *
 *   The default on the opening of a terminal file is that neither its input nor
 *   its output are suspended.
 *
 * Returned Value:
 *   Upon successful completion, 0 is returned. Otherwise, -1 is returned and
 *   errno is set to indicate the nature of the error.
 *
 ****************************************************************************/

int tcflow(int fd, int action)
{
  return ioctl(fd, TCXONC, (unsigned long)action);
}
