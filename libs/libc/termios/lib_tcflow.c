/****************************************************************************
 * libs/libc/termios/lib_tcflow.c
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
 *   The default on the opening of a terminal file is that neither its input
 *   nor its output are suspended.
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
