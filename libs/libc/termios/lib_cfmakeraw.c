/****************************************************************************
 * libs/libc/termios/lib_cfmakeraw.c
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

#include <termios.h>
#include <assert.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cfmakeraw
 *
 * Description:
 *   The cfmakeraw() function is a non-POSIX function that sets the terminal
 *   to something like the "raw" mode.
 *
 * Input Parameters:
 *   termiosp - The termiosp argument is a pointer to a termios structure.
 *
 * Returned Value:
 *
 ****************************************************************************/

void cfmakeraw(FAR struct termios *termiosp)
{
  DEBUGASSERT(termiosp);
  termiosp->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                         | INLCR | IGNCR | ICRNL | IXON);
  termiosp->c_oflag &= ~OPOST;
  termiosp->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
#ifdef CONFIG_SERIAL_TERMIOS
  termiosp->c_cflag &= ~(CSIZE | PARENB);
  termiosp->c_cflag |= CS8;
#endif
}
