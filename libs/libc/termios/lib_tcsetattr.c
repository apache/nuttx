/****************************************************************************
 * libs/libc/termios/lib_tcsetattr.c
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
 * Name: tcsetattr
 *
 * Description:
 *   The tcsetattr() function sets the parameters associated with the
 *   terminal referred to by the open file descriptor 'fd' from the termios
 *   structure referenced by 'termiop' as follows:
 *
 *   If 'options' is TCSANOW, the change will occur immediately.
 *
 *   If 'options' is TCSADRAIN, the change will occur after all output
 *   written to 'fd' is transmitted. This function should be used when
 *   changing parameters that affect output.
 *
 *   If 'options' is TCSAFLUSH, the change will occur after all
 *   output written to 'fd' is transmitted, and all input so far received but
 *   not read will be discarded before the change is made.
 *
 *   The tcsetattr() function will return successfully if it was able to
 *   perform any of the requested actions, even if some of the requested
 *   actions could not be performed. It will set all the attributes that
 *   implementation supports as requested and leave all the attributes not
 *   supported by the implementation unchanged. If no part of the request
 *   can be honoured, it will return -1 and set errno to EINVAL.
 *
 *   The effect of tcsetattr() is undefined if the value of the termios
 *   structure pointed to by 'termiop' was not derived from the result of
 *   a call to tcgetattr() on 'fd'; an application should modify only fields
 *   and flags defined by this specification between the call to tcgetattr()
 *   and tcsetattr(), leaving all other fields and flags unmodified.
 *
 * Returned Value:
 *
 *   Upon successful completion, 0 is returned. Otherwise, -1 is returned
 *   and errno is set to indicate the error.  The following errors may be
 *   reported:
 *
 *   - EBADF: The 'fd' argument is not a valid file descriptor.
 *   - EINTR:  A signal interrupted tcsetattr().
 *   - EINVAL: The 'options' argument is not a supported value, or
 *     an attempt was made to change an attribute represented in the
 *     termios structure to an unsupported value.
 *   - ENOTTY: The file associated with 'fd' is not a terminal.
 *
 ****************************************************************************/

int tcsetattr(int fd, int options, FAR const struct termios *termiosp)
{
  if (options != TCSANOW)
    {
      tcdrain(fd);
    }

  if (options == TCSAFLUSH)
    {
      tcflush(fd, TCIFLUSH);
    }

  return ioctl(fd, TCSETS, (unsigned long)(uintptr_t)termiosp);
}
