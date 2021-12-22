/****************************************************************************
 * drivers/serial/tcdrain.c
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

#include <nuttx/cancelpt.h>
#include <nuttx/serial/tioctl.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcdrain
 *
 * Description:
 *   Function for draining the output buffer of a terminal/serial device
 *
 * Input Parameters:
 *   fd  - The 'fd' argument is an open file descriptor associated with a
 *         terminal.
 *
 * Returned Value:
 *   Upon successful completion, 0 is returned. Otherwise, -1 is returned and
 *   errno is set to indicate the error.
 *
 ****************************************************************************/

int tcdrain(int fd)
{
  int ret;

  /* tcdrain is a cancellation point */

  if (enter_cancellation_point())
    {
#ifdef CONFIG_CANCELLATION_POINTS
      /* If there is a pending cancellation, then do not perform
       * the wait.  Exit now with ECANCELED.
       */

      set_errno(ECANCELED);
      leave_cancellation_point();
      return ERROR;
#endif
    }

  /* Perform the TCDRM IOCTL command.  It is safe to use the file descriptor
   * in this context because we are executing on the calling application's
   * thread.
   *
   * NOTE: ioctl() will set the errno variable and return ERROR if any error
   * occurs.
   */

  ret = ioctl(fd, TCDRN, (unsigned long)0);

  leave_cancellation_point();
  return ret;
}
