/****************************************************************************
 * boards/arm/cxd56xx/spresense/src/cxd56_ioctl.c
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

#include <sys/types.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include <arch/chip/chip.h>

#include "cxd56_usbdev.h"
#include "arch/board/cxd56_sdcard.h"

#ifdef CONFIG_BOARDCTL_IOCTL

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: boardctl
 *
 * Description:
 *   In a small embedded system, there will typically be a much greater
 *   interaction between application and low-level board features.  The
 *   canonically correct to implement such interactions is by implementing a
 *   character driver and performing the interactions via low level ioctl
 *   calls.  This, however, may not be practical in many cases and will lead
 *   to "correct" but awkward implementations.
 *
 *   boardctl() is non-standard OS interface to alleviate the problem.
 *   It basically circumvents the normal device driver ioctl interlace and
 *   allows the application to perform direct IOCTL-like calls to the
 *   board-specific logic.
 *   It is especially useful for setting up board operational and
 *   test configurations.
 *
 * Input Parameters:
 *   cmd - Identifies the board command to be executed
 *   arg - The argument that accompanies the command.  The nature of the
 *         argument is determined by the specific command.
 *
 * Returned Value:
 *   On success zero (OK) is returned; -1 (ERROR) is returned on failure
 *   with the errno variable to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_ioctl(unsigned int cmd, uintptr_t arg)
{
  int ret = -ENOTTY; /* The correct return for the case of an unrecognized command */

  switch (cmd)
    {
#ifdef CONFIG_USBDEV
      /* CMD:           BOARDIOC_USBDEV_SETNOTIFYSIG
       * DESCRIPTION:   Set signal id for notify USB device connection status
       *                and supply current value.
       * ARG:           None
       * CONFIGURATION: CONFIG_BOARDCTL
       * DEPENDENCIES:  Board logic must provide board_app_initialization
       */

      case BOARDIOC_USBDEV_SETNOTIFYSIG:
        {
          ret = cxd56_usbdev_setsigno((int)arg);
        }
        break;
#endif

      default:
        break;
    }

  /* Any failure errno value will be set in boardctl() */

  return ret;
}
#endif
