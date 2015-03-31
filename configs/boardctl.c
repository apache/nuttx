/****************************************************************************
 * configs/boardctl.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <sys/boardctl.h>
#include <stdint.h>
#include <errno.h>

#include <nuttx/board.h>

#ifdef CONFIG_LIB_BOARDCTL

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
 *   boardctl() is non-standard OS interface to alleviate the problem.  It
 *   basically circumvents the normal device driver ioctl interlace and allows
 *   the application to perform direction IOCTL-like calls to the board-specific
 *   logic.  In it is especially useful for setting up board operational and
 *   test configurations.
 *
 * Input Parameters:
 *   cmd - Identifies the board command to be executed
 *   arg - The argument that accompanies the command.  The nature of the
 *         argument is determined by the specific command.
 *
 * Returned Value:
 *   On success zero (OK) is returned; -1 (ERROR) is returned on failure
 *   with the errno variable to to indicate the nature of the failure.
 *
 ****************************************************************************/

int boardctl(unsigned int cmd, uintptr_t arg)
{
  int ret;

  switch (cmd)
    {
      /* CMD:           BOARDIOC_INIT
       * DESCRIPTION:   Perform one-time application initialization.
       * ARG:           None
       * CONFIGURATION: CONFIG_LIB_BOARDCTL
       * DEPENDENCIES:  Board logic must provide board_app_initialization
       */

      case BOARDIOC_INIT:
        ret = board_app_initialize();
        break;

#ifdef CONFIG_BOARDCTL_TSCTEST
      /* CMD:           BOARDIOC_TSCTEST
       * DESCRIPTION:   Touchscreen controller test configuration
       * ARG:           0: Setup touchscreen test, 1: Teardown touchscreen test
       * CONFIGURATION: CONFIG_LIB_BOARDCTL && 
       * DEPENDENCIES:  Board logic must provide board_tsc_setup() and
       *                board_tsc_teardown().
       */

      case BOARDIOC_TSCTEST:
        if (arg)
          {
            ret = board_tsc_setup();
          }
        else
          {
            ret = board_tsc_teardown();
          }

        break;
#endif

       default:
         ret = -ENOTTY;
         break;
    }

  /* Set the errno value on any errors */

  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}

#endif /* CONFIG_LIB_BOARDCTL */
