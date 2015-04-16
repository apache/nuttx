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
      /* CMD:           BOARDIOC_TSCTEST_SETUP
       * DESCRIPTION:   Touchscreen controller test configuration
       * ARG:           Touch controller device minor number
       * CONFIGURATION: CONFIG_LIB_BOARDCTL && CONFIG_BOARDCTL_TSCTEST
       * DEPENDENCIES:  Board logic must provide board_tsc_setup()
       */

      case BOARDIOC_TSCTEST_SETUP:
        {
          ret = board_tsc_setup((int)arg);
        }
        break;

      /* CMD:           BOARDIOC_TSCTEST_TEARDOWN
       * DESCRIPTION:   Touchscreen controller test configuration
       * ARG:           None
       * CONFIGURATION: CONFIG_LIB_BOARDCTL && CONFIG_BOARDCTL_TSCTEST
       * DEPENDENCIES:  Board logic must provide board_tsc_teardown()
       */

      case BOARDIOC_TSCTEST_TEARDOWN:
        {
          board_tsc_teardown();
          ret = OK;
        }
        break;
#endif

#ifdef CONFIG_BOARDCTL_ADCTEST
      /* CMD:           BOARDIOC_ADCTEST_SETUP
       * DESCRIPTION:   ADC controller test configuration
       * ARG:           None
       * CONFIGURATION: CONFIG_LIB_BOARDCTL && CONFIG_BOARDCTL_ADCTEST
       * DEPENDENCIES:  Board logic must provide board_adc_setup()
       */

      case BOARDIOC_ADCTEST_SETUP:
        {
          ret = board_adc_setup();
        }
        break;
#endif

#ifdef CONFIG_BOARDCTL_PWMTEST
      /* CMD:           BOARDIOC_PWMTEST_SETUP
       * DESCRIPTION:   PWM controller test configuration
       * ARG:           None
       * CONFIGURATION: CONFIG_LIB_BOARDCTL && CONFIG_BOARDCTL_PWMTEST
       * DEPENDENCIES:  Board logic must provide board_pwm_setup()
       */

      case BOARDIOC_PWMTEST_SETUP:
        {
          ret = board_pwm_setup();
        }
        break;
#endif

#ifdef CONFIG_BOARDCTL_GRAPHICS
      /* CMD:           BOARDIOC_GRAPHICS_SETUP
       * DESCRIPTION:   Configure graphics that require special initialization
       *                procedures
       * ARG:           A pointer to an instance of struct boardioc_graphics_s
       * CONFIGURATION: CONFIG_LIB_BOARDCTL && CONFIG_BOARDCTL_GRAPHICS
       * DEPENDENCIES:  Board logic must provide board_adc_setup()
       */

      case BOARDIOC_GRAPHICS_SETUP:
        {
          FAR struct boardioc_graphics_s *setup = 
            ( FAR struct boardioc_graphics_s *)arg;

          setup->dev = board_graphics_setup(setup->devno);
          ret = setup->dev ? OK : -ENODEV;
        }
        break;
#endif

       default:
         {
#ifdef CONFIG_BOARDCTL_IOCTL
           /* Boards may also select CONFIG_BOARDCTL_IOCTL=y to enable board-
            * specific commands.  In this case, all commands not recognized
            * by boardctl() will be forwarded to the board-provided board_ioctl()
            * function.
            */

           ret = board_ioctl(cmd, arg);
#else
           ret = -ENOTTY;
#endif
         }
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
