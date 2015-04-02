/****************************************************************************
 * include/sys/boardctl.h
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

#ifndef __INCLUDE_SYS_BOARDCTL_H
#define __INCLUDE_SYS_BOARDCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_LIB_BOARDCTL

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Common commands
 *
 * CMD:           BOARDIOC_INIT
 * DESCRIPTION:   Perform one-time application initialization.
 * ARG:           None
 * CONFIGURATION: CONFIG_LIB_BOARDCTL
 * DEPENDENCIES:  Board logic must provide board_app_initialization
 *
 * CMD:           BOARDIOC_TSCTEST_SETUP
 * DESCRIPTION:   Touchscreen controller test configuration
 * ARG:           Touch controller device minor number
 * CONFIGURATION: CONFIG_LIB_BOARDCTL && CONFIG_BOARDCTL_TSCTEST
 * DEPENDENCIES:  Board logic must provide board_tsc_setup()
 *
 * CMD:           BOARDIOC_TSCTEST_TEARDOWN
 * DESCRIPTION:   Touchscreen controller test configuration
 * ARG:           None
 * CONFIGURATION: CONFIG_LIB_BOARDCTL && CONFIG_BOARDCTL_TSCTEST
 * DEPENDENCIES:  Board logic must provide board_tsc_teardown()
 *
 * CMD:           BOARDIOC_ADCTEST_SETUP
 * DESCRIPTION:   ADC controller test configuration
 * ARG:           None
 * CONFIGURATION: CONFIG_LIB_BOARDCTL && CONFIG_BOARDCTL_ADCTEST
 * DEPENDENCIES:  Board logic must provide board_adc_setup()
 *
 * CMD:           BOARDIOC_PWMTEST_SETUP
 * DESCRIPTION:   PWM controller test configuration
 * ARG:           None
 * CONFIGURATION: CONFIG_LIB_BOARDCTL && CONFIG_BOARDCTL_PWMTEST
 * DEPENDENCIES:  Board logic must provide board_pwm_setup()
 *
 * CMD:           BOARDIOC_GRAPHICS_SETUP
 * DESCRIPTION:   Configure graphics that require special initialization
 *                procedures
 * ARG:           A pointer to an instance of struct boardioc_graphics_s
 * CONFIGURATION: CONFIG_LIB_BOARDCTL && CONFIG_BOARDCTL_GRAPHICS
 * DEPENDENCIES:  Board logic must provide board_adc_setup()
 */

#define BOARDIOC_INIT              _BOARDIOC(0x0001)
#define BOARDIOC_TSCTEST_SETUP     _BOARDIOC(0x0002)
#define BOARDIOC_TSCTEST_TEARDOWN  _BOARDIOC(0x0003)
#define BOARDIOC_ADCTEST_SETUP     _BOARDIOC(0x0004)
#define BOARDIOC_PWMTEST_SETUP     _BOARDIOC(0x0005)
#define BOARDIOC_GRAPHICS_SETUP    _BOARDIOC(0x0006)

/* If CONFIG_BOARDCTL_IOCTL=y, then boad-specific commands will be support.
 * In this case, all commands not recognized by boardctl() will be forwarded
 * to the board-provided board_ioctl() function.
 *
 * User defined board commands may begin with this value:
 */

#define BOARDIOC_USER              _BOARDIOC(0x0007)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Structure used to pass arguments and get returned values from the
 * BOARDIOC_GRAPHICS_SETUP command.
 */

#ifdef CONFIG_NX_LCDDRIVER
struct lcd_dev_s;                /* Forward reference */
#else
struct fb_vtable_s;              /* Forward reference */
#endif

struct boardioc_graphics_s
{
  int devno;                     /* IN: Graphics device number */
#ifdef CONFIG_NX_LCDDRIVER
  FAR struct lcd_dev_s *dev;     /* OUT: LCD driver instance */
#else
  FAR struct fb_vtable_s *dev;   /* OUT: Framebuffer driver instance */
#endif
};

/****************************************************************************
 * Public Function Prototypes
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
 *   the application to perform direct IOCTL-like calls to the board-specific
 *   logic.  It is especially useful for setting up board operational and
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

int boardctl(unsigned int cmd, uintptr_t arg);

#endif /* CONFIG_LIB_BOARDCTL */
#endif /* __INCLUDE_SYS_BOARDCTL_H */
