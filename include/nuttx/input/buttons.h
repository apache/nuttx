/************************************************************************************
 * include/nuttx/input/buttons.h
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __INCLUDE_NUTTX_INPUT_BUTTONS_H
#define __INCLUDE_NUTTX_INPUT_BUTTONS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_BUTTONS_NPOLLWAITERS
#  define CONFIG_BUTTONS_NPOLLWAITERS 2
#endif

/* ioctl commands */

/* Command:     BTNIOC_SUPPORTED
 * Description: Report the set of button events supported by the hardware;
 * Argument:    A pointer to writeable btn_buttonset_t value in which to
 *              return the set of supported buttons.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define BTNIOC_SUPPORTED  _BTNIOC(0x0001)

/* Command:     BTNIOC_POLLEVENTS
 * Description: Specify the set of button events that can cause a poll()
 *              to awaken.  The default is all button depressions and all
 *              button releases (all supported buttons);
 * Argument:    A read-only pointer to an instance of struct btn_pollevents_s
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define BTNIOC_POLLEVENTS _BTNIOC(0x0002)

/* Command:     BTNIOC_REGISTER
 * Description: Register to receive a signal whenever there is a change in
 *              the state of button inputs.  This feature, of course, depends
 *              upon interrupt GPIO support from the platform.
 * Argument:    A read-only pointer to an instance of struct btn_notify_s
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define BTNIOC_REGISTER   _BTNIOC(0x0003)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This type is a bit set that contains the state of all buttons as defined
 * in arch/board/board.h.  This is the value that is returned when reading
 * from the button driver.
 */

typedef uint32_t btn_buttonset_t;

/* A reference to this structure is provided with the BTNIOC_POLLEVENTS IOCTL
 * command and describes the conditions under which the client would like
 * to receive notification.
 */

struct btn_pollevents_s
{
  btn_buttonset_t bp_press;   /* Set of button depressions to wake up the poll */
  btn_buttonset_t bp_release; /* Set of button releases to wake up the poll */
};

/* A reference to this structure is provided with the BTNIOC_REGISTER IOCTL
 * command and describes the conditions under which the client would like
 * to receive notification.
 */

struct btn_notify_s
{
  btn_buttonset_t bn_press;   /* Set of button depressions to be notified */
  btn_buttonset_t bn_release; /* Set of button releases to be notified */
  uint8_t         bn_signo;   /* Signal number to use in the notification */
};

/* This is the type of the button interrupt handler used with the struct
 * btn_lowerhalf_s enable() method.
 */

struct btn_lowerhalf_s;
typedef CODE void (*btn_handler_t)
  (FAR const struct btn_lowerhalf_s *lower, FAR void *arg);

/* The button driver is a two-part driver:
 *
 * 1) A common upper half driver that provides the common user interface to
 *    the buttons,
 * 2) Platform-specific lower half drivers that provide the interface
 *    between the common upper half and the platform discrete button inputs.
 *
 * This structure defines the interface between an instance of the lower
 * half driver and the common upper half driver.  Such an instance is
 * passed to the upper half driver when the driver is initialized, binding
 * the upper and lower halves into one driver.
 */

struct btn_lowerhalf_s
{
  /* Return the set of buttons supported by the board */

  CODE btn_buttonset_t (*bl_supported)(FAR const struct btn_lowerhalf_s *lower);

  /* Return the current state of button data (only) */

  CODE btn_buttonset_t (*bl_buttons)(FAR const struct btn_lowerhalf_s *lower);

  /* Enable interrupts on the selected set of buttons.  An empty set will
   * disable all interrupts.
   */

  CODE void (*bl_enable)(FAR const struct btn_lowerhalf_s *lower,
                         btn_buttonset_t press, btn_buttonset_t release,
                         btn_handler_t handler, FAR void *arg);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: btn_register
 *
 * Description:
 *   Bind the lower half button driver to an instance of the upper half
 *   button driver and register the composite character driver as the
 *   specified device.
 *
 * Input Parameters:
 *   devname - The name of the button device to be registered.
 *     This should be a string of the form "/dev/btnN" where N is the
 *     minor device number.
 *   lower - An instance of the platform-specific button lower half driver.
 *
 * Returned Values:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int btn_register(FAR const char *devname,
                 FAR const struct btn_lowerhalf_s *lower);

/****************************************************************************
 * Name: btn_lower_initialize
 *
 * Description:
 *   Initialize the generic button lower half driver, bind it and register
 *   it with the upper half button driver as devname.
 *
 ****************************************************************************/

#if CONFIG_BUTTONS_LOWER
int btn_lower_initialize(FAR const char *devname);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_INPUT_BUTTONS_H */

