/****************************************************************************
 * include/nuttx/input/djoystick.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

/* This header file provides definition for a standard discrete joystick
 * interface.  A discrete joystick refers to a joystick that could be
 * implemented entirely with GPIO input pins.  So up, down, left, and right
 * are all discrete values like buttons (as opposed to integer values like
 * you might obtain from an analog joystick).
 *
 * The discrete joystick driver exports a standard character driver
 * interface. By convention, the discrete joystick is registered as an input
 * device at /dev/djoyN where N uniquely identifies the driver instance.
 *
 * This header file documents the generic interface that all NuttX discrete
 * joystick devices must conform.  It adds standards and conventions on top
 * of the standard character driver interface.
 */

#ifndef __INCLUDE_NUTTX_INPUT_DJOYSTICK_H
#define __INCLUDE_NUTTX_INPUT_DJOYSTICK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_DJOYSTICK_NPOLLWAITERS
#  define CONFIG_DJOYSTICK_NPOLLWAITERS 2
#endif

/* Joystick Interface *******************************************************/
/* These definitions provide the meaning of all of the bits that may be
 * reported in the djoy_buttonset_t bitset.
 */

#define DJOY_UP              (0)                  /* Bit 0: Joystick UP */
#define DJOY_DOWN            (1)                  /* Bit 1: Joystick DOWN */
#define DJOY_LEFT            (2)                  /* Bit 2: Joystick LEFT */
#define DJOY_RIGHT           (3)                  /* Bit 3: Joystick RIGHT */
#define DJOY_BUTTON_1        (4)                  /* Bit 4: Button 1 */
#define DJOY_BUTTON_2        (5)                  /* Bit 5: Button 2 */
#define DJOY_BUTTON_3        (6)                  /* Bit 6: Button 3 */
#define DJOY_BUTTON_4        (7)                  /* Bit 7: Button 4 */
#define DJOY_NDISCRETES      (8)                  /* Total number of discrete signals */

#define DJOY_UP_BIT          (1 << DJOY_UP)       /* 1:Joystick UP selected */
#define DJOY_DOWN_BIT        (1 << DJOY_DOWN)     /* 1:Joystick DOWN selected */
#define DJOY_LEFT_BIT        (1 << DJOY_LEFT)     /* 1:Joystick LEFT selected */
#define DJOY_RIGHT_BIT       (1 << DJOY_RIGHT)    /* 1:Joystick RIGHT selected */
#define DJOY_BUTTONS_JOYBITS 0x0f                 /* Set of all joystick directions */
#define DJOY_BUTTON_1_BIT    (1 << DJOY_BUTTON_1) /* 1:Button 1 pressed */
#define DJOY_BUTTON_2_BIT    (1 << DJOY_BUTTON_2) /* 1:Button 2 pressed */
#define DJOY_BUTTON_3_BIT    (1 << DJOY_BUTTON_3) /* 1:Button 3 pressed */
#define DJOY_BUTTON_4_BIT    (1 << DJOY_BUTTON_4) /* 1:Button 4 pressed */
#define DJOY_BUTTONS_ALLBITS 0xf0                 /* Set of all buttons */
#define DJOY_ALLBITS         0xff                 /* Set of all bits */

/* Typical usage */

#define DJOY_BUTTON_SELECT     DJOY_BUTTON_1
#define DJOY_BUTTON_FIRE       DJOY_BUTTON_2
#define DJOY_BUTTON_JUMP       DJOY_BUTTON_3
#define DJOY_BUTTON_RUN        DJOY_BUTTON_4

#define DJOY_BUTTON_SELECT_BIT DJOY_BUTTON_1_BIT
#define DJOY_BUTTON_FIRE_BIT   DJOY_BUTTON_2_BIT
#define DJOY_BUTTON_JUMP_BIT   DJOY_BUTTON_3_BIT
#define DJOY_BUTTON_RUN_BIT    DJOY_BUTTON_4_BIT

/* IOCTL commands
 *
 * Discrete joystick drivers do not support the character driver write() or
 * seek() methods.  The remaining driver methods behave as follows:
 *
 * 1) The read() method will always return a single value of size
 *    djoy_buttonset_t represent the current state of the joystick buttons.
 *    read() never blocks.
 * 2) The poll() method can be used to notify a client if there is a change
 *    in any of the joystick discrete inputs.  This feature, of course,
 *    depends upon interrupt GPIO support from the platform.  NOTE: that
 *    semantics of poll() for POLLIN are atypical:  The successful poll
 *    means that the data has changed and has nothing to with the
 *    availability of data to be read; data is always available to be
 *    read.
 * 3) The ioctl() method supports the commands documented below:
 */

/* Command:     DJOYIOC_SUPPORTED
 * Description: Report the set of button events supported by the hardware;
 * Argument:    A pointer to writeable integer value in which to return the
 *              set of supported buttons.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define DJOYIOC_SUPPORTED  _DJOYIOC(0x0001)

/* Command:     DJOYIOC_POLLEVENTS
 * Description: Specify the set of button events that can cause a poll()
 *              to awaken.  The default is all button depressions and all
 *              button releases (all supported buttons);
 * Argument:    A read-only pointer to an instance of struct djoy_pollevents_s
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define DJOYIOC_POLLEVENTS  _DJOYIOC(0x0002)

/* Command:     DJOYIOC_REGISTER
 * Description: Register to receive a signal whenever there is a change in
 *              any of the joystick discrete inputs.  This feature, of
 *              course, depends upon interrupt GPIO support from the
 *              platform.
 * Argument:    A read-only pointer to an instance of struct djoy_notify_s
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define DJOYIOC_REGISTER   _DJOYIOC(0x0003)

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* This type is a bit set that contains the state of all discrete joystick
 * buttons.
 */

typedef uint8_t djoy_buttonset_t;

/* A reference to this structure is provided with the DJOYIOC_POLLEVENTS IOCTL
 * command and describes the conditions under which the client would like
 * to receive notification.
 */

struct djoy_pollevents_s
{
  djoy_buttonset_t dp_press;   /* Set of button depressions to wake up the poll */
  djoy_buttonset_t dp_release; /* Set of button releases to wake up the poll */
};

/* A reference to this structure is provided with the DJOYIOC_REGISTER IOCTL
 * command and describes the conditions under which the client would like
 * to receive notification.
 */

struct djoy_notify_s
{
  djoy_buttonset_t dn_press;   /* Set of button depressions to be notified */
  djoy_buttonset_t dn_release; /* Set of button releases to be notified */
  uint8_t          dn_signo;   /* Signal number to use in the notification */
};

/* This is the type of the discrete joystick interrupt handler used with
 * the struct djoy_lowerhalf_s enable() method.
 */

struct djoy_lowerhalf_s;
typedef CODE void (*djoy_interrupt_t)
  (FAR const struct djoy_lowerhalf_s *lower, FAR void *arg);

/* The discrete joystick driver is a two-part driver:
 *
 * 1) A common upper half driver that provides the common user interface to
 *    the joystick,
 * 2) Platform-specific lower half drivers that provide the interface
 *    between the common upper half and the platform discrete inputs.
 *
 * This structure defines the interface between an instance of the lower
 * half driver and the common upper half driver.  Such an instance is
 * passed to the upper half driver when the driver is initialized, binding
 * the upper and lower halves into one driver.
 */

struct djoy_lowerhalf_s
{
  /* Return the set of buttons supported on the discrete joystick device */

  CODE djoy_buttonset_t (*dl_supported)(FAR const struct djoy_lowerhalf_s *lower);

  /* Return the current state of all discrete joystick buttons */

  CODE djoy_buttonset_t (*dl_sample)(FAR const struct djoy_lowerhalf_s *lower);

  /* Enable interrupts on the selected set of joystick buttons.  And empty
   * set will disable all interrupts.
   */

  CODE void (*dl_enable)(FAR const struct djoy_lowerhalf_s *lower,
                         djoy_buttonset_t press, djoy_buttonset_t release,
                         djoy_interrupt_t handler, FAR void *arg);
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
 * Name: djoy_register
 *
 * Description:
 *   Bind the lower half discrete joystick driver to an instance of the
 *   upper half discrete joystick driver and register the composite character
 *   driver as the specified device.
 *
 * Input Parameters:
 *   devname - The name of the discrete joystick device to be registers.
 *     This should be a string of the form "/dev/djoyN" where N is the
 *     minor device number.
 *   lower - An instance of the platform-specific discrete joystick lower
 *     half driver.
 *
 * Returned Values:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int djoy_register(FAR const char *devname,
                  FAR const struct djoy_lowerhalf_s *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_INPUT_DJOYSTICK_H */
