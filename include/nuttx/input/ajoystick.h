/****************************************************************************
 * include/nuttx/input/ajoystick.h
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

/* This header file provides definition for a standard analog joystick
 * interface.  An analog joystick refers to a joystick that provides X/Y
 * positional data as integer values such as might be provides by Analog-
 * to-Digital Conversion (ADC).  The analog positional data may also be
 * accompanied by discrete button data.
 *
 * The analog joystick driver exports a standard character driver
 * interface. By convention, the analog joystick is registered as an input
 * device at /dev/ajoyN where N uniquely identifies the driver instance.
 *
 * This header file documents the generic interface that all NuttX analog
 * joystick devices must conform.  It adds standards and conventions on top
 * of the standard character driver interface.
 */

#ifndef __INCLUDE_NUTTX_INPUT_AJOYSTICK_H
#define __INCLUDE_NUTTX_INPUT_AJOYSTICK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/input/ioctl.h>

#include <signal.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_INPUT_AJOYSTICK_NPOLLWAITERS
#  define CONFIG_INPUT_AJOYSTICK_NPOLLWAITERS 2
#endif

/* Joystick Interface *******************************************************/

/* These definitions provide the meaning of all of the bits that may be
 * reported in the ajoy_buttonset_t bitset.
 */

#define AJOY_BUTTON(n)       ((n)-1)              /* Bit n-1: Button n, n=1..8 */
#define AJOY_BUTTON_1        (0)                  /* Bit 0: Button 1 */
#define AJOY_BUTTON_2        (1)                  /* Bit 1: Button 2 */
#define AJOY_BUTTON_3        (2)                  /* Bit 2: Button 3 */
#define AJOY_BUTTON_4        (3)                  /* Bit 3: Button 4 */
#define AJOY_BUTTON_5        (4)                  /* Bit 4: Button 5 */
#define AJOY_BUTTON_6        (5)                  /* Bit 5: Button 6 */
#define AJOY_BUTTON_7        (6)                  /* Bit 6: Button 7 */
#define AJOY_BUTTON_8        (7)                  /* Bit 7: Button 8 */
#define AJOY_NBUTTONS        (8)                  /* Total number of buttons */

#define AJOY_BUTTON_1_BIT    (1 << AJOY_BUTTON_1) /* 1:Button 1 pressed */
#define AJOY_BUTTON_2_BIT    (1 << AJOY_BUTTON_2) /* 1:Button 2 pressed */
#define AJOY_BUTTON_3_BIT    (1 << AJOY_BUTTON_3) /* 1:Button 3 pressed */
#define AJOY_BUTTON_4_BIT    (1 << AJOY_BUTTON_4) /* 1:Button 4 pressed */
#define AJOY_BUTTON_5_BIT    (1 << AJOY_BUTTON_5) /* 1:Button 5 pressed */
#define AJOY_BUTTON_6_BIT    (1 << AJOY_BUTTON_6) /* 1:Button 6 pressed */
#define AJOY_BUTTON_7_BIT    (1 << AJOY_BUTTON_7) /* 1:Button 7 pressed */
#define AJOY_BUTTON_8_BIT    (1 << AJOY_BUTTON_8) /* 1:Button 8 pressed */
#define AJOY_BUTTONS_ALL     0xff                 /* Set of all buttons */

/* Typical usage */

#define AJOY_BUTTON_SELECT     AJOY_BUTTON_1
#define AJOY_BUTTON_FIRE       AJOY_BUTTON_2
#define AJOY_BUTTON_JUMP       AJOY_BUTTON_3

#define AJOY_BUTTON_SELECT_BIT AJOY_BUTTON_1_BIT
#define AJOY_BUTTON_FIRE_BIT   AJOY_BUTTON_2_BIT
#define AJOY_BUTTON_JUMP_BIT   AJOY_BUTTON_3_BIT

/* IOCTL commands
 *
 * Discrete joystick drivers do not support the character driver write() or
 * seek() methods.  The remaining driver methods behave as follows:
 *
 * 1) The read() method will always return a single value of size
 *    struct ajoy_sample_s represent the current joystick positional and the
 *    state of all joystick buttons. read() never blocks.  X an Y position
 *    data is raw converted data.  Zeroing and scaling must be performed by
 *    the application.
 * 2) The poll() method can be used to notify a client if there is a change
 *    in any of the joystick button inputs.  This feature, of course,
 *    depends upon interrupt GPIO support from the platform.  NOTE: that
 *    semantics of poll() for POLLIN are atypical:  The successful poll
 *    means that the button data has changed and has nothing to with the
 *    availability of data to be read; data is always available to be
 *    read.
 * 3) The ioctl() method supports the commands documented below:
 */

/* Command:     AJOYIOC_SUPPORTED
 * Description: Report the set of button events supported by the hardware;
 * Argument:    A pointer to writeable integer value in which to return the
 *              set of supported buttons.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define AJOYIOC_SUPPORTED  _AJOYIOC(0x0001)

/* Command:     AJOYIOC_POLLEVENTS
 * Description: Specify the set of button events that can cause a poll()
 *              to awaken.  The default is all button depressions and all
 *              button releases (all supported buttons);
 * Argument:    A read-only pointer to an instance of struct
 *              ajoy_pollevents_s
 * Return:      Zero (OK) on success.  Minus one will be returned on
 *              failure with the errno value set appropriately.
 */

#define AJOYIOC_POLLEVENTS  _AJOYIOC(0x0002)

/* Command:     AJOYIOC_REGISTER
 * Description: Register to receive a signal whenever there is a change in
 *              any of the joystick analog inputs.  This feature, of
 *              course, depends upon interrupt GPIO support from the
 *              platform.
 * Argument:    A read-only pointer to an instance of struct ajoy_notify_s
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define AJOYIOC_REGISTER   _AJOYIOC(0x0003)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This type is a bit set that contains the state of all analog joystick
 * buttons.
 */

typedef uint8_t ajoy_buttonset_t;

/* A reference to this structure is provided with the AJOYIOC_POLLEVENTS
 * IOCTL command and describes the conditions under which the client would
 * like to receive notification.
 */

struct ajoy_pollevents_s
{
  ajoy_buttonset_t ap_press;   /* Set of button depressions to wake up the poll */
  ajoy_buttonset_t ap_release; /* Set of button releases to wake up the poll */
};

/* A reference to this structure is provided with the AJOYIOC_REGISTER IOCTL
 * command and describes the conditions under which the client would like
 * to receive notification.
 */

struct ajoy_notify_s
{
  ajoy_buttonset_t an_press;   /* Set of button depressions to be notified */
  ajoy_buttonset_t an_release; /* Set of button releases to be notified */
  struct sigevent  an_event;   /* Describe the way a task is to be notified */
};

/* This structure is returned by read() and provides the sample state of the
 * analog joystick.
 *
 * NOTE: that this structure is equivalent to the struct mouse_report_s
 * structure (with no wheel) defined in include/nuttx/input/mouse.h and can
 * be used interchangeably in certain contexts.
 */

struct ajoy_sample_s
{
  ajoy_buttonset_t as_buttons; /* State of all buttons */
                               /* Possibly padded with 1 byte here */
  int16_t          as_x;       /* X/horizontal position */
  int16_t          as_y;       /* Y/vertical position */
};

/* This is the type of the analog joystick interrupt handler used with
 * the struct ajoy_lowerhalf_s enable() method.
 */

struct ajoy_lowerhalf_s;
typedef CODE void (*ajoy_handler_t)
  (FAR const struct ajoy_lowerhalf_s *lower, FAR void *arg);

/* The analog joystick driver is a two-part driver:
 *
 * 1) A common upper half driver that provides the common user interface to
 *    the joystick,
 * 2) Platform-specific lower half drivers that provide the interface
 *    between the common upper half and the platform analog and discrete
 *    inputs.
 *
 * This structure defines the interface between an instance of the lower
 * half driver and the common upper half driver.  Such an instance is
 * passed to the upper half driver when the driver is initialized, binding
 * the upper and lower halves into one driver.
 */

struct ajoy_lowerhalf_s
{
  /* Return the set of buttons supported on the analog joystick device */

  CODE ajoy_buttonset_t (*al_supported)
                        (FAR const struct ajoy_lowerhalf_s *lower);

  /* Return the current state of all analog joystick position
   * and button data
   */

  CODE int (*al_sample)(FAR const struct ajoy_lowerhalf_s *lower,
                        FAR struct ajoy_sample_s *sample);

  /* Return the current state of button data (only) */

  CODE ajoy_buttonset_t (*al_buttons)
                        (FAR const struct ajoy_lowerhalf_s *lower);

  /* Enable interrupts on the selected set of joystick buttons.
   * An empty set will disable all interrupts.
   */

  CODE void (*al_enable)(FAR const struct ajoy_lowerhalf_s *lower,
                         ajoy_buttonset_t press,
                         ajoy_buttonset_t release,
                         ajoy_handler_t handler,
                         FAR void *arg);
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
 * Name: ajoy_register
 *
 * Description:
 *   Bind the lower half analog joystick driver to an instance of the upper
 *   half analog joystick driver and register the composite character
 *   driver as the specified device.
 *
 * Input Parameters:
 *   devname - The name of the analog joystick device to be registered.
 *     This should be a string of the form "/dev/ajoyN" where N is the
 *     minor device number.
 *   lower - An instance of the platform-specific analog joystick lower
 *     half driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int ajoy_register(FAR const char *devname,
                  FAR const struct ajoy_lowerhalf_s *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_INPUT_AJOYSTICK_H */
