/****************************************************************************
 * include/nuttx/leds/userled.h
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

#ifndef __INCLUDE_NUTTX_LEDS_USERLED_H
#define __INCLUDE_NUTTX_LEDS_USERLED_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ioctl commands */

/* Command:     ULEDIOC_SUPPORTED
 * Description: Report the set of LEDs supported by the hardware;
 * Argument:    A pointer to writeable userled_set_t value in which to
 *              return the set of supported LEDs.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define ULEDIOC_SUPPORTED  _ULEDIOC(0x0001)

/* Command:     ULEDIOC_SETLED
 * Description: Set the state of one LED.
 * Argument:    A read-only pointer to an instance of struct userled_s
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define ULEDIOC_SETLED     _ULEDIOC(0x0002)

/* Command:     ULEDIOC_SETALL
 * Description: Set the state of all LEDs.
 * Argument:    A value of type userled_set_t cast to unsigned long
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define ULEDIOC_SETALL     _ULEDIOC(0x0003)

/* Command:     ULEDIOC_GETALL
 * Description: Get the state of one LED.
 * Argument:    A write-able pointer to a userled_set_t memory location in
 *              which to return the LED state.
 * Return:      Zero (OK) on success.  Minus one will be returned on failure
 *              with the errno value set appropriately.
 */

#define ULEDIOC_GETALL     _ULEDIOC(0x0004)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This type is a bit set that contains the state of all LEDs as defined
 * in arch/board/board.h.  This is the value that is returned when reading
 * from or writing to the LED driver.
 */

typedef uint32_t userled_set_t;

/* A reference to this structure is provided with the ULEDIOC_SETLED IOCTL
 * command and describes the LED to be set and the new value of the LED.
 * The encoding of LEDs is provided in the board-specific board.h header
 * file.
 */

struct userled_s
{
  uint8_t  ul_led;          /* Identifies the LED */
  bool     ul_on;           /* The LED state.  true: ON; false: OFF */
};

/* The user LED driver is a two-part driver:
 *
 * 1) A common upper half driver that provides the common user interface to
 *    the LEDs,
 * 2) Platform-specific lower half drivers that provide the interface
 *    between the common upper half and the platform discrete LED outputs.
 *
 * This structure defines the interface between an instance of the lower
 * half driver and the common upper half driver.  Such an instance is
 * passed to the upper half driver when the driver is initialized, binding
 * the upper and lower halves into one driver.
 */

struct userled_lowerhalf_s
{
  /* Return the set of LEDs supported by the board */

  CODE userled_set_t
  (*ll_supported)(FAR const struct userled_lowerhalf_s *lower);

  /* Set the current state of one LED */

  CODE void (*ll_led)(FAR const struct userled_lowerhalf_s *lower,
                      int led, bool ledon);

  /* Set the state of all LEDs */

  CODE void (*ll_ledset)(FAR const struct userled_lowerhalf_s *lower,
                         userled_set_t ledset);
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
 * Name: userled_register
 *
 * Description:
 *   Bind the lower half LED driver to an instance of the upper half
 *   LED driver and register the composite character driver as the
 *   specified device.
 *
 * Input Parameters:
 *   devname - The name of the LED device to be registered.
 *     This should be a string of the form "/dev/ledN" where N is the
 *     minor device number.
 *   lower - An instance of the platform-specific LED lower half driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int userled_register(FAR const char *devname,
                     FAR const struct userled_lowerhalf_s *lower);

/****************************************************************************
 * Name: userled_lower_initialize
 *
 * Description:
 *   Initialize the generic LED lower half driver, bind it and register
 *   it with the upper half LED driver as devname.
 *
 ****************************************************************************/

#ifdef CONFIG_USERLED_LOWER
int userled_lower_initialize(FAR const char *devname);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LEDS_USERLED_H */
