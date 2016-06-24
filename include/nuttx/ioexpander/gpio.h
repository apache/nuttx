/********************************************************************************************
 * include/nuttx/ioexpander/gpio.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

#ifndef __INCLUDE_NUTTX_IOEXPANDER_GPIO_H
#define __INCLUDE_NUTTX_IOEXPANDER_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Command:     GPIO_WRITE
 * Description: Set the value of an output GPIO
 * Argument:    0=output a low value; 1=outut a high value
 *
 * Command:     GPIO_READ
 * Description: Read the value of an input or output GPIO
 * Argument:    A pointer to an integer value to receive the result:
 *              0=low value; 1=high value.
 */

#define GPIO_WRITE _GPIOC(1)
#define GPIO_READ  _GPIOC(2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Common interface definition.  Must be cast-compatible with struct
 * gpio_input_dev_s and struct gpio_output_dev_s
 */

struct gpio_common_dev_s
{
  bool gp_output;
  uint8_t gp_unused[3];
};

/* The interface to a GPIO input pin */

struct gpio_input_dev_s
{
  bool gpin_output;
  uint8_t gpin_unused[3];
  CODE int (*gpin_read)(FAR struct gpio_input_dev_s *dev);
};

/* The interface to a GPIO input pin */

struct gpio_output_dev_s
{
  bool gpout_output;
  uint8_t gpout_unused[3];
  CODE int (*gpout_read)(FAR struct gpio_output_dev_s *dev);
  CODE int (*gpout_write)(FAR struct gpio_output_dev_s *dev, int value);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: gpio_input_register
 *
 * Description:
 *   Register GPIO input pin device driver.
 *
 ****************************************************************************/

int gpio_input_register(FAR struct gpio_input_dev_s *dev, int minor);

/****************************************************************************
 * Name: gpio_output_register
 *
 * Description:
 *   Register GPIO output pin device driver.
 *
 ****************************************************************************/

int gpio_output_register(FAR struct gpio_output_dev_s *dev, int minor);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_IOEXPANDER_GPIO_H */
