/****************************************************************************
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_IOEXPANDER_GPIO_H
#define __INCLUDE_NUTTX_IOEXPANDER_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_DEV_GPIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Command:     GPIOC_WRITE
 * Description: Set the value of an output GPIO
 * Argument:    T0=output a low value; 1=outut a high value
 *
 * Command:     GPIOC_READ
 * Description: Read the value of an input or output GPIO
 * Argument:    A pointer to an bool value to receive the result:
 *              false=low value; true=high value.
 *
 * Command:     GPIOC_PINTYPE
 * Description: Return the GPIO pin type.
 * Argument:    A pointer to an instance of type enum gpio_pintype_e
 *
 * Command:     GPIOC_REGISTER
 * Description: Register to receive a signal whenever there an interrupt
 *              is received on an input gpio pin.  This feature, of course,
 *              depends upon interrupt GPIO support from the platform.
 * Argument:    The number of signal to be generated when the interrupt
 *              occurs.
 *
 * Command:     GPIOC_UNREGISTER
 * Description: Stop receiving signals for pin interrupts.
 * Argument:    None.
 *
 * Command:     GPIOC_SETPINTYPE
 * Description: Set the GPIO pin type.
 * Argument:    The enum gpio_pintype_e type.
 *
 */

#define GPIOC_WRITE      _GPIOC(1)
#define GPIOC_READ       _GPIOC(2)
#define GPIOC_PINTYPE    _GPIOC(3)
#define GPIOC_REGISTER   _GPIOC(4)
#define GPIOC_UNREGISTER _GPIOC(5)
#define GPIOC_SETPINTYPE _GPIOC(6)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Identifies the type of the GPIO pin */

enum gpio_pintype_e
{
  GPIO_INPUT_PIN = 0,
  GPIO_OUTPUT_PIN,
  GPIO_INTERRUPT_PIN,
  GPIO_INTERRUPT_HIGH_PIN,
  GPIO_INTERRUPT_LOW_PIN,
  GPIO_INTERRUPT_RISING_PIN,
  GPIO_INTERRUPT_FALLING_PIN,
  GPIO_INTERRUPT_BOTH_PIN,
  GPIO_NPINTYPES
};

/* Interrupt callback */

struct gpio_dev_s;
typedef CODE int (*pin_interrupt_t)(FAR struct gpio_dev_s *dev, uint8_t pin);

/* Pin interface vtable definition.  Instances of this vtable are read-only
 * and may reside in FLASH.
 *
 *   - go_read.  Required for all all pin types.
 *   - go_write.  Required only for the GPIO_OUTPUT_PIN pin type.  Unused
 *     for other pin types may be NULL.
 *   - go_attach and gp_eanble.  Required only the GPIO_INTERRUPT_PIN pin
 *     type.  Unused for other pin types may be NULL.
 *   - go_setpinytype.  Required for all all pin types.
 */

struct gpio_dev_s;
struct gpio_operations_s
{
  /* Interface methods */

  CODE int (*go_read)(FAR struct gpio_dev_s *dev, FAR bool *value);
  CODE int (*go_write)(FAR struct gpio_dev_s *dev, bool value);
  CODE int (*go_attach)(FAR struct gpio_dev_s *dev,
                        pin_interrupt_t callback);
  CODE int (*go_enable)(FAR struct gpio_dev_s *dev, bool enable);
  CODE int (*go_setpintype)(FAR struct gpio_dev_s *dev,
                            enum gpio_pintype_e pintype);
};

 /* Signal information */

struct gpio_signal_s
{
  struct sigevent gp_event;
  struct sigwork_s gp_work;
  pid_t gp_pid;        /* The task to be signaled */
};

/* Pin interface definition.  Must lie in writable memory. */

struct gpio_dev_s
{
  /* Information provided from the lower half driver to the upper half
   * driver when gpio_pin_register() is called.
   */

  uint8_t gp_pintype;  /* See enum gpio_pintype_e */;

  /* Writable storage used by the upper half driver */

  struct gpio_signal_s gp_signals[CONFIG_DEV_GPIO_NSIGNALS];

  /* Read-only pointer to GPIO device operations (also provided by the
   * lower half driver).
   */

  FAR const struct gpio_operations_s *gp_ops;

  /* Device specific, lower-half information may follow. */
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
 * Name: gpio_pin_register
 *
 * Description:
 *   Register GPIO pin device driver.
 *
 *   - Input pin types will be registered at /dev/gpinN
 *   - Output pin types will be registered at /dev/gpoutN
 *   - Interrupt pin types will be registered at /dev/gpintN
 *
 *   Where N is the provided minor number in the range of 0-99.
 *
 *
 ****************************************************************************/

int gpio_pin_register(FAR struct gpio_dev_s *dev, int minor);

/****************************************************************************
 * Name: gpio_pin_unregister
 *
 * Description:
 *   Unregister GPIO pin device driver.
 *
 *   - Input pin types will be registered at /dev/gpinN
 *   - Output pin types will be registered at /dev/gpoutN
 *   - Interrupt pin types will be registered at /dev/gpintN
 *
 *   Where N is the provided minor number in the range of 0-99.
 *
 *
 ****************************************************************************/

void gpio_pin_unregister(FAR struct gpio_dev_s *dev, int minor);

/****************************************************************************
 * Name: gpio_lower_half
 *
 * Description:
 *   Create a GPIO pin device driver instance for an I/O expander pin.
 *   The I/O expander pin must have already been configured by the caller
 *   for the particular pintype.
 *
 * Input Parameters:
 *   ioe     - An instance of the I/O expander interface
 *   pin     - The I/O expander pin number for the driver
 *   pintype - See enum gpio_pintype_e
 *   minor   - The minor device number to use when registering the device
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_GPIO_LOWER_HALF
struct ioexpander_dev_s;
int gpio_lower_half(FAR struct ioexpander_dev_s *ioe, unsigned int pin,
                    enum gpio_pintype_e pintype, int minor);
#endif

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DEV_GPIO */
#endif /* __INCLUDE_NUTTX_IOEXPANDER_GPIO_H */
