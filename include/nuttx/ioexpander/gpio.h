/****************************************************************************
 * include/nuttx/ioexpander/gpio.h
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
 * Argument:    0=output a low value; 1=output a high value
 *
 * Command:     GPIOC_READ
 * Description: Read the value of an input or output GPIO
 * Argument:    A pointer to a bool value to receive the result:
 *              false=low value; true=high value.
 *
 * Command:     GPIOC_PINTYPE
 * Description: Return the GPIO pin type.
 * Argument:    A pointer to an instance of type enum gpio_pintype_e
 *
 * Command:     GPIOC_REGISTER
 * Description: Register to receive a signal whenever there is an interrupt
 *              received on an input gpio pin.  This feature, of course,
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
  GPIO_INPUT_PIN = 0, /* float */
  GPIO_INPUT_PIN_PULLUP,
  GPIO_INPUT_PIN_PULLDOWN,
  GPIO_OUTPUT_PIN, /* push-pull */
  GPIO_OUTPUT_PIN_OPENDRAIN,
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
 *   - go_read.  Required for all pin types.
 *   - go_write.  Required only for the GPIO_OUTPUT_PIN pin type.  Unused
 *     for other pin types, may be NULL.
 *   - go_attach and go_enable.  Required only for the GPIO_INTERRUPT_PIN pin
 *     type.  Unused for other pin types, may be NULL.
 *   - go_setpintype.  Required for all pin types.
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
 *   Register GPIO pin device driver at /dev/gpioN, where N is the provided
 *   minor number.
 *
 ****************************************************************************/

int gpio_pin_register(FAR struct gpio_dev_s *dev, int minor);

/****************************************************************************
 * Name: gpio_pin_unregister
 *
 * Description:
 *   Unregister GPIO pin device driver at /dev/gpioN, where N is the provided
 *   minor number.
 *
 ****************************************************************************/

int gpio_pin_unregister(FAR struct gpio_dev_s *dev, int minor);

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
