/****************************************************************************
 * include/nuttx/input/nt38350.h
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

#ifndef __INCLUDE_NUTTX_INPUT_NT38350_H
#define __INCLUDE_NUTTX_INPUT_NT38350_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/ioexpander/ioexpander.h>

#ifdef CONFIG_INPUT_NT38350

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Specific IOCTL commands for NT38350 */

#define TSIOC_GETNVTDIFF     _TSIOC(0x0006)  /* arg: Pointer to get struct nvt_diff_s */
#define TSIOC_SELFTEST       _TSIOC(0x0007)  /* arg: Pointer to int self test value */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the NT38350
 * driver.  This structure provides information about the configuration
 * of the NT38350 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writeable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

struct nt38350_config_s
{
  /* Device characterization */

  uint8_t                     address;          /* 7-bit I2C address (only bits 0-6 used) */
  uint16_t                    rxplate;          /* Calibrated X plate resistance */
  uint32_t                    frequency;        /* Default I2C frequency */
  int                         gpio_irq_pin;     /* IRQ pin */
  int                         gpio_rst_pin;     /* Reset pin */
  ioe_callback_t              handler;          /* The nt38350 interrupt handler */
  FAR void                    *arg;             /* Interrupt handler argument */
  FAR struct ioexpander_dev_s *ioe_dev;         /* Ioexpander device. */
  FAR struct i2c_master_s     *i2c;

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the NT38350 driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.
   *
   * attach  - Attach an NT38350 interrupt handler to a GPIO interrupt
   * enable  - Enable or disable a GPIO interrupt
   * clear   - Acknowledge/clear any pending GPIO interrupt
   * wakeup  - Issue WAKE interrupt to NT38350 to change the NT38350 from
   *           Hibernate to Active mode.
   * nreset  - Control the chip reset pin (active low)

   */

#ifndef CONFIG_NT38350_POLLMODE
  int  (*attach)(FAR struct nt38350_config_s *config,
                 ioe_callback_t isr, FAR void *arg);
  void (*enable)(FAR const struct nt38350_config_s *config, bool enable);
  void (*clear)(FAR const struct nt38350_config_s *config);
  int  (*detach)(FAR struct nt38350_config_s *config,
                 ioe_callback_t isr);
#endif
  void (*wakeup)(FAR const struct nt38350_config_s *config);
  void (*nreset)(FAR const struct nt38350_config_s *config,
                 bool state);
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
 * Name: nt38350_register
 *
 * Description:
 *   Configure the nt38350 to use the provided I2C device instance.  This
 *   will register the driver as /dev/inputN where N is the minor device
 *   number
 *
 * Input Parameters:
 *   config  - Persistent board configuration data
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int nt38350_register(FAR struct nt38350_config_s *config,
                     FAR const char *devname);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_INPUT_NT38350 */
#endif /* __INCLUDE_NUTTX_INPUT_NT38350_H */
