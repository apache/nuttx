/****************************************************************************
 * include/nuttx/input/sbutton.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* The single button driver exports a standard character driver interface. By
 * convention, the driver is exposed as /dev/kbd[n] and works like a common
 * keyboard device, but generates only INPUT_TAB and INPUT_ENTER events.
 */

#ifndef __INCLUDE_NUTTX_INPUT_SBUTTON_H
#define __INCLUDE_NUTTX_INPUT_SBUTTON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include <stdbool.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the
 * single button keyboard driver.  This structure provides information
 * about the configuration and provides some board-specific hooks.
 */

struct sbutton_config_s
{
  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the Q10 Keyboard driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.
   *
   * attach  - Attach the Q10 kbd interrupt handler to the GPIO interrupt
   * enable  - Enable or disable the GPIO interrupt
   * clear   - Acknowledge/clear any pending GPIO interrupt
   */

  int  (*attach)(FAR const struct sbutton_config_s *config, xcpt_t isr,
                 FAR void *arg);
  void (*enable)(FAR const struct sbutton_config_s *config, bool enable);
  void (*clear)(FAR const struct sbutton_config_s *config);
  bool (*status)(void);
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
 * Name: spq10kbd_register
 *
 * Description:
 *   Configure the single button driver to use the provided instance.
 *   This will register the driver as /dev/kbdN where N is the
 *   minor device number.
 *
 * Input Parameters:
 *   config      - Persistent board configuration data
 *   kbdminor    - The keyboard input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int sbutton_register(FAR const struct sbutton_config_s *config,
                     char kbdminor);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_INPUT_SBUTTON_H */
