/****************************************************************************
 * include/nuttx/input/mpr121.h
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

#ifndef __INCLUDE_NUTTX_INPUT_MPR121_H
#define __INCLUDE_NUTTX_INPUT_MPR121_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/input/keyboard.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of keys used by MPR121 */

#define MPR121_NUMKEY        12

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef uint32_t mpr121_pin_t;

/* Keypad matrix configuration structure passed to mpr121_register() */

struct mpr121_config_s
{
  CODE int (*irq_attach)(FAR const struct mpr121_config_s *state,
                         xcpt_t isr, FAR void *arg);
  FAR struct i2c_master_s *i2c_dev;
  uint8_t i2c_addr;

  /* Keymap: keycode[MPR121_NUMKEY] to map the keys of MPR121 keypad */

  FAR const uint32_t *keymap;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: mpr121_register
 *
 * Description:
 *   Configure and register a keypad matrix device.  This will create the
 *   /dev/keypadN device node and enable keyboard scanning.
 *
 * Input Parameters:
 *   config - The keyboard matrix configuration.  This structure is not
 *            copied; it must persist for the lifetime of the driver.
 *   devpath - The device path for the /dev/keypadN device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int mpr121_register(FAR const struct mpr121_config_s *config,
                    FAR const char *devpath);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_INPUT_MPR121_H */
