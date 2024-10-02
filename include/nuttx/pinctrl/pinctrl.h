/****************************************************************************
 * include/nuttx/pinctrl/pinctrl.h
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

#ifndef __INCLUDE_NUTTX_PINCTRL_PINCTRL_H
#define __INCLUDE_NUTTX_PINCTRL_PINCTRL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Command:     PINCTRLC_SETFUNCTION
 * Description: Set the mux function of the pinctrl pin
 * Argument:    A pointer to an instance of struct pinctrl_iotrans_s
 *
 * Command:     PINCTRLC_SETSTRENGTH
 * Description: Set the driver strength of the pinctrl pin
 * Argument:    A pointer to an instance of struct pinctrl_iotrans_s
 *
 * Command:     PINCTRLC_SETDRIVER
 * Description: Set the driver type of the pinctrl pin
 * Argument:    A pointer to an instance of struct pinctrl_iotrans_s
 *
 * Command:     PINCTRLC_SETSLEWRATE
 * Description: Set slewrate of the pinctrl pin
 * Argument:    A pointer to an instance of struct pinctrl_iotrans_s
 *
 * Command:     PINCTRLC_SELECTGPIO
 * Description: Select gpio function of pinctrl pin
 * Argument:    The uint32_t pinctrl number
 *
 */

#define PINCTRLC_SETFUNCTION _PINCTRLIOC(1)
#define PINCTRLC_SETSTRENGTH _PINCTRLIOC(2)
#define PINCTRLC_SETDRIVER   _PINCTRLIOC(3)
#define PINCTRLC_SETSLEWRATE _PINCTRLIOC(4)
#define PINCTRLC_SELECTGPIO  _PINCTRLIOC(5)

/* Access macros ************************************************************/

/****************************************************************************
 * Name: PINCTRL_SETFUNCTION
 *
 * Description:
 *   Set the mux function of the pinctrl pin.
 *
 * Input Parameters:
 *   dev      - Device-specific state data.
 *   pin      - the pinctrl controller number.
 *   function - the pinctrl pin function number.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#define PINCTRL_SETFUNCTION(dev, pin, function) ((dev)->ops->set_function(dev, pin, function))

/****************************************************************************
 * Name: PINCTRL_SETSTRENGTH
 *
 * Description:
 *   Set the driver strength of the pinctrl pin
 *
 * Input Parameters:
 *   dev      - Device-specific state data.
 *   pin      - the pinctrl controller number.
 *   strength - the pinctrl pin driver strength number.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#define PINCTRL_SETSTRENGTH(dev, pin, strength) ((dev)->ops->set_strength(dev, pin, strength))

/****************************************************************************
 * Name: PINCTRL_SETDRIVER
 *
 * Description:
 *   Set the driver type of the pinctrl pin
 *
 * Input Parameters:
 *   dev    - Device-specific state data.
 *   pin    - the pinctrl controller number.
 *   driver - the pinctrl_drivertype_e type.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#define PINCTRL_SETDRIVER(dev, pin, driver) ((dev)->ops->set_driver(dev, pin, driver))

/****************************************************************************
 * Name: PINCTRL_SETSLEWRATE
 *
 * Description:
 *   Set the slewrate of the pinctrl pin
 *
 * Input Parameters:
 *   dev      - Device-specific state data.
 *   pin      - the pinctrl controller number.
 *   slewrate - the slewsrate state.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#define PINCTRL_SETSLEWRATE(dev, pin, slewrate) ((dev)->ops->set_slewrate(dev, pin, slewrate))

/****************************************************************************
 * Name: PINCTRL_SELECTGPIO
 *
 * Description:
 *   Select gpio function the pinctrl pin
 *
 * Input Parameters:
 *   dev   - Device-specific state data.
 *   pin   - the pinctrl controller number.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#define PINCTRL_SELECTGPIO(dev, pin) ((dev)->ops->select_gpio(dev, pin))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Identifies the type of the pinctrl driver type */

enum pinctrl_drivertype_e
{
  BIAS_DISABLE = 0,
  BIAS_PULLUP,
  BIAS_PULLDOWN,
  BIAS_STRONG_PULLDOWN,
  BIAS_NDRIVERTYPES
};

/****************************************************************************
 * This describes pinctrl ioctl structure in pinctrl command operation.
 * the follow command use this structure:
 * PINCTRLC_SETFUNCTION, PINCTRLC_SETSTRENGTH,
 * PINCTRLC_SETDRIVER, PINCTRLC_SETSLEWRATE.
 ****************************************************************************/

struct pinctrl_param_s
{
  uint32_t pin;
  union
  {
    uint32_t                  function;
    uint32_t                  strength;
    enum pinctrl_drivertype_e type;
    uint32_t                  slewrate;
  } para;
};

/* pinctrl interface methods */

struct pinctrl_dev_s;
struct pinctrl_ops_s
{
  int (*set_function)(FAR struct pinctrl_dev_s *dev, uint32_t pin,
                      uint32_t function);
  int (*set_strength)(FAR struct pinctrl_dev_s *dev,  uint32_t pin,
                      uint32_t strength);
  int (*set_driver)(FAR struct pinctrl_dev_s *dev, uint32_t pin,
                    enum pinctrl_drivertype_e type);
  int (*set_slewrate)(FAR struct pinctrl_dev_s *dev, uint32_t pin,
                      uint32_t slewrate);
  int (*select_gpio)(FAR struct pinctrl_dev_s *dev, uint32_t pin);
};

struct pinctrl_dev_s
{
  /* "Lower half" operations provided by the pinctrl lower half */

  FAR const struct pinctrl_ops_s *ops;

  /* Internal storage used by the pinctrl may (internal to the pinctrl
   * implementation).
   */
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
 * Name: pinctrl_register
 *
 * Description:
 *   Register PINCTRL device driver.
 *
 ****************************************************************************/

int pinctrl_register(FAR struct pinctrl_dev_s *dev, int minor);

/****************************************************************************
 * Name: pinctrl_unregister
 *
 * Description:
 *   Unregister PINCTRL device driver.
 *
 ****************************************************************************/

void pinctrl_unregister(FAR struct pinctrl_dev_s *dev, int minor);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_PINCTRL_PINCTRL_H */
