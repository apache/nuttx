/****************************************************************************
 * arch/arm64/src/imx9/imx9_scmi.h
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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

#ifndef __ARCH_ARM_SRC_IMX9_IMX9_SCMI_H
#define __ARCH_ARM_SRC_IMX9_IMX9_SCMI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imx95/imx95_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SCMI_PLATFORM_A2P                     0 /* Agent -> Platform */
#define SCMI_PLATFORM_NOTIFY                  1 /* Platform -> Agent */
#define SCMI_PLATFORM_PRIORITY                2

#define SCMI_CLOCK_RATE_MASK                  0xFFFFFFFFU

/* SCMI clock round options */

#define SCMI_CLOCK_ROUND_DOWN                 0U /* Round down */
#define SCMI_CLOCK_ROUND_UP                   1U /* Round up */
#define SCMI_CLOCK_ROUND_AUTO                 2U /* Round to nearest */

/* SCMI clock rate flags */

/* Round up/down */

#define SCMI_CLOCK_RATE_FLAGS_ROUND(x)        (((x) & 0x3U) << 2U)

/* Ignore delayed response */

#define SCMI_CLOCK_RATE_FLAGS_NO_RESP(x)      (((x) & 0x1U) << 1U)

/* Async flag */

#define SCMI_CLOCK_RATE_FLAGS_ASYNC(x)        (((x) & 0x1U) << 0U)

/* SCMI clock config attributes */

/* OEM specified config type */

#define SCMI_CLOCK_CONFIG_SET_OEM(x)          (((x) & 0xFFU) << 16U)

/* Enable/Disable */

#define SCMI_CLOCK_CONFIG_SET_ENABLE(x)       (((x) & 0x3U) << 0U)

/* SCMI pin control types */

#define SCMI_PINCTRL_SEL_PIN                  0U
#define SCMI_PINCTRL_SEL_GROUP                1U
#define SCMI_PINCTRL_TYPE_MUX                 192U /* Mux type */
#define SCMI_PINCTRL_TYPE_CONFIG              193U /* Config type */
#define SCMI_PINCTRL_TYPE_DAISY_ID            194U /* Daisy ID type */
#define SCMI_PINCTRL_TYPE_DAISY_CFG           195U /* Daisy config type */
#define SCMI_PINCTRL_SET_ATTR_NUM_CONFIGS(x)  (((x) & 0xFFU) << 2U)
#define SCMI_PINCTRL_SET_ATTR_SELECTOR(x)     (((x) & 0x3U) << 0U)

/* Pinctrl */

#define SCMI_PLATFORM_PINCTRL_MUX_MODE_MASK   (0x7U)
#define SCMI_PLATFORM_PINCTRL_MUX_MODE_SHIFT  (0U)
#define SCMI_PLATFORM_PINCTRL_MUX_MODE(x) \
    (((uint32_t)(((uint32_t)(x)) << SCMI_PLATFORM_PINCTRL_MUX_MODE_SHIFT)) \
     & SCMI_PLATFORM_PINCTRL_MUX_MODE_MASK)

#define SCMI_PLATFORM_PINCTRL_SION_MASK       (0x10)
#define SCMI_PLATFORM_PINCTRL_SION_SHIFT      (4U)
#define SCMI_PLATFORM_PINCTRL_SION(x) \
    (((uint32_t)(((uint32_t)(x)) << SCMI_PLATFORM_PINCTRL_SION_SHIFT)) \
     & SCMI_PLATFORM_PINCTRL_SION_MASK)

#define SCMI_PLATFORM_PINCTRL_BASE            IMX9_IOMUXC1_BASE
#define SCMI_PLATFORM_PINCTRL_MUXREG_OFF      (SCMI_PLATFORM_PINCTRL_BASE)
#define SCMI_PLATFORM_PINCTRL_CFGREG_OFF      (SCMI_PLATFORM_PINCTRL_BASE + 0x204)
#define SCMI_PLATFORM_PINCTRL_DAISYREG_OFF    (SCMI_PLATFORM_PINCTRL_BASE + 0x408)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct
{
  uint32_t channel;    /* Channel id: SCMI_A2P, SCMI_NOTIRY, SCMI_P2A, */
  uint32_t rateu;
  uint32_t ratel;
  uint32_t clk_id;     /* Clock device id */
  uint32_t pclk_id;    /* Parent clock device id */
  uint32_t div;        /* Clock divider */
  uint32_t attributes; /* Clock attributes */
  uint32_t oem_config_val;
  uint32_t flags;
} scmi_clock_t;

typedef struct
{
  uint32_t channel;
  uint32_t mux_register;
  uint32_t mux_mode;
  uint32_t input_register;
  uint32_t input_daisy;
  uint32_t config_register;
  uint32_t config_value;
  uint32_t input_on_field;
} scmi_pinctrl_t;

/* SCMI clock rate */

typedef struct
{
  uint32_t lower; /* Lower 32 bits of the physical rate in Hz */
  uint32_t upper; /* Upper 32 bits of the physical rate in Hz */
} scmi_clock_rate_t;

/* SCMI pin control config */

typedef struct
{
  uint32_t type;  /* The type of config */
  uint32_t value; /* The configuration value */
} scmi_pin_config_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_scmi_initialize
 *
 * Description:
 *   Initialize the scmi
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imx9_scmi_initialize(void);

/****************************************************************************
 * Name: imx9_scmi_get_clock_parent
 *
 * Description:
 *   Use scmi get clockid's parentid
 *
 * Input Parameters:
 *   channel        - Channel id
 *   clockid        - Identifier for the clock
 *   parentid       - Identifier for the parent clock id
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_scmi_get_clock_parent(uint32_t channel, uint32_t clockid,
                               uint32_t *parentid);

/****************************************************************************
 * Name: imx9_scmi_set_clock_parent
 *
 * Description:
 *   Use scmi set clockid's parentid
 *
 * Input Parameters:
 *   channel        - Channel id
 *   clockid        - Identifier for the clock
 *   parentid       - Identifier for the parent clock id
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_scmi_set_clock_parent(uint32_t channel, uint32_t clockid,
                               uint32_t parentid);

/****************************************************************************
 * Name: imx9_scmi_get_clock_rate
 *
 * Description:
 *   Use scmi get clock rate.
 *
 * Input Parameters:
 *   channel - Channel id
 *   clockid - Identifier for the clock
 *   rate    - Returned rate
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_scmi_get_clock_rate(uint32_t channel, uint32_t clockid,
                             scmi_clock_rate_t *rate);

/****************************************************************************
 * Name: imx9_scmi_set_clock_rate
 *
 * Description:
 *   Use scmi set clock rate.
 *
 * Input Parameters:
 *   channel - Channel id
 *   clockid - Identifier for the clock
 *   flags   - Rate flags
 *   rate    - The rate to be set
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_scmi_set_clock_rate(uint32_t channel, uint32_t clockid,
                             uint32_t flags, scmi_clock_rate_t rate);

/****************************************************************************
 * Name: imx9_scmi_set_clock_config
 *
 * Description:
 *   Use scmi set clock config
 *
 * Input Parameters:
 *   channel        - Channel id
 *   clockid        - Identifier for the clock
 *   attributes     - The attributes to be set
 *   oem_config_val - The oem config val
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_scmi_set_clock_config(uint32_t channel, uint32_t clockid,
                               uint32_t attributes,
                               uint32_t oem_config_val);

/****************************************************************************
 * Name: imx9_scmi_set_pinctrl_config
 *
 * Description:
 *   Use scmi set pinctrl config.
 *
 * Input Parameters:
 *   channel    - Channel id
 *   identifier - Identifier for the pin or group
 *   attributes - Pin control attributes
 *   configs    - Array of configurations: sorted in numerically
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_scmi_set_pinctrl_config(uint32_t channel, uint32_t identifier,
                                 uint32_t attributes,
                                 const scmi_pin_config_t *configs);

/****************************************************************************
 * Name: imx9_scmi_get_power_state
 *
 * Description:
 *   Use scmi set powerdomain state.
 *
 * Input Parameters:
 *   channel   - Channel id
 *   domain    - The power domain id
 *   state     - The returnrd power state
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_scmi_get_power_state(uint32_t channel, uint32_t domain,
                              uint32_t *state);

/****************************************************************************
 * Name: imx9_scmi_set_power_state
 *
 * Description:
 *   Use scmi set powerdomain state.
 *
 * Input Parameters:
 *   channel   - Channel id
 *   domain    - The power domain id
 *   state     - The power state
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_scmi_set_power_state(uint32_t channel, uint32_t domain,
                              uint32_t state);

#endif /* __ARCH_ARM_SRC_IMX9_IMX9_SCMI_H */
