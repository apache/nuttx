/****************************************************************************
 * arch/arm/src/imx9/imx9_scmi.h
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SM_PLATFORM_A2P      0 /* SCMI Agent -> SCMI Platform */
#define SM_PLATFORM_NOTIFY   1 /* SCMI Platform -> SCMI Agent */
#define SM_PLATFORM_PRIORITY 2

#define SM_CLOCK_RATE_MASK 0xFFFFFFFFU

/* SCMI clock round options */
#define SCMI_CLOCK_ROUND_DOWN  0U /* Round down */
#define SCMI_CLOCK_ROUND_UP    1U /* Round up */
#define SCMI_CLOCK_ROUND_AUTO  2U /* Round to nearest */

/* SCMI clock rate flags */

#define SCMI_CLOCK_RATE_FLAGS_ROUND(x)    (((x) & 0x3U) << 2U) /* Round up/down */
#define SCMI_CLOCK_RATE_FLAGS_NO_RESP(x)  (((x) & 0x1U) << 1U) /* Ignore delayed response */
#define SCMI_CLOCK_RATE_FLAGS_ASYNC(x)    (((x) & 0x1U) << 0U) /* Async flag */

/* SCMI clock config attributes */

#define SCMI_CLOCK_CONFIG_SET_OEM(x)     (((x) & 0xFFU) << 16U) /* OEM specified config type */
#define SCMI_CLOCK_CONFIG_SET_ENABLE(x)  (((x) & 0x3U) << 0U)   /* Enable/Disable */

/* SCMI pin control types */

#define SCMI_PINCTRL_SEL_PIN         0U
#define SCMI_PINCTRL_SEL_GROUP       1U
#define SCMI_PINCTRL_TYPE_MUX        192U /* Mux type */
#define SCMI_PINCTRL_TYPE_CONFIG     193U /* Config type */
#define SCMI_PINCTRL_TYPE_DAISY_ID   194U /* Daisy ID type */
#define SCMI_PINCTRL_TYPE_DAISY_CFG  195U /* Daisy config type */
#define SCMI_PINCTRL_SET_ATTR_NUM_CONFIGS(x)  (((x) & 0xFFU) << 2U)
#define SCMI_PINCTRL_SET_ATTR_SELECTOR(x)     (((x) & 0x3U) << 0U)

/* Pinctrl */
#define SM_PLATFORM_PINCTRL_MUX_MODE_MASK  (0x7U)
#define SM_PLATFORM_PINCTRL_MUX_MODE_SHIFT (0U)
#define SM_PLATFORM_PINCTRL_MUX_MODE(x) \
    (((uint32_t)(((uint32_t)(x)) << SM_PLATFORM_PINCTRL_MUX_MODE_SHIFT)) & SM_PLATFORM_PINCTRL_MUX_MODE_MASK)

#define SM_PLATFORM_PINCTRL_SION_MASK  (0x10)
#define SM_PLATFORM_PINCTRL_SION_SHIFT (4U)
#define SM_PLATFORM_PINCTRL_SION(x) \
    (((uint32_t)(((uint32_t)(x)) << SM_PLATFORM_PINCTRL_SION_SHIFT)) & SM_PLATFORM_PINCTRL_SION_MASK)

#define SM_PLATFORM_PINCTRL_BASE         IMX9_IOMUXC_BASE
#define SM_PLATFORM_PINCTRL_MUXREG_OFF   (SM_PLATFORM_PINCTRL_BASE)
#define SM_PLATFORM_PINCTRL_CFGREG_OFF   (SM_PLATFORM_PINCTRL_BASE + 0x204) /* 0x443c0204 */
#define SM_PLATFORM_PINCTRL_DAISYREG_OFF (SM_PLATFORM_PINCTRL_BASE + 0x408) /* 0x443c0408 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct
{
    uint32_t channel;    /* channel id: SCMI_A2P, SCMI_NOTIRY, SCMI_P2A, */
    uint32_t rateu;
    uint32_t ratel;
    uint32_t clk_id;     /* clock device id */
    uint32_t pclk_id;    /* parent clock device id */
    uint32_t div;        /* clock divider */
    uint32_t attributes; /* clock attributes */
    uint32_t oem_config_val;
    uint32_t flags;
} sm_clock_t;

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
} sm_pinctrl_t;

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
 *
 ****************************************************************************/

void imx9_scmi_initialize(void);

int32_t imx9_scmi_clockparentget(uint32_t channel, uint32_t clockid,
    uint32_t *parentid);
int32_t imx9_scmi_clockparentset(uint32_t channel, uint32_t clockid,
    uint32_t parentid);
int32_t imx9_scmi_clockrateget(uint32_t channel, uint32_t clockid,
    scmi_clock_rate_t *rate);
int32_t imx9_scmi_clockrateset(uint32_t channel, uint32_t clockid,
    uint32_t flags, scmi_clock_rate_t rate);
int32_t imx9_scmi_clockconfigset(uint32_t channel, uint32_t clockid,
    uint32_t attributes, uint32_t oem_config_val);
int32_t imx9_scmi_pinctrlconfigset(uint32_t channel, uint32_t identifier,
    uint32_t attributes, const scmi_pin_config_t *configs);

#endif /* __ARCH_ARM_SRC_IMX9_IMX9_SCMI_H */
