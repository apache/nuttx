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

/* SCMI protocol IDs */

#define SCMI_PROTOCOL_BASE    0x10u /* Base protocol */
#define SCMI_PROTOCOL_POWER   0x11u /* Power domain management protocol */
#define SCMI_PROTOCOL_SYS     0x12u /* System power management protocol */
#define SCMI_PROTOCOL_PERF    0x13u /* Performance domain management protocol */
#define SCMI_PROTOCOL_CLOCK   0x14u /* Clock management protocol */
#define SCMI_PROTOCOL_SENSOR  0x15u /* Sensor management protocol */
#define SCMI_PROTOCOL_RESET   0x16u /* Reset domain management protocol */
#define SCMI_PROTOCOL_VOLTAGE 0x17u /* Voltage domain management protocol */
#define SCMI_PROTOCOL_PINCTRL 0x19u /* Pin control protocol */
#define SCMI_PROTOCOL_LMM     0x80u /* LM management protocol */
#define SCMI_PROTOCOL_BBM     0x81u /* BBM management protocol */
#define SCMI_PROTOCOL_CPU     0x82u /* CPU management protocol */
#define SCMI_PROTOCOL_FUSA    0x83u /* FuSa protocol */
#define SCMI_PROTOCOL_MISC    0x84u /* Misc protocol */

/* SCMI status codes returned by the System Manager. The request functions
 * pass these through unchanged; they overlap with negated errno values.
 */

#define SCMI_ERR_NOT_SUPPORTED      (-1)
#define SCMI_ERR_INVALID_PARAMETERS (-2)
#define SCMI_ERR_DENIED             (-3)
#define SCMI_ERR_NOT_FOUND          (-4)
#define SCMI_ERR_OUT_OF_RANGE       (-5)
#define SCMI_ERR_BUSY               (-6)
#define SCMI_ERR_COMMS_ERROR        (-7)
#define SCMI_ERR_GENERIC_ERROR      (-8)
#define SCMI_ERR_HARDWARE_ERROR     (-9)
#define SCMI_ERR_PROTOCOL_ERROR     (-10)

/* SCMI LMM notification message ID and event flags */

#define SCMI_MSG_LMM_EVENT      0x0u
#define SCMI_LMM_EVENT_BOOT     (1u << 0u)
#define SCMI_LMM_EVENT_SHUTDOWN (1u << 1u)
#define SCMI_LMM_EVENT_SUSPEND  (1u << 2u)
#define SCMI_LMM_EVENT_WAKE     (1u << 3u)

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

/* Length of the silicon name string returned by MISC_SI_INFO */

#define SCMI_MISC_MAX_SINAME 16u

/* SCMI silicon info, as reported by the misc protocol MISC_SI_INFO */

typedef struct
{
    uint32_t deviceid;                     /* Silicon specific device ID */
    uint32_t sirev;                        /* Silicon specific revision */
    uint32_t partnum;                      /* Silicon specific part number */
    uint8_t  siname[SCMI_MISC_MAX_SINAME]; /* Silicon name/revision string */
} scmi_si_info_t;

/* SCMI notification callback, see imx9_scmi_notify_register() */

typedef void (*imx9_scmi_notify_cb_t)(uint32_t protocol, uint32_t msgid,
                                      void *arg);

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

/****************************************************************************
 * Name: imx9_scmi_siinfoget
 *
 * Description:
 *   Read the silicon info (device ID, revision, part number and name) from
 *   the System Manager via the SCMI misc protocol.
 *
 * Input Parameters:
 *   channel - SCMI channel to use
 *   info    - Returned silicon info
 *
 * Returned Value:
 *   OK on success, a negated errno value on failure.
 *
 ****************************************************************************/

int32_t imx9_scmi_siinfoget(uint32_t channel, scmi_si_info_t *info);

/****************************************************************************
 * Name: imx9_scmi_notify_register
 *
 * Description:
 *   Register the callback for SCMI notifications from the System Manager
 *   and enable their reception. The callback is called from interrupt
 *   context with the protocol and message ID of the notification and must
 *   not block, so it must not call the SCMI request functions (they lock
 *   the A2P channel); defer such work, e.g. with work_queue(). It may read
 *   the message with the matching event function
 *   (e.g. imx9_scmi_lmmevent()); this is only valid inside the callback.
 *   The channel is handed back to the System Manager when it returns.
 *   Must be called after the OS is initialized.
 *
 * Input Parameters:
 *   cb  - Notification callback
 *   arg - Argument passed to the callback
 *
 * Returned Value:
 *   OK on success, -EBUSY if a callback is already registered, a negated
 *   errno value on other failures.
 *
 ****************************************************************************/

int32_t imx9_scmi_notify_register(imx9_scmi_notify_cb_t cb, void *arg);

/****************************************************************************
 * Name: imx9_scmi_lmmnotify
 *
 * Description:
 *   Subscribe to boot/shutdown notifications of a logical machine via the
 *   SCMI LMM protocol. Register the notification callback first, the
 *   System Manager may send an event right away.
 *
 * Input Parameters:
 *   channel - SCMI channel to use for the request
 *   lmid    - Logical machine to receive events from
 *   flags   - Events to enable (SCMI_LMM_EVENT_*)
 *
 * Returned Value:
 *   OK on success, a negated errno value on failure.
 *
 ****************************************************************************/

int32_t imx9_scmi_lmmnotify(uint32_t channel, uint32_t lmid, uint32_t flags);

/****************************************************************************
 * Name: imx9_scmi_lmmevent
 *
 * Description:
 *   Read the pending LMM event notification. Only valid in the
 *   notification callback for SCMI_PROTOCOL_LMM / SCMI_MSG_LMM_EVENT.
 *
 * Input Parameters:
 *   lmid    - Returned ID of the logical machine that caused the
 *             transition, may be NULL
 *   eventlm - Returned ID of the logical machine the event is for, may be
 *             NULL
 *   flags   - Returned events (SCMI_LMM_EVENT_*), may be NULL
 *
 * Returned Value:
 *   OK on success, -ENODATA if no notification is pending, -EIO on a CRC
 *   error, -EPROTO if the message is not an LMM event.
 *
 ****************************************************************************/

int32_t imx9_scmi_lmmevent(uint32_t *lmid, uint32_t *eventlm,
                           uint32_t *flags);

/****************************************************************************
 * Name: imx9_scmi_bbmgprset
 *
 * Description:
 *   Write a battery-backed general purpose register (BBNSM GPR) via the
 *   SCMI BBM protocol. The agent needs EXCLUSIVE or higher access to the
 *   GPR in the System Manager configuration. Blocks until the System
 *   Manager responds, must not be called from interrupt context.
 *
 * Input Parameters:
 *   channel - SCMI channel to use for the request
 *   index   - Index of the GPR to write
 *   value   - 32-bit value to write
 *
 * Returned Value:
 *   OK on success, SCMI_ERR_NOT_FOUND if the index is not valid,
 *   SCMI_ERR_DENIED if the agent may not write the GPR, another SCMI
 *   status or a negated errno value on other failures. A CRC error on the
 *   response returns -EIO, which has the same value as
 *   SCMI_ERR_OUT_OF_RANGE.
 *
 ****************************************************************************/

int32_t imx9_scmi_bbmgprset(uint32_t channel, uint32_t index,
                            uint32_t value);

/****************************************************************************
 * Name: imx9_scmi_bbmgprget
 *
 * Description:
 *   Read a battery-backed general purpose register (BBNSM GPR) via the
 *   SCMI BBM protocol. The agent needs GET or higher access to the GPR in
 *   the System Manager configuration. Blocks until the System Manager
 *   responds, must not be called from interrupt context.
 *
 * Input Parameters:
 *   channel - SCMI channel to use for the request
 *   index   - Index of the GPR to read
 *   value   - Returned 32-bit GPR value, may be NULL
 *
 * Returned Value:
 *   OK on success, SCMI_ERR_NOT_FOUND if the index is not valid,
 *   SCMI_ERR_DENIED if the agent may not read the GPR, another SCMI
 *   status or a negated errno value on other failures. A CRC error on the
 *   response returns -EIO, which has the same value as
 *   SCMI_ERR_OUT_OF_RANGE.
 *
 ****************************************************************************/

int32_t imx9_scmi_bbmgprget(uint32_t channel, uint32_t index,
                            uint32_t *value);

#endif /* __ARCH_ARM_SRC_IMX9_IMX9_SCMI_H */
