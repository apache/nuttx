/****************************************************************************
 * arch/arm/src/nrf52/nrf52_ppi.h
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

#ifndef __ARCH_ARM_SRC_NRF52_NRF52_PPI_H
#define __ARCH_ARM_SRC_NRF52_NRF52_PPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Nordic SDC makes use of PPI channels 17-31 */

#ifdef CONFIG_NRF52_SOFTDEVICE_CONTROLLER
#  define NRF52_PPI_NUM_CHANNELS              16    /* Total number of PPI channels */
#  define NRF52_PPI_NUM_CONFIGURABLE_CHANNELS 16    /* Number of configurable PPI channels */
#  define NRF52_PPI_NUM_GROUPS                6     /* Number of PPI channel groups */
#else
#  define NRF52_PPI_NUM_CHANNELS              32    /* Total number of PPI channels */
#  define NRF52_PPI_NUM_CONFIGURABLE_CHANNELS 20    /* Number of configurable PPI channels */
#  define NRF52_PPI_NUM_GROUPS                6     /* Number of PPI channel groups */
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_ppi_channel_enable
 *
 * Description:
 *   Enable/disable a given PPI channel
 *
 * Input Parameters:
 *   - ch: channel number
 *   - enable: enable (true) or disable (false)
 *
 ****************************************************************************/

void nrf52_ppi_channel_enable(uint8_t ch, bool enable);

/****************************************************************************
 * Name: nrf52_ppi_set_event_ep
 *
 * Description:
 *   Associate a channel with an event
 *
 * Input Parameters:
 *   - ch: channel number
 *   - event_reg: address of event register
 *
 ****************************************************************************/

void nrf52_ppi_set_event_ep(uint8_t ch, uint32_t event_reg);

/****************************************************************************
 * Name: nrf52_ppi_set_task_ep
 *
 * Description:
 *   Associate a channel with a task
 *
 * Input Parameters:
 *   - ch: channel number
 *   - event_reg: address of task register
 *
 ****************************************************************************/

void nrf52_ppi_set_task_ep(uint8_t ch, uint32_t task_reg);

/****************************************************************************
 * Name: nrf52_ppi_set_task2_ep
 *
 * Description:
 *   Associate a second channel with a task ("fork" task)
 *
 * Input Parameters:
 *   - ch: channel number
 *   - event_reg: address of task register
 *
 ****************************************************************************/

void nrf52_ppi_set_task2_ep(uint8_t ch, uint32_t task_reg);

/****************************************************************************
 * Name: nrf52_ppi_grp_channel_enable
 *
 * Description:
 *   Add/remove a channel to/from a group
 *
 * Input Parameters:
 *   - group: group number
 *   - ch: channel number
 *   - enable: add (true) or remove (false)
 *
 ****************************************************************************/

void nrf52_ppi_grp_channel_enable(uint8_t group, uint8_t ch, bool enable);

/****************************************************************************
 * Name: nrf52_ppi_grp_clear
 *
 * Description:
 *   Clear group (disable all its channels)
 *
 * Input Parameters:
 *   - group: group number
 *
 ****************************************************************************/

void nrf52_ppi_grp_clear(uint8_t group);

/****************************************************************************
 * Name: nrf52_ppi_grp_enable
 *
 * Description:
 *   Enable/disable a given PPI channel group
 *
 * Input Parameters:
 *   - group: group number
 *   - enable: enable (true) or disable (false)
 *
 ****************************************************************************/

void nrf52_ppi_grp_enable(uint8_t group, bool enable);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif // __ARCH_ARM_SRC_NRF52_NRF52_PPI_H
