/****************************************************************************
 * arch/arm/src/nrf91/nrf91_nrfx_ipc.h
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

#ifndef __ARCH_ARM_SRC_NRF91_NRF91_NRFX_IPC_H
#define __ARCH_ARM_SRC_NRF91_NRF91_NRFX_IPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/nrf91_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPC_CONF_NUM (NRF91_IPC_CHANS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (*nrfx_ipc_handler_t)(uint8_t event_idx, void *context);

typedef struct
{
  uint32_t send_task_config[IPC_CONF_NUM];
  uint32_t receive_event_config[IPC_CONF_NUM];
  uint32_t receive_events_enabled;
} nrfx_ipc_config_t;

typedef int nrfx_err_t;

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrfx_ipc_uninit
 ****************************************************************************/

void nrfx_ipc_uninit(void);

/****************************************************************************
 * Name: nrfx_ipc_init
 ****************************************************************************/

nrfx_err_t nrfx_ipc_init(uint8_t irq_priority, nrfx_ipc_handler_t handler,
                         void *context);

/****************************************************************************
 * Name: nrfx_ipc_config_load
 ****************************************************************************/

void nrfx_ipc_config_load(const nrfx_ipc_config_t *config);

/****************************************************************************
 * Name: nrfx_ipc_receive_event_enable
 ****************************************************************************/

void nrfx_ipc_receive_event_enable(uint8_t event_index);

/****************************************************************************
 * Name: nrfx_ipc_receive_event_disable
 ****************************************************************************/

void nrfx_ipc_receive_event_disable(uint8_t event_index);

#endif /* __ARCH_ARM_SRC_NRF91_NRF91_NRFX_IPC_H */
