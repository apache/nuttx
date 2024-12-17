/****************************************************************************
 * arch/arm/src/nrf91/nrf91_modem.h
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

#ifndef __ARCH_ARM_SRC_NRF91_NRF91_MODEM_H
#define __ARCH_ARM_SRC_NRF91_NRF91_MODEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/mm/mm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Constant from nrf_modem */

#define NRF91_SHMEM_CTRL_SIZE  (0x4e8)

/* REVISIT: heap overhead */

#define HEAP_OVERHEAD          (400)

/* Shared memory configuration */

#define NRF91_SHMEM_START_ADDR (CONFIG_RAM_START)
#define NRF91_SHMEM_CTRL_BASE  (NRF91_SHMEM_START_ADDR)
#define NRF91_SHMEM_TX_BASE    (NRF91_SHMEM_CTRL_BASE+NRF91_SHMEM_CTRL_SIZE)
#define NRF91_SHMEM_TX_SIZE    (CONFIG_NRF91_MODEM_SHMEM_TX_SIZE-HEAP_OVERHEAD)
#define NRF91_SHMEM_RX_BASE    (NRF91_SHMEM_TX_BASE+NRF91_SHMEM_TX_SIZE)
#define NRF91_SHMEM_RX_SIZE    (CONFIG_NRF91_MODEM_SHMEM_RX_SIZE)
#define NRF91_SHMEM_TRACE_BASE (NRF91_SHMEM_RX_BASE+NRF91_SHMEM_RX_SIZE)
#define NRF91_SHMEM_TRACE_SIZE (CONFIG_NRF91_MODEM_SHMEM_TRACE_SIZE)

#if !(NRF91_SHMEM_CTRL_BASE % 4 == 0)
#  error SHMEM base address must be word-aligned (4 bytes)
#endif

#if !(NRF91_SHMEM_TX_BASE % 4 == 0)
#  error SHMEM base address must be word-aligned (4 bytes)
#endif

#if !(NRF91_SHMEM_RX_BASE % 4 == 0)
#  error SHMEM base address must be word-aligned (4 bytes)
#endif

#if !(NRF91_SHMEM_TRACE_BASE % 4 == 0)
#  error SHMEM base address must be word-aligned (4 bytes)
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_modem_board_init
 *
 * This function must be provided by board-spcific logic.
 *
 ****************************************************************************/

int nrf91_modem_board_init(void);

/****************************************************************************
 * Name: nrf91_modem_initialize
 ****************************************************************************/

int nrf91_modem_initialize(void);

#endif /* __ARCH_ARM_SRC_NRF91_NRF91_MODEM_H */
