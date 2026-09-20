/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_spi.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_SPI_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets for SPI master (SPIM) ***********************************/

#define NRF54L_SPIM_TASK_START_OFFSET                       0x0000             /* Start SPI transaction */
#define NRF54L_SPIM_TASK_STOP_OFFSET                        0x0004             /* Stop SPI transaction */
#define NRF54L_SPIM_TASK_SUSPEND_OFFSET                     0x000c             /* Suspend SPI transaction */
#define NRF54L_SPIM_TASK_RESUME_OFFSET                      0x0010             /* Resume SPI transaction */
#define NRF54L_SPIM_TASK_DMA_RX_ENABLEMATCH_OFFSET(n)       (0x0028 + 4 * (n)) /* Enable RX match event n */
#define NRF54L_SPIM_TASK_DMA_RX_DISABLEMATCH_OFFSET(n)      (0x0038 + 4 * (n)) /* Disable RX match event n */
#define NRF54L_SPIM_SUBSCRIBE_START_OFFSET                  0x0080             /* Subscribe to START task */
#define NRF54L_SPIM_SUBSCRIBE_STOP_OFFSET                   0x0084             /* Subscribe to STOP task */
#define NRF54L_SPIM_SUBSCRIBE_SUSPEND_OFFSET                0x008c             /* Subscribe to SUSPEND task */
#define NRF54L_SPIM_SUBSCRIBE_RESUME_OFFSET                 0x0090             /* Subscribe to RESUME task */
#define NRF54L_SPIM_SUBSCRIBE_DMA_RX_ENABLEMATCH_OFFSET(n)  (0x00a8 + 4 * (n)) /* Subscribe to RX ENABLEMATCH task n */
#define NRF54L_SPIM_SUBSCRIBE_DMA_RX_DISABLEMATCH_OFFSET(n) (0x00b8 + 4 * (n)) /* Subscribe to RX DISABLEMATCH task n */
#define NRF54L_SPIM_EVENTS_STARTED_OFFSET                   0x0100             /* Transaction started */
#define NRF54L_SPIM_EVENTS_STOPPED_OFFSET                   0x0104             /* Transaction stopped */
#define NRF54L_SPIM_EVENTS_END_OFFSET                       0x0108             /* RX and TX complete */
#define NRF54L_SPIM_EVENTS_DMA_RX_END_OFFSET                0x014c             /* RX DMA buffer completed */
#define NRF54L_SPIM_EVENTS_DMA_RX_READY_OFFSET              0x0150             /* RX DMA buffer prepared */
#define NRF54L_SPIM_EVENTS_DMA_RX_BUSERROR_OFFSET           0x0154             /* RX DMA bus error */
#define NRF54L_SPIM_EVENTS_DMA_RX_MATCH_OFFSET(n)           (0x0158 + 4 * (n)) /* RX data matched candidate n */
#define NRF54L_SPIM_EVENTS_DMA_TX_END_OFFSET                0x0168             /* TX DMA buffer completed */
#define NRF54L_SPIM_EVENTS_DMA_TX_READY_OFFSET              0x016c             /* TX DMA buffer prepared */
#define NRF54L_SPIM_EVENTS_DMA_TX_BUSERROR_OFFSET           0x0170             /* TX DMA bus error */
#define NRF54L_SPIM_PUBLISH_STARTED_OFFSET                  0x0180             /* Publish STARTED event */
#define NRF54L_SPIM_PUBLISH_STOPPED_OFFSET                  0x0184             /* Publish STOPPED event */
#define NRF54L_SPIM_PUBLISH_END_OFFSET                      0x0188             /* Publish END event */
#define NRF54L_SPIM_PUBLISH_DMA_RX_END_OFFSET               0x01cc             /* Publish RX END event */
#define NRF54L_SPIM_PUBLISH_DMA_RX_READY_OFFSET             0x01d0             /* Publish RX READY event */
#define NRF54L_SPIM_PUBLISH_DMA_RX_BUSERROR_OFFSET          0x01d4             /* Publish RX BUSERROR event */
#define NRF54L_SPIM_PUBLISH_DMA_RX_MATCH_OFFSET(n)          (0x01d8 + 4 * (n)) /* Publish RX MATCH event n */
#define NRF54L_SPIM_PUBLISH_DMA_TX_END_OFFSET               0x01e8             /* Publish TX END event */
#define NRF54L_SPIM_PUBLISH_DMA_TX_READY_OFFSET             0x01ec             /* Publish TX READY event */
#define NRF54L_SPIM_PUBLISH_DMA_TX_BUSERROR_OFFSET          0x01f0             /* Publish TX BUSERROR event */
#define NRF54L_SPIM_SHORTS_OFFSET                           0x0200             /* Event/task shortcuts */
#define NRF54L_SPIM_INTENSET_OFFSET                         0x0304             /* Enable interrupt */
#define NRF54L_SPIM_INTENCLR_OFFSET                         0x0308             /* Disable interrupt */
#define NRF54L_SPIM_ENABLE_OFFSET                           0x0500             /* Enable SPIM */
#define NRF54L_SPIM_PRESCALER_OFFSET                        0x052c             /* Core clock to SCK divisor */
#define NRF54L_SPIM_CONFIG_OFFSET                           0x0554             /* Configuration */
#define NRF54L_SPIM_RXDELAY_OFFSET                          0x05ac             /* Input sample delay */
#define NRF54L_SPIM_CSNDUR_OFFSET                           0x05b0             /* CSN to SCK duration */
#define NRF54L_SPIM_DCXCNT_OFFSET                           0x05b4             /* DCX configuration */
#define NRF54L_SPIM_CSNPOL_OFFSET                           0x05b8             /* CSN polarity */
#define NRF54L_SPIM_ORC_OFFSET                              0x05c0             /* Over-read character */
#define NRF54L_SPIM_PSELSCK_OFFSET                          0x0600             /* SCK pin select */
#define NRF54L_SPIM_PSELMOSI_OFFSET                         0x0604             /* MOSI pin select */
#define NRF54L_SPIM_PSELMISO_OFFSET                         0x0608             /* MISO pin select */
#define NRF54L_SPIM_PSELDCX_OFFSET                          0x060c             /* DCX pin select */
#define NRF54L_SPIM_PSELCSN_OFFSET                          0x0610             /* CSN pin select */
#define NRF54L_SPIM_DMA_RX_PTR_OFFSET                       0x0704             /* RX buffer address */
#define NRF54L_SPIM_DMA_RX_MAXCNT_OFFSET                    0x0708             /* RX buffer size */
#define NRF54L_SPIM_DMA_RX_AMOUNT_OFFSET                    0x070c             /* Number of received bytes */
#define NRF54L_SPIM_DMA_RX_LIST_OFFSET                      0x0714             /* RX list type */
#define NRF54L_SPIM_DMA_RX_TERMINATEONBUSERROR_OFFSET       0x071c             /* Terminate RX DMA on bus error */
#define NRF54L_SPIM_DMA_RX_BUSERRORADDRESS_OFFSET           0x0720             /* Address of last RX DMA bus error */
#define NRF54L_SPIM_DMA_RX_MATCH_CONFIG_OFFSET              0x0724             /* RX DMA match configuration */
#define NRF54L_SPIM_DMA_RX_MATCH_CANDIDATE_OFFSET(n)        (0x0728 + 4 * (n)) /* RX DMA match candidate n */
#define NRF54L_SPIM_DMA_TX_PTR_OFFSET                       0x073c             /* TX buffer address */
#define NRF54L_SPIM_DMA_TX_MAXCNT_OFFSET                    0x0740             /* TX buffer size */
#define NRF54L_SPIM_DMA_TX_AMOUNT_OFFSET                    0x0744             /* Number of transmitted bytes */
#define NRF54L_SPIM_DMA_TX_LIST_OFFSET                      0x074c             /* TX list type */
#define NRF54L_SPIM_DMA_TX_TERMINATEONBUSERROR_OFFSET       0x0754             /* Terminate TX DMA on bus error */
#define NRF54L_SPIM_DMA_TX_BUSERRORADDRESS_OFFSET           0x0758             /* Address of last TX DMA bus error */

/* Register Bitfield Definitions for SPIM ***********************************/

/* Task and event registers */

#define SPIM_TASKS_START                   (1 << 0)
#define SPIM_TASKS_STOP                    (1 << 0)
#define SPIM_TASKS_SUSPEND                 (1 << 0)
#define SPIM_TASKS_RESUME                  (1 << 0)
#define SPIM_EVENTS_STARTED                (1 << 0)
#define SPIM_EVENTS_STOPPED                (1 << 0)
#define SPIM_EVENTS_END                    (1 << 0)

/* SHORTS register */

#define SPIM_SHORTS_ENDSTART               (1 << 17)
#define SPIM_SHORTS_MATCH_ENABLE(n)        (1 << (21 + (n)))
#define SPIM_SHORTS_MATCH_DISABLE(n)       (1 << (25 + (n)))

/* INTENSET/INTENCLR registers */

#define SPIM_INT_STARTED                   (1 << 0)
#define SPIM_INT_STOPPED                   (1 << 1)
#define SPIM_INT_END                       (1 << 2)
#define SPIM_INT_DMA_RX_END                (1 << 19)
#define SPIM_INT_DMA_RX_READY              (1 << 20)
#define SPIM_INT_DMA_RX_BUSERROR           (1 << 21)
#define SPIM_INT_DMA_RX_MATCH(n)           (1 << (22 + (n)))
#define SPIM_INT_DMA_TX_END                (1 << 26)
#define SPIM_INT_DMA_TX_READY              (1 << 27)
#define SPIM_INT_DMA_TX_BUSERROR           (1 << 28)

/* ENABLE register */

#define SPIM_ENABLE_DIS                    (0)
#define SPIM_ENABLE_EN                     (7)

/* PSEL registers */

#define SPIM_PSEL_PIN_SHIFT                (0)
#define SPIM_PSEL_PIN_MASK                 (0x1f << SPIM_PSEL_PIN_SHIFT)
#define SPIM_PSEL_PORT_SHIFT               (5)
#define SPIM_PSEL_PORT_MASK                (0x7 << SPIM_PSEL_PORT_SHIFT)
#define SPIM_PSEL_CONNECTED                (0 << 31)
#define SPIM_PSEL_DISCONNECTED             (1 << 31)
#define SPIM_PSEL_RESET                    (0xffffffff)

/* PRESCALER register: even divisors only */

#define SPIM_PRESCALER_DIVISOR_SHIFT        (0)
#define SPIM_PRESCALER_DIVISOR_MASK         (0x7f << SPIM_PRESCALER_DIVISOR_SHIFT)
#define SPIM_PRESCALER_DIVISOR_MIN          (2)
#define SPIM_PRESCALER_DIVISOR_MAX          (126)

/* CONFIG register */

#define SPIM_CONFIG_ORDER                  (1 << 0)
#define SPIM_CONFIG_CPHA                   (1 << 1)
#define SPIM_CONFIG_CPOL                   (1 << 2)

/* DMA registers */

#define SPIM_DMA_MAXCNT_SHIFT              (0)
#define SPIM_DMA_MAXCNT_MASK               (0xffff << SPIM_DMA_MAXCNT_SHIFT)
#define SPIM_DMA_AMOUNT_SHIFT              (0)
#define SPIM_DMA_AMOUNT_MASK               (0xffff << SPIM_DMA_AMOUNT_SHIFT)
#define SPIM_DMA_LIST_DISABLED             (0)
#define SPIM_DMA_LIST_ARRAYLIST            (1)
#define SPIM_DMA_TERMINATEONBUSERROR       (1)
#define SPIM_DMA_MATCH_ENABLE(n)           (1 << (n))
#define SPIM_DMA_MATCH_ONESHOT(n)          (1 << (16 + (n)))
#define SPIM_DMA_MATCH_CANDIDATE_MASK      (0xff)

/* DPPI subscribe and publish registers */

#define SPIM_DPPI_CHIDX_SHIFT              (0)
#define SPIM_DPPI_CHIDX_MASK               (0xff << SPIM_DPPI_CHIDX_SHIFT)
#define SPIM_DPPI_EN                       (1 << 31)

/* IFTIMING, DCXCNT, CSNPOL and ORC registers */

#define SPIM_RXDELAY_SHIFT                 (0)
#define SPIM_RXDELAY_MASK                  (0x7 << SPIM_RXDELAY_SHIFT)
#define SPIM_CSNDUR_SHIFT                  (0)
#define SPIM_CSNDUR_MASK                   (0xff << SPIM_CSNDUR_SHIFT)
#define SPIM_DCXCNT_SHIFT                  (0)
#define SPIM_DCXCNT_MASK                   (0xf << SPIM_DCXCNT_SHIFT)
#define SPIM_CSNPOL_LOW                    (0)
#define SPIM_CSNPOL_HIGH                   (1)
#define SPIM_ORC_SHIFT                     (0)
#define SPIM_ORC_MASK                      (0xff << SPIM_ORC_SHIFT)

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_SPI_H */
