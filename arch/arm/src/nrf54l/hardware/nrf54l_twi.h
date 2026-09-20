/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_twi.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_TWI_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_TWI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets for TWI master (TWIM) ***********************************/

#define NRF54L_TWIM_TASKS_STOP_OFFSET                       0x0004             /* Stop TWIM transaction */
#define NRF54L_TWIM_TASKS_SUSPEND_OFFSET                    0x000c             /* Suspend TWIM transaction */
#define NRF54L_TWIM_TASKS_RESUME_OFFSET                     0x0010             /* Resume TWIM transaction */
#define NRF54L_TWIM_TASKS_DMA_RX_START_OFFSET               0x0028             /* Start receive sequence */
#define NRF54L_TWIM_TASKS_DMA_RX_STOP_OFFSET                0x002c             /* Stop RX DMA */
#define NRF54L_TWIM_TASKS_DMA_RX_ENABLEMATCH_OFFSET(n)      (0x0030 + 4 * (n)) /* Enable RX match event n */
#define NRF54L_TWIM_TASKS_DMA_RX_DISABLEMATCH_OFFSET(n)     (0x0040 + 4 * (n)) /* Disable RX match event n */
#define NRF54L_TWIM_TASKS_DMA_TX_START_OFFSET               0x0050             /* Start transmit sequence */
#define NRF54L_TWIM_TASKS_DMA_TX_STOP_OFFSET                0x0054             /* Stop TX DMA */
#define NRF54L_TWIM_SUBSCRIBE_STOP_OFFSET                   0x0084             /* Subscribe to STOP task */
#define NRF54L_TWIM_SUBSCRIBE_SUSPEND_OFFSET                0x008c             /* Subscribe to SUSPEND task */
#define NRF54L_TWIM_SUBSCRIBE_RESUME_OFFSET                 0x0090             /* Subscribe to RESUME task */
#define NRF54L_TWIM_SUBSCRIBE_DMA_RX_START_OFFSET           0x00a8             /* Subscribe to RX START task */
#define NRF54L_TWIM_SUBSCRIBE_DMA_RX_STOP_OFFSET            0x00ac             /* Subscribe to RX STOP task */
#define NRF54L_TWIM_SUBSCRIBE_DMA_RX_ENABLEMATCH_OFFSET(n)  (0x00b0 + 4 * (n)) /* Subscribe to RX ENABLEMATCH task n */
#define NRF54L_TWIM_SUBSCRIBE_DMA_RX_DISABLEMATCH_OFFSET(n) (0x00c0 + 4 * (n)) /* Subscribe to RX DISABLEMATCH task n */
#define NRF54L_TWIM_SUBSCRIBE_DMA_TX_START_OFFSET           0x00d0             /* Subscribe to TX START task */
#define NRF54L_TWIM_SUBSCRIBE_DMA_TX_STOP_OFFSET            0x00d4             /* Subscribe to TX STOP task */
#define NRF54L_TWIM_EVENTS_STOPPED_OFFSET                   0x0104             /* TWIM stopped */
#define NRF54L_TWIM_EVENTS_ERROR_OFFSET                     0x0114             /* TWIM error */
#define NRF54L_TWIM_EVENTS_SUSPENDED_OFFSET                 0x0128             /* TWIM suspended */
#define NRF54L_TWIM_EVENTS_LASTRX_OFFSET                    0x0134             /* Starting to receive the last byte */
#define NRF54L_TWIM_EVENTS_LASTTX_OFFSET                    0x0138             /* Starting to transmit the last byte */
#define NRF54L_TWIM_EVENTS_DMA_RX_END_OFFSET                0x014c             /* RX DMA buffer completed */
#define NRF54L_TWIM_EVENTS_DMA_RX_READY_OFFSET              0x0150             /* RX DMA buffer prepared */
#define NRF54L_TWIM_EVENTS_DMA_RX_BUSERROR_OFFSET           0x0154             /* RX DMA bus error */
#define NRF54L_TWIM_EVENTS_DMA_RX_MATCH_OFFSET(n)           (0x0158 + 4 * (n)) /* RX data matched candidate n */
#define NRF54L_TWIM_EVENTS_DMA_TX_END_OFFSET                0x0168             /* TX DMA buffer completed */
#define NRF54L_TWIM_EVENTS_DMA_TX_READY_OFFSET              0x016c             /* TX DMA buffer prepared */
#define NRF54L_TWIM_EVENTS_DMA_TX_BUSERROR_OFFSET           0x0170             /* TX DMA bus error */
#define NRF54L_TWIM_PUBLISH_STOPPED_OFFSET                  0x0184             /* Publish STOPPED event */
#define NRF54L_TWIM_PUBLISH_ERROR_OFFSET                    0x0194             /* Publish ERROR event */
#define NRF54L_TWIM_PUBLISH_SUSPENDED_OFFSET                0x01a8             /* Publish SUSPENDED event */
#define NRF54L_TWIM_PUBLISH_LASTRX_OFFSET                   0x01b4             /* Publish LASTRX event */
#define NRF54L_TWIM_PUBLISH_LASTTX_OFFSET                   0x01b8             /* Publish LASTTX event */
#define NRF54L_TWIM_PUBLISH_DMA_RX_END_OFFSET               0x01cc             /* Publish RX END event */
#define NRF54L_TWIM_PUBLISH_DMA_RX_READY_OFFSET             0x01d0             /* Publish RX READY event */
#define NRF54L_TWIM_PUBLISH_DMA_RX_BUSERROR_OFFSET          0x01d4             /* Publish RX BUSERROR event */
#define NRF54L_TWIM_PUBLISH_DMA_RX_MATCH_OFFSET(n)          (0x01d8 + 4 * (n)) /* Publish RX MATCH event n */
#define NRF54L_TWIM_PUBLISH_DMA_TX_END_OFFSET               0x01e8             /* Publish TX END event */
#define NRF54L_TWIM_PUBLISH_DMA_TX_READY_OFFSET             0x01ec             /* Publish TX READY event */
#define NRF54L_TWIM_PUBLISH_DMA_TX_BUSERROR_OFFSET          0x01f0             /* Publish TX BUSERROR event */
#define NRF54L_TWIM_SHORTS_OFFSET                           0x0200             /* Shortcuts between local events and tasks */
#define NRF54L_TWIM_INTEN_OFFSET                            0x0300             /* Enable or disable interrupt */
#define NRF54L_TWIM_INTENSET_OFFSET                         0x0304             /* Enable interrupt */
#define NRF54L_TWIM_INTENCLR_OFFSET                         0x0308             /* Disable interrupt */
#define NRF54L_TWIM_ERRORSRC_OFFSET                         0x04c4             /* Error source */
#define NRF54L_TWIM_ENABLE_OFFSET                           0x0500             /* Enable TWIM */
#define NRF54L_TWIM_PSELSCL_OFFSET                          0x0600             /* Pin select for SCL signal */
#define NRF54L_TWIM_PSELSDA_OFFSET                          0x0604             /* Pin select for SDA signal */
#define NRF54L_TWIM_FREQUENCY_OFFSET                        0x0524             /* TWIM frequency */
#define NRF54L_TWIM_DMA_RX_PTR_OFFSET                       0x0704             /* RX buffer address */
#define NRF54L_TWIM_DMA_RX_MAXCNT_OFFSET                    0x0708             /* RX buffer size */
#define NRF54L_TWIM_DMA_RX_AMOUNT_OFFSET                    0x070c             /* Number of received bytes */
#define NRF54L_TWIM_DMA_RX_LIST_OFFSET                      0x0714             /* RX list type */
#define NRF54L_TWIM_DMA_RX_TERMINATEONBUSERROR_OFFSET       0x071c             /* Terminate RX DMA on bus error */
#define NRF54L_TWIM_DMA_RX_BUSERRORADDRESS_OFFSET           0x0720             /* Address of last RX DMA bus error */
#define NRF54L_TWIM_DMA_RX_MATCH_CONFIG_OFFSET              0x0724             /* RX DMA match configuration */
#define NRF54L_TWIM_DMA_RX_MATCH_CANDIDATE_OFFSET(n)        (0x0728 + 4 * (n)) /* RX DMA match candidate n */
#define NRF54L_TWIM_DMA_TX_PTR_OFFSET                       0x073c             /* TX buffer address */
#define NRF54L_TWIM_DMA_TX_MAXCNT_OFFSET                    0x0740             /* TX buffer size */
#define NRF54L_TWIM_DMA_TX_AMOUNT_OFFSET                    0x0744             /* Number of transmitted bytes */
#define NRF54L_TWIM_DMA_TX_LIST_OFFSET                      0x074c             /* TX list type */
#define NRF54L_TWIM_DMA_TX_TERMINATEONBUSERROR_OFFSET       0x0754             /* Terminate TX DMA on bus error */
#define NRF54L_TWIM_DMA_TX_BUSERRORADDRESS_OFFSET           0x0758             /* Address of last TX DMA bus error */
#define NRF54L_TWIM_ADDRESS_OFFSET                          0x0588             /* TWIM address */

/* Register Bitfield Definitions for TWIM ***********************************/

/* SHORTS Register */

#define TWIM_SHORTS_LASTTX_STARTRX          (1 << 7)   /* Bit 7: Shortcut between event LASTTX and task STARTRX */
#define TWIM_SHORTS_LASTTX_SUSPEND          (1 << 8)   /* Bit 8: Shortcut between event LASTTX and task SUSPEND */
#define TWIM_SHORTS_LASTTX_STOP             (1 << 9)   /* Bit 9: Shortcut between event LASTTX and task STOP */
#define TWIM_SHORTS_LASTRX_STARTTX          (1 << 10)  /* Bit 10: Shortcut between event LASTRX and task STARTTX */
#define TWIM_SHORTS_LASTRX_STOP             (1 << 12)  /* Bit 12: Shortcut between event LASTRX and task STOP */

/* INTEN/INTENSET/INTENCLR Register */

#define TWIM_INT_STOPPED                    (1 << 1)   /* Bit 1: Interrupt for event STOPPED */
#define TWIM_INT_ERROR                      (1 << 5)   /* Bit 5: ERROR */
#define TWIM_INT_SUSPENDED                  (1 << 10)  /* Bit 10: SUSPENDED */
#define TWIM_INT_LASTRX                     (1 << 13)  /* Bit 13: LASTRX */
#define TWIM_INT_LASTTX                     (1 << 14)  /* Bit 14: LASTTX */
#define TWIM_INT_DMA_RX_END                 (1 << 19)
#define TWIM_INT_DMA_RX_READY               (1 << 20)
#define TWIM_INT_DMA_RX_BUSERROR            (1 << 21)
#define TWIM_INT_DMA_RX_MATCH(n)            (1 << (22 + (n)))
#define TWIM_INT_DMA_TX_END                 (1 << 26)
#define TWIM_INT_DMA_TX_READY               (1 << 27)
#define TWIM_INT_DMA_TX_BUSERROR            (1 << 28)

/* ERRORSRC Register */

#define TWIM_ERRORSRC_OVERRUN               (1 << 0)   /* Bit 0: Overrun error */
#define TWIM_ERRORSRC_ANACK                 (1 << 1)   /* Bit 1: NACK received after sending the address */
#define TWIM_ERRORSRC_DNACK                 (1 << 2)   /* Bit 2: NACK received after sending a data byte */

/* ENABLE Register */

#define TWIM_ENABLE_DIS                     (0)        /* Disable TWIM */
#define TWIM_ENABLE_EN                      (0x6 << 0) /* Enable TWIM */

/* PSELSCL Register */

#define TWIM_PSELSCL_PIN_SHIFT              (0)        /* Bits 0-4: SCL pin number */
#define TWIM_PSELSCL_PIN_MASK               (0x1f << TWIM_PSELSCL_PIN_SHIFT)
#define TWIM_PSELSCL_PORT_SHIFT             (5)        /* Bits 5-7: SCL port number */
#define TWIM_PSELSCL_PORT_MASK              (0x7 << TWIM_PSELSCL_PORT_SHIFT)
#define TWIM_PSELSCL_CONNECTED              (0 << 31)  /* Bit 31: Connection */
#define TWIM_PSELSCL_DISCONNECTED           (1 << 31)
#define TWIM_PSELSCL_RESET                  (0xffffffff)

/* PSELSDA Register */

#define TWIM_PSELSDA_PIN_SHIFT              (0)        /* Bits 0-4: SDA pin number */
#define TWIM_PSELSDA_PIN_MASK               (0x1f << TWIM_PSELSDA_PIN_SHIFT)
#define TWIM_PSELSDA_PORT_SHIFT             (5)        /* Bits 5-7: SDA port number */
#define TWIM_PSELSDA_PORT_MASK              (0x7 << TWIM_PSELSDA_PORT_SHIFT)
#define TWIM_PSELSDA_CONNECTED              (0 << 31)  /* Bit 31: Connection */
#define TWIM_PSELSDA_DISCONNECTED           (1 << 31)
#define TWIM_PSELSDA_RESET                  (0xffffffff)

/* FREQUENCY Register */

#define TWIM_FREQUENCY_100KBPS              (0x01980000) /* 100 kbps */
#define TWIM_FREQUENCY_250KBPS              (0x04000000) /* 250 kbps */
#define TWIM_FREQUENCY_400KBPS              (0x06400000) /* 400 kbps */
#define TWIM_FREQUENCY_1000KBPS             (0x0ff00000) /* 1000 kbps */

/* RXDMAXCNT Register */

#define TWIM_RXDMAXCNT_SHIFT                (0)        /* Bits 0-15: Maximum number of bytes in receive buffer */
#define TWIM_RXDMAXCNT_MASK                 (0xffff << TWIM_RXDMAXCNT_SHIFT)

/* RXDAMOUNT Register */

#define TWIM_RXDAMOUNT_SHIFT                (0)        /* Bits 0-15: Number of bytes transferred in the last transaction */
#define TWIM_RXDAMOUNT_MASK                 (0xffff << TWIM_RXDAMOUNT_SHIFT)

/* TXDMAXCNT Register */

#define TWIM_TXDMAXCNT_SHIFT                (0)        /* Bits 0-15: Maximum number of bytes in transmit buffer */
#define TWIM_TXDMAXCNT_MASK                 (0xffff << TWIM_TXDMAXCNT_SHIFT)

/* TXDAMOUNT Register */

#define TWIM_TXDAMOUNT_SHIFT                (0)        /* Bits 0-15: Number of bytes transferred in the last transaction */
#define TWIM_TXDAMOUNT_MASK                 (0xffff << TWIM_TXDAMOUNT_SHIFT)

/* ADDRESS Register */

#define TWIM_ADDRESS_SHIFT                  (0)        /* Bits 0-6: Address used in the TWI transfer */
#define TWIM_ADDRESS_MASK                   (0x7f << TWIM_ADDRESS_SHIFT)

/* DPPI subscribe and publish registers */

#define TWIM_DPPI_CHIDX_SHIFT               (0)
#define TWIM_DPPI_CHIDX_MASK                (0xff << TWIM_DPPI_CHIDX_SHIFT)
#define TWIM_DPPI_EN                        (1 << 31)

/* DMA registers */

#define TWIM_DMA_LIST_DISABLED              (0)
#define TWIM_DMA_LIST_ARRAYLIST             (1)
#define TWIM_DMA_TERMINATEONBUSERROR        (1)
#define TWIM_DMA_MATCH_ENABLE(n)            (1 << (n))
#define TWIM_DMA_MATCH_ONESHOT(n)           (1 << (16 + (n)))
#define TWIM_DMA_MATCH_CANDIDATE_MASK       (0xff)

/* Pattern matcher shortcuts */

#define TWIM_SHORTS_MATCH_ENABLE(n)         (1 << (21 + (n)))
#define TWIM_SHORTS_MATCH_DISABLE(n)        (1 << (25 + (n)))

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_TWI_H */
