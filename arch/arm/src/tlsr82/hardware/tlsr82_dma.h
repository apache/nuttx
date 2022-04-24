/****************************************************************************
 * arch/arm/src/tlsr82/hardware/tlsr82_dma.h
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

#ifndef __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_DMA_H
#define __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/tlsr82/chip.h>

#include "hardware/tlsr82_register.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA address, size, mode resigster definitions
 * By default, DMA0 is for Uart RX
 * By default, DMA1 is for Uart TX
 * By default, DMA2 is for RF RX
 * By default, DMA3,4,5 is for RF TX
 * By default, DMA7 is for PWM
 */

#define DMA0_ADDR_REG              REG_ADDR16(0xc00)
#define DMA0_ADDRHI_REG            REG_ADDR8(0xc40)
#define DMA0_SIZE_REG              REG_ADDR8(0xc02)
#define DMA0_MODE_REG              REG_ADDR8(0xc03)
#define DMA_UART_RX_ADDR_REG       DMA0_ADDR_REG
#define DMA_UART_RX_ADDRHI_REG     DMA0_ADDRHI_REG
#define DMA_UART_RX_SIZE_REG       DMA0_SIZE_REG
#define DMA_UART_RX_MODE_REG       DMA0_MODE_REG

#define DMA1_ADDR_REG              REG_ADDR16(0xc04)
#define DMA1_ADDRHI_REG            REG_ADDR8(0xc41)
#define DMA1_SIZE_REG              REG_ADDR8(0xc06)
#define DMA1_MODE_REG              REG_ADDR8(0xc07)
#define DMA_UART_TX_ADDR_REG       DMA1_ADDR_REG
#define DMA_UART_TX_ADDRHI_REG     DMA1_ADDRHI_REG
#define DMA_UART_TX_SIZE_REG       DMA1_SIZE_REG
#define DMA_UART_TX_MODE_REG       DMA1_MODE_REG

#define DMA2_ADDR_REG              REG_ADDR16(0xc08)
#define DMA2_SIZE_REG              REG_ADDR8(0xc0a)
#define DMA2_MODE_REG              REG_ADDR8(0xc0b)

#define DMA3_ADDR_REG              REG_ADDR16(0xc0c)
#define DMA3_SIZE_REG              REG_ADDR8(0xc0e)
#define DMA3_MODE_REG              REG_ADDR8(0xc0f)

#define DMA4_ADDR_REG              REG_ADDR16(0xc10)
#define DMA4_SIZE_REG              REG_ADDR8(0xc16)
#define DMA4_MODE_REG              REG_ADDR8(0xc17)

#define DMA5_ADDR_REG              REG_ADDR16(0xc14)
#define DMA5_SIZE_REG              REG_ADDR8(0xc16)
#define DMA5_MODE_REG              REG_ADDR8(0xc17)

#define DMA7_ADDR_REG              REG_ADDR16(0xc18)
#define DMA7_SIZE_REG              REG_ADDR8(0xc1a)
#define DMA7_MODE_REG              REG_ADDR8(0xc1b)

/* DMA realdy register definitions */

#define DMA_TX_RDY0_REG            REG_ADDR8(0xc24)
#define DMA_TX_RDY1_REG            REG_ADDR8(0xc25)
#define DMA_RX_RDY0_REG            REG_ADDR8(0xc26)
#define DMA_RX_RDY1_REG            REG_ADDR8(0xc27)

/* DMA irq register definitions
 * DMA_IRQ_MASK_REG: enable or disable the dma interrupt
 * DMA_IRQ_EN_REG  : enable or disable the dma channel
 * DMA_IRQ_STA_REG : get the dma interupt status and write 1 to clear
 */

#define DMA_IRQ_MASK_REG           REG_ADDR8(0xc21)
#define DMA_IRQ_EN_REG             REG_ADDR8(0xc20)
#define DMA_IRQ_STA_REG            REG_ADDR8(0xc26)

/* DMA Channel */

#define DMA_CHAN0                  BIT(0)
#define DMA_CHAN1                  BIT(1)
#define DMA_CHAN2                  BIT(2)
#define DMA_CHAN3                  BIT(3)
#define DMA_CHAN4                  BIT(4)
#define DMA_CHAN5                  BIT(5)
#define DMA_CHAN7                  BIT(7)

#define DMA_CHAN_UART_RX           DMA_CHAN0
#define DMA_CHAN_UART_TX           DMA_CHAN1
#define DMA_CHAN_RF_RX             DMA_CHAN2
#define DMA_CHAN_RF_TX             DMA_CHAN3
#define DMA_CHAN_AES_OUT           DMA_CHAN4
#define DMA_CHAN_AES_IN            DMA_CHAN5
#define DMA_CHAN_PWM               DMA_CHAN7

/* DMA Mode */

#define DMA_MODE_WR_MEM            BIT(0)
#define DMA_MODE_PINGPONG_EN       BIT(1)
#define DMA_MODE_FIFO_EN           BIT(2)
#define DMA_MODE_AUTO_MODE         BIT(3)
#define DMA_MODE_READ_MODE         BIT(4)
#define DMA_MODE_BYTE_MODE         BIT(5)

#endif /* __ARCH_ARM_SRC_TLSR82_HARDWARE_TLSR82_DMA_H */
