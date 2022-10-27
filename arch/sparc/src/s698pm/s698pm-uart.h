/****************************************************************************
 * arch/sparc/src/s698pm/s698pm-uart.h
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

#ifndef __ARCH_SPARC_SRC_S698PM_S698PM_UART_H
#define __ARCH_SPARC_SRC_S698PM_S698PM_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *************************************************^*******/
#define S698PM_UART_TXREG_OFFSET    0x0000 /* UARTx transmit register */
#define S698PM_UART_RXREG_OFFSET    0x0000 /* UARTx receive register */
#define S698PM_UART_STATREG_OFFSET  0x0004 /* UARTx status register */
#define S698PM_UART_CTRLREG_OFFSET  0x0008 /* UARTx control register */
#define S698PM_UART_SCALREG_OFFSET  0x000c /* UARTx scaler register */

/* Register Addresses *******************************************************/
#if CHIP_NUARTS > 0
#define S698PM_UART1_TXREG    (S698PM_UART1_BASE+S698PM_UART_TXREG_OFFSET)
#define S698PM_UART1_RXREG    (S698PM_UART1_BASE+S698PM_UART_RXREG_OFFSET)
#define S698PM_UART1_STATREG  (S698PM_UART1_BASE+S698PM_UART_STATREG_OFFSET)
#define S698PM_UART1_CTRLREG  (S698PM_UART1_BASE+S698PM_UART_CTRLREG_OFFSET)
#define S698PM_UART1_SCALREG  (S698PM_UART1_BASE+S698PM_UART_SCALREG_OFFSET)
#endif

#if CHIP_NUARTS > 1
#define S698PM_UART2_TXREG    (S698PM_UART2_BASE+S698PM_UART_TXREG_OFFSET)
#define S698PM_UART2_RXREG    (S698PM_UART2_BASE+S698PM_UART_RXREG_OFFSET)
#define S698PM_UART2_STATREG  (S698PM_UART2_BASE+S698PM_UART_STATREG_OFFSET)
#define S698PM_UART2_CTRLREG  (S698PM_UART2_BASE+S698PM_UART_CTRLREG_OFFSET)
#define S698PM_UART2_SCALREG  (S698PM_UART2_BASE+S698PM_UART_SCALREG_OFFSET)
#endif

#if CHIP_NUARTS > 2
#define S698PM_UART3_TXREG    (S698PM_UART3_BASE+S698PM_UART_TXREG_OFFSET)
#define S698PM_UART3_RXREG    (S698PM_UART3_BASE+S698PM_UART_RXREG_OFFSET)
#define S698PM_UART3_STATREG  (S698PM_UART3_BASE+S698PM_UART_STATREG_OFFSET)
#define S698PM_UART3_CTRLREG  (S698PM_UART3_BASE+S698PM_UART_CTRLREG_OFFSET)
#define S698PM_UART3_SCALREG  (S698PM_UART3_BASE+S698PM_UART_SCALREG_OFFSET)
#endif

#if CHIP_NUARTS > 3
#define S698PM_UART4_TXREG    (S698PM_UART4_BASE+S698PM_UART_TXREG_OFFSET)
#define S698PM_UART4_RXREG    (S698PM_UART4_BASE+S698PM_UART_RXREG_OFFSET)
#define S698PM_UART4_STATREG  (S698PM_UART4_BASE+S698PM_UART_STATREG_OFFSET)
#define S698PM_UART4_CTRLREG  (S698PM_UART4_BASE+S698PM_UART_CTRLREG_OFFSET)
#define S698PM_UART4_SCALREG  (S698PM_UART4_BASE+S698PM_UART_SCALREG_OFFSET)
#endif

/* Register Bit-Field Definitions *******************************************/

#define ODD	1
#define EVEN    0
#define ON 	1
#define OFF     0
#define NONE    2
#define RX      0
#define TX      1
#define RXTX    3

/** Uart control list - Mask */

#define MSK_UART_ENABLE_RX    0x01
#define MSK_UART_ENABLE_TX    0x02
#define MSK_UART_ENABLE_RXIT  0x04
#define MSK_UART_ENABLE_TXIT  0x08
#define MSK_UART_PAR          0x10
#define MSK_UART_ENABLE_PAR   0x20
#define MSK_UART_ENABLE_FLOW  0x40
#define MSK_UART_LOOPBACK     0x80
#define MSK_UART_CLOCK        0x100
#define MSK_UART_ALLINTS      0x0C

/* UARTx status and control register */

#define UART_STA_DR           (1 << 0)  /* Bit 0: Receive buffer data available */
#define UART_STA_TS           (1 << 1)  /* Bit 1: Transmit shift register is empty */
#define UART_STA_TE           (1 << 2)  /* Bit 2: TX buffer empty */
#define UART_STA_BR           (1 << 3)  /* Bit 3: Transmit break */
#define UART_STA_OV           (1 << 4)  /* Bit 4: overflow error status */
#define UART_STA_PE           (1 << 5)  /* Bit 5: Parity error status */
#define UART_STA_FE           (1 << 6)  /* Bit 6: Framing error status */
#define UART_STA_TH           (1 << 7)  /* Bit 7: TX buffer 1/2 full */
#define UART_STA_RH           (1 << 8)  /* Bit 8: RX buffer 1/2 full */
#define UART_STA_TF           (1 << 9)  /* Bit 9: Transmit buffer full status */
#define UART_STA_RF           (1 << 10) /* Bit 10: Receive buffer full status */
#define REG_STAT_TX_CNT       (0x3f << 20)
#define REG_STAT_RX_CNT       (0x3f << 26)

/* UARTx transmit register */

#define UART_TXREG_MASK             0xff

/* UARTx receive register */

#define UART_RXREG_MASK             0xff

/* UARTx baud rate register */

#define UART_BRG_MASK               0xfff

#define uart_set_baudrate(baudrate)	         ((uint32_t)((((BOARD_CPU_CLOCK*10)/(baudrate * 8))-5)/10))

#define uart_parity_config(reg, uart_parity)     ((uart_parity ==  ODD) ?                                     \
                                                  (reg   =   ((reg |  MSK_UART_PAR) | MSK_UART_ENABLE_PAR)) : \
                                                  ((uart_parity  ==  EVEN) ?                                  \
                                                   (reg =   ((reg & ~MSK_UART_PAR) | MSK_UART_ENABLE_PAR)) :  \
                                                   (reg &= ~MSK_UART_ENABLE_PAR )                             \
                                                  )                                                           \
                                                 )

#define Uart_interrupt_config(reg, uart_its)     ((uart_its == RXTX) ?                                         \
                                                  (reg |= (MSK_UART_ENABLE_RXIT | MSK_UART_ENABLE_TXIT)) :     \
                                                  ((uart_its == RX ) ?                                         \
                                                   (reg |= (MSK_UART_ENABLE_RXIT & ~MSK_UART_ENABLE_TXIT)) :   \
                                                   ((uart_its ==  TX) ?                                        \
                                                    (reg |= (MSK_UART_ENABLE_TXIT & ~MSK_UART_ENABLE_RXIT)) :  \
                                                    (reg &= ~(MSK_UART_ENABLE_RXIT |  MSK_UART_ENABLE_TXIT))   \
                                                   )                                                           \
                                                  )                                                            \
                                                 ) 

#define uart_flow_ctrl_config(reg, uart_flow)    ((uart_flow    ==  ON) ?         \
                                                  (reg |= MSK_UART_ENABLE_FLOW) : \
                                                  (reg &= ~MSK_UART_ENABLE_FLOW)  \
                                                 )      

#define uart_loopback_config(reg, uart_loopb)    ((uart_loopb == ON) ?            \
                                                  (reg |=  MSK_UART_LOOPBACK) :   \
                                                  (reg &= ~MSK_UART_LOOPBACK)     \
                                                 )    

#define uart_enable(reg)                         (reg |=  (MSK_UART_ENABLE_RX | MSK_UART_ENABLE_TX))
#define uart_disable(reg)                        (reg &= ~(MSK_UART_ENABLE_RX | MSK_UART_ENABLE_TX))

#define uart_tx_ready()                     ((S698PM_REG.uart_status1 & UART_STA_TF) != UART_STA_TF)
#define uart_rx_ready()                     ((S698PM_REG.uart_status1 & UART_STA_DR) == UART_STA_DR)

#define uart_send_byte(ch)  		     (S698PM_REG.uart_data1 = ch)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

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

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_SPARC_SRC_S698PM_S698PM_UART_H */
