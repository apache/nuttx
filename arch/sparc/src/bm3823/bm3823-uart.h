/****************************************************************************
 * arch/sparc/src/bm3823/bm3823-uart.h
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

#ifndef __ARCH_SPARC_SRC_BM3823_BM3823_UART_H
#define __ARCH_SPARC_SRC_BM3823_BM3823_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define BM3823_UART_TXREG_OFFSET    0x0000 /* UARTx transmit register */
#define BM3823_UART_RXREG_OFFSET    0x0000 /* UARTx receive register */
#define BM3823_UART_STATREG_OFFSET  0x0004 /* UARTx status register */
#define BM3823_UART_CTRLREG_OFFSET  0x0008 /* UARTx control register */
#define BM3823_UART_SCALREG_OFFSET  0x000c /* UARTx scaler register */

/* Register Addresses *******************************************************/

#if CHIP_NUARTS > 0
#define BM3823_UART1_TXREG     (BM3823_UART1_BASE+BM3823_UART_TXREG_OFFSET)
#define BM3823_UART1_RXREG     (BM3823_UART1_BASE+BM3823_UART_RXREG_OFFSET)
#define BM3823_UART1_STATREG   (BM3823_UART1_BASE+BM3823_UART_STATREG_OFFSET)
#define BM3823_UART1_CTRLREG   (BM3823_UART1_BASE+BM3823_UART_CTRLREG_OFFSET)
#define BM3823_UART1_SCALREG   (BM3823_UART1_BASE+BM3823_UART_SCALREG_OFFSET)
#endif

#if CHIP_NUARTS > 1
#define BM3823_UART2_TXREG     (BM3823_UART2_BASE+BM3823_UART_TXREG_OFFSET)
#define BM3823_UART2_RXREG     (BM3823_UART2_BASE+BM3823_UART_RXREG_OFFSET)
#define BM3823_UART2_STATREG   (BM3823_UART2_BASE+BM3823_UART_STATREG_OFFSET)
#define BM3823_UART2_CTRLREG   (BM3823_UART2_BASE+BM3823_UART_CTRLREG_OFFSET)
#define BM3823_UART2_SCALREG   (BM3823_UART2_BASE+BM3823_UART_SCALREG_OFFSET)
#endif

#if CHIP_NUARTS > 2
#define BM3823_UART3_TXREG     (BM3823_UART3_BASE+BM3823_UART_TXREG_OFFSET)
#define BM3823_UART3_RXREG     (BM3823_UART3_BASE+BM3823_UART_RXREG_OFFSET)
#define BM3823_UART3_STATREG   (BM3823_UART3_BASE+BM3823_UART_STATREG_OFFSET)
#define BM3823_UART3_CTRLREG   (BM3823_UART3_BASE+BM3823_UART_CTRLREG_OFFSET)
#define BM3823_UART3_SCALREG   (BM3823_UART3_BASE+BM3823_UART_SCALREG_OFFSET)
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

/* Uart control list - Mask */

#define MSK_UART_ENABLE_RX    0x01
#define MSK_UART_ENABLE_TX    0x02
#define MSK_UART_ENABLE_RXIT  0x04
#define MSK_UART_ENABLE_TXIT  0x08
#define MSK_UART_ENABLE_PAR   0x20
#define MSK_UART_PAR          0x10
#define MSK_UART_ENABLE_FLOW  0x40
#define MSK_UART_LOOPBACK     0x80
#define MSK_UART_CLOCK        0x100
#define MSK_UART_ALLINTS      0x0C

#define MSK_UART_DATA_READY   0x01
#define MSK_UART_TXS_READY    0x02
#define MSK_UART_TXH_READY    0x04
#define MSK_UART_BREAK        0x08
#define MSK_UART_OVERRUN      0x10
#define MSK_UART_PAR_ERR      0x20
#define MSK_UART_FRAME_ERR    0x40

/* UARTx transmit register */

#define UART_TXREG_MASK             0xff

/* UARTx receive register */

#define UART_RXREG_MASK             0xff

/* UARTx baud rate register */

#define UART_BRG_MASK               0xfff

#define uart1_set_baudrate(baudrate)	    (EXTER_REG.uart_scaler1 = (uint32_t)((((BOARD_PERIPH_CLOCK*10)/(baudrate * 8))-5)/10))

#define uart1_parity_config(uart_parity)     ( uart_parity    ==  ODD                                                      \
                                            ? (EXTER_REG.uart_ctrl1   =   ((EXTER_REG.uart_ctrl1 |  MSK_UART_PAR) | MSK_UART_ENABLE_PAR)) \
                                            : ( uart_parity  ==  EVEN                                                     \
                                              ? (EXTER_REG.uart_ctrl1 =   ((EXTER_REG.uart_ctrl1 & ~MSK_UART_PAR) | MSK_UART_ENABLE_PAR)) \
                                              : (EXTER_REG.uart_ctrl1 &= ~MSK_UART_ENABLE_PAR )                                   \
                                              )                                                                           \
                                            )

#define uart1_flow_ctrl_config(uart_flow)    ( uart_flow    ==  ON                     \
                                            ? (EXTER_REG.uart_ctrl1 |=  MSK_UART_ENABLE_FLOW) \
                                            : (EXTER_REG.uart_ctrl1 &= ~MSK_UART_ENABLE_FLOW) \
                                            )

#define uart1_loopback_config(uart_loopb)    ( uart_loopb   ==  ON                     \
                                            ? (EXTER_REG.uart_ctrl1 |=  MSK_UART_LOOPBACK)    \
                                            : (EXTER_REG.uart_ctrl1 &= ~MSK_UART_LOOPBACK)    \
                                            )

#define uart1_enable()                       (EXTER_REG.uart_ctrl1 |=  (MSK_UART_ENABLE_RX | MSK_UART_ENABLE_TX))
#define uart1_disable()                      (EXTER_REG.uart_ctrl1 &= ~(MSK_UART_ENABLE_RX | MSK_UART_ENABLE_TX))

#define uart1_tx_ready()                     ((EXTER_REG.uart_status1 & MSK_UART_TXH_READY)  == MSK_UART_TXH_READY )
#define uart1_rx_ready()                     ((EXTER_REG.uart_status1 & MSK_UART_DATA_READY) == MSK_UART_DATA_READY)

#define uart1_send_byte(ch)  		     (EXTER_REG.uart_data1 = ch)

/****************************************************************************/

#define uart2_set_baudrate(baudrate)	    (EXTER_REG.uart_scaler2 = (((BOARD_PERIPH_CLOCK*10)/(baudrate*8))-5)/10)

#define uart2_parity_config(uart_parity)     ( uart_parity    ==  ODD                                                      \
                                            ? (EXTER_REG.uart_ctrl2   =   ((EXTER_REG.uart_ctrl2 |  MSK_UART_PAR) | MSK_UART_ENABLE_PAR)) \
                                            : ( uart_parity  ==  EVEN                                                     \
                                              ? (EXTER_REG.uart_ctrl2 =   ((EXTER_REG.uart_ctrl2 & ~MSK_UART_PAR) | MSK_UART_ENABLE_PAR)) \
                                              : (EXTER_REG.uart_ctrl2 &= ~MSK_UART_ENABLE_PAR )                                   \
                                              )                                                                           \
                                            )

#define uart2_flow_ctrl_config(uart_flow)    ( uart_flow    ==  ON                     \
                                            ? (EXTER_REG.uart_ctrl2 |=  MSK_UART_ENABLE_FLOW) \
                                            : (EXTER_REG.uart_ctrl2 &= ~MSK_UART_ENABLE_FLOW) \
                                            )

#define uart2_loopback_config(uart_loopb)    ( uart_loopb   ==  ON                     \
                                            ? (EXTER_REG.uart_ctrl2 |=  MSK_UART_LOOPBACK)    \
                                            : (EXTER_REG.uart_ctrl2 &= ~MSK_UART_LOOPBACK)    \
                                            )

#define uart2_enable()                       (EXTER_REG.uart_ctrl2 |=  (MSK_UART_ENABLE_RX | MSK_UART_ENABLE_TX))
#define uart2_disable()                      (EXTER_REG.uart_ctrl2 &= ~(MSK_UART_ENABLE_RX | MSK_UART_ENABLE_TX))

#define uart2_tx_ready()                     ((EXTER_REG.uart_status2 & MSK_UART_TXH_READY)  == MSK_UART_TXH_READY )
#define uart2_rx_ready()                     ((EXTER_REG.uart_status2 & MSK_UART_DATA_READY) == MSK_UART_DATA_READY)

#define uart2_send_byte(ch)  		     (EXTER_REG.uart_data2 = ch)

/****************************************************************************/

#define uart3_set_baudrate(baudrate)	    (BM3823_REG.uart_scaler3 = (((BOARD_PERIPH_CLOCK*10)/(baudrate*8))-5)/10)

#define uart3_parity_config(uart_parity)     ( uart_parity    ==  ODD                                                      \
                                            ? (BM3823_REG.uart_ctrl3   =   ((BM3823_REG.uart_ctrl3 |  MSK_UART_PAR) | MSK_UART_ENABLE_PAR)) \
                                            : ( uart_parity  ==  EVEN                                                     \
                                              ? (BM3823_REG.uart_ctrl3 =   ((BM3823_REG.uart_ctrl3 & ~MSK_UART_PAR) | MSK_UART_ENABLE_PAR)) \
                                              : (BM3823_REG.uart_ctrl3 &= ~MSK_UART_ENABLE_PAR )                                   \
                                              )                                                                           \
                                            )

#define uart3_flow_ctrl_config(uart_flow)    ( uart_flow    ==  ON                     \
                                            ? (BM3823_REG.uart_ctrl3 |=  MSK_UART_ENABLE_FLOW) \
                                            : (BM3823_REG.uart_ctrl3 &= ~MSK_UART_ENABLE_FLOW) \
                                            )

#define uart3_loopback_config(uart_loopb)    ( uart_loopb   ==  ON                     \
                                            ? (BM3823_REG.uart_ctrl3 |=  MSK_UART_LOOPBACK)    \
                                            : (BM3823_REG.uart_ctrl3 &= ~MSK_UART_LOOPBACK)    \
                                            )

#define uart3_enable()                       (BM3823_REG.uart_ctrl3 |=  (MSK_UART_ENABLE_RX | MSK_UART_ENABLE_TX))
#define uart3_disable()                      (BM3823_REG.uart_ctrl3 &= ~(MSK_UART_ENABLE_RX | MSK_UART_ENABLE_TX))

#define uart3_tx_ready()                     ((BM3823_REG.uart_status3 & MSK_UART_TXH_READY)  == MSK_UART_TXH_READY )
#define uart3_rx_ready()                     ((BM3823_REG.uart_status3 & MSK_UART_DATA_READY) == MSK_UART_DATA_READY)

#define uart3_send_byte(ch)  		     (BM3823_REG.uart_data3 = ch)

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
#endif /* __ARCH_SPARC_SRC_BM3823_BM3823_UART_H */
