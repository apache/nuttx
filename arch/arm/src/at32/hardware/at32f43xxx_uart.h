/****************************************************************************
 * arch/arm/src/at32/hardware/at32f43xxx_uart.h
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

#ifndef __ARCH_ARM_SRC_AT32_HARDWARE_AT32F43XXX_UART_H
#define __ARCH_ARM_SRC_AT32_HARDWARE_AT32F43XXX_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define AT32_USART_STS_OFFSET               (0x00) /* Status register */
#define AT32_USART_DT_OFFSET                (0x04) /* Data register */
#define AT32_USART_BAUDR_OFFSET             (0x08) /* Baud Rate register */
#define AT32_USART_CTRL1_OFFSET             (0x0c) /* Control register 1 */
#define AT32_USART_CTRL2_OFFSET             (0x10) /* Control register 2 */
#define AT32_USART_CTRL3_OFFSET             (0x14) /* Control register 3 */
#define AT32_USART_GDIV_OFFSET              (0x18) /* Guard time register */

/* Register Addresses *******************************************************/

#if AT32_NUSART > 0
#  define AT32_USART1_STS                   (AT32_USART1_BASE+AT32_USART_STS_OFFSET)
#  define AT32_USART1_DT                    (AT32_USART1_BASE+AT32_USART_DT_OFFSET)
#  define AT32_USART1_BAUDR                 (AT32_USART1_BASE+AT32_USART_BAUDR_OFFSET)
#  define AT32_USART1_CTRL1                 (AT32_USART1_BASE+AT32_USART_CTRL1_OFFSET)
#  define AT32_USART1_CTRL2                 (AT32_USART1_BASE+AT32_USART_CTRL2_OFFSET)
#  define AT32_USART1_CTRL3                 (AT32_USART1_BASE+AT32_USART_CTRL3_OFFSET)
#  define AT32_USART1_GDIV                  (AT32_USART1_BASE+AT32_USART_GDIV_OFFSET)
#endif

#if AT32_NUSART > 1
#  define AT32_USART2_STS                   (AT32_USART2_BASE+AT32_USART_STS_OFFSET)
#  define AT32_USART2_DT                    (AT32_USART2_BASE+AT32_USART_DT_OFFSET)
#  define AT32_USART2_BAUDR                 (AT32_USART2_BASE+AT32_USART_BAUDR_OFFSET)
#  define AT32_USART2_CTRL1                 (AT32_USART2_BASE+AT32_USART_CTRL1_OFFSET)
#  define AT32_USART2_CTRL2                 (AT32_USART2_BASE+AT32_USART_CTRL2_OFFSET)
#  define AT32_USART2_CTRL3                 (AT32_USART2_BASE+AT32_USART_CTRL3_OFFSET)
#  define AT32_USART2_GDIV                  (AT32_USART2_BASE+AT32_USART_GDIV_OFFSET)
#endif

#if AT32_NUSART > 2
#  define AT32_USART3_STS                   (AT32_USART3_BASE+AT32_USART_STS_OFFSET)
#  define AT32_USART3_DT                    (AT32_USART3_BASE+AT32_USART_DT_OFFSET)
#  define AT32_USART3_BAUDR                 (AT32_USART3_BASE+AT32_USART_BAUDR_OFFSET)
#  define AT32_USART3_CTRL1                 (AT32_USART3_BASE+AT32_USART_CTRL1_OFFSET)
#  define AT32_USART3_CTRL2                 (AT32_USART3_BASE+AT32_USART_CTRL2_OFFSET)
#  define AT32_USART3_CTRL3                 (AT32_USART3_BASE+AT32_USART_CTRL3_OFFSET)
#  define AT32_USART3_GDIV                  (AT32_USART3_BASE+AT32_USART_GDIV_OFFSET)
#endif

#if AT32_NUSART > 3
#  define AT32_UART4_STS                    (AT32_UART4_BASE+AT32_USART_STS_OFFSET)
#  define AT32_UART4_DT                     (AT32_UART4_BASE+AT32_USART_DT_OFFSET)
#  define AT32_UART4_BAUDR                  (AT32_UART4_BASE+AT32_USART_BAUDR_OFFSET)
#  define AT32_UART4_CTRL1                  (AT32_UART4_BASE+AT32_USART_CTRL1_OFFSET)
#  define AT32_UART4_CTRL2                  (AT32_UART4_BASE+AT32_USART_CTRL2_OFFSET)
#  define AT32_UART4_CTRL3                  (AT32_UART4_BASE+AT32_USART_CTRL3_OFFSET)
#  define AT32_UART4_GDIV                   (AT32_UART4_BASE+AT32_USART_GDIV_OFFSET)
#endif

#if AT32_NUSART > 4
#  define AT32_UART5_STS                    (AT32_UART5_BASE+AT32_USART_STS_OFFSET)
#  define AT32_UART5_DT                     (AT32_UART5_BASE+AT32_USART_DT_OFFSET)
#  define AT32_UART5_BAUDR                  (AT32_UART5_BASE+AT32_USART_BAUDR_OFFSET)
#  define AT32_UART5_CTRL1                  (AT32_UART5_BASE+AT32_USART_CTRL1_OFFSET)
#  define AT32_UART5_CTRL2                  (AT32_UART5_BASE+AT32_USART_CTRL2_OFFSET)
#  define AT32_UART5_CTRL3                  (AT32_UART5_BASE+AT32_USART_CTRL3_OFFSET)
#  define AT32_UART5_GDIV                   (AT32_UART5_BASE+AT32_USART_GDIV_OFFSET)
#endif

#if AT32_NUSART > 5
#  define AT32_USART6_STS                   (AT32_USART6_BASE+AT32_USART_STS_OFFSET)
#  define AT32_USART6_DT                    (AT32_USART6_BASE+AT32_USART_DT_OFFSET)
#  define AT32_USART6_BAUDR                 (AT32_USART6_BASE+AT32_USART_BAUDR_OFFSET)
#  define AT32_USART6_CTRL1                 (AT32_USART6_BASE+AT32_USART_CTRL1_OFFSET)
#  define AT32_USART6_CTRL2                 (AT32_USART6_BASE+AT32_USART_CTRL2_OFFSET)
#  define AT32_USART6_CTRL3                 (AT32_USART6_BASE+AT32_USART_CTRL3_OFFSET)
#  define AT32_USART6_GDIV                  (AT32_USART6_BASE+AT32_USART_GDIV_OFFSET)
#endif

#if AT32_NUSART > 6
#  define AT32_UART7_STS                    (AT32_UART7_BASE+AT32_USART_STS_OFFSET)
#  define AT32_UART7_DT                     (AT32_UART7_BASE+AT32_USART_DT_OFFSET)
#  define AT32_UART7_BAUDR                  (AT32_UART7_BASE+AT32_USART_BAUDR_OFFSET)
#  define AT32_UART7_CTRL1                  (AT32_UART7_BASE+AT32_USART_CTRL1_OFFSET)
#  define AT32_UART7_CTRL2                  (AT32_UART7_BASE+AT32_USART_CTRL2_OFFSET)
#  define AT32_UART7_CTRL3                  (AT32_UART7_BASE+AT32_USART_CTRL3_OFFSET)
#  define AT32_UART7_GDIV                   (AT32_UART7_BASE+AT32_USART_GDIV_OFFSET)
#endif

#if AT32_NUSART > 7
#  define AT32_UART8_STS                    (AT32_UART8_BASE+AT32_USART_STS_OFFSET)
#  define AT32_UART8_DT                     (AT32_UART8_BASE+AT32_USART_DT_OFFSET)
#  define AT32_UART8_BAUDR                  (AT32_UART8_BASE+AT32_USART_BAUDR_OFFSET)
#  define AT32_UART8_CTRL1                  (AT32_UART8_BASE+AT32_USART_CTRL1_OFFSET)
#  define AT32_UART8_CTRL2                  (AT32_UART8_BASE+AT32_USART_CTRL2_OFFSET)
#  define AT32_UART8_CTRL3                  (AT32_UART8_BASE+AT32_USART_CTRL3_OFFSET)
#  define AT32_UART8_GDIV                   (AT32_UART8_BASE+AT32_USART_GDIV_OFFSET)
#endif

/* Register Bitfield Definitions ********************************************/

/* Status register */

#define USART_STS_PERR                      (1 << 0) /* Parity error */
#define USART_STS_FERR                      (1 << 1) /* Framing error */
#define USART_STS_NERR                      (1 << 2) /* Noise error */
#define USART_STS_ROERR                     (1 << 3) /* Receiver overflow error */
#define USART_STS_IDLEF                     (1 << 4) /* Idle flag */
#define USART_STS_RDBF                      (1 << 5) /* Receive data buffer full */
#define USART_STS_TDC                       (1 << 6) /* Transmit data complete */
#define USART_STS_TDBE                      (1 << 7) /* Transmit data buffer empty */
#define USART_STS_BFF                       (1 << 8) /* break frame flag */
#define USART_STS_CTSCF                     (1 << 9) /* CTS change flag */

#define USART_STS_ALLBITS                   (0x03ff)
#define USART_STS_CLRBITS                   (USART_STS_CTSCF | USART_STS_BFF)

/* Data register */

#define USART_DT_SHIFT                      (0)       /* Data value */
#define USART_DT_MASK                       (0xff << USART_DT_SHIFT)

/* Baud Rate Register */

#define USART_BAUDR_DIV_SHIFT               (0) /* Baud Rate division */
#define USART_BAUDR_DIV_MASK                (0xffff << USART_BAUDR_DIV_SHIFT)

/* Control register 1 */

#define USART_CTRL1_SBF                     (1 << 0)  /* Send break frame */
#define USART_CTRL1_RM                      (1 << 1)  /* Receiver mute */
#define USART_CTRL1_REN                     (1 << 2)  /* Receiver enable */
#define USART_CTRL1_TEN                     (1 << 3)  /* Transmitter enable */
#define USART_CTRL1_IDLEIEN                 (1 << 4)  /* IDLE interrupt enable */
#define USART_CTRL1_RDBFIEN                 (1 << 5)  /* RDBF interrupt enable */
#define USART_CTRL1_TDCIEN                  (1 << 6)  /* TDC interrupt enable */
#define USART_CTRL1_TDBEIEN                 (1 << 7)  /* TDBE interrupt enable */
#define USART_CTRL1_PERRIEN                 (1 << 8)  /* PERR interrupt enable */
#define USART_CTRL1_PSEL                    (1 << 9)  /* Parity selection */
#define USART_CTRL1_PEN                     (1 << 10) /* Parity enable */
#define USART_CTRL1_WUM                     (1 << 11) /* Wake up mode */
#define USART_CTRL1_DBN0                    (1 << 12) /* Data bit num */
#define USART_CTRL1_UEN                     (1 << 13) /* USART enable */

#define USART_CTRL1_TCDT_SHIFT              (16)  /* transmit complete delay time */
#define USART_CTRL1_TCDT_MASK               (31 << USART_CTRL1_TCDT_SHIFT)
#define USART_CTRL1_TCDT(X)                 ((X) << USART_CTRL1_TCDT_SHIFT)
#define USART_CTRL1_TSDT_SHIFT              (21) /* transmit start delay time */
#define USART_CTRL1_TSDT_MASK               (31 << USART_CTRL1_TSDT_SHIFT) 
#define USART_CTRL1_TSDT(X)                 ((X) << USART_CTRL1_TSDT_SHIFT)
#define USART_CTRL1_DBN1                    (1 << 28) /* Data bit num */

#define USART_CTRL1_ALLINTS                 (USART_CTRL1_IDLEIEN|USART_CTRL1_RDBFIEN|USART_CTRL1_TDCIEN|USART_CTRL1_PERRIEN)
/* Control register 2 */

#define USART_CTRL2_ID_L_SHIFT              (0) /* USART identification low */
#define USART_CTRL2_ID_L_MASK               (15 << USART_CTRL2_ID_L_SHIFT)
#define USART_CTRL2_ID_L(X)                 ((X) << USART_CTRL2_ID_L_SHIFT)
#define USART_CTRL2_IDBN                    (1 << 4)  /* Identification bit num */
#define USART_CTRL2_BFBN                    (1 << 5)  /* break frame bit num */
#define USART_CTRL2_BFIEN                   (1 << 6)  /* break frame interrupt enable */
#define USART_CTRL2_LBCP                    (1 << 8)  /* Last bit clock pulse */
#define USART_CTRL2_CLKPHA                  (1 << 9)  /* Clock phase */
#define USART_CTRL2_CLKPOL                  (1 << 10) /* Clock polarity */
#define USART_CTRL2_CLKEN                   (1 << 11) /* Clock enable */

#define USART_CTRL2_STOPBN_SHIFT            (12) /* STOP bit num */
#define USART_CTRL2_STOPBN_MASK             (3 << USART_CTRL2_STOPBN_SHIFT)
#  define USART_CTRL2_STOPBN_10             (0 << USART_CTRL2_STOPBN_SHIFT) /* 1 stop */
#  define USART_CTRL2_STOPBN_05             (1 << USART_CTRL2_STOPBN_SHIFT) /* 0.5 stop */
#  define USART_CTRL2_STOPBN_20             (2 << USART_CTRL2_STOPBN_SHIFT) /* 2 stop */
#  define USART_CTRL2_STOPBN_15             (3 << USART_CTRL2_STOPBN_SHIFT) /* 1.5 stop */

#define USART_CTRL2_LINEN                   (1 << 14) /* LIN mode enable */
#define USART_CTRL2_TRPSWAP                 (1 << 15) /* Transmit receive pin swap */

#define USART_CTRL2_ID_H_SHIFT              (28) /* USART identification high */
#define USART_CTRL2_ID_H_MASK               (31 << USART_CTRL2_ID_H_SHIFT)
#define USART_CTRL2_ID_H(X)                 ((X) << USART_CTRL2_ID_H_SHIFT)

/* Control register 3 */

#define USART_CTRL3_ERRIEN                  (1 << 0)  /* Error interrupt enable */
#define USART_CTRL3_IRDAEN                  (1 << 1)  /* IrDA enable */
#define USART_CTRL3_IRDALP                  (1 << 2)  /* IrDA low-power mode */
#define USART_CTRL3_SLBEN                   (1 << 3)  /* Single line bidirectional halfduplex enable */
#define USART_CTRL3_SCNACKEN                (1 << 4)  /* Smart card NACK enable */
#define USART_CTRL3_SCMEN                   (1 << 5)  /* Smart card mode enable */
#define USART_CTRL3_DMAREN                  (1 << 6)  /* DMA receiver enable */
#define USART_CTRL3_DMATEN                  (1 << 7)  /* DMA transmit enable */
#define USART_CTRL3_RTSEN                   (1 << 8)  /* RTS enable */
#define USART_CTRL3_CTSEN                   (1 << 9)  /* CTS enable */
#define USART_CTRL3_CTSCFIEN                (1 << 10) /* CTSCF interrupt enable */
#define USART_CTRL3_RS485EN                 (1 << 14) /* RS485 enable */
#define USART_CTRL3_DEP                     (1 << 15) /* DE polarity selection */

/* Guard time and prescaler register */

#define USART_GDIV_ISDIV_SHIFT              (0) /* IrDA/smartcard division */
#define USART_GDIV_ISDIV_MASK               (255 << USART_GDIV_ISDIV_SHIFT)
#define USART_GDIV_ISDIV(X)                 ((X) << USART_GDIV_ISDIV_SHIFT)

#define USART_GDIV_SCGT_SHIFT               (8) /* Smart card guard time */
#define USART_GDIV_SCGT_MASK                (255 << USART_GDIV_SCGT_SHIFT)
#define USART_GDIV_SCGT(X)                  ((X) << USART_GDIV_SCGT_SHIFT)

#define AT32_USART_RDR_OFFSET    AT32_USART_DT_OFFSET  /* Receive data register */
#define AT32_USART_TDR_OFFSET    AT32_USART_DT_OFFSET  /* Transmit data register */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_AT32_HARDWARE_AT32F43XXX_UART_H */
