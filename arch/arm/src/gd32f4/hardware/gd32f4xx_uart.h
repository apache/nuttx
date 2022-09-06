/****************************************************************************
 * arch/arm/src/gd32f4/hardware/gd32f4xx_uart.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_UART_H
#define __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USARTx(x=0,1,2,5)/UARTx(x=3,4,6,7) definitions */
#define GD32_USART1_BASE           (GD32_USART_BASE+0x00000000)    /* USART1 base address */
#define GD32_USART2_BASE           (GD32_USART_BASE+0x00000400)    /* USART2 base address */
#define GD32_UART3_BASE            (GD32_USART_BASE+0x00000800)    /* UART3 base address */
#define GD32_UART4_BASE            (GD32_USART_BASE+0x00000C00)    /* UART4 base address */
#define GD32_UART6_BASE            (GD32_USART_BASE+0x00003400)    /* UART6 base address */
#define GD32_UART7_BASE            (GD32_USART_BASE+0x00003800)    /* UART7 base address */
#define GD32_USART0_BASE           (GD32_USART_BASE+0x0000CC00)    /* USART0 base address */
#define GD32_USART5_BASE           (GD32_USART_BASE+0x0000D000)    /* USART5 base address */

/* Register Offsets *********************************************************/

#define GD32_USART_STAT0_OFFSET    0x0000  /* USART status register 0 offset */
#define GD32_USART_DATA_OFFSET     0x0004  /* USART data register offset */
#define GD32_USART_BAUD_OFFSET     0x0008  /* USART baud rate register offset */
#define GD32_USART_CTL0_OFFSET     0x000c  /* USART control register 0 offset */
#define GD32_USART_CTL1_OFFSET     0x0010  /* USART control register 1 offset */
#define GD32_USART_CTL2_OFFSET     0x0014  /* USART control register 2 offset */
#define GD32_USART_GP_OFFSET       0x0018  /* USART guard time and prescaler registet offset */
#define GD32_USART_CTL3_OFFSET     0x0080  /* USART control register 3 offset */
#define GD32_USART_RT_OFFSET       0x0084  /* USART receiver timeout register offset */
#define GD32_USART_STAT1_OFFSET    0x0088  /* USART status register 1 offset */
#define GD32_USART_CHC_OFFSET      0x00C0  /* USART coherence control register offset */

/* Register Addresses *******************************************************/

#define GD32_USART1                GD32_USART1_BASE
#define GD32_USART2                GD32_USART2_BASE
#define GD32_UART3                 GD32_UART3_BASE
#define GD32_UART4                 GD32_UART4_BASE
#define GD32_UART6                 GD32_UART6_BASE
#define GD32_UART7                 GD32_UART7_BASE
#define GD32_USART0                GD32_USART0_BASE
#define GD32_USART5                GD32_USART5_BASE

/* USART0,1,2,5 Register ****************************************************/

#define GD32_USART_STAT0(usartx)   ((usartx)+GD32_USART_STAT0_OFFSET)    /* USART status register 0 */
#define GD32_USART_DATA(usartx)    ((usartx)+GD32_USART_DATA_OFFSET)     /* USART data register */
#define GD32_USART_BAUD(usartx)    ((usartx)+GD32_USART_BAUD_OFFSET)     /* USART baud rate register */
#define GD32_USART_CTL0(usartx)    ((usartx)+GD32_USART_CTL0_OFFSET)     /* USART control register 0 */
#define GD32_USART_CTL1(usartx)    ((usartx)+GD32_USART_CTL1_OFFSET)     /* USART control register 1 */
#define GD32_USART_CTL2(usartx)    ((usartx)+GD32_USART_CTL2_OFFSET)     /* USART control register 2 */
#define GD32_USART_GP(usartx)      ((usartx)+GD32_USART_GP_OFFSET)       /* USART guard time and prescaler register */
#define GD32_USART_CTL3(usartx)    ((usartx)+GD32_USART_CTL3_OFFSET)     /* USART control register 3 offset */
#define GD32_USART_RT(usartx)      ((usartx)+GD32_USART_RT_OFFSET)       /* USART receiver timeout register */
#define GD32_USART_STAT1(usartx)   ((usartx)+GD32_USART_STAT1_OFFSET)    /* USART status register 1 */
#define GD32_USART_CHC(usartx)     ((usartx)+GD32_USART_CHC_OFFSET)      /* USART coherence control register */

/* UART3,4,6,7 Register *****************************************************/

#define GD32_UART_STAT0(uartx)     ((uartx)+GD32_USART_STAT0_OFFSET)     /* UART status register 0 */
#define GD32_UART_DATA(uartx)      ((uartx)+GD32_USART_DATA_OFFSET)      /* UART data register */
#define GD32_UART_BAUD(uartx)      ((uartx)+GD32_USART_BAUD_OFFSET)      /* UART baud rate register */
#define GD32_UART_CTL0(uartx)      ((uartx)+GD32_USART_CTL0_OFFSET)      /* UART control register 0 */
#define GD32_UART_CTL1(uartx)      ((uartx)+GD32_USART_CTL1_OFFSET)      /* UART control register 1 */
#define GD32_UART_CTL2(uartx)      ((uartx)+GD32_USART_CTL2_OFFSET)      /* UART control register 2 */
#define GD32_UART_GP(uartx)        ((uartx)+GD32_USART_GP_OFFSET)        /* UART guard time and prescaler register */
#define GD32_UART_CHC(uartx)       ((uartx)+GD32_USART_CHC_OFFSET)       /* UART coherence control register */

/* Register Bitfield Definitions ********************************************/

/* Status register 0 */

#define USART_STAT0_PERR           (1 << 0)    /* Bit 0: Parity error flag */
#define USART_STAT0_FERR           (1 << 1)    /* Bit 1: Frame error flag */
#define USART_STAT0_NERR           (1 << 2)    /* Bit 2: Noise error flag */
#define USART_STAT0_ORERR          (1 << 3)    /* Bit 3: OverRun error */
#define USART_STAT0_IDLEF          (1 << 4)    /* Bit 4: IDLE frame detected flag */
#define USART_STAT0_RBNE           (1 << 5)    /* Bit 5: Read data buffer not empty */
#define USART_STAT0_TC             (1 << 6)    /* Bit 6: Transmission complete */
#define USART_STAT0_TBE            (1 << 7)    /* Bit 7: Transmit data buffer empty */
#define USART_STAT0_LBDF           (1 << 8)    /* Bit 8: LIN break detected flag */
#define USART_STAT0_CTSF           (1 << 9)    /* Bit 9: CTS change flag */

#define USART_STAT0_MASK           (0x03ff)

/* Data register */

#define USART_DATA_SHIFT           (0)         /* Bits 0-8: Transmit or read data value */
#define USART_DATA_MASK            (0xff << USART_DATA_SHIFT)

/* Baud Rate Register */

#define USART_BAUD_FRADIV_SHIFT    (0)         /* Bits 0-3: Fraction part of baud-rate divider */
#define USART_BAUD_FRADIV_MASK     (0x0f << USART_BAUD_FRADIV_SHIFT)
#  define USART_BAUD_FRADIV(n)     ((n) << USART_BAUD_FRADIV_SHIFT)
#define USART_BAUD_INTDIV_SHIFT    (4)         /* Bits 4-15: Integer part of baud-rate divider */
#define USART_BAUD_INTDIV_MASK     (0xfff << USART_BAUD_INTDIV_SHIFT)
#  define USART_BAUD_INTDIV(n)     ((n) << USART_BAUD_INTDIV_SHIFT)

/* Control register 0 */

#define USART_CTL0_SBKCMD          (1 << 0)    /* Bit 0: Send break command */
#define USART_CTL0_RWU             (1 << 1)    /* Bit 1: Receiver wakeup from mute mode */
#define USART_CTL0_REN             (1 << 2)    /* Bit 2: Receiver Enable */
#define USART_CTL0_TEN             (1 << 3)    /* Bit 3: Transmitter Enable */
#define USART_CTL0_IDLEIE          (1 << 4)    /* Bit 4: Idle line detected interrupt enable */
#define USART_CTL0_RBNEIE          (1 << 5)    /* Bit 5: Read data buffer not empty interrupt and overrun error interrupt enable */
#define USART_CTL0_TCIE            (1 << 6)    /* Bit 6: Transmission complete interrupt enable */
#define USART_CTL0_TBEIE           (1 << 7)    /* Bit 7: Transmitter buffer empty interrupt enable */
#define USART_CTL0_PERRIE          (1 << 8)    /* Bit 8: Parity error interrupt enable */
#define USART_CTL0_PM              (1 << 9)    /* Bit 9: Parity mode */
#define USART_CTL0_PCEN            (1 << 10)   /* Bit 10: Parity check function enable */
#define USART_CTL0_WM              (1 << 11)   /* Bit 11: Wakeup method in mute mode */
#define USART_CTL0_WL              (1 << 12)   /* Bit 12: Word length */
#define USART_CTL0_UEN             (1 << 13)   /* Bit 13: USART Enable */
#define USART_CTL0_OVSMOD          (1 << 15)   /* Bit 15: Oversample mode */

#define USART_CTL0_PM_SHIFT        (9)
#  define USART_CTL0_PM_MASK       (0x3 << USART_CTL0_PM_SHIFT)
#  define USART_CTL0_PMEN(n)       ((n) << USART_CTL0_PM_SHIFT)
#  define USART_CTL0_PM_NONE       USART_CTL0_PMEN(0)
#  define USART_CTL0_PM_EVEN       USART_CTL0_PMEN(2)
#  define USART_CTL0_PM_ODD        USART_CTL0_PMEN(3)

#define USART_WL_9BIT              USART_CTL0_WL
#define USART_WL_8BIT              (0)       

#define USART_CTL0_INT_SHIFT       (4)
#define USART_CTL0_INT_MASK        (0x1f << USART_CTL0_INT_SHIFT)

/* Control register 1 */

#define USART_CTL1_ADDR_SHIFT      (0)         /* Bits 0-3: Address of USART */
#define USART_CTL1_ADDR_MASK       (0x0f << USART_CTL1_ADDR_SHIFT)
#define USART_CTL1_ADDR(n)         ((n) << USART_CTL1_ADDR_SHIFT)
#define USART_CTL1_LBLEN           (1 << 5)    /* Bit 5: LIN break frame Length */
#define USART_CTL1_LBDIE           (1 << 6)    /* Bit 6: LIN break detected interrupt enable */
#define USART_CTL1_CLEN            (1 << 8)    /* Bit 8: CK length */
#define USART_CTL1_CPH             (1 << 9)    /* Bit 9: CK phase */
#define USART_CTL1_CPL             (1 << 10)   /* Bit 10: CK polarity */
#define USART_CTL1_CKEN            (1 << 11)   /* Bit 11: CK pin enable */
#define USART_CTL1_STB_SHIFT       (12)        /* Bits 12-13: STOP bits length */
#define USART_CTL1_STB_MASK        (3 << USART_CTL1_STB_SHIFT)
#  define USART_CTL1_STB(n)        (n << USART_CTL1_STB_SHIFT)
#  define USART_CTL1_STB1BIT       USART_CTL1_STB(0) /* 00: 1 bit */
#  define USART_CTL1_STB0_5BIT     USART_CTL1_STB(1) /* 01: 0.5 bit */
#  define USART_CTL1_STB2BIT       USART_CTL1_STB(2) /* 10: 2 bits */
#  define USART_CTL1_STB1_5BIT     USART_CTL1_STB(3) /* 11: 1.5 bit */

#define USART_CTL1_LMEN            (1 << 14)   /* Bit 14: LIN mode enable */

#define USART_CTL1_INT_MASK        USART_CTL1_LBDIE

/* Control register 2 */

#define USART_CTL2_ERRIE           (1 << 0)    /* Bit 0: Error interrupt enable */
#define USART_CTL2_IREN            (1 << 1)    /* Bit 1: IrDA mode enable */
#define USART_CTL2_IRLP            (1 << 2)    /* Bit 2: IrDA low-power */
#define USART_CTL2_HDEN            (1 << 3)    /* Bit 3: Half-duplex enable */
#define USART_CTL2_NKEN            (1 << 4)    /* Bit 4: NACK enable in smartcard mode */
#define USART_CTL2_SCEN            (1 << 5)    /* Bit 5: Smartcard mode enable */
#define USART_CTL2_DENR            (1 << 6)    /* Bit 6: DMA request enable for reception */
#define USART_CTL2_DENT            (1 << 7)    /* Bit 7: DMA request enable for transmission */
#define USART_CTL2_RTSEN           (1 << 8)    /* Bit 8: RTS enable */
#define USART_CTL2_CTSEN           (1 << 9)    /* Bit 9: CTS enable */
#define USART_CTL2_CTSIE           (1 << 10)   /* Bit 10: CTS interrupt enable */
#define USART_CTL2_OSB             (1 << 11)   /* Bit 11: One sample bit method */

#define USART_CTL2_INT_MASK        (USART_CTL2_ERRIE | USART_CTL2_CTSIE)

/* USART guard time and prescaler register */

#define USART_GP_PSC_SHIFT         (0)         /* Bits 0-7: Prescaler value for dividing the system clock */
#define USART_GP_PSC_MASK          (0xff << USART_GP_PSC_SHIFT)
#  define USART_GP_PSC(n)            ((n) << USART_GP_PSC_SHIFT)
#define USART_GP_GUAT_SHIFT        (8)         /* Bits 8-15: Guard time value in smartcard mode */
#define USART_GP_GUAT_MASK         (0xff << USART_GP_GUAT_SHIFT)
#  define USART_GP_GUAT(n)           ((n) << USART_GP_GUAT_SHIFT)

/* USART control register 3 offset */

#define USART_CTL3_RTEN            (1 << 0)    /* Bit 0: Receiver timeout enable */
#define USART_CTL3_SCRTNUM_SHIFT   (1)         /* Bit 1-3: Smartcard auto-retry number */
#define USART_CTL3_SCRTNUM_MASK    (0x07 << USART_CTL3_SCRTNUM_SHIFT)
#  define USART_CTL3_SCRTNUM(n)      ((n) << USART_CTL3_SCRTNUM_SHIFT)
#define USART_CTL3_RTIE            (1 << 4)    /* Bit 4: Interrupt enable bit of receive timeout event */
#define USART_CTL3_EBIE            (1 << 5)    /* Bit 5: Interrupt enable bit of end of block event */
#define USART_CTL3_RINV            (1 << 8)    /* Bit 8: RX pin level inversion */
#define USART_CTL3_TINV            (1 << 9)    /* Bit 9: TX pin level inversion */
#define USART_CTL3_DINV            (1 << 10)   /* Bit 10: Data bit level inversion */
#define USART_CTL3_MSBF            (1 << 11)   /* Bit 11: Most significant bit first */

#define USART_CTL3_INT_MASK        (USART_CTL3_RTIE | USART_CTL3_EBIE)

/* USART receiver timeout register */

#define USART_RT_RT_SHIFT          (0)        /* Bit 0-23: Receiver timeout threshold */
#define USART_RT_RT_MASK           (0xffffff << USART_RT_RT_SHIFT)
#  define USART_RT_RT(n)           ((n) << USART_RT_RT_SHIFT)
#define USART_RT_BL_SHIFT          (24)       /* Bit 24-31: Block length */
#define USART_RT_BL_MASK           (0xff << USART_RT_BL_SHIFT)
#  define USART_RT_BL(n)           ((n) << USART_RT_BL_SHIFT)

/* USART status register 1 */

#define USART_STAT1_RTF            (1 << 11)   /* Bit 11: Receiver timeout flag */
#define USART_STAT1_EBF            (1 << 12)   /* Bit 12: End of block flag */
#define USART_STAT1_BSY            (1 << 16)   /* Bit 16: Busy flag */

/* USART coherence control register */

#define USART_CHC_HCM              (1 << 0)    /* Bit 0: Hardware flow control coherence mode */
#define USART_CHC_PCM              (1 << 1)    /* Bit 1: Parity check coherence mode */
#define USART_CHC_BCM              (1 << 2)    /* Bit 2: Break frame coherence mode */
#define USART_CHC_EPERR            (1 << 8)    /* Bit 8: Early parity error flag */

#endif /* __ARCH_ARM_SRC_GD32F4_HARDWARE_GD32F4XX_UART_H */
