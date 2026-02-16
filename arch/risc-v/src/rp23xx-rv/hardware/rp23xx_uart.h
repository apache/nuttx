/****************************************************************************
 * arch/risc-v/src/rp23xx-rv/hardware/rp23xx_uart.h
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

#ifndef __ARCH_RISC_V_SRC_RP23XX_HARDWARE_RP23XX_RV_UART_H
#define __ARCH_RISC_V_SRC_RP23XX_HARDWARE_RP23XX_RV_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_RV_UART_UARTDR_OFFSET         0x000000  /* Data Register, UARTDR */
#define RP23XX_RV_UART_UARTRSR_OFFSET        0x000004  /* Receive Status Register/Error Clear Register, UARTRSR/UARTECR */
#define RP23XX_RV_UART_UARTFR_OFFSET         0x000018  /* Flag Register, UARTFR */
#define RP23XX_RV_UART_UARTILPR_OFFSET       0x000020  /* IrDA Low-Power Counter Register, UARTILPR */
#define RP23XX_RV_UART_UARTIBRD_OFFSET       0x000024  /* Integer Baud Rate Register, UARTIBRD */
#define RP23XX_RV_UART_UARTFBRD_OFFSET       0x000028  /* Fractional Baud Rate Register, UARTFBRD */
#define RP23XX_RV_UART_UARTLCR_H_OFFSET      0x00002c  /* Line Control Register, UARTLCR_H */
#define RP23XX_RV_UART_UARTCR_OFFSET         0x000030  /* Control Register, UARTCR */
#define RP23XX_RV_UART_UARTIFLS_OFFSET       0x000034  /* Interrupt FIFO Level Select Register, UARTIFLS */
#define RP23XX_RV_UART_UARTIMSC_OFFSET       0x000038  /* Interrupt Mask Set/Clear Register, UARTIMSC */
#define RP23XX_RV_UART_UARTRIS_OFFSET        0x00003c  /* Raw Interrupt Status Register, UARTRIS */
#define RP23XX_RV_UART_UARTMIS_OFFSET        0x000040  /* Masked Interrupt Status Register, UARTMIS */
#define RP23XX_RV_UART_UARTICR_OFFSET        0x000044  /* Interrupt Clear Register, UARTICR */
#define RP23XX_RV_UART_UARTDMACR_OFFSET      0x000048  /* DMA Control Register, UARTDMACR */
#define RP23XX_RV_UART_UARTPERIPHID0_OFFSET  0x000fe0  /* UARTPeriphID0 Register */
#define RP23XX_RV_UART_UARTPERIPHID1_OFFSET  0x000fe4  /* UARTPeriphID1 Register */
#define RP23XX_RV_UART_UARTPERIPHID2_OFFSET  0x000fe8  /* UARTPeriphID2 Register */
#define RP23XX_RV_UART_UARTPERIPHID3_OFFSET  0x000fec  /* UARTPeriphID3 Register */
#define RP23XX_RV_UART_UARTPCELLID0_OFFSET   0x000ff0  /* UARTPCellID0 Register */
#define RP23XX_RV_UART_UARTPCELLID1_OFFSET   0x000ff4  /* UARTPCellID1 Register */
#define RP23XX_RV_UART_UARTPCELLID2_OFFSET   0x000ff8  /* UARTPCellID2 Register */
#define RP23XX_RV_UART_UARTPCELLID3_OFFSET   0x000ffc  /* UARTPCellID3 Register */

/* Register definitions (UART) **********************************************/

#define RP23XX_RV_UART0_UARTDR         (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTDR_OFFSET)
#define RP23XX_RV_UART0_UARTRSR        (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTRSR_OFFSET)
#define RP23XX_RV_UART0_UARTFR         (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTFR_OFFSET)
#define RP23XX_RV_UART0_UARTILPR       (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTILPR_OFFSET)
#define RP23XX_RV_UART0_UARTIBRD       (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTIBRD_OFFSET)
#define RP23XX_RV_UART0_UARTFBRD       (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTFBRD_OFFSET)
#define RP23XX_RV_UART0_UARTLCR_H      (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTLCR_H_OFFSET)
#define RP23XX_RV_UART0_UARTCR         (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTCR_OFFSET)
#define RP23XX_RV_UART0_UARTIFLS       (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTIFLS_OFFSET)
#define RP23XX_RV_UART0_UARTIMSC       (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTIMSC_OFFSET)
#define RP23XX_RV_UART0_UARTRIS        (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTRIS_OFFSET)
#define RP23XX_RV_UART0_UARTMIS        (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTMIS_OFFSET)
#define RP23XX_RV_UART0_UARTICR        (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTICR_OFFSET)
#define RP23XX_RV_UART0_UARTDMACR      (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTDMACR_OFFSET)
#define RP23XX_RV_UART0_UARTPERIPHID0  (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTPERIPHID0_OFFSET)
#define RP23XX_RV_UART0_UARTPERIPHID1  (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTPERIPHID1_OFFSET)
#define RP23XX_RV_UART0_UARTPERIPHID2  (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTPERIPHID2_OFFSET)
#define RP23XX_RV_UART0_UARTPERIPHID3  (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTPERIPHID3_OFFSET)
#define RP23XX_RV_UART0_UARTPCELLID0   (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTPCELLID0_OFFSET)
#define RP23XX_RV_UART0_UARTPCELLID1   (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTPCELLID1_OFFSET)
#define RP23XX_RV_UART0_UARTPCELLID2   (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTPCELLID2_OFFSET)
#define RP23XX_RV_UART0_UARTPCELLID3   (RP23XX_RV_UART0_BASE + RP23XX_RV_UART_UARTPCELLID3_OFFSET)

/* Register definitions (UART1) *********************************************/

#define RP23XX_RV_UART1_UARTDR         (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTDR_OFFSET)
#define RP23XX_RV_UART1_UARTRSR        (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTRSR_OFFSET)
#define RP23XX_RV_UART1_UARTFR         (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTFR_OFFSET)
#define RP23XX_RV_UART1_UARTILPR       (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTILPR_OFFSET)
#define RP23XX_RV_UART1_UARTIBRD       (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTIBRD_OFFSET)
#define RP23XX_RV_UART1_UARTFBRD       (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTFBRD_OFFSET)
#define RP23XX_RV_UART1_UARTLCR_H      (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTLCR_H_OFFSET)
#define RP23XX_RV_UART1_UARTCR         (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTCR_OFFSET)
#define RP23XX_RV_UART1_UARTIFLS       (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTIFLS_OFFSET)
#define RP23XX_RV_UART1_UARTIMSC       (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTIMSC_OFFSET)
#define RP23XX_RV_UART1_UARTRIS        (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTRIS_OFFSET)
#define RP23XX_RV_UART1_UARTMIS        (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTMIS_OFFSET)
#define RP23XX_RV_UART1_UARTICR        (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTICR_OFFSET)
#define RP23XX_RV_UART1_UARTDMACR      (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTDMACR_OFFSET)
#define RP23XX_RV_UART1_UARTPERIPHID0  (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTPERIPHID0_OFFSET)
#define RP23XX_RV_UART1_UARTPERIPHID1  (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTPERIPHID1_OFFSET)
#define RP23XX_RV_UART1_UARTPERIPHID2  (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTPERIPHID2_OFFSET)
#define RP23XX_RV_UART1_UARTPERIPHID3  (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTPERIPHID3_OFFSET)
#define RP23XX_RV_UART1_UARTPCELLID0   (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTPCELLID0_OFFSET)
#define RP23XX_RV_UART1_UARTPCELLID1   (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTPCELLID1_OFFSET)
#define RP23XX_RV_UART1_UARTPCELLID2   (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTPCELLID2_OFFSET)
#define RP23XX_RV_UART1_UARTPCELLID3   (RP23XX_RV_UART1_BASE + RP23XX_RV_UART_UARTPCELLID3_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_RV_UART_UARTDR_OE                         (1 << 11)
#define RP23XX_RV_UART_UARTDR_BE                         (1 << 10)
#define RP23XX_RV_UART_UARTDR_PE                         (1 << 9)
#define RP23XX_RV_UART_UARTDR_FE                         (1 << 8)
#define RP23XX_RV_UART_UARTDR_DATA_MASK                  (0xff)

#define RP23XX_RV_UART_UARTRSR_OE                        (1 << 3)
#define RP23XX_RV_UART_UARTRSR_BE                        (1 << 2)
#define RP23XX_RV_UART_UARTRSR_PE                        (1 << 1)
#define RP23XX_RV_UART_UARTRSR_FE                        (1 << 0)

#define RP23XX_RV_UART_UARTFR_RI                         (1 << 8)
#define RP23XX_RV_UART_UARTFR_TXFE                       (1 << 7)
#define RP23XX_RV_UART_UARTFR_RXFF                       (1 << 6)
#define RP23XX_RV_UART_UARTFR_TXFF                       (1 << 5)
#define RP23XX_RV_UART_UARTFR_RXFE                       (1 << 4)
#define RP23XX_RV_UART_UARTFR_BUSY                       (1 << 3)
#define RP23XX_RV_UART_UARTFR_DCD                        (1 << 2)
#define RP23XX_RV_UART_UARTFR_DSR                        (1 << 1)
#define RP23XX_RV_UART_UARTFR_CTS                        (1 << 0)

#define RP23XX_RV_UART_UARTILPR_ILPDVSR_MASK             (0xff)

#define RP23XX_RV_UART_UARTIBRD_BAUD_DIVINT_MASK         (0xffff)

#define RP23XX_RV_UART_UARTFBRD_BAUD_DIVFRAC_MASK        (0x3f)

#define RP23XX_RV_UART_UARTLCR_H_SPS                     (1 << 7)
#define RP23XX_RV_UART_UARTLCR_H_WLEN_SHIFT              (5)
#define RP23XX_RV_UART_UARTLCR_H_WLEN_MASK               (0x03 << RP23XX_RV_UART_UARTLCR_H_WLEN_SHIFT)
#define RP23XX_RV_UART_UARTLCR_H_FEN                     (1 << 4)
#define RP23XX_RV_UART_UARTLCR_H_STP2                    (1 << 3)
#define RP23XX_RV_UART_UARTLCR_H_EPS                     (1 << 2)
#define RP23XX_RV_UART_UARTLCR_H_PEN                     (1 << 1)
#define RP23XX_RV_UART_UARTLCR_H_BRK                     (1 << 0)

#define RP23XX_RV_UART_LCR_H_WLEN(x)                     ((((x) - 5) << RP23XX_RV_UART_UARTLCR_H_WLEN_SHIFT) & RP23XX_RV_UART_UARTLCR_H_WLEN_MASK)

#define RP23XX_RV_UART_UARTCR_CTSEN                      (1 << 15)
#define RP23XX_RV_UART_UARTCR_RTSEN                      (1 << 14)
#define RP23XX_RV_UART_UARTCR_OUT2                       (1 << 13)
#define RP23XX_RV_UART_UARTCR_OUT1                       (1 << 12)
#define RP23XX_RV_UART_UARTCR_RTS                        (1 << 11)
#define RP23XX_RV_UART_UARTCR_DTR                        (1 << 10)
#define RP23XX_RV_UART_UARTCR_RXE                        (1 << 9)
#define RP23XX_RV_UART_UARTCR_TXE                        (1 << 8)
#define RP23XX_RV_UART_UARTCR_LBE                        (1 << 7)
#define RP23XX_RV_UART_UARTCR_SIRLP                      (1 << 2)
#define RP23XX_RV_UART_UARTCR_SIREN                      (1 << 1)
#define RP23XX_RV_UART_UARTCR_UARTEN                     (1 << 0)

#define RP23XX_RV_UART_UARTIFLS_RXIFLSEL_SHIFT           (3)
#define RP23XX_RV_UART_UARTIFLS_RXIFLSEL_MASK            (0x07 << RP23XX_RV_UART_UARTIFLS_RXIFLSEL_SHIFT)
#define RP23XX_RV_UART_UARTIFLS_TXIFLSEL_MASK            (0x07)

#define RP23XX_RV_UART_INTR_ALL                          (0x7ff)   /* All of interrupts */

#define RP23XX_RV_UART_UARTIMSC_OEIM                     (1 << 10) /* Overrun error interrupt mask. A read returns the current mask for the UARTOEINTR interrupt. On a write of 1, the mask of the UARTOEINTR interrupt is set. A write of 0 clears the mask. */
#define RP23XX_RV_UART_UARTIMSC_BEIM                     (1 << 9)  /* Break error interrupt mask. A read returns the current mask for the UARTBEINTR interrupt. On a write of 1, the mask of the UARTBEINTR interrupt is set. A write of 0 clears the mask. */
#define RP23XX_RV_UART_UARTIMSC_PEIM                     (1 << 8)  /* Parity error interrupt mask. A read returns the current mask for the UARTPEINTR interrupt. On a write of 1, the mask of the UARTPEINTR interrupt is set. A write of 0 clears the mask. */
#define RP23XX_RV_UART_UARTIMSC_FEIM                     (1 << 7)  /* Framing error interrupt mask. A read returns the current mask for the UARTFEINTR interrupt. On a write of 1, the mask of the UARTFEINTR interrupt is set. A write of 0 clears the mask. */
#define RP23XX_RV_UART_UARTIMSC_RTIM                     (1 << 6)  /* Receive timeout interrupt mask. A read returns the current mask for the UARTRTINTR interrupt. On a write of 1, the mask of the UARTRTINTR interrupt is set. A write of 0 clears the mask. */
#define RP23XX_RV_UART_UARTIMSC_TXIM                     (1 << 5)  /* Transmit interrupt mask. A read returns the current mask for the UARTTXINTR interrupt. On a write of 1, the mask of the UARTTXINTR interrupt is set. A write of 0 clears the mask. */
#define RP23XX_RV_UART_UARTIMSC_RXIM                     (1 << 4)  /* Receive interrupt mask. A read returns the current mask for the UARTRXINTR interrupt. On a write of 1, the mask of the UARTRXINTR interrupt is set. A write of 0 clears the mask. */
#define RP23XX_RV_UART_UARTIMSC_DSRMIM                   (1 << 3)  /* nUARTDSR modem interrupt mask. A read returns the current mask for the UARTDSRINTR interrupt. On a write of 1, the mask of the UARTDSRINTR interrupt is set. A write of 0 clears the mask. */
#define RP23XX_RV_UART_UARTIMSC_DCDMIM                   (1 << 2)  /* nUARTDCD modem interrupt mask. A read returns the current mask for the UARTDCDINTR interrupt. On a write of 1, the mask of the UARTDCDINTR interrupt is set. A write of 0 clears the mask. */
#define RP23XX_RV_UART_UARTIMSC_CTSMIM                   (1 << 1)  /* nUARTCTS modem interrupt mask. A read returns the current mask for the UARTCTSINTR interrupt. On a write of 1, the mask of the UARTCTSINTR interrupt is set. A write of 0 clears the mask. */
#define RP23XX_RV_UART_UARTIMSC_RIMIM                    (1 << 0)  /* nUARTRI modem interrupt mask. A read returns the current mask for the UARTRIINTR interrupt. On a write of 1, the mask of the UARTRIINTR interrupt is set. A write of 0 clears the mask. */

#define RP23XX_RV_UART_UARTRIS_OERIS                     (1 << 10) /* Overrun error interrupt status. Returns the raw interrupt state of the UARTOEINTR interrupt. */
#define RP23XX_RV_UART_UARTRIS_BERIS                     (1 << 9)  /* Break error interrupt status. Returns the raw interrupt state of the UARTBEINTR interrupt. */
#define RP23XX_RV_UART_UARTRIS_PERIS                     (1 << 8)  /* Parity error interrupt status. Returns the raw interrupt state of the UARTPEINTR interrupt. */
#define RP23XX_RV_UART_UARTRIS_FERIS                     (1 << 7)  /* Framing error interrupt status. Returns the raw interrupt state of the UARTFEINTR interrupt. */
#define RP23XX_RV_UART_UARTRIS_RTRIS                     (1 << 6)  /* Receive timeout interrupt status. Returns the raw interrupt state of the UARTRTINTR interrupt. a */
#define RP23XX_RV_UART_UARTRIS_TXRIS                     (1 << 5)  /* Transmit interrupt status. Returns the raw interrupt state of the UARTTXINTR interrupt. */
#define RP23XX_RV_UART_UARTRIS_RXRIS                     (1 << 4)  /* Receive interrupt status. Returns the raw interrupt state of the UARTRXINTR interrupt. */
#define RP23XX_RV_UART_UARTRIS_DSRRMIS                   (1 << 3)  /* nUARTDSR modem interrupt status. Returns the raw interrupt state of the UARTDSRINTR interrupt. */
#define RP23XX_RV_UART_UARTRIS_DCDRMIS                   (1 << 2)  /* nUARTDCD modem interrupt status. Returns the raw interrupt state of the UARTDCDINTR interrupt. */
#define RP23XX_RV_UART_UARTRIS_CTSRMIS                   (1 << 1)  /* nUARTCTS modem interrupt status. Returns the raw interrupt state of the UARTCTSINTR interrupt. */
#define RP23XX_RV_UART_UARTRIS_RIRMIS                    (1 << 0)  /* nUARTRI modem interrupt status. Returns the raw interrupt state of the UARTRIINTR interrupt. */

#define RP23XX_RV_UART_UARTMIS_OEMIS                     (1 << 10) /* Overrun error masked interrupt status. Returns the masked interrupt state of the UARTOEINTR interrupt. */
#define RP23XX_RV_UART_UARTMIS_BEMIS                     (1 << 9)  /* Break error masked interrupt status. Returns the masked interrupt state of the UARTBEINTR interrupt. */
#define RP23XX_RV_UART_UARTMIS_PEMIS                     (1 << 8)  /* Parity error masked interrupt status. Returns the masked interrupt state of the UARTPEINTR interrupt. */
#define RP23XX_RV_UART_UARTMIS_FEMIS                     (1 << 7)  /* Framing error masked interrupt status. Returns the masked interrupt state of the UARTFEINTR interrupt. */
#define RP23XX_RV_UART_UARTMIS_RTMIS                     (1 << 6)  /* Receive timeout masked interrupt status. Returns the masked interrupt state of the UARTRTINTR interrupt. */
#define RP23XX_RV_UART_UARTMIS_TXMIS                     (1 << 5)  /* Transmit masked interrupt status. Returns the masked interrupt state of the UARTTXINTR interrupt. */
#define RP23XX_RV_UART_UARTMIS_RXMIS                     (1 << 4)  /* Receive masked interrupt status. Returns the masked interrupt state of the UARTRXINTR interrupt. */
#define RP23XX_RV_UART_UARTMIS_DSRMMIS                   (1 << 3)  /* nUARTDSR modem masked interrupt status. Returns the masked interrupt state of the UARTDSRINTR interrupt. */
#define RP23XX_RV_UART_UARTMIS_DCDMMIS                   (1 << 2)  /* nUARTDCD modem masked interrupt status. Returns the masked interrupt state of the UARTDCDINTR interrupt. */
#define RP23XX_RV_UART_UARTMIS_CTSMMIS                   (1 << 1)  /* nUARTCTS modem masked interrupt status. Returns the masked interrupt state of the UARTCTSINTR interrupt. */
#define RP23XX_RV_UART_UARTMIS_RIMMIS                    (1 << 0)  /* nUARTRI modem masked interrupt status. Returns the masked interrupt state of the UARTRIINTR interrupt. */

#define RP23XX_RV_UART_UARTICR_OEIC                      (1 << 10) /* Overrun error interrupt clear. Clears the UARTOEINTR interrupt. */
#define RP23XX_RV_UART_UARTICR_BEIC                      (1 << 9)  /* Break error interrupt clear. Clears the UARTBEINTR interrupt. */
#define RP23XX_RV_UART_UARTICR_PEIC                      (1 << 8)  /* Parity error interrupt clear. Clears the UARTPEINTR interrupt. */
#define RP23XX_RV_UART_UARTICR_FEIC                      (1 << 7)  /* Framing error interrupt clear. Clears the UARTFEINTR interrupt. */
#define RP23XX_RV_UART_UARTICR_RTIC                      (1 << 6)  /* Receive timeout interrupt clear. Clears the UARTRTINTR interrupt. */
#define RP23XX_RV_UART_UARTICR_TXIC                      (1 << 5)  /* Transmit interrupt clear. Clears the UARTTXINTR interrupt. */
#define RP23XX_RV_UART_UARTICR_RXIC                      (1 << 4)  /* Receive interrupt clear. Clears the UARTRXINTR interrupt. */
#define RP23XX_RV_UART_UARTICR_DSRMIC                    (1 << 3)  /* nUARTDSR modem interrupt clear. Clears the UARTDSRINTR interrupt. */
#define RP23XX_RV_UART_UARTICR_DCDMIC                    (1 << 2)  /* nUARTDCD modem interrupt clear. Clears the UARTDCDINTR interrupt. */
#define RP23XX_RV_UART_UARTICR_CTSMIC                    (1 << 1)  /* nUARTCTS modem interrupt clear. Clears the UARTCTSINTR interrupt. */
#define RP23XX_RV_UART_UARTICR_RIMIC                     (1 << 0)  /* nUARTRI modem interrupt clear. Clears the UARTRIINTR interrupt. */

#define RP23XX_RV_UART_UARTDMACR_DMAONERR                (1 << 2)
#define RP23XX_RV_UART_UARTDMACR_TXDMAE                  (1 << 1)  /* Transmit DMA enable. If this bit is set to 1, DMA for the transmit FIFO is enabled. */
#define RP23XX_RV_UART_UARTDMACR_RXDMAE                  (1 << 0)  /* Receive DMA enable. If this bit is set to 1, DMA for the receive FIFO is enabled. */

#define RP23XX_RV_UART_UARTPERIPHID0_PARTNUMBER0_MASK    (0xff)    /* These bits read back as 0x11 */

#define RP23XX_RV_UART_UARTPERIPHID1_DESIGNER0_SHIFT     (4)       /* These bits read back as 0x1 */
#define RP23XX_RV_UART_UARTPERIPHID1_DESIGNER0_MASK      (0x0f << RP23XX_RV_UART_UARTPERIPHID1_DESIGNER0_SHIFT)
#define RP23XX_RV_UART_UARTPERIPHID1_PARTNUMBER1_MASK    (0x0f)    /* These bits read back as 0x0 */

#define RP23XX_RV_UART_UARTPERIPHID2_REVISION_SHIFT      (4)
#define RP23XX_RV_UART_UARTPERIPHID2_REVISION_MASK       (0x0f << RP23XX_RV_UART_UARTPERIPHID2_REVISION_SHIFT)
#define RP23XX_RV_UART_UARTPERIPHID2_DESIGNER1_MASK      (0x0f)
#define RP23XX_RV_UART_UARTPERIPHID3_CONFIGURATION_MASK  (0xff)
#define RP23XX_RV_UART_UARTPCELLID0_MASK                 (0xff)
#define RP23XX_RV_UART_UARTPCELLID1_MASK                 (0xff)
#define RP23XX_RV_UART_UARTPCELLID2_MASK                 (0xff)
#define RP23XX_RV_UART_UARTPCELLID3_MASK                 (0xff)

#endif
