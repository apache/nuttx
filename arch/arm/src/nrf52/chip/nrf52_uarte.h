/*****************************************************************************************************
 * arch/arm/src/nrf52/chip/nrf52_uarte.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Janne Rosberg <janne@offcode.fi>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF52_CHIP_NRF52_UARTE_H
#define __ARCH_ARM_SRC_NRF52_CHIP_NRF52_UARTE_H

/*****************************************************************************************************
 * Included Files
 *****************************************************************************************************/

#include <nuttx/config.h>
#include "chip/nrf52_memorymap.h"

/*****************************************************************************************************
 * Pre-processor Definitions
 *****************************************************************************************************/

/* UARTE Register Offsets ****************************************************************************/

/* Registers for the UARTE function: */

#define NRF52_UARTE_TASKS_STARTRX_OFFSET    0x0000  /* Start UART receiver */
#define NRF52_UARTE_TASKS_STOPRX_OFFSET     0x0004  /* Stop UART receiver */
#define NRF52_UARTE_TASKS_STARTTX_OFFSET    0x0008  /* Start UART transmitter */
#define NRF52_UARTE_TASKS_STOPTX_OFFSET     0x000c  /* Stop UART transmitter */
#define NRF52_UARTE_TASKS_FLUSHRX_OFFSET    0x002c  /* Flush RX FIFO into RX buffer */
#define NRF52_UARTE_EVENTS_CTS_OFFSET       0x0100  /* CTS is activated (set low). Clear To Send. */
#define NRF52_UARTE_EVENTS_NCTS_OFFSET      0x0104  /* CTS is deactivated (set high). Not Clear To Send. */
#define NRF52_UARTE_EVENTS_RXDRDY_OFFSET    0x0108  /* Data received in RXD (but potentially not yet transferred to Data RAM) */
#define NRF52_UARTE_EVENTS_ENDRX_OFFSET     0x0110  /* Receive buffer is filled up */
#define NRF52_UARTE_EVENTS_TXDRDY_OFFSET    0x011c  /* Data sent from TXD */
#define NRF52_UARTE_EVENTS_ENDTX_OFFSET     0x0120  /* Last TX byte transmitted */
#define NRF52_UARTE_EVENTS_ERROR_OFFSET     0x0124  /* Error detected */
#define NRF52_UARTE_EVENTS_RXTO_OFFSET      0x0144  /* Receiver timeout */
#define NRF52_UARTE_EVENTS_RXSTARTED_OFFSET 0x014c  /* UART receiver has started */
#define NRF52_UARTE_EVENTS_TXSTARTED_OFFSET 0x0150  /* UART transmitter has started */
#define NRF52_UARTE_EVENTS_TXSTOPPED_OFFSET 0x0158  /* Transmitter stopped */
#define NRF52_UARTE_SHORTS_OFFSET           0x0200  /* Shortcut register */
#define NRF52_UARTE_INTEN_OFFSET            0x0300  /* Enable or disable interrupt */
#define NRF52_UARTE_INTENSET_OFFSET         0x0304  /* Enable interrupt */
#define NRF52_UARTE_INTENCLR_OFFSET         0x0308  /* Disable interrupt */
#define NRF52_UARTE_ERRORSRC_OFFSET         0x0480  /* Error source */
#define NRF52_UARTE_ENABLE_OFFSET           0x0500  /* Enable UART */
#define NRF52_UARTE_PSEL_RTS_OFFSET         0x0508  /* Pin select for RTS signal */
#define NRF52_UARTE_PSEL_TXD_OFFSET         0x050c  /* Pin select for TXD signal */
#define NRF52_UARTE_PSEL_CTS_OFFSET         0x0510  /* Pin select for CTS signal */
#define NRF52_UARTE_PSEL_RXD_OFFSET         0x0514  /* Pin select for RXD signal */
#define NRF52_UARTE_BAUDRATE_OFFSET         0x0524  /* Baud rate. Accuracy depends on the HFCLK source selected. */
#define NRF52_UARTE_RXD_PTR_OFFSET          0x0534  /* Data pointer */
#define NRF52_UARTE_RXD_MAXCNT_OFFSET       0x0538  /* Maximum number of bytes in receive buffer */
#define NRF52_UARTE_RXD_AMOUNT_OFFSET       0x053c  /* Number of bytes transferred in the last transaction */
#define NRF52_UARTE_TXD_PTR_OFFSET          0x0544  /* Data pointer */
#define NRF52_UARTE_TXD_MAXCNT_OFFSET       0x0548  /* Maximum number of bytes in transmit buffer */
#define NRF52_UARTE_TXD_AMOUNT_OFFSET       0x054c  /* Number of bytes transferred in the last transaction */
#define NRF52_UARTE_CONFIG_OFFSET           0x056c  /* Configuration of parity and hardware flow control */


#define NRF52_UARTE_TASKS_STARTRX           (NRF52_UARTE0_BASE + NRF52_UARTE_TASKS_STARTRX_OFFSET)
#define NRF52_UARTE_TASKS_STOPRX            (NRF52_UARTE0_BASE + NRF52_UARTE_TASKS_STOPRX_OFFSET)
#define NRF52_UARTE_TASKS_STARTTX           (NRF52_UARTE0_BASE + NRF52_UARTE_TASKS_STARTTX_OFFSET)
#define NRF52_UARTE_TASKS_STOPTX            (NRF52_UARTE0_BASE + NRF52_UARTE_TASKS_STOPTX_OFFSET)
#define NRF52_UARTE_TASKS_FLUSHRX           (NRF52_UARTE0_BASE + NRF52_UARTE_TASKS_FLUSHRX_OFFSET)
#define NRF52_UARTE_EVENTS_CTS              (NRF52_UARTE0_BASE + NRF52_UARTE_EVENTS_CTS_OFFSET)
#define NRF52_UARTE_EVENTS_NCTS             (NRF52_UARTE0_BASE + NRF52_UARTE_EVENTS_NCTS_OFFSET)
#define NRF52_UARTE_EVENTS_RXDRDY           (NRF52_UARTE0_BASE + NRF52_UARTE_EVENTS_RXDRDY_OFFSET)
#define NRF52_UARTE_EVENTS_ENDRX            (NRF52_UARTE0_BASE + NRF52_UARTE_EVENTS_ENDRX_OFFSET)
#define NRF52_UARTE_EVENTS_TXDRDY           (NRF52_UARTE0_BASE + NRF52_UARTE_EVENTS_TXDRDY_OFFSET)
#define NRF52_UARTE_EVENTS_ENDTX            (NRF52_UARTE0_BASE + NRF52_UARTE_EVENTS_ENDTX_OFFSET)
#define NRF52_UARTE_EVENTS_ERROR            (NRF52_UARTE0_BASE + NRF52_UARTE_EVENTS_ERROR_OFFSET)
#define NRF52_UARTE_EVENTS_RXTO             (NRF52_UARTE0_BASE + NRF52_UARTE_EVENTS_RXTO_OFFSET)
#define NRF52_UARTE_EVENTS_RXSTARTED        (NRF52_UARTE0_BASE + NRF52_UARTE_EVENTS_RXSTARTED_OFFSET)
#define NRF52_UARTE_EVENTS_TXSTARTED        (NRF52_UARTE0_BASE + NRF52_UARTE_EVENTS_TXSTARTED_OFFSET)
#define NRF52_UARTE_EVENTS_TXSTOPPED        (NRF52_UARTE0_BASE + NRF52_UARTE_EVENTS_TXSTOPPED_OFFSET)
#define NRF52_UARTE_SHORTS                  (NRF52_UARTE0_BASE + NRF52_UARTE_SHORTS_OFFSET)
#define NRF52_UARTE_INTEN                   (NRF52_UARTE0_BASE + NRF52_UARTE_INTEN_OFFSET)
#define NRF52_UARTE_INTENSET                (NRF52_UARTE0_BASE + NRF52_UARTE_INTENSET_OFFSET)
#define NRF52_UARTE_INTENCLR                (NRF52_UARTE0_BASE + NRF52_UARTE_INTENCLR_OFFSET)
#define NRF52_UARTE_ERRORSRC                (NRF52_UARTE0_BASE + NRF52_UARTE_ERRORSRC_OFFSET)
#define NRF52_UARTE_ENABLE                  (NRF52_UARTE0_BASE + NRF52_UARTE_ENABLE_OFFSET)
#define NRF52_UARTE_PSEL_RTS                (NRF52_UARTE0_BASE + NRF52_UARTE_PSEL_RTS_OFFSET)
#define NRF52_UARTE_PSEL_TXD                (NRF52_UARTE0_BASE + NRF52_UARTE_PSEL_TXD_OFFSET)
#define NRF52_UARTE_PSEL_CTS                (NRF52_UARTE0_BASE + NRF52_UARTE_PSEL_CTS_OFFSET)
#define NRF52_UARTE_PSEL_RXD                (NRF52_UARTE0_BASE + NRF52_UARTE_PSEL_RXD_OFFSET)
#define NRF52_UARTE_BAUDRATE                (NRF52_UARTE0_BASE + NRF52_UARTE_BAUDRATE_OFFSET)
#define NRF52_UARTE_RXD_PTR                 (NRF52_UARTE0_BASE + NRF52_UARTE_RXD_PTR_OFFSET)
#define NRF52_UARTE_RXD_MAXCNT              (NRF52_UARTE0_BASE + NRF52_UARTE_RXD_MAXCNT_OFFSET)
#define NRF52_UARTE_RXD_AMOUNT              (NRF52_UARTE0_BASE + NRF52_UARTE_RXD_AMOUNT_OFFSET)
#define NRF52_UARTE_TXD_PTR                 (NRF52_UARTE0_BASE + NRF52_UARTE_TXD_PTR_OFFSET)
#define NRF52_UARTE_TXD_MAXCNT              (NRF52_UARTE0_BASE + NRF52_UARTE_TXD_MAXCNT_OFFSET)
#define NRF52_UARTE_TXD_AMOUNT              (NRF52_UARTE0_BASE + NRF52_UARTE_TXD_AMOUNT_OFFSET)
#define NRF52_UARTE_CONFIG                  (NRF52_UARTE0_BASE + NRF52_UARTE_CONFIG_OFFSET)

#define NRF52_UART_TASKS_STARTRX_OFFSET     0x0000 /* Start UART receiver */
#define NRF52_UART_TASKS_STOPRX_OFFSET      0x0004 /* Stop UART receiver */
#define NRF52_UART_TASKS_STARTTX_OFFSET     0x0008 /* Start UART transmitter */
#define NRF52_UART_TASKS_STOPTX_OFFSET      0x000c /* Stop UART transmitter */
#define NRF52_UART_TASKS_SUSPEND_OFFSET     0x001c /* Suspend UART */
#define NRF52_UART_EVENTS_CTS_OFFSET        0x0100 /* CTS is activated (set low). Clear To Send. */
#define NRF52_UART_EVENTS_NCTS_OFFSET       0x0104 /* CTS is deactivated (set high). Not Clear To Send. */
#define NRF52_UART_EVENTS_RXDRDY_OFFSET     0x0108 /* Data received in RXD */
#define NRF52_UART_EVENTS_TXDRDY_OFFSET     0x011c /* Data sent from TXD */
#define NRF52_UART_EVENTS_ERROR_OFFSET      0x0124 /* Error detected */
#define NRF52_UART_EVENTS_RXTO_OFFSET       0x0144 /* Receiver timeout */
#define NRF52_UART_SHORTS_OFFSET            0x0200 /* Shortcut register */
#define NRF52_UART_INTENSET_OFFSET          0x0304 /* Enable interrupt */
#define NRF52_UART_INTENCLR_OFFSET          0x0308 /* Disable interrupt */
#define NRF52_UART_ERRORSRC_OFFSET          0x0480 /* Error source */
#define NRF52_UART_ENABLE_OFFSET            0x0500 /* Enable UART */
#define NRF52_UART_PSELRTS_OFFSET           0x0508 /* Pin select for RTS */
#define NRF52_UART_PSELTXD_OFFSET           0x050c /* Pin select for TXD */
#define NRF52_UART_PSELCTS_OFFSET           0x0510 /* Pin select for CTS */
#define NRF52_UART_PSELRXD_OFFSET           0x0514 /* Pin select for RXD */
#define NRF52_UART_RXD_OFFSET               0x0518 /* RXD register */
#define NRF52_UART_TXD_OFFSET               0x051c /* TXD register */
#define NRF52_UART_BAUDRATE_OFFSET          0x0524 /* Baud rate */
#define NRF52_UART_CONFIG_OFFSET            0x056c /* Configuration of parity and hardware flow control */

#define NRF52_UART_TASKS_STARTRX            (NRF52_UART0_BASE + NRF52_UART_TASKS_STARTRX_OFFSET)
#define NRF52_UART_TASKS_STOPRX             (NRF52_UART0_BASE + NRF52_UART_TASKS_STOPRX_OFFSET)
#define NRF52_UART_TASKS_STARTTX            (NRF52_UART0_BASE + NRF52_UART_TASKS_STARTTX_OFFSET)
#define NRF52_UART_TASKS_STOPTX             (NRF52_UART0_BASE + NRF52_UART_TASKS_STOPTX_OFFSET)
#define NRF52_UART_TASKS_SUSPEND            (NRF52_UART0_BASE + NRF52_UART_TASKS_SUSPEND_OFFSET)
#define NRF52_UART_EVENTS_CTS               (NRF52_UART0_BASE + NRF52_UART_EVENTS_CTS_OFFSET)
#define NRF52_UART_EVENTS_NCTS              (NRF52_UART0_BASE + NRF52_UART_EVENTS_NCTS_OFFSET)
#define NRF52_UART_EVENTS_RXDRDY            (NRF52_UART0_BASE + NRF52_UART_EVENTS_RXDRDY_OFFSET)
#define NRF52_UART_EVENTS_TXDRDY            (NRF52_UART0_BASE + NRF52_UART_EVENTS_TXDRDY_OFFSET)
#define NRF52_UART_EVENTS_ERROR             (NRF52_UART0_BASE + NRF52_UART_EVENTS_ERROR_OFFSET)
#define NRF52_UART_EVENTS_RXTO              (NRF52_UART0_BASE + NRF52_UART_EVENTS_RXTO_OFFSET)
#define NRF52_UART_SHORTS                   (NRF52_UART0_BASE + NRF52_UART_SHORTS_OFFSET)
#define NRF52_UART_INTENSET                 (NRF52_UART0_BASE + NRF52_UART_INTENSET_OFFSET)
#define NRF52_UART_INTENCLR                 (NRF52_UART0_BASE + NRF52_UART_INTENCLR_OFFSET)
#define NRF52_UART_ERRORSRC                 (NRF52_UART0_BASE + NRF52_UART_ERRORSRC_OFFSET)
#define NRF52_UART_ENABLE                   (NRF52_UART0_BASE + NRF52_UART_ENABLE_OFFSET)
#define NRF52_UART_PSELRTS                  (NRF52_UART0_BASE + NRF52_UART_PSELRTS_OFFSET)
#define NRF52_UART_PSELTXD                  (NRF52_UART0_BASE + NRF52_UART_PSELTXD_OFFSET)
#define NRF52_UART_PSELCTS                  (NRF52_UART0_BASE + NRF52_UART_PSELCTS_OFFSET)
#define NRF52_UART_PSELRXD                  (NRF52_UART0_BASE + NRF52_UART_PSELRXD_OFFSET)
#define NRF52_UART_RXD                      (NRF52_UART0_BASE + NRF52_UART_RXD_OFFSET)
#define NRF52_UART_TXD                      (NRF52_UART0_BASE + NRF52_UART_TXD_OFFSET)
#define NRF52_UART_BAUDRATE                 (NRF52_UART0_BASE + NRF52_UART_BAUDRATE_OFFSET)
#define NRF52_UART_CONFIG                   (NRF52_UART0_BASE + NRF52_UART_CONFIG_OFFSET)

/* UARTE Register Addresses **************************************************************************/

/* UART Register Bitfield Definitions ****************************************************************/

/* ENABLE Register */

#define NRF52_UART_ENABLE_DISABLE           (0)
#define NRF52_UART_ENABLE_ENABLE            (4)

/* INTENSET Register */

#define NRF52_UART_INTENSET_RXDRDY          (1 << 2)

#endif /* __ARCH_ARM_SRC_NRF52_CHIP_NRF52_UARTE_H */
