/***************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_uarte.h
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
 ***************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_UARTE_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_UARTE_H

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf53_memorymap.h"

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* UART/UARTE Register Offsets *********************************************/

#define NRF53_UARTE_TASKS_STARTRX_OFFSET    0x0000  /* Start UART receiver */
#define NRF53_UARTE_TASKS_STOPRX_OFFSET     0x0004  /* Stop UART receiver */
#define NRF53_UARTE_TASKS_STARTTX_OFFSET    0x0008  /* Start UART transmitter */
#define NRF53_UARTE_TASKS_STOPTX_OFFSET     0x000c  /* Stop UART transmitter */
#define NRF53_UARTE_TASKS_FLUSHRX_OFFSET    0x002c  /* Flush RX FIFO into RX buffer */
#define NRF53_UARTE_EVENTS_CTS_OFFSET       0x0100  /* CTS is activated (set low). Clear To Send. */
#define NRF53_UARTE_EVENTS_NCTS_OFFSET      0x0104  /* CTS is deactivated (set high). Not Clear To Send. */
#define NRF53_UARTE_EVENTS_RXDRDY_OFFSET    0x0108  /* Data received in RXD (but potentially not yet transferred to Data RAM) */
#define NRF53_UARTE_EVENTS_ENDRX_OFFSET     0x0110  /* Receive buffer is filled up */
#define NRF53_UARTE_EVENTS_TXDRDY_OFFSET    0x011c  /* Data sent from TXD */
#define NRF53_UARTE_EVENTS_ENDTX_OFFSET     0x0120  /* Last TX byte transmitted */
#define NRF53_UARTE_EVENTS_ERROR_OFFSET     0x0124  /* Error detected */
#define NRF53_UARTE_EVENTS_RXTO_OFFSET      0x0144  /* Receiver timeout */
#define NRF53_UARTE_EVENTS_RXSTARTED_OFFSET 0x014c  /* UART receiver has started */
#define NRF53_UARTE_EVENTS_TXSTARTED_OFFSET 0x0150  /* UART transmitter has started */
#define NRF53_UARTE_EVENTS_TXSTOPPED_OFFSET 0x0158  /* Transmitter stopped */
#define NRF53_UARTE_SHORTS_OFFSET           0x0200  /* Shortcut register */
#define NRF53_UARTE_INTEN_OFFSET            0x0300  /* Enable or disable interrupt */
#define NRF53_UARTE_INTENSET_OFFSET         0x0304  /* Enable interrupt */
#define NRF53_UARTE_INTENCLR_OFFSET         0x0308  /* Disable interrupt */
#define NRF53_UARTE_ERRORSRC_OFFSET         0x0480  /* Error source */
#define NRF53_UARTE_ENABLE_OFFSET           0x0500  /* Enable UART */
#define NRF53_UARTE_PSEL_RTS_OFFSET         0x0508  /* Pin select for RTS signal */
#define NRF53_UARTE_PSEL_TXD_OFFSET         0x050c  /* Pin select for TXD signal */
#define NRF53_UARTE_PSEL_CTS_OFFSET         0x0510  /* Pin select for CTS signal */
#define NRF53_UARTE_PSEL_RXD_OFFSET         0x0514  /* Pin select for RXD signal */
#define NRF53_UARTE_BAUDRATE_OFFSET         0x0524  /* Baud rate. Accuracy depends on the HFCLK source selected. */
#define NRF53_UARTE_RXD_PTR_OFFSET          0x0534  /* Data pointer */
#define NRF53_UARTE_RXD_MAXCNT_OFFSET       0x0538  /* Maximum number of bytes in receive buffer */
#define NRF53_UARTE_RXD_AMOUNT_OFFSET       0x053c  /* Number of bytes transferred in the last transaction */
#define NRF53_UARTE_TXD_PTR_OFFSET          0x0544  /* Data pointer */
#define NRF53_UARTE_TXD_MAXCNT_OFFSET       0x0548  /* Maximum number of bytes in transmit buffer */
#define NRF53_UARTE_TXD_AMOUNT_OFFSET       0x054c  /* Number of bytes transferred in the last transaction */
#define NRF53_UARTE_CONFIG_OFFSET           0x056c  /* Configuration of parity and hardware flow control */

#define NRF53_UART_TASKS_STARTRX_OFFSET     0x0000 /* Start UART receiver */
#define NRF53_UART_TASKS_STOPRX_OFFSET      0x0004 /* Stop UART receiver */
#define NRF53_UART_TASKS_STARTTX_OFFSET     0x0008 /* Start UART transmitter */
#define NRF53_UART_TASKS_STOPTX_OFFSET      0x000c /* Stop UART transmitter */
#define NRF53_UART_TASKS_SUSPEND_OFFSET     0x001c /* Suspend UART */
#define NRF53_UART_EVENTS_CTS_OFFSET        0x0100 /* CTS is activated (set low). Clear To Send. */
#define NRF53_UART_EVENTS_NCTS_OFFSET       0x0104 /* CTS is deactivated (set high). Not Clear To Send. */
#define NRF53_UART_EVENTS_RXDRDY_OFFSET     0x0108 /* Data received in RXD */
#define NRF53_UART_EVENTS_TXDRDY_OFFSET     0x011c /* Data sent from TXD */
#define NRF53_UART_EVENTS_ERROR_OFFSET      0x0124 /* Error detected */
#define NRF53_UART_EVENTS_RXTO_OFFSET       0x0144 /* Receiver timeout */
#define NRF53_UART_SHORTS_OFFSET            0x0200 /* Shortcut register */
#define NRF53_UART_INTENSET_OFFSET          0x0304 /* Enable interrupt */
#define NRF53_UART_INTENCLR_OFFSET          0x0308 /* Disable interrupt */
#define NRF53_UART_ERRORSRC_OFFSET          0x0480 /* Error source */
#define NRF53_UART_ENABLE_OFFSET            0x0500 /* Enable UART */
#define NRF53_UART_PSELRTS_OFFSET           0x0508 /* Pin select for RTS */
#define NRF53_UART_PSELTXD_OFFSET           0x050c /* Pin select for TXD */
#define NRF53_UART_PSELCTS_OFFSET           0x0510 /* Pin select for CTS */
#define NRF53_UART_PSELRXD_OFFSET           0x0514 /* Pin select for RXD */
#define NRF53_UART_RXD_OFFSET               0x0518 /* RXD register */
#define NRF53_UART_TXD_OFFSET               0x051c /* TXD register */
#define NRF53_UART_BAUDRATE_OFFSET          0x0524 /* Baud rate */
#define NRF53_UART_CONFIG_OFFSET            0x056c /* Configuration of parity and hardware flow control */

/* UART/UARTE Register Addresses *******************************************/

#define NRF53_UARTE0_TASKS_STARTRX          (NRF53_UARTE0_BASE + NRF53_UARTE_TASKS_STARTRX_OFFSET)
#define NRF53_UARTE0_TASKS_STOPRX           (NRF53_UARTE0_BASE + NRF53_UARTE_TASKS_STOPRX_OFFSET)
#define NRF53_UARTE0_TASKS_STARTTX          (NRF53_UARTE0_BASE + NRF53_UARTE_TASKS_STARTTX_OFFSET)
#define NRF53_UARTE0_TASKS_STOPTX           (NRF53_UARTE0_BASE + NRF53_UARTE_TASKS_STOPTX_OFFSET)
#define NRF53_UARTE0_TASKS_FLUSHRX          (NRF53_UARTE0_BASE + NRF53_UARTE_TASKS_FLUSHRX_OFFSET)
#define NRF53_UARTE0_EVENTS_CTS             (NRF53_UARTE0_BASE + NRF53_UARTE_EVENTS_CTS_OFFSET)
#define NRF53_UARTE0_EVENTS_NCTS            (NRF53_UARTE0_BASE + NRF53_UARTE_EVENTS_NCTS_OFFSET)
#define NRF53_UARTE0_EVENTS_RXDRDY          (NRF53_UARTE0_BASE + NRF53_UARTE_EVENTS_RXDRDY_OFFSET)
#define NRF53_UARTE0_EVENTS_ENDRX           (NRF53_UARTE0_BASE + NRF53_UARTE_EVENTS_ENDRX_OFFSET)
#define NRF53_UARTE0_EVENTS_TXDRDY          (NRF53_UARTE0_BASE + NRF53_UARTE_EVENTS_TXDRDY_OFFSET)
#define NRF53_UARTE0_EVENTS_ENDTX           (NRF53_UARTE0_BASE + NRF53_UARTE_EVENTS_ENDTX_OFFSET)
#define NRF53_UARTE0_EVENTS_ERROR           (NRF53_UARTE0_BASE + NRF53_UARTE_EVENTS_ERROR_OFFSET)
#define NRF53_UARTE0_EVENTS_RXTO            (NRF53_UARTE0_BASE + NRF53_UARTE_EVENTS_RXTO_OFFSET)
#define NRF53_UARTE0_EVENTS_RXSTARTED       (NRF53_UARTE0_BASE + NRF53_UARTE_EVENTS_RXSTARTED_OFFSET)
#define NRF53_UARTE0_EVENTS_TXSTARTED       (NRF53_UARTE0_BASE + NRF53_UARTE_EVENTS_TXSTARTED_OFFSET)
#define NRF53_UARTE0_EVENTS_TXSTOPPED       (NRF53_UARTE0_BASE + NRF53_UARTE_EVENTS_TXSTOPPED_OFFSET)
#define NRF53_UARTE0_SHORTS                 (NRF53_UARTE0_BASE + NRF53_UARTE_SHORTS_OFFSET)
#define NRF53_UARTE0_INTEN                  (NRF53_UARTE0_BASE + NRF53_UARTE_INTEN_OFFSET)
#define NRF53_UARTE0_INTENSET               (NRF53_UARTE0_BASE + NRF53_UARTE_INTENSET_OFFSET)
#define NRF53_UARTE0_INTENCLR               (NRF53_UARTE0_BASE + NRF53_UARTE_INTENCLR_OFFSET)
#define NRF53_UARTE0_ERRORSRC               (NRF53_UARTE0_BASE + NRF53_UARTE_ERRORSRC_OFFSET)
#define NRF53_UARTE0_ENABLE                 (NRF53_UARTE0_BASE + NRF53_UARTE_ENABLE_OFFSET)
#define NRF53_UARTE0_PSEL_RTS               (NRF53_UARTE0_BASE + NRF53_UARTE_PSEL_RTS_OFFSET)
#define NRF53_UARTE0_PSEL_TXD               (NRF53_UARTE0_BASE + NRF53_UARTE_PSEL_TXD_OFFSET)
#define NRF53_UARTE0_PSEL_CTS               (NRF53_UARTE0_BASE + NRF53_UARTE_PSEL_CTS_OFFSET)
#define NRF53_UARTE0_PSEL_RXD               (NRF53_UARTE0_BASE + NRF53_UARTE_PSEL_RXD_OFFSET)
#define NRF53_UARTE0_BAUDRATE               (NRF53_UARTE0_BASE + NRF53_UARTE_BAUDRATE_OFFSET)
#define NRF53_UARTE0_RXD_PTR                (NRF53_UARTE0_BASE + NRF53_UARTE_RXD_PTR_OFFSET)
#define NRF53_UARTE0_RXD_MAXCNT             (NRF53_UARTE0_BASE + NRF53_UARTE_RXD_MAXCNT_OFFSET)
#define NRF53_UARTE0_RXD_AMOUNT             (NRF53_UARTE0_BASE + NRF53_UARTE_RXD_AMOUNT_OFFSET)
#define NRF53_UARTE0_TXD_PTR                (NRF53_UARTE0_BASE + NRF53_UARTE_TXD_PTR_OFFSET)
#define NRF53_UARTE0_TXD_MAXCNT             (NRF53_UARTE0_BASE + NRF53_UARTE_TXD_MAXCNT_OFFSET)
#define NRF53_UARTE0_TXD_AMOUNT             (NRF53_UARTE0_BASE + NRF53_UARTE_TXD_AMOUNT_OFFSET)
#define NRF53_UARTE0_CONFIG                 (NRF53_UARTE0_BASE + NRF53_UARTE_CONFIG_OFFSET)

#define NRF53_UART0_TASKS_STARTRX           (NRF53_UART0_BASE + NRF53_UART_TASKS_STARTRX_OFFSET)
#define NRF53_UART0_TASKS_STOPRX            (NRF53_UART0_BASE + NRF53_UART_TASKS_STOPRX_OFFSET)
#define NRF53_UART0_TASKS_STARTTX           (NRF53_UART0_BASE + NRF53_UART_TASKS_STARTTX_OFFSET)
#define NRF53_UART0_TASKS_STOPTX            (NRF53_UART0_BASE + NRF53_UART_TASKS_STOPTX_OFFSET)
#define NRF53_UART0_TASKS_SUSPEND           (NRF53_UART0_BASE + NRF53_UART_TASKS_SUSPEND_OFFSET)
#define NRF53_UART0_EVENTS_CTS              (NRF53_UART0_BASE + NRF53_UART_EVENTS_CTS_OFFSET)
#define NRF53_UART0_EVENTS_NCTS             (NRF53_UART0_BASE + NRF53_UART_EVENTS_NCTS_OFFSET)
#define NRF53_UART0_EVENTS_RXDRDY           (NRF53_UART0_BASE + NRF53_UART_EVENTS_RXDRDY_OFFSET)
#define NRF53_UART0_EVENTS_TXDRDY           (NRF53_UART0_BASE + NRF53_UART_EVENTS_TXDRDY_OFFSET)
#define NRF53_UART0_EVENTS_ERROR            (NRF53_UART0_BASE + NRF53_UART_EVENTS_ERROR_OFFSET)
#define NRF53_UART0_EVENTS_RXTO             (NRF53_UART0_BASE + NRF53_UART_EVENTS_RXTO_OFFSET)
#define NRF53_UART0_SHORTS                  (NRF53_UART0_BASE + NRF53_UART_SHORTS_OFFSET)
#define NRF53_UART0_INTENSET                (NRF53_UART0_BASE + NRF53_UART_INTENSET_OFFSET)
#define NRF53_UART0_INTENCLR                (NRF53_UART0_BASE + NRF53_UART_INTENCLR_OFFSET)
#define NRF53_UART0_ERRORSRC                (NRF53_UART0_BASE + NRF53_UART_ERRORSRC_OFFSET)
#define NRF53_UART0_ENABLE                  (NRF53_UART0_BASE + NRF53_UART_ENABLE_OFFSET)
#define NRF53_UART0_PSELRTS                 (NRF53_UART0_BASE + NRF53_UART_PSELRTS_OFFSET)
#define NRF53_UART0_PSELTXD                 (NRF53_UART0_BASE + NRF53_UART_PSELTXD_OFFSET)
#define NRF53_UART0_PSELCTS                 (NRF53_UART0_BASE + NRF53_UART_PSELCTS_OFFSET)
#define NRF53_UART0_PSELRXD                 (NRF53_UART0_BASE + NRF53_UART_PSELRXD_OFFSET)
#define NRF53_UART0_RXD                     (NRF53_UART0_BASE + NRF53_UART_RXD_OFFSET)
#define NRF53_UART0_TXD                     (NRF53_UART0_BASE + NRF53_UART_TXD_OFFSET)
#define NRF53_UART0_BAUDRATE                (NRF53_UART0_BASE + NRF53_UART_BAUDRATE_OFFSET)
#define NRF53_UART0_CONFIG                  (NRF53_UART0_BASE + NRF53_UART_CONFIG_OFFSET)

#ifdef CONFIG_NRF53_UART1
#  define NRF53_UARTE1_TASKS_STARTRX        (NRF53_UARTE1_BASE + NRF53_UARTE_TASKS_STARTRX_OFFSET)
#  define NRF53_UARTE1_TASKS_STOPRX         (NRF53_UARTE1_BASE + NRF53_UARTE_TASKS_STOPRX_OFFSET)
#  define NRF53_UARTE1_TASKS_STARTTX        (NRF53_UARTE1_BASE + NRF53_UARTE_TASKS_STARTTX_OFFSET)
#  define NRF53_UARTE1_TASKS_STOPTX         (NRF53_UARTE1_BASE + NRF53_UARTE_TASKS_STOPTX_OFFSET)
#  define NRF53_UARTE1_TASKS_FLUSHRX        (NRF53_UARTE1_BASE + NRF53_UARTE_TASKS_FLUSHRX_OFFSET)
#  define NRF53_UARTE1_EVENTS_CTS           (NRF53_UARTE1_BASE + NRF53_UARTE_EVENTS_CTS_OFFSET)
#  define NRF53_UARTE1_EVENTS_NCTS          (NRF53_UARTE1_BASE + NRF53_UARTE_EVENTS_NCTS_OFFSET)
#  define NRF53_UARTE1_EVENTS_RXDRDY        (NRF53_UARTE1_BASE + NRF53_UARTE_EVENTS_RXDRDY_OFFSET)
#  define NRF53_UARTE1_EVENTS_ENDRX         (NRF53_UARTE1_BASE + NRF53_UARTE_EVENTS_ENDRX_OFFSET)
#  define NRF53_UARTE1_EVENTS_TXDRDY        (NRF53_UARTE1_BASE + NRF53_UARTE_EVENTS_TXDRDY_OFFSET)
#  define NRF53_UARTE1_EVENTS_ENDTX         (NRF53_UARTE1_BASE + NRF53_UARTE_EVENTS_ENDTX_OFFSET)
#  define NRF53_UARTE1_EVENTS_ERROR         (NRF53_UARTE1_BASE + NRF53_UARTE_EVENTS_ERROR_OFFSET)
#  define NRF53_UARTE1_EVENTS_RXTO          (NRF53_UARTE1_BASE + NRF53_UARTE_EVENTS_RXTO_OFFSET)
#  define NRF53_UARTE1_EVENTS_RXSTARTED     (NRF53_UARTE1_BASE + NRF53_UARTE_EVENTS_RXSTARTED_OFFSET)
#  define NRF53_UARTE1_EVENTS_TXSTARTED     (NRF53_UARTE1_BASE + NRF53_UARTE_EVENTS_TXSTARTED_OFFSET)
#  define NRF53_UARTE1_EVENTS_TXSTOPPED     (NRF53_UARTE1_BASE + NRF53_UARTE_EVENTS_TXSTOPPED_OFFSET)
#  define NRF53_UARTE1_SHORTS               (NRF53_UARTE1_BASE + NRF53_UARTE_SHORTS_OFFSET)
#  define NRF53_UARTE1_INTEN                (NRF53_UARTE1_BASE + NRF53_UARTE_INTEN_OFFSET)
#  define NRF53_UARTE1_INTENSET             (NRF53_UARTE1_BASE + NRF53_UARTE_INTENSET_OFFSET)
#  define NRF53_UARTE1_INTENCLR             (NRF53_UARTE1_BASE + NRF53_UARTE_INTENCLR_OFFSET)
#  define NRF53_UARTE1_ERRORSRC             (NRF53_UARTE1_BASE + NRF53_UARTE_ERRORSRC_OFFSET)
#  define NRF53_UARTE1_ENABLE               (NRF53_UARTE1_BASE + NRF53_UARTE_ENABLE_OFFSET)
#  define NRF53_UARTE1_PSEL_RTS             (NRF53_UARTE1_BASE + NRF53_UARTE_PSEL_RTS_OFFSET)
#  define NRF53_UARTE1_PSEL_TXD             (NRF53_UARTE1_BASE + NRF53_UARTE_PSEL_TXD_OFFSET)
#  define NRF53_UARTE1_PSEL_CTS             (NRF53_UARTE1_BASE + NRF53_UARTE_PSEL_CTS_OFFSET)
#  define NRF53_UARTE1_PSEL_RXD             (NRF53_UARTE1_BASE + NRF53_UARTE_PSEL_RXD_OFFSET)
#  define NRF53_UARTE1_BAUDRATE             (NRF53_UARTE1_BASE + NRF53_UARTE_BAUDRATE_OFFSET)
#  define NRF53_UARTE1_RXD_PTR              (NRF53_UARTE1_BASE + NRF53_UARTE_RXD_PTR_OFFSET)
#  define NRF53_UARTE1_RXD_MAXCNT           (NRF53_UARTE1_BASE + NRF53_UARTE_RXD_MAXCNT_OFFSET)
#  define NRF53_UARTE1_RXD_AMOUNT           (NRF53_UARTE1_BASE + NRF53_UARTE_RXD_AMOUNT_OFFSET)
#  define NRF53_UARTE1_TXD_PTR              (NRF53_UARTE1_BASE + NRF53_UARTE_TXD_PTR_OFFSET)
#  define NRF53_UARTE1_TXD_MAXCNT           (NRF53_UARTE1_BASE + NRF53_UARTE_TXD_MAXCNT_OFFSET)
#  define NRF53_UARTE1_TXD_AMOUNT           (NRF53_UARTE1_BASE + NRF53_UARTE_TXD_AMOUNT_OFFSET)
#  define NRF53_UARTE1_CONFIG               (NRF53_UARTE1_BASE + NRF53_UARTE_CONFIG_OFFSET)

#  define NRF53_UART1_TASKS_STARTRX         (NRF53_UART1_BASE + NRF53_UART_TASKS_STARTRX_OFFSET)
#  define NRF53_UART1_TASKS_STOPRX          (NRF53_UART1_BASE + NRF53_UART_TASKS_STOPRX_OFFSET)
#  define NRF53_UART1_TASKS_STARTTX         (NRF53_UART1_BASE + NRF53_UART_TASKS_STARTTX_OFFSET)
#  define NRF53_UART1_TASKS_STOPTX          (NRF53_UART1_BASE + NRF53_UART_TASKS_STOPTX_OFFSET)
#  define NRF53_UART1_TASKS_SUSPEND         (NRF53_UART1_BASE + NRF53_UART_TASKS_SUSPEND_OFFSET)
#  define NRF53_UART1_EVENTS_CTS            (NRF53_UART1_BASE + NRF53_UART_EVENTS_CTS_OFFSET)
#  define NRF53_UART1_EVENTS_NCTS           (NRF53_UART1_BASE + NRF53_UART_EVENTS_NCTS_OFFSET)
#  define NRF53_UART1_EVENTS_RXDRDY         (NRF53_UART1_BASE + NRF53_UART_EVENTS_RXDRDY_OFFSET)
#  define NRF53_UART1_EVENTS_TXDRDY         (NRF53_UART1_BASE + NRF53_UART_EVENTS_TXDRDY_OFFSET)
#  define NRF53_UART1_EVENTS_ERROR          (NRF53_UART1_BASE + NRF53_UART_EVENTS_ERROR_OFFSET)
#  define NRF53_UART1_EVENTS_RXTO           (NRF53_UART1_BASE + NRF53_UART_EVENTS_RXTO_OFFSET)
#  define NRF53_UART1_SHORTS                (NRF53_UART1_BASE + NRF53_UART_SHORTS_OFFSET)
#  define NRF53_UART1_INTENSET              (NRF53_UART1_BASE + NRF53_UART_INTENSET_OFFSET)
#  define NRF53_UART1_INTENCLR              (NRF53_UART1_BASE + NRF53_UART_INTENCLR_OFFSET)
#  define NRF53_UART1_ERRORSRC              (NRF53_UART1_BASE + NRF53_UART_ERRORSRC_OFFSET)
#  define NRF53_UART1_ENABLE                (NRF53_UART1_BASE + NRF53_UART_ENABLE_OFFSET)
#  define NRF53_UART1_PSELRTS               (NRF53_UART1_BASE + NRF53_UART_PSELRTS_OFFSET)
#  define NRF53_UART1_PSELTXD               (NRF53_UART1_BASE + NRF53_UART_PSELTXD_OFFSET)
#  define NRF53_UART1_PSELCTS               (NRF53_UART1_BASE + NRF53_UART_PSELCTS_OFFSET)
#  define NRF53_UART1_PSELRXD               (NRF53_UART1_BASE + NRF53_UART_PSELRXD_OFFSET)
#  define NRF53_UART1_RXD                   (NRF53_UART1_BASE + NRF53_UART_RXD_OFFSET)
#  define NRF53_UART1_TXD                   (NRF53_UART1_BASE + NRF53_UART_TXD_OFFSET)
#  define NRF53_UART1_BAUDRATE              (NRF53_UART1_BASE + NRF53_UART_BAUDRATE_OFFSET)
#  define NRF53_UART1_CONFIG                (NRF53_UART1_BASE + NRF53_UART_CONFIG_OFFSET)
#endif

/* UART Register Bitfield Definitions **************************************/

/* PSELRTS Register */

#define UART_PSELRTS_PIN_SHIFT              (0)       /* Bits 0-4: Pin number*/
#define UART_PSELRTS_PIN_MASK               (0x1f << UART_PSELRTS_PIN_SHIFT)
#define UART_PSELRTS_PORT_SHIFT             (5)       /* Bit 5: Port number */
#define UART_PSELRTS_PORT_MASK              (0x1 << UART_PSELRTS_PORT_SHIFT)
#define UART_PSELRTS_CONNECT                (1 << 31) /* Bit 31: Connection */
#define UART_PSELRTS_RESET                  (0xffffffff)

/* PSELTXD Register */

#define UART_PSELTXD_PIN_SHIFT              (0)       /* Bits 0-4: Pin number*/
#define UART_PSELTXD_PIN_MASK               (0x1f << UART_PSELTXD_PIN_SHIFT)
#define UART_PSELTXD_PORT_SHIFT             (5)       /* Bit 5: Port number */
#define UART_PSELTXD_PORT_MASK              (0x1 << UART_PSELTXD_PORT_SHIFT)
#define UART_PSELTXD_CONNECT                (1 << 31) /* Bit 31: Connection */
#define UART_PSELTXD_RESET                  (0xffffffff)

/* PSELCTS Register */

#define UART_PSELCTS_PIN_SHIFT              (0)       /* Bits 0-4: Pin number*/
#define UART_PSELCTS_PIN_MASK               (0x1f << UART_PSELCTS_PIN_SHIFT)
#define UART_PSELCTS_PORT_SHIFT             (5)       /* Bit 5: Port number */
#define UART_PSELCTS_PORT_MASK              (0x1 << UART_PSELCTS_PORT_SHIFT)
#define UART_PSELCTS_CONNECT                (1 << 31) /* Bit 31: Connection */
#define UART_PSELCTS_RESET                  (0xffffffff)

/* PSELRXD Register */

#define UART_PSELRXD_PIN_SHIFT              (0)       /* Bits 0-4: Pin number*/
#define UART_PSELRXD_PIN_MASK               (0x1f << UART_PSELRXD_PIN_SHIFT)
#define UART_PSELRXD_PORT_SHIFT             (5)       /* Bit 5: Port number */
#define UART_PSELRXD_PORT_MASK              (0x1 << UART_PSELRXD_PORT_SHIFT)
#define UART_PSELRXD_CONNECT                (1 << 31) /* Bit 31: Connection */
#define UART_PSELRXD_RESET                  (0xffffffff)

/* ENABLE Register */

#define NRF53_UART_ENABLE_DISABLE           (0)
#define NRF53_UART_ENABLE_ENABLE            (4)

/* INTEN/INTENSET/INTENCLR Register */

#define UART_INT_CTS                        (1 << 0)
#define UART_INT_NCTS                       (1 << 1)
#define UART_INT_RXDRDY                     (1 << 2)
#define UART_INT_ENDRX                      (1 << 4)
#define UART_INT_TXDRDY                     (1 << 7)
#define UART_INT_ENDTX                      (1 << 8)
#define UART_INT_ERROR                      (1 << 9)
#define UART_INT_RXTO                       (1 << 17)
#define UART_INT_RXSTARTED                  (1 << 19)
#define UART_INT_TXSTARTED                  (1 << 20)
#define UART_INT_TXSTOPPED                  (1 << 22)

/* BAUDRATE Register */

#define UART_BAUDRATE_1200                  (0x0004f000)
#define UART_BAUDRATE_2400                  (0x0009d000)
#define UART_BAUDRATE_4800                  (0x0013b000)
#define UART_BAUDRATE_9600                  (0x00275000)
#define UART_BAUDRATE_14400                 (0x003af000)
#define UART_BAUDRATE_19200                 (0x004ea000)
#define UART_BAUDRATE_28800                 (0x0075c000)
#define UART_BAUDRATE_31250                 (0x00800000)
#define UART_BAUDRATE_38400                 (0x009d0000)
#define UART_BAUDRATE_56000                 (0x00e50000)
#define UART_BAUDRATE_57600                 (0x00eb0000)
#define UART_BAUDRATE_76000                 (0x013a9000)
#define UART_BAUDRATE_115200                (0x01d60000)
#define UART_BAUDRATE_230400                (0x03b00000)
#define UART_BAUDRATE_250000                (0x04000000)
#define UART_BAUDRATE_460800                (0x07400000)
#define UART_BAUDRATE_921600                (0x0f000000)
#define UART_BAUDRATE_1000000               (0x10000000)

/* CONFIG Register */

#define UART_CONFIG_HWFC                    (1 << 0) /* Bit 0: Hardware flow control */
#define UART_CONFIG_PARITY_SHIFT            (1)      /* Bits 1-3: Parity */
#define UART_CONFIG_PARITY                  (7 << UART_CONFIG_PARITY_SHIFT)
#define UART_CONFIG_STOP                    (1 << 4) /* Bit 4: Stop bits */
#define UART_CONFIG_PARITYTYPE              (1 << 8) /* Bit 8: Parity type */

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_UARTE_H */
