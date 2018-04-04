/****************************************************************************
 * arch/arm/src/lc823450/lc823450_serial.h
 *
 *   Copyright 2014,2015,2016,2017 Sony Video & Sound Products Inc.
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_LC823450_LC823450_SERIAL_H
#define __ARCH_ARM_SRC_LC823450_LC823450_SERIAL_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define LC823450_UART0_REGBASE  0x4008b000
#define LC823450_UART1_REGBASE  0x4008c000
#define LC823450_UART2_REGBASE  0x4008d000

#define UART_UTR                0x00
#define UART_URR                0x04
#define UART_UMD                0x08
#define UART_UMD_CL             (1 << 0)
#define UART_UMD_PS0            (1 << 1)
#define UART_UMD_PS1            (1 << 2)
#define UART_UMD_STL            (1 << 3)
#define UART_UMD_RTSEN          (1 << 5)
#define UART_UMD_CTSEN          (1 << 6)
#define UART_UCM                0x0c
#define UART_UCM_RE             (1 << 0)
#define UART_UCM_TE             (1 << 1)
#define UART_USR                0x10
#define UART_USR_TFF            (1 << 0)
#define UART_USR_RRF            (1 << 1)
#define UART_USR_TXFULL         (1 << 8)
#define UART_USR_TXEMP          (1 << 9)
#define UART_USR_RXEMP          (1 << 13)
#define UART_UBR                0x14
#define UART_UISR               0x18
#define UART_UISR_UARTRF        (1 << 0)
#define UART_UISR_UARTTF        (1 << 1)
#define UART_UISR_PE            (1 << 2)
#define UART_UISR_ROWE          (1 << 3)
#define UART_UISR_FE            (1 << 4)
#define UART_UISR_RXOWE         (1 << 12)
#define UART_UDIV               0x1c
#define UART_UIEN               0x20
#define UART_UIEN_UARTRF_IEN    (1 << 0)
#define UART_UIEN_UARTTF_IEN    (1 << 1)
#define UART_UIEN_PE_IEN        (1 << 2)
#define UART_UIEN_ROWE_IEN      (1 << 3)
#define UART_UIEN_FE_IEN        (1 << 4)
#define UART_UIEN_RXOWE_IEN     (1 << 12)
#define UART_UINT               0x24
#define UART_UINT_UARTRF_INT    (1 << 0)
#define UART_UINT_UARTTF_INT    (1 << 1)
#define UART_USTF               0x28
#define UART_USRF               0x2c
#define UART_USFC               0x30
#define UART_USFC_RXFF_EN       (1 << 3)
#define UART_USFC_TXFF_EN       (1 << 7)
#define UART_UDMA               0x34
#define UART_UDMA_RREQ_EN       (1 << 0)
#define UART_UDMA_TREQ_EN       (1 << 1)
#define UART_USFS               0x38
#define UART_USFS_TXFF_LV(v)    (((v) >> 8 ) & 0x1f)
#define UART_USFS_RXFF_LV(v)    (((v) >> 0 ) & 0x1f)

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifdef CONFIG_DEV_CONSOLE_SWITCH
extern int g_console_disable;
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifdef CONFIG_DEV_CONSOLE_SWITCH
void up_console_disable(int disable);
#endif

#ifdef CONFIG_HSUART
void hsuart_wdtimer(void);
#endif

#endif /* __ARCH_ARM_SRC_LC823450_LC823450_SERIAL_H */
