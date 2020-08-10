/****************************************************************************
 * arch/arm/src/lc823450/lc823450_serial.h
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

#ifndef __ARCH_ARM_SRC_LC823450_LC823450_SERIAL_H
#define __ARCH_ARM_SRC_LC823450_LC823450_SERIAL_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_DEV_CONSOLE_SWITCH
extern int g_console_disable;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_DEV_CONSOLE_SWITCH
void up_console_disable(int disable);
#endif

#ifdef CONFIG_HSUART
void hsuart_wdtimer(void);
#endif

#endif /* __ARCH_ARM_SRC_LC823450_LC823450_SERIAL_H */
