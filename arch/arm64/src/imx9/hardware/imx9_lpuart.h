/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx9_lpuart.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_LPUART_H
#define __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_LPUART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 32-bit register definition */

#define UARTVERID                   0x0000  /* Version ID Register */
#define UARTPARAM                   0x0004  /* Parameter Register */
#define UARTGLOBAL                  0x0008  /* LPUART Global Register */
#define UARTPINCFG                  0x000c  /* LPUART Pin Configuration Register */
#define UARTBAUD                    0x0010  /* LPUART Baud Rate Register */
#define UARTSTAT                    0x0014  /* LPUART Status Register */
#define UARTCTRL                    0x0018  /* LPUART Control Register */
#define UARTDATA                    0x001c  /* LPUART Data Register */
#define UARTMATCH                   0x0020  /* LPUART Match Address Register */
#define UARTMODIR                   0x0024  /* LPUART Modem IrDA Register */
#define UARTFIFO                    0x0028  /* LPUART FIFO Register */
#define UARTWATER                   0x002c  /* LPUART Watermark Register */

#define UARTBAUD_MAEN1              0x80000000
#define UARTBAUD_MAEN2              0x40000000
#define UARTBAUD_M10                0x20000000
#define UARTBAUD_TDMAE              0x00800000
#define UARTBAUD_RDMAE              0x00200000
#define UARTBAUD_RIDMAE             0x00100000
#define UARTBAUD_MATCFG             0x00400000
#define UARTBAUD_BOTHEDGE           0x00020000
#define UARTBAUD_RESYNCDIS          0x00010000
#define UARTBAUD_LBKDIE             0x00008000
#define UARTBAUD_RXEDGIE            0x00004000
#define UARTBAUD_SBNS               0x00002000
#define UARTBAUD_SBR                0x00000000
#define UARTBAUD_SBR_MASK           0x1fff
#define UARTBAUD_OSR_MASK           0x1f
#define UARTBAUD_OSR_SHIFT          24

#define UARTSTAT_LBKDIF             0x80000000
#define UARTSTAT_RXEDGIF            0x40000000
#define UARTSTAT_MSBF               0x20000000
#define UARTSTAT_RXINV              0x10000000
#define UARTSTAT_RWUID              0x08000000
#define UARTSTAT_BRK13              0x04000000
#define UARTSTAT_LBKDE              0x02000000
#define UARTSTAT_RAF                0x01000000
#define UARTSTAT_TDRE               0x00800000
#define UARTSTAT_TC                 0x00400000
#define UARTSTAT_RDRF               0x00200000
#define UARTSTAT_IDLE               0x00100000
#define UARTSTAT_OR                 0x00080000
#define UARTSTAT_NF                 0x00040000
#define UARTSTAT_FE                 0x00020000
#define UARTSTAT_PE                 0x00010000
#define UARTSTAT_MA1F               0x00008000
#define UARTSTAT_M21F               0x00004000

#define UARTCTRL_R8T9               0x80000000
#define UARTCTRL_R9T8               0x40000000
#define UARTCTRL_TXDIR              0x20000000
#define UARTCTRL_TXINV              0x10000000
#define UARTCTRL_ORIE               0x08000000
#define UARTCTRL_NEIE               0x04000000
#define UARTCTRL_FEIE               0x02000000
#define UARTCTRL_PEIE               0x01000000
#define UARTCTRL_TIE                0x00800000
#define UARTCTRL_TCIE               0x00400000
#define UARTCTRL_RIE                0x00200000
#define UARTCTRL_ILIE               0x00100000
#define UARTCTRL_TE                 0x00080000
#define UARTCTRL_RE                 0x00040000
#define UARTCTRL_RWU                0x00020000
#define UARTCTRL_SBK                0x00010000
#define UARTCTRL_MA1IE              0x00008000
#define UARTCTRL_MA2IE              0x00004000
#define UARTCTRL_IDLECFG_OFF        0x8
#define UARTCTRL_LOOPS              0x00000080
#define UARTCTRL_DOZEEN             0x00000040
#define UARTCTRL_RSRC               0x00000020
#define UARTCTRL_M                  0x00000010
#define UARTCTRL_WAKE               0x00000008
#define UARTCTRL_ILT                0x00000004
#define UARTCTRL_PE                 0x00000002
#define UARTCTRL_PT                 0x00000001

#define UARTDATA_NOISY              0x00008000
#define UARTDATA_PARITYE            0x00004000
#define UARTDATA_FRETSC             0x00002000
#define UARTDATA_RXEMPT             0x00001000
#define UARTDATA_IDLINE             0x00000800
#define UARTDATA_MASK               0x3ff

#define UARTMODIR_IREN              0x00020000
#define UARTMODIR_RTSWATER_S        0x8
#define UARTMODIR_TXCTSSRC          0x00000020
#define UARTMODIR_TXCTSC            0x00000010
#define UARTMODIR_RXRTSE            0x00000008
#define UARTMODIR_TXRTSPOL          0x00000004
#define UARTMODIR_TXRTSE            0x00000002
#define UARTMODIR_TXCTSE            0x00000001

#define UARTFIFO_TXEMPT             0x00800000
#define UARTFIFO_RXEMPT             0x00400000
#define UARTFIFO_TXOF               0x00020000
#define UARTFIFO_RXUF               0x00010000
#define UARTFIFO_TXFLUSH            0x00008000
#define UARTFIFO_RXFLUSH            0x00004000
#define UARTFIFO_RXIDEN_MASK        0x7
#define UARTFIFO_RXIDEN_OFF         10
#define UARTFIFO_TXOFE              0x00000200
#define UARTFIFO_RXUFE              0x00000100
#define UARTFIFO_TXFE               0x00000080
#define UARTFIFO_FIFOSIZE_MASK      0x7
#define UARTFIFO_TXSIZE_OFF         4
#define UARTFIFO_RXFE               0x00000008
#define UARTFIFO_RXSIZE_OFF         0
#define UARTFIFO_DEPTH(x)           (0x1 << ((x) ? ((x) + 1) : 0))

#define UARTWATER_COUNT_MASK        0xff
#define UARTWATER_TXCNT_OFF         8
#define UARTWATER_RXCNT_OFF         24
#define UARTWATER_WATER_MASK        0xff
#define UARTWATER_TXWATER_OFF       0
#define UARTWATER_RXWATER_OFF       16

#define UARTGLOBAL_RST              0x2

#define UARTFIFO_RXIDEN_RDRF        0x3
#define UARTCTRL_IDLECFG            0x7

#endif /* __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_LPUART_H */
