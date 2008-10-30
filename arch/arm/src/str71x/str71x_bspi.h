/************************************************************************************
 * arch/arm/src/str71x/str71x_bspi.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_BSPI_H
#define __ARCH_ARM_SRC_STR71X_STR71X_BSPI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include "str71x_map.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STR71X_BSPI_RXR_OFFSET  (0x0000) /* 16-bits wide */
#define STR71X_BSPI_TXR_OFFSET  (0x0004) /* 16-bits wide */
#define STR71X_BSPI_CSR1_OFFSET (0x0008) /* 16-bits wide */
#define STR71X_BSPI_CSR2_OFFSET (0x000c) /* 16-bits wide */
#define STR71X_BSPI_CLK_OFFSET  (0x0010) /* 16-bits wide */

/* Registers ************************************************************************/

#define STR71X_BSPI_RXR(b)      ((b) + STR71X_BSPI_RXR_OFFSET)
#define STR71X_BSPI_TXR(b)      ((b) + STR71X_BSPI_TXR_OFFSET)
#define STR71X_BSPI_CSR1(b)     ((b) + STR71X_BSPI_CSR1_OFFSET)
#define STR71X_BSPI_CSR2(b)     ((b) + STR71X_BSPI_CSR2_OFFSET)
#define STR71X_BSPI_CLK(b)      ((b) + STR71X_BSPI_CLK_OFFSET)

#define STR71X_BSPI0_RXR        (STR71X_BSPI0_BASE + STR71X_BSPI_RXR_OFFSET)
#define STR71X_BSPI0_TXR        (STR71X_BSPI0_BASE + STR71X_BSPI_TXR_OFFSET)
#define STR71X_BSPI0_CSR1       (STR71X_BSPI0_BASE + STR71X_BSPI_CSR1_OFFSET)
#define STR71X_BSPI0_CSR2       (STR71X_BSPI0_BASE + STR71X_BSPI_CSR2_OFFSET)
#define STR71X_BSPI0_CLK        (STR71X_BSPI0_BASE + STR71X_BSPI_CLK_OFFSET)

#define STR71X_BSPI1_RXR        (STR71X_BSPI1_BASE + STR71X_BSPI_RXR_OFFSET)
#define STR71X_BSPI1_TXR        (STR71X_BSPI1_BASE + STR71X_BSPI_TXR_OFFSET)
#define STR71X_BSPI1_CSR1       (STR71X_BSPI1_BASE + STR71X_BSPI_CSR1_OFFSET)
#define STR71X_BSPI1_CSR2       (STR71X_BSPI1_BASE + STR71X_BSPI_CSR2_OFFSET)
#define STR71X_BSPI1_CLK        (STR71X_BSPI1_BASE + STR71X_BSPI_CLK_OFFSET)

/* Register bit settings ***********************************************************/

/* BSPI control/status register 1 */

#define STR71X_BSPICSR1_BSPE        0x0001 /* Bit 0: BSPI enable */
#define STR71X_BSPICSR1_MSTR        0x0002 /* Bit 1: Master/Slave select */
#define STR71X_BSPICSR1_RIEMASK     0x000c /* Bit 2-3: BSPI receive interrupt enable */
#define STR71X_BSPICSR1_RIEDISABLED 0x0000 /*   Disabled */
#define STR71X_BSPICSR1_RIERFNE     0x0004 /*   Receive FIFO not empty */
#define STR71X_BSPICSR1_RIERFF      0x000c /*   Receive FIFO full */
#define STR71X_BSPICSR1_BEIE        0x0080 /* Bit 7: Bus error interrupt enable */
#define STR71X_BSPICSR1_CPOL        0x0100 /* Bit 8: Clock polarity select */
#define STR71X_BSPICSR1_CPHA        0x0200 /* Bit 9: Clock phase select */
#define STR71X_BSPICSR1_WLMASK      0x0c00 /* Bits 10-11: Word length */
#define STR71X_BSPICSR1_WL8BIT      0x0000 /*   8-bits */
#define STR71X_BSPICSR1_WL16BIT     0x0400 /*   16-bits */
#define STR71X_BSPICSR1_RFEMASK     0xf000 /* Bits 12-15: Receive FIFO enable */
#define STR71X_BSPICSR1_RFE1        0x0000 /*   Word 1 enabled */
#define STR71X_BSPICSR1_RFE12       0x1000 /*   Word 1-2 enabled */
#define STR71X_BSPICSR1_RFE13       0x2000 /*   Word 1-3 enabled */
#define STR71X_BSPICSR1_RFE14       0x3000 /*   Word 1-4 enabled */
#define STR71X_BSPICSR1_RFE15       0x4000 /*   Word 1-5 enabled */
#define STR71X_BSPICSR1_RFE16       0x5000 /*   Word 1-6 enabled */
#define STR71X_BSPICSR1_RFE17       0x6000 /*   Word 1-7 enabled */
#define STR71X_BSPICSR1_RFE18       0x7000 /*   Word 1-8 enabled */
#define STR71X_BSPICSR1_RFE19       0x8000 /*   Word 1-9 enabled */
#define STR71X_BSPICSR1_RFE110      0x9000 /*   Word 1-10 enabled */

/* BSPI control/status register 1 */

#define STR71X_BSPICSR2_DFIFO       0x0000 /* Bit 0: FIFO disable */
#define STR71X_BSPICSR2_BERR        0x0000 /* Bit 2: Bus error */
#define STR71X_BSPICSR2_RFNE        0x0000 /* Bit 3: Receiver FIFO not empty */
#define STR71X_BSPICSR2_RFF         0x0000 /* Bit 4: Receiver FIFO full */
#define STR71X_BSPICSR2_ROFL        0x0000 /* Bit 5: Receiver overflow */
#define STR71X_BSPICSR2_TFE         0x0000 /* Bit 6: Transmit FIFO empty */
#define STR71X_BSPICSR2_TUFL        0x0000 /* Bit 7: Transmit FIFO underflow */
#define STR71X_BSPICSR2_TFF         0x0000 /* Bit 8: Transmit FIFO full */
#define STR71X_BSPICSR2_TFNE        0x0000 /* Bit 9: Transmit FIFO not empty */
#define STR71X_BSPICSR2_TFEMASK     0x3c00 /* Bit 10-13:  Transmit FIFO enable*/
#define STR71X_BSPICSR2_TFE1        0x0000 /*   Word 1 enabled  */
#define STR71X_BSPICSR2_TFE12       0x0000 /*   Word 1-2 enabled  */
#define STR71X_BSPICSR2_TFE13       0x0000 /*   Word 1-3 enabled  */
#define STR71X_BSPICSR2_TFE14       0x0000 /*   Word 1-4 enabled  */
#define STR71X_BSPICSR2_TFE15       0x0000 /*   Word 1-5 enabled  */
#define STR71X_BSPICSR2_TFE16       0x0000 /*   Word 1-6 enabled  */
#define STR71X_BSPICSR2_TFE17       0x0000 /*   Word 1-7 enabled  */
#define STR71X_BSPICSR2_TFE18       0x0000 /*   Word 1-8 enabled  */
#define STR71X_BSPICSR2_TFE19       0x0000 /*   Word 1-9 enabled  */
#define STR71X_BSPICSR2_TFE110      0x0000 /*   Word 1-10 enabled  */
#define STR71X_BSPICSR2_TIEMASK     0xc000 /* Bit 14-15:  BSPI transmit interrupt enable */
#define STR71X_BSPICSR2_TIEDISABLED 0x0000 /*   Disabled  */
#define STR71X_BSPICSR2_TIETFE      0x4000 /*   Interrupt on transmit FIFO empty  */
#define STR71X_BSPICSR2_TIETUFL     0x8000 /*   Interrupt on transmit underlow */
#define STR71X_BSPICSR2_TIETFF      0xc000 /*   Interrupt on transmit FIFO full  */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_BSPI_H */
