/************************************************************************************************
 * arch/arm/src/lpc313x/lpc313x_mci.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC313X_MCI_H
#define __ARCH_ARM_SRC_LPC313X_MCI_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "lpc313x_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* MCI register base address offset into the MCI domain *****************************************/

#define LPC313X_MCI_VBASE                (LPC313X_MCI_VSECTION)
#define LPC313X_MCI_PBASE                (LPC313X_MCI_PSECTION)

/* MCI register offsets (with respect to the MCI base) ******************************************/

#define LPC313X_MCI_CTRL_OFFSET          0x000 /* Control register */
#define LPC313X_MCI_PWREN_OFFSET         0x004 /* Reserved */
#define LPC313X_MCI_CLKDIV_OFFSET        0x008 /* Clock-divider register */
#define LPC313X_MCI_CLKSRC_OFFSET        0x00c /* Clock-source register */
#define LPC313X_MCI_CLKENA_OFFSET        0x010 /* Clock-enable register */
#define LPC313X_MCI_TMOUT_OFFSET         0x014 /* Time-out register */
#define LPC313X_MCI_CTYPE_OFFSET         0x018 /* Card-type register */
#define LPC313X_MCI_BLKSIZ_OFFSET        0x01c /* Block-size register */
#define LPC313X_MCI_BYTCNT_OFFSET        0x020 /* Byte-count register */
#define LPC313X_MCI_INTMASK_OFFSET       0x024 /* Interrupt-mask register */
#define LPC313X_MCI_CMDARG_OFFSET        0x028 /* Command-argument register */
#define LPC313X_MCI_CMD_OFFSET           0x02c /* Command register */
#define LPC313X_MCI_RESP0_OFFSET         0x030 /* Response-0 register */
#define LPC313X_MCI_RESP1_OFFSET         0x034 /* Response-1register */
#define LPC313X_MCI_RESP2_OFFSET         0x038 /* Response-2 register */
#define LPC313X_MCI_RESP3_OFFSET         0x03c /* Response-3 register */
#define LPC313X_MCI_MINTSTS_OFFSET       0x040 /* Masked interrupt-status register */
#define LPC313X_MCI_RINTSTS_OFFSET       0x044 /* Raw interrupt-status register */
#define LPC313X_MCI_STATUS_OFFSET        0x048 /* Status register */
#define LPC313X_MCI_FIFOTH_OFFSET        0x04c /* FIFO threshold register */
#define LPC313X_MCI_CDETECT_OFFSET       0x050 /* Card-detect register value */
#define LPC313X_MCI_WRTPRT_OFFSET        0x054 /* Write-protect register */
                                               /* 0x58: Reserved */
#define LPC313X_MCI_TCBCNT_OFFSET        0x05c /* Transferred CIU card byte count */
#define LPC313X_MCI_TBBCNT_OFFSET        0x060 /* Transferred cpu/DMA to/from BIU-FIFO byte count */
                                               /* 0x064-0x0ff: Reserved */
#define LPC313X_MCI_DATA_OFFSET          0x100 /* Data FIFO read/write (>=) */

/* MCI register (virtual) addresses *************************************************************/

#define LPC313X_MCI_CTRL                 (LPC313X_MCI_VBASE+LPC313X_MCI_CTRL_OFFSET)
#define LPC313X_MCI_PWREN                (LPC313X_MCI_VBASE+LPC313X_MCI_PWREN_OFFSET)
#define LPC313X_MCI_CLKDIV               (LPC313X_MCI_VBASE+LPC313X_MCI_CLKDIV_OFFSET)
#define LPC313X_MCI_CLKSRC               (LPC313X_MCI_VBASE+LPC313X_MCI_CLKSRC_OFFSET)
#define LPC313X_MCI_CLKENA               (LPC313X_MCI_VBASE+LPC313X_MCI_CLKENA_OFFSET)
#define LPC313X_MCI_TMOUT                (LPC313X_MCI_VBASE+LPC313X_MCI_TMOUT_OFFSET)
#define LPC313X_MCI_CTYPE                (LPC313X_MCI_VBASE+LPC313X_MCI_CTYPE_OFFSET)
#define LPC313X_MCI_BLKSIZ               (LPC313X_MCI_VBASE+LPC313X_MCI_BLKSIZ_OFFSET)
#define LPC313X_MCI_BYTCNT               (LPC313X_MCI_VBASE+LPC313X_MCI_BYTCNT_OFFSET)
#define LPC313X_MCI_INTMASK              (LPC313X_MCI_VBASE+LPC313X_MCI_INTMASK_OFFSET)
#define LPC313X_MCI_CMDARG               (LPC313X_MCI_VBASE+LPC313X_MCI_CMDARG_OFFSET)
#define LPC313X_MCI_CMD                  (LPC313X_MCI_VBASE+LPC313X_MCI_CMD_OFFSET)
#define LPC313X_MCI_RESP0                (LPC313X_MCI_VBASE+LPC313X_MCI_RESP0_OFFSET)
#define LPC313X_MCI_RESP1                (LPC313X_MCI_VBASE+LPC313X_MCI_RESP1_OFFSET)
#define LPC313X_MCI_RESP2                (LPC313X_MCI_VBASE+LPC313X_MCI_RESP2_OFFSET)
#define LPC313X_MCI_RESP3                (LPC313X_MCI_VBASE+LPC313X_MCI_RESP3_OFFSET)
#define LPC313X_MCI_MINTSTS              (LPC313X_MCI_VBASE+LPC313X_MCI_MINTSTS_OFFSET)
#define LPC313X_MCI_RINTSTS              (LPC313X_MCI_VBASE+LPC313X_MCI_RINTSTS_OFFSET)
#define LPC313X_MCI_STATUS               (LPC313X_MCI_VBASE+LPC313X_MCI_STATUS_OFFSET)
#define LPC313X_MCI_FIFOTH               (LPC313X_MCI_VBASE+LPC313X_MCI_FIFOTH_OFFSET)
#define LPC313X_MCI_CDETECT              (LPC313X_MCI_VBASE+LPC313X_MCI_CDETECT_OFFSET)
#define LPC313X_MCI_WRTPRT               (LPC313X_MCI_VBASE+LPC313X_MCI_WRTPRT_OFFSET)
#define LPC313X_MCI_TCBCNT               (LPC313X_MCI_VBASE+LPC313X_MCI_TCBCNT_OFFSET)
#define LPC313X_MCI_TBBCNT               (LPC313X_MCI_VBASE+LPC313X_MCI_TBBCNT_OFFSET)
#define LPC313X_MCI_DATA                 (LPC313X_MCI_VBASE+LPC313X_MCI_DATA_OFFSET)

/* MCI register bit definitions *****************************************************************/

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC313X_MCI_H */
