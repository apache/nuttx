/********************************************************************************************
 * arch/arm/src/samd5e5/hardware/sam_rstc.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_RSTC_H
#define __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_RSTC_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "hardware/sam_memorymap.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* RSTC register offsets ********************************************************************/

#define SAM_RSTC_RCAUSE_OFFSET         0x0000  /* Reset cause */
#define SAM_RSTC_BKUPEXIT_OFFSET       0x0002  /* Backup exit source */

/* RSTC register addresses ******************************************************************/

#define SAM_RSTC_RCAUSE                (SAM_RSTC_BASE + SAM_RSTC_RCAUSE_OFFSET)
#define SAM_RSTC_BKUPEXIT              (SAM_RSTC_BASE + SAM_RSTC_BKUPEXIT_OFFSET)

/* RSTC register bit definitions ************************************************************/

/* Reset cause */

#define RSTC_RCAUSE_POR                (1 << 0)  /* Bit 0: Power on reset */
#define RSTC_RCAUSE_BOD12              (1 << 1)  /* Bit 1: Brown out 12 detector reset */
#define RSTC_RCAUSE_BOD33              (1 << 2)  /* Bit 2: Brown out 33 detector reset */
#define RSTC_RCAUSE_NVM                (1 << 3)  /* Bit 3: External reset */
#define RSTC_RCAUSE_EXT                (1 << 4)  /* Bit 4: External reset */
#define RSTC_RCAUSE_WDT                (1 << 5)  /* Bit 5: Watchdog reset */
#define RSTC_RCAUSE_SYST               (1 << 6)  /* Bit 6: System reset request */
#define RSTC_RCAUSE_BACKUP             (1 << 7)  /* Bit 7: Backup reset*/

/* Backup exit source */

#define RSTC_BKUPEXIT_RTC              (1 << 1)  /* Bit 1: Real time counter interrupt */
#define RSTC_BKUPEXIT_BBPS             (1 << 2)  /* Bit 2: Battery backup power switch */
#define RSTC_BKUPEXIT_HIB              (1 << 7)  /* Bit 7: Hibernate */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMD5E5_HARDWARE_SAM_RSTC_H */
