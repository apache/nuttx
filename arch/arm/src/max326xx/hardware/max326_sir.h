/************************************************************************************
 * arch/arm/src/max326xx/hardware/max326_sir.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_SIR_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_SIR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/max326_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define MAX326_SIR_STAT_OFFSET        0x0000 /* System Initialization Status Register */
#define MAX326_SIR_ADDRER_OFFSET      0x0004 /* System Initialization Address Error Register */

/* Register Addresses ***************************************************************/

#define MAX326_SIR_STAT               (MAX326_SIR_BASE + MAX326_SIR_STAT_OFFSET)
#define MAX326_SIR_ADDRER             (MAX326_SIR_BASE + MAX326_SIR_ADDRER_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* System Initialization Status Register */

#define SIR_STAT_CFGVALID             (1 << 0)  /* Bit 0:  Configuration Valid Flag */
#define SIR_STAT_CFGERR               (1 << 1)  /* Bit 1:  Configuration Error Flag */

/* System Initialization Address Error Register (32-bit Configuration Error Address) */

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_SIR_H */
