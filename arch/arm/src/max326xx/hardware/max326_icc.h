/************************************************************************************
 * arch/arm/src/max326xx/hardware/max326_icc.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_ICC_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_ICC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/max326_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define MAX326_ICC_ID_OFFSET          0x0000 /* Cache ID Register */
#define MAX326_ICC_MEMCFG_OFFSET      0x0004 /* Memory Configuration Register */
#define MAX326_ICC_CTRLSTAT_OFFSET    0x0100 /* Cache Control and Status Register */
#define MAX326_ICC_INVDTALL_OFFSET    0x0700 /* Cache Invalidate Register */

/* Register Addresses ***************************************************************/

#define MAX326_ICC_ID                 (MAX326_ICC_BASE + MAX326_ICC_ID_OFFSET)
#define MAX326_ICC_MEMCFG             (MAX326_ICC_BASE + MAX326_ICC_MEMCFG_OFFSET)
#define MAX326_ICC_CTRLSTAT           (MAX326_ICC_BASE + MAX326_ICC_CTRLSTAT_OFFSET)
#define MAX326_ICC_INVDTALL           (MAX326_ICC_BASE + MAX326_ICC_INVDTALL_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* Cache ID Register */

#define ICC_ID_RELNUM_SHIFT           (0)       /* Bits 0-5: Cache Release Number */
#define ICC_ID_RELNUM_MASK            (0x3f << ICC_ID_RELNUM_SHIFT)
#define ICC_ID_PARTNUM_SHIFT          (6)       /* Bits 6-9: Cache Part Number */
#define ICC_ID_PARTNUM_MASK           (15 << ICC_ID_PARTNUM_SHIFT)
#define ICC_ID_CCHID_SHIFT            (10)      /* Bits 10-15: Cache ID */
#define ICC_ID_CCHID_MASK             (0x3f << ICC_ID_CCHID_SHIFT)

/* Memory Configuration Register:
 *
 * Cache size is in units 1Kb.
 * Memory size is in units of 32Kb (MAX32620/30) or 128Kb (MAX32660)
 */

#define ICC_MEMCFG_CCHSZ_SHIFT        (0)       /* Bits 0-15: Cache Size */
#define ICC_MEMCFG_CCHSZ_MASK         (0xffff << ICC_MEMCFG_CCHSZ_SHIFT)
#define ICC_MEMCFG_MEMSZ_SHIFT        (16)      /* Bits 16-31: Addressable Memory Size */
#define ICC_MEMCFG_MEMSZ_MASK         (0xffff << ICC_MEMCFG_MEMSZ_SHIFT)

/* Cache Control and Status Register */

#define ICC_CTRLSTAT_ENABLE           (1 << 0)  /* Bit 0:  Cache enable */
#define ICC_CTRLSTAT_READY            (1 << 16) /* Bit 16: Cache is ready */

/* Cache Invalidate Register.  Any write to this register of any value invalidates
 * the cache
 */

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_ICC_H */
