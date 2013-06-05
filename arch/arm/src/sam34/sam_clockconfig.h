/************************************************************************************
 * arch/arm/src/sam34/sam_clockconfig.h
 *
 *   Copyright (C) 2009-2011, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAM34_SAM_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_SAM34_SAM_CLOCKCONFIG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* SAM4L helper functions */

#ifdef CONFIG_ARCH_CHIP_SAM4L
#  define sam_enableperipheral(a,s)    sam_modifyperipheral(a,0,s)
#  define sam_disableperipheral(a,s)   sam_modifyperipheral(a,s,0)

#  define sam_cpu_enableperipheral(s)  sam_enableperipheral(SAM_PM_CPUMASK,s)
#  define sam_hsb_enableperipheral(s)  sam_enableperipheral(SAM_PM_HSBMASK,s)
#  define sam_pbc_enableperipheral(s)  sam_enableperipheral(SAM_PM_PBCMASK,s)
#  define sam_pbd_enableperipheral(s)  sam_enableperipheral(SAM_PM_PBDMASK,s)

#  define sam_cpu_disableperipheral(s) sam_disableperipheral(SAM_PM_CPUMASK,s)
#  define sam_hsb_disableperipheral(s) sam_disableperipheral(SAM_PM_HSBMASK,s)
#  define sam_pbc_enableperipheral(s)  sam_enableperipheral(SAM_PM_PBCMASK,s)
#  define sam_pbd_enableperipheral(s)  sam_enableperipheral(SAM_PM_PBDMASK,s)
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: sam_clockconfig
 *
 * Description:
 *   Called to initialize the SAM3/4.  This does whatever setup is needed to put the
 *   SoC in a usable state.  This includes the initialization of clocking using the
 *   settings in board.h.
 *
 ************************************************************************************/

void sam_clockconfig(void);

/****************************************************************************
 * Name: sam_modifyperipheral
 *
 * Description:
 *   This is a convenience function that is intended to be used to enable
 *   or disable module clocking.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_SAM4L
void sam_modifyperipheral(uintptr_t regaddr, uint32_t clrbits, uint32_t setbits);
#endif

/****************************************************************************
 * Name: sam_pba_enableperipheral
 *
 * Description:
 *   This is a convenience function to enable a peripheral on the APBA
 *   bridge.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_SAM4L
void sam_pba_enableperipheral(uint32_t bitset);
#endif

/****************************************************************************
 * Name: sam_pba_disableperipheral
 *
 * Description:
 *   This is a convenience function to disable a peripheral on the APBA
 *   bridge.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_SAM4L
void sam_pba_disableperipheral(uint32_t bitset);
#endif

/****************************************************************************
 * Name: sam_pbb_enableperipheral
 *
 * Description:
 *   This is a convenience function to enable a peripheral on the APBB
 *   bridge.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_SAM4L
void sam_pbb_enableperipheral(uint32_t bitset);
#endif

/****************************************************************************
 * Name: sam_pbb_disableperipheral
 *
 * Description:
 *   This is a convenience function to disable a peripheral on the APBA
 *   bridge.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_SAM4L
void sam_pbb_disableperipheral(uint32_t bitset);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAM34_SAM_CLOCKCONFIG_H */
