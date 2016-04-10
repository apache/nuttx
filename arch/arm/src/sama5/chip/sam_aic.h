/************************************************************************************
 * arch/arm/src/sama5/chip/sam_aic.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMA5_CHIP_SAM_AIC_H
#define __ARCH_ARM_SRC_SAMA5_CHIP_SAM_AIC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* AIC Register Offsets *************************************************************/

#define SAM_AIC_SSR_OFFSET     0x0000 /* Source Select Register */
#define SAM_AIC_SMR_OFFSET     0x0004 /* Source Mode Register */
#define SAM_AIC_SVR_OFFSET     0x0008 /* Source Vector Register */
#define SAM_AIC_IVR_OFFSET     0x0010 /* Interrupt Vector Register */
#define SAM_AIC_FVR_OFFSET     0x0014 /* FIQ Interrupt Vector Register */
#define SAM_AIC_ISR_OFFSET     0x0018 /* Interrupt Status Register */
#define SAM_AIC_IPR0_OFFSET    0x0020 /* Interrupt Pending Register 0 */
#define SAM_AIC_IPR1_OFFSET    0x0024 /* Interrupt Pending Register 1 */
#define SAM_AIC_IPR2_OFFSET    0x0028 /* Interrupt Pending Register 2 */
#define SAM_AIC_IPR3_OFFSET    0x002c /* Interrupt Pending Register 3 */
#define SAM_AIC_IMR_OFFSET     0x0030 /* Interrupt Mask Register */
#define SAM_AIC_CISR_OFFSET    0x0034 /* Core Interrupt Status Register */
#define SAM_AIC_EOICR_OFFSET   0x0038 /* End of Interrupt Command Register */
#define SAM_AIC_SPU_OFFSET     0x003c /* Spurious Interrupt Vector Register */
#define SAM_AIC_IECR_OFFSET    0x0040 /* Interrupt Enable Command Register */
#define SAM_AIC_IDCR_OFFSET    0x0044 /* Interrupt Disable Command Register */
#define SAM_AIC_ICCR_OFFSET    0x0048 /* Interrupt Clear Command Register */
#define SAM_AIC_ISCR_OFFSET    0x004c /* Interrupt Set Command Register */

#ifdef ATSAMA5D3
#  define SAM_AIC_FFER_OFFSET  0x0050 /* Fast Forcing Enable Register */
#  define SAM_AIC_FFDR_OFFSET  0x0054 /* Fast Forcing Disable Register */
#  define SAM_AIC_FFSR_OFFSET  0x0058 /* Fast Forcing Status Register */
#endif

#define SAM_AIC_DCR_OFFSET     0x006c /* Debug Control Register */
#define SAM_AIC_WPMR_OFFSET    0x00e4 /* Write Protect Mode Register */
#define SAM_AIC_WPSR_OFFSET    0x00e8 /* Write Protect Status Register */

/* AIC Register Addresses ***********************************************************/

#define SAM_AIC_SSR            (SAM_AIC_VBASE+SAM_AIC_SSR_OFFSET)
#define SAM_AIC_SMR            (SAM_AIC_VBASE+SAM_AIC_SMR_OFFSET)
#define SAM_AIC_SVR            (SAM_AIC_VBASE+SAM_AIC_SVR_OFFSET)
#define SAM_AIC_IVR            (SAM_AIC_VBASE+SAM_AIC_IVR_OFFSET)
#define SAM_AIC_FVR            (SAM_AIC_VBASE+SAM_AIC_FVR_OFFSET)
#define SAM_AIC_ISR            (SAM_AIC_VBASE+SAM_AIC_ISR_OFFSET)
#define SAM_AIC_IPR0           (SAM_AIC_VBASE+SAM_AIC_IPR0_OFFSET)
#define SAM_AIC_IPR1           (SAM_AIC_VBASE+SAM_AIC_IPR1_OFFSET)
#define SAM_AIC_IPR2           (SAM_AIC_VBASE+SAM_AIC_IPR2_OFFSET)
#define SAM_AIC_IPR3           (SAM_AIC_VBASE+SAM_AIC_IPR3_OFFSET)
#define SAM_AIC_IMR            (SAM_AIC_VBASE+SAM_AIC_IMR_OFFSET)
#define SAM_AIC_CISR           (SAM_AIC_VBASE+SAM_AIC_CISR_OFFSET)
#define SAM_AIC_EOICR          (SAM_AIC_VBASE+SAM_AIC_EOICR_OFFSET)
#define SAM_AIC_SPU            (SAM_AIC_VBASE+SAM_AIC_SPU_OFFSET)
#define SAM_AIC_IECR           (SAM_AIC_VBASE+SAM_AIC_IECR_OFFSET)
#define SAM_AIC_IDCR           (SAM_AIC_VBASE+SAM_AIC_IDCR_OFFSET)
#define SAM_AIC_ICCR           (SAM_AIC_VBASE+SAM_AIC_ICCR_OFFSET)
#define SAM_AIC_ISCR           (SAM_AIC_VBASE+SAM_AIC_ISCR_OFFSET)

#ifdef ATSAMA5D3
#  define SAM_AIC_FFER         (SAM_AIC_VBASE+SAM_AIC_FFER_OFFSET)
#  define SAM_AIC_FFDR         (SAM_AIC_VBASE+SAM_AIC_FFDR_OFFSET)
#  define SAM_AIC_FFSR         (SAM_AIC_VBASE+SAM_AIC_FFSR_OFFSET)
#endif

#define SAM_AIC_DCR            (SAM_AIC_VBASE+SAM_AIC_DCR_OFFSET)
#define SAM_AIC_WPMR           (SAM_AIC_VBASE+SAM_AIC_WPMR_OFFSET)
#define SAM_AIC_WPSR           (SAM_AIC_VBASE+SAM_AIC_WPSR_OFFSET)

#ifdef CONFIG_SAMA5_HAVE_SAIC
#  define SAM_SAIC_SSR         (SAM_SAIC_VBASE+SAM_AIC_SSR_OFFSET)
#  define SAM_SAIC_SMR         (SAM_SAIC_VBASE+SAM_AIC_SMR_OFFSET)
#  define SAM_SAIC_SVR         (SAM_SAIC_VBASE+SAM_AIC_SVR_OFFSET)
#  define SAM_SAIC_IVR         (SAM_SAIC_VBASE+SAM_AIC_IVR_OFFSET)
#  define SAM_SAIC_FVR         (SAM_SAIC_VBASE+SAM_AIC_FVR_OFFSET)
#  define SAM_SAIC_ISR         (SAM_SAIC_VBASE+SAM_AIC_ISR_OFFSET)
#  define SAM_SAIC_IPR0        (SAM_SAIC_VBASE+SAM_AIC_IPR0_OFFSET)
#  define SAM_SAIC_IPR1        (SAM_SAIC_VBASE+SAM_AIC_IPR1_OFFSET)
#  define SAM_SAIC_IPR2        (SAM_SAIC_VBASE+SAM_AIC_IPR2_OFFSET)
#  define SAM_SAIC_IPR3        (SAM_SAIC_VBASE+SAM_AIC_IPR3_OFFSET)
#  define SAM_SAIC_IMR         (SAM_SAIC_VBASE+SAM_AIC_IMR_OFFSET)
#  define SAM_SAIC_CISR        (SAM_SAIC_VBASE+SAM_AIC_CISR_OFFSET)
#  define SAM_SAIC_EOICR       (SAM_SAIC_VBASE+SAM_AIC_EOICR_OFFSET)
#  define SAM_SAIC_SPU         (SAM_SAIC_VBASE+SAM_AIC_SPU_OFFSET)
#  define SAM_SAIC_IECR        (SAM_SAIC_VBASE+SAM_AIC_IECR_OFFSET)
#  define SAM_SAIC_IDCR        (SAM_SAIC_VBASE+SAM_AIC_IDCR_OFFSET)
#  define SAM_SAIC_ICCR        (SAM_SAIC_VBASE+SAM_AIC_ICCR_OFFSET)
#  define SAM_SAIC_ISCR        (SAM_SAIC_VBASE+SAM_AIC_ISCR_OFFSET)
#  define SAM_SAIC_DCR         (SAM_SAIC_VBASE+SAM_AIC_DCR_OFFSET)
#  define SAM_SAIC_WPMR        (SAM_SAIC_VBASE+SAM_AIC_WPMR_OFFSET)
#  define SAM_SAIC_WPSR        (SAM_SAIC_VBASE+SAM_AIC_WPSR_OFFSET)
#endif

/* AIC Register Bit Definitions *****************************************************/

/* Source Select Register */

#define AIC_SSR_MASK           (0x7f)    /* Bits 0-6: Interrupt line Selection */

/* Source Mode Register */

#define AIC_SMR_PRIOR_SHIFT    (0)       /* Bits 0-2: Priority level */
#define AIC_SMR_PRIOR_MASK     (7 << AIC_SMR_PRIOR_SHIFT)
#  define AIC_SMR_PRIOR_MIN    (0)
#  define AIC_SMR_PRIOR_MAX    (7)
#define AIC_SMR_SRCTYPE_SHIFT  (5)       /* Bits 5-6: Interrupt source type */
#define AIC_SMR_SRCTYPE_MASK   (3 << AIC_SMR_SRCTYPE_SHIFT)
#  define AIC_SMR_SRCTYPE_IHIGH    (0 << AIC_SMR_SRCTYPE_SHIFT) /* Internal high level */
#  define AIC_SMR_SRCTYPE_XLOW     (0 << AIC_SMR_SRCTYPE_SHIFT) /* External low level */
#  define AIC_SMR_SRCTYPE_IRISING  (1 << AIC_SMR_SRCTYPE_SHIFT) /* Internal positive edge */
#  define AIC_SMR_SRCTYPE_XFALLING (1 << AIC_SMR_SRCTYPE_SHIFT) /* External negative edge */
#  define AIC_SMR_SRCTYPE_XHIGH    (2 << AIC_SMR_SRCTYPE_SHIFT) /* External high level */
#  define AIC_SMR_SRCTYPE_XRISING  (3 << AIC_SMR_SRCTYPE_SHIFT) /* External rising edge */

/* Source Vector Register (32-bit address) */
/* Interrupt Vector Register (32-bit address) */
/* FIQ Interrupt Vector Register (32-bit address) */

/* Interrupt Status Register */

#define AIC_ISR_MASK           (0x7f)    /* Bits 0-6: Current Interrupt Identifier */

/* Interrupt Pending Register 0-3 */

#define AIC_IPR0(pid)          (1 << (pid))
#define AIC_IPR1(pid)          (1 << ((pid) - 32)
#define AIC_IPR2(pid)          (1 << ((pid) - 64)
#define AIC_IPR3(pid)          (1 << ((pid) - 96)

/* Interrupt Mask Register */

#define AIC_IMR_INTM           (1 << 0)  /* Bit 0:  Interrupt Mask */

/* Core Interrupt Status Register */

#define AIC_CISR_NFIQ          (1 << 0)  /* Bit 0:  NFIQ Status */
#define AIC_CISR_NIRQ          (1 << 1)  /* Bit 1:  NIRQ Status */

/* End of Interrupt Command Register */

#define AIC_EOICR_ENDIT        (1 << 0)  /* Bit 0:  Interrupt Processing Complete Command */

/* Spurious Interrupt Vector Register (32-bit address) */

/* Interrupt Enable Command Register */

#define AIC_IECR_INTEN         (1 << 0)  /* Bit 0:  Interrupt Enable */

/* Interrupt Disable Command Register */

#define AIC_IDCR_INTD          (1 << 0)  /* Bit 0:  Interrupt Disable */

/* Interrupt Clear Command Register */

#define AIC_ICCR_INTCLR        (1 << 0)  /* Bit 0:  Interrupt Clear */

/* Interrupt Set Command Register */

#define AIC_ISCR_INTSET        (1 << 0)  /* Bit 0:  Interrupt Set */

#ifdef ATSAMA5D3
/* Fast Forcing Enable Register */

#  define AIC_FFER_FFEN        (1 << 0)  /* Bit 0:  Fast Forcing Enable */

/* Fast Forcing Disable Register */

#  define AIC_FFDR_FFDIS       (1 << 0)  /* Bit 0:  Fast Forcing Disable */

/* Fast Forcing Status Register */

#  define AIC_FFSR_FFS         (1 << 0)  /* Bit 0:  Fast Forcing Status */
#endif

/* Debug Control Register */

#define AIC_DCR_PROT           (1 << 0)  /* Bit 0:  Protection Mode */
#define AIC_DCR_GMSK           (1 << 1)  /* Bit 1:  General Mask */

/* Write Protect Mode Register */

#define AIC_WPMR_WPEN          (1 << 0)  /* Bit 0:  Write Protect Enable */
#define AIC_WPMR_WPKEY_SHIFT   (8)       /* Bits 8-31: Write Protect KEY */
#define AIC_WPMR_WPKEY_MASK    (0x00ffffff << AIC_WPMR_WPKEY_SHIFT)
#  define AIC_WPMR_WPKEY       (0x00414943 << AIC_WPMR_WPKEY_SHIFT)

/* Write Protect Status Register */

#define AIC_WPSR_WPVS          (1 << 0)  /* Bit 0:  Write Protect Violation Status */
#define AIC_WPSR_WPVSRC_SHIFT  (8)       /* Bits 8-23: Write Protect Violation Source */
#define AIC_WPSR_WPVSRC_MASK   (0x0000ffff << AIC_WPSR_WPVSRC_SHIFT)

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_AIC_H */
