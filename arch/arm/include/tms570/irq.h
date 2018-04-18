/****************************************************************************************
 * arch/arm/include/tms570/irq.h
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
 ****************************************************************************************/

/* This file should never be included directed but, rather, only indirectly through
 * nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_TMS570_IRQ_H
#define __ARCH_ARM_INCLUDE_TMS570_IRQ_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>
#include <arch/tms570/chip.h>

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/
/* The interrupt vector table only has 96 entries, one phantom vector and 95 interrupt
 * channels. Channel 95 does not have a dedicated vector and shall not be used.
 */

#define TMS570_VECT_PHANTOM    0  /* The first is the "phantom" interrupt */

/* Default channel assignments are MCU-dependent */

#if defined(CONFIG_ARCH_CHIP_TMS570LS0232PZ)
#  error No IRQ definitions for the TMS570LS0232PZ
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0332PZ)
#  include <arch/tms570/tms570ls04x03x_irq.h>
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0432PZ)
#  include <arch/tms570/tms570ls04x03x_irq.h>
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0714PZ)
#  error No IRQ definitions for the TMS570LS0714PZ
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0714PGE)
#  error No IRQ definitions for the TMS570LS0714PGE
#elif defined(CONFIG_ARCH_CHIP_TMS570LS0714ZWT)
#  error No IRQ definitions for the TMS570LS0714ZWT
#elif defined(CONFIG_ARCH_CHIP_TMS570LS1227ZWT)
#  error No IRQ definitions for the TMS570LS1227ZWT
#elif defined(CONFIG_ARCH_CHIP_TMS570LS3137ZWT)
#  include <arch/tms570/tms570ls31xx_irq.h>
#else
#  error "Unrecognized Hercules chip"
#endif

/* Total number of IRQ numbers. Includes all channels plus GIO second-level interrupts
 * (if enabled).  Excluds the phantom vector.  Zero corresponds to channel 0, vector 1.
 */

#define NR_IRQS  (TMS570_IRQ_NCHANNELS + TMS570_NGIO_IRQS)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************************
 * Public Function Prototypes
 ****************************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_TMS570_IRQ_H */
