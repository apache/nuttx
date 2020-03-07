/************************************************************************************
 * arch/arm/src/sama5/sam_pio.h
 * Parallel Input/Output (PIO) definitions for the SAMA5
 *
 *   Copyright (C) 2013, 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_PIO_H
#define __ARCH_ARM_SRC_SAMA5_SAM_PIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/sama5/chip.h>

#include "hardware/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Definitions and types customized for each SAMA5Dx family */

#if defined(ATSAMA5D2)
#  include "sama5d2x_pio.h"
#elif defined(ATSAMA5D3) || defined(ATSAMA5D4)
#  include "sama5d3x4x_pio.h"
#else
#  error Unrecognized SAMA5 architecture
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* Lookup for non-secure PIOs */

extern const uintptr_t g_piobase[SAM_NPIO];
#define sam_pion_vbase(n) (g_piobase[(n)])

#ifdef ATSAMA5D2
/* Lookup for secrure PIOs */

extern const uintptr_t g_spiobase[SAM_NPIO];
#  define sam_spion_vbase(n) (g_spiobase[(n)])
#endif

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
 * Name: sam_pioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for PIO pins.
 *
 ************************************************************************************/

#ifdef CONFIG_SAMA5_PIO_IRQ
void sam_pioirqinitialize(void);
#else
#  define sam_pioirqinitialize()
#endif

/************************************************************************************
 * Name: sam_configpio
 *
 * Description:
 *   Configure a PIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

int sam_configpio(pio_pinset_t cfgset);

/************************************************************************************
 * Name: sam_piowrite
 *
 * Description:
 *   Write one or zero to the selected PIO pin
 *
 ************************************************************************************/

void sam_piowrite(pio_pinset_t pinset, bool value);

/************************************************************************************
 * Name: sam_pioread
 *
 * Description:
 *   Read one or zero from the selected PIO pin
 *
 ************************************************************************************/

bool sam_pioread(pio_pinset_t pinset);

/************************************************************************************
 * Name: sam_pioirq
 *
 * Description:
 *   Configure an interrupt for the specified PIO pin.
 *
 ************************************************************************************/

#ifdef CONFIG_SAMA5_PIO_IRQ
void sam_pioirq(pio_pinset_t pinset);
#else
#  define sam_pioirq(pinset)
#endif

/************************************************************************************
 * Name: sam_pioirqenable
 *
 * Description:
 *   Enable the interrupt for specified PIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_SAMA5_PIO_IRQ
void sam_pioirqenable(int irq);
#else
#  define sam_pioirqenable(irq)
#endif

/************************************************************************************
 * Name: sam_pioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified PIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_SAMA5_PIO_IRQ
void sam_pioirqdisable(int irq);
#else
#  define sam_pioirqdisable(irq)
#endif

/************************************************************************************
 * Name: sam_pio_forceclk
 *
 * Description:
 *   Enable PIO clocking.  This logic is overly conservative and does not enable PIO
 *   clocking unless necessary (PIO input selected, glitch/filtering enable, or PIO
 *   interrupts enabled).  There are, however, certain conditions were we may want
 *   for force the PIO clock to be enabled.  An example is reading the input value
 *   from an open drain output.
 *
 *   The PIO automatic enable/disable logic is not smart enough enough to know about
 *   these cases.  For those cases, sam_pio_forceclk() is provided.
 *
 ************************************************************************************/

void sam_pio_forceclk(pio_pinset_t pinset, bool enable);

/************************************************************************************
 * Function:  sam_dumppio
 *
 * Description:
 *   Dump all PIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int sam_dumppio(uint32_t pinset, const char *msg);
#else
#  define sam_dumppio(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_PIO_H */
