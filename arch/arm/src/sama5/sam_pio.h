/****************************************************************************
 * arch/arm/src/sama5/sam_pio.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_PIO_H
#define __ARCH_ARM_SRC_SAMA5_SAM_PIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/sama5/chip.h>

#include "hardware/sam_memorymap.h"

/* Definitions and types customized for each SAMA5Dx family */

#if defined(ATSAMA5D2)
#  include "sama5d2x_pio.h"
#elif defined(ATSAMA5D3) || defined(ATSAMA5D4)
#  include "sama5d3x4x_pio.h"
#else
#  error Unrecognized SAMA5 architecture
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Lookup for non-secure PIOs */

extern const uintptr_t g_piobase[SAM_NPIO];
#define sam_pion_vbase(n) (g_piobase[(n)])

#ifdef ATSAMA5D2
/* Lookup for secrure PIOs */

extern const uintptr_t g_spiobase[SAM_NPIO];
#  define sam_spion_vbase(n) (g_spiobase[(n)])
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_pioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for PIO
 *   pins.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_PIO_IRQ
void sam_pioirqinitialize(void);
#else
#  define sam_pioirqinitialize()
#endif

/****************************************************************************
 * Name: sam_configpio
 *
 * Description:
 *   Configure a PIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int sam_configpio(pio_pinset_t cfgset);

/****************************************************************************
 * Name: sam_piowrite
 *
 * Description:
 *   Write one or zero to the selected PIO pin
 *
 ****************************************************************************/

void sam_piowrite(pio_pinset_t pinset, bool value);

/****************************************************************************
 * Name: sam_pioread
 *
 * Description:
 *   Read one or zero from the selected PIO pin
 *
 ****************************************************************************/

bool sam_pioread(pio_pinset_t pinset);

/****************************************************************************
 * Name: sam_pioirq
 *
 * Description:
 *   Configure an interrupt for the specified PIO pin.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_PIO_IRQ
void sam_pioirq(pio_pinset_t pinset);
#else
#  define sam_pioirq(pinset)
#endif

/****************************************************************************
 * Name: sam_pioirqenable
 *
 * Description:
 *   Enable the interrupt for specified PIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_PIO_IRQ
void sam_pioirqenable(int irq);
#else
#  define sam_pioirqenable(irq)
#endif

/****************************************************************************
 * Name: sam_pioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified PIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_PIO_IRQ
void sam_pioirqdisable(int irq);
#else
#  define sam_pioirqdisable(irq)
#endif

/****************************************************************************
 * Name: sam_pio_forceclk
 *
 * Description:
 *  Enable PIO clocking.
 *  This logic is overly conservative and does not enable PIO clocking unless
 *  necessary (PIO input selected, glitch/filtering enable, or PIO interrupts
 *  enabled).  There are, however, certain conditions were we may want for
 *  force the PIO clock to be enabled.  An example is reading the input value
 *  from an open drain output.
 *
 *  The PIO automatic enable/disable logic is not smart enough enough to know
 *  about these cases.
 *  For those cases, sam_pio_forceclk() is provided.
 *
 ****************************************************************************/

void sam_pio_forceclk(pio_pinset_t pinset, bool enable);

/****************************************************************************
 * Function:  sam_dumppio
 *
 * Description:
 *   Dump all PIO registers associated with the base address of the provided
 *   pinset.
 *
 ****************************************************************************/

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
