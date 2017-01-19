/****************************************************************************
 * arm/arm/src/armv7-m/up_ramvec_initialize.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "nvic.h"
#include "ram_vectors.h"

#include "chip.h"             /* May redefine VECTAB fields */
#include "up_arch.h"
#include "up_internal.h"

#ifdef CONFIG_ARCH_RAMVECTORS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Alignment ****************************************************************/
/* Per the ARMv7M Architecture reference manual, the NVIC vector table
 * requires 7-bit address alignment (i.e, bits 0-6 of the address of the
 * vector table must be zero).  In this case alignment to a 128 byte address
 * boundary is sufficient.
 *
 * Some parts, such as the LPC17xx family, require alignment to a 256 byte
 * address boundary.  Any other unusual alignment requirements for the vector
 * can be specified for a given architecture be redefining
 * NVIC_VECTAB_TBLOFF_MASK in the chip-specific chip.h header file for the
 * appropriate mask.
 */

#define RAMVEC_ALIGN ((~NVIC_VECTAB_TBLOFF_MASK & 0xffff) + 1)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* If CONFIG_ARCH_RAMVECTORS is defined, then the ARM logic must provide
 * ARM-specific implementations of up_ramvec_initialize(), irq_attach(), and
 * irq_dispatch.  In this case, it is also assumed that the ARM vector
 * table resides in RAM, has the name up_ram_vectors, and has been
 * properly positioned and aligned in memory by the linker script.
 *
 * REVISIT: Can this alignment requirement vary from core-to-core?  Yes, it
 * depends on the number of vectors supported by the MCU. The safest thing
 * to do is to put the vector table at the beginning of RAM in order toforce
 * the highest alignment possible.
 */

up_vector_t g_ram_vectors[ARMV7M_VECTAB_SIZE]
  __attribute__ ((section (".ram_vectors"), aligned (RAMVEC_ALIGN)));

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_ramvec_initialize
 *
 * Description:
 *   Copy vectors to RAM an configure the NVIC to use the RAM vectors.
 *
 ****************************************************************************/

void up_ramvec_initialize(void)
{
  const up_vector_t *src;
  up_vector_t *dest;
  int i;

  /* The vector table must be aligned */

  DEBUGASSERT(((uint32_t)g_ram_vectors & ~NVIC_VECTAB_TBLOFF_MASK) == 0);

  /* Copy the ROM vector table at address zero to RAM vector table.
   *
   * This must be done BEFORE the MPU is enable if the MPU is being used to
   * protect against NULL pointer references.
   */

  src  = (const CODE up_vector_t *)getreg32(NVIC_VECTAB);
  dest = g_ram_vectors;

  irqinfo("src=%p dest=%p\n", src, dest);

  for (i = 0; i < ARMV7M_VECTAB_SIZE; i++)
    {
      *dest++ = *src++;
    }

  /* Now configure the NVIC to use the new vector table. */

  putreg32((uint32_t)g_ram_vectors, NVIC_VECTAB);

  /* The number bits required to align the RAM vector table seem to vary
   * from part-to-part.  The following assertion will catch the case where
   * the table alignment is insufficient.
   */

  irqinfo("NVIC_VECTAB=%08x\n", getreg32(NVIC_VECTAB));
  DEBUGASSERT(getreg32(NVIC_VECTAB) == (uint32_t)g_ram_vectors);
}

#endif /* !CONFIG_ARCH_RAMVECTORS */
