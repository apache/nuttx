/****************************************************************************
 * arch/arm/src/sama5/sam_memories.c
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include "chip.h"
#include "sam_memories.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: peripha_physregaddr
 *
 * Description:
 *   Give the virtual address of a peripheral A register, return the
 *   physical address of the register
 *
 ****************************************************************************/

static inline uintptr_t peripha_physregaddr(uintptr_t vregaddr)
{
#if SAM_PERIPHA_PSECTION != SAM_PERIPHA_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * register
   */

  uintptr_t sectoffset = vregaddr - SAM_PERIPHA_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_PERIPHA_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return vregaddr;

#endif
}

/****************************************************************************
 * Name: periphb_physregaddr
 *
 * Description:
 *   Give the virtual address of a peripheral B register, return the
 *   physical address of the register
 *
 ****************************************************************************/

static inline uintptr_t periphb_physregaddr(uintptr_t vregaddr)
{
#if SAM_PERIPHB_PSECTION != SAM_PERIPHB_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * register
   */

  uintptr_t sectoffset = vregaddr - SAM_PERIPHB_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_PERIPHB_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return vregaddr;

#endif
}

/****************************************************************************
 * Name: sysc_physregaddr
 *
 * Description:
 *   Give the virtual address of a system controller register, return the
 *   physical address of the register
 *
 ****************************************************************************/

static inline uintptr_t sysc_physregaddr(uintptr_t vregaddr)
{
#if SAM_SYSC_PSECTION != SAM_SYSC_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * register
   */

  uintptr_t sectoffset = vregaddr - SAM_SYSC_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_SYSC_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return vregaddr;

#endif
}

/****************************************************************************
 * Name: isram_physramaddr
 *
 * Description:
 *   Give the virtual address of an internal SRAM memory location, return the
 *   physical address of that location
 *
 ****************************************************************************/

static inline uintptr_t isram_physramaddr(uintptr_t vregaddr)
{
#if SAM_ISRAM_PSECTION != SAM_ISRAM_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = vregaddr - SAM_ISRAM_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_ISRAM_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return vregaddr;

#endif
}

/****************************************************************************
 * Name: sdram_physramaddr
 *
 * Description:
 *   Give the virtual address of an external SDRAM memory location, return
 *   the physical address of that location
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_DDRCS
static inline uintptr_t sdram_physramaddr(uintptr_t vregaddr)
{
#if SAM_DDRCS_PSECTION != SAM_DDRCS_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = vregaddr - SAM_DDRCS_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_DDRCS_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return vregaddr;

#endif
}
#endif

/****************************************************************************
 * Name: nfcsram_physramaddr
 *
 * Description:
 *   Give the virtual address of an NFC SRAM memory location, return the
 *   physical address of that location
 *
 ****************************************************************************/

static inline uintptr_t nfcsram_physramaddr(uintptr_t vregaddr)
{
#if SAM_NFCSRAM_PSECTION != SAM_NFCSRAM_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = vregaddr - SAM_NFCSRAM_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_NFCSRAM_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return vregaddr;

#endif
}

/****************************************************************************
 * Name: udphsram_physramaddr
 *
 * Description:
 *   Give the virtual address of an UDPH SRAM memory location, return the
 *   physical address of that location
 *
 ****************************************************************************/

static inline uintptr_t udphsram_physramaddr(uintptr_t vregaddr)
{
#if SAM_UDPHSRAM_PSECTION != SAM_UDPHSRAM_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = vregaddr - SAM_UDPHSRAM_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_UDPHSRAM_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return vregaddr;

#endif
}

/****************************************************************************
 * Name: ebics0_physramaddr
 *
 * Description:
 *   Give the virtual address of an external CS0 SRAM memory location,
 *   return the physical address of that location
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_EBICS0) && (defined(CONFIG_SAMA5_EBICS0_SRAM) || \
    defined(CONFIG_SAMA5_EBICS0_PSRAM))
static inline uintptr_t ebics0_physramaddr(uintptr_t vregaddr)
{
#if SAM_EBICS0_PSECTION != SAM_EBICS0_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = vregaddr - SAM_EBICS0_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_EBICS0_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return vregaddr;

#endif
}
#endif

/****************************************************************************
 * Name: ebics1_physramaddr
 *
 * Description:
 *   Give the virtual address of an external CS1 SRAM memory location,
 *   return the physical address of that location
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_EBICS1) && (defined(CONFIG_SAMA5_EBICS1_SRAM) || \
    defined(CONFIG_SAMA5_EBICS1_PSRAM))
static inline uintptr_t ebics1_physramaddr(uintptr_t vregaddr)
{
#if SAM_EBICS1_PSECTION != SAM_EBICS1_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = vregaddr - SAM_EBICS1_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_EBICS1_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return vregaddr;

#endif
}
#endif

/****************************************************************************
 * Name: ebics2_physramaddr
 *
 * Description:
 *   Give the virtual address of an external CS2 SRAM memory location,
 *   return the physical address of that location
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_EBICS2) && (defined(CONFIG_SAMA5_EBICS2_SRAM) || \
    defined(CONFIG_SAMA5_EBICS2_PSRAM))
static inline uintptr_t ebics2_physramaddr(uintptr_t vregaddr)
{
#if SAM_EBICS2_PSECTION != SAM_EBICS2_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = vregaddr - SAM_EBICS2_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_EBICS2_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return vregaddr;

#endif
}
#endif

/****************************************************************************
 * Name: ebics3_physramaddr
 *
 * Description:
 *   Give the virtual address of an external CS3 SRAM memory location,
 *   return the physical address of that location
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_EBICS3) && (defined(CONFIG_SAMA5_EBICS3_SRAM) || \
    defined(CONFIG_SAMA5_EBICS3_PSRAM))
static inline uintptr_t ebics3_physramaddr(uintptr_t vregaddr)
{
#if SAM_EBICS3_PSECTION != SAM_EBICS3_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = vregaddr - SAM_EBICS3_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_EBICS3_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return vregaddr;

#endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_physregaddr
 *
 * Description:
 *   Give the virtual address of a register, return the physical address of
 *   the register
 *
 ****************************************************************************/

uintptr_t sam_physregaddr(uintptr_t vregaddr)
{
  /* Check for a peripheral A register */

  if (vregaddr >= SAM_PERIPHA_VSECTION &&
      vregaddr < (SAM_PERIPHA_VSECTION + SAM_PERIPHA_SIZE))
    {
      return peripha_physregaddr(vregaddr);
    }

  /* Check for a peripheral A register */

  else if (vregaddr >= SAM_PERIPHB_VSECTION &&
           vregaddr < (SAM_PERIPHB_VSECTION + SAM_PERIPHB_SIZE))
    {
      return periphb_physregaddr(vregaddr);
    }

  /* Check for a system controller register */

  else if (vregaddr >= SAM_SYSC_VSECTION &&
           vregaddr < (SAM_SYSC_VSECTION + SAM_SYSC_SIZE))
    {
      return sysc_physregaddr(vregaddr);
    }

  /* We will not get here unless we are called with an invalid register
   * address
   */

  DEBUGPANIC();
  return vregaddr;
}

/****************************************************************************
 * Name: sam_physramaddr
 *
 * Description:
 *   Give the virtual address of a RAM memory location, return the physical
 *   address of that location.
 *
 ****************************************************************************/

uintptr_t sam_physramaddr(uintptr_t vregaddr)
{
  /* Check for internal SRAM.  We we assume that ISRAM0 and ISRAM1 are
   * contiguous.
   */

  if (vregaddr >= SAM_ISRAM_VSECTION &&
      vregaddr < (SAM_ISRAM_VSECTION + SAM_ISRAM_SIZE))
    {
      return isram_physramaddr(vregaddr);
    }

#ifdef CONFIG_SAMA5_DDRCS
  /* Check for external SDRAM */

  else if (vregaddr >= SAM_DDRCS_VSECTION &&
           vregaddr < (SAM_DDRCS_VSECTION + SAMA5_DDRCS_SIZE))
    {
      return sdram_physramaddr(vregaddr);
    }
#endif

  /* Check for NFCS SRAM.  */

  if (vregaddr >= SAM_NFCSRAM_VSECTION &&
      vregaddr < (SAM_NFCSRAM_VSECTION + SAM_NFCSRAM_SIZE))
    {
      return nfcsram_physramaddr(vregaddr);
    }

  /* Check for UDPH SRAM.  */

  if (vregaddr >= SAM_UDPHSRAM_VSECTION &&
      vregaddr < (SAM_UDPHSRAM_VSECTION + SAM_UDPHSRAM_SIZE))
    {
      return udphsram_physramaddr(vregaddr);
    }

#if defined(CONFIG_SAMA5_EBICS0) && (defined(CONFIG_SAMA5_EBICS0_SRAM) || \
    defined(CONFIG_SAMA5_EBICS0_PSRAM))
  /* Check for external SRAM or PSRAM on CS0 */

  else if (vregaddr >= SAM_EBICS0_VSECTION &&
           vregaddr < (SAM_EBICS0_VSECTION + SAMA5_EBICS0_SIZE))
    {
      return ebics0_physramaddr(vregaddr);
    }
#endif

#if defined(CONFIG_SAMA5_EBICS1) && (defined(CONFIG_SAMA5_EBICS1_SRAM) || \
    defined(CONFIG_SAMA5_EBICS1_PSRAM))
  /* Check for external SRAM or PSRAM on CS1 */

  else if (vregaddr >= SAM_EBICS1_VSECTION &&
           vregaddr < (SAM_EBICS1_VSECTION + SAMA5_EBICS1_SIZE))
    {
      return ebics1_physramaddr(vregaddr);
    }
#endif

#if defined(CONFIG_SAMA5_EBICS2) && (defined(CONFIG_SAMA5_EBICS2_SRAM) || \
    defined(CONFIG_SAMA5_EBICS2_PSRAM))
  /* Check for external SRAM or PSRAM on CS2 */

  else if (vregaddr >= SAM_EBICS2_VSECTION &&
           vregaddr < (SAM_EBICS2_VSECTION + SAMA5_EBICS2_SIZE))
    {
      return ebics2_physramaddr(vregaddr);
    }
#endif

#if defined(CONFIG_SAMA5_EBICS3) && (defined(CONFIG_SAMA5_EBICS3_SRAM) || \
    defined(CONFIG_SAMA5_EBICS3_PSRAM))
  /* Check for external SRAM or PSRAM on CS3 */

  else if (vregaddr >= SAM_EBICS3_VSECTION &&
           vregaddr < (SAM_EBICS3_VSECTION + SAMA5_EBICS3_SIZE))
    {
      return ebics3_physramaddr(vregaddr);
    }
#endif

  /* We will not get here unless we are called with an invalid or
   * unsupported RAM address
   */

  DEBUGPANIC();
  return vregaddr;
}
