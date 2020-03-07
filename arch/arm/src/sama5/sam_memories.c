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

#include <nuttx/addrenv.h>

#include "chip.h"
#include "sam_pgalloc.h"
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
 *   Given the virtual address of a peripheral A register, return the
 *   physical address of the register
 *
 ****************************************************************************/

static inline uintptr_t peripha_physregaddr(uintptr_t virtregaddr)
{
#if SAM_PERIPHA_PSECTION != SAM_PERIPHA_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * register
   */

  uintptr_t sectoffset = virtregaddr - SAM_PERIPHA_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_PERIPHA_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return virtregaddr;

#endif
}

/****************************************************************************
 * Name: periphb_physregaddr
 *
 * Description:
 *   Given the virtual address of a peripheral B register, return the
 *   physical address of the register
 *
 ****************************************************************************/

static inline uintptr_t periphb_physregaddr(uintptr_t virtregaddr)
{
#if SAM_PERIPHB_PSECTION != SAM_PERIPHB_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * register
   */

  uintptr_t sectoffset = virtregaddr - SAM_PERIPHB_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_PERIPHB_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return virtregaddr;

#endif
}

/****************************************************************************
 * Name: periphc_physregaddr
 *
 * Description:
 *   Given the virtual address of a peripheral C register, return the
 *   physical address of the register
 *
 ****************************************************************************/

#ifdef SAM_PERIPHC_VSECTION
static inline uintptr_t periphc_physregaddr(uintptr_t virtregaddr)
{
#if SAM_PERIPHB_PSECTION != SAM_PERIPHB_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * register
   */

  uintptr_t sectoffset = virtregaddr - SAM_PERIPHC_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_PERIPHC_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return virtregaddr;

#endif
}
#endif

/****************************************************************************
 * Name: sysc_physregaddr
 *
 * Description:
 *   Given the virtual address of a system controller register, return the
 *   physical address of the register
 *
 ****************************************************************************/

#ifdef SAM_SYSC_VSECTION
static inline uintptr_t sysc_physregaddr(uintptr_t virtregaddr)
{
#if SAM_SYSC_PSECTION != SAM_SYSC_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * register
   */

  uintptr_t sectoffset = virtregaddr - SAM_SYSC_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_SYSC_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return virtregaddr;

#endif
}
#endif

/****************************************************************************
 * Name: isram_physramaddr
 *
 * Description:
 *   Given the virtual address of an internal SRAM memory location, return the
 *   physical address of that location
 *
 ****************************************************************************/

static inline uintptr_t isram_physramaddr(uintptr_t virtramaddr)
{
#if SAM_ISRAM_PSECTION != SAM_ISRAM_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = virtramaddr - SAM_ISRAM_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_ISRAM_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return virtramaddr;

#endif
}

/****************************************************************************
 * Name: sdram_physramaddr
 *
 * Description:
 *   Given the virtual address of an external SDRAM memory location, return
 *   the physical address of that location
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_DDRCS) || defined(CONFIG_SAMA5_BOOT_SDRAM) || \
    defined(CONFIG_BOOT_SDRAM_DATA)
static inline uintptr_t sdram_physramaddr(uintptr_t virtramaddr)
{
#if SAM_DDRCS_PSECTION != SAM_DDRCS_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = virtramaddr - SAM_DDRCS_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_DDRCS_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return virtramaddr;

#endif
}
#endif

/****************************************************************************
 * Name: nfcsram_physramaddr
 *
 * Description:
 *   Given the virtual address of an NFC SRAM memory location, return the
 *   physical address of that location.  If NFC SRAM is not being used by
 *   the NAND logic, then it may be used a general purpose SRAM.
 *
 ****************************************************************************/

static inline uintptr_t nfcsram_physramaddr(uintptr_t virtramaddr)
{
#if SAM_NFCSRAM_PSECTION != SAM_NFCSRAM_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = virtramaddr - SAM_NFCSRAM_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_NFCSRAM_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return virtramaddr;

#endif
}

/****************************************************************************
 * Name: udphsram_physramaddr
 *
 * Description:
 *   Given the virtual address of an UDPH SRAM memory location, return the
 *   physical address of that location
 *
 ****************************************************************************/

static inline uintptr_t udphsram_physramaddr(uintptr_t virtramaddr)
{
#if SAM_UDPHSRAM_PSECTION != SAM_UDPHSRAM_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = virtramaddr - SAM_UDPHSRAM_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_UDPHSRAM_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return virtramaddr;

#endif
}

/****************************************************************************
 * Name: ebics0_physramaddr
 *
 * Description:
 *   Given the virtual address of an external CS0 SRAM memory location,
 *   return the physical address of that location
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_EBICS0) && (defined(CONFIG_SAMA5_EBICS0_SRAM) || \
    defined(CONFIG_SAMA5_EBICS0_PSRAM))
static inline uintptr_t ebics0_physramaddr(uintptr_t virtramaddr)
{
#if SAM_EBICS0_PSECTION != SAM_EBICS0_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = virtramaddr - SAM_EBICS0_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_EBICS0_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return virtramaddr;

#endif
}
#endif

/****************************************************************************
 * Name: ebics1_physramaddr
 *
 * Description:
 *   Given the virtual address of an external CS1 SRAM memory location,
 *   return the physical address of that location
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_EBICS1) && (defined(CONFIG_SAMA5_EBICS1_SRAM) || \
    defined(CONFIG_SAMA5_EBICS1_PSRAM))
static inline uintptr_t ebics1_physramaddr(uintptr_t virtramaddr)
{
#if SAM_EBICS1_PSECTION != SAM_EBICS1_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = virtramaddr - SAM_EBICS1_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_EBICS1_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return virtramaddr;

#endif
}
#endif

/****************************************************************************
 * Name: ebics2_physramaddr
 *
 * Description:
 *   Given the virtual address of an external CS2 SRAM memory location,
 *   return the physical address of that location
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_EBICS2) && (defined(CONFIG_SAMA5_EBICS2_SRAM) || \
    defined(CONFIG_SAMA5_EBICS2_PSRAM))
static inline uintptr_t ebics2_physramaddr(uintptr_t virtramaddr)
{
#if SAM_EBICS2_PSECTION != SAM_EBICS2_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = virtramaddr - SAM_EBICS2_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_EBICS2_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return virtramaddr;

#endif
}
#endif

/****************************************************************************
 * Name: ebics3_physramaddr
 *
 * Description:
 *   Given the virtual address of an external CS3 SRAM memory location,
 *   return the physical address of that location
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_EBICS3) && (defined(CONFIG_SAMA5_EBICS3_SRAM) || \
    defined(CONFIG_SAMA5_EBICS3_PSRAM))
static inline uintptr_t ebics3_physramaddr(uintptr_t virtramaddr)
{
#if SAM_EBICS3_PSECTION != SAM_EBICS3_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = virtramaddr - SAM_EBICS3_VSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_EBICS3_PSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return virtramaddr;

#endif
}
#endif

/****************************************************************************
 * Name: isram_virtramaddr
 *
 * Description:
 *   Given the physical address of an internal SRAM memory location, return
 *   the virtual address of that location
 *
 ****************************************************************************/

static inline uintptr_t isram_virtramaddr(uintptr_t physramaddr)
{
#if SAM_ISRAM_PSECTION != SAM_ISRAM_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = physramaddr - SAM_ISRAM_PSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_ISRAM_VSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return physramaddr;

#endif
}

/****************************************************************************
 * Name: sdram_virtramaddr
 *
 * Description:
 *   Given the physical address of an external SDRAM memory location, return
 *   the virtual address of that location
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_DDRCS) || defined(CONFIG_SAMA5_BOOT_SDRAM) || \
    defined(CONFIG_BOOT_SDRAM_DATA)
static inline uintptr_t sdram_virtramaddr(uintptr_t physramaddr)
{
#if SAM_DDRCS_PSECTION != SAM_DDRCS_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = physramaddr - SAM_DDRCS_PSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_DDRCS_VSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return physramaddr;

#endif
}
#endif

/****************************************************************************
 * Name: nfcsram_virtramaddr
 *
 * Description:
 *   Given the physical address of an NFC SRAM memory location, return the
 *   virtual address of that location.  If NFC SRAM is not being used by
 *   the NAND logic, then it may be used a general purpose SRAM.
 *
 ****************************************************************************/

#ifndef CONFIG_SAMA5_HAVE_NAND
static inline uintptr_t nfcsram_virtramaddr(uintptr_t physramaddr)
{
#if SAM_NFCSRAM_PSECTION != SAM_NFCSRAM_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = physramaddr - SAM_NFCSRAM_PSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_NFCSRAM_VSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return physramaddr;

#endif
}
#endif

/****************************************************************************
 * Name: udphsram_virtramaddr
 *
 * Description:
 *   Given the physical address of an UDPH SRAM memory location, return the
 *   virtual address of that location
 *
 ****************************************************************************/

static inline uintptr_t udphsram_virtramaddr(uintptr_t physramaddr)
{
#if SAM_UDPHSRAM_PSECTION != SAM_UDPHSRAM_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = physramaddr - SAM_UDPHSRAM_PSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_UDPHSRAM_VSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return physramaddr;

#endif
}

/****************************************************************************
 * Name: ebics0_virtramaddr
 *
 * Description:
 *   Given the physical address of an external CS0 SRAM memory location,
 *   return the virtual address of that location
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_EBICS0) && (defined(CONFIG_SAMA5_EBICS0_SRAM) || \
    defined(CONFIG_SAMA5_EBICS0_PSRAM))
static inline uintptr_t ebics0_virtramaddr(uintptr_t physramaddr)
{
#if SAM_EBICS0_PSECTION != SAM_EBICS0_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = physramaddr - SAM_EBICS0_PSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_EBICS0_VSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return physramaddr;

#endif
}
#endif

/****************************************************************************
 * Name: ebics1_virtramaddr
 *
 * Description:
 *   Given the physical address of an external CS1 SRAM memory location,
 *   return the virtual address of that location
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_EBICS1) && (defined(CONFIG_SAMA5_EBICS1_SRAM) || \
    defined(CONFIG_SAMA5_EBICS1_PSRAM))
static inline uintptr_t ebics1_virtramaddr(uintptr_t physramaddr)
{
#if SAM_EBICS1_PSECTION != SAM_EBICS1_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = physramaddr - SAM_EBICS1_PSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_EBICS1_VSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return physramaddr;

#endif
}
#endif

/****************************************************************************
 * Name: ebics2_virtramaddr
 *
 * Description:
 *   Given the physical address of an external CS2 SRAM memory location,
 *   return the virtual address of that location
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_EBICS2) && (defined(CONFIG_SAMA5_EBICS2_SRAM) || \
    defined(CONFIG_SAMA5_EBICS2_PSRAM))
static inline uintptr_t ebics2_virtramaddr(uintptr_t physramaddr)
{
#if SAM_EBICS2_PSECTION != SAM_EBICS2_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = physramaddr - SAM_EBICS2_PSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_EBICS2_VSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return physramaddr;

#endif
}
#endif

/****************************************************************************
 * Name: ebics3_virtramaddr
 *
 * Description:
 *   Given the physical address of an external CS3 SRAM memory location,
 *   return the virtual address of that location
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_EBICS3) && (defined(CONFIG_SAMA5_EBICS3_SRAM) || \
    defined(CONFIG_SAMA5_EBICS3_PSRAM))
static inline uintptr_t ebics3_virtramaddr(uintptr_t physramaddr)
{
#if SAM_EBICS3_PSECTION != SAM_EBICS3_VSECTION
  /* Get the offset into the virtual memory region section containing the
   * RAM memory location.
   */

  uintptr_t sectoffset = physramaddr - SAM_EBICS3_PSECTION;

  /* Add that offset to the physical base address of the memory region */

  return SAM_EBICS3_VSECTION + sectoffset;

#else
  /* 1-to-1 mapping */

  return physramaddr;

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
 *   Given the virtual address of a register, return the physical address of
 *   the register
 *
 ****************************************************************************/

uintptr_t sam_physregaddr(uintptr_t virtregaddr)
{
  /* Check for a peripheral A register */

  if (virtregaddr >= SAM_PERIPHA_VSECTION &&
      virtregaddr < (SAM_PERIPHA_VSECTION + SAM_PERIPHA_SIZE))
    {
      return peripha_physregaddr(virtregaddr);
    }

  /* Check for a peripheral A register */

  else if (virtregaddr >= SAM_PERIPHB_VSECTION &&
           virtregaddr < (SAM_PERIPHB_VSECTION + SAM_PERIPHB_SIZE))
    {
      return periphb_physregaddr(virtregaddr);
    }

  /* Check for a system controller/peripheral C register
   *
   * Naming of peripheral sections differs between the SAMA5D3 and SAMA5D4.
   * There is nothing called SYSC in the SAMA5D4 memory map.  The third
   * peripheral section is un-named in the SAMA5D4 memory map, but I have
   * chosen the name PERIPHC for this usage.
   */

#ifdef SAM_PERIPHC_VSECTION
  else if (virtregaddr >= SAM_PERIPHC_VSECTION &&
           virtregaddr < (SAM_PERIPHC_VSECTION + SAM_PERIPHC_SIZE))
    {
      return periphc_physregaddr(virtregaddr);
    }
#endif

#ifdef SAM_SYSC_VSECTION
  else if (virtregaddr >= SAM_SYSC_VSECTION &&
           virtregaddr < (SAM_SYSC_VSECTION + SAM_SYSC_SIZE))
    {
      return sysc_physregaddr(virtregaddr);
    }
#endif

  /* Check for NFCS SRAM.  If NFC SRAM is being used by the NAND logic,
   * then it will be treated as peripheral space.
   */

#ifdef CONFIG_SAMA5_HAVE_NAND
  if (virtregaddr >= SAM_NFCSRAM_VSECTION &&
      virtregaddr < (SAM_NFCSRAM_VSECTION + SAM_NFCSRAM_SIZE))
    {
      return nfcsram_physramaddr(virtregaddr);
    }
#endif

  /* We will not get here unless we are called with an invalid register
   * address
   */

  serr("ERROR: Bad virtual address: %08lx\n", virtregaddr);
  DEBUGPANIC();
  return virtregaddr;
}

/****************************************************************************
 * Name: sam_physramaddr
 *
 * Description:
 *   Given the virtual address of a RAM memory location, return the physical
 *   address of that location.
 *
 ****************************************************************************/

uintptr_t sam_physramaddr(uintptr_t virtramaddr)
{
  /* Check for internal SRAM.  We we assume that ISRAM0 and ISRAM1 are
   * contiguous.
   */

  if (virtramaddr >= SAM_ISRAM_VSECTION &&
      virtramaddr < (SAM_ISRAM_VSECTION + SAM_ISRAM_SIZE))
    {
      return isram_physramaddr(virtramaddr);
    }

#if defined(CONFIG_SAMA5_DDRCS)
  /* Check for external SDRAM.  We may have external DRAM if, as examples,
   * we are running from NOR FLASH or ISRAM with data in DRAM
   * (SAMA5_BOOT_ISRAM or SAMA5_BOOT_CS0FLASH with CONFIG_SAMA5_DDRCS).  In
   * this case, the DRAM configuration settings, SAM_DDRCS_VSECTION and
   * SAMA5_DDRCS_SIZE give us the correct size of the installed DRAM.
   *
   *   SAM_DDRCS_VSECTION -- Virtual start of the DRAM memory region
   *   SAMA5_DDRCS_SIZE   -- The installed DRAM size.
   *
   * REVISIT:  However, not all of it may be mapped?  Only the "primary"
   * RAM region will be mapped by default.  See below.
   */

  else if (virtramaddr >= SAM_DDRCS_VSECTION &&
           virtramaddr < (SAM_DDRCS_VSECTION + SAMA5_DDRCS_SIZE))
    {
      return sdram_physramaddr(virtramaddr);
    }

#elif defined(CONFIG_SAMA5_BOOT_SDRAM) || defined(CONFIG_BOOT_SDRAM_DATA)
  /* The DDRCS values may be highly couple to the CONFIG_RAM_START,
   * CONFIG_RAM_VSTART, and CONFIG_RAM_SIZE when CONFIG_SAMA5_BOOT_SDRAM or
   * CONFIG_BOOT_SDRAM_DATA is selected.
   *
   *   CONFIG_SAMA5_BOOT_SDRAM -- We were booted into DRAM by some bootloader.
   *      DRAM support is not enabled, SAMA5_DDRCS_SIZE is not valid.
   *   CONFIG_BOOT_SDRAM_DATA -- We are running from NOR or ISRAM, but our
   *      .bss, .data, and primary heap are in DRAM (In this case, I would
   *      expect CONFIG_SAMA5_DDRCS to also be set, however).
   *
   * In all cases, CONFIG_RAM_START, RAM_VSTART, and RAM_SIZE describe the
   * "primary" RAM region that is mapped at boot time.  This "primary" RAM
   * region is the one that holds .bss, .data, and primary head.  And this
   * is the only DRAM memory region that is mapped at boot time.
   *
   * REVISIT:  How does this work if we want to set aside a block of DRAM
   * outside of .bss, .data, and .heap for, as an example, for a framebuffer.
   * In that case, we will to revisit this.
   */

  else if (virtramaddr >= CONFIG_RAM_VSTART &&
           virtramaddr < (CONFIG_RAM_VSTART + CONFIG_RAM_SIZE))
    {
      return sdram_physramaddr(virtramaddr);
    }
#endif

  /* Check for NFCS SRAM.  If NFC SRAM is not being used by the NAND logic,
   * then it may be used a general purpose SRAM.
   */

#ifndef CONFIG_SAMA5_HAVE_NAND
  if (virtramaddr >= SAM_NFCSRAM_VSECTION &&
      virtramaddr < (SAM_NFCSRAM_VSECTION + SAM_NFCSRAM_SIZE))
    {
      return nfcsram_physramaddr(virtramaddr);
    }
#endif

  /* Check for UDPH SRAM.  */

  if (virtramaddr >= SAM_UDPHSRAM_VSECTION &&
      virtramaddr < (SAM_UDPHSRAM_VSECTION + SAM_UDPHSRAM_SIZE))
    {
      return udphsram_physramaddr(virtramaddr);
    }

#if defined(CONFIG_SAMA5_EBICS0) && (defined(CONFIG_SAMA5_EBICS0_SRAM) || \
    defined(CONFIG_SAMA5_EBICS0_PSRAM))
  /* Check for external SRAM or PSRAM on CS0 */

  else if (virtramaddr >= SAM_EBICS0_VSECTION &&
           virtramaddr < (SAM_EBICS0_VSECTION + SAMA5_EBICS0_SIZE))
    {
      return ebics0_physramaddr(virtramaddr);
    }
#endif

#if defined(CONFIG_SAMA5_EBICS1) && (defined(CONFIG_SAMA5_EBICS1_SRAM) || \
    defined(CONFIG_SAMA5_EBICS1_PSRAM))
  /* Check for external SRAM or PSRAM on CS1 */

  else if (virtramaddr >= SAM_EBICS1_VSECTION &&
           virtramaddr < (SAM_EBICS1_VSECTION + SAMA5_EBICS1_SIZE))
    {
      return ebics1_physramaddr(virtramaddr);
    }
#endif

#if defined(CONFIG_SAMA5_EBICS2) && (defined(CONFIG_SAMA5_EBICS2_SRAM) || \
    defined(CONFIG_SAMA5_EBICS2_PSRAM))
  /* Check for external SRAM or PSRAM on CS2 */

  else if (virtramaddr >= SAM_EBICS2_VSECTION &&
           virtramaddr < (SAM_EBICS2_VSECTION + SAMA5_EBICS2_SIZE))
    {
      return ebics2_physramaddr(virtramaddr);
    }
#endif

#if defined(CONFIG_SAMA5_EBICS3) && (defined(CONFIG_SAMA5_EBICS3_SRAM) || \
    defined(CONFIG_SAMA5_EBICS3_PSRAM))
  /* Check for external SRAM or PSRAM on CS3 */

  else if (virtramaddr >= SAM_EBICS3_VSECTION &&
           virtramaddr < (SAM_EBICS3_VSECTION + SAMA5_EBICS3_SIZE))
    {
      return ebics3_physramaddr(virtramaddr);
    }
#endif

#ifdef CONFIG_ARCH_ADDRENV
  /* Check if the virtual address lies in the user data area and, if so
   * get the mapping to the physical address in the page pool.
   */

  else
    {
      uintptr_t paddr = sam_physpgaddr(virtramaddr);
      if (paddr != 0)
        {
          return paddr;
        }
    }
#endif

  /* We will not get here unless we are called with an invalid or
   * unsupported RAM address.  Special case the NULL address.
   */

  if (virtramaddr != 0)
    {
      serr("ERROR: Bad virtual address: %08lx\n", virtramaddr);
      DEBUGPANIC();
    }

  return virtramaddr;
}

/****************************************************************************
 * Name: sam_virtramaddr
 *
 * Description:
 *   Give the physical address of a RAM memory location, return the virtual
 *   address of that location.
 *
 ****************************************************************************/

uintptr_t sam_virtramaddr(uintptr_t physramaddr)
{
  /* Check for internal SRAM.  We we assume that ISRAM0 and ISRAM1 are
   * contiguous.
   */

  if (physramaddr >= SAM_ISRAM_PSECTION &&
      physramaddr < (SAM_ISRAM_PSECTION + SAM_ISRAM_SIZE))
    {
      return isram_virtramaddr(physramaddr);
    }

#ifdef CONFIG_SAMA5_DDRCS
  /* Check for external SDRAM.  NOTE:  See comments in sam_physramaddr */

  else if (physramaddr >= SAM_DDRCS_PSECTION &&
           physramaddr < (SAM_DDRCS_PSECTION + SAMA5_DDRCS_SIZE))
    {
      return sdram_virtramaddr(physramaddr);
    }

#elif defined(CONFIG_SAMA5_BOOT_SDRAM) || defined(CONFIG_BOOT_SDRAM_DATA)
  /* See comments in sam_physramaddr */

  else if (physramaddr >= CONFIG_RAM_START &&
           physramaddr < (CONFIG_RAM_START + CONFIG_RAM_SIZE))
    {
      return sdram_physramaddr(physramaddr);
    }

#endif

  /* Check for NFCS SRAM.  If NFC SRAM is not being used by the NAND logic,
   * then it may be used a general purpose SRAM.
   */

#ifndef CONFIG_SAMA5_HAVE_NAND
  if (physramaddr >= SAM_NFCSRAM_PSECTION &&
      physramaddr < (SAM_NFCSRAM_PSECTION + SAM_NFCSRAM_SIZE))
    {
      return nfcsram_virtramaddr(physramaddr);
    }
#endif

  /* Check for UDPH SRAM.  */

  if (physramaddr >= SAM_UDPHSRAM_PSECTION &&
      physramaddr < (SAM_UDPHSRAM_PSECTION + SAM_UDPHSRAM_SIZE))
    {
      return udphsram_virtramaddr(physramaddr);
    }

#if defined(CONFIG_SAMA5_EBICS0) && (defined(CONFIG_SAMA5_EBICS0_SRAM) || \
    defined(CONFIG_SAMA5_EBICS0_PSRAM))
  /* Check for external SRAM or PSRAM on CS0 */

  else if (physramaddr >= SAM_EBICS0_PSECTION &&
           physramaddr < (SAM_EBICS0_PSECTION + SAMA5_EBICS0_SIZE))
    {
      return ebics0_virtramaddr(physramaddr);
    }
#endif

#if defined(CONFIG_SAMA5_EBICS1) && (defined(CONFIG_SAMA5_EBICS1_SRAM) || \
    defined(CONFIG_SAMA5_EBICS1_PSRAM))
  /* Check for external SRAM or PSRAM on CS1 */

  else if (physramaddr >= SAM_EBICS1_PSECTION &&
           physramaddr < (SAM_EBICS1_PSECTION + SAMA5_EBICS1_SIZE))
    {
      return ebics1_virtramaddr(physramaddr);
    }
#endif

#if defined(CONFIG_SAMA5_EBICS2) && (defined(CONFIG_SAMA5_EBICS2_SRAM) || \
    defined(CONFIG_SAMA5_EBICS2_PSRAM))
  /* Check for external SRAM or PSRAM on CS2 */

  else if (physramaddr >= SAM_EBICS2_PSECTION &&
           physramaddr < (SAM_EBICS2_PSECTION + SAMA5_EBICS2_SIZE))
    {
      return ebics2_virtramaddr(physramaddr);
    }
#endif

#if defined(CONFIG_SAMA5_EBICS3) && (defined(CONFIG_SAMA5_EBICS3_SRAM) || \
    defined(CONFIG_SAMA5_EBICS3_PSRAM))
  /* Check for external SRAM or PSRAM on CS3 */

  else if (physramaddr >= SAM_EBICS3_PSECTION &&
           physramaddr < (SAM_EBICS3_PSECTION + SAMA5_EBICS3_SIZE))
    {
      return ebics3_virtramaddr(physramaddr);
    }
#endif

#ifdef CONFIG_ARCH_ADDRENV
  /* Check if the physical address lies in the page pool and, if so
   * get the mapping to the virtual address in the user data area.
   */

  else
    {
      uintptr_t vaddr = sam_virtpgaddr(physramaddr);
      if (vaddr != 0)
        {
          return vaddr;
        }
    }
#endif

  /* We will not get here unless we are called with an invalid or
   * unsupported RAM address.  Special case the NULL address.
   */

  if (physramaddr != 0)
    {
      serr("ERROR: Bad physical address: %08lx\n|", physramaddr);
      DEBUGPANIC();
    }

  return physramaddr;
}
