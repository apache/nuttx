/************************************************************************************
 * arch/arm/src/lpc313x/lpc313x_boot.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm.h"
#include "up_internal.h"
#include "up_arch.h"

#include "lpc313x_syscreg.h"
#include "lpc313x_cgudrvr.h"
#include "lpc313x_internal.h"

/************************************************************************************
 * Private Types
 ************************************************************************************/

/************************************************************************************
 * Private Types
 ************************************************************************************/

struct section_mapping_s
{
  uint32_t physbase;   /* Physical address of the region to be mapped */
  uint32_t virtbase;   /* Virtual address of the region to be mapped */
  uint32_t mmuflags;   /* MMU settings for the region (e.g., cache-able) */
  uint32_t nsections;  /* Number of mappings in the region */
};

/************************************************************************************
 * Public Variables
 ************************************************************************************/

extern uint32_t _vector_start; /* Beginning of vector block */
extern uint32_t _vector_end;   /* End+1 of vector block */

/************************************************************************************
 * Private Variables
 ************************************************************************************/

/* This table describes how to map a set of 1Mb pages to space the physical address
 * space of the LPCD313x.
 */

#ifndef CONFIG_ARCH_ROMPGTABLE
static const struct section_mapping_s section_mapping[] =
{
  { LPC313X_SHADOWSPACE_PSECTION, LPC313X_SHADOWSPACE_VSECTION, 
    LPC313X_SHADOWSPACE_MMUFLAGS, LPC313X_SHADOWSPACE_NSECTIONS},
  { LPC313X_INTSRAM_PSECTION, LPC313X_INTSRAM_VSECTION, 
    LPC313X_INTSRAM_MMUFLAGS, LPC313X_INTSRAM_NSECTIONS},
#ifdef CONFIG_ARCH_ROMPGTABLE
  { LPC313X_INTSROM0_PSECTION, LPC313X_INTSROM0_VSECTION, 
    LPC313X_INTSROM_MMUFLAGS, LPC313X_INTSROM0_NSECTIONS},
#endif
  { LPC313X_APB0_PSECTION, LPC313X_APB0_VSECTION, 
    LPC313X_APB0_MMUFLAGS, LPC313X_APB0_NSECTIONS},
  { LPC313X_APB1_PSECTION, LPC313X_APB1_VSECTION, 
    LPC313X_APB1_MMUFLAGS, LPC313X_APB1_NSECTIONS},
  { LPC313X_APB2_PSECTION, LPC313X_APB2_VSECTION, 
    LPC313X_APB2_MMUFLAGS, LPC313X_APB2_NSECTIONS},
  { LPC313X_APB3_PSECTION, LPC313X_APB3_VSECTION, 
    LPC313X_APB3_MMUFLAGS, LPC313X_APB3_NSECTIONS},
  { LPC313X_APB4MPMC_PSECTION, LPC313X_APB4MPMC_VSECTION, 
    LPC313X_APB4MPMC_MMUFLAGS, LPC313X_APB4MPMC_NSECTIONS},
  { LPC313X_MCI_PSECTION, LPC313X_MCI_VSECTION, 
    LPC313X_MCI_MMUFLAGS, LPC313X_MCI_NSECTIONS},
  { LPC313X_USBOTG_PSECTION, LPC313X_USBOTG_VSECTION, 
    LPC313X_USBOTG_MMUFLAGS, LPC313X_USBOTG_NSECTIONS},
#if defined(CONFIG_LPC313X_EXTSRAM0) && CONFIG_LPC313X_EXTSRAM0SIZE > 0
  { LPC313X_EXTSRAM_PSECTION, LPC313X_EXTSRAM_VSECTION, 
    LPC313X_EXTSDRAM_MMUFLAGS, LPC313X_EXTSRAM_NSECTIONS},
#endif
#if defined(CONFIG_LPC313X_EXTSDRAM) && CONFIG_LPC313X_EXTSDRAMSIZE > 0
  { LPC313X_EXTSDRAM0_PSECTION, LPC313X_EXTSDRAM0_VSECTION, 
    LPC313X_EXTSDRAM_MMUFLAGS, LPC313X_EXTSDRAM0_NSECTIONS},
#endif
  { LPC313X_INTC_PSECTION, LPC313X_INTC_VSECTION, 
    LPC313X_INTC_MMUFLAGS, LPC313X_INTC_NSECTIONS},
#ifdef CONFIG_LPC313X_EXTNAND
  { LPC313X_NAND_PSECTION, LPC313X_NAND_VSECTION 
    LPC313X_NAND_MMUFLAGS, LPC313X_NAND_NSECTIONS},
#endif
};
#define NMAPPINGS (sizeof(section_mapping) / sizeof(struct section_mapping_s))
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_setlevel1entry
 ************************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
static inline void up_setlevel1entry(uint32_t paddr, uint32_t vaddr, uint32_t mmuflags)
{
  uint32_t *pgtable = (uint32_t*)PGTABLE_BASE_VADDR;
  uint32_t  index   = vaddr >> 20;

  /* Save the page table entry */

  pgtable[index]  = (paddr | mmuflags);
}
#endif

/************************************************************************************
 * Name: up_setlevel2coarseentry
 ************************************************************************************/

static inline void up_setlevel2coarseentry(uint32_t ctabvaddr, uint32_t paddr,
                                           uint32_t vaddr, uint32_t mmuflags)
{
  uint32_t *ctable  = (uint32_t*)ctabvaddr;
  uint32_t  index;

  /* The coarse table divides a 1Mb address space up into 256 entries, each
   * corresponding to 4Kb of address space.  The coarse page table index is
   * related to the offset from the beginning of 1Mb region.
   */

  index = (vaddr & 0x000ff000) >> 12;

  /* Save the coarse table entry */

  ctable[index] = (paddr | mmuflags);
}

/************************************************************************************
 * Name: up_setupmappings
 ************************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
static void up_setupmappings(void)
{
  int i, j;

  for (i = 0; i < NMAPPINGS; i++)
    {
      uint32_t sect_paddr = section_mapping[i].physbase;
      uint32_t sect_vaddr = section_mapping[i].virtbase;
      uint32_t mmuflags   = section_mapping[i].mmuflags;

      for (j = 0; j < section_mapping[i].nsections; j++)
        {
          up_setlevel1entry(sect_paddr, sect_vaddr, mmuflags);
          sect_paddr += SECTION_SIZE;
          sect_vaddr += SECTION_SIZE;
        }
    }
}
#endif

/************************************************************************************
 * Name: up_vectormapping
 *
 * Description:
 *   Setup a special mapping for the interrupt vectors when (1) the interrupt
 *   vectors are not positioned in ROM, and when (2) the interrupt vectors are
 *   located at the high address, 0xffff0000.  When the interrupt vectors are located
 *   in ROM, we just have to assume that they were set up correctly;  When vectors
 *   are located in low memory, 0x00000000, the shadow memory region will be mapped
 *   to support them.
 *
 ************************************************************************************/

#if !defined(CONFIG_ARCH_ROMPGTABLE) && !defined(CONFIG_ARCH_LOWVECTORS)
static void up_vectormapping(void)
{
  uint32_t vector_paddr = LPC313X_VECTOR_PADDR;
  uint32_t vector_vaddr = LPC313X_VECTOR_VADDR;
  uint32_t end_paddr    = vector_paddr + VECTOR_TABLE_SIZE;

  /* We want to keep our interrupt vectors and interrupt-related logic in zero-wait
   * state internal RAM (IRAM).  The DM320 has 16Kb of IRAM positioned at physical
   * address 0x0000:0000; we need to map this to 0xffff:0000.
   */

  while (vector_paddr < end_paddr)
    {
      up_setlevel2coarseentry(PGTABLE_COARSE_BASE_VADDR,  vector_paddr,
                              vector_vaddr, MMU_L2_VECTORFLAGS);
      vector_paddr += 4096;
      vector_vaddr += 4096;
    }

  /* Now set the level 1 descriptor to refer to the level 2 coarse page table. */

  up_setlevel1entry(PGTABLE_COARSE_BASE_PADDR, LPC313X_VECTOR_VCOARSE,
                    MMU_L1_VECTORFLAGS);
}
#endif

/************************************************************************************
 * Name: up_copyvectorblock
 ************************************************************************************/

static void up_copyvectorblock(void)
{
  uint32_t *src  = (uint32_t*)&_vector_start;
  uint32_t *end  = (uint32_t*)&_vector_end;
  uint32_t *dest = (uint32_t*)LPC313X_VECTOR_VADDR;

  while (src < end)
    {
      *dest++ = *src++;
    }
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

void up_boot(void)
{
  /* __start provided the basic MMU mappings for SDRAM.  Now provide mappings for all
   * IO regions (Including the vector region).
   */

#ifndef CONFIG_ARCH_ROMPGTABLE
  up_setupmappings();

  /* Provide a special mapping for the IRAM interrupt vector positioned in high
   * memory.
   */

#ifndef CONFIG_ARCH_LOWVECTORS
  up_vectormapping();
#endif
#endif

  /* Setup up vector block.  _vector_start and _vector_end are exported from
   * up_vector.S
   */

  up_copyvectorblock();

  /* Reset all clocks */

  lpc313x_resetclks();

  /* Initialize the PLLs */

  lpc313x_hp1pllconfig();
  lpc313x_hp0pllconfig();
  
  /* Initialize clocking to settings provided by board-specific logic */

  lpc313x_clkinit(&g_boardclks);   

  /* Map first 4KB of ARM space to ISRAM area */

  putreg32(LPC313X_INTSRAM0_PADDR, LPC313X_SYSCREG_ARM926SHADOWPTR);

  /* Perform common, low-level chip initialization (might do nothing) */

  lpc313x_lowsetup();

  /* Perform early serial initialization if we are going to use the serial driver */

#ifdef CONFIG_USE_EARLYSERIALINIT
  up_earlyserialinit();
#endif

  /* Perform board-specific initialization */

#ifdef CONFIG_ARCH_LEDS
  lpc313x_boardinitialize();
#endif
}
