/****************************************************************************
 * arch/arm/src/dm320/dm320_boot.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include "chip.h"
#include "arm.h"
#include "arm_internal.h"
#include "arm_arch.h"

#include <nuttx/board.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct section_mapping_s
{
  uint32_t physbase;   /* Physical address of the region to be mapped */
  uint32_t virtbase;   /* Virtual address of the region to be mapped */
  uint32_t mmuflags;   /* MMU settings for the region (e.g., cache-able) */
  uint32_t nsections;  /* Number of mappings in the region */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t _vector_start; /* Beginning of vector block */
extern uint32_t _vector_end;   /* End+1 of vector block */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct section_mapping_s section_mapping[] =
{
  { DM320_PERIPHERALS_PSECTION, DM320_PERIPHERALS_VSECTION,
    DM320_PERIPHERALS_MMUFLAGS, DM320_PERIPHERALS_NSECTIONS},
  { DM320_FLASH_PSECTION,       DM320_FLASH_VSECTION,
    DM320_FLASH_MMUFLAGS,       DM320_FLASH_NSECTIONS},
  { DM320_CFI_PSECTION,         DM320_CFI_VSECTION,
    DM320_CFI_MMUFLAGS,         DM320_CFI_NSECTIONS},
  { DM320_SSFDC_PSECTION,       DM320_SSFDC_VSECTION,
    DM320_SSFDC_MMUFLAGS,       DM320_SSFDC_NSECTIONS},
  { DM320_CE1_PSECTION,         DM320_CE1_VSECTION,
    DM320_CE1_MMUFLAGS,         DM320_CE1_NSECTIONS},
  { DM320_CE2_PSECTION,         DM320_CE2_VSECTION,
    DM320_CE2_MMUFLAGS,         DM320_CE2_NSECTIONS},
  { DM320_VLYNQ_PSECTION,       DM320_VLYNQ_VSECTION,
    DM320_VLYNQ_MMUFLAGS,       DM320_VLYNQ_NSECTIONS},
  { DM320_USBOTG_PSECTION,      DM320_USBOTG_VSECTION,
    DM320_USBOTG_MMUFLAGS,      DM320_USBOTG_NSECTIONS}
};
#define NMAPPINGS (sizeof(section_mapping) / sizeof(struct section_mapping_s))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_setlevel1entry
 ****************************************************************************/

static inline void up_setlevel1entry(uint32_t paddr,
                                     uint32_t vaddr, uint32_t mmuflags)
{
  uint32_t *pgtable = (uint32_t *)PGTABLE_BASE_VADDR;
  uint32_t  index   = vaddr >> 20;

  /* Save the page table entry */

  pgtable[index]  = (paddr | mmuflags);
}

/****************************************************************************
 * Name: up_setlevel2coarseentry
 ****************************************************************************/

static inline void
up_setlevel2coarseentry(uint32_t ctabvaddr, uint32_t paddr,
                        uint32_t vaddr, uint32_t mmuflags)
{
  uint32_t *ctable  = (uint32_t *)ctabvaddr;
  uint32_t  index;

  /* The coarse table divides a 1Mb address space up into 256 entries, each
   * corresponding to 4Kb of address space.  The coarse page table index is
   * related to the offset from the beginning of 1Mb region.
   */

  index = (vaddr & 0x000ff000) >> 12;

  /* Save the coarse table entry */

  ctable[index] = (paddr | mmuflags);
}

/****************************************************************************
 * Name: up_setupmappings
 ****************************************************************************/

static void up_setupmappings(void)
{
  int i;
  int j;

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

/****************************************************************************
 * Name: up_vectormapping
 ****************************************************************************/

static void up_vectormapping(void)
{
  uint32_t vector_paddr = DM320_IRAM_PADDR;
  uint32_t vector_vaddr = DM320_VECTOR_VADDR;
  uint32_t end_paddr    = vector_paddr + DM320_IRAM_SIZE;

  /* We want to keep our interrupt vectors and interrupt-related logic in
   * zero-wait state internal RAM (IRAM).  The DM320 has 16Kb of IRAM
   * positioned at physical address 0x0000:0000;
   * we need to map this to 0xffff:0000.
   */

  while (vector_paddr < end_paddr)
    {
      up_setlevel2coarseentry(PGTABLE_L2_COARSE_VBASE,
                              vector_paddr,
                              vector_vaddr,
                              MMU_L2_VECTORFLAGS);
      vector_paddr += 4096;
      vector_vaddr += 4096;
    }

  /* Now set the level 1 descriptor to refer to the level 2 coarse page
   * table.
   */

  up_setlevel1entry(PGTABLE_L2_COARSE_PBASE,
                    DM320_VECTOR_VCOARSE,
                    MMU_L1_VECTORFLAGS);
}

/****************************************************************************
 * Name: up_copyvectorblock
 ****************************************************************************/

static void up_copyvectorblock(void)
{
  uint32_t *src  = (uint32_t *)&_vector_start;
  uint32_t *end  = (uint32_t *)&_vector_end;
  uint32_t *dest = (uint32_t *)VECTOR_BASE;

  while (src < end)
    {
      *dest++ = *src++;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void arm_boot(void)
{
  /* __start provided the basic MMU mappings for SDRAM.
   * Now provide mappings for all IO regions (Including the vector region).
   */

  up_setupmappings();

  /* Provide a special mapping for the IRAM interrupt vector positioned in
   * high memory.
   */

  up_vectormapping();

  /* Setup up vector block.  _vector_start and _vector_end are exported from
   * up_vector.S
   */

  up_copyvectorblock();

  /* Set up the board-specific LEDs */

#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif

  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
  arm_earlyserialinit();
#endif
}
