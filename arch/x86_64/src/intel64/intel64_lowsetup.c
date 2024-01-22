/****************************************************************************
 * arch/x86_64/src/intel64/intel64_lowsetup.c
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

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <arch/multiboot2.h>

#include "x86_64_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The actual address of the page table and gdt/ist after mapping the kernel
 * in high address.
 */

volatile uint64_t *pdpt;
volatile uint64_t *pd;
volatile uint64_t *pt;

volatile struct ist_s *ist64;
volatile struct gdt_entry_s *gdt64;

/* This holds information passed by the multiboot2 bootloader */

uint32_t mb_magic __attribute__((section(".loader.bss")));
uint32_t mb_info_struct __attribute__((section(".loader.bss")));

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_MULTIBOOT2
/****************************************************************************
 * Name: x86_64_mb2_config
 *
 * Description:
 *   Parse multiboot2 info.
 *
 ****************************************************************************/

static void x86_64_mb2_config(void)
{
  struct multiboot_tag *tag;

  /* Check that we were actually booted by a mulitboot2 bootloader */

  if (mb_magic != MULTIBOOT2_BOOTLOADER_MAGIC)
    return;

  for (tag = (struct multiboot_tag *)(uintptr_t)(mb_info_struct + 8);
    tag->type != MULTIBOOT_TAG_TYPE_END;
    tag = (struct multiboot_tag *)((uint8_t *)tag + ((tag->size + 7) & ~7)))
    {
      switch (tag->type)
        {
          case MULTIBOOT_TAG_TYPE_EFI64:
            {
              break;
            }

#ifdef CONFIG_MULTBOOT2_FB_TERM
          case MULTIBOOT_TAG_TYPE_FRAMEBUFFER:
            {
              x86_64_mb2_fbinitialize(
                (struct multiboot_tag_framebuffer *)tag);
              break;
            }
#endif

          default:
            break;
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_lowsetup
 *
 * Description:
 *   Called from intel64_head BEFORE starting the operating system in order
 *   perform any necessary, early initialization.
 *
 ****************************************************************************/

void up_lowsetup(void)
{
  /* we should be in long mode at this point */

  /* GDT is loaded with 64bit GDT  */

  /* Paging is enabled */

  /* Setup pointers for accessing Page table and GDT in high address */

  pdpt = (uint64_t *)((uintptr_t)&pdpt_low + X86_64_LOAD_OFFSET);
  pd   = (uint64_t *)((uintptr_t)&pd_low   + X86_64_LOAD_OFFSET);
  pt   = (uint64_t *)((uintptr_t)&pt_low   + X86_64_LOAD_OFFSET);

  ist64 = (struct ist_s *)((uintptr_t)&ist64_low       + X86_64_LOAD_OFFSET);
  gdt64 = (struct gdt_entry_s *)((uintptr_t)&gdt64_low + X86_64_LOAD_OFFSET);

  /* reload the GDTR with mapped high memory address */

  setgdt((void *)gdt64, (uintptr_t)(&gdt64_low_end - &gdt64_low) - 1);

  /* Do some checking on CPU compatibilities */

  x86_64_check_and_enable_capability();

#ifdef CONFIG_ARCH_MULTIBOOT2
  /* Handle multiboot2 info */

  x86_64_mb2_config();
#endif

  /* Revoke the lower memory */

  __revoke_low_memory();

  /* perform board-specific initializations */

  x86_64_boardinitialize();

#ifdef USE_EARLYSERIALINIT
  /* Early serial driver initialization */

  x86_64_earlyserialinit();
#endif

  x86_64_timer_calibrate_freq();

#ifdef CONFIG_LIB_SYSCALL
  enable_syscall();
#endif
}

