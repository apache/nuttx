/****************************************************************************
 * arch/x86_64/src/intel64/intel64_start.c
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

#include <nuttx/init.h>

#include <arch/board/board.h>
#include <arch/multiboot2.h>

#include <arch/acpi.h>

#include "x86_64_internal.h"

#include "intel64_lowsetup.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This holds information passed by the multiboot2 bootloader */

uint32_t g_mb_magic __attribute__((section(".loader.bss")));
uint32_t g_mb_info_struct __attribute__((section(".loader.bss")));
uintptr_t g_acpi_rsdp = 0;

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

  /* Check that we were actually booted by a multiboot2 bootloader */

  if (g_mb_magic != MULTIBOOT2_BOOTLOADER_MAGIC)
    {
      return;
    }

  for (tag = (struct multiboot_tag *)(uintptr_t)(g_mb_info_struct + 8);
       tag->type != MULTIBOOT_TAG_TYPE_END;
       tag = (struct multiboot_tag *)((uint8_t *)tag +
                                      ((tag->size + 7) & ~7)))
    {
      switch (tag->type)
        {
          case MULTIBOOT_TAG_TYPE_EFI64:
            {
              break;
            }

          case MULTIBOOT_TAG_TYPE_ACPI_OLD:
            {
              struct multiboot_tag_old_acpi *acpi
                  = (struct multiboot_tag_old_acpi *)tag;
              g_acpi_rsdp = (uintptr_t)acpi->rsdp;
              break;
            }

          case MULTIBOOT_TAG_TYPE_ACPI_NEW:
            {
              struct multiboot_tag_new_acpi *acpi =
                (struct multiboot_tag_new_acpi *)tag;
              g_acpi_rsdp = (uintptr_t)acpi->rsdp;
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
 * Name: __nxstart
 *
 * Description:
 *   Do low-level initialization and call nx_start.
 *
 ****************************************************************************/

void __nxstart(void)
{
  uint64_t *dest = NULL;

  /* Do some checking on CPU compatibilities at the top of this function.
   * BSS cleanup can be optimized with vector instructions, so we need to
   * enable SSE at this point.
   */

  x86_64_check_and_enable_capability();

  /* Clear .bss. The compiler can optimize this with vector instructions,
   * so this *must be* called after enabling SSE instructions.
   */

  for (dest = (uint64_t *)_sbss; dest < (uint64_t *)_ebss; )
    {
      *dest++ = 0;
    }

#ifdef CONFIG_ARCH_MULTIBOOT2
  /* Handle multiboot2 info */

  x86_64_mb2_config();
#endif

  /* Low-level, pre-OS initialization */

  intel64_lowsetup();

#ifdef CONFIG_ARCH_X86_64_ACPI
  /* Initialize ACPI */

  acpi_init(g_acpi_rsdp);
#endif

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

  /* Start NuttX */

  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
