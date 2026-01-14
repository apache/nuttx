/****************************************************************************
 * boards/arm/nrf91/common/src/nrf91_boot_image.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <debug.h>
#include <stdio.h>
#include <fcntl.h>

#include <sys/boardctl.h>
#include <nuttx/irq.h>
#include <nuttx/cache.h>
#include <arch/barriers.h>

#include "nvic.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NRF91_OTA_PRIMARY_SLOT_OFFSET
#  error CONFIG_NRF91_OTA_PRIMARY_SLOT_OFFSET not defined
#endif

#ifndef CONFIG_NRF91_MCUBOOT_HEADER_SIZE
#  error CONFIG_NRF91_MCUBOOT_HEADER_SIZE not defined
#endif

#define VECTOR_TABLE_NS ((CONFIG_NRF91_OTA_PRIMARY_SLOT_OFFSET + \
                          CONFIG_NRF91_MCUBOOT_HEADER_SIZE))

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_NRF91_NONSECURE_BOOT
typedef void tz_nonsecure_call nsfunc(void);
#endif

/* This structure represents the first two entries on NVIC vector table */

struct arm_vector_table
{
  uint32_t spr;   /* Stack pointer on reset */
  uint32_t reset; /* Pointer to reset exception handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_NRF91_NONSECURE_BOOT
static void cleanup_arm_nvic(void);
static void systick_disable(void);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_NRF91_NONSECURE_BOOT
/****************************************************************************
 * Name:  cleanup_arm_nvic
 *
 * Description:
 *   Acknowledge and disable all interrupts in NVIC
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    None
 *
 ****************************************************************************/

static void cleanup_arm_nvic(void)
{
  int i;

  /* Allow any pending interrupts to be recognized */

  UP_ISB();
  cpsid();

  /* Disable all interrupts */

  for (i = 0; i < NR_IRQS; i += 32)
    {
      putreg32(0xffffffff, NVIC_IRQ_CLEAR(i));
    }

  /* Clear all pending interrupts */

  for (i = 0; i < NR_IRQS; i += 32)
    {
      putreg32(0xffffffff, NVIC_IRQ_CLRPEND(i));
    }
}
#endif

/****************************************************************************
 * Name:  systick_disable
 *
 * Description:
 *   Disable the SysTick system timer
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    None
 *
 ****************************************************************************/

static void systick_disable(void)
{
  putreg32(0, NVIC_SYSTICK_CTRL);
  putreg32(NVIC_SYSTICK_RELOAD_MASK, NVIC_SYSTICK_RELOAD);
  putreg32(0, NVIC_SYSTICK_CURRENT);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_boot_image
 *
 * Description:
 *   This entry point is called by bootloader to jump to application image.
 *
 ****************************************************************************/

int board_boot_image(const char *path, uint32_t hdr_size)
{
  static struct arm_vector_table vt;
  struct file file;
  ssize_t bytes;
  int ret;

  ret = file_open(&file, path, O_RDONLY | O_CLOEXEC);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to open %s with: %d", path, ret);
      return ret;
    }

  bytes = file_pread(&file, &vt, sizeof(vt), hdr_size);
  if (bytes != sizeof(vt))
    {
      syslog(LOG_ERR, "Failed to read ARM vector table: %d", bytes);
      return bytes < 0 ? bytes : -1;
    }

#ifndef CONFIG_NRF91_NONSECURE_BOOT
  /* Secure entry point */

  systick_disable();

  cleanup_arm_nvic();

  /* Set main and process stack pointers */

  __asm__ __volatile__("\tmsr msp, %0\n"
                       "\tmsr control, %1\n"
                       "\tisb\n"
                       "\tmov pc, %2\n"
                       :
                       : "r" (vt.spr), "r" (0), "r" (vt.reset));

#else
  /* Non-secure entry point */

  systick_disable();

  /* Set non-secure vector table */

  putreg32(VECTOR_TABLE_NS, (NVIC_VECTAB + ARMV8M_NS_OFFSET));

  /* Set non-secure stack pointers */

  __asm__ __volatile__("\tmsr msp_ns, %0\n" : : "r" (vt.spr));

  UP_ISB();

  /* Check if reset is valid */

  if (vt.reset == 0xffffffff)
    {
      syslog(LOG_ERR, "Not found image to boot!");
      PANIC();
    }

  syslog(LOG_INFO, "Jump to 0x%" PRIx32 "\n", vt.reset);

  /* Jump to non-secure entry point */

  nsfunc *ns_reset = (nsfunc *)(vt.reset);
  ns_reset();
#endif

  return 0;
}
