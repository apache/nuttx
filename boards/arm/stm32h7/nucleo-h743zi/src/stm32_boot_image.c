/****************************************************************************
 * boards/arm/stm32h7/nucleo-h743zi/src/stm32_boot_image.c
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

#include "nvic.h"
#include "arm_internal.h"
#include "barriers.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the first two entries on NVIC vector table */

struct arm_vector_table
{
  uint32_t spr;   /* Stack pointer on reset */
  uint32_t reset; /* Pointer to reset exception handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void cleanup_arm_nvic(void);
#ifdef CONFIG_ARMV7M_SYSTICK
static void systick_disable(void);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

  ARM_ISB();
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

#ifdef CONFIG_ARMV7M_SYSTICK
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
#endif

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
  int fd;
  ssize_t bytes;

  fd = open(path, O_RDONLY | O_CLOEXEC);
  if (fd < 0)
    {
      syslog(LOG_ERR, "Failed to open %s with: %d", path, fd);
      return fd;
    }

  bytes = pread(fd, &vt, sizeof(vt), hdr_size);
  if (bytes != sizeof(vt))
    {
      syslog(LOG_ERR, "Failed to read ARM vector table: %d", bytes);
      return bytes < 0 ? bytes : -1;
    }

#ifdef CONFIG_ARMV7M_SYSTICK
  systick_disable();
#endif

  cleanup_arm_nvic();

#ifdef CONFIG_ARMV7M_DCACHE
  up_disable_dcache();
#endif
#ifdef CONFIG_ARMV7M_ICACHE
  up_disable_icache();
#endif

#ifdef CONFIG_ARM_MPU
  mpu_control(false, false, false);
#endif

  /* Set main and process stack pointers */

  __asm__ __volatile__("\tmsr msp, %0\n" : : "r" (vt.spr));
  setcontrol(0x00);
  ARM_ISB();
  ((void (*)(void))vt.reset)();

  return 0;
}
