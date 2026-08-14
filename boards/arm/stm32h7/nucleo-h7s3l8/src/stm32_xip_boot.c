/****************************************************************************
 * boards/arm/stm32h7/nucleo-h7s3l8/src/stm32_xip_boot.c
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

#include <errno.h>
#include <stdint.h>
#include <syslog.h>

#include <arch/barriers.h>
#include <nuttx/cache.h>

#include "arm_internal.h"
#include "nvic.h"
#include "nucleo-h7s3l8.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_XIP_BASE           0x70000000
#define STM32_XIP_SIZE           0x02000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct xip_vector_s
{
  uint32_t stack;
  uint32_t reset;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xip_vector_valid
 *
 * Description:
 *   Validate the initial stack pointer and reset vector of the external
 *   image before transferring control to it.
 *
 ****************************************************************************/

static int xip_vector_valid(const struct xip_vector_s *vector)
{
  uint32_t reset = vector->reset & ~1;

  if (vector->stack < 0x20000000 || vector->stack >= 0x40000000 ||
      reset < STM32_XIP_BASE || reset >= STM32_XIP_BASE + STM32_XIP_SIZE ||
      (vector->reset & 1) == 0)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: xip_jump
 *
 * Description:
 *   Disable active core state and transfer control to an external image.
 *   This function does not return.
 *
 ****************************************************************************/

static void xip_jump(const struct xip_vector_s *vector)
{
  int i;

  putreg32(0, NVIC_SYSTICK_CTRL);
  UP_ISB();
  cpsid();

  for (i = 0; i < NR_IRQS; i += 32)
    {
      putreg32(0xffffffff, NVIC_IRQ_CLEAR(i));
      putreg32(0xffffffff, NVIC_IRQ_CLRPEND(i));
    }

#ifdef CONFIG_ARMV7M_DCACHE
  up_disable_dcache();
#endif
#ifdef CONFIG_ARMV7M_ICACHE
  up_disable_icache();
#endif

  putreg32(STM32_XIP_BASE, NVIC_VECTAB);
  UP_DSB();
  UP_ISB();

  __asm__ __volatile__("msr msp, %0\n"
                       "msr control, %1\n"
                       "msr primask, %1\n"
                       "isb\n"
                       "bx %2\n"
                       :
                       : "r" (vector->stack), "r" (0),
                         "r" (vector->reset));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_xip_boot
 *
 * Description:
 *   Bootloader entry point (CONFIG_INIT_ENTRYPOINT) which initializes the
 *   external flash and starts the raw NuttX image at the beginning of the
 *   XSPI2 memory-mapped window.
 *
 * Returned Value:
 *   This function does not return after a successful transfer.  A negated
 *   errno value is returned if initialization or image validation fails.
 *
 ****************************************************************************/

int stm32_xip_boot(int argc, char *argv[])
{
  const struct xip_vector_s *vector =
    (const struct xip_vector_s *)STM32_XIP_BASE;
  int ret;

  UNUSED(argc);
  UNUSED(argv);

  ret = stm32_xspi_flash_initialize();
  if (ret == OK)
    {
      ret = xip_vector_valid(vector);
    }

  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: XIP boot failed: %d\n", ret);
      return ret;
    }

  xip_jump(vector);
  return -EIO;
}
