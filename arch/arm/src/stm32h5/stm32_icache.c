/****************************************************************************
 * arch/arm/src/stm32h5/stm32_icache.c
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
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include "arm_internal.h"
#include "mpu.h"
#include "stm32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STM32_ICACHE_INTERRUPT  (defined(CONFIG_STM32_ICACHE_INV_INT) ||\
                                   defined(CONFIG_STM32_ICACHE_ERR_INT))

/* The OTP (0x08fff000), read-only (0x08fff800, UID) and EDATA
 * (0x09000000-0x09017fff) flash areas only accept 16/32-bit accesses
 * (RM0481 Table 77) and must be mapped non-cacheable (RM0481 7.3.2).  They
 * are contiguous, so one MPU region covers them.
 */

#define STM32_ICACHE_NC_BASE      0x08fff000
#define STM32_ICACHE_NC_END       0x09018000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_icache_s
{
  uint32_t          ier;        /* Saved interrupt mask bits value */

  /* Has been initialized and HW is setup. */

  bool              initialized;

  volatile uint32_t invalidate;
  volatile uint32_t invalidate_finished;
  spinlock_t        lock;
};

struct stm32_icache_region
{
  uint8_t  num;
  uint8_t  baseaddr;
  uint8_t  rsize;
  uint16_t remapaddr;
  uint8_t  mstsel;
  uint8_t  hburst;
};

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static struct stm32_icache_s icache1 =
{
  .ier                 = 0x0,
  .initialized         = false,
  .invalidate_finished = 0,
  .lock                = SP_UNLOCKED,
};

#ifdef CONFIG_STM32_ICACHE_REGION0
static struct stm32_icache_region region0 =
{
  .num = 0,
  .baseaddr = CONFIG_STM32_ICACHE_REGION0_BADDR,
  .rsize = CONFIG_STM32_ICACHE_REGION0_RSIZE,
  .remapaddr = CONFIG_STM32_ICACHE_REGION0_REMAPADDR,
  .mstsel = CONFIG_STM32_ICACHE_REGION0_MSTSEL,
  .hburst = CONFIG_STM32_ICACHE_REGION0_HBURST,
};
#endif

#ifdef CONFIG_STM32_ICACHE_REGION1
static struct stm32_icache_region region1 =
{
  .num = 1,
  .baseaddr = CONFIG_STM32_ICACHE_REGION1_BADDR,
  .rsize = CONFIG_STM32_ICACHE_REGION1_RSIZE,
  .remapaddr = CONFIG_STM32_ICACHE_REGION1_REMAPADDR,
  .mstsel = CONFIG_STM32_ICACHE_REGION1_MSTSEL,
  .hburst = CONFIG_STM32_ICACHE_REGION1_HBURST,
};
#endif

#ifdef CONFIG_STM32_ICACHE_REGION2
static struct stm32_icache_region region2 =
{
  .num = 2,
  .baseaddr = CONFIG_STM32_ICACHE_REGION2_BADDR,
  .rsize = CONFIG_STM32_ICACHE_REGION2_RSIZE,
  .remapaddr = CONFIG_STM32_ICACHE_REGION2_REMAPADDR,
  .mstsel = CONFIG_STM32_ICACHE_REGION2_MSTSEL,
  .hburst = CONFIG_STM32_ICACHE_REGION2_HBURST,
};
#endif

#ifdef CONFIG_STM32_ICACHE_REGION3
static struct stm32_icache_region region3 =
{
  .num = 3,
  .baseaddr = CONFIG_STM32_ICACHE_REGION3_BADDR,
  .rsize = CONFIG_STM32_ICACHE_REGION3_RSIZE,
  .remapaddr = CONFIG_STM32_ICACHE_REGION3_REMAPADDR,
  .mstsel = CONFIG_STM32_ICACHE_REGION3_MSTSEL,
  .hburst = CONFIG_STM32_ICACHE_REGION3_HBURST,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void stm32_icache_interrupt(int irq, void *context, void *arg)
{
  uint32_t sr = getreg32(STM32_ICACHE_SR);

  if ((icache1.ier & ICACHE_IER_BSYENDIE) && (sr & ICACHE_SR_BSYENDF))
    {
      putreg32(ICACHE_FCR_CBSYENDF, STM32_ICACHE_FCR);
      icache1.invalidate_finished = true;
    }

  if ((icache1.ier & ICACHE_IER_ERRIE) && (sr & ICACHE_SR_ERRF))
    {
      /* Clear Error Flag */

      putreg32(ICACHE_FCR_CERRF, STM32_ICACHE_FCR);
    }
}

static inline void stm32_icache_invf_poll(void)
{
  while (!(getreg32(STM32_ICACHE_SR) & ICACHE_SR_BSYENDF))
    {
    }

  putreg32(ICACHE_FCR_CBSYENDF, STM32_ICACHE_FCR);
  icache1.invalidate_finished = true;
}

static inline void stm32_icache_invf_interrupt(void)
{
  while (!(icache1.invalidate_finished))
    {
    }

  /* Report invalidate is finished */
}

static inline void stm32_icache_set_ier(uint32_t ier)
{
  icache1.ier = ier & ICACHE_IER_ALLINTS;
  putreg32(icache1.ier, STM32_ICACHE_IER);
}

static inline void stm32_icache_reset_hmon(void)
{
  uint32_t regval;

  regval = getreg32(STM32_ICACHE_CR);
  regval |= ICACHE_CR_HITMRST;
  putreg32(regval, STM32_ICACHE_CR);
  regval &= ~(ICACHE_CR_HITMRST);
  putreg32(regval, STM32_ICACHE_CR);
}

static inline void stm32_icache_reset_mmon(void)
{
  uint32_t regval;

  regval = getreg32(STM32_ICACHE_CR);
  regval |= ICACHE_CR_MISSMRST;
  putreg32(regval, STM32_ICACHE_CR);
  regval &= ~(ICACHE_CR_MISSMRST);
  putreg32(regval, STM32_ICACHE_CR);
}

static inline void stm32_icache_enable_monitors(void)
{
  uint32_t regval;

  regval = getreg32(STM32_ICACHE_CR);
  regval |= (ICACHE_CR_MISSMEN | ICACHE_CR_HITMEN);
  putreg32(regval, STM32_ICACHE_CR);
}

static inline void stm32_icache_disable_monitors(void)
{
  uint32_t regval;

  regval = getreg32(STM32_ICACHE_CR);
  regval &= ~(ICACHE_CR_MISSMEN | ICACHE_CR_HITMEN);
  putreg32(regval, STM32_ICACHE_CR);
}

#if defined(CONFIG_STM32_ICACHE_REGION0) || \
    defined(CONFIG_STM32_ICACHE_REGION1) || \
    defined(CONFIG_STM32_ICACHE_REGION2) || \
    defined(CONFIG_STM32_ICACHE_REGION3)
static void stm32_icache_setup_region(struct stm32_icache_region region)
{
  uint32_t regval = 0;

  regval |= (region.baseaddr << ICACHE_CRR_BASEADDR_SHIFT);
  regval |= ((region.rsize << ICACHE_CRR_RSIZE_SHIFT) & \
             ICACHE_CRR_RSIZE_MASK);
  regval |= ICACHE_CRR_REN;
  regval |= ((region.remapaddr << ICACHE_CRR_REMAPADDR_SHIFT) & \
             ICACHE_CRR_REMAPADDR_MASK);
  regval |= (region.mstsel << ICACHE_CRR_MSTSEL_SHIFT);
  regval |= (region.hburst << ICACHE_CRR_HBURST_SHIFT);

  putreg32(regval, STM32_ICACHE_CRR(region.num));
}
#endif

static bool stm32_icache_wait_invalidate(void)
{
  uint32_t timeout = STM32_ICACHE_BUSY_TIMEOUT;

  /* Wait for a running invalidate (after reset, CACHEINV or EN=0) to end;
   * the ICACHE should not be enabled before (RM0481 8.4.5).  Returns false
   * on timeout.
   */

  while ((getreg32(STM32_ICACHE_SR) & ICACHE_SR_BUSYF) != 0)
    {
      if (--timeout == 0)
        {
          return false;
        }
    }

  return true;
}

static void stm32_icache_mpu_setup(void)
{
  /* Non-cacheable, execute-never.  stm32_mpuinitialize() enabled the MPU. */

  DEBUGASSERT((getreg32(MPU_CTRL) & MPU_CTRL_ENABLE) != 0);

  mpu_configure_region(STM32_ICACHE_NC_BASE,
                       STM32_ICACHE_NC_END - STM32_ICACHE_NC_BASE,
                       MPU_RBAR_XN | MPU_RBAR_AP_RWRW | MPU_RBAR_SH_NO,
                       MPU_RLAR_NONCACHEABLE);
}

void stm32_icache_initialize(void)
{
#ifdef CONFIG_STM32_ICACHE_DIRECT
  uint32_t regval;
#endif

  /* The flash areas that cannot be cached must be excluded before the
   * ICACHE is enabled.
   */

  stm32_icache_mpu_setup();

  /* Set associativity */

#ifdef CONFIG_STM32_ICACHE_DIRECT
  regval = getreg32(STM32_ICACHE_CR);
  regval &= ~(ICACHE_CR_WAYSEL);
  putreg32(regval, STM32_ICACHE_CR);
#endif

/* Enable Hit/Miss Monitors
 * Use CONFIG options to Enable Hit/Miss
 * Reset Monitors on Initialization
 */

#ifdef CONFIG_STM32_ICACHE_MONITOR_EN
  stm32_icache_enable_monitors();
  stm32_icache_reset_monitors();
#endif

  /* Set up region configuration registers */

#ifdef CONFIG_STM32_ICACHE_REGION0
  stm32_icache_setup_region(region0);
#endif
#ifdef CONFIG_STM32_ICACHE_REGION1
  stm32_icache_setup_region(region1);
#endif
#ifdef CONFIG_STM32_ICACHE_REGION2
  stm32_icache_setup_region(region2);
#endif
#ifdef CONFIG_STM32_ICACHE_REGION3
  stm32_icache_setup_region(region3);
#endif

#if STM32_ICACHE_INTERRUPT
  /* Attach ISR */

  int ret;

  ret = irq_attach(STM32_IRQ_ICACHE, (xcpt_t) stm32_icache_interrupt, NULL);

  /* Enable Interrupts */

  if (ret == OK)
    {
      uint32_t ier = 0;
#  ifdef CONFIG_STM32_ICACHE_INV_INT
      ier |= ICACHE_IER_BSYENDIE;
#  endif
#  ifdef CONFIG_STM32_ICACHE_ERR_INT
      ier |= ICACHE_IER_ERRIE;
#  endif
      stm32_icache_set_ier(ier);

      up_enable_irq(STM32_IRQ_ICACHE);
    }
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void stm32_icache_reset_monitors(void)
{
  uint32_t regval;

  regval = getreg32(STM32_ICACHE_CR);
  regval |= (ICACHE_CR_MISSMRST | ICACHE_CR_HITMRST);
  putreg32(regval, STM32_ICACHE_CR);
  regval &= ~(ICACHE_CR_MISSMRST | ICACHE_CR_HITMRST);
  putreg32(regval, STM32_ICACHE_CR);
}

size_t stm32_get_icache_linesize(void)
{
  return 16;
}

size_t stm32_get_icache_size(void)
{
  return 8192;
}

void stm32_disable_icache(void)
{
  uint32_t regval;

  regval = getreg32(STM32_ICACHE_CR);
  regval &= ~(ICACHE_CR_EN);
  putreg32(regval, STM32_ICACHE_CR);

  /* Disabling the ICACHE starts a full invalidate, wait for it to finish.
   * The ICACHE is disabled whether or not the wait times out, so there is
   * nothing to report: stm32_enable_icache() waits again before enabling.
   */

  stm32_icache_wait_invalidate();
  putreg32(ICACHE_FCR_CBSYENDF | ICACHE_FCR_CERRF, STM32_ICACHE_FCR);
}

bool stm32_icache_enabled(void)
{
  return (getreg32(STM32_ICACHE_CR) & ICACHE_CR_EN) != 0;
}

int stm32_enable_icache(void)
{
  uint32_t regval;

  if (icache1.initialized != true)
    {
      /* A bootloader may have left the ICACHE enabled.  Disable it, which
       * also invalidates it: WAYSEL and the region registers can only be
       * written while EN=0 (RM0481 8.4.4, 8.4.7).
       */

      stm32_disable_icache();

      stm32_icache_initialize();
      icache1.initialized = true;
    }

  /* If the invalidate times out, leave the ICACHE disabled */

  if (!stm32_icache_wait_invalidate())
    {
      return -ETIMEDOUT;
    }

  /* Enable the ICACHE */

  regval = getreg32(STM32_ICACHE_CR);
  regval |= ICACHE_CR_EN;
  putreg32(regval, STM32_ICACHE_CR);
  return OK;
}

void stm32_invalidate_icache(void)
{
  uint32_t regval;

  /* Preemptively clear BSYENDF */

  putreg32(ICACHE_FCR_CBSYENDF, STM32_ICACHE_FCR);

  /* Set invalidate finished to false */

  icache1.invalidate_finished = false;

  /* Start the icache invalidate process */

  regval = getreg32(STM32_ICACHE_CR);
  regval |= ICACHE_CR_CACHEINV;
  putreg32(regval, STM32_ICACHE_CR);

#if defined(CONFIG_STM32_ICACHE_INV_INT)
  stm32_icache_invf_interrupt();
#else
  stm32_icache_invf_poll();
#endif
}
