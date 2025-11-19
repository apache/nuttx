/****************************************************************************
 * arch/x86_64/src/intel64/intel64_tsc_oneshot.c
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
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/oneshot.h>
#include <nuttx/timers/arch_alarm.h>

#include "intel64_lowsetup.h"
#include "x86_64_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TMR_IRQ IRQ14

/****************************************************************************
 * Private Data
 ****************************************************************************/

static clkcnt_t intel64_tsc_max_delay(struct oneshot_lowerhalf_s *lower);
static clkcnt_t intel64_tsc_current(struct oneshot_lowerhalf_s *lower);
static void intel64_tsc_start_absolute(struct oneshot_lowerhalf_s *lower,
                                       clkcnt_t expected);
static void intel64_tsc_start(struct oneshot_lowerhalf_s *lower,
                              clkcnt_t delta);
static void intel64_tsc_cancel(struct oneshot_lowerhalf_s *lower);

static const struct oneshot_operations_s g_intel64_tsc_ops =
{
  .current        = intel64_tsc_current,
  .start          = intel64_tsc_start,
  .start_absolute = intel64_tsc_start_absolute,
  .cancel         = intel64_tsc_cancel,
  .max_delay      = intel64_tsc_max_delay
};

static struct oneshot_lowerhalf_s g_intel64_tsc_lowerhalf =
{
  .ops = &g_intel64_tsc_ops
};

static uint64_t g_tsc_offset;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

extern unsigned long g_x86_64_timer_freq;

#ifdef CONFIG_ARCH_INTEL64_HAVE_TSC_ADJUST
static inline_function void intel64_tsc_set_offset(uint64_t offset)
{
  uint64_t val = read_msr(MSR_IA32_TSC_ADJUST) + offset;
  write_msr(MSR_IA32_TSC_ADJUST, val);
}

static inline_function uint64_t intel64_tsc_get_offset(void)
{
  return 0ull;
}
#else
static inline_function void intel64_tsc_set_offset(uint64_t offset)
{
  g_tsc_offset = offset;
}
static inline_function uint64_t intel64_tsc_get_offset(void)
{
  return g_tsc_offset;
}
#endif

static inline_function void intel64_mask_tmr(void)
{
  /* Disable TSC Deadline interrupt */

#ifdef CONFIG_ARCH_INTEL64_TSC_DEADLINE
  write_msr(MSR_X2APIC_LVTT, TMR_IRQ | MSR_X2APIC_LVTT_TSC_DEADLINE |
            (1 << 16));
#else
  write_msr(MSR_X2APIC_LVTT, TMR_IRQ | (1 << 16));
#endif

  /* Required when using TSC deadline mode. */

  __asm__ volatile("mfence" : : : "memory");
}

static inline_function void intel64_unmask_tmr(void)
{
  /* Enable TSC Deadline interrupt */

#ifdef CONFIG_ARCH_INTEL64_TSC_DEADLINE
  write_msr(MSR_X2APIC_LVTT, TMR_IRQ | MSR_X2APIC_LVTT_TSC_DEADLINE);
#else
  write_msr(MSR_X2APIC_LVTT, TMR_IRQ);
#endif

  /* Required when using TSC deadline mode. */

  __asm__ volatile("mfence" : : : "memory");
}

static inline_function void intel64_tsc_set_compare(uint64_t deadline)
{
  write_msr(MSR_IA32_TSC_DEADLINE, deadline);
}

static inline_function uint64_t intel64_tsc_count(void)
{
  /* We do not need the memory barrier here. */

  return rdtsc();
}

static inline_function uint64_t intel64_tsc_freq(void)
{
#if CONFIG_ARCH_INTEL64_CORE_FREQ_KHZ == 0
  return g_x86_64_timer_freq;
#else
  return CONFIG_ARCH_INTEL64_CORE_FREQ_KHZ * 1000ull;
#endif
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: intel64_tsc_compare_isr
 *
 * Description:
 *   Common timer interrupt callback.  When any oneshot timer interrupt
 *   expires, this function will be called.  It will forward the call to
 *   the next level up.
 *
 * Input Parameters:
 *   oneshot - The state associated with the expired timer
 *
 * Returned Value:
 *   Always returns OK
 *
 ****************************************************************************/

static int intel64_tsc_compare_isr(int irq, void *regs, void *arg)
{
  struct oneshot_lowerhalf_s *priv = &g_intel64_tsc_lowerhalf;

  intel64_tsc_set_compare(UINT64_MAX);

  /* Then perform the callback */

  oneshot_process_callback(priv);

  return OK;
}

static clkcnt_t intel64_tsc_max_delay(struct oneshot_lowerhalf_s *lower)
{
  return UINT64_MAX;
}

static clkcnt_t intel64_tsc_current(struct oneshot_lowerhalf_s *lower)
{
  /* We do not need memory barrier here. */

  return intel64_tsc_count() + intel64_tsc_get_offset();
}

static void intel64_tsc_start_absolute(struct oneshot_lowerhalf_s *lower,
                                       clkcnt_t expected)
{
  intel64_tsc_set_compare(expected - intel64_tsc_get_offset());
}

static void intel64_tsc_start(struct oneshot_lowerhalf_s *lower,
                              clkcnt_t delta)
{
  /* TSC Deadline is triggered when the counter is less than or equal
   * to the compare value, so it cannot set an overflow value.
   */

  clkcnt_t now      = intel64_tsc_count() + intel64_tsc_get_offset();
  clkcnt_t expected = now + delta >= now ? now + delta : UINT64_MAX;
  intel64_tsc_set_compare(expected);
}

static void intel64_tsc_cancel(struct oneshot_lowerhalf_s *lower)
{
  intel64_tsc_set_compare(UINT64_MAX);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: oneshot_initialize
 *
 * Description:
 *   Initialize the oneshot timer and return a oneshot lower half driver
 *   instance.
 *
 * Returned Value:
 *   On success, a non-NULL instance of the oneshot lower-half driver is
 *   returned.  NULL is return on any failure.
 *
 ****************************************************************************/

struct oneshot_lowerhalf_s *intel64_tsc_initialize(void)
{
  struct oneshot_lowerhalf_s *priv = &g_intel64_tsc_lowerhalf;
  uint64_t freq;

  tmrinfo("oneshot_initialize\n");

  /* Attach handler */

  irq_attach(TMR_IRQ, (xcpt_t)intel64_tsc_compare_isr, NULL);

  intel64_timer_secondary_init();

  freq = intel64_tsc_freq();

  DEBUGASSERT(freq <= UINT32_MAX);

  oneshot_count_init(priv, (uint32_t)freq);

  tmrinfo("oneshot_initialize ok %p \n", priv);

  return priv;
}

void up_timer_initialize(void)
{
  uint64_t tsc = intel64_tsc_count();

  g_tsc_offset = 0ull - tsc;

  up_alarm_set_lowerhalf(intel64_tsc_initialize());
}

void intel64_timer_secondary_init(void)
{
  intel64_tsc_set_offset(g_tsc_offset);

  intel64_tsc_set_compare(UINT64_MAX);
  intel64_unmask_tmr();
}
