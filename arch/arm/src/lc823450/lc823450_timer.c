/****************************************************************************
 * arch/arm/src/lc823450/lc823450_timer.c
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
#include <stddef.h>
#include <time.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "nvic.h"
#include "clock/clock.h"
#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"
#include "lc823450_gpio.h"
#ifdef CONFIG_LC823450_MTM0_TICK
#  include "lc823450_pwm.h"
#endif
#include "lc823450_syscontrol.h"
#include "lc823450_clockconfig.h"
#include "lc823450_serial.h"
#include "lc823450_timer.h"

#ifdef CONFIG_DVFS
#  include "lc823450_dvfs2.h"
#endif

#if !defined(CONFIG_LC823450_MTM0_TICK) && defined (CONFIG_DVFS)
#  error "Use CONFIG_LC823450_MTM0_TICK=y"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SYSTICK_RELOAD ((lc823450_get_systemfreq() / CLK_TCK) - 1)

/* TIMER_PIN will be used to check the interval */

#define TIMER_PIN (GPIO_PORT5|GPIO_PIN7)

/* #define CHECK_INTERVAL */

#ifdef CONFIG_LC823450_MTM0_TICK
#  define MT00STS  (LC823450_MTM0_REGBASE + LC823450_MTM_0STS)
#  define MT00A    (LC823450_MTM0_REGBASE + LC823450_MTM_0A)
#  define MT00B    (LC823450_MTM0_REGBASE + LC823450_MTM_0B)
#  define MT00CTL  (LC823450_MTM0_REGBASE + LC823450_MTM_0CTL)
#  define MT00PSCL (LC823450_MTM0_REGBASE + LC823450_MTM_0PSCL)
#  define MT00TIER (LC823450_MTM0_REGBASE + LC823450_MTM_0TIER)
#  define MT00OPR  (LC823450_MTM0_REGBASE + LC823450_MTM_OPR)
#  define MT00CNT  (LC823450_MTM0_REGBASE + LC823450_MTM_0CNT)
#  define MTM_RELOAD (XT1OSC_CLK / (CLK_TCK * 10))
#endif

#ifdef CONFIG_HRT_TIMER
#  define LC823450_MTM2_REGBASE 0x40045000
#  define MT20STS  (LC823450_MTM2_REGBASE + LC823450_MTM_0STS)
#  define MT20A    (LC823450_MTM2_REGBASE + LC823450_MTM_0A)
#  define MT20PSCL (LC823450_MTM2_REGBASE + LC823450_MTM_0PSCL)
#  define MT20TIER (LC823450_MTM2_REGBASE + LC823450_MTM_0TIER)
#  define MT2OPR   (LC823450_MTM2_REGBASE + LC823450_MTM_OPR)
#  define MT20CNT  (LC823450_MTM2_REGBASE + LC823450_MTM_0CNT)
#endif /* CONFIG_HRT_TIMER */

#ifdef CONFIG_PROFILE
#  define LC823450_MTM3_REGBASE 0x40046000
#  define MT30STS  (LC823450_MTM3_REGBASE + LC823450_MTM_0STS)
#  define MT30A    (LC823450_MTM3_REGBASE + LC823450_MTM_0A)
#  define MT30B    (LC823450_MTM3_REGBASE + LC823450_MTM_0B)
#  define MT30CTL  (LC823450_MTM3_REGBASE + LC823450_MTM_0CTL)
#  define MT30PSCL (LC823450_MTM3_REGBASE + LC823450_MTM_0PSCL)
#  define MT30TIER (LC823450_MTM3_REGBASE + LC823450_MTM_0TIER)
#  define MT30OPR  (LC823450_MTM3_REGBASE + LC823450_MTM_OPR)
#  define MT30CNT  (LC823450_MTM3_REGBASE + LC823450_MTM_0CNT)
#endif /* CONFIG_PROFILE */

#ifdef CONFIG_DVFS
#  define MT01STS  (LC823450_MTM0_REGBASE + LC823450_MTM_1STS)
#  define MT01A    (LC823450_MTM0_REGBASE + LC823450_MTM_1A)
#  define MT01B    (LC823450_MTM0_REGBASE + LC823450_MTM_1B)
#  define MT01CTL  (LC823450_MTM0_REGBASE + LC823450_MTM_1CTL)
#  define MT01PSCL (LC823450_MTM0_REGBASE + LC823450_MTM_1PSCL)
#  define MT01TIER (LC823450_MTM0_REGBASE + LC823450_MTM_1TIER)
#  define MT01CNT  (LC823450_MTM0_REGBASE + LC823450_MTM_1CNT)
#  define MT0OPR   (LC823450_MTM0_REGBASE + LC823450_MTM_OPR)
#endif

#ifndef container_of
#  define container_of(ptr, type, member) \
    ((type *)((void *)(ptr) - offsetof(type, member)))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_HRT_TIMER
struct hrt_s
{
  dq_entry_t ent;
  sem_t sem;
  int usec;
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_HRT_TIMER
static dq_queue_t hrt_timer_queue;
static void hrt_queue_refresh(void);
static void hrt_usleep_setup(void);
static int hrt_interrupt(int irq, FAR void *context, FAR void *arg);
static void hrt_usleep_add(struct hrt_s *phrt);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CHECK_INTERVAL
static bool _timer_val = true;
#endif
#ifdef CONFIG_PROFILE
static int dbg;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_PROFILE
uint32_t profile_data[CONFIG_PROFILE_SAMPLES];
int profile_ptr;
int profile_en;
#endif /* CONFIG_PROFILE */

#ifdef CONFIG_HRT_TIMER
extern int g_taskid_init;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrt_queue_refresh
 ****************************************************************************/

#ifdef CONFIG_HRT_TIMER
static void hrt_queue_refresh(void)
{
  int elapsed;
  dq_entry_t *pent;
  struct hrt_s *tmp;
  irqstate_t flags;

  flags = spin_lock_irqsave();
  elapsed = (uint64_t)getreg32(MT20CNT) * (1000 * 1000) * 10 / XT1OSC_CLK;

  for (pent = hrt_timer_queue.head; pent; pent = dq_next(pent))
    {
      tmp = container_of(pent, struct hrt_s, ent);
      tmp->usec -= elapsed;
    }

cont:

  /* serch for expired */

  for (pent = hrt_timer_queue.head; pent; pent = dq_next(pent))
    {
      tmp = container_of(pent, struct hrt_s, ent);
      if (tmp->usec <= 0)
        {
          dq_rem(pent, &hrt_timer_queue);
          spin_unlock_irqrestore(flags);
          nxsem_post(&tmp->sem);
          flags = spin_lock_irqsave();
          goto cont;
        }
      else
        {
          break;
        }
    }

  spin_unlock_irqrestore(flags);
}
#endif

/****************************************************************************
 * Name: hrt_usleep_setup
 ****************************************************************************/

#ifdef CONFIG_HRT_TIMER
static void hrt_usleep_setup(void)
{
  uint32_t count;
  struct hrt_s *head;
  irqstate_t flags;

  flags = spin_lock_irqsave();
  head = container_of(hrt_timer_queue.head, struct hrt_s, ent);
  if (head == NULL)
    {
      /* MTM2: disable clocking */

      modifyreg32(MCLKCNTEXT1, MCLKCNTEXT1_MTM2C_CLKEN, 0x0);
      modifyreg32(MCLKCNTEXT1, MCLKCNTEXT1_MTM2_CLKEN, 0x0);
      spin_unlock_irqrestore(flags);
      return;
    }

  /* MTM2: enable clocking */

  modifyreg32(MCLKCNTEXT1, 0x0, MCLKCNTEXT1_MTM2_CLKEN);
  modifyreg32(MCLKCNTEXT1, 0x0, MCLKCNTEXT1_MTM2C_CLKEN);

  count = (uint64_t)XT1OSC_CLK * head->usec / (1000 * 1000) / 10;

  if (count >= 0x8000)
    {
      count = 0x7fff;
    }

  putreg32(0, MT20CNT);   /* counter */
  putreg32(count, MT20A); /* AEVT counter */

  /* Enable MTM2-Ch0 */

  putreg32(1, MT2OPR);
  spin_unlock_irqrestore(flags);
}
#endif

/****************************************************************************
 * Name: hrt_interrupt
 ****************************************************************************/

#ifdef CONFIG_HRT_TIMER
static int hrt_interrupt(int irq, FAR void *context, FAR void *arg)
{
  /* Disable MTM2-Ch0 */

  putreg32(0, MT2OPR);

  /* clear AEVT Interrupt */

  putreg32(1 << 0, MT20STS);

  hrt_queue_refresh();
  hrt_usleep_setup();
  return OK;
}

/****************************************************************************
 * Name: hrt_usleep_add
 ****************************************************************************/

static void hrt_usleep_add(struct hrt_s *phrt)
{
  dq_entry_t *pent;
  irqstate_t flags;

  /* Disable MTM2-Ch0 */

  putreg32(0, MT2OPR);

  hrt_queue_refresh();

  flags = spin_lock_irqsave();

  /* add phrt to hrt_timer_queue */

  for (pent = hrt_timer_queue.head; pent; pent = dq_next(pent))
    {
      struct hrt_s *tmp = container_of(pent, struct hrt_s, ent);
      if (tmp->usec >= phrt->usec)
        {
          break;
        }
    }

  if (pent)
    {
      dq_addbefore(pent, &phrt->ent, &hrt_timer_queue);
    }
  else
    {
      dq_addlast(&phrt->ent, &hrt_timer_queue);
    }

  spin_unlock_irqrestore(flags);

  hrt_usleep_setup();
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  up_proftimerisr
 ****************************************************************************/

#ifdef CONFIG_PROFILE
int up_proftimerisr(int irq, uint32_t *regs, FAR void *arg)
{
  putreg32(1 << 1, MT30STS);
  if (profile_en)
    {
      if (profile_ptr != CONFIG_PROFILE_SAMPLES)
        {
          DEBUGASSERT(current_regs);
          profile_data[profile_ptr++] = current_regs[REG_R15];
        }
      else
        {
          profile_en = 0;
          tmrinfo("PROFILING DONE\n");
        }
    }

  return 0;
}
#endif /* CONFIG_PROFILE */

/****************************************************************************
 * Name:  up_timerisr
 ****************************************************************************/

int up_timerisr(int irq, uint32_t *regs, FAR void *arg)
{
  /* Process timer interrupt */

#ifdef CONFIG_DVFS
  lc823450_dvfs_tick_callback();
#endif

#ifdef CONFIG_LC823450_MTM0_TICK
  /* Clear the interrupt (BEVT) */

  putreg32(1 << 1, MT00STS);
#endif

  nxsched_process_timer();

#ifdef CONFIG_LCA_SOUNDSKIP_CHECK
  extern void lca_check_soundskip(void);
  lca_check_soundskip();
#endif

#ifdef CHECK_INTERVAL
  _timer_val = !_timer_val;
  lc823450_gpio_write(TIMER_PIN, _timer_val);
#endif
#ifdef CONFIG_HSUART
  hsuart_wdtimer();
#endif /* CONFIG_HSUART */

  return 0;
}

/****************************************************************************
 * Name: up_hrttimer_usleep
 ****************************************************************************/

#ifdef CONFIG_HRT_TIMER
int up_hrttimer_usleep(unsigned int usec)
{
  struct hrt_s hrt;

  nxsem_init(&hrt.sem, 0, 0);
  hrt.usec = usec;

  hrt_usleep_add(&hrt);
  nxsem_wait(&hrt.sem);

  return 0;
}
#endif /* CONFIG_HRT_TIMER */

/****************************************************************************
 * Name: up_get_timer_fraction
 ****************************************************************************/

static uint64_t up_get_timer_fraction(void)
{
#ifdef CONFIG_LC823450_MTM0_TICK
  uint32_t regval;
  uint64_t nsec;

  /* read the counter */

  regval  = getreg32(MT00CNT);

  /* check if the timer interrupt is underway */

  if (getreg32(MT00STS) & 0x2 && regval < (MTM_RELOAD / 10))
    {
      return NSEC_PER_TICK;
    }

  nsec = ((uint64_t)regval * NSEC_PER_TICK / MTM_RELOAD);
  return nsec;
#else
  uint32_t cur;
  uint64_t nsec;

  /* read the counter */

  cur  = getreg32(NVIC_SYSTICK_CURRENT);

  /* check if the systick interrupt is pending or active */

  if ((getreg32(0xe000ed04) & (1 << 26) ||
       getreg32(0xe000ed24) & (1 << 11))
      && (SYSTICK_RELOAD - cur) < (SYSTICK_RELOAD / 10))
    {
      return NSEC_PER_TICK;
    }

  nsec = ((uint64_t)(SYSTICK_RELOAD - cur) * NSEC_PER_TICK / SYSTICK_RELOAD);
  return nsec;
#endif
}

/****************************************************************************
 * Name: up_timer_initialize
 ****************************************************************************/

void up_timer_initialize(void)
{
#ifdef CHECK_INTERVAL
  lc823450_gpio_config(TIMER_PIN |
                       GPIO_MODE_OUTPUT |
                       GPIO_VALUE_ONE);
#endif

#ifdef CONFIG_HRT_TIMER
  modifyreg32(MCLKCNTEXT1, 0x0, MCLKCNTEXT1_MTM2_CLKEN);
  modifyreg32(MCLKCNTEXT1, 0x0, MCLKCNTEXT1_MTM2C_CLKEN);

  /* MTM2: unreset */

  modifyreg32(MRSTCNTEXT1, 0x0, MRSTCNTEXT1_MTM2_RSTB);

  /* Enable AEVT Interrupt */

  putreg32(1 << 0, MT20TIER);

  /* Set prescaler to (1/10) */

  putreg32(10 - 1, MT20PSCL);

  modifyreg32(MCLKCNTEXT1, MCLKCNTEXT1_MTM2C_CLKEN, 0);
  modifyreg32(MCLKCNTEXT1, MCLKCNTEXT1_MTM2_CLKEN, 0);

  irq_attach(LC823450_IRQ_MTIMER20, (xcpt_t)hrt_interrupt, NULL);
  up_enable_irq(LC823450_IRQ_MTIMER20);

#endif /* CONFIG_HRT_TIMER */
#ifdef CONFIG_PROFILE

  /* MTM3: enable clocking */

  modifyreg32(MCLKCNTEXT1, 0x0, MCLKCNTEXT1_MTM3_CLKEN);
  modifyreg32(MCLKCNTEXT1, 0x0, MCLKCNTEXT1_MTM3C_CLKEN);

  /* MTM3: unreset */

  modifyreg32(MRSTCNTEXT1, 0x0, MRSTCNTEXT1_MTM3_RSTB);

  /* Input clock for the MTM3 is XT1 (i.e. 24M or 20M)
   * then the clock will be set to 1/10 by the internal divider
   * To implement 10ms timer, ADT=0, BDT=MTM_RELOAD
   */

  putreg32(0, MT30A);                       /* AEVT counter */
  putreg32((XT1OSC_CLK / 1010) - 1, MT30B); /* BEVT counter */

  /* Clear the counter by BEVT */

  putreg32(0x80, MT30CTL);

  /* Set prescaler to 9 : (1/10) */

  putreg32(9, MT30PSCL);

  /* Enable BEVT Interrupt */

  putreg32(1 << 1, MT30TIER);

  /* Enable MTM3-Ch0 */

  putreg32(1, MT30OPR);

  /* Attach the timer interrupt vector */

  irq_attach(LC823450_IRQ_MTIMER30, (xcpt_t)up_proftimerisr, NULL);

  /* And enable the system timer interrupt */

  up_enable_irq(LC823450_IRQ_MTIMER30);

#endif /* CONFIG_PROFILE */

#ifdef CONFIG_LC823450_MTM0_TICK

  /* MTM0: enable clocking */

  modifyreg32(MCLKCNTEXT1, 0x0, MCLKCNTEXT1_MTM0_CLKEN);
  modifyreg32(MCLKCNTEXT1, 0x0, MCLKCNTEXT1_MTM0C_CLKEN);

  /* MTM0: unreset */

  modifyreg32(MRSTCNTEXT1, 0x0, MRSTCNTEXT1_MTM0_RSTB);

  /* Input clock for the MTM0 is XT1 (i.e. 24M or 20M)
   * then the clock will be set to 1/10 by the internal divider
   * To implement the tick timer, ADT=0, BDT=MTM_RELOAD-1
   */

  putreg32(0, MT00A);              /* AEVT counter */
  putreg32(MTM_RELOAD - 1, MT00B); /* BEVT counter */

  /* Clear the counter by BEVT */

  putreg32(0x80, MT00CTL);

  /* Set prescaler to 9 : (1/10) */

  putreg32(9, MT00PSCL);

  /* Enable BEVT Interrupt */

  putreg32(1 << 1, MT00TIER);

  /* Enable MTM0-Ch0 */

  putreg32(1, MT00OPR);

  /* Attach the timer interrupt vector */

  irq_attach(LC823450_IRQ_MTIMER00, (xcpt_t)up_timerisr, NULL);

  /* And enable the system timer interrupt */

  up_enable_irq(LC823450_IRQ_MTIMER00);
#else
  uint32_t regval;

  /* Set the SysTick interrupt to the default priority */

  regval = getreg32(NVIC_SYSH12_15_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR15_MASK;
  regval |= (NVIC_SYSH_PRIORITY_DEFAULT << NVIC_SYSH_PRIORITY_PR15_SHIFT);
  putreg32(regval, NVIC_SYSH12_15_PRIORITY);

  /* Make sure that the SYSTICK clock source is set correctly */

#if 0 /* Does not work.  Comes up with HCLK source and I can't change it */
  regval = getreg32(NVIC_SYSTICK_CTRL);
#if CONFIG_LC823450_SYSTICK_HCLKd8
  regval &= ~NVIC_SYSTICK_CTRL_CLKSOURCE;
#else
  regval |= NVIC_SYSTICK_CTRL_CLKSOURCE;
#endif
  putreg32(regval, NVIC_SYSTICK_CTRL);
#endif

  /* Configure SysTick to interrupt at the requested rate */

  putreg32(SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);

  /* Attach the timer interrupt vector */

  irq_attach(LC823450_IRQ_SYSTICK, (xcpt_t)up_timerisr, NULL);

  /* Enable SysTick interrupts */

  putreg32((NVIC_SYSTICK_CTRL_CLKSOURCE |
            NVIC_SYSTICK_CTRL_TICKINT |
            NVIC_SYSTICK_CTRL_ENABLE),
            NVIC_SYSTICK_CTRL);

  /* And enable the timer interrupt */

  up_enable_irq(LC823450_IRQ_SYSTICK);
#endif

#ifdef CONFIG_DVFS
  /* attach timer interrupt handler */

  irq_attach(LC823450_IRQ_MTIMER01, (xcpt_t)lc823450_dvfs_oneshot, NULL);

  /* enable MTM0-ch1 */

  up_enable_irq(LC823450_IRQ_MTIMER01);
#endif
}

/****************************************************************************
 * Name: lc823450_mtm_start_oneshot
 * NOTE: Assumption: MTM0-ch0 is running (i.e. tick timer)
 ****************************************************************************/

#ifdef CONFIG_DVFS
void lc823450_mtm_start_oneshot(int msec)
{
  uint32_t r = MTM_RELOAD; /* 10ms */

  r /= 10;   /* 1ms */
  r *= msec;

  putreg32(0, MT01A);     /* AEVT counter */
  putreg32(r - 1, MT01B); /* BEVT counter */

  /* Clear the counter by BEVT */

  putreg32(0x80, MT01CTL);

  /* Set prescaler to 9 : (1/10) */

  putreg32(9, MT01PSCL);

  /* Enable BEVT Interrupt */

  putreg32(1 << 1, MT01TIER);

  /* Enable MTM0-ch1 */

  modifyreg32(MT0OPR, 0, 1 << 1);
}
#endif

/****************************************************************************
 * Name: lc823450_mtm_stop_oneshot
 ****************************************************************************/

#ifdef CONFIG_DVFS
void lc823450_mtm_stop_oneshot(void)
{
  /* Clear the interrupt (BEVT) */

  putreg32(1 << 1, MT01STS);

  /* Disable MTM0-ch1 */

  modifyreg32(MT0OPR, 1 << 1, 0);
}
#endif

/****************************************************************************
 * Name: up_rtc_gettime
 *
 * Description:
 *   This function is used in clock_gettime() to obtain high resolution time.
 *
 ****************************************************************************/

int up_rtc_gettime(FAR struct timespec *tp)
{
  uint64_t secs;
  uint64_t nsecs;
  uint64_t elapsed; /* in nsec */
  irqstate_t   flags;
  uint64_t f;

  flags = spin_lock_irqsave();

  /* Get the elapsed time */

  elapsed = NSEC_PER_TICK * (uint64_t)g_system_timer;

  /* Add the tiemr fraction in nanoseconds */

  f = up_get_timer_fraction();
  elapsed += f;

  spin_unlock_irqrestore(flags);

  tmrinfo("elapsed = %lld \n", elapsed);

  /* Convert the elapsed time in seconds and nanoseconds. */

  secs  = elapsed / NSEC_PER_SEC;
  nsecs = (elapsed - secs * NSEC_PER_SEC);

  /* And return the result to the caller. */

  tp->tv_sec  = (time_t)secs;
  tp->tv_nsec = (long)nsecs;

  tmrinfo("Returning tp=(%d,%d)\n", (int)tp->tv_sec, (int)tp->tv_nsec);
  return OK;
}
