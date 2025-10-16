/****************************************************************************
 * arch/arm/src/am67/am67_timer.c
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
#include <nuttx/timers/arch_alarm.h>
#include <arm_internal.h>
#include <fcntl.h>
#include <stdint.h>
#include <time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TIMER0_CLOCK_SRC_MUX_ADDR          (0x1081b0u)
#define TIMER0_CLOCK_SRC_HFOSC0_CLKOUT     (0x0u)
#define TIMER0_BASE_ADDR                   (0x2400000u)

#define AM67_DMTIMER1_1MS_TIMER0_VADDR     (0x2400000)
#define AM67_DMTMR1MS_TIDR_OFFSET          (0x0000) /* Identification Register Section */
#define AM67_DMTMR1MS_TIOCP_CFG_OFFSET     (0x0010) /* 1ms Timer OCP Configuration Register Section */
#define AM67_DMTMR1MS_IRQ_EOI_OFFSET       (0x0020) /* 1ms Timer IRQ Wakeup Enable Register */
#define AM67_DMTMR1MS_IRQSTATUS_RAW_OFFSET (0x0024) /* 1ms Timer IRQ Status Register */
#define AM67_DMTMR1MS_IRQSTATUS_OFFSET     (0x0028) /* 1ms Timer IRQ Enable Register */
#define AM67_DMTMR1MS_IRQSTATUS_SET_OFFSET (0x002c) /* 1ms Timer IRQ Enable Register */
#define AM67_DMTMR1MS_IRQSTATUS_CLR_OFFSET (0x0030) /* 1ms Timer IRQ Enable Register */
#define AM67_DMTMR1MS_IRQWAKEEN_OFFSET     (0x0034) /* 1ms Timer IRQ Enable Register */
#define AM67_DMTMR1MS_TCLR_OFFSET          (0x0038) /* 1ms Timer Control Register */
#define AM67_DMTMR1MS_TCRR_OFFSET          (0x003c) /* 1ms Timer Counter Register */
#define AM67_DMTMR1MS_TLDR_OFFSET          (0x0040) /* 1ms Timer Load Register */
#define AM67_DMTMR1MS_TTGR_OFFSET          (0x0044) /* 1ms Timer Trigger Register */
#define AM67_DMTMR1MS_TWPS_OFFSET          (0x0048) /* 1ms Timer Write Posting Bits Register */
#define AM67_DMTMR1MS_TMAR_OFFSET          (0x004c) /* 1ms Timer Match Register */
#define AM67_DMTMR1MS_TCAR1_OFFSET         (0x0050) /* 1ms Timer Capture 1 Register */
#define AM67_DMTMR1MS_TSICR_OFFSET         (0x0054) /* 1ms Timer Synchronous Interface Control Register */
#define AM67_DMTMR1MS_TCAR2_OFFSET         (0x0058) /* 1ms Timer Capture 2 Register */
#define AM67_DMTMR1MS_TPIR_OFFSET          (0x005c) /* 1ms Timer Positive Increment Register */
#define AM67_DMTMR1MS_TNIR_OFFSET          (0x0060) /* 1ms Timer Negative Increment Register */
#define AM67_DMTMR1MS_TCVR_OFFSET          (0x0064) /* 1ms Timer Counter Value Register */
#define AM67_DMTMR1MS_TOCR_OFFSET          (0x0068) /* 1ms Timer Overflow Counter Register */
#define AM67_DMTMR1MS_TOWR_OFFSET          (0x006c) /* 1ms Timer Overflow Interrupts Register */

#define TIMER_OVF_INT_SHIFT                (0x1)
#define TIMER_IRQ_EOI                      (0x20u)
#define TIMER_IRQ_STATUS_RAW               (0x24u)
#define TIMER_IRQ_STATUS                   (0x28u)
#define TIMER_IRQ_INT_ENABLE               (0x2cu)
#define TIMER_IRQ_INT_DISABLE              (0x30u)
#define TIMER_TCLR                         (0x38u)
#define TIMER_TCRR                         (0x3cu)
#define TIMER_TLDR                         (0x40u)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct timer_params_s
{
  /* Input pre-scaler divisor ro apply
   *
   * Must be power of 2 and between 1 and 256 for GP Timer.
   *
   * This field is valid only when underlying timer is DM Timer.
   *
   * This field is not valid when underlying timer is RTI Timer,
   * set to 1 in this case.
   */

  uint32_t input_pre_scaler;

  /* Timer input clock in unit of Hz before pre-scaler
   *
   * System initialization must make any system level muxes, PLLs,
   * power required to input this clock are setup properly.
   *
   * Make sure this value is not 0.
   */

  uint32_t input_clk_hz;

  /* Timer period in units of usecs
   *
   * Internally timer_params_s.input_clk_hz and
   * timer_params_s.input_pre_scaler is used to compute the value to be put
   * inside the timer HW register.
   *
   * When value is 0, period_in_nsec is used instead.
   * When both period_in_usec and period_in_nsec are non-zero,
   * period_in_nsec is used.
   */

  uint32_t period_in_usec;

  /* Timer period in units of nsecs
   *
   * Internally timer_params_s.input_clk_hz and
   * timer_params_s.input_pre_scaler is used to compute the value to be put
   * inside the timer HW register.
   *
   * When value is 0, period_in_nsec is used instead.
   * When both period_in_usec and period_in_nsec are non-zero,
   * period_in_nsec is used.
   */

  uint32_t period_in_nsec;

  /* 0: continuous mode of operation
   * 1: oneshot mode of operation
   *
   * Not supported for RTI timer, always set to 0 in this case.
   */

  uint32_t oneshot_mode;

  /* 0: do not enable timer overflow interrupt
   * 1: enable timer overflow interrupt
   */

  uint32_t enable_overflow_int;

  /* 0: do not enable DMA trigger from timer
   * 1: enable DMA trigger from timer
   */

  uint32_t enable_dma_trigger;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void am67_timer_start(uint32_t base_addr);
static void am67_timer_stop(uint32_t base_addr);
static uint32_t am67_timer_get_count(uint32_t base_addr);
static void am67_timer_clear_overflow_int(uint32_t base_addr);
static void am67_timer_params_init(struct timer_params_s *params);
static void am67_timer_setup(uint32_t base_addr,
                             struct timer_params_s *params);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am67_timer_start
 *
 * Description:
 *   Start the timer by setting the enable bit in the timer control register.
 *
 ****************************************************************************/

static void am67_timer_start(uint32_t base_addr)
{
  uint32_t value;
  uint32_t addr = base_addr + TIMER_TCLR;

  value = getreg32(addr);
  putreg32(value | (0x1u << 0), addr);
}

/****************************************************************************
 * Name: am67_timer_stop
 *
 * Description:
 *   Stop the timer by clearing the enable bit in the timer control register.
 *
 ****************************************************************************/

static void am67_timer_stop(uint32_t base_addr)
{
  uint32_t addr = base_addr + TIMER_TCLR;
  uint32_t value;

  value = getreg32(addr);
  putreg32(value & ~(0x1u << 0), addr);
}

/****************************************************************************
 * Name: am67_timer_get_count
 *
 * Description:
 *   Read and return the current timer count value from the timer counter
 *   register.
 *
 ****************************************************************************/

static uint32_t am67_timer_get_count(uint32_t base_addr)
{
  return getreg32(base_addr + TIMER_TCRR);
}

/****************************************************************************
 * Name: am67_timer_clear_overflow_int
 *
 * Description:
 *   Clear the timer overflow interrupt by writing to the interrupt status
 *   register and verify the interrupt is cleared.
 *
 ****************************************************************************/

static void am67_timer_clear_overflow_int(uint32_t base_addr)
{
  uint32_t addr;
  uint32_t value = (0x1u << TIMER_OVF_INT_SHIFT);

  /* Clear status for overflow interrupt. */

  addr = base_addr + TIMER_IRQ_STATUS;
  putreg32(value, addr);

  /* Read back and make sure interrupt was indeed cleared,
   * if not clear it again.
   */

  if ((bool)(getreg32(addr) & value) == true)
    {
      putreg32(value, addr);
    }
}

/****************************************************************************
 * Name: am67_timer_params_init
 *
 * Description:
 *   Initialize timer parameters with default values including prescaler,
 *   clock frequency, period, and operational mode settings.
 *
 ****************************************************************************/

static void am67_timer_params_init(struct timer_params_s *params)
{
  params->input_pre_scaler = 1;
  params->input_clk_hz = 25 * 1000000;
  params->period_in_usec = 10000;
  params->period_in_nsec = 0;
  params->oneshot_mode = 0;
  params->enable_overflow_int = 1;
  params->enable_dma_trigger = 0;
}

/****************************************************************************
 * Name: am67_timer_setup
 *
 * Description:
 *   Configure timer parameters including period, mode, prescaler, and
 *   interrupts based on the provided timer parameters structure.
 *
 ****************************************************************************/

static void am67_timer_setup(uint32_t base_addr,
                             struct timer_params_s *params)
{
  uint32_t ctrl_val;
  uint32_t count_val;
  uint32_t reload_val;
  uint64_t time_in_nsec;
  uint64_t input_clk_hz;
  uint64_t timer_cycles;

  /* Stop timer and clear pending interrupts. */

  am67_timer_stop(base_addr);
  am67_timer_clear_overflow_int(base_addr);

  time_in_nsec = (uint64_t)params->period_in_nsec;
  if (time_in_nsec == 0U)
    {
      time_in_nsec = params->period_in_usec * 1000U;
    }

  input_clk_hz = params->input_clk_hz / params->input_pre_scaler;
  timer_cycles = (input_clk_hz * time_in_nsec) / 1000000000U;

  /* If timerCycles > 32b then we cannot give accurate timing. */

  /* Calculate count and reload value register value. */

  count_val = 0xffffffffu - (uint32_t)timer_cycles - 1u;

  /* Keep reload value as 0, later if is auto-reload is enabled,
   * it will be set a value > 0.
   */

  reload_val = 0;

  /* Calculate control register value, keep timer disabled. */

  ctrl_val = 0;
  if (params->input_pre_scaler > 1u)
    {
      uint32_t pre_scale_val;

      for (pre_scale_val = 8; pre_scale_val >= 1u; pre_scale_val--)
        {
          if ((params->input_pre_scaler & (0x1u << pre_scale_val)) != 0u)
            {
              break;
            }
        }

      /* Enable pre-scaler. */

      ctrl_val |= (0x1u << 5);

      /* Set pre-scaler value. */

      ctrl_val |= (((pre_scale_val - 1U) & 0x7u) << 2);
    }

  if (params->oneshot_mode == 0u)
    {
      /* Auto-reload timer. */

      ctrl_val |= (0x1u << 1);
      reload_val = count_val;
    }

  /* Set timer control value. */

  putreg32(ctrl_val, base_addr + TIMER_TCLR);

  /* Set timer count value. */

  putreg32(count_val, base_addr + TIMER_TCRR);

  /* Set reload value. */

  putreg32(reload_val, base_addr + TIMER_TLDR);

  /* Enable/disable interrupts. */

  if ((bool)params->enable_overflow_int == true)
    {
      /* Enable interrupt. */

      putreg32((0x1u << TIMER_OVF_INT_SHIFT),
               base_addr + TIMER_IRQ_INT_ENABLE);
    }
  else
    {
      /* Disable interrupt. */

      putreg32((0x1u << TIMER_OVF_INT_SHIFT),
               base_addr + TIMER_IRQ_INT_DISABLE);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timer_tick_isr
 *
 * Description:
 *   Timer tick interrupt service routine that clears the timer overflow
 *   interrupt and processes system timer events.
 *
 ****************************************************************************/

__attribute__((section(".tick_timer")))
int timer_tick_isr(int irq, void *context, void *arg)
{
  am67_timer_clear_overflow_int(AM67_DMTIMER1_1MS_TIMER0_VADDR);

  nxsched_process_timer();
  return 0;
}

/****************************************************************************
 * Name: up_timer_gettime
 *
 * Description:
 *   Return the elapsed time since power-up (or, more correctly, since
 *   the architecture-specific timer was initialized).  This function is
 *   functionally equivalent to:
 *
 *      int clock_gettime(clockid_t clockid, FAR struct timespec *ts);
 *
 *   when clockid is CLOCK_MONOTONIC.
 *
 *   This function provides the basis for reporting the current time and
 *   also is used to eliminate error build-up from small errors in interval
 *   time calculations.
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Provides the location in which to return the up-time.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   Called from the normal tasking context.  The implementation must
 *   provide whatever mutual exclusion is necessary for correct operation.
 *   This can include disabling interrupts in order to assure atomic register
 *   operations.
 *
 ****************************************************************************/

int up_timer_gettime(struct timespec *ts)
{
  return am67_timer_get_count(AM67_DMTIMER1_1MS_TIMER0_VADDR);
}

/****************************************************************************
 * Name: up_timer_start
 *
 * Description:
 *   Start the timer by setting the enable bit in the timer control register.
 *
 ****************************************************************************/

int up_timer_start(struct timespec const *ts)
{
  am67_timer_start(AM67_DMTIMER1_1MS_TIMER0_VADDR);
  return 0;
}

/****************************************************************************
 * Name: up_timer_cancel
 *
 * Description:
 *   Stop the timer by clearing the enable bit in the timer control register.
 *
 ****************************************************************************/

int up_timer_cancel(struct timespec *ts)
{
  am67_timer_stop(AM67_DMTIMER1_1MS_TIMER0_VADDR);
  return 0;
}

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer hardware.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  struct timer_params_s params;

  am67_timer_params_init(&params);

  up_disable_irq(CSLR_R5FSS0_CORE0_INTR_TIMER0_INTR_PEND_0);
  irq_attach(CSLR_R5FSS0_CORE0_INTR_TIMER0_INTR_PEND_0,
             timer_tick_isr, NULL);

  am67_timer_stop(AM67_DMTIMER1_1MS_TIMER0_VADDR);
  am67_timer_setup(AM67_DMTIMER1_1MS_TIMER0_VADDR, &params);
  am67_timer_start(AM67_DMTIMER1_1MS_TIMER0_VADDR);

  up_enable_irq(CSLR_R5FSS0_CORE0_INTR_TIMER0_INTR_PEND_0);
}
