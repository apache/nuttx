/****************************************************************************
 * arch/risc-v/src/allwinner-d1/allwinner_d1_timerisr.c
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/debug.h>
#include <nuttx/irq.h>

#include "chip.h"
#include "riscv_internal.h"
#include "hardware/allwinner_d1_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALLWINNER_D1_TICK_TIMER       1
#define ALLWINNER_D1_TIMER_FREQUENCY  24000000ul
#define ALLWINNER_D1_TIMER_COUNTS_PER_TICK \
  (ALLWINNER_D1_TIMER_FREQUENCY / TICK_PER_SEC)
#define ALLWINNER_D1_TIMER_INTERVAL_VALUE \
  (ALLWINNER_D1_TIMER_COUNTS_PER_TICK - 1)

#if (ALLWINNER_D1_TIMER_FREQUENCY % TICK_PER_SEC) != 0
#  error "The D1 timer requires an integer number of counts per tick"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: allwinner_d1_timerisr
 ****************************************************************************/

static int allwinner_d1_timerisr(int irq, void *context, void *arg)
{
  /* The timer interrupt status is write-one-to-clear.  Clear the peripheral
   * before processing the scheduler tick; the PLIC dispatcher completes the
   * external interrupt after this handler returns.
   */

  putreg32(ALLWINNER_D1_TIMER_IRQ(ALLWINNER_D1_TICK_TIMER),
           ALLWINNER_D1_TIMER_IRQ_STATUS);
  nxsched_process_timer();
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t regval;
  int ret;

  /* Channel 1 is the scheduler timer used by the proven D1 RT-Smart BSP.
   * Select the 24 MHz oscillator, divide by one, and use periodic mode.
   */

  regval = getreg32(ALLWINNER_D1_TIMER_CTRL(ALLWINNER_D1_TICK_TIMER));
  regval &= ~(ALLWINNER_D1_TIMER_CTRL_ENABLE |
              ALLWINNER_D1_TIMER_CTRL_RELOAD |
              ALLWINNER_D1_TIMER_CTRL_CLK_SRC_MASK |
              ALLWINNER_D1_TIMER_CTRL_PRES_MASK |
              ALLWINNER_D1_TIMER_CTRL_ONESHOT);
  regval |= ALLWINNER_D1_TIMER_CTRL_CLK_OSC24M |
            ALLWINNER_D1_TIMER_CTRL_PRES_DIV1;
  putreg32(regval,
           ALLWINNER_D1_TIMER_CTRL(ALLWINNER_D1_TICK_TIMER));

  /* The down-counter includes zero, so N input clocks require N - 1 in the
   * interval register.
   */

  putreg32(ALLWINNER_D1_TIMER_INTERVAL_VALUE,
           ALLWINNER_D1_TIMER_INTERVAL(ALLWINNER_D1_TICK_TIMER));

  /* Clear a stale channel interrupt before attaching and enabling it. */

  putreg32(ALLWINNER_D1_TIMER_IRQ(ALLWINNER_D1_TICK_TIMER),
           ALLWINNER_D1_TIMER_IRQ_STATUS);

  ret = irq_attach(ALLWINNER_D1_IRQ_TIMER1,
                   allwinner_d1_timerisr, NULL);
  DEBUGASSERT(ret >= 0);
  if (ret < 0)
    {
      return;
    }

  /* This port no longer uses the OpenSBI supervisor timer interrupt. */

  up_disable_irq(RISCV_IRQ_STIMER);
  up_enable_irq(ALLWINNER_D1_IRQ_TIMER1);

  modifyreg32(ALLWINNER_D1_TIMER_IRQ_EN, 0,
              ALLWINNER_D1_TIMER_IRQ(ALLWINNER_D1_TICK_TIMER));

  /* Reload the interval and start the channel in periodic mode. */

  putreg32(regval | ALLWINNER_D1_TIMER_CTRL_RELOAD |
           ALLWINNER_D1_TIMER_CTRL_ENABLE,
           ALLWINNER_D1_TIMER_CTRL(ALLWINNER_D1_TICK_TIMER));
}
