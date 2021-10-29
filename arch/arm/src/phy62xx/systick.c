
#include "rom_sym_def.h"
#include "timer.h"
#include "error.h"
#include "clock.h"
#include "pwrmgr.h"
#include "nvic.h"
#include "arm_arch.h"
#include "jump_function.h"
#include <arch/irq.h>

static int systic_timerisr(int irq, uint32_t *regs, FAR void *arg)
{
  /* Process timer interrupt */

  nxsched_process_timer();
  return 0;
}
void up_timer_initialize(void)
{

  irq_attach(PHY62XX_IRQ_SYSTICK, (xcpt_t)systic_timerisr, NULL);

  putreg32((sysclk_get_clk()/100-1), ARMV6M_SYSTICK_RVR);//10ms tick
  putreg32((SYSTICK_CSR_TICKINT | SYSTICK_CSR_ENABLE | SYSTICK_CSR_CLKSOURCE), ARMV6M_SYSTICK_CSR);

  /* And enable the timer interrupt */

  up_enable_irq(PHY62XX_IRQ_SYSTICK);
}
