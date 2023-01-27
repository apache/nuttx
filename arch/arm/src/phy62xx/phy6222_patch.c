#include "types.h"
#include "rom_sym_def.h"
#include <arch/irq.h>
#include "mcu_phy_bumbee.h"

extern int clear_timer_int(AP_TIM_TypeDef* TIMx);
extern void clear_timer(AP_TIM_TypeDef* TIMx);
extern void LL_evt_schedule(void);

#ifndef CONFIG_PHY6222_SDK
void TIM1_IRQHandler1(void)
{
  /*  HAL_ENTER_CRITICAL_SECTION() */

  if (AP_TIM1->status & 0x1)
    {
      clear_timer_int(AP_TIM1);
      clear_timer(AP_TIM1);
      LL_evt_schedule();
    }

  /* HAL_EXIT_CRITICAL_SECTION(); */
}
#endif
