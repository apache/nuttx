#include "types.h"
#include <arch/irq.h>
#include "mcu_phy_bumbee.h"

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
