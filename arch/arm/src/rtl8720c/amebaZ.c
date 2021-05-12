#include <nuttx/config.h>
#include <nuttx/timers/arch_timer.h>
#include "systick.h"
#include "hal_syson.h"
#include "hal_wdt.h"

#ifdef CONFIG_ARCH_CHIP_AMEBAZ

void up_timer_initialize(void)
{
  up_timer_set_lowerhalf(systick_initialize(true, 100000000, -1));
}

void ameba_reset(int status)
{
  hal_sys_set_fast_boot(0, 0);
  hal_misc_rst_by_wdt();
}

#endif /* CONFIG_ARCH_CHIP_AMEBAZ */
