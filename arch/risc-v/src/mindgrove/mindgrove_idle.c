
/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
// #include"
#include <stdio.h>
#include"stdint.h"
#include "riscv_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when there is no other
 *   ready-to-run task.  This is processor idle time and will continue until
 *   some interrupt occurs to cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g., this is where
 *   power management operations might be performed.
 *
 ****************************************************************************/

void up_idle(void)
{
#if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_TIMER_INTS)
  /* If the system is idle and there are no timer interrupts, then process
   * "fake" timer interrupts. Hopefully, something will wake up.
   */

  nxsched_process_timer();
#else

  /* This would be an appropriate place to put some MCU-specific logic to
   * sleep in a reduced power mode until an interrupt occurs to save power
   */
  asm("WFI");
  // printf("Before while 1\n\r");
  // uint64_t mstatus;
  // asm volatile("csrr %0, mstatus" : "=r"(mstatus));
  // mstatus |= (1 << 3);  // Set MIE
  // asm volatile("csrw mstatus, %0" : : "r"(mstatus));
  // uint64_t mtvec;
  // asm volatile("csrr %0, mtvec" : "=r"(mtvec));
  // printf("mtvec: 0x%x\n", mtvec);
  // uint64_t mie;
  // asm volatile("csrr %0, mie" : "=r"(mie));
  // printf("mie: 0x%lx\n", mie);
  // while(1)
  // {

  //   uint32_t* plic_pending = (uint32_t*) 0x0C001001;

  //   if(*plic_pending!=0)
  //   {
  //     printf("Pending register %x\n\r",*plic_pending);
  //   }



  // }

#endif
}
