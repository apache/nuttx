/****************************************************************************
 * arch/risc-v/src/allwinner-d1/allwinner_d1_start.c
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/barriers.h>

#include <stdint.h>
#include <nuttx/arch.h>
#include <nuttx/init.h>
#include <nuttx/serial/uart_16550.h>

#include "chip.h"
#include "riscv_internal.h"
#include "hardware/allwinner_d1_wdt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) up_putc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Extern Function Declarations
 ****************************************************************************/

extern void __trap_vec(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: allwinner_d1_disable_inherited_watchdog
 ****************************************************************************/

static void allwinner_d1_disable_inherited_watchdog(void)
{
  /* U-Boot may leave the D1 RISC-V watchdog running with a 16-second
   * timeout.  The key with all mode bits clear stops that inherited timer.
   */

  putreg32(ALLWINNER_D1_RISCV_WDT_DISABLE_KEY,
           ALLWINNER_D1_RISCV_WDT_MODE);
  UP_DSB();
}

/****************************************************************************
 * Name: allwinner_d1_clear_bss
 ****************************************************************************/

static void allwinner_d1_clear_bss(void)
{
  uint32_t *dest;

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }
}

/****************************************************************************
 * Name: allwinner_d1_lowsetup
 ****************************************************************************/

static void allwinner_d1_lowsetup(void)
{
  uint32_t regval;

  /* Enable the UART0 APB gate and deassert its bus reset. */

  modifyreg32(ALLWINNER_D1_UART_BGR, 0, (1u << 0) | (1u << 16));

  /* Select UART0 function 6 on PB8 (TX) and PB9 (RX). */

  regval  = getreg32(ALLWINNER_D1_PB_CFG1);
  regval &= ~0xffu;
  regval |= 0x66u;
  putreg32(regval, ALLWINNER_D1_PB_CFG1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: allwinner_d1_start
 ****************************************************************************/

void allwinner_d1_start(int hartid, uintptr_t dtb)
{
  (void)dtb; /* The initial FLAT port does not consume a device tree. */

  allwinner_d1_disable_inherited_watchdog();

  if (hartid != 0)
    {
      for (; ; )
        {
          asm volatile ("wfi");
        }
    }

  allwinner_d1_clear_bss();

#ifdef CONFIG_RISCV_PERCPU_SCRATCH
  riscv_percpu_add_hart(hartid);
#endif

  WRITE_CSR(CSR_SATP, 0);
  asm volatile ("sfence.vma" ::: "memory");
  WRITE_CSR(CSR_STVEC, (uintptr_t)__trap_vec);

  riscv_fpuconfig();
  allwinner_d1_lowsetup();

#ifdef USE_EARLYSERIALINIT
  riscv_earlyserialinit();
#endif

  showprogress('A');
  nx_start();

  for (; ; )
    {
      asm volatile ("wfi");
    }
}

/****************************************************************************
 * Name: riscv_earlyserialinit
 ****************************************************************************/

void riscv_earlyserialinit(void)
{
#ifdef CONFIG_16550_UART
  u16550_earlyserialinit();
#endif
}

/****************************************************************************
 * Name: riscv_serialinit
 ****************************************************************************/

void riscv_serialinit(void)
{
#ifdef CONFIG_16550_UART
  u16550_serialinit();
#endif
}
