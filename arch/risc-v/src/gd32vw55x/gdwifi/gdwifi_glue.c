/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gdwifi/gdwifi_glue.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform glue for the GD32VW55x SDK Wi-Fi stack running on NuttX:
 * console output for dbg_print, the Wi-Fi interrupt attach, and the SNTP
 * system-time hook.
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
#include <time.h>
#include <debug.h>
#include <syslog.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <malloc.h>

#include "riscv_internal.h"
#include "hardware/gd32vw55x_eclic.h"
#include "hardware/gd32vw55x_rcu.h"
#include "chip.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Console output used by the SDK debug_print machinery.  The SDK log_uart
 * driver is not compiled; route everything to the NuttX console.
 */

void log_uart_putc_noint(char c)
{
  up_putc(c);
}

void uart_putc_noint(uint32_t uartx, char c)
{
  up_putc(c);
}

/* The supplicant tunes stdout buffering at init; stdio streams are not
 * usable from the SDK service tasks - ignore it.
 */

int __wrap_setvbuf(void *stream, char *buf, int mode, unsigned int size)
{
  return 0;
}

/* SDK stack-depth helper referenced by the wrapper/debug code */

int32_t xGetCurrentTaskStackDepth(unsigned long sp)
{
  return 1024;
}

/****************************************************************************
 * Wi-Fi interrupt wiring
 *
 * The SDK gd32vw55x_it.c provides the handler bodies (WIFI_INT_IRQHandler
 * etc.) that call into the prebuilt MAC library.  Attach them to the NuttX
 * IRQ table; the SDK platform code enables the sources in the ECLIC.
 ****************************************************************************/

extern void WIFI_INT_IRQHandler(void);
extern void WIFI_INTGEN_IRQHandler(void);
extern void WIFI_PROT_IRQHandler(void);
extern void WIFI_RX_IRQHandler(void);
extern void WIFI_TX_IRQHandler(void);
extern void LA_IRQHandler(void);
extern void WIFI_WKUP_IRQHandler(void);

static int gdwifi_isr(int irq, void *context, void *arg)
{
  ((void (*)(void))arg)();
  return 0;
}

void gdwifi_irq_attach(void)
{
  irq_attach(GD32VW55X_IRQ_WIFI_INT, gdwifi_isr, WIFI_INT_IRQHandler);
  irq_attach(GD32VW55X_IRQ_WIFI_INTGEN, gdwifi_isr, WIFI_INTGEN_IRQHandler);
  irq_attach(GD32VW55X_IRQ_WIFI_PROT, gdwifi_isr, WIFI_PROT_IRQHandler);
  irq_attach(GD32VW55X_IRQ_WIFI_RX, gdwifi_isr, WIFI_RX_IRQHandler);
  irq_attach(GD32VW55X_IRQ_WIFI_TX, gdwifi_isr, WIFI_TX_IRQHandler);
  irq_attach(GD32VW55X_IRQ_LA, gdwifi_isr, LA_IRQHandler);
  irq_attach(GD32VW55X_IRQ_WIFI_WKUP, gdwifi_isr, WIFI_WKUP_IRQHandler);
}

/****************************************************************************
 * SNTP system time hook
 *
 * lwipopts.h maps SNTP_SET_SYSTEM_TIME() to sntp_set_system_time(); the
 * SDK sntp_api.c is not compiled, so provide it here: set CLOCK_REALTIME,
 * which also writes through to the hardware RTC (up_rtc_settime).
 ****************************************************************************/

void sntp_set_system_time(uint32_t sec)
{
  struct timespec ts;

  ts.tv_sec  = sec;
  ts.tv_nsec = 0;
  clock_settime(CLOCK_REALTIME, &ts);

  ninfo("SNTP: system time set to %lu\n", (unsigned long)sec);
}

/* Poll interval hook referenced by lwipopts.h (SNTP_UPDATE_DELAY) */

uint32_t sntp_get_update_intv(void)
{
  return 86400 * 1000;  /* Once a day, in ms */
}

/* The SDK vendors its own clock global; NuttX runs the chip at 160 MHz */

uint32_t SystemCoreClock = 160000000;

/* Called by deep_sleep_exit() to restore clocks after LPDS wakeup */

void system_clock_config(void)
{
  extern void gd32vw55x_clockconfig(void);
  gd32vw55x_clockconfig();
}

/****************************************************************************
 * Name: gdwifi_eclic_normalize
 *
 * Description:
 *   The SDK enables its interrupt sources through the NMSIS inline eclic
 *   helpers, programming per-source ECLIC levels that could hardware-
 *   preempt NuttX handlers.  Flatten every source back to the kernel
 *   level after the Wi-Fi stack has set up its interrupts.
 ****************************************************************************/

/* Radio interrupt sources keep the higher ECLIC level programmed by the
 * SDK: the wrapper's sys_enter_critical() masks via the MTH threshold,
 * mirroring the FreeRTOS configMAX_SYSCALL_INTERRUPT_PRIORITY model.
 */

/****************************************************************************
 * Name: gdwifi_start
 *
 * Description:
 *   Platform bring-up subset of the SDK platform_init() (skipping the
 *   pieces NuttX already owns: clocks, ECLIC, console, tick) followed by
 *   the Wi-Fi stack start.  Called once from the wifi application.
 ****************************************************************************/

extern void sys_os_init(void);
extern void systick_init(void);
extern void rom_init(void);
extern void dma_config(void);
extern void sysctrl_init(void);
extern void rf_power_on(void);
extern int  wifi_power_on(void);
extern void wifi_power_off(void);
extern void raw_flash_init(void);
extern int  nvds_flash_internal_init(void);
extern int  wifi_init(void);
extern void sys_wakelock_acquire(uint32_t lock_id);

#define GDWIFI_LOCK_ID_WLAN  0

int gdwifi_start(void)
{
  static bool started;

  if (started)
    {
      return 0;
    }

#define GDWIFI_STEP(name, call) \
  do { syslog(LOG_INFO, "gdwifi: " name "\n"); call; } while (0)

  GDWIFI_STEP("sys_os_init", sys_os_init());
  GDWIFI_STEP("systick_init", systick_init());

  /* Peripheral clocks the SDK platform_init() would have enabled before
   * touching TRNG/CRC/PMU registers (unclocked AHB access bus-hangs).
   */

  modifyreg32(GD32VW55X_RCU_APB1EN, 0, RCU_APB1EN_PMUEN);
  modifyreg32(GD32VW55X_RCU_AHB2EN, 0, RCU_AHB2EN_TRNGEN);
  modifyreg32(GD32VW55X_RCU_AHB1EN, 0, RCU_AHB1EN_CRCEN);

  GDWIFI_STEP("rom_init", rom_init());
  GDWIFI_STEP("dma_config", dma_config());
  GDWIFI_STEP("sysctrl_init", sysctrl_init());
  GDWIFI_STEP("rf_power_on", rf_power_on());

  /* Power the Wi-Fi PMU domain before any Wi-Fi interrupt can fire: a
   * spurious WIFI_INTGEN with the domain off spins forever inside
   * wifi_lpds_exit() in interrupt context.
   */

  /* Cycle the Wi-Fi power domain: a JTAG/system reset does not clear the
   * radio PMU state left by a previous run, and the MAC init asserts on
   * dirty registers.
   */

  GDWIFI_STEP("wifi_power_off", wifi_power_off());
  GDWIFI_STEP("wifi_power_on", wifi_power_on());
  GDWIFI_STEP("raw_flash_init", raw_flash_init());

  if (nvds_flash_internal_init() != 0)
    {
      nerr("nvds flash init failed\n");
    }

  GDWIFI_STEP("irq_attach", gdwifi_irq_attach());

  /* Hold the WLAN wakelock for good: the LPDS/doze path of the vendor
   * firmware spins forever in wifi_lpds_exit() waiting for a PMU wake-up
   * this port does not implement yet (PM phase).  With the lock held the
   * MAC never enters low-power, so that path is never taken.
   */

  sys_wakelock_acquire(GDWIFI_LOCK_ID_WLAN);

  syslog(LOG_INFO, "gdwifi: wifi_init\n");
  if (wifi_init() != 0)
    {
      nerr("wifi_init failed\n");
      return -1;
    }

  syslog(LOG_INFO, "gdwifi: wifi_init done\n");

  started = true;
  return 0;
}
