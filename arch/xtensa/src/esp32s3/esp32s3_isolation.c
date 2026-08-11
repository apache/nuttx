/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_isolation.c
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

/* Permission control that a protected build and a kernel build share: the
 * violation interrupt, and the peripheral permissions.  Everything here
 * concerns the WORLD0 (privileged) / WORLD1 (unprivileged) split and is
 * independent of how the user memory itself is laid out, which is what
 * separates the two build models.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/irq.h>
#include <nuttx/nuttx.h>
#include <nuttx/sched.h>

#ifdef CONFIG_ESP32S3_USERFAULT_ABORT
#include <signal.h>
#include <arch/irq.h>
#include <arch/xtensa/xtensa_corebits.h>
#endif

#include <assert.h>
#include <debug.h>

#include "chip.h"
#include "xtensa.h"
#include "esp_attr.h"
#include "esp_irq.h"
#include "esp32s3_isolation.h"
#include "esp32s3_addrenv.h"
#include "esp32s3_pms.h"
#include "esp32s3_spiram.h"
#include "esp32s3_wcl.h"
#include "hardware/esp32s3_rom_layout.h"
#include "hardware/esp32s3_sensitive.h"
#include "hardware/esp32s3_soc.h"

#include "soc/extmem_reg.h"

#ifdef CONFIG_ESP32S3_USERFAULT_ABORT
#include "sched/sched.h"
#include "signal/signal.h"
#endif

#ifndef CONFIG_BUILD_FLAT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
/* The WORLD1 vector table is one Xtensa vector table: 1 KB. */

#  define WORLD1_VECTORS_SIZE  0x400
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL

/* The WORLD1 vector table (esp32s3_world1_vectors.S), which the linker
 * script places at the 1 KB alignment a vector base requires.
 */

extern uint8_t _world1_vectors[];

/* The end of the kernel's instruction memory, and so the line that divides
 * Internal SRAM1 between the instruction and the data bus.  The linker
 * script aligns it to the 256 bytes a split line needs.
 */

extern uint8_t _iram_end[];

/* Vectors in the kernel's own table.  Fetching one of these switches the CPU
 * to WORLD0 once it is registered as a World Controller entry address.
 */

extern void _user_exception_vector(void);
extern void _xtensa_level3_vector(void);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_USERFAULT_ABORT

/****************************************************************************
 * Name: pms_clear_violations
 *
 * Description:
 *   Acknowledge and re-arm every PMS violation monitor.  The monitors raise
 *   a level-triggered interrupt, so the latch must be cleared (pulse the CLR
 *   bit) before returning from the ISR or the interrupt re-fires forever.
 *
 ****************************************************************************/

static void IRAM_ATTR pms_clear_violations(void)
{
  /* IRAM0 / DRAM0 / PIF monitors: pulse VIOLATE_CLR (keeping VIOLATE_EN). */

  modifyreg32(SENSITIVE_CORE_0_IRAM0_PMS_MONITOR_1_REG, 0,
              SENSITIVE_CORE_0_IRAM0_PMS_MONITOR_VIOLATE_CLR_M);
  modifyreg32(SENSITIVE_CORE_0_IRAM0_PMS_MONITOR_1_REG,
              SENSITIVE_CORE_0_IRAM0_PMS_MONITOR_VIOLATE_CLR_M, 0);

  modifyreg32(SENSITIVE_CORE_0_DRAM0_PMS_MONITOR_1_REG, 0,
              SENSITIVE_CORE_0_DRAM0_PMS_MONITOR_VIOLATE_CLR_M);
  modifyreg32(SENSITIVE_CORE_0_DRAM0_PMS_MONITOR_1_REG,
              SENSITIVE_CORE_0_DRAM0_PMS_MONITOR_VIOLATE_CLR_M, 0);

  modifyreg32(SENSITIVE_CORE_0_PIF_PMS_MONITOR_1_REG, 0,
              SENSITIVE_CORE_0_PIF_PMS_MONITOR_VIOLATE_CLR_M);
  modifyreg32(SENSITIVE_CORE_0_PIF_PMS_MONITOR_1_REG,
              SENSITIVE_CORE_0_PIF_PMS_MONITOR_VIOLATE_CLR_M, 0);

  /* Flash instruction/data cache reject monitors. */

  modifyreg32(EXTMEM_CORE0_ACS_CACHE_INT_CLR_REG, 0,
              EXTMEM_CORE0_IBUS_REJECT_INT_CLR_M |
              EXTMEM_CORE0_DBUS_REJECT_INT_CLR_M);
  modifyreg32(EXTMEM_CORE0_ACS_CACHE_INT_CLR_REG,
              EXTMEM_CORE0_IBUS_REJECT_INT_CLR_M |
              EXTMEM_CORE0_DBUS_REJECT_INT_CLR_M, 0);

  /* The invalid MMU entry monitor.  An access that no entry translates never
   * reaches the PMS, which checks physical addresses, so without this the
   * cache answers it with zeros and nothing is reported.
   */

  modifyreg32(EXTMEM_CACHE_ILG_INT_CLR_REG, 0,
              EXTMEM_MMU_ENTRY_FAULT_INT_CLR_M);
  modifyreg32(EXTMEM_CACHE_ILG_INT_CLR_REG,
              EXTMEM_MMU_ENTRY_FAULT_INT_CLR_M, 0);
}
#endif

/****************************************************************************
 * Name: pms_violation_isr
 *
 * Description:
 *   This is the common PMS interrupt handler. It will be invoked the PMS
 *   detects an access violation.
 *
 * Parameters:
 *   cpuint        - CPU interrupt index
 *   context       - Context data from the ISR
 *   arg           - Opaque pointer to the internal driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int IRAM_ATTR pms_violation_isr(int cpuint, void *context, void *arg)
{
#ifdef CONFIG_ESP32S3_USERFAULT_ABORT
  uint32_t *regs = (uint32_t *)context;
  const char *cause;

  /* Read why before acknowledging, because the clear below drops the latch.
   * The two causes are answered the same way and are worth telling apart in
   * the log:  a PMS violation is a refused translation, an MMU entry fault
   * is an access that was never translated at all.
   */

  cause = (getreg32(EXTMEM_CACHE_ILG_INT_ST_REG) &
           EXTMEM_MMU_ENTRY_FAULT_ST_M) != 0 ? "MMU entry" : "PMS";

  /* Acknowledge and re-arm the monitors first so the level-triggered
   * interrupt does not immediately re-fire while we handle it.
   */

  pms_clear_violations();

  /* An ESP32-S3 PMS permission violation is asynchronous (unlike the precise
   * cache-attribute faults).  If the interruptee was an unprivileged (user)
   * WORLD1 task -- its saved PS carries the User Mode bit -- terminate only
   * that task with SIGSEGV instead of the whole system.  The IRQ dispatch
   * return path applies the up_schedule_sigaction() redirect.
   */

  if (regs != NULL && (regs[REG_PS] & PS_UM) != 0)
    {
      struct tcb_s *tcb = this_task();
      siginfo_t     info;

      _alert("SIGSEGV (%s) task %s: PC=%08x\n",
             cause, get_task_name(tcb), (unsigned)regs[REG_PC]);

      info.si_signo           = SIGSEGV;
      info.si_code            = SI_USER;
      info.si_errno           = 0;
      info.si_value.sival_ptr = NULL;

      nxsig_tcbdispatch(tcb, &info, false);
      return OK;
    }
#endif

  /* Privileged (WORLD0) violation, or abort disabled: not survivable. */

  PANIC();

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_isolation_revoke_peripherals
 *
 * Description:
 *   Refuse World 1 every peripheral.  A user process reaches a device
 *      through the kernel, so it needs none of them directly.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_isolation_revoke_peripherals(void)
{
  /* Revoke User access permission to every peripheral */

  esp32s3_pms_configure_peripheral(PMS_UART1, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_I2C, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_MISC, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_IO_MUX, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_RTC, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_FE, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_FE2, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_GPIO, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_G0SPI_0, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_G0SPI_1, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_UART, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_SYSTIMER, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_TIMERGROUP1, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_TIMERGROUP, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_BB, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_LEDC, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_RMT, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_UHCI0, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_I2C_EXT0, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_BT, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_PWR, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_WIFIMAC, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_RWBT, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_I2S1, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_CAN, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_APB_CTRL, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_SPI_2, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_WORLD_CONTROLLER, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_DIO, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_AD, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_CACHE_CONFIG, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_DMA_COPY, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_INTERRUPT, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_SENSITIVE, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_SYSTEM, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_BT_PWR, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_APB_ADC, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_CRYPTO_DMA, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_CRYPTO_PERI, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_USB_WRAP, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_USB_DEVICE, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_I2S0, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_HINF, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_PWM0, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_BACKUP, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_SLC, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_PCNT, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_SLCHOST, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_UART2, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_PWM1, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_SDIO_HOST, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_I2C_EXT1, PMS_WORLD_1,
                                   PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_SPI_3, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_peripheral(PMS_USB, PMS_WORLD_1, PMS_ACCESS_NONE);
}

#ifdef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Name: esp32s3_isolation_worlds
 *
 * Description:
 *   Give World 1 its own vector table and register the kernel entry points
 *      that return the CPU to World 0.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_isolation_worlds(void)
{
  /* Give each world its own vector table.  The override applies to both
   * worlds at once, so WORLD0 has to be pointed at the kernel table it has
   * been using all along, the one __start() loaded into VECBASE.
   */

  esp32s3_wcl_set_vecbase(PMS_WORLD_0, (uintptr_t)_init_start);
  esp32s3_wcl_set_vecbase(PMS_WORLD_1, (uintptr_t)_world1_vectors);

  /* Fetching one of these kernel vectors is what takes the CPU back to
   * WORLD0.  Only the level 1 and level 3 paths record the interruptee's
   * world on the way in and restore it on the way out, so only those two
   * may be reached from WORLD1; the WORLD1 table handles window spills
   * itself and never leaves the world.
   */

  esp32s3_wcl_set_world0_entry(1, (uintptr_t)_user_exception_vector);
  esp32s3_wcl_set_world0_entry(2, (uintptr_t)_xtensa_level3_vector);
}

/****************************************************************************
 * Name: isolation_enable_interrupts
 *
 * Description:
 *   Arm the permission violation monitors.  The handler itself is registered
 *   later, from up_irqinitialize(); until then a violation is a panic, which
 *   is what an early kernel violation should be anyway.
 *
 ****************************************************************************/

static void isolation_enable_interrupts(void)
{
  modifyreg32(SENSITIVE_CORE_0_IRAM0_PMS_MONITOR_1_REG,
              SENSITIVE_CORE_0_IRAM0_PMS_MONITOR_VIOLATE_CLR_M,
              SENSITIVE_CORE_0_IRAM0_PMS_MONITOR_VIOLATE_EN);

  modifyreg32(SENSITIVE_CORE_0_DRAM0_PMS_MONITOR_1_REG,
              SENSITIVE_CORE_0_DRAM0_PMS_MONITOR_VIOLATE_CLR_M,
              SENSITIVE_CORE_0_DRAM0_PMS_MONITOR_VIOLATE_EN);

  modifyreg32(SENSITIVE_CORE_0_PIF_PMS_MONITOR_1_REG,
              SENSITIVE_CORE_0_PIF_PMS_MONITOR_VIOLATE_CLR_M,
              SENSITIVE_CORE_0_PIF_PMS_MONITOR_VIOLATE_EN);

  /* Instruction and data cache reject monitors. */

  modifyreg32(EXTMEM_CORE0_ACS_CACHE_INT_CLR_REG,
              EXTMEM_CORE0_IBUS_REJECT_INT_CLR_M |
              EXTMEM_CORE0_DBUS_REJECT_INT_CLR_M, 0);
  modifyreg32(EXTMEM_CORE0_ACS_CACHE_INT_ENA_REG,
              EXTMEM_CORE0_IBUS_REJECT_INT_ENA_M |
              EXTMEM_CORE0_DBUS_REJECT_INT_ENA_M,
              EXTMEM_CORE0_IBUS_REJECT_INT_ENA |
              EXTMEM_CORE0_DBUS_REJECT_INT_ENA);
}

/****************************************************************************
 * Name: isolation_configure_iram
 *
 * Description:
 *   Instruction memory.  All of it belongs to the kernel except the WORLD1
 *   vector table, which the unprivileged world must be able to fetch and
 *   nothing more.
 *
 ****************************************************************************/

static void isolation_configure_iram(void)
{
  uintptr_t vstart = (uintptr_t)_world1_vectors;
  uintptr_t vend   = vstart + WORLD1_VECTORS_SIZE;

  /* Internal SRAM0, the blocks not given to the instruction cache.  They
   * hold the kernel's own vector table and the start of its IRAM code, and
   * the hardware can say nothing finer than a whole 16 KB block here, which
   * is why the WORLD1 table is not among them.
   */

  esp32s3_pms_configure_icache(PMS_AREA_0, PMS_WORLD_0, PMS_ACCESS_ALL);
  esp32s3_pms_configure_icache(PMS_AREA_1, PMS_WORLD_0, PMS_ACCESS_ALL);
  esp32s3_pms_configure_icache(PMS_AREA_0, PMS_WORLD_1, PMS_ACCESS_NONE);
  esp32s3_pms_configure_icache(PMS_AREA_1, PMS_WORLD_1, PMS_ACCESS_NONE);

  /* Internal SRAM1, split around the WORLD1 vector table:
   *
   *   area 0   the kernel's IRAM code
   *   area 1   the WORLD1 vector table
   *   area 2   whatever IRAM follows it
   *   area 3   past the main split line, which is data memory
   */

  esp32s3_pms_set_iram_split_line(PMS_SPLIT_LINE_0, vstart);
  esp32s3_pms_set_iram_split_line(PMS_SPLIT_LINE_1, vend);

  esp32s3_pms_configure_iram_region(PMS_AREA_0, PMS_WORLD_0, PMS_ACCESS_ALL);
  esp32s3_pms_configure_iram_region(PMS_AREA_1, PMS_WORLD_0, PMS_ACCESS_ALL);
  esp32s3_pms_configure_iram_region(PMS_AREA_2, PMS_WORLD_0, PMS_ACCESS_ALL);
  esp32s3_pms_configure_iram_region(PMS_AREA_3, PMS_WORLD_0,
                                    PMS_ACCESS_NONE);

  esp32s3_pms_configure_iram_region(PMS_AREA_0, PMS_WORLD_1,
                                    PMS_ACCESS_NONE);
  esp32s3_pms_configure_iram_region(PMS_AREA_1, PMS_WORLD_1, PMS_ACCESS_X);
  esp32s3_pms_configure_iram_region(PMS_AREA_2, PMS_WORLD_1,
                                    PMS_ACCESS_NONE);
  esp32s3_pms_configure_iram_region(PMS_AREA_3, PMS_WORLD_1,
                                    PMS_ACCESS_NONE);
}

/****************************************************************************
 * Name: isolation_configure_dram
 *
 * Description:
 *   Data memory.  A kernel build has none that belongs to the user: a
 *   process keeps its data, heap and stacks in PSRAM behind the cache MMU,
 *   so the unprivileged world has no business in internal RAM at all.
 *
 ****************************************************************************/

static void isolation_configure_dram(void)
{
  uintptr_t rom_reserved =
    ALIGN_DOWN(ets_rom_layout_p->dram0_rtos_reserved_start, 256);

  /* Internal SRAM2, the blocks not given to the data cache. */

  esp32s3_pms_configure_dcache(PMS_WORLD_0, PMS_ACCESS_ALL);
  esp32s3_pms_configure_dcache(PMS_WORLD_1, PMS_ACCESS_NONE);

  /* Area 0 is what lies before the main split line -- instruction memory,
   * which is not to be reached over the data bus by either world.  The rest
   * is the kernel's.
   */

  esp32s3_pms_set_dram_split_line(PMS_SPLIT_LINE_0,
                                  MAP_IRAM_TO_DRAM((uintptr_t)_iram_end));
  esp32s3_pms_set_dram_split_line(PMS_SPLIT_LINE_1, rom_reserved);

  esp32s3_pms_configure_dram_region(PMS_AREA_0, PMS_WORLD_0,
                                    PMS_ACCESS_NONE);
  esp32s3_pms_configure_dram_region(PMS_AREA_1, PMS_WORLD_0, PMS_ACCESS_ALL);
  esp32s3_pms_configure_dram_region(PMS_AREA_2, PMS_WORLD_0, PMS_ACCESS_ALL);
  esp32s3_pms_configure_dram_region(PMS_AREA_3, PMS_WORLD_0, PMS_ACCESS_ALL);

  esp32s3_pms_configure_dram_region(PMS_AREA_0, PMS_WORLD_1,
                                    PMS_ACCESS_NONE);
  esp32s3_pms_configure_dram_region(PMS_AREA_1, PMS_WORLD_1,
                                    PMS_ACCESS_NONE);
  esp32s3_pms_configure_dram_region(PMS_AREA_2, PMS_WORLD_1,
                                    PMS_ACCESS_NONE);
  esp32s3_pms_configure_dram_region(PMS_AREA_3, PMS_WORLD_1,
                                    PMS_ACCESS_NONE);
}

/****************************************************************************
 * Name: esp32s3_isolation_permissions
 *
 * Description:
 *   Give World 1 its permissions:  its own pages of the page pool, and
 *      nothing else.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_isolation_permissions(void)
{
  size_t psram_size;

  /* The WORLD1 vector table has to be in Internal SRAM1 for any of this to
   * mean anything (see isolation_configure_iram), and a mistake here would
   * be silent: the split lines would land somewhere harmless and the
   * unprivileged world would keep its access to instruction memory.
   */

  ASSERT((uintptr_t)_world1_vectors >= SOC_DIRAM_IRAM_LOW &&
         (uintptr_t)_world1_vectors + WORLD1_VECTORS_SIZE <=
         (uintptr_t)_iram_end);

  isolation_enable_interrupts();

  /* Divide Internal SRAM1 into its instruction and data halves.  The kernel
   * image is linked with its IRAM below _iram_end and its data above the
   * corresponding data-bus address.
   */

  esp32s3_pms_set_sram_main_split_line((uintptr_t)_iram_end);

  esp32s3_pms_configure_irom_access();
  esp32s3_pms_configure_drom_access();

  isolation_configure_iram();
  isolation_configure_dram();

  /* Cached external PSRAM.  Every page of a user process -- text, data and
   * heap alike -- is a page of the pgalloc pool, and that pool is a
   * contiguous physical window of the PSRAM device.  Give WORLD1 exactly
   * that window and nothing else, so the kernel's own PSRAM above and below
   * it is out of reach.  The ACE addresses are physical offsets into the
   * device, which is the same space mm_pgalloc() hands out.
   *
   * These registers were never programmed before, which left PSRAM at its
   * reset value -- open to both worlds -- while the whole of user space
   * lived in it.
   */

  psram_size = esp_spiram_get_size();

  DEBUGASSERT(ESP32S3_PGPOOL_PEND <= psram_size);

  esp32s3_pms_set_sram_split_line(PMS_SPLIT_LINE_0, 0,
                                  ESP32S3_PGPOOL_PBASE);
  esp32s3_pms_set_sram_split_line(PMS_SPLIT_LINE_1, ESP32S3_PGPOOL_PBASE,
                                  ESP32S3_PGPOOL_SIZE);
  esp32s3_pms_set_sram_split_line(PMS_SPLIT_LINE_2, ESP32S3_PGPOOL_PEND,
                                  psram_size - ESP32S3_PGPOOL_PEND);

  /* Region 3 is unused.  Park it at the end of the device with zero length:
   * the TRM forbids overlapping regions, so it cannot be left at zero.
   */

  esp32s3_pms_set_sram_split_line(PMS_SPLIT_LINE_3, psram_size, 0);

  esp32s3_pms_configure_sram_region(PMS_AREA_0, PMS_WORLD_0,
                                    PMS_ACCESS_ALL);
  esp32s3_pms_configure_sram_region(PMS_AREA_1, PMS_WORLD_0,
                                    PMS_ACCESS_ALL);
  esp32s3_pms_configure_sram_region(PMS_AREA_2, PMS_WORLD_0,
                                    PMS_ACCESS_ALL);
  esp32s3_pms_configure_sram_region(PMS_AREA_3, PMS_WORLD_0,
                                    PMS_ACCESS_ALL);

  /* The pool holds text and data pages interleaved, so the grant has to
   * cover both; the ACE cannot separate them at page granularity and W^X
   * within a process is not what this boundary is for.
   */

  esp32s3_pms_configure_sram_region(PMS_AREA_0, PMS_WORLD_1,
                                    PMS_ACCESS_NONE);
  esp32s3_pms_configure_sram_region(PMS_AREA_1, PMS_WORLD_1,
                                    PMS_ACCESS_ALL);
  esp32s3_pms_configure_sram_region(PMS_AREA_2, PMS_WORLD_1,
                                    PMS_ACCESS_NONE);
  esp32s3_pms_configure_sram_region(PMS_AREA_3, PMS_WORLD_1,
                                    PMS_ACCESS_NONE);

  /* Cached external flash.  A user process whose text was copied into PSRAM
   * needs nothing from flash -- but one running XIP does, so this cannot
   * simply deny every region.  Until the XIP case carries its own split
   * line between the kernel image and the mapped application, WORLD1 keeps
   * no flash access and XIP is unsupported here.
   */

  esp32s3_pms_configure_flash_cache_region(PMS_AREA_0, PMS_WORLD_0,
                                           PMS_ACCESS_ALL);
  esp32s3_pms_configure_flash_cache_region(PMS_AREA_1, PMS_WORLD_0,
                                           PMS_ACCESS_ALL);
  esp32s3_pms_configure_flash_cache_region(PMS_AREA_2, PMS_WORLD_0,
                                           PMS_ACCESS_ALL);
  esp32s3_pms_configure_flash_cache_region(PMS_AREA_3, PMS_WORLD_0,
                                           PMS_ACCESS_ALL);

  esp32s3_pms_configure_flash_cache_region(PMS_AREA_0, PMS_WORLD_1,
                                           PMS_ACCESS_NONE);
  esp32s3_pms_configure_flash_cache_region(PMS_AREA_1, PMS_WORLD_1,
                                           PMS_ACCESS_NONE);
  esp32s3_pms_configure_flash_cache_region(PMS_AREA_2, PMS_WORLD_1,
                                           PMS_ACCESS_NONE);
  esp32s3_pms_configure_flash_cache_region(PMS_AREA_3, PMS_WORLD_1,
                                           PMS_ACCESS_NONE);

  esp32s3_isolation_revoke_peripherals();
}
#endif

/****************************************************************************
 * Name: esp32s3_pmsirqinitialize
 *
 * Description:
 *   Install the handlers that report a permission violation and an access
 *      that no MMU entry translates.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_pmsirqinitialize(void)
{
  VERIFY(esp_setup_irq(ESP32S3_PERIPH_CORE_0_IRAM0_PMS_MONITOR_VIOLATE,
                       1, ESP_IRQ_TRIGGER_LEVEL, pms_violation_isr, NULL));
  VERIFY(esp_setup_irq(ESP32S3_PERIPH_CORE_0_DRAM0_PMS_MONITOR_VIOLATE,
                       1, ESP_IRQ_TRIGGER_LEVEL, pms_violation_isr, NULL));
  VERIFY(esp_setup_irq(ESP32S3_PERIPH_CACHE_CORE0_ACS,
                       1, ESP_IRQ_TRIGGER_LEVEL, pms_violation_isr, NULL));
  VERIFY(esp_setup_irq(ESP32S3_PERIPH_CORE_0_PIF_PMS_MONITOR_VIOLATE,
                       1, ESP_IRQ_TRIGGER_LEVEL, pms_violation_isr, NULL));

  /* Report an access that no MMU entry translates.  The PMS grants and
   * refuses physical addresses, so an untranslated access is invisible to
   * it:  the cache returns zeros and the task carries on with a value it
   * never should have had.  This monitor is what turns that into a fault.
   */

  VERIFY(esp_setup_irq(ESP32S3_PERIPH_CACHE_IA,
                       1, ESP_IRQ_TRIGGER_LEVEL, pms_violation_isr, NULL));

  modifyreg32(EXTMEM_CACHE_ILG_INT_ENA_REG, 0,
              EXTMEM_MMU_ENTRY_FAULT_INT_ENA_M);

  up_enable_irq(ESP32S3_IRQ_CORE_0_IRAM0_PMS_MONITOR_VIOLATE);
  up_enable_irq(ESP32S3_IRQ_CORE_0_DRAM0_PMS_MONITOR_VIOLATE);
  up_enable_irq(ESP32S3_IRQ_CACHE_CORE0_ACS);
  up_enable_irq(ESP32S3_IRQ_CORE_0_PIF_PMS_MONITOR_VIOLATE);
  up_enable_irq(ESP32S3_IRQ_CACHE_IA);
}

#endif /* !CONFIG_BUILD_FLAT */
