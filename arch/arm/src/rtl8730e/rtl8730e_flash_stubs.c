/****************************************************************************
 * arch/arm/src/rtl8730e/rtl8730e_flash_stubs.c
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
 * NuttX CA32 stubs required by the SDK's FLASH_Write_Lock/Unlock protocol.
 *
 * When the SDK's ameba_flash_ram.c erases or programs flash on CA32 it:
 *   1. Calls vPortGateOtherCore() to WFE core1 (SMP gate).
 *   2. Sends IPC_A2N_FLASHPG_REQ to KM4 and waits for acknowledgement.
 *   3. Calls xlat_flash_region_device() to remap the flash XIP window as
 *      Device/non-cacheable in the ARMv7-A L1 page table so no speculative
 *      fetch crosses the SPIC during the program operation.
 *
 * On single-core NuttX (CONFIG_SMP=n) vPortGateOtherCore/Wake are no-ops.
 * xlat_flash_region_device/xip update the first-level page table directly.
 *
 * _memcpy / _memset are SDK-internal libc equivalents called by the IPC
 * layer (ameba_ipc_api.c); on CA32 there is no ROM table for them, so we
 * alias them to the standard library functions.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>

#include <nuttx/arch.h>

#include "mmu.h"
#include "cp15_cacheops.h"
#include "rtl8730e_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Flash XIP window: 0x08000000-0x0FFFFFFF (128 MiB), 128 x 1 MiB sections.
 * Matches the range the SDK maps as MT_CODE in setupMMUTable().
 */

#define FLASH_XIP_BASE   0x08000000u
#define FLASH_XIP_SIZE   (128u * 1024u * 1024u)  /* 128 MiB */
#define FLASH_XIP_END    (FLASH_XIP_BASE + FLASH_XIP_SIZE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vPortGateOtherCore / vPortWakeOtherCore
 *
 * Description:
 *   SMP gate/wake helpers.  NuttX is single-core on CA32 (SMP not yet
 *   enabled), so these are harmless no-ops.
 ****************************************************************************/

void vPortGateOtherCore(void)
{
}

void vPortWakeOtherCore(void)
{
}

/****************************************************************************
 * Name: xlat_flash_region_device
 *
 * Description:
 *   Remap the flash XIP window (0x08000000–0x0FFFFFFF) to Device/non-
 *   cacheable in the ARMv7-A L1 page table.  Called by FLASH_Write_Lock
 *   before programming or erasing flash, so no speculative cache fill can
 *   happen while the SPIC is busy.
 *
 *   Steps:
 *     1. Clean+invalidate D-cache for the flash window (write back any dirty
 *        lines so the device view is coherent).
 *     2. Re-map all 128 L1 sections to MMU_IOFLAGS (Device, non-cacheable).
 *     3. Flush the TLB (via mmu_invalidate_region which also issues
 *        DSB/ISB).
 ****************************************************************************/

void xlat_flash_region_device(void)
{
  uintptr_t addr;

  /* Flush dirty D-cache lines for the flash window. */

  cp15_flush_dcache(FLASH_XIP_BASE, FLASH_XIP_END);

  /* Remap each 1 MiB section to Device (non-cacheable, non-executable). */

  for (addr = FLASH_XIP_BASE; addr < FLASH_XIP_END; addr += 0x00100000u)
    {
      mmu_l1_setentry(addr, addr, MMU_IOFLAGS);
    }

  /* Invalidate TLB for the remapped range. */

  mmu_invalidate_region(FLASH_XIP_BASE, FLASH_XIP_SIZE);
}

/****************************************************************************
 * Name: xlat_flash_region_xip
 *
 * Description:
 *   Restore the flash XIP window to Normal-Cacheable/Executable.  Called
 *   by FLASH_Write_Unlock after programming or erasing flash.
 *
 *   Steps:
 *     1. Re-map all 128 L1 sections to MMU_ROMFLAGS (normal, cacheable, RO,
 *        executable).
 *     2. Flush the TLB and I-cache so the processor re-fetches from flash
 *        rather than from stale cache lines.
 ****************************************************************************/

void xlat_flash_region_xip(void)
{
  uintptr_t addr;

  /* Remap each 1 MiB section back to Normal Cacheable/Executable. */

  for (addr = FLASH_XIP_BASE; addr < FLASH_XIP_END; addr += 0x00100000u)
    {
      mmu_l1_setentry(addr, addr, MMU_ROMFLAGS);
    }

  /* Invalidate TLB and I-cache for the restored range. */

  mmu_invalidate_region(FLASH_XIP_BASE, FLASH_XIP_SIZE);
}

/****************************************************************************
 * Name: _memcpy / _memset
 *
 * Description:
 *   SDK-internal libc equivalents used by ameba_ipc_api.c.  On CA32 there
 *   is no ROM symbol table supplying these, so alias them to libc.
 ****************************************************************************/

void *_memcpy(void *s1, const void *s2, size_t n)
{
  return memcpy(s1, s2, n);
}

void *_memset(void *s, int c, size_t n)
{
  return memset(s, c, n);
}

int _strcmp(const char *s1, const char *s2)
{
  return strcmp(s1, s2);
}

/****************************************************************************
 * RTOS OS-wrapper stubs
 *
 * ameba_ipc_api.c uses IPC_wait_idle() which takes the busy-wait path
 * whenever IPC_IrqHandler[chan] == NULL (the default).  The semaphore/mutex
 * functions below are therefore never actually called, but the linker
 * requires their symbols to be present.
 *
 * log.c wraps DiagPrintfNano/DiagVprintfNano with a mutex; that mutex is
 * also NULL-initialised (log_mutex == NULL), so rtos_mutex_take/give are
 * called with a NULL handle.  We treat NULL as "no mutex, proceed".
 ****************************************************************************/

/* RTK_SUCCESS = 0 */

#define RTK_SUCCESS 0

int CPU_InInterrupt(void)
{
  /* Check CPSR mode: anything other than User (0x10) or System (0x1f)
   * means we are in an exception.  Return 1 so callers know to skip
   * sleeping primitives.
   */

  uint32_t cpsr;

  __asm__ volatile("mrs %0, cpsr" : "=r"(cpsr));
  return ((cpsr & 0x1f) != 0x10) && ((cpsr & 0x1f) != 0x1f);
}

/* rtos_sema_* / rtos_mutex_* are weak stubs used when ameba_os_wrap.c is
 * NOT linked (i.e. FLASH_FS without WiFi).  When WiFi is enabled,
 * ameba_os_wrap.c provides the real NuttX-backed implementations and the
 * linker selects them over these weak definitions.
 */

__attribute__((weak)) int rtos_sema_create_binary(void **pp_handle)
{
  (void)pp_handle;
  return RTK_SUCCESS;
}

__attribute__((weak)) int rtos_sema_take(void *p_handle, uint32_t timeout_ms)
{
  (void)p_handle;
  (void)timeout_ms;
  return RTK_SUCCESS;
}

__attribute__((weak)) int rtos_sema_give(void *p_handle)
{
  (void)p_handle;
  return RTK_SUCCESS;
}

__attribute__((weak)) int rtos_mutex_create_static(void **pp_handle)
{
  (void)pp_handle;
  return RTK_SUCCESS;
}

__attribute__((weak)) int rtos_mutex_take(void *p_handle, uint32_t wait_ms)
{
  (void)p_handle;
  (void)wait_ms;
  return RTK_SUCCESS;
}

__attribute__((weak)) int rtos_mutex_give(void *p_handle)
{
  (void)p_handle;
  return RTK_SUCCESS;
}

void io_assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
  __asm__ volatile("b .");
}

int DiagPrintfNano(const char *fmt, ...)
{
  va_list ap;
  int ret;

  va_start(ap, fmt);
  ret = vprintf(fmt, ap);
  va_end(ap);
  return ret;
}

int DiagVprintfNano(const char *fmt, va_list ap)
{
  return vprintf(fmt, ap);
}

/****************************************************************************
 * __km0_ipc_memory_start__
 *
 * ameba_ipc_api.c casts this symbol address to PIPC_MSG_STRUCT:
 *   IPC_MSG = (PIPC_MSG_STRUCT)__km0_ipc_memory_start__;
 * The KM0 SRAM IPC ring buffer lives at 0x2301fd00 (vendor SDK).
 * Define the symbol as an absolute address via inline asm so no linker
 * script change is required.
 ****************************************************************************/

__asm__(".global __km0_ipc_memory_start__\n"
        "__km0_ipc_memory_start__ = 0x2301fd00\n");

/****************************************************************************
 * SDK ROM function stubs
 *
 * lib_rom.a objects reference these SDK platform functions.  On CA32 /
 * NuttX we either route them through NuttX primitives or stub them out
 * (the LOGUART/pinmux paths in lib_rom.a are never actually called at
 * runtime because NuttX owns the interrupt and pinmux setup).
 ****************************************************************************/

/* irq_register / irq_enable: SDK interrupt registration shim.
 * NuttX owns all GIC wiring via irq_attach / up_enable_irq; these stubs
 * are present only to satisfy lib_rom.a objects that are never called.
 */

bool irq_register(void *irqfun, int irqnum, uint32_t data, uint32_t prio)
{
  (void)irqfun;
  (void)irqnum;
  (void)data;
  (void)prio;
  return true;
}

bool irq_enable(int irqnum, uint32_t prio)
{
  (void)irqnum;
  (void)prio;
  return true;
}

/* System_Reset: not implemented for CA32 (KM4 owns power management). */

void System_Reset(void)
{
  __asm__ volatile("b .");
}

/* Pinmux_Config: flash/LOGUART pinmux is handled by KM4; stub for CA32. */

void Pinmux_Config(uint32_t pinname, uint32_t pinfunc)
{
  (void)pinname;
  (void)pinfunc;
}

/****************************************************************************
 * Name: rtl8730e_flash_init_para
 *
 * Description:
 *   Copy flash_init_para from KM4's initialized copy.
 *
 *   KM4 runs FLASH_Init() early in its startup and stores the address of
 *   the resulting FLASH_InitTypeDef in LP system-control register
 *   SYSTEM_CTRL_BASE_LP + REG_LSYS_FLASH_PARA_ADDR (0x42008270).
 *   CA32 must copy it before any write/erase; otherwise FLASH_TxData in
 *   lib_rom.a reads zeroed parameters and hangs waiting for the SPIC.
 *
 *   flash_init_para is 0x74 bytes (confirmed via nm lib_rom.a).
 *   Must be called once, before ameba_flash_fs_initialize().
 ****************************************************************************/

/* flash_init_para lives in lib_rom.a BSS; sizeof = 0x74. */

extern uint8_t flash_init_para[0x74];

void rtl8730e_flash_init_para(void)
{
  volatile uint32_t *reg =
    (volatile uint32_t *)0x42008270u; /* LSYS + REG_LSYS_FLASH_PARA_ADDR */
  uint32_t src = *reg;

  if (src != 0u)
    {
      memcpy(flash_init_para, (const void *)src,
             sizeof(flash_init_para));
    }
}
