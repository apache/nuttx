/****************************************************************************
 * arch/arm/src/rtl8720f/ameba_app_start.c
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
 * NuttX-owned image2 entry for the RTL8720F km4tz application core.
 *
 * This is a NuttX adaptation of the Realtek SDK's app_start()
 * (component/soc/RTL8720F/fwlib/ram_km4tz/ameba_app_start.c).  It runs the
 * same OS-independent silicon init the SDK boot performs -- enable cache,
 * clear image2 BSS, set the log level, data-flash high-speed setup, OSC
 * calibration, the system timer, pin-mux and MPU, then call NuttX's main().
 * The SDK FreeRTOS/newlib bring-up and its fault-backtrace patch are
 * intentionally omitted; NuttX provides its own scheduler, libc and crash
 * reporting.
 *
 * Owning this file (not patching the SDK ameba_app_start.c) keeps the
 * vendor SDK pristine.  It is compiled with the SDK fwlib include set (see
 * the board AMEBA_FWLIB_SRCS), not the NuttX header set.  The image2 entry
 * uses a NULL ram_wakeup so the SDK deep-sleep (SOCPS) wake path is not
 * pulled into the image.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "ameba_soc.h"
#include "os_wrapper.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *TAG = "APP";

/****************************************************************************
 * External Function Prototypes
 ****************************************************************************/

extern int main(void);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

u32 app_mpu_nocache_check(u32 mem_addr)
{
  mpu_region_config mpu_cfg;

  mpu_cfg.region_base = (uint32_t)__ram_nocache_start__;
  mpu_cfg.region_size = __ram_nocache_end__ - __ram_nocache_start__;

  if ((mem_addr >= mpu_cfg.region_base) &&
      (mem_addr < (mpu_cfg.region_base + mpu_cfg.region_size)))
    {
      return TRUE;
    }
  else
    {
      return FALSE;
    }
}

/* AP has 8 secure mpu entries & 8 non-secure mpu entries. */

u32 app_mpu_nocache_init(void)
{
  mpu_region_config mpu_cfg;
  u32 mpu_entry = 0;

  /* ROM code inside the CPU does not enter cache; set it RO so a NULL-ptr
   * access faults.
   */

  mpu_entry = mpu_entry_alloc();
  mpu_cfg.region_base = 0x20000;
  mpu_cfg.region_size = 0x18000;
  mpu_cfg.xn = MPU_EXEC_ALLOW;
  mpu_cfg.ap = MPU_UN_PRIV_RO;
  mpu_cfg.sh = MPU_NON_SHAREABLE;
  mpu_cfg.attr_idx = MPU_MEM_ATTR_IDX_NC;
  mpu_region_cfg(mpu_entry, &mpu_cfg);

  /* nocache region */

  mpu_entry = mpu_entry_alloc();
  mpu_cfg.region_base = (uint32_t)__ram_nocache_start__;
  mpu_cfg.region_size = __ram_nocache_end__ - __ram_nocache_start__;
  mpu_cfg.xn = MPU_EXEC_ALLOW;
  mpu_cfg.ap = MPU_UN_PRIV_RW;
  mpu_cfg.sh = MPU_NON_SHAREABLE;
  mpu_cfg.attr_idx = MPU_MEM_ATTR_IDX_NC;
  if (mpu_cfg.region_size >= 32)
    {
      mpu_region_cfg(mpu_entry, &mpu_cfg);
    }

  /* set global bss to nocache */

  DCache_CleanInvalidate((u32)__global_bss_start__,
                         (u32)(__global_bss_end__ - __global_bss_start__));
  mpu_entry = mpu_entry_alloc();
  mpu_cfg.region_base = (uint32_t)__global_bss_start__;
  mpu_cfg.region_size = __global_bss_end__ - __global_bss_start__;
  mpu_cfg.xn = MPU_EXEC_ALLOW;
  mpu_cfg.ap = MPU_UN_PRIV_RW;
  mpu_cfg.sh = MPU_NON_SHAREABLE;
  mpu_cfg.attr_idx = MPU_MEM_ATTR_IDX_NC;
  if (mpu_cfg.region_size >= 32)
    {
      mpu_region_cfg(mpu_entry, &mpu_cfg);
    }

  /* nocache region */

  mpu_entry = mpu_entry_alloc();
  mpu_cfg.region_base = (uint32_t)__sram_image2_start__;
  mpu_cfg.region_size = N_BYTE_ALIGMENT(((u32)__sram_image2_end__ -
                                         (u32)__sram_image2_start__), 32);
  mpu_cfg.xn = MPU_EXEC_ALLOW;
  mpu_cfg.ap = MPU_UN_PRIV_RW;
  mpu_cfg.sh = MPU_NON_SHAREABLE;
  mpu_cfg.attr_idx = MPU_MEM_ATTR_IDX_NC;
  if (mpu_cfg.region_size >= 32)
    {
      mpu_region_cfg(mpu_entry, &mpu_cfg);
    }

  return 0;
}

#if defined(__GNUC__)
/* Provided for C++ support so the toolchain init does not fail to link. */

void _init(void)
{
}
#endif

void app_testmode_status(void)
{
  /* OTPC and SIC share one master port; OTPC uses it by default and SIC can
   * use it once OTPC autoload is done.
   */

  if (SYSCFG_TRP_TestMode())
    {
      if (SYSCFG_TRP_OTPBYP())
        {
          RTK_LOGI(TAG, "Bypass OTP autoload\r\n");
        }
      else
        {
          RTK_LOGI(TAG, "In Test mode: 0x%lx\r\n", SYSCFG_TRP_ICFG());
        }
    }
}

void app_init_debug_flag(void)
{
  /* Initialise the log level used by the ROM-code global variable. */

  if (SYSCFG_OTP_DisBootLog() == FALSE)
    {
      rtk_log_level_set("*", RTK_LOG_INFO);
    }
  else
    {
      rtk_log_level_set("*", RTK_LOG_ERROR);
    }
}

void os_init(void)
{
#ifdef CONFIG_PSRAM_ALL_FOR_AP_HEAP
#if (defined CONFIG_WHC_HOST || defined CONFIG_WHC_NONE)
  extern bool os_heap_add(u8 *start_addr, size_t heap_size);
  if (ChipInfo_PsramExists())
    {
      os_heap_add((uint8_t *)__km4tz_bd_psram_start__,
                  (size_t)(__non_secure_psram_end__ -
                           __km4tz_bd_psram_start__));
    }

#endif
#endif
  rtos_mem_init();
}

/* The image2 application entry point. */

void app_start(void)
{
  /* Enable the non-secure cache. */

  Cache_Enable(ENABLE);

  /* Clear the image2 BSS (covers NuttX's .bss too). */

  _memset((void *)__bss_start__, 0, (__bss_end__ - __bss_start__));

  app_init_debug_flag();

#ifdef CONFIG_TRUSTZONE
  PutChar = (void (*)(char))LOGUART_PutChar;
  SCB->VTOR = (u32)RomVectorTable;
  RomVectorTable[0] = (HAL_VECTOR_FUN)MSP_RAM_HP_NS;
#endif

  app_testmode_status();

  data_flash_highspeed_setup();

  SystemCoreClockUpdate();
  RTK_LOGI(TAG, "AP CPU CLK: %lu Hz \n", SystemCoreClock);

  /* Heap region setup (a no-op under NuttX, which owns its own heap). */

  os_init();

  if (SYSCFG_CHIPType_Get() == CHIP_TYPE_ASIC)
    {
      /* Only ASIC needs OSC calibration. */

      OSC2M_Calibration(30000);
    }

  SYSTIMER_Init();

  /* Low-power pins do not need pinmap init again after wake from dslp. */

  pinmap_init();

  mpu_init();
  app_mpu_nocache_init();

  main();
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Image2 entry descriptor: the SDK bootloader (image1) jumps to ram_start.
 * A NULL ram_wakeup keeps the SDK deep-sleep (SOCPS) wake path -- which the
 * SDK's own descriptor wires to SOCPS_WakeFromPG_AP -- out of the image.
 */

IMAGE2_ENTRY_SECTION
RAM_START_FUNCTION Img2EntryFun0 =
{
  app_start,
  NULL,
  (u32)RomVectorTable
};
