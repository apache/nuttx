/****************************************************************************
 * arch/arm/src/rtl8721f/ameba_app_start.c
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
 * NuttX-owned image2 entry for the RTL8721F km4tz application core.
 *
 * This is a NuttX adaptation of the Realtek SDK's app_start()
 * (component/soc/RTL8721F/fwlib/ram_km4tz/ameba_app_start.c).  It runs the
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
   * access faults.  On Green2 the TCM cache window (0x000F0000..0x00100000)
   * is used as RAM in fullmac mode and must not be read-only, so the RO
   * region stops at 0x000F0000.
   */

  mpu_entry = mpu_entry_alloc();
  mpu_cfg.region_base = 0;
  mpu_cfg.region_size = 0x000f0000;
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

/* Seed the RTC on first power-on (mirrors the SDK app_rtc_init). */

void app_rtc_init(void)
{
  RTC_InitTypeDef rtc_initstruct;
  RTC_TimeTypeDef rtc_timestruct;

  RTC_TimeStructInit(&rtc_timestruct);
  rtc_timestruct.RTC_Year = 2021;
  rtc_timestruct.RTC_Hours = 10;
  rtc_timestruct.RTC_Minutes = 20;
  rtc_timestruct.RTC_Seconds = 30;

  RTC_StructInit(&rtc_initstruct);
  RTC_Enable(ENABLE);
  RTC_Init(&rtc_initstruct);
  RTC_SetTime(RTC_Format_BIN, &rtc_timestruct);
}

/* The image2 application entry point. */

void app_start(void)
{
  /* Enable the non-secure cache. */

  Cache_Enable(ENABLE);

  /* Clear the non-secure ROM BSS and the image2 BSS (the latter covers
   * NuttX's .bss too).
   */

  _memset((void *)__rom_bss_start_ns__, 0,
          (__rom_bss_end_ns__ - __rom_bss_start_ns__));
  _memset((void *)__bss_start__, 0, (__bss_end__ - __bss_start__));

  RBSS_UDELAY_DIV = 5;

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
  XTAL_INIT();

  if (EFUSE_GetChipVersion() >= SYSCFG_CUT_VERSION_B)
    {
      if (SYSCFG_CHIPType_Get() == CHIP_TYPE_ASIC_POSTSIM)
        {
          /* Only ASIC needs OSC calibration. */

          OSC4M_Init();
          OSC4M_Calibration(30000);
        }
    }

  /* Low-power pins do not need pinmap init again after wake from dslp. */

  pinmap_init();

  mpu_init();
  app_mpu_nocache_init();

  /* Green2's system timer is clocked from SDM32K and needs the RTC
   * brought up first.  The SDK does the first-power-on half of this
   * asynchronously from
   * the RTC_DET_IRQ handler (rtc_irq_init), but NuttX replaces the vector
   * table immediately after app_start so that deferred handler would never
   * run -- do the whole sequence synchronously here instead.
   */

  RTC_ClearDetINT();
  SDM32K_Enable();
  SYSTIMER_Init();
  RCC_PeriphClockCmd(NULL, APBPeriph_RTC_CLOCK, ENABLE);

  if ((Get_OSC131_STATE() & RTC_BIT_FIRST_PON) == 0)
    {
      app_rtc_init();
      Set_OSC131_STATE(Get_OSC131_STATE() | RTC_BIT_FIRST_PON);

      /* Only ASIC needs OSC131K calibration (cke_rtc enabled just above). */

      if (SYSCFG_CHIPType_Get() == CHIP_TYPE_ASIC_POSTSIM)
        {
          OSC131K_Calibration(30000);
        }
    }

  /* SDM32K clock-source switch must be done after rtc_fen is enabled. */

  RTC_ClkSource_Select(SDM32K);

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
