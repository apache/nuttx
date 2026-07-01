/****************************************************************************
 * arch/arm/src/rtl8721dx/ameba_app_start.c
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
 * NuttX-owned image2 entry for the RTL8721Dx KM4 application core.
 *
 * This is a NuttX adaptation of the Realtek SDK's app_start()
 * (component/soc/amebadplus/fwlib/ram_km4/ameba_app_start.c).  It runs the
 * same OS-independent silicon init the SDK boot performs -- enable cache,
 * clear image2 BSS, init the NS vector table, MPU, brown-out detector,
 * crystal/OSC calibration and the data-flash high-speed setup, then call
 * NuttX's main().  The SDK FreeRTOS/newlib bring-up and its fault-handler
 * redirect are intentionally omitted; NuttX provides its own scheduler, libc
 * and crash reporting.
 *
 * Owning this file (not patching the SDK ameba_app_start.c) keeps the
 * vendor SDK pristine.  It is compiled with the SDK fwlib include set (see
 * the board AMEBA_FWLIB_SRCS), not the NuttX header set, so it uses the SDK
 * types and APIs directly.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "ameba_soc.h"
#include "os_wrapper.h"
#include "os_wrapper_memory.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *const TAG = "APP";

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

/* KM4 has 4 secure mpu entries & 8 non-secure mpu entries. */

u32 app_mpu_nocache_init(void)
{
  mpu_region_config mpu_cfg;
  u32 mpu_entry = 0;

  /* ROM code inside the CPU does not enter cache; set it RO so a NULL-ptr
   * access faults.
   */

  mpu_entry = mpu_entry_alloc();
  mpu_cfg.region_base = 0x00000000;
  mpu_cfg.region_size = 0x00080000;
  mpu_cfg.xn = MPU_EXEC_ALLOW;
  mpu_cfg.ap = MPU_UN_PRIV_RO;
  mpu_cfg.sh = MPU_NON_SHAREABLE;
  mpu_cfg.attr_idx = MPU_MEM_ATTR_IDX_NC;
  mpu_region_cfg(mpu_entry, &mpu_cfg);

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

void app_bod_init(void)
{
  /* Only init BOD on the first power-on. */

  if (BOOT_Reason() != 0)
    {
      return;
    }

  BOR_ThresholdSet(0x8, 0x5);
  RTK_LOGI(TAG, "BOR arises when supply voltage decreases under 2.57V and "
                "recovers above 2.7V.\n");

  BOR_ModeSet(BOR_RESET);
  BOR_Enable(ENABLE);

  /* Avoid an unwanted extra reset.  Default debounce delay is 100us
   * (BOR_TDBC = 0x1); it takes 100us for the BOD output to sync to the
   * digital circuit.
   */

  DelayUs(100);
  RCC_PeriphClockCmd(APBPeriph_BOR, APBPeriph_CLOCK_NULL, ENABLE);
}

void app_vdd1833_detect(void)
{
  ADC_InitTypeDef ADC_InitStruct;
  u32 buf[16];
  u32 i = 0;
  u32 temp = 0;
  u32 data = 0;

  RCC_PeriphClockCmd(APBPeriph_ADC, APBPeriph_ADC_CLOCK, ENABLE);

  ADC_StructInit(&ADC_InitStruct);
  ADC_InitStruct.ADC_OpMode = ADC_AUTO_MODE;
  ADC_InitStruct.ADC_CvlistLen = 0;
  ADC_InitStruct.ADC_Cvlist[0] = ADC_CH9;     /* AVDD33 */
  ADC_Init(&ADC_InitStruct);

  ADC_Cmd(ENABLE);
  ADC_ReceiveBuf(buf, 16);
  ADC_Cmd(DISABLE);

  while (i < 16)
    {
      data += ADC_GET_DATA_GLOBAL(buf[i++]);
    }

  data >>= 4;

  temp = HAL_READ32(SYSTEM_CTRL_BASE, REG_AON_RSVD_FOR_SW1);
  if (data > 3000)                            /* 3000: about 2.4V */
    {
      temp |= AON_BIT_WIFI_RF_1833_SEL;
    }
  else
    {
      temp &= ~AON_BIT_WIFI_RF_1833_SEL;
      RTK_LOGI(TAG, "IO Power 1.8V\n");
    }

  HAL_WRITE32(SYSTEM_CTRL_BASE, REG_AON_RSVD_FOR_SW1, temp);
}

/* The image2 application entry point. */

void app_start(void)
{
  RTK_LOGI(TAG, "KM4 APP START \n");

  /* Enable the non-secure cache. */

  Cache_Enable(ENABLE);

  /* Clear the image2 BSS (covers NuttX's .bss too). */

  _memset((void *)__bss_start__, 0, (__bss_end__ - __bss_start__));

  /* NS vector table init (NuttX repoints VTOR at its own table in main()). */

  irq_table_init(MSP_RAM_HP_NS);

  RTK_LOGI(TAG, "VTOR: %lx, VTOR_NS:%lx\n", SCB->VTOR, SCB_NS->VTOR);

#if (defined CONFIG_WHC_HOST || defined CONFIG_WHC_NONE)
  extern bool os_heap_add(u8 *start_addr, size_t heap_size);
#ifdef CONFIG_PSRAM_ALL_FOR_AP_HEAP
  if (ChipInfo_PsramExists())
    {
      os_heap_add((uint8_t *)__km4_bd_psram_start__,
                  (size_t)(__non_secure_psram_end__ -
                           __km4_bd_psram_start__));
    }
#endif
#endif

  /* Heap region setup (a no-op under NuttX, which owns its own heap). */

  rtos_mem_init();

  RTK_LOGI(TAG, "VTOR: %lx, VTOR_NS:%lx\n", SCB->VTOR, SCB_NS->VTOR);

  app_testmode_status();
  data_flash_highspeed_setup();

  cmse_address_info_t cmse_address_info = cmse_TT((void *)app_start);
  RTK_LOGI(TAG, "IMG2 SECURE STATE: %d\n", cmse_address_info.flags.secure);

  SystemCoreClockUpdate();

  XTAL_INIT();

  if (SYSCFG_CHIPType_Get() == CHIP_TYPE_ASIC_POSTSIM)
    {
      /* Only ASIC needs OSC calibration. */

      OSC4M_Init();
      OSC4M_Calibration(30000);
      if ((((BOOT_Reason()) & AON_BIT_RSTF_DSLP) == FALSE) &&
          (RTCIO_IsEnabled() == FALSE))
        {
          OSC131K_Calibration(30000);         /* PPM=30000=3% (7.5ms) */
        }
    }

  mpu_init();
  app_mpu_nocache_init();

  app_vdd1833_detect();

  app_bod_init();

  /* Force SP alignment to 8 bytes. */

  __asm("ldr r1, =#0xFFFFFFF8\n"
        "mov r0, sp \n"
        "and r0, r0, r1\n"
        "mov sp, r0\n");

  main();
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Image2 entry descriptor: the SDK bootloader (image1) jumps to ram_start.
 * A NULL ram_wakeup keeps the SDK deep-sleep (SOCPS) path out of the image.
 */

IMAGE2_ENTRY_SECTION
RAM_START_FUNCTION Img2EntryFun0 =
{
  app_start,
  NULL,
  (u32)NewVectorTable
};
