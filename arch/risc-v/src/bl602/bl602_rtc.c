/****************************************************************************
 * arch/risc-v/src/bl602/bl602_rtc.c
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

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/timers/rtc.h>

#include "riscv_internal.h"
#include "chip.h"
#include "hardware/bl602_hbn.h"
#include "bl602_rtc.h"

#define ROM_APITABLE            ((uint32_t *)0x21010800)
#define ROMAPI_HBN_32K_SEL      (ROM_APITABLE + 66)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef void (*bl602_romapi_hbn_sel_t)(uint8_t clk_type);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_hbn_sel
 *
 * Description:
 *   HBN select 32K
 *
 * Input Parameters:
 *   clk_type: HBN 32k clock type
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_hbn_sel(uint8_t clk_type)
{
  modifyreg32(BL602_HBN_GLB,
              HBN_GLB_HBN_F32K_SEL_MASK,
              clk_type << HBN_GLB_HBN_F32K_SEL_SHIFT);
}

/****************************************************************************
 * Name: bl602_hbn_clear_rtc_int
 *
 * Description:
 *   HBN clear RTC timer interrupt,this function must be called to clear
 *   delayed rtc IRQ
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_hbn_clear_rtc_int(void)
{
  /* Clear RTC commpare:bit1-3 for clearing Delayed RTC IRQ */

  modifyreg32(BL602_HBN_CTL, 0x7 << 1, 0);
}

/****************************************************************************
 * Name: bl602_hbn_set_rtc_timer
 *
 * Description:
 *   HBN set RTC timer configuration
 *
 * Input Parameters:
 *   delay: RTC interrupt delay 32 clocks
 *   compval_low: RTC interrupt commpare value low 32 bits
 *   compval_high: RTC interrupt commpare value high 32 bits
 *   comp_mode: RTC interrupt commpare
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_hbn_set_rtc_timer(uint8_t delay_type, uint32_t compval_low,
                             uint32_t compval_high,  uint8_t comp_mode)
{
  putreg32(compval_low, BL602_HBN_TIME_L);
  putreg32(compval_high & 0xff, BL602_HBN_TIME_H);

  modifyreg32(BL602_HBN_CTL,
              HBN_CTL_RTC_DLY_OPTION | (0x7 << 1),
              delay_type | (comp_mode << 1));
}

/****************************************************************************
 * Name: bl602_hbn_clear_rtc_counter
 *
 * Description:
 *   HBN set RTC timer configuration
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_hbn_clear_rtc_counter(void)
{
  modifyreg32(BL602_HBN_CTL, 1, 0);
}

/****************************************************************************
 * Name: bl602_hbn_enable_rtc_counter
 *
 * Description:
 *   HBN clear RTC timer counter
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_hbn_enable_rtc_counter(void)
{
  modifyreg32(BL602_HBN_CTL, 0, 1);
}

/****************************************************************************
 * Name: bl602_hbn_get_rtc_timer_val
 *
 * Description:
 *   HBN get RTC timer count value
 *
 * Input Parameters:
 *   val_low: RTC count value pointer for low 32 bits
 *   val_high: RTC count value pointer for high 8 bits
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_hbn_get_rtc_timer_val(uint32_t *val_low, uint32_t *val_high)
{
  modifyreg32(BL602_HBN_RTC_TIME_H, 0, HBN_RTC_TIME_H_RTC_TIME_LATCH);

  modifyreg32(BL602_HBN_RTC_TIME_H, HBN_RTC_TIME_H_RTC_TIME_LATCH, 0);

  *val_low = getreg32(BL602_HBN_RTC_TIME_L);
  *val_high = getreg32(BL602_HBN_RTC_TIME_H) & 0xff;
}

