/****************************************************************************
 * arch/arm/src/phy62xx/clock.h
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

#ifndef _HAL_CLOCK_H
#define _HAL_CLOCK_H

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "types.h"
#include "bus_dev.h"

typedef enum
{
    CLK_32K_XTAL        = 0,
    CLK_32K_RCOSC       = 1,
}   CLK32K_e;

typedef enum
{
    XTAL_16M  = 0,
    DBL_B_32M = 1,
    DBL_32    = 2,
    DLL_32M   = 3,
}   ClkSrc_e;

typedef enum  _SYSCLK_SEL
{
    SYS_CLK_RC_32M      = 0,
    SYS_CLK_DLL_32M     = 1,
    SYS_CLK_XTAL_16M    = 2,
    SYS_CLK_DLL_48M     = 3,
    SYS_CLK_DLL_64M     = 4,
    SYS_CLK_DLL_96M     = 5,
    SYS_CLK_8M          = 6,
    SYS_CLK_4M          = 7,
    SYS_CLK_NUM         = 8,
} sysclk_t;

typedef enum
{
    HCLK_CHANGE = 0,
    AP_CLK_CHANGE = 1,
    CP_CLK_CHANGE = 2,
} clk_update_Type_t;

typedef struct _clk_Evt_t
{
    uint8_t   flag;
} clk_Evt_t;

typedef void (*clk_Hdl_t)(clk_Evt_t *pev);

typedef struct _clk_Contex_t
{
    bool      enable;
    clk_Hdl_t evt_handler;
} clk_Ctx_t;

#define   CLAER_RTC_COUNT   AP_AON->RTCCTL |= BIT(1)
#define   RUN_RTC           AP_AON->RTCCTL |= BIT(0)
#define   STOP_RTC          AP_AON->RTCCTL &= ~BIT(0)

#define hal_system_init clk_init
extern volatile uint32_t  g_hclk;
#define  clk_get_hclk()   g_hclk
uint32_t clk_get_pclk(void);

void hal_clk_gate_enable(MODULE_e module);
void hal_clk_gate_disable(MODULE_e module);
int hal_clk_gate_get(MODULE_e module);
void hal_clk_get_modules_state(uint32_t *buff);
void hal_clk_reset(MODULE_e module);
void hal_clk_rf_config(ClkSrc_e sel);
void hal_clk_rxadc_config(ClkSrc_e sel);

bool hal_clk_set_pclk(uint32_t div);
int hal_clk_init(sysclk_t hclk_sel, clk_Hdl_t evt_handler);
void hal_rtc_clock_config(CLK32K_e clk32Mode);

uint32_t hal_systick(void);
uint32_t hal_ms_intv(uint32_t tick);

extern uint32_t rtc_get_counter(void);
void WaitMs(uint32_t msecond);
void WaitUs(uint32_t wtTime);
void hal_system_soft_reset(void);

extern int clk_init(sysclk_t h_system_clk_sel);
extern void WaitRTCCount(uint32_t rtcDelyCnt);
extern int clk_spif_ref_clk(sysclk_t spif_ref_sel);
extern uint32_t getMcuPrecisionCount(void);

#ifdef __cplusplus
}
#endif

#endif
