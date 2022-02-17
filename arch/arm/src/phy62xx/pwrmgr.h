/****************************************************************************
 * arch/arm/src/phy62xx/pwrmgr.h
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

#ifndef _HAL_PWRMGR_HD
#define _HAL_PWRMGR_HD

#ifdef __cplusplus
extern "C"
{
#endif

#include "bus_dev.h"
#include "gpio.h"

#define PWR_MODE_NO_SLEEP           1
#define PWR_MODE_SLEEP              2
#define PWR_MODE_PWROFF_NO_SLEEP    4

/* WAKEUP FROM STANDBY MODE */

#define WAKEUP_PIN_MAX   3

#define HAL_PWRMGR_TASK_MAX_NUM     10

#define RET_SRAM0         BIT(0)  /* 32K, 0x1fff0000~0x1fff7fff */
#define RET_SRAM1         BIT(1)  /* 16K, 0x1fff8000~0x1fffbfff */
#define RET_SRAM2         BIT(2)  /* 16K, 0x1fffc000~0x1fffffff */

#define DEF_CLKG_CONFIG_0  (_CLK_IOMUX | _CLK_UART0 | _CLK_GPIO | _CLK_SPIF)

#define DEF_CLKG_CONFIG_1  (_CLK_M0_CPU | _CLK_BB |_CLK_TIMER |_CLK_BBREG \
                           | _CLK_TIMER1 | _CLK_TIMER2 | _CLK_TIMER3 | \
                           _CLK_TIMER4 | _CLK_COM)
typedef struct
{
  gpio_pin_e pin;
  gpio_polarity_e type;
  uint16_t on_time;
} pwroff_cfg_t;

extern uint32_t g_system_reset_cause;

typedef void (*pwrmgr_Hdl_t)(void);

int hal_pwrmgr_init(void);
bool hal_pwrmgr_is_lock(MODULE_e mod);
int hal_pwrmgr_lock(MODULE_e mod);
int hal_pwrmgr_unlock(MODULE_e mod);
int hal_pwrmgr_register(MODULE_e mod, pwrmgr_Hdl_t sleepHandle,
    pwrmgr_Hdl_t wakeupHandle);
int hal_pwrmgr_unregister(MODULE_e mod);
int hal_pwrmgr_wakeup_process(void) __attribute__((weak));
int hal_pwrmgr_sleep_process(void) __attribute__((weak));
int hal_pwrmgr_RAM_retention(uint32_t sram);
int hal_pwrmgr_clk_gate_config(MODULE_e module);
int hal_pwrmgr_RAM_retention_clr(void);
int hal_pwrmgr_RAM_retention_set(void);
int hal_pwrmgr_LowCurrentLdo_enable(void);
int hal_pwrmgr_LowCurrentLdo_disable(void);

void hal_pwrmgr_poweroff(pwroff_cfg_t *pcfg, uint8_t wakeup_pin_num);
void hal_pwrmgr_enter_standby(pwroff_cfg_t *pcfg, uint8_t wakeup_pin_num);

#ifdef __cplusplus
}
#endif

#endif
