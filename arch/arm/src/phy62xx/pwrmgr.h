/**************************************************************************************************

    Phyplus Microelectronics Limited confidential and proprietary.
    All rights reserved.

    IMPORTANT: All rights of this software belong to Phyplus Microelectronics
    Limited ("Phyplus"). Your use of this Software is limited to those
    specific rights granted under  the terms of the business contract, the
    confidential agreement, the non-disclosure agreement and any other forms
    of agreements as a customer or a partner of Phyplus. You may not use this
    Software unless you agree to abide by the terms of these agreements.
    You acknowledge that the Software may not be modified, copied,
    distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy
    (BLE) integrated circuit, either as a product or is integrated into your
    products.  Other than for the aforementioned purposes, you may not use,
    reproduce, copy, prepare derivative works of, modify, distribute, perform,
    display or sell this Software and/or its documentation for any purposes.

    YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
    PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
    INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
    NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
    PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
    NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
    LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
    OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
    OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

**************************************************************************************************/



#ifndef _HAL_PWRMGR_HD
#define _HAL_PWRMGR_HD

#ifdef __cplusplus
extern "C" {
#endif


#include "bus_dev.h"
#include "gpio.h"

#define PWR_MODE_NO_SLEEP           1
#define PWR_MODE_SLEEP              2
#define PWR_MODE_PWROFF_NO_SLEEP    4

//WAKEUP FROM STANDBY MODE
#define WAKEUP_PIN_MAX   3

#define HAL_PWRMGR_TASK_MAX_NUM     10

#define   RET_SRAM0         BIT(0)  /*32K, 0x1fff0000~0x1fff7fff*/
#define   RET_SRAM1         BIT(1)  /*16K, 0x1fff8000~0x1fffbfff*/
#define   RET_SRAM2         BIT(2)  /*16K, 0x1fffc000~0x1fffffff*/

#define   DEF_CLKG_CONFIG_0       (_CLK_IOMUX|_CLK_UART0|_CLK_GPIO|_CLK_SPIF)

#define   DEF_CLKG_CONFIG_1       (_CLK_M0_CPU | _CLK_BB |_CLK_TIMER |_CLK_BBREG \
                                   |_CLK_TIMER1|_CLK_TIMER2|_CLK_TIMER3|_CLK_TIMER4|_CLK_COM)

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
int hal_pwrmgr_register(MODULE_e mod, pwrmgr_Hdl_t sleepHandle, pwrmgr_Hdl_t wakeupHandle);
int hal_pwrmgr_unregister(MODULE_e mod);
int hal_pwrmgr_wakeup_process(void) __attribute__((weak));
int hal_pwrmgr_sleep_process(void) __attribute__((weak));
int hal_pwrmgr_RAM_retention(uint32_t sram);
int hal_pwrmgr_clk_gate_config(MODULE_e module);
int hal_pwrmgr_RAM_retention_clr(void);
int hal_pwrmgr_RAM_retention_set(void);
int hal_pwrmgr_LowCurrentLdo_enable(void);
int hal_pwrmgr_LowCurrentLdo_disable(void);

void hal_pwrmgr_poweroff(pwroff_cfg_t* pcfg, uint8_t wakeup_pin_num);
void hal_pwrmgr_enter_standby(pwroff_cfg_t* pcfg,uint8_t wakeup_pin_num) ;

#ifdef __cplusplus
}
#endif


#endif


