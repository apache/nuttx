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
#ifndef __TIMER_H__
#define __TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"
#include "bus_dev.h"

#define FREE_TIMER_NUMBER 2

typedef enum
{
    AP_TIMER_ID_5 = 5,
    AP_TIMER_ID_6 = 6,

} User_Timer_e;

enum
{
    HAL_EVT_TIMER_5 = AP_TIMER_ID_5,
    HAL_EVT_TIMER_6 = AP_TIMER_ID_6,
    HAL_EVT_WAKEUP = 0x10,
    HAL_EVT_SLEEP
};

typedef void(*ap_tm_hdl_t)(uint8_t evt);

int hal_timer_init(ap_tm_hdl_t callback);

int hal_timer_deinit(void);

int hal_timer_set(User_Timer_e timeId, uint32_t us);

int hal_timer_mask_int(User_Timer_e timeId, bool en);

int hal_timer_stop(User_Timer_e timeId);

void __attribute__((used)) hal_TIMER5_IRQHandler(void);
void __attribute__((used)) hal_TIMER6_IRQHandler(void);

extern void set_timer(AP_TIM_TypeDef* TIMx, int time);

extern uint32_t read_current_fine_time(void);

extern uint32  read_LL_remainder_time(void);

#ifndef BASE_TIME_UINTS
#define BASE_TIME_UNITS   (0x3fffff)
#endif

#ifdef __cplusplus
}
#endif

#endif
