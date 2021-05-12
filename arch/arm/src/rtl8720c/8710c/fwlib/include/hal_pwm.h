/**************************************************************************//**
 * @file     hal_pwm.h
 * @brief    The HAL API implementation for the PWM device.
 * @version  V1.00
 * @date     2016-07-15
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#ifndef _HAL_PWM_H_
#define _HAL_PWM_H_
#include "cmsis.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/**
 * @addtogroup hs_hal_pwm PWM
 * @ingroup 8710c_hal
 * @{
 * @brief The PWM HAL module of the AmebaZ2 platform.
 */

/**
  * @brief The stubs functions table to exports PWM HAL functions in ROM.
  */

extern const hal_pwm_func_stubs_t hal_pwm_stubs;


/**
 *  @brief To read the PWM devives enable status
 *
 *  @param[in] ppwm_com_adp The PWM devices common adapter.
 *
 *  @returns The PWM devices enable status, bit 0 ~ 7 map to PWM0 ~ PWM7.
 */
__STATIC_INLINE
u32 hal_pwm_comm_enable_sts (hal_pwm_comm_adapter_t *ppwm_com_adp)
{
    return ppwm_com_adp->base_addr->enable_status;
}

/**
 *  @brief To update a new PWM timing setting to HW register.
 *
 *  @param[in] ppwm_adp The PWM device adapter.
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_pwm_update_ctrl (hal_pwm_adapter_t *ppwm_adp)
{
    // assert the ctrl_set bit to trigger the HW to fetch the new setting,
    // HW will clear this ctrl_set bit when the update is done
    ppwm_adp->base_addr->ctrl_b.ctrl_set = 1;
}

/**
 *  @brief To set the period time of the PWM on duty.
 *
 *  @param[in] ppwm_adp The PWM device adapter.
 *  @param[in] duty_ticks The number of ticks: the time period of the on duty.
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_pwm_set_duty_size (hal_pwm_adapter_t *ppwm_adp, u32 duty_ticks)
{
    ppwm_adp->base_addr->ctrl_b.duty = duty_ticks;
}

/**
 *  @brief To set the start offset of the PWM on duty.
 *
 *  @param[in] ppwm_adp The PWM device adapter.
 *  @param[in] start_ticks The on duty start offset time, in ticks.
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_pwm_set_onduty_start (hal_pwm_adapter_t *ppwm_adp, u32 start_ticks)
{
    ppwm_adp->base_addr->timing_ctrl_b.duty_start = start_ticks;
}

/**
 *  @brief To set the period time of a PWM cycle.
 *
 *  @param[in] ppwm_adp The PWM device adapter.
 *  @param[in] period_ticks The period time of the PWM cycle.
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_pwm_set_period (hal_pwm_adapter_t *ppwm_adp, u32 period_ticks)
{
    ppwm_adp->base_addr->timing_ctrl_b.period = period_ticks;
}

/**
 *  @brief To pause a PWM out. When pausing a PWM, the PWM HW will stop at the end of
 *         current period.
 *
 *  @param[in] ppwm_adp The PWM device adapter.
 *  @param[in] paus_ctrl The pause control, 0: un-paus the PWM, 1: pause the PWM.
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_pwm_pause (hal_pwm_adapter_t *ppwm_adp, BOOL pause_ctrl)
{
    ppwm_adp->base_addr->ctrl_b.pause = pause_ctrl;
}

/**
 *  @brief To get the status of the PWM pause state.
 *
 *  @param[in] ppwm_adp The PWM device adapter.
 *
 *  @return     0:  PWM is paused.
 *  @return     1:  PWM still is running.
 */
__STATIC_INLINE
u8 hal_pwm_get_run_sts (hal_pwm_adapter_t *ppwm_adp)
{
    return ppwm_adp->base_addr->ctrl_b.run_sts;
}

/**
 *  @brief To enable or disable the PWM period end interrupt.
 *
 *  @param[in] ppwm_adp The PWM device adapter.
 *  @param[in] int_en The interrupt enable control, disable (0) / enable (1).
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_pwm_set_period_ie (hal_pwm_adapter_t *ppwm_adp, BOOL int_en)
{
    ppwm_adp->base_addr->ctrl_b.period_ie = int_en;
}

/**
 *  @brief To set the duty size up-limit of the duty auto-adjustment.
 *
 *  @param[in] ppwm_adp The PWM device adapter.
 *  @param[in] max_duty_tick The up-limit duty size, in number of ticks.
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_pwm_set_max_duty (hal_pwm_adapter_t *ppwm_adp, u32 max_duty_tick)
{
    ppwm_adp->base_addr->auto_adj_limit_b.duty_adj_up_lim = max_duty_tick;
}

/**
 *  @brief To set the duty size down-limit of the duty auto-adjustment.
 *
 *  @param[in] ppwm_adp The PWM device adapter.
 *  @param[in] min_duty_tick The down-limit duty size, in number of ticks.
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_pwm_set_min_duty (hal_pwm_adapter_t *ppwm_adp, u32 min_duty_tick)
{
    ppwm_adp->base_addr->auto_adj_limit_b.duty_adj_dn_lim = min_duty_tick;
}

/**
 *  @brief To set the increasing step size of the PWM duty auto-adjustment.
 *
 *  @param[in] ppwm_adp The PWM device adapter.
 *  @param[in] inc_step_ticks The step size to increase on every duty auto-adjustment, in ticks
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_pwm_set_duty_inc_step (hal_pwm_adapter_t *ppwm_adp, u32 inc_step_ticks)
{
    ppwm_adp->base_addr->auto_adj_ctrl_b.duty_inc_step = inc_step_ticks;
}

/**
 *  @brief To set the decreasing step size of the PWM duty auto-adjustment.
 *
 *  @param[in] ppwm_adp The PWM device adapter.
 *  @param[in] dec_step_ticks The step size to decrease on every duty auto-adjustment, in ticks
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_pwm_set_duty_dec_step (hal_pwm_adapter_t *ppwm_adp, u32 dec_step_ticks)
{
    ppwm_adp->base_addr->auto_adj_ctrl_b.duty_dec_step = dec_step_ticks;
}

/**
 *  @brief To set the time period (number of PWM period) to do a PWM duty auto-adjustment
 *
 *  @param[in] ppwm_adp The PWM device adapter.
 *  @param[in] cycle_cnt The number of PWM period.
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_pwm_set_duty_adj_cycle (hal_pwm_adapter_t *ppwm_adp, u32 cycle_cnt)
{
    ppwm_adp->base_addr->auto_adj_cycle_b.adj_cycles = cycle_cnt;
}

/**
 *  @brief To enable or disable the PWM duty auto-adjustment HW.
 *
 *  @param[in] ppwm_adp The PWM device adapter.
 *  @param[in] enable The duty auto-adjustment enable control (0: disable, 1: enable)
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_pwm_set_duty_adj_en (hal_pwm_adapter_t *ppwm_adp, BOOLEAN enable)
{
    ppwm_adp->base_addr->auto_adj_ctrl_b.adj_en = enable;
}

/**
 *  @brief To set the PWM duty auto-adjustment direction, increase or decreas.
 *
 *  @param[in] ppwm_adp The PWM device adapter.
 *  @param[in] dir The duty adjustment direction, 0: decrease, 1: encrease.
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_pwm_set_duty_adj_dir (hal_pwm_adapter_t *ppwm_adp, pwm_duty_adj_dir_t dir)
{
    ppwm_adp->base_addr->auto_adj_ctrl_b.adj_dir = dir;
}

/**
 *  @brief To set the PWM duty auto-adjustment loop mode.
 *
 *  @param[in] ppwm_adp The PWM device adapter.
 *  @param[in] loop_en The duty auto-adjustment loop mode enable, 0: disable, 1: enable.
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_pwm_set_duty_loop_mode (hal_pwm_adapter_t *ppwm_adp, u8 loop_en)
{
    ppwm_adp->base_addr->auto_adj_ctrl_b.adj_loop_en = loop_en;
}

/**
 *  @brief To enable or disable the interrupt of the adjusted duty meet the up-limit.
 *
 *  @param[in] ppwm_adp The PWM device adapter.
 *  @param[in] int_en The interrupt enable, 0: disable, 1: enable.
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_pwm_set_max_duty_int (hal_pwm_adapter_t *ppwm_adp, u8 int_en)
{
    ppwm_adp->base_addr->auto_adj_ctrl_b.duty_up_lim_ie = int_en;
}

/**
 *  @brief To enable or disable the interrupt of the adjusted duty meet the down-limit.
 *
 *  @param[in] ppwm_adp The PWM device adapter.
 *  @param[in] int_en The interrupt enable, 0: disable, 1: enable.
 *
 *  @returns void
 */
__STATIC_INLINE
void hal_pwm_set_min_duty_int (hal_pwm_adapter_t *ppwm_adp, u8 int_en)
{
    ppwm_adp->base_addr->auto_adj_ctrl_b.duty_dn_lim_ie = int_en;
}

void hal_pwm_comm_irq_reg (irq_handler_t irq_handler);
void hal_pwm_comm_irq_unreg (void);
void hal_pwm_comm_init (hal_pwm_comm_adapter_t *ppwm_com_adp);
void hal_pwm_comm_tick_source_list (uint8_t *timer_list);
hal_status_t hal_pwm_init (hal_pwm_adapter_t *ppwm_adp, pin_name_t pin_name, u16 duty_res_us);
void hal_pwm_deinit (hal_pwm_adapter_t *ppwm_adp);
BOOLEAN hal_pwm_enable_sts (hal_pwm_adapter_t *ppwm_adp);
void hal_pwm_comm_enable (u32 en_ctrl);
void hal_pwm_comm_disable (u32 dis_ctrl);
void hal_pwm_enable (hal_pwm_adapter_t *ppwm_adp);
void hal_pwm_disable (hal_pwm_adapter_t *ppwm_adp);
void hal_pwm_set_clk_sel (hal_pwm_adapter_t *ppwm_adp, pwm_clk_sel_t clk_sel);
void hal_pwm_wait_ctrl_ready (hal_pwm_adapter_t *ppwm_adp);
hal_status_t hal_pwm_set_tick_time (hal_pwm_adapter_t *ppwm_adp, u32 tick_p5us);
hal_status_t hal_pwm_set_duty (hal_pwm_adapter_t *ppwm_adp, u32 period_us, \
                                  u32 duty_us, u32 start_offset_us);
hal_status_t hal_pwm_set_duty_ns (hal_pwm_adapter_t *ppwm_adp, u32 period_ns, \
                                  u32 duty_ns, u32 start_offset_ns);

u32 hal_pwm_read_duty (hal_pwm_adapter_t *ppwm_adp);
void hal_pwm_change_duty (hal_pwm_adapter_t *ppwm_adp, u32 duty_us);
hal_status_t hal_pwm_set_duty_limit (hal_pwm_adapter_t *ppwm_adp, u32 max_duty_us, u32 min_duty_us);
void hal_pwm_set_auto_duty_adj (hal_pwm_adapter_t *ppwm_adp, hal_pwm_auto_duty_adj_t *pauto_duty);
void hal_pwm_auto_duty_en (hal_pwm_adapter_t *ppwm_adp, BOOLEAN enable);
hal_status_t hal_pwm_set_auto_duty_inc (hal_pwm_adapter_t *ppwm_adp, u32 max_duty_us, \
                                                u32 step_sz_us, u32 step_period_cnt);
hal_status_t hal_pwm_set_auto_duty_dec (hal_pwm_adapter_t *ppwm_adp, u32 min_duty_us, \
                                                u32 step_sz_us, u32 step_period_cnt);
hal_status_t hal_pwm_set_auto_duty_loop (hal_pwm_adapter_t *ppwm_adp, u8 ini_dir, u32 loop_cnt);
void hal_pwm_set_period_int (hal_pwm_adapter_t *ppwm_adp, pwm_period_callback_t callback, void *arg, u8 int_en);
void hal_pwm_set_autoadj_int (hal_pwm_adapter_t *ppwm_adp, pwm_lim_callback_t callback, void *arg, u8 int_en);
void hal_pwm_set_autoadj_loop_int (hal_pwm_adapter_t *ppwm_adp, pwm_lo_callback_t callback, void *arg);
hal_status_t hal_pwm_auto_duty_inc (hal_pwm_adapter_t *ppwm_adp, u32 max_duty_us, u32 step_sz_us, \
                                        u32 step_period_cnt);
hal_status_t hal_pwm_auto_duty_dec (hal_pwm_adapter_t *ppwm_adp, u32 min_duty_us, \
                                                u32 step_sz_us, u32 step_period_cnt);
hal_status_t hal_pwm_auto_duty_loop (hal_pwm_adapter_t *ppwm_adp, u32 ini_duty_us, u8 ini_dir, u32 loop_cnt);
void hal_pwm_stop_duty_loop (hal_pwm_adapter_t *ppwm_adp, u8 stop_now);

hal_status_t hal_pwm_auto_duty_ns_inc (hal_pwm_adapter_t *ppwm_adp, u32 max_duty_ns,
                                    u32 step_sz_ns, u32 step_period_cnt);
hal_status_t hal_pwm_auto_duty_ns_dec (hal_pwm_adapter_t *ppwm_adp, u32 min_duty_ns,
                                    u32 step_sz_ns, u32 step_period_cnt);
/** @} */ /* End of group hs_hal_pwm */

#ifdef  __cplusplus
}
#endif


#endif  // end of "#define _HAL_PWM_H_"

