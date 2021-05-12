 /**************************************************************************//**
  * @file     rtl8710c_pwm.h
  * @brief    The HAL related definition and macros for the PWM device.
  *           Includes Registers and data type definition.
  * @version  V1.00
  * @date     2016-07-22
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

#ifndef _RTL8710C_PWM_H_
#define _RTL8710C_PWM_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include "rtl8710c_timer.h"
#include "rtl8710c_pwm_comm_type.h"
#include "rtl8710c_pwm_type.h"

/**
 * @addtogroup hs_hal_pwm PWM
 * @{
 */

/**
  * \brief define PWM ID
**/
enum  pwm_id_e {
    Pwm0           = 0, //!< PWM0
    Pwm1           = 1, //!< PWM1
    Pwm2           = 2, //!< PWM2
    Pwm3           = 3, //!< PWM3
    Pwm4           = 4, //!< PWM4
    pwm5           = 5, //!< PWM5
    pwm6           = 6, //!< PWM6
    pwm7           = 7, //!< PWM7

    MaxPwmNum      = 8  //!< MAX_PWM_NUM
};
typedef uint8_t pwm_id_t;

/**
  * \brief define PWM Limit Interrupt direction
**/
enum  pwm_limit_dir_e {
    PwmDownLimitInt      = 0,   //!< Interrupt triggered by duty ratio = down limit
    PwmUpLimitInt        = 1    //!< Interrupt triggered by duty ratio = up limit
};
typedef uint8_t pwm_limit_dir_t;

/**
  * \brief define PWM tick source selection, as the PWM control register definition
**/
enum  pwm_clk_sel_e {
    PwmClkSrc_Tm0       = GTimer0,    //!< Tick source from G-Timer 0
    PwmClkSrc_Tm1       = GTimer1,    //!< Tick source from G-Timer 1
    PwmClkSrc_Tm2       = GTimer2,    //!< Tick source from G-Timer 2
    PwmClkSrc_Tm3       = GTimer3,    //!< Tick source from G-Timer 3
    PwmClkSrc_Tm4       = GTimer4,    //!< Tick source from G-Timer 4
    PwmClkSrc_Tm5       = GTimer5,    //!< Tick source from G-Timer 5
    PwmClkSrc_Tm6       = GTimer6,    //!< Tick source from G-Timer 6
    PwmClkSrc_Tm7       = GTimer7,    //!< Tick source from G-Timer 7
    PwmClkSrc_SClk      = 8,         //!< Tick source from SCLK

    PwmClkSrc_None      = 0xFF //!< Not assign yet
};
typedef uint8_t pwm_clk_sel_t;

/**
  * \brief define PWM duty auto-adjustment direction
**/
enum  pwm_duty_adj_dir_e {
    PwmDutyAdj_Decrease     = 0,        //!< Increase the duty
    PwmDutyAdj_Increase     = 1         //!< decrease the duty
};
typedef uint8_t pwm_duty_adj_dir_t;

/**
  * \brief define PWM Interrupt ID
**/
enum  pwm_adj_int_e {
    PwmAdjIntDnLim = 1, //!< Interrupt ID = down limit
    PwmAdjIntUpLim = 2, //!< Interrupt ID = up limit
};

/**
  * \brief define PWM Duty Auto-Adjustment loop mode
**/
enum  pwm_duty_loop_mode_e {
    PwmDutyLoopOff = 0,     //!< Interrupt loop mode = loop off
    PwmDutyLoopCnt = 1,     //!< Interrupt ID = loop cnt
    PwmDutyLoopForever = 2  //!< Interrupt ID = loop forever
};

/**
  * \brief define PWM Interrupt clock source
**/
enum  pwm_int_clk_e {
    PWM_IntClk_APB      = 0,    //!< APB
    PWM_IntClk_32K      = 1     //!< 32k
};
typedef uint8_t pwm_int_clk_t;

#define PWM_CURRENT_DUTY            0xFFFFFFFF


/*! define PWM interrupt call back function */
typedef void (*pwm_lim_callback_t) (void *, pwm_limit_dir_t dir);
typedef void (*pwm_lo_callback_t) (void *);
typedef void (*pwm_period_callback_t) (void *);

/**
  * \brief  The data structure to control the PWM duty auto adjstment.
*/
typedef struct hal_pwm_auto_duty_adj_s {
    u32 init_duty_us;   /*!< The initial duty size of the PWM to start a duty auto-adjustment, in us */
    u32 max_duty_us;   /*!< The maximum duty size of the duty auto-adjustment, in us */
    u32 min_duty_us;   /*!< The minium duty size of the duty auto-adjustment, in us */
    u32 duty_inc_step_us;   /*!< The step size for the duty auto-adjustment increasing, in us */
    u32 duty_dec_step_us;   /*!< The step size for the duty auto-adjustment decreasing, in us */
    u32 step_period_cnt;   /*!< The number of PWM period to perform a duty auto-adjustment */
    u8 init_dir;    /*!< The initial duty auto-adjustment direction */
    u8 loop_mode;   /*!< Is enable the duty auto-adjustment loop mode */
} hal_pwm_auto_duty_adj_t, *phal_pwm_auto_duty_adj_t;

/**
  * \brief  The data structure to handle a PWM device, it includes the configuration, register base address
          and interrupt call back functions.
*/
typedef struct hal_pwm_adapter_s {
    PWM0_Type *base_addr;      /*!< The PWM registers base address */
    pwm_id_t pwm_id;   /*! The PWM device index, 0 ~ 7 */
    pin_name_t pin_name;  /*! IO pin name for this PWM */
    pwm_clk_sel_t pwm_clk_sel;  /*! The PWM tick source selection, 0 ~ 8 */
    u8 adj_int_en;       /*! The duty ratio auto adjustment interrupt enable control */
    u16 duty_res_us;     /*! minimum resolution for the PWM duty, in us */
    u32 adj_loop_count;    /*! The duty ratio auto adjustment loop count */
    u32 tick_p5us;    /*! The tick time of this PWM, the unit of tick time is 0.5us */
    u32 period_us;    /*! The period time of current PWM cycle, in us */
    u32 duty_us;      /*! The duty time of current PWM cycle, in us */
    hal_pwm_auto_duty_adj_t duty_adj; /*! The data structure to control the PWM duty auto adjstment. */

    pwm_lim_callback_t bound_callback;      /*! User callback function for duty ratio reachs the boundary limit */
    void *bound_cb_para;   /*! the argument for user duty size reachs the limit callback function */
    pwm_lo_callback_t loopout_callback;      /*! User callback function for duty ratio auto adjustment loop count down to 0 */
    void *lo_cb_para;   /*! the argument for duty auto-adj loopout callback function */
    pwm_period_callback_t period_callback;      /*! User callback function for PWM period time end */
    void *pe_cb_para;   /*! the argument for PWM period end callback function */
    void (*enter_critical)(void);   /*! the function for entering timer critical state */
    void (*exit_critical)(void);    /*! the function for exit timer critical state */
} hal_pwm_adapter_t, *phal_pwm_adapter_t;

/**
  \brief  The data structure to handle PWM devices common part, includes the base address of common registers
          and PWM adapters for interrupt process.
*/
typedef struct hal_pwm_comm_adapter_s {
    PWM_COMM_Type *base_addr;      /*!< The PWM common register base address */
    hal_pwm_adapter_t *pwm_adapter[MaxPwmNum];   /*!< All the PWM adapters share the same interrupt IRQ */
    uint8_t *timer_list;    /*!< the list of avaliable timer as the PWM tick source, keep NULL for no assign */
} hal_pwm_comm_adapter_t, *phal_pwm_comm_adapter_t;

/**
  \brief  The data structure of the stubs function for the PWM HAL functions in ROM
*/
typedef struct hal_pwm_func_stubs_s {
    phal_pwm_comm_adapter_t *pppwm_comm_adp;
    io_pin_t *pwm_pin_table;
    void (*hal_pwm_irq_handler) (void);
    void (*hal_pwm_comm_irq_reg) (irq_handler_t irq_handler);
    void (*hal_pwm_comm_irq_unreg) (void);
    void (*hal_pwm_comm_init) (hal_pwm_comm_adapter_t *ppwm_com_adp);
    void (*hal_pwm_comm_deinit) (void);
    void (*hal_pwm_comm_tick_source_list) (uint8_t *timer_list);
    hal_status_t (*hal_pwm_init) (hal_pwm_adapter_t *ppwm_adp, pin_name_t pin_name, u16 duty_res_us);

    BOOLEAN (*hal_pwm_enable_sts) (hal_pwm_adapter_t *ppwm_adp);
    void (*hal_pwm_comm_enable) (u32 en_ctrl);
    void (*hal_pwm_comm_disable) (u32 dis_ctrl);
    void (*hal_pwm_enable) (hal_pwm_adapter_t *ppwm_adp);
    void (*hal_pwm_disable) (hal_pwm_adapter_t *ppwm_adp);
    void (*hal_pwm_deinit) (hal_pwm_adapter_t *ppwm_adp);
    void (*hal_pwm_set_clk_sel) (hal_pwm_adapter_t *ppwm_adp, pwm_clk_sel_t clk_sel);
    void (*hal_pwm_wait_ctrl_ready) (hal_pwm_adapter_t *ppwm_adp);
    hal_status_t (*hal_pwm_set_tick_time) (hal_pwm_adapter_t *ppwm_adp, u32 tick_p5us);
    hal_status_t (*hal_pwm_set_duty) (hal_pwm_adapter_t *ppwm_adp, u32 period_us, \
                                      u32 duty_us, u32 start_offset_us);
    u32 (*hal_pwm_read_duty) (hal_pwm_adapter_t *ppwm_adp);
    void (*hal_pwm_change_duty) (hal_pwm_adapter_t *ppwm_adp, u32 duty_us);
    hal_status_t (*hal_pwm_set_duty_limit) (hal_pwm_adapter_t *ppwm_adp, u32 max_duty_us, u32 min_duty_us);
    void (*hal_pwm_set_auto_duty_adj) (hal_pwm_adapter_t *ppwm_adp, hal_pwm_auto_duty_adj_t *pauto_duty);
    void (*hal_pwm_auto_duty_en) (hal_pwm_adapter_t *ppwm_adp, BOOLEAN enable);
    hal_status_t (*hal_pwm_set_auto_duty_inc) (hal_pwm_adapter_t *ppwm_adp, u32 max_duty_us, \
                                                    u32 step_sz_us, u32 step_period_cnt);
    hal_status_t (*hal_pwm_set_auto_duty_dec) (hal_pwm_adapter_t *ppwm_adp, u32 min_duty_us, \
                                                    u32 step_sz_us, u32 step_period_cnt);
    hal_status_t (*hal_pwm_set_auto_duty_loop) (hal_pwm_adapter_t *ppwm_adp, u8 ini_dir, u32 loop_cnt);
    void (*hal_pwm_set_period_int) (hal_pwm_adapter_t *ppwm_adp, pwm_period_callback_t callback, void *arg, u8 int_en);
    void (*hal_pwm_set_autoadj_int) (hal_pwm_adapter_t *ppwm_adp, pwm_lim_callback_t callback, void *arg, u8 int_en);
    void (*hal_pwm_set_autoadj_loop_int) (hal_pwm_adapter_t *ppwm_adp, pwm_lo_callback_t callback, void *arg);
    hal_status_t (*hal_pwm_auto_duty_inc) (hal_pwm_adapter_t *ppwm_adp, u32 max_duty_us, u32 step_sz_us, \
                                            u32 step_period_cnt);
    hal_status_t (*hal_pwm_auto_duty_dec) (hal_pwm_adapter_t *ppwm_adp, u32 min_duty_us, \
                                                    u32 step_sz_us, u32 step_period_cnt);
    hal_status_t (*hal_pwm_auto_duty_loop) (hal_pwm_adapter_t *ppwm_adp, u32 ini_duty_us, u8 ini_dir, u32 loop_cnt);
    void (*hal_pwm_stop_duty_loop) (hal_pwm_adapter_t *ppwm_adp, u8 stop_now);
    hal_status_t (*hal_pwm_set_duty_ns) (hal_pwm_adapter_t *ppwm_adp, u32 period_ns, u32 duty_ns, u32 start_offset_ns);
    hal_status_t (*hal_pwm_auto_duty_ns_inc) (hal_pwm_adapter_t *ppwm_adp, u32 max_duty_ns, u32 step_sz_ns, u32 step_period_cnt);
    hal_status_t (*hal_pwm_auto_duty_ns_dec) (hal_pwm_adapter_t *ppwm_adp, u32 min_duty_ns, u32 step_sz_ns, u32 step_period_cnt);
    uint32_t reserved[13];  // reserved space for next ROM code version function table extending.
} hal_pwm_func_stubs_t;

/// @cond DOXYGEN_ROM_HAL_API

/**
 * @addtogroup hs_hal_pwm_rom_func PWM HAL ROM APIs.
 * @{
 */

/**
 *  \brief To read the PWM devives enable status
 *
 *  \param[in] ppwm_com_adp The PWM devices common adapter.
 *
 *  \returns The PWM devices enable status, bit 0 ~ 7 map to PWM0 ~ PWM7.
 */
__STATIC_INLINE
u32 hal_pwm_comm_enable_sts_rtl8710c (hal_pwm_comm_adapter_t *ppwm_com_adp)
{
    return ppwm_com_adp->base_addr->enable_status;
}

/**
 *  \brief To update a new PWM timing setting to HW register.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *
 *  \returns void
 */
__STATIC_INLINE
void hal_pwm_update_ctrl_rtl8710c (hal_pwm_adapter_t *ppwm_adp)
{
    // assert the ctrl_set bit to trigger the HW to fetch the new setting,
    // HW will clear this ctrl_set bit when the update is done
    ppwm_adp->base_addr->ctrl_b.ctrl_set = 1;
}

/**
 *  \brief To set the period time of the PWM on duty.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] duty_ticks The number of ticks: the time period of the on duty.
 *
 *  \returns void
 */
__STATIC_INLINE
void hal_pwm_set_duty_size_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u32 duty_ticks)
{
    ppwm_adp->base_addr->ctrl_b.duty = duty_ticks;
}

/**
 *  \brief To set the start offset of the PWM on duty.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] start_ticks The on duty start offset time, in ticks.
 *
 *  \returns void
 */
__STATIC_INLINE
void hal_pwm_set_onduty_start_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u32 start_ticks)
{
    ppwm_adp->base_addr->timing_ctrl_b.duty_start = start_ticks;
}

/**
 *  \brief To set the period time of a PWM cycle.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] period_ticks The period time of the PWM cycle.
 *
 *  \returns void
 */
__STATIC_INLINE
void hal_pwm_set_period_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u32 period_ticks)
{
    ppwm_adp->base_addr->timing_ctrl_b.period = period_ticks;
}

/**
 *  \brief To pause a PWM out. When pausing a PWM, the PWM HW will stop at the end of
 *         current period.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] pause_ctrl The pause control, 0: un-pause the PWM, 1: pause the PWM.
 *
 *  \returns void
 */
__STATIC_INLINE
void hal_pwm_pause_rtl8710c (hal_pwm_adapter_t *ppwm_adp, BOOL pause_ctrl)
{
    ppwm_adp->base_addr->ctrl_b.pause = pause_ctrl;
}

/**
 *  \brief To get the status of the PWM pause state.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *
 *  \return     0:  PWM is paused.
 *  \return     1:  PWM still is running.
 */
__STATIC_INLINE
u8 hal_pwm_get_run_sts_rtl8710c (hal_pwm_adapter_t *ppwm_adp)
{
    return ppwm_adp->base_addr->ctrl_b.run_sts;
}

/**
 *  \brief To enable or disable the PWM period end interrupt.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] int_en The interrupt enable control, disable (0) / enable (1).
 *
 *  \returns void
 */
__STATIC_INLINE
void hal_pwm_set_period_ie_rtl8710c (hal_pwm_adapter_t *ppwm_adp, BOOL int_en)
{
    ppwm_adp->base_addr->ctrl_b.period_ie = int_en;
}

/**
 *  \brief To set the duty size up-limit of the duty auto-adjustment.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] max_duty_tick The up-limit duty size, in number of ticks.
 *
 *  \returns void
 */
__STATIC_INLINE
void hal_pwm_set_max_duty_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u32 max_duty_tick)
{
    ppwm_adp->base_addr->auto_adj_limit_b.duty_adj_up_lim = max_duty_tick;
}

/**
 *  \brief To set the duty size down-limit of the duty auto-adjustment.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] min_duty_tick The down-limit duty size, in number of ticks.
 *
 *  \returns void
 */
__STATIC_INLINE
void hal_pwm_set_min_duty_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u32 min_duty_tick)
{
    ppwm_adp->base_addr->auto_adj_limit_b.duty_adj_dn_lim = min_duty_tick;
}

/**
 *  \brief To set the increasing step size of the PWM duty auto-adjustment.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] inc_step_ticks The step size to increase on every duty auto-adjustment, in ticks
 *
 *  \returns void
 */
__STATIC_INLINE
void hal_pwm_set_duty_inc_step_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u32 inc_step_ticks)
{
    ppwm_adp->base_addr->auto_adj_ctrl_b.duty_inc_step = inc_step_ticks;
}

/**
 *  \brief To set the decreasing step size of the PWM duty auto-adjustment.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] dec_step_ticks The step size to decrease on every duty auto-adjustment, in ticks
 *
 *  \returns void
 */
__STATIC_INLINE
void hal_pwm_set_duty_dec_step_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u32 dec_step_ticks)
{
    ppwm_adp->base_addr->auto_adj_ctrl_b.duty_dec_step = dec_step_ticks;
}

/**
 *  \brief To set the time period (number of PWM period) to do a PWM duty auto-adjustment
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] cycle_cnt The number of PWM period.
 *
 *  \returns void
 */
__STATIC_INLINE
void hal_pwm_set_duty_adj_cycle_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u32 cycle_cnt)
{
    ppwm_adp->base_addr->auto_adj_cycle_b.adj_cycles = cycle_cnt;
}

/**
 *  \brief To enable or disable the PWM duty auto-adjustment HW.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] enable The duty auto-adjustment enable control (0: disable, 1: enable)
 *
 *  \returns void
 */
__STATIC_INLINE
void hal_pwm_set_duty_adj_en_rtl8710c (hal_pwm_adapter_t *ppwm_adp, BOOLEAN enable)
{
    ppwm_adp->base_addr->auto_adj_ctrl_b.adj_en = enable;
}

/**
 *  \brief To set the PWM duty auto-adjustment direction, increase or decreas.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] dir The duty adjustment direction, 0: decrease, 1: encrease.
 *
 *  \returns void
 */
__STATIC_INLINE
void hal_pwm_set_duty_adj_dir_rtl8710c (hal_pwm_adapter_t *ppwm_adp, pwm_duty_adj_dir_t dir)
{
    ppwm_adp->base_addr->auto_adj_ctrl_b.adj_dir = dir;
}

/**
 *  \brief To set the PWM duty auto-adjustment loop mode.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] loop_en The duty auto-adjustment loop mode enable, 0: disable, 1: enable.
 *
 *  \returns void
 */
__STATIC_INLINE
void hal_pwm_set_duty_loop_mode_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u8 loop_en)
{
    ppwm_adp->base_addr->auto_adj_ctrl_b.adj_loop_en = loop_en;
}

/**
 *  \brief To enable or disable the interrupt of the adjusted duty meet the up-limit.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] int_en The interrupt enable, 0: disable, 1: enable.
 *
 *  \returns void
 */
__STATIC_INLINE
void hal_pwm_set_max_duty_int_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u8 int_en)
{
    ppwm_adp->base_addr->auto_adj_ctrl_b.duty_up_lim_ie = int_en;
}

/**
 *  \brief To enable or disable the interrupt of the adjusted duty meet the down-limit.
 *
 *  \param[in] ppwm_adp The PWM device adapter.
 *  \param[in] int_en The interrupt enable, 0: disable, 1: enable.
 *
 *  \returns void
 */
__STATIC_INLINE
void hal_pwm_set_min_duty_int_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u8 int_en)
{
    ppwm_adp->base_addr->auto_adj_ctrl_b.duty_dn_lim_ie = int_en;
}

void hal_pwm_comm_irq_reg_rtl8710c (irq_handler_t irq_handler);
void hal_pwm_comm_irq_unreg_rtl8710c (void);
void hal_pwm_limirq_reg_rtl8710c (hal_pwm_adapter_t *ppwm_adp, pwm_lim_callback_t callback, void *phid);
void hal_pwm_limirq_unreg_rtl8710c (hal_pwm_adapter_t *ppwm_adp);
void hal_pwm_comm_init_rtl8710c (hal_pwm_comm_adapter_t *ppwm_com_adp);
void hal_pwm_comm_deinit_rtl8710c (void);
void hal_pwm_comm_tick_source_list_rtl8710c (uint8_t *timer_list);
hal_status_t hal_pwm_init_rtl8710c (hal_pwm_adapter_t *ppwm_adp, pin_name_t pin_name, u16 duty_res_us);
BOOLEAN hal_pwm_enable_sts_rtl8710c (hal_pwm_adapter_t *ppwm_adp);
void hal_pwm_comm_enable_rtl8710c (u32 en_ctrl);
void hal_pwm_comm_disable_rtl8710c (u32 dis_ctrl);
void hal_pwm_enable_rtl8710c (hal_pwm_adapter_t *ppwm_adp);
void hal_pwm_disable_rtl8710c (hal_pwm_adapter_t *ppwm_adp);
void hal_pwm_deinit_rtl8710c (hal_pwm_adapter_t *ppwm_adp);
void hal_pwm_set_clk_sel_rtl8710c (hal_pwm_adapter_t *ppwm_adp, pwm_clk_sel_t clk_sel);
void hal_pwm_wait_ctrl_ready_rtl8710c (hal_pwm_adapter_t *ppwm_adp);
hal_status_t hal_pwm_set_tick_time_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u32 tick_p5us);
hal_status_t hal_pwm_set_duty_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u32 period_us,
                                            u32 duty_us, u32 start_offset_us);
u32 hal_pwm_read_duty_rtl8710c (hal_pwm_adapter_t *ppwm_adp);
void hal_pwm_change_duty_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u32 duty_us);
hal_status_t hal_pwm_set_duty_limit_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u32 max_duty_us, u32 min_duty_us);
void hal_pwm_set_auto_duty_adj_rtl8710c (hal_pwm_adapter_t *ppwm_adp, hal_pwm_auto_duty_adj_t *pauto_duty);
void hal_pwm_auto_duty_en_rtl8710c (hal_pwm_adapter_t *ppwm_adp, BOOLEAN enable);
hal_status_t hal_pwm_set_auto_duty_inc_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u32 max_duty_us,
                                                u32 step_sz_us, u32 step_period_cnt);
hal_status_t hal_pwm_set_auto_duty_dec_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u32 min_duty_us,
                                                u32 step_sz_us, u32 step_period_cnt);
hal_status_t hal_pwm_set_auto_duty_loop_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u8 ini_dir, u32 loop_cnt);
void hal_pwm_set_period_int_rtl8710c (hal_pwm_adapter_t *ppwm_adp, pwm_period_callback_t callback, \
                                        void *arg, u8 int_en);
void hal_pwm_set_autoadj_int_rtl8710c (hal_pwm_adapter_t *ppwm_adp, pwm_lim_callback_t callback, \
                                        void *arg, u8 int_en);
void hal_pwm_set_autoadj_loop_int_rtl8710c (hal_pwm_adapter_t *ppwm_adp, pwm_lo_callback_t callback, void *arg);

hal_status_t hal_pwm_auto_duty_inc_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u32 max_duty_us,
                                                u32 step_sz_us, u32 step_period_cnt);
hal_status_t hal_pwm_auto_duty_dec_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u32 min_duty_us,
                                                u32 step_sz_us, u32 step_period_cnt);
hal_status_t hal_pwm_auto_duty_loop_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u32 ini_duty_us, u8 ini_dir, u32 loop_cnt);
void hal_pwm_stop_duty_loop_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u8 stop_now);
hal_status_t hal_pwm_set_duty_ns_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u32 period_ns,
                                            u32 duty_ns, u32 start_offset_ns);
hal_status_t hal_pwm_auto_duty_ns_inc_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u32 max_duty_ns,
                                                u32 step_sz_ns, u32 step_period_cnt);
hal_status_t hal_pwm_auto_duty_ns_dec_rtl8710c (hal_pwm_adapter_t *ppwm_adp, u32 min_duty_ns,
                                                u32 step_sz_ns, u32 step_period_cnt);

/** @} */ /* End of group hs_hal_pwm_rom_func */

/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */

#ifdef  __cplusplus
}
#endif

/** @} */ /* End of group hs_hal_pwm */

#endif  // end of "#define _RTL8710C_PWM_H_"

