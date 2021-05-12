/**************************************************************************//**
 * @file     rtl8710c_misc.h
 * @brief    The rtl8710c platform HAL misc header file.
 * @version  V1.00
 * @date     2016-06-07
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

#ifndef _RTL8710C_MISC_H_
#define _RTL8710C_MISC_H_
#include "basic_types.h"
#include "stdio_port.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/**
 * @addtogroup hs_hal_misc Misc. Functions
 * @ingroup 8710c_hal
 * @{
 * @brief The HAL miscellaneous APIs provides the HAL functions for watch dog timer control, system timer utility
 *        and NMI interrupt handler register.
 */

/**
  \brief  32K clock SDM control registers. The SDM registers are accessed by indirection method.
*/
enum  sdm_reg_sel_e
{
    SDM_Ctrl0       = 0x00,   ///< REG_SDM_CTRL0
    SDM_Ctrl1       = 0x01,   ///< REG_SDM_CTRL1
    SDM_Ctrl2       = 0x02,   ///< REG_SDM_CTRL2
    SDM_Ctrl3       = 0x03,   ///< REG_SDM_CTRL3
    SDM_Ctrl4       = 0x04,   ///< REG_SDM_CTRL4
    SDM_Ctrl5       = 0x05,   ///< REG_SDM_CTRL5
    SDM_Ctrl6       = 0x06,   ///< REG_SDM_CTRL6
};
typedef uint8_t sdm_reg_sel_t;

/// The funcion mode of the WDT(watch dog timer) timeout event: Interrupt or System Reset
enum  wdt_mode_e {
    WDT_Mode_Int        = 0,    ///< Trigger the NMI interrupt on the WDT timeout event.
    WDT_Mode_Rst        = 1     ///< Invokes a system reset on the WDT timeout event.
};
typedef uint8_t wdt_mode_t;

/// The data type for the HAL misc. entity management.
typedef struct hal_misc_adapter_s {
    irq_handler_t nmi_handler;  /*!< the callback function for NMI IRQ */
    void *nmi_arg;              /*!< the application data will be back to the application
                                     with the NMI IRQ callback function */
    irq_handler_t wdt_handler;  /*!< the call-back function for the WDT timeout event */
    void *wdt_arg;              /*!< the application data will be back to the application
                                     with the WDT event callback function */

    // B-Cut & after
    irq_handler_t wdt_user_handler; /*!< the user's call-back function for the WDT timeout event */
    uint32_t wdt_timeout_us;        /*!< the WDT timeout period, in us */
    uint8_t wdt_expired;            /*!< the flag to indicated is the WDT is expired ever */

//    uint32_t reserved[20];  // reserved for RAM code extension
} hal_misc_adapter_t, *phal_misc_adapter_t;

#if (defined(ROM_REGION) && (ROM_REGION==1)) || (defined(CONFIG_BUILD_LIB) && (CONFIG_BUILD_LIB==1))

/**
 *  @brief Enables the watch dog timer.
 *
 *  @returns    void
 */
__STATIC_INLINE
void hal_misc_wdt_enable_rtl8710c (void)
{
    VDR->watch_dog_timer_b.wdt_clear = 1;
    VDR->watch_dog_timer_b.wdt_en_byte = 0xA5;
}

/**
 *  @brief Disables the watch dog timer.
 *
 *  @returns    void
 */
__STATIC_INLINE
void hal_misc_wdt_disable_rtl8710c (void)
{
    VDR->watch_dog_timer_b.wdt_en_byte = 0;
}

/**
 *  @brief Refresh(reload) the watch dog timer counter.
 *         To prevents the watch dog timer timeout event occurred.
 *
 *  @returns    void
 */
__STATIC_INLINE
void hal_misc_wdt_refresh_rtl8710c (void)
{
    VDR->watch_dog_timer_b.wdt_clear = 1;
}

#endif

/// @cond DOXYGEN_ROM_HAL_API

/**
 * @addtogroup hs_hal_misc_rom_func HAL Misc. ROM APIs.
 * @ingroup hs_hal_misc
 * @{
 */

void hal_misc_init_rtl8710c (hal_misc_adapter_t *pmisc_adp);
void hal_misc_start_systimer_rtl8710c (phal_timer_adapter_t ptimer_adp, uint32_t tmr_id,
                                            timer_cnt_mode_t cnt_md, uint32_t tick_us);
u64 hal_misc_read_systime_rtl8710c (void);
u32 hal_misc_read_curtime_rtl8710c (void);
void hal_misc_delay_us_rtl8710c (u32 time_us);
BOOLEAN hal_misc_is_timeout (u32 start_us, u32 timeout_us);
void hal_misc_wdt_set_timeout_rtl8710c (uint32_t time_us);
void hal_misc_wdt_init_rtl8710c (uint32_t time_us);
void hal_misc_wdt_reg_irq_rtl8710c (irq_handler_t handler, void *arg);
void hal_misc_nmi_reg_irq_rtl8710c (irq_handler_t handler, void *arg);
void hal_misc_rst_by_wdt_rtl8710c (void);
void hal_misc_cpu_rst_rtl8710c (void);
void hal_misc_sdm_32k_enable_rtl8710c (u8 bypass_mode);
void hal_misc_set_sdm_32k_time_loss_rtl8710c (u8 time_loss_set, u32 time_loss_reg);
u32 hal_misc_read_sdm_32k_time_loss_rtl8710c (void);
void hal_misc_bt_enable_ctrl_rtl8710c (BOOLEAN enable);

/** @} */ /* End of group hs_hal_misc_rom_func */
/// @endcond    /* End of cond DOXYGEN_ROM_HAL_API */

/**
  \brief  The structure for HAL Misc. module to export the ROM HAL APIs.
*/
typedef struct hal_misc_func_stubs_s {
    stdio_port_t *pstdio_port;
    void (*hal_misc_init) (hal_misc_adapter_t *pmisc_adp);
    void (*hal_misc_wdt_set_timeout) (uint32_t time_us);
    void (*hal_misc_wdt_init) (uint32_t time_us);
    void (*hal_misc_wdt_reg_irq) (irq_handler_t handler, void *arg);
    void (*hal_misc_nmi_reg_irq) (irq_handler_t handler, void *arg);
    void (*hal_misc_rst_by_wdt) (void);
    void (*hal_misc_cpu_rst) (void);
    void (*hal_misc_sdm_32k_enable) (u8 bypass_mode);
    u32  (*hal_misc_read_sdm_32k_time_loss) (void);
    void (*hal_misc_set_sdm_32k_time_loss) (u8 time_loss_set, u32 time_loss_reg);    
    void (*hal_misc_bt_enable_ctrl) (BOOLEAN enable);
    uint32_t reserved[16];  // reserved space for next ROM code version function table extending.
} hal_misc_func_stubs_t;


#if defined(ROM_REGION)
#define hal_read_sys_time()                     hal_misc_read_systime_rtl8710c()
#define hal_read_cur_time()                     hal_misc_read_curtime_rtl8710c()
#define hal_is_timeout(start_us, timeout_us)    hal_misc_is_timeout(start_us, timeout_us)
#define hal_delay_us(time_us)                   do {hal_misc_delay_us_rtl8710c (time_us);} while (0)
#define hal_delay_ms(time_ms)                   do {hal_misc_delay_us_rtl8710c ((time_ms)*1000);} while (0)
#else
#define hal_read_sys_time()                     hal_read_systime_us()
#define hal_read_cur_time()                     hal_read_curtime_us()
#define hal_delay_ms(time_ms)                   do {hal_delay_us((time_ms)*1000);} while (0)
#endif

/** @} */ /* End of group hs_hal_misc */

#ifdef  __cplusplus
}
#endif

#endif  // end of "#define _RTL8710C_MISC_H_"

