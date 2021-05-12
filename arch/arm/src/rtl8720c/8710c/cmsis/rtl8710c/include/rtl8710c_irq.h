/**************************************************************************//**
 * @file     rtl8710c_irq.h
 * @brief    CMSIS Device System Header File for the AmebaZ2 platform.
 *           Defines the IRQ number for the System interrupts and Peripheral
 *           interrupts.
 * @version  V1.00
 * @date     2016-07-19
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2016 Realtek Corporation. All rights reserved.
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

#ifndef _RTL8710C_IRQ_H_
#define _RTL8710C_IRQ_H_


#ifdef __cplusplus
 extern "C" {
#endif
/**
 * @addtogroup hs_hal_irq IRQ
 * @ingroup 8710c_hal
 * @{
 * @brief The IRQ APIs of the AmebaZ2 platform.
 */

/// Number of interrupt for the system level, it's defined by the ARM Cortex-M CPU.
#define MAX_SYSTEM_IRQ_NUM                  16
/// Number of interrupt for peripheral devices on this platform.
#define MAX_PERIPHERAL_IRQ_NUM              16
/// Total number of interrupt on this platform.
#define MAX_VECTOR_TABLE_NUM                (MAX_SYSTEM_IRQ_NUM + MAX_PERIPHERAL_IRQ_NUM)

/* -------------------------  Interrupt Priority Definition  ------------------------ */
/**
  \brief  Default interrupt priority for external(peripheral devices) interrupts.
*/
enum IRQ_Priority {
    SystemOn_IRQPri             =  0,       /*!< System Interrupt Priority              */
    TimerGroup0_IRQPri          =  4,       /*!< Timer Group0 Interrupt Priority        */
    TimerGroup1_IRQPri          =  4,       /*!< Timer Group1 Interrupt Priority        */
    GPIO_IRQPri                 =  3,       /*!< GPIO Interrupt Priority                */
    PWM_IRQPri                  =  6,       /*!< PWM Interrupt Priority                 */
    UART_IRQPri                 =  4,       /*!< UART Interrupt Priority                */
    I2C_IRQPri                  =  6,       /*!< I2C Interrupt Priority                 */
    SPI_IRQPri                  =  3,       /*!< SPI Interrupt Priority                 */
    SDIOD_IRQPri                =  5,       /*!< SDIO Device Interrupt Priority         */
    WLAN_IRQPri                 =  3,       /*!< WLan Interrupt Priority                */
    GDMA0_IRQPri                =  7,       /*!< General-DMA 0 Interrupt Priority       */
    Crypto_IRQPri               =  6,       /*!< Crypto engine Interrupt Priority       */
    FlashCtrl_IRQPri            =  3,       /*!< SPI Flash Controller Interrupt Priority    */
    SGDMA0_IRQPri               =  4,       /*!< Secure GDMA0 Interrupt Priority       */
    SCrypto_IRQPri              =  6,       /*!< Secure Crypto engine Interrupt Priority    */
    SLowPri_IRQPri              =  7,       /*!< Secure low priority Interrupt Priority     */
    LowPri_IRQPri               =  7        /*!< Low priority Interrupt Priority       */
};

/**
  \brief  The data structure of the IRQ API function table.
          For RAM code can hook another optional IRQ APIs for ROM HAL code.
*/
typedef struct hal_irq_api_s {
    void     (*irq_enable)(int32_t irqn);           /*!< enable interrupt */
    void     (*irq_disable)(int32_t irqn);          /*!< disable interrupt */
    void     (*irq_set_vector)(int32_t irqn, uint32_t vector);      /*!< set interrupt vector */
    uint32_t (*irq_get_vector)(int32_t irqn);       /*!< get interrupt vector */
    void     (*irq_set_priority)(int32_t irqn, uint32_t priority);  /*!< set interrupt priority */
    uint32_t (*irq_get_priority)(int32_t irqn);     /*!< get interrupt priority */
    void     (*irq_set_pending)(int32_t irqn);      /*!< set pending interrupt */
    uint32_t (*irq_get_pending)(int32_t irqn);      /*!< get pending interrupt */
    void     (*irq_clear_pending)(int32_t irqn);    /*!< clear pending interrupt */
    void     (*interrupt_enable)(void);             /*!< enable all interrupts */
    void     (*interrupt_disable)(void);            /*!< disable all interrupts */
} hal_irq_api_t;

/**
  \brief  Function type of interrupt handler.
*/
typedef void (*int_vector_t) (void);

/**
  \brief  User application call back function for an interrupt.
*/
typedef void (*irq_handler_t)(void *data);

typedef struct irq_config_s {
    irq_handler_t   irq_fun;
    void            *data;
    int16_t         irq_num;
    uint16_t        priority;
} irq_config_t, *pirq_config_t;

/**
  \brief  The structure of the handler for stack back trace. 
          It is used to list of the call trace when a hard fault is occurred.
*/
typedef struct fault_handler_back_trace_s {
    uint32_t msp_top;         /*!< the Top address of the MSP */
    uint32_t msp_limit;       /*!< the Limit address of the MSP */
    uint32_t ps_max_size;     /*!< the maximum stack size of PSP */
    uint32_t *ptxt_range_list;  /*!< point to the array for the list of text code range */
    uint32_t trace_depth;   /*!< the size of the trace buffer */
    uint32_t *ptrace_buf;   /*!< point to the trace buffer */
    uint32_t *poffset_buf;   /*!< point to the buffer for offset from SP in trace */
} fault_handler_back_trace_t, *pfault_handler_back_trace_t;

/**
  \brief  The stubs function table type of the IRQ HAL functions in ROM.
*/
typedef struct hal_int_vector_func_stubs_s {
    int_vector_t *ram_vector_table;
    hal_irq_api_t *pirq_api_tbl;
    pfault_handler_back_trace_t *ppbk_trace_hdl;
    pfault_handler_back_trace_t *ppbk_trace_hdl_ns;

    void (*hal_vector_table_init)(uint32_t stack_ptr, int_vector_t *vector_tbl);
    void (*hal_irq_api_init)(hal_irq_api_t *pirq_api);
    void (*hal_irq_enable)(int32_t irqn);
    void (*hal_irq_disable)(int32_t irqn);
    void (*hal_irq_set_vector)(int32_t irqn, uint32_t vector);
    uint32_t (*hal_irq_get_vector)(int32_t irqn);
    void (*hal_irq_set_priority)(int32_t irqn, uint32_t priority);
    uint32_t (*hal_irq_get_priority)(int32_t irqn);
    void (*hal_irq_set_pending)(int32_t irqn);
    uint32_t (*hal_irq_get_pending)(int32_t irqn);
    void (*hal_irq_clear_pending)(int32_t irqn);
    void (*hal_irq_unreg)(int32_t irqn);

    uint32_t reserved[4];  // reserved space for next ROM code version function table extending.
} hal_int_vector_func_stubs_t;

/// @cond DOXYGEN_ROM_HAL_API

/**
 * @addtogroup hs_hal_irq_rom_func IRQ ROM APIs.
 * @{
 */

void hal_vector_table_init_rtl8710c(uint32_t stack_ptr, int_vector_t *vector_tbl);

void _default_handler_rtl8710c(void);
void hal_irq_api_init_rtl8710c(hal_irq_api_t *pirq_api);
void hal_irq_enable_rtl8710c(int32_t irqn);
void hal_irq_disable_rtl8710c(int32_t irqn);
void hal_irq_set_vector_rtl8710c(int32_t irqn, uint32_t vector);
uint32_t hal_irq_get_vector_rtl8710c(int32_t irqn);
void hal_irq_set_priority_rtl8710c(int32_t irqn, uint32_t priority);
uint32_t hal_irq_get_priority_rtl8710c(int32_t irqn);
void hal_irq_set_pending_rtl8710c(int32_t irqn);
uint32_t hal_irq_get_pending_rtl8710c(int32_t irqn);
void hal_irq_clear_pending_rtl8710c(int32_t irqn);
void hal_irq_unreg_rtl8710c(int32_t irqn);
void hal_interrupt_enable_rtl8710c(void);
void hal_interrupt_disable_rtl8710c(void);

/** @} */ /* End of group hs_hal_irq_rom_func */
/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */


/** @} */ /* End of group hs_hal_irq */

#ifdef __cplusplus
}
#endif

#endif //_RTL8710C_IRQ_H_


