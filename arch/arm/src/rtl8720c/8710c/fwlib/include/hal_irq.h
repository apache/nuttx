/**************************************************************************//**
 * @file     hal_irq.h
 * @brief    The HAL API implementation for the NVIC control.
 * @version  V1.00
 * @date     2016-09-30
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

#ifndef _HAL_IRQ_H_
#define _HAL_IRQ_H_
#include "cmsis.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/**
* @addtogroup hs_hal_irq IRQ
* @{
*/

/**
  * @brief The stubs functions table to exports IRQ HAL functions in ROM.
  */
extern const hal_int_vector_func_stubs_t hal_int_vector_stubs;

/**
 *  @brief Initials the RAM base interrupt vector table and configure the NVIC to use this table.
 *         It copys the default ROM space vector table to the new vector table.
 *
 *  @param[in]  stack_ptr The stack pointer. As the Cortex-m spec,
 *                        this value must be filled to the start of the vector table.
 *
 *  @param[in]  vector_tbl The vector table.
 *
 *  @returns void
 */
__STATIC_INLINE void hal_vector_table_init(uint32_t stack_ptr, int_vector_t *vector_tbl)
{
    hal_int_vector_stubs.hal_vector_table_init (stack_ptr, vector_tbl);
}

/**
  \brief   Hooks IRQ APIs for ROM code.
  \details Hooks a set of NVIC control APIs for the ROM HAL code.
  \param [in]      pirq_api  The IRQ APIs to hook.
 */
__STATIC_INLINE void hal_irq_api_init(hal_irq_api_t *pirq_api)
{
    hal_int_vector_stubs.hal_irq_api_init(pirq_api);
}

/**
  \brief   Enable Interrupt
  \details Enables a device specific interrupt in the NVIC interrupt controller.
  \param [in]      irqn  Device specific interrupt number.
  \note    irqn must not be negative.
 */
__STATIC_INLINE void hal_irq_enable(int32_t irqn)
{
    hal_int_vector_stubs.hal_irq_enable(irqn);
}

/**
  \brief   Disable Interrupt
  \details Disables a device specific interrupt in the NVIC interrupt controller.
  \param [in]      irqn  Device specific interrupt number.
  \note    irqn must not be negative.
 */
__STATIC_INLINE void hal_irq_disable(int32_t irqn)
{
    hal_int_vector_stubs.hal_irq_disable(irqn);
}

/**
  \brief   Set Interrupt Vector
  \details Sets an interrupt vector in SRAM based interrupt vector table.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
           VTOR must been relocated to SRAM before.
  \param [in]   irqn      Interrupt number
  \param [in]   vector    Address of interrupt handler function
 */
__STATIC_INLINE void hal_irq_set_vector(int32_t irqn, uint32_t vector)
{
    hal_int_vector_stubs.hal_irq_set_vector(irqn, vector);
}

/**
  \brief   Get Interrupt Vector
  \details Reads an interrupt vector from interrupt vector table.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
  \param [in]   irqn      Interrupt number.
  \return                 Address of interrupt handler function
 */
__STATIC_INLINE uint32_t hal_irq_get_vector(int32_t irqn)
{
    return hal_int_vector_stubs.hal_irq_get_vector(irqn);
}

/**
  \brief   Set Interrupt Priority
  \details Sets the priority of a device specific interrupt or a processor exception.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
  \param [in]      irqn  Interrupt number.
  \param [in]  priority  Priority to set.
  \note    The priority cannot be set for every processor exception.
 */
__STATIC_INLINE void hal_irq_set_priority(int32_t irqn, uint32_t priority)
{
    hal_int_vector_stubs.hal_irq_set_priority(irqn, priority);
}

/**
  \brief   Get Interrupt Priority
  \details Reads the priority of a device specific interrupt or a processor exception.
           The interrupt number can be positive to specify a device specific interrupt,
           or negative to specify a processor exception.
  \param [in]   irqn  Interrupt number.
  \return             Interrupt Priority.
                      Value is aligned automatically to the implemented priority bits of the microcontroller.
 */
__STATIC_INLINE uint32_t hal_irq_get_priority(int32_t irqn)
{
    return hal_int_vector_stubs.hal_irq_get_priority(irqn);
}

/**
  \brief   Set Pending Interrupt
  \details Sets the pending bit of a device specific interrupt in the NVIC pending register.
  \param [in]      irqn  Device specific interrupt number.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void hal_irq_set_pending(int32_t irqn)
{
    hal_int_vector_stubs.hal_irq_set_pending(irqn);
}

/**
  \brief   Get Pending Interrupt
  \details Reads the NVIC pending register and returns the pending bit for the specified device specific interrupt.
  \param [in]      irqn  Device specific interrupt number.
  \return             0  Interrupt status is not pending.
  \return             1  Interrupt status is pending.
  \note    irqn must not be negative.
 */
__STATIC_INLINE uint32_t hal_irq_get_pending(int32_t irqn)
{
    return hal_int_vector_stubs.hal_irq_get_pending(irqn);
}

/**
  \brief   Clear Pending Interrupt
  \details Clears the pending bit of a device specific interrupt in the NVIC pending register.
  \param [in]      irqn  Device specific interrupt number.
  \note    irqn must not be negative.
 */
__STATIC_INLINE void hal_irq_clear_pending(int32_t irqn)
{
    hal_int_vector_stubs.hal_irq_clear_pending(irqn);
}

/** @} */ /* End of group hs_hal_irq */

#ifdef  __cplusplus
}
#endif


#endif  // end of "#define _HAL_IRQ_H_"

