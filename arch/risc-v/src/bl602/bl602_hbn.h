/****************************************************************************
 * arch/risc-v/src/bl602/bl602_hbn.h
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

#ifndef __ARCH_RISCV_SRC_BL602_BL602_HBN_H
#define __ARCH_RISCV_SRC_BL602_BL602_HBN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

#include <sys/types.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BL602_HBN_OUT0_INT_GPIO7 (0)     /* HBN out 0 interrupt type: GPIO7 */
#define BL602_HBN_OUT0_INT_GPIO8 (1)     /* HBN out 0 interrupt type: GPIO8 */
#define BL602_HBN_OUT0_INT_RTC   (2)     /* HBN out 0 interrupt type: RTC */

#define BL602_HBN_INT_GPIO7      (0)     /* HBN interrupt type: GPIO7 */
#define BL602_HBN_INT_GPIO8      (1)     /* HBN interrupt type: GPIO8 */
#define BL602_HBN_INT_RTC        (16)    /* HBN interrupt type: RTC */
#define BL602_HBN_INT_PIR        (17)    /* HBN interrupt type: PIR */
#define BL602_HBN_INT_BOR        (18)    /* HBN interrupt type: BOR */
#define BL602_HBN_INT_ACOMP0     (20)    /* HBN interrupt type: ACOMP0 */
#define BL602_HBN_INT_ACOMP1     (22)    /* HBN interrupt type: ACOMP1 */

typedef int (*bl602_hbn_cb_t)(void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_hbn_set_uart_clk_sel
 *
 * Description:
 *   Select uart clock source.
 *
 * Input Parameters:
 *   clk_sel: uart clock type selection, 0 for FCLK or 1 for 160MHz CLK
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   Anything else that one might need to know to use this function.
 *
 ****************************************************************************/

void bl602_set_uart_clk_sel(int clk_sel);

/****************************************************************************
 * Name: bl602_hbn_get_int_state
 *
 * Description:
 *   HBN get interrupt status.
 *
 * Input Parameters:
 *   irq_type: HBN interrupt type
 *
 * Returned Value:
 *   true or false
 *
 ****************************************************************************/

bool bl602_hbn_get_int_state(uint8_t irq_type);

/****************************************************************************
 * Name: bl602_hbn_clear_irq
 *
 * Description:
 *   HBN clear interrupt status.
 *
 * Input Parameters:
 *   hbn_int_type: HBN interrupt type
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bl602_hbn_clear_irq(uint8_t hbn_int_type);

/****************************************************************************
 * Name: bl602_hbn_out0_int_register
 *
 * Description:
 *   HBN out0 interrupt cllback register.
 *
 * Input Parameters:
 *   irq_type: HBN interrupt type
 *   isr_cb: callback
 *   arg: callback arg
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

int bl602_hbn_out0_int_register(uint8_t irq_type, bl602_hbn_cb_t isr_cb,
                                void *arg);

/****************************************************************************
 * Name: bl602_hbn_out0_int_unregister
 *
 * Description:
 *   HBN out0 interrupt cllback unregister.
 *
 * Input Parameters:
 *   irq_type: HBN interrupt type
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

int bl602_hbn_out0_int_unregister(uint8_t irq_type);

/****************************************************************************
 * Name: bl602_hbn_out0_int_enable
 *
 * Description:
 *   HBN out0 interrupt enable.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_hbn_out0_int_enable(void);

/****************************************************************************
 * Name: bl602_hbn_out0_int_disable
 *
 * Description:
 *   HBN out0 interrupt disable.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void bl602_hbn_out0_int_disable(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_BL602_BL602_HBN_H */
