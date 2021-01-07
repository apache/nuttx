/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_pwm.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_PWM_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PWM0,1,2,3,4 are the same so create this helper for register offsets */

#define BL602_PWM_N_BASE_OFFSET(n)    (BL602_PWM_BASE + ((n) * 0x20))

/* Register offsets *********************************************************/

#define BL602_PWM_INT_CONFIG_OFFSET  0x000000  /* pwm_int_config */
#define BL602_PWM0_BASE_OFFSET       0x000020  /* pwm0 base */
#define BL602_PWM0_CLKDIV_OFFSET     0x000020  /* pwm0_clkdiv */
#define BL602_PWM0_THRE1_OFFSET      0x000024  /* pwm0_thre1 */
#define BL602_PWM0_THRE2_OFFSET      0x000028  /* pwm0_thre2 */
#define BL602_PWM0_PERIOD_OFFSET     0x00002c  /* pwm0_period */
#define BL602_PWM0_CONFIG_OFFSET     0x000030  /* pwm0_config */
#define BL602_PWM0_INTERRUPT_OFFSET  0x000034  /* pwm0_interrupt */
#define BL602_PWM1_BASE_OFFSET       0x000040  /* pwm1 base */
#define BL602_PWM1_CLKDIV_OFFSET     0x000040  /* pwm1_clkdiv */
#define BL602_PWM1_THRE1_OFFSET      0x000044  /* pwm1_thre1 */
#define BL602_PWM1_THRE2_OFFSET      0x000048  /* pwm1_thre2 */
#define BL602_PWM1_PERIOD_OFFSET     0x00004c  /* pwm1_period */
#define BL602_PWM1_CONFIG_OFFSET     0x000050  /* pwm1_config */
#define BL602_PWM1_INTERRUPT_OFFSET  0x000054  /* pwm1_interrupt */
#define BL602_PWM2_BASE_OFFSET       0x000060  /* pwm2 base */
#define BL602_PWM2_CLKDIV_OFFSET     0x000060  /* pwm2_clkdiv */
#define BL602_PWM2_THRE1_OFFSET      0x000064  /* pwm2_thre1 */
#define BL602_PWM2_THRE2_OFFSET      0x000068  /* pwm2_thre2 */
#define BL602_PWM2_PERIOD_OFFSET     0x00006c  /* pwm2_period */
#define BL602_PWM2_CONFIG_OFFSET     0x000070  /* pwm2_config */
#define BL602_PWM2_INTERRUPT_OFFSET  0x000074  /* pwm2_interrupt */
#define BL602_PWM3_BASE_OFFSET       0x000080  /* pwm3 base */
#define BL602_PWM3_CLKDIV_OFFSET     0x000080  /* pwm3_clkdiv */
#define BL602_PWM3_THRE1_OFFSET      0x000084  /* pwm3_thre1 */
#define BL602_PWM3_THRE2_OFFSET      0x000088  /* pwm3_thre2 */
#define BL602_PWM3_PERIOD_OFFSET     0x00008c  /* pwm3_period */
#define BL602_PWM3_CONFIG_OFFSET     0x000090  /* pwm3_config */
#define BL602_PWM3_INTERRUPT_OFFSET  0x000094  /* pwm3_interrupt */
#define BL602_PWM4_BASE_OFFSET       0x0000a0  /* pwm4 base */
#define BL602_PWM4_CLKDIV_OFFSET     0x0000a0  /* pwm4_clkdiv */
#define BL602_PWM4_THRE1_OFFSET      0x0000a4  /* pwm4_thre1 */
#define BL602_PWM4_THRE2_OFFSET      0x0000a8  /* pwm4_thre2 */
#define BL602_PWM4_PERIOD_OFFSET     0x0000ac  /* pwm4_period */
#define BL602_PWM4_CONFIG_OFFSET     0x0000b0  /* pwm4_config */
#define BL602_PWM4_INTERRUPT_OFFSET  0x0000b4  /* pwm4_interrupt */

/* Register definitions *****************************************************/

#define BL602_PWM_INT_CONFIG      (BL602_PWM_BASE + BL602_PWM_INT_CONFIG_OFFSET)
#define BL602_PWM_N_CLKDIV(n)     (BL602_PWM_N_BASE_OFFSET(n) + BL602_PWM0_CLKDIV_OFFSET)
#define BL602_PWM_N_THRE1(n)      (BL602_PWM_N_BASE_OFFSET(n) + BL602_PWM0_THRE1_OFFSET)
#define BL602_PWM_N_THRE2(n)      (BL602_PWM_N_BASE_OFFSET(n) + BL602_PWM0_THRE2_OFFSET)
#define BL602_PWM_N_PERIOD(n)     (BL602_PWM_N_BASE_OFFSET(n) + BL602_PWM0_PERIOD_OFFSET)
#define BL602_PWM_N_CONFIG(n)     (BL602_PWM_N_BASE_OFFSET(n) + BL602_PWM0_CONFIG_OFFSET)
#define BL602_PWM_N_INTERRUPT(n)  (BL602_PWM_N_BASE_OFFSET(n) + BL602_PWM0_INTERRUPT_OFFSET)
#define BL602_PWM0_CLKDIV          BL602_PWM_N_CLKDIV(0)
#define BL602_PWM0_THRE1           BL602_PWM_N_THRE1(0)
#define BL602_PWM0_THRE2           BL602_PWM_N_THRE2(0)
#define BL602_PWM0_PERIOD          BL602_PWM_N_PERIOD(0)
#define BL602_PWM0_CONFIG          BL602_PWM_N_CONFIG(0)
#define BL602_PWM0_INTERRUPT       BL602_PWM_N_INTERRUPT(0)
#define BL602_PWM1_CLKDIV          BL602_PWM_N_CLKDIV(1)
#define BL602_PWM1_THRE1           BL602_PWM_N_THRE1(1)
#define BL602_PWM1_THRE2           BL602_PWM_N_THRE2(1)
#define BL602_PWM1_PERIOD          BL602_PWM_N_PERIOD(1)
#define BL602_PWM1_CONFIG          BL602_PWM_N_CONFIG(1)
#define BL602_PWM1_INTERRUPT       BL602_PWM_N_INTERRUPT(1)
#define BL602_PWM2_CLKDIV          BL602_PWM_N_CLKDIV(2)
#define BL602_PWM2_THRE1           BL602_PWM_N_THRE1(2)
#define BL602_PWM2_THRE2           BL602_PWM_N_THRE2(2)
#define BL602_PWM2_PERIOD          BL602_PWM_N_PERIOD(2)
#define BL602_PWM2_CONFIG          BL602_PWM_N_CONFIG(2)
#define BL602_PWM2_INTERRUPT       BL602_PWM_N_INTERRUPT(2)
#define BL602_PWM3_CLKDIV          BL602_PWM_N_CLKDIV(3)
#define BL602_PWM3_THRE1           BL602_PWM_N_THRE1(3)
#define BL602_PWM3_THRE2           BL602_PWM_N_THRE2(3)
#define BL602_PWM3_PERIOD          BL602_PWM_N_PERIOD(3)
#define BL602_PWM3_CONFIG          BL602_PWM_N_CONFIG(3)
#define BL602_PWM3_INTERRUPT       BL602_PWM_N_INTERRUPT(3)
#define BL602_PWM4_CLKDIV          BL602_PWM_N_CLKDIV(4)
#define BL602_PWM4_THRE1           BL602_PWM_N_THRE1(4)
#define BL602_PWM4_THRE2           BL602_PWM_N_THRE2(4)
#define BL602_PWM4_PERIOD          BL602_PWM_N_PERIOD(4)
#define BL602_PWM4_CONFIG          BL602_PWM_N_CONFIG(4)
#define BL602_PWM4_INTERRUPT       BL602_PWM_N_INTERRUPT(4)

/* Register bit definitions *************************************************/

#define INT_CONFIG_PWM_INT_CLEAR_MASK      (0x3f << 8)
#define INT_CONFIG_PWM_INTERRUPT_STS_MASK  (0x3f)

#define CLKDIV_PWM_CLK_DIV_MASK            (0xffff)

#define THRE1_PWM_THRE1_MASK               (0xffff)

#define THRE2_PWM_THRE2_MASK               (0xffff)

#define PERIOD_PWM_PERIOD_MASK             (0xffff)

#define CONFIG_PWM_STS_TOP                 (1 << 7)
#define CONFIG_PWM_STOP_EN                 (1 << 6)
#define CONFIG_PWM_SW_MODE                 (1 << 5)
#define CONFIG_PWM_SW_FORCE_VAL            (1 << 4)
#define CONFIG_PWM_STOP_MODE               (1 << 3)
#define CONFIG_PWM_OUT_INV                 (1 << 2)
#define CONFIG_REG_CLK_SEL_MASK            (0x03)

#define INTERRUPT_PWM_INT_ENABLE           (1 << 16)
#define INTERRUPT_PWM_INT_PERIOD_CNT_MASK  (0xffff)

#define   BL602_PWM_CH0                     0  /* PWM Channel 0 */
#define   BL602_PWM_CH1                     1  /* PWM Channel 1 define */
#define   BL602_PWM_CH2                     2  /* PWM Channel 2 define */
#define   BL602_PWM_CH3                     3  /* PWM Channel 3 define */
#define   BL602_PWM_CH4                     4  /* PWM Channel 4 define */

#define   BL602_PWM_CLK_XCLK                0  /* PWM Clock source :XTAL */
#define   BL602_PWM_CLK_BCLK                1  /* PWM Clock source :Bus */
#define   BL602_PWM_CLK_32K                 2  /* PWM Clock source :32K */

#define   BL602_PWM_STOP_ABRUPT             0  /* PWM stop abrupt */
#define   BL602_PWM_STOP_GRACEFUL           1  /* PWM stop graceful */

#define   BL602_PWM_POL_NORMAL              0  /* PWM normal polarity */
#define   BL602_PWM_POL_INVERT              1  /* PWM invert polarity */

#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_PWM_H */
