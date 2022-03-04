/****************************************************************************
 * arch/xtensa/src/esp32s3/hardware/esp32s3_system.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_SYSTEM_H
#define __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_SYSTEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32s3_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SYSTEM_CORE_1_CONTROL_0_REG register
 * Core0 control regiter 0
 */

#define SYSTEM_CORE_1_CONTROL_0_REG (DR_REG_SYSTEM_BASE + 0x0)

/* SYSTEM_CONTROL_CORE_1_RESETING : R/W; bitpos: [2]; default: 1;
 * Set 1 to let core1 reset
 */

#define SYSTEM_CONTROL_CORE_1_RESETING    (BIT(2))
#define SYSTEM_CONTROL_CORE_1_RESETING_M  (SYSTEM_CONTROL_CORE_1_RESETING_V << SYSTEM_CONTROL_CORE_1_RESETING_S)
#define SYSTEM_CONTROL_CORE_1_RESETING_V  0x00000001
#define SYSTEM_CONTROL_CORE_1_RESETING_S  2

/* SYSTEM_CONTROL_CORE_1_CLKGATE_EN : R/W; bitpos: [1]; default: 0;
 * Set 1 to open core1 clock
 */

#define SYSTEM_CONTROL_CORE_1_CLKGATE_EN    (BIT(1))
#define SYSTEM_CONTROL_CORE_1_CLKGATE_EN_M  (SYSTEM_CONTROL_CORE_1_CLKGATE_EN_V << SYSTEM_CONTROL_CORE_1_CLKGATE_EN_S)
#define SYSTEM_CONTROL_CORE_1_CLKGATE_EN_V  0x00000001
#define SYSTEM_CONTROL_CORE_1_CLKGATE_EN_S  1

/* SYSTEM_CONTROL_CORE_1_RUNSTALL : R/W; bitpos: [0]; default: 0;
 * Set 1 to stall core1
 */

#define SYSTEM_CONTROL_CORE_1_RUNSTALL    (BIT(0))
#define SYSTEM_CONTROL_CORE_1_RUNSTALL_M  (SYSTEM_CONTROL_CORE_1_RUNSTALL_V << SYSTEM_CONTROL_CORE_1_RUNSTALL_S)
#define SYSTEM_CONTROL_CORE_1_RUNSTALL_V  0x00000001
#define SYSTEM_CONTROL_CORE_1_RUNSTALL_S  0

/* SYSTEM_CORE_1_CONTROL_1_REG register
 * Core0 control regiter 1
 */

#define SYSTEM_CORE_1_CONTROL_1_REG (DR_REG_SYSTEM_BASE + 0x4)

/* SYSTEM_CONTROL_CORE_1_MESSAGE : R/W; bitpos: [31:0]; default: 0;
 * it's only a R/W register, no function, software can write any value
 */

#define SYSTEM_CONTROL_CORE_1_MESSAGE    0xffffffff
#define SYSTEM_CONTROL_CORE_1_MESSAGE_M  (SYSTEM_CONTROL_CORE_1_MESSAGE_V << SYSTEM_CONTROL_CORE_1_MESSAGE_S)
#define SYSTEM_CONTROL_CORE_1_MESSAGE_V  0xffffffff
#define SYSTEM_CONTROL_CORE_1_MESSAGE_S  0

/* SYSTEM_CPU_PERI_CLK_EN_REG register
 * cpu_peripheral clock configuration register
 */

#define SYSTEM_CPU_PERI_CLK_EN_REG (DR_REG_SYSTEM_BASE + 0x8)

/* SYSTEM_CLK_EN_DEDICATED_GPIO : R/W; bitpos: [7]; default: 0;
 * Set 1 to open dedicated_gpio module clk
 */

#define SYSTEM_CLK_EN_DEDICATED_GPIO    (BIT(7))
#define SYSTEM_CLK_EN_DEDICATED_GPIO_M  (SYSTEM_CLK_EN_DEDICATED_GPIO_V << SYSTEM_CLK_EN_DEDICATED_GPIO_S)
#define SYSTEM_CLK_EN_DEDICATED_GPIO_V  0x00000001
#define SYSTEM_CLK_EN_DEDICATED_GPIO_S  7

/* SYSTEM_CLK_EN_ASSIST_DEBUG : R/W; bitpos: [6]; default: 0;
 * Set 1 to open assist_debug module clock
 */

#define SYSTEM_CLK_EN_ASSIST_DEBUG    (BIT(6))
#define SYSTEM_CLK_EN_ASSIST_DEBUG_M  (SYSTEM_CLK_EN_ASSIST_DEBUG_V << SYSTEM_CLK_EN_ASSIST_DEBUG_S)
#define SYSTEM_CLK_EN_ASSIST_DEBUG_V  0x00000001
#define SYSTEM_CLK_EN_ASSIST_DEBUG_S  6

/* SYSTEM_CPU_PERI_RST_EN_REG register
 * cpu_peripheral reset configuration regsiter
 */

#define SYSTEM_CPU_PERI_RST_EN_REG (DR_REG_SYSTEM_BASE + 0xc)

/* SYSTEM_RST_EN_DEDICATED_GPIO : R/W; bitpos: [7]; default: 1;
 * Set 1 to let dedicated_gpio module reset
 */

#define SYSTEM_RST_EN_DEDICATED_GPIO    (BIT(7))
#define SYSTEM_RST_EN_DEDICATED_GPIO_M  (SYSTEM_RST_EN_DEDICATED_GPIO_V << SYSTEM_RST_EN_DEDICATED_GPIO_S)
#define SYSTEM_RST_EN_DEDICATED_GPIO_V  0x00000001
#define SYSTEM_RST_EN_DEDICATED_GPIO_S  7

/* SYSTEM_RST_EN_ASSIST_DEBUG : R/W; bitpos: [6]; default: 1;
 * Set 1 to let assist_debug module reset
 */

#define SYSTEM_RST_EN_ASSIST_DEBUG    (BIT(6))
#define SYSTEM_RST_EN_ASSIST_DEBUG_M  (SYSTEM_RST_EN_ASSIST_DEBUG_V << SYSTEM_RST_EN_ASSIST_DEBUG_S)
#define SYSTEM_RST_EN_ASSIST_DEBUG_V  0x00000001
#define SYSTEM_RST_EN_ASSIST_DEBUG_S  6

/* SYSTEM_CPU_PER_CONF_REG register
 * cpu peripheral clock configuration register
 */

#define SYSTEM_CPU_PER_CONF_REG (DR_REG_SYSTEM_BASE + 0x10)

/* SYSTEM_CPU_WAITI_DELAY_NUM : R/W; bitpos: [7:4]; default: 0;
 * This field used to set delay cycle when cpu enter waiti mode, after delay
 * waiti_clk will close
 */

#define SYSTEM_CPU_WAITI_DELAY_NUM    0x0000000f
#define SYSTEM_CPU_WAITI_DELAY_NUM_M  (SYSTEM_CPU_WAITI_DELAY_NUM_V << SYSTEM_CPU_WAITI_DELAY_NUM_S)
#define SYSTEM_CPU_WAITI_DELAY_NUM_V  0x0000000f
#define SYSTEM_CPU_WAITI_DELAY_NUM_S  4

/* SYSTEM_CPU_WAIT_MODE_FORCE_ON : R/W; bitpos: [3]; default: 1;
 * Set 1 to force cpu_waiti_clk enable.
 */

#define SYSTEM_CPU_WAIT_MODE_FORCE_ON    (BIT(3))
#define SYSTEM_CPU_WAIT_MODE_FORCE_ON_M  (SYSTEM_CPU_WAIT_MODE_FORCE_ON_V << SYSTEM_CPU_WAIT_MODE_FORCE_ON_S)
#define SYSTEM_CPU_WAIT_MODE_FORCE_ON_V  0x00000001
#define SYSTEM_CPU_WAIT_MODE_FORCE_ON_S  3

/* SYSTEM_PLL_FREQ_SEL : R/W; bitpos: [2]; default: 1;
 * This field used to sel pll frequent.
 */

#define SYSTEM_PLL_FREQ_SEL    (BIT(2))
#define SYSTEM_PLL_FREQ_SEL_M  (SYSTEM_PLL_FREQ_SEL_V << SYSTEM_PLL_FREQ_SEL_S)
#define SYSTEM_PLL_FREQ_SEL_V  0x00000001
#define SYSTEM_PLL_FREQ_SEL_S  2

/* SYSTEM_CPUPERIOD_SEL : R/W; bitpos: [1:0]; default: 0;
 * This field used to sel cpu clock frequent.
 */

#define SYSTEM_CPUPERIOD_SEL    0x00000003
#define SYSTEM_CPUPERIOD_SEL_M  (SYSTEM_CPUPERIOD_SEL_V << SYSTEM_CPUPERIOD_SEL_S)
#define SYSTEM_CPUPERIOD_SEL_V  0x00000003
#define SYSTEM_CPUPERIOD_SEL_S  0

/* SYSTEM_MEM_PD_MASK_REG register
 * memory power down mask configuration register
 */

#define SYSTEM_MEM_PD_MASK_REG (DR_REG_SYSTEM_BASE + 0x14)

/* SYSTEM_LSLP_MEM_PD_MASK : R/W; bitpos: [0]; default: 1;
 * Set 1 to mask memory power down.
 */

#define SYSTEM_LSLP_MEM_PD_MASK    (BIT(0))
#define SYSTEM_LSLP_MEM_PD_MASK_M  (SYSTEM_LSLP_MEM_PD_MASK_V << SYSTEM_LSLP_MEM_PD_MASK_S)
#define SYSTEM_LSLP_MEM_PD_MASK_V  0x00000001
#define SYSTEM_LSLP_MEM_PD_MASK_S  0

/* SYSTEM_PERIP_CLK_EN0_REG register
 * peripheral clock configuration regsiter 0
 */

#define SYSTEM_PERIP_CLK_EN0_REG (DR_REG_SYSTEM_BASE + 0x18)

/* SYSTEM_SPI4_CLK_EN : R/W; bitpos: [31]; default: 1;
 * Set 1 to enable SPI4 clock
 */

#define SYSTEM_SPI4_CLK_EN    (BIT(31))
#define SYSTEM_SPI4_CLK_EN_M  (SYSTEM_SPI4_CLK_EN_V << SYSTEM_SPI4_CLK_EN_S)
#define SYSTEM_SPI4_CLK_EN_V  0x00000001
#define SYSTEM_SPI4_CLK_EN_S  31

/* SYSTEM_ADC2_ARB_CLK_EN : R/W; bitpos: [30]; default: 1;
 * Set 1 to enable ADC2_ARB clock
 */

#define SYSTEM_ADC2_ARB_CLK_EN    (BIT(30))
#define SYSTEM_ADC2_ARB_CLK_EN_M  (SYSTEM_ADC2_ARB_CLK_EN_V << SYSTEM_ADC2_ARB_CLK_EN_S)
#define SYSTEM_ADC2_ARB_CLK_EN_V  0x00000001
#define SYSTEM_ADC2_ARB_CLK_EN_S  30

/* SYSTEM_SYSTIMER_CLK_EN : R/W; bitpos: [29]; default: 1;
 * Set 1 to enable SYSTEMTIMER clock
 */

#define SYSTEM_SYSTIMER_CLK_EN    (BIT(29))
#define SYSTEM_SYSTIMER_CLK_EN_M  (SYSTEM_SYSTIMER_CLK_EN_V << SYSTEM_SYSTIMER_CLK_EN_S)
#define SYSTEM_SYSTIMER_CLK_EN_V  0x00000001
#define SYSTEM_SYSTIMER_CLK_EN_S  29

/* SYSTEM_APB_SARADC_CLK_EN : R/W; bitpos: [28]; default: 1;
 * Set 1 to enable APB_SARADC clock
 */

#define SYSTEM_APB_SARADC_CLK_EN    (BIT(28))
#define SYSTEM_APB_SARADC_CLK_EN_M  (SYSTEM_APB_SARADC_CLK_EN_V << SYSTEM_APB_SARADC_CLK_EN_S)
#define SYSTEM_APB_SARADC_CLK_EN_V  0x00000001
#define SYSTEM_APB_SARADC_CLK_EN_S  28

/* SYSTEM_SPI3_DMA_CLK_EN : R/W; bitpos: [27]; default: 1;
 * Set 1 to enable SPI4 clock
 */

#define SYSTEM_SPI3_DMA_CLK_EN    (BIT(27))
#define SYSTEM_SPI3_DMA_CLK_EN_M  (SYSTEM_SPI3_DMA_CLK_EN_V << SYSTEM_SPI3_DMA_CLK_EN_S)
#define SYSTEM_SPI3_DMA_CLK_EN_V  0x00000001
#define SYSTEM_SPI3_DMA_CLK_EN_S  27

/* SYSTEM_PWM3_CLK_EN : R/W; bitpos: [26]; default: 0;
 * Set 1 to enable PWM3 clock
 */

#define SYSTEM_PWM3_CLK_EN    (BIT(26))
#define SYSTEM_PWM3_CLK_EN_M  (SYSTEM_PWM3_CLK_EN_V << SYSTEM_PWM3_CLK_EN_S)
#define SYSTEM_PWM3_CLK_EN_V  0x00000001
#define SYSTEM_PWM3_CLK_EN_S  26

/* SYSTEM_PWM2_CLK_EN : R/W; bitpos: [25]; default: 0;
 * Set 1 to enable PWM2 clock
 */

#define SYSTEM_PWM2_CLK_EN    (BIT(25))
#define SYSTEM_PWM2_CLK_EN_M  (SYSTEM_PWM2_CLK_EN_V << SYSTEM_PWM2_CLK_EN_S)
#define SYSTEM_PWM2_CLK_EN_V  0x00000001
#define SYSTEM_PWM2_CLK_EN_S  25

/* SYSTEM_UART_MEM_CLK_EN : R/W; bitpos: [24]; default: 1;
 * Set 1 to enable UART_MEM clock
 */

#define SYSTEM_UART_MEM_CLK_EN    (BIT(24))
#define SYSTEM_UART_MEM_CLK_EN_M  (SYSTEM_UART_MEM_CLK_EN_V << SYSTEM_UART_MEM_CLK_EN_S)
#define SYSTEM_UART_MEM_CLK_EN_V  0x00000001
#define SYSTEM_UART_MEM_CLK_EN_S  24

/* SYSTEM_USB_CLK_EN : R/W; bitpos: [23]; default: 1;
 * Set 1 to enable USB clock
 */

#define SYSTEM_USB_CLK_EN    (BIT(23))
#define SYSTEM_USB_CLK_EN_M  (SYSTEM_USB_CLK_EN_V << SYSTEM_USB_CLK_EN_S)
#define SYSTEM_USB_CLK_EN_V  0x00000001
#define SYSTEM_USB_CLK_EN_S  23

/* SYSTEM_SPI2_DMA_CLK_EN : R/W; bitpos: [22]; default: 1;
 * Set 1 to enable SPI2_DMA clock
 */

#define SYSTEM_SPI2_DMA_CLK_EN    (BIT(22))
#define SYSTEM_SPI2_DMA_CLK_EN_M  (SYSTEM_SPI2_DMA_CLK_EN_V << SYSTEM_SPI2_DMA_CLK_EN_S)
#define SYSTEM_SPI2_DMA_CLK_EN_V  0x00000001
#define SYSTEM_SPI2_DMA_CLK_EN_S  22

/* SYSTEM_I2S1_CLK_EN : R/W; bitpos: [21]; default: 0;
 * Set 1 to enable I2S1 clock
 */

#define SYSTEM_I2S1_CLK_EN    (BIT(21))
#define SYSTEM_I2S1_CLK_EN_M  (SYSTEM_I2S1_CLK_EN_V << SYSTEM_I2S1_CLK_EN_S)
#define SYSTEM_I2S1_CLK_EN_V  0x00000001
#define SYSTEM_I2S1_CLK_EN_S  21

/* SYSTEM_PWM1_CLK_EN : R/W; bitpos: [20]; default: 0;
 * Set 1 to enable PWM1 clock
 */

#define SYSTEM_PWM1_CLK_EN    (BIT(20))
#define SYSTEM_PWM1_CLK_EN_M  (SYSTEM_PWM1_CLK_EN_V << SYSTEM_PWM1_CLK_EN_S)
#define SYSTEM_PWM1_CLK_EN_V  0x00000001
#define SYSTEM_PWM1_CLK_EN_S  20

/* SYSTEM_CAN_CLK_EN : R/W; bitpos: [19]; default: 0;
 * Set 1 to enable CAN clock
 */

#define SYSTEM_CAN_CLK_EN    (BIT(19))
#define SYSTEM_CAN_CLK_EN_M  (SYSTEM_CAN_CLK_EN_V << SYSTEM_CAN_CLK_EN_S)
#define SYSTEM_CAN_CLK_EN_V  0x00000001
#define SYSTEM_CAN_CLK_EN_S  19

/* SYSTEM_I2C_EXT1_CLK_EN : R/W; bitpos: [18]; default: 0;
 * Set 1 to enable I2C_EXT1 clock
 */

#define SYSTEM_I2C_EXT1_CLK_EN    (BIT(18))
#define SYSTEM_I2C_EXT1_CLK_EN_M  (SYSTEM_I2C_EXT1_CLK_EN_V << SYSTEM_I2C_EXT1_CLK_EN_S)
#define SYSTEM_I2C_EXT1_CLK_EN_V  0x00000001
#define SYSTEM_I2C_EXT1_CLK_EN_S  18

/* SYSTEM_PWM0_CLK_EN : R/W; bitpos: [17]; default: 0;
 * Set 1 to enable PWM0 clock
 */

#define SYSTEM_PWM0_CLK_EN    (BIT(17))
#define SYSTEM_PWM0_CLK_EN_M  (SYSTEM_PWM0_CLK_EN_V << SYSTEM_PWM0_CLK_EN_S)
#define SYSTEM_PWM0_CLK_EN_V  0x00000001
#define SYSTEM_PWM0_CLK_EN_S  17

/* SYSTEM_SPI3_CLK_EN : R/W; bitpos: [16]; default: 1;
 * Set 1 to enable SPI3 clock
 */

#define SYSTEM_SPI3_CLK_EN    (BIT(16))
#define SYSTEM_SPI3_CLK_EN_M  (SYSTEM_SPI3_CLK_EN_V << SYSTEM_SPI3_CLK_EN_S)
#define SYSTEM_SPI3_CLK_EN_V  0x00000001
#define SYSTEM_SPI3_CLK_EN_S  16

/* SYSTEM_TIMERGROUP1_CLK_EN : R/W; bitpos: [15]; default: 1;
 * Set 1 to enable TIMERGROUP1 clock
 */

#define SYSTEM_TIMERGROUP1_CLK_EN    (BIT(15))
#define SYSTEM_TIMERGROUP1_CLK_EN_M  (SYSTEM_TIMERGROUP1_CLK_EN_V << SYSTEM_TIMERGROUP1_CLK_EN_S)
#define SYSTEM_TIMERGROUP1_CLK_EN_V  0x00000001
#define SYSTEM_TIMERGROUP1_CLK_EN_S  15

/* SYSTEM_EFUSE_CLK_EN : R/W; bitpos: [14]; default: 1;
 * Set 1 to enable EFUSE clock
 */

#define SYSTEM_EFUSE_CLK_EN    (BIT(14))
#define SYSTEM_EFUSE_CLK_EN_M  (SYSTEM_EFUSE_CLK_EN_V << SYSTEM_EFUSE_CLK_EN_S)
#define SYSTEM_EFUSE_CLK_EN_V  0x00000001
#define SYSTEM_EFUSE_CLK_EN_S  14

/* SYSTEM_TIMERGROUP_CLK_EN : R/W; bitpos: [13]; default: 1;
 * Set 1 to enable TIMERGROUP clock
 */

#define SYSTEM_TIMERGROUP_CLK_EN    (BIT(13))
#define SYSTEM_TIMERGROUP_CLK_EN_M  (SYSTEM_TIMERGROUP_CLK_EN_V << SYSTEM_TIMERGROUP_CLK_EN_S)
#define SYSTEM_TIMERGROUP_CLK_EN_V  0x00000001
#define SYSTEM_TIMERGROUP_CLK_EN_S  13

/* SYSTEM_UHCI1_CLK_EN : R/W; bitpos: [12]; default: 0;
 * Set 1 to enable UHCI1 clock
 */

#define SYSTEM_UHCI1_CLK_EN    (BIT(12))
#define SYSTEM_UHCI1_CLK_EN_M  (SYSTEM_UHCI1_CLK_EN_V << SYSTEM_UHCI1_CLK_EN_S)
#define SYSTEM_UHCI1_CLK_EN_V  0x00000001
#define SYSTEM_UHCI1_CLK_EN_S  12

/* SYSTEM_LEDC_CLK_EN : R/W; bitpos: [11]; default: 0;
 * Set 1 to enable LEDC clock
 */

#define SYSTEM_LEDC_CLK_EN    (BIT(11))
#define SYSTEM_LEDC_CLK_EN_M  (SYSTEM_LEDC_CLK_EN_V << SYSTEM_LEDC_CLK_EN_S)
#define SYSTEM_LEDC_CLK_EN_V  0x00000001
#define SYSTEM_LEDC_CLK_EN_S  11

/* SYSTEM_PCNT_CLK_EN : R/W; bitpos: [10]; default: 0;
 * Set 1 to enable PCNT clock
 */

#define SYSTEM_PCNT_CLK_EN    (BIT(10))
#define SYSTEM_PCNT_CLK_EN_M  (SYSTEM_PCNT_CLK_EN_V << SYSTEM_PCNT_CLK_EN_S)
#define SYSTEM_PCNT_CLK_EN_V  0x00000001
#define SYSTEM_PCNT_CLK_EN_S  10

/* SYSTEM_RMT_CLK_EN : R/W; bitpos: [9]; default: 0;
 * Set 1 to enable RMT clock
 */

#define SYSTEM_RMT_CLK_EN    (BIT(9))
#define SYSTEM_RMT_CLK_EN_M  (SYSTEM_RMT_CLK_EN_V << SYSTEM_RMT_CLK_EN_S)
#define SYSTEM_RMT_CLK_EN_V  0x00000001
#define SYSTEM_RMT_CLK_EN_S  9

/* SYSTEM_UHCI0_CLK_EN : R/W; bitpos: [8]; default: 0;
 * Set 1 to enable UHCI0 clock
 */

#define SYSTEM_UHCI0_CLK_EN    (BIT(8))
#define SYSTEM_UHCI0_CLK_EN_M  (SYSTEM_UHCI0_CLK_EN_V << SYSTEM_UHCI0_CLK_EN_S)
#define SYSTEM_UHCI0_CLK_EN_V  0x00000001
#define SYSTEM_UHCI0_CLK_EN_S  8

/* SYSTEM_I2C_EXT0_CLK_EN : R/W; bitpos: [7]; default: 0;
 * Set 1 to enable I2C_EXT0 clock
 */

#define SYSTEM_I2C_EXT0_CLK_EN    (BIT(7))
#define SYSTEM_I2C_EXT0_CLK_EN_M  (SYSTEM_I2C_EXT0_CLK_EN_V << SYSTEM_I2C_EXT0_CLK_EN_S)
#define SYSTEM_I2C_EXT0_CLK_EN_V  0x00000001
#define SYSTEM_I2C_EXT0_CLK_EN_S  7

/* SYSTEM_SPI2_CLK_EN : R/W; bitpos: [6]; default: 1;
 * Set 1 to enable SPI2 clock
 */

#define SYSTEM_SPI2_CLK_EN    (BIT(6))
#define SYSTEM_SPI2_CLK_EN_M  (SYSTEM_SPI2_CLK_EN_V << SYSTEM_SPI2_CLK_EN_S)
#define SYSTEM_SPI2_CLK_EN_V  0x00000001
#define SYSTEM_SPI2_CLK_EN_S  6

/* SYSTEM_UART1_CLK_EN : R/W; bitpos: [5]; default: 1;
 * Set 1 to enable UART1 clock
 */

#define SYSTEM_UART1_CLK_EN    (BIT(5))
#define SYSTEM_UART1_CLK_EN_M  (SYSTEM_UART1_CLK_EN_V << SYSTEM_UART1_CLK_EN_S)
#define SYSTEM_UART1_CLK_EN_V  0x00000001
#define SYSTEM_UART1_CLK_EN_S  5

/* SYSTEM_I2S0_CLK_EN : R/W; bitpos: [4]; default: 0;
 * Set 1 to enable I2S0 clock
 */

#define SYSTEM_I2S0_CLK_EN    (BIT(4))
#define SYSTEM_I2S0_CLK_EN_M  (SYSTEM_I2S0_CLK_EN_V << SYSTEM_I2S0_CLK_EN_S)
#define SYSTEM_I2S0_CLK_EN_V  0x00000001
#define SYSTEM_I2S0_CLK_EN_S  4

/* SYSTEM_WDG_CLK_EN : R/W; bitpos: [3]; default: 1;
 * Set 1 to enable WDG clock
 */

#define SYSTEM_WDG_CLK_EN    (BIT(3))
#define SYSTEM_WDG_CLK_EN_M  (SYSTEM_WDG_CLK_EN_V << SYSTEM_WDG_CLK_EN_S)
#define SYSTEM_WDG_CLK_EN_V  0x00000001
#define SYSTEM_WDG_CLK_EN_S  3

/* SYSTEM_UART_CLK_EN : R/W; bitpos: [2]; default: 1;
 * Set 1 to enable UART clock
 */

#define SYSTEM_UART_CLK_EN    (BIT(2))
#define SYSTEM_UART_CLK_EN_M  (SYSTEM_UART_CLK_EN_V << SYSTEM_UART_CLK_EN_S)
#define SYSTEM_UART_CLK_EN_V  0x00000001
#define SYSTEM_UART_CLK_EN_S  2

/* SYSTEM_SPI01_CLK_EN : R/W; bitpos: [1]; default: 1;
 * Set 1 to enable SPI01 clock
 */

#define SYSTEM_SPI01_CLK_EN    (BIT(1))
#define SYSTEM_SPI01_CLK_EN_M  (SYSTEM_SPI01_CLK_EN_V << SYSTEM_SPI01_CLK_EN_S)
#define SYSTEM_SPI01_CLK_EN_V  0x00000001
#define SYSTEM_SPI01_CLK_EN_S  1

/* SYSTEM_TIMERS_CLK_EN : R/W; bitpos: [0]; default: 1;
 * Set 1 to enable TIMERS clock
 */

#define SYSTEM_TIMERS_CLK_EN    (BIT(0))
#define SYSTEM_TIMERS_CLK_EN_M  (SYSTEM_TIMERS_CLK_EN_V << SYSTEM_TIMERS_CLK_EN_S)
#define SYSTEM_TIMERS_CLK_EN_V  0x00000001
#define SYSTEM_TIMERS_CLK_EN_S  0

/* SYSTEM_PERIP_CLK_EN1_REG register
 * peripheral clock configuration regsiter 1
 */

#define SYSTEM_PERIP_CLK_EN1_REG (DR_REG_SYSTEM_BASE + 0x1c)

/* SYSTEM_USB_DEVICE_CLK_EN : R/W; bitpos: [10]; default: 1;
 * Set 1 to enable USB_DEVICE clock
 */

#define SYSTEM_USB_DEVICE_CLK_EN    (BIT(10))
#define SYSTEM_USB_DEVICE_CLK_EN_M  (SYSTEM_USB_DEVICE_CLK_EN_V << SYSTEM_USB_DEVICE_CLK_EN_S)
#define SYSTEM_USB_DEVICE_CLK_EN_V  0x00000001
#define SYSTEM_USB_DEVICE_CLK_EN_S  10

/* SYSTEM_UART2_CLK_EN : R/W; bitpos: [9]; default: 1;
 * Set 1 to enable UART2 clock
 */

#define SYSTEM_UART2_CLK_EN    (BIT(9))
#define SYSTEM_UART2_CLK_EN_M  (SYSTEM_UART2_CLK_EN_V << SYSTEM_UART2_CLK_EN_S)
#define SYSTEM_UART2_CLK_EN_V  0x00000001
#define SYSTEM_UART2_CLK_EN_S  9

/* SYSTEM_LCD_CAM_CLK_EN : R/W; bitpos: [8]; default: 0;
 * Set 1 to enable LCD_CAM clock
 */

#define SYSTEM_LCD_CAM_CLK_EN    (BIT(8))
#define SYSTEM_LCD_CAM_CLK_EN_M  (SYSTEM_LCD_CAM_CLK_EN_V << SYSTEM_LCD_CAM_CLK_EN_S)
#define SYSTEM_LCD_CAM_CLK_EN_V  0x00000001
#define SYSTEM_LCD_CAM_CLK_EN_S  8

/* SYSTEM_SDIO_HOST_CLK_EN : R/W; bitpos: [7]; default: 0;
 * Set 1 to enable SDIO_HOST clock
 */

#define SYSTEM_SDIO_HOST_CLK_EN    (BIT(7))
#define SYSTEM_SDIO_HOST_CLK_EN_M  (SYSTEM_SDIO_HOST_CLK_EN_V << SYSTEM_SDIO_HOST_CLK_EN_S)
#define SYSTEM_SDIO_HOST_CLK_EN_V  0x00000001
#define SYSTEM_SDIO_HOST_CLK_EN_S  7

/* SYSTEM_DMA_CLK_EN : R/W; bitpos: [6]; default: 0;
 * Set 1 to enable DMA clock
 */

#define SYSTEM_DMA_CLK_EN    (BIT(6))
#define SYSTEM_DMA_CLK_EN_M  (SYSTEM_DMA_CLK_EN_V << SYSTEM_DMA_CLK_EN_S)
#define SYSTEM_DMA_CLK_EN_V  0x00000001
#define SYSTEM_DMA_CLK_EN_S  6

/* SYSTEM_CRYPTO_HMAC_CLK_EN : R/W; bitpos: [5]; default: 0;
 * Set 1 to enable HMAC clock
 */

#define SYSTEM_CRYPTO_HMAC_CLK_EN    (BIT(5))
#define SYSTEM_CRYPTO_HMAC_CLK_EN_M  (SYSTEM_CRYPTO_HMAC_CLK_EN_V << SYSTEM_CRYPTO_HMAC_CLK_EN_S)
#define SYSTEM_CRYPTO_HMAC_CLK_EN_V  0x00000001
#define SYSTEM_CRYPTO_HMAC_CLK_EN_S  5

/* SYSTEM_CRYPTO_DS_CLK_EN : R/W; bitpos: [4]; default: 0;
 * Set 1 to enable DS clock
 */

#define SYSTEM_CRYPTO_DS_CLK_EN    (BIT(4))
#define SYSTEM_CRYPTO_DS_CLK_EN_M  (SYSTEM_CRYPTO_DS_CLK_EN_V << SYSTEM_CRYPTO_DS_CLK_EN_S)
#define SYSTEM_CRYPTO_DS_CLK_EN_V  0x00000001
#define SYSTEM_CRYPTO_DS_CLK_EN_S  4

/* SYSTEM_CRYPTO_RSA_CLK_EN : R/W; bitpos: [3]; default: 0;
 * Set 1 to enable RSA clock
 */

#define SYSTEM_CRYPTO_RSA_CLK_EN    (BIT(3))
#define SYSTEM_CRYPTO_RSA_CLK_EN_M  (SYSTEM_CRYPTO_RSA_CLK_EN_V << SYSTEM_CRYPTO_RSA_CLK_EN_S)
#define SYSTEM_CRYPTO_RSA_CLK_EN_V  0x00000001
#define SYSTEM_CRYPTO_RSA_CLK_EN_S  3

/* SYSTEM_CRYPTO_SHA_CLK_EN : R/W; bitpos: [2]; default: 0;
 * Set 1 to enable SHA clock
 */

#define SYSTEM_CRYPTO_SHA_CLK_EN    (BIT(2))
#define SYSTEM_CRYPTO_SHA_CLK_EN_M  (SYSTEM_CRYPTO_SHA_CLK_EN_V << SYSTEM_CRYPTO_SHA_CLK_EN_S)
#define SYSTEM_CRYPTO_SHA_CLK_EN_V  0x00000001
#define SYSTEM_CRYPTO_SHA_CLK_EN_S  2

/* SYSTEM_CRYPTO_AES_CLK_EN : R/W; bitpos: [1]; default: 0;
 * Set 1 to enable AES clock
 */

#define SYSTEM_CRYPTO_AES_CLK_EN    (BIT(1))
#define SYSTEM_CRYPTO_AES_CLK_EN_M  (SYSTEM_CRYPTO_AES_CLK_EN_V << SYSTEM_CRYPTO_AES_CLK_EN_S)
#define SYSTEM_CRYPTO_AES_CLK_EN_V  0x00000001
#define SYSTEM_CRYPTO_AES_CLK_EN_S  1

/* SYSTEM_PERI_BACKUP_CLK_EN : R/W; bitpos: [0]; default: 0;
 * Set 1 to enable BACKUP clock
 */

#define SYSTEM_PERI_BACKUP_CLK_EN    (BIT(0))
#define SYSTEM_PERI_BACKUP_CLK_EN_M  (SYSTEM_PERI_BACKUP_CLK_EN_V << SYSTEM_PERI_BACKUP_CLK_EN_S)
#define SYSTEM_PERI_BACKUP_CLK_EN_V  0x00000001
#define SYSTEM_PERI_BACKUP_CLK_EN_S  0

/* SYSTEM_PERIP_RST_EN0_REG register
 * peripheral reset configuration register0
 */

#define SYSTEM_PERIP_RST_EN0_REG (DR_REG_SYSTEM_BASE + 0x20)

/* SYSTEM_SPI4_RST : R/W; bitpos: [31]; default: 0;
 * Set 1 to let SPI4 reset
 */

#define SYSTEM_SPI4_RST    (BIT(31))
#define SYSTEM_SPI4_RST_M  (SYSTEM_SPI4_RST_V << SYSTEM_SPI4_RST_S)
#define SYSTEM_SPI4_RST_V  0x00000001
#define SYSTEM_SPI4_RST_S  31

/* SYSTEM_ADC2_ARB_RST : R/W; bitpos: [30]; default: 0;
 * Set 1 to let ADC2_ARB reset
 */

#define SYSTEM_ADC2_ARB_RST    (BIT(30))
#define SYSTEM_ADC2_ARB_RST_M  (SYSTEM_ADC2_ARB_RST_V << SYSTEM_ADC2_ARB_RST_S)
#define SYSTEM_ADC2_ARB_RST_V  0x00000001
#define SYSTEM_ADC2_ARB_RST_S  30

/* SYSTEM_SYSTIMER_RST : R/W; bitpos: [29]; default: 0;
 * Set 1 to let SYSTIMER reset
 */

#define SYSTEM_SYSTIMER_RST    (BIT(29))
#define SYSTEM_SYSTIMER_RST_M  (SYSTEM_SYSTIMER_RST_V << SYSTEM_SYSTIMER_RST_S)
#define SYSTEM_SYSTIMER_RST_V  0x00000001
#define SYSTEM_SYSTIMER_RST_S  29

/* SYSTEM_APB_SARADC_RST : R/W; bitpos: [28]; default: 0;
 * Set 1 to let APB_SARADC reset
 */

#define SYSTEM_APB_SARADC_RST    (BIT(28))
#define SYSTEM_APB_SARADC_RST_M  (SYSTEM_APB_SARADC_RST_V << SYSTEM_APB_SARADC_RST_S)
#define SYSTEM_APB_SARADC_RST_V  0x00000001
#define SYSTEM_APB_SARADC_RST_S  28

/* SYSTEM_SPI3_DMA_RST : R/W; bitpos: [27]; default: 0;
 * Set 1 to let SPI3 reset
 */

#define SYSTEM_SPI3_DMA_RST    (BIT(27))
#define SYSTEM_SPI3_DMA_RST_M  (SYSTEM_SPI3_DMA_RST_V << SYSTEM_SPI3_DMA_RST_S)
#define SYSTEM_SPI3_DMA_RST_V  0x00000001
#define SYSTEM_SPI3_DMA_RST_S  27

/* SYSTEM_PWM3_RST : R/W; bitpos: [26]; default: 0;
 * Set 1 to let PWM3 reset
 */

#define SYSTEM_PWM3_RST    (BIT(26))
#define SYSTEM_PWM3_RST_M  (SYSTEM_PWM3_RST_V << SYSTEM_PWM3_RST_S)
#define SYSTEM_PWM3_RST_V  0x00000001
#define SYSTEM_PWM3_RST_S  26

/* SYSTEM_PWM2_RST : R/W; bitpos: [25]; default: 0;
 * Set 1 to let PWM2 reset
 */

#define SYSTEM_PWM2_RST    (BIT(25))
#define SYSTEM_PWM2_RST_M  (SYSTEM_PWM2_RST_V << SYSTEM_PWM2_RST_S)
#define SYSTEM_PWM2_RST_V  0x00000001
#define SYSTEM_PWM2_RST_S  25

/* SYSTEM_UART_MEM_RST : R/W; bitpos: [24]; default: 0;
 * Set 1 to let UART_MEM reset
 */

#define SYSTEM_UART_MEM_RST    (BIT(24))
#define SYSTEM_UART_MEM_RST_M  (SYSTEM_UART_MEM_RST_V << SYSTEM_UART_MEM_RST_S)
#define SYSTEM_UART_MEM_RST_V  0x00000001
#define SYSTEM_UART_MEM_RST_S  24

/* SYSTEM_USB_RST : R/W; bitpos: [23]; default: 0;
 * Set 1 to let USB reset
 */

#define SYSTEM_USB_RST    (BIT(23))
#define SYSTEM_USB_RST_M  (SYSTEM_USB_RST_V << SYSTEM_USB_RST_S)
#define SYSTEM_USB_RST_V  0x00000001
#define SYSTEM_USB_RST_S  23

/* SYSTEM_SPI2_DMA_RST : R/W; bitpos: [22]; default: 0;
 * Set 1 to let SPI2 reset
 */

#define SYSTEM_SPI2_DMA_RST    (BIT(22))
#define SYSTEM_SPI2_DMA_RST_M  (SYSTEM_SPI2_DMA_RST_V << SYSTEM_SPI2_DMA_RST_S)
#define SYSTEM_SPI2_DMA_RST_V  0x00000001
#define SYSTEM_SPI2_DMA_RST_S  22

/* SYSTEM_I2S1_RST : R/W; bitpos: [21]; default: 0;
 * Set 1 to let I2S1 reset
 */

#define SYSTEM_I2S1_RST    (BIT(21))
#define SYSTEM_I2S1_RST_M  (SYSTEM_I2S1_RST_V << SYSTEM_I2S1_RST_S)
#define SYSTEM_I2S1_RST_V  0x00000001
#define SYSTEM_I2S1_RST_S  21

/* SYSTEM_PWM1_RST : R/W; bitpos: [20]; default: 0;
 * Set 1 to let PWM1 reset
 */

#define SYSTEM_PWM1_RST    (BIT(20))
#define SYSTEM_PWM1_RST_M  (SYSTEM_PWM1_RST_V << SYSTEM_PWM1_RST_S)
#define SYSTEM_PWM1_RST_V  0x00000001
#define SYSTEM_PWM1_RST_S  20

/* SYSTEM_CAN_RST : R/W; bitpos: [19]; default: 0;
 * Set 1 to let CAN reset
 */

#define SYSTEM_CAN_RST    (BIT(19))
#define SYSTEM_CAN_RST_M  (SYSTEM_CAN_RST_V << SYSTEM_CAN_RST_S)
#define SYSTEM_CAN_RST_V  0x00000001
#define SYSTEM_CAN_RST_S  19

/* SYSTEM_I2C_EXT1_RST : R/W; bitpos: [18]; default: 0;
 * Set 1 to let I2C_EXT1 reset
 */

#define SYSTEM_I2C_EXT1_RST    (BIT(18))
#define SYSTEM_I2C_EXT1_RST_M  (SYSTEM_I2C_EXT1_RST_V << SYSTEM_I2C_EXT1_RST_S)
#define SYSTEM_I2C_EXT1_RST_V  0x00000001
#define SYSTEM_I2C_EXT1_RST_S  18

/* SYSTEM_PWM0_RST : R/W; bitpos: [17]; default: 0;
 * Set 1 to let PWM0 reset
 */

#define SYSTEM_PWM0_RST    (BIT(17))
#define SYSTEM_PWM0_RST_M  (SYSTEM_PWM0_RST_V << SYSTEM_PWM0_RST_S)
#define SYSTEM_PWM0_RST_V  0x00000001
#define SYSTEM_PWM0_RST_S  17

/* SYSTEM_SPI3_RST : R/W; bitpos: [16]; default: 0;
 * Set 1 to let SPI3 reset
 */

#define SYSTEM_SPI3_RST    (BIT(16))
#define SYSTEM_SPI3_RST_M  (SYSTEM_SPI3_RST_V << SYSTEM_SPI3_RST_S)
#define SYSTEM_SPI3_RST_V  0x00000001
#define SYSTEM_SPI3_RST_S  16

/* SYSTEM_TIMERGROUP1_RST : R/W; bitpos: [15]; default: 0;
 * Set 1 to let TIMERGROUP1 reset
 */

#define SYSTEM_TIMERGROUP1_RST    (BIT(15))
#define SYSTEM_TIMERGROUP1_RST_M  (SYSTEM_TIMERGROUP1_RST_V << SYSTEM_TIMERGROUP1_RST_S)
#define SYSTEM_TIMERGROUP1_RST_V  0x00000001
#define SYSTEM_TIMERGROUP1_RST_S  15

/* SYSTEM_EFUSE_RST : R/W; bitpos: [14]; default: 0;
 * Set 1 to let EFUSE reset
 */

#define SYSTEM_EFUSE_RST    (BIT(14))
#define SYSTEM_EFUSE_RST_M  (SYSTEM_EFUSE_RST_V << SYSTEM_EFUSE_RST_S)
#define SYSTEM_EFUSE_RST_V  0x00000001
#define SYSTEM_EFUSE_RST_S  14

/* SYSTEM_TIMERGROUP_RST : R/W; bitpos: [13]; default: 0;
 * Set 1 to let TIMERGROUP reset
 */

#define SYSTEM_TIMERGROUP_RST    (BIT(13))
#define SYSTEM_TIMERGROUP_RST_M  (SYSTEM_TIMERGROUP_RST_V << SYSTEM_TIMERGROUP_RST_S)
#define SYSTEM_TIMERGROUP_RST_V  0x00000001
#define SYSTEM_TIMERGROUP_RST_S  13

/* SYSTEM_UHCI1_RST : R/W; bitpos: [12]; default: 0;
 * Set 1 to let UHCI1 reset
 */

#define SYSTEM_UHCI1_RST    (BIT(12))
#define SYSTEM_UHCI1_RST_M  (SYSTEM_UHCI1_RST_V << SYSTEM_UHCI1_RST_S)
#define SYSTEM_UHCI1_RST_V  0x00000001
#define SYSTEM_UHCI1_RST_S  12

/* SYSTEM_LEDC_RST : R/W; bitpos: [11]; default: 0;
 * Set 1 to let LEDC reset
 */

#define SYSTEM_LEDC_RST    (BIT(11))
#define SYSTEM_LEDC_RST_M  (SYSTEM_LEDC_RST_V << SYSTEM_LEDC_RST_S)
#define SYSTEM_LEDC_RST_V  0x00000001
#define SYSTEM_LEDC_RST_S  11

/* SYSTEM_PCNT_RST : R/W; bitpos: [10]; default: 0;
 * Set 1 to let PCNT reset
 */

#define SYSTEM_PCNT_RST    (BIT(10))
#define SYSTEM_PCNT_RST_M  (SYSTEM_PCNT_RST_V << SYSTEM_PCNT_RST_S)
#define SYSTEM_PCNT_RST_V  0x00000001
#define SYSTEM_PCNT_RST_S  10

/* SYSTEM_RMT_RST : R/W; bitpos: [9]; default: 0;
 * Set 1 to let RMT reset
 */

#define SYSTEM_RMT_RST    (BIT(9))
#define SYSTEM_RMT_RST_M  (SYSTEM_RMT_RST_V << SYSTEM_RMT_RST_S)
#define SYSTEM_RMT_RST_V  0x00000001
#define SYSTEM_RMT_RST_S  9

/* SYSTEM_UHCI0_RST : R/W; bitpos: [8]; default: 0;
 * Set 1 to let UHCI0 reset
 */

#define SYSTEM_UHCI0_RST    (BIT(8))
#define SYSTEM_UHCI0_RST_M  (SYSTEM_UHCI0_RST_V << SYSTEM_UHCI0_RST_S)
#define SYSTEM_UHCI0_RST_V  0x00000001
#define SYSTEM_UHCI0_RST_S  8

/* SYSTEM_I2C_EXT0_RST : R/W; bitpos: [7]; default: 0;
 * Set 1 to let I2C_EXT0 reset
 */

#define SYSTEM_I2C_EXT0_RST    (BIT(7))
#define SYSTEM_I2C_EXT0_RST_M  (SYSTEM_I2C_EXT0_RST_V << SYSTEM_I2C_EXT0_RST_S)
#define SYSTEM_I2C_EXT0_RST_V  0x00000001
#define SYSTEM_I2C_EXT0_RST_S  7

/* SYSTEM_SPI2_RST : R/W; bitpos: [6]; default: 0;
 * Set 1 to let SPI2 reset
 */

#define SYSTEM_SPI2_RST    (BIT(6))
#define SYSTEM_SPI2_RST_M  (SYSTEM_SPI2_RST_V << SYSTEM_SPI2_RST_S)
#define SYSTEM_SPI2_RST_V  0x00000001
#define SYSTEM_SPI2_RST_S  6

/* SYSTEM_UART1_RST : R/W; bitpos: [5]; default: 0;
 * Set 1 to let UART1 reset
 */

#define SYSTEM_UART1_RST    (BIT(5))
#define SYSTEM_UART1_RST_M  (SYSTEM_UART1_RST_V << SYSTEM_UART1_RST_S)
#define SYSTEM_UART1_RST_V  0x00000001
#define SYSTEM_UART1_RST_S  5

/* SYSTEM_I2S0_RST : R/W; bitpos: [4]; default: 0;
 * Set 1 to let I2S0 reset
 */

#define SYSTEM_I2S0_RST    (BIT(4))
#define SYSTEM_I2S0_RST_M  (SYSTEM_I2S0_RST_V << SYSTEM_I2S0_RST_S)
#define SYSTEM_I2S0_RST_V  0x00000001
#define SYSTEM_I2S0_RST_S  4

/* SYSTEM_WDG_RST : R/W; bitpos: [3]; default: 0;
 * Set 1 to let WDG reset
 */

#define SYSTEM_WDG_RST    (BIT(3))
#define SYSTEM_WDG_RST_M  (SYSTEM_WDG_RST_V << SYSTEM_WDG_RST_S)
#define SYSTEM_WDG_RST_V  0x00000001
#define SYSTEM_WDG_RST_S  3

/* SYSTEM_UART_RST : R/W; bitpos: [2]; default: 0;
 * Set 1 to let UART reset
 */

#define SYSTEM_UART_RST    (BIT(2))
#define SYSTEM_UART_RST_M  (SYSTEM_UART_RST_V << SYSTEM_UART_RST_S)
#define SYSTEM_UART_RST_V  0x00000001
#define SYSTEM_UART_RST_S  2

/* SYSTEM_SPI01_RST : R/W; bitpos: [1]; default: 0;
 * Set 1 to let SPI01 reset
 */

#define SYSTEM_SPI01_RST    (BIT(1))
#define SYSTEM_SPI01_RST_M  (SYSTEM_SPI01_RST_V << SYSTEM_SPI01_RST_S)
#define SYSTEM_SPI01_RST_V  0x00000001
#define SYSTEM_SPI01_RST_S  1

/* SYSTEM_TIMERS_RST : R/W; bitpos: [0]; default: 0;
 * Set 1 to let TIMERS reset
 */

#define SYSTEM_TIMERS_RST    (BIT(0))
#define SYSTEM_TIMERS_RST_M  (SYSTEM_TIMERS_RST_V << SYSTEM_TIMERS_RST_S)
#define SYSTEM_TIMERS_RST_V  0x00000001
#define SYSTEM_TIMERS_RST_S  0

/* SYSTEM_PERIP_RST_EN1_REG register
 * peripheral reset configuration regsiter 1
 */

#define SYSTEM_PERIP_RST_EN1_REG (DR_REG_SYSTEM_BASE + 0x24)

/* SYSTEM_USB_DEVICE_RST : R/W; bitpos: [10]; default: 0;
 * Set 1 to let USB_DEVICE reset
 */

#define SYSTEM_USB_DEVICE_RST    (BIT(10))
#define SYSTEM_USB_DEVICE_RST_M  (SYSTEM_USB_DEVICE_RST_V << SYSTEM_USB_DEVICE_RST_S)
#define SYSTEM_USB_DEVICE_RST_V  0x00000001
#define SYSTEM_USB_DEVICE_RST_S  10

/* SYSTEM_UART2_RST : R/W; bitpos: [9]; default: 0;
 * Set 1 to let UART2 reset
 */

#define SYSTEM_UART2_RST    (BIT(9))
#define SYSTEM_UART2_RST_M  (SYSTEM_UART2_RST_V << SYSTEM_UART2_RST_S)
#define SYSTEM_UART2_RST_V  0x00000001
#define SYSTEM_UART2_RST_S  9

/* SYSTEM_LCD_CAM_RST : R/W; bitpos: [8]; default: 1;
 * Set 1 to let LCD_CAM reset
 */

#define SYSTEM_LCD_CAM_RST    (BIT(8))
#define SYSTEM_LCD_CAM_RST_M  (SYSTEM_LCD_CAM_RST_V << SYSTEM_LCD_CAM_RST_S)
#define SYSTEM_LCD_CAM_RST_V  0x00000001
#define SYSTEM_LCD_CAM_RST_S  8

/* SYSTEM_SDIO_HOST_RST : R/W; bitpos: [7]; default: 1;
 * Set 1 to let SDIO_HOST reset
 */

#define SYSTEM_SDIO_HOST_RST    (BIT(7))
#define SYSTEM_SDIO_HOST_RST_M  (SYSTEM_SDIO_HOST_RST_V << SYSTEM_SDIO_HOST_RST_S)
#define SYSTEM_SDIO_HOST_RST_V  0x00000001
#define SYSTEM_SDIO_HOST_RST_S  7

/* SYSTEM_DMA_RST : R/W; bitpos: [6]; default: 1;
 * Set 1 to let DMA reset
 */

#define SYSTEM_DMA_RST    (BIT(6))
#define SYSTEM_DMA_RST_M  (SYSTEM_DMA_RST_V << SYSTEM_DMA_RST_S)
#define SYSTEM_DMA_RST_V  0x00000001
#define SYSTEM_DMA_RST_S  6

/* SYSTEM_CRYPTO_HMAC_RST : R/W; bitpos: [5]; default: 1;
 * Set 1 to let CRYPTO_HMAC reset
 */

#define SYSTEM_CRYPTO_HMAC_RST    (BIT(5))
#define SYSTEM_CRYPTO_HMAC_RST_M  (SYSTEM_CRYPTO_HMAC_RST_V << SYSTEM_CRYPTO_HMAC_RST_S)
#define SYSTEM_CRYPTO_HMAC_RST_V  0x00000001
#define SYSTEM_CRYPTO_HMAC_RST_S  5

/* SYSTEM_CRYPTO_DS_RST : R/W; bitpos: [4]; default: 1;
 * Set 1 to let CRYPTO_DS reset
 */

#define SYSTEM_CRYPTO_DS_RST    (BIT(4))
#define SYSTEM_CRYPTO_DS_RST_M  (SYSTEM_CRYPTO_DS_RST_V << SYSTEM_CRYPTO_DS_RST_S)
#define SYSTEM_CRYPTO_DS_RST_V  0x00000001
#define SYSTEM_CRYPTO_DS_RST_S  4

/* SYSTEM_CRYPTO_RSA_RST : R/W; bitpos: [3]; default: 1;
 * Set 1 to let CRYPTO_RSA reset
 */

#define SYSTEM_CRYPTO_RSA_RST    (BIT(3))
#define SYSTEM_CRYPTO_RSA_RST_M  (SYSTEM_CRYPTO_RSA_RST_V << SYSTEM_CRYPTO_RSA_RST_S)
#define SYSTEM_CRYPTO_RSA_RST_V  0x00000001
#define SYSTEM_CRYPTO_RSA_RST_S  3

/* SYSTEM_CRYPTO_SHA_RST : R/W; bitpos: [2]; default: 1;
 * Set 1 to let CRYPTO_SHA reset
 */

#define SYSTEM_CRYPTO_SHA_RST    (BIT(2))
#define SYSTEM_CRYPTO_SHA_RST_M  (SYSTEM_CRYPTO_SHA_RST_V << SYSTEM_CRYPTO_SHA_RST_S)
#define SYSTEM_CRYPTO_SHA_RST_V  0x00000001
#define SYSTEM_CRYPTO_SHA_RST_S  2

/* SYSTEM_CRYPTO_AES_RST : R/W; bitpos: [1]; default: 1;
 * Set 1 to let CRYPTO_AES reset
 */

#define SYSTEM_CRYPTO_AES_RST    (BIT(1))
#define SYSTEM_CRYPTO_AES_RST_M  (SYSTEM_CRYPTO_AES_RST_V << SYSTEM_CRYPTO_AES_RST_S)
#define SYSTEM_CRYPTO_AES_RST_V  0x00000001
#define SYSTEM_CRYPTO_AES_RST_S  1

/* SYSTEM_PERI_BACKUP_RST : R/W; bitpos: [0]; default: 0;
 * Set 1 to let BACKUP reset
 */

#define SYSTEM_PERI_BACKUP_RST    (BIT(0))
#define SYSTEM_PERI_BACKUP_RST_M  (SYSTEM_PERI_BACKUP_RST_V << SYSTEM_PERI_BACKUP_RST_S)
#define SYSTEM_PERI_BACKUP_RST_V  0x00000001
#define SYSTEM_PERI_BACKUP_RST_S  0

/* SYSTEM_BT_LPCK_DIV_INT_REG register
 * low power clock frequent division factor configuration regsiter
 */

#define SYSTEM_BT_LPCK_DIV_INT_REG (DR_REG_SYSTEM_BASE + 0x28)

/* SYSTEM_BT_LPCK_DIV_NUM : R/W; bitpos: [11:0]; default: 255;
 * This field is lower power clock frequent division factor
 */

#define SYSTEM_BT_LPCK_DIV_NUM    0x00000fff
#define SYSTEM_BT_LPCK_DIV_NUM_M  (SYSTEM_BT_LPCK_DIV_NUM_V << SYSTEM_BT_LPCK_DIV_NUM_S)
#define SYSTEM_BT_LPCK_DIV_NUM_V  0x00000fff
#define SYSTEM_BT_LPCK_DIV_NUM_S  0

/* SYSTEM_BT_LPCK_DIV_FRAC_REG register
 * low power clock configuration register
 */

#define SYSTEM_BT_LPCK_DIV_FRAC_REG (DR_REG_SYSTEM_BASE + 0x2c)

/* SYSTEM_LPCLK_RTC_EN : R/W; bitpos: [28]; default: 0;
 * Set 1 to enable RTC low power clock
 */

#define SYSTEM_LPCLK_RTC_EN    (BIT(28))
#define SYSTEM_LPCLK_RTC_EN_M  (SYSTEM_LPCLK_RTC_EN_V << SYSTEM_LPCLK_RTC_EN_S)
#define SYSTEM_LPCLK_RTC_EN_V  0x00000001
#define SYSTEM_LPCLK_RTC_EN_S  28

/* SYSTEM_LPCLK_SEL_XTAL32K : R/W; bitpos: [27]; default: 0;
 * Set 1 to select xtal32k clock as low power clock
 */

#define SYSTEM_LPCLK_SEL_XTAL32K    (BIT(27))
#define SYSTEM_LPCLK_SEL_XTAL32K_M  (SYSTEM_LPCLK_SEL_XTAL32K_V << SYSTEM_LPCLK_SEL_XTAL32K_S)
#define SYSTEM_LPCLK_SEL_XTAL32K_V  0x00000001
#define SYSTEM_LPCLK_SEL_XTAL32K_S  27

/* SYSTEM_LPCLK_SEL_XTAL : R/W; bitpos: [26]; default: 0;
 * Set 1 to select xtal clock as rtc low power clock
 */

#define SYSTEM_LPCLK_SEL_XTAL    (BIT(26))
#define SYSTEM_LPCLK_SEL_XTAL_M  (SYSTEM_LPCLK_SEL_XTAL_V << SYSTEM_LPCLK_SEL_XTAL_S)
#define SYSTEM_LPCLK_SEL_XTAL_V  0x00000001
#define SYSTEM_LPCLK_SEL_XTAL_S  26

/* SYSTEM_LPCLK_SEL_8M : R/W; bitpos: [25]; default: 1;
 * Set 1 to select 8m clock as rtc low power clock
 */

#define SYSTEM_LPCLK_SEL_8M    (BIT(25))
#define SYSTEM_LPCLK_SEL_8M_M  (SYSTEM_LPCLK_SEL_8M_V << SYSTEM_LPCLK_SEL_8M_S)
#define SYSTEM_LPCLK_SEL_8M_V  0x00000001
#define SYSTEM_LPCLK_SEL_8M_S  25

/* SYSTEM_LPCLK_SEL_RTC_SLOW : R/W; bitpos: [24]; default: 0;
 * Set 1 to select rtc-slow clock as rtc low power clock
 */

#define SYSTEM_LPCLK_SEL_RTC_SLOW    (BIT(24))
#define SYSTEM_LPCLK_SEL_RTC_SLOW_M  (SYSTEM_LPCLK_SEL_RTC_SLOW_V << SYSTEM_LPCLK_SEL_RTC_SLOW_S)
#define SYSTEM_LPCLK_SEL_RTC_SLOW_V  0x00000001
#define SYSTEM_LPCLK_SEL_RTC_SLOW_S  24

/* SYSTEM_BT_LPCK_DIV_A : R/W; bitpos: [23:12]; default: 1;
 * This field is lower power clock frequent division factor a
 */

#define SYSTEM_BT_LPCK_DIV_A    0x00000fff
#define SYSTEM_BT_LPCK_DIV_A_M  (SYSTEM_BT_LPCK_DIV_A_V << SYSTEM_BT_LPCK_DIV_A_S)
#define SYSTEM_BT_LPCK_DIV_A_V  0x00000fff
#define SYSTEM_BT_LPCK_DIV_A_S  12

/* SYSTEM_BT_LPCK_DIV_B : R/W; bitpos: [11:0]; default: 1;
 * This field is lower power clock frequent division factor b
 */

#define SYSTEM_BT_LPCK_DIV_B    0x00000fff
#define SYSTEM_BT_LPCK_DIV_B_M  (SYSTEM_BT_LPCK_DIV_B_V << SYSTEM_BT_LPCK_DIV_B_S)
#define SYSTEM_BT_LPCK_DIV_B_V  0x00000fff
#define SYSTEM_BT_LPCK_DIV_B_S  0

/* SYSTEM_CPU_INTR_FROM_CPU_0_REG register
 * interrupt source register 0
 */

#define SYSTEM_CPU_INTR_FROM_CPU_0_REG (DR_REG_SYSTEM_BASE + 0x30)

/* SYSTEM_CPU_INTR_FROM_CPU_0 : R/W; bitpos: [0]; default: 0;
 * Set 1 to generate cpu interrupt 0
 */

#define SYSTEM_CPU_INTR_FROM_CPU_0    (BIT(0))
#define SYSTEM_CPU_INTR_FROM_CPU_0_M  (SYSTEM_CPU_INTR_FROM_CPU_0_V << SYSTEM_CPU_INTR_FROM_CPU_0_S)
#define SYSTEM_CPU_INTR_FROM_CPU_0_V  0x00000001
#define SYSTEM_CPU_INTR_FROM_CPU_0_S  0

/* SYSTEM_CPU_INTR_FROM_CPU_1_REG register
 * interrupt source register 1
 */

#define SYSTEM_CPU_INTR_FROM_CPU_1_REG (DR_REG_SYSTEM_BASE + 0x34)

/* SYSTEM_CPU_INTR_FROM_CPU_1 : R/W; bitpos: [0]; default: 0;
 * Set 1 to generate cpu interrupt 1
 */

#define SYSTEM_CPU_INTR_FROM_CPU_1    (BIT(0))
#define SYSTEM_CPU_INTR_FROM_CPU_1_M  (SYSTEM_CPU_INTR_FROM_CPU_1_V << SYSTEM_CPU_INTR_FROM_CPU_1_S)
#define SYSTEM_CPU_INTR_FROM_CPU_1_V  0x00000001
#define SYSTEM_CPU_INTR_FROM_CPU_1_S  0

/* SYSTEM_CPU_INTR_FROM_CPU_2_REG register
 * interrupt source register 2
 */

#define SYSTEM_CPU_INTR_FROM_CPU_2_REG (DR_REG_SYSTEM_BASE + 0x38)

/* SYSTEM_CPU_INTR_FROM_CPU_2 : R/W; bitpos: [0]; default: 0;
 * Set 1 to generate cpu interrupt 2
 */

#define SYSTEM_CPU_INTR_FROM_CPU_2    (BIT(0))
#define SYSTEM_CPU_INTR_FROM_CPU_2_M  (SYSTEM_CPU_INTR_FROM_CPU_2_V << SYSTEM_CPU_INTR_FROM_CPU_2_S)
#define SYSTEM_CPU_INTR_FROM_CPU_2_V  0x00000001
#define SYSTEM_CPU_INTR_FROM_CPU_2_S  0

/* SYSTEM_CPU_INTR_FROM_CPU_3_REG register
 * interrupt source register 3
 */

#define SYSTEM_CPU_INTR_FROM_CPU_3_REG (DR_REG_SYSTEM_BASE + 0x3c)

/* SYSTEM_CPU_INTR_FROM_CPU_3 : R/W; bitpos: [0]; default: 0;
 * Set 1 to generate cpu interrupt 3
 */

#define SYSTEM_CPU_INTR_FROM_CPU_3    (BIT(0))
#define SYSTEM_CPU_INTR_FROM_CPU_3_M  (SYSTEM_CPU_INTR_FROM_CPU_3_V << SYSTEM_CPU_INTR_FROM_CPU_3_S)
#define SYSTEM_CPU_INTR_FROM_CPU_3_V  0x00000001
#define SYSTEM_CPU_INTR_FROM_CPU_3_S  0

/* SYSTEM_RSA_PD_CTRL_REG register
 * rsa memory power control register
 */

#define SYSTEM_RSA_PD_CTRL_REG (DR_REG_SYSTEM_BASE + 0x40)

/* SYSTEM_RSA_MEM_FORCE_PD : R/W; bitpos: [2]; default: 0;
 * Set 1 to force power down RSA memory,this bit has the highest priority.
 */

#define SYSTEM_RSA_MEM_FORCE_PD    (BIT(2))
#define SYSTEM_RSA_MEM_FORCE_PD_M  (SYSTEM_RSA_MEM_FORCE_PD_V << SYSTEM_RSA_MEM_FORCE_PD_S)
#define SYSTEM_RSA_MEM_FORCE_PD_V  0x00000001
#define SYSTEM_RSA_MEM_FORCE_PD_S  2

/* SYSTEM_RSA_MEM_FORCE_PU : R/W; bitpos: [1]; default: 0;
 * Set 1 to force power up RSA memory, this bit has the second highest
 * priority.
 */

#define SYSTEM_RSA_MEM_FORCE_PU    (BIT(1))
#define SYSTEM_RSA_MEM_FORCE_PU_M  (SYSTEM_RSA_MEM_FORCE_PU_V << SYSTEM_RSA_MEM_FORCE_PU_S)
#define SYSTEM_RSA_MEM_FORCE_PU_V  0x00000001
#define SYSTEM_RSA_MEM_FORCE_PU_S  1

/* SYSTEM_RSA_MEM_PD : R/W; bitpos: [0]; default: 1;
 * Set 1 to power down RSA memory. This bit has the lowest priority.When
 * Digital Signature occupies the RSA, this bit is invalid.
 */

#define SYSTEM_RSA_MEM_PD    (BIT(0))
#define SYSTEM_RSA_MEM_PD_M  (SYSTEM_RSA_MEM_PD_V << SYSTEM_RSA_MEM_PD_S)
#define SYSTEM_RSA_MEM_PD_V  0x00000001
#define SYSTEM_RSA_MEM_PD_S  0

/* SYSTEM_EDMA_CTRL_REG register
 * EDMA control register
 */

#define SYSTEM_EDMA_CTRL_REG (DR_REG_SYSTEM_BASE + 0x44)

/* SYSTEM_EDMA_RESET : R/W; bitpos: [1]; default: 0;
 * Set 1 to let EDMA reset
 */

#define SYSTEM_EDMA_RESET    (BIT(1))
#define SYSTEM_EDMA_RESET_M  (SYSTEM_EDMA_RESET_V << SYSTEM_EDMA_RESET_S)
#define SYSTEM_EDMA_RESET_V  0x00000001
#define SYSTEM_EDMA_RESET_S  1

/* SYSTEM_EDMA_CLK_ON : R/W; bitpos: [0]; default: 1;
 * Set 1 to enable EDMA clock.
 */

#define SYSTEM_EDMA_CLK_ON    (BIT(0))
#define SYSTEM_EDMA_CLK_ON_M  (SYSTEM_EDMA_CLK_ON_V << SYSTEM_EDMA_CLK_ON_S)
#define SYSTEM_EDMA_CLK_ON_V  0x00000001
#define SYSTEM_EDMA_CLK_ON_S  0

/* SYSTEM_CACHE_CONTROL_REG register
 * Cache control register
 */

#define SYSTEM_CACHE_CONTROL_REG (DR_REG_SYSTEM_BASE + 0x48)

/* SYSTEM_DCACHE_RESET : R/W; bitpos: [3]; default: 0;
 * Set 1 to let dcache reset
 */

#define SYSTEM_DCACHE_RESET    (BIT(3))
#define SYSTEM_DCACHE_RESET_M  (SYSTEM_DCACHE_RESET_V << SYSTEM_DCACHE_RESET_S)
#define SYSTEM_DCACHE_RESET_V  0x00000001
#define SYSTEM_DCACHE_RESET_S  3

/* SYSTEM_DCACHE_CLK_ON : R/W; bitpos: [2]; default: 1;
 * Set 1 to enable dcache clock
 */

#define SYSTEM_DCACHE_CLK_ON    (BIT(2))
#define SYSTEM_DCACHE_CLK_ON_M  (SYSTEM_DCACHE_CLK_ON_V << SYSTEM_DCACHE_CLK_ON_S)
#define SYSTEM_DCACHE_CLK_ON_V  0x00000001
#define SYSTEM_DCACHE_CLK_ON_S  2

/* SYSTEM_ICACHE_RESET : R/W; bitpos: [1]; default: 0;
 * Set 1 to let icache reset
 */

#define SYSTEM_ICACHE_RESET    (BIT(1))
#define SYSTEM_ICACHE_RESET_M  (SYSTEM_ICACHE_RESET_V << SYSTEM_ICACHE_RESET_S)
#define SYSTEM_ICACHE_RESET_V  0x00000001
#define SYSTEM_ICACHE_RESET_S  1

/* SYSTEM_ICACHE_CLK_ON : R/W; bitpos: [0]; default: 1;
 * Set 1 to enable icache clock
 */

#define SYSTEM_ICACHE_CLK_ON    (BIT(0))
#define SYSTEM_ICACHE_CLK_ON_M  (SYSTEM_ICACHE_CLK_ON_V << SYSTEM_ICACHE_CLK_ON_S)
#define SYSTEM_ICACHE_CLK_ON_V  0x00000001
#define SYSTEM_ICACHE_CLK_ON_S  0

/* SYSTEM_EXTERNAL_DEVICE_ENCRYPT_DECRYPT_CONTROL_REG register
 * External memory encrypt and decrypt control register
 */

#define SYSTEM_EXTERNAL_DEVICE_ENCRYPT_DECRYPT_CONTROL_REG (DR_REG_SYSTEM_BASE + 0x4c)

/* SYSTEM_ENABLE_DOWNLOAD_MANUAL_ENCRYPT : R/W; bitpos: [3]; default: 0;
 * Set 1 to enable download manual encrypt
 */

#define SYSTEM_ENABLE_DOWNLOAD_MANUAL_ENCRYPT    (BIT(3))
#define SYSTEM_ENABLE_DOWNLOAD_MANUAL_ENCRYPT_M  (SYSTEM_ENABLE_DOWNLOAD_MANUAL_ENCRYPT_V << SYSTEM_ENABLE_DOWNLOAD_MANUAL_ENCRYPT_S)
#define SYSTEM_ENABLE_DOWNLOAD_MANUAL_ENCRYPT_V  0x00000001
#define SYSTEM_ENABLE_DOWNLOAD_MANUAL_ENCRYPT_S  3

/* SYSTEM_ENABLE_DOWNLOAD_G0CB_DECRYPT : R/W; bitpos: [2]; default: 0;
 * Set 1 to enable download G0CB decrypt
 */

#define SYSTEM_ENABLE_DOWNLOAD_G0CB_DECRYPT    (BIT(2))
#define SYSTEM_ENABLE_DOWNLOAD_G0CB_DECRYPT_M  (SYSTEM_ENABLE_DOWNLOAD_G0CB_DECRYPT_V << SYSTEM_ENABLE_DOWNLOAD_G0CB_DECRYPT_S)
#define SYSTEM_ENABLE_DOWNLOAD_G0CB_DECRYPT_V  0x00000001
#define SYSTEM_ENABLE_DOWNLOAD_G0CB_DECRYPT_S  2

/* SYSTEM_ENABLE_DOWNLOAD_DB_ENCRYPT : R/W; bitpos: [1]; default: 0;
 * Set 1 to enable download DB encrypt.
 */

#define SYSTEM_ENABLE_DOWNLOAD_DB_ENCRYPT    (BIT(1))
#define SYSTEM_ENABLE_DOWNLOAD_DB_ENCRYPT_M  (SYSTEM_ENABLE_DOWNLOAD_DB_ENCRYPT_V << SYSTEM_ENABLE_DOWNLOAD_DB_ENCRYPT_S)
#define SYSTEM_ENABLE_DOWNLOAD_DB_ENCRYPT_V  0x00000001
#define SYSTEM_ENABLE_DOWNLOAD_DB_ENCRYPT_S  1

/* SYSTEM_ENABLE_SPI_MANUAL_ENCRYPT : R/W; bitpos: [0]; default: 0;
 * Set 1 to enable the SPI manual encrypt.
 */

#define SYSTEM_ENABLE_SPI_MANUAL_ENCRYPT    (BIT(0))
#define SYSTEM_ENABLE_SPI_MANUAL_ENCRYPT_M  (SYSTEM_ENABLE_SPI_MANUAL_ENCRYPT_V << SYSTEM_ENABLE_SPI_MANUAL_ENCRYPT_S)
#define SYSTEM_ENABLE_SPI_MANUAL_ENCRYPT_V  0x00000001
#define SYSTEM_ENABLE_SPI_MANUAL_ENCRYPT_S  0

/* SYSTEM_RTC_FASTMEM_CONFIG_REG register
 * RTC fast memory configuration register
 */

#define SYSTEM_RTC_FASTMEM_CONFIG_REG (DR_REG_SYSTEM_BASE + 0x50)

/* SYSTEM_RTC_MEM_CRC_FINISH : RO; bitpos: [31]; default: 0;
 * This bit stores the status of RTC memory CRC.1 means finished.
 */

#define SYSTEM_RTC_MEM_CRC_FINISH    (BIT(31))
#define SYSTEM_RTC_MEM_CRC_FINISH_M  (SYSTEM_RTC_MEM_CRC_FINISH_V << SYSTEM_RTC_MEM_CRC_FINISH_S)
#define SYSTEM_RTC_MEM_CRC_FINISH_V  0x00000001
#define SYSTEM_RTC_MEM_CRC_FINISH_S  31

/* SYSTEM_RTC_MEM_CRC_LEN : R/W; bitpos: [30:20]; default: 2047;
 * This field is used to set length of RTC memory for CRC based on start
 * address.
 */

#define SYSTEM_RTC_MEM_CRC_LEN    0x000007ff
#define SYSTEM_RTC_MEM_CRC_LEN_M  (SYSTEM_RTC_MEM_CRC_LEN_V << SYSTEM_RTC_MEM_CRC_LEN_S)
#define SYSTEM_RTC_MEM_CRC_LEN_V  0x000007ff
#define SYSTEM_RTC_MEM_CRC_LEN_S  20

/* SYSTEM_RTC_MEM_CRC_ADDR : R/W; bitpos: [19:9]; default: 0;
 * This field is used to set address of RTC memory for CRC.
 */

#define SYSTEM_RTC_MEM_CRC_ADDR    0x000007ff
#define SYSTEM_RTC_MEM_CRC_ADDR_M  (SYSTEM_RTC_MEM_CRC_ADDR_V << SYSTEM_RTC_MEM_CRC_ADDR_S)
#define SYSTEM_RTC_MEM_CRC_ADDR_V  0x000007ff
#define SYSTEM_RTC_MEM_CRC_ADDR_S  9

/* SYSTEM_RTC_MEM_CRC_START : R/W; bitpos: [8]; default: 0;
 * Set 1 to start the CRC of RTC memory
 */

#define SYSTEM_RTC_MEM_CRC_START    (BIT(8))
#define SYSTEM_RTC_MEM_CRC_START_M  (SYSTEM_RTC_MEM_CRC_START_V << SYSTEM_RTC_MEM_CRC_START_S)
#define SYSTEM_RTC_MEM_CRC_START_V  0x00000001
#define SYSTEM_RTC_MEM_CRC_START_S  8

/* SYSTEM_RTC_FASTMEM_CRC_REG register
 * RTC fast memory CRC control register
 */

#define SYSTEM_RTC_FASTMEM_CRC_REG (DR_REG_SYSTEM_BASE + 0x54)

/* SYSTEM_RTC_MEM_CRC_RES : RO; bitpos: [31:0]; default: 0;
 * This field stores the CRC result of RTC memory.
 */

#define SYSTEM_RTC_MEM_CRC_RES    0xffffffff
#define SYSTEM_RTC_MEM_CRC_RES_M  (SYSTEM_RTC_MEM_CRC_RES_V << SYSTEM_RTC_MEM_CRC_RES_S)
#define SYSTEM_RTC_MEM_CRC_RES_V  0xffffffff
#define SYSTEM_RTC_MEM_CRC_RES_S  0

/* SYSTEM_REDUNDANT_ECO_CTRL_REG register
 * ******* Description ***********
 */

#define SYSTEM_REDUNDANT_ECO_CTRL_REG (DR_REG_SYSTEM_BASE + 0x58)

/* SYSTEM_REDUNDANT_ECO_RESULT : RO; bitpos: [1]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_REDUNDANT_ECO_RESULT    (BIT(1))
#define SYSTEM_REDUNDANT_ECO_RESULT_M  (SYSTEM_REDUNDANT_ECO_RESULT_V << SYSTEM_REDUNDANT_ECO_RESULT_S)
#define SYSTEM_REDUNDANT_ECO_RESULT_V  0x00000001
#define SYSTEM_REDUNDANT_ECO_RESULT_S  1

/* SYSTEM_REDUNDANT_ECO_DRIVE : R/W; bitpos: [0]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_REDUNDANT_ECO_DRIVE    (BIT(0))
#define SYSTEM_REDUNDANT_ECO_DRIVE_M  (SYSTEM_REDUNDANT_ECO_DRIVE_V << SYSTEM_REDUNDANT_ECO_DRIVE_S)
#define SYSTEM_REDUNDANT_ECO_DRIVE_V  0x00000001
#define SYSTEM_REDUNDANT_ECO_DRIVE_S  0

/* SYSTEM_CLOCK_GATE_REG register
 * ******* Description ***********
 */

#define SYSTEM_CLOCK_GATE_REG (DR_REG_SYSTEM_BASE + 0x5c)

/* SYSTEM_CLK_EN : R/W; bitpos: [0]; default: 1;
 * ******* Description ***********
 */

#define SYSTEM_CLK_EN    (BIT(0))
#define SYSTEM_CLK_EN_M  (SYSTEM_CLK_EN_V << SYSTEM_CLK_EN_S)
#define SYSTEM_CLK_EN_V  0x00000001
#define SYSTEM_CLK_EN_S  0

/* SYSTEM_SYSCLK_CONF_REG register
 * System clock configuration register.
 */

#define SYSTEM_SYSCLK_CONF_REG (DR_REG_SYSTEM_BASE + 0x60)

/* SYSTEM_CLK_DIV_EN : RO; bitpos: [19]; default: 0;
 * Reserved.
 */

#define SYSTEM_CLK_DIV_EN    (BIT(19))
#define SYSTEM_CLK_DIV_EN_M  (SYSTEM_CLK_DIV_EN_V << SYSTEM_CLK_DIV_EN_S)
#define SYSTEM_CLK_DIV_EN_V  0x00000001
#define SYSTEM_CLK_DIV_EN_S  19

/* SYSTEM_CLK_XTAL_FREQ : RO; bitpos: [18:12]; default: 0;
 * This field is used to read xtal frequency in MHz.
 */

#define SYSTEM_CLK_XTAL_FREQ    0x0000007f
#define SYSTEM_CLK_XTAL_FREQ_M  (SYSTEM_CLK_XTAL_FREQ_V << SYSTEM_CLK_XTAL_FREQ_S)
#define SYSTEM_CLK_XTAL_FREQ_V  0x0000007f
#define SYSTEM_CLK_XTAL_FREQ_S  12

/* SYSTEM_SOC_CLK_SEL : R/W; bitpos: [11:10]; default: 0;
 * This field is used to select soc clock.
 */

#define SYSTEM_SOC_CLK_SEL    0x00000003
#define SYSTEM_SOC_CLK_SEL_M  (SYSTEM_SOC_CLK_SEL_V << SYSTEM_SOC_CLK_SEL_S)
#define SYSTEM_SOC_CLK_SEL_V  0x00000003
#define SYSTEM_SOC_CLK_SEL_S  10

/* SYSTEM_PRE_DIV_CNT : R/W; bitpos: [9:0]; default: 1;
 * This field is used to set the count of prescaler of XTAL_CLK.
 */

#define SYSTEM_PRE_DIV_CNT    0x000003ff
#define SYSTEM_PRE_DIV_CNT_M  (SYSTEM_PRE_DIV_CNT_V << SYSTEM_PRE_DIV_CNT_S)
#define SYSTEM_PRE_DIV_CNT_V  0x000003ff
#define SYSTEM_PRE_DIV_CNT_S  0

/* SYSTEM_MEM_PVT_REG register
 * ******* Description ***********
 */

#define SYSTEM_MEM_PVT_REG (DR_REG_SYSTEM_BASE + 0x64)

/* SYSTEM_MEM_VT_SEL : R/W; bitpos: [23:22]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_MEM_VT_SEL    0x00000003
#define SYSTEM_MEM_VT_SEL_M  (SYSTEM_MEM_VT_SEL_V << SYSTEM_MEM_VT_SEL_S)
#define SYSTEM_MEM_VT_SEL_V  0x00000003
#define SYSTEM_MEM_VT_SEL_S  22

/* SYSTEM_MEM_TIMING_ERR_CNT : RO; bitpos: [21:6]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_MEM_TIMING_ERR_CNT    0x0000ffff
#define SYSTEM_MEM_TIMING_ERR_CNT_M  (SYSTEM_MEM_TIMING_ERR_CNT_V << SYSTEM_MEM_TIMING_ERR_CNT_S)
#define SYSTEM_MEM_TIMING_ERR_CNT_V  0x0000ffff
#define SYSTEM_MEM_TIMING_ERR_CNT_S  6

/* SYSTEM_MEM_PVT_MONITOR_EN : R/W; bitpos: [5]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_MEM_PVT_MONITOR_EN    (BIT(5))
#define SYSTEM_MEM_PVT_MONITOR_EN_M  (SYSTEM_MEM_PVT_MONITOR_EN_V << SYSTEM_MEM_PVT_MONITOR_EN_S)
#define SYSTEM_MEM_PVT_MONITOR_EN_V  0x00000001
#define SYSTEM_MEM_PVT_MONITOR_EN_S  5

/* SYSTEM_MEM_ERR_CNT_CLR : WO; bitpos: [4]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_MEM_ERR_CNT_CLR    (BIT(4))
#define SYSTEM_MEM_ERR_CNT_CLR_M  (SYSTEM_MEM_ERR_CNT_CLR_V << SYSTEM_MEM_ERR_CNT_CLR_S)
#define SYSTEM_MEM_ERR_CNT_CLR_V  0x00000001
#define SYSTEM_MEM_ERR_CNT_CLR_S  4

/* SYSTEM_MEM_PATH_LEN : R/W; bitpos: [3:0]; default: 3;
 * ******* Description ***********
 */

#define SYSTEM_MEM_PATH_LEN    0x0000000f
#define SYSTEM_MEM_PATH_LEN_M  (SYSTEM_MEM_PATH_LEN_V << SYSTEM_MEM_PATH_LEN_S)
#define SYSTEM_MEM_PATH_LEN_V  0x0000000f
#define SYSTEM_MEM_PATH_LEN_S  0

/* SYSTEM_COMB_PVT_LVT_CONF_REG register
 * ******* Description ***********
 */

#define SYSTEM_COMB_PVT_LVT_CONF_REG (DR_REG_SYSTEM_BASE + 0x68)

/* SYSTEM_COMB_PVT_MONITOR_EN_LVT : R/W; bitpos: [6]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_COMB_PVT_MONITOR_EN_LVT    (BIT(6))
#define SYSTEM_COMB_PVT_MONITOR_EN_LVT_M  (SYSTEM_COMB_PVT_MONITOR_EN_LVT_V << SYSTEM_COMB_PVT_MONITOR_EN_LVT_S)
#define SYSTEM_COMB_PVT_MONITOR_EN_LVT_V  0x00000001
#define SYSTEM_COMB_PVT_MONITOR_EN_LVT_S  6

/* SYSTEM_COMB_ERR_CNT_CLR_LVT : WO; bitpos: [5]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_COMB_ERR_CNT_CLR_LVT    (BIT(5))
#define SYSTEM_COMB_ERR_CNT_CLR_LVT_M  (SYSTEM_COMB_ERR_CNT_CLR_LVT_V << SYSTEM_COMB_ERR_CNT_CLR_LVT_S)
#define SYSTEM_COMB_ERR_CNT_CLR_LVT_V  0x00000001
#define SYSTEM_COMB_ERR_CNT_CLR_LVT_S  5

/* SYSTEM_COMB_PATH_LEN_LVT : R/W; bitpos: [4:0]; default: 3;
 * ******* Description ***********
 */

#define SYSTEM_COMB_PATH_LEN_LVT    0x0000001f
#define SYSTEM_COMB_PATH_LEN_LVT_M  (SYSTEM_COMB_PATH_LEN_LVT_V << SYSTEM_COMB_PATH_LEN_LVT_S)
#define SYSTEM_COMB_PATH_LEN_LVT_V  0x0000001f
#define SYSTEM_COMB_PATH_LEN_LVT_S  0

/* SYSTEM_COMB_PVT_NVT_CONF_REG register
 * ******* Description ***********
 */

#define SYSTEM_COMB_PVT_NVT_CONF_REG (DR_REG_SYSTEM_BASE + 0x6c)

/* SYSTEM_COMB_PVT_MONITOR_EN_NVT : R/W; bitpos: [6]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_COMB_PVT_MONITOR_EN_NVT    (BIT(6))
#define SYSTEM_COMB_PVT_MONITOR_EN_NVT_M  (SYSTEM_COMB_PVT_MONITOR_EN_NVT_V << SYSTEM_COMB_PVT_MONITOR_EN_NVT_S)
#define SYSTEM_COMB_PVT_MONITOR_EN_NVT_V  0x00000001
#define SYSTEM_COMB_PVT_MONITOR_EN_NVT_S  6

/* SYSTEM_COMB_ERR_CNT_CLR_NVT : WO; bitpos: [5]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_COMB_ERR_CNT_CLR_NVT    (BIT(5))
#define SYSTEM_COMB_ERR_CNT_CLR_NVT_M  (SYSTEM_COMB_ERR_CNT_CLR_NVT_V << SYSTEM_COMB_ERR_CNT_CLR_NVT_S)
#define SYSTEM_COMB_ERR_CNT_CLR_NVT_V  0x00000001
#define SYSTEM_COMB_ERR_CNT_CLR_NVT_S  5

/* SYSTEM_COMB_PATH_LEN_NVT : R/W; bitpos: [4:0]; default: 3;
 * ******* Description ***********
 */

#define SYSTEM_COMB_PATH_LEN_NVT    0x0000001f
#define SYSTEM_COMB_PATH_LEN_NVT_M  (SYSTEM_COMB_PATH_LEN_NVT_V << SYSTEM_COMB_PATH_LEN_NVT_S)
#define SYSTEM_COMB_PATH_LEN_NVT_V  0x0000001f
#define SYSTEM_COMB_PATH_LEN_NVT_S  0

/* SYSTEM_COMB_PVT_HVT_CONF_REG register
 * ******* Description ***********
 */

#define SYSTEM_COMB_PVT_HVT_CONF_REG (DR_REG_SYSTEM_BASE + 0x70)

/* SYSTEM_COMB_PVT_MONITOR_EN_HVT : R/W; bitpos: [6]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_COMB_PVT_MONITOR_EN_HVT    (BIT(6))
#define SYSTEM_COMB_PVT_MONITOR_EN_HVT_M  (SYSTEM_COMB_PVT_MONITOR_EN_HVT_V << SYSTEM_COMB_PVT_MONITOR_EN_HVT_S)
#define SYSTEM_COMB_PVT_MONITOR_EN_HVT_V  0x00000001
#define SYSTEM_COMB_PVT_MONITOR_EN_HVT_S  6

/* SYSTEM_COMB_ERR_CNT_CLR_HVT : WO; bitpos: [5]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_COMB_ERR_CNT_CLR_HVT    (BIT(5))
#define SYSTEM_COMB_ERR_CNT_CLR_HVT_M  (SYSTEM_COMB_ERR_CNT_CLR_HVT_V << SYSTEM_COMB_ERR_CNT_CLR_HVT_S)
#define SYSTEM_COMB_ERR_CNT_CLR_HVT_V  0x00000001
#define SYSTEM_COMB_ERR_CNT_CLR_HVT_S  5

/* SYSTEM_COMB_PATH_LEN_HVT : R/W; bitpos: [4:0]; default: 3;
 * ******* Description ***********
 */

#define SYSTEM_COMB_PATH_LEN_HVT    0x0000001f
#define SYSTEM_COMB_PATH_LEN_HVT_M  (SYSTEM_COMB_PATH_LEN_HVT_V << SYSTEM_COMB_PATH_LEN_HVT_S)
#define SYSTEM_COMB_PATH_LEN_HVT_V  0x0000001f
#define SYSTEM_COMB_PATH_LEN_HVT_S  0

/* SYSTEM_COMB_PVT_ERR_LVT_SITE0_REG register
 * ******* Description ***********
 */

#define SYSTEM_COMB_PVT_ERR_LVT_SITE0_REG (DR_REG_SYSTEM_BASE + 0x74)

/* SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE0 : RO; bitpos: [15:0]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE0    0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE0_M  (SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE0_V << SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE0_S)
#define SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE0_V  0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE0_S  0

/* SYSTEM_COMB_PVT_ERR_NVT_SITE0_REG register
 * ******* Description ***********
 */

#define SYSTEM_COMB_PVT_ERR_NVT_SITE0_REG (DR_REG_SYSTEM_BASE + 0x78)

/* SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE0 : RO; bitpos: [15:0]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE0    0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE0_M  (SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE0_V << SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE0_S)
#define SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE0_V  0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE0_S  0

/* SYSTEM_COMB_PVT_ERR_HVT_SITE0_REG register
 * ******* Description ***********
 */

#define SYSTEM_COMB_PVT_ERR_HVT_SITE0_REG (DR_REG_SYSTEM_BASE + 0x7c)

/* SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE0 : RO; bitpos: [15:0]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE0    0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE0_M  (SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE0_V << SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE0_S)
#define SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE0_V  0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE0_S  0

/* SYSTEM_COMB_PVT_ERR_LVT_SITE1_REG register
 * ******* Description ***********
 */

#define SYSTEM_COMB_PVT_ERR_LVT_SITE1_REG (DR_REG_SYSTEM_BASE + 0x80)

/* SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE1 : RO; bitpos: [15:0]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE1    0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE1_M  (SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE1_V << SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE1_S)
#define SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE1_V  0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE1_S  0

/* SYSTEM_COMB_PVT_ERR_NVT_SITE1_REG register
 * ******* Description ***********
 */

#define SYSTEM_COMB_PVT_ERR_NVT_SITE1_REG (DR_REG_SYSTEM_BASE + 0x84)

/* SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE1 : RO; bitpos: [15:0]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE1    0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE1_M  (SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE1_V << SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE1_S)
#define SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE1_V  0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE1_S  0

/* SYSTEM_COMB_PVT_ERR_HVT_SITE1_REG register
 * ******* Description ***********
 */

#define SYSTEM_COMB_PVT_ERR_HVT_SITE1_REG (DR_REG_SYSTEM_BASE + 0x88)

/* SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE1 : RO; bitpos: [15:0]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE1    0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE1_M  (SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE1_V << SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE1_S)
#define SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE1_V  0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE1_S  0

/* SYSTEM_COMB_PVT_ERR_LVT_SITE2_REG register
 * ******* Description ***********
 */

#define SYSTEM_COMB_PVT_ERR_LVT_SITE2_REG (DR_REG_SYSTEM_BASE + 0x8c)

/* SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE2 : RO; bitpos: [15:0]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE2    0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE2_M  (SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE2_V << SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE2_S)
#define SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE2_V  0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE2_S  0

/* SYSTEM_COMB_PVT_ERR_NVT_SITE2_REG register
 * ******* Description ***********
 */

#define SYSTEM_COMB_PVT_ERR_NVT_SITE2_REG (DR_REG_SYSTEM_BASE + 0x90)

/* SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE2 : RO; bitpos: [15:0]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE2    0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE2_M  (SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE2_V << SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE2_S)
#define SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE2_V  0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE2_S  0

/* SYSTEM_COMB_PVT_ERR_HVT_SITE2_REG register
 * ******* Description ***********
 */

#define SYSTEM_COMB_PVT_ERR_HVT_SITE2_REG (DR_REG_SYSTEM_BASE + 0x94)

/* SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE2 : RO; bitpos: [15:0]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE2    0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE2_M  (SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE2_V << SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE2_S)
#define SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE2_V  0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE2_S  0

/* SYSTEM_COMB_PVT_ERR_LVT_SITE3_REG register
 * ******* Description ***********
 */

#define SYSTEM_COMB_PVT_ERR_LVT_SITE3_REG (DR_REG_SYSTEM_BASE + 0x98)

/* SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE3 : RO; bitpos: [15:0]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE3    0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE3_M  (SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE3_V << SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE3_S)
#define SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE3_V  0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_LVT_SITE3_S  0

/* SYSTEM_COMB_PVT_ERR_NVT_SITE3_REG register
 * ******* Description ***********
 */

#define SYSTEM_COMB_PVT_ERR_NVT_SITE3_REG (DR_REG_SYSTEM_BASE + 0x9c)

/* SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE3 : RO; bitpos: [15:0]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE3    0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE3_M  (SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE3_V << SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE3_S)
#define SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE3_V  0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_NVT_SITE3_S  0

/* SYSTEM_COMB_PVT_ERR_HVT_SITE3_REG register
 * ******* Description ***********
 */

#define SYSTEM_COMB_PVT_ERR_HVT_SITE3_REG (DR_REG_SYSTEM_BASE + 0xa0)

/* SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE3 : RO; bitpos: [15:0]; default: 0;
 * ******* Description ***********
 */

#define SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE3    0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE3_M  (SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE3_V << SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE3_S)
#define SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE3_V  0x0000ffff
#define SYSTEM_COMB_TIMING_ERR_CNT_HVT_SITE3_S  0

/* SYSTEM_DATE_REG register
 * version register
 */

#define SYSTEM_DATE_REG (DR_REG_SYSTEM_BASE + 0xffc)

/* SYSTEM_DATE : R/W; bitpos: [27:0]; default: 34607648;
 * version register
 */

#define SYSTEM_DATE    0x0fffffff
#define SYSTEM_DATE_M  (SYSTEM_DATE_V << SYSTEM_DATE_S)
#define SYSTEM_DATE_V  0x0fffffff
#define SYSTEM_DATE_S  0

#endif /* __ARCH_XTENSA_SRC_ESP32S3_HARDWARE_ESP32S3_SYSTEM_H */
