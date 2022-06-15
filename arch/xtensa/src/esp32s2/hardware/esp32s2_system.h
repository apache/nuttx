/****************************************************************************
 * arch/xtensa/src/esp32s2/hardware/esp32s2_system.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_SYSTEM_H
#define __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_SYSTEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32s2_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SYSTEM_ROM_CTRL_0_REG register
 * System ROM configuration register 0
 */

#define SYSTEM_ROM_CTRL_0_REG (DR_REG_SYSTEM_BASE + 0x0)

/* SYSTEM_ROM_FO : R/W; bitpos: [1:0]; default: 3;
 * This field is used to force on clock gate of internal ROM.
 */

#define SYSTEM_ROM_FO    0x00000003
#define SYSTEM_ROM_FO_M  (SYSTEM_ROM_FO_V << SYSTEM_ROM_FO_S)
#define SYSTEM_ROM_FO_V  0x00000003
#define SYSTEM_ROM_FO_S  0

/* SYSTEM_ROM_CTRL_1_REG register
 * System ROM configuration register 1
 */

#define SYSTEM_ROM_CTRL_1_REG (DR_REG_SYSTEM_BASE + 0x4)

/* SYSTEM_ROM_FORCE_PU : R/W; bitpos: [3:2]; default: 3;
 * This field is used to power up internal ROM.
 */

#define SYSTEM_ROM_FORCE_PU    0x00000003
#define SYSTEM_ROM_FORCE_PU_M  (SYSTEM_ROM_FORCE_PU_V << SYSTEM_ROM_FORCE_PU_S)
#define SYSTEM_ROM_FORCE_PU_V  0x00000003
#define SYSTEM_ROM_FORCE_PU_S  2

/* SYSTEM_ROM_FORCE_PD : R/W; bitpos: [1:0]; default: 0;
 * This field is used to power down internal ROM.
 */

#define SYSTEM_ROM_FORCE_PD    0x00000003
#define SYSTEM_ROM_FORCE_PD_M  (SYSTEM_ROM_FORCE_PD_V << SYSTEM_ROM_FORCE_PD_S)
#define SYSTEM_ROM_FORCE_PD_V  0x00000003
#define SYSTEM_ROM_FORCE_PD_S  0

/* SYSTEM_SRAM_CTRL_0_REG register
 * System SRAM configuration register 0
 */

#define SYSTEM_SRAM_CTRL_0_REG (DR_REG_SYSTEM_BASE + 0x8)

/* SYSTEM_SRAM_FO : R/W; bitpos: [21:0]; default: 4194303;
 * This field is used to force on clock gate of internal SRAM.
 */

#define SYSTEM_SRAM_FO    0x003fffff
#define SYSTEM_SRAM_FO_M  (SYSTEM_SRAM_FO_V << SYSTEM_SRAM_FO_S)
#define SYSTEM_SRAM_FO_V  0x003fffff
#define SYSTEM_SRAM_FO_S  0

/* SYSTEM_SRAM_CTRL_1_REG register
 * System SRAM configuration register 1
 */

#define SYSTEM_SRAM_CTRL_1_REG (DR_REG_SYSTEM_BASE + 0xc)

/* SYSTEM_SRAM_FORCE_PD : R/W; bitpos: [21:0]; default: 0;
 * This field is used to power down internal SRAM.
 */

#define SYSTEM_SRAM_FORCE_PD    0x003fffff
#define SYSTEM_SRAM_FORCE_PD_M  (SYSTEM_SRAM_FORCE_PD_V << SYSTEM_SRAM_FORCE_PD_S)
#define SYSTEM_SRAM_FORCE_PD_V  0x003fffff
#define SYSTEM_SRAM_FORCE_PD_S  0

/* SYSTEM_CPU_PERI_CLK_EN_REG register
 * CPU peripheral clock enable register
 */

#define SYSTEM_CPU_PERI_CLK_EN_REG (DR_REG_SYSTEM_BASE + 0x10)

/* SYSTEM_CLK_EN_DEDICATED_GPIO : R/W; bitpos: [7]; default: 0;
 * Set this bit to enable clock of DEDICATED GPIO module.
 */

#define SYSTEM_CLK_EN_DEDICATED_GPIO    (BIT(7))
#define SYSTEM_CLK_EN_DEDICATED_GPIO_M  (SYSTEM_CLK_EN_DEDICATED_GPIO_V << SYSTEM_CLK_EN_DEDICATED_GPIO_S)
#define SYSTEM_CLK_EN_DEDICATED_GPIO_V  0x00000001
#define SYSTEM_CLK_EN_DEDICATED_GPIO_S  7

/* SYSTEM_CPU_PERI_RST_EN_REG register
 * CPU peripheral reset register
 */

#define SYSTEM_CPU_PERI_RST_EN_REG (DR_REG_SYSTEM_BASE + 0x14)

/* SYSTEM_RST_EN_DEDICATED_GPIO : R/W; bitpos: [7]; default: 1;
 * Set this bit to reset DEDICATED GPIO module.
 */

#define SYSTEM_RST_EN_DEDICATED_GPIO    (BIT(7))
#define SYSTEM_RST_EN_DEDICATED_GPIO_M  (SYSTEM_RST_EN_DEDICATED_GPIO_V << SYSTEM_RST_EN_DEDICATED_GPIO_S)
#define SYSTEM_RST_EN_DEDICATED_GPIO_V  0x00000001
#define SYSTEM_RST_EN_DEDICATED_GPIO_S  7

/* SYSTEM_CPU_PER_CONF_REG register
 * CPU peripheral clock configuration register
 */

#define SYSTEM_CPU_PER_CONF_REG (DR_REG_SYSTEM_BASE + 0x18)

/* SYSTEM_CPU_WAITI_DELAY_NUM : R/W; bitpos: [7:4]; default: 0;
 * Sets the number of delay cycles to enter CPU wait mode after a WAITI
 * instruction.
 */

#define SYSTEM_CPU_WAITI_DELAY_NUM    0x0000000f
#define SYSTEM_CPU_WAITI_DELAY_NUM_M  (SYSTEM_CPU_WAITI_DELAY_NUM_V << SYSTEM_CPU_WAITI_DELAY_NUM_S)
#define SYSTEM_CPU_WAITI_DELAY_NUM_V  0x0000000f
#define SYSTEM_CPU_WAITI_DELAY_NUM_S  4

/* SYSTEM_CPU_WAIT_MODE_FORCE_ON : R/W; bitpos: [3]; default: 1;
 * Set this bit to force on CPU wait mode. In this mode, the clock gate of
 * CPU is turned off until any interrupts happen. This mode could also be
 * force on via WAITI instruction.
 */

#define SYSTEM_CPU_WAIT_MODE_FORCE_ON    (BIT(3))
#define SYSTEM_CPU_WAIT_MODE_FORCE_ON_M  (SYSTEM_CPU_WAIT_MODE_FORCE_ON_V << SYSTEM_CPU_WAIT_MODE_FORCE_ON_S)
#define SYSTEM_CPU_WAIT_MODE_FORCE_ON_V  0x00000001
#define SYSTEM_CPU_WAIT_MODE_FORCE_ON_S  3

/* SYSTEM_PLL_FREQ_SEL : R/W; bitpos: [2]; default: 1;
 * This field is used to select the PLL clock frequency based on CPU period.
 */

#define SYSTEM_PLL_FREQ_SEL    (BIT(2))
#define SYSTEM_PLL_FREQ_SEL_M  (SYSTEM_PLL_FREQ_SEL_V << SYSTEM_PLL_FREQ_SEL_S)
#define SYSTEM_PLL_FREQ_SEL_V  0x00000001
#define SYSTEM_PLL_FREQ_SEL_S  2

/* SYSTEM_CPUPERIOD_SEL : R/W; bitpos: [1:0]; default: 0;
 * This field is used to select the clock frequency of CPU or CPU period.
 */

#define SYSTEM_CPUPERIOD_SEL    0x00000003
#define SYSTEM_CPUPERIOD_SEL_M  (SYSTEM_CPUPERIOD_SEL_V << SYSTEM_CPUPERIOD_SEL_S)
#define SYSTEM_CPUPERIOD_SEL_V  0x00000003
#define SYSTEM_CPUPERIOD_SEL_S  0

/* SYSTEM_JTAG_CTRL_0_REG register
 * JTAG configuration register 0
 */

#define SYSTEM_JTAG_CTRL_0_REG (DR_REG_SYSTEM_BASE + 0x1c)

/* SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_0 : WOR; bitpos: [31:0];
 * default: 0;
 * Stores the 0 to 31 bits of the 256 bits register used to cancel the
 * temporary disable of eFuse to JTAG.
 */

#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_0    0xffffffff
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_0_M  (SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_0_V << SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_0_S)
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_0_V  0xffffffff
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_0_S  0

/* SYSTEM_JTAG_CTRL_1_REG register
 * JTAG configuration register 1
 */

#define SYSTEM_JTAG_CTRL_1_REG (DR_REG_SYSTEM_BASE + 0x20)

/* SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_1 : WOR; bitpos: [31:0];
 * default: 0;
 * Stores the 32 to 63 bits of the 256 bits register used to cancel the
 * temporary disable of eFuse to JTAG.
 */

#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_1    0xffffffff
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_1_M  (SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_1_V << SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_1_S)
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_1_V  0xffffffff
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_1_S  0

/* SYSTEM_JTAG_CTRL_2_REG register
 * JTAG configuration register 2
 */

#define SYSTEM_JTAG_CTRL_2_REG (DR_REG_SYSTEM_BASE + 0x24)

/* SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_2 : WOR; bitpos: [31:0];
 * default: 0;
 * Stores the 64 to 95 bits of the 256 bits register used to cancel the
 * temporary disable of eFuse to JTAG.
 */

#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_2    0xffffffff
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_2_M  (SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_2_V << SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_2_S)
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_2_V  0xffffffff
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_2_S  0

/* SYSTEM_JTAG_CTRL_3_REG register
 * JTAG configuration register 3
 */

#define SYSTEM_JTAG_CTRL_3_REG (DR_REG_SYSTEM_BASE + 0x28)

/* SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_3 : WOR; bitpos: [31:0];
 * default: 0;
 * Stores the 96 to 127 bits of the 256 bits register used to cancel the
 * temporary disable of eFuse to JTAG.
 */

#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_3    0xffffffff
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_3_M  (SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_3_V << SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_3_S)
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_3_V  0xffffffff
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_3_S  0

/* SYSTEM_JTAG_CTRL_4_REG register
 * JTAG configuration register 4
 */

#define SYSTEM_JTAG_CTRL_4_REG (DR_REG_SYSTEM_BASE + 0x2c)

/* SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_4 : WOR; bitpos: [31:0];
 * default: 0;
 * Stores the 128 to 159 bits of the 256 bits register used to cancel the
 * temporary disable of eFuse to JTAG.
 */

#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_4    0xffffffff
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_4_M  (SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_4_V << SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_4_S)
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_4_V  0xffffffff
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_4_S  0

/* SYSTEM_JTAG_CTRL_5_REG register
 * JTAG configuration register 5
 */

#define SYSTEM_JTAG_CTRL_5_REG (DR_REG_SYSTEM_BASE + 0x30)

/* SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_5 : WOR; bitpos: [31:0];
 * default: 0;
 * Stores the 160 to 191 bits of the 256 bits register used to cancel the
 * temporary disable of eFuse to JTAG.
 */

#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_5    0xffffffff
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_5_M  (SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_5_V << SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_5_S)
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_5_V  0xffffffff
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_5_S  0

/* SYSTEM_JTAG_CTRL_6_REG register
 * JTAG configuration register 6
 */

#define SYSTEM_JTAG_CTRL_6_REG (DR_REG_SYSTEM_BASE + 0x34)

/* SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_6 : WOR; bitpos: [31:0];
 * default: 0;
 * Stores the 192 to 223 bits of the 256 bits register used to cancel the
 * temporary disable of eFuse to JTAG.
 */

#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_6    0xffffffff
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_6_M  (SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_6_V << SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_6_S)
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_6_V  0xffffffff
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_6_S  0

/* SYSTEM_JTAG_CTRL_7_REG register
 * JTAG configuration register 7
 */

#define SYSTEM_JTAG_CTRL_7_REG (DR_REG_SYSTEM_BASE + 0x38)

/* SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_7 : WOR; bitpos: [31:0];
 * default: 0;
 * Stores the 0 to 224 bits of the 255 bits register used to cancel the
 * temporary disable of eFuse to JTAG.
 */

#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_7    0xffffffff
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_7_M  (SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_7_V << SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_7_S)
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_7_V  0xffffffff
#define SYSTEM_CANCEL_EFUSE_DISABLE_JTAG_TEMPORARY_7_S  0

/* SYSTEM_MEM_PD_MASK_REG register
 * Memory power-related controlling register (under low-sleep)
 */

#define SYSTEM_MEM_PD_MASK_REG (DR_REG_SYSTEM_BASE + 0x3c)

/* SYSTEM_LSLP_MEM_PD_MASK : R/W; bitpos: [0]; default: 1;
 * Set this bit to allow the memory to work as usual when the chip enters
 * the light-sleep state.
 */

#define SYSTEM_LSLP_MEM_PD_MASK    (BIT(0))
#define SYSTEM_LSLP_MEM_PD_MASK_M  (SYSTEM_LSLP_MEM_PD_MASK_V << SYSTEM_LSLP_MEM_PD_MASK_S)
#define SYSTEM_LSLP_MEM_PD_MASK_V  0x00000001
#define SYSTEM_LSLP_MEM_PD_MASK_S  0

/* SYSTEM_PERIP_CLK_EN0_REG register
 * System peripheral clock (for hardware accelerators) enable register
 */

#define SYSTEM_PERIP_CLK_EN0_REG (DR_REG_SYSTEM_BASE + 0x40)

/* SYSTEM_SPI4_CLK_EN : R/W; bitpos: [31]; default: 1;
 * Set this bit to enable clock of SPI4.
 */

#define SYSTEM_SPI4_CLK_EN    (BIT(31))
#define SYSTEM_SPI4_CLK_EN_M  (SYSTEM_SPI4_CLK_EN_V << SYSTEM_SPI4_CLK_EN_S)
#define SYSTEM_SPI4_CLK_EN_V  0x00000001
#define SYSTEM_SPI4_CLK_EN_S  31

/* SYSTEM_ADC2_ARB_CLK_EN : R/W; bitpos: [30]; default: 1;
 * Set this bit to enable clock of aribiter of ADC2.
 */

#define SYSTEM_ADC2_ARB_CLK_EN    (BIT(30))
#define SYSTEM_ADC2_ARB_CLK_EN_M  (SYSTEM_ADC2_ARB_CLK_EN_V << SYSTEM_ADC2_ARB_CLK_EN_S)
#define SYSTEM_ADC2_ARB_CLK_EN_V  0x00000001
#define SYSTEM_ADC2_ARB_CLK_EN_S  30

/* SYSTEM_SYSTIMER_CLK_EN : R/W; bitpos: [29]; default: 1;
 * Set this bit to enable clock of system timer.
 */

#define SYSTEM_SYSTIMER_CLK_EN    (BIT(29))
#define SYSTEM_SYSTIMER_CLK_EN_M  (SYSTEM_SYSTIMER_CLK_EN_V << SYSTEM_SYSTIMER_CLK_EN_S)
#define SYSTEM_SYSTIMER_CLK_EN_V  0x00000001
#define SYSTEM_SYSTIMER_CLK_EN_S  29

/* SYSTEM_APB_SARADC_CLK_EN : R/W; bitpos: [28]; default: 1;
 * Set this bit to enable clock of SAR ADC.
 */

#define SYSTEM_APB_SARADC_CLK_EN    (BIT(28))
#define SYSTEM_APB_SARADC_CLK_EN_M  (SYSTEM_APB_SARADC_CLK_EN_V << SYSTEM_APB_SARADC_CLK_EN_S)
#define SYSTEM_APB_SARADC_CLK_EN_V  0x00000001
#define SYSTEM_APB_SARADC_CLK_EN_S  28

/* SYSTEM_SPI3_DMA_CLK_EN : R/W; bitpos: [27]; default: 1;
 * Set this bit to enable clock of SPI3 DMA.
 */

#define SYSTEM_SPI3_DMA_CLK_EN    (BIT(27))
#define SYSTEM_SPI3_DMA_CLK_EN_M  (SYSTEM_SPI3_DMA_CLK_EN_V << SYSTEM_SPI3_DMA_CLK_EN_S)
#define SYSTEM_SPI3_DMA_CLK_EN_V  0x00000001
#define SYSTEM_SPI3_DMA_CLK_EN_S  27

/* SYSTEM_PWM3_CLK_EN : R/W; bitpos: [26]; default: 0;
 * Set this bit to enable clock of PWM3.
 */

#define SYSTEM_PWM3_CLK_EN    (BIT(26))
#define SYSTEM_PWM3_CLK_EN_M  (SYSTEM_PWM3_CLK_EN_V << SYSTEM_PWM3_CLK_EN_S)
#define SYSTEM_PWM3_CLK_EN_V  0x00000001
#define SYSTEM_PWM3_CLK_EN_S  26

/* SYSTEM_PWM2_CLK_EN : R/W; bitpos: [25]; default: 0;
 * Set this bit to enable clock of PWM2.
 */

#define SYSTEM_PWM2_CLK_EN    (BIT(25))
#define SYSTEM_PWM2_CLK_EN_M  (SYSTEM_PWM2_CLK_EN_V << SYSTEM_PWM2_CLK_EN_S)
#define SYSTEM_PWM2_CLK_EN_V  0x00000001
#define SYSTEM_PWM2_CLK_EN_S  25

/* SYSTEM_UART_MEM_CLK_EN : R/W; bitpos: [24]; default: 1;
 * Set this bit to enable clock of UART memory.
 */

#define SYSTEM_UART_MEM_CLK_EN    (BIT(24))
#define SYSTEM_UART_MEM_CLK_EN_M  (SYSTEM_UART_MEM_CLK_EN_V << SYSTEM_UART_MEM_CLK_EN_S)
#define SYSTEM_UART_MEM_CLK_EN_V  0x00000001
#define SYSTEM_UART_MEM_CLK_EN_S  24

/* SYSTEM_USB_CLK_EN : R/W; bitpos: [23]; default: 1;
 * Set this bit to enable clock of USB.
 */

#define SYSTEM_USB_CLK_EN    (BIT(23))
#define SYSTEM_USB_CLK_EN_M  (SYSTEM_USB_CLK_EN_V << SYSTEM_USB_CLK_EN_S)
#define SYSTEM_USB_CLK_EN_V  0x00000001
#define SYSTEM_USB_CLK_EN_S  23

/* SYSTEM_SPI2_DMA_CLK_EN : R/W; bitpos: [22]; default: 1;
 * Set this bit to enable clock of SPI2 DMA.
 */

#define SYSTEM_SPI2_DMA_CLK_EN    (BIT(22))
#define SYSTEM_SPI2_DMA_CLK_EN_M  (SYSTEM_SPI2_DMA_CLK_EN_V << SYSTEM_SPI2_DMA_CLK_EN_S)
#define SYSTEM_SPI2_DMA_CLK_EN_V  0x00000001
#define SYSTEM_SPI2_DMA_CLK_EN_S  22

/* SYSTEM_I2S1_CLK_EN : R/W; bitpos: [21]; default: 0;
 * Set this bit to enable clock of I2S1.
 */

#define SYSTEM_I2S1_CLK_EN    (BIT(21))
#define SYSTEM_I2S1_CLK_EN_M  (SYSTEM_I2S1_CLK_EN_V << SYSTEM_I2S1_CLK_EN_S)
#define SYSTEM_I2S1_CLK_EN_V  0x00000001
#define SYSTEM_I2S1_CLK_EN_S  21

/* SYSTEM_PWM1_CLK_EN : R/W; bitpos: [20]; default: 0;
 * Set this bit to enable clock of PWM1.
 */

#define SYSTEM_PWM1_CLK_EN    (BIT(20))
#define SYSTEM_PWM1_CLK_EN_M  (SYSTEM_PWM1_CLK_EN_V << SYSTEM_PWM1_CLK_EN_S)
#define SYSTEM_PWM1_CLK_EN_V  0x00000001
#define SYSTEM_PWM1_CLK_EN_S  20

/* SYSTEM_CAN_CLK_EN : R/W; bitpos: [19]; default: 0;
 * Set this bit to enable clock of CAN.
 */

#define SYSTEM_CAN_CLK_EN    (BIT(19))
#define SYSTEM_CAN_CLK_EN_M  (SYSTEM_CAN_CLK_EN_V << SYSTEM_CAN_CLK_EN_S)
#define SYSTEM_CAN_CLK_EN_V  0x00000001
#define SYSTEM_CAN_CLK_EN_S  19

/* SYSTEM_I2C_EXT1_CLK_EN : R/W; bitpos: [18]; default: 0;
 * Set this bit to enable clock of I2C EXT1.
 */

#define SYSTEM_I2C_EXT1_CLK_EN    (BIT(18))
#define SYSTEM_I2C_EXT1_CLK_EN_M  (SYSTEM_I2C_EXT1_CLK_EN_V << SYSTEM_I2C_EXT1_CLK_EN_S)
#define SYSTEM_I2C_EXT1_CLK_EN_V  0x00000001
#define SYSTEM_I2C_EXT1_CLK_EN_S  18

/* SYSTEM_PWM0_CLK_EN : R/W; bitpos: [17]; default: 0;
 * Set this bit to enable clock of PWM0.
 */

#define SYSTEM_PWM0_CLK_EN    (BIT(17))
#define SYSTEM_PWM0_CLK_EN_M  (SYSTEM_PWM0_CLK_EN_V << SYSTEM_PWM0_CLK_EN_S)
#define SYSTEM_PWM0_CLK_EN_V  0x00000001
#define SYSTEM_PWM0_CLK_EN_S  17

/* SYSTEM_SPI3_CLK_EN : R/W; bitpos: [16]; default: 1;
 * Set this bit to enable clock of SPI3.
 */

#define SYSTEM_SPI3_CLK_EN    (BIT(16))
#define SYSTEM_SPI3_CLK_EN_M  (SYSTEM_SPI3_CLK_EN_V << SYSTEM_SPI3_CLK_EN_S)
#define SYSTEM_SPI3_CLK_EN_V  0x00000001
#define SYSTEM_SPI3_CLK_EN_S  16

/* SYSTEM_TIMERGROUP1_CLK_EN : R/W; bitpos: [15]; default: 1;
 * Set this bit to enable clock of timer group1.
 */

#define SYSTEM_TIMERGROUP1_CLK_EN    (BIT(15))
#define SYSTEM_TIMERGROUP1_CLK_EN_M  (SYSTEM_TIMERGROUP1_CLK_EN_V << SYSTEM_TIMERGROUP1_CLK_EN_S)
#define SYSTEM_TIMERGROUP1_CLK_EN_V  0x00000001
#define SYSTEM_TIMERGROUP1_CLK_EN_S  15

/* SYSTEM_EFUSE_CLK_EN : R/W; bitpos: [14]; default: 1;
 * Set this bit to enable clock of eFuse.
 */

#define SYSTEM_EFUSE_CLK_EN    (BIT(14))
#define SYSTEM_EFUSE_CLK_EN_M  (SYSTEM_EFUSE_CLK_EN_V << SYSTEM_EFUSE_CLK_EN_S)
#define SYSTEM_EFUSE_CLK_EN_V  0x00000001
#define SYSTEM_EFUSE_CLK_EN_S  14

/* SYSTEM_TIMERGROUP_CLK_EN : R/W; bitpos: [13]; default: 1;
 * Set this bit to enable clock of timer group0.
 */

#define SYSTEM_TIMERGROUP_CLK_EN    (BIT(13))
#define SYSTEM_TIMERGROUP_CLK_EN_M  (SYSTEM_TIMERGROUP_CLK_EN_V << SYSTEM_TIMERGROUP_CLK_EN_S)
#define SYSTEM_TIMERGROUP_CLK_EN_V  0x00000001
#define SYSTEM_TIMERGROUP_CLK_EN_S  13

/* SYSTEM_UHCI1_CLK_EN : R/W; bitpos: [12]; default: 0;
 * Set this bit to enable clock of UHCI1.
 */

#define SYSTEM_UHCI1_CLK_EN    (BIT(12))
#define SYSTEM_UHCI1_CLK_EN_M  (SYSTEM_UHCI1_CLK_EN_V << SYSTEM_UHCI1_CLK_EN_S)
#define SYSTEM_UHCI1_CLK_EN_V  0x00000001
#define SYSTEM_UHCI1_CLK_EN_S  12

/* SYSTEM_LEDC_CLK_EN : R/W; bitpos: [11]; default: 0;
 * Set this bit to enable clock of LED PWM.
 */

#define SYSTEM_LEDC_CLK_EN    (BIT(11))
#define SYSTEM_LEDC_CLK_EN_M  (SYSTEM_LEDC_CLK_EN_V << SYSTEM_LEDC_CLK_EN_S)
#define SYSTEM_LEDC_CLK_EN_V  0x00000001
#define SYSTEM_LEDC_CLK_EN_S  11

/* SYSTEM_PCNT_CLK_EN : R/W; bitpos: [10]; default: 0;
 * Set this bit to enable clock of pulse count.
 */

#define SYSTEM_PCNT_CLK_EN    (BIT(10))
#define SYSTEM_PCNT_CLK_EN_M  (SYSTEM_PCNT_CLK_EN_V << SYSTEM_PCNT_CLK_EN_S)
#define SYSTEM_PCNT_CLK_EN_V  0x00000001
#define SYSTEM_PCNT_CLK_EN_S  10

/* SYSTEM_RMT_CLK_EN : R/W; bitpos: [9]; default: 0;
 * Set this bit to enable clock of remote controller.
 */

#define SYSTEM_RMT_CLK_EN    (BIT(9))
#define SYSTEM_RMT_CLK_EN_M  (SYSTEM_RMT_CLK_EN_V << SYSTEM_RMT_CLK_EN_S)
#define SYSTEM_RMT_CLK_EN_V  0x00000001
#define SYSTEM_RMT_CLK_EN_S  9

/* SYSTEM_UHCI0_CLK_EN : R/W; bitpos: [8]; default: 0;
 * Set this bit to enable clock of UHCI0.
 */

#define SYSTEM_UHCI0_CLK_EN    (BIT(8))
#define SYSTEM_UHCI0_CLK_EN_M  (SYSTEM_UHCI0_CLK_EN_V << SYSTEM_UHCI0_CLK_EN_S)
#define SYSTEM_UHCI0_CLK_EN_V  0x00000001
#define SYSTEM_UHCI0_CLK_EN_S  8

/* SYSTEM_I2C_EXT0_CLK_EN : R/W; bitpos: [7]; default: 0;
 * Set this bit to enable clock of I2C EXT0.
 */

#define SYSTEM_I2C_EXT0_CLK_EN    (BIT(7))
#define SYSTEM_I2C_EXT0_CLK_EN_M  (SYSTEM_I2C_EXT0_CLK_EN_V << SYSTEM_I2C_EXT0_CLK_EN_S)
#define SYSTEM_I2C_EXT0_CLK_EN_V  0x00000001
#define SYSTEM_I2C_EXT0_CLK_EN_S  7

/* SYSTEM_SPI2_CLK_EN : R/W; bitpos: [6]; default: 1;
 * Set this bit to enable clock of SPI2.
 */

#define SYSTEM_SPI2_CLK_EN    (BIT(6))
#define SYSTEM_SPI2_CLK_EN_M  (SYSTEM_SPI2_CLK_EN_V << SYSTEM_SPI2_CLK_EN_S)
#define SYSTEM_SPI2_CLK_EN_V  0x00000001
#define SYSTEM_SPI2_CLK_EN_S  6

/* SYSTEM_UART1_CLK_EN : R/W; bitpos: [5]; default: 1;
 * Set this bit to enable clock of UART1.
 */

#define SYSTEM_UART1_CLK_EN    (BIT(5))
#define SYSTEM_UART1_CLK_EN_M  (SYSTEM_UART1_CLK_EN_V << SYSTEM_UART1_CLK_EN_S)
#define SYSTEM_UART1_CLK_EN_V  0x00000001
#define SYSTEM_UART1_CLK_EN_S  5

/* SYSTEM_I2S0_CLK_EN : R/W; bitpos: [4]; default: 0;
 * Set this bit to enable clock of I2S0.
 */

#define SYSTEM_I2S0_CLK_EN    (BIT(4))
#define SYSTEM_I2S0_CLK_EN_M  (SYSTEM_I2S0_CLK_EN_V << SYSTEM_I2S0_CLK_EN_S)
#define SYSTEM_I2S0_CLK_EN_V  0x00000001
#define SYSTEM_I2S0_CLK_EN_S  4

/* SYSTEM_WDG_CLK_EN : R/W; bitpos: [3]; default: 1;
 * Set this bit to enable clock of WDG.
 */

#define SYSTEM_WDG_CLK_EN    (BIT(3))
#define SYSTEM_WDG_CLK_EN_M  (SYSTEM_WDG_CLK_EN_V << SYSTEM_WDG_CLK_EN_S)
#define SYSTEM_WDG_CLK_EN_V  0x00000001
#define SYSTEM_WDG_CLK_EN_S  3

/* SYSTEM_UART_CLK_EN : R/W; bitpos: [2]; default: 1;
 * Set this bit to enable clock of UART0.
 */

#define SYSTEM_UART_CLK_EN    (BIT(2))
#define SYSTEM_UART_CLK_EN_M  (SYSTEM_UART_CLK_EN_V << SYSTEM_UART_CLK_EN_S)
#define SYSTEM_UART_CLK_EN_V  0x00000001
#define SYSTEM_UART_CLK_EN_S  2

/* SYSTEM_SPI01_CLK_EN : R/W; bitpos: [1]; default: 1;
 * Set this bit to enable clock of SPI0 and SPI1.
 */

#define SYSTEM_SPI01_CLK_EN    (BIT(1))
#define SYSTEM_SPI01_CLK_EN_M  (SYSTEM_SPI01_CLK_EN_V << SYSTEM_SPI01_CLK_EN_S)
#define SYSTEM_SPI01_CLK_EN_V  0x00000001
#define SYSTEM_SPI01_CLK_EN_S  1

/* SYSTEM_TIMERS_CLK_EN : R/W; bitpos: [0]; default: 1;
 * Set this bit to enable clock of timers.
 */

#define SYSTEM_TIMERS_CLK_EN    (BIT(0))
#define SYSTEM_TIMERS_CLK_EN_M  (SYSTEM_TIMERS_CLK_EN_V << SYSTEM_TIMERS_CLK_EN_S)
#define SYSTEM_TIMERS_CLK_EN_V  0x00000001
#define SYSTEM_TIMERS_CLK_EN_S  0

/* SYSTEM_PERIP_CLK_EN1_REG register
 * System peripheral clock  (for hardware accelerators) enable register 1
 */

#define SYSTEM_PERIP_CLK_EN1_REG (DR_REG_SYSTEM_BASE + 0x44)

/* SYSTEM_CRYPTO_DMA_CLK_EN : R/W; bitpos: [6]; default: 0;
 * Set this bit to enable clock of cryptography DMA.
 */

#define SYSTEM_CRYPTO_DMA_CLK_EN    (BIT(6))
#define SYSTEM_CRYPTO_DMA_CLK_EN_M  (SYSTEM_CRYPTO_DMA_CLK_EN_V << SYSTEM_CRYPTO_DMA_CLK_EN_S)
#define SYSTEM_CRYPTO_DMA_CLK_EN_V  0x00000001
#define SYSTEM_CRYPTO_DMA_CLK_EN_S  6

/* SYSTEM_CRYPTO_HMAC_CLK_EN : R/W; bitpos: [5]; default: 0;
 * Set this bit to enable clock of cryptography HMAC.
 */

#define SYSTEM_CRYPTO_HMAC_CLK_EN    (BIT(5))
#define SYSTEM_CRYPTO_HMAC_CLK_EN_M  (SYSTEM_CRYPTO_HMAC_CLK_EN_V << SYSTEM_CRYPTO_HMAC_CLK_EN_S)
#define SYSTEM_CRYPTO_HMAC_CLK_EN_V  0x00000001
#define SYSTEM_CRYPTO_HMAC_CLK_EN_S  5

/* SYSTEM_CRYPTO_DS_CLK_EN : R/W; bitpos: [4]; default: 0;
 * Set this bit to enable clock of cryptography Digital Signature.
 */

#define SYSTEM_CRYPTO_DS_CLK_EN    (BIT(4))
#define SYSTEM_CRYPTO_DS_CLK_EN_M  (SYSTEM_CRYPTO_DS_CLK_EN_V << SYSTEM_CRYPTO_DS_CLK_EN_S)
#define SYSTEM_CRYPTO_DS_CLK_EN_V  0x00000001
#define SYSTEM_CRYPTO_DS_CLK_EN_S  4

/* SYSTEM_CRYPTO_RSA_CLK_EN : R/W; bitpos: [3]; default: 0;
 * Set this bit to enable clock of cryptography RSA.
 */

#define SYSTEM_CRYPTO_RSA_CLK_EN    (BIT(3))
#define SYSTEM_CRYPTO_RSA_CLK_EN_M  (SYSTEM_CRYPTO_RSA_CLK_EN_V << SYSTEM_CRYPTO_RSA_CLK_EN_S)
#define SYSTEM_CRYPTO_RSA_CLK_EN_V  0x00000001
#define SYSTEM_CRYPTO_RSA_CLK_EN_S  3

/* SYSTEM_CRYPTO_SHA_CLK_EN : R/W; bitpos: [2]; default: 0;
 * Set this bit to enable clock of cryptography SHA.
 */

#define SYSTEM_CRYPTO_SHA_CLK_EN    (BIT(2))
#define SYSTEM_CRYPTO_SHA_CLK_EN_M  (SYSTEM_CRYPTO_SHA_CLK_EN_V << SYSTEM_CRYPTO_SHA_CLK_EN_S)
#define SYSTEM_CRYPTO_SHA_CLK_EN_V  0x00000001
#define SYSTEM_CRYPTO_SHA_CLK_EN_S  2

/* SYSTEM_CRYPTO_AES_CLK_EN : R/W; bitpos: [1]; default: 0;
 * Set this bit to enable clock of cryptography AES.
 */

#define SYSTEM_CRYPTO_AES_CLK_EN    (BIT(1))
#define SYSTEM_CRYPTO_AES_CLK_EN_M  (SYSTEM_CRYPTO_AES_CLK_EN_V << SYSTEM_CRYPTO_AES_CLK_EN_S)
#define SYSTEM_CRYPTO_AES_CLK_EN_V  0x00000001
#define SYSTEM_CRYPTO_AES_CLK_EN_S  1

/* SYSTEM_PERIP_RST_EN0_REG register
 * System peripheral (hardware accelerators) reset register 0
 */

#define SYSTEM_PERIP_RST_EN0_REG (DR_REG_SYSTEM_BASE + 0x48)

/* SYSTEM_SPI4_RST : R/W; bitpos: [31]; default: 0;
 * Set this bit to reset SPI4.
 */

#define SYSTEM_SPI4_RST    (BIT(31))
#define SYSTEM_SPI4_RST_M  (SYSTEM_SPI4_RST_V << SYSTEM_SPI4_RST_S)
#define SYSTEM_SPI4_RST_V  0x00000001
#define SYSTEM_SPI4_RST_S  31

/* SYSTEM_ADC2_ARB_RST : R/W; bitpos: [30]; default: 0;
 * Set this bit to reset aribiter of ADC2.
 */

#define SYSTEM_ADC2_ARB_RST    (BIT(30))
#define SYSTEM_ADC2_ARB_RST_M  (SYSTEM_ADC2_ARB_RST_V << SYSTEM_ADC2_ARB_RST_S)
#define SYSTEM_ADC2_ARB_RST_V  0x00000001
#define SYSTEM_ADC2_ARB_RST_S  30

/* SYSTEM_SYSTIMER_RST : R/W; bitpos: [29]; default: 0;
 * Set this bit to reset system timer.
 */

#define SYSTEM_SYSTIMER_RST    (BIT(29))
#define SYSTEM_SYSTIMER_RST_M  (SYSTEM_SYSTIMER_RST_V << SYSTEM_SYSTIMER_RST_S)
#define SYSTEM_SYSTIMER_RST_V  0x00000001
#define SYSTEM_SYSTIMER_RST_S  29

/* SYSTEM_APB_SARADC_RST : R/W; bitpos: [28]; default: 0;
 * Set this bit to reset SAR ADC.
 */

#define SYSTEM_APB_SARADC_RST    (BIT(28))
#define SYSTEM_APB_SARADC_RST_M  (SYSTEM_APB_SARADC_RST_V << SYSTEM_APB_SARADC_RST_S)
#define SYSTEM_APB_SARADC_RST_V  0x00000001
#define SYSTEM_APB_SARADC_RST_S  28

/* SYSTEM_SPI3_DMA_RST : R/W; bitpos: [27]; default: 0;
 * Set this bit to reset SPI3 DMA.
 */

#define SYSTEM_SPI3_DMA_RST    (BIT(27))
#define SYSTEM_SPI3_DMA_RST_M  (SYSTEM_SPI3_DMA_RST_V << SYSTEM_SPI3_DMA_RST_S)
#define SYSTEM_SPI3_DMA_RST_V  0x00000001
#define SYSTEM_SPI3_DMA_RST_S  27

/* SYSTEM_PWM3_RST : R/W; bitpos: [26]; default: 0;
 * Set this bit to reset PWM3.
 */

#define SYSTEM_PWM3_RST    (BIT(26))
#define SYSTEM_PWM3_RST_M  (SYSTEM_PWM3_RST_V << SYSTEM_PWM3_RST_S)
#define SYSTEM_PWM3_RST_V  0x00000001
#define SYSTEM_PWM3_RST_S  26

/* SYSTEM_PWM2_RST : R/W; bitpos: [25]; default: 0;
 * Set this bit to reset PWM2.
 */

#define SYSTEM_PWM2_RST    (BIT(25))
#define SYSTEM_PWM2_RST_M  (SYSTEM_PWM2_RST_V << SYSTEM_PWM2_RST_S)
#define SYSTEM_PWM2_RST_V  0x00000001
#define SYSTEM_PWM2_RST_S  25

/* SYSTEM_UART_MEM_RST : R/W; bitpos: [24]; default: 0;
 * Set this bit to reset UART memory.
 */

#define SYSTEM_UART_MEM_RST    (BIT(24))
#define SYSTEM_UART_MEM_RST_M  (SYSTEM_UART_MEM_RST_V << SYSTEM_UART_MEM_RST_S)
#define SYSTEM_UART_MEM_RST_V  0x00000001
#define SYSTEM_UART_MEM_RST_S  24

/* SYSTEM_USB_RST : R/W; bitpos: [23]; default: 0;
 * Set this bit to reset USB.
 */

#define SYSTEM_USB_RST    (BIT(23))
#define SYSTEM_USB_RST_M  (SYSTEM_USB_RST_V << SYSTEM_USB_RST_S)
#define SYSTEM_USB_RST_V  0x00000001
#define SYSTEM_USB_RST_S  23

/* SYSTEM_SPI2_DMA_RST : R/W; bitpos: [22]; default: 0;
 * Set this bit to reset SPI2 DMA.
 */

#define SYSTEM_SPI2_DMA_RST    (BIT(22))
#define SYSTEM_SPI2_DMA_RST_M  (SYSTEM_SPI2_DMA_RST_V << SYSTEM_SPI2_DMA_RST_S)
#define SYSTEM_SPI2_DMA_RST_V  0x00000001
#define SYSTEM_SPI2_DMA_RST_S  22

/* SYSTEM_I2S1_RST : R/W; bitpos: [21]; default: 0;
 * Set this bit to reset I2S1.
 */

#define SYSTEM_I2S1_RST    (BIT(21))
#define SYSTEM_I2S1_RST_M  (SYSTEM_I2S1_RST_V << SYSTEM_I2S1_RST_S)
#define SYSTEM_I2S1_RST_V  0x00000001
#define SYSTEM_I2S1_RST_S  21

/* SYSTEM_PWM1_RST : R/W; bitpos: [20]; default: 0;
 * Set this bit to reset PWM1.
 */

#define SYSTEM_PWM1_RST    (BIT(20))
#define SYSTEM_PWM1_RST_M  (SYSTEM_PWM1_RST_V << SYSTEM_PWM1_RST_S)
#define SYSTEM_PWM1_RST_V  0x00000001
#define SYSTEM_PWM1_RST_S  20

/* SYSTEM_CAN_RST : R/W; bitpos: [19]; default: 0;
 * Set this bit to reset CAN.
 */

#define SYSTEM_CAN_RST    (BIT(19))
#define SYSTEM_CAN_RST_M  (SYSTEM_CAN_RST_V << SYSTEM_CAN_RST_S)
#define SYSTEM_CAN_RST_V  0x00000001
#define SYSTEM_CAN_RST_S  19

/* SYSTEM_I2C_EXT1_RST : R/W; bitpos: [18]; default: 0;
 * Set this bit to reset I2C EXT1.
 */

#define SYSTEM_I2C_EXT1_RST    (BIT(18))
#define SYSTEM_I2C_EXT1_RST_M  (SYSTEM_I2C_EXT1_RST_V << SYSTEM_I2C_EXT1_RST_S)
#define SYSTEM_I2C_EXT1_RST_V  0x00000001
#define SYSTEM_I2C_EXT1_RST_S  18

/* SYSTEM_PWM0_RST : R/W; bitpos: [17]; default: 0;
 * Set this bit to reset PWM0.
 */

#define SYSTEM_PWM0_RST    (BIT(17))
#define SYSTEM_PWM0_RST_M  (SYSTEM_PWM0_RST_V << SYSTEM_PWM0_RST_S)
#define SYSTEM_PWM0_RST_V  0x00000001
#define SYSTEM_PWM0_RST_S  17

/* SYSTEM_SPI3_RST : R/W; bitpos: [16]; default: 0;
 * Set this bit to reset SPI3.
 */

#define SYSTEM_SPI3_RST    (BIT(16))
#define SYSTEM_SPI3_RST_M  (SYSTEM_SPI3_RST_V << SYSTEM_SPI3_RST_S)
#define SYSTEM_SPI3_RST_V  0x00000001
#define SYSTEM_SPI3_RST_S  16

/* SYSTEM_TIMERGROUP1_RST : R/W; bitpos: [15]; default: 0;
 * Set this bit to reset timer group1.
 */

#define SYSTEM_TIMERGROUP1_RST    (BIT(15))
#define SYSTEM_TIMERGROUP1_RST_M  (SYSTEM_TIMERGROUP1_RST_V << SYSTEM_TIMERGROUP1_RST_S)
#define SYSTEM_TIMERGROUP1_RST_V  0x00000001
#define SYSTEM_TIMERGROUP1_RST_S  15

/* SYSTEM_EFUSE_RST : R/W; bitpos: [14]; default: 0;
 * Set this bit to reset eFuse.
 */

#define SYSTEM_EFUSE_RST    (BIT(14))
#define SYSTEM_EFUSE_RST_M  (SYSTEM_EFUSE_RST_V << SYSTEM_EFUSE_RST_S)
#define SYSTEM_EFUSE_RST_V  0x00000001
#define SYSTEM_EFUSE_RST_S  14

/* SYSTEM_TIMERGROUP_RST : R/W; bitpos: [13]; default: 0;
 * Set this bit to reset timer group0.
 */

#define SYSTEM_TIMERGROUP_RST    (BIT(13))
#define SYSTEM_TIMERGROUP_RST_M  (SYSTEM_TIMERGROUP_RST_V << SYSTEM_TIMERGROUP_RST_S)
#define SYSTEM_TIMERGROUP_RST_V  0x00000001
#define SYSTEM_TIMERGROUP_RST_S  13

/* SYSTEM_UHCI1_RST : R/W; bitpos: [12]; default: 0;
 * Set this bit to reset UHCI1.
 */

#define SYSTEM_UHCI1_RST    (BIT(12))
#define SYSTEM_UHCI1_RST_M  (SYSTEM_UHCI1_RST_V << SYSTEM_UHCI1_RST_S)
#define SYSTEM_UHCI1_RST_V  0x00000001
#define SYSTEM_UHCI1_RST_S  12

/* SYSTEM_LEDC_RST : R/W; bitpos: [11]; default: 0;
 * Set this bit to reset LED PWM.
 */

#define SYSTEM_LEDC_RST    (BIT(11))
#define SYSTEM_LEDC_RST_M  (SYSTEM_LEDC_RST_V << SYSTEM_LEDC_RST_S)
#define SYSTEM_LEDC_RST_V  0x00000001
#define SYSTEM_LEDC_RST_S  11

/* SYSTEM_PCNT_RST : R/W; bitpos: [10]; default: 0;
 * Set this bit to reset pulse count.
 */

#define SYSTEM_PCNT_RST    (BIT(10))
#define SYSTEM_PCNT_RST_M  (SYSTEM_PCNT_RST_V << SYSTEM_PCNT_RST_S)
#define SYSTEM_PCNT_RST_V  0x00000001
#define SYSTEM_PCNT_RST_S  10

/* SYSTEM_RMT_RST : R/W; bitpos: [9]; default: 0;
 * Set this bit to reset remote controller.
 */

#define SYSTEM_RMT_RST    (BIT(9))
#define SYSTEM_RMT_RST_M  (SYSTEM_RMT_RST_V << SYSTEM_RMT_RST_S)
#define SYSTEM_RMT_RST_V  0x00000001
#define SYSTEM_RMT_RST_S  9

/* SYSTEM_UHCI0_RST : R/W; bitpos: [8]; default: 0;
 * Set this bit to reset UHCI0.
 */

#define SYSTEM_UHCI0_RST    (BIT(8))
#define SYSTEM_UHCI0_RST_M  (SYSTEM_UHCI0_RST_V << SYSTEM_UHCI0_RST_S)
#define SYSTEM_UHCI0_RST_V  0x00000001
#define SYSTEM_UHCI0_RST_S  8

/* SYSTEM_I2C_EXT0_RST : R/W; bitpos: [7]; default: 0;
 * Set this bit to reset I2C EXT0.
 */

#define SYSTEM_I2C_EXT0_RST    (BIT(7))
#define SYSTEM_I2C_EXT0_RST_M  (SYSTEM_I2C_EXT0_RST_V << SYSTEM_I2C_EXT0_RST_S)
#define SYSTEM_I2C_EXT0_RST_V  0x00000001
#define SYSTEM_I2C_EXT0_RST_S  7

/* SYSTEM_SPI2_RST : R/W; bitpos: [6]; default: 0;
 * Set this bit to reset SPI2.
 */

#define SYSTEM_SPI2_RST    (BIT(6))
#define SYSTEM_SPI2_RST_M  (SYSTEM_SPI2_RST_V << SYSTEM_SPI2_RST_S)
#define SYSTEM_SPI2_RST_V  0x00000001
#define SYSTEM_SPI2_RST_S  6

/* SYSTEM_UART1_RST : R/W; bitpos: [5]; default: 0;
 * Set this bit to reset UART1.
 */

#define SYSTEM_UART1_RST    (BIT(5))
#define SYSTEM_UART1_RST_M  (SYSTEM_UART1_RST_V << SYSTEM_UART1_RST_S)
#define SYSTEM_UART1_RST_V  0x00000001
#define SYSTEM_UART1_RST_S  5

/* SYSTEM_I2S0_RST : R/W; bitpos: [4]; default: 0;
 * Set this bit to reset I2S0.
 */

#define SYSTEM_I2S0_RST    (BIT(4))
#define SYSTEM_I2S0_RST_M  (SYSTEM_I2S0_RST_V << SYSTEM_I2S0_RST_S)
#define SYSTEM_I2S0_RST_V  0x00000001
#define SYSTEM_I2S0_RST_S  4

/* SYSTEM_WDG_RST : R/W; bitpos: [3]; default: 0;
 * Set this bit to reset WDG.
 */

#define SYSTEM_WDG_RST    (BIT(3))
#define SYSTEM_WDG_RST_M  (SYSTEM_WDG_RST_V << SYSTEM_WDG_RST_S)
#define SYSTEM_WDG_RST_V  0x00000001
#define SYSTEM_WDG_RST_S  3

/* SYSTEM_UART_RST : R/W; bitpos: [2]; default: 0;
 * Set this bit to reset UART0.
 */

#define SYSTEM_UART_RST    (BIT(2))
#define SYSTEM_UART_RST_M  (SYSTEM_UART_RST_V << SYSTEM_UART_RST_S)
#define SYSTEM_UART_RST_V  0x00000001
#define SYSTEM_UART_RST_S  2

/* SYSTEM_SPI01_RST : R/W; bitpos: [1]; default: 0;
 * Set this bit to reset SPI0 and SPI1.
 */

#define SYSTEM_SPI01_RST    (BIT(1))
#define SYSTEM_SPI01_RST_M  (SYSTEM_SPI01_RST_V << SYSTEM_SPI01_RST_S)
#define SYSTEM_SPI01_RST_V  0x00000001
#define SYSTEM_SPI01_RST_S  1

/* SYSTEM_TIMERS_RST : R/W; bitpos: [0]; default: 0;
 * Set this bit to reset timers.
 */

#define SYSTEM_TIMERS_RST    (BIT(0))
#define SYSTEM_TIMERS_RST_M  (SYSTEM_TIMERS_RST_V << SYSTEM_TIMERS_RST_S)
#define SYSTEM_TIMERS_RST_V  0x00000001
#define SYSTEM_TIMERS_RST_S  0

/* SYSTEM_PERIP_RST_EN1_REG register
 * System peripheral (hardware accelerators) reset register 1
 */

#define SYSTEM_PERIP_RST_EN1_REG (DR_REG_SYSTEM_BASE + 0x4c)

/* SYSTEM_CRYPTO_DMA_RST : R/W; bitpos: [6]; default: 1;
 * Set this bit to reset cryptography DMA.
 */

#define SYSTEM_CRYPTO_DMA_RST    (BIT(6))
#define SYSTEM_CRYPTO_DMA_RST_M  (SYSTEM_CRYPTO_DMA_RST_V << SYSTEM_CRYPTO_DMA_RST_S)
#define SYSTEM_CRYPTO_DMA_RST_V  0x00000001
#define SYSTEM_CRYPTO_DMA_RST_S  6

/* SYSTEM_CRYPTO_HMAC_RST : R/W; bitpos: [5]; default: 1;
 * Set this bit to reset cryptography HMAC.
 */

#define SYSTEM_CRYPTO_HMAC_RST    (BIT(5))
#define SYSTEM_CRYPTO_HMAC_RST_M  (SYSTEM_CRYPTO_HMAC_RST_V << SYSTEM_CRYPTO_HMAC_RST_S)
#define SYSTEM_CRYPTO_HMAC_RST_V  0x00000001
#define SYSTEM_CRYPTO_HMAC_RST_S  5

/* SYSTEM_CRYPTO_DS_RST : R/W; bitpos: [4]; default: 1;
 * Set this bit to reset cryptography digital signature.
 */

#define SYSTEM_CRYPTO_DS_RST    (BIT(4))
#define SYSTEM_CRYPTO_DS_RST_M  (SYSTEM_CRYPTO_DS_RST_V << SYSTEM_CRYPTO_DS_RST_S)
#define SYSTEM_CRYPTO_DS_RST_V  0x00000001
#define SYSTEM_CRYPTO_DS_RST_S  4

/* SYSTEM_CRYPTO_RSA_RST : R/W; bitpos: [3]; default: 1;
 * Set this bit to reset cryptography RSA.
 */

#define SYSTEM_CRYPTO_RSA_RST    (BIT(3))
#define SYSTEM_CRYPTO_RSA_RST_M  (SYSTEM_CRYPTO_RSA_RST_V << SYSTEM_CRYPTO_RSA_RST_S)
#define SYSTEM_CRYPTO_RSA_RST_V  0x00000001
#define SYSTEM_CRYPTO_RSA_RST_S  3

/* SYSTEM_CRYPTO_SHA_RST : R/W; bitpos: [2]; default: 1;
 * Set this bit to reset cryptography SHA.
 */

#define SYSTEM_CRYPTO_SHA_RST    (BIT(2))
#define SYSTEM_CRYPTO_SHA_RST_M  (SYSTEM_CRYPTO_SHA_RST_V << SYSTEM_CRYPTO_SHA_RST_S)
#define SYSTEM_CRYPTO_SHA_RST_V  0x00000001
#define SYSTEM_CRYPTO_SHA_RST_S  2

/* SYSTEM_CRYPTO_AES_RST : R/W; bitpos: [1]; default: 1;
 * Set this bit to reset cryptography AES.
 */

#define SYSTEM_CRYPTO_AES_RST    (BIT(1))
#define SYSTEM_CRYPTO_AES_RST_M  (SYSTEM_CRYPTO_AES_RST_V << SYSTEM_CRYPTO_AES_RST_S)
#define SYSTEM_CRYPTO_AES_RST_V  0x00000001
#define SYSTEM_CRYPTO_AES_RST_S  1

/* SYSTEM_LPCK_DIV_INT_REG register
 * Low power clock divider integer register
 */

#define SYSTEM_LPCK_DIV_INT_REG (DR_REG_SYSTEM_BASE + 0x50)

/* SYSTEM_LPCK_DIV_NUM : R/W; bitpos: [11:0]; default: 255;
 * This field is used to set the integer number of the divider value.
 */

#define SYSTEM_LPCK_DIV_NUM    0x00000fff
#define SYSTEM_LPCK_DIV_NUM_M  (SYSTEM_LPCK_DIV_NUM_V << SYSTEM_LPCK_DIV_NUM_S)
#define SYSTEM_LPCK_DIV_NUM_V  0x00000fff
#define SYSTEM_LPCK_DIV_NUM_S  0

/* SYSTEM_BT_LPCK_DIV_FRAC_REG register
 * Divider fraction configuration register for low-power clock
 */

#define SYSTEM_BT_LPCK_DIV_FRAC_REG (DR_REG_SYSTEM_BASE + 0x54)

/* SYSTEM_LPCLK_RTC_EN : R/W; bitpos: [28]; default: 0;
 * Set this bit to enable the RTC low power clock.
 */

#define SYSTEM_LPCLK_RTC_EN    (BIT(28))
#define SYSTEM_LPCLK_RTC_EN_M  (SYSTEM_LPCLK_RTC_EN_V << SYSTEM_LPCLK_RTC_EN_S)
#define SYSTEM_LPCLK_RTC_EN_V  0x00000001
#define SYSTEM_LPCLK_RTC_EN_S  28

/* SYSTEM_LPCLK_SEL_XTAL32K : R/W; bitpos: [27]; default: 0;
 * Set this bit to select xtal32k clock as the low power clock.
 */

#define SYSTEM_LPCLK_SEL_XTAL32K    (BIT(27))
#define SYSTEM_LPCLK_SEL_XTAL32K_M  (SYSTEM_LPCLK_SEL_XTAL32K_V << SYSTEM_LPCLK_SEL_XTAL32K_S)
#define SYSTEM_LPCLK_SEL_XTAL32K_V  0x00000001
#define SYSTEM_LPCLK_SEL_XTAL32K_S  27

/* SYSTEM_LPCLK_SEL_XTAL : R/W; bitpos: [26]; default: 0;
 * Set this bit to select xtal clock as the low power clock.
 */

#define SYSTEM_LPCLK_SEL_XTAL    (BIT(26))
#define SYSTEM_LPCLK_SEL_XTAL_M  (SYSTEM_LPCLK_SEL_XTAL_V << SYSTEM_LPCLK_SEL_XTAL_S)
#define SYSTEM_LPCLK_SEL_XTAL_V  0x00000001
#define SYSTEM_LPCLK_SEL_XTAL_S  26

/* SYSTEM_LPCLK_SEL_8M : R/W; bitpos: [25]; default: 1;
 * Set this bit to select 8m clock as the low power clock.
 */

#define SYSTEM_LPCLK_SEL_8M    (BIT(25))
#define SYSTEM_LPCLK_SEL_8M_M  (SYSTEM_LPCLK_SEL_8M_V << SYSTEM_LPCLK_SEL_8M_S)
#define SYSTEM_LPCLK_SEL_8M_V  0x00000001
#define SYSTEM_LPCLK_SEL_8M_S  25

/* SYSTEM_LPCLK_SEL_RTC_SLOW : R/W; bitpos: [24]; default: 0;
 * Set this bit to select RTC slow clock as the low power clock.
 */

#define SYSTEM_LPCLK_SEL_RTC_SLOW    (BIT(24))
#define SYSTEM_LPCLK_SEL_RTC_SLOW_M  (SYSTEM_LPCLK_SEL_RTC_SLOW_V << SYSTEM_LPCLK_SEL_RTC_SLOW_S)
#define SYSTEM_LPCLK_SEL_RTC_SLOW_V  0x00000001
#define SYSTEM_LPCLK_SEL_RTC_SLOW_S  24

/* SYSTEM_CPU_INTR_FROM_CPU_0_REG register
 * CPU interrupt controlling register 0
 */

#define SYSTEM_CPU_INTR_FROM_CPU_0_REG (DR_REG_SYSTEM_BASE + 0x58)

/* SYSTEM_CPU_INTR_FROM_CPU_0 : R/W; bitpos: [0]; default: 0;
 * Set this bit to generate CPU interrupt 0. This bit needs to be reset by
 * software in the ISR process.
 */

#define SYSTEM_CPU_INTR_FROM_CPU_0    (BIT(0))
#define SYSTEM_CPU_INTR_FROM_CPU_0_M  (SYSTEM_CPU_INTR_FROM_CPU_0_V << SYSTEM_CPU_INTR_FROM_CPU_0_S)
#define SYSTEM_CPU_INTR_FROM_CPU_0_V  0x00000001
#define SYSTEM_CPU_INTR_FROM_CPU_0_S  0

/* SYSTEM_CPU_INTR_FROM_CPU_1_REG register
 * CPU interrupt controlling register 1
 */

#define SYSTEM_CPU_INTR_FROM_CPU_1_REG (DR_REG_SYSTEM_BASE + 0x5c)

/* SYSTEM_CPU_INTR_FROM_CPU_1 : R/W; bitpos: [0]; default: 0;
 * Set this bit to generate CPU interrupt 1. This bit needs to be reset by
 * software in the ISR process.
 */

#define SYSTEM_CPU_INTR_FROM_CPU_1    (BIT(0))
#define SYSTEM_CPU_INTR_FROM_CPU_1_M  (SYSTEM_CPU_INTR_FROM_CPU_1_V << SYSTEM_CPU_INTR_FROM_CPU_1_S)
#define SYSTEM_CPU_INTR_FROM_CPU_1_V  0x00000001
#define SYSTEM_CPU_INTR_FROM_CPU_1_S  0

/* SYSTEM_CPU_INTR_FROM_CPU_2_REG register
 * CPU interrupt controlling register 2
 */

#define SYSTEM_CPU_INTR_FROM_CPU_2_REG (DR_REG_SYSTEM_BASE + 0x60)

/* SYSTEM_CPU_INTR_FROM_CPU_2 : R/W; bitpos: [0]; default: 0;
 * Set this bit to generate CPU interrupt 2. This bit needs to be reset by
 * software in the ISR process.
 */

#define SYSTEM_CPU_INTR_FROM_CPU_2    (BIT(0))
#define SYSTEM_CPU_INTR_FROM_CPU_2_M  (SYSTEM_CPU_INTR_FROM_CPU_2_V << SYSTEM_CPU_INTR_FROM_CPU_2_S)
#define SYSTEM_CPU_INTR_FROM_CPU_2_V  0x00000001
#define SYSTEM_CPU_INTR_FROM_CPU_2_S  0

/* SYSTEM_CPU_INTR_FROM_CPU_3_REG register
 * CPU interrupt controlling register 3
 */

#define SYSTEM_CPU_INTR_FROM_CPU_3_REG (DR_REG_SYSTEM_BASE + 0x64)

/* SYSTEM_CPU_INTR_FROM_CPU_3 : R/W; bitpos: [0]; default: 0;
 * Set this bit to generate CPU interrupt 3. This bit needs to be reset by
 * software in the ISR process.
 */

#define SYSTEM_CPU_INTR_FROM_CPU_3    (BIT(0))
#define SYSTEM_CPU_INTR_FROM_CPU_3_M  (SYSTEM_CPU_INTR_FROM_CPU_3_V << SYSTEM_CPU_INTR_FROM_CPU_3_S)
#define SYSTEM_CPU_INTR_FROM_CPU_3_V  0x00000001
#define SYSTEM_CPU_INTR_FROM_CPU_3_S  0

/* SYSTEM_RSA_PD_CTRL_REG register
 * RSA memory remapping register
 */

#define SYSTEM_RSA_PD_CTRL_REG (DR_REG_SYSTEM_BASE + 0x68)

/* SYSTEM_RSA_MEM_FORCE_PD : R/W; bitpos: [2]; default: 0;
 * Set this bit to force power down RSA memory. This bit has the highest
 * priority.
 */

#define SYSTEM_RSA_MEM_FORCE_PD    (BIT(2))
#define SYSTEM_RSA_MEM_FORCE_PD_M  (SYSTEM_RSA_MEM_FORCE_PD_V << SYSTEM_RSA_MEM_FORCE_PD_S)
#define SYSTEM_RSA_MEM_FORCE_PD_V  0x00000001
#define SYSTEM_RSA_MEM_FORCE_PD_S  2

/* SYSTEM_RSA_MEM_FORCE_PU : R/W; bitpos: [1]; default: 0;
 * Set this bit to force power up RSA memory. This bit has the second
 * highest priority.
 */

#define SYSTEM_RSA_MEM_FORCE_PU    (BIT(1))
#define SYSTEM_RSA_MEM_FORCE_PU_M  (SYSTEM_RSA_MEM_FORCE_PU_V << SYSTEM_RSA_MEM_FORCE_PU_S)
#define SYSTEM_RSA_MEM_FORCE_PU_V  0x00000001
#define SYSTEM_RSA_MEM_FORCE_PU_S  1

/* SYSTEM_RSA_MEM_PD : R/W; bitpos: [0]; default: 1;
 * Set this bit to power down RSA memory. This bit has the lowest priority.
 * When Digital Signature occupies the RSA, this bit is invalid.
 */

#define SYSTEM_RSA_MEM_PD    (BIT(0))
#define SYSTEM_RSA_MEM_PD_M  (SYSTEM_RSA_MEM_PD_V << SYSTEM_RSA_MEM_PD_S)
#define SYSTEM_RSA_MEM_PD_V  0x00000001
#define SYSTEM_RSA_MEM_PD_S  0

/* SYSTEM_BUSTOEXTMEM_ENA_REG register
 * EDMA enable register
 */

#define SYSTEM_BUSTOEXTMEM_ENA_REG (DR_REG_SYSTEM_BASE + 0x6c)

/* SYSTEM_BUSTOEXTMEM_ENA : R/W; bitpos: [0]; default: 1;
 * Set this bit to enable bus to EDMA.
 */

#define SYSTEM_BUSTOEXTMEM_ENA    (BIT(0))
#define SYSTEM_BUSTOEXTMEM_ENA_M  (SYSTEM_BUSTOEXTMEM_ENA_V << SYSTEM_BUSTOEXTMEM_ENA_S)
#define SYSTEM_BUSTOEXTMEM_ENA_V  0x00000001
#define SYSTEM_BUSTOEXTMEM_ENA_S  0

/* SYSTEM_CACHE_CONTROL_REG register
 * Cache control register
 */

#define SYSTEM_CACHE_CONTROL_REG (DR_REG_SYSTEM_BASE + 0x70)

/* SYSTEM_PRO_CACHE_RESET : R/W; bitpos: [2]; default: 0;
 * Set this bit to reset cache.
 */

#define SYSTEM_PRO_CACHE_RESET    (BIT(2))
#define SYSTEM_PRO_CACHE_RESET_M  (SYSTEM_PRO_CACHE_RESET_V << SYSTEM_PRO_CACHE_RESET_S)
#define SYSTEM_PRO_CACHE_RESET_V  0x00000001
#define SYSTEM_PRO_CACHE_RESET_S  2

/* SYSTEM_PRO_DCACHE_CLK_ON : R/W; bitpos: [1]; default: 1;
 * Set this bit to enable clock of d-cache.
 */

#define SYSTEM_PRO_DCACHE_CLK_ON    (BIT(1))
#define SYSTEM_PRO_DCACHE_CLK_ON_M  (SYSTEM_PRO_DCACHE_CLK_ON_V << SYSTEM_PRO_DCACHE_CLK_ON_S)
#define SYSTEM_PRO_DCACHE_CLK_ON_V  0x00000001
#define SYSTEM_PRO_DCACHE_CLK_ON_S  1

/* SYSTEM_PRO_ICACHE_CLK_ON : R/W; bitpos: [0]; default: 1;
 * Set this bit to enable clock of i-cache.
 */

#define SYSTEM_PRO_ICACHE_CLK_ON    (BIT(0))
#define SYSTEM_PRO_ICACHE_CLK_ON_M  (SYSTEM_PRO_ICACHE_CLK_ON_V << SYSTEM_PRO_ICACHE_CLK_ON_S)
#define SYSTEM_PRO_ICACHE_CLK_ON_V  0x00000001
#define SYSTEM_PRO_ICACHE_CLK_ON_S  0

/* SYSTEM_EXTERNAL_DEVICE_ENCRYPT_DECRYPT_CONTROL_REG register
 * External memory encrypt and decrypt controlling register
 */

#define SYSTEM_EXTERNAL_DEVICE_ENCRYPT_DECRYPT_CONTROL_REG (DR_REG_SYSTEM_BASE + 0x74)

/* SYSTEM_ENABLE_DOWNLOAD_MANUAL_ENCRYPT : R/W; bitpos: [3]; default: 0;
 * Set this bit to enable Manual Encryption under Download Boot mode.
 */

#define SYSTEM_ENABLE_DOWNLOAD_MANUAL_ENCRYPT    (BIT(3))
#define SYSTEM_ENABLE_DOWNLOAD_MANUAL_ENCRYPT_M  (SYSTEM_ENABLE_DOWNLOAD_MANUAL_ENCRYPT_V << SYSTEM_ENABLE_DOWNLOAD_MANUAL_ENCRYPT_S)
#define SYSTEM_ENABLE_DOWNLOAD_MANUAL_ENCRYPT_V  0x00000001
#define SYSTEM_ENABLE_DOWNLOAD_MANUAL_ENCRYPT_S  3

/* SYSTEM_ENABLE_DOWNLOAD_G0CB_DECRYPT : R/W; bitpos: [2]; default: 0;
 * Set this bit to enable Auto Decryption under Download Boot mode.
 */

#define SYSTEM_ENABLE_DOWNLOAD_G0CB_DECRYPT    (BIT(2))
#define SYSTEM_ENABLE_DOWNLOAD_G0CB_DECRYPT_M  (SYSTEM_ENABLE_DOWNLOAD_G0CB_DECRYPT_V << SYSTEM_ENABLE_DOWNLOAD_G0CB_DECRYPT_S)
#define SYSTEM_ENABLE_DOWNLOAD_G0CB_DECRYPT_V  0x00000001
#define SYSTEM_ENABLE_DOWNLOAD_G0CB_DECRYPT_S  2

/* SYSTEM_ENABLE_DOWNLOAD_DB_ENCRYPT : R/W; bitpos: [1]; default: 0;
 * Set this bit to enable Auto Encryption under Download Boot mode.
 */

#define SYSTEM_ENABLE_DOWNLOAD_DB_ENCRYPT    (BIT(1))
#define SYSTEM_ENABLE_DOWNLOAD_DB_ENCRYPT_M  (SYSTEM_ENABLE_DOWNLOAD_DB_ENCRYPT_V << SYSTEM_ENABLE_DOWNLOAD_DB_ENCRYPT_S)
#define SYSTEM_ENABLE_DOWNLOAD_DB_ENCRYPT_V  0x00000001
#define SYSTEM_ENABLE_DOWNLOAD_DB_ENCRYPT_S  1

/* SYSTEM_ENABLE_SPI_MANUAL_ENCRYPT : R/W; bitpos: [0]; default: 0;
 * Set this bit to enable Manual Encryption under SPI Boot mode.
 */

#define SYSTEM_ENABLE_SPI_MANUAL_ENCRYPT    (BIT(0))
#define SYSTEM_ENABLE_SPI_MANUAL_ENCRYPT_M  (SYSTEM_ENABLE_SPI_MANUAL_ENCRYPT_V << SYSTEM_ENABLE_SPI_MANUAL_ENCRYPT_S)
#define SYSTEM_ENABLE_SPI_MANUAL_ENCRYPT_V  0x00000001
#define SYSTEM_ENABLE_SPI_MANUAL_ENCRYPT_S  0

/* SYSTEM_RTC_FASTMEM_CONFIG_REG register
 * RTC fast memory configuration register
 */

#define SYSTEM_RTC_FASTMEM_CONFIG_REG (DR_REG_SYSTEM_BASE + 0x78)

/* SYSTEM_RTC_MEM_CRC_FINISH : RO; bitpos: [31]; default: 0;
 * This bit stores the status of RTC memory CRC. High level means finished
 * while low level means not finished.
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
 * Set this bit to start the CRC of RTC memory.
 */

#define SYSTEM_RTC_MEM_CRC_START    (BIT(8))
#define SYSTEM_RTC_MEM_CRC_START_M  (SYSTEM_RTC_MEM_CRC_START_V << SYSTEM_RTC_MEM_CRC_START_S)
#define SYSTEM_RTC_MEM_CRC_START_V  0x00000001
#define SYSTEM_RTC_MEM_CRC_START_S  8

/* SYSTEM_RTC_FASTMEM_CRC_REG register
 * RTC fast memory CRC controlling register
 */

#define SYSTEM_RTC_FASTMEM_CRC_REG (DR_REG_SYSTEM_BASE + 0x7c)

/* SYSTEM_RTC_MEM_CRC_RES : RO; bitpos: [31:0]; default: 0;
 * This field stores the CRC result of RTC memory.
 */

#define SYSTEM_RTC_MEM_CRC_RES    0xffffffff
#define SYSTEM_RTC_MEM_CRC_RES_M  (SYSTEM_RTC_MEM_CRC_RES_V << SYSTEM_RTC_MEM_CRC_RES_S)
#define SYSTEM_RTC_MEM_CRC_RES_V  0xffffffff
#define SYSTEM_RTC_MEM_CRC_RES_S  0

/* SYSTEM_Redundant_ECO_Ctrl_REG register
 * Redundant ECO control register
 */

#define SYSTEM_REDUNDANT_ECO_CTRL_REG (DR_REG_SYSTEM_BASE + 0x80)

/* SYSTEM_REDUNDANT_ECO_RESULT : RO; bitpos: [1]; default: 0;
 * The redundant ECO result bit to avoid optimization in circuits.
 */

#define SYSTEM_REDUNDANT_ECO_RESULT    (BIT(1))
#define SYSTEM_REDUNDANT_ECO_RESULT_M  (SYSTEM_REDUNDANT_ECO_RESULT_V << SYSTEM_REDUNDANT_ECO_RESULT_S)
#define SYSTEM_REDUNDANT_ECO_RESULT_V  0x00000001
#define SYSTEM_REDUNDANT_ECO_RESULT_S  1

/* SYSTEM_REDUNDANT_ECO_DRIVE : R/W; bitpos: [0]; default: 0;
 * The redundant ECO drive bit to avoid optimization in circuits.
 */

#define SYSTEM_REDUNDANT_ECO_DRIVE    (BIT(0))
#define SYSTEM_REDUNDANT_ECO_DRIVE_M  (SYSTEM_REDUNDANT_ECO_DRIVE_V << SYSTEM_REDUNDANT_ECO_DRIVE_S)
#define SYSTEM_REDUNDANT_ECO_DRIVE_V  0x00000001
#define SYSTEM_REDUNDANT_ECO_DRIVE_S  0

/* SYSTEM_CLOCK_GATE_REG register
 * Clock gate control register
 */

#define SYSTEM_CLOCK_GATE_REG (DR_REG_SYSTEM_BASE + 0x84)

/* SYSTEM_CLK_EN : R/W; bitpos: [0]; default: 1;
 * Set this bit to enable clock of this module.
 */

#define SYSTEM_CLK_EN    (BIT(0))
#define SYSTEM_CLK_EN_M  (SYSTEM_CLK_EN_V << SYSTEM_CLK_EN_S)
#define SYSTEM_CLK_EN_V  0x00000001
#define SYSTEM_CLK_EN_S  0

/* SYSTEM_SRAM_CTRL_2_REG register
 * System SRAM configuration register 2
 */

#define SYSTEM_SRAM_CTRL_2_REG (DR_REG_SYSTEM_BASE + 0x88)

/* SYSTEM_SRAM_FORCE_PU : R/W; bitpos: [21:0]; default: 4194303;
 * This field is used to power up internal SRAM.
 */

#define SYSTEM_SRAM_FORCE_PU    0x003fffff
#define SYSTEM_SRAM_FORCE_PU_M  (SYSTEM_SRAM_FORCE_PU_V << SYSTEM_SRAM_FORCE_PU_S)
#define SYSTEM_SRAM_FORCE_PU_V  0x003fffff
#define SYSTEM_SRAM_FORCE_PU_S  0

/* SYSTEM_SYSCLK_CONF_REG register
 * SoC clock configuration register
 */

#define SYSTEM_SYSCLK_CONF_REG (DR_REG_SYSTEM_BASE + 0x8c)

/* SYSTEM_CLK_DIV_EN : RO; bitpos: [19]; default: 0;
 * Not used, extends from ESP32.
 */

#define SYSTEM_CLK_DIV_EN    (BIT(19))
#define SYSTEM_CLK_DIV_EN_M  (SYSTEM_CLK_DIV_EN_V << SYSTEM_CLK_DIV_EN_S)
#define SYSTEM_CLK_DIV_EN_V  0x00000001
#define SYSTEM_CLK_DIV_EN_S  19

/* SYSTEM_CLK_XTAL_FREQ : RO; bitpos: [18:12]; default: 0;
 * This field is used to read XTAL frequency in MHz.
 */

#define SYSTEM_CLK_XTAL_FREQ    0x0000007f
#define SYSTEM_CLK_XTAL_FREQ_M  (SYSTEM_CLK_XTAL_FREQ_V << SYSTEM_CLK_XTAL_FREQ_S)
#define SYSTEM_CLK_XTAL_FREQ_V  0x0000007f
#define SYSTEM_CLK_XTAL_FREQ_S  12

/* SYSTEM_SOC_CLK_SEL : R/W; bitpos: [11:10]; default: 0;
 * This field is used to select SOC clock.
 */

#define SYSTEM_SOC_CLK_SEL    0x00000003
#define SYSTEM_SOC_CLK_SEL_M  (SYSTEM_SOC_CLK_SEL_V << SYSTEM_SOC_CLK_SEL_S)
#define SYSTEM_SOC_CLK_SEL_V  0x00000003
#define SYSTEM_SOC_CLK_SEL_S  10

/* SYSTEM_PRE_DIV_CNT : R/W; bitpos: [9:0]; default: 1;
 * This field is used to set the count of prescaler of XTAL\_CLK.
 */

#define SYSTEM_PRE_DIV_CNT    0x000003ff
#define SYSTEM_PRE_DIV_CNT_M  (SYSTEM_PRE_DIV_CNT_V << SYSTEM_PRE_DIV_CNT_S)
#define SYSTEM_PRE_DIV_CNT_V  0x000003ff
#define SYSTEM_PRE_DIV_CNT_S  0

/* SYSTEM_DATE_REG register
 * Version control register
 */

#define SYSTEM_DATE_REG (DR_REG_SYSTEM_BASE + 0xffc)

/* SYSTEM_DATE : R/W; bitpos: [27:0]; default: 26247200;
 * Version control register.
 */

#define SYSTEM_DATE    0x0fffffff
#define SYSTEM_DATE_M  (SYSTEM_DATE_V << SYSTEM_DATE_S)
#define SYSTEM_DATE_V  0x0fffffff
#define SYSTEM_DATE_S  0

#endif /* __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_SYSTEM_H */
