/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_gpio.h
 *
 * Copyright (C) 2012, 2015 Gregory Nutt. All rights reserved.
 * Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_GPIO_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/bl602_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Input Floating Mode */

#define GPIO_MODE_INPUT (0x00000000)

/* Output Push Pull Mode */

#define GPIO_MODE_OUTPUT (0x00000001)

/* Alternate function */

#define GPIO_MODE_AF (0x00000002)

/* GPIO pull up */

#define GPIO_PULL_UP (0x00000000)

/* GPIO pull down */

#define GPIO_PULL_DOWN (0x00000001)

/* GPIO no pull up or down */

#define GPIO_PULL_NONE (0x00000002)

/* GPIO0 function definition */

#define GPIO0_FUN_SDIO_CLK          1
#define GPIO0_FUN_SF_D1             2
#define GPIO0_FUN_UNUSED3           3
#define GPIO0_FUN_SPI_MISO_SPI_MOSI 4
#define GPIO0_FUN_UNUSED5           5
#define GPIO0_FUN_I2C_SCL           6
#define GPIO0_FUN_UART_SIG0         7
#define GPIO0_FUN_PWM_CH0           8
#define GPIO0_FUN_FEM_GPIO_0        9
#define GPIO0_FUN_ATEST_IN          10
#define GPIO0_FUN_SWGPIO_0          11
#define GPIO0_FUN_E21_TMS           14

/* GPIO1 function definition */

#define GPIO1_FUN_SDIO_CMD          1
#define GPIO1_FUN_SF_D2             2
#define GPIO1_FUN_UNUSED3           3
#define GPIO1_FUN_SPI_MOSI_SPI_MISO 4
#define GPIO1_FUN_UNUSED5           5
#define GPIO1_FUN_I2C_SDA           6
#define GPIO1_FUN_UART_SIG1         7
#define GPIO1_FUN_PWM_CH1           8
#define GPIO1_FUN_FEM_GPIO_1        9
#define GPIO1_FUN_ATEST_IP          10
#define GPIO1_FUN_SWGPIO_1          11
#define GPIO1_FUN_E21_TDI           14

/* GPIO2 function definition */

#define GPIO2_FUN_SDIO_DAT0  1
#define GPIO2_FUN_SF_D3      2
#define GPIO2_FUN_UNUSED3    3
#define GPIO2_FUN_SPI_SS     4
#define GPIO2_FUN_UNUSED5    5
#define GPIO2_FUN_I2C_SCL    6
#define GPIO2_FUN_UART_SIG2  7
#define GPIO2_FUN_PWM_CH2    8
#define GPIO2_FUN_FEM_GPIO_2 9
#define GPIO2_FUN_ATEST_QN   10
#define GPIO2_FUN_SWGPIO_2   11
#define GPIO2_FUN_E21_TCK    14

/* GPIO3 function definition */

#define GPIO3_FUN_SDIO_DAT1  1
#define GPIO3_FUN_UNUSED2    2
#define GPIO3_FUN_UNUSED3    3
#define GPIO3_FUN_SPI_SCLK   4
#define GPIO3_FUN_UNUSED5    5
#define GPIO3_FUN_I2C_SDA    6
#define GPIO3_FUN_UART_SIG3  7
#define GPIO3_FUN_PWM_CH3    8
#define GPIO3_FUN_FEM_GPIO_3 9
#define GPIO3_FUN_ATEST_QP   10
#define GPIO3_FUN_SWGPIO_3   11
#define GPIO3_FUN_E21_TDO    14

/* GPIO4 function definition */

#define GPIO4_FUN_SDIO_DAT2         1
#define GPIO4_FUN_UNUSED2           2
#define GPIO4_FUN_UNUSED3           3
#define GPIO4_FUN_SPI_MISO_SPI_MOSI 4
#define GPIO4_FUN_UNUSED5           5
#define GPIO4_FUN_I2C_SCL           6
#define GPIO4_FUN_UART_SIG4         7
#define GPIO4_FUN_PWM_CH4           8
#define GPIO4_FUN_FEM_GPIO_0        9
#define GPIO4_FUN_GPIP_CH1          10
#define GPIO4_FUN_SWGPIO_4          11
#define GPIO4_FUN_E21_TMS           14

/* GPIO5 function definition */

#define GPIO5_FUN_SDIO_DAT3         1
#define GPIO5_FUN_UNUSED2           2
#define GPIO5_FUN_UNUSED3           3
#define GPIO5_FUN_SPI_MOSI_SPI_MISO 4
#define GPIO5_FUN_UNUSED5           5
#define GPIO5_FUN_I2C_SDA           6
#define GPIO5_FUN_UART_SIG5         7
#define GPIO5_FUN_PWM_CH0           8
#define GPIO5_FUN_FEM_GPIO_1        9
#define GPIO5_FUN_GPIP_CH4          10
#define GPIO5_FUN_SWGPIO_5          11
#define GPIO5_FUN_E21_TDI           14

/* GPIO6 function definition */

#define GPIO6_FUN_UNUSED1    1
#define GPIO6_FUN_UNUSED2    2
#define GPIO6_FUN_UNUSED3    3
#define GPIO6_FUN_SPI_SS     4
#define GPIO6_FUN_UNUSED5    5
#define GPIO6_FUN_I2C_SCL    6
#define GPIO6_FUN_UART_SIG6  7
#define GPIO6_FUN_PWM_CH1    8
#define GPIO6_FUN_FEM_GPIO_2 9
#define GPIO6_FUN_GPIP_CH5   10
#define GPIO6_FUN_SWGPIO_6   11
#define GPIO6_FUN_E21_TCK    14

/* GPIO7 function definition */

#define GPIO7_FUN_UNUSED1    1
#define GPIO7_FUN_UNUSED2    2
#define GPIO7_FUN_UNUSED3    3
#define GPIO7_FUN_SPI_SCLK   4
#define GPIO7_FUN_UNUSED5    5
#define GPIO7_FUN_I2C_SDA    6
#define GPIO7_FUN_UART_SIG7  7
#define GPIO7_FUN_PWM_CH2    8
#define GPIO7_FUN_FEM_GPIO_3 9
#define GPIO7_FUN_UNUSED10   10
#define GPIO7_FUN_SWGPIO_7   11
#define GPIO7_FUN_E21_TDO    14

/* GPIO8 function definition */

#define GPIO8_FUN_UNUSED1           1
#define GPIO8_FUN_UNUSED2           2
#define GPIO8_FUN_UNUSED3           3
#define GPIO8_FUN_SPI_MISO_SPI_MOSI 4
#define GPIO8_FUN_UNUSED5           5
#define GPIO8_FUN_I2C_SCL           6
#define GPIO8_FUN_UART_SIG0         7
#define GPIO8_FUN_PWM_CH3           8
#define GPIO8_FUN_FEM_GPIO_0        9
#define GPIO8_FUN_UNUSED10          10
#define GPIO8_FUN_SWGPIO_8          11
#define GPIO8_FUN_E21_TMS           14

/* GPIO9 function definition */

#define GPIO9_FUN_UNUSED1           1
#define GPIO9_FUN_UNUSED2           2
#define GPIO9_FUN_UNUSED3           3
#define GPIO9_FUN_SPI_MOSI_SPI_MISO 4
#define GPIO9_FUN_UNUSED5           5
#define GPIO9_FUN_I2C_SDA           6
#define GPIO9_FUN_UART_SIG1         7
#define GPIO9_FUN_PWM_CH4           8
#define GPIO9_FUN_FEM_GPIO_1        9
#define GPIO9_FUN_GPIP_CH6_GPIP_CH7 10
#define GPIO9_FUN_SWGPIO_9          11
#define GPIO9_FUN_E21_TDI           14

/* GPIO10 function definition */

#define GPIO10_FUN_UNUSED1                   1
#define GPIO10_FUN_UNUSED2                   2
#define GPIO10_FUN_UNUSED3                   3
#define GPIO10_FUN_SPI_SS                    4
#define GPIO10_FUN_UNUSED5                   5
#define GPIO10_FUN_I2C_SCL                   6
#define GPIO10_FUN_UART_SIG2                 7
#define GPIO10_FUN_PWM_CH0                   8
#define GPIO10_FUN_FEM_GPIO_2                9
#define GPIO10_FUN_MICBIAS_GPIP_CH8_GPIP_CH9 10
#define GPIO10_FUN_SWGPIO_10                 11
#define GPIO10_FUN_E21_TCK                   14

/* GPIO11 function definition */

#define GPIO11_FUN_UNUSED1             1
#define GPIO11_FUN_UNUSED2             2
#define GPIO11_FUN_UNUSED3             3
#define GPIO11_FUN_SPI_SCLK            4
#define GPIO11_FUN_UNUSED5             5
#define GPIO11_FUN_I2C_SDA             6
#define GPIO11_FUN_UART_SIG3           7
#define GPIO11_FUN_PWM_CH1             8
#define GPIO11_FUN_FEM_GPIO_3          9
#define GPIO11_FUN_IRLED_OUT_GPIP_CH10 10
#define GPIO11_FUN_SWGPIO_11           11
#define GPIO11_FUN_E21_TDO             14

/* GPIO12 function definition */

#define GPIO12_FUN_UNUSED1                 1
#define GPIO12_FUN_UNUSED2                 2
#define GPIO12_FUN_UNUSED3                 3
#define GPIO12_FUN_SPI_MISO_SPI_MOSI       4
#define GPIO12_FUN_UNUSED5                 5
#define GPIO12_FUN_I2C_SCL                 6
#define GPIO12_FUN_UART_SIG4               7
#define GPIO12_FUN_PWM_CH2                 8
#define GPIO12_FUN_FEM_GPIO_0              9
#define GPIO12_FUN_GPIP_CH0_GPADC_VREF_EXT 10
#define GPIO12_FUN_SWGPIO_12               11
#define GPIO12_FUN_E21_TMS                 14

/* GPIO13 function definition */

#define GPIO13_FUN_UNUSED1           1
#define GPIO13_FUN_UNUSED2           2
#define GPIO13_FUN_UNUSED3           3
#define GPIO13_FUN_SPI_MOSI_SPI_MISO 4
#define GPIO13_FUN_UNUSED5           5
#define GPIO13_FUN_I2C_SDA           6
#define GPIO13_FUN_UART_SIG5         7
#define GPIO13_FUN_PWM_CH3           8
#define GPIO13_FUN_FEM_GPIO_1        9
#define GPIO13_FUN_GPIP_CH3          10
#define GPIO13_FUN_SWGPIO_13         11
#define GPIO13_FUN_E21_TDI           14

/* GPIO14 function definition */

#define GPIO14_FUN_UNUSED1    1
#define GPIO14_FUN_UNUSED2    2
#define GPIO14_FUN_UNUSED3    3
#define GPIO14_FUN_SPI_SS     4
#define GPIO14_FUN_UNUSED5    5
#define GPIO14_FUN_I2C_SCL    6
#define GPIO14_FUN_UART_SIG6  7
#define GPIO14_FUN_PWM_CH4    8
#define GPIO14_FUN_FEM_GPIO_2 9
#define GPIO14_FUN_GPIP_CH2   10
#define GPIO14_FUN_SWGPIO_14  11
#define GPIO14_FUN_E21_TCK    14

/* GPIO15 function definition */

#define GPIO15_FUN_UNUSED1                 1
#define GPIO15_FUN_UNUSED2                 2
#define GPIO15_FUN_UNUSED3                 3
#define GPIO15_FUN_SPI_SCLK                4
#define GPIO15_FUN_UNUSED5                 5
#define GPIO15_FUN_I2C_SDA                 6
#define GPIO15_FUN_UART_SIG7               7
#define GPIO15_FUN_PWM_CH0                 8
#define GPIO15_FUN_FEM_GPIO_3              9
#define GPIO15_FUN_PSW_IRRCV_OUT_GPIP_CH11 10
#define GPIO15_FUN_SWGPIO_15               11
#define GPIO15_FUN_E21_TDO                 14

/* GPIO16 function definition */

#define GPIO16_FUN_UNUSED1           1
#define GPIO16_FUN_UNUSED2           2
#define GPIO16_FUN_UNUSED3           3
#define GPIO16_FUN_SPI_MISO_SPI_MOSI 4
#define GPIO16_FUN_UNUSED5           5
#define GPIO16_FUN_I2C_SCL           6
#define GPIO16_FUN_UART_SIG0         7
#define GPIO16_FUN_PWM_CH1           8
#define GPIO16_FUN_FEM_GPIO_0        9
#define GPIO16_FUN_UNUSED10          10
#define GPIO16_FUN_SWGPIO_16         11
#define GPIO16_FUN_E21_TMS           14

/* GPIO17 function definition */

#define GPIO17_FUN_UNUSED1           1
#define GPIO17_FUN_SF_D3             2
#define GPIO17_FUN_UNUSED3           3
#define GPIO17_FUN_SPI_MOSI_SPI_MISO 4
#define GPIO17_FUN_UNUSED5           5
#define GPIO17_FUN_I2C_SDA           6
#define GPIO17_FUN_UART_SIG1         7
#define GPIO17_FUN_PWM_CH2           8
#define GPIO17_FUN_FEM_GPIO_1        9
#define GPIO17_FUN_PMIP_DC_TP_OUT    10
#define GPIO17_FUN_SWGPIO_17         11
#define GPIO17_FUN_E21_TDI           14

/* GPIO18 function definition */

#define GPIO18_FUN_UNUSED1    1
#define GPIO18_FUN_SF_D2      2
#define GPIO18_FUN_UNUSED3    3
#define GPIO18_FUN_SPI_SS     4
#define GPIO18_FUN_UNUSED5    5
#define GPIO18_FUN_I2C_SCL    6
#define GPIO18_FUN_UART_SIG2  7
#define GPIO18_FUN_PWM_CH3    8
#define GPIO18_FUN_FEM_GPIO_2 9
#define GPIO18_FUN_UNUSED10   10
#define GPIO18_FUN_SWGPIO_18  11
#define GPIO18_FUN_E21_TCK    14

/* GPIO19 function definition */

#define GPIO19_FUN_UNUSED1    1
#define GPIO19_FUN_SF_D1      2
#define GPIO19_FUN_UNUSED3    3
#define GPIO19_FUN_SPI_SCLK   4
#define GPIO19_FUN_UNUSED5    5
#define GPIO19_FUN_I2C_SDA    6
#define GPIO19_FUN_UART_SIG3  7
#define GPIO19_FUN_PWM_CH4    8
#define GPIO19_FUN_FEM_GPIO_3 9
#define GPIO19_FUN_UNUSED10   10
#define GPIO19_FUN_SWGPIO_19  11
#define GPIO19_FUN_E21_TDO    14

/* GPIO20 function definition */

#define GPIO20_FUN_UNUSED1           1
#define GPIO20_FUN_SF_D0             2
#define GPIO20_FUN_UNUSED3           3
#define GPIO20_FUN_SPI_MISO_SPI_MOSI 4
#define GPIO20_FUN_UNUSED5           5
#define GPIO20_FUN_I2C_SCL           6
#define GPIO20_FUN_UART_SIG4         7
#define GPIO20_FUN_PWM_CH0           8
#define GPIO20_FUN_FEM_GPIO_0        9
#define GPIO20_FUN_UNUSED10          10
#define GPIO20_FUN_SWGPIO_20         11
#define GPIO20_FUN_E21_TMS           14

/* GPIO21 function definition */

#define GPIO21_FUN_UNUSED1           1
#define GPIO21_FUN_SF_CS             2
#define GPIO21_FUN_UNUSED3           3
#define GPIO21_FUN_SPI_MOSI_SPI_MISO 4
#define GPIO21_FUN_UNUSED5           5
#define GPIO21_FUN_I2C_SDA           6
#define GPIO21_FUN_UART_SIG5         7
#define GPIO21_FUN_PWM_CH1           8
#define GPIO21_FUN_FEM_GPIO_1        9
#define GPIO21_FUN_UNUSED10          10
#define GPIO21_FUN_SWGPIO_21         11
#define GPIO21_FUN_E21_TDI           14

/* GPIO22 function definition */

#define GPIO22_FUN_UNUSED1    1
#define GPIO22_FUN_SF_CLK_OUT 2
#define GPIO22_FUN_UNUSED3    3
#define GPIO22_FUN_SPI_SS     4
#define GPIO22_FUN_UNUSED5    5
#define GPIO22_FUN_I2C_SCL    6
#define GPIO22_FUN_UART_SIG6  7
#define GPIO22_FUN_PWM_CH2    8
#define GPIO22_FUN_FEM_GPIO_2 9
#define GPIO22_FUN_UNUSED10   10
#define GPIO22_FUN_SWGPIO_22  11
#define GPIO22_FUN_E21_TCK    14

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

#define GLB_GPIO_PIN_0  0
#define GLB_GPIO_PIN_1  1
#define GLB_GPIO_PIN_2  2
#define GLB_GPIO_PIN_3  3
#define GLB_GPIO_PIN_4  4
#define GLB_GPIO_PIN_5  5
#define GLB_GPIO_PIN_6  6
#define GLB_GPIO_PIN_7  7
#define GLB_GPIO_PIN_8  8
#define GLB_GPIO_PIN_9  9
#define GLB_GPIO_PIN_10 10
#define GLB_GPIO_PIN_11 11
#define GLB_GPIO_PIN_12 12
#define GLB_GPIO_PIN_13 13
#define GLB_GPIO_PIN_14 14
#define GLB_GPIO_PIN_15 15
#define GLB_GPIO_PIN_16 16
#define GLB_GPIO_PIN_17 17
#define GLB_GPIO_PIN_18 18
#define GLB_GPIO_PIN_19 19
#define GLB_GPIO_PIN_20 20
#define GLB_GPIO_PIN_21 21
#define GLB_GPIO_PIN_22 22
#define GLB_GPIO_PIN_MA 23

#define GPIO_FUN_SDIO   1
#define GPIO_FUN_FLASH  2
#define GPIO_FUN_SPI    4
#define GPIO_FUN_I2C    6
#define GPIO_FUN_UART   7
#define GPIO_FUN_PWM    8
#define GPIO_FUN_EXT_PA 8
#define GPIO_FUN_ANALOG 10
#define GPIO_FUN_SWGPIO 11
#define GPIO_FUN_JTAG   14

struct gpio_cfg_s
{
  int gpio_pin;
  int gpio_fun;
  int gpio_mode;
  int pull_type;
  int drive;
  int smt_ctrl;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bl602_gpio_init
 *
 * Description:
 *   Init a gpio pin.
 *
 * Input Parameters:
 *   cfg: gpio configuration
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   Anything else that one might need to know to use this function.
 *
 ****************************************************************************/

EXTERN void bl602_gpio_init(struct gpio_cfg_s *cfg);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_GPIO_H */
