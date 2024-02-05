/****************************************************************************
 * boards/arm/stm32/odrive36/src/odrive.h
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

#ifndef __BOARDS_ARM_STM32_ODRIVE36_SRC_ODRIVE_H
#define __BOARDS_ARM_STM32_ODRIVE36_SRC_ODRIVE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* DRV8301 pins */

#define GPIO_DRV8301_ENGATE (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                             GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN12)
#define GPIO_DRV8301_NFAULT (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD| \
                             GPIO_PIN2)

/* SPI chip selects */

#define GPIO_GATEDRV0_CS (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                          GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN13)
#define GPIO_GATEDRV1_CS (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                          GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN14)

/* QE index pins */

#define GPIO_QE3_INDEX (GPIO_INPUT | GPIO_FLOAT |           \
                        GPIO_EXTI | GPIO_PORTC | GPIO_PIN9)
#define GPIO_QE4_INDEX (GPIO_INPUT | GPIO_FLOAT |           \
                        GPIO_EXTI | GPIO_PORTC | GPIO_PIN15)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture specific initialization
 *
 *   CONFIG_LIB_BOARDCTL=y:
 *     If CONFIG_NSH_ARCHINITIALIZE=y:
 *       Called from the NSH library (or other application)
 *     Otherwise, assumed to be called from some other application.
 *
 *   Otherwise CONFIG_BOARD_LATE_INITIALIZE=y:
 *     Called from board_late_initialize().
 *
 *   Otherwise, bad news:  Never called
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ****************************************************************************/

void stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int stm32_adc_setup(void);
#endif

/****************************************************************************
 * Name: stm32_dac_setup
 *
 * Description:
 *   Initialize DAC and register the DAC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_DAC
int stm32_dac_setup(void);
#endif

/****************************************************************************
 * Name: stm32_foc_setup
 *
 * Description:
 *  Initialize FOC peripherals for the board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FOC
int stm32_foc_setup(void);
#endif

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in initialization to setup
 *   USB-related GPIO pins for the STM32F4Discovery board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_OTGFS
void weak_function stm32_usbinitialize(void);
#endif

#endif /* __BOARDS_ARM_STM32_ODRIVE36_SRC_ODRIVE_H */
