/****************************************************************************
 * boards/arm/stm32f7/stm32f769i-disco/src/stm32f769i-disco.h
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

#ifndef __BOARDS_ARM_STM32F7_STM32F769I_DISCO_SRC_STM32F769I_DISCO_H
#define __BOARDS_ARM_STM32F7_STM32F769I_DISCO_SRC_STM32F769I_DISCO_H

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

/* STM32F769 Discovery GPIOs ************************************************/

/* The STM32F769I-DISCO has one user controllable LED: LD3.
 *
 * LD3 is controlled by PA12 which is also the SPI2_SCK at the Arduino
 * interface.
 * LD3 is on when PA12 is high.
 */

#define GPIO_LD3           (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTA | GPIO_PIN12)

/* Pushbutton B1, labelled "User", is connected to GPIO PA0.
 * A high value will be sensed when the button is depressed.
 * Note that the EXTI interrupt is configured.
 */

#define GPIO_BTN_USER      (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTA | GPIO_PIN0)

/* Sporadic scheduler instrumentation.
 * This configuration has been used for evaluating the NuttX sporadic
 * scheduler.
 * In this evaluation, two GPIO outputs are used. One indicating the priority
 * (high or low) of the sporadic thread and one indicating where the thread
 * is running or not.
 *
 * There is nothing special about the pin selections:
 *
 *   Arduino D2 PJ1 - Indicates priority1
 *   Arduino D4 PJ0 - Indicates that the thread is running
 */

#define GPIO_SCHED_HIGHPRI (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTJ | GPIO_PIN1)
#define GPIO_SCHED_RUNNING (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTJ | GPIO_PIN0)

/* PWM
 *
 * The STM32F769 Discovery can be configured to output a pulse train using
 * TIM1 CH4 on PA11.
 */

#define STM32F769I_DISCO_PWMTIMER   1
#define STM32F769I_DISCO_PWMCHANNEL 4

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int stm32_pwm_setup(void);
#endif

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the stm32f769i-disco
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void);

/****************************************************************************
 * Name: arch_sporadic_initialize
 *
 * Description:
 *   This configuration has been used for evaluating the NuttX sporadic
 *   scheduler.
 *
 ****************************************************************************/

#ifdef CONFIG_SPORADIC_INSTRUMENTATION
void arch_sporadic_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_sdram_initialize
 *
 * Description:
 *   Called from stm32_bringup to initialize external SDRAM access.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_FMC
void stm32_sdram_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32F7_STM32F769I_DISCO_SRC_STM32F769I_DISCO_H */
