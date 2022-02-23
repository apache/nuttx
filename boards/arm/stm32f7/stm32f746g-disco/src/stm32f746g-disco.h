/****************************************************************************
 * boards/arm/stm32f7/stm32f746g-disco/src/stm32f746g-disco.h
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

#ifndef __BOARDS_ARM_STM32F7_STM32F746G_DISCO_SRC_STM32F746G_DISCO_H
#define __BOARDS_ARM_STM32F7_STM32F746G_DISCO_SRC_STM32F746G_DISCO_H

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

#ifdef CONFIG_STM32F7_SDMMC
#define HAVE_SDIO
#else
#undef HAVE_SDIO
#endif

/* STM32F736G Discovery GPIOs */

/* The STM32F746G-DISCO board has numerous LEDs but only one,
 * LD1 located near the reset button, that can be controlled by software
 * (LD2 is a power indicator, LD3-6 indicate USB status, LD7 is
 * controlled by the ST-Link).
 *
 * LD1 is controlled by PI1 which is also the SPI2_SCK at the Arduino
 * interface.
 * One end of LD1 is grounded so a high output on PI1 will illuminate the
 * LED.
 */

#define GPIO_LD1           (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTI | GPIO_PIN1)

/* Pushbutton B1, labelled "User", is connected to GPIO PI11.
 * A high value will be sensed when the button is depressed.
 * Note that the EXTI interrupt is configured.
 */

#define MIN_IRQBUTTON      BUTTON_USER
#define MAX_IRQBUTTON      BUTTON_USER
#define NUM_IRQBUTTONS     1

#define GPIO_BTN_USER      (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTI | GPIO_PIN11)

/* Sporadic scheduler instrumentation.
 * This configuration has been used for evaluating the NuttX sporadic
 *  scheduler.
 * In this evaluation, two GPIO outputs are used.
 * One indicating the priority (high or low) of the sporadic thread and one
 * indicating where the thread is running or not.
 *
 * There is nothing special about the pin selections:
 *
 *   Arduino D2 PG6 - Indicates priority
 *   Arduino D4 PG7 - Indicates that the thread is running
 */

#define GPIO_SCHED_HIGHPRI (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTG | GPIO_PIN6)
#define GPIO_SCHED_RUNNING (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                            GPIO_PORTG | GPIO_PIN7)

#define GPIO_LCD_DISP      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                            GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN12)

#define GPIO_LCD_BL        (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                            GPIO_OUTPUT_SET|GPIO_PORTK|GPIO_PIN3)

/* SD/TF Card'detected pin */

#define GPIO_SDIO_NCD      (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)

#define SDIO_SLOTNO        0
#define SDIO_MINOR         0

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

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
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the
 *   stm32f746g-disco board.
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

 * Name: stm32_enablefmc
 *
 * Description:
 *  enable clocking to the FMC module
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_FMC
void stm32_enablefmc(void);
#endif

/****************************************************************************
 * Name: stm32_disablefmc
 *
 * Description:
 *  disable clocking to the FMC module
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_FMC
void stm32_disablefmc(void);
#endif

/****************************************************************************
 * Name: stm32_tsc_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.  This function will register the driver as
 *   /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_INPUT_FT5X06
int stm32_tsc_setup(int minor);
#endif

#ifdef CONFIG_MTD_N25QXXX
int stm32_n25qxxx_setup(void);
#endif

#ifdef HAVE_SDIO
int stm32_sdio_initialize(void);
#endif

#ifdef CONFIG_AUDIO_WM8994
int stm32_wm8994_initialize(int minor);
#endif

#endif /* __ASSEMBLY__ */

#endif /* __BOARDS_ARM_STM32F7_STM32F746G_DISCO_SRC_STM32F746G_DISCO_H */
