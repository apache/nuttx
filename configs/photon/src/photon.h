/****************************************************************************
 * configs/photon/src/photon.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Simon Piriou <spiriou31@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __CONFIGS_PHOTON_SRC_PHOTON_H
#define __CONFIGS_PHOTON_SRC_PHOTON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <arch/stm32/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LEDs */

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN13)

/* BUTTONS -- EXTI interrupts are available on Photon board button */

#define MIN_IRQBUTTON   BOARD_BUTTON1
#define MAX_IRQBUTTON   BOARD_BUTTON1
#define NUM_IRQBUTTONS  1

#define GPIO_BUTTON1    (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTC|GPIO_PIN7)

/* WLAN chip */

#define SDIO_WLAN0_SLOTNO 0 /* Photon board has only one sdio device */
#define SDIO_WLAN0_MINOR  0 /* Register "wlan0" device */

#define GPIO_WLAN0_RESET (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                          GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN1)

#define GPIO_WLAN0_32K_CLK (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                            GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)

#define GPIO_WLAN0_OOB_INT (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|\
                           GPIO_PORTB|GPIO_PIN0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Called either by board_intialize() if CONFIG_BOARD_INITIALIZE or by
 *   board_app_initialize if CONFIG_LIB_BOARDCTL is selected.  This function
 *   initializes and configures all on-board features appropriate for the
 *   selected configuration.
 *
 ****************************************************************************/

#if defined(CONFIG_LIB_BOARDCTL) || defined(CONFIG_BOARD_INITIALIZE)
int stm32_bringup(void);
#endif

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Photon board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void);

/****************************************************************************
 * Name: photon_watchdog_initialize()
 *
 * Description:
 *   Perform architecture-specific initialization of the Watchdog hardware.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_PHOTON_WDG
int photon_watchdog_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_rgbled_setup
 *
 * Description:
 *   This function is called by board initialization logic to configure the
 *   RGB LED driver.  This function will register the driver as /dev/rgbled0.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RGBLED
int stm32_rgbled_setup(void);
#endif

/****************************************************************************
 * Name: photon_wlan_initialize
 *
 * Description:
 *   Initialize wlan hardware and driver for Photon board.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_PHOTON_WLAN
int photon_wlan_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in initialization to setup
 *   USB-related GPIO pins for the Photon board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_OTGHS
void weak_function stm32_usbinitialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_PHOTON_SRC_PHOTON_H */
