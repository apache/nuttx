/****************************************************************************
 * configs/freedom-k28f/src/freedom-k28f.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __CONFIGS_FREEDOM_K28F_SRC_FREEDOM_K28F_H
#define __CONFIGS_FREEDOM_K28F_SRC_FREEDOM_K28F_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Freedom-K28F GPIOs *******************************************************/
/* An RGB LED is connected through GPIO as shown below:
 *
 *   LED    K28
 *   ------ -------------------------------------------------------
 *   RED    PTE6
 *   GREEN  PTE7
 *   BLUE   PTE8
 */

#define GPIO_LED_R         (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTE | PIN6)
#define GPIO_LED_G         (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTE | PIN7)
#define GPIO_LED_B         (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTE | PIN8)

/* Two push buttons, SW2 and SW3, are available on FRDM-K28F board, where SW2 is
 * connected to PTA4 and SW3 is connected to PTD0. Besides the general purpose
 * input/output functions, SW2 and SW3 can be low-power wake up signal. Also, only
 * SW3 can be a non-maskable interrupt.
 *
 *   Switch    GPIO Function
 *   --------- ---------------------------------------------------------------
 *   SW2       PTA4/NMI_B
 *   SW3       PTD0/LLWU_P12
 */

#define GPIO_SW2           (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTA | PIN4)
#define GPIO_SW3           (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTD | PIN0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

struct i2c_master_s;  /* Forward reference */

#ifdef CONFIG_KINETIS_I2C0
extern FAR struct i2c_master_s* g_i2c0_dev;
#endif
#ifdef CONFIG_KINETIS_I2C1
extern FAR struct i2c_master_s* g_i2c1_dev;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k28_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int k28_bringup(void);

/****************************************************************************
 * Name: k28_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the KwikStik-K40
 *   board.
 *
 ****************************************************************************/

void weak_function k28_spidev_initialize(void);

/****************************************************************************
 * Name: k28_i2cdev_initialize
 *
 * Description:
 *   Called to configure I2C
 *
 *****************************************************************************/

void k28_i2cdev_initialize(void);

/****************************************************************************
 * Name: k28_usbdev_initialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the Freedom K28F board.
 *
 ****************************************************************************/

extern void weak_function k28_usbdev_initialize(void);

/****************************************************************************
 * Name: k28_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int k28_pwm_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_FREEDOM_K28F_SRC_FREEDOM_K28F_H */
