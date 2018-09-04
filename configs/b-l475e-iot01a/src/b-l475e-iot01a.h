/****************************************************************************
 * configs/b-l475e-iot01a/src/b-l475e-iot01a.h
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

#ifndef __CONFIGS_B_L475E_IOT01A_SRC_B_L475E_IOT01A_H
#define __CONFIGS_B_L475E_IOT01A_SRC_B_L475E_IOT01A_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <arch/stm32l4/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_SPSGRF 1
#define HAVE_MX25R6435F 1
#define HAVE_MX25R6435F_SMARTFS 1

/* SPSGRF support depends on:
 *
 *   CONFIG_STM32L4_SPI3  - SPI3 support
 *   CONFIG_WL_SPIRIT     - Spirit wireless library
 *   CONFIG_SPIRIT_NETDEV - Spirit network driver
 *   CONFIG_SCHED_HPWORK  - HP work queue support
 *   CONFIG_SCHED_LPWORK  - LP work queue support
 *   CONFIG_NET           - Networking enabled
 *   CONFIG_NET_6LOWPAN   - 6LoWPAN stack enabled
 *
 * And probably a few other things.
 */

#if !defined(CONFIG_STM32L4_SPI3)
#  undef HAVE_SPSGRF
#endif

#if !defined(CONFIG_WL_SPIRIT) || !defined(CONFIG_SPIRIT_NETDEV)
#  undef HAVE_SPSGRF
#endif

#if !defined(CONFIG_SCHED_HPWORK) && !defined(CONFIG_SCHED_LPWORK)
#  undef HAVE_SPSGRF
#endif

#if !defined(CONFIG_NET) || !defined(CONFIG_NET_6LOWPAN)
#  undef HAVE_SPSGRF
#endif

#if !defined(CONFIG_MTD_MX25RXX) || !defined(CONFIG_STM32L4_QSPI)
#  undef HAVE_MX25R6435F
#endif

#if !defined(HAVE_MX25R6435F) || !defined(CONFIG_MTD_SMART) || \
    !defined(CONFIG_FS_SMARTFS)
#  undef HAVE_MX25R6435F_SMARTFS
#endif

/* GPIO Definitions *********************************************************/
/* LEDs */

#define GPIO_LED1        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |\
                          GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN5)
#define GPIO_LED2        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |\
                          GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN14)

/* SPSGRF
 *
 * -------- ----------------------- ----------------
 * SPSGRF   Board Signal            STM32L4 pin
 * -------- ----------------------- ----------------
 * SPI_CLK  INTERNAL-SPI3_SCK       PC10 SPI3_SCK
 * SPI_MISO INTERNAL-SPI3_MISO      PC11 SPI3_MISO
 * SPI_MOSI INTERNAL-SPI3_MOSI      PC12 SPI3_MOSI
 * SPI_CS   SPSGRF-915-SPI3_CSN     PB5  GPIO_Output
 * GPIO(3)  SPSGRF-915-GPIO3_EXTI5  PE5  GPIO_EXTI5
 * GPIO(2)  N/C                     N/A
 * GPIO(1)  N/C                     N/A
 * GPIO(0)  N/C                     N/A
 * SDN      SPSGRF-915-SDN          PB15 GPIO_Output
 * -------- ----------------------- ----------------
 *
 * NOTES:
 * - The Interrupt request is active low.
 * - When SDN =1 the Spirit is completely shut down and the contents of the
 *   registers are lost.
 */

#define GPIO_SPSGRF_CS   (GPIO_OUTPUT | GPIO_FLOAT | GPIO_PUSHPULL | \
                          GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
                          GPIO_PORTB | GPIO_PIN5)
#define GPIO_SPSGRF_INT  (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_100MHz | \
                          GPIO_EXTI | GPIO_PORTE | GPIO_PIN5)
#define GPIO_SPSGRF_SDN  (GPIO_OUTPUT | GPIO_FLOAT | GPIO_PUSHPULL | \
                          GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
                          GPIO_PORTB | GPIO_PIN15)

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_bringup
 *
 * Description:
 *   Called either by board_intialize() if CONFIG_BOARD_INITIALIZE or by
 *   board_app_initialize if CONFIG_LIB_BOARDCTL is selected.  This function
 *   initializes and configures all on-board features appropriate for the
 *   selected configuration.
 *
 ****************************************************************************/

#if defined(CONFIG_LIB_BOARDCTL) || defined(CONFIG_BOARD_INITIALIZE)
int stm32l4_bringup(void);
#endif

/****************************************************************************
 * Name: stm32l4_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Nucleo-F401RE and
 *   Nucleo-F411RE boards.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32L4_SPI1) || defined(CONFIG_STM32L4_SPI2) || \
    defined(CONFIG_STM32L4_SPI3)
void weak_function stm32l4_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_timer_driver_setup
 *
 * Description:
 *   Configure the timer drivers.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int stm32l4_timer_driver_setup(void);
#endif

/****************************************************************************
 * Name: stm32l4_spirit_initialize
 *
 * Description:
 *   Initialize the Spirit device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef HAVE_SPSGRF
int stm32l4_spirit_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_B_L475E_IOT01A_SRC_B_L475E_IOT01A_H */
