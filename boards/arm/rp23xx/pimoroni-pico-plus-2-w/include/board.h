/****************************************************************************
 * boards/arm/rp23xx/pimoroni-pico-plus-2-w/include/board.h
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

#ifndef __BOARDS_ARM_RP23XX_PIMORONI_PICO_PLUS_2_W_INCLUDE_BOARD_H
#define __BOARDS_ARM_RP23XX_PIMORONI_PICO_PLUS_2_W_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "rp23xx_i2cdev.h"
#include "rp23xx_spidev.h"
#include "rp23xx_i2sdev.h"
#include "rp23xx_spisd.h"

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

#define MHZ                     1000000

#define BOARD_XOSC_FREQ         (12 * MHZ)
#define BOARD_XOSC_STARTUPDELAY 1
#define BOARD_PLL_SYS_FREQ      (150 * MHZ)
#define BOARD_PLL_USB_FREQ      (48 * MHZ)

#define BOARD_REF_FREQ          (12 * MHZ)
#define BOARD_SYS_FREQ          (150 * MHZ)
#define BOARD_PERI_FREQ         (150 * MHZ)
#define BOARD_USB_FREQ          (48 * MHZ)
#define BOARD_ADC_FREQ          (48 * MHZ)
#define BOARD_HSTX_FREQ         (150 * MHZ)

#define BOARD_UART_BASEFREQ     BOARD_PERI_FREQ

#define BOARD_TICK_CLOCK        (1 * MHZ)

/* definitions for pico-sdk */

/* GPIO definitions *********************************************************/

#define BOARD_NGPIOOUT          1
#define BOARD_NGPIOIN           1
#define BOARD_NGPIOINT          1

/* LED definitions **********************************************************/

/* The only LED on this board is wired to GPIO 0 of the CYW43439 wireless
 * chip, not to a GPIO of the RP2350.  Driving it means sending an iovar
 * request over the gSPI bus, which can only be done from task context and
 * only once the wireless chip has been activated by bringing wlan0 up.
 *
 * That rules out CONFIG_ARCH_LEDS -- board_autoled_on() is called from
 * interrupt handlers and from assertion handling, where a bus transaction
 * is not permissible.  So this board provides the user LED interface only
 * (CONFIG_USERLED) and does not select ARCH_HAVE_LEDS.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_NLEDS       1

#define BOARD_LED_GREEN   BOARD_LED1

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)

/* BUTTON definitions *******************************************************/

/* The board has two physical buttons, BOOT and RESET.  BOOT is also wired
 * to GPIO 45, active low, so once NuttX is running it can be read as an
 * ordinary user button.  pico-sdk calls that pin the board's USER_SW_PIN.
 * RESET is not visible as a GPIO.
 */

#define NUM_BUTTONS       1

#define BUTTON_USER1      0
#define BUTTON_USER1_BIT  (1 << BUTTON_USER1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_boardearlyinitialize
 *
 * Description:
 *
 ****************************************************************************/

void rp23xx_boardearlyinitialize(void);

/****************************************************************************
 * Name: rp23xx_boardinitialize
 *
 * Description:
 *
 ****************************************************************************/

void rp23xx_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_RP23XX_PIMORONI_PICO_PLUS_2_W_INCLUDE_BOARD_H */
