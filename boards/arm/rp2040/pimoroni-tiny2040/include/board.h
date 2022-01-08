/****************************************************************************
 * boards/arm/rp2040/pimoroni-tiny2040/include/board.h
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

#ifndef __BOARDS_ARM_RP2040_PIMORONI_TINY2040_INCLUDE_BOARD_H
#define __BOARDS_ARM_RP2040_PIMORONI_TINY2040_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "rp2040_i2cdev.h"
#include "rp2040_spidev.h"
#include "rp2040_i2sdev.h"

#include "rp2040_spisd.h"

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

#define MHZ                     1000000

#define BOARD_XOSC_FREQ         (12 * MHZ)
#define BOARD_PLL_SYS_FREQ      (125 * MHZ)
#define BOARD_PLL_USB_FREQ      (48 * MHZ)

#define BOARD_REF_FREQ          (12 * MHZ)
#define BOARD_SYS_FREQ          (125 * MHZ)
#define BOARD_PERI_FREQ         (125 * MHZ)
#define BOARD_USB_FREQ          (48 * MHZ)
#define BOARD_ADC_FREQ          (48 * MHZ)
#define BOARD_RTC_FREQ          46875

#define BOARD_UART_BASEFREQ     BOARD_PERI_FREQ

#define BOARD_TICK_CLOCK        (1 * MHZ)

/* GPIO definitions *********************************************************/

#define BOARD_GPIO_LED_PIN_R    18
#define BOARD_GPIO_LED_PIN_G    19
#define BOARD_GPIO_LED_PIN_B    20

#define BOARD_GPIO_BOOT_PIN     23

#define BOARD_NGPIOOUT          3
#define BOARD_NGPIOIN           1
#define BOARD_NGPIOINT          0

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
 * Name: rp2040_boardearlyinitialize
 *
 * Description:
 *
 ****************************************************************************/

void rp2040_boardearlyinitialize(void);

/****************************************************************************
 * Name: rp2040_boardinitialize
 *
 * Description:
 *
 ****************************************************************************/

void rp2040_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_RP2040_PIMORONI_TINY2040_INCLUDE_BOARD_H */
