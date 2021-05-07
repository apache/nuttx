/****************************************************************************
 * arch/arm/src/str71x/chip.h
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

#ifndef __ARCH_ARM_SRC_STR71X_CHIP_H
#define __ARCH_ARM_SRC_STR71X_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"     /* Memory map */
#include "str71x_emi.h"     /* External memory interface */
#include "str71x_rccu.h"    /* Reset and clock control unit */
#include "str71x_pcu.h"     /* Power control unit */
#include "str71x_gpio.h"    /* I/O ports */
#include "str71x_eic.h"     /* Enhanced interrupt controller */
#include "str71x_xti.h"     /* External interrupts (XTI) */
#include "str71x_rtc.h"     /* Real Time Clock (RTC) */
#include "str71x_wdog.h"    /* Watchdog timer */
#include "str71x_timer.h"   /* Timers */
#include "str71x_can.h"     /* Controller Area Network (CAN) */
#include "str71x_i2c.h"     /* I2C */
#include "str71x_bspi.h"    /* Buffered SPI (BSPI) */
#include "str71x_uart.h"    /* UART */
#include "str71x_usb.h"     /* USB */
#include "str71x_adc12.h"   /* ADC */
#include "str71x_apb.h"     /* USB */
#include "str71x_flash.h"   /* Flash */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_CHIP_H */
