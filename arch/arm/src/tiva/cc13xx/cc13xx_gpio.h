/****************************************************************************
 * arch/arm/src/tiva/cc13xx/cc13xx_gpio.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * With modifications from Calvin Maguranis <calvin.maguranis@trd2inc.com>
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

#ifndef __ARCH_ARM_SRC_TIVA_CC13XX_CC13XX_GPIO_H
#define __ARCH_ARM_SRC_TIVA_CC13XX_CC13XX_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_gpio.h"
#include "hardware/tiva_ioc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bit-encoded input to tiva_configgpio() ***********************************/

/* 64-Bit Encoding:
 *
 *   Word 1: .... .... .... ....  .... ..OV ...D DDDD
 *   Word 2: .HIC CIII 210. .GEE  .PPS AASS RWPP PPPP
 *
 * Word 1: Describes GPIO and provides DIO index
 *
 *   O      - 1 bit    GPIO output
 *   V      - 1 bit    GPIO output initial value
 *   DDDDD  - 5 bits   DIO 0-31
 *
 * Word 2: An image of the IOC configuration register (see definitions in
 * hardware/c13x0/c13x0_ioc.h and hardware.cc13x2_cc26x2/cc13x2_cc26x2_ioc)
 *
 *   PPPPPP - 6 bits.  Port ID.  Selects DIO usage
 *   W      - 1 bit     Input edge asserts MCU_WU event (CC13x2/CC26x only)
 *   R      - 1 bit    Input edge asserts RTC event (CC13x2/CC26x only)
 *   SS     - 2 bits   Drive strength
 *   AA     - 2 bits   I/O current mode
 *   S      - 1 bit    Reduced output slew enable
 *   PP     - 2 bits   Pull-up mode control
 *   EE     - 2 bits   Edge event generation
 *   G      - 1 bit    Enable interrupt Generation
 *   0      - 1 bit    Edge asserts AON_PROG0 (CC13x2/CC26x only)
 *   1      - 1 bit    Edge asserts AON_PROG1 (CC13x2/CC26x only)
 *   2      - 1 bit    Edge asserts AON_PROG2 (CC13x2/CC26x only)
 *   III    - 3 bits   I/O mode
 *   CC     - 2 bits   Wakeup Configuration
 *   I      - 1 bit    Input Enable
 *   H      - 1 bit    Input hysteresis
 */

/* GPIO output:
 *
 *   Word 1: .... .... .... ....  .... ..O. .... ....
 *
 * Valid only if Port ID=IOC_IOCFG_PORTID_GPIO
 */

#define GPIO_OUTPUT                 (1 << 9) /* Bit 9: GPIO output (if GPIO) */

/* GPIO output initial value:
 *
 *   Word 1: .... .... .... ....  .... ...V .... ....
 *
 * Valid only if Port ID=IOC_IOCFG_PORTID_GPIO and GPIO_OUTPUT selected.
 */

#define GPIO_VALUE_SHIFT            8                        /* Bit 8: If GPIO output,
                                                              * initial value of output */
#define GPIO_VALUE_MASK             (1 << GPIO_VALUE_SHIFT)
#  define GPIO_VALUE_ZERO           (0 << GPIO_VALUE_SHIFT)  /*   Initial value is zero */
#  define GPIO_VALUE_ONE            (1 << GPIO_VALUE_SHIFT)  /*   Initial value is one */

/* DIO:
 *
 *   Word 1: .... .... .... ....  .... .... ...D DDDD
 */

#define GPIO_DIO_SHIFT              (0)      /* Bits 0-4:  DIO */
#define GPIO_DIO_MASK               (0x1f << GPIO_DIO_SHIFT)
#  define GPIO_DIO(n)               ((uint32_t)(n) << GPIO_DIO_SHIFT)

/* Helper Definitions *******************************************************/

#define IOC_STD_INPUT               (IOC_IOCFG_IOCURR_2MA | \
                                     IOC_IOCFG_IOSTR_AUTO | \
                                     IOC_IOCFG_PULLCTL_DIS | \
                                     IOC_IOCFG_EDGEDET_NONE | \
                                     IOC_IOCFG_IOMODE_NORMAL | \
                                     IOC_IOCFG_IE)
#define IOC_STD_OUTPUT              (IOC_IOCFG_IOCURR_2MA | \
                                     IOC_IOCFG_IOSTR_AUTO | \
                                     IOC_IOCFG_PULLCTL_DIS | \
                                     IOC_IOCFG_EDGEDET_NONE | \
                                     IOC_IOCFG_IOMODE_NORMAL)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure represents the pin configuration */

struct cc13xx_pinconfig_s
{
  uint32_t gpio;                /* GPIO and DIO definitions */
  uint32_t ioc;                 /* IOC configuration register image */
};

/* This opaque type permits common function prototype for GPIO functions
 * across all MCUs.
 */

typedef const struct cc13xx_pinconfig_s *pinconfig_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_TIVA_CC13XX_CC13XX_GPIO_H */
