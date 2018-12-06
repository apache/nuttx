/****************************************************************************
 * arch/arm/src/tiva/cc13x0/cc13x0_gpio.h
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

#ifndef __ARCH_ARM_SRC_TIVA_CC13X0_CC13X0_GPIO_H
#define __ARCH_ARM_SRC_TIVA_CC13X0_CC13X0_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bit-encoded input to tiva_configgpio() ***********************************/

/* Encoding:
 *
 *    PPPP PPIO WCCR SSHS  PPAA GIII EEVD DDDD
 *
 * PPPPPP - 6 bits.  Port ID (see definitions in hardware/c13x0/c13x0_ioc.h
 *                   and hardware.cc13x2_cc26x2/cc13x2_cc26x2_ioc)
 * I      - 1 bit    Input Enable
 * O      - 1 bit    GPIO output
 * W      - 1 bit    Wakeup enable (CC13x2/CC26x only)
 * CC     - 2 bits   Wakeup Configuration
 * R      - 1 bit    RTC event enable (CC13x2/CC26x only)
 * SS     - 2 bits   Drive strength
 * H      - 1 bit    Input hysteresis
 * S      - 1 bit    Reduced output slew enable
 * PP     - 2 bits   Pull-up mode
 * AA     - 2 bits   Current mode
 * G      - 1 bit    Enable interrupt Generation
 * III    - 3 bits   I/O mode
 * EE     - 2 bits   Edge detect mode
 * 0      - 1 bit    Edge asserts AON_PROG0 (CC13x2/CC26x only) NOTE 1
 * 1      - 1 bit    Edge asserts AON_PROG1 (CC13x2/CC26x only) NOTE 1
 * 2      - 1 bit    Edge asserts AON_PROG2 (CC13x2/CC26x only) NOTE 1
 * V      - 1 bit    GPIO output initial value
 * DDDDD  - 5 bits   DIO 0-31
 *
 * NOTE 1:  Not currently implemented because no bits are available in
 * the uint32_t.  We need more bits!
 */

/* Port ID
 *
 *    PPPP PP.. .... ....  .... .... .... ....
 *
 * See PORTID definitions in the IOC register definitions file.
 */

#define GPIO_PORTID_SHIFT           (26)      /* Bits 26-31:  Port ID */
#define GPIO_PORTID_MASK            (0x3f << GPIO_PORTID_SHIFT)
#  define GPIO_PORTID(n)            ((uint32_t)(n) << GPIO_PORTID_SHIFT)

/* Input Enable:
 *
 *    .... ..I. .... ....  .... .... .... ....
 */

#define GPIO_IE                     (1 << 25) /* Bit 25: Input Enable */

/* GPIO output:
 *
 *    .... ...O .... ....  .... .... .... ....
 *
 * Valid only if Port ID=IOC_IOCFG_PORTID_GPIO
 */

#define GPIO_OUTPUT                 (1 << 24) /* Bit 24: Input Enable */

/* Wakeup enable (CC13x2/CC26x only):
 *
 *    .... .... W... ....  .... .... .... ....
 */

#define GPIO_WUEN                   (1 << 23) /* Bit 23: Input edge asserts MCU_WU event */

/* Wakeup Configuration:
 *
 *    .... .... .CC. ....  .... .... .... ....
 */

#define GPIO_WUCFG_SHIFT            (21)      /* Bits 21-22:  Wakeup Configuration */
#define GPIO_WUCFG_MASK             (3 << GPIO_WUCFG_SHIFT)
#  define GPIO_WUCFG_NONE           (0 << GPIO_WUCFG_SHIFT) /* 0, 1: Wakeup disabled */
#  define GPIO_WUCFG_ENABLE         (2 << GPIO_WUCFG_SHIFT) /* 2, 3: Wakeup enabled */
#  define GPIO_WUCFG_WAKEUPL        (2 << GPIO_WUCFG_SHIFT) /* Wakeup on transition low */
#  define GPIO_WUCFG_WEKUPH         (3 << GPIO_WUCFG_SHIFT) /* Wakeup on transition high */

/* RTC event enable (CC13x2/CC26x only):
 *
 *    .... .... ...R ....  .... .... .... ....
 */

#define GPIO_RTCEN                  (1 << 20) /* Bit 20: Input edge asserts RTC event */

/* Drive strength:
 *
 *    .... .... .... SS..  .... .... .... ....
 */

#define GPIO_IOSTR_SHIFT            (18)     /* Bits 18-19: I/O drive strength */
#define GPIO_IOSTR_MASK             (3 << GPIO_IOSTR_SHIFT)
#  define GPIO_IOSTR_AUTO           (0 << GPIO_IOSTR_SHIFT) /* Automatic drive strength */
#  define GPIO_IOSTR_MIN            (1 << GPIO_IOSTR_SHIFT) /* Minimum drive strength */
#  define GPIO_IOSTR_MED            (2 << GPIO_IOSTR_SHIFT) /* Medium drive strength */
#  define GPIO_IOSTR_MAX            (3 << GPIO_IOSTR_SHIFT) /* Maximum drive strength */

/* Input hysteresis:
 *
 *    .... .... .... ..H.  .... .... .... ....
 */

#define GPIO_HYSTEN                 (1 << 17) /* Bit 17: Input hysteresis enable */

/* Reduced output slew enable:
 *
 *    .... .... .... ...S  .... .... .... ....
 */

#define GPIO_SLEW_RED               (1 << 16) /* Bit 16:  Reduces output slew rate */

/* Pull-up mode:
 *
 *    .... .... .... ....  PP.. .... .... ....
 */

#define GPIO_PULL_SHIFT             (14)      /* Bits 14-15: Pull Control */
#define GPIO_PULL_MASK              (3 << GPIO_PULL_SHIFT)
#  define GPIO_PULL_DISABLE         (3 << GPIO_PULL_SHIFT) /* No pull */
#  define GPIO_PULL_DOWN            (1 << GPIO_PULL_SHIFT) /* Pull down */
#  define GPIO_PULL_UP              (2 << GPIO_PULL_SHIFT) /* Pull up */

/* Current mode:
 *
 *    .... .... .... ....  ..AA .... .... ....
 */

#define GPIO_IOCURR_SHIFT           (12)      /* Bits 12-13: I/O current mode */
#define GPIO_IOCURR_MASK            (3 << GPIO_IOCURR_SHIFT)
#  define GPIO_IOCURR_2MA           (0 << GPIO_IOCURR_SHIFT) /* Extended-Current (EC) mode */
#  define GPIO_IOCURR_4MA           (1 << GPIO_IOCURR_SHIFT) /* High-Current (HC) mode */
#  define GPIO_IOCURR_8MA           (2 << GPIO_IOCURR_SHIFT) /* Low-Current (LC) mode */

/* Enable edge interrupt generation:
 *
 *    .... .... .... ....  .... G... .... ....
 */

#define GPIO_EDGE_IRQEN             (1 << 11) /* Bit 11: Enable edge interrupt generation */

/* I/O mode:
 *
 *    .... .... .... ....  .... .III .... ....
 */

#define GPIO_IOMODE_SHIFT           (8)       /* Bits 8-10:  I/O Mode */
#define GPIO_IOMODE_MASK            (7 << IOC_IOCFG1_IOMODE_SHIFT)
#  define GPIO_IOMODE_NORMAL        (0 << IOC_IOCFG1_IOMODE_SHIFT) /* Normal I/O */
#  define GPIO_IOMODE_INV           (1 << IOC_IOCFG1_IOMODE_SHIFT) /* Inverted I/O */
#  define GPIO_IOMODE_OPENDR        (4 << IOC_IOCFG1_IOMODE_SHIFT) /* Open drain */
#  define GPIO_IOMODE_OPENDRINV     (5 << IOC_IOCFG1_IOMODE_SHIFT) /* Open drain, inverted I/O */
#  define GPIO_IOMODE_OPENSRC       (6 << IOC_IOCFG1_IOMODE_SHIFT) /* Open source */
#  define GPIO_IOMODE_OPENSRCINV    (7 << IOC_IOCFG1_IOMODE_SHIFT) /* Open source, inverted I/O */

/* Edge detect mode
 *
 *    .... .... .... ....  .... .... EE.. ....
 */

#define GPIO_EDGE_SHIFT             (6)       /* Bits 6-7: Enable edge events generation */
#define GPIO_EDGE_MASK              (3 << GPIO_EDGE_SHIFT)
#  define GPIO_EDGE_NONE            (0 << GPIO_EDGE_SHIFT) /* No edge detection */
#  define GPIO_EDGE_NEG             (1 << GPIO_EDGE_SHIFT) /* Negative edge detection */
#  define GPIO_EDGE_POS             (2 << GPIO_EDGE_SHIFT) /* Positive edge detection */
#  define GPIO_EDGE_BOTH            (3 << GPIO_EDGE_SHIFT) /* Both edge detection */

#if 0 /* Need more bits! */
/* Edge asserts AON_PROG0/1/2 (CC13x2/CC26x only):
 *
 *    .... .... .... ....  .... .... .... ....
 */

#define GPIO_AON_PROG0              (1 << xx) /* Bit xx: Input edge asserts AON_PROG0 */
#define GPIO_AON_PROG1              (1 << xx) /* Bit xx: Input edge asserts AON_PROG1 */
#define GPIO_AON_PROG2              (1 << xx) /* Bit xx: Input edge assert AON_PROG2 */

#endif

/* GPIO output initial value:
 *
 *    .... .... .... ....  .... .... ..V. ....
 *
 * Valid only if Port ID=IOC_IOCFG_PORTID_GPIO and GPIO_OUTPUT selected.
 */

#define GPIO_VALUE_SHIFT            8                          /* Bit 5: If output, inital value of output */
#define GPIO_VALUE_MASK             (1 << GPIO_VALUE_SHIFT)
#  define GPIO_VALUE_ZERO           (0 << GPIO_VALUE_SHIFT)    /*   Initial value is zero */
#  define GPIO_VALUE_ONE            (1 << GPIO_VALUE_SHIFT)    /*   Initial value is one */

/* DIO:
 *
 *    .... .... .... ....  .... .... ...D DDDD
 */

#define GPIO_DIO_SHIFT              (0)      /* Bits 0-4:  DIO */
#define GPIO_DIO_MASK               (0x1f << GPIO_PORTID_SHIFT)
#  define GPIO_DIO(n)               ((uint32_t)(n) << GPIO_PORTID_SHIFT)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_TIVA_CC13X0_CC13X0_GPIO_H */
