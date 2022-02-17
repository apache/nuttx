/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_pinconfig.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_LPC43_PINCONFIG_H
#define __ARCH_ARM_SRC_LPC43XX_LPC43_PINCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Include the chip capabilities and SCU definitions file */

#include "chip.h"
#include "hardware/lpc43_scu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Each configurable pin can be individually configured by software in
 * several modes. The following definitions provide the bit encoding that
 * is used to define a pin configuration.
 * Note that these pins do not corresponding GPIO ports and pins.
 *
 * 21-bit Encoding:
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * FFFU UDDI GWSS SSSP PPPP
 */

/* Alternate function number:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * FFF. .... .... .... ....
 */

#define PINCONF_FUNC_SHIFT            (17)       /* Bits 16-18: Alternate function number */
#define PINCONF_FUNC_MASK             (7 << PINCONF_FUNC_SHIFT)
#  define PINCONF_FUNC(n)             ((n) << PINCONF_FUNC_SHIFT)
#  define PINCONF_FUNC0               (0 << PINCONF_FUNC_SHIFT)
#  define PINCONF_FUNC1               (1 << PINCONF_FUNC_SHIFT)
#  define PINCONF_FUNC2               (2 << PINCONF_FUNC_SHIFT)
#  define PINCONF_FUNC3               (3 << PINCONF_FUNC_SHIFT)
#  define PINCONF_FUNC4               (4 << PINCONF_FUNC_SHIFT)
#  define PINCONF_FUNC5               (5 << PINCONF_FUNC_SHIFT)
#  define PINCONF_FUNC6               (6 << PINCONF_FUNC_SHIFT)
#  define PINCONF_FUNC7               (7 << PINCONF_FUNC_SHIFT)

/* Pull-up/down resisters.  These selections are available for all pins but
 * may not make sense for all pins.  NOTE: that both pull up and down is
 * not precluded.
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * ...U U... .... .... ....
 */

#define PINCONF_PULLUP                (1 << 16) /* Bit 16: 1=Pull-up */
#define PINCONF_PULLDOWN              (1 << 15) /* Bit 15: 1=Pull-down */
#define PINCONF_FLOAT                 (0)       /* Bit 15-16=0 if neither */

#define PINCONF_IS_PULLUP(p)          (((p) & PINCONF_PULLUP) != 0)
#define PINCONF_IS_PULLDOWN(p)        (((p) & PINCONF_PULLDOWN) != 0)
#define PINCONF_IS_FLOAT(p)           (((p) & (PINCONF_PULLUP|PINCONF_PULLDOWN) == 0)

/* Drive strength.  These selections are available only for high-drive pins
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .DD. .... .... ....
 */

#define PINCONF_DRIVE_SHIFT           (13)       /* Bits 12-13 = Pin drive strength */
#define PINCONF_DRIVE_MASK            (3 << PINCONF_DRIVE_SHIFT)
#  define PINCONF_DRIVE_NORMAL        (0 << PINCONF_DRIVE_SHIFT)
#  define PINCONF_DRIVE_MEDIUM        (1 << PINCONF_DRIVE_SHIFT)
#  define PINCONF_DRIVE_HIGH          (2 << PINCONF_DRIVE_SHIFT)
#  define PINCONF_DRIVE_ULTRA         (3 << PINCONF_DRIVE_SHIFT)

/* Input buffer enable
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... ...I .... .... ....
 */

#define PINCONF_INBUFFER              (1 << 12)  /* Bit 11: 1=Enabled input buffer */
#define PINCONF_INBUFFER_ENABLED(p)   (((p) & PINCONF_INBUFFER) != 0)

/* Glitch filter enable
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... G... .... ....
 */

#define PINCONF_GLITCH                (1 << 11)  /* Bit 10: 1=Glitch filter enable */
#define PINCONF_GLITCH_ENABLE(p)      (((p) & PINCONF_GLITCH) == 0)

/* Slew rate
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .W.. .... ....
 */

#define PINCONF_SLEW_FAST             (1 << 10)   /* Bit 9: 1=Alternate function */
#define PINCONF_SLEW_SLOW             (0)         /* Bit 9: 0=Normal function */

#define PINCONF_IS_SLEW_FAST(p)       (((p) & PINCONF_SLEW_FAST) != 0)
#define PINCONF_IS_SLOW_SLOW(p)       (((p) & PINCONF_SLEW_FAST) == 0)

/* Pin configuration sets:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... ..SS SSS. ....
 */

#define PINCONF_PINS_SHIFT            (5)        /* Bits 5-9: Pin set */
#define PINCONF_PINS_MASK             (31 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINS0               (0 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINS1               (1 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINS2               (2 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINS3               (3 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINS4               (4 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINS5               (5 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINS6               (6 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINS7               (7 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINS8               (8 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINS9               (9 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINSA               (10 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINSB               (11 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINSC               (12 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINSD               (13 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINSE               (14 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINSF               (15 << PINCONF_PINS_SHIFT)
#  define PINCONF_PINSG               (16 << PINCONF_PINS_SHIFT) /* Reserved */
#  define PINCONF_PINSH               (17 << PINCONF_PINS_SHIFT) /* Reserved */
#  define PINCONF_PINSI               (18 << PINCONF_PINS_SHIFT) /* Reserved */
#  define PINCONF_PINSJ               (19 << PINCONF_PINS_SHIFT) /* Reserved */
#  define PINCONF_PINSK               (20 << PINCONF_PINS_SHIFT) /* Reserved */
#  define PINCONF_PINSL               (21 << PINCONF_PINS_SHIFT) /* Reserved */
#  define PINCONF_PINSM               (22 << PINCONF_PINS_SHIFT) /* Reserved */
#  define PINCONF_PINSN               (23 << PINCONF_PINS_SHIFT) /* Reserved */
#  define PINCONF_PINSO               (24 << PINCONF_PINS_SHIFT) /* SFSCLK0 - SFSCLK3 */
#  define PINCONF_PINSP               (25 << PINCONF_PINS_SHIFT) /* Reserved */
#  define PINCONF_PINSQ               (26 << PINCONF_PINS_SHIFT) /* Reserved */
#  define PINCONF_PINSR               (27 << PINCONF_PINS_SHIFT) /* Reserved */
#  define PINCONF_PINSS               (28 << PINCONF_PINS_SHIFT) /* Reserved */
#  define PINCONF_PINST               (29 << PINCONF_PINS_SHIFT) /* Reserved */
#  define PINCONF_PINSU               (30 << PINCONF_PINS_SHIFT) /* Reserved */
#  define PINCONF_PINSV               (31 << PINCONF_PINS_SHIFT) /* Reserved */

/* Pin numbers:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .... ...P PPPP
 */

#define PINCONF_PIN_SHIFT             (0)        /* Bits 0-4: Pin number */
#define PINCONF_PIN_MASK              (31 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_0               (0 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_1               (1 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_2               (2 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_3               (3 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_4               (4 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_5               (5 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_6               (6 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_7               (7 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_8               (8 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_9               (9 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_10              (10 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_11              (11 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_12              (12 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_13              (13 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_14              (14 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_15              (15 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_16              (16 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_17              (17 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_18              (18 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_19              (19 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_20              (20 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_21              (21 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_22              (22 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_23              (23 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_24              (24 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_25              (25 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_26              (26 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_27              (27 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_28              (28 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_29              (29 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_30              (30 << PINCONF_PIN_SHIFT)
#  define PINCONF_PIN_31              (31 << PINCONF_PIN_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_pin_config
 *
 * Description:
 *   Configure a pin based on bit-encoded description of the pin.
 *
 * Input Parameters:
 *   20-bit encoded value describing the pin.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

int lpc43_pin_config(uint32_t pinconf);

/****************************************************************************
 * Function:  lpc43_pin_dump
 *
 * Description:
 *   Dump all pin configuration registers associated with the provided pin
 *   configuration
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
int lpc43_pin_dump(uint32_t pinconf, const char *msg);
#else
#  define lpc43_pin_dump(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_LPC43XX_LPC43_PINCONFIG_H */
