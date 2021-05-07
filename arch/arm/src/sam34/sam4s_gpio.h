/****************************************************************************
 * arch/arm/src/sam34/sam4s_gpio.h
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

#ifndef __ARCH_ARM_SRC_SAM34_SAM4S_GPIO_H
#define __ARCH_ARM_SRC_SAM34_SAM4S_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define GPIO_HAVE_PULLDOWN         1
#define GPIO_HAVE_PERIPHCD         1
#define GPIO_HAVE_SCHMITT          1
#undef  GPIO_HAVE_DELAYR

/* Bit-encoded input to sam_configgpio() ************************************/

/* 32-bit Encoding:
 *
 *   MMMC CCCC III. VPPB BBBB
 */

/* Input/Output mode:
 *
 *   MMM. .... .... .... ....
 */

#define GPIO_MODE_SHIFT            (17)        /* Bits 17-19: GPIO mode */
#define GPIO_MODE_MASK             (7 << GPIO_MODE_SHIFT)
#  define GPIO_INPUT               (0 << GPIO_MODE_SHIFT) /* Input */
#  define GPIO_OUTPUT              (1 << GPIO_MODE_SHIFT) /* Output */
#  define GPIO_PERIPHA             (2 << GPIO_MODE_SHIFT) /* Controlled by periph A signal */
#  define GPIO_PERIPHB             (3 << GPIO_MODE_SHIFT) /* Controlled by periph B signal */
#  define GPIO_PERIPHC             (4 << GPIO_MODE_SHIFT) /* Controlled by periph C signal */
#  define GPIO_PERIPHD             (5 << GPIO_MODE_SHIFT) /* Controlled by periph D signal */

/* These bits set the configuration of the pin:
 * NOTE: No definitions for parallel capture mode
 *
 *   ...C CCCC .... .... ....
 */

#define GPIO_CFG_SHIFT             (12)        /* Bits 12-16: GPIO configuration bits */
#define GPIO_CFG_MASK              (31 << GPIO_CFG_SHIFT)
#  define GPIO_CFG_DEFAULT         (0  << GPIO_CFG_SHIFT) /* Default, no attribute */
#  define GPIO_CFG_PULLUP          (1  << GPIO_CFG_SHIFT) /* Bit 12: Internal pull-up */
#  define GPIO_CFG_PULLDOWN        (2  << GPIO_CFG_SHIFT) /* Bit 13: Internal pull-down */
#  define GPIO_CFG_DEGLITCH        (4  << GPIO_CFG_SHIFT) /* Bit 14: Internal glitch filter */
#  define GPIO_CFG_OPENDRAIN       (8  << GPIO_CFG_SHIFT) /* Bit 15: Open drain */
#  define GPIO_CFG_SCHMITT         (16 << GPIO_CFG_SHIFT) /* Bit 16: Schmitt trigger */

/* Additional interrupt modes:
 *
 *   .... .... III. .... ....
 */

#define GPIO_INT_SHIFT             (9)         /* Bits 9-11: GPIO interrupt bits */
#define GPIO_INT_MASK              (7 << GPIO_INT_SHIFT)
#  define _GIO_INT_AIM             (1 << 10)   /* Bit 10: Additional Interrupt modes */
#  define _GPIO_INT_LEVEL          (1 << 9)    /* Bit 9: Level detection interrupt */
#  define _GPIO_INT_EDGE           (0)         /*        (vs. Edge detection interrupt) */
#  define _GPIO_INT_RH             (1 << 8)    /* Bit 8: Rising edge/High level detection interrupt */
#  define _GPIO_INT_FL             (0)         /*        (vs. Falling edge/Low level detection interrupt) */

#  define GPIO_INT_HIGHLEVEL       (_GIO_INT_AIM | _GPIO_INT_LEVEL | _GPIO_INT_RH)
#  define GPIO_INT_LOWLEVEL        (_GIO_INT_AIM | _GPIO_INT_LEVEL | _GPIO_INT_FL)
#  define GPIO_INT_RISING          (_GIO_INT_AIM | _GPIO_INT_EDGE  | _GPIO_INT_RH)
#  define GPIO_INT_FALLING         (_GIO_INT_AIM | _GPIO_INT_EDGE  | _GPIO_INT_FL)
#  define GPIO_INT_BOTHEDGES       (0)

/* If the pin is an GPIO output, then this identifies the initial
 * output value:
 *
 *   .... .... .... V... ....
 */

#define GPIO_OUTPUT_SET            (1 << 7)    /* Bit 7: Initial value of output */
#define GPIO_OUTPUT_CLEAR          (0)

/* This identifies the GPIO port:
 *
 *   .... .... .... .PP. ....
 */

#define GPIO_PORT_SHIFT            (5)         /* Bit 5-6:  Port number */
#define GPIO_PORT_MASK             (3 << GPIO_PORT_SHIFT)
#  define GPIO_PORT_PIOA           (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORT_PIOB           (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORT_PIOC           (2 << GPIO_PORT_SHIFT)

/* This identifies the bit in the port:
 *
 *   .... .... .... ...B BBBB
 */

#define GPIO_PIN_SHIFT             (0)         /* Bits 0-4: GPIO number: 0-31 */
#define GPIO_PIN_MASK              (31 << GPIO_PIN_SHIFT)
#define GPIO_PIN0                  (0  << GPIO_PIN_SHIFT)
#define GPIO_PIN1                  (1  << GPIO_PIN_SHIFT)
#define GPIO_PIN2                  (2  << GPIO_PIN_SHIFT)
#define GPIO_PIN3                  (3  << GPIO_PIN_SHIFT)
#define GPIO_PIN4                  (4  << GPIO_PIN_SHIFT)
#define GPIO_PIN5                  (5  << GPIO_PIN_SHIFT)
#define GPIO_PIN6                  (6  << GPIO_PIN_SHIFT)
#define GPIO_PIN7                  (7  << GPIO_PIN_SHIFT)
#define GPIO_PIN8                  (8  << GPIO_PIN_SHIFT)
#define GPIO_PIN9                  (9  << GPIO_PIN_SHIFT)
#define GPIO_PIN10                 (10 << GPIO_PIN_SHIFT)
#define GPIO_PIN11                 (11 << GPIO_PIN_SHIFT)
#define GPIO_PIN12                 (12 << GPIO_PIN_SHIFT)
#define GPIO_PIN13                 (13 << GPIO_PIN_SHIFT)
#define GPIO_PIN14                 (14 << GPIO_PIN_SHIFT)
#define GPIO_PIN15                 (15 << GPIO_PIN_SHIFT)
#define GPIO_PIN16                 (16 << GPIO_PIN_SHIFT)
#define GPIO_PIN17                 (17 << GPIO_PIN_SHIFT)
#define GPIO_PIN18                 (18 << GPIO_PIN_SHIFT)
#define GPIO_PIN19                 (19 << GPIO_PIN_SHIFT)
#define GPIO_PIN20                 (20 << GPIO_PIN_SHIFT)
#define GPIO_PIN21                 (21 << GPIO_PIN_SHIFT)
#define GPIO_PIN22                 (22 << GPIO_PIN_SHIFT)
#define GPIO_PIN23                 (23 << GPIO_PIN_SHIFT)
#define GPIO_PIN24                 (24 << GPIO_PIN_SHIFT)
#define GPIO_PIN25                 (25 << GPIO_PIN_SHIFT)
#define GPIO_PIN26                 (26 << GPIO_PIN_SHIFT)
#define GPIO_PIN27                 (27 << GPIO_PIN_SHIFT)
#define GPIO_PIN28                 (28 << GPIO_PIN_SHIFT)
#define GPIO_PIN29                 (29 << GPIO_PIN_SHIFT)
#define GPIO_PIN30                 (30 << GPIO_PIN_SHIFT)
#define GPIO_PIN31                 (31 << GPIO_PIN_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Must be big enough to hold the 32-bit encoding */

typedef uint32_t gpio_pinset_t;

/****************************************************************************
 * Inline Functions
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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAM34_SAM4S_GPIO_H */
