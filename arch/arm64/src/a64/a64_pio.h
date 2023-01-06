/****************************************************************************
 * arch/arm64/src/a64/a64_pio.h
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

#ifndef __ARCH_ARM64_SRC_A64_A64_PIO_H
#define __ARCH_ARM64_SRC_A64_A64_PIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "hardware/a64_pio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bit-encoded input to a64_pio_config() ************************************/

/* 32-bit Encoding:
 *
 *   3322 2222 2222 1111 1111 11
 *   1098 7654 3210 9876 5432 1098 7654 3210
 *   ---- ---- ---- ---- ---- ---- ---- ----
 *   .... .... MMMM PPDD IIIV ...P PPPB BBBB
 */

/* Input/Output mode:
 *
 *   3322 2222 2222 1111 1111 11
 *   1098 7654 3210 9876 5432 1098 7654 3210
 *   ---- ---- ---- ---- ---- ---- ---- ----
 *   .... .... MMMX .... .... .... .... ....
 */

#define PIO_MODE_SHIFT            (21)        /* Bits 21-23: PIO mode */
#define PIO_MODE_MASK             (7 << PIO_MODE_SHIFT)
#define PIO_PERIPH0               (PIO_REG_CFG_INPUT << PIO_MODE_SHIFT)
#define PIO_PERIPH1               (PIO_REG_CFG_OUTPUT << PIO_MODE_SHIFT)
#define PIO_PERIPH2               (2 << PIO_MODE_SHIFT)
#define PIO_PERIPH3               (3 << PIO_MODE_SHIFT)
#define PIO_PERIPH4               (4 << PIO_MODE_SHIFT)
#define PIO_PERIPH5               (5 << PIO_MODE_SHIFT)
#define PIO_PERIPH6               (6 << PIO_MODE_SHIFT)
#define PIO_PERIPH7               (7 << PIO_MODE_SHIFT)

#define PIO_INPUT                 PIO_PERIPH0 /* Input */
#define PIO_OUTPUT                PIO_PERIPH1 /* Output */
#define PIO_PWM                   PIO_PERIPH2 /* PWM for PL10 */

/* Bit 20 also specifies an external interrupt which must go with ID=6 */

#define PIO_EINT_BIT              (1 << 20)   /* Bit 20: External PIO interrupt */
#define PIO_EINT_SHIFT            (20)        /* Bits 20-23: Extended PIO mode */
#define PIO_EINT_MASK             (15 << PIO_EINT_SHIFT)
#define PIO_EINT                  (PIO_EINT_BIT | PIO_PERIPH6)

/* These bits set the pull-up/down configuration of the pin:
 *
 *   3322 2222 2222 1111 1111 11
 *   1098 7654 3210 9876 5432 1098 7654 3210
 *   ---- ---- ---- ---- ---- ---- ---- ----
 *   .... .... .... PP.. .... .... .... ....
 */

#define PIO_PULL_SHIFT            (18)        /* Bits 18-19: PIO configuration bits */
#define PIO_PULL_MASK             (3 << PIO_PULL_SHIFT)
#define PIO_PULL_NONE             (PIO_REG_PULL_NONE  << PIO_PULL_SHIFT)
#define PIO_PULL_PULLUP           (PIO_REG_PULL_UP  << PIO_PULL_SHIFT)
#define PIO_PULL_PULLDOWN         (PIO_REG_PULL_DOWN  << PIO_PULL_SHIFT)

/* Drive (outputs only):
 *
 *   3322 2222 2222 1111 1111 11
 *   1098 7654 3210 9876 5432 1098 7654 3210
 *   ---- ---- ---- ---- ---- ---- ---- ----
 *   .... .... .... ..DD .... .... .... ....
 */

#define PIO_DRIVE_SHIFT           (16)        /* Bits 16-17: Drive strength */
#define PIO_DRIVE_MASK            (3 << PIO_DRIVE_SHIFT)
#define PIO_DRIVE_NONE            (0 << PIO_DRIVE_SHIFT)
#define PIO_DRIVE_LOW             (PIO_REG_DRV_LEVEL0 << PIO_DRIVE_SHIFT)
#define PIO_DRIVE_MEDLOW          (PIO_REG_DRV_LEVEL1 << PIO_DRIVE_SHIFT)
#define PIO_DRIVE_MEDHIGH         (PIO_REG_DRV_LEVEL2 << PIO_DRIVE_SHIFT)
#define PIO_DRIVE_HIGH            (PIO_REG_DRV_LEVEL3 << PIO_DRIVE_SHIFT)

/* Interrupt modes (inputs only):
 *
 *   3322 2222 2222 1111 1111 11
 *   1098 7654 3210 9876 5432 1098 7654 3210
 *   ---- ---- ---- ---- ---- ---- ---- ----
 *   .... .... ... ....  III. .... .... ....
 */

#define PIO_INT_SHIFT             (13)        /* Bits 13-15: PIO interrupt bits */
#define PIO_INT_MASK              (7 << PIO_INT_SHIFT)
#define PIO_INT_NONE              (0 << PIO_INT_SHIFT)
#define PIO_INT_RISING            (PIO_REG_INT_POSEDGE << PIO_INT_SHIFT)
#define PIO_INT_FALLING           (PIO_REG_INT_NEGEDGE << PIO_INT_SHIFT)
#define PIO_INT_HIGHLEVEL         (PIO_REG_INT_HILEVEL << PIO_INT_SHIFT)
#define PIO_INT_LOWLEVEL          (PIO_REG_INT_LOWLEVEL << PIO_INT_SHIFT)
#define PIO_INT_BOTHEDGES         (PIO_REG_INT_BOTHEDGES << PIO_INT_SHIFT)

/* If the pin is an PIO output, then this identifies the initial
 * output value:
 *
 *   3322 2222 2222 1111 1111 11
 *   1098 7654 3210 9876 5432 1098 7654 3210
 *   ---- ---- ---- ---- ---- ---- ---- ----
 *   .... .... .... .... ...V .... .... ....
 *   V
 */

#define PIO_OUTPUT_SET            (1 << 12)   /* Bit 12: Initial value of output */
#define PIO_OUTPUT_CLEAR          (0)

/* This identifies the PIO port:
 *
 *   3322 2222 2222 1111 1111 11
 *   1098 7654 3210 9876 5432 1098 7654 3210
 *   ---- ---- ---- ---- ---- ---- ---- ----
 *   .... .... .... .... .... ...P PPP. ....
 *   PPPP
 */

#define PIO_PORT_SHIFT            (5)         /* Bit 5-8:  Port number */
#define PIO_PORT_MASK             (15 << PIO_PORT_SHIFT)
#define PIO_PORT_PIOB             (PIO_REG_PORTB << PIO_PORT_SHIFT)
#define PIO_PORT_PIOC             (PIO_REG_PORTC << PIO_PORT_SHIFT)
#define PIO_PORT_PIOD             (PIO_REG_PORTD << PIO_PORT_SHIFT)
#define PIO_PORT_PIOE             (PIO_REG_PORTE << PIO_PORT_SHIFT)
#define PIO_PORT_PIOF             (PIO_REG_PORTF << PIO_PORT_SHIFT)
#define PIO_PORT_PIOG             (PIO_REG_PORTG << PIO_PORT_SHIFT)
#define PIO_PORT_PIOH             (PIO_REG_PORTH << PIO_PORT_SHIFT)
#define PIO_PORT_PIOL             (PIO_REG_PORTL << PIO_PORT_SHIFT)

/* This identifies the bit in the port:
 *
 *   3322 2222 2222 1111 1111 11
 *   1098 7654 3210 9876 5432 1098 7654 3210
 *   ---- ---- ---- ---- ---- ---- ---- ----
 *   .... .... .... .... .... .... ...B BBBB
 */

#define PIO_PIN_SHIFT             (0)         /* Bits 0-4: PIO number: 0-31 */
#define PIO_PIN_MASK              (31 << PIO_PIN_SHIFT)
#define PIO_PIN0                  (0  << PIO_PIN_SHIFT)
#define PIO_PIN1                  (1  << PIO_PIN_SHIFT)
#define PIO_PIN2                  (2  << PIO_PIN_SHIFT)
#define PIO_PIN3                  (3  << PIO_PIN_SHIFT)
#define PIO_PIN4                  (4  << PIO_PIN_SHIFT)
#define PIO_PIN5                  (5  << PIO_PIN_SHIFT)
#define PIO_PIN6                  (6  << PIO_PIN_SHIFT)
#define PIO_PIN7                  (7  << PIO_PIN_SHIFT)
#define PIO_PIN8                  (8  << PIO_PIN_SHIFT)
#define PIO_PIN9                  (9  << PIO_PIN_SHIFT)
#define PIO_PIN10                 (10 << PIO_PIN_SHIFT)
#define PIO_PIN11                 (11 << PIO_PIN_SHIFT)
#define PIO_PIN12                 (12 << PIO_PIN_SHIFT)
#define PIO_PIN13                 (13 << PIO_PIN_SHIFT)
#define PIO_PIN14                 (14 << PIO_PIN_SHIFT)
#define PIO_PIN15                 (15 << PIO_PIN_SHIFT)
#define PIO_PIN16                 (16 << PIO_PIN_SHIFT)
#define PIO_PIN17                 (17 << PIO_PIN_SHIFT)
#define PIO_PIN18                 (18 << PIO_PIN_SHIFT)
#define PIO_PIN19                 (19 << PIO_PIN_SHIFT)
#define PIO_PIN20                 (20 << PIO_PIN_SHIFT)
#define PIO_PIN21                 (21 << PIO_PIN_SHIFT)
#define PIO_PIN22                 (22 << PIO_PIN_SHIFT)
#define PIO_PIN23                 (23 << PIO_PIN_SHIFT)
#define PIO_PIN24                 (24 << PIO_PIN_SHIFT)
#define PIO_PIN25                 (25 << PIO_PIN_SHIFT)
#define PIO_PIN26                 (26 << PIO_PIN_SHIFT)
#define PIO_PIN27                 (27 << PIO_PIN_SHIFT)
#define PIO_PIN28                 (28 << PIO_PIN_SHIFT)
#define PIO_PIN29                 (29 << PIO_PIN_SHIFT)
#define PIO_PIN30                 (30 << PIO_PIN_SHIFT)
#define PIO_PIN31                 (31 << PIO_PIN_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Must be big enough to hold the 32-bit encoding */

typedef uint32_t pio_pinset_t;

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

/****************************************************************************
 * Name: a64_pio_config
 *
 * Description:
 *   Configure a PIO pin based on bit-encoded description of the pin.
 *
 * Input Parameters:
 *   cfgset - Bit-encoded description of a pin
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int a64_pio_config(pio_pinset_t cfgset);

/****************************************************************************
 * Name: a64_piowrite
 *
 * Description:
 *   Write one or zero to the selected PIO pin.
 *
 * Input Parameters:
 *   pinset - PIO pin
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void a64_pio_write(pio_pinset_t pinset, bool value);

/****************************************************************************
 * Name: a64_pio_read
 *
 * Description:
 *   Read one or zero from the selected PIO pin
 *
 * Input Parameters:
 *   pinset - PIO pin
 *
 * Returned Value:
 *   Input value of PIO pin
 *
 ****************************************************************************/

bool a64_pio_read(pio_pinset_t pinset);

/****************************************************************************
 * Name: a64_pio_irqenable
 *
 * Description:
 *   Enable the interrupt for specified PIO pin.  Only Ports B, G and H are
 *   supported for interrupts.
 *
 * Input Parameters:
 *   pinset - Bit-encoded description of a pin. Port should be B, G or H.
 *
 * Returned Value:
 *   Zero (OK) on success; -EINVAL if pin is not from Port B, G or H.
 *
 ****************************************************************************/

int a64_pio_irqenable(pio_pinset_t pinset);

/****************************************************************************
 * Name: a64_pio_irqdisable
 *
 * Description:
 *   Disable the interrupt for specified PIO pin.  Only Ports B, G and H are
 *   supported for interrupts.
 *
 * Input Parameters:
 *   pinset - Bit-encoded description of a pin. Port should be B, G or H.
 *
 * Returned Value:
 *   Zero (OK) on success; -EINVAL if pin is not from Port B, G or H.
 *
 ****************************************************************************/

int a64_pio_irqdisable(pio_pinset_t pinset);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_A64_A64_PIO_H */
