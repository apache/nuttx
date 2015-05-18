/****************************************************************************
 * arch/arm/src/samdl/sam_port.h
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMDL_SAM_PORT_H
#define __ARCH_ARM_SRC_SAMDL_SAM_PORT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#include <nuttx/irq.h>

#include "chip.h"

#if defined(CONFIG_ARCH_FAMILY_SAMD20)
#  include "chip/samd_port.h"
#elif defined(CONFIG_ARCH_FAMILY_SAML21)
#  include "chip/saml_port.h"
#else
#  error Unrecognized SAMD/L architecture
#endif

/****************************************************************************
 * Pre-processor Declarations
 ****************************************************************************/

/* Bit-encoded input to sam_configport() */

/* 24-bit Encoding.  This could be compacted into 16-bits by making the bit usage
 * mode specific.  However, by giving each bit field a unique position, we handle
 * bad combinations of properties safely.
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   PORT Input:  MMRR .... .S.. .... ..PB BBBB
 *   PORT Output: MM.. .... D..V .... ..PB BBBB
 *   Peripheral:  MM.. FFF. .... II.. ..PB BBBB
 *   ------------ -----------------------------
 *                MMRR FFF. DS.V II.. ..PB BBBB
 */

/* Input/output/peripheral mode:
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   PORT Input:  MM.. .... .... .... .... ....
 *   PORT Output: MM.. .... .... .... .... ....
 *   Peripheral:  MM.. .... .... .... .... ....
 */

#define PORT_MODE_SHIFT            (22)        /* Bits 22-23: PORT mode */
#define PORT_MODE_MASK             (3 << PORT_MODE_SHIFT)
#  define PORT_INPUT               (0 << PORT_MODE_SHIFT) /* PORT Input */
#  define PORT_OUTPUT              (1 << PORT_MODE_SHIFT) /* PORT Output */
#  define PORT_PERIPHERAL          (2 << PORT_MODE_SHIFT) /* Controlled by peripheral */
#  define PORT_INTERRUPT           (3 << PORT_MODE_SHIFT) /* Interrupting input */

/* Pull-up/down resistor control for inputs
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   PORT Input:  ..RR .... .... .... .... ....
 *   PORT Output: ..RR .... .... .... .... ....
 *   Peripheral:  .... .... .... .... .... ....
 */

#define PORT_PULL_SHIFT            (20)       /* Bits 20-21: Pull-up/down resistor control */
#define PORT_PULL_MASK             (3 << PORT_PULL_SHIFT)
#  define PORT_PULL_NONE           (0 << PORT_PULL_SHIFT)
#  define PORT_PULL_UP             (1 << PORT_PULL_SHIFT)
#  define PORT_PULL_DOWN           (2 << PORT_PULL_SHIFT)

/* Peripheral Function
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   PORT Input:  .... .... .... .... .... ....
 *   PORT Output: .... .... .... .... .... ....
 *   Peripheral:  .... FFF. .... .... .... ....
 */

#define PORT_FUNC_SHIFT            (17)       /* Bits 17-19: Peripheral function */
#define PORT_FUNC_MASK             (7 << PORT_FUNC_SHIFT)
#  define _PORT_FUNCA              (0 << PORT_FUNC_SHIFT) /* Function A */
#  define _PORT_FUNCB              (1 << PORT_FUNC_SHIFT) /* Function B */
#  define _PORT_FUNCC              (2 << PORT_FUNC_SHIFT) /* Function C */
#  define _PORT_FUNCD              (3 << PORT_FUNC_SHIFT) /* Function D */
#  define _PORT_FUNCE              (4 << PORT_FUNC_SHIFT) /* Function E */
#  define _PORT_FUNCF              (5 << PORT_FUNC_SHIFT) /* Function F */
#  define _PORT_FUNCG              (6 << PORT_FUNC_SHIFT) /* Function G */
#  define _PORT_FUNCH              (7 << PORT_FUNC_SHIFT) /* Function H */

/* Extended input/output/peripheral mode:
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   PORT Input:  .... .... .... .... .... ....
 *   PORT Output: .... .... .... .... .... ....
 *   Peripheral:  MM.. FFF. .... .... .... ....
 */

#define PORT_FUNCA                 (PORT_PERIPHERAL | _PORT_FUNCA) /* Function A */
#define PORT_FUNCB                 (PORT_PERIPHERAL | _PORT_FUNCB) /* Function B */
#define PORT_FUNCC                 (PORT_PERIPHERAL | _PORT_FUNCC) /* Function C */
#define PORT_FUNCD                 (PORT_PERIPHERAL | _PORT_FUNCD) /* Function D */
#define PORT_FUNCE                 (PORT_PERIPHERAL | _PORT_FUNCE) /* Function E */
#define PORT_FUNCF                 (PORT_PERIPHERAL | _PORT_FUNCF) /* Function F */
#define PORT_FUNCG                 (PORT_PERIPHERAL | _PORT_FUNCG) /* Function G */
#define PORT_FUNCH                 (PORT_PERIPHERAL | _PORT_FUNCH) /* Function H */

/* Output drive control
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   PORT Input:  .... .... .... .... .... ....
 *   PORT Output: .... .... D... .... .... ....
 *   Peripheral:  .... .... .... .... .... ....
 */

#define PORT_DRIVE_SHIFT           (15)       /* Bit 15: Interrupting input control */
#define PORT_DRIVE_MASK            (1 << PORT_INT_SHIFT)
#  define PORT_DRIVE_LOW           (0 << PORT_INT_SHIFT)
#  define PORT_DRIVE_HIGH          (1 << PORT_INT_SHIFT)

/* Input sampling
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   PORT Input:  .... .... .S.. .... .... ....
 *   PORT Output: .... .... .... .... .... ....
 *   Peripheral:  .... .... .... .... .... ....
 */

#define PORT_SYNCHRONIZER_SHIFT    (14)       /* Bit 14: Input synchronizer input control */
#define PORT_SYNCHRONIZER_MASK     (1 << PORT_SYNCHRONIZER_SHIFT)
#  define PORT_SYNCHRONIZER_OFF    (0 << PORT_SYNCHRONIZER_SHIFT)
#  define PORT_SYNCHRONIZER_ON     (1 << PORT_SYNCHRONIZER_SHIFT)

/* If the pin is an PORT output, then this identifies the initial output value:
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   PORT Input:  .... .... .... .... .... ....
 *   PORT Output: .... .... ...V .... .... ....
 *   Peripheral:  .... .... .... .... .... ....
 */

#define PORT_OUTVALUE_SHIFT      (12)       /* Bit 12: Initial value of output */
#define PORT_OUTVALUE_MASK       (1 << PORT_SYNCHRONIZER_SHIFT)
#  define PORT_OUTPUT_CLEAR      (0 << PORT_SYNCHRONIZER_SHIFT)
#  define PORT_OUTPUT_SET        (1 << PORT_SYNCHRONIZER_SHIFT)

/* Selections for external interrupts:
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   PORT Input:  .... .... .... .... .... ....
 *   PORT Output: .... .... .... .... .... ....
 *   Peripheral:  .... .... .... II.. .... ....
 */

#define PORT_INT_SHIFT             (10)       /* Bits 10-11: Interrupting input control */
#define PORT_INT_MASK              (3 << PORT_INT_SHIFT)
#  define PORT_INT_CHANGE          (0 << PORT_INT_SHIFT) /* Pin change */
#  define PORT_INT_RISING          (1 << PORT_INT_SHIFT) /* Rising edge */
#  define PORT_INT_FALLING         (2 << PORT_INT_SHIFT) /* Falling edge */

/* This identifies the PORT port:
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   PORT Input:  .... .... .... .... ..P. ....
 *   PORT Output: .... .... .... .... ..P. ....
 *   Peripheral:  .... .... .... .... ..P. ....
 */

#define PORT_SHIFT                 (5)         /* Bit 5:  Port number */
#define PORT_MASK                  (1 << PORT_SHIFT)
#  define PORTA                    (SAM_PORTA << PORT_SHIFT)
#  define PORTB                    (SAM_PORTB << PORT_SHIFT)

/* This identifies the bit in the port:
 *
 *   MODE         BITFIELDS
 *   ------------ -----------------------------
 *                2222 1111 1111 1100 0000 0000
 *                3210 9876 5432 1098 7654 3210
 *   ------------ -----------------------------
 *   PORT Input:  .... .... .... .... ...B BBBB
 *   PORT Output: .... .... .... .... ...B BBBB
 *   Peripheral:  .... .... .... .... ...B BBBB
 */

#define PORT_PIN_SHIFT             0        /* Bits 0-4: PORT number: 0-31 */
#define PORT_PIN_MASK              (31 << PORT_PIN_SHIFT)
#define PORT_PIN0                  (0  << PORT_PIN_SHIFT)
#define PORT_PIN1                  (1  << PORT_PIN_SHIFT)
#define PORT_PIN2                  (2  << PORT_PIN_SHIFT)
#define PORT_PIN3                  (3  << PORT_PIN_SHIFT)
#define PORT_PIN4                  (4  << PORT_PIN_SHIFT)
#define PORT_PIN5                  (5  << PORT_PIN_SHIFT)
#define PORT_PIN6                  (6  << PORT_PIN_SHIFT)
#define PORT_PIN7                  (7  << PORT_PIN_SHIFT)
#define PORT_PIN8                  (8  << PORT_PIN_SHIFT)
#define PORT_PIN9                  (9  << PORT_PIN_SHIFT)
#define PORT_PIN10                 (10 << PORT_PIN_SHIFT)
#define PORT_PIN11                 (11 << PORT_PIN_SHIFT)
#define PORT_PIN12                 (12 << PORT_PIN_SHIFT)
#define PORT_PIN13                 (13 << PORT_PIN_SHIFT)
#define PORT_PIN14                 (14 << PORT_PIN_SHIFT)
#define PORT_PIN15                 (15 << PORT_PIN_SHIFT)
#define PORT_PIN16                 (16 << PORT_PIN_SHIFT)
#define PORT_PIN17                 (17 << PORT_PIN_SHIFT)
#define PORT_PIN18                 (18 << PORT_PIN_SHIFT)
#define PORT_PIN19                 (19 << PORT_PIN_SHIFT)
#define PORT_PIN20                 (20 << PORT_PIN_SHIFT)
#define PORT_PIN21                 (21 << PORT_PIN_SHIFT)
#define PORT_PIN22                 (22 << PORT_PIN_SHIFT)
#define PORT_PIN23                 (23 << PORT_PIN_SHIFT)
#define PORT_PIN24                 (24 << PORT_PIN_SHIFT)
#define PORT_PIN25                 (25 << PORT_PIN_SHIFT)
#define PORT_PIN26                 (26 << PORT_PIN_SHIFT)
#define PORT_PIN27                 (27 << PORT_PIN_SHIFT)
#define PORT_PIN28                 (28 << PORT_PIN_SHIFT)
#define PORT_PIN29                 (29 << PORT_PIN_SHIFT)
#define PORT_PIN30                 (30 << PORT_PIN_SHIFT)
#define PORT_PIN31                 (31 << PORT_PIN_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef uint32_t port_pinset_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

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
 * Name: sam_configport
 *
 * Description:
 *   Configure a PORT pin based on bit-encoded description of the pin.
 *
 * Returns:
 *   OK (always)
 *
 ****************************************************************************/

int sam_configport(port_pinset_t pinset);

/****************************************************************************
 * Name: sam_portwrite
 *
 * Description:
 *   Write one or zero to the selected PORT pin
 *
 ****************************************************************************/

void sam_portwrite(port_pinset_t pinset, bool value);

/****************************************************************************
 * Name: sam_portread
 *
 * Description:
 *   Read one or zero from the selected PORT pin
 *
 ****************************************************************************/

bool sam_portread(port_pinset_t pinset);

/****************************************************************************
 * Function:  sam_dumpport
 *
 * Description:
 *   Dump all PORT registers associated with the provided pin description
 *   along with a descriptive message.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG
void sam_dumpport(port_pinset_t pinset, const char *msg);
#else
#  define sam_dumpport(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAMDL_SAM_PORT_H */
