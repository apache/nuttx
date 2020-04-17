/************************************************************************************
 * arch/arm/src/a1x/a1x_pio.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_A1X_A1X_PIO_H
#define __ARCH_ARM_SRC_A1X_A1X_PIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "hardware/a1x_pio.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Bit-encoded input to a1x_pio_config() ********************************************/

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
#  define PIO_PERIPH0             (PIO_REG_CFG_INPUT << PIO_MODE_SHIFT)
#  define PIO_PERIPH1             (PIO_REG_CFG_OUTPUT << PIO_MODE_SHIFT)
#  define PIO_PERIPH2             (2 << PIO_MODE_SHIFT)
#  define PIO_PERIPH3             (3 << PIO_MODE_SHIFT)
#  define PIO_PERIPH4             (4 << PIO_MODE_SHIFT)
#  define PIO_PERIPH5             (5 << PIO_MODE_SHIFT)
#  define PIO_PERIPH6             (6 << PIO_MODE_SHIFT)
#  define PIO_PERIPH7             (7 << PIO_MODE_SHIFT)

#  define PIO_INPUT               PIO_PERIPH0 /* Input */
#  define PIO_OUTPUT              PIO_PERIPH1 /* Output */

/* Bit 20 also specifies an external interrupt which must go with ID=6 */

#define PIO_EINT_BIT              (1 << 20)   /* Bit 20: External PIO interrupt */
#define PIO_EINT_SHIFT            (20)        /* Bits 20-23: Extended PIO mode */
#define PIO_EINT_MASK             (15 << PIO_EINT_SHIFT)
#  define PIO_EINT                (PIO_EINT_BIT | PIO_PERIPH6)

/* These bits set the pull-up/down configuration of the pin:
 *
 *   3322 2222 2222 1111 1111 11
 *   1098 7654 3210 9876 5432 1098 7654 3210
 *   ---- ---- ---- ---- ---- ---- ---- ----
 *   .... .... .... PP.. .... .... .... ....
 */

#define PIO_PULL_SHIFT            (18)        /* Bits 18-19: PIO configuration bits */
#define PIO_PULL_MASK             (3 << PIO_PULL_SHIFT)
#  define PIO_PULL_NONE           (PIO_REG_PULL_NONE  << PIO_PULL_SHIFT)
#  define PIO_PULL_PULLUP         (PIO_REG_PULL_UP  << PIO_PULL_SHIFT)
#  define PIO_PULL_PULLDOWN       (PIO_REG_PULL_DOWN  << PIO_PULL_SHIFT)

/* Drive (outputs only):
 *
 *   3322 2222 2222 1111 1111 11
 *   1098 7654 3210 9876 5432 1098 7654 3210
 *   ---- ---- ---- ---- ---- ---- ---- ----
 *   .... .... .... ..DD .... .... .... ....
 */

#define PIO_DRIVE_SHIFT           (16)        /* Bits 16-17: Drive strength */
#define PIO_DRIVE_MASK            (3 << PIO_DRIVE_SHIFT)
#  define PIO_DRIVE_NONE          (0 << PIO_DRIVE_SHIFT)
#  define PIO_DRIVE_LOW           (PIO_REG_DRV_LEVEL0 << PIO_DRIVE_SHIFT)
#  define PIO_DRIVE_MEDLOW        (PIO_REG_DRV_LEVEL1 << PIO_DRIVE_SHIFT)
#  define PIO_DRIVE_MEDHIGH       (PIO_REG_DRV_LEVEL2 << PIO_DRIVE_SHIFT)
#  define PIO_DRIVE_HIGH          (PIO_REG_DRV_LEVEL3 << PIO_DRIVE_SHIFT)

/* Interrupt modes (inputs only):
 *
 *   3322 2222 2222 1111 1111 11
 *   1098 7654 3210 9876 5432 1098 7654 3210
 *   ---- ---- ---- ---- ---- ---- ---- ----
 *   .... .... ... ....  III. .... .... ....
 */

#define PIO_INT_SHIFT             (13)        /* Bits 13-15: PIO interrupt bits */
#define PIO_INT_MASK              (7 << PIO_INT_SHIFT)
#  define PIO_INT_NONE            (0 << PIO_INT_SHIFT)
#  define PIO_INT_RISING          (PIO_REG_INT_POSEDGE << PIO_INT_SHIFT)
#  define PIO_INT_FALLING         (PIO_REG_INT_NEGEDGE << PIO_INT_SHIFT)
#  define PIO_INT_HIGHLEVEL       (PIO_REG_INT_HILEVEL << PIO_INT_SHIFT)
#  define PIO_INT_LOWLEVEL        (PIO_REG_INT_LOWLEVEL << PIO_INT_SHIFT)
#  define PIO_INT_BOTHEDGES       (PIO_REG_INT_BOTHEDGES << PIO_INT_SHIFT)

/* If the pin is an PIO output, then this identifies the initial output value:
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
#  define PIO_PORT_PIOA           (PIO_REG_PORTA << PIO_PORT_SHIFT)
#  define PIO_PORT_PIOB           (PIO_REG_PORTB << PIO_PORT_SHIFT)
#  define PIO_PORT_PIOC           (PIO_REG_PORTC << PIO_PORT_SHIFT)
#  define PIO_PORT_PIOD           (PIO_REG_PORTD << PIO_PORT_SHIFT)
#  define PIO_PORT_PIOE           (PIO_REG_PORTE << PIO_PORT_SHIFT)
#  define PIO_PORT_PIOF           (PIO_REG_PORTF << PIO_PORT_SHIFT)
#  define PIO_PORT_PIOG           (PIO_REG_PORTG << PIO_PORT_SHIFT)
#  define PIO_PORT_PIOH           (PIO_REG_PORTH << PIO_PORT_SHIFT)
#  define PIO_PORT_PIOI           (PIO_REG_PORTI << PIO_PORT_SHIFT)

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

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* Must be big enough to hold the 32-bit encoding */

typedef uint32_t pio_pinset_t;

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: a1x_pio_irqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for PIO pins.
 *
 ************************************************************************************/

#ifdef CONFIG_A1X_PIO_IRQ
void a1x_pio_irqinitialize(void);
#else
#  define a1x_pio_irqinitialize()
#endif

/************************************************************************************
 * Name: a1x_pio_config
 *
 * Description:
 *   Configure a PIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

int a1x_pio_config(pio_pinset_t cfgset);

/************************************************************************************
 * Name: a1x_pio_write
 *
 * Description:
 *   Write one or zero to the selected PIO pin
 *
 ************************************************************************************/

void a1x_pio_write(pio_pinset_t pinset, bool value);

/************************************************************************************
 * Name: a1x_pio_read
 *
 * Description:
 *   Read one or zero from the selected PIO pin
 *
 ************************************************************************************/

bool a1x_pio_read(pio_pinset_t pinset);

/************************************************************************************
 * Name: a1x_pio_irqenable
 *
 * Description:
 *   Enable the interrupt for specified PIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_A1X_PIO_IRQ
void a1x_pio_irqenable(int irq);
#else
#  define a1x_pio_irqenable(irq)
#endif

/************************************************************************************
 * Name: a1x_pio_irqdisable
 *
 * Description:
 *   Disable the interrupt for specified PIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_A1X_PIO_IRQ
void a1x_pio_irqdisable(int irq);
#else
#  define a1x_pio_irqdisable(irq)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_A1X_A1X_PIO_H */
