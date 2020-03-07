/************************************************************************************
 * arch/arm/src/sama5/sama5d2x_pio.h
 * Parallel Input/Output (PIO) definitions for the SAMA5D2 family
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAMA5D2X_PIO_H
#define __ARCH_ARM_SRC_SAMA5_SAMA5D2X_PIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "hardware/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#if !defined(CONFIG_SAMA5_PIOA_IRQ) && !defined(CONFIG_SAMA5_PIOB_IRQ) && \
    !defined(CONFIG_SAMA5_PIOC_IRQ) && !defined(CONFIG_SAMA5_PIOD_IRQ)
#  undef CONFIG_SAMA5_PIO_IRQ
#endif

#define PIO_HAVE_PULLDOWN         1
#define PIO_HAVE_PERIPHCD         1
#define PIO_HAVE_SCHMITT          1
#define PIO_HAVE_DRIVE            1

#define SAM_NPIO                  4 /* (4) PIOA-D */

/* Bit-encoded input to sam_configpio() ********************************************/

/* 32-bit Encoding:
 *
 *   .... .MMM MM.C CCCC CDDI IISV .PPB BBBB
 */

/* Input/Output mode:
 *
 *   .... .MMM MM.. .... .... .... .... ....
 */

#define PIO_MODE_SHIFT            (22)        /* Bits 22-26: PIO mode */
#define PIO_MODE_MASK             (15 << PIO_MODE_SHIFT)
#  define PIO_INPUT               (0 << PIO_MODE_SHIFT) /* Input */
#  define PIO_OUTPUT              (1 << PIO_MODE_SHIFT) /* Output */
#  define PIO_ANALOG              (2 << PIO_MODE_SHIFT) /* Analog */
#  define PIO_PERIPHA             (3 << PIO_MODE_SHIFT) /* Controlled by periph A signal */
#  define PIO_PERIPHB             (4 << PIO_MODE_SHIFT) /* Controlled by periph B signal */
#  define PIO_PERIPHC             (5 << PIO_MODE_SHIFT) /* Controlled by periph C signal */
#  define PIO_PERIPHD             (6 << PIO_MODE_SHIFT) /* Controlled by periph D signal */
#  define PIO_PERIPHE             (7 << PIO_MODE_SHIFT) /* Controlled by periph E signal */
#  define PIO_PERIPHF             (8 << PIO_MODE_SHIFT) /* Controlled by periph F signal */
#  define PIO_PERIPHG             (9 << PIO_MODE_SHIFT) /* Controlled by periph G signal */

/* These bits set the configuration of the pin:
 * NOTE: No definitions for parallel capture mode
 *
 *   .... .... ...C CCCC C... .... .... ....
 */

#define PIO_CFG_SHIFT             (15)        /* Bits 15-20: PIO configuration bits */
#define PIO_CFG_MASK              (0x3f << PIO_CFG_SHIFT)
#  define PIO_CFG_DEFAULT         (0x00 << PIO_CFG_SHIFT) /* Default, no attribute */
#  define PIO_CFG_PULLUP          (0x01 << PIO_CFG_SHIFT) /* Bit 15: Internal pull-up */
#  define PIO_CFG_PULLDOWN        (0x02 << PIO_CFG_SHIFT) /* Bit 16: Internal pull-down */
#  define PIO_CFG_DEGLITCH        (0x04 << PIO_CFG_SHIFT) /* Bit 17: Internal input filter (Tmck/2)*/
#  define PIO_CFG_SLOWCLK         (0x0c << PIO_CFG_SHIFT) /* Bits 17+18: Internal input filter (Tslwclk/2)*/
#  define PIO_CFG_OPENDRAIN       (0x10 << PIO_CFG_SHIFT) /* Bit 19: Open drain */
#  define PIO_CFG_SCHMITT         (0x20 << PIO_CFG_SHIFT) /* Bit 20: Schmitt trigger */

/* Drive Strength:
 *
 *   .... .... .... .... .DD. .... .... ....
 */

#define PIO_DRIVE_SHIFT           (13)        /* Bits 13-14: Drive strength */
#define PIO_DRIVE_MASK            (7 << PIO_DRIVE_SHIFT)
#  define PIO_DRIVE_LOW           (0 << PIO_DRIVE_SHIFT)
#  define PIO_DRIVE_MEDIUM        (2 << PIO_DRIVE_SHIFT)
#  define PIO_DRIVE_HIGH          (3 << PIO_DRIVE_SHIFT)

/* Additional interrupt modes:
 *
 *   .... .... .... .... ...I II.. .... ....
 */

#define PIO_INT_SHIFT             (10)        /* Bits 9-12: PIO interrupt bits */
#define PIO_INT_MASK              (7 << PIO_INT_SHIFT)
#  define PIO_INT_NONE            (0 << PIO_INT_SHIFT)
#  define PIO_INT_FALLING         (1 << PIO_INT_SHIFT)
#  define PIO_INT_RISING          (2 << PIO_INT_SHIFT)
#  define PIO_INT_BOTHEDGES       (3 << PIO_INT_SHIFT)
#  define PIO_INT_LOWLEVEL        (4 << PIO_INT_SHIFT)
#  define PIO_INT_HIGHLEVEL       (5 << PIO_INT_SHIFT)

/* If the pin is an interrupt, then this determines if the pin is a secure:
 *
 *   .... .... .... .... .... ..S. .... ....
 */

#define PIO_INT_SECURE            (1 << 9)    /* Bit 9: Secure PIO */
#define PIO_INT_UNSECURE          (0)

/* If the pin is an PIO output, then this identifies the initial output value:
 *
 *   .... .... .... .... .... ...V .... ....
 */

#define PIO_OUTPUT_SET            (1 << 8)    /* Bit 8: Initial value of output */
#define PIO_OUTPUT_CLEAR          (0)

/* This identifies the PIO port:
 *
 *   .... .... .... .... .... .... .PP. ....
 */

#define PIO_PORT_SHIFT            (5)         /* Bit 5-6:  Port number */
#define PIO_PORT_MASK             (3 << PIO_PORT_SHIFT)
#  define PIO_PORT_PIOA           (0 << PIO_PORT_SHIFT)
#  define PIO_PORT_PIOB           (1 << PIO_PORT_SHIFT)
#  define PIO_PORT_PIOC           (2 << PIO_PORT_SHIFT)
#  define PIO_PORT_PIOD           (3 << PIO_PORT_SHIFT)

/* This identifies the bit in the port:
 *
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

#endif /* __ARCH_ARM_SRC_SAMA5_SAMA5D2X_PIO_H */
