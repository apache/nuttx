/****************************************************************************
 * arch/mips/src/pic32mz/hardware/pic32mz_osc.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_OSC_H
#define __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_OSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "pic32mz_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define PIC32MZ_OSCCON_OFFSET     0x0000 /* Oscillator control register offset */
#define PIC32MZ_OSCTUN_OFFSET     0x0010 /* FRC tuning register offset */
#define PIC32MZ_SPLLCON_OFFSET    0x0020 /* System PLL control register */
#define PIC32MZ_REFO1CON_OFFSET   0x0080 /* Reference oscillator control register 1 */
#define PIC32MZ_REFO1TRIM_OFFSET  0x00a0 /* Reference oscillator trim register 1 */
#define PIC32MZ_REFO2CON_OFFSET   0x00b0 /* Reference oscillator control register 2 */
#define PIC32MZ_REFO2TRIM_OFFSET  0x00c0 /* Reference oscillator trim register 2 */
#define PIC32MZ_REFO3CON_OFFSET   0x00d0 /* Reference oscillator control register 3 */
#define PIC32MZ_REFO3TRIM_OFFSET  0x00e0 /* Reference oscillator trim register 3 */
#define PIC32MZ_REFO4CON_OFFSET   0x00f0 /* Reference oscillator control register 4 */
#define PIC32MZ_REFO4TRIM_OFFSET  0x0100 /* Reference oscillator trim register 4 */
#define PIC32MZ_PB1DIV_OFFSET     0x0100 /* Peripheral bus 1 clock divisor control register */
#define PIC32MZ_PB2DIV_OFFSET     0x0110 /* Peripheral bus 2 clock divisor control register */
#define PIC32MZ_PB3DIV_OFFSET     0x0120 /* Peripheral bus 3 clock divisor control register */
#define PIC32MZ_PB4DIV_OFFSET     0x0130 /* Peripheral bus 4 clock divisor control register */
#define PIC32MZ_PB5DIV_OFFSET     0x0140 /* Peripheral bus 5 clock divisor control register */
#define PIC32MZ_PB6DIV_OFFSET     0x0150 /* Peripheral bus 6 clock divisor control register */
#define PIC32MZ_PB7DIV_OFFSET     0x0160 /* Peripheral bus 7 clock divisor control register */
#define PIC32MZ_PB8DIV_OFFSET     0x0170 /* Peripheral bus 8 clock divisor control register */

/* Register Addresses *******************************************************/

#define PIC32MZ_OSCCON            (PIC32MZ_OSC_K1BASE+PIC32MZ_OSCCON_OFFSET)
#define PIC32MZ_OSCTUN            (PIC32MZ_OSC_K1BASE+PIC32MZ_OSCTUN_OFFSET)

#define PIC32MZ_SPLLCON           (PIC32MZ_OSC_K1BASE+PIC32MZ_SPLLCON_OFFSET)
#define PIC32MZ_REFO1CON          (PIC32MZ_OSC_K1BASE+PIC32MZ_REFO1CON_OFFSET)
#define PIC32MZ_REFO1TRIM         (PIC32MZ_OSC_K1BASE+PIC32MZ_REFO1TRIM_OFFSET)
#define PIC32MZ_REFO2CON          (PIC32MZ_OSC_K1BASE+PIC32MZ_REFO2CON_OFFSET)
#define PIC32MZ_REFO2TRIM         (PIC32MZ_OSC_K1BASE+PIC32MZ_REFO2TRIM_OFFSET)
#define PIC32MZ_REFO3CON          (PIC32MZ_OSC_K1BASE+PIC32MZ_REFO3CON_OFFSET)
#define PIC32MZ_REFO3TRIM         (PIC32MZ_OSC_K1BASE+PIC32MZ_REFO3TRIM_OFFSET)
#define PIC32MZ_REFO4CON          (PIC32MZ_OSC_K1BASE+PIC32MZ_REFO4CON_OFFSET)
#define PIC32MZ_REFO4TRIM         (PIC32MZ_OSC_K1BASE+PIC32MZ_REFO4TRIM_OFFSET)
#define PIC32MZ_PB1DIV            (PIC32MZ_OSC_K1BASE+PIC32MZ_PB1DIV_OFFSET)
#define PIC32MZ_PB2DIV            (PIC32MZ_OSC_K1BASE+PIC32MZ_PB2DIV_OFFSET)
#define PIC32MZ_PB3DIV            (PIC32MZ_OSC_K1BASE+PIC32MZ_PB3DIV_OFFSET)
#define PIC32MZ_PB4DIV            (PIC32MZ_OSC_K1BASE+PIC32MZ_PB4DIV_OFFSET)
#define PIC32MZ_PB5DIV            (PIC32MZ_OSC_K1BASE+PIC32MZ_PB5DIV_OFFSET)
#define PIC32MZ_PB6DIV            (PIC32MZ_OSC_K1BASE+PIC32MZ_PB6DIV_OFFSET)
#define PIC32MZ_PB7DIV            (PIC32MZ_OSC_K1BASE+PIC32MZ_PB7DIV_OFFSET)
#define PIC32MZ_PB8DIV            (PIC32MZ_OSC_K1BASE+PIC32MZ_PB8DIV_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Oscillator control register offset */

#define OSCCON_OSWEN              (1 << 0)  /* Bit 0: Oscillator switch enable */
#define OSCCON_SOSCEN             (1 << 1)  /* Bit 1: 32.768kHz secondary oscillator enable */
#define OSCCON_CF                 (1 << 3)  /* Bit 3: Clock fail detect */
#define OSCCON_SLPEN              (1 << 4)  /* Bit 4: Sleep mode enable */
#define OSCCON_SLOCK              (1 << 5)  /* Bit 5: PLL lock status */
#define OSCCON_ULOCK              (1 << 6)  /* Bit 6: USB PLL lock status */
#define OSCCON_CLKLOCK            (1 << 7)  /* Bit 7: Clock selection lock enable */
#define OSCCON_NOSC_SHIFT         (8)       /* Bits 8-10: New oscillator selection */
#define OSCCON_NOSC_MASK          (7 << OSCCON_NOSC_SHIFT)
#  define OSCCON_NOSC_FRC         (0 << OSCCON_NOSC_SHIFT) /* Internal fast RC oscillator / FRCDIV */
#  define OSCCON_NOSC_SPLL        (1 << OSCCON_NOSC_SHIFT) /* System PLL */
#  define OSCCON_NOSC_POSC        (2 << OSCCON_NOSC_SHIFT) /* Primary oscillator (HS or EC) */
#  define OSCCON_NOSC_SOSC        (4 << OSCCON_NOSC_SHIFT) /* Secondary oscillator */
#  define OSCCON_NOSC_LPRC        (5 << OSCCON_NOSC_SHIFT) /* Internal low power RC oscillator */
#  define OSCCON_NOSC_FRCDIV      (7 << OSCCON_NOSC_SHIFT) /* Internal fast RC / FRCDIV */

#define OSCCON_COSC_SHIFT         (12)      /* Bits 12-14: Current oscillator selection */
#define OSCCON_COSC_MASK          (7 << OSCCON_COSC_SHIFT)
#  define OSCCON_COSC_FRC         (0 << OSCCON_COSC_SHIFT) /* Internal fast RC oscillator / FRCDIV */
#  define OSCCON_COSC_SPLL        (1 << OSCCON_COSC_SHIFT) /* System PLL */
#  define OSCCON_COSC_POSC        (2 << OSCCON_COSC_SHIFT) /* Primary oscillator (HS or EC) */
#  define OSCCON_COSC_SOSC        (4 << OSCCON_COSC_SHIFT) /* Secondary oscillator */
#  define OSCCON_COSC_LPRC        (5 << OSCCON_COSC_SHIFT) /* Internal low power RC oscillator */
#  define OSCCON_COSC_BFRC        (5 << OSCCON_COSC_SHIFT) /* Back-up Fast RC Oscillator */
#  define OSCCON_COSC_FRCDIV      (7 << OSCCON_COSC_SHIFT) /* Internal fast RC / FRCDIV */

#define OSCCON_SOSCRDY            (1 << 22) /* Bit 22: Secondary oscillator ready */
#define OSCCON_DRMEN              (1 << 23) /* Bit 23: Dream mode enable */
#define OSCCON_FRCDIV_SHIFT       (24)      /* Bits 24-26: FRC oscillator divider */
#define OSCCON_FRCDIV_MASK        (7 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV(n)        ((uint32_t)((n)-1) << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV1      (0 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV2      (1 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV4      (2 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV8      (3 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV16     (4 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV32     (5 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV64     (6 << OSCCON_FRCDIV_SHIFT)
#  define OSCCON_FRCDIV_DIV256    (7 << OSCCON_FRCDIV_SHIFT)

/* FRC tuning register offset (6-bit, signed twos complement) */

#define OSCTUN_SHIFT              (0)       /* Bits 0-5: FRC tuning bits */
#define OSCTUN_MASK               (0x3f << OSCTUN_SHIFT)
#  define OSCTUN_MIN              (0x20 << OSCTUN_SHIFT)
#  define OSCTUN_CENTER           (0x00 << OSCTUN_SHIFT)
#  define OSCTUN_MAX              (0x1f << OSCTUN_SHIFT)

/* System PLL control register */

#define SPLLCON_PLLRANGE_SHIFT    (0)      /* Bits 0-2: System PLL Frequency Range Selection bits */
#define SPLLCON_PLLRANGE_MASK     (7 << SPLLCON_PLLRANGE_SHIFT)
#  define SPLLCON_PLLRANGE_BYPASS   (0 << SPLLCON_PLLRANGE_SHIFT) /* Bypass */
#  define SPLLCON_PLLRANGE_5_10MHZ  (1 << SPLLCON_PLLRANGE_SHIFT) /* 5-10 MHz */
#  define SPLLCON_PLLRANGE_8_16MHZ  (2 << SPLLCON_PLLRANGE_SHIFT) /* 8-16 MHz */
#  define SPLLCON_PLLRANGE_13_26MHZ (3 << SPLLCON_PLLRANGE_SHIFT) /* 13-26 MHz */
#  define SPLLCON_PLLRANGE_21_42MHZ (4 << SPLLCON_PLLRANGE_SHIFT) /* 21-42 MHz */
#  define SPLLCON_PLLRANGE_34_64MHZ (5 << SPLLCON_PLLRANGE_SHIFT) /* 34-64 MHz */

#define SPLLCON_PLLICLK          (1 << 7)  /* Bit 7:  System PLL Input Clock Source bit */
#define SPLLCON_PLLIDIV_SHIFT    (8)       /* Bits 8-10: System PLL Input Clock Divider bits */
#define SPLLCON_PLLIDIV_MASK     (7 << SPLLCON_PLLIDIV_SHIFT)
#  define SPLLCON_PLLIDIV(n)     ((uint32_t)((n)-1) << SPLLCON_PLLIDIV_SHIFT) /* Divide by n, n=1..8 */

#define SPLLCON_PLLMULT_SHIFT    (16)      /* Bits 16-22 <6:0>: System PLL Multiplier bits */
#define SPLLCON_PLLMULT_MASK     (0x7f << SPLLCON_PLLMULT_SHIFT)
#  define SPLLCON_PLLMULT(n)     ((uint32_t)((n)-1) << SPLLCON_PLLMULT_SHIFT) /* Muliply by n, n=1..128 */

#define SPLLCON_PLLODIV_SHIFT    (24)      /* Bits 24-26: System PLL Output Clock Divider bits */
#define SPLLCON_PLLODIV_MASK     (7 << SPLLCON_PLLODIV_SHIFT)
#  define SPLLCON_PLLODIV_2      (1 << SPLLCON_PLLODIV_SHIFT) /* PLL Divide by 2 */
#  define SPLLCON_PLLODIV_4      (2 << SPLLCON_PLLODIV_SHIFT) /* PLL Divide by 4 */
#  define SPLLCON_PLLODIV_8      (3 << SPLLCON_PLLODIV_SHIFT) /* PLL Divide by 8 */
#  define SPLLCON_PLLODIV_16     (4 << SPLLCON_PLLODIV_SHIFT) /* PLL Divide by 16 */
#  define SPLLCON_PLLODIV_32     (5 << SPLLCON_PLLODIV_SHIFT) /* PLL Divide by 32 */

/* Reference oscillator control register n, n=1..4 */

#define REFOCON_ROSEL_SHIFT      (0)       /* Bits 0-3: Reference Clock Source Select bits */
#define REFOCON_ROSEL_MASK       (15 << REFOCON_ROSEL_SHIFT)
#  define REFOCON_ROSEL_SYSCLK   (0 << REFOCON_ROSEL_SHIFT) /* SYSCLK */
#  define REFOCON_ROSEL_PBCLK1   (1 << REFOCON_ROSEL_SHIFT) /* PBCLK1 */
#  define REFOCON_ROSEL_POSC     (2 << REFOCON_ROSEL_SHIFT) /* POSC */
#  define REFOCON_ROSEL_FRC      (3 << REFOCON_ROSEL_SHIFT) /* FRC */
#  define REFOCON_ROSEL_LPRC     (4 << REFOCON_ROSEL_SHIFT) /* LPRC */
#  define REFOCON_ROSEL_SOSC     (5 << REFOCON_ROSEL_SHIFT) /* SOSC */
#  define REFOCON_ROSEL_SPLL     (7 << REFOCON_ROSEL_SHIFT) /* System PLL output */
#  define REFOCON_ROSEL_REFCLKI  (8 << REFOCON_ROSEL_SHIFT) /* REFCLKIx */
#  define REFOCON_ROSEL_BFRC     (9 << REFOCON_ROSEL_SHIFT) /* BFRC */

#define REFOCON_ACTIVE           (1 << 8)  /* Bit 8:  Reference Clock Request Status bit */
#define REFOCON_DIVSWEN          (1 << 9)  /* Bit 9:  Divider Switch Enable bit */
#define REFOCON_RSLP             (1 << 11) /* Bit 11: Reference Oscillator Module Run in Sleep bit */
#define REFOCON_OE               (1 << 12) /* Bit 12: Reference Clock Output Enable bit */
#define REFOCON_SIDL             (1 << 13) /* Bit 13: Peripheral Stop in Idle Mode bit */
#define REFOCON_ON               (1 << 15) /* Bit 15: Output Enable bit */
#define REFOCON_RODIV_SHIFT      (16)      /* Bits 16-30: Reference Clock Divider bits */
#define REFOCON_RODIV_MASK       (0x7fff << REFOCON_RODIV_SHIFT)
#  define REFOCON_RODIV(n)       ((uint32_t)(n) << REFOCON_RODIV_SHIFT)

/* Reference oscillator trim register n, n=1..4 */

#define REFOTRIM_SHIFT           (23)      /* Bits 23-31: Reference Oscillator Trim bits */
#define REFOTRIM_MASK            (0x1ff << REFOTRIM_SHIFT)

/* Peripheral bus n clock divisor control register n=1..8 */

#define PBDIV_SHIFT              (0)       /* Bits 0-6: Peripheral Bus Clock Divisor Control bits */
#define PBDIV_MASK               (0x7f << PBDIV_SHIFT)
#  define PBDIV(n)               ((uint32_t)((n)-1) << PBDIV_SHIFT) /* PBCLK = SYSLCK/n, n=1..128 */

#define PBDIV_PBDIVRDY           (1 << 11) /* Bit 11: Peripheral Bus Clock Divisor Ready bit */
#define PBDIV_ON                 (1 << 15) /* Bit 15: Peripheral Bus Output Clock Enable bit */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_OSC_H */
