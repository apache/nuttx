/****************************************************************************
 * arch/arm/src/ra8m1/hardware/ra8m1_pinmap.h
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

#ifndef __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_PINMAP_H
#define __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/ra8m1_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/
#define R_PFS_PSEL_PORT_OFFSET          0x40
#define R_PFS_PSEL_PIN_OFFSET           0x04

/* RA8M1 has no PMISC peripheral.  Its PFS write-protect register (PWPR)
 * lives inside the PFS block itself -- User's Manual section 19.2.8.
 *
 * This port is a flat (no TrustZone) image, which always runs in the
 * secure state and uses the secure aliases of every register, so PWPR is
 * the secure instance (offset 0x514 from R_PFS_BASE).  The manual also
 * defines a non-secure PWPR at a different offset (0x50c from PFS_NS);
 * it is deliberately not used here.
 */

#define R_PFS_PWPR_OFFSET                0x0514

/* Port Control Registers (PCNTR1-4): confirmed against sections 19.2.1-
 * 19.2.4. RA8M1 packs two 16-bit halves into each 32-bit PCNTRn,
 * individually accessible at PCNTRn's own offset (low half) and +2
 * (high half).
 * PCNTR4/EOSR/EORR only exist on PORT1-4 (ELC port-group function); the
 * offsets are still defined generically here since nothing stops issuing
 * the access, only the manual limits which ports implement it.
 */

#define R_PORT_PCNTR1_OFFSET             0x0000  /* PDR (low 16) / PODR (high 16) */
#define R_PORT_PDR_OFFSET                0x0000  /* Pmn Direction (16-bit) */
#define R_PORT_PODR_OFFSET               0x0002  /* Pmn Output Data (16-bit) */
#define R_PORT_PCNTR2_OFFSET             0x0004  /* PIDR (low 16) / EIDR (high 16) */
#define R_PORT_PIDR_OFFSET               0x0004  /* Pmn State (16-bit, RO) */
#define R_PORT_EIDR_OFFSET               0x0006  /* Port Event Input Data (16-bit, RO) */
#define R_PORT_PCNTR3_OFFSET             0x0008  /* POSR (low 16) / PORR (high 16) */
#define R_PORT_POSR_OFFSET               0x0008  /* Pmn Output Set (16-bit, WO) */
#define R_PORT_PORR_OFFSET               0x000a  /* Pmn Output Reset (16-bit, WO) */
#define R_PORT_PCNTR4_OFFSET             0x000c  /* EOSR (low 16) / EORR (high 16), PORT1-4 only */
#define R_PORT_EOSR_OFFSET               0x000c  /* Pmn Event Output Set (16-bit) */
#define R_PORT_EORR_OFFSET               0x000e  /* Pmn Event Output Reset (16-bit) */

/* Register Addresses *******************************************************/

#define R_PFS(port,pin)                 (R_PFS_BASE + (port)*R_PFS_PSEL_PORT_OFFSET + (pin)*R_PFS_PSEL_PIN_OFFSET)

#define R_PFS_PWPR                      (R_PFS_BASE + R_PFS_PWPR_OFFSET)

/* R_PORT0_BASE..R_PORT9_BASE, R_PORT10_BASE (PORTA) and R_PORT11_BASE
 * (PORTB) in ra8m1_memorymap.h are all R_PORT0_BASE + port*0x20 -- this
 * indexes that uniform stride instead of switching over 12 discrete
 * per-instance macros.
 */

#define R_PORT_BASE(port)               (R_PORT0_BASE + (port)*0x20)

#define R_PORT_PDR(port)                (R_PORT_BASE(port) + R_PORT_PDR_OFFSET)
#define R_PORT_PODR(port)               (R_PORT_BASE(port) + R_PORT_PODR_OFFSET)
#define R_PORT_PIDR(port)               (R_PORT_BASE(port) + R_PORT_PIDR_OFFSET)
#define R_PORT_EIDR(port)               (R_PORT_BASE(port) + R_PORT_EIDR_OFFSET)
#define R_PORT_POSR(port)               (R_PORT_BASE(port) + R_PORT_POSR_OFFSET)
#define R_PORT_PORR(port)               (R_PORT_BASE(port) + R_PORT_PORR_OFFSET)
#define R_PORT_EOSR(port)               (R_PORT_BASE(port) + R_PORT_EOSR_OFFSET)
#define R_PORT_EORR(port)               (R_PORT_BASE(port) + R_PORT_EORR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* PmnPFS/PmnPFS_HA/PmnPFS_BY - Port mn Pin Function Select Register.
 *
 * Bit positions confirmed against the RA8M1 User's Manual (R01UH0994EJ0130
 * Rev.1.30), section 19.2.5.
 */

#define R_PFS_PSEL_SHIFT          (24) /* 28:24: Peripheral Select. For individual pin functions, see the tables in RA8M1 User's Manual chapter 19.6 */
#define R_PFS_PSEL_MASK           (0x1f)
#define R_PFS_PMR                 (1 << 16) /* Bit 16: Port Mode Control */
#define R_PFS_ASEL                (1 << 15) /* Bit 15: Analog Input enable */
#define R_PFS_ISEL                (1 << 14) /* Bit 14: IRQ input enable */
#define R_PFS_EOR                 (1 << 13) /* Bit 13: Event on Rising (EOFR[1:0] upper bit) */
#define R_PFS_EOF                 (1 << 12) /* Bit 12: Event on Falling (EOFR[1:0] lower bit) */
#define R_PFS_DSCR1               (1 << 11) /* Bit 11: Port Drive Capability 1 */
#define R_PFS_DSCR                (1 << 10) /* Bit 10: Port Drive Capability */
#define R_PFS_NCODR               (1 <<  6) /* Bit 6: N-Channel Open Drain Control */
#define R_PFS_PCR                 (1 <<  4) /* Bit 4: Pull-up Control */
#define R_PFS_PDR                 (1 <<  2) /* Bit 2: Port Direction */
#define R_PFS_PIDR                (1 <<  1) /* Bit 1: Port Input Data */
#define R_PFS_PODR                (1 <<  0) /* Bit 0: Port Output Data */

/* PWPR - Write-Protect Register for the PmnPFS registers (section 19.2.8).
 * PFSWE can only be set after B0WI has been cleared.
 */

#define R_PFS_PWPR_B0WI            (1 <<  7) /* Bit 7: PFSWE Bit Write Disable */
#define R_PFS_PWPR_PFSWE           (1 <<  6) /* Bit 6: PmnPFS Register Write Enable */

/* PSEL[4:0] peripheral-function values.
 *
 * RA8M1-specific: confirmed against User's Manual chapter 19.6
 * ("Peripheral Select Settings for Each Product", Tables 19.6-19.17,
 * covering PORT0 through PORTB). PSEL is a per-pin, per-port mux slot --
 * the same numeric value means a different peripheral/channel depending on
 * which pin it's applied to (e.g. PSEL=00100b is SCI0 on P112/P113 but
 * SCI4 on P205/P206) -- so PFS_PSEL_SCI/PFS_PSEL_SCI1 below name the two
 * generic "SCI mux slots" a port can offer, not a specific channel; the
 * GPIO_RXDn_MISOn_SCLn_x/GPIO_TXDn_MOSIn_SDAn_x macros below already bake
 * in which slot each specific pin needs.
 *
 * Only the SCI slots are defined here -- these are the only PSEL values
 * this file's pin macros use, and the only ones confirmed against chapter
 * 19.6. Every other peripheral (AGT/GPT/SPI/IIC/CLKOUT/CAN/etc.) has its
 * own PSEL value per chapter 19.6 too; add those here once verified the
 * same way, rather than guessing.
 */

#define PFS_PSEL_SCI                 (0x04 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_SCI1                (0x05 << R_PFS_PSEL_SHIFT)

/* GPIO ports and pins.
 *
 * RA8M1 has PORT0-PORT9 plus PORTA/PORTB (12 ports total, chapter 19.1
 * Table 19.1). PIN0-PIN15 are just the generic 0-15 per-port pin index
 * (every RA8M1 port has up to 16 pins, PmnPFS's own n range).
 */

#define PORT0  (0)
#define PORT1  (1)
#define PORT2  (2)
#define PORT3  (3)
#define PORT4  (4)
#define PORT5  (5)
#define PORT6  (6)
#define PORT7  (7)
#define PORT8  (8)
#define PORT9  (9)
#define PORTA  (10)
#define PORTB  (11)
#define PORT_INVALID (0xff)

#define PIN0   (0)
#define PIN1   (1)
#define PIN2   (2)
#define PIN3   (3)
#define PIN4   (4)
#define PIN5   (5)
#define PIN6   (6)
#define PIN7   (7)
#define PIN8   (8)
#define PIN9   (9)
#define PIN10  (10)
#define PIN11  (11)
#define PIN12  (12)
#define PIN13  (13)
#define PIN14  (14)
#define PIN15  (15)
#define PIN_INVALID (0xff)

/* SCI Alternative *********************************************************
 *
 * RA8M1 has SCI0-SCI4 and SCI9 (SCI5-8 do not exist). Each channel's RXD/
 * TXD pair can be routed to one of three alternate pin groups (the
 * manual's "_A"/"_B"/"_C" suffixes, numbered _1/_2/_3 below to match this
 * port's existing GPIO_RXDn..._x / GPIO_TXDn..._x convention). RXD shares
 * its pin with the simple-SPI MISO and simple-I2C SCL alternate functions;
 * TXD shares its pin with simple-SPI MOSI and simple-I2C SDA -- the PFS
 * PSEL value only selects "SCI" (or "SCI slot 2"); which of RXD/MISO/SCL
 * actually drives the pin depends on the SCI's own mode register, not PFS.
 *
 * Source: RA8M1 User's Manual chapter 19.6, Tables 19.6-19.17 (PORT0-
 * PORTB). SCK/CTS/RTS/DE (RS-485 driver-enable) pins exist for every
 * channel/group too but are not defined here: no current RA8M1 SCI driver
 * code (ra_serial.c) configures synchronous clock, hardware flow control,
 * or RS-485 mode, so there is nothing yet to consume them.
 */

#define GPIO_RXD0_MISO0_SCL0_1              (gpio_pinset_t){ PORT1,PIN13, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD0_MOSI0_SDA0_1              (gpio_pinset_t){ PORT1,PIN12, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_RXD0_MISO0_SCL0_2              (gpio_pinset_t){ PORT6,PIN2,  (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD0_MOSI0_SDA0_2              (gpio_pinset_t){ PORT6,PIN3,  (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_RXD0_MISO0_SCL0_3              (gpio_pinset_t){ PORT6,PIN10, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD0_MOSI0_SDA0_3              (gpio_pinset_t){ PORT6,PIN9,  (PFS_PSEL_SCI | R_PFS_PMR)}

#define GPIO_RXD1_MISO1_SCL1_1              (gpio_pinset_t){ PORT4,PIN1,  (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_TXD1_MOSI1_SDA1_1              (gpio_pinset_t){ PORT4,PIN0,  (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_RXD1_MISO1_SCL1_2              (gpio_pinset_t){ PORT7,PIN6,  (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_TXD1_MOSI1_SDA1_2              (gpio_pinset_t){ PORT7,PIN7,  (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_RXD1_MISO1_SCL1_3              (gpio_pinset_t){ PORT2,PIN12, (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_TXD1_MOSI1_SDA1_3              (gpio_pinset_t){ PORT2,PIN13, (PFS_PSEL_SCI1 | R_PFS_PMR)}

#define GPIO_RXD2_MISO2_SCL2_1              (gpio_pinset_t){ PORT8,PIN2,  (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD2_MOSI2_SDA2_1              (gpio_pinset_t){ PORT8,PIN1,  (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_RXD2_MISO2_SCL2_2              (gpio_pinset_t){ PORT7,PIN0,  (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD2_MOSI2_SDA2_2              (gpio_pinset_t){ PORT4,PIN6,  (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_RXD2_MISO2_SCL2_3              (gpio_pinset_t){ PORTA,PIN2,  (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD2_MOSI2_SDA2_3              (gpio_pinset_t){ PORTA,PIN3,  (PFS_PSEL_SCI | R_PFS_PMR)}

#define GPIO_RXD3_MISO3_SCL3_1              (gpio_pinset_t){ PORT4,PIN8,  (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_TXD3_MOSI3_SDA3_1              (gpio_pinset_t){ PORT4,PIN9,  (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_RXD3_MISO3_SCL3_2              (gpio_pinset_t){ PORT3,PIN9,  (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_TXD3_MOSI3_SDA3_2              (gpio_pinset_t){ PORT3,PIN10, (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_RXD3_MISO3_SCL3_3              (gpio_pinset_t){ PORT9,PIN1,  (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_TXD3_MOSI3_SDA3_3              (gpio_pinset_t){ PORT9,PIN0,  (PFS_PSEL_SCI1 | R_PFS_PMR)}

#define GPIO_RXD4_MISO4_SCL4_1              (gpio_pinset_t){ PORT2,PIN6,  (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD4_MOSI4_SDA4_1              (gpio_pinset_t){ PORT2,PIN5,  (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_RXD4_MISO4_SCL4_2              (gpio_pinset_t){ PORT4,PIN14, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD4_MOSI4_SDA4_2              (gpio_pinset_t){ PORT4,PIN15, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_RXD4_MISO4_SCL4_3              (gpio_pinset_t){ PORT7,PIN15, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD4_MOSI4_SDA4_3              (gpio_pinset_t){ PORT7,PIN14, (PFS_PSEL_SCI | R_PFS_PMR)}

#define GPIO_RXD9_MISO9_SCL9_1              (gpio_pinset_t){ PORT1,PIN1,  (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_TXD9_MOSI9_SDA9_1              (gpio_pinset_t){ PORT1,PIN2,  (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_RXD9_MISO9_SCL9_2              (gpio_pinset_t){ PORT2,PIN8,  (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_TXD9_MOSI9_SDA9_2              (gpio_pinset_t){ PORT2,PIN9,  (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_RXD9_MISO9_SCL9_3              (gpio_pinset_t){ PORTA,PIN15, (PFS_PSEL_SCI1 | R_PFS_PMR)}
#define GPIO_TXD9_MOSI9_SDA9_3              (gpio_pinset_t){ PORTA,PIN14, (PFS_PSEL_SCI1 | R_PFS_PMR)}

/* GPIO Configuration */

#define GPIO_OUTPUT               R_PFS_PDR
#define GPIO_INPUT               ~(R_PFS_PDR | 0xFFFFFFFF)

#define GPIO_LOW_DRIVE          ~(R_PFS_DSCR | 0xFFFFFFFF)
#define GPIO_MIDDLE_DRIVE       R_PFS_DSCR

#define GPIO_OUTPUT_HIGH         R_PFS_PODR
#define GPIO_OUTPUT_LOW         ~(R_PFS_PODR | 0xFFFFFFFF)

#define GPIO_PIN_INVALID         (gpio_pinset_t){ PORT_INVALID, PIN_INVALID, 0xFFFFFFFF }

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA8M1_HARDWARE_RA8M1_PINMAP_H */
