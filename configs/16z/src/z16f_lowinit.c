/***************************************************************************
 * configs/16z/src/z16f_lowinit.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based upon sample code included with the Zilog ZDS-II toolchain.
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>

#include "chip/chip.h"

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/***************************************************************************
 * Private Functions
 ***************************************************************************/

/***************************************************************************
 * Name: z16f_extcsinit
 ***************************************************************************/

static void z16f_extcsinit(void)
{
  /* CS0 enabled, Data [0:7]
   * Post Read: No wait states; Chip select: 1 wait state
   */

  putreg16(0x9001, Z16F_EXTCS0H);

  /* CS1 enabled, Data [0:7]
   * Post Read: No wait states; Chip select: 1 wait state
   */

  putreg16(0x9001, Z16F_EXTCS1);

  /* CS2 enabled, Data [0:7]
   * Post Read: 1 wait state; Chip select: 1 wait state
   */

  putreg16(0x9011, Z16F_EXTCS2);

  /* CS3 enabled, Data [0:7]
   * Post Read: 1 wait state; Chip select: 5 wait states
   */

  putreg16(0x9015, Z16F_EXTCS3);

  /* CS4 enabled, Data [0:7]
   * Post Read: 1 wait state; Chip select: 5 wait states
   */

  putreg16(0x9015, Z16F_EXTCS4);

  /* CS5 enabled, Data [0:7]
   * Post Read: 1 wait state; Chip select: 5 wait states
   */

  putreg16(0x9015, Z16F_EXTCS5);

  /* Enable the 8-bit external bus interface */

  putreg8(0x40, Z16F_EXTCT);
}

/***************************************************************************
 * Name: z16f_gpioinit
 *
 * Description:
 *   Configure board-specific GPIO usage here.  Driver pin configurations
 *   are set in the associated device drivers (such as UART, SPI, I2C,
 *   etc.) and must be preserved.
 *
 ***************************************************************************/

static void z16f_gpioinit(void)
{
  /* NOTE:  Here we assume that all ports are in the default reset state */

  /* --------------------------- ------ --------------------------------------------
   * GPIO                        SIGNAL On-Board Connections
   * --------------------------- ------ --------------------------------------------
   * PA0/T0IN/T0OUT/DMA0REQ      GP8    PS/2 / GPIO, Expansion slots
   * PA1/T0OUT/DMA0ACK           GP9    PS/2 / GPIO, Expansion slots
   * PA2/DE0/FAULTY              ~INTI  Power section, RF transceiver (1)
   * PA3/CTS0/FAULT0             ~INTX  Expansion slots
   * PA4/RXD0/CS1                RXD    MAX3232D RS-232
   * PA5/TXD0/CS2                TXD    MAX3232D RS-232
   * PA6/SCL/CS3                 SCL    RTC / UID, Expansion slots
   * PA7/SDA/CS4                 SDA    RTC / UID, Expansion slots
   * --------------------------- ------ --------------------------------------------
   */

  putreg8(0x9f, Z16F_GPIOA_DD);  /* Inputs: GP8, GP9, ~INTI, ~INTX, RXD, and SDA */
  putreg8(0x03, Z16F_GPIOA_HDE); /* High drive current: GP8 and GP9 */
  putreg8(0xf0, Z16F_GPIOA_AFL); /* RXD=Alt 1, TXD=Alt 1, SCL=Alt 1, SDA=Alt 1 */

  /* --------------------------- ------ --------------------------------------------
   * GPIO                        SIGNAL On-Board Connections
   * --------------------------- ------ --------------------------------------------
   * PB0/ANA0/T0IN0              GP0    Expansion slots
   * PB1/ANA1/T0IN1              GP1    Expansion slots
   * PB2/ANA2/T0IN2              GP2    Expansion slots
   * PB3/ANA3/OPOUT              GP3    Expansion slots
   * PB4/ANA4                    GP4    Expansion slots
   * PB5/ANA5                    GP5    Expansion slots
   * PB6/ANA6/OPINP/CINN         GP6    Expansion slots
   * PB7/ANA7/OPINN              GP7    Expansion slots
   * --------------------------- ------ --------------------------------------------
   */

  putreg8(0xff, Z16F_GPIOB_DD);  /* Inputs: all */
  putreg8(0xff, Z16F_GPIOB_HDE); /* High drive current: all */

  /* --------------------------- ------ --------------------------------------------
   * GPIO                        SIGNAL On-Board Connections
   * --------------------------- ------ --------------------------------------------
   * PC0/T1IN/T1OUT/DMA1REQ/CINN GP10   PS/2 / GPIO, Expansion slots
   * PC1/T1OUT/DMA1ACK/COMPOUT   GP11   PS/2 / GPIO, Expansion slots
   * PC2/SS/CS4                  ~EXP   Expansion slots
   * PC3/SCK/DMA2REQ             SCK    FT800Q, Serial memory (1), RF Transceiver (1),
   *                                    Expansion slots, SD0, 1, and 2
   * PC4/MOSI/DMA2ACK            MOSI   FT800Q, Serial memory (1), RF Transceiver (1),
   *                                    Expansion slots, SD0, 1, and 2
   * PC5/MISO/CS5                MISO   FT800Q, Serial memory (1), RF Transceiver (1),
   *                                    Expansion slots, SD0, 1, and 2
   * PC6/T2IN/T2OUT/PWMH0        ~CTS   MAX3232D RS-232
   * PC7/T2OUT/PWML0             ~RTS   MAX3232D RS-232, Power section (?)
   * --------------------------- ------ --------------------------------------------
   */

  putreg8(0x63, Z16F_GPIOC_DD);  /* Inputs: GP10, GP11, MISO, and ~CTS */
  putreg8(0x03, Z16F_GPIOC_HDE); /* High drive current: GP10 and GP11 */
  putreg8(0x38, Z16F_GPIOC_AFL); /* SCK=Alt 1, MOSI=Alt 1, MISO=Alt 1 */
  putreg8(0x04, Z16F_GPIOC_OUT); /* ~EXP output high */

  /* --------------------------- ------ --------------------------------------------
   * GPIO                        SIGNAL On-Board Connections
   * --------------------------- ------ --------------------------------------------
   * PD0/PWMH1/ADR20             A20    RAM, Expansion slots
   * PD1/PWML1/ADR21             A21    RAM, Expansion slots
   * PD2/PWMH2/ADR22             A22    RAM, Expansion slots
   * PD3/DE1/ADR16               A16    RAM, Expansion slots
   * PD4/RXD1/ADR18              A18    RAM, Expansion slots
   * PD5/TXD1/ADR19              A19    RAM, Expansion slots
   * PD6/CTS1/ADR17              A17    RAM, Expansion slots
   * PD7/PWML2/ADR23             A23    Expansion slots
   * --------------------------- ------ --------------------------------------------
   */

  putreg8(0x00, Z16F_GPIOD_DD);  /* Inputs: None */
  putreg8(0xff, Z16F_GPIOD_AFH); /* All=Alt 2

  /* --------------------------- ------ --------------------------------------------
   * GPIO                        SIGNAL On-Board Connections
   * --------------------------- ------ --------------------------------------------
   * PE0/DATA0                   D0     RAM, Expansion slots
   * PE1/DATA1                   D1     RAM, Expansion slots
   * PE2/DATA2                   D2     RAM, Expansion slots
   * PE3/DATA3                   D3     RAM, Expansion slots
   * PE4/DATA4                   D4     RAM, Expansion slots
   * PE5/DATA5                   D5     RAM, Expansion slots
   * PE6/DATA6                   D6     RAM, Expansion slots
   * PE7/DATA7                   D7     RAM, Expansion slots
   * --------------------------- ------ --------------------------------------------
   */

  putreg8(0xff, Z16F_GPIOE_DD); /* Inputs: all */

  /* --------------------------- ------ --------------------------------------------
   * GPIO                        SIGNAL On-Board Connections
   * --------------------------- ------ --------------------------------------------
   * PF0/ADR0                    A0     Expansion slots
   * PF1/ADR1                    A1     RAM, Expansion slots
   * PF2/ADR2                    A2     RAM, Expansion slots
   * PF3/ADR3                    A3     RAM, Expansion slots
   * PF4/ADR4                    A4     RAM, Expansion slots
   * PF5/ADR5                    A5     RAM, Expansion slots
   * PF6/ADR6                    A6     RAM, Expansion slots
   * PF7/ADR7                    A7     RAM, Expansion slots
   * --------------------------- ------ --------------------------------------------
   */

  putreg8(0x00, Z16F_GPIOF_DD);  /* Inputs: None */
  putreg8(0xff, Z16F_GPIOF_AFL); /* All=Alt 1 */

  /* --------------------------- ------ --------------------------------------------
   * GPIO                        SIGNAL On-Board Connections
   * --------------------------- ------ --------------------------------------------
   * PG0/ADR0                    A8     RAM, Expansion slots
   * PG1/ADR0                    A9     RAM, Expansion slots
   * PG2/ADR0                    A10    RAM, Expansion slots
   * PG3/ADR0                    A11    RAM, Expansion slots
   * PG4/ADR0                    A12    RAM, Expansion slots
   * PG5/ADR0                    A13    RAM, Expansion slots
   * PG6/ADR0                    A14    RAM, Expansion slots
   * PG7/ADR0                    A15    RAM, Expansion slots
   * --------------------------- ------ --------------------------------------------
   */

  putreg8(0x00, Z16F_GPIOG_DD);  /* Inputs: None */
  putreg8(0xff, Z16F_GPIOG_AFL); /* All=Alt 1 */

  /* --------------------------- ------ --------------------------------------------
   * GPIO                        SIGNAL On-Board Connections
   * --------------------------- ------ --------------------------------------------
   * PH0/ANA8/WR                 ~WR    RAM, Expansion slots
   * PH1/ANA9/RD                 ~RD    RAM, Expansion slots
   * PH2/ANA10/CS0               ~RF    LED3, RF transceiver, X2 (1)
   * PH3/ANA11/CINP/WAIT         ~SXM   LED4, Chip select for the serial memory, U4 (1)
   * --------------------------- ------ --------------------------------------------
   */

  putreg8(0xf0, Z16F_GPIOH_DD);  /* Inputs: None (PH4-PH7 undefined) */
  putreg8(0x03, Z16F_GPIOH_AFH); /*~WR=Alt2 ~RD=Alt 2 */
  putreg8(0x0c, Z16F_GPIOH_OUT); /* Output high: ~RF and ~SXM */

  /* --------------------------- ------ --------------------------------------------
   * GPIO                        SIGNAL On-Board Connections
   * --------------------------- ------ --------------------------------------------
   * PJ0/DATA8                   ~SD1   LED5, Chip select for the SD card 1, X11.
   * PJ1/DATA9                   ~DT1   Card detect for SD card 1
   * PJ2/DATA10                  WP1    Write protect for SD card 1
   * PJ3/DATA11                  EVE    EVE chip select
   * PJ4/DATA12                  ~SD2   LED6, Chip select for the SD card 2, X10.
   * PJ5/DATA13                  ~DT2   Card detect for SD card 2
   * PJ6/DATA14                  WP2    Write protect for SD card 2
   * PJ7/DATA15                  ~SD0   LED7, Chip select for the microSD 0, X12.
   * --------------------------- ------ --------------------------------------------
   */

  putreg8(0x66, Z16F_GPIOJ_DD);  /* Inputs: ~DT1, WP1, ~DT2, and WP2 */
  putreg8(0x99, Z16F_GPIOJ_OUT); /* Output high:  ~SD1, EVE, ~SD2, ~SD0 */

  /* --------------------------- ------ --------------------------------------------
   * GPIO                        SIGNAL On-Board Connections
   * --------------------------- ------ --------------------------------------------
   * PK0/BHEN                    ~BHE   RAM, Expansion slots
   * PK1/BLEN                    ~BLE   RAM, Expansion slots
   * PK2/CS0                     ~0000  Bottom RAM bank, Expansion slots
   * PK3/CS1                     ~8000  Top RAM bank, Expansion slots
   * PK4/CS2                     ~F000  Expansion slots
   * PK5/CS3                     ~FFC8  Expansion slots
   * PK6/CS4                     ~FFD0  Expansion slots
   * PK7/CS5                     ~FFD8  Expansion slots
   * --------------------------- ------ --------------------------------------------
   */

  putreg8(0x00, Z16F_GPIOK_DD);  /* Inputs: None */
  putreg8(0xff, Z16F_GPIOK_AFL); /* All=Alt 1 */
}

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Name: z16f_lowinit
 ***************************************************************************/

void z16f_lowinit(void)
{
  z16f_extcsinit();  /* Configure external memory */
  z16f_gpioinit();   /* Configure board GPIOs */
}
