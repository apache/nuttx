/****************************************************************************
 * boards/z80/ez80/z20x/src/z20x.h
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

#ifndef __BOARDS_Z80_EZ80_Z20X_SRC_Z20X_H
#define __BOARDS_Z80_EZ80_Z20X_SRC_Z20X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration */

#define HAVE_MMCSD 1
#if !defined(CONFIG_MMCSD_SPI) || !defined(CONFIG_EZ80_SPI)
#  undef HAVE_MMCSD
#endif

/* Helpers for accessing memory mapped registers */

#define ez80_getreg8(a)   (*(volatile uint8_t *)(a))
#define ez80_putreg8(v,a) (*(volatile uint8_t *)(a) = (v))

/* Memory map.  Board-specific extensions to the basic ez80f91 memory map
 * (see arch/z80/src/ez80/ez80f91.h)
 *
 *   00 0000 - 01 ffff - 128Kb FLASH
 *   02 0000 - 03 ffff - (Reserved for parts with 256Kb FLASH)
 *   04 0000 - 0b ffff - 512Kb External SRAM
 *                       SSD1963  LCD frame buffer interface
 *                       YM2413B Sound Generator
 *   af e000 - af ffff - 8Kb on-chip SRAM
 *   af e000 - af e3ff - IDLE stack
 *
 * Chip select 0 is for the 512Kb AS6C4008 SRAM starting at address 0x40000
 * (after the flash).
 *
 *   __CS0_LBR_INIT_PARAM = 0x04   Lower address 04 0000
 *   __CS0_UBR_INIT_PARAM = 0x0b   Upper address 0b ffff
 *   __CS0_CTL_INIT_PARAM = 0x08   CTL[5-7] = Zero wait states
 *                                 CTL[4]   = Memory (vs I/O)
 *                                 CTL[3]   = Enable
 *                                 CTL[0-2] = Unused
 *   __CS0_BMC_INIT_PARAM = 0x00   BMC[6-7] = eZ80 bus mode
 *                                 BMC[5]   = Separate address and data
 *                                 BMC[4]   = Unused
 *                                 BMC[0-3] = Ignored in eZ80 mode
 *
 * Chip select 1 is for the SSD1963 LCD frame buffer interface
 * Chip select 2 is for the YM2413B Sound Generator
 * Chip select 3 is not used
 */

/* LED and port emulation memory register addresses */

/* GPIO data bit definitions */

#define EZ80_GPIOD0       (1 << 0)
#define EZ80_GPIOD1       (1 << 1)
#define EZ80_GPIOD2       (1 << 2)
#define EZ80_GPIOD3       (1 << 3)
#define EZ80_GPIOD4       (1 << 4)
#define EZ80_GPIOD5       (1 << 5)
#define EZ80_GPIOD6       (1 << 6)
#define EZ80_GPIOD7       (1 << 7)

/****************************************************************************
 * Public Functions
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
 * Name: ez80_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int ez80_bringup(void);

/*****************************************************************************
 * Name: ez80_mmcsd_initialize
 *
 * Description:
 *   Initialize SPI-based SD card.
 *
 ****************************************************************************/

#ifdef HAVE_MMCSD
int ez80_mmcsd_initialize(void);
#endif

/****************************************************************************
 * Name: ez80_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the z20x board.
 *
 ****************************************************************************/

#ifdef CONFIG_EZ80_SPI
void ez80_spidev_initialize(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __BOARDS_Z80_EZ80_Z20X_SRC_Z20X_H */
