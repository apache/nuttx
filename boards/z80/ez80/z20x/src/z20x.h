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

#define HAVE_SPIFLASH 1
#define HAVE_MMCSD    1
#define HAVE_XPT2046  1

#if !defined(CONFIG_MTD_W25) || !defined(CONFIG_EZ80_SPI)
#  undef HAVE_SPIFLASH
#endif

#if !defined(CONFIG_MMCSD_SPI) || !defined(CONFIG_EZ80_SPI)
#  undef HAVE_MMCSD
#endif

#if !defined(CONFIG_INPUT_ADS7843E) || !defined(CONFIG_EZ80_SPI)
#  undef HAVE_XPT2046
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

/* RAM Memory map
 *
 * 040000              Beginning of RAM
 * 040000 _vecstart    Beginning of Interrupt Redirection information.  This
 *                     is used to hand off to RAM-based handlers for
 *                     interrupts caught by FLASH interrupt vectors. 512b is
 *                     set aside for RAM-based interrupt handling
 *                     information.
 * 0401ff _vecend      End of the Interrupt Redirection information.
 * 040200 _loaderstart Start of RAM used exclusively by the bootloader.
 *                     This memory region an be recovered by the RAM-based
 *                     program.
 * 04ffff _loaderend
 * 050000 _progstart   Start of CODE for the RAM-based program.  The
 *                     program can freely use the memory region from
 *                     050000-0bffff and can recover the memory for
 *                     40400-04ffff for heap usage.
 * 0bffff _progend     End of RAM
 */

extern uint8_t _vecstart[];
#define VECSTART     ((uintptr_t)_vecstart)

extern uint8_t _vecend[];
#define VECEND       ((uintptr_t)_vecend)

#define VECSIZE      (VECEND - VECSTART + 1)

extern uint8_t _loaderstart[];
#define LOADERSTART  ((uintptr_t)_loaderstart)

extern uint8_t _loaderend[];
#define LOADEREND    ((uintptr_t)_loaderend)

#define LOADERSIZE   (LOADEREND - LOADERSTART + 1)

extern uint8_t _progstart[];
#define PROGSTART    ((uintptr_t)_progstart)

extern uint8_t _progend[];
#define PROGEND      ((uintptr_t)_progend)

#define PROGSIZE     (PROGEND - PROGSTART + 1)

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

/* Winbond W25 SPI FLASH */

#ifndef CONFIG_Z20X_W25_MINOR
#  define CONFIG_Z20X_W25_MINOR 0
#endif

#define __STR(s) #s
#define __XSTR(s) __STR(s)

#define W25_CHARDEV  "/dev/mtd" __XSTR(CONFIG_Z20X_W25_MINOR)
#define W25_BLOCKDEV "/dev/mtdblock" __XSTR(CONFIG_Z20X_W25_MINOR)

/****************************************************************************
 * Public Function Prototypes
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
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int ez80_bringup(void);

/****************************************************************************
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

/****************************************************************************
 * Name: ez80_w25_initialize
 *
 * Description:
 *   Called to initialize Winbond W25 memory
 *
 ****************************************************************************/

#ifdef HAVE_SPIFLASH
int ez80_w25_initialize(int minor);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __BOARDS_Z80_EZ80_Z20X_SRC_Z20X_H */
