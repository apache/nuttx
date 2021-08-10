/****************************************************************************
 * boards/z80/ez80/makerlisp/src/makerlisp.h
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

#ifndef __BOARDS_Z80_EZ80_MAKERLISP_SRC_MAKERLISP_H
#define __BOARDS_Z80_EZ80_MAKERLISP_SRC_MAKERLISP_H

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

#define ez80_getreg8(a)   (*(uint8_t*)(a))
#define ez80_putreg8(v,a) (*(uint8_t*)(a) = (v))

/* Memory map.  Board-specific extensions to the basic ez80f91 memory map
 * (see arch/z80/src/ez80/ez80f91.h)
 *
 * - Chip select 0 is for RAM 0 to (512k - 1), starting at address 0x40000
 *   (after the flash).
 * - Chip select 1 is for 512k to (1M - 1), at address 0xc0000.
 *
 * All RAM has zero wait states.
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
 * Public Data
 ****************************************************************************/

extern bool g_ebpresent;  /* True:  I/O Expansion board is present */

/****************************************************************************
 * Public Functions Definitions
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
 *   Called to configure SPI chip select GPIO pins for the MakerLisp board.
 *
 ****************************************************************************/

#ifdef CONFIG_EZ80_SPI
void ez80_spidev_initialize(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __BOARDS_Z80_EZ80_MAKERLISP_SRC_MAKERLISP_H */
