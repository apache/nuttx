/****************************************************************************
 * boards/z80/ez80/makerlisp/src/makerlisp.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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
