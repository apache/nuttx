/****************************************************************************
 * boards/arm64/am62x/beagleplay/include/board_memorymap.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * BeaglePlay (AM6254) — 2 GB LPDDR4 at 0x8000_0000.
 * NuttX load address = 0x8200_0000 to avoid the DDR region reserved by
 * the BeagleBoard.org U-Boot environment.
 * Device space = 0x0000_0000 – 0x7FFF_FFFF (2 GB).
 *
 * defconfig uses only the first 512 MB so images are portable between
 * BeaglePlay and PocketBeagle 2.  Increase CONFIG_RAM_SIZE to use more.
 ****************************************************************************/

#ifndef __BOARDS_ARM64_AM62X_BEAGLEPLAY_INCLUDE_BOARD_MEMORYMAP_H
#define __BOARDS_ARM64_AM62X_BEAGLEPLAY_INCLUDE_BOARD_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#define BEAGLEPLAY_DEVICEIO_BASE    0x00000000ul
#define BEAGLEPLAY_DEVICEIO_SIZE    0x80000000ul
#define BEAGLEPLAY_DDR_BASE         0x80000000ul
#define BEAGLEPLAY_DDR_SIZE         0x20000000ul   /* 512 MB window */
#define BEAGLEPLAY_LOAD_ADDR        0x82000000ul

#endif /* __BOARDS_ARM64_AM62X_BEAGLEPLAY_INCLUDE_BOARD_MEMORYMAP_H */
