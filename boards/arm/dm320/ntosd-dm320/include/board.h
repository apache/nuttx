/****************************************************************************
 * boards/arm/dm320/ntosd-dm320/include/board.h
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

#ifndef __BOARDS_ARM_DM320_NTOSD_DM320_INCLUDE_BOARD_H
#define __BOARDS_ARM_DM320_NTOSD_DM320_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* This platform has the ARM at 175 MHz and the DSP at 101.25 MHz */

#define DM320_ARM_CLOCK  175500000
#define DM320_SDR_CLOCK  101250000
#define DM320_DSP_CLOCK  101250000
#define DM320_AXL_CLOCK  175500000
#define DM320_AHB_CLOCK   87750000

/* UART0/1 and TIMER0/1 are clocked by PLLIN=27MHz */

#define CONFIG_DM320_UARTPPLIN 1

/* Configuration for dm9000 network device */

#define DM9000_BASE      CONFIG_DM9000_BASE

/* Memory Map ***************************************************************/

/* The Neuros development board has 16MiB RAM starting at 0x01000000
 * (physical) and 8MiB of FLASH.
 * The Neuros OSD 1.0 consumer  has 32MiB RAM starting at 0x01100000
 * (physical) and 16MiB of FLASH.
 *
 * FIXME: Flash location may also differ on OSD 1.0 consumer unit!
 */

#ifdef CONFIG_ARCH_NTOSD_DEVBOARD
#  if CONFIG_RAM_START != 0x01000000
#    error "Invalid setting for CONFIG_RAM_START"
#  endif
#  if CONFIG_RAM_SIZE != 0x01000000
#    warning "Check CONFIG_RAM_SIZE.  This Neuros OSD has 0x01000000 bytes of SDRAM"
#  endif
#  define DM320_SDRAM_PSECTION         0x01000000 /* 496Mb  many section   -- */
#  define   DM320_SDRAM_PADDR          0x01000000 /* 496Mb  many sections  CW */
#else
#  if CONFIG_RAM_START != 0x01100000
#    error "Invalid setting for CONFIG_RAM_START"
#  endif
#  if CONFIG_RAM_SIZE != 0x02000000
#    warning "Check CONFIG_RAM_SIZE.  This Neuros OSD has 0x02000000 bytes of SDRAM"
#  endif
#  define DM320_SDRAM_PSECTION         0x01100000 /* 496Mb  many section   -- */
#  define   DM320_SDRAM_PADDR          0x01100000 /* 496Mb  many sections  CW */
#endif

/* GIO keyboard (GIO 1-5) */

#define KEY_MASK         0x003E
#define KEY_SCAN0_BIT    0x0002
#define KEY_SCAN1_BIT    0x0004
#define KEY_SCAN2_BIT    0x0008
#define KEY_SCAN3_BIT    0x0010
#define KEY_SCAN4_BIT    0x0020

#define KEY_GIO_DIR0_VAL KEY_MASK     /* Configure as INPUT */
#define KEY_GIO_INV0_VAL KEY_MASK     /* All inverted */
#define KEY_GIO_SET0_VAL (0)          /* Initialized to zero */
#define KEY_GIO_CLR0_VAL (0)

#define GIO_KEY_SCAN0    1
#define GIO_KEY_SCAN1    2
#define GIO_KEY_SCAN2    3
#define GIO_KEY_SCAN3    4
#define GIO_KEY_SCAN4    5
#define GIO_MS_DETECT    5
#define GIO_DM9000A_INT  6
#define GIO_MMC_DETECT   8
#define GIO_CFC_DETECT   9
#define GIO_VIDEO_IN     10
#define GIO_LED_RED      16
#define GIO_LED_GREEN    17
#define GIO_CFC_ENABLE   25
#define GIO_I2C_SCL      30
#define GIO_I2C_SDA      31
#define GIO_ENA_VIDEO    32
#define GIO_CFC_RESET    36
#define GIO_CFC_STSCHG   37

/* LED Usage */

#define LED_STARTED      0
#define LED_HEAPALLOCATE 0
#define LED_IRQSENABLED  0
#define LED_STACKCREATED GIO_LED_GREEN
#define LED_INIRQ        GIO_LED_RED
#define LED_SIGNAL       GIO_LED_RED
#define LED_ASSERTION    GIO_LED_RED
#define LED_PANIC        GIO_LED_RED
#define LED_IDLE         0

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#endif
#endif /* __BOARDS_ARM_DM320_NTOSD_DM320_INCLUDE_BOARD_H */
