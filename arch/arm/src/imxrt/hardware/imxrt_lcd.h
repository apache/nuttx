/************************************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_lcd.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2016-2018 NXP. All rights reserved.
 *   Author: Johannes Schock (Port)
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_LCD_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_LCD_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* Register offsets *****************************************************************************/

#define IMXRT_LCDIF_CTRL_OFFSET               0x0000  /* General Control Register */
#define IMXRT_LCDIF_CTRL_SET_OFFSET           0x0004  /* General Control Register */
#define IMXRT_LCDIF_CTRL_CLR_OFFSET           0x0008  /* General Control Register */
#define IMXRT_LCDIF_CTRL_TOG_OFFSET           0x000c  /* General Control Register */
#define IMXRT_LCDIF_CTRL1_OFFSET              0x0010  /* General Control1 Register */
#define IMXRT_LCDIF_CTRL1_SET_OFFSET          0x0014  /* General Control1 Register */
#define IMXRT_LCDIF_CTRL1_CLR_OFFSET          0x0018  /* General Control1 Register */
#define IMXRT_LCDIF_CTRL1_TOG_OFFSET          0x001c  /* General Control1 Register */
#define IMXRT_LCDIF_CTRL2_OFFSET              0x0020  /* General Control2 Register */
#define IMXRT_LCDIF_CTRL2_SET_OFFSET          0x0024  /* General Control2 Register */
#define IMXRT_LCDIF_CTRL2_CLR_OFFSET          0x0028  /* General Control2 Register */
#define IMXRT_LCDIF_CTRL2_TOG_OFFSET          0x002c  /* General Control2 Register */
#define IMXRT_LCDIF_TRANSFER_COUNT_OFFSET     0x0030  /* Horizontal and Vertical Valid Data Count Register */
#define IMXRT_LCDIF_CUR_BUF_OFFSET            0x0040  /* Current Buffer Address Register */
#define IMXRT_LCDIF_NEXT_BUF_OFFSET           0x0050  /* Next Buffer Address Register */
#define IMXRT_LCDIF_VDCTRL0_OFFSET            0x0070  /* VSYNC Mode and Dotclk Mode Control Register0 */
#define IMXRT_LCDIF_VDCTRL0_SET_OFFSET        0x0074  /* VSYNC Mode and Dotclk Mode Control Register0 */
#define IMXRT_LCDIF_VDCTRL0_CLR_OFFSET        0x0078  /* VSYNC Mode and Dotclk Mode Control Register0 */
#define IMXRT_LCDIF_VDCTRL0_TOG_OFFSET        0x007c  /* VSYNC Mode and Dotclk Mode Control Register0 */
#define IMXRT_LCDIF_VDCTRL1_OFFSET            0x0080  /* VSYNC Mode and Dotclk Mode Control Register1 */
#define IMXRT_LCDIF_VDCTRL2_OFFSET            0x0090  /* VSYNC Mode and Dotclk Mode Control Register2 */
#define IMXRT_LCDIF_VDCTRL3_OFFSET            0x00a0  /* VSYNC Mode and Dotclk Mode Control Register3 */
#define IMXRT_LCDIF_VDCTRL4_OFFSET            0x00b0  /* VSYNC Mode and Dotclk Mode Control Register4 */
#define IMXRT_LCDIF_BM_ERROR_STAT_OFFSET      0x0190  /* Bus Master Error Status Register */
#define IMXRT_LCDIF_CRC_STAT_OFFSET           0x01a0  /* CRC Status Register */
#define IMXRT_LCDIF_STAT_OFFSET               0x01b0  /* LCD Interface Status Register */
#define IMXRT_LCDIF_THRES_OFFSET              0x0200  /* Threshold Register */

#define IMXRT_LCDIF_PIGEONCTRL0_OFFSET        0x0380  /* Pigeon Mode Control0 Register */
#define IMXRT_LCDIF_PIGEONCTRL0_SET_OFFSET    0x0384  /* Pigeon Mode Control0 Register */
#define IMXRT_LCDIF_PIGEONCTRL0_CLR_OFFSET    0x0388  /* Pigeon Mode Control0 Register */
#define IMXRT_LCDIF_PIGEONCTRL0_TOG_OFFSET    0x038c  /* Pigeon Mode Control0 Register */
#define IMXRT_LCDIF_PIGEONCTRL1_OFFSET        0x0390  /* Pigeon Mode Control1 Register */
#define IMXRT_LCDIF_PIGEONCTRL1_SET_OFFSET    0x0394  /* Pigeon Mode Control1 Register */
#define IMXRT_LCDIF_PIGEONCTRL1_CLR_OFFSET    0x0398  /* Pigeon Mode Control1 Register */
#define IMXRT_LCDIF_PIGEONCTRL1_TOG_OFFSET    0x039c  /* Pigeon Mode Control1 Register */
#define IMXRT_LCDIF_PIGEONCTRL2_OFFSET        0x03a0  /* Pigeon Mode Control2 Register */
#define IMXRT_LCDIF_PIGEONCTRL2_SET_OFFSET    0x03a4  /* Pigeon Mode Control2 Register */
#define IMXRT_LCDIF_PIGEONCTRL2_CLR_OFFSET    0x03a8  /* Pigeon Mode Control2 Register */
#define IMXRT_LCDIF_PIGEONCTRL2_TOG_OFFSET    0x03ac  /* Pigeon Mode Control2 Register */

#define IMXRT_LCDIF_PIGEON_START_OFFSET       0x0800  /* Pigeon */
#  define IMXRT_LCDIF_PIGEON0_OFFSET          0x0000  /* Pigeon 0 */
#  define IMXRT_LCDIF_PIGEON1_OFFSET          0x0010  /* Pigeon 1 */
#  define IMXRT_LCDIF_PIGEON2_OFFSET          0x0020  /* Pigeon 2 */
#  define IMXRT_LCDIF_PIGEON_SIZE             0x0040  /* Pigeon Element Size */
#  define IMXRT_LCDIF_PIGEON_NUM              12      /* Pigeon Element Number */
#  define IMXRT_LCDIF_PIGEON_OFFSET(n)        (IMXRT_LCDIF_PIGEON_START_OFFSET + ((n) * IMXRT_LCDIF_PIGEON_SIZE))

#define IMXRT_LCDIF_LUT_CTRL_OFFSET           0x0b00  /* Lookup Table Data Register */
#define IMXRT_LCDIF_LUT0_ADDR_OFFSET          0x0b10  /* Lookup Table Control Register */
#define IMXRT_LCDIF_LUT0_DATA_OFFSET          0x0b20  /* Lookup Table Data Register */
#define IMXRT_LCDIF_LUT1_ADDR_OFFSET          0x0b30  /* Lookup Table Control Register */
#define IMXRT_LCDIF_LUT1_DATA_OFFSET          0x0b40  /* Lookup Table Data Register */

/* Register Addresses ***************************************************************************/

#define IMXRT_LCDIF_CTRL                  (IMXRT_LCDIF_BASE + IMXRT_LCDIF_CTRL_OFFSET)
#define IMXRT_LCDIF_CTRL_SET              (IMXRT_LCDIF_BASE + IMXRT_LCDIF_CTRL_SET_OFFSET)
#define IMXRT_LCDIF_CTRL_CLR              (IMXRT_LCDIF_BASE + IMXRT_LCDIF_CTRL_CLR_OFFSET)
#define IMXRT_LCDIF_CTRL_TOG              (IMXRT_LCDIF_BASE + IMXRT_LCDIF_CTRL_TOG_OFFSET)
#define IMXRT_LCDIF_CTRL1                 (IMXRT_LCDIF_BASE + IMXRT_LCDIF_CTRL1_OFFSET)
#define IMXRT_LCDIF_CTRL1_SET             (IMXRT_LCDIF_BASE + IMXRT_LCDIF_CTRL1_SET_OFFSET)
#define IMXRT_LCDIF_CTRL1_CLR             (IMXRT_LCDIF_BASE + IMXRT_LCDIF_CTRL1_CLR_OFFSET)
#define IMXRT_LCDIF_CTRL1_TOG             (IMXRT_LCDIF_BASE + IMXRT_LCDIF_CTRL1_TOG_OFFSET)
#define IMXRT_LCDIF_CTRL2                 (IMXRT_LCDIF_BASE + IMXRT_LCDIF_CTRL2_OFFSET)
#define IMXRT_LCDIF_CTRL2_SET             (IMXRT_LCDIF_BASE + IMXRT_LCDIF_CTRL2_SET_OFFSET)
#define IMXRT_LCDIF_CTRL2_CLR             (IMXRT_LCDIF_BASE + IMXRT_LCDIF_CTRL2_CLR_OFFSET)
#define IMXRT_LCDIF_CTRL2_TOG             (IMXRT_LCDIF_BASE + IMXRT_LCDIF_CTRL2_TOG_OFFSET)
#define IMXRT_LCDIF_TRANSFER_COUNT        (IMXRT_LCDIF_BASE + IMXRT_LCDIF_TRANSFER_COUNT_OFFSET)
#define IMXRT_LCDIF_CUR_BUF               (IMXRT_LCDIF_BASE + IMXRT_LCDIF_CUR_BUF_OFFSET)
#define IMXRT_LCDIF_NEXT_BUF              (IMXRT_LCDIF_BASE + IMXRT_LCDIF_NEXT_BUF_OFFSET)
#define IMXRT_LCDIF_VDCTRL0               (IMXRT_LCDIF_BASE + IMXRT_LCDIF_VDCTRL0_OFFSET)
#define IMXRT_LCDIF_VDCTRL0_SET           (IMXRT_LCDIF_BASE + IMXRT_LCDIF_VDCTRL0_SET_OFFSET)
#define IMXRT_LCDIF_VDCTRL0_CLR           (IMXRT_LCDIF_BASE + IMXRT_LCDIF_VDCTRL0_CLR_OFFSET)
#define IMXRT_LCDIF_VDCTRL0_TOG           (IMXRT_LCDIF_BASE + IMXRT_LCDIF_VDCTRL0_TOG_OFFSET)
#define IMXRT_LCDIF_VDCTRL1               (IMXRT_LCDIF_BASE + IMXRT_LCDIF_VDCTRL1_OFFSET)
#define IMXRT_LCDIF_VDCTRL2               (IMXRT_LCDIF_BASE + IMXRT_LCDIF_VDCTRL2_OFFSET)
#define IMXRT_LCDIF_VDCTRL3               (IMXRT_LCDIF_BASE + IMXRT_LCDIF_VDCTRL3_OFFSET)
#define IMXRT_LCDIF_VDCTRL4               (IMXRT_LCDIF_BASE + IMXRT_LCDIF_VDCTRL4_OFFSET)
#define IMXRT_LCDIF_BM_ERROR_STAT         (IMXRT_LCDIF_BASE + IMXRT_LCDIF_BM_ERROR_STAT_OFFSET)
#define IMXRT_LCDIF_CRC_STAT              (IMXRT_LCDIF_BASE + IMXRT_LCDIF_CRC_STAT_OFFSET)
#define IMXRT_LCDIF_STAT                  (IMXRT_LCDIF_BASE + IMXRT_LCDIF_STAT_OFFSET)
#define IMXRT_LCDIF_THRES                 (IMXRT_LCDIF_BASE + IMXRT_LCDIF_THRES_OFFSET)

#define IMXRT_LCDIF_PIGEONCTRL0           (IMXRT_LCDIF_BASE + IMXRT_LCDIF_PIGEONCTRL0_OFFSET)
#define IMXRT_LCDIF_PIGEONCTRL0_SET       (IMXRT_LCDIF_BASE + IMXRT_LCDIF_PIGEONCTRL0_SET_OFFSET)
#define IMXRT_LCDIF_PIGEONCTRL0_CLR       (IMXRT_LCDIF_BASE + IMXRT_LCDIF_PIGEONCTRL0_CLR_OFFSET)
#define IMXRT_LCDIF_PIGEONCTRL0_TOG       (IMXRT_LCDIF_BASE + IMXRT_LCDIF_PIGEONCTRL0_TOG_OFFSET)
#define IMXRT_LCDIF_PIGEONCTRL1           (IMXRT_LCDIF_BASE + IMXRT_LCDIF_PIGEONCTRL1_OFFSET)
#define IMXRT_LCDIF_PIGEONCTRL1_SET       (IMXRT_LCDIF_BASE + IMXRT_LCDIF_PIGEONCTRL1_SET_OFFSET)
#define IMXRT_LCDIF_PIGEONCTRL1_CLR       (IMXRT_LCDIF_BASE + IMXRT_LCDIF_PIGEONCTRL1_CLR_OFFSET)
#define IMXRT_LCDIF_PIGEONCTRL1_TOG       (IMXRT_LCDIF_BASE + IMXRT_LCDIF_PIGEONCTRL1_TOG_OFFSET)
#define IMXRT_LCDIF_PIGEONCTRL2           (IMXRT_LCDIF_BASE + IMXRT_LCDIF_PIGEONCTRL2_OFFSET)
#define IMXRT_LCDIF_PIGEONCTRL2_SET       (IMXRT_LCDIF_BASE + IMXRT_LCDIF_PIGEONCTRL2_SET_OFFSET)
#define IMXRT_LCDIF_PIGEONCTRL2_CLR       (IMXRT_LCDIF_BASE + IMXRT_LCDIF_PIGEONCTRL2_CLR_OFFSET)
#define IMXRT_LCDIF_PIGEONCTRL2_TOG       (IMXRT_LCDIF_BASE + IMXRT_LCDIF_PIGEONCTRL2_TOG_OFFSET)

#define IMXRT_LCDIF_PIGEON0(n)            (IMXRT_LCDIF_BASE + IMXRT_LCDIF_PIGEON_OFFSET(n) + IMXRT_LCDIF_PIGEON0_OFFSET)
#define IMXRT_LCDIF_PIGEON1(n)            (IMXRT_LCDIF_BASE + IMXRT_LCDIF_PIGEON_OFFSET(n) + IMXRT_LCDIF_PIGEON1_OFFSET)
#define IMXRT_LCDIF_PIGEON2(n)            (IMXRT_LCDIF_BASE + IMXRT_LCDIF_PIGEON_OFFSET(n) + IMXRT_LCDIF_PIGEON2_OFFSET)

#define IMXRT_LCDIF_LUT_CTRL              (IMXRT_LCDIF_BASE + IMXRT_LCDIF_LUT_CTRL_OFFSET)
#define IMXRT_LCDIF_LUT0_ADDR             (IMXRT_LCDIF_BASE + IMXRT_LCDIF_LUT0_ADDR_OFFSET)
#define IMXRT_LCDIF_LUT0_DATA             (IMXRT_LCDIF_BASE + IMXRT_LCDIF_LUT0_DATA_OFFSET)
#define IMXRT_LCDIF_LUT1_ADDR             (IMXRT_LCDIF_BASE + IMXRT_LCDIF_LUT1_ADDR_OFFSET)
#define IMXRT_LCDIF_LUT1_DATA             (IMXRT_LCDIF_BASE + IMXRT_LCDIF_LUT1_DATA_OFFSET)
#define IMXRT_LCDIF_LUT_ENTRY_NUM         (256)

/* Register Bitfield Definitions ****************************************************************/

/* CTRL - LCDIF General Control Register */

#define LCDIF_CTRL_RUN_MASK                      (0x1U)
#define LCDIF_CTRL_RUN_SHIFT                     (0U)
#define LCDIF_CTRL_RUN(x)                        (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_RUN_SHIFT)) & LCDIF_CTRL_RUN_MASK)
#define LCDIF_CTRL_DATA_FORMAT_24_BIT_MASK       (0x2U)
#define LCDIF_CTRL_DATA_FORMAT_24_BIT_SHIFT      (1U)
/* DATA_FORMAT_24_BIT
 *  0b0..Data input to the block is in 24 bpp format, such that all RGB 888 data is contained in 24 bits.
 *  0b1..Data input to the block is actually RGB 18 bpp, but there is 1 color per byte, hence the upper 2 bits in each byte do not contain any useful data, and should be dropped.
 */
#define LCDIF_CTRL_DATA_FORMAT_24_BIT(x)         (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_DATA_FORMAT_24_BIT_SHIFT)) & LCDIF_CTRL_DATA_FORMAT_24_BIT_MASK)
#define LCDIF_CTRL_DATA_FORMAT_18_BIT_MASK       (0x4U)
#define LCDIF_CTRL_DATA_FORMAT_18_BIT_SHIFT      (2U)
/* DATA_FORMAT_18_BIT
 *  0b0..Data input to the block is in 18 bpp format, such that lower 18 bits contain RGB 666 and upper 14 bits do not contain any useful data.
 *  0b1..Data input to the block is in 18 bpp format, such that upper 18 bits contain RGB 666 and lower 14 bits do not contain any useful data.
 */
#define LCDIF_CTRL_DATA_FORMAT_18_BIT(x)         (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_DATA_FORMAT_18_BIT_SHIFT)) & LCDIF_CTRL_DATA_FORMAT_18_BIT_MASK)
#define LCDIF_CTRL_DATA_FORMAT_16_BIT_MASK       (0x8U)
#define LCDIF_CTRL_DATA_FORMAT_16_BIT_SHIFT      (3U)
#define LCDIF_CTRL_DATA_FORMAT_16_BIT(x)         (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_DATA_FORMAT_16_BIT_SHIFT)) & LCDIF_CTRL_DATA_FORMAT_16_BIT_MASK)
#define LCDIF_CTRL_RSRVD0_MASK                   (0x10U)
#define LCDIF_CTRL_RSRVD0_SHIFT                  (4U)
#define LCDIF_CTRL_RSRVD0(x)                     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_RSRVD0_SHIFT)) & LCDIF_CTRL_RSRVD0_MASK)
#define LCDIF_CTRL_MASTER_MASK                   (0x20U)
#define LCDIF_CTRL_MASTER_SHIFT                  (5U)
#define LCDIF_CTRL_MASTER(x)                     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_MASTER_SHIFT)) & LCDIF_CTRL_MASTER_MASK)
#define LCDIF_CTRL_ENABLE_PXP_HANDSHAKE_MASK     (0x40U)
#define LCDIF_CTRL_ENABLE_PXP_HANDSHAKE_SHIFT    (6U)
#define LCDIF_CTRL_ENABLE_PXP_HANDSHAKE(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_ENABLE_PXP_HANDSHAKE_SHIFT)) & LCDIF_CTRL_ENABLE_PXP_HANDSHAKE_MASK)
#define LCDIF_CTRL_WORD_LENGTH_MASK              (0x300U)
#define LCDIF_CTRL_WORD_LENGTH_SHIFT             (8U)
/* WORD_LENGTH
 *  0b00..Input data is 16 bits per pixel.
 *  0b01..Input data is 8 bits wide.
 *  0b10..Input data is 18 bits per pixel.
 *  0b11..Input data is 24 bits per pixel.
 */
#define LCDIF_CTRL_WORD_LENGTH(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_WORD_LENGTH_SHIFT)) & LCDIF_CTRL_WORD_LENGTH_MASK)
#define LCDIF_CTRL_LCD_DATABUS_WIDTH_MASK        (0xc00U)
#define LCDIF_CTRL_LCD_DATABUS_WIDTH_SHIFT       (10U)

/* LCD_DATABUS_WIDTH
 *  0b00..16-bit data bus mode.
 *  0b01..8-bit data bus mode.
 *  0b10..18-bit data bus mode.
 *  0b11..24-bit data bus mode.
 */

#define LCDIF_CTRL_LCD_DATABUS_WIDTH(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_LCD_DATABUS_WIDTH_SHIFT)) & LCDIF_CTRL_LCD_DATABUS_WIDTH_MASK)
#define LCDIF_CTRL_CSC_DATA_SWIZZLE_MASK         (0x3000U)
#define LCDIF_CTRL_CSC_DATA_SWIZZLE_SHIFT        (12U)

/* CSC_DATA_SWIZZLE
 *  0b00..No byte swapping.(Little endian)
 *  0b00..Little Endian byte ordering (same as NO_SWAP).
 *  0b01..Big Endian swap (swap bytes 0,3 and 1,2).
 *  0b01..Swizzle all bytes, swap bytes 0,3 and 1,2 (aka Big Endian).
 *  0b10..Swap half-words.
 *  0b11..Swap bytes within each half-word.
 */

#define LCDIF_CTRL_CSC_DATA_SWIZZLE(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_CSC_DATA_SWIZZLE_SHIFT)) & LCDIF_CTRL_CSC_DATA_SWIZZLE_MASK)
#define LCDIF_CTRL_INPUT_DATA_SWIZZLE_MASK       (0xc000U)
#define LCDIF_CTRL_INPUT_DATA_SWIZZLE_SHIFT      (14U)

/* INPUT_DATA_SWIZZLE
 *  0b00..No byte swapping.(Little endian)
 *  0b00..Little Endian byte ordering (same as NO_SWAP).
 *  0b01..Big Endian swap (swap bytes 0,3 and 1,2).
 *  0b01..Swizzle all bytes, swap bytes 0,3 and 1,2 (aka Big Endian).
 *  0b10..Swap half-words.
 *  0b11..Swap bytes within each half-word.
 */

#define LCDIF_CTRL_INPUT_DATA_SWIZZLE(x)         (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_INPUT_DATA_SWIZZLE_SHIFT)) & LCDIF_CTRL_INPUT_DATA_SWIZZLE_MASK)
#define LCDIF_CTRL_DOTCLK_MODE_MASK              (0x20000U)
#define LCDIF_CTRL_DOTCLK_MODE_SHIFT             (17U)
#define LCDIF_CTRL_DOTCLK_MODE(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_DOTCLK_MODE_SHIFT)) & LCDIF_CTRL_DOTCLK_MODE_MASK)
#define LCDIF_CTRL_BYPASS_COUNT_MASK             (0x80000U)
#define LCDIF_CTRL_BYPASS_COUNT_SHIFT            (19U)
#define LCDIF_CTRL_BYPASS_COUNT(x)               (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_BYPASS_COUNT_SHIFT)) & LCDIF_CTRL_BYPASS_COUNT_MASK)
#define LCDIF_CTRL_SHIFT_NUM_BITS_MASK           (0x3e00000U)
#define LCDIF_CTRL_SHIFT_NUM_BITS_SHIFT          (21U)
#define LCDIF_CTRL_SHIFT_NUM_BITS(x)             (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_SHIFT_NUM_BITS_SHIFT)) & LCDIF_CTRL_SHIFT_NUM_BITS_MASK)
#define LCDIF_CTRL_DATA_SHIFT_DIR_MASK           (0x4000000U)
#define LCDIF_CTRL_DATA_SHIFT_DIR_SHIFT          (26U)

/* DATA_SHIFT_DIR
 *  0b0..Data to be transmitted is shifted LEFT by SHIFT_NUM_BITS bits.
 *  0b1..Data to be transmitted is shifted RIGHT by SHIFT_NUM_BITS bits.
 */

#define LCDIF_CTRL_DATA_SHIFT_DIR(x)             (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_DATA_SHIFT_DIR_SHIFT)) & LCDIF_CTRL_DATA_SHIFT_DIR_MASK)
#define LCDIF_CTRL_CLKGATE_MASK                  (0x40000000U)
#define LCDIF_CTRL_CLKGATE_SHIFT                 (30U)
#define LCDIF_CTRL_CLKGATE(x)                    (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_CLKGATE_SHIFT)) & LCDIF_CTRL_CLKGATE_MASK)
#define LCDIF_CTRL_SFTRST_MASK                   (0x80000000U)
#define LCDIF_CTRL_SFTRST_SHIFT                  (31U)
#define LCDIF_CTRL_SFTRST(x)                     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_SFTRST_SHIFT)) & LCDIF_CTRL_SFTRST_MASK)

/* CTRL_SET - LCDIF General Control Register */

#define LCDIF_CTRL_SET_RUN_MASK                  (0x1U)
#define LCDIF_CTRL_SET_RUN_SHIFT                 (0U)
#define LCDIF_CTRL_SET_RUN(x)                    (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_SET_RUN_SHIFT)) & LCDIF_CTRL_SET_RUN_MASK)
#define LCDIF_CTRL_SET_DATA_FORMAT_24_BIT_MASK   (0x2U)
#define LCDIF_CTRL_SET_DATA_FORMAT_24_BIT_SHIFT  (1U)

/* DATA_FORMAT_24_BIT
 *  0b0..Data input to the block is in 24 bpp format, such that all RGB 888 data is contained in 24 bits.
 *  0b1..Data input to the block is actually RGB 18 bpp, but there is 1 color per byte, hence the upper 2 bits in each byte do not contain any useful data, and should be dropped.
 */

#define LCDIF_CTRL_SET_DATA_FORMAT_24_BIT(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_SET_DATA_FORMAT_24_BIT_SHIFT)) & LCDIF_CTRL_SET_DATA_FORMAT_24_BIT_MASK)
#define LCDIF_CTRL_SET_DATA_FORMAT_18_BIT_MASK   (0x4U)
#define LCDIF_CTRL_SET_DATA_FORMAT_18_BIT_SHIFT  (2U)

/* DATA_FORMAT_18_BIT
 *  0b0..Data input to the block is in 18 bpp format, such that lower 18 bits contain RGB 666 and upper 14 bits do not contain any useful data.
 *  0b1..Data input to the block is in 18 bpp format, such that upper 18 bits contain RGB 666 and lower 14 bits do not contain any useful data.
 */

#define LCDIF_CTRL_SET_DATA_FORMAT_18_BIT(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_SET_DATA_FORMAT_18_BIT_SHIFT)) & LCDIF_CTRL_SET_DATA_FORMAT_18_BIT_MASK)
#define LCDIF_CTRL_SET_DATA_FORMAT_16_BIT_MASK   (0x8U)
#define LCDIF_CTRL_SET_DATA_FORMAT_16_BIT_SHIFT  (3U)
#define LCDIF_CTRL_SET_DATA_FORMAT_16_BIT(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_SET_DATA_FORMAT_16_BIT_SHIFT)) & LCDIF_CTRL_SET_DATA_FORMAT_16_BIT_MASK)
#define LCDIF_CTRL_SET_RSRVD0_MASK               (0x10U)
#define LCDIF_CTRL_SET_RSRVD0_SHIFT              (4U)
#define LCDIF_CTRL_SET_RSRVD0(x)                 (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_SET_RSRVD0_SHIFT)) & LCDIF_CTRL_SET_RSRVD0_MASK)
#define LCDIF_CTRL_SET_MASTER_MASK               (0x20U)
#define LCDIF_CTRL_SET_MASTER_SHIFT              (5U)
#define LCDIF_CTRL_SET_MASTER(x)                 (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_SET_MASTER_SHIFT)) & LCDIF_CTRL_SET_MASTER_MASK)
#define LCDIF_CTRL_SET_ENABLE_PXP_HANDSHAKE_MASK (0x40U)
#define LCDIF_CTRL_SET_ENABLE_PXP_HANDSHAKE_SHIFT (6U)
#define LCDIF_CTRL_SET_ENABLE_PXP_HANDSHAKE(x)   (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_SET_ENABLE_PXP_HANDSHAKE_SHIFT)) & LCDIF_CTRL_SET_ENABLE_PXP_HANDSHAKE_MASK)
#define LCDIF_CTRL_SET_WORD_LENGTH_MASK          (0x300U)
#define LCDIF_CTRL_SET_WORD_LENGTH_SHIFT         (8U)

/* WORD_LENGTH
 *  0b00..Input data is 16 bits per pixel.
 *  0b01..Input data is 8 bits wide.
 *  0b10..Input data is 18 bits per pixel.
 *  0b11..Input data is 24 bits per pixel.
 */

#define LCDIF_CTRL_SET_WORD_LENGTH(x)            (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_SET_WORD_LENGTH_SHIFT)) & LCDIF_CTRL_SET_WORD_LENGTH_MASK)
#define LCDIF_CTRL_SET_LCD_DATABUS_WIDTH_MASK    (0xc00U)
#define LCDIF_CTRL_SET_LCD_DATABUS_WIDTH_SHIFT   (10U)

/* LCD_DATABUS_WIDTH
 *  0b00..16-bit data bus mode.
 *  0b01..8-bit data bus mode.
 *  0b10..18-bit data bus mode.
 *  0b11..24-bit data bus mode.
 */

#define LCDIF_CTRL_SET_LCD_DATABUS_WIDTH(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_SET_LCD_DATABUS_WIDTH_SHIFT)) & LCDIF_CTRL_SET_LCD_DATABUS_WIDTH_MASK)
#define LCDIF_CTRL_SET_CSC_DATA_SWIZZLE_MASK     (0x3000U)
#define LCDIF_CTRL_SET_CSC_DATA_SWIZZLE_SHIFT    (12U)

/* CSC_DATA_SWIZZLE
 *  0b00..No byte swapping.(Little endian)
 *  0b00..Little Endian byte ordering (same as NO_SWAP).
 *  0b01..Big Endian swap (swap bytes 0,3 and 1,2).
 *  0b01..Swizzle all bytes, swap bytes 0,3 and 1,2 (aka Big Endian).
 *  0b10..Swap half-words.
 *  0b11..Swap bytes within each half-word.
 */

#define LCDIF_CTRL_SET_CSC_DATA_SWIZZLE(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_SET_CSC_DATA_SWIZZLE_SHIFT)) & LCDIF_CTRL_SET_CSC_DATA_SWIZZLE_MASK)
#define LCDIF_CTRL_SET_INPUT_DATA_SWIZZLE_MASK   (0xc000U)
#define LCDIF_CTRL_SET_INPUT_DATA_SWIZZLE_SHIFT  (14U)

/* INPUT_DATA_SWIZZLE
 *  0b00..No byte swapping.(Little endian)
 *  0b00..Little Endian byte ordering (same as NO_SWAP).
 *  0b01..Big Endian swap (swap bytes 0,3 and 1,2).
 *  0b01..Swizzle all bytes, swap bytes 0,3 and 1,2 (aka Big Endian).
 *  0b10..Swap half-words.
 *  0b11..Swap bytes within each half-word.
 */

#define LCDIF_CTRL_SET_INPUT_DATA_SWIZZLE(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_SET_INPUT_DATA_SWIZZLE_SHIFT)) & LCDIF_CTRL_SET_INPUT_DATA_SWIZZLE_MASK)
#define LCDIF_CTRL_SET_DOTCLK_MODE_MASK          (0x20000U)
#define LCDIF_CTRL_SET_DOTCLK_MODE_SHIFT         (17U)
#define LCDIF_CTRL_SET_DOTCLK_MODE(x)            (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_SET_DOTCLK_MODE_SHIFT)) & LCDIF_CTRL_SET_DOTCLK_MODE_MASK)
#define LCDIF_CTRL_SET_BYPASS_COUNT_MASK         (0x80000U)
#define LCDIF_CTRL_SET_BYPASS_COUNT_SHIFT        (19U)
#define LCDIF_CTRL_SET_BYPASS_COUNT(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_SET_BYPASS_COUNT_SHIFT)) & LCDIF_CTRL_SET_BYPASS_COUNT_MASK)
#define LCDIF_CTRL_SET_SHIFT_NUM_BITS_MASK       (0x3e00000U)
#define LCDIF_CTRL_SET_SHIFT_NUM_BITS_SHIFT      (21U)
#define LCDIF_CTRL_SET_SHIFT_NUM_BITS(x)         (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_SET_SHIFT_NUM_BITS_SHIFT)) & LCDIF_CTRL_SET_SHIFT_NUM_BITS_MASK)
#define LCDIF_CTRL_SET_DATA_SHIFT_DIR_MASK       (0x4000000U)
#define LCDIF_CTRL_SET_DATA_SHIFT_DIR_SHIFT      (26U)

/* DATA_SHIFT_DIR
 *  0b0..Data to be transmitted is shifted LEFT by SHIFT_NUM_BITS bits.
 *  0b1..Data to be transmitted is shifted RIGHT by SHIFT_NUM_BITS bits.
 */

#define LCDIF_CTRL_SET_DATA_SHIFT_DIR(x)         (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_SET_DATA_SHIFT_DIR_SHIFT)) & LCDIF_CTRL_SET_DATA_SHIFT_DIR_MASK)
#define LCDIF_CTRL_SET_CLKGATE_MASK              (0x40000000U)
#define LCDIF_CTRL_SET_CLKGATE_SHIFT             (30U)
#define LCDIF_CTRL_SET_CLKGATE(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_SET_CLKGATE_SHIFT)) & LCDIF_CTRL_SET_CLKGATE_MASK)
#define LCDIF_CTRL_SET_SFTRST_MASK               (0x80000000U)
#define LCDIF_CTRL_SET_SFTRST_SHIFT              (31U)
#define LCDIF_CTRL_SET_SFTRST(x)                 (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_SET_SFTRST_SHIFT)) & LCDIF_CTRL_SET_SFTRST_MASK)

/* CTRL_CLR - LCDIF General Control Register */

#define LCDIF_CTRL_CLR_RUN_MASK                  (0x1U)
#define LCDIF_CTRL_CLR_RUN_SHIFT                 (0U)
#define LCDIF_CTRL_CLR_RUN(x)                    (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_CLR_RUN_SHIFT)) & LCDIF_CTRL_CLR_RUN_MASK)
#define LCDIF_CTRL_CLR_DATA_FORMAT_24_BIT_MASK   (0x2U)
#define LCDIF_CTRL_CLR_DATA_FORMAT_24_BIT_SHIFT  (1U)

/* DATA_FORMAT_24_BIT
 *  0b0..Data input to the block is in 24 bpp format, such that all RGB 888 data is contained in 24 bits.
 *  0b1..Data input to the block is actually RGB 18 bpp, but there is 1 color per byte, hence the upper 2 bits in each byte do not contain any useful data, and should be dropped.
 */

#define LCDIF_CTRL_CLR_DATA_FORMAT_24_BIT(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_CLR_DATA_FORMAT_24_BIT_SHIFT)) & LCDIF_CTRL_CLR_DATA_FORMAT_24_BIT_MASK)
#define LCDIF_CTRL_CLR_DATA_FORMAT_18_BIT_MASK   (0x4U)
#define LCDIF_CTRL_CLR_DATA_FORMAT_18_BIT_SHIFT  (2U)

/* DATA_FORMAT_18_BIT
 *  0b0..Data input to the block is in 18 bpp format, such that lower 18 bits contain RGB 666 and upper 14 bits do not contain any useful data.
 *  0b1..Data input to the block is in 18 bpp format, such that upper 18 bits contain RGB 666 and lower 14 bits do not contain any useful data.
 */

#define LCDIF_CTRL_CLR_DATA_FORMAT_18_BIT(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_CLR_DATA_FORMAT_18_BIT_SHIFT)) & LCDIF_CTRL_CLR_DATA_FORMAT_18_BIT_MASK)
#define LCDIF_CTRL_CLR_DATA_FORMAT_16_BIT_MASK   (0x8U)
#define LCDIF_CTRL_CLR_DATA_FORMAT_16_BIT_SHIFT  (3U)
#define LCDIF_CTRL_CLR_DATA_FORMAT_16_BIT(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_CLR_DATA_FORMAT_16_BIT_SHIFT)) & LCDIF_CTRL_CLR_DATA_FORMAT_16_BIT_MASK)
#define LCDIF_CTRL_CLR_RSRVD0_MASK               (0x10U)
#define LCDIF_CTRL_CLR_RSRVD0_SHIFT              (4U)
#define LCDIF_CTRL_CLR_RSRVD0(x)                 (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_CLR_RSRVD0_SHIFT)) & LCDIF_CTRL_CLR_RSRVD0_MASK)
#define LCDIF_CTRL_CLR_MASTER_MASK               (0x20U)
#define LCDIF_CTRL_CLR_MASTER_SHIFT              (5U)
#define LCDIF_CTRL_CLR_MASTER(x)                 (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_CLR_MASTER_SHIFT)) & LCDIF_CTRL_CLR_MASTER_MASK)
#define LCDIF_CTRL_CLR_ENABLE_PXP_HANDSHAKE_MASK (0x40U)
#define LCDIF_CTRL_CLR_ENABLE_PXP_HANDSHAKE_SHIFT (6U)
#define LCDIF_CTRL_CLR_ENABLE_PXP_HANDSHAKE(x)   (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_CLR_ENABLE_PXP_HANDSHAKE_SHIFT)) & LCDIF_CTRL_CLR_ENABLE_PXP_HANDSHAKE_MASK)
#define LCDIF_CTRL_CLR_WORD_LENGTH_MASK          (0x300U)
#define LCDIF_CTRL_CLR_WORD_LENGTH_SHIFT         (8U)

/* WORD_LENGTH
 *  0b00..Input data is 16 bits per pixel.
 *  0b01..Input data is 8 bits wide.
 *  0b10..Input data is 18 bits per pixel.
 *  0b11..Input data is 24 bits per pixel.
 */

#define LCDIF_CTRL_CLR_WORD_LENGTH(x)            (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_CLR_WORD_LENGTH_SHIFT)) & LCDIF_CTRL_CLR_WORD_LENGTH_MASK)
#define LCDIF_CTRL_CLR_LCD_DATABUS_WIDTH_MASK    (0xc00U)
#define LCDIF_CTRL_CLR_LCD_DATABUS_WIDTH_SHIFT   (10U)

/* LCD_DATABUS_WIDTH
 *  0b00..16-bit data bus mode.
 *  0b01..8-bit data bus mode.
 *  0b10..18-bit data bus mode.
 *  0b11..24-bit data bus mode.
 */

#define LCDIF_CTRL_CLR_LCD_DATABUS_WIDTH(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_CLR_LCD_DATABUS_WIDTH_SHIFT)) & LCDIF_CTRL_CLR_LCD_DATABUS_WIDTH_MASK)
#define LCDIF_CTRL_CLR_CSC_DATA_SWIZZLE_MASK     (0x3000U)
#define LCDIF_CTRL_CLR_CSC_DATA_SWIZZLE_SHIFT    (12U)

/* CSC_DATA_SWIZZLE
 *  0b00..No byte swapping.(Little endian)
 *  0b00..Little Endian byte ordering (same as NO_SWAP).
 *  0b01..Big Endian swap (swap bytes 0,3 and 1,2).
 *  0b01..Swizzle all bytes, swap bytes 0,3 and 1,2 (aka Big Endian).
 *  0b10..Swap half-words.
 *  0b11..Swap bytes within each half-word.
 */

#define LCDIF_CTRL_CLR_CSC_DATA_SWIZZLE(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_CLR_CSC_DATA_SWIZZLE_SHIFT)) & LCDIF_CTRL_CLR_CSC_DATA_SWIZZLE_MASK)
#define LCDIF_CTRL_CLR_INPUT_DATA_SWIZZLE_MASK   (0xc000U)
#define LCDIF_CTRL_CLR_INPUT_DATA_SWIZZLE_SHIFT  (14U)

/* INPUT_DATA_SWIZZLE
 *  0b00..No byte swapping.(Little endian)
 *  0b00..Little Endian byte ordering (same as NO_SWAP).
 *  0b01..Big Endian swap (swap bytes 0,3 and 1,2).
 *  0b01..Swizzle all bytes, swap bytes 0,3 and 1,2 (aka Big Endian).
 *  0b10..Swap half-words.
 *  0b11..Swap bytes within each half-word.
 */

#define LCDIF_CTRL_CLR_INPUT_DATA_SWIZZLE(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_CLR_INPUT_DATA_SWIZZLE_SHIFT)) & LCDIF_CTRL_CLR_INPUT_DATA_SWIZZLE_MASK)
#define LCDIF_CTRL_CLR_DOTCLK_MODE_MASK          (0x20000U)
#define LCDIF_CTRL_CLR_DOTCLK_MODE_SHIFT         (17U)
#define LCDIF_CTRL_CLR_DOTCLK_MODE(x)            (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_CLR_DOTCLK_MODE_SHIFT)) & LCDIF_CTRL_CLR_DOTCLK_MODE_MASK)
#define LCDIF_CTRL_CLR_BYPASS_COUNT_MASK         (0x80000U)
#define LCDIF_CTRL_CLR_BYPASS_COUNT_SHIFT        (19U)
#define LCDIF_CTRL_CLR_BYPASS_COUNT(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_CLR_BYPASS_COUNT_SHIFT)) & LCDIF_CTRL_CLR_BYPASS_COUNT_MASK)
#define LCDIF_CTRL_CLR_SHIFT_NUM_BITS_MASK       (0x3e00000U)
#define LCDIF_CTRL_CLR_SHIFT_NUM_BITS_SHIFT      (21U)
#define LCDIF_CTRL_CLR_SHIFT_NUM_BITS(x)         (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_CLR_SHIFT_NUM_BITS_SHIFT)) & LCDIF_CTRL_CLR_SHIFT_NUM_BITS_MASK)
#define LCDIF_CTRL_CLR_DATA_SHIFT_DIR_MASK       (0x4000000U)
#define LCDIF_CTRL_CLR_DATA_SHIFT_DIR_SHIFT      (26U)

/* DATA_SHIFT_DIR
 *  0b0..Data to be transmitted is shifted LEFT by SHIFT_NUM_BITS bits.
 *  0b1..Data to be transmitted is shifted RIGHT by SHIFT_NUM_BITS bits.
 */

#define LCDIF_CTRL_CLR_DATA_SHIFT_DIR(x)         (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_CLR_DATA_SHIFT_DIR_SHIFT)) & LCDIF_CTRL_CLR_DATA_SHIFT_DIR_MASK)
#define LCDIF_CTRL_CLR_CLKGATE_MASK              (0x40000000U)
#define LCDIF_CTRL_CLR_CLKGATE_SHIFT             (30U)
#define LCDIF_CTRL_CLR_CLKGATE(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_CLR_CLKGATE_SHIFT)) & LCDIF_CTRL_CLR_CLKGATE_MASK)
#define LCDIF_CTRL_CLR_SFTRST_MASK               (0x80000000U)
#define LCDIF_CTRL_CLR_SFTRST_SHIFT              (31U)
#define LCDIF_CTRL_CLR_SFTRST(x)                 (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_CLR_SFTRST_SHIFT)) & LCDIF_CTRL_CLR_SFTRST_MASK)

/* CTRL_TOG - LCDIF General Control Register */

#define LCDIF_CTRL_TOG_RUN_MASK                  (0x1U)
#define LCDIF_CTRL_TOG_RUN_SHIFT                 (0U)
#define LCDIF_CTRL_TOG_RUN(x)                    (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_TOG_RUN_SHIFT)) & LCDIF_CTRL_TOG_RUN_MASK)
#define LCDIF_CTRL_TOG_DATA_FORMAT_24_BIT_MASK   (0x2U)
#define LCDIF_CTRL_TOG_DATA_FORMAT_24_BIT_SHIFT  (1U)

/* DATA_FORMAT_24_BIT
 *  0b0..Data input to the block is in 24 bpp format, such that all RGB 888 data is contained in 24 bits.
 *  0b1..Data input to the block is actually RGB 18 bpp, but there is 1 color per byte, hence the upper 2 bits in each byte do not contain any useful data, and should be dropped.
 */

#define LCDIF_CTRL_TOG_DATA_FORMAT_24_BIT(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_TOG_DATA_FORMAT_24_BIT_SHIFT)) & LCDIF_CTRL_TOG_DATA_FORMAT_24_BIT_MASK)
#define LCDIF_CTRL_TOG_DATA_FORMAT_18_BIT_MASK   (0x4U)
#define LCDIF_CTRL_TOG_DATA_FORMAT_18_BIT_SHIFT  (2U)

/* DATA_FORMAT_18_BIT
 *  0b0..Data input to the block is in 18 bpp format, such that lower 18 bits contain RGB 666 and upper 14 bits do not contain any useful data.
 *  0b1..Data input to the block is in 18 bpp format, such that upper 18 bits contain RGB 666 and lower 14 bits do not contain any useful data.
 */

#define LCDIF_CTRL_TOG_DATA_FORMAT_18_BIT(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_TOG_DATA_FORMAT_18_BIT_SHIFT)) & LCDIF_CTRL_TOG_DATA_FORMAT_18_BIT_MASK)
#define LCDIF_CTRL_TOG_DATA_FORMAT_16_BIT_MASK   (0x8U)
#define LCDIF_CTRL_TOG_DATA_FORMAT_16_BIT_SHIFT  (3U)
#define LCDIF_CTRL_TOG_DATA_FORMAT_16_BIT(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_TOG_DATA_FORMAT_16_BIT_SHIFT)) & LCDIF_CTRL_TOG_DATA_FORMAT_16_BIT_MASK)
#define LCDIF_CTRL_TOG_RSRVD0_MASK               (0x10U)
#define LCDIF_CTRL_TOG_RSRVD0_SHIFT              (4U)
#define LCDIF_CTRL_TOG_RSRVD0(x)                 (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_TOG_RSRVD0_SHIFT)) & LCDIF_CTRL_TOG_RSRVD0_MASK)
#define LCDIF_CTRL_TOG_MASTER_MASK               (0x20U)
#define LCDIF_CTRL_TOG_MASTER_SHIFT              (5U)
#define LCDIF_CTRL_TOG_MASTER(x)                 (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_TOG_MASTER_SHIFT)) & LCDIF_CTRL_TOG_MASTER_MASK)
#define LCDIF_CTRL_TOG_ENABLE_PXP_HANDSHAKE_MASK (0x40U)
#define LCDIF_CTRL_TOG_ENABLE_PXP_HANDSHAKE_SHIFT (6U)
#define LCDIF_CTRL_TOG_ENABLE_PXP_HANDSHAKE(x)   (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_TOG_ENABLE_PXP_HANDSHAKE_SHIFT)) & LCDIF_CTRL_TOG_ENABLE_PXP_HANDSHAKE_MASK)
#define LCDIF_CTRL_TOG_WORD_LENGTH_MASK          (0x300U)
#define LCDIF_CTRL_TOG_WORD_LENGTH_SHIFT         (8U)

/* WORD_LENGTH
 *  0b00..Input data is 16 bits per pixel.
 *  0b01..Input data is 8 bits wide.
 *  0b10..Input data is 18 bits per pixel.
 *  0b11..Input data is 24 bits per pixel.
 */

#define LCDIF_CTRL_TOG_WORD_LENGTH(x)            (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_TOG_WORD_LENGTH_SHIFT)) & LCDIF_CTRL_TOG_WORD_LENGTH_MASK)
#define LCDIF_CTRL_TOG_LCD_DATABUS_WIDTH_MASK    (0xc00U)
#define LCDIF_CTRL_TOG_LCD_DATABUS_WIDTH_SHIFT   (10U)

/* LCD_DATABUS_WIDTH
 *  0b00..16-bit data bus mode.
 *  0b01..8-bit data bus mode.
 *  0b10..18-bit data bus mode.
 *  0b11..24-bit data bus mode.
 */

#define LCDIF_CTRL_TOG_LCD_DATABUS_WIDTH(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_TOG_LCD_DATABUS_WIDTH_SHIFT)) & LCDIF_CTRL_TOG_LCD_DATABUS_WIDTH_MASK)
#define LCDIF_CTRL_TOG_CSC_DATA_SWIZZLE_MASK     (0x3000U)
#define LCDIF_CTRL_TOG_CSC_DATA_SWIZZLE_SHIFT    (12U)

/* CSC_DATA_SWIZZLE
 *  0b00..No byte swapping.(Little endian)
 *  0b00..Little Endian byte ordering (same as NO_SWAP).
 *  0b01..Big Endian swap (swap bytes 0,3 and 1,2).
 *  0b01..Swizzle all bytes, swap bytes 0,3 and 1,2 (aka Big Endian).
 *  0b10..Swap half-words.
 *  0b11..Swap bytes within each half-word.
 */

#define LCDIF_CTRL_TOG_CSC_DATA_SWIZZLE(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_TOG_CSC_DATA_SWIZZLE_SHIFT)) & LCDIF_CTRL_TOG_CSC_DATA_SWIZZLE_MASK)
#define LCDIF_CTRL_TOG_INPUT_DATA_SWIZZLE_MASK   (0xc000U)
#define LCDIF_CTRL_TOG_INPUT_DATA_SWIZZLE_SHIFT  (14U)

/* INPUT_DATA_SWIZZLE
 *  0b00..No byte swapping.(Little endian)
 *  0b00..Little Endian byte ordering (same as NO_SWAP).
 *  0b01..Big Endian swap (swap bytes 0,3 and 1,2).
 *  0b01..Swizzle all bytes, swap bytes 0,3 and 1,2 (aka Big Endian).
 *  0b10..Swap half-words.
 *  0b11..Swap bytes within each half-word.
 */

#define LCDIF_CTRL_TOG_INPUT_DATA_SWIZZLE(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_TOG_INPUT_DATA_SWIZZLE_SHIFT)) & LCDIF_CTRL_TOG_INPUT_DATA_SWIZZLE_MASK)
#define LCDIF_CTRL_TOG_DOTCLK_MODE_MASK          (0x20000U)
#define LCDIF_CTRL_TOG_DOTCLK_MODE_SHIFT         (17U)
#define LCDIF_CTRL_TOG_DOTCLK_MODE(x)            (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_TOG_DOTCLK_MODE_SHIFT)) & LCDIF_CTRL_TOG_DOTCLK_MODE_MASK)
#define LCDIF_CTRL_TOG_BYPASS_COUNT_MASK         (0x80000U)
#define LCDIF_CTRL_TOG_BYPASS_COUNT_SHIFT        (19U)
#define LCDIF_CTRL_TOG_BYPASS_COUNT(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_TOG_BYPASS_COUNT_SHIFT)) & LCDIF_CTRL_TOG_BYPASS_COUNT_MASK)
#define LCDIF_CTRL_TOG_SHIFT_NUM_BITS_MASK       (0x3e00000U)
#define LCDIF_CTRL_TOG_SHIFT_NUM_BITS_SHIFT      (21U)
#define LCDIF_CTRL_TOG_SHIFT_NUM_BITS(x)         (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_TOG_SHIFT_NUM_BITS_SHIFT)) & LCDIF_CTRL_TOG_SHIFT_NUM_BITS_MASK)
#define LCDIF_CTRL_TOG_DATA_SHIFT_DIR_MASK       (0x4000000U)
#define LCDIF_CTRL_TOG_DATA_SHIFT_DIR_SHIFT      (26U)

/* DATA_SHIFT_DIR
 *  0b0..Data to be transmitted is shifted LEFT by SHIFT_NUM_BITS bits.
 *  0b1..Data to be transmitted is shifted RIGHT by SHIFT_NUM_BITS bits.
 */

#define LCDIF_CTRL_TOG_DATA_SHIFT_DIR(x)         (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_TOG_DATA_SHIFT_DIR_SHIFT)) & LCDIF_CTRL_TOG_DATA_SHIFT_DIR_MASK)
#define LCDIF_CTRL_TOG_CLKGATE_MASK              (0x40000000U)
#define LCDIF_CTRL_TOG_CLKGATE_SHIFT             (30U)
#define LCDIF_CTRL_TOG_CLKGATE(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_TOG_CLKGATE_SHIFT)) & LCDIF_CTRL_TOG_CLKGATE_MASK)
#define LCDIF_CTRL_TOG_SFTRST_MASK               (0x80000000U)
#define LCDIF_CTRL_TOG_SFTRST_SHIFT              (31U)
#define LCDIF_CTRL_TOG_SFTRST(x)                 (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL_TOG_SFTRST_SHIFT)) & LCDIF_CTRL_TOG_SFTRST_MASK)

/* CTRL1 - LCDIF General Control1 Register */

#define LCDIF_CTRL1_RSRVD0_MASK                  (0xf8U)
#define LCDIF_CTRL1_RSRVD0_SHIFT                 (3U)
#define LCDIF_CTRL1_RSRVD0(x)                    (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_RSRVD0_SHIFT)) & LCDIF_CTRL1_RSRVD0_MASK)
#define LCDIF_CTRL1_VSYNC_EDGE_IRQ_MASK          (0x100U)
#define LCDIF_CTRL1_VSYNC_EDGE_IRQ_SHIFT         (8U)

/* VSYNC_EDGE_IRQ
 *  0b0..No Interrupt Request Pending.
 *  0b1..Interrupt Request Pending.
 */

#define LCDIF_CTRL1_VSYNC_EDGE_IRQ(x)            (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_VSYNC_EDGE_IRQ_SHIFT)) & LCDIF_CTRL1_VSYNC_EDGE_IRQ_MASK)
#define LCDIF_CTRL1_CUR_FRAME_DONE_IRQ_MASK      (0x200U)
#define LCDIF_CTRL1_CUR_FRAME_DONE_IRQ_SHIFT     (9U)

/* CUR_FRAME_DONE_IRQ
 *  0b0..No Interrupt Request Pending.
 *  0b1..Interrupt Request Pending.
 */

#define LCDIF_CTRL1_CUR_FRAME_DONE_IRQ(x)        (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CUR_FRAME_DONE_IRQ_SHIFT)) & LCDIF_CTRL1_CUR_FRAME_DONE_IRQ_MASK)
#define LCDIF_CTRL1_UNDERFLOW_IRQ_MASK           (0x400U)
#define LCDIF_CTRL1_UNDERFLOW_IRQ_SHIFT          (10U)

/* UNDERFLOW_IRQ
 *  0b0..No Interrupt Request Pending.
 *  0b1..Interrupt Request Pending.
 */

#define LCDIF_CTRL1_UNDERFLOW_IRQ(x)             (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_UNDERFLOW_IRQ_SHIFT)) & LCDIF_CTRL1_UNDERFLOW_IRQ_MASK)
#define LCDIF_CTRL1_OVERFLOW_IRQ_MASK            (0x800U)
#define LCDIF_CTRL1_OVERFLOW_IRQ_SHIFT           (11U)

/* OVERFLOW_IRQ
 *  0b0..No Interrupt Request Pending.
 *  0b1..Interrupt Request Pending.
 */

#define LCDIF_CTRL1_OVERFLOW_IRQ(x)              (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_OVERFLOW_IRQ_SHIFT)) & LCDIF_CTRL1_OVERFLOW_IRQ_MASK)
#define LCDIF_CTRL1_VSYNC_EDGE_IRQ_EN_MASK       (0x1000U)
#define LCDIF_CTRL1_VSYNC_EDGE_IRQ_EN_SHIFT      (12U)
#define LCDIF_CTRL1_VSYNC_EDGE_IRQ_EN(x)         (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_VSYNC_EDGE_IRQ_EN_SHIFT)) & LCDIF_CTRL1_VSYNC_EDGE_IRQ_EN_MASK)
#define LCDIF_CTRL1_CUR_FRAME_DONE_IRQ_EN_MASK   (0x2000U)
#define LCDIF_CTRL1_CUR_FRAME_DONE_IRQ_EN_SHIFT  (13U)
#define LCDIF_CTRL1_CUR_FRAME_DONE_IRQ_EN(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CUR_FRAME_DONE_IRQ_EN_SHIFT)) & LCDIF_CTRL1_CUR_FRAME_DONE_IRQ_EN_MASK)
#define LCDIF_CTRL1_UNDERFLOW_IRQ_EN_MASK        (0x4000U)
#define LCDIF_CTRL1_UNDERFLOW_IRQ_EN_SHIFT       (14U)
#define LCDIF_CTRL1_UNDERFLOW_IRQ_EN(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_UNDERFLOW_IRQ_EN_SHIFT)) & LCDIF_CTRL1_UNDERFLOW_IRQ_EN_MASK)
#define LCDIF_CTRL1_OVERFLOW_IRQ_EN_MASK         (0x8000U)
#define LCDIF_CTRL1_OVERFLOW_IRQ_EN_SHIFT        (15U)
#define LCDIF_CTRL1_OVERFLOW_IRQ_EN(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_OVERFLOW_IRQ_EN_SHIFT)) & LCDIF_CTRL1_OVERFLOW_IRQ_EN_MASK)
#define LCDIF_CTRL1_BYTE_PACKING_FORMAT_MASK     (0xf0000U)
#define LCDIF_CTRL1_BYTE_PACKING_FORMAT_SHIFT    (16U)
#define LCDIF_CTRL1_BYTE_PACKING_FORMAT(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_BYTE_PACKING_FORMAT_SHIFT)) & LCDIF_CTRL1_BYTE_PACKING_FORMAT_MASK)
#define LCDIF_CTRL1_IRQ_ON_ALTERNATE_FIELDS_MASK (0x100000U)
#define LCDIF_CTRL1_IRQ_ON_ALTERNATE_FIELDS_SHIFT (20U)
#define LCDIF_CTRL1_IRQ_ON_ALTERNATE_FIELDS(x)   (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_IRQ_ON_ALTERNATE_FIELDS_SHIFT)) & LCDIF_CTRL1_IRQ_ON_ALTERNATE_FIELDS_MASK)
#define LCDIF_CTRL1_FIFO_CLEAR_MASK              (0x200000U)
#define LCDIF_CTRL1_FIFO_CLEAR_SHIFT             (21U)
#define LCDIF_CTRL1_FIFO_CLEAR(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_FIFO_CLEAR_SHIFT)) & LCDIF_CTRL1_FIFO_CLEAR_MASK)
#define LCDIF_CTRL1_START_INTERLACE_FROM_SECOND_FIELD_MASK (0x400000U)
#define LCDIF_CTRL1_START_INTERLACE_FROM_SECOND_FIELD_SHIFT (22U)
#define LCDIF_CTRL1_START_INTERLACE_FROM_SECOND_FIELD(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_START_INTERLACE_FROM_SECOND_FIELD_SHIFT)) & LCDIF_CTRL1_START_INTERLACE_FROM_SECOND_FIELD_MASK)
#define LCDIF_CTRL1_INTERLACE_FIELDS_MASK        (0x800000U)
#define LCDIF_CTRL1_INTERLACE_FIELDS_SHIFT       (23U)
#define LCDIF_CTRL1_INTERLACE_FIELDS(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_INTERLACE_FIELDS_SHIFT)) & LCDIF_CTRL1_INTERLACE_FIELDS_MASK)
#define LCDIF_CTRL1_RECOVER_ON_UNDERFLOW_MASK    (0x1000000U)
#define LCDIF_CTRL1_RECOVER_ON_UNDERFLOW_SHIFT   (24U)
#define LCDIF_CTRL1_RECOVER_ON_UNDERFLOW(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_RECOVER_ON_UNDERFLOW_SHIFT)) & LCDIF_CTRL1_RECOVER_ON_UNDERFLOW_MASK)
#define LCDIF_CTRL1_BM_ERROR_IRQ_MASK            (0x2000000U)
#define LCDIF_CTRL1_BM_ERROR_IRQ_SHIFT           (25U)

/* BM_ERROR_IRQ
 *  0b0..No Interrupt Request Pending.
 *  0b1..Interrupt Request Pending.
 */

#define LCDIF_CTRL1_BM_ERROR_IRQ(x)              (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_BM_ERROR_IRQ_SHIFT)) & LCDIF_CTRL1_BM_ERROR_IRQ_MASK)
#define LCDIF_CTRL1_BM_ERROR_IRQ_EN_MASK         (0x4000000U)
#define LCDIF_CTRL1_BM_ERROR_IRQ_EN_SHIFT        (26U)
#define LCDIF_CTRL1_BM_ERROR_IRQ_EN(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_BM_ERROR_IRQ_EN_SHIFT)) & LCDIF_CTRL1_BM_ERROR_IRQ_EN_MASK)
#define LCDIF_CTRL1_CS_OUT_SELECT_MASK           (0x40000000U)
#define LCDIF_CTRL1_CS_OUT_SELECT_SHIFT          (30U)
#define LCDIF_CTRL1_CS_OUT_SELECT(x)             (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CS_OUT_SELECT_SHIFT)) & LCDIF_CTRL1_CS_OUT_SELECT_MASK)
#define LCDIF_CTRL1_IMAGE_DATA_SELECT_MASK       (0x80000000U)
#define LCDIF_CTRL1_IMAGE_DATA_SELECT_SHIFT      (31U)
#define LCDIF_CTRL1_IMAGE_DATA_SELECT(x)         (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_IMAGE_DATA_SELECT_SHIFT)) & LCDIF_CTRL1_IMAGE_DATA_SELECT_MASK)

/* CTRL1_SET - LCDIF General Control1 Register */

#define LCDIF_CTRL1_SET_RSRVD0_MASK              (0xf8U)
#define LCDIF_CTRL1_SET_RSRVD0_SHIFT             (3U)
#define LCDIF_CTRL1_SET_RSRVD0(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_SET_RSRVD0_SHIFT)) & LCDIF_CTRL1_SET_RSRVD0_MASK)
#define LCDIF_CTRL1_SET_VSYNC_EDGE_IRQ_MASK      (0x100U)
#define LCDIF_CTRL1_SET_VSYNC_EDGE_IRQ_SHIFT     (8U)

/* VSYNC_EDGE_IRQ
 *  0b0..No Interrupt Request Pending.
 *  0b1..Interrupt Request Pending.
 */

#define LCDIF_CTRL1_SET_VSYNC_EDGE_IRQ(x)        (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_SET_VSYNC_EDGE_IRQ_SHIFT)) & LCDIF_CTRL1_SET_VSYNC_EDGE_IRQ_MASK)
#define LCDIF_CTRL1_SET_CUR_FRAME_DONE_IRQ_MASK  (0x200U)
#define LCDIF_CTRL1_SET_CUR_FRAME_DONE_IRQ_SHIFT (9U)

/* CUR_FRAME_DONE_IRQ
 *  0b0..No Interrupt Request Pending.
 *  0b1..Interrupt Request Pending.
 */

#define LCDIF_CTRL1_SET_CUR_FRAME_DONE_IRQ(x)    (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_SET_CUR_FRAME_DONE_IRQ_SHIFT)) & LCDIF_CTRL1_SET_CUR_FRAME_DONE_IRQ_MASK)
#define LCDIF_CTRL1_SET_UNDERFLOW_IRQ_MASK       (0x400U)
#define LCDIF_CTRL1_SET_UNDERFLOW_IRQ_SHIFT      (10U)

/* UNDERFLOW_IRQ
 *  0b0..No Interrupt Request Pending.
 *  0b1..Interrupt Request Pending.
 */

#define LCDIF_CTRL1_SET_UNDERFLOW_IRQ(x)         (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_SET_UNDERFLOW_IRQ_SHIFT)) & LCDIF_CTRL1_SET_UNDERFLOW_IRQ_MASK)
#define LCDIF_CTRL1_SET_OVERFLOW_IRQ_MASK        (0x800U)
#define LCDIF_CTRL1_SET_OVERFLOW_IRQ_SHIFT       (11U)

/* OVERFLOW_IRQ
 *  0b0..No Interrupt Request Pending.
 *  0b1..Interrupt Request Pending.
 */

#define LCDIF_CTRL1_SET_OVERFLOW_IRQ(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_SET_OVERFLOW_IRQ_SHIFT)) & LCDIF_CTRL1_SET_OVERFLOW_IRQ_MASK)
#define LCDIF_CTRL1_SET_VSYNC_EDGE_IRQ_EN_MASK   (0x1000U)
#define LCDIF_CTRL1_SET_VSYNC_EDGE_IRQ_EN_SHIFT  (12U)
#define LCDIF_CTRL1_SET_VSYNC_EDGE_IRQ_EN(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_SET_VSYNC_EDGE_IRQ_EN_SHIFT)) & LCDIF_CTRL1_SET_VSYNC_EDGE_IRQ_EN_MASK)
#define LCDIF_CTRL1_SET_CUR_FRAME_DONE_IRQ_EN_MASK (0x2000U)
#define LCDIF_CTRL1_SET_CUR_FRAME_DONE_IRQ_EN_SHIFT (13U)
#define LCDIF_CTRL1_SET_CUR_FRAME_DONE_IRQ_EN(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_SET_CUR_FRAME_DONE_IRQ_EN_SHIFT)) & LCDIF_CTRL1_SET_CUR_FRAME_DONE_IRQ_EN_MASK)
#define LCDIF_CTRL1_SET_UNDERFLOW_IRQ_EN_MASK    (0x4000U)
#define LCDIF_CTRL1_SET_UNDERFLOW_IRQ_EN_SHIFT   (14U)
#define LCDIF_CTRL1_SET_UNDERFLOW_IRQ_EN(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_SET_UNDERFLOW_IRQ_EN_SHIFT)) & LCDIF_CTRL1_SET_UNDERFLOW_IRQ_EN_MASK)
#define LCDIF_CTRL1_SET_OVERFLOW_IRQ_EN_MASK     (0x8000U)
#define LCDIF_CTRL1_SET_OVERFLOW_IRQ_EN_SHIFT    (15U)
#define LCDIF_CTRL1_SET_OVERFLOW_IRQ_EN(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_SET_OVERFLOW_IRQ_EN_SHIFT)) & LCDIF_CTRL1_SET_OVERFLOW_IRQ_EN_MASK)
#define LCDIF_CTRL1_SET_BYTE_PACKING_FORMAT_MASK (0xf0000U)
#define LCDIF_CTRL1_SET_BYTE_PACKING_FORMAT_SHIFT (16U)
#define LCDIF_CTRL1_SET_BYTE_PACKING_FORMAT(x)   (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_SET_BYTE_PACKING_FORMAT_SHIFT)) & LCDIF_CTRL1_SET_BYTE_PACKING_FORMAT_MASK)
#define LCDIF_CTRL1_SET_IRQ_ON_ALTERNATE_FIELDS_MASK (0x100000U)
#define LCDIF_CTRL1_SET_IRQ_ON_ALTERNATE_FIELDS_SHIFT (20U)
#define LCDIF_CTRL1_SET_IRQ_ON_ALTERNATE_FIELDS(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_SET_IRQ_ON_ALTERNATE_FIELDS_SHIFT)) & LCDIF_CTRL1_SET_IRQ_ON_ALTERNATE_FIELDS_MASK)
#define LCDIF_CTRL1_SET_FIFO_CLEAR_MASK          (0x200000U)
#define LCDIF_CTRL1_SET_FIFO_CLEAR_SHIFT         (21U)
#define LCDIF_CTRL1_SET_FIFO_CLEAR(x)            (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_SET_FIFO_CLEAR_SHIFT)) & LCDIF_CTRL1_SET_FIFO_CLEAR_MASK)
#define LCDIF_CTRL1_SET_START_INTERLACE_FROM_SECOND_FIELD_MASK (0x400000U)
#define LCDIF_CTRL1_SET_START_INTERLACE_FROM_SECOND_FIELD_SHIFT (22U)
#define LCDIF_CTRL1_SET_START_INTERLACE_FROM_SECOND_FIELD(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_SET_START_INTERLACE_FROM_SECOND_FIELD_SHIFT)) & LCDIF_CTRL1_SET_START_INTERLACE_FROM_SECOND_FIELD_MASK)
#define LCDIF_CTRL1_SET_INTERLACE_FIELDS_MASK    (0x800000U)
#define LCDIF_CTRL1_SET_INTERLACE_FIELDS_SHIFT   (23U)
#define LCDIF_CTRL1_SET_INTERLACE_FIELDS(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_SET_INTERLACE_FIELDS_SHIFT)) & LCDIF_CTRL1_SET_INTERLACE_FIELDS_MASK)
#define LCDIF_CTRL1_SET_RECOVER_ON_UNDERFLOW_MASK (0x1000000U)
#define LCDIF_CTRL1_SET_RECOVER_ON_UNDERFLOW_SHIFT (24U)
#define LCDIF_CTRL1_SET_RECOVER_ON_UNDERFLOW(x)  (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_SET_RECOVER_ON_UNDERFLOW_SHIFT)) & LCDIF_CTRL1_SET_RECOVER_ON_UNDERFLOW_MASK)
#define LCDIF_CTRL1_SET_BM_ERROR_IRQ_MASK        (0x2000000U)
#define LCDIF_CTRL1_SET_BM_ERROR_IRQ_SHIFT       (25U)

/* BM_ERROR_IRQ
 *  0b0..No Interrupt Request Pending.
 *  0b1..Interrupt Request Pending.
 */

#define LCDIF_CTRL1_SET_BM_ERROR_IRQ(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_SET_BM_ERROR_IRQ_SHIFT)) & LCDIF_CTRL1_SET_BM_ERROR_IRQ_MASK)
#define LCDIF_CTRL1_SET_BM_ERROR_IRQ_EN_MASK     (0x4000000U)
#define LCDIF_CTRL1_SET_BM_ERROR_IRQ_EN_SHIFT    (26U)
#define LCDIF_CTRL1_SET_BM_ERROR_IRQ_EN(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_SET_BM_ERROR_IRQ_EN_SHIFT)) & LCDIF_CTRL1_SET_BM_ERROR_IRQ_EN_MASK)
#define LCDIF_CTRL1_SET_CS_OUT_SELECT_MASK       (0x40000000U)
#define LCDIF_CTRL1_SET_CS_OUT_SELECT_SHIFT      (30U)
#define LCDIF_CTRL1_SET_CS_OUT_SELECT(x)         (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_SET_CS_OUT_SELECT_SHIFT)) & LCDIF_CTRL1_SET_CS_OUT_SELECT_MASK)
#define LCDIF_CTRL1_SET_IMAGE_DATA_SELECT_MASK   (0x80000000U)
#define LCDIF_CTRL1_SET_IMAGE_DATA_SELECT_SHIFT  (31U)
#define LCDIF_CTRL1_SET_IMAGE_DATA_SELECT(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_SET_IMAGE_DATA_SELECT_SHIFT)) & LCDIF_CTRL1_SET_IMAGE_DATA_SELECT_MASK)

/* CTRL1_CLR - LCDIF General Control1 Register */

#define LCDIF_CTRL1_CLR_RSRVD0_MASK              (0xf8U)
#define LCDIF_CTRL1_CLR_RSRVD0_SHIFT             (3U)
#define LCDIF_CTRL1_CLR_RSRVD0(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CLR_RSRVD0_SHIFT)) & LCDIF_CTRL1_CLR_RSRVD0_MASK)
#define LCDIF_CTRL1_CLR_VSYNC_EDGE_IRQ_MASK      (0x100U)
#define LCDIF_CTRL1_CLR_VSYNC_EDGE_IRQ_SHIFT     (8U)

/* VSYNC_EDGE_IRQ
 *  0b0..No Interrupt Request Pending.
 *  0b1..Interrupt Request Pending.
 */

#define LCDIF_CTRL1_CLR_VSYNC_EDGE_IRQ(x)        (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CLR_VSYNC_EDGE_IRQ_SHIFT)) & LCDIF_CTRL1_CLR_VSYNC_EDGE_IRQ_MASK)
#define LCDIF_CTRL1_CLR_CUR_FRAME_DONE_IRQ_MASK  (0x200U)
#define LCDIF_CTRL1_CLR_CUR_FRAME_DONE_IRQ_SHIFT (9U)

/* CUR_FRAME_DONE_IRQ
 *  0b0..No Interrupt Request Pending.
 *  0b1..Interrupt Request Pending.
 */

#define LCDIF_CTRL1_CLR_CUR_FRAME_DONE_IRQ(x)    (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CLR_CUR_FRAME_DONE_IRQ_SHIFT)) & LCDIF_CTRL1_CLR_CUR_FRAME_DONE_IRQ_MASK)
#define LCDIF_CTRL1_CLR_UNDERFLOW_IRQ_MASK       (0x400U)
#define LCDIF_CTRL1_CLR_UNDERFLOW_IRQ_SHIFT      (10U)

/* UNDERFLOW_IRQ
 *  0b0..No Interrupt Request Pending.
 *  0b1..Interrupt Request Pending.
 */

#define LCDIF_CTRL1_CLR_UNDERFLOW_IRQ(x)         (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CLR_UNDERFLOW_IRQ_SHIFT)) & LCDIF_CTRL1_CLR_UNDERFLOW_IRQ_MASK)
#define LCDIF_CTRL1_CLR_OVERFLOW_IRQ_MASK        (0x800U)
#define LCDIF_CTRL1_CLR_OVERFLOW_IRQ_SHIFT       (11U)

/* OVERFLOW_IRQ
 *  0b0..No Interrupt Request Pending.
 *  0b1..Interrupt Request Pending.
 */

#define LCDIF_CTRL1_CLR_OVERFLOW_IRQ(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CLR_OVERFLOW_IRQ_SHIFT)) & LCDIF_CTRL1_CLR_OVERFLOW_IRQ_MASK)
#define LCDIF_CTRL1_CLR_VSYNC_EDGE_IRQ_EN_MASK   (0x1000U)
#define LCDIF_CTRL1_CLR_VSYNC_EDGE_IRQ_EN_SHIFT  (12U)
#define LCDIF_CTRL1_CLR_VSYNC_EDGE_IRQ_EN(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CLR_VSYNC_EDGE_IRQ_EN_SHIFT)) & LCDIF_CTRL1_CLR_VSYNC_EDGE_IRQ_EN_MASK)
#define LCDIF_CTRL1_CLR_CUR_FRAME_DONE_IRQ_EN_MASK (0x2000U)
#define LCDIF_CTRL1_CLR_CUR_FRAME_DONE_IRQ_EN_SHIFT (13U)
#define LCDIF_CTRL1_CLR_CUR_FRAME_DONE_IRQ_EN(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CLR_CUR_FRAME_DONE_IRQ_EN_SHIFT)) & LCDIF_CTRL1_CLR_CUR_FRAME_DONE_IRQ_EN_MASK)
#define LCDIF_CTRL1_CLR_UNDERFLOW_IRQ_EN_MASK    (0x4000U)
#define LCDIF_CTRL1_CLR_UNDERFLOW_IRQ_EN_SHIFT   (14U)
#define LCDIF_CTRL1_CLR_UNDERFLOW_IRQ_EN(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CLR_UNDERFLOW_IRQ_EN_SHIFT)) & LCDIF_CTRL1_CLR_UNDERFLOW_IRQ_EN_MASK)
#define LCDIF_CTRL1_CLR_OVERFLOW_IRQ_EN_MASK     (0x8000U)
#define LCDIF_CTRL1_CLR_OVERFLOW_IRQ_EN_SHIFT    (15U)
#define LCDIF_CTRL1_CLR_OVERFLOW_IRQ_EN(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CLR_OVERFLOW_IRQ_EN_SHIFT)) & LCDIF_CTRL1_CLR_OVERFLOW_IRQ_EN_MASK)
#define LCDIF_CTRL1_CLR_BYTE_PACKING_FORMAT_MASK (0xf0000U)
#define LCDIF_CTRL1_CLR_BYTE_PACKING_FORMAT_SHIFT (16U)
#define LCDIF_CTRL1_CLR_BYTE_PACKING_FORMAT(x)   (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CLR_BYTE_PACKING_FORMAT_SHIFT)) & LCDIF_CTRL1_CLR_BYTE_PACKING_FORMAT_MASK)
#define LCDIF_CTRL1_CLR_IRQ_ON_ALTERNATE_FIELDS_MASK (0x100000U)
#define LCDIF_CTRL1_CLR_IRQ_ON_ALTERNATE_FIELDS_SHIFT (20U)
#define LCDIF_CTRL1_CLR_IRQ_ON_ALTERNATE_FIELDS(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CLR_IRQ_ON_ALTERNATE_FIELDS_SHIFT)) & LCDIF_CTRL1_CLR_IRQ_ON_ALTERNATE_FIELDS_MASK)
#define LCDIF_CTRL1_CLR_FIFO_CLEAR_MASK          (0x200000U)
#define LCDIF_CTRL1_CLR_FIFO_CLEAR_SHIFT         (21U)
#define LCDIF_CTRL1_CLR_FIFO_CLEAR(x)            (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CLR_FIFO_CLEAR_SHIFT)) & LCDIF_CTRL1_CLR_FIFO_CLEAR_MASK)
#define LCDIF_CTRL1_CLR_START_INTERLACE_FROM_SECOND_FIELD_MASK (0x400000U)
#define LCDIF_CTRL1_CLR_START_INTERLACE_FROM_SECOND_FIELD_SHIFT (22U)
#define LCDIF_CTRL1_CLR_START_INTERLACE_FROM_SECOND_FIELD(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CLR_START_INTERLACE_FROM_SECOND_FIELD_SHIFT)) & LCDIF_CTRL1_CLR_START_INTERLACE_FROM_SECOND_FIELD_MASK)
#define LCDIF_CTRL1_CLR_INTERLACE_FIELDS_MASK    (0x800000U)
#define LCDIF_CTRL1_CLR_INTERLACE_FIELDS_SHIFT   (23U)
#define LCDIF_CTRL1_CLR_INTERLACE_FIELDS(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CLR_INTERLACE_FIELDS_SHIFT)) & LCDIF_CTRL1_CLR_INTERLACE_FIELDS_MASK)
#define LCDIF_CTRL1_CLR_RECOVER_ON_UNDERFLOW_MASK (0x1000000U)
#define LCDIF_CTRL1_CLR_RECOVER_ON_UNDERFLOW_SHIFT (24U)
#define LCDIF_CTRL1_CLR_RECOVER_ON_UNDERFLOW(x)  (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CLR_RECOVER_ON_UNDERFLOW_SHIFT)) & LCDIF_CTRL1_CLR_RECOVER_ON_UNDERFLOW_MASK)
#define LCDIF_CTRL1_CLR_BM_ERROR_IRQ_MASK        (0x2000000U)
#define LCDIF_CTRL1_CLR_BM_ERROR_IRQ_SHIFT       (25U)

/* BM_ERROR_IRQ
 *  0b0..No Interrupt Request Pending.
 *  0b1..Interrupt Request Pending.
 */

#define LCDIF_CTRL1_CLR_BM_ERROR_IRQ(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CLR_BM_ERROR_IRQ_SHIFT)) & LCDIF_CTRL1_CLR_BM_ERROR_IRQ_MASK)
#define LCDIF_CTRL1_CLR_BM_ERROR_IRQ_EN_MASK     (0x4000000U)
#define LCDIF_CTRL1_CLR_BM_ERROR_IRQ_EN_SHIFT    (26U)
#define LCDIF_CTRL1_CLR_BM_ERROR_IRQ_EN(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CLR_BM_ERROR_IRQ_EN_SHIFT)) & LCDIF_CTRL1_CLR_BM_ERROR_IRQ_EN_MASK)
#define LCDIF_CTRL1_CLR_CS_OUT_SELECT_MASK       (0x40000000U)
#define LCDIF_CTRL1_CLR_CS_OUT_SELECT_SHIFT      (30U)
#define LCDIF_CTRL1_CLR_CS_OUT_SELECT(x)         (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CLR_CS_OUT_SELECT_SHIFT)) & LCDIF_CTRL1_CLR_CS_OUT_SELECT_MASK)
#define LCDIF_CTRL1_CLR_IMAGE_DATA_SELECT_MASK   (0x80000000U)
#define LCDIF_CTRL1_CLR_IMAGE_DATA_SELECT_SHIFT  (31U)
#define LCDIF_CTRL1_CLR_IMAGE_DATA_SELECT(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_CLR_IMAGE_DATA_SELECT_SHIFT)) & LCDIF_CTRL1_CLR_IMAGE_DATA_SELECT_MASK)

/* CTRL1_TOG - LCDIF General Control1 Register */

#define LCDIF_CTRL1_TOG_RSRVD0_MASK              (0xf8U)
#define LCDIF_CTRL1_TOG_RSRVD0_SHIFT             (3U)
#define LCDIF_CTRL1_TOG_RSRVD0(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_TOG_RSRVD0_SHIFT)) & LCDIF_CTRL1_TOG_RSRVD0_MASK)
#define LCDIF_CTRL1_TOG_VSYNC_EDGE_IRQ_MASK      (0x100U)
#define LCDIF_CTRL1_TOG_VSYNC_EDGE_IRQ_SHIFT     (8U)

/* VSYNC_EDGE_IRQ
 *  0b0..No Interrupt Request Pending.
 *  0b1..Interrupt Request Pending.
 */

#define LCDIF_CTRL1_TOG_VSYNC_EDGE_IRQ(x)        (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_TOG_VSYNC_EDGE_IRQ_SHIFT)) & LCDIF_CTRL1_TOG_VSYNC_EDGE_IRQ_MASK)
#define LCDIF_CTRL1_TOG_CUR_FRAME_DONE_IRQ_MASK  (0x200U)
#define LCDIF_CTRL1_TOG_CUR_FRAME_DONE_IRQ_SHIFT (9U)

/* CUR_FRAME_DONE_IRQ
 *  0b0..No Interrupt Request Pending.
 *  0b1..Interrupt Request Pending.
 */

#define LCDIF_CTRL1_TOG_CUR_FRAME_DONE_IRQ(x)    (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_TOG_CUR_FRAME_DONE_IRQ_SHIFT)) & LCDIF_CTRL1_TOG_CUR_FRAME_DONE_IRQ_MASK)
#define LCDIF_CTRL1_TOG_UNDERFLOW_IRQ_MASK       (0x400U)
#define LCDIF_CTRL1_TOG_UNDERFLOW_IRQ_SHIFT      (10U)

/* UNDERFLOW_IRQ
 *  0b0..No Interrupt Request Pending.
 *  0b1..Interrupt Request Pending.
 */

#define LCDIF_CTRL1_TOG_UNDERFLOW_IRQ(x)         (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_TOG_UNDERFLOW_IRQ_SHIFT)) & LCDIF_CTRL1_TOG_UNDERFLOW_IRQ_MASK)
#define LCDIF_CTRL1_TOG_OVERFLOW_IRQ_MASK        (0x800U)
#define LCDIF_CTRL1_TOG_OVERFLOW_IRQ_SHIFT       (11U)

/* OVERFLOW_IRQ
 *  0b0..No Interrupt Request Pending.
 *  0b1..Interrupt Request Pending.
 */

#define LCDIF_CTRL1_TOG_OVERFLOW_IRQ(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_TOG_OVERFLOW_IRQ_SHIFT)) & LCDIF_CTRL1_TOG_OVERFLOW_IRQ_MASK)
#define LCDIF_CTRL1_TOG_VSYNC_EDGE_IRQ_EN_MASK   (0x1000U)
#define LCDIF_CTRL1_TOG_VSYNC_EDGE_IRQ_EN_SHIFT  (12U)
#define LCDIF_CTRL1_TOG_VSYNC_EDGE_IRQ_EN(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_TOG_VSYNC_EDGE_IRQ_EN_SHIFT)) & LCDIF_CTRL1_TOG_VSYNC_EDGE_IRQ_EN_MASK)
#define LCDIF_CTRL1_TOG_CUR_FRAME_DONE_IRQ_EN_MASK (0x2000U)
#define LCDIF_CTRL1_TOG_CUR_FRAME_DONE_IRQ_EN_SHIFT (13U)
#define LCDIF_CTRL1_TOG_CUR_FRAME_DONE_IRQ_EN(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_TOG_CUR_FRAME_DONE_IRQ_EN_SHIFT)) & LCDIF_CTRL1_TOG_CUR_FRAME_DONE_IRQ_EN_MASK)
#define LCDIF_CTRL1_TOG_UNDERFLOW_IRQ_EN_MASK    (0x4000U)
#define LCDIF_CTRL1_TOG_UNDERFLOW_IRQ_EN_SHIFT   (14U)
#define LCDIF_CTRL1_TOG_UNDERFLOW_IRQ_EN(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_TOG_UNDERFLOW_IRQ_EN_SHIFT)) & LCDIF_CTRL1_TOG_UNDERFLOW_IRQ_EN_MASK)
#define LCDIF_CTRL1_TOG_OVERFLOW_IRQ_EN_MASK     (0x8000U)
#define LCDIF_CTRL1_TOG_OVERFLOW_IRQ_EN_SHIFT    (15U)
#define LCDIF_CTRL1_TOG_OVERFLOW_IRQ_EN(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_TOG_OVERFLOW_IRQ_EN_SHIFT)) & LCDIF_CTRL1_TOG_OVERFLOW_IRQ_EN_MASK)
#define LCDIF_CTRL1_TOG_BYTE_PACKING_FORMAT_MASK (0xf0000U)
#define LCDIF_CTRL1_TOG_BYTE_PACKING_FORMAT_SHIFT (16U)
#define LCDIF_CTRL1_TOG_BYTE_PACKING_FORMAT(x)   (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_TOG_BYTE_PACKING_FORMAT_SHIFT)) & LCDIF_CTRL1_TOG_BYTE_PACKING_FORMAT_MASK)
#define LCDIF_CTRL1_TOG_IRQ_ON_ALTERNATE_FIELDS_MASK (0x100000U)
#define LCDIF_CTRL1_TOG_IRQ_ON_ALTERNATE_FIELDS_SHIFT (20U)
#define LCDIF_CTRL1_TOG_IRQ_ON_ALTERNATE_FIELDS(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_TOG_IRQ_ON_ALTERNATE_FIELDS_SHIFT)) & LCDIF_CTRL1_TOG_IRQ_ON_ALTERNATE_FIELDS_MASK)
#define LCDIF_CTRL1_TOG_FIFO_CLEAR_MASK          (0x200000U)
#define LCDIF_CTRL1_TOG_FIFO_CLEAR_SHIFT         (21U)
#define LCDIF_CTRL1_TOG_FIFO_CLEAR(x)            (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_TOG_FIFO_CLEAR_SHIFT)) & LCDIF_CTRL1_TOG_FIFO_CLEAR_MASK)
#define LCDIF_CTRL1_TOG_START_INTERLACE_FROM_SECOND_FIELD_MASK (0x400000U)
#define LCDIF_CTRL1_TOG_START_INTERLACE_FROM_SECOND_FIELD_SHIFT (22U)
#define LCDIF_CTRL1_TOG_START_INTERLACE_FROM_SECOND_FIELD(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_TOG_START_INTERLACE_FROM_SECOND_FIELD_SHIFT)) & LCDIF_CTRL1_TOG_START_INTERLACE_FROM_SECOND_FIELD_MASK)
#define LCDIF_CTRL1_TOG_INTERLACE_FIELDS_MASK    (0x800000U)
#define LCDIF_CTRL1_TOG_INTERLACE_FIELDS_SHIFT   (23U)
#define LCDIF_CTRL1_TOG_INTERLACE_FIELDS(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_TOG_INTERLACE_FIELDS_SHIFT)) & LCDIF_CTRL1_TOG_INTERLACE_FIELDS_MASK)
#define LCDIF_CTRL1_TOG_RECOVER_ON_UNDERFLOW_MASK (0x1000000U)
#define LCDIF_CTRL1_TOG_RECOVER_ON_UNDERFLOW_SHIFT (24U)
#define LCDIF_CTRL1_TOG_RECOVER_ON_UNDERFLOW(x)  (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_TOG_RECOVER_ON_UNDERFLOW_SHIFT)) & LCDIF_CTRL1_TOG_RECOVER_ON_UNDERFLOW_MASK)
#define LCDIF_CTRL1_TOG_BM_ERROR_IRQ_MASK        (0x2000000U)
#define LCDIF_CTRL1_TOG_BM_ERROR_IRQ_SHIFT       (25U)

/* BM_ERROR_IRQ
 *  0b0..No Interrupt Request Pending.
 *  0b1..Interrupt Request Pending.
 */

#define LCDIF_CTRL1_TOG_BM_ERROR_IRQ(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_TOG_BM_ERROR_IRQ_SHIFT)) & LCDIF_CTRL1_TOG_BM_ERROR_IRQ_MASK)
#define LCDIF_CTRL1_TOG_BM_ERROR_IRQ_EN_MASK     (0x4000000U)
#define LCDIF_CTRL1_TOG_BM_ERROR_IRQ_EN_SHIFT    (26U)
#define LCDIF_CTRL1_TOG_BM_ERROR_IRQ_EN(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_TOG_BM_ERROR_IRQ_EN_SHIFT)) & LCDIF_CTRL1_TOG_BM_ERROR_IRQ_EN_MASK)
#define LCDIF_CTRL1_TOG_CS_OUT_SELECT_MASK       (0x40000000U)
#define LCDIF_CTRL1_TOG_CS_OUT_SELECT_SHIFT      (30U)
#define LCDIF_CTRL1_TOG_CS_OUT_SELECT(x)         (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_TOG_CS_OUT_SELECT_SHIFT)) & LCDIF_CTRL1_TOG_CS_OUT_SELECT_MASK)
#define LCDIF_CTRL1_TOG_IMAGE_DATA_SELECT_MASK   (0x80000000U)
#define LCDIF_CTRL1_TOG_IMAGE_DATA_SELECT_SHIFT  (31U)
#define LCDIF_CTRL1_TOG_IMAGE_DATA_SELECT(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL1_TOG_IMAGE_DATA_SELECT_SHIFT)) & LCDIF_CTRL1_TOG_IMAGE_DATA_SELECT_MASK)

/* CTRL2 - LCDIF General Control2 Register */

#define LCDIF_CTRL2_RSRVD0_MASK                  (0xfffU)
#define LCDIF_CTRL2_RSRVD0_SHIFT                 (0U)
#define LCDIF_CTRL2_RSRVD0(x)                    (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_RSRVD0_SHIFT)) & LCDIF_CTRL2_RSRVD0_MASK)
#define LCDIF_CTRL2_EVEN_LINE_PATTERN_MASK       (0x7000U)
#define LCDIF_CTRL2_EVEN_LINE_PATTERN_SHIFT      (12U)

/* EVEN_LINE_PATTERN
 *  0b000..RGB
 *  0b001..RBG
 *  0b010..GBR
 *  0b011..GRB
 *  0b100..BRG
 *  0b101..BGR
 */

#define LCDIF_CTRL2_EVEN_LINE_PATTERN(x)         (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_EVEN_LINE_PATTERN_SHIFT)) & LCDIF_CTRL2_EVEN_LINE_PATTERN_MASK)
#define LCDIF_CTRL2_RSRVD3_MASK                  (0x8000U)
#define LCDIF_CTRL2_RSRVD3_SHIFT                 (15U)
#define LCDIF_CTRL2_RSRVD3(x)                    (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_RSRVD3_SHIFT)) & LCDIF_CTRL2_RSRVD3_MASK)
#define LCDIF_CTRL2_ODD_LINE_PATTERN_MASK        (0x70000U)
#define LCDIF_CTRL2_ODD_LINE_PATTERN_SHIFT       (16U)

/* ODD_LINE_PATTERN
 *  0b000..RGB
 *  0b001..RBG
 *  0b010..GBR
 *  0b011..GRB
 *  0b100..BRG
 *  0b101..BGR
 */

#define LCDIF_CTRL2_ODD_LINE_PATTERN(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_ODD_LINE_PATTERN_SHIFT)) & LCDIF_CTRL2_ODD_LINE_PATTERN_MASK)
#define LCDIF_CTRL2_RSRVD4_MASK                  (0x80000U)
#define LCDIF_CTRL2_RSRVD4_SHIFT                 (19U)
#define LCDIF_CTRL2_RSRVD4(x)                    (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_RSRVD4_SHIFT)) & LCDIF_CTRL2_RSRVD4_MASK)
#define LCDIF_CTRL2_BURST_LEN_8_MASK             (0x100000U)
#define LCDIF_CTRL2_BURST_LEN_8_SHIFT            (20U)
#define LCDIF_CTRL2_BURST_LEN_8(x)               (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_BURST_LEN_8_SHIFT)) & LCDIF_CTRL2_BURST_LEN_8_MASK)
#define LCDIF_CTRL2_OUTSTANDING_REQS_MASK        (0xe00000U)
#define LCDIF_CTRL2_OUTSTANDING_REQS_SHIFT       (21U)

/* OUTSTANDING_REQS
 *  0b000..REQ_1
 *  0b001..REQ_2
 *  0b010..REQ_4
 *  0b011..REQ_8
 *  0b100..REQ_16
 */

#define LCDIF_CTRL2_OUTSTANDING_REQS(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_OUTSTANDING_REQS_SHIFT)) & LCDIF_CTRL2_OUTSTANDING_REQS_MASK)
#define LCDIF_CTRL2_RSRVD5_MASK                  (0xff000000U)
#define LCDIF_CTRL2_RSRVD5_SHIFT                 (24U)
#define LCDIF_CTRL2_RSRVD5(x)                    (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_RSRVD5_SHIFT)) & LCDIF_CTRL2_RSRVD5_MASK)

/* CTRL2_SET - LCDIF General Control2 Register */

#define LCDIF_CTRL2_SET_RSRVD0_MASK              (0xfffU)
#define LCDIF_CTRL2_SET_RSRVD0_SHIFT             (0U)
#define LCDIF_CTRL2_SET_RSRVD0(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_SET_RSRVD0_SHIFT)) & LCDIF_CTRL2_SET_RSRVD0_MASK)
#define LCDIF_CTRL2_SET_EVEN_LINE_PATTERN_MASK   (0x7000U)
#define LCDIF_CTRL2_SET_EVEN_LINE_PATTERN_SHIFT  (12U)

/* EVEN_LINE_PATTERN
 *  0b000..RGB
 *  0b001..RBG
 *  0b010..GBR
 *  0b011..GRB
 *  0b100..BRG
 *  0b101..BGR
 */

#define LCDIF_CTRL2_SET_EVEN_LINE_PATTERN(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_SET_EVEN_LINE_PATTERN_SHIFT)) & LCDIF_CTRL2_SET_EVEN_LINE_PATTERN_MASK)
#define LCDIF_CTRL2_SET_RSRVD3_MASK              (0x8000U)
#define LCDIF_CTRL2_SET_RSRVD3_SHIFT             (15U)
#define LCDIF_CTRL2_SET_RSRVD3(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_SET_RSRVD3_SHIFT)) & LCDIF_CTRL2_SET_RSRVD3_MASK)
#define LCDIF_CTRL2_SET_ODD_LINE_PATTERN_MASK    (0x70000U)
#define LCDIF_CTRL2_SET_ODD_LINE_PATTERN_SHIFT   (16U)

/* ODD_LINE_PATTERN
 *  0b000..RGB
 *  0b001..RBG
 *  0b010..GBR
 *  0b011..GRB
 *  0b100..BRG
 *  0b101..BGR
 */

#define LCDIF_CTRL2_SET_ODD_LINE_PATTERN(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_SET_ODD_LINE_PATTERN_SHIFT)) & LCDIF_CTRL2_SET_ODD_LINE_PATTERN_MASK)
#define LCDIF_CTRL2_SET_RSRVD4_MASK              (0x80000U)
#define LCDIF_CTRL2_SET_RSRVD4_SHIFT             (19U)
#define LCDIF_CTRL2_SET_RSRVD4(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_SET_RSRVD4_SHIFT)) & LCDIF_CTRL2_SET_RSRVD4_MASK)
#define LCDIF_CTRL2_SET_BURST_LEN_8_MASK         (0x100000U)
#define LCDIF_CTRL2_SET_BURST_LEN_8_SHIFT        (20U)
#define LCDIF_CTRL2_SET_BURST_LEN_8(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_SET_BURST_LEN_8_SHIFT)) & LCDIF_CTRL2_SET_BURST_LEN_8_MASK)
#define LCDIF_CTRL2_SET_OUTSTANDING_REQS_MASK    (0xe00000U)
#define LCDIF_CTRL2_SET_OUTSTANDING_REQS_SHIFT   (21U)

/* OUTSTANDING_REQS
 *  0b000..REQ_1
 *  0b001..REQ_2
 *  0b010..REQ_4
 *  0b011..REQ_8
 *  0b100..REQ_16
 */

#define LCDIF_CTRL2_SET_OUTSTANDING_REQS(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_SET_OUTSTANDING_REQS_SHIFT)) & LCDIF_CTRL2_SET_OUTSTANDING_REQS_MASK)
#define LCDIF_CTRL2_SET_RSRVD5_MASK              (0xff000000U)
#define LCDIF_CTRL2_SET_RSRVD5_SHIFT             (24U)
#define LCDIF_CTRL2_SET_RSRVD5(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_SET_RSRVD5_SHIFT)) & LCDIF_CTRL2_SET_RSRVD5_MASK)

/* CTRL2_CLR - LCDIF General Control2 Register */

#define LCDIF_CTRL2_CLR_RSRVD0_MASK              (0xfffU)
#define LCDIF_CTRL2_CLR_RSRVD0_SHIFT             (0U)
#define LCDIF_CTRL2_CLR_RSRVD0(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_CLR_RSRVD0_SHIFT)) & LCDIF_CTRL2_CLR_RSRVD0_MASK)
#define LCDIF_CTRL2_CLR_EVEN_LINE_PATTERN_MASK   (0x7000U)
#define LCDIF_CTRL2_CLR_EVEN_LINE_PATTERN_SHIFT  (12U)

/* EVEN_LINE_PATTERN
 *  0b000..RGB
 *  0b001..RBG
 *  0b010..GBR
 *  0b011..GRB
 *  0b100..BRG
 *  0b101..BGR
 */

#define LCDIF_CTRL2_CLR_EVEN_LINE_PATTERN(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_CLR_EVEN_LINE_PATTERN_SHIFT)) & LCDIF_CTRL2_CLR_EVEN_LINE_PATTERN_MASK)
#define LCDIF_CTRL2_CLR_RSRVD3_MASK              (0x8000U)
#define LCDIF_CTRL2_CLR_RSRVD3_SHIFT             (15U)
#define LCDIF_CTRL2_CLR_RSRVD3(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_CLR_RSRVD3_SHIFT)) & LCDIF_CTRL2_CLR_RSRVD3_MASK)
#define LCDIF_CTRL2_CLR_ODD_LINE_PATTERN_MASK    (0x70000U)
#define LCDIF_CTRL2_CLR_ODD_LINE_PATTERN_SHIFT   (16U)

/* ODD_LINE_PATTERN
 *  0b000..RGB
 *  0b001..RBG
 *  0b010..GBR
 *  0b011..GRB
 *  0b100..BRG
 *  0b101..BGR
 */

#define LCDIF_CTRL2_CLR_ODD_LINE_PATTERN(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_CLR_ODD_LINE_PATTERN_SHIFT)) & LCDIF_CTRL2_CLR_ODD_LINE_PATTERN_MASK)
#define LCDIF_CTRL2_CLR_RSRVD4_MASK              (0x80000U)
#define LCDIF_CTRL2_CLR_RSRVD4_SHIFT             (19U)
#define LCDIF_CTRL2_CLR_RSRVD4(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_CLR_RSRVD4_SHIFT)) & LCDIF_CTRL2_CLR_RSRVD4_MASK)
#define LCDIF_CTRL2_CLR_BURST_LEN_8_MASK         (0x100000U)
#define LCDIF_CTRL2_CLR_BURST_LEN_8_SHIFT        (20U)
#define LCDIF_CTRL2_CLR_BURST_LEN_8(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_CLR_BURST_LEN_8_SHIFT)) & LCDIF_CTRL2_CLR_BURST_LEN_8_MASK)
#define LCDIF_CTRL2_CLR_OUTSTANDING_REQS_MASK    (0xe00000U)
#define LCDIF_CTRL2_CLR_OUTSTANDING_REQS_SHIFT   (21U)

/* OUTSTANDING_REQS
 *  0b000..REQ_1
 *  0b001..REQ_2
 *  0b010..REQ_4
 *  0b011..REQ_8
 *  0b100..REQ_16
 */

#define LCDIF_CTRL2_CLR_OUTSTANDING_REQS(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_CLR_OUTSTANDING_REQS_SHIFT)) & LCDIF_CTRL2_CLR_OUTSTANDING_REQS_MASK)
#define LCDIF_CTRL2_CLR_RSRVD5_MASK              (0xff000000U)
#define LCDIF_CTRL2_CLR_RSRVD5_SHIFT             (24U)
#define LCDIF_CTRL2_CLR_RSRVD5(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_CLR_RSRVD5_SHIFT)) & LCDIF_CTRL2_CLR_RSRVD5_MASK)

/* CTRL2_TOG - LCDIF General Control2 Register */

#define LCDIF_CTRL2_TOG_RSRVD0_MASK              (0xfffU)
#define LCDIF_CTRL2_TOG_RSRVD0_SHIFT             (0U)
#define LCDIF_CTRL2_TOG_RSRVD0(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_TOG_RSRVD0_SHIFT)) & LCDIF_CTRL2_TOG_RSRVD0_MASK)
#define LCDIF_CTRL2_TOG_EVEN_LINE_PATTERN_MASK   (0x7000U)
#define LCDIF_CTRL2_TOG_EVEN_LINE_PATTERN_SHIFT  (12U)

/* EVEN_LINE_PATTERN
 *  0b000..RGB
 *  0b001..RBG
 *  0b010..GBR
 *  0b011..GRB
 *  0b100..BRG
 *  0b101..BGR
 */

#define LCDIF_CTRL2_TOG_EVEN_LINE_PATTERN(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_TOG_EVEN_LINE_PATTERN_SHIFT)) & LCDIF_CTRL2_TOG_EVEN_LINE_PATTERN_MASK)
#define LCDIF_CTRL2_TOG_RSRVD3_MASK              (0x8000U)
#define LCDIF_CTRL2_TOG_RSRVD3_SHIFT             (15U)
#define LCDIF_CTRL2_TOG_RSRVD3(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_TOG_RSRVD3_SHIFT)) & LCDIF_CTRL2_TOG_RSRVD3_MASK)
#define LCDIF_CTRL2_TOG_ODD_LINE_PATTERN_MASK    (0x70000U)
#define LCDIF_CTRL2_TOG_ODD_LINE_PATTERN_SHIFT   (16U)

/* ODD_LINE_PATTERN
 *  0b000..RGB
 *  0b001..RBG
 *  0b010..GBR
 *  0b011..GRB
 *  0b100..BRG
 *  0b101..BGR
 */

#define LCDIF_CTRL2_TOG_ODD_LINE_PATTERN(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_TOG_ODD_LINE_PATTERN_SHIFT)) & LCDIF_CTRL2_TOG_ODD_LINE_PATTERN_MASK)
#define LCDIF_CTRL2_TOG_RSRVD4_MASK              (0x80000U)
#define LCDIF_CTRL2_TOG_RSRVD4_SHIFT             (19U)
#define LCDIF_CTRL2_TOG_RSRVD4(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_TOG_RSRVD4_SHIFT)) & LCDIF_CTRL2_TOG_RSRVD4_MASK)
#define LCDIF_CTRL2_TOG_BURST_LEN_8_MASK         (0x100000U)
#define LCDIF_CTRL2_TOG_BURST_LEN_8_SHIFT        (20U)
#define LCDIF_CTRL2_TOG_BURST_LEN_8(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_TOG_BURST_LEN_8_SHIFT)) & LCDIF_CTRL2_TOG_BURST_LEN_8_MASK)
#define LCDIF_CTRL2_TOG_OUTSTANDING_REQS_MASK    (0xe00000U)
#define LCDIF_CTRL2_TOG_OUTSTANDING_REQS_SHIFT   (21U)

/* OUTSTANDING_REQS
 *  0b000..REQ_1
 *  0b001..REQ_2
 *  0b010..REQ_4
 *  0b011..REQ_8
 *  0b100..REQ_16
 */

#define LCDIF_CTRL2_TOG_OUTSTANDING_REQS(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_TOG_OUTSTANDING_REQS_SHIFT)) & LCDIF_CTRL2_TOG_OUTSTANDING_REQS_MASK)
#define LCDIF_CTRL2_TOG_RSRVD5_MASK              (0xff000000U)
#define LCDIF_CTRL2_TOG_RSRVD5_SHIFT             (24U)
#define LCDIF_CTRL2_TOG_RSRVD5(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_CTRL2_TOG_RSRVD5_SHIFT)) & LCDIF_CTRL2_TOG_RSRVD5_MASK)

/* TRANSFER_COUNT - LCDIF Horizontal and Vertical Valid Data Count Register */

#define LCDIF_TRANSFER_COUNT_H_COUNT_MASK        (0xffffU)
#define LCDIF_TRANSFER_COUNT_H_COUNT_SHIFT       (0U)
#define LCDIF_TRANSFER_COUNT_H_COUNT(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_TRANSFER_COUNT_H_COUNT_SHIFT)) & LCDIF_TRANSFER_COUNT_H_COUNT_MASK)
#define LCDIF_TRANSFER_COUNT_V_COUNT_MASK        (0xffff0000U)
#define LCDIF_TRANSFER_COUNT_V_COUNT_SHIFT       (16U)
#define LCDIF_TRANSFER_COUNT_V_COUNT(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_TRANSFER_COUNT_V_COUNT_SHIFT)) & LCDIF_TRANSFER_COUNT_V_COUNT_MASK)

/* CUR_BUF - LCD Interface Current Buffer Address Register */

#define LCDIF_CUR_BUF_ADDR_MASK                  (0xffffffffU)
#define LCDIF_CUR_BUF_ADDR_SHIFT                 (0U)
#define LCDIF_CUR_BUF_ADDR(x)                    (((uint32_t)(((uint32_t)(x)) << LCDIF_CUR_BUF_ADDR_SHIFT)) & LCDIF_CUR_BUF_ADDR_MASK)

/* NEXT_BUF - LCD Interface Next Buffer Address Register */

#define LCDIF_NEXT_BUF_ADDR_MASK                 (0xffffffffU)
#define LCDIF_NEXT_BUF_ADDR_SHIFT                (0U)
#define LCDIF_NEXT_BUF_ADDR(x)                   (((uint32_t)(((uint32_t)(x)) << LCDIF_NEXT_BUF_ADDR_SHIFT)) & LCDIF_NEXT_BUF_ADDR_MASK)

/* VDCTRL0 - LCDIF VSYNC Mode and Dotclk Mode Control Register0 */

#define LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH_MASK     (0x3ffffU)
#define LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH_SHIFT    (0U)
#define LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH_SHIFT)) & LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH_MASK)
#define LCDIF_VDCTRL0_HALF_LINE_MODE_MASK        (0x40000U)
#define LCDIF_VDCTRL0_HALF_LINE_MODE_SHIFT       (18U)
#define LCDIF_VDCTRL0_HALF_LINE_MODE(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_HALF_LINE_MODE_SHIFT)) & LCDIF_VDCTRL0_HALF_LINE_MODE_MASK)
#define LCDIF_VDCTRL0_HALF_LINE_MASK             (0x80000U)
#define LCDIF_VDCTRL0_HALF_LINE_SHIFT            (19U)
#define LCDIF_VDCTRL0_HALF_LINE(x)               (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_HALF_LINE_SHIFT)) & LCDIF_VDCTRL0_HALF_LINE_MASK)
#define LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH_UNIT_MASK (0x100000U)
#define LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH_UNIT_SHIFT (20U)
#define LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH_UNIT(x)  (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH_UNIT_SHIFT)) & LCDIF_VDCTRL0_VSYNC_PULSE_WIDTH_UNIT_MASK)
#define LCDIF_VDCTRL0_VSYNC_PERIOD_UNIT_MASK     (0x200000U)
#define LCDIF_VDCTRL0_VSYNC_PERIOD_UNIT_SHIFT    (21U)
#define LCDIF_VDCTRL0_VSYNC_PERIOD_UNIT(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_VSYNC_PERIOD_UNIT_SHIFT)) & LCDIF_VDCTRL0_VSYNC_PERIOD_UNIT_MASK)
#define LCDIF_VDCTRL0_RSRVD1_MASK                (0xc00000U)
#define LCDIF_VDCTRL0_RSRVD1_SHIFT               (22U)
#define LCDIF_VDCTRL0_RSRVD1(x)                  (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_RSRVD1_SHIFT)) & LCDIF_VDCTRL0_RSRVD1_MASK)
#define LCDIF_VDCTRL0_ENABLE_POL_MASK            (0x1000000U)
#define LCDIF_VDCTRL0_ENABLE_POL_SHIFT           (24U)
#define LCDIF_VDCTRL0_ENABLE_POL(x)              (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_ENABLE_POL_SHIFT)) & LCDIF_VDCTRL0_ENABLE_POL_MASK)
#define LCDIF_VDCTRL0_DOTCLK_POL_MASK            (0x2000000U)
#define LCDIF_VDCTRL0_DOTCLK_POL_SHIFT           (25U)
#define LCDIF_VDCTRL0_DOTCLK_POL(x)              (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_DOTCLK_POL_SHIFT)) & LCDIF_VDCTRL0_DOTCLK_POL_MASK)
#define LCDIF_VDCTRL0_HSYNC_POL_MASK             (0x4000000U)
#define LCDIF_VDCTRL0_HSYNC_POL_SHIFT            (26U)
#define LCDIF_VDCTRL0_HSYNC_POL(x)               (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_HSYNC_POL_SHIFT)) & LCDIF_VDCTRL0_HSYNC_POL_MASK)
#define LCDIF_VDCTRL0_VSYNC_POL_MASK             (0x8000000U)
#define LCDIF_VDCTRL0_VSYNC_POL_SHIFT            (27U)
#define LCDIF_VDCTRL0_VSYNC_POL(x)               (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_VSYNC_POL_SHIFT)) & LCDIF_VDCTRL0_VSYNC_POL_MASK)
#define LCDIF_VDCTRL0_ENABLE_PRESENT_MASK        (0x10000000U)
#define LCDIF_VDCTRL0_ENABLE_PRESENT_SHIFT       (28U)
#define LCDIF_VDCTRL0_ENABLE_PRESENT(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_ENABLE_PRESENT_SHIFT)) & LCDIF_VDCTRL0_ENABLE_PRESENT_MASK)
#define LCDIF_VDCTRL0_RSRVD2_MASK                (0xe0000000U)
#define LCDIF_VDCTRL0_RSRVD2_SHIFT               (29U)
#define LCDIF_VDCTRL0_RSRVD2(x)                  (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_RSRVD2_SHIFT)) & LCDIF_VDCTRL0_RSRVD2_MASK)

/* VDCTRL0_SET - LCDIF VSYNC Mode and Dotclk Mode Control Register0 */

#define LCDIF_VDCTRL0_SET_VSYNC_PULSE_WIDTH_MASK (0x3ffffU)
#define LCDIF_VDCTRL0_SET_VSYNC_PULSE_WIDTH_SHIFT (0U)
#define LCDIF_VDCTRL0_SET_VSYNC_PULSE_WIDTH(x)   (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_SET_VSYNC_PULSE_WIDTH_SHIFT)) & LCDIF_VDCTRL0_SET_VSYNC_PULSE_WIDTH_MASK)
#define LCDIF_VDCTRL0_SET_HALF_LINE_MODE_MASK    (0x40000U)
#define LCDIF_VDCTRL0_SET_HALF_LINE_MODE_SHIFT   (18U)
#define LCDIF_VDCTRL0_SET_HALF_LINE_MODE(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_SET_HALF_LINE_MODE_SHIFT)) & LCDIF_VDCTRL0_SET_HALF_LINE_MODE_MASK)
#define LCDIF_VDCTRL0_SET_HALF_LINE_MASK         (0x80000U)
#define LCDIF_VDCTRL0_SET_HALF_LINE_SHIFT        (19U)
#define LCDIF_VDCTRL0_SET_HALF_LINE(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_SET_HALF_LINE_SHIFT)) & LCDIF_VDCTRL0_SET_HALF_LINE_MASK)
#define LCDIF_VDCTRL0_SET_VSYNC_PULSE_WIDTH_UNIT_MASK (0x100000U)
#define LCDIF_VDCTRL0_SET_VSYNC_PULSE_WIDTH_UNIT_SHIFT (20U)
#define LCDIF_VDCTRL0_SET_VSYNC_PULSE_WIDTH_UNIT(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_SET_VSYNC_PULSE_WIDTH_UNIT_SHIFT)) & LCDIF_VDCTRL0_SET_VSYNC_PULSE_WIDTH_UNIT_MASK)
#define LCDIF_VDCTRL0_SET_VSYNC_PERIOD_UNIT_MASK (0x200000U)
#define LCDIF_VDCTRL0_SET_VSYNC_PERIOD_UNIT_SHIFT (21U)
#define LCDIF_VDCTRL0_SET_VSYNC_PERIOD_UNIT(x)   (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_SET_VSYNC_PERIOD_UNIT_SHIFT)) & LCDIF_VDCTRL0_SET_VSYNC_PERIOD_UNIT_MASK)
#define LCDIF_VDCTRL0_SET_RSRVD1_MASK            (0xc00000U)
#define LCDIF_VDCTRL0_SET_RSRVD1_SHIFT           (22U)
#define LCDIF_VDCTRL0_SET_RSRVD1(x)              (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_SET_RSRVD1_SHIFT)) & LCDIF_VDCTRL0_SET_RSRVD1_MASK)
#define LCDIF_VDCTRL0_SET_ENABLE_POL_MASK        (0x1000000U)
#define LCDIF_VDCTRL0_SET_ENABLE_POL_SHIFT       (24U)
#define LCDIF_VDCTRL0_SET_ENABLE_POL(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_SET_ENABLE_POL_SHIFT)) & LCDIF_VDCTRL0_SET_ENABLE_POL_MASK)
#define LCDIF_VDCTRL0_SET_DOTCLK_POL_MASK        (0x2000000U)
#define LCDIF_VDCTRL0_SET_DOTCLK_POL_SHIFT       (25U)
#define LCDIF_VDCTRL0_SET_DOTCLK_POL(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_SET_DOTCLK_POL_SHIFT)) & LCDIF_VDCTRL0_SET_DOTCLK_POL_MASK)
#define LCDIF_VDCTRL0_SET_HSYNC_POL_MASK         (0x4000000U)
#define LCDIF_VDCTRL0_SET_HSYNC_POL_SHIFT        (26U)
#define LCDIF_VDCTRL0_SET_HSYNC_POL(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_SET_HSYNC_POL_SHIFT)) & LCDIF_VDCTRL0_SET_HSYNC_POL_MASK)
#define LCDIF_VDCTRL0_SET_VSYNC_POL_MASK         (0x8000000U)
#define LCDIF_VDCTRL0_SET_VSYNC_POL_SHIFT        (27U)
#define LCDIF_VDCTRL0_SET_VSYNC_POL(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_SET_VSYNC_POL_SHIFT)) & LCDIF_VDCTRL0_SET_VSYNC_POL_MASK)
#define LCDIF_VDCTRL0_SET_ENABLE_PRESENT_MASK    (0x10000000U)
#define LCDIF_VDCTRL0_SET_ENABLE_PRESENT_SHIFT   (28U)
#define LCDIF_VDCTRL0_SET_ENABLE_PRESENT(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_SET_ENABLE_PRESENT_SHIFT)) & LCDIF_VDCTRL0_SET_ENABLE_PRESENT_MASK)
#define LCDIF_VDCTRL0_SET_RSRVD2_MASK            (0xe0000000U)
#define LCDIF_VDCTRL0_SET_RSRVD2_SHIFT           (29U)
#define LCDIF_VDCTRL0_SET_RSRVD2(x)              (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_SET_RSRVD2_SHIFT)) & LCDIF_VDCTRL0_SET_RSRVD2_MASK)

/* VDCTRL0_CLR - LCDIF VSYNC Mode and Dotclk Mode Control Register0 */

#define LCDIF_VDCTRL0_CLR_VSYNC_PULSE_WIDTH_MASK (0x3ffffU)
#define LCDIF_VDCTRL0_CLR_VSYNC_PULSE_WIDTH_SHIFT (0U)
#define LCDIF_VDCTRL0_CLR_VSYNC_PULSE_WIDTH(x)   (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_CLR_VSYNC_PULSE_WIDTH_SHIFT)) & LCDIF_VDCTRL0_CLR_VSYNC_PULSE_WIDTH_MASK)
#define LCDIF_VDCTRL0_CLR_HALF_LINE_MODE_MASK    (0x40000U)
#define LCDIF_VDCTRL0_CLR_HALF_LINE_MODE_SHIFT   (18U)
#define LCDIF_VDCTRL0_CLR_HALF_LINE_MODE(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_CLR_HALF_LINE_MODE_SHIFT)) & LCDIF_VDCTRL0_CLR_HALF_LINE_MODE_MASK)
#define LCDIF_VDCTRL0_CLR_HALF_LINE_MASK         (0x80000U)
#define LCDIF_VDCTRL0_CLR_HALF_LINE_SHIFT        (19U)
#define LCDIF_VDCTRL0_CLR_HALF_LINE(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_CLR_HALF_LINE_SHIFT)) & LCDIF_VDCTRL0_CLR_HALF_LINE_MASK)
#define LCDIF_VDCTRL0_CLR_VSYNC_PULSE_WIDTH_UNIT_MASK (0x100000U)
#define LCDIF_VDCTRL0_CLR_VSYNC_PULSE_WIDTH_UNIT_SHIFT (20U)
#define LCDIF_VDCTRL0_CLR_VSYNC_PULSE_WIDTH_UNIT(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_CLR_VSYNC_PULSE_WIDTH_UNIT_SHIFT)) & LCDIF_VDCTRL0_CLR_VSYNC_PULSE_WIDTH_UNIT_MASK)
#define LCDIF_VDCTRL0_CLR_VSYNC_PERIOD_UNIT_MASK (0x200000U)
#define LCDIF_VDCTRL0_CLR_VSYNC_PERIOD_UNIT_SHIFT (21U)
#define LCDIF_VDCTRL0_CLR_VSYNC_PERIOD_UNIT(x)   (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_CLR_VSYNC_PERIOD_UNIT_SHIFT)) & LCDIF_VDCTRL0_CLR_VSYNC_PERIOD_UNIT_MASK)
#define LCDIF_VDCTRL0_CLR_RSRVD1_MASK            (0xc00000U)
#define LCDIF_VDCTRL0_CLR_RSRVD1_SHIFT           (22U)
#define LCDIF_VDCTRL0_CLR_RSRVD1(x)              (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_CLR_RSRVD1_SHIFT)) & LCDIF_VDCTRL0_CLR_RSRVD1_MASK)
#define LCDIF_VDCTRL0_CLR_ENABLE_POL_MASK        (0x1000000U)
#define LCDIF_VDCTRL0_CLR_ENABLE_POL_SHIFT       (24U)
#define LCDIF_VDCTRL0_CLR_ENABLE_POL(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_CLR_ENABLE_POL_SHIFT)) & LCDIF_VDCTRL0_CLR_ENABLE_POL_MASK)
#define LCDIF_VDCTRL0_CLR_DOTCLK_POL_MASK        (0x2000000U)
#define LCDIF_VDCTRL0_CLR_DOTCLK_POL_SHIFT       (25U)
#define LCDIF_VDCTRL0_CLR_DOTCLK_POL(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_CLR_DOTCLK_POL_SHIFT)) & LCDIF_VDCTRL0_CLR_DOTCLK_POL_MASK)
#define LCDIF_VDCTRL0_CLR_HSYNC_POL_MASK         (0x4000000U)
#define LCDIF_VDCTRL0_CLR_HSYNC_POL_SHIFT        (26U)
#define LCDIF_VDCTRL0_CLR_HSYNC_POL(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_CLR_HSYNC_POL_SHIFT)) & LCDIF_VDCTRL0_CLR_HSYNC_POL_MASK)
#define LCDIF_VDCTRL0_CLR_VSYNC_POL_MASK         (0x8000000U)
#define LCDIF_VDCTRL0_CLR_VSYNC_POL_SHIFT        (27U)
#define LCDIF_VDCTRL0_CLR_VSYNC_POL(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_CLR_VSYNC_POL_SHIFT)) & LCDIF_VDCTRL0_CLR_VSYNC_POL_MASK)
#define LCDIF_VDCTRL0_CLR_ENABLE_PRESENT_MASK    (0x10000000U)
#define LCDIF_VDCTRL0_CLR_ENABLE_PRESENT_SHIFT   (28U)
#define LCDIF_VDCTRL0_CLR_ENABLE_PRESENT(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_CLR_ENABLE_PRESENT_SHIFT)) & LCDIF_VDCTRL0_CLR_ENABLE_PRESENT_MASK)
#define LCDIF_VDCTRL0_CLR_RSRVD2_MASK            (0xe0000000U)
#define LCDIF_VDCTRL0_CLR_RSRVD2_SHIFT           (29U)
#define LCDIF_VDCTRL0_CLR_RSRVD2(x)              (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_CLR_RSRVD2_SHIFT)) & LCDIF_VDCTRL0_CLR_RSRVD2_MASK)

/* VDCTRL0_TOG - LCDIF VSYNC Mode and Dotclk Mode Control Register0 */

#define LCDIF_VDCTRL0_TOG_VSYNC_PULSE_WIDTH_MASK (0x3ffffU)
#define LCDIF_VDCTRL0_TOG_VSYNC_PULSE_WIDTH_SHIFT (0U)
#define LCDIF_VDCTRL0_TOG_VSYNC_PULSE_WIDTH(x)   (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_TOG_VSYNC_PULSE_WIDTH_SHIFT)) & LCDIF_VDCTRL0_TOG_VSYNC_PULSE_WIDTH_MASK)
#define LCDIF_VDCTRL0_TOG_HALF_LINE_MODE_MASK    (0x40000U)
#define LCDIF_VDCTRL0_TOG_HALF_LINE_MODE_SHIFT   (18U)
#define LCDIF_VDCTRL0_TOG_HALF_LINE_MODE(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_TOG_HALF_LINE_MODE_SHIFT)) & LCDIF_VDCTRL0_TOG_HALF_LINE_MODE_MASK)
#define LCDIF_VDCTRL0_TOG_HALF_LINE_MASK         (0x80000U)
#define LCDIF_VDCTRL0_TOG_HALF_LINE_SHIFT        (19U)
#define LCDIF_VDCTRL0_TOG_HALF_LINE(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_TOG_HALF_LINE_SHIFT)) & LCDIF_VDCTRL0_TOG_HALF_LINE_MASK)
#define LCDIF_VDCTRL0_TOG_VSYNC_PULSE_WIDTH_UNIT_MASK (0x100000U)
#define LCDIF_VDCTRL0_TOG_VSYNC_PULSE_WIDTH_UNIT_SHIFT (20U)
#define LCDIF_VDCTRL0_TOG_VSYNC_PULSE_WIDTH_UNIT(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_TOG_VSYNC_PULSE_WIDTH_UNIT_SHIFT)) & LCDIF_VDCTRL0_TOG_VSYNC_PULSE_WIDTH_UNIT_MASK)
#define LCDIF_VDCTRL0_TOG_VSYNC_PERIOD_UNIT_MASK (0x200000U)
#define LCDIF_VDCTRL0_TOG_VSYNC_PERIOD_UNIT_SHIFT (21U)
#define LCDIF_VDCTRL0_TOG_VSYNC_PERIOD_UNIT(x)   (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_TOG_VSYNC_PERIOD_UNIT_SHIFT)) & LCDIF_VDCTRL0_TOG_VSYNC_PERIOD_UNIT_MASK)
#define LCDIF_VDCTRL0_TOG_RSRVD1_MASK            (0xc00000U)
#define LCDIF_VDCTRL0_TOG_RSRVD1_SHIFT           (22U)
#define LCDIF_VDCTRL0_TOG_RSRVD1(x)              (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_TOG_RSRVD1_SHIFT)) & LCDIF_VDCTRL0_TOG_RSRVD1_MASK)
#define LCDIF_VDCTRL0_TOG_ENABLE_POL_MASK        (0x1000000U)
#define LCDIF_VDCTRL0_TOG_ENABLE_POL_SHIFT       (24U)
#define LCDIF_VDCTRL0_TOG_ENABLE_POL(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_TOG_ENABLE_POL_SHIFT)) & LCDIF_VDCTRL0_TOG_ENABLE_POL_MASK)
#define LCDIF_VDCTRL0_TOG_DOTCLK_POL_MASK        (0x2000000U)
#define LCDIF_VDCTRL0_TOG_DOTCLK_POL_SHIFT       (25U)
#define LCDIF_VDCTRL0_TOG_DOTCLK_POL(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_TOG_DOTCLK_POL_SHIFT)) & LCDIF_VDCTRL0_TOG_DOTCLK_POL_MASK)
#define LCDIF_VDCTRL0_TOG_HSYNC_POL_MASK         (0x4000000U)
#define LCDIF_VDCTRL0_TOG_HSYNC_POL_SHIFT        (26U)
#define LCDIF_VDCTRL0_TOG_HSYNC_POL(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_TOG_HSYNC_POL_SHIFT)) & LCDIF_VDCTRL0_TOG_HSYNC_POL_MASK)
#define LCDIF_VDCTRL0_TOG_VSYNC_POL_MASK         (0x8000000U)
#define LCDIF_VDCTRL0_TOG_VSYNC_POL_SHIFT        (27U)
#define LCDIF_VDCTRL0_TOG_VSYNC_POL(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_TOG_VSYNC_POL_SHIFT)) & LCDIF_VDCTRL0_TOG_VSYNC_POL_MASK)
#define LCDIF_VDCTRL0_TOG_ENABLE_PRESENT_MASK    (0x10000000U)
#define LCDIF_VDCTRL0_TOG_ENABLE_PRESENT_SHIFT   (28U)
#define LCDIF_VDCTRL0_TOG_ENABLE_PRESENT(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_TOG_ENABLE_PRESENT_SHIFT)) & LCDIF_VDCTRL0_TOG_ENABLE_PRESENT_MASK)
#define LCDIF_VDCTRL0_TOG_RSRVD2_MASK            (0xe0000000U)
#define LCDIF_VDCTRL0_TOG_RSRVD2_SHIFT           (29U)
#define LCDIF_VDCTRL0_TOG_RSRVD2(x)              (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL0_TOG_RSRVD2_SHIFT)) & LCDIF_VDCTRL0_TOG_RSRVD2_MASK)

/* VDCTRL1 - LCDIF VSYNC Mode and Dotclk Mode Control Register1 */

#define LCDIF_VDCTRL1_VSYNC_PERIOD_MASK          (0xffffffffU)
#define LCDIF_VDCTRL1_VSYNC_PERIOD_SHIFT         (0U)
#define LCDIF_VDCTRL1_VSYNC_PERIOD(x)            (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL1_VSYNC_PERIOD_SHIFT)) & LCDIF_VDCTRL1_VSYNC_PERIOD_MASK)

/* VDCTRL2 - LCDIF VSYNC Mode and Dotclk Mode Control Register2 */

#define LCDIF_VDCTRL2_HSYNC_PERIOD_MASK          (0x3ffffU)
#define LCDIF_VDCTRL2_HSYNC_PERIOD_SHIFT         (0U)
#define LCDIF_VDCTRL2_HSYNC_PERIOD(x)            (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL2_HSYNC_PERIOD_SHIFT)) & LCDIF_VDCTRL2_HSYNC_PERIOD_MASK)
#define LCDIF_VDCTRL2_HSYNC_PULSE_WIDTH_MASK     (0xfffc0000U)
#define LCDIF_VDCTRL2_HSYNC_PULSE_WIDTH_SHIFT    (18U)
#define LCDIF_VDCTRL2_HSYNC_PULSE_WIDTH(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL2_HSYNC_PULSE_WIDTH_SHIFT)) & LCDIF_VDCTRL2_HSYNC_PULSE_WIDTH_MASK)

/* VDCTRL3 - LCDIF VSYNC Mode and Dotclk Mode Control Register3 */

#define LCDIF_VDCTRL3_VERTICAL_WAIT_CNT_MASK     (0xffffU)
#define LCDIF_VDCTRL3_VERTICAL_WAIT_CNT_SHIFT    (0U)
#define LCDIF_VDCTRL3_VERTICAL_WAIT_CNT(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL3_VERTICAL_WAIT_CNT_SHIFT)) & LCDIF_VDCTRL3_VERTICAL_WAIT_CNT_MASK)
#define LCDIF_VDCTRL3_HORIZONTAL_WAIT_CNT_MASK   (0xfff0000U)
#define LCDIF_VDCTRL3_HORIZONTAL_WAIT_CNT_SHIFT  (16U)
#define LCDIF_VDCTRL3_HORIZONTAL_WAIT_CNT(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL3_HORIZONTAL_WAIT_CNT_SHIFT)) & LCDIF_VDCTRL3_HORIZONTAL_WAIT_CNT_MASK)
#define LCDIF_VDCTRL3_VSYNC_ONLY_MASK            (0x10000000U)
#define LCDIF_VDCTRL3_VSYNC_ONLY_SHIFT           (28U)
#define LCDIF_VDCTRL3_VSYNC_ONLY(x)              (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL3_VSYNC_ONLY_SHIFT)) & LCDIF_VDCTRL3_VSYNC_ONLY_MASK)
#define LCDIF_VDCTRL3_MUX_SYNC_SIGNALS_MASK      (0x20000000U)
#define LCDIF_VDCTRL3_MUX_SYNC_SIGNALS_SHIFT     (29U)
#define LCDIF_VDCTRL3_MUX_SYNC_SIGNALS(x)        (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL3_MUX_SYNC_SIGNALS_SHIFT)) & LCDIF_VDCTRL3_MUX_SYNC_SIGNALS_MASK)
#define LCDIF_VDCTRL3_RSRVD0_MASK                (0xc0000000U)
#define LCDIF_VDCTRL3_RSRVD0_SHIFT               (30U)
#define LCDIF_VDCTRL3_RSRVD0(x)                  (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL3_RSRVD0_SHIFT)) & LCDIF_VDCTRL3_RSRVD0_MASK)

/* VDCTRL4 - LCDIF VSYNC Mode and Dotclk Mode Control Register4 */

#define LCDIF_VDCTRL4_DOTCLK_H_VALID_DATA_CNT_MASK (0x3ffffU)
#define LCDIF_VDCTRL4_DOTCLK_H_VALID_DATA_CNT_SHIFT (0U)
#define LCDIF_VDCTRL4_DOTCLK_H_VALID_DATA_CNT(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL4_DOTCLK_H_VALID_DATA_CNT_SHIFT)) & LCDIF_VDCTRL4_DOTCLK_H_VALID_DATA_CNT_MASK)
#define LCDIF_VDCTRL4_SYNC_SIGNALS_ON_MASK       (0x40000U)
#define LCDIF_VDCTRL4_SYNC_SIGNALS_ON_SHIFT      (18U)
#define LCDIF_VDCTRL4_SYNC_SIGNALS_ON(x)         (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL4_SYNC_SIGNALS_ON_SHIFT)) & LCDIF_VDCTRL4_SYNC_SIGNALS_ON_MASK)
#define LCDIF_VDCTRL4_RSRVD0_MASK                (0x1ff80000U)
#define LCDIF_VDCTRL4_RSRVD0_SHIFT               (19U)
#define LCDIF_VDCTRL4_RSRVD0(x)                  (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL4_RSRVD0_SHIFT)) & LCDIF_VDCTRL4_RSRVD0_MASK)
#define LCDIF_VDCTRL4_DOTCLK_DLY_SEL_MASK        (0xe0000000U)
#define LCDIF_VDCTRL4_DOTCLK_DLY_SEL_SHIFT       (29U)
#define LCDIF_VDCTRL4_DOTCLK_DLY_SEL(x)          (((uint32_t)(((uint32_t)(x)) << LCDIF_VDCTRL4_DOTCLK_DLY_SEL_SHIFT)) & LCDIF_VDCTRL4_DOTCLK_DLY_SEL_MASK)

/* BM_ERROR_STAT - Bus Master Error Status Register */

#define LCDIF_BM_ERROR_STAT_ADDR_MASK            (0xffffffffU)
#define LCDIF_BM_ERROR_STAT_ADDR_SHIFT           (0U)
#define LCDIF_BM_ERROR_STAT_ADDR(x)              (((uint32_t)(((uint32_t)(x)) << LCDIF_BM_ERROR_STAT_ADDR_SHIFT)) & LCDIF_BM_ERROR_STAT_ADDR_MASK)

/* CRC_STAT - CRC Status Register */

#define LCDIF_CRC_STAT_CRC_VALUE_MASK            (0xffffffffU)
#define LCDIF_CRC_STAT_CRC_VALUE_SHIFT           (0U)
#define LCDIF_CRC_STAT_CRC_VALUE(x)              (((uint32_t)(((uint32_t)(x)) << LCDIF_CRC_STAT_CRC_VALUE_SHIFT)) & LCDIF_CRC_STAT_CRC_VALUE_MASK)

/* STAT - LCD Interface Status Register */

#define LCDIF_STAT_LFIFO_COUNT_MASK              (0x1ffU)
#define LCDIF_STAT_LFIFO_COUNT_SHIFT             (0U)
#define LCDIF_STAT_LFIFO_COUNT(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_STAT_LFIFO_COUNT_SHIFT)) & LCDIF_STAT_LFIFO_COUNT_MASK)
#define LCDIF_STAT_RSRVD0_MASK                   (0x1fffe00U)
#define LCDIF_STAT_RSRVD0_SHIFT                  (9U)
#define LCDIF_STAT_RSRVD0(x)                     (((uint32_t)(((uint32_t)(x)) << LCDIF_STAT_RSRVD0_SHIFT)) & LCDIF_STAT_RSRVD0_MASK)
#define LCDIF_STAT_TXFIFO_EMPTY_MASK             (0x4000000U)
#define LCDIF_STAT_TXFIFO_EMPTY_SHIFT            (26U)
#define LCDIF_STAT_TXFIFO_EMPTY(x)               (((uint32_t)(((uint32_t)(x)) << LCDIF_STAT_TXFIFO_EMPTY_SHIFT)) & LCDIF_STAT_TXFIFO_EMPTY_MASK)
#define LCDIF_STAT_TXFIFO_FULL_MASK              (0x8000000U)
#define LCDIF_STAT_TXFIFO_FULL_SHIFT             (27U)
#define LCDIF_STAT_TXFIFO_FULL(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_STAT_TXFIFO_FULL_SHIFT)) & LCDIF_STAT_TXFIFO_FULL_MASK)
#define LCDIF_STAT_LFIFO_EMPTY_MASK              (0x10000000U)
#define LCDIF_STAT_LFIFO_EMPTY_SHIFT             (28U)
#define LCDIF_STAT_LFIFO_EMPTY(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_STAT_LFIFO_EMPTY_SHIFT)) & LCDIF_STAT_LFIFO_EMPTY_MASK)
#define LCDIF_STAT_LFIFO_FULL_MASK               (0x20000000U)
#define LCDIF_STAT_LFIFO_FULL_SHIFT              (29U)
#define LCDIF_STAT_LFIFO_FULL(x)                 (((uint32_t)(((uint32_t)(x)) << LCDIF_STAT_LFIFO_FULL_SHIFT)) & LCDIF_STAT_LFIFO_FULL_MASK)
#define LCDIF_STAT_DMA_REQ_MASK                  (0x40000000U)
#define LCDIF_STAT_DMA_REQ_SHIFT                 (30U)
#define LCDIF_STAT_DMA_REQ(x)                    (((uint32_t)(((uint32_t)(x)) << LCDIF_STAT_DMA_REQ_SHIFT)) & LCDIF_STAT_DMA_REQ_MASK)
#define LCDIF_STAT_PRESENT_MASK                  (0x80000000U)
#define LCDIF_STAT_PRESENT_SHIFT                 (31U)
#define LCDIF_STAT_PRESENT(x)                    (((uint32_t)(((uint32_t)(x)) << LCDIF_STAT_PRESENT_SHIFT)) & LCDIF_STAT_PRESENT_MASK)

/* THRES - LCDIF Threshold Register */

#define LCDIF_THRES_PANIC_MASK                   (0x1ffU)
#define LCDIF_THRES_PANIC_SHIFT                  (0U)
#define LCDIF_THRES_PANIC(x)                     (((uint32_t)(((uint32_t)(x)) << LCDIF_THRES_PANIC_SHIFT)) & LCDIF_THRES_PANIC_MASK)
#define LCDIF_THRES_RSRVD1_MASK                  (0xfe00U)
#define LCDIF_THRES_RSRVD1_SHIFT                 (9U)
#define LCDIF_THRES_RSRVD1(x)                    (((uint32_t)(((uint32_t)(x)) << LCDIF_THRES_RSRVD1_SHIFT)) & LCDIF_THRES_RSRVD1_MASK)
#define LCDIF_THRES_FASTCLOCK_MASK               (0x1ff0000U)
#define LCDIF_THRES_FASTCLOCK_SHIFT              (16U)
#define LCDIF_THRES_FASTCLOCK(x)                 (((uint32_t)(((uint32_t)(x)) << LCDIF_THRES_FASTCLOCK_SHIFT)) & LCDIF_THRES_FASTCLOCK_MASK)
#define LCDIF_THRES_RSRVD2_MASK                  (0xfe000000U)
#define LCDIF_THRES_RSRVD2_SHIFT                 (25U)
#define LCDIF_THRES_RSRVD2(x)                    (((uint32_t)(((uint32_t)(x)) << LCDIF_THRES_RSRVD2_SHIFT)) & LCDIF_THRES_RSRVD2_MASK)

/* PIGEONCTRL0 - LCDIF Pigeon Mode Control0 Register */

#define LCDIF_PIGEONCTRL0_FD_PERIOD_MASK         (0xfffU)
#define LCDIF_PIGEONCTRL0_FD_PERIOD_SHIFT        (0U)
#define LCDIF_PIGEONCTRL0_FD_PERIOD(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL0_FD_PERIOD_SHIFT)) & LCDIF_PIGEONCTRL0_FD_PERIOD_MASK)
#define LCDIF_PIGEONCTRL0_LD_PERIOD_MASK         (0xfff0000U)
#define LCDIF_PIGEONCTRL0_LD_PERIOD_SHIFT        (16U)
#define LCDIF_PIGEONCTRL0_LD_PERIOD(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL0_LD_PERIOD_SHIFT)) & LCDIF_PIGEONCTRL0_LD_PERIOD_MASK)

/* PIGEONCTRL0_SET - LCDIF Pigeon Mode Control0 Register */

#define LCDIF_PIGEONCTRL0_SET_FD_PERIOD_MASK     (0xfffU)
#define LCDIF_PIGEONCTRL0_SET_FD_PERIOD_SHIFT    (0U)
#define LCDIF_PIGEONCTRL0_SET_FD_PERIOD(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL0_SET_FD_PERIOD_SHIFT)) & LCDIF_PIGEONCTRL0_SET_FD_PERIOD_MASK)
#define LCDIF_PIGEONCTRL0_SET_LD_PERIOD_MASK     (0xfff0000U)
#define LCDIF_PIGEONCTRL0_SET_LD_PERIOD_SHIFT    (16U)
#define LCDIF_PIGEONCTRL0_SET_LD_PERIOD(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL0_SET_LD_PERIOD_SHIFT)) & LCDIF_PIGEONCTRL0_SET_LD_PERIOD_MASK)

/* PIGEONCTRL0_CLR - LCDIF Pigeon Mode Control0 Register */

#define LCDIF_PIGEONCTRL0_CLR_FD_PERIOD_MASK     (0xfffU)
#define LCDIF_PIGEONCTRL0_CLR_FD_PERIOD_SHIFT    (0U)
#define LCDIF_PIGEONCTRL0_CLR_FD_PERIOD(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL0_CLR_FD_PERIOD_SHIFT)) & LCDIF_PIGEONCTRL0_CLR_FD_PERIOD_MASK)
#define LCDIF_PIGEONCTRL0_CLR_LD_PERIOD_MASK     (0xfff0000U)
#define LCDIF_PIGEONCTRL0_CLR_LD_PERIOD_SHIFT    (16U)
#define LCDIF_PIGEONCTRL0_CLR_LD_PERIOD(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL0_CLR_LD_PERIOD_SHIFT)) & LCDIF_PIGEONCTRL0_CLR_LD_PERIOD_MASK)

/* PIGEONCTRL0_TOG - LCDIF Pigeon Mode Control0 Register */

#define LCDIF_PIGEONCTRL0_TOG_FD_PERIOD_MASK     (0xfffU)
#define LCDIF_PIGEONCTRL0_TOG_FD_PERIOD_SHIFT    (0U)
#define LCDIF_PIGEONCTRL0_TOG_FD_PERIOD(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL0_TOG_FD_PERIOD_SHIFT)) & LCDIF_PIGEONCTRL0_TOG_FD_PERIOD_MASK)
#define LCDIF_PIGEONCTRL0_TOG_LD_PERIOD_MASK     (0xfff0000U)
#define LCDIF_PIGEONCTRL0_TOG_LD_PERIOD_SHIFT    (16U)
#define LCDIF_PIGEONCTRL0_TOG_LD_PERIOD(x)       (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL0_TOG_LD_PERIOD_SHIFT)) & LCDIF_PIGEONCTRL0_TOG_LD_PERIOD_MASK)

/* PIGEONCTRL1 - LCDIF Pigeon Mode Control1 Register */

#define LCDIF_PIGEONCTRL1_FRAME_CNT_PERIOD_MASK  (0xfffU)
#define LCDIF_PIGEONCTRL1_FRAME_CNT_PERIOD_SHIFT (0U)
#define LCDIF_PIGEONCTRL1_FRAME_CNT_PERIOD(x)    (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL1_FRAME_CNT_PERIOD_SHIFT)) & LCDIF_PIGEONCTRL1_FRAME_CNT_PERIOD_MASK)
#define LCDIF_PIGEONCTRL1_FRAME_CNT_CYCLES_MASK  (0xfff0000U)
#define LCDIF_PIGEONCTRL1_FRAME_CNT_CYCLES_SHIFT (16U)
#define LCDIF_PIGEONCTRL1_FRAME_CNT_CYCLES(x)    (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL1_FRAME_CNT_CYCLES_SHIFT)) & LCDIF_PIGEONCTRL1_FRAME_CNT_CYCLES_MASK)

/* PIGEONCTRL1_SET - LCDIF Pigeon Mode Control1 Register */

#define LCDIF_PIGEONCTRL1_SET_FRAME_CNT_PERIOD_MASK (0xfffU)
#define LCDIF_PIGEONCTRL1_SET_FRAME_CNT_PERIOD_SHIFT (0U)
#define LCDIF_PIGEONCTRL1_SET_FRAME_CNT_PERIOD(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL1_SET_FRAME_CNT_PERIOD_SHIFT)) & LCDIF_PIGEONCTRL1_SET_FRAME_CNT_PERIOD_MASK)
#define LCDIF_PIGEONCTRL1_SET_FRAME_CNT_CYCLES_MASK (0xfff0000U)
#define LCDIF_PIGEONCTRL1_SET_FRAME_CNT_CYCLES_SHIFT (16U)
#define LCDIF_PIGEONCTRL1_SET_FRAME_CNT_CYCLES(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL1_SET_FRAME_CNT_CYCLES_SHIFT)) & LCDIF_PIGEONCTRL1_SET_FRAME_CNT_CYCLES_MASK)

/* PIGEONCTRL1_CLR - LCDIF Pigeon Mode Control1 Register */

#define LCDIF_PIGEONCTRL1_CLR_FRAME_CNT_PERIOD_MASK (0xfffU)
#define LCDIF_PIGEONCTRL1_CLR_FRAME_CNT_PERIOD_SHIFT (0U)
#define LCDIF_PIGEONCTRL1_CLR_FRAME_CNT_PERIOD(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL1_CLR_FRAME_CNT_PERIOD_SHIFT)) & LCDIF_PIGEONCTRL1_CLR_FRAME_CNT_PERIOD_MASK)
#define LCDIF_PIGEONCTRL1_CLR_FRAME_CNT_CYCLES_MASK (0xfff0000U)
#define LCDIF_PIGEONCTRL1_CLR_FRAME_CNT_CYCLES_SHIFT (16U)
#define LCDIF_PIGEONCTRL1_CLR_FRAME_CNT_CYCLES(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL1_CLR_FRAME_CNT_CYCLES_SHIFT)) & LCDIF_PIGEONCTRL1_CLR_FRAME_CNT_CYCLES_MASK)

/* PIGEONCTRL1_TOG - LCDIF Pigeon Mode Control1 Register */

#define LCDIF_PIGEONCTRL1_TOG_FRAME_CNT_PERIOD_MASK (0xfffU)
#define LCDIF_PIGEONCTRL1_TOG_FRAME_CNT_PERIOD_SHIFT (0U)
#define LCDIF_PIGEONCTRL1_TOG_FRAME_CNT_PERIOD(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL1_TOG_FRAME_CNT_PERIOD_SHIFT)) & LCDIF_PIGEONCTRL1_TOG_FRAME_CNT_PERIOD_MASK)
#define LCDIF_PIGEONCTRL1_TOG_FRAME_CNT_CYCLES_MASK (0xfff0000U)
#define LCDIF_PIGEONCTRL1_TOG_FRAME_CNT_CYCLES_SHIFT (16U)
#define LCDIF_PIGEONCTRL1_TOG_FRAME_CNT_CYCLES(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL1_TOG_FRAME_CNT_CYCLES_SHIFT)) & LCDIF_PIGEONCTRL1_TOG_FRAME_CNT_CYCLES_MASK)

/* PIGEONCTRL2 - LCDIF Pigeon Mode Control2 Register */

#define LCDIF_PIGEONCTRL2_PIGEON_DATA_EN_MASK    (0x1U)
#define LCDIF_PIGEONCTRL2_PIGEON_DATA_EN_SHIFT   (0U)
#define LCDIF_PIGEONCTRL2_PIGEON_DATA_EN(x)      (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL2_PIGEON_DATA_EN_SHIFT)) & LCDIF_PIGEONCTRL2_PIGEON_DATA_EN_MASK)
#define LCDIF_PIGEONCTRL2_PIGEON_CLK_GATE_MASK   (0x2U)
#define LCDIF_PIGEONCTRL2_PIGEON_CLK_GATE_SHIFT  (1U)
#define LCDIF_PIGEONCTRL2_PIGEON_CLK_GATE(x)     (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL2_PIGEON_CLK_GATE_SHIFT)) & LCDIF_PIGEONCTRL2_PIGEON_CLK_GATE_MASK)

/* PIGEONCTRL2_SET - LCDIF Pigeon Mode Control2 Register */

#define LCDIF_PIGEONCTRL2_SET_PIGEON_DATA_EN_MASK (0x1U)
#define LCDIF_PIGEONCTRL2_SET_PIGEON_DATA_EN_SHIFT (0U)
#define LCDIF_PIGEONCTRL2_SET_PIGEON_DATA_EN(x)  (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL2_SET_PIGEON_DATA_EN_SHIFT)) & LCDIF_PIGEONCTRL2_SET_PIGEON_DATA_EN_MASK)
#define LCDIF_PIGEONCTRL2_SET_PIGEON_CLK_GATE_MASK (0x2U)
#define LCDIF_PIGEONCTRL2_SET_PIGEON_CLK_GATE_SHIFT (1U)
#define LCDIF_PIGEONCTRL2_SET_PIGEON_CLK_GATE(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL2_SET_PIGEON_CLK_GATE_SHIFT)) & LCDIF_PIGEONCTRL2_SET_PIGEON_CLK_GATE_MASK)

/* PIGEONCTRL2_CLR - LCDIF Pigeon Mode Control2 Register */

#define LCDIF_PIGEONCTRL2_CLR_PIGEON_DATA_EN_MASK (0x1U)
#define LCDIF_PIGEONCTRL2_CLR_PIGEON_DATA_EN_SHIFT (0U)
#define LCDIF_PIGEONCTRL2_CLR_PIGEON_DATA_EN(x)  (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL2_CLR_PIGEON_DATA_EN_SHIFT)) & LCDIF_PIGEONCTRL2_CLR_PIGEON_DATA_EN_MASK)
#define LCDIF_PIGEONCTRL2_CLR_PIGEON_CLK_GATE_MASK (0x2U)
#define LCDIF_PIGEONCTRL2_CLR_PIGEON_CLK_GATE_SHIFT (1U)
#define LCDIF_PIGEONCTRL2_CLR_PIGEON_CLK_GATE(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL2_CLR_PIGEON_CLK_GATE_SHIFT)) & LCDIF_PIGEONCTRL2_CLR_PIGEON_CLK_GATE_MASK)

/* PIGEONCTRL2_TOG - LCDIF Pigeon Mode Control2 Register */

#define LCDIF_PIGEONCTRL2_TOG_PIGEON_DATA_EN_MASK (0x1U)
#define LCDIF_PIGEONCTRL2_TOG_PIGEON_DATA_EN_SHIFT (0U)
#define LCDIF_PIGEONCTRL2_TOG_PIGEON_DATA_EN(x)  (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL2_TOG_PIGEON_DATA_EN_SHIFT)) & LCDIF_PIGEONCTRL2_TOG_PIGEON_DATA_EN_MASK)
#define LCDIF_PIGEONCTRL2_TOG_PIGEON_CLK_GATE_MASK (0x2U)
#define LCDIF_PIGEONCTRL2_TOG_PIGEON_CLK_GATE_SHIFT (1U)
#define LCDIF_PIGEONCTRL2_TOG_PIGEON_CLK_GATE(x) (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEONCTRL2_TOG_PIGEON_CLK_GATE_SHIFT)) & LCDIF_PIGEONCTRL2_TOG_PIGEON_CLK_GATE_MASK)

/* PIGEON_0 - Panel Interface Signal Generator Register */

#define LCDIF_PIGEON_0_EN_MASK                   (0x1U)
#define LCDIF_PIGEON_0_EN_SHIFT                  (0U)
#define LCDIF_PIGEON_0_EN(x)                     (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEON_0_EN_SHIFT)) & LCDIF_PIGEON_0_EN_MASK)
#define LCDIF_PIGEON_0_POL_MASK                  (0x2U)
#define LCDIF_PIGEON_0_POL_SHIFT                 (1U)

/* POL
 *  0b0..Normal Signal (Active high)
 *  0b1..Inverted signal (Active low)
 */

#define LCDIF_PIGEON_0_POL(x)                    (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEON_0_POL_SHIFT)) & LCDIF_PIGEON_0_POL_MASK)
#define LCDIF_PIGEON_0_INC_SEL_MASK              (0xcU)
#define LCDIF_PIGEON_0_INC_SEL_SHIFT             (2U)

/* INC_SEL
 *  0b00..pclk
 *  0b01..Line start pulse
 *  0b10..Frame start pulse
 *  0b11..Use another signal as tick event
 */

#define LCDIF_PIGEON_0_INC_SEL(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEON_0_INC_SEL_SHIFT)) & LCDIF_PIGEON_0_INC_SEL_MASK)
#define LCDIF_PIGEON_0_OFFSET_MASK               (0xf0U)
#define LCDIF_PIGEON_0_OFFSET_SHIFT              (4U)
#define LCDIF_PIGEON_0_OFFSET(x)                 (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEON_0_OFFSET_SHIFT)) & LCDIF_PIGEON_0_OFFSET_MASK)
#define LCDIF_PIGEON_0_MASK_CNT_SEL_MASK         (0xf00U)
#define LCDIF_PIGEON_0_MASK_CNT_SEL_SHIFT        (8U)

/* MASK_CNT_SEL
 *  0b0000..pclk counter within one hscan state
 *  0b0001..pclk cycle within one hscan state
 *  0b0010..line counter within one vscan state
 *  0b0011..line cycle within one vscan state
 *  0b0100..frame counter
 *  0b0101..frame cycle
 *  0b0110..horizontal counter (pclk counter within one line )
 *  0b0111..vertical counter (line counter within one frame)
 */

#define LCDIF_PIGEON_0_MASK_CNT_SEL(x)           (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEON_0_MASK_CNT_SEL_SHIFT)) & LCDIF_PIGEON_0_MASK_CNT_SEL_MASK)
#define LCDIF_PIGEON_0_MASK_CNT_MASK             (0xfff000U)
#define LCDIF_PIGEON_0_MASK_CNT_SHIFT            (12U)
#define LCDIF_PIGEON_0_MASK_CNT(x)               (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEON_0_MASK_CNT_SHIFT)) & LCDIF_PIGEON_0_MASK_CNT_MASK)
#define LCDIF_PIGEON_0_STATE_MASK_MASK           (0xff000000U)
#define LCDIF_PIGEON_0_STATE_MASK_SHIFT          (24U)

/* STATE_MASK
 *  0b00000001..FRAME SYNC
 *  0b00000010..FRAME BEGIN
 *  0b00000100..FRAME DATA
 *  0b00001000..FRAME END
 *  0b00010000..LINE SYNC
 *  0b00100000..LINE BEGIN
 *  0b01000000..LINE DATA
 *  0b10000000..LINE END
 */

#define LCDIF_PIGEON_0_STATE_MASK(x)             (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEON_0_STATE_MASK_SHIFT)) & LCDIF_PIGEON_0_STATE_MASK_MASK)

/* The count of LCDIF_PIGEON_0 */

#define LCDIF_PIGEON_0_COUNT                     (12U)

/* PIGEON_1 - Panel Interface Signal Generator Register */

#define LCDIF_PIGEON_1_SET_CNT_MASK              (0xffffU)
#define LCDIF_PIGEON_1_SET_CNT_SHIFT             (0U)

/* SET_CNT
 *  0b0000000000000000..Start as active
 */

#define LCDIF_PIGEON_1_SET_CNT(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEON_1_SET_CNT_SHIFT)) & LCDIF_PIGEON_1_SET_CNT_MASK)
#define LCDIF_PIGEON_1_CLR_CNT_MASK              (0xffff0000U)
#define LCDIF_PIGEON_1_CLR_CNT_SHIFT             (16U)

/* CLR_CNT
 *  0b0000000000000000..Keep active until mask off
 */

#define LCDIF_PIGEON_1_CLR_CNT(x)                (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEON_1_CLR_CNT_SHIFT)) & LCDIF_PIGEON_1_CLR_CNT_MASK)

/* The count of LCDIF_PIGEON_1 */

#define LCDIF_PIGEON_1_COUNT                     (12U)

/* PIGEON_2 - Panel Interface Signal Generator Register */

#define LCDIF_PIGEON_2_SIG_LOGIC_MASK            (0xfU)
#define LCDIF_PIGEON_2_SIG_LOGIC_SHIFT           (0U)

/* SIG_LOGIC
 *  0b0000..No logic operation
 *  0b0001..sigout = sig_another AND this_sig
 *  0b0010..sigout = sig_another OR this_sig
 *  0b0011..mask = sig_another AND other_masks
 */

#define LCDIF_PIGEON_2_SIG_LOGIC(x)              (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEON_2_SIG_LOGIC_SHIFT)) & LCDIF_PIGEON_2_SIG_LOGIC_MASK)
#define LCDIF_PIGEON_2_SIG_ANOTHER_MASK          (0x1f0U)
#define LCDIF_PIGEON_2_SIG_ANOTHER_SHIFT         (4U)

/* SIG_ANOTHER
 *  0b00000..Keep active until mask off
 */

#define LCDIF_PIGEON_2_SIG_ANOTHER(x)            (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEON_2_SIG_ANOTHER_SHIFT)) & LCDIF_PIGEON_2_SIG_ANOTHER_MASK)
#define LCDIF_PIGEON_2_RSVD_MASK                 (0xfffffe00U)
#define LCDIF_PIGEON_2_RSVD_SHIFT                (9U)
#define LCDIF_PIGEON_2_RSVD(x)                   (((uint32_t)(((uint32_t)(x)) << LCDIF_PIGEON_2_RSVD_SHIFT)) & LCDIF_PIGEON_2_RSVD_MASK)

/* The count of LCDIF_PIGEON_2 */

#define LCDIF_PIGEON_2_COUNT                     (12U)

/* LUT_CTRL - Lookup Table Data Register. */

#define LCDIF_LUT_CTRL_LUT_BYPASS_MASK           (0x1U)
#define LCDIF_LUT_CTRL_LUT_BYPASS_SHIFT          (0U)
#define LCDIF_LUT_CTRL_LUT_BYPASS(x)             (((uint32_t)(((uint32_t)(x)) << LCDIF_LUT_CTRL_LUT_BYPASS_SHIFT)) & LCDIF_LUT_CTRL_LUT_BYPASS_MASK)

/* LUT0_ADDR - Lookup Table Control Register. */

#define LCDIF_LUT0_ADDR_ADDR_MASK                (0xffU)
#define LCDIF_LUT0_ADDR_ADDR_SHIFT               (0U)
#define LCDIF_LUT0_ADDR_ADDR(x)                  (((uint32_t)(((uint32_t)(x)) << LCDIF_LUT0_ADDR_ADDR_SHIFT)) & LCDIF_LUT0_ADDR_ADDR_MASK)

/* LUT0_DATA - Lookup Table Data Register. */

#define LCDIF_LUT0_DATA_DATA_MASK                (0xffffffffU)
#define LCDIF_LUT0_DATA_DATA_SHIFT               (0U)
#define LCDIF_LUT0_DATA_DATA(x)                  (((uint32_t)(((uint32_t)(x)) << LCDIF_LUT0_DATA_DATA_SHIFT)) & LCDIF_LUT0_DATA_DATA_MASK)

/* LUT1_ADDR - Lookup Table Control Register. */

#define LCDIF_LUT1_ADDR_ADDR_MASK                (0xffU)
#define LCDIF_LUT1_ADDR_ADDR_SHIFT               (0U)
#define LCDIF_LUT1_ADDR_ADDR(x)                  (((uint32_t)(((uint32_t)(x)) << LCDIF_LUT1_ADDR_ADDR_SHIFT)) & LCDIF_LUT1_ADDR_ADDR_MASK)

/* LUT1_DATA - Lookup Table Data Register. */

#define LCDIF_LUT1_DATA_DATA_MASK                (0xffffffffU)
#define LCDIF_LUT1_DATA_DATA_SHIFT               (0U)
#define LCDIF_LUT1_DATA_DATA(x)                  (((uint32_t)(((uint32_t)(x)) << LCDIF_LUT1_DATA_DATA_SHIFT)) & LCDIF_LUT1_DATA_DATA_MASK)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_LCD_H */
