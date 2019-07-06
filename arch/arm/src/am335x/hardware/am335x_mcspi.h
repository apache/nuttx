/************************************************************************************
 * arch/arm/src/am335x/hardware/am335x_mcspi.h
 *
 *   Copyright (C) 2019 Petro Karashchenko. All rights reserved.
 *   Author: Petro Karashchenko <petro.karashchenko@gmail.com>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_MCSPI_H
#define __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_MCSPI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/am335x_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define AM335X_MCSPI_SYSC_OFFSET        0x0110
#define AM335X_MCSPI_SYSS_OFFSET        0x0114
#define AM335X_MCSPI_IRQ_STAT_OFFSET    0x0118
#define AM335X_MCSPI_IRQ_EN_OFFSET      0x011c
#define AM335X_MCSPI_SYST_OFFSET        0x0124
#define AM335X_MCSPI_MODUL_CTRL_OFFSET  0x0128
#define AM335X_MCSPI_CH0_CONF_OFFSET    0x012c
#define AM335X_MCSPI_CH0_STAT_OFFSET    0x0130
#define AM335X_MCSPI_CH0_CTRL_OFFSET    0x0134
#define AM335X_MCSPI_TX0_OFFSET         0x0138
#define AM335X_MCSPI_RX0_OFFSET         0x013c
#define AM335X_MCSPI_CH1_CONF_OFFSET    0x0140
#define AM335X_MCSPI_CH1_STAT_OFFSET    0x0144
#define AM335X_MCSPI_CH1_CTRL_OFFSET    0x0148
#define AM335X_MCSPI_TX1_OFFSET         0x014c
#define AM335X_MCSPI_RX1_OFFSET         0x0150
#define AM335X_MCSPI_CH2_CONF_OFFSET    0x0154
#define AM335X_MCSPI_CH2_STAT_OFFSET    0x0158
#define AM335X_MCSPI_CH2_CTRL_OFFSET    0x015c
#define AM335X_MCSPI_TX2_OFFSET         0x0160
#define AM335X_MCSPI_RX2_OFFSET         0x0164
#define AM335X_MCSPI_CH3_CONF_OFFSET    0x0168
#define AM335X_MCSPI_CH3_STAT_OFFSET    0x016c
#define AM335X_MCSPI_CH3_CTRL_OFFSET    0x0170
#define AM335X_MCSPI_TX3_OFFSET         0x0174
#define AM335X_MCSPI_RX3_OFFSET         0x0178
#define AM335X_MCSPI_XFER_LEVEL_OFFSET  0x017c
#define AM335X_MCSPI_DAFTX_OFFSET       0x0180
#define AM335X_MCSPI_DAFRX_OFFSET       0x01a0

#define AM335X_MCSPI_CH_CONF_OFFSET(n)  (0x012c + (unsigned int)(n) * 0x14)
#define AM335X_MCSPI_CH_STAT_OFFSET(n)  (0x0130 + (unsigned int)(n) * 0x14)
#define AM335X_MCSPI_CH_CTRL_OFFSET(n)  (0x0134 + (unsigned int)(n) * 0x14)
#define AM335X_MCSPI_TX_OFFSET(n)       (0x0138 + (unsigned int)(n) * 0x14)
#define AM335X_MCSPI_RX_OFFSET(n)       (0x013c + (unsigned int)(n) * 0x14)

/* Register virtual addresses *******************************************************/

#define AM335X_MCSPI0_SYSC              (AM335X_MCSPI0_VADDR + AM335X_MCSPI_SYSC_OFFSET)
#define AM335X_MCSPI0_SYSS              (AM335X_MCSPI0_VADDR + AM335X_MCSPI_SYSS_OFFSET)
#define AM335X_MCSPI0_IRQ_STAT          (AM335X_MCSPI0_VADDR + AM335X_MCSPI_IRQ_STAT_OFFSET)
#define AM335X_MCSPI0_IRQ_EN            (AM335X_MCSPI0_VADDR + AM335X_MCSPI_IRQ_EN_OFFSET)
#define AM335X_MCSPI0_SYST              (AM335X_MCSPI0_VADDR + AM335X_MCSPI_SYST_OFFSET)
#define AM335X_MCSPI0_MODUL_CTRL        (AM335X_MCSPI0_VADDR + AM335X_MCSPI_MODUL_CTRL_OFFSET)
#define AM335X_MCSPI0_CH0_CONF          (AM335X_MCSPI0_VADDR + AM335X_MCSPI_CH0_CONF_OFFSET)
#define AM335X_MCSPI0_CH0_STAT          (AM335X_MCSPI0_VADDR + AM335X_MCSPI_CH0_STAT_OFFSET)
#define AM335X_MCSPI0_CH0_CTRL          (AM335X_MCSPI0_VADDR + AM335X_MCSPI_CH0_CTRL_OFFSET)
#define AM335X_MCSPI0_TX0               (AM335X_MCSPI0_VADDR + AM335X_MCSPI_TX0_OFFSET)
#define AM335X_MCSPI0_RX0               (AM335X_MCSPI0_VADDR + AM335X_MCSPI_RX0_OFFSET)
#define AM335X_MCSPI0_CH1_CONF          (AM335X_MCSPI0_VADDR + AM335X_MCSPI_CH1_CONF_OFFSET)
#define AM335X_MCSPI0_CH1_STAT          (AM335X_MCSPI0_VADDR + AM335X_MCSPI_CH1_STAT_OFFSET)
#define AM335X_MCSPI0_CH1_CTRL          (AM335X_MCSPI0_VADDR + AM335X_MCSPI_CH1_CTRL_OFFSET)
#define AM335X_MCSPI0_TX1               (AM335X_MCSPI0_VADDR + AM335X_MCSPI_TX1_OFFSET)
#define AM335X_MCSPI0_RX1               (AM335X_MCSPI0_VADDR + AM335X_MCSPI_RX1_OFFSET)
#define AM335X_MCSPI0_CH2_CONF          (AM335X_MCSPI0_VADDR + AM335X_MCSPI_CH2_CONF_OFFSET)
#define AM335X_MCSPI0_CH2_STAT          (AM335X_MCSPI0_VADDR + AM335X_MCSPI_CH2_STAT_OFFSET)
#define AM335X_MCSPI0_CH2_CTRL          (AM335X_MCSPI0_VADDR + AM335X_MCSPI_CH2_CTRL_OFFSET)
#define AM335X_MCSPI0_TX2               (AM335X_MCSPI0_VADDR + AM335X_MCSPI_TX2_OFFSET)
#define AM335X_MCSPI0_RX2               (AM335X_MCSPI0_VADDR + AM335X_MCSPI_RX2_OFFSET)
#define AM335X_MCSPI0_CH3_CONF          (AM335X_MCSPI0_VADDR + AM335X_MCSPI_CH3_CONF_OFFSET)
#define AM335X_MCSPI0_CH3_STAT          (AM335X_MCSPI0_VADDR + AM335X_MCSPI_CH3_STAT_OFFSET)
#define AM335X_MCSPI0_CH3_CTRL          (AM335X_MCSPI0_VADDR + AM335X_MCSPI_CH3_CTRL_OFFSET)
#define AM335X_MCSPI0_TX3               (AM335X_MCSPI0_VADDR + AM335X_MCSPI_TX3_OFFSET)
#define AM335X_MCSPI0_RX3               (AM335X_MCSPI0_VADDR + AM335X_MCSPI_RX3_OFFSET)
#define AM335X_MCSPI0_XFER_LEVEL        (AM335X_MCSPI0_VADDR + AM335X_MCSPI_XFER_LEVEL_OFFSET)
#define AM335X_MCSPI0_DAFTX             (AM335X_MCSPI0_VADDR + AM335X_MCSPI_DAFTX_OFFSET)
#define AM335X_MCSPI0_DAFRX             (AM335X_MCSPI0_VADDR + AM335X_MCSPI_DAFRX_OFFSET)

#define AM335X_MCSPI0_CH_CONF(n)        (AM335X_MCSPI0_VADDR + AM335X_MCSPI_CH_CONF_OFFSET(n))
#define AM335X_MCSPI0_CH_STAT(n)        (AM335X_MCSPI0_VADDR + AM335X_MCSPI_CH_STAT_OFFSET(n))
#define AM335X_MCSPI0_CH_CTRL(n)        (AM335X_MCSPI0_VADDR + AM335X_MCSPI_CH_CTRL_OFFSET(n))
#define AM335X_MCSPI0_TX(n)             (AM335X_MCSPI0_VADDR + AM335X_MCSPI_TX_OFFSET(n))
#define AM335X_MCSPI0_RX(n)             (AM335X_MCSPI0_VADDR + AM335X_MCSPI_RX_OFFSET(n))

#define AM335X_MCSPI1_SYSC              (AM335X_MCSPI1_VADDR + AM335X_MCSPI_SYSC_OFFSET)
#define AM335X_MCSPI1_SYSS              (AM335X_MCSPI1_VADDR + AM335X_MCSPI_SYSS_OFFSET)
#define AM335X_MCSPI1_IRQ_STAT          (AM335X_MCSPI1_VADDR + AM335X_MCSPI_IRQ_STAT_OFFSET)
#define AM335X_MCSPI1_IRQ_EN            (AM335X_MCSPI1_VADDR + AM335X_MCSPI_IRQ_EN_OFFSET)
#define AM335X_MCSPI1_SYST              (AM335X_MCSPI1_VADDR + AM335X_MCSPI_SYST_OFFSET)
#define AM335X_MCSPI1_MODUL_CTRL        (AM335X_MCSPI1_VADDR + AM335X_MCSPI_MODUL_CTRL_OFFSET)
#define AM335X_MCSPI1_CH0_CONF          (AM335X_MCSPI1_VADDR + AM335X_MCSPI_CH0_CONF_OFFSET)
#define AM335X_MCSPI1_CH0_STAT          (AM335X_MCSPI1_VADDR + AM335X_MCSPI_CH0_STAT_OFFSET)
#define AM335X_MCSPI1_CH0_CTRL          (AM335X_MCSPI1_VADDR + AM335X_MCSPI_CH0_CTRL_OFFSET)
#define AM335X_MCSPI1_TX0               (AM335X_MCSPI1_VADDR + AM335X_MCSPI_TX0_OFFSET)
#define AM335X_MCSPI1_RX0               (AM335X_MCSPI1_VADDR + AM335X_MCSPI_RX0_OFFSET)
#define AM335X_MCSPI1_CH1_CONF          (AM335X_MCSPI1_VADDR + AM335X_MCSPI_CH1_CONF_OFFSET)
#define AM335X_MCSPI1_CH1_STAT          (AM335X_MCSPI1_VADDR + AM335X_MCSPI_CH1_STAT_OFFSET)
#define AM335X_MCSPI1_CH1_CTRL          (AM335X_MCSPI1_VADDR + AM335X_MCSPI_CH1_CTRL_OFFSET)
#define AM335X_MCSPI1_TX1               (AM335X_MCSPI1_VADDR + AM335X_MCSPI_TX1_OFFSET)
#define AM335X_MCSPI1_RX1               (AM335X_MCSPI1_VADDR + AM335X_MCSPI_RX1_OFFSET)
#define AM335X_MCSPI1_CH2_CONF          (AM335X_MCSPI1_VADDR + AM335X_MCSPI_CH2_CONF_OFFSET)
#define AM335X_MCSPI1_CH2_STAT          (AM335X_MCSPI1_VADDR + AM335X_MCSPI_CH2_STAT_OFFSET)
#define AM335X_MCSPI1_CH2_CTRL          (AM335X_MCSPI1_VADDR + AM335X_MCSPI_CH2_CTRL_OFFSET)
#define AM335X_MCSPI1_TX2               (AM335X_MCSPI1_VADDR + AM335X_MCSPI_TX2_OFFSET)
#define AM335X_MCSPI1_RX2               (AM335X_MCSPI1_VADDR + AM335X_MCSPI_RX2_OFFSET)
#define AM335X_MCSPI1_CH3_CONF          (AM335X_MCSPI1_VADDR + AM335X_MCSPI_CH3_CONF_OFFSET)
#define AM335X_MCSPI1_CH3_STAT          (AM335X_MCSPI1_VADDR + AM335X_MCSPI_CH3_STAT_OFFSET)
#define AM335X_MCSPI1_CH3_CTRL          (AM335X_MCSPI1_VADDR + AM335X_MCSPI_CH3_CTRL_OFFSET)
#define AM335X_MCSPI1_TX3               (AM335X_MCSPI1_VADDR + AM335X_MCSPI_TX3_OFFSET)
#define AM335X_MCSPI1_RX3               (AM335X_MCSPI1_VADDR + AM335X_MCSPI_RX3_OFFSET)
#define AM335X_MCSPI1_XFER_LEVEL        (AM335X_MCSPI1_VADDR + AM335X_MCSPI_XFER_LEVEL_OFFSET)
#define AM335X_MCSPI1_DAFTX             (AM335X_MCSPI1_VADDR + AM335X_MCSPI_DAFTX_OFFSET)
#define AM335X_MCSPI1_DAFRX             (AM335X_MCSPI1_VADDR + AM335X_MCSPI_DAFRX_OFFSET)

#define AM335X_MCSPI1_CH_CONF(n)        (AM335X_MCSPI1_VADDR + AM335X_MCSPI_CH_CONF_OFFSET(n))
#define AM335X_MCSPI1_CH_STAT(n)        (AM335X_MCSPI1_VADDR + AM335X_MCSPI_CH_STAT_OFFSET(n))
#define AM335X_MCSPI1_CH_CTRL(n)        (AM335X_MCSPI1_VADDR + AM335X_MCSPI_CH_CTRL_OFFSET(n))
#define AM335X_MCSPI1_TX(n)             (AM335X_MCSPI1_VADDR + AM335X_MCSPI_TX_OFFSET(n))
#define AM335X_MCSPI1_RX(n)             (AM335X_MCSPI1_VADDR + AM335X_MCSPI_RX_OFFSET(n))

/* Register bit field definitions ***************************************************/

#define MCSPI_SYSC_AUTOIDLE             (1 << 0)  /* Bit 0:  Internal OCP Clock gating strategy */
#define MCSPI_SYSC_SRST                 (1 << 1)  /* Bit 1:  Software reset */
#define MCSPI_SYSC_IDLE_SHIFT           (3)  /* Bits 3-4:  Power management */
#define MCSPI_SYSC_IDLE_MASK            (3 << MCSPI_SYSC_IDLE_MASK)
#  define MCSPI_SYSC_IDLE_FORCE         (0 << MCSPI_SYSC_IDLE_SHIFT) /* Force-idle mode */
#  define MCSPI_SYSC_IDLE_NO            (1 << MCSPI_SYSC_IDLE_SHIFT) /* No-idle mode */
#  define MCSPI_SYSC_IDLE_SMART         (2 << MCSPI_SYSC_IDLE_SHIFT) /* Smart-idle mode */
#define MCSPI_SYSC_CLK_SHIFT            (8)  /* Bits 8-9:  Clocks activity during wake-up mode period */
#define MCSPI_SYSC_CLK_MASK             (3 << MCSPI_SYSC_CLK_SHIFT)
#  define MCSPI_SYSC_CLK_NONE           (0 << MCSPI_SYSC_CLK_SHIFT) /* Both clocks may be switched off */
#  define MCSPI_SYSC_CLK_OCP            (1 << MCSPI_SYSC_CLK_SHIFT) /* Only OCP clock is maintained */
#  define MCSPI_SYSC_CLK_FUNC           (2 << MCSPI_SYSC_CLK_SHIFT) /* Only functions clock is maintained */
#  define MCSPI_SYSC_CLK_BOTH           (3 << MCSPI_SYSC_CLK_SHIFT) /* Both clocks are maintained */

#define MCSPI_SYSS_RST_DONE             (1 << 0)  /* Bit 0:  Reset done */

#define MCSPI_IRQ_TX0_EMPTY             (0)  /* Bit 0:  Channel 0 transmitter register empty or almost empty */
#define MCSPI_IRQ_TX0_UFLOW             (1)  /* Bit 1:  Channel 0 transmitter register underflow */
#define MCSPI_IRQ_RX0_FULL              (2)  /* Bit 2:  Channel 0 receiver register full or almost full */
#define MCSPI_IRQ_RX0_OFLOW             (3)  /* Bit 3:  Channel 0 receiver register overflow (slave mode only). */
#define MCSPI_IRQ_TX1_EMPTY             (4)  /* Bit 4:  Channel 1 transmitter register empty or almost empty */
#define MCSPI_IRQ_TX1_UFLOW             (5)  /* Bit 5:  Channel 1 transmitter register underflow */
#define MCSPI_IRQ_RX1_FULL              (6)  /* Bit 6:  Channel 1 receiver register full or almost full */
#define MCSPI_IRQ_TX2_EMPTY             (8)  /* Bit 8:  Channel 2 transmitter register empty or almost empty */
#define MCSPI_IRQ_TX2_UFLOW             (9)  /* Bit 9:  Channel 2 transmitter register underflow */
#define MCSPI_IRQ_RX2_FULL              (10)  /* Bit 10:  Channel 2 receiver register full or almost full */
#define MCSPI_IRQ_TX3_EMPTY             (12)  /* Bit 12:  Channel 3 transmitter register empty or almost empty */
#define MCSPI_IRQ_TX3_UFLOW             (13)  /* Bit 13:  Channel 3 transmitter register underflow */
#define MCSPI_IRQ_RX3_FULL              (14)  /* Bit 14:  Channel 3 receiver register full or almost full */
#define MCSPI_IRQ_EOW                   (17)  /* Bit 17:  End of word (EOW) count event */

#define MCSPI_SYST_SPIEN0               (1 << 0)  /* Bit 0:  SPIEN[0] line (signal data value) */
#define MCSPI_SYST_SPIEN1               (1 << 1)  /* Bit 1:  SPIEN[1] line (signal data value) */
#define MCSPI_SYST_SPIEN2               (1 << 2)  /* Bit 2:  SPIEN[2] line (signal data value) */
#define MCSPI_SYST_SPIEN3               (1 << 3)  /* Bit 3:  SPIEN[3] line (signal data value) */
#define MCSPI_SYST_SPIDAT0              (1 << 4)  /* Bit 4:  SPIDAT[0] line (signal data value) */
#define MCSPI_SYST_SPIDAT1              (1 << 5)  /* Bit 5:  SPIDAT[1] line (signal data value) */
#define MCSPI_SYST_SPICLK               (1 << 6)  /* Bit 6:  SPICLK line (signal data value) */
#define MCSPI_SYST_SPIDAT_DIR0          (1 << 8)  /* Bit 8:  Sets the direction of the SPIDAT[0] */
#define MCSPI_SYST_SPIDAT_DIR1          (1 << 9)  /* Bit 9:  Sets the direction of the SPIDAT[1] */
#define MCSPI_SYST_SPIEN_DIR            (1 << 10)  /* Bit 10:  Sets the direction of the SPIEN */
#define MCSPI_SYST_SSB                  (1 << 11)  /* Bit 11:  Set status bit */

#define MCSPI_MODUL_CTRL_SINGLE         (1 << 0)  /* Bit 0:  Single Channel / Multi Channel selection */
#define MCSPI_MODUL_CTRL_PIN34          (1 << 1)  /* Bit 1:  Pin mode selection */
#define MCSPI_MODUL_CTRL_MS             (1 << 2)  /* Bit 2:  Master / Slave selection */
#define MCSPI_MODUL_CTRL_SYS_TEST       (1 << 3)  /* Bit 3:  Enables the system test mode */
#define MCSPI_MODUL_CTRL_IDLY_SHIFT     (4)  /* Bits 4-6: Initial SPI delay for first transfer */
#define MCSPI_MODUL_CTRL_IDLY_MASK      (15 << MCSPI_MODUL_CTRL_INIT_DLY_SHIFT)
#  define MCSPI_MODUL_CTRL_IDLY_NONE    (0 << MCSPI_MODUL_CTRL_INIT_DLY_SHIFT) /* No delay for first SPI transfer */
#  define MCSPI_MODUL_CTRL_IDLY_4CLK    (1 << MCSPI_MODUL_CTRL_INIT_DLY_SHIFT) /* The controller wait 4 SPI bus clock */
#  define MCSPI_MODUL_CTRL_IDLY_8CLK    (2 << MCSPI_MODUL_CTRL_INIT_DLY_SHIFT) /* The controller wait 8 SPI bus clock */
#  define MCSPI_MODUL_CTRL_IDLY_16CLK   (3 << MCSPI_MODUL_CTRL_INIT_DLY_SHIFT) /* The controller wait 16 SPI bus clock */
#  define MCSPI_MODUL_CTRL_IDLY_32CLK   (4 << MCSPI_MODUL_CTRL_INIT_DLY_SHIFT) /*The controller wait 32 SPI bus clock */
#define MCSPI_MODUL_CTRL_MOA            (1 << 7)  /* Bit 7:  Multiple word OCP access */
#define MCSPI_MODUL_CTRL_FDAA           (1 << 8)  /* Bit 8:  FIFO DMA Address 256 bit aligned */

#define MCSPI_CH_CONF_PHA               (1 << 0)  /* Bit 0:  SPICLK phase */
#define MCSPI_CH_CONF_POL               (1 << 1)  /* Bit 1:  SPICLK polarity */
#define MCSPI_CH_CONF_CLKD_SHIFT        (2)  /* Bits 2-5:  Frequency divider for SPICLK */
#define MCSPI_CH_CONF_CLKD_MASK         (15 << MCSPI_CH_CONF_CLKD_SHIFT)
#  define MCSPI_CH_CONF_CLKD_DIV1       (0 << MCSPI_CH_CONF_CLKD_SHIFT) /* Divide by 1 */
#  define MCSPI_CH_CONF_CLKD_DIV2       (1 << MCSPI_CH_CONF_CLKD_SHIFT) /* Divide by 2 */
#  define MCSPI_CH_CONF_CLKD_DIV4       (2 << MCSPI_CH_CONF_CLKD_SHIFT) /* Divide by 4 */
#  define MCSPI_CH_CONF_CLKD_DIV8       (3 << MCSPI_CH_CONF_CLKD_SHIFT) /* Divide by 8 */
#  define MCSPI_CH_CONF_CLKD_DIV16      (4 << MCSPI_CH_CONF_CLKD_SHIFT) /* Divide by 16 */
#  define MCSPI_CH_CONF_CLKD_DIV32      (5 << MCSPI_CH_CONF_CLKD_SHIFT) /* Divide by 32 */
#  define MCSPI_CH_CONF_CLKD_DIV64      (6 << MCSPI_CH_CONF_CLKD_SHIFT) /* Divide by 64 */
#  define MCSPI_CH_CONF_CLKD_DIV128     (7 << MCSPI_CH_CONF_CLKD_SHIFT) /* Divide by 128 */
#  define MCSPI_CH_CONF_CLKD_DIV256     (8 << MCSPI_CH_CONF_CLKD_SHIFT) /* Divide by 256 */
#  define MCSPI_CH_CONF_CLKD_DIV512     (9 << MCSPI_CH_CONF_CLKD_SHIFT) /* Divide by 512 */
#  define MCSPI_CH_CONF_CLKD_DIV1K      (10 << MCSPI_CH_CONF_CLKD_SHIFT) /* Divide by 1024 */
#  define MCSPI_CH_CONF_CLKD_DIV2K      (11 << MCSPI_CH_CONF_CLKD_SHIFT) /* Divide by 2048 */
#  define MCSPI_CH_CONF_CLKD_DIV4K      (12 << MCSPI_CH_CONF_CLKD_SHIFT) /* Divide by 4096 */
#  define MCSPI_CH_CONF_CLKD_DIV8K      (13 << MCSPI_CH_CONF_CLKD_SHIFT) /* Divide by 8192 */
#  define MCSPI_CH_CONF_CLKD_DIV16K     (14 << MCSPI_CH_CONF_CLKD_SHIFT) /* Divide by 16384 */
#  define MCSPI_CH_CONF_CLKD_DIV32K     (15 << MCSPI_CH_CONF_CLKD_SHIFT) /* Divide by 32768 */
#define MCSPI_CH_CONF_EPOL              (1 << 6)  /* Bit 6:  SPIEN polarity */
#define MCSPI_CH_CONF_WL_SHIFT          (7)  /* Bits 7-11:  SPI word length */
#define MCSPI_CH_CONF_WL_MASK           (31 << MCSPI_CH_CONF_WL_SHIFT)
#  define MCSPI_CH_CONF_WL_4BITS        (3 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 4-bits long */
#  define MCSPI_CH_CONF_WL_5BITS        (4 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 5-bits long */
#  define MCSPI_CH_CONF_WL_6BITS        (5 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 6-bits long */
#  define MCSPI_CH_CONF_WL_7BITS        (6 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 7-bits long */
#  define MCSPI_CH_CONF_WL_8BITS        (7 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 8-bits long */
#  define MCSPI_CH_CONF_WL_9BITS        (8 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 9-bits long */
#  define MCSPI_CH_CONF_WL_10BITS       (9 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 10-bits long */
#  define MCSPI_CH_CONF_WL_11BITS       (10 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 11-bits long */
#  define MCSPI_CH_CONF_WL_12BITS       (11 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 12-bits long */
#  define MCSPI_CH_CONF_WL_13BITS       (12 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 13-bits long */
#  define MCSPI_CH_CONF_WL_14BITS       (13 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 14-bits long */
#  define MCSPI_CH_CONF_WL_15BITS       (14 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 15-bits long */
#  define MCSPI_CH_CONF_WL_16BITS       (15 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 16-bits long */
#  define MCSPI_CH_CONF_WL_17BITS       (16 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 17-bits long */
#  define MCSPI_CH_CONF_WL_18BITS       (17 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 18-bits long */
#  define MCSPI_CH_CONF_WL_19BITS       (18 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 19-bits long */
#  define MCSPI_CH_CONF_WL_20BITS       (19 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 20-bits long */
#  define MCSPI_CH_CONF_WL_21BITS       (20 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 21-bits long */
#  define MCSPI_CH_CONF_WL_22BITS       (21 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 22-bits long */
#  define MCSPI_CH_CONF_WL_23BITS       (22 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 23-bits long */
#  define MCSPI_CH_CONF_WL_24BITS       (23 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 24-bits long */
#  define MCSPI_CH_CONF_WL_25BITS       (24 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 25-bits long */
#  define MCSPI_CH_CONF_WL_26BITS       (25 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 26-bits long */
#  define MCSPI_CH_CONF_WL_27BITS       (26 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 27-bits long */
#  define MCSPI_CH_CONF_WL_28BITS       (27 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 28-bits long */
#  define MCSPI_CH_CONF_WL_29BITS       (28 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 29-bits long */
#  define MCSPI_CH_CONF_WL_30BITS       (29 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 30-bits long */
#  define MCSPI_CH_CONF_WL_31BITS       (30 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 31-bits long */
#  define MCSPI_CH_CONF_WL_32BITS       (31 << MCSPI_CH_CONF_WL_SHIFT) /* The SPI word is 32-bits long */
#define MCSPI_CH_CONF_TRM_SHIFT         (12)  /* Bits 12-13:  Transmit/receive modes */
#define MCSPI_CH_CONF_TRM_MASK          (3 << MCSPI_CH_CONF_TRM_SHIFT)
#  define MCSPI_CH_CONF_TRM_RXTX        (0 << MCSPI_CH_CONF_TRM_SHIFT) /* Transmit and receive mode */
#  define MCSPI_CH_CONF_TRM_RX          (1 << MCSPI_CH_CONF_TRM_SHIFT) /* Receive-only mode */
#  define MCSPI_CH_CONF_TRM_TX          (2 << MCSPI_CH_CONF_TRM_SHIFT) /* Transmit-only mode */
#define MCSPI_CH_CONF_DMAW              (1 << 14)  /* Bit 14:  DMA write request */
#define MCSPI_CH_CONF_DMAR              (1 << 15)  /* Bit 15:  DMA read request */
#define MCSPI_CH_CONF_DPE0              (1 << 16)  /* Bit 16:  Transmission enable for data line 0 */
#define MCSPI_CH_CONF_DPE1              (1 << 17)  /* Bit 17:  Transmission enable for data line 1 */
#define MCSPI_CH_CONF_IS                (1 << 18)  /* Bit 18:  Input select */
#define MCSPI_CH_CONF_TURBO             (1 << 19)  /* Bit 19:  Turbo mode */
#define MCSPI_CH_CONF_FORCE             (1 << 20)  /* Bit 20:  Manual SPIEN assertion to keep SPIEN active between SPI words */
#define MCSPI_CH_CONF_SPIENSLV_SHIFT    (21)  /* Bits 21-22:  SPI slave select signal detection */
#define MCSPI_CH_CONF_SPIENSLV_MASK     (3 << MCSPI_CH_CONF_SPIENSLV_SHIFT)
#  define MCSPI_CH_CONF_SPIENSLV0       (0 << MCSPI_CH_CONF_SPIENSLV_SHIFT) /* Detection enabled only on SPIEN[0] */
#  define MCSPI_CH_CONF_SPIENSLV1       (1 << MCSPI_CH_CONF_SPIENSLV_SHIFT) /* Detection enabled only on SPIEN[1] */
#  define MCSPI_CH_CONF_SPIENSLV2       (2 << MCSPI_CH_CONF_SPIENSLV_SHIFT) /* Detection enabled only on SPIEN[2] */
#  define MCSPI_CH_CONF_SPIENSLV3       (3 << MCSPI_CH_CONF_SPIENSLV_SHIFT) /* Detection enabled only on SPIEN[3] */
#define MCSPI_CH_CONF_SBE               (1 << 23)  /* Bit 23:  Start bit enable for SPI transfer */
#define MCSPI_CH_CONF_SBPOL             (1 << 24)  /* Bit 24:  Start bit polarity */
#define MCSPI_CH_CONF_TCS_SHIFT         (25)  /* Bits 25-26:  Chip select time control */
#define MCSPI_CH_CONF_TCS_MASK          (3 << MCSPI_CH_CONF_TCS_SHIFT)
#  define MCSPI_CH_CONF_TCS_0P5         (0 << MCSPI_CH_CONF_TCS_SHIFT) /* 0.5 clock cycles */
#  define MCSPI_CH_CONF_TCS_1P5         (1 << MCSPI_CH_CONF_TCS_SHIFT) /* 1.5 clock cycles */
#  define MCSPI_CH_CONF_TCS_2P5         (2 << MCSPI_CH_CONF_TCS_SHIFT) /* 2.5 clock cycles */
#  define MCSPI_CH_CONF_TCS_3P5         (3 << MCSPI_CH_CONF_TCS_SHIFT) /* 3.5 clock cycles */
#define MCSPI_CH_CONF_FFEW              (1 << 27)  /* Bit 27:  FIFO enabled for transmit */
#define MCSPI_CH_CONF_FFER              (1 << 28)  /* Bit 28:  FIFO enabled for receive */
#define MCSPI_CH_CONF_CLKG              (1 << 29)  /* Bit 29:  Clock divider granularity */

#define MCSPI_CH_STAT_RXS               (1 << 0)  /* Bit 0:  Channel receiver register status */
#define MCSPI_CH_STAT_TXS               (1 << 1)  /* Bit 1:  Channel transmitter register status */
#define MCSPI_CH_STAT_EOT               (1 << 2)  /* Bit 2:  Channel end-of-transfer status */
#define MCSPI_CH_STAT_TXFFE             (1 << 3)  /* Bit 3:  Channel FIFO transmit buffer empty status */
#define MCSPI_CH_STAT_TXFFF             (1 << 4)  /* Bit 4:  Channel FIFO transmit buffer full status */
#define MCSPI_CH_STAT_RXFFE             (1 << 5)  /* Bit 5:  Channel FIFO receive buffer empty status */
#define MCSPI_CH_STAT_RXFFF             (1 << 6)  /* Bit 6:  Channel FIFO receive buffer full status */

#define MCSPI_CH_CTRL_EN                (1 << 0)  /* Bit 0:  Channel enable */
#define MCSPI_CH_CTRL_EXTCLK_SHFT       (8)  /* Bits 8-15: Clock ratio extension */
#define MCSPI_CH_CTRL_EXTCLK_MASK       (255 << MCSPI_CH_CTRL_EXTCLK_SHFT)

#define MCSPI_XFER_LEVEL_AEL_SHIFT      (0)  /* Bits 0-7:  Buffer almost empty */
#define MCSPI_XFER_LEVEL_AEL_MASK       (255 << MCSPI_XFER_LEVEL_AEL_SHIFT)
#define MCSPI_XFER_LEVEL_AFL_SHIFT      (8)  /* Bits 8-15:  Buffer almost full */
#define MCSPI_XFER_LEVEL_AFL_MASK       (255 << MCSPI_XFER_LEVEL_AFL_SHIFT)
#define MCSPI_XFER_LEVEL_WCNT_SHIFT     (16)  /* Bits 16-31:  SPI word counter */
#define MCSPI_XFER_LEVEL_WCNT_MASK      (65535 << MCSPI_XFER_LEVEL_WCNT_SHIFT)

#endif /* __ARCH_ARM_SRC_AM335X_HARDWARE_AM335X_MCSPI_H */
