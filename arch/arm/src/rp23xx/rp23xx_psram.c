/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_psram.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/****************************************************************************
 * Included Files
 *
 * Driver for the external QSPI PSRAM (an APS6404-family part) fitted on
 * boards such as the Pimoroni Pico Plus 2.  It hangs off QMI chip select 1
 * and, once configured, is directly addressable and writable at
 * RP23XX_PSRAM_BASE (0x11000000).
 *
 * The QMI is the same serial interface that serves execute-in-place from the
 * flash on chip select 0.  Reprogramming it uses direct (non-XIP) mode,
 * which stalls all XIP fetches, so the detection and configuration code must
 * run from RAM with interrupts disabled -- hence the .time_critical section,
 * which the board linker scripts copy to RAM.  The sequence follows the
 * Raspberry Pi Pico SDK's setup_psram().
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stddef.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "rp23xx_gpio.h"
#include "rp23xx_psram.h"
#include "hardware/rp23xx_qmi.h"
#include "hardware/rp23xx_xip.h"
#include "hardware/rp23xx_memorymap.h"

#ifdef CONFIG_RP23XX_PSRAM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Detection and (re)configuration touch the QMI in direct mode, which stalls
 * execute-in-place from the flash.  The code that does so must therefore run
 * from RAM: the board linker scripts copy the .time_critical section there.
 */

#define RP23XX_PSRAM_RAMFUNC \
  __attribute__((section(".time_critical.rp23xx_psram"), noinline))

/* GPIO function select for the XIP chip-select-1 line (PSRAM). */

#define RP23XX_GPIO_FUNC_XIP_CS1  9

/* PSRAM interface width for direct-mode transfers (quad). */

#define QMI_DIRECT_TX_IWIDTH_Q    (2 << RP23XX_QMI_DIRECT_TX_IWIDTH_SHIFT)

/* Final QMI M1 register values for the APS6404 in quad mode.  Computed from
 * the field layout in the RP2350 datasheet to match the Pico SDK:
 *
 * TIMING: COOLDOWN=1, PAGEBREAK=1024, SELECT_HOLD=3, MAX_SELECT=16,
 *         MIN_DESELECT=7, RXDELAY=1, CLKDIV=2.
 * RFMT:   quad prefix/addr/suffix/dummy/data, 8-bit prefix, 24 dummy bits.
 * RCMD:   read prefix 0xeb.
 * WFMT:   quad prefix/addr/suffix/dummy/data, 8-bit prefix, no dummy.
 * WCMD:   write prefix 0x38.
 */

#define RP23XX_PSRAM_M1_TIMING    0x61a07102
#define RP23XX_PSRAM_M1_RFMT      0x000612aa
#define RP23XX_PSRAM_M1_RCMD      0x000000eb
#define RP23XX_PSRAM_M1_WFMT      0x000012aa
#define RP23XX_PSRAM_M1_WCMD      0x00000038

/* APS6404 SPI opcodes used during detection and mode switching. */

#define PSRAM_CMD_READ_ID         0x9f
#define PSRAM_CMD_RSTEN           0x66
#define PSRAM_CMD_RST             0x99
#define PSRAM_CMD_QUAD_ENABLE     0x35

#define PSRAM_KGD_PASS            0x5d

/****************************************************************************
 * Private Data
 ****************************************************************************/

static size_t g_psram_size;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_psram_apply_format
 *
 * Description:
 *   Program the QMI M1 timing/format/command registers and mark the region
 *   writable.  Shared by the initial configuration and the post-flash
 *   restore.  Runs from RAM.
 *
 ****************************************************************************/

static void RP23XX_PSRAM_RAMFUNC
rp23xx_psram_apply_format(void)
{
  putreg32(RP23XX_PSRAM_M1_TIMING, RP23XX_QMI_M1_TIMING);
  putreg32(RP23XX_PSRAM_M1_RFMT,   RP23XX_QMI_M1_RFMT);
  putreg32(RP23XX_PSRAM_M1_RCMD,   RP23XX_QMI_M1_RCMD);
  putreg32(RP23XX_PSRAM_M1_WFMT,   RP23XX_QMI_M1_WFMT);
  putreg32(RP23XX_PSRAM_M1_WCMD,   RP23XX_QMI_M1_WCMD);

  /* Allow the memory-mapped window to accept writes. */

  putreg32(getreg32(RP23XX_XIP_CTRL_BASE) | RP23XX_XIP_CTRL_WRITABLE_M1,
           RP23XX_XIP_CTRL_BASE);
}

/****************************************************************************
 * Name: rp23xx_psram_detect
 *
 * Description:
 *   Read the PSRAM device ID over the QMI direct interface and return the
 *   detected size in bytes, or 0 if the part does not respond correctly.
 *   Runs from RAM with the caller holding interrupts disabled.
 *
 ****************************************************************************/

static size_t RP23XX_PSRAM_RAMFUNC
rp23xx_psram_detect(void)
{
  uint8_t kgd = 0;
  uint8_t eid = 0;
  size_t size;
  size_t i;

  /* Enable direct mode at a conservative clock divisor.  No BUSY wait is
   * needed for the previous XIP transfer's cooldown: it drains before the
   * first chip-select assertion below (verified on hardware).
   */

  putreg32((30 << RP23XX_QMI_DIRECT_CSR_CLKDIV_SHIFT) |
           RP23XX_QMI_DIRECT_CSR_EN, RP23XX_QMI_DIRECT_CSR);

  /* Nudge the part out of any quad-continuation mode left by a prior init by
   * clocking one quad byte with CS asserted.
   */

  putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) |
           RP23XX_QMI_DIRECT_CSR_ASSERT_CS1N, RP23XX_QMI_DIRECT_CSR);
  putreg32(RP23XX_QMI_DIRECT_TX_OE | QMI_DIRECT_TX_IWIDTH_Q | 0xf5,
           RP23XX_QMI_DIRECT_TX);

  /* This BUSY wait is required: the frame must finish shifting before the
   * chip select is deasserted, else the nudge is truncated, the part stays
   * in quad-continuation mode, and the ID read below returns garbage so
   * detection fails (verified on hardware).
   */

  while ((getreg32(RP23XX_QMI_DIRECT_CSR) & RP23XX_QMI_DIRECT_CSR_BUSY) != 0)
    {
    }

  (void)getreg32(RP23XX_QMI_DIRECT_RX);
  putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) &
           ~RP23XX_QMI_DIRECT_CSR_ASSERT_CS1N, RP23XX_QMI_DIRECT_CSR);

  /* Read the identification: command 0x9f followed by six bytes.  The
   * Known-Good-Die marker is byte 5 and the size code is byte 6.
   */

  putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) |
           RP23XX_QMI_DIRECT_CSR_ASSERT_CS1N, RP23XX_QMI_DIRECT_CSR);
  for (i = 0; i < 7; i++)
    {
      putreg32(i == 0 ? PSRAM_CMD_READ_ID : 0xff, RP23XX_QMI_DIRECT_TX);

      /* This BUSY wait is required: the byte must finish shifting in before
       * DIRECT_RX is read, else the KGD/EID bytes are sampled early as
       * garbage and detection fails (verified on hardware).  A TXEMPTY wait
       * ahead of it is redundant -- BUSY already covers the whole transfer.
       */

      while ((getreg32(RP23XX_QMI_DIRECT_CSR) &
              RP23XX_QMI_DIRECT_CSR_BUSY) != 0)
        {
        }

      if (i == 5)
        {
          kgd = (uint8_t)getreg32(RP23XX_QMI_DIRECT_RX);
        }
      else if (i == 6)
        {
          eid = (uint8_t)getreg32(RP23XX_QMI_DIRECT_RX);
        }
      else
        {
          (void)getreg32(RP23XX_QMI_DIRECT_RX);
        }
    }

  putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) &
           ~(RP23XX_QMI_DIRECT_CSR_ASSERT_CS1N | RP23XX_QMI_DIRECT_CSR_EN),
           RP23XX_QMI_DIRECT_CSR);

  if (kgd != PSRAM_KGD_PASS)
    {
      return 0;
    }

  /* Reset the device and place it in quad mode: RSTEN, RST, quad-enable. */

  putreg32((30 << RP23XX_QMI_DIRECT_CSR_CLKDIV_SHIFT) |
           RP23XX_QMI_DIRECT_CSR_EN, RP23XX_QMI_DIRECT_CSR);

  for (i = 0; i < 3; i++)
    {
      /* The command byte is selected with a ternary rather than a lookup
       * table on purpose: a "static const" array is emitted to .rodata in
       * flash, and reading it here -- while the QMI is in direct mode and
       * flash is inaccessible -- would fault.  An immediate keeps it in RAM.
       */

      uint8_t cmd = (i == 0) ? PSRAM_CMD_RSTEN :
                    (i == 1) ? PSRAM_CMD_RST   :
                               PSRAM_CMD_QUAD_ENABLE;

      putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) |
               RP23XX_QMI_DIRECT_CSR_ASSERT_CS1N, RP23XX_QMI_DIRECT_CSR);
      putreg32(cmd, RP23XX_QMI_DIRECT_TX);

      /* This BUSY wait is required: the command must finish shifting before
       * the chip select is deasserted.  Cutting it short truncates the
       * RSTEN/RST/quad-enable commands, so the part is not reset into quad
       * mode and later accesses fail or wedge the shared QMI bus (verified
       * on hardware).  No extra settling delay is needed after CS deassert:
       * the QMI's own minimum-deselect timing covers it.
       */

      while ((getreg32(RP23XX_QMI_DIRECT_CSR) &
              RP23XX_QMI_DIRECT_CSR_BUSY) != 0)
        {
        }

      putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) &
               ~RP23XX_QMI_DIRECT_CSR_ASSERT_CS1N, RP23XX_QMI_DIRECT_CSR);

      (void)getreg32(RP23XX_QMI_DIRECT_RX);
    }

  putreg32(getreg32(RP23XX_QMI_DIRECT_CSR) &
           ~(RP23XX_QMI_DIRECT_CSR_ASSERT_CS1N | RP23XX_QMI_DIRECT_CSR_EN),
           RP23XX_QMI_DIRECT_CSR);

  /* Decode the size from the EID byte (APS6404 family). */

  size = 1024 * 1024;
  if (eid == 0x26 || (eid >> 5) == 2)
    {
      size *= 8;
    }
  else if ((eid >> 5) == 0)
    {
      size *= 2;
    }
  else if ((eid >> 5) == 1)
    {
      size *= 4;
    }

  return size;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_psramconfig
 ****************************************************************************/

size_t RP23XX_PSRAM_RAMFUNC
rp23xx_psramconfig(void)
{
  irqstate_t flags;
  size_t size;

  rp23xx_gpio_set_function(CONFIG_RP23XX_PSRAM_CS1_GPIO,
                           RP23XX_GPIO_FUNC_XIP_CS1);

  flags = up_irq_save();

  size = rp23xx_psram_detect();
  if (size != 0)
    {
      rp23xx_psram_apply_format();
    }

  up_irq_restore(flags);

  g_psram_size = size;
  return size;
}

/****************************************************************************
 * Name: rp23xx_psram_size
 ****************************************************************************/

size_t rp23xx_psram_size(void)
{
  return g_psram_size;
}

/****************************************************************************
 * Name: rp23xx_psram_restore
 ****************************************************************************/

void RP23XX_PSRAM_RAMFUNC
rp23xx_psram_restore(void)
{
  if (g_psram_size != 0)
    {
      rp23xx_psram_apply_format();
    }
}

#endif /* CONFIG_RP23XX_PSRAM */
