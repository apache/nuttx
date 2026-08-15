/****************************************************************************
 * arch/risc-v/src/eic7700x/eic7700x_reset.c
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
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <assert.h>
#include <sys/param.h>

#include <nuttx/debug.h>
#include <nuttx/reset/reset-controller.h>

#include "riscv_internal.h"
#include "eic7700x_reset.h"
#include "hardware/eic7700x_reset.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* One table row.  Writing the register index into the designated
 * initialiser keeps a row's position in the array visibly equal to the
 * register index that a reset line id carries in its upper bits.
 */

#define RSTREG(i, v, r, c) \
  [i] = \
  { \
    .valid = (v), .rdonly = (r), .critical = (c) \
  }

/* System critical means a line whose assertion takes down the system that
 * is asserting it: the fabric that carries instruction fetch, the memory
 * the kernel runs from, the cluster it runs on, the configuration path
 * back to this very register block, and the pipeline and translation
 * buffer cells inside those paths.  Those lines are registered so that a
 * driver can still release them and read their state, but assert and
 * reset return -EPERM.
 *
 * The line is drawn at "will hang", not "might upset something".  Section
 * 3.1 of the manual warns that resetting any block that is not idle can
 * hang the bus, which is true of every line in the table and so cannot be
 * the criterion here.  Defining each critical mask in terms of the
 * matching valid mask keeps the two in step by construction.
 */

#define SNOC_RST_CRIT              SNOC_RST_VALID
#define DDR_RST_CRIT               DDR_RST_VALID
#define TCU_RST_CRIT               TCU_RST_VALID
#define LSPCFG_RST_CRIT            LSPCFG_RST_VALID
#define U84_RST_CRIT               U84_RST_VALID
#define MNOC_RST_CRIT              MNOC_RST_VALID
#define RNOC_RST_CRIT              RNOC_RST_VALID
#define CNOC_RST_CRIT              CNOC_RST_VALID
#define LNOC_RST_CRIT              LNOC_RST_VALID
#define PIPE_RST_CRIT              PIPE_RST_VALID
#define TBU_RST_CRIT               TBU_RST_VALID

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The reset controller.  The framework does not allocate this and there is
 * no per instance state to carry: the register base is a compile time
 * constant and everything else falls out of the line id, so a file scope
 * object is all that is needed and no container_of is involved.
 */

static struct reset_controller_dev g_eic7700x_rcdev =
{
  .name = EIC7700X_RESET_CONTROLLER,
  .ops  = &g_eic7700x_reset_ops,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Which bits of each control register name a reset line, which of those
 * the hardware will not let software drive, and which of them refuse to be
 * asserted.  Rows not listed are all zero, so every id in them is
 * rejected, which is what the registers this SoC does not have resolve to.
 */

const struct eic7700x_reset_reg_s
g_eic7700x_reset_regs[EIC7700X_RESET_NREGS] =
{
  RSTREG(0x00, SNOC_RST_VALID,   0,                SNOC_RST_CRIT),
  RSTREG(0x01, GPU_RST_VALID,    0,                0),
  RSTREG(0x02, DSP_RST_VALID,    0,                0),
  RSTREG(0x03, D2D_RST_VALID,    0,                0),
  RSTREG(0x04, DDR_RST_VALID,    0,                DDR_RST_CRIT),
  RSTREG(0x05, TCU_RST_VALID,    0,                TCU_RST_CRIT),
  RSTREG(0x06, NPU_RST_VALID,    0,                0),
  RSTREG(0x07, HSPDMA_RST_VALID, 0,                0),
  RSTREG(0x08, PCIE_RST_VALID,   0,                0),
  RSTREG(0x09, I2C_RST_VALID,    0,                0),
  RSTREG(0x0a, FAN_RST_VALID,    0,                0),
  RSTREG(0x0b, PVT_RST_VALID,    0,                0),
  RSTREG(0x0c, MBOX_RST_VALID,   0,                0),
  RSTREG(0x0d, UART_RST_VALID,   0,                0),
  RSTREG(0x0e, GPIO_RST_VALID,   0,                0),
  RSTREG(0x0f, TIMER_RST_VALID,  0,                0),
  RSTREG(0x10, SSI_RST_VALID,    0,                0),
  RSTREG(0x11, WDT_RST_VALID,    0,                0),
  RSTREG(0x12, LSPCFG_RST_VALID, 0,                LSPCFG_RST_CRIT),
  RSTREG(0x13, U84_RST_VALID,    0,                U84_RST_CRIT),
  RSTREG(0x14, SCPU_RST_VALID,   0,                0),
  RSTREG(0x15, LPCPU_RST_VALID,  0,                0),
  RSTREG(0x16, VC_RST_VALID,     0,                0),
  RSTREG(0x17, JD_RST_VALID,     0,                0),
  RSTREG(0x18, JE_RST_VALID,     0,                0),
  RSTREG(0x19, VD_RST_VALID,     0,                0),
  RSTREG(0x1a, VE_RST_VALID,     0,                0),
  RSTREG(0x1b, G2D_RST_VALID,    0,                0),
  RSTREG(0x1c, VI_RST_VALID,     0,                0),
  RSTREG(0x1d, DVP_RST_VALID,    0,                0),
  RSTREG(0x1e, ISP0_RST_VALID,   0,                0),
  RSTREG(0x1f, ISP1_RST_VALID,   0,                0),
  RSTREG(0x20, SHUTTER_RST_VALID, 0,               0),
  RSTREG(0x21, VOPHY_RST_VALID,  0,                0),
  RSTREG(0x22, VOI2S_RST_VALID,  0,                0),
  RSTREG(0x23, VO_RST_VALID,     0,                0),
  RSTREG(0x24, BOOTSPI_RST_VALID, 0,               0),
  RSTREG(0x25, I2C1_RST_VALID,   0,                0),
  RSTREG(0x26, I2C0_RST_VALID,   0,                0),
  RSTREG(0x27, DMA1_RST_VALID,   0,                0),
  RSTREG(0x28, FPRT_RST_VALID,   0,                0),
  RSTREG(0x29, HBLOCK_RST_VALID, 0,                0),
  RSTREG(0x2a, SECSR_RST_VALID,  0,                0),
  RSTREG(0x2b, OTP_RST_VALID,    0,                0),
  RSTREG(0x2c, PKA_RST_VALID,    0,                0),
  RSTREG(0x2d, SPACC_RST_VALID,  0,                0),
  RSTREG(0x2e, TRNG_RST_VALID,   0,                0),

  /* 0x4bc is reserved.  The manual documents nothing at this offset
   * and the vendor Linux binding names the index RESERVED, so the row
   * is written out explicitly rather than left to the zero fill: any
   * id landing in it is rejected, and the gap is visible in the table.
   */

  RSTREG(0x2f, 0,                0,                0),

  RSTREG(0x30, TIMER0_RST_VALID, 0,                0),
  RSTREG(0x31, TIMER1_RST_VALID, 0,                0),
  RSTREG(0x32, TIMER2_RST_VALID, 0,                0),
  RSTREG(0x33, TIMER3_RST_VALID, 0,                0),
  RSTREG(0x34, RTC_RST_VALID,    0,                0),
  RSTREG(0x35, MNOC_RST_VALID,   0,                MNOC_RST_CRIT),
  RSTREG(0x36, RNOC_RST_VALID,   0,                RNOC_RST_CRIT),
  RSTREG(0x37, CNOC_RST_VALID,   0,                CNOC_RST_CRIT),
  RSTREG(0x38, LNOC_RST_VALID,   0,                LNOC_RST_CRIT),
  RSTREG(0x39, PIPE_RST_VALID,   0,                PIPE_RST_CRIT),
  RSTREG(0x3a, TBU_RST_VALID,    0,                TBU_RST_CRIT),
  RSTREG(0x3b, STATUS_RST_VALID, STATUS_RST_VALID, 0),
  RSTREG(0x3c, TEST_RST_VALID,   0,                0),
};

/* The table has to describe exactly the id space the public header
 * promises, and the last row has to land on the last control register.
 */

static_assert(nitems(g_eic7700x_reset_regs) == EIC7700X_RESET_NREGS,
              "reset table length must match the id space bound");
static_assert(EIC7700X_RESET_CTRL(EIC7700X_RESET_NREGS - 1) ==
              EIC7700X_RESET_BASE + 0x00f0,
              "the last reset control register is at offset 0x4f0");

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: eic7700x_reset_initialize
 *
 * Description:
 *   Register the reset lines of the Clock and Reset Generator with the
 *   reset framework.  See eic7700x_reset.h.
 *
 ****************************************************************************/

/* What eic7700x_reset_initialize() found, for the board to report */

static unsigned int g_eic7700x_reset_count;
static unsigned int g_eic7700x_reset_held;

int eic7700x_reset_initialize(void)
{
  size_t reg;
  int ret;

#ifdef CONFIG_DEBUG_ASSERTIONS
  size_t i;

  /* A critical or read only bit that names no line, or a line marked as
   * both, means a mistyped mask.  Either would show up on hardware as a
   * reset that silently does nothing, which is a poor way to find out.
   */

  for (i = 0; i < nitems(g_eic7700x_reset_regs); i++)
    {
      FAR const struct eic7700x_reset_reg_s *desc =
        &g_eic7700x_reset_regs[i];

      DEBUGASSERT((desc->critical & ~desc->valid) == 0);
      DEBUGASSERT((desc->rdonly & ~desc->valid) == 0);
      DEBUGASSERT((desc->critical & desc->rdonly) == 0);
    }
#endif

  ret = reset_controller_register(&g_eic7700x_rcdev);
  if (ret < 0)
    {
      rsterr("failed to register the reset controller: %d\n", ret);
      return ret;
    }

  rstinfo("registered %s\n", g_eic7700x_rcdev.name);

  /* Count what the boot loader left, one register read per register
   * rather than one per line.  The lines are active low, so a valid bit
   * reading zero is a line still held.
   */

  for (reg = 0; reg < nitems(g_eic7700x_reset_regs); reg++)
    {
      uint32_t valid = g_eic7700x_reset_regs[reg].valid;
      uint32_t raw = getreg32(EIC7700X_RESET_CTRL(reg));

      g_eic7700x_reset_count += __builtin_popcount(valid);
      g_eic7700x_reset_held += __builtin_popcount(valid & ~raw);
    }

  return OK;
}

/****************************************************************************
 * Name: eic7700x_reset_count
 *
 * Description:
 *   What eic7700x_reset_initialize() found.  See eic7700x_reset.h.
 *
 ****************************************************************************/

unsigned int eic7700x_reset_count(FAR unsigned int *held)
{
  if (held != NULL)
    {
      *held = g_eic7700x_reset_held;
    }

  return g_eic7700x_reset_count;
}
