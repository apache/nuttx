/****************************************************************************
 * arch/arm/include/lc823450/clk.h
 *
 *   Copyright (C) 2014-2017 Sony Corporation. All rights reserved.
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
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

#ifndef __ARCH_ARM_INCLUDE_LC823450_CLK_H
#define __ARCH_ARM_INCLUDE_LC823450_CLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LC823450_CLOCKS \
  { \
    {0, "dmac", MCLKCNTBASIC, MCLKCNTBASIC_DMAC_CLKEN}, \
    {0, "mtm0", MCLKCNTEXT1, MCLKCNTEXT1_MTM0_CLKEN}, \
    {0, "mtm0c", MCLKCNTEXT1, MCLKCNTEXT1_MTM0C_CLKEN}, \
    {0, "mtm1", MCLKCNTEXT1, MCLKCNTEXT1_MTM1_CLKEN}, \
    {0, "mtm1c", MCLKCNTEXT1, MCLKCNTEXT1_MTM1C_CLKEN}, \
  }


/************************************************************************************
 * Public Types
 ************************************************************************************/

struct clk_st
{
  int count;
  char *name;
  uint32_t regaddr;
  uint32_t regmask;
};

enum clock_e
{
  LC823450_CLOCK_DMA = 0,
  LC823450_CLOCK_MTM0,
  LC823450_CLOCK_MTM0C,
  LC823450_CLOCK_MTM1,
  LC823450_CLOCK_MTM1C,

  LC823450_CLOCK_NUM,
};


/************************************************************************************
 * Public Functions
 ************************************************************************************/

void up_enable_clk(enum clock_e clk);
void up_disable_clk(enum clock_e clk);

#endif /* __ARCH_ARM_INCLUDE_LC823450_CLK_H */
