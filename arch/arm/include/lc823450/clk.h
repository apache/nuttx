/****************************************************************************
 * arch/arm/include/lc823450/clk.h
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

/****************************************************************************
 * Public Types
 ****************************************************************************/

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

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

void up_enable_clk(enum clock_e clk);
void up_disable_clk(enum clock_e clk);

#endif /* __ARCH_ARM_INCLUDE_LC823450_CLK_H */
