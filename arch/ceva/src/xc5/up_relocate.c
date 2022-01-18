/****************************************************************************
 * arch/ceva/src/xc5/up_relocate.c
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

#include "cpm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MSS_PDEA            0x004
#define MSS_PDIA            0x008
#define MSS_PDTC            0x00c
#define MSS_PDTC_MASK       0xfffff
#define MSS_PDTC_PDST       (1 << 29)
#define MSS_DDEA             0x208
#define MSS_DDIA             0x20c
#define MSS_DDTC             0x210
#define MSS_DDTC_MASK        0x1fffff
#define MSS_DDTC_PDST        (1 << 29)
#define MSS_DDTC_BSZ_SHIFT   25
#define MSS_DDTC_DDIR_SHIFT  30

#define BSZ_1_TRANS          (0 << MSS_DDTC_BSZ_SHIFT)
#define BSZ_4_TRANS          (6 << MSS_DDTC_BSZ_SHIFT)
#define BSZ_8_TRANS          (10 << MSS_DDTC_BSZ_SHIFT)
#define BSZ_16_TRANS         (14 << MSS_DDTC_BSZ_SHIFT)

#define DDIR_EX2IN           (0 << MSS_DDTC_DDIR_SHIFT)
#define DDIR_IN2EX           (1 << MSS_DDTC_DDIR_SHIFT)

#define _START_INTTBL        ((void *)&_sinttbl)

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern char _sinttbl;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void pdma_config(unsigned long iaddr, unsigned long eaddr,
                        uint32_t count)
{
  putcpm(MSS_PDIA, iaddr);
  putcpm(MSS_PDEA, eaddr);
  putcpm(MSS_PDTC, count & MSS_PDTC_MASK);
}

static void pdma_wait_idle(void)
{
  while (getcpm(MSS_PDTC) & MSS_PDTC_PDST);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_relocate(void)
{
  pdma_config(0, (unsigned long)_START_INTTBL, 0x320);
  pdma_wait_idle();
}
