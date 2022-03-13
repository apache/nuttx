/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_region.c
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

#include <stdint.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint32_t g_protected_pages[] =
{
  0x00000000, 0x80000000, 0xa0000000, 0xc0000000, 0xe0000000
};

#define NPROTECTED_PAGES (sizeof(g_protected_pages)/sizeof(uint32_t))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_write_dtlb and xtensa_write_itlb
 *
 * Description:
 *   Functions to set page attributes for Region Protection option in the
 *   CPU.  See Xtensa ISA Reference manual for explanation of arguments
 *   (section 4.6.3.2).
 *
 ****************************************************************************/

static inline void xtensa_write_dtlb(uint32_t vpn, uint32_t attr)
{
  __asm__ __volatile__
  (
    "wdtlb  %1, %0\n"
    "dsync\n"
    : : "r" (vpn), "r" (attr)
  );
}

static inline void xtensa_write_itlb(uint32_t vpn, uint32_t attr)
{
  __asm__ __volatile__
  (
    "witlb  %1, %0\n"
    "isync\n"
    : : "r" (vpn), "r" (attr)
  );
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_region_protection
 *
 * Description:
 *   Make page 0 access raise an exception.  Also protect some other unused
 *   pages so we can catch weirdness.
 *
 *   Useful attribute values:
 *     0  — cached, RW
 *     2  — bypass cache, RWX (default value after CPU reset)
 *     15 — no access, raise exception
 *
 ****************************************************************************/

void esp32s3_region_protection(void)
{
  int i;

  for (i = 0; i < NPROTECTED_PAGES; ++i)
    {
      xtensa_write_dtlb(g_protected_pages[i], 0xf);
      xtensa_write_itlb(g_protected_pages[i], 0xf);
    }

  xtensa_write_dtlb(0x20000000, 0);
  xtensa_write_itlb(0x20000000, 0);
}
