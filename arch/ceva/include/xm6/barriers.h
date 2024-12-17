/****************************************************************************
 * arch/ceva/include/xm6/barriers.h
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

#ifndef __ARCH_CEVA_INCLUDE_XM6_BARRIERS_H
#define __ARCH_CEVA_INCLUDE_XM6_BARRIERS_H

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

static inline void up_dsb(void)
{
  /* MSS_BARRIER(0x638):
   * Bit [7] Internal Barrier Activation
   */
#define MSS_BARRIER 0x638

  uint32_t barrier = 0x80;

  __asm__ __volatile__
  (
    "out {cpm} %0.ui, (%1.ui).ui"
     : : "r"(barrier), "r"(MSS_BARRIER)
  );

  do
    {
      __asm__ __volatile__
      (
        "in {cpm} (%1.ui).ui, %0.ui\n"
        "nop #0x04\nnop #0x02"
        : "=r"(barrier)
        : "r"(MSS_BARRIER)
      );

      /* Wait unitl the barrier operation complete */
    }
  while ((barrier & 0x80) != 0);
#undef MSS_BARRIER
}

static inline void up_dmb(void)
{
  up_dsb(); /* use dsb instead since dmb doesn't exist on xm6 */
}

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_CEVA_INCLUDE_M65_BARRIERS_H */
