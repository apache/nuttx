/****************************************************************************
 * arch/arm64/src/common/arm64_hwdebug.h
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
 *
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_COMMON_ARM64_HWDEBUG_H
#define __ARCH_ARM64_SRC_COMMON_ARM64_HWDEBUG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/bits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARM64_DBGBWCR_E                   BIT(0)

#define ARM64_DBGBWCR_LSC_OFFSET          3
#define ARM64_DBGBWCR_LSC_EXECUTE         0
#define ARM64_DBGBWCR_LSC_LOAD            1
#define ARM64_DBGBWCR_LSC_STORE           2

#define ARM64_DBGBWCR_BAS_OFFSET          5
#define ARM64_DBGBWCR_BAS_LEN_1           0x1
#define ARM64_DBGBWCR_BAS_LEN_2           0x3
#define ARM64_DBGBWCR_BAS_LEN_3           0x7
#define ARM64_DBGBWCR_BAS_LEN_4           0xf
#define ARM64_DBGBWCR_BAS_LEN_5           0x1f
#define ARM64_DBGBWCR_BAS_LEN_6           0x3f
#define ARM64_DBGBWCR_BAS_LEN_7           0x7f
#define ARM64_DBGBWCR_BAS_LEN_8           0xff

#define ARM64_DBGBCR_PAC_PRIV             BIT(1)
#define ARM64_DBGBCR_PAC_USER             BIT(2)
#define ARM64_DBGBCR_PAC_ALL              (ARM64_DBGBCR_PAC_PRIV | ARM64_DBGBCR_PAC_USER)

#define ARM64_MDSCR_EL1_KDE               BIT(13)
#define ARM64_MDSCR_EL1_MDE               BIT(15)

#define ID_AA64DFR0_EL1_BRPS_MASK         0xf
#define ID_AA64DFR0_EL1_BRPS_OFFSET       12

#define ID_AA64DFR0_EL1_WRPS_MASK         0xf
#define ID_AA64DFR0_EL1_WRPS_OFFSET       20

#define ID_AA64DFR0_MAX_BRPS              16
#define ID_AA64DFR0_MAX_WRPS              16

#define ARM64_DBG_SET_CASE(reg, n, val)   \
  case n:                                 \
    write_sysreg(val, dbg##reg##n##_el1); \
    break;

#define ARM64_DBG_GET_CASE(reg, n, val)   \
  case n:                                 \
    val = read_sysreg(dbg##reg##n##_el1); \
    break;

#define ARM64_DBG_SETN(reg, n, val)       \
  switch (n)                              \
    {                                     \
      ARM64_DBG_SET_CASE(reg, 0, val)     \
      ARM64_DBG_SET_CASE(reg, 1, val)     \
      ARM64_DBG_SET_CASE(reg, 2, val)     \
      ARM64_DBG_SET_CASE(reg, 3, val)     \
      ARM64_DBG_SET_CASE(reg, 4, val)     \
      ARM64_DBG_SET_CASE(reg, 5, val)     \
      ARM64_DBG_SET_CASE(reg, 6, val)     \
      ARM64_DBG_SET_CASE(reg, 7, val)     \
      ARM64_DBG_SET_CASE(reg, 8, val)     \
      ARM64_DBG_SET_CASE(reg, 9, val)     \
      ARM64_DBG_SET_CASE(reg, 10, val)    \
      ARM64_DBG_SET_CASE(reg, 11, val)    \
      ARM64_DBG_SET_CASE(reg, 12, val)    \
      ARM64_DBG_SET_CASE(reg, 13, val)    \
      ARM64_DBG_SET_CASE(reg, 14, val)    \
      ARM64_DBG_SET_CASE(reg, 15, val)    \
    }

#define ARM64_DBG_GETN(reg, n)            \
  ({                                      \
    uint64_t _val = 0;                    \
    switch (n)                            \
      {                                   \
        ARM64_DBG_GET_CASE(reg, 0, _val)  \
        ARM64_DBG_GET_CASE(reg, 1, _val)  \
        ARM64_DBG_GET_CASE(reg, 2, _val)  \
        ARM64_DBG_GET_CASE(reg, 3, _val)  \
        ARM64_DBG_GET_CASE(reg, 4, _val)  \
        ARM64_DBG_GET_CASE(reg, 5, _val)  \
        ARM64_DBG_GET_CASE(reg, 6, _val)  \
        ARM64_DBG_GET_CASE(reg, 7, _val)  \
        ARM64_DBG_GET_CASE(reg, 8, _val)  \
        ARM64_DBG_GET_CASE(reg, 9, _val)  \
        ARM64_DBG_GET_CASE(reg, 10, _val) \
        ARM64_DBG_GET_CASE(reg, 11, _val) \
        ARM64_DBG_GET_CASE(reg, 12, _val) \
        ARM64_DBG_GET_CASE(reg, 13, _val) \
        ARM64_DBG_GET_CASE(reg, 14, _val) \
        ARM64_DBG_GET_CASE(reg, 15, _val) \
      }                                   \
    _val;                                 \
  })

#endif  /* __ARCH_ARM64_SRC_COMMON_ARM64_HWDEBUG_H */