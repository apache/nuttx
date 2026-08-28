/****************************************************************************
 * arch/tricore/src/common/tricore_mpu.c
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

#include <stdio.h>
#include <nuttx/debug.h>

#include <arch/barriers.h>

#include "tricore_mpu.h"
#include "tricore_internal.h"

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static unsigned int g_mpu_data_region;
static unsigned int g_mpu_code_region;
static unsigned int g_mpu_set;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t get_dpre_value(unsigned int set)
{
  switch (set)
    {
#if CONFIG_ARCH_MPU_NSETS > 0
      case 0:
        return tricore_mfcr(CPU_DPRE_0);
#endif
#if CONFIG_ARCH_MPU_NSETS > 1
      case 1:
        return tricore_mfcr(CPU_DPRE_1);
#endif
#if CONFIG_ARCH_MPU_NSETS > 2
      case 2:
        return tricore_mfcr(CPU_DPRE_2);
#endif
#if CONFIG_ARCH_MPU_NSETS > 3
      case 3:
        return tricore_mfcr(CPU_DPRE_3);
#endif
#if CONFIG_ARCH_MPU_NSETS > 4
      case 4:
        return tricore_mfcr(CPU_DPRE_4);
#endif
#if CONFIG_ARCH_MPU_NSETS > 5
      case 5:
        return tricore_mfcr(CPU_DPRE_5);
#endif
#if CONFIG_ARCH_MPU_NSETS > 6
      case 6:
        return tricore_mfcr(CPU_DPRE_6);
#endif
#if CONFIG_ARCH_MPU_NSETS > 7
      case 7:
        return tricore_mfcr(CPU_DPRE_7);
#endif
      default:
        return 0;
    }
}

static void set_dpre_value(unsigned int set, uint32_t dpre_value)
{
  switch (set)
    {
#if CONFIG_ARCH_MPU_NSETS > 0
      case 0:
        tricore_mtcr(CPU_DPRE_0, dpre_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 1
      case 1:
        tricore_mtcr(CPU_DPRE_1, dpre_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 2
      case 2:
        tricore_mtcr(CPU_DPRE_2, dpre_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 3
      case 3:
        tricore_mtcr(CPU_DPRE_3, dpre_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 4
      case 4:
        tricore_mtcr(CPU_DPRE_4, dpre_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 5
      case 5:
        tricore_mtcr(CPU_DPRE_5, dpre_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 6
      case 6:
        tricore_mtcr(CPU_DPRE_6, dpre_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 7
      case 7:
        tricore_mtcr(CPU_DPRE_7, dpre_value);
        break;
#endif
      default:
        break;
    }

  UP_ISB();
}

static uint32_t get_dpwe_value(unsigned int set)
{
  switch (set)
    {
#if CONFIG_ARCH_MPU_NSETS > 0
      case 0:
        return tricore_mfcr(CPU_DPWE_0);
#endif
#if CONFIG_ARCH_MPU_NSETS > 1
      case 1:
        return tricore_mfcr(CPU_DPWE_1);
#endif
#if CONFIG_ARCH_MPU_NSETS > 2
      case 2:
        return tricore_mfcr(CPU_DPWE_2);
#endif
#if CONFIG_ARCH_MPU_NSETS > 3
      case 3:
        return tricore_mfcr(CPU_DPWE_3);
#endif
#if CONFIG_ARCH_MPU_NSETS > 4
      case 4:
        return tricore_mfcr(CPU_DPWE_4);
#endif
#if CONFIG_ARCH_MPU_NSETS > 5
      case 5:
        return tricore_mfcr(CPU_DPWE_5);
#endif
#if CONFIG_ARCH_MPU_NSETS > 6
      case 6:
        return tricore_mfcr(CPU_DPWE_6);
#endif
#if CONFIG_ARCH_MPU_NSETS > 7
      case 7:
        return tricore_mfcr(CPU_DPWE_7);
#endif
      default:
        return 0;
    }
}

static void set_dpwe_value(unsigned int set, uint32_t dpwe_value)
{
  switch (set)
    {
#if CONFIG_ARCH_MPU_NSETS > 0
      case 0:
        tricore_mtcr(CPU_DPWE_0, dpwe_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 1
      case 1:
        tricore_mtcr(CPU_DPWE_1, dpwe_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 2
      case 2:
        tricore_mtcr(CPU_DPWE_2, dpwe_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 3
      case 3:
        tricore_mtcr(CPU_DPWE_3, dpwe_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 4
      case 4:
        tricore_mtcr(CPU_DPWE_4, dpwe_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 5
      case 5:
        tricore_mtcr(CPU_DPWE_5, dpwe_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 6
      case 6:
        tricore_mtcr(CPU_DPWE_6, dpwe_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 7
      case 7:
        tricore_mtcr(CPU_DPWE_7, dpwe_value);
        break;
#endif
      default:
        break;
    }

  UP_ISB();
}

static uint32_t get_cpxe_value(unsigned int set)
{
  switch (set)
    {
#if CONFIG_ARCH_MPU_NSETS > 0
      case 0:
        return tricore_mfcr(CPU_CPXE_0);
#endif
#if CONFIG_ARCH_MPU_NSETS > 1
      case 1:
        return tricore_mfcr(CPU_CPXE_1);
#endif
#if CONFIG_ARCH_MPU_NSETS > 2
      case 2:
        return tricore_mfcr(CPU_CPXE_2);
#endif
#if CONFIG_ARCH_MPU_NSETS > 3
      case 3:
        return tricore_mfcr(CPU_CPXE_3);
#endif
#if CONFIG_ARCH_MPU_NSETS > 4
      case 4:
        return tricore_mfcr(CPU_CPXE_4);
#endif
#if CONFIG_ARCH_MPU_NSETS > 5
      case 5:
        return tricore_mfcr(CPU_CPXE_5);
#endif
#if CONFIG_ARCH_MPU_NSETS > 6
      case 6:
        return tricore_mfcr(CPU_CPXE_6);
#endif
#if CONFIG_ARCH_MPU_NSETS > 7
      case 7:
        return tricore_mfcr(CPU_CPXE_7);
#endif
      default:
        return 0;
    }
}

static void set_cpxe_value(unsigned int set, uint32_t cpxe_value)
{
  switch (set)
    {
#if CONFIG_ARCH_MPU_NSETS > 0
      case 0:
        tricore_mtcr(CPU_CPXE_0, cpxe_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 1
      case 1:
        tricore_mtcr(CPU_CPXE_1, cpxe_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 2
      case 2:
        tricore_mtcr(CPU_CPXE_2, cpxe_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 3
      case 3:
        tricore_mtcr(CPU_CPXE_3, cpxe_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 4
      case 4:
        tricore_mtcr(CPU_CPXE_4, cpxe_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 5
      case 5:
        tricore_mtcr(CPU_CPXE_5, cpxe_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 6
      case 6:
        tricore_mtcr(CPU_CPXE_6, cpxe_value);
        break;
#endif
#if CONFIG_ARCH_MPU_NSETS > 7
      case 7:
        tricore_mtcr(CPU_CPXE_7, cpxe_value);
        break;
#endif
      default:
        break;
    }

  UP_ISB();
}

static int get_dpr_addrass_value(unsigned int region,
                                 uint32_t *dpr_l,
                                 uint32_t *dpr_u)
{
  switch (region)
    {
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 0
      case 0:
        *dpr_l = tricore_mfcr(CPU_DPR0_L);
        *dpr_u = tricore_mfcr(CPU_DPR0_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 1
      case 1:
        *dpr_l = tricore_mfcr(CPU_DPR1_L);
        *dpr_u = tricore_mfcr(CPU_DPR1_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 2
      case 2:
        *dpr_l = tricore_mfcr(CPU_DPR2_L);
        *dpr_u = tricore_mfcr(CPU_DPR2_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 3
      case 3:
        *dpr_l = tricore_mfcr(CPU_DPR3_L);
        *dpr_u = tricore_mfcr(CPU_DPR3_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 4
      case 4:
        *dpr_l = tricore_mfcr(CPU_DPR4_L);
        *dpr_u = tricore_mfcr(CPU_DPR4_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 5
      case 5:
        *dpr_l = tricore_mfcr(CPU_DPR5_L);
        *dpr_u = tricore_mfcr(CPU_DPR5_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 6
      case 6:
        *dpr_l = tricore_mfcr(CPU_DPR6_L);
        *dpr_u = tricore_mfcr(CPU_DPR6_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 7
      case 7:
        *dpr_l = tricore_mfcr(CPU_DPR7_L);
        *dpr_u = tricore_mfcr(CPU_DPR7_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 8
      case 8:
        *dpr_l = tricore_mfcr(CPU_DPR8_L);
        *dpr_u = tricore_mfcr(CPU_DPR8_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 9
      case 9:
        *dpr_l = tricore_mfcr(CPU_DPR9_L);
        *dpr_u = tricore_mfcr(CPU_DPR9_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 10
      case 10:
        *dpr_l = tricore_mfcr(CPU_DPR10_L);
        *dpr_u = tricore_mfcr(CPU_DPR10_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 11
      case 11:
        *dpr_l = tricore_mfcr(CPU_DPR11_L);
        *dpr_u = tricore_mfcr(CPU_DPR11_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 12
      case 12:
        *dpr_l = tricore_mfcr(CPU_DPR12_L);
        *dpr_u = tricore_mfcr(CPU_DPR12_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 13
      case 13:
        *dpr_l = tricore_mfcr(CPU_DPR13_L);
        *dpr_u = tricore_mfcr(CPU_DPR13_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 14
      case 14:
        *dpr_l = tricore_mfcr(CPU_DPR14_L);
        *dpr_u = tricore_mfcr(CPU_DPR14_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 15
      case 15:
        *dpr_l = tricore_mfcr(CPU_DPR15_L);
        *dpr_u = tricore_mfcr(CPU_DPR15_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 16
      case 16:
        *dpr_l = tricore_mfcr(CPU_DPR16_L);
        *dpr_u = tricore_mfcr(CPU_DPR16_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 17
      case 17:
        *dpr_l = tricore_mfcr(CPU_DPR17_L);
        *dpr_u = tricore_mfcr(CPU_DPR17_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 18
      case 18:
        *dpr_l = tricore_mfcr(CPU_DPR18_L);
        *dpr_u = tricore_mfcr(CPU_DPR18_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 19
      case 19:
        *dpr_l = tricore_mfcr(CPU_DPR19_L);
        *dpr_u = tricore_mfcr(CPU_DPR19_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 20
      case 20:
        *dpr_l = tricore_mfcr(CPU_DPR20_L);
        *dpr_u = tricore_mfcr(CPU_DPR20_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 21
      case 21:
        *dpr_l = tricore_mfcr(CPU_DPR21_L);
        *dpr_u = tricore_mfcr(CPU_DPR21_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 22
      case 22:
        *dpr_l = tricore_mfcr(CPU_DPR22_L);
        *dpr_u = tricore_mfcr(CPU_DPR22_U);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 23
      case 23:
        *dpr_l = tricore_mfcr(CPU_DPR23_L);
        *dpr_u = tricore_mfcr(CPU_DPR23_U);
        break;
#endif
      default:
        return -EINVAL;
    }

  return 0;
}

static void set_dpr_address_value(unsigned int region,
                                  uint32_t dpr_l,
                                  uint32_t dpr_u)
{
  switch (region)
    {
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 0
      case 0:
        tricore_mtcr(CPU_DPR0_L, dpr_l);
        tricore_mtcr(CPU_DPR0_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 1
      case 1:
        tricore_mtcr(CPU_DPR1_L, dpr_l);
        tricore_mtcr(CPU_DPR1_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 2
      case 2:
        tricore_mtcr(CPU_DPR2_L, dpr_l);
        tricore_mtcr(CPU_DPR2_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 3
      case 3:
        tricore_mtcr(CPU_DPR3_L, dpr_l);
        tricore_mtcr(CPU_DPR3_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 4
      case 4:
        tricore_mtcr(CPU_DPR4_L, dpr_l);
        tricore_mtcr(CPU_DPR4_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 5
      case 5:
        tricore_mtcr(CPU_DPR5_L, dpr_l);
        tricore_mtcr(CPU_DPR5_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 6
      case 6:
        tricore_mtcr(CPU_DPR6_L, dpr_l);
        tricore_mtcr(CPU_DPR6_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 7
      case 7:
        tricore_mtcr(CPU_DPR7_L, dpr_l);
        tricore_mtcr(CPU_DPR7_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 8
      case 8:
        tricore_mtcr(CPU_DPR8_L, dpr_l);
        tricore_mtcr(CPU_DPR8_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 9
      case 9:
        tricore_mtcr(CPU_DPR9_L, dpr_l);
        tricore_mtcr(CPU_DPR9_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 10
      case 10:
        tricore_mtcr(CPU_DPR10_L, dpr_l);
        tricore_mtcr(CPU_DPR10_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 11
      case 11:
        tricore_mtcr(CPU_DPR11_L, dpr_l);
        tricore_mtcr(CPU_DPR11_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 12
      case 12:
        tricore_mtcr(CPU_DPR12_L, dpr_l);
        tricore_mtcr(CPU_DPR12_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 13
      case 13:
        tricore_mtcr(CPU_DPR13_L, dpr_l);
        tricore_mtcr(CPU_DPR13_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 14
      case 14:
        tricore_mtcr(CPU_DPR14_L, dpr_l);
        tricore_mtcr(CPU_DPR14_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 15
      case 15:
        tricore_mtcr(CPU_DPR15_L, dpr_l);
        tricore_mtcr(CPU_DPR15_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 16
      case 16:
        tricore_mtcr(CPU_DPR16_L, dpr_l);
        tricore_mtcr(CPU_DPR16_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 17
      case 17:
        tricore_mtcr(CPU_DPR17_L, dpr_l);
        tricore_mtcr(CPU_DPR17_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 18
      case 18:
        tricore_mtcr(CPU_DPR18_L, dpr_l);
        tricore_mtcr(CPU_DPR18_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 19
      case 19:
        tricore_mtcr(CPU_DPR19_L, dpr_l);
        tricore_mtcr(CPU_DPR19_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 20
      case 20:
        tricore_mtcr(CPU_DPR20_L, dpr_l);
        tricore_mtcr(CPU_DPR20_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 21
      case 21:
        tricore_mtcr(CPU_DPR21_L, dpr_l);
        tricore_mtcr(CPU_DPR21_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 22
      case 22:
        tricore_mtcr(CPU_DPR22_L, dpr_l);
        tricore_mtcr(CPU_DPR22_U, dpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_DATA_NREGIONS > 23
      case 23:
        tricore_mtcr(CPU_DPR23_L, dpr_l);
        tricore_mtcr(CPU_DPR23_U, dpr_u);
        break;
#endif
      default:
        break;
    }

  UP_ISB();
}

static int get_cpr_address_region(unsigned int region,
                                  uint32_t *cpr_l,
                                  uint32_t *cpr_u)
{
  switch (region)
    {
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 0
      case 0:
        *cpr_l = tricore_mfcr(CPU_CPR0_L);
        *cpr_u = tricore_mfcr(CPU_CPR0_U);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 1
      case 1:
        *cpr_l = tricore_mfcr(CPU_CPR1_L);
        *cpr_u = tricore_mfcr(CPU_CPR1_U);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 2
      case 2:
        *cpr_l = tricore_mfcr(CPU_CPR2_L);
        *cpr_u = tricore_mfcr(CPU_CPR2_U);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 3
      case 3:
        *cpr_l = tricore_mfcr(CPU_CPR3_L);
        *cpr_u = tricore_mfcr(CPU_CPR3_U);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 4
      case 4:
        *cpr_l = tricore_mfcr(CPU_CPR4_L);
        *cpr_u = tricore_mfcr(CPU_CPR4_U);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 5
      case 5:
        *cpr_l = tricore_mfcr(CPU_CPR5_L);
        *cpr_u = tricore_mfcr(CPU_CPR5_U);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 6
      case 6:
        *cpr_l = tricore_mfcr(CPU_CPR6_L);
        *cpr_u = tricore_mfcr(CPU_CPR6_U);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 7
      case 7:
        *cpr_l = tricore_mfcr(CPU_CPR7_L);
        *cpr_u = tricore_mfcr(CPU_CPR7_U);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 8
      case 8:
        *cpr_l = tricore_mfcr(CPU_CPR8_L);
        *cpr_u = tricore_mfcr(CPU_CPR8_U);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 9
      case 9:
        *cpr_l = tricore_mfcr(CPU_CPR9_L);
        *cpr_u = tricore_mfcr(CPU_CPR9_U);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 10
      case 10:
        *cpr_l = tricore_mfcr(CPU_CPR10_L);
        *cpr_u = tricore_mfcr(CPU_CPR10_U);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 11
      case 11:
        *cpr_l = tricore_mfcr(CPU_CPR11_L);
        *cpr_u = tricore_mfcr(CPU_CPR11_U);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 12
      case 12:
        *cpr_l = tricore_mfcr(CPU_CPR12_L);
        *cpr_u = tricore_mfcr(CPU_CPR12_U);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 13
      case 13:
        *cpr_l = tricore_mfcr(CPU_CPR13_L);
        *cpr_u = tricore_mfcr(CPU_CPR13_U);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 14
      case 14:
        *cpr_l = tricore_mfcr(CPU_CPR14_L);
        *cpr_u = tricore_mfcr(CPU_CPR14_U);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 15
      case 15:
        *cpr_l = tricore_mfcr(CPU_CPR15_L);
        *cpr_u = tricore_mfcr(CPU_CPR15_U);
        break;
#endif
      default:
        return -EINVAL;
    }

  return 0;
}

static void set_cpr_address_value(unsigned int region,
                                  uint32_t cpr_l,
                                  uint32_t cpr_u)
{
  switch (region)
    {
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 0
      case 0:
        tricore_mtcr(CPU_CPR0_L, cpr_l);
        tricore_mtcr(CPU_CPR0_U, cpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 1
      case 1:
        tricore_mtcr(CPU_CPR1_L, cpr_l);
        tricore_mtcr(CPU_CPR1_U, cpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 2
      case 2:
        tricore_mtcr(CPU_CPR2_L, cpr_l);
        tricore_mtcr(CPU_CPR2_U, cpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 3
      case 3:
        tricore_mtcr(CPU_CPR3_L, cpr_l);
        tricore_mtcr(CPU_CPR3_U, cpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 4
      case 4:
        tricore_mtcr(CPU_CPR4_L, cpr_l);
        tricore_mtcr(CPU_CPR4_U, cpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 5
      case 5:
        tricore_mtcr(CPU_CPR5_L, cpr_l);
        tricore_mtcr(CPU_CPR5_U, cpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 6
      case 6:
        tricore_mtcr(CPU_CPR6_L, cpr_l);
        tricore_mtcr(CPU_CPR6_U, cpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 7
      case 7:
        tricore_mtcr(CPU_CPR7_L, cpr_l);
        tricore_mtcr(CPU_CPR7_U, cpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 8
      case 8:
        tricore_mtcr(CPU_CPR8_L, cpr_l);
        tricore_mtcr(CPU_CPR8_U, cpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 9
      case 9:
        tricore_mtcr(CPU_CPR9_L, cpr_l);
        tricore_mtcr(CPU_CPR9_U, cpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 10
      case 10:
        tricore_mtcr(CPU_CPR10_L, cpr_l);
        tricore_mtcr(CPU_CPR10_U, cpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 11
      case 11:
        tricore_mtcr(CPU_CPR11_L, cpr_l);
        tricore_mtcr(CPU_CPR11_U, cpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 12
      case 12:
        tricore_mtcr(CPU_CPR12_L, cpr_l);
        tricore_mtcr(CPU_CPR12_U, cpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 13
      case 13:
        tricore_mtcr(CPU_CPR13_L, cpr_l);
        tricore_mtcr(CPU_CPR13_U, cpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 14
      case 14:
        tricore_mtcr(CPU_CPR14_L, cpr_l);
        tricore_mtcr(CPU_CPR14_U, cpr_u);
        break;
#endif
#if CONFIG_ARCH_MPU_CODE_NREGIONS > 15
      case 15:
        tricore_mtcr(CPU_CPR15_L, cpr_l);
        tricore_mtcr(CPU_CPR15_U, cpr_u);
        break;
#endif
    }

  UP_ISB();
}

static void mpu_modify_data_region(unsigned int region, uintptr_t base,
                                   size_t size)
{
  /* Check valid */

  DEBUGASSERT(region < CONFIG_ARCH_MPU_DATA_NREGIONS);

  /* Set the lower bound and upper bound of CPU Data Protection Region */

  set_dpr_address_value(region, base, base + size);
}

static void mpu_modify_code_region(unsigned int region, uintptr_t base,
                                   size_t size)
{
  /* Check valid */

  DEBUGASSERT(region < CONFIG_ARCH_MPU_CODE_NREGIONS);

  /* Set the lower bound and upper bound of CPU Code Protection Region */

  set_cpr_address_value(region, base, base + size);
}

static void mpu_modify_data_set(unsigned int set, unsigned int region,
                                int flags)
{
  /* Check valid */

  DEBUGASSERT(set < CONFIG_ARCH_MPU_NSETS);
  DEBUGASSERT(region < CONFIG_ARCH_MPU_DATA_NREGIONS);

  uint32_t dpre_value = get_dpre_value(set);
  uint32_t dpwe_value = get_dpwe_value(set);

  /* Set the bit corresponding to the given Data Protection Region */

  dpre_value = flags & REGION_ATTR_RE ?
               dpre_value | (0x01 << region) :
               dpre_value & ~(0x01 << region);
  dpwe_value = flags & REGION_ATTR_WE ?
               dpwe_value | (0x01 << region) :
               dpwe_value & ~(0x01 << region);

  set_dpre_value(set, dpre_value);
  set_dpwe_value(set, dpwe_value);
}

static void mpu_modify_code_set(unsigned int set, unsigned int region,
                                int flags)
{
  /* Check valid */

  DEBUGASSERT(set < CONFIG_ARCH_MPU_NSETS);
  DEBUGASSERT(region < CONFIG_ARCH_MPU_CODE_NREGIONS);

  /* Get the CPU CPXE register value of the input protection set */

  uint32_t cpxe_value = get_cpxe_value(set);

  /* Set the bit corresponding to the given code protection region */

  cpxe_value = flags & REGION_ATTR_XE ?
               cpxe_value | (0x1 << region) :
               cpxe_value & ~(0x1 << region);

  /* Set the CPU CPXE value to enable code execution */

  set_cpxe_value(set, cpxe_value);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu_allocdataregions
 *
 * Description:
 *   Allocate data regions
 *
 * Input Parameters:
 *   nregions - Number of regions to allocate.
 *
 * Returned Value:
 *   The index of the allocated region.
 *
 ****************************************************************************/

unsigned int mpu_allocdataregions(unsigned int nregions)
{
  unsigned int i = ffs(~g_mpu_data_region) - 1;

  /* There are not enough regions to apply */

  DEBUGASSERT((i + nregions - 1) < CONFIG_ARCH_MPU_DATA_NREGIONS);

  g_mpu_data_region |= ((1 << nregions) - 1) << i;
  return i;
}

/****************************************************************************
 * Name: mpu_alloccoderegions
 *
 * Description:
 *   Allocate code regions
 *
 * Input Parameters:
 *   nregions - Number of regions to allocate.
 *
 * Returned Value:
 *   The index of the allocated region.
 *
 ****************************************************************************/

unsigned int mpu_alloccoderegions(unsigned int nregions)
{
  unsigned int i = ffs(~g_mpu_code_region) - 1;

  /* There are not enough regions to apply */

  DEBUGASSERT((i + nregions - 1) < CONFIG_ARCH_MPU_CODE_NREGIONS);

  g_mpu_code_region |= ((1 << nregions) - 1) << i;
  return i;
}

/****************************************************************************
 * Name: mpu_freedataregion
 *
 * Description:
 *   Free data region
 *
 * Input Parameters:
 *   region - Region to free.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpu_freedataregion(unsigned int region)
{
  DEBUGASSERT(region < CONFIG_ARCH_MPU_DATA_NREGIONS);

  mpu_modify_data_region(region, 0, 0);

  g_mpu_data_region &= ~(1 << region);
}

/****************************************************************************
 * Name: mpu_freecoderegion
 *
 * Description:
 *   Free code region
 *
 * Input Parameters:
 *   region - Region to free.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpu_freecoderegion(unsigned int region)
{
  DEBUGASSERT(region < CONFIG_ARCH_MPU_CODE_NREGIONS);

  mpu_modify_code_region(region, 0, 0);

  g_mpu_code_region &= ~(1 << region);
}

/****************************************************************************
 * Name: mpu_allocregions
 *
 * Description:
 *   Allocate data or code regions
 *
 * Input Parameters:
 *   nregions - Number of regions to allocate.
 *   flags    - Region flags.
 *
 * Returned Value:
 *   The index of the allocated region.
 *
 ****************************************************************************/

unsigned int mpu_allocregions(unsigned int nregions, int flags)
{
  if ((flags & REGION_TYPE_MASK) == REGION_TYPE_CODE)
    {
      return mpu_alloccoderegions(nregions);
    }
  else
    {
      return mpu_allocdataregions(nregions);
    }
}

/****************************************************************************
 * Name: mpu_freeregion
 *
 * Description:
 *   Free data region
 *
 * Input Parameters:
 *   flags  - Region flags.
 *   region - Region to free.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpu_freeregion(unsigned int region, int flags)
{
  if ((flags & REGION_TYPE_MASK) == REGION_TYPE_CODE)
    {
      mpu_freecoderegion(region);
    }
  else
    {
      mpu_freedataregion(region);
    }
}

/****************************************************************************
 * Name: mpu_alloc_set
 *
 * Description:
 *   Allocate a protection set
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The index of the allocated set.
 *
 ****************************************************************************/

unsigned int mpu_alloc_set(void)
{
  unsigned int i = ffs(~g_mpu_set) - 1;

  /* There are not enough set to apply */

  DEBUGASSERT(i < CONFIG_ARCH_MPU_NSETS);

  g_mpu_set |= 1 << i;

  return i;
}

/****************************************************************************
 * Name: mpu_free_set
 *
 * Description:
 *   Free a protection set
 *
 * Input Parameters:
 *   set - Set to free.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void mpu_free_set(unsigned int set)
{
  DEBUGASSERT(set < CONFIG_ARCH_MPU_NSETS);
  g_mpu_set &= ~(1 << set);
}

/****************************************************************************
 * Name: mpu_control
 *
 * Description:
 *   Configure and enable (or disable) the MPU
 *
 * Input Parameters:
 *   enable - Flag indicating whether to enable the MPU.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void mpu_control(bool enable)
{
  uint32_t corecon_value = tricore_mfcr(CPU_CORECON);

  corecon_value = enable ?
                  corecon_value | (1 << 1) :
                  corecon_value & ~(1 << 1);
  tricore_mtcr(CPU_CORECON, corecon_value);

  UP_ISB();
}

/****************************************************************************
 * Name: mpu_dump_set
 *
 * Description:
 *   Dump the regions of a protection set.
 *
 * Input Parameters:
 *   set - protection set.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void mpu_dump_set(unsigned int set)
{
  /* Check valid */

  DEBUGASSERT(set < CONFIG_ARCH_MPU_NSETS);

  uint32_t dpre_value = get_dpre_value(set);
  uint32_t dpwe_value = get_dpwe_value(set);
  uint32_t cpxe_value = get_cpxe_value(set);
  uint32_t dprl_value;
  uint32_t dpru_value;
  uint32_t cprl_value;
  uint32_t cpru_value;
  unsigned int i;
  int ret;

  _info("data regions:\n");
  for (i = 0; i < CONFIG_ARCH_MPU_DATA_NREGIONS; i++)
    {
      ret = get_dpr_addrass_value(i, &dprl_value, &dpru_value);
      if (ret == 0)
        {
          _info("region: %02d, lower address: 0x%08X, \
                upper address: 0x%08X, \
                read enable:%s, write enable:%s\n",
                i, dprl_value, dpru_value,
                dpre_value & (0x01 << i) ? "yes" : "no",
                dpwe_value & (0x01 << i) ? "yes" : "no");
        }
      else
        {
          _info("region: %02d, get region info fail\n", i);
        }
    }

  _info("code regions :\n");
  for (i = 0; i < CONFIG_ARCH_MPU_CODE_NREGIONS; i++)
    {
      ret = get_cpr_address_region(i, &cprl_value, &cpru_value);
      if (ret == 0)
        {
          _info("region: %02d, lower address: 0x%08X, \
                upper address: 0x%08X, \
                exec enable:%s\n",
                i, cprl_value, cpru_value,
                cpxe_value & (0x01 << i) ? "yes" : "no");
        }
      else
        {
          _info("region: %02d, get region info fail\n", i);
        }
    }
}

/****************************************************************************
 * Name: mpu_dump_regions
 *
 * Description:
 *   Dump the regions of all sets.
 *
 * Input Parameters:
 *   set - protection set.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void mpu_dump_regions(void)
{
  unsigned int i;

  _info("active set is: %02d\n", mpu_get_active_set());
  _info("dump all sets info:\n");

  for (i = 0; i < CONFIG_ARCH_MPU_NSETS; i++)
    {
      if (g_mpu_set & (1 << i))
        {
          _info("set %02d:\n", i);
          mpu_dump_set(i);
        }
    }
}

/****************************************************************************
 * Name: mpu_modify_region
 *
 * Description:
 *   Modify a region's attributes in the special protection set.
 *
 * Input Parameters:
 *   set        - Set number to modify.
 *   region     - Region number to modify.
 *   base       - Base address of the region.
 *   size       - Size of the region.
 *   flags      - Flags to configure the region.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void mpu_modify_region(unsigned int set, unsigned int region,
                       uintptr_t base, size_t size, int flags)
{
  if ((flags & REGION_TYPE_MASK) == REGION_TYPE_CODE)
    {
      if (size != 0)
        {
          mpu_modify_code_region(region, base, base + size);
        }

      mpu_modify_code_set(set, region, flags);
    }
  else
    {
      if (size != 0)
        {
          mpu_modify_data_region(region, base, base + size);
        }

      mpu_modify_data_set(set, region, flags);
    }
}

/****************************************************************************
 * Name: mpu_configure_region
 *
 * Description:
 *   Configure a region's attributes in the special protection set.
 *
 * Input Parameters:
 *   set        - Set number to modify.
 *   base       - Base address of the region.
 *   size       - Size of the region.
 *   flags      - Flags to configure the region.
 *
 * Returned Value:
 *   The index of the allocated region.
 *
 ****************************************************************************/

unsigned int mpu_configure_region(unsigned int set, uintptr_t base,
                                  size_t size, int flags)
{
  unsigned int region;

  if ((flags & REGION_TYPE_MASK) == REGION_TYPE_CODE)
    {
      region = mpu_alloccoderegion();
      mpu_modify_code_region(region, base, size);
      mpu_modify_code_set(set, region, flags);
    }
  else
    {
      region = mpu_allocdataregion();
      mpu_modify_data_region(region, base, size);
      mpu_modify_data_set(set, region, flags);
    }

  return region;
}

/****************************************************************************
 * Name: mpu_get_active_set
 *
 * Description:
 *   Get current protection set
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Current protection set.
 *
 ****************************************************************************/

unsigned int mpu_get_active_set(void)
{
  uint32_t psw_value = tricore_mfcr(CPU_PSW);

  return ((psw_value & (0x1 << 15)) >> 15) << 2 |
         ((psw_value & (0x3 << 12)) >> 12);
}

/****************************************************************************
 * Name: mpu_set_active_set
 *
 * Description:
 *   Set active protection set
 *
 * Input Parameters:
 *   Protection set .
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void mpu_set_active_set(unsigned int set)
{
  /* Check that the set valid */

  DEBUGASSERT(set < CONFIG_ARCH_MPU_NSETS);

  uint32_t psw_value = tricore_mfcr(CPU_PSW);

  psw_value &= ~((0x3 << 12) | (0x1 << 15));
  psw_value |= (set >> 2) << 15;
  psw_value |= (set & 0x03) << 12;
  tricore_mtcr(CPU_PSW, psw_value);

  UP_ISB();
}

/****************************************************************************
 * Name: mpu_initialize
 *
 * Description:
 *   Initialize the MPU regions.
 *
 * Input Parameters:
 *   table      - MPU initialization table.
 *   count      - Initialize the number of entries in the region table.
 *
 * Returned Value:
 *   NULL.
 *
 ****************************************************************************/

void mpu_initialize(const struct mpu_region_s *table, size_t count)
{
  const struct mpu_region_s *conf;
  unsigned int cpu = this_cpu();
  unsigned int region;
  int set;
  int index;

  mpu_control(false);
  for (index = 0; index < count; index++)
    {
      conf = &table[index];

      if (conf->cpuset & (1 << cpu))
        {
          /* Allocate new region based on type */

          if ((conf->flags[0] & REGION_TYPE_MASK) == REGION_TYPE_CODE)
            {
              region = mpu_alloccoderegion();
              mpu_modify_code_region(region, conf->base, conf->size);
            }
          else if ((conf->flags[0] & REGION_TYPE_MASK) == REGION_TYPE_DATA)
            {
              region = mpu_allocdataregion();
              mpu_modify_data_region(region, conf->base, conf->size);
            }
          else
            {
              continue;
            }

          /* Set permissions for each protection set */

          for (set = 0; set < CONFIG_ARCH_MPU_NSETS; set++)
            {
              mpu_modify_region(set, region, 0, 0, conf->flags[set]);
            }
        }
    }

  mpu_control(true);
}
