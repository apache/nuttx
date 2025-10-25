/****************************************************************************
 * arch/arm/src/samv7/sam_chipid.c
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

#include "sam_chipid.h"
#include "hardware/sam_chipid.h"
#include "arm_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

bool sam_has_revb_periphs(void)
{
#ifdef CONFIG_ARCH_CHIP_PIC32CZCA70
  return true;
#else
  /* If not built for PIC32CZ, decide which chip is being used */

  uint32_t regval;
  uint32_t id_arch;
  uint32_t id_version;

  regval = getreg32(SAM_CHIPID_CIDR);
  id_arch = (regval & CHIPID_CIDR_ARCH_MASK) >> CHIPID_CIDR_ARCH_SHIFT;
  id_version = (regval & CHIPID_CIDR_VERSION_MASK)
                >> CHIPID_CIDR_VERSION_SHIFT;

  /* SAMV7s are identified by 0x10-0x13, therefore
   * anything else should be PIC32
   */

  if ((id_arch & ~3) != 0x10)
    {
      return true;
    }

  return id_version != 0;
#endif /* CONFIG_ARCH_CHIP_PIC32CZCA70 */
}
