/****************************************************************************
 * arch/arm/src/samv7/sam_ethernet.c
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

#include <debug.h>
#include "sam_ethernet.h"

#ifdef CONFIG_NET

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: arm_netinitialize
 *
 * Description:
 *   This is the "standard" network initialization logic called from the
 *   low-level initialization logic in arm_initialize.c.  This is just
 *   a shim to support the slightly different prototype of
 *   sam_emac_intiialize() and to provide support for future chips that
 *   may have multiple EMAC peripherals.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

void arm_netinitialize(void)
{
#ifdef CONFIG_SAMV7_EMAC0
  int ret;

#ifdef CONFIG_SAMV7_EMAC0
  /* Initialize the EMAC0 driver */

  ret = sam_emac_initialize(EMAC0_INTF);
  if (ret < 0)
    {
      nerr("ERROR: up_emac_initialize(EMAC0) failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SAMV7_EMAC1
  /* Initialize the EMAC1 driver */

  ret = sam_emac_initialize(EMAC1_INTF);
  if (ret < 0)
    {
      nerr("ERROR: up_emac_initialize(EMAC1) failed: %d\n", ret);
    }
#endif
#endif
}

#endif /* CONFIG_NET */
