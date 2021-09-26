/****************************************************************************
 * boards/arm/samd5e5/same54-xplained-pro/src/sam_phyinit.c
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

#include "sam_port.h"
#include "sam_ethernet.h"

#include "same54-xplained-pro.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if defined(CONFIG_ETH0_PHY_KSZ8081) && defined(CONFIG_SAMD5E5_GMAC_PHYINIT)
int sam_phy_boardinitialize(int intf)
{
  /* Configure the KSZ8081 PHY reset pin and take it out of reset */

  sam_portconfig(PORT_PHY_RESET);
  return 0;
}
#endif
