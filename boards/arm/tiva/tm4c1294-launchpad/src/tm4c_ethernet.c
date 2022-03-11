/****************************************************************************
 * boards/arm/tiva/tm4c1294-launchpad/src/tm4c_ethernet.c
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

#include <inttypes.h>
#include <stdint.h>
#include <debug.h>
#include <assert.h>

#include <arch/board/board.h>
#include <net/ethernet.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/tiva_flash.h"
#include "tiva_ethernet.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_ethernetmac
 *
 * Description:
 *   For the Ethernet Eval Kits, the MAC address will be stored in the
 *   non-volatile USER0 and USER1 registers.
 *  If CONFIG_TIVA_BOARDMAC is defined, this function
 *   will obtain the MAC address from these registers.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_BOARDMAC
void tiva_ethernetmac(struct ether_addr *ethaddr)
{
  uint32_t user0;
  uint32_t user1;

  /* Get the current value of the user registers */

  user0 = getreg32(TIVA_FLASH_USERREG0);
  user1 = getreg32(TIVA_FLASH_USERREG1);

  ninfo("user: %06" PRIx32 ":%06" PRIx32 "\n",
        user1 & 0x00ffffff, user0 & 0x00ffffff);
  DEBUGASSERT(user0 != 0xffffffff && user1 != 0xffffffff);

  /* Re-format that MAC address the way that the network expects to see it */

  ethaddr->ether_addr_octet[0] = ((user0 >>  0) & 0xff);
  ethaddr->ether_addr_octet[1] = ((user0 >>  8) & 0xff);
  ethaddr->ether_addr_octet[2] = ((user0 >> 16) & 0xff);
  ethaddr->ether_addr_octet[3] = ((user1 >>  0) & 0xff);
  ethaddr->ether_addr_octet[4] = ((user1 >>  8) & 0xff);
  ethaddr->ether_addr_octet[5] = ((user1 >> 16) & 0xff);
}
#endif
