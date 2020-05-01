/****************************************************************************
 * boards/arm/tiva/eagle100/src/lm_ethernet.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>
#include <assert.h>

#include <arch/board/board.h>
#include <net/ethernet.h>

#include "arm_arch.h"
#include "chip.h"
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
 *   If CONFIG_TIVA_BOARDMAC is defined, this function will obtain the
 *   MAC address from these registers.
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

  ninfo("user: %06x:%06x\n", user1 & 0x00ffffff, user0 & 0x00ffffff);
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
