/****************************************************************************
 * configs/misoc/include/generated/common.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Ramtin Amin <keytwo@gmail.com>
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

#ifndef __CONFIGS_MISOC_INCLUDE_GENERATED_SDRAM_PHY_H
#define __CONFIGS_MISOC_INCLUDE_GENERATED_SDRAM_PHY_H

/****************************************************************************
 * Included Filese
 ****************************************************************************/

#include "hw/common.h"
#include "hw/flags.h"

#include <arch/board/generated/csr.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DFII_NPHASES 1

#define sdram_dfii_pird_address_write(X) sdram_dfii_pi0_address_write(X)
#define sdram_dfii_piwr_address_write(X) sdram_dfii_pi0_address_write(X)

#define sdram_dfii_pird_baddress_write(X) sdram_dfii_pi0_baddress_write(X)
#define sdram_dfii_piwr_baddress_write(X) sdram_dfii_pi0_baddress_write(X)

#define command_prd(X) command_p0(X)
#define command_pwr(X) command_p0(X)

#define DFII_PIX_DATA_SIZE CSR_SDRAM_DFII_PI0_WRDATA_SIZE

/****************************************************************************
 * Private Data
 ****************************************************************************/

const unsigned int sdram_dfii_pix_wrdata_addr[1] =
{
  CSR_SDRAM_DFII_PI0_WRDATA_ADDR
};

const unsigned int sdram_dfii_pix_rddata_addr[1] =
{
  CSR_SDRAM_DFII_PI0_RDDATA_ADDR
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void cdelay(int i);

static void command_p0(int cmd)
{
  sdram_dfii_pi0_command_write(cmd);
  sdram_dfii_pi0_command_issue_write(1);
}

static void init_sequence(void)
{
  /* Bring CKE high */

  sdram_dfii_pi0_address_write(0x0);
  sdram_dfii_pi0_baddress_write(0);
  sdram_dfii_control_write(DFII_CONTROL_CKE | DFII_CONTROL_ODT |
                           DFII_CONTROL_RESET_N);
  cdelay(20000);

  /* Precharge All */

  sdram_dfii_pi0_address_write(0x400);
  sdram_dfii_pi0_baddress_write(0);
  command_p0(DFII_COMMAND_RAS | DFII_COMMAND_WE | DFII_COMMAND_CS);

  /* Load Mode Register / Reset DLL, CL=2, BL=1 */

  sdram_dfii_pi0_address_write(0x120);
  sdram_dfii_pi0_baddress_write(0);
  command_p0(DFII_COMMAND_RAS | DFII_COMMAND_CAS | DFII_COMMAND_WE |
             DFII_COMMAND_CS);
  cdelay(200);

  /* Precharge All */

  sdram_dfii_pi0_address_write(0x400);
  sdram_dfii_pi0_baddress_write(0);
  command_p0(DFII_COMMAND_RAS | DFII_COMMAND_WE | DFII_COMMAND_CS);

  /* Auto Refresh */

  sdram_dfii_pi0_address_write(0x0);
  sdram_dfii_pi0_baddress_write(0);
  command_p0(DFII_COMMAND_RAS | DFII_COMMAND_CAS | DFII_COMMAND_CS);
  cdelay(4);

  /* Auto Refresh */

  sdram_dfii_pi0_address_write(0x0);
  sdram_dfii_pi0_baddress_write(0);
  command_p0(DFII_COMMAND_RAS | DFII_COMMAND_CAS | DFII_COMMAND_CS);
  cdelay(4);

  /* Load Mode Register / CL=2, BL=1 */

  sdram_dfii_pi0_address_write(0x20);
  sdram_dfii_pi0_baddress_write(0);
  command_p0(DFII_COMMAND_RAS | DFII_COMMAND_CAS | DFII_COMMAND_WE |
             DFII_COMMAND_CS);
  cdelay(200);
}

#endif /* __CONFIGS_MISOC_INCLUDE_GENERATED_SDRAM_PHY_H
