/****************************************************************************
 * arch/arm64/src/a64/a64_rsb.c
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

/* Reference:
 *
 * "Rendering PinePhone's Display (DE and TCON0)"
 * https://lupyuen.github.io/articles/de
 *
 * "NuttX RTOS for PinePhone: Render Graphics in Zig"
 * https://lupyuen.github.io/articles/de2
 *
 * "A80 Page" refers to Allwinner A80 User Manual
 * https://lupyuen.github.io/images/A80_User_Manual_v1.3.1_20150513.pdf
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include "hardware/a64_memorymap.h"
#include "arm64_arch.h"
#include "a64_rsb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timeout for RSB Transactions in milliseconds */

#define TRANS_TIMEOUT_MS 5

/* A64 Reduced Serial Bus Registers and Bit Definitions *********************/

/* RSB Control Register (A80 Page 923) */

#define RSB_CTRL    (A64_RSB_ADDR + 0x00)
#define START_TRANS (1 << 7)

/* RSB Status Register (A80 Page 924) */

#define RSB_STAT    (A64_RSB_ADDR + 0x0c)
#define TRANS_OVER  (1 << 0)

/* RSB Address Register (A80 Page 926) */

#define RSB_AR      (A64_RSB_ADDR + 0x10)

/* RSB Data Buffer Register (A80 Page 926) */

#define RSB_DATA    (A64_RSB_ADDR + 0x1c)

/* RSB Command Register (A80 Page 928) */

#define RSB_CMD     (A64_RSB_ADDR + 0x2c)

/* RSB Device Address Register (A80 Page 928) */

#define RSB_DAR     (A64_RSB_ADDR + 0x30)
#define RTA(n)      ((n) << 16)

/* A64 Reduced Serial Bus Commands ******************************************/

/* Write a byte to Reduced Serial Bus (A80 Page 918) */

#define RSBCMD_WR8  0x4e

/* Read a byte from Reduced Serial Bus (A80 Page 918) */

#define RSBCMD_RD8  0x8b

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rsb_wait_trans
 *
 * Description:
 *   Wait for Reduced Serial Bus Transaction to complete or fail with error.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; ERROR if timeout.
 *
 ****************************************************************************/

static int rsb_wait_trans(void)
{
  int i;

  for (i = 0; i < TRANS_TIMEOUT_MS; i++)
    {
      /* Poll on START_TRANS (Bit 7) of RSB Control Register */

      if ((getreg32(RSB_CTRL) & START_TRANS) == 0)
        {
          /* If START_TRANS is 0, then transaction has completed or failed */

          return OK;
        }

      /* Sleep 1 millisecond and try again */

      up_mdelay(1);
    }

  baterr("RSB Transaction Timeout");
  return ERROR;
}

/****************************************************************************
 * Name: rsb_wait_status
 *
 * Description:
 *   Wait for Reduced Serial Bus Transaction to complete or fail with error.
 *   Return the Transaction Status.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; ERROR if status is error or timeout.
 *
 ****************************************************************************/

static int rsb_wait_status(void)
{
  int ret;

  /* Wait for transaction to complete or fail with error */

  ret = rsb_wait_trans();
  if (ret < 0)
    {
      return ret;
    }

  /* RSB Status Register (A80 Page 924)
   * If TRANS_OVER (Bit 0) is 1, RSB Transfer has completed without error
   */

  if (getreg32(RSB_STAT) == TRANS_OVER)
    {
      return OK;
    }

  baterr("RSB Transaction Failed\n");
  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: a64_rsb_read
 *
 * Description:
 *   Read a byte from a device on the Reduced Serial Bus.
 *
 * Input Parameters:
 *   rt_addr  - Run-Time Address of RSB Device
 *   reg_addr - Register Address of RSB Device
 *
 * Returned Value:
 *   Byte read from the RSB Device; ERROR if the read failed or timed out.
 *
 ****************************************************************************/

int a64_rsb_read(uint8_t rt_addr, uint8_t reg_addr)
{
  int ret;

  /* RSB Command Register (A80 Page 928)
   * Set to 0x8B (RD8) to read one byte
   */

  batinfo("rt_addr=0x%x, reg_addr=0x%x\n", rt_addr, reg_addr);
  putreg32(RSBCMD_RD8, RSB_CMD);

  /* RSB Device Address Register (A80 Page 928)
   * Set RTA (Bits 16 to 23) to the Run-Time Address of RSB Device
   */

  putreg32(RTA(rt_addr), RSB_DAR);

  /* RSB Address Register (A80 Page 926)
   * Set to the Register Address that will be read from RSB Device
   */

  putreg32(reg_addr, RSB_AR);

  /* RSB Control Register (A80 Page 923)
   * Set START_TRANS (Bit 7) to 1 (Start Transaction)
   */

  putreg32(START_TRANS, RSB_CTRL);

  /* Wait for RSB Transaction to complete and read the RSB Status */

  ret = rsb_wait_status();
  if (ret < 0)
    {
      return ret;
    }

  /* RSB Data Buffer Register (A80 Page 926)
   * Contains the Register Value read from RSB Device
   */

  return getreg8(RSB_DATA);
}

/****************************************************************************
 * Name: a64_rsb_write
 *
 * Description:
 *   Write a byte to a device on the Reduced Serial Bus.
 *
 * Input Parameters:
 *   rt_addr  - Run-Time Address of RSB Device
 *   reg_addr - Register Address of RSB Device
 *   value    - Byte to be written
 *
 * Returned Value:
 *   Zero (OK) on success; ERROR if the write failed or timed out.
 *
 ****************************************************************************/

int a64_rsb_write(uint8_t rt_addr, uint8_t reg_addr, uint8_t value)
{
  /* RSB Command Register (A80 Page 928)
   * Set to 0x4E (WR8) to write one byte
   */

  batinfo("rt_addr=0x%x, reg_addr=0x%x, value=0x%x\n",
        rt_addr, reg_addr, value);
  putreg32(RSBCMD_WR8, RSB_CMD);

  /* RSB Device Address Register (A80 Page 928)
   * Set RTA (Bits 16 to 23) to the Run-Time Address of RSB Device
   */

  putreg32(RTA(rt_addr), RSB_DAR);

  /* RSB Address Register (A80 Page 926)
   * Set to the Register Address that will be written to RSB Device
   */

  putreg32(reg_addr, RSB_AR);

  /* RSB Data Buffer Register (A80 Page 926)
   * Set to the Register Value that will be written to RSB Device
   */

  putreg32(value, RSB_DATA);

  /* RSB Control Register (A80 Page 923)
   * Set START_TRANS (Bit 7) to 1 (Start Transaction)
   */

  putreg32(START_TRANS, RSB_CTRL);

  /* Wait for RSB Transaction to complete and return the RSB Status */

  return rsb_wait_status();
}
