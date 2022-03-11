/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_clkdomain.c
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

/* References:
 *   - UM10314 LPC3130/31 User manual Rev. 1.01 — 9 September 2009
 *   - lpc313x.cdl.drivers.zip example driver code
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include "arm_internal.h"
#include "lpc31_cgudrvr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc31_clkdomain
 *
 * Description:
 *   Given a clock ID, return the ID of the domain in which the clock
 *   resides.
 *
 ****************************************************************************/

enum lpc31_domainid_e lpc31_clkdomain(enum lpc31_clockid_e clkid)
{
  if (clkid <= CLKID_SYSBASE_LAST)             /* Domain 0: SYS_BASE */
    {
      return DOMAINID_SYS;
    }
  else if (clkid <= CLKID_AHB0APB0_LAST)       /* Domain 1: AHB0APB0_BASE */
    {
      return DOMAINID_AHB0APB0;
    }
  else if (clkid <= CLKID_AHB0APB1_LAST)       /* Domain 2: AHB0APB1_BASE */
    {
      return DOMAINID_AHB0APB1;
    }
  else if (clkid <= CLKID_AHB0APB2_LAST)       /* Domain 3: AHB0APB2_BASE */
    {
      return DOMAINID_AHB0APB2;
    }
  else if (clkid <= CLKID_AHB0APB3_LAST)       /* Domain 4: AHB0APB3_BASE */
    {
      return DOMAINID_AHB0APB3;
    }
  else if (clkid <= CLKID_PCM_LAST)            /* Domain 5: PCM_BASE */
    {
      return DOMAINID_PCM;
    }
  else if (clkid <= CLKID_UART_LAST)           /* Domain 6: UART_BASE */
    {
      return DOMAINID_UART;
    }
  else if (clkid <= CLKID_CLK1024FS_LAST)      /* Domain 7: CLK1024FS_BASE */
    {
      return DOMAINID_CLK1024FS;
    }
  else if (clkid <= CLKID_I2SRXBCK0_LAST)      /* Domain 8: BCK0_BASE */
    {
      return DOMAINID_BCK0;
    }
  else if (clkid <= CLKID_I2SRXBCK1_LAST)      /* Domain 9: BCK1_BASE */
    {
      return DOMAINID_BCK1;
    }
  else if (clkid <= CLKID_SPI_LAST)            /* Domain 10: SPI_BASE */
    {
      return DOMAINID_SPI;
    }
  else /* if (clkid <= CLKID_SYSCLKO_LAST) */  /* Domain 11: SYSCLKO_BASE */
    {
      return DOMAINID_SYSCLKO;
    }
}
