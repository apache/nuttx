/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_bcrndx.c
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
 *   - UM10314 LPC3130/31 User manual Rev. 1.01 - 9 September 2009
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
 * Name: lpc31_bcrndx
 *
 * Description:
 *   Only 5 of the 12 domains have an associated BCR register.  This
 *   function returns the index to the associated BCR register (if any)
 *   or BCRNDX_INVALID otherwise.
 *
 ****************************************************************************/

int lpc31_bcrndx(enum lpc31_domainid_e dmnid)
{
  switch (dmnid)
    {
      /* BCR0-3 correspond directly to domains 0-3 */

      case DOMAINID_SYS:        /* Domain 0: SYS_BASE */
      case DOMAINID_AHB0APB0:   /* Domain 1: AHB0APB0_BASE */
      case DOMAINID_AHB0APB1:   /* Domain 2: AHB0APB1_BASE */
      case DOMAINID_AHB0APB2:   /* Domain 3: AHB0APB2_BASE */
        return (int)dmnid;

      /* There is a BCR register corresponding to domain 7, but it is at
       * index 4
       */

      case DOMAINID_CLK1024FS: /* Domain 7: CLK1024FS_BASE */
        return 4;

      default:                 /* There is no BCR register for the other
                                * domains. */
        break;
    }

  return BCRNDX_INVALID;
}
