/****************************************************************************
 * /home/v01d/coding/nuttx_nrf_ble/nuttx/arch/arm/src/chip/nrf52_ecb.c
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
#include "arm_arch.h"
#include "chip.h"
#include "hardware/nrf52_ecb.h"
#include "nrf52_ecb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void nrf52_ecb_encrypt(struct nrf52_ecb_data_s *data)
{
  /* configure data pointer to supplied value */

  putreg32((uint32_t)data, NRF52_ECB_ECBDATAPTR);

  /* start encrypt task */

  putreg32(0, NRF52_ECB_EVENTS_ENDECB);
  putreg32(0, NRF52_ECB_EVENTS_ERRORECB);
  putreg32(1, NRF52_ECB_TASKS_STARTECB);

  /* wait for task to complete */

  while (!getreg32(NRF52_ECB_EVENTS_ENDECB) &&
         !getreg32(NRF52_ECB_EVENTS_ERRORECB))
    {
      /* wait */
    }

  DEBUGASSERT(!getreg32(NRF52_ECB_EVENTS_ERRORECB));

  /* encryption complete */
}
