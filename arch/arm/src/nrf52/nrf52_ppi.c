/****************************************************************************
 * arch/arm/src/nrf52/nrf52_ppi.c
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
#include "nrf52_ppi.h"
#include "hardware/nrf52_ppi.h"

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

/****************************************************************************
 * Name: nrf52_ppi_channel_enable
 ****************************************************************************/

void nrf52_ppi_channel_enable(uint8_t ch, bool enable)
{
  DEBUGASSERT(ch < NRF52_PPI_NUM_CHANNELS);

  if (enable)
    {
      DEBUGASSERT(!(getreg32(NRF52_PPI_CHENSET) & PPI_CHEN_CH(ch)));
      putreg32(PPI_CHEN_CH(ch), NRF52_PPI_CHENSET);
    }
  else
    {
      DEBUGASSERT(getreg32(NRF52_PPI_CHENSET) & PPI_CHEN_CH(ch));
      putreg32(PPI_CHEN_CH(ch), NRF52_PPI_CHENCLR);
    }
}

/****************************************************************************
 * Name: nrf52_ppi_set_event_ep
 ****************************************************************************/

void nrf52_ppi_set_event_ep(uint8_t ch, uint32_t event_reg)
{
  DEBUGASSERT(ch < NRF52_PPI_NUM_CONFIGURABLE_CHANNELS);

  putreg32(event_reg, NRF52_PPI_CHEEP(ch));
}

/****************************************************************************
 * Name: nrf52_ppi_set_task_ep
 ****************************************************************************/

void nrf52_ppi_set_task_ep(uint8_t ch, uint32_t task_reg)
{
  DEBUGASSERT(ch < NRF52_PPI_NUM_CONFIGURABLE_CHANNELS);

  putreg32(task_reg, NRF52_PPI_CHTEP(ch));
}

/****************************************************************************
 * Name: nrf52_ppi_set_task2_ep
 ****************************************************************************/

void nrf52_ppi_set_task2_ep(uint8_t ch, uint32_t task_reg)
{
  DEBUGASSERT(ch < NRF52_PPI_NUM_CHANNELS);

  putreg32(task_reg, NRF52_PPI_FORKTEP(ch));
}

/****************************************************************************
 * Name: nrf52_ppi_grp_channel_enable
 ****************************************************************************/

void nrf52_ppi_grp_channel_enable(uint8_t group, uint8_t ch, bool enable)
{
  DEBUGASSERT(group < NRF52_PPI_NUM_GROUPS);
  DEBUGASSERT(ch < NRF52_PPI_NUM_CHANNELS);

  modifyreg32(NRF52_PPI_CHG(group), (enable ? 0 : PPI_CHEN_CH(ch)),
                                    (enable ? PPI_CHEN_CH(ch) : 0));
}

/****************************************************************************
 * Name: nrf52_ppi_grp_clear
 ****************************************************************************/

void nrf52_ppi_grp_clear(uint8_t group)
{
  DEBUGASSERT(group < NRF52_PPI_NUM_GROUPS);

  putreg32(0, NRF52_PPI_CHG(group));
}

/****************************************************************************
 * Name: nrf52_ppi_grp_enable
 ****************************************************************************/

void nrf52_ppi_grp_enable(uint8_t group, bool enable)
{
  DEBUGASSERT(group < NRF52_PPI_NUM_GROUPS);

  putreg32(1, (enable ? NRF52_PPI_TASK_CHGEN(group) :
                        NRF52_PPI_TASK_CHGDIS(group)));
}
