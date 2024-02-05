/****************************************************************************
 * arch/arm/src/nrf91/nrf91_modem_at.h
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

#ifndef __ARCH_ARM_SRC_NRF91_NRF91_MODEM_AT_H
#define __ARCH_ARM_SRC_NRF91_NRF91_MODEM_AT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "nrf_modem_at.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Modem functional mode */

enum nrf91_modem_func_e
{
  NRF91_MODEM_FUNC_DISABLED        = 0,
  NRF91_MODEM_FUNC_FULL            = 1,
  NRF91_MODEM_FUNC_RXONLY          = 2,
  NRF91_MODEM_FUNC_FLIGHT          = 4,
  NRF91_MODEM_FUNC_DEACTIVATE_LTE  = 20,
  NRF91_MODEM_FUNC_ACTIVATE_LTE    = 21,
  NRF91_MODEM_FUNC_DEACTIVATE_GNSS = 30,
  NRF91_MODEM_FUNC_ACTIVATE_GNSS   = 31,
  NRF91_MODEM_FUNC_DEACTIVATE_UICC = 40,
  NRF91_MODEM_FUNC_ACTIVATE_UICC   = 41,
  NRF91_MODEM_FUNC_FLIGHT_UICC     = 44
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_at_register
 ****************************************************************************/

int nrf91_at_register(const char *path);

#endif /* __ARCH_ARM_SRC_NRF91_NRF91_MODEM_AT_H */
