/****************************************************************************
 * arch/arm/src/mcx-nxxx/nxxx_port.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <sys/types.h>
#include <stdint.h>

#include "chip.h"
#include "nxxx_port.h"
#include "arm_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxxx_port_configure
 *
 * Description:
 *   This function writes the PORT configuration for a port.pin pair.
 *
 * Input Parameters:
 *   cfg - The PORT configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int nxxx_port_configure(port_cfg_t cfg)
{
  uint32_t regaddr = NXXX_PORT_PCR_BASE(cfg.port, cfg.pin);
  putreg32(cfg.cfg, regaddr);

  return OK;
}

/****************************************************************************
 * Name: nxxx_port_gpio
 *
 * Description:
 *   This can be used to forcibly set a port.pin to GPIO mode. This overrides
 *   and disconnects any peripheral using the pin.
 *
 * Input Parameters:
 *   cfg - The PORT configuration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int nxxx_port_gpio(port_cfg_t cfg)
{
  uint32_t regaddr = NXXX_PORT_PCR_BASE(cfg.port, cfg.pin);
  modifyreg32(regaddr, PORT_PCR_MUX_MASK, PORT_PCR_MUX_GPIO);

  return OK;
}
