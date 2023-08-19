/****************************************************************************
 * arch/arm/src/nrf53/nrf53_spu.c
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

#include "arm_internal.h"

#include "hardware/nrf53_spu.h"

#include "nrf53_spu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef HAVE_SPU_CONFIG
/****************************************************************************
 * Name: nrf53_spu_configure
 ****************************************************************************/

void nrf53_spu_configure(void)
{
#ifdef CONFIG_RPTUN
  /* Set secure domain - this allows net core to access shared mem */

  putreg32(SPU_EXTDOMAIN_SECUREMAPPING_SECATTR, NRF53_SPU_EXTDOMAIN(0));
#endif
}
#endif
