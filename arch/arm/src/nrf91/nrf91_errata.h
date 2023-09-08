/****************************************************************************
 * arch/arm/src/nrf91/nrf91_errata.h
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

#ifndef __ARCH_ARM_SRC_NRF91_NRF91_ERRATA_H
#define __ARCH_ARM_SRC_NRF91_NRF91_ERRATA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_ARCH_TRUSTZONE_NONSECURE
/****************************************************************************
 * Name: nrf91_errata_secure
 ****************************************************************************/

void nrf91_errata_secure(void);

/****************************************************************************
 * Name: nrf91_ficr_ram_copy
 ****************************************************************************/

void nrf91_ficr_ram_copy(void);
#endif

#endif /* __ARCH_ARM_SRC_NRF91_NRF91_ERRATA_H */
