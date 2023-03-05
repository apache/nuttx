/***************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_uicr_cpunet.h
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
 ***************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_UICR_CPUNET_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_UICR_CPUNET_H

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf53_memorymap.h"

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* UICR Register Offsets ***************************************************/

#define NRF53_UICR_APPROTECT_OFFSET         0x000  /* Access port protection */
#define NRF53_UICR_ERASEPROTECT_OFFSET      0x004  /* Erase protection */
#define NRF53_UICR_NRFFW_OFFSET             0x200  /* Reserved for Nordic firmware design */
#define NRF53_UICR_CUSTOMER_OFFSET          0x300  /* Reserved for customer */

/* UICR Register Addresses *************************************************/

#define NRF53_UICR_APPROTECT         (NRF53_UICR_BASE + NRF53_UICR_APPROTECT_OFFSET)
#define NRF53_UICR_ERASEPROTECT      (NRF53_UICR_BASE + NRF53_UICR_ERASEPROTECT_OFFSET)
#define NRF53_UICR_NRFFW             (NRF53_UICR_BASE + NRF53_UICR_NRFFW_OFFSET)
#define NRF53_UICR_CUSTOMER          (NRF53_UICR_BASE + NRF53_UICR_CUSTOMER_OFFSET)

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_UICR_CPUNET_H */
