/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_dppi.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_DPPI_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_DPPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf53_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets for DPPI ************************************************/

#define NRF53_DPPI_TASK_CHGEN_OFFSET(x)       (0x000 + (x * 0x8))   /* Enable channel group x */
#define NRF53_DPPI_TASK_CHGDIS_OFFSET(x)      (0x004 + (x * 0x8))   /* Disable channel group x */
#define NRF53_DPPI_TASK_SUBCHGEN_OFFSET(x)    (0x080 + (x * 0x8))   /* Subscribe configuration for task CHG.EN */
#define NRF53_DPPI_TASK_SUBCHGDIS_OFFSET(x)   (0x084 + (x * 0x8))   /* Subscribe configuration for task CHG.DIS */
#define NRF53_DPPI_CHEN_OFFSET                (0x500)               /* Channel enable register */
#define NRF53_DPPI_CHENSET_OFFSET             (0x504)               /* Channel enable set register */
#define NRF53_DPPI_CHENCLR_OFFSET             (0x508)               /* Channel enable clear register*/
#define NRF53_DPPI_CHG_OFFSET(x)              (0x800 + (x * 0x4))   /* Channel group x */

/* Register addresses for DPPI **********************************************/

#define NRF53_DPPI_CHEEP(x)                   (NRF53_DPPI_BASE + NRF53_DPPI_CHEEP_OFFSET(x))
#define NRF53_DPPI_CHTEP(x)                   (NRF53_DPPI_BASE + NRF53_DPPI_CHTEP_OFFSET(x))
#define NRF53_DPPI_SUBCHGEN(x)                (NRF53_DPPI_BASE + NRF53_DPPI_SUBCHGEN_OFFSET(x))
#define NRF53_DPPI_SUBCHGDIS(x)               (NRF53_DPPI_BASE + NRF53_DPPI_SUBCHGDIS_OFFSET(x))
#define NRF53_DPPI_CHEN                       (NRF53_DPPI_BASE + NRF53_DPPI_CHEN_OFFSET)
#define NRF53_DPPI_CHENSET                    (NRF53_DPPI_BASE + NRF53_DPPI_CHENSET_OFFSET)
#define NRF53_DPPI_CHENCLR                    (NRF53_DPPI_BASE + NRF53_DPPI_CHENCLR_OFFSET)
#define NRF53_DPPI_CHG(x)                     (NRF53_DPPI_BASE + NRF53_DPPI_CHG_OFFSET(x))

/* Register Bitfield Definitions for DPPI ***********************************/

#define DPPI_CHEN_CH(x)                       (1 << x)              /* Enable or disable channel x */

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_DPPI_H */
