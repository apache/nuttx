/****************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_ppi.h
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

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_PPI_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_PPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/nrf52_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets for PPI *************************************************/

#define NRF52_PPI_TASK_CHGEN_OFFSET(x)       (0x000 + (x * 0x8))   /* Enable channel group x */
#define NRF52_PPI_TASK_CHGDIS_OFFSET(x)      (0x004 + (x * 0x8))   /* Disable channel group x */
#define NRF52_PPI_CHEN_OFFSET                (0x500)               /* Channel enable register */
#define NRF52_PPI_CHENSET_OFFSET             (0x504)               /* Channel enable set register */
#define NRF52_PPI_CHENCLR_OFFSET             (0x508)               /* Channel enable clear register*/
#define NRF52_PPI_CHEEP_OFFSET(x)            (0x510 + (x * 0x8))   /* Channel x event end-point */
#define NRF52_PPI_CHTEP_OFFSET(x)            (0x514 + (x * 0x8))   /* Channel x task end-point */
#define NRF52_PPI_CHG_OFFSET(x)              (0x800 + (x * 0x4))   /* Channel group x */
#define NRF52_PPI_FORKTEP_OFFSET(x)          (0x910 + (x * 0x4))   /* Channel x task end-point */

/* Register addresses for PPI ***********************************************/

#define NRF52_PPI_TASK_CHGEN(x)              (NRF52_PPI_BASE + NRF52_PPI_TASK_CHGEN_OFFSET(x))
#define NRF52_PPI_TASK_CHGDIS(x)             (NRF52_PPI_BASE + NRF52_PPI_TASK_CHGDIS_OFFSET(x))
#define NRF52_PPI_CHEN                       (NRF52_PPI_BASE + NRF52_PPI_CHEN_OFFSET)
#define NRF52_PPI_CHENSET                    (NRF52_PPI_BASE + NRF52_PPI_CHENSET_OFFSET)
#define NRF52_PPI_CHENCLR                    (NRF52_PPI_BASE + NRF52_PPI_CHENCLR_OFFSET)
#define NRF52_PPI_CHEEP(x)                   (NRF52_PPI_BASE + NRF52_PPI_CHEEP_OFFSET(x))
#define NRF52_PPI_CHTEP(x)                   (NRF52_PPI_BASE + NRF52_PPI_CHTEP_OFFSET(x))
#define NRF52_PPI_CHG(x)                     (NRF52_PPI_BASE + NRF52_PPI_CHG_OFFSET(x))
#define NRF52_PPI_FORKTEP(x)                 (NRF52_PPI_BASE + NRF52_PPI_FORKTEP_OFFSET(x))

/* Register Bitfield Definitions for PPI ************************************/

#define PPI_CHEN_CH(x)                       (1 << x)              /* Enable or disable channel x */

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_PPI_H */
