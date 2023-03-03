/****************************************************************************
 * arch/arm/src/nrf53/hardware/nrf53_ctrlap.h
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

#ifndef __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_CTRLAP_H
#define __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_CTRLAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "nrf53_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF53_CTRLAP_MAILBOXRXDATA_OFFSET            0x400  /* Data sent from the debugger to the CPU */
#define NRF53_CTRLAP_MAILBOXRXSTATUS_OFFSET          0x404  /* Status to indicate if data sent from the debugger to the CPU has been read */
#define NRF53_CTRLAP_MAILBOXTXDATA_OFFSET            0x480  /* Data sent from the CPU to the debugger */
#define NRF53_CTRLAP_MAILBOXTXSTATUS_OFFSET          0x484  /* Status to indicate if data sent from the CPU to the debugger has been read */
#define NRF53_CTRLAP_ERASEPROTECTLOCK_OFFSET         0x500  /* Lock register ERASEPROTECT.DISABLE from being written until next reset */
#define NRF53_CTRLAP_ERASEPROTECTDISABLE_OFFSET      0x504  /* Disable ERASEPROTECT and perform ERASEALL */
#define NRF53_CTRLAP_APPROTECTLOCK_OFFSET            0x540  /* Lock register APPROTECT.DISABLE from being written to until next reset */
#define NRF53_CTRLAP_APPROTECTDISABLE_OFFSET         0x544  /* Disable APPROTECT and enable debug access to non-secure mode */
#define NRF53_CTRLAP_SECUREREADPROTECTLOCK_OFFSET    0x548  /* Lock register SECUREAPPROTECT.DISABLE from being written until next reset */
#define NRF53_CTRLAP_SECUREREADPROTECTDISABLE_OFFSET 0x54C  /* Disable SECUREAPPROTECT and enable debug access to secure mode */
#define NRF53_CTRLAP_STATUS_OFFSET                   0x600  /* Status bits for CTRL-AP peripheral */

/* Register definitions *****************************************************/

#define NRF53_CTRLAP_MAILBOXRXDATA            (NRF53_CTRLAP_MAILBOXRXDATA_OFFSET + NRF53_CTRLAPPERI_BASE)
#define NRF53_CTRLAP_MAILBOXRXSTATUS          (NRF53_CTRLAP_MAILBOXRXSTATUS_OFFSET + NRF53_CTRLAPPERI_BASE)
#define NRF53_CTRLAP_MAILBOXTXDATA            (NRF53_CTRLAP_MAILBOXTXDATA_OFFSET + NRF53_CTRLAPPERI_BASE)
#define NRF53_CTRLAP_MAILBOXTXSTATUS          (NRF53_CTRLAP_MAILBOXTXSTATUS_OFFSET + NRF53_CTRLAPPERI_BASE)
#define NRF53_CTRLAP_ERASEPROTECTLOCK         (NRF53_CTRLAP_ERASEPROTECTLOCK_OFFSET + NRF53_CTRLAPPERI_BASE)
#define NRF53_CTRLAP_ERASEPROTECTDISABLE      (NRF53_CTRLAP_ERASEPROTECTDISABLE_OFFSET + NRF53_CTRLAPPERI_BASE)
#define NRF53_CTRLAP_APPROTECTLOCK            (NRF53_CTRLAP_APPROTECTLOCK_OFFSET + NRF53_CTRLAPPERI_BASE)
#define NRF53_CTRLAP_APPROTECTDISABLE         (NRF53_CTRLAP_APPROTECTDISABLE_OFFSET + NRF53_CTRLAPPERI_BASE)
#define NRF53_CTRLAP_SECUREREADPROTECTLOCK    (NRF53_CTRLAP_SECUREREADPROTECTLOCK_OFFSET + NRF53_CTRLAPPERI_BASE)
#define NRF53_CTRLAP_SECUREREADPROTECTDISABLE (NRF53_CTRLAP_SECUREREADPROTECTDISABLE_OFFSET + NRF53_CTRLAPPERI_BASE)
#define NRF53_CTRLAP_STATUS                   (NRF53_CTRLAP_STATUS_OFFSET + NRF53_CTRLAPPERI_BASE)

/* Register bit definitions *************************************************/

#define CTRLAP_APPROTECTLOCK_LOCKED  (1 << 0)

#endif /* __ARCH_ARM_SRC_NRF53_HARDWARE_NRF53_CTRLAP_H */
