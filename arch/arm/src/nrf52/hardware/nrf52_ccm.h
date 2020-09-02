/****************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_ccm.h
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

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_CCM_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_CCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define NRF52_CCM_TASKS_KSGEN_OFFSET      0x000000  /* Start generation of key-stream. This operation will stop by itself when completed. */
#define NRF52_CCM_TASKS_CRYPT_OFFSET      0x000004  /* Start encryption/decryption. This operation will stop by itself when completed. */
#define NRF52_CCM_TASKS_STOP_OFFSET       0x000008  /* Stop encryption/decryption */
#define NRF52_CCM_EVENTS_ENDKSGEN_OFFSET  0x000100  /* Key-stream generation complete */
#define NRF52_CCM_EVENTS_ENDCRYPT_OFFSET  0x000104  /* Encrypt/decrypt complete */
#define NRF52_CCM_EVENTS_ERROR_OFFSET     0x000108  /* CCM error event */
#define NRF52_CCM_SHORTS_OFFSET           0x000200  /* Shortcut register */
#define NRF52_CCM_INTENSET_OFFSET         0x000304  /* Enable interrupt */
#define NRF52_CCM_INTENCLR_OFFSET         0x000308  /* Disable interrupt */
#define NRF52_CCM_MICSTATUS_OFFSET        0x000400  /* MIC check result */
#define NRF52_CCM_ENABLE_OFFSET           0x000500  /* Enable */
#define NRF52_CCM_MODE_OFFSET             0x000504  /* Operation mode */
#define NRF52_CCM_CNFPTR_OFFSET           0x000508  /* Pointer to data structure holding AES key and NONCE vector */
#define NRF52_CCM_INPTR_OFFSET            0x00050c  /* Input pointer */
#define NRF52_CCM_OUTPTR_OFFSET           0x000510  /* Output pointer */
#define NRF52_CCM_SCRATCHPTR_OFFSET       0x000514  /* Pointer to data area used for temporary storage */

/* Register definitions *****************************************************/

#define NRF52_CCM_TASKS_KSGEN             (NRF52_CCM_BASE + NRF52_CCM_TASKS_KSGEN_OFFSET)
#define NRF52_CCM_TASKS_CRYPT             (NRF52_CCM_BASE + NRF52_CCM_TASKS_CRYPT_OFFSET)
#define NRF52_CCM_TASKS_STOP              (NRF52_CCM_BASE + NRF52_CCM_TASKS_STOP_OFFSET)
#define NRF52_CCM_EVENTS_ENDKSGEN         (NRF52_CCM_BASE + NRF52_CCM_EVENTS_ENDKSGEN_OFFSET)
#define NRF52_CCM_EVENTS_ENDCRYPT         (NRF52_CCM_BASE + NRF52_CCM_EVENTS_ENDCRYPT_OFFSET)
#define NRF52_CCM_EVENTS_ERROR            (NRF52_CCM_BASE + NRF52_CCM_EVENTS_ERROR_OFFSET)
#define NRF52_CCM_SHORTS                  (NRF52_CCM_BASE + NRF52_CCM_SHORTS_OFFSET)
#define NRF52_CCM_INTENSET                (NRF52_CCM_BASE + NRF52_CCM_INTENSET_OFFSET)
#define NRF52_CCM_INTENCLR                (NRF52_CCM_BASE + NRF52_CCM_INTENCLR_OFFSET)
#define NRF52_CCM_MICSTATUS               (NRF52_CCM_BASE + NRF52_CCM_MICSTATUS_OFFSET)
#define NRF52_CCM_ENABLE                  (NRF52_CCM_BASE + NRF52_CCM_ENABLE_OFFSET)
#define NRF52_CCM_MODE                    (NRF52_CCM_BASE + NRF52_CCM_MODE_OFFSET)
#define NRF52_CCM_CNFPTR                  (NRF52_CCM_BASE + NRF52_CCM_CNFPTR_OFFSET)
#define NRF52_CCM_INPTR                   (NRF52_CCM_BASE + NRF52_CCM_INPTR_OFFSET)
#define NRF52_CCM_OUTPTR                  (NRF52_CCM_BASE + NRF52_CCM_OUTPTR_OFFSET)
#define NRF52_CCM_SCRATCHPTR              (NRF52_CCM_BASE + NRF52_CCM_SCRATCHPTR_OFFSET)

/* Register bit definitions *************************************************/

#define NRF52_CCM_SHORTS_ENDKSGEN_CRYPT   (1 << 0)  /* Enable shortcut */

#define NRF52_CCM_INTENSET_ENDKSGEN       (1 << 0)  /* Read: Enabled */
#define NRF52_CCM_INTENSET_ENDCRYPT       (1 << 1)  /* Read: Enabled */
#define NRF52_CCM_INTENSET_ERROR          (1 << 2)  /* Read: Enabled */

#define NRF52_CCM_INTENCLR_ENDKSGEN       (1 << 0)  /* Read: Enabled */
#define NRF52_CCM_INTENCLR_ENDCRYPT       (1 << 1)  /* Read: Enabled */
#define NRF52_CCM_INTENCLR_ERROR          (1 << 2)  /* Read: Enabled */

#define NRF52_CCM_MICSTATUS_PASS          (1 << 0)  /* MIC check passed */

#define NRF52_CCM_ENABLE_MASK             (0x03)
#define NRF52_CCM_ENABLE_DISABLED         (0x0)  /* Disable */
#define NRF52_CCM_ENABLE_ENABLED          (0x2)  /* Enable */

#define NRF52_CCM_MODE_DECRYPT            (1 << 0)   /* AES CCM packet decryption mode */
#define NRF52_CCM_MODE_DATARATE_2M        (1 << 16)  /* In synch with 2 Mbit data rate */
#define NRF52_CCM_MODE_LENGTH_EXT         (1 << 24)  /* Extended length. Effective length of LENGTH field is 8-bit */

#endif // __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_CCM_H
