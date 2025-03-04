/****************************************************************************
 * drivers/coresight/coresight_common.h
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

#ifndef __DRIVERS_CORESIGHT_CORESIGHT_COMMON_H
#define __DRIVERS_CORESIGHT_CORESIGHT_COMMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Coresight registers
 * 0xF00 - 0xF9C: Management    registers
 * 0xFA0 - 0xFA4: Management    registers in PFTv1.0
 *                Trace         registers in PFTv1.1
 * 0xFA8 - 0xFFC: Management    registers
 */

#define CORESIGHT_ITCTRL               0xf00
#define CORESIGHT_CLAIMSET             0xfa0
#define CORESIGHT_CLAIMCLR             0xfa4
#define CORESIGHT_LAR                  0xfb0
#define CORESIGHT_LSR                  0xfb4
#define CORESIGHT_DEVARCH              0xfbc
#define CORESIGHT_AUTHSTATUS           0xfb8
#define CORESIGHT_DEVID                0xfc8
#define CORESIGHT_DEVTYPE              0xfcc

#define CORESIGHT_CLAIM_SELF_HOSTED    BIT(1)

/* Register operations */

#define coresight_put8(val, addr) \
        (*(FAR volatile uint8_t *)(addr) = (val))
#define coresight_put16(val, addr) \
        (*(FAR volatile uint16_t *)(addr) = (val))
#define coresight_put32(val, addr) \
        (*(FAR volatile uint32_t *)(addr) = (val))
#define coresight_put64(val, addr) \
        (*(FAR volatile uint64_t *)(addr) = (val))

#define coresight_get32(addr) \
        (*(FAR volatile uint32_t *)(addr))
#define coresight_modify32(val, mask, addr) \
        coresight_put32((coresight_get32(addr) & ~(mask)) | \
        ((val) & (mask)), (addr))

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: coresight_lock
 *
 * Description:
 *   To ensure that the software being debugged can never access an unlocked
 *   CoreSight component, a software monitor that accesses debug registers
 *   must unlock the component before accessing any registers, and lock the
 *   component again before exiting the monitor.
 *
 * Input Parameters:
 *   addr  - Base addr of the coresight device.
 *
 ****************************************************************************/

void coresight_lock(uintptr_t addr);

/****************************************************************************
 * Name: coresight_unlock
 ****************************************************************************/

void coresight_unlock(uintptr_t addr);

/****************************************************************************
 * Name: coresight_claim_device
 *
 * Description:
 *   Claim the device for self-hosted usage to prevent an external tool from
 *   touching this device.Use Protocol 3: Set private bit and check for race.
 *
 * Input Parameters:
 *   addr  - Base addr of the coresight device.
 *
 * Returned Value:
 *   Zero on success; a negative value on failure.
 *
 ****************************************************************************/

int coresight_claim_device(uintptr_t addr);

/****************************************************************************
 * Name: coresight_disclaim_device
 *
 * Description:
 *   Disclaim the device, then an external tool can touch the device.
 *
 * Input Parameters:
 *   addr  - Base addr of the coresight device.
 *
 ****************************************************************************/

void coresight_disclaim_device(uintptr_t addr);

/****************************************************************************
 * Name: coresight_get_cpu_trace_id
 *
 * Description:
 *   Used to get an unique trace id associated with cpu id of an ETM
 *   coresight device.
 *
 * Input Parameters:
 *   cpu  - CPU index to generate an unique trace id.
 *
 * Returned Value:
 *   Unique trace id on success; a negative value on failure.
 *
 ****************************************************************************/

int coresight_get_cpu_trace_id(int cpu);

/****************************************************************************
 * Name: coresight_get_system_trace_id
 *
 * Description:
 *   Used to get an unique trace id of a STM coresight device. The trace ID
 *   value for *ETM* tracers start at CPU_ID * 2 + 0x10, and Trace ID 0x00
 *   and anything equal to or higher than 0x70 is reserved.
 *
 * Returned Value:
 *   Unique trace id on success; a negative value on failure.
 *
 ****************************************************************************/

int coresight_get_system_trace_id(void);

/****************************************************************************
 * Name: coresight_put_system_trace_id
 *
 * Description:
 *    Release an allocated system trace ID.
 *
 * Input Parameters:
 *   traceid  - Traceid to be released.
 *
 ****************************************************************************/

void coresight_put_system_trace_id(int traceid);

/****************************************************************************
 * Name: coresight_timeout
 *
 * Description:
 *   Loop until a bitmask of register has changed to a specific value.
 *
 * Input Parameters:
 *   addr    - Base addr of the coresight device.
 *   off     - Register offset of the coresight device.
 *   bitmask - Bitmask to be checked.
 *   val     - Value to be matched.
 *
 * Returned Value:
 *   Zero on success; a negative value on failure.
 *
 ****************************************************************************/

int coresight_timeout(uint32_t val, uint32_t mask, uintptr_t addr);

/****************************************************************************
 * Name: coresight_insert_barrier_packet
 *
 * Description:
 *   When losing synchronisation a new barrier packet needs to be inserted at
 *   the beginning of the data collected in a buffer.  That way the decoder
 *   knows that it needs to look for another sync sequence.
 *
 * Input Parameters:
 *   buf  - buffer that a new barrier packet inserts to.
 *
 ****************************************************************************/

void coresight_insert_barrier_packet(FAR void *buf);

#endif  /* __DRIVERS_CORESIGHT_CORESIGHT_COMMON_H */
