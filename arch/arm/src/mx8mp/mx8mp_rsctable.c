/****************************************************************************
 * arch/arm/src/mx8mp/mx8mp_rsctable.c
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

#include "mx8mp_rsctable.h"
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NUM_VRINGS              0x02
#define RL_BUFFER_COUNT         0x100
#define VRING_ALIGN             0x1000
#define VDEV0_VRING_BASE        0x55000000
#define RESOURCE_TABLE_BASE     0x550FF000
#define VRING_SIZE              0x8000

#define NO_RESOURCE_ENTRIES     (1)
#define RSC_VDEV_FEATURE_NS     (1) /* Support name service announcement */
#define MX8MP_RSC_TABLE_VERSION (1)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Place resource table in special ELF section */
#if defined(__ARMCC_VERSION) || defined(__GNUC__)
__attribute__ ((section(".resource_table")))
#elif defined(__ICCARM__)
#pragma location = ".resource_table"
#else
#error Compiler not supported!
#endif
const struct rptun_rsc_s g_mx8mp_rsc_table =
{
    .rsc_tbl_hdr =
    {
        MX8MP_RSC_TABLE_VERSION,
        NO_RESOURCE_ENTRIES,
        {
            0, 0
        }
    },

    .offset =
    {
        offsetof(struct rptun_rsc_s, rpmsg_vdev)
    },

    .log_trace =
    {
        RSC_TRACE, 0, 0
    },

    .rpmsg_vdev = /* SRTM virtio device entry */
    {
        RSC_VDEV,
        7,
        2,
        RSC_VDEV_FEATURE_NS,
        0,
        0,
        0,
        NUM_VRINGS,
        {
            0, 0
        }
    },

    .rpmsg_vring0 =
    {
        VDEV0_VRING_BASE,
        VRING_ALIGN,
        RL_BUFFER_COUNT,
        0,
        0
    },

    .rpmsg_vring1 =
    {
        VDEV0_VRING_BASE + VRING_SIZE,
        VRING_ALIGN,
        RL_BUFFER_COUNT,
        1,
        0
    },

    .config =
    {
        0
    }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void mx8mp_copy_rsc_table(void)
{
  memcpy((void *)RESOURCE_TABLE_BASE, (void *)&g_mx8mp_rsc_table,
          sizeof(g_mx8mp_rsc_table));
}
