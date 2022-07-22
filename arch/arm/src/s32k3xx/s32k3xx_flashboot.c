/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_flashboot.c
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

/* Copyright 2022 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const uint32_t CM7_0_START_ADDRESS;

typedef const struct image_vector_table
{
    uint32_t HEADER;                     /* Header of IVT Structure */
    uint32_t BOOTCONFIG;                 /* Boot Configuration Word */
    const uint32_t reserved1;            /* Reserved */
    const uint32_t * CM7_0_STARTADDRESS; /* Start Address of Application on CM7_0 Core */
    const uint32_t reserved2;            /* Reserved */
    const uint32_t * CM7_1_STARTADDRESS; /* Start Address of Application on CM7_1 Core */
    const uint64_t reserved3;            /* Reserved */
    const uint32_t reserved4;            /* Reserved */
    const uint32_t * LCCONFIG;           /* Address of LC configuration Word */
    const uint32_t reserved5;            /* Reserved */
    const uint32_t reserved6;            /* Reserved */
    uint8_t reserved7[192];              /* Reserved for future use */
    uint8_t reserved8[16];               /* Reserved. */
}ivt_t;

const ivt_t boot_header locate_data(".boot_header") =
{
  .HEADER = 0x5aa55aa5,                         /* Header of IVT Structure */
  .BOOTCONFIG = 1,                              /* Boot Configuration Word: CM7_0_ENABLE */
  .CM7_0_STARTADDRESS = (const void *)&CM7_0_START_ADDRESS,
  .CM7_1_STARTADDRESS = 0,                      /* Application on CM7_1 Core */
  .LCCONFIG = 0,                                /* Address of LC configuration Word */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

