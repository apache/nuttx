/****************************************************************************
 * boards/arm/imxrt/imxrt1020-evk/src/imxrt_flexspi_nor_boot.h
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

#ifndef __BOARDS_ARM_IMXRT1020_EVK_SRC_IMXRT_FLEXSPI_NOR_BOOT_H
#define __BOARDS_ARM_IMXRT1020_EVK_SRC_IMXRT_FLEXSPI_NOR_BOOT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IVT Data */

#define IVT_MAJOR_VERSION           0x4
#define IVT_MAJOR_VERSION_SHIFT     0x4
#define IVT_MAJOR_VERSION_MASK      0xf
#define IVT_MINOR_VERSION           0x1
#define IVT_MINOR_VERSION_SHIFT     0x0
#define IVT_MINOR_VERSION_MASK      0xf

#define IVT_VERSION(major, minor)   \
  ((((major) & IVT_MAJOR_VERSION_MASK) << IVT_MAJOR_VERSION_SHIFT) |  \
  (((minor) & IVT_MINOR_VERSION_MASK) << IVT_MINOR_VERSION_SHIFT))

#define IVT_TAG_HEADER              (0xd1)       /* Image Vector Table */
#define IVT_SIZE                    0x2000
#define IVT_PAR                     IVT_VERSION(IVT_MAJOR_VERSION, IVT_MINOR_VERSION)

#define IVT_HEADER                  (IVT_TAG_HEADER | (IVT_SIZE << 8) | (IVT_PAR << 24))
#define IVT_RSVD                    (uint32_t)(0x00000000)

/* DCD Data */

#define DCD_TAG_HEADER              (0xd2)
#define DCD_TAG_HEADER_SHIFT        (24)
#define DCD_VERSION                 (0x40)
#define DCD_ARRAY_SIZE              1

#define FLASH_BASE                  0x60000000
#define FLASH_END                   0x7f7fffff
#define SCLK 1

#  define DCD_ADDRESS               0

#define BOOT_DATA_ADDRESS           &g_boot_data
#define CSF_ADDRESS                 0
#define PLUGIN_FLAG                 (uint32_t)0

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* IVT Data */

struct ivt_s
{
  /* Header with tag #HAB_TAG_IVT, length and HAB version fields
   * (see data)
   */

  uint32_t hdr;

  /* Absolute address of the first instruction to execute from the
   * image
   */

  uint32_t entry;

  /* Reserved in this version of HAB: should be NULL. */

  uint32_t reserved1;

  /* Absolute address of the image DCD: may be NULL. */

  uint32_t dcd;

  /* Absolute address of the Boot Data: may be NULL, but not interpreted
   * any further by HAB
   */

  uint32_t boot_data;

  /* Absolute address of the IVT. */

  uint32_t self;

  /* Absolute address of the image CSF. */

  uint32_t csf;

  /* Reserved in this version of HAB: should be zero. */

  uint32_t reserved2;
};

/* Boot Data */

struct boot_data_s
{
  uint32_t start;           /* boot start location */
  uint32_t size;            /* size */
  uint32_t plugin;          /* plugin flag - 1 if downloaded application is plugin */
  uint32_t placeholder;     /* placeholder to make even 0x10 size */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const struct boot_data_s g_boot_data;

#endif /* __BOARDS_ARM_IMXRT1020_EVK_SRC_IMXRT_FLEXSPI_NOR_BOOT_H */
