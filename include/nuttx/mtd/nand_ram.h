/****************************************************************************
 * include/nuttx/mtd/nand_ram.h
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
#ifndef __INCLUDE_NUTTX_MTD_NAND_RAM_H
#define __INCLUDE_NUTTX_MTD_NAND_RAM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <syslog.h>

#include <nuttx/drivers/drivers.h>
#include <nuttx/mtd/nand.h>
#include <nuttx/mtd/nand_scheme.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NAND_RAM_DEBUG                CONFIG_MTD_NAND_RAM_DEBUG

#define NAND_RAM_B(x)                 (x)
#define NAND_RAM_KB(x)                (NAND_RAM_B(x) << 10)
#define NAND_RAM_MB(x)                (NAND_RAM_KB(x) << 10)

#define NAND_RAM_SIZE                 NAND_RAM_MB(CONFIG_MTD_NAND_RAM_SIZE)

#define NAND_RAM_LOG_PAGES_PER_BLOCK  ((uint32_t) 7)
#define NAND_RAM_PAGE_SIZE            ((uint32_t) (1 << 9)) /* 512 B */
#define NAND_RAM_SPARE_SIZE           ((uint32_t) (1 << 4)) /* 16 B */
#define NAND_RAM_N_PAGES              ((uint32_t) NAND_RAM_SIZE / NAND_RAM_PAGE_SIZE)
#define NAND_RAM_TOTAL_PAGE_SIZE      ((uint32_t) (NAND_RAM_PAGE_SIZE + NAND_RAM_SPARE_SIZE))
#define NAND_RAM_PAGES_PER_BLOCK      ((uint32_t) (NAND_RAM_BLOCK_SIZE / NAND_RAM_PAGE_SIZE))
#define NAND_RAM_N_BLOCKS             ((uint32_t) (NAND_RAM_N_PAGES / NAND_RAM_PAGES_PER_BLOCK))
#define NAND_RAM_BLOCK_SIZE           ((uint32_t) ((1 << NAND_RAM_LOG_PAGES_PER_BLOCK) * NAND_RAM_PAGE_SIZE))

#define NAND_RAM_PAGE_WRITTEN         0
#define NAND_RAM_PAGE_FREE            1

#define NAND_RAM_BLOCK_GOOD           0xff

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN FAR struct mtd_dev_s    *g_nand_ram_mtd_wrapper;
EXTERN FAR struct mtd_dev_s    *g_nand_ram_mtd_under;
EXTERN FAR struct nand_raw_s   *g_nand_mtd_raw;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int nand_ram_eraseblock(FAR struct nand_raw_s *raw, off_t block);
int nand_ram_rawread(FAR struct nand_raw_s *raw, off_t block,
                      unsigned int page, FAR void *data, FAR void *spare);
int nand_ram_rawwrite(FAR struct nand_raw_s *raw, off_t block,
                      unsigned int page, FAR const void *data,
                      FAR const void *spare);
FAR struct mtd_dev_s *nand_ram_initialize(struct nand_raw_s *raw);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __TESTING_NAND_RAM_NAND_RAM_H */