/****************************************************************************
 * include/nuttx/mtd/nand_wrapper.h
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
#ifndef __INCLUDE_NUTTX_MTD_NAND_WRAPPER_H
#define __INCLUDE_NUTTX_MTD_NAND_WRAPPER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <syslog.h>

#include <nuttx/drivers/drivers.h>
#include <nuttx/mtd/nand.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct nand_wrapper_dev_s
{
  struct nand_dev_s wrapper; /* Wrapper device */
  struct nand_dev_s under;   /* Underlying actuall upper half device */
};

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

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int nand_wrapper_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks);
ssize_t nand_wrapper_bread(FAR struct mtd_dev_s *dev, off_t startpage,
                      size_t npages, FAR uint8_t *buffer);
ssize_t nand_wrapper_bwrite(FAR struct mtd_dev_s *dev, off_t startpage,
                        size_t npages, FAR const uint8_t *buffer);
int nand_wrapper_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                            unsigned long arg);
int nand_wrapper_isbad(FAR struct mtd_dev_s *dev, off_t block);
int nand_wrapper_markbad(FAR struct mtd_dev_s *dev, off_t block);
void nand_wrapper_initialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __TESTING_NAND_RAM_NAND_RAM_H */