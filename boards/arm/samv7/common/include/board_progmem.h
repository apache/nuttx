/****************************************************************************
 * boards/arm/samv7/common/include/board_progmem.h
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

#ifndef __BOARDS_ARM_SAMV7_COMMON_INCLUDE_BOARD_PROGMEM_H
#define __BOARDS_ARM_SAMV7_COMMON_INCLUDE_BOARD_PROGMEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>

#include <nuttx/mtd/mtd.h>

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct mtd_partition_s
{
  size_t            offset;    /* Partition offset from the beginning of MTD */
  size_t            size;      /* Partition size in bytes */
  uint8_t           type;      /* Optional if more devices types (block
                                * character, smartfs) have to be distinguish
                                * in board related code.
                                */
  const char       *devpath;   /* Partition device path */
  struct mtd_dev_s *mtd;       /* Pointer to allocated MTD partition */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: board_progmem_init
 *
 * Description:
 *   Initialize the FLASH and register MTD devices.
 *
 * Input Parameters:
 *   minor - The starting minor number for progmem MTD partitions.
 *   table - Progmem MTD partition table
 *   count - Number of element in progmem MTD partition table
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_progmem_init(int minor, struct mtd_partition_s *table,
                       size_t count);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAMV7_COMMON_INCLUDE_BOARD_PROGMEM_H */
