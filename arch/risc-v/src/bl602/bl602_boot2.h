/****************************************************************************
 * arch/risc-v/src/bl602/bl602_boot2.h
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

#ifndef __ARCH_RISCV_SRC_BL602_BL602_BOOT2_H
#define __ARCH_RISCV_SRC_BL602_BL602_BOOT2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Partition table error type definition */

#define PT_ERROR_SUCCESS           0 /* Success */
#define PT_ERROR_TABLE_NOT_VALID   1 /* Table not valid */
#define PT_ERROR_ENTRY_NOT_FOUND   2 /* Entry not found */
#define PT_ERROR_ENTRY_UPDATE_FAIL 3 /* Entry update fail */
#define PT_ERROR_CRC32             4 /* CRC32 error */
#define PT_ERROR_PARAMETER         5 /* Input parameter error */
#define PT_ERROR_FALSH_READ        6 /* Flash read error */
#define PT_ERROR_FALSH_WRITE       7 /* Flash write error */
#define PT_ERROR_FALSH_ERASE       8 /* Flash erase error */

/* Partition id type definition */

#define PT_TABLE_ID_0       0 /* Partition table ID 0 */
#define PT_TABLE_ID_1       1 /* Partition table ID 1 */
#define PT_TABLE_ID_INVALID 2 /* Partition table ID invalid */

/* Partition id type definition */

#define PT_ENTRY_FW_CPU0 0  /* Partition entry type:CPU0 firmware */
#define PT_ENTRY_FW_CPU1 1  /* Partition entry type:CPU1 firmware */
#define PT_ENTRY_MAX     16 /* Partition entry type:Max */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Partition table config definition */

struct pt_config_s
{
  uint32_t magic_code; /* Partition table magic code */
  uint16_t version;    /* Partition table version */
  uint16_t entry_cnt;  /* Partition table entry count */
  uint32_t age;        /* Partition table age */
  uint32_t crc32;      /* Partition table CRC32 value */
};

/* Partition table entry config definition */

struct pt_entry_config_s
{
  uint8_t  type;         /* Partition entry type */
  uint8_t  device;       /* Partition entry device */
  uint8_t  active_index; /* Partition entry active index */
  uint8_t  name[9];      /* Partition entry name */
  uint32_t address[2];   /* Partition entry start address */
  uint32_t max_len[2];   /* Partition entry max length */
  uint32_t len;          /* Partition entry length */
  uint32_t age;          /* Partition entry age */
};

/* Partition table stuff config definition */

struct pt_stuff_config_s
{
  struct pt_config_s       table;                 /* Partition table */
  struct pt_entry_config_s entries[PT_ENTRY_MAX]; /* Entries */
  uint32_t                 crc32;                 /* Entries crc32 */
};

struct boot2_partition_table_s
{
  uint8_t                  partition_active_idx;
  uint8_t                  pad[3];
  struct pt_stuff_config_s table;
};

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_BL602_BL602_BOOT2_H */
