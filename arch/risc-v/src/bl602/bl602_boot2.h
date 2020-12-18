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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_BOOT2_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_BOOT2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
/* Partition table error type definition */

enum pt_table_error_e
{
  PT_ERROR_SUCCESS,           /* Partition table error type:success */
  PT_ERROR_TABLE_NOT_VALID,   /* Partition table error type:entry not found */
  PT_ERROR_ENTRY_NOT_FOUND,   /* Partition table error type:entry not found */
  PT_ERROR_ENTRY_UPDATE_FAIL, /* Partition table error type:entry update fail
                               */

  PT_ERROR_CRC32,       /* Partition table error type:crc32 error */
  PT_ERROR_PARAMETER,   /* Partition table error type:input parameter error */
  PT_ERROR_FALSH_READ,  /* Partition table error type:flash read error */
  PT_ERROR_FALSH_WRITE, /* Partition table error type:flash write error */
  PT_ERROR_FALSH_ERASE  /* Partition table error type:flash erase error */
};

/* Partition id type definition */

enum pt_table_id_e
{
  PT_TABLE_ID_0,       /* Partition table ID 0 */
  PT_TABLE_ID_1,       /* Partition table ID 1 */
  PT_TABLE_ID_INVALID, /* Partition table ID invalid */
};

/* Partition id type definition */

enum pt_table_entry_type_e
{
  PT_ENTRY_FW_CPU0,  /* Partition entry type:CPU0 firmware */
  PT_ENTRY_FW_CPU1,  /* Partition entry type:CPU1 firmware */
  PT_ENTRY_MAX = 16, /* Partition entry type:Max */
};

/* Partition table config definition */

struct pt_table_config_s
{
  uint32_t magic_code; /* Partition table magic code */
  uint16_t version;    /* Partition table verdion */
  uint16_t entry_cnt;  /* Partition table entry count */
  uint32_t age;        /* Partition table age */
  uint32_t crc32;      /* Partition table CRC32 value */
};

/* Partition table entry config definition */

struct pt_table_entry_config_s
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

struct pt_table_stuff_config_s
{
  struct pt_table_config_s pt_table; /* Partition table */
  struct pt_table_entry_config_s
           pt_entries[PT_ENTRY_MAX]; /* Partition entries */
  uint32_t crc32;                    /* Partition entries crc32 */
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
 * Name: <Inline function name>
 *
 * Description:
 *   Description of the operation of the inline function.
 *
 * Input Parameters:
 *   A list of input parameters, one-per-line, appears here along with a
 *   description of each input parameter.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   Anything else that one might need to know to use this function.
 *
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: <Global function name>
 *
 * Description:
 *   Description of the operation of the function.
 *
 * Input Parameters:
 *   A list of input parameters, one-per-line, appears here along with a
 *   description of each input parameter.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   Anything else that one might need to know to use this function.
 *
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_BOOT2_H */
