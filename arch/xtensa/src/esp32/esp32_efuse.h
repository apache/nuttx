/****************************************************************************
 * arch/xtensa/src/esp32/esp32_efuse.h
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

/* Type of eFuse blocks for ESP32 */

typedef enum
{
  EFUSE_BLK0 = 0,     /* Reserved. */
  EFUSE_BLK1 = 1,     /* Used for Flash Encryption. */
  EFUSE_BLK2 = 2,     /* Used for Secure Boot. */
  EFUSE_BLK3 = 3,     /* Uses for the purpose of the user. */
  EFUSE_BLK_MAX
} esp_efuse_block_t;

/* This is type of function that will handle the efuse field register.
 *
 *  num_reg          The register number in the block.
 *  efuse_block      Block number.
 *  bit_start        Start bit in the register.
 *  bit_count        The number of bits used in the register.
 *  arr              A pointer to an array or variable.
 *  bits_counter     Counter bits.
 *
 * return
 *  - OK: The operation was successfully completed.
 *  - other efuse component errors.
 */

typedef int (*efuse_func_proc_t) (unsigned int num_reg,
                                  int starting_bit_num_in_reg,
                                  int num_bits_used_in_reg,
                                  void *arr, int *bits_counter);

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

int esp_efuse_read_field(const efuse_desc_t *field[], void *dst,
                         size_t dst_size_bits);

int esp_efuse_write_field(const efuse_desc_t *field[],
                          const void *src, size_t src_size_bits);

void esp_efuse_burn_efuses(void);

int esp32_efuse_initialize(const char *devpath);
