/****************************************************************************
 * libs/libc/misc/lib_tea_encrypt.c
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

/* https://en.wikipedia.org/wiki/Tiny_Encryption_Algorithm:  "Following is
 * an adaptation of the reference encryption and decryption routines in C,
 * released into the public domain by David Wheeler and Roger Needham."
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <nuttx/crypto/tea.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TEA_KEY_SCHEDULE_CONSTANT  0x9e3779b9
#define TEA_SUM_SETUP_CONSTANT     0xc6ef3720

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tea_encrypt
 *
 * Input Parameters:
 *   value = 2 x 32-bit value (input/output)
 *   key   = 4 x 32-bit Cache key (input)
 *
 ****************************************************************************/

void tea_encrypt(FAR uint32_t *value, FAR const uint32_t *key)
{
  uint32_t v0  = value[0];
  uint32_t v1  = value[1];
  uint32_t sum = 0;
  int i;

  /* Basic cycle start */

  for (i = 0; i < 32; i++)
    {
      sum += TEA_KEY_SCHEDULE_CONSTANT;
      v0  += ((v1 << 4) + key[0]) ^ (v1 + sum) ^ ((v1 >> 5) + key[1]);
      v1  += ((v0 << 4) + key[2]) ^ (v0 + sum) ^ ((v0 >> 5) + key[3]);
    }

  /* End cycle */

  value[0] = v0;
  value[1] = v1;
}
