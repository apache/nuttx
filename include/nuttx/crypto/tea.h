/****************************************************************************
 * include/nuttx/crypto/tea.h
 * Tiny Encryption Algorithm
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

#ifndef __INCLUDE_NUTTX_CRYPTO_TEA_H
#define __INCLUDE_NUTTX_CRYPTO_TEA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tea_encrypt
 *
 * Input Parameters:
 *   value = 2 x 32-bit value (input/output)
 *   key   = 4 x 32-bit Cache key (input)
 *
 ****************************************************************************/

void tea_encrypt(FAR uint32_t *value, FAR const uint32_t *key);

/****************************************************************************
 * Name: tea_decrypt
 *
 * Input Parameters:
 *   value = 2 x 32-bit value (input/output)
 *   key   = 4 x 32-bit Cache key (input)
 *
 ****************************************************************************/

void tea_decrypt(FAR uint32_t *value, FAR const uint32_t *key);

#endif /* __INCLUDE_NUTTX_CRYPTO_TEA_H */
