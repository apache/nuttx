/****************************************************************************
 * drivers/crypto/pnt/scp03_keys.h
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

/* Copyright 2023 NXP */

#ifndef __INCLUDE_NUTTX_CRYPTO_PNT_SCP03_KEYS_H_
#define __INCLUDE_NUTTX_CRYPTO_PNT_SCP03_KEYS_H_

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SCP03_ENC_KEY                                                        \
  {0xbd, 0x1d, 0xe2, 0x0a, 0x81, 0xea, 0xb2, 0xbf,                           \
   0x3b, 0x70, 0x9a, 0x9d, 0x69, 0xa3, 0x12, 0x54};
#define SCP03_MAC_KEY                                                        \
  {0x9a, 0x76, 0x1b, 0x8d, 0xba, 0x6b, 0xed, 0xf2,                           \
   0x27, 0x41, 0xe4, 0x5d, 0x8d, 0x42, 0x36, 0xf5};
#define SCP03_DEK_KEY                                                        \
  {0x9b, 0x99, 0x3b, 0x60, 0x0f, 0x1c, 0x64, 0xf5,                           \
   0xad, 0xc0, 0x63, 0x19, 0x2a, 0x96, 0xc9, 0x47};

#endif /* __INCLUDE_NUTTX_CRYPTO_PNT_SCP03_KEYS_H_ */
