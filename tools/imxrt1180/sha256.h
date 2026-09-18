/****************************************************************************
 * tools/imxrt1180/sha256.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* A minimal, self-contained SHA-256 implementation (FIPS 180-4) for the
 * host-side "mkahab" tool.  Written from the published algorithm
 * specification so this host build tool has no dependency on OpenSSL or
 * any other third-party library.
 */

#ifndef __TOOLS_IMXRT1180_SHA256_H
#define __TOOLS_IMXRT1180_SHA256_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SHA256_DIGEST_SIZE 32

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct sha256_ctx_s
{
  uint32_t state[8];
  uint64_t bitcount;
  uint8_t buf[64];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void sha256_init(struct sha256_ctx_s *ctx);
void sha256_update(struct sha256_ctx_s *ctx, const void *data, size_t len);
void sha256_final(struct sha256_ctx_s *ctx,
                   uint8_t digest[SHA256_DIGEST_SIZE]);

/* Convenience one-shot helper */

void sha256_buffer(const void *data, size_t len,
                    uint8_t digest[SHA256_DIGEST_SIZE]);

#endif /* __TOOLS_IMXRT1180_SHA256_H */
