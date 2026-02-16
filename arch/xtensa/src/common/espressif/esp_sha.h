/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_sha.h
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_SHA_H
#define __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_SHA_H

#include <nuttx/config.h>
#include <stdint.h>

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum esp_sha_type_e
{
    ESP_SHA1_1 = 0,
    ESP_SHA2_224,
    ESP_SHA2_256,
    ESP_SHA3_384,
    ESP_SHA3_512,
    ESP_SHA_TYPE_MAX
};

enum esp_sha_state_e
{
    ESP_SHA_STATE_INIT,
    ESP_SHA_STATE_IN_PROCESS
};

/* SHA-1 context structure */

struct esp_sha1_context_s
{
    uint32_t total[2];          /* number of bytes processed  */
    uint32_t state[5];          /* intermediate digest state  */
    unsigned char buffer[64];   /* data block being processed */
    bool first_block;           /* if first then true, else false */
    enum esp_sha_type_e mode;
    enum esp_sha_state_e sha_state;
};

/* SHA-256 context structure */

struct esp_sha256_context_s
{
    uint32_t total[2];          /* number of bytes processed  */
    uint32_t state[8];          /* intermediate digest state  */
    unsigned char buffer[64];   /* data block being processed */
    bool first_block;           /* if first then true, else false */
    enum esp_sha_type_e mode;
    enum esp_sha_state_e sha_state;
};

/* SHA-512 context structure */

struct esp_sha512_context_s
{
    uint64_t total[2];          /* number of bytes processed  */
    uint64_t state[8];          /* intermediate digest state  */
    unsigned char buffer[128];  /* data block being processed */
    bool first_block;           /* if first then true, else false */
    enum esp_sha_type_e mode;
    enum esp_sha_state_e sha_state;
};

/****************************************************************************
 * Name: esp_sha_init
 *
 * Description:
 *   Initialize ESP32-C3 SHA hardware.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp_sha_init(void);

/****************************************************************************
 * Name: esp_sha1_init
 *
 * Description:
 *   Initializes a SHA-1 context.
 *
 * Input Parameters:
 *   ctx - The SHA-1 context to initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_sha1_init(struct esp_sha1_context_s *ctx);

/****************************************************************************
 * Name: esp_sha1_starts
 *
 * Description:
 *   Starts a SHA-1 checksum calculation.
 *
 * Input Parameters:
 *   ctx - The SHA-1 context to initialize
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

int esp_sha1_starts(struct esp_sha1_context_s *ctx);

/****************************************************************************
 * Name: esp_sha1_update
 *
 * Description:
 *   Feeds an input buffer into an ongoing SHA-1 checksum calculation.
 *
 * Input Parameters:
 *   ctx   - The SHA-1 context to use
 *   input - The buffer holding the input data
 *   ilen  - The length of the input data in Bytes
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp_sha1_update(struct esp_sha1_context_s *ctx,
                    const unsigned char *input,
                    size_t ilen);

/****************************************************************************
 * Name: esp_sha1_finish
 *
 * Description:
 *   Finishes the SHA-1 operation,
 *   and writes the result to the output buffer.
 *
 * Input Parameters:
 *   ctx    - The SHA-1 context to use
 *   output - The SHA-1 checksum result
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp_sha1_finish(struct esp_sha1_context_s *ctx,
                    unsigned char output[20]);

/****************************************************************************
 * Name: esp_sha256_init
 *
 * Description:
 *   Initializes a SHA-256 context.
 *
 * Input Parameters:
 *   ctx - The SHA-256 context to initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_sha256_init(struct esp_sha256_context_s *ctx);

/****************************************************************************
 * Name: esp_sha256_starts
 *
 * Description:
 *   Starts a SHA-224 or SHA-256 checksum calculation.
 *
 * Input Parameters:
 *   ctx   - The SHA-256 context to initialize
 *   is224 - Determines which function to use
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

int esp_sha256_starts(struct esp_sha256_context_s *ctx, bool is224);

/****************************************************************************
 * Name: esp_sha256_update
 *
 * Description:
 *   Feeds an input buffer into an ongoing SHA-224 or SHA-256
 *   checksum calculation.
 *
 * Input Parameters:
 *   ctx   - The SHA-256 context to use
 *   input - The buffer holding the input data
 *   ilen  - The length of the input data in Bytes
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp_sha256_update(struct esp_sha256_context_s *ctx,
                      const unsigned char *input,
                      size_t ilen);

/****************************************************************************
 * Name: esp_sha256_finish
 *
 * Description:
 *   Finishes the SHA-224 or SHA-256 operation, and writes the result to
 *   the output buffer.
 *
 * Input Parameters:
 *   ctx    - The SHA-256 context to use
 *   output - The SHA-256 checksum result
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp_sha256_finish(struct esp_sha256_context_s *ctx,
                      unsigned char output[32]);

/****************************************************************************
 * Name: esp_sha512_init
 *
 * Description:
 *   Initializes a SHA-512 context.
 *
 * Input Parameters:
 *   ctx - The SHA-512 context to initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_sha512_init(struct esp_sha512_context_s *ctx);

/****************************************************************************
 * Name: esp_sha512_starts
 *
 * Description:
 *   Starts a SHA-384 or SHA-512 checksum calculation.
 *
 * Input Parameters:
 *   ctx   - The SHA-512 context to initialize
 *   is384 - Determines which function to use
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

int esp_sha512_starts(struct esp_sha512_context_s *ctx, bool is384);

/****************************************************************************
 * Name: esp_sha512_update
 *
 * Description:
 *   Feeds an input buffer into an ongoing SHA-384 or SHA-512
 *   checksum calculation.
 *
 * Input Parameters:
 *   ctx   - The SHA-512 context to use
 *   input - The buffer holding the input data
 *   ilen  - The length of the input data in Bytes
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp_sha512_update(struct esp_sha512_context_s *ctx,
                      const unsigned char *input,
                      size_t ilen);

/****************************************************************************
 * Name: esp_sha512_finish
 *
 * Description:
 *   Finishes the SHA-384 or SHA-512 operation, and writes the result to
 *   the output buffer.
 *
 * Input Parameters:
 *   ctx    - The SHA-512 context to use
 *   output - The SHA-512 checksum result
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp_sha512_finish(struct esp_sha512_context_s *ctx,
                      unsigned char output[64]);

/****************************************************************************
 * Name: esp_sha512_free
 *
 * Description:
 *   Clears a SHA-512 context.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_sha512_free(struct esp_sha512_context_s *ctx);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_SHA_H */
