/****************************************************************************
 * arch/xtensa/src/esp32/esp32_sha.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_SHA_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_SHA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum esp32_sha_type_e
{
    ESP32_SHA1_1 = 0,
    ESP32_SHA2_256,
    ESP32_SHA3_384,
    ESP32_SHA3_512,
    ESP32_SHA_TYPE_MAX
};

enum esp32_sha_state_e
{
    ESP32_SHA_STATE_INIT,
    ESP32_SHA_STATE_IN_PROCESS
};

/* SHA context structure */

struct esp32_sha_context_s
{
    uint64_t count[2];           /* number of bits processed  */
    uint32_t state[16];          /* intermediate digest state  */
    bool final_block;
    unsigned char buffer[128];   /* data block being processed */
    bool first_block;            /* if first then true else false */
    uint16_t output_size;
    uint16_t block_size;
    enum esp32_sha_type_e mode;
    enum esp32_sha_state_e sha_state;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_sha_init
 *
 * Description:
 *   Initialize ESP32 SHA hardware.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32_sha_init(void);

/****************************************************************************
 * Name: esp32_sha1_init
 *
 * Description:
 *   Initializes a SHA context.
 *
 * Input Parameters:
 *   ctx      - The SHA context to initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32_sha1_init(struct esp32_sha_context_s *ctx);

/****************************************************************************
 * Name: esp32_sha1_starts
 *
 * Description:
 *   Starts a SHA checksum calculation.
 *
 * Input Parameters:
 *   ctx      - The SHA context to initialize
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

int esp32_sha1_starts(struct esp32_sha_context_s *ctx);

/****************************************************************************
 * Name: esp32_sha_update
 *
 * Description:
 *   Feeds an input buffer into an ongoing SHA checksum calculation.
 *
 * Input Parameters:
 *   ctx      - The SHA context to use
 *   input    - The buffer holding the input data
 *   ilen     - The length of the input data in Bytes
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32_sha_update(struct esp32_sha_context_s *ctx,
                     const unsigned char *input,
                     size_t ilen);

/****************************************************************************
 * Name: esp32_sha_finish
 *
 * Description:
 *   Finishes the SHA operation,
 *   and writes the result to the output buffer.
 *
 * Input Parameters:
 *   ctx      - The SHA context to use
 *   output   - The SHA checksum result
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32_sha_finish(struct esp32_sha_context_s *ctx,
                     unsigned char output[64]);

/****************************************************************************
 * Name: esp32_sha256_starts
 *
 * Description:
 *   Starts a SHA-256 checksum calculation.
 *
 * Input Parameters:
 *   ctx      - The SHA context to initialize
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

int esp32_sha256_starts(struct esp32_sha_context_s *ctx);

/****************************************************************************
 * Name: esp32_sha384_starts
 *
 * Description:
 *   Starts a SHA-384 checksum calculation.
 *
 * Input Parameters:
 *   ctx      - The SHA context to initialize
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

int esp32_sha384_starts(struct esp32_sha_context_s *ctx);

/****************************************************************************
 * Name: esp32_sha512_starts
 *
 * Description:
 *   Starts a SHA-512 checksum calculation.
 *
 * Input Parameters:
 *   ctx      - The SHA context to initialize
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

int esp32_sha512_starts(struct esp32_sha_context_s *ctx);

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_SHA_H */
