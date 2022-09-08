/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_aes.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_ESP32C3_AES_H
#define __ARCH_RISCV_SRC_ESP32C3_ESP32C3_AES_H

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

/* AES private description */

struct esp32c3_aes_s
{
  uint32_t  key[8];     /* Key data value */
  uint16_t  keybits;    /* Key data bits */
};

/* AES XTS private description */

struct esp32c3_aes_xts_s
{
  struct esp32c3_aes_s crypt;  /* AES block encryption/decryption */
  struct esp32c3_aes_s tweak;  /* AES tweak encryption/decryption */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_aes_ecb_cypher
 *
 * Description:
 *   Process AES ECB encryption/decryption.
 *
 * Input Parameters:
 *   aes     - AES object data pointer
 *   encrypt - True: encryption mode; False: decryption mode
 *   input   - Input data pointer
 *   output  - Output buffer pointer
 *   size    - Data size in bytes
 *
 * Returned Value:
 *   OK is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32c3_aes_ecb_cypher(struct esp32c3_aes_s *aes, bool encrypt,
                           const void *input, void *output, uint32_t size);

/****************************************************************************
 * Name: esp32c3_aes_cbc_cypher
 *
 * Description:
 *   Process AES CBC encryption/decryption.
 *
 * Input Parameters:
 *   aes     - AES object data pointer
 *   encrypt - True: encryption mode; False: decryption mode
 *   ivptr   - Initialization vector pointer
 *   input   - Input data pointer
 *   output  - Output buffer pointer
 *   size    - Data size in bytes
 *
 * Returned Value:
 *   OK is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32c3_aes_cbc_cypher(struct esp32c3_aes_s *aes, bool encrypt,
                           void *ivptr, const void *input, void *output,
                           uint32_t size);

/****************************************************************************
 * Name: esp32c3_aes_ctr_cypher
 *
 * Description:
 *   Process AES CTR encryption/decryption.
 *
 * Input Parameters:
 *   aes      - AES object data pointer
 *   offptr   - Offset buffer pointer
 *   cntptr   - Counter buffer pointer
 *   cacheptr - Counter calculation buffer pointer
 *   input    - Input data pointer
 *   output   - Output buffer pointer
 *   size     - Data size in bytes
 *
 * Returned Value:
 *   OK is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32c3_aes_ctr_cypher(struct esp32c3_aes_s *aes, uint32_t *offptr,
                           void *cntptr, void *cacheptr, const void *input,
                           void *output, uint32_t size);

/****************************************************************************
 * Name: esp32c3_aes_xts_cypher
 *
 * Description:
 *   Process AES XTS encryption/decryption.
 *
 * Input Parameters:
 *   aes     - AES object data pointer
 *   encrypt - True: encryption mode; False: decryption mode
 *   unitptr - Unit data buffer pointer
 *   input   - Input data pointer
 *   output  - Output buffer pointer
 *   size    - Data size in bytes
 *
 * Returned Value:
 *   OK is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32c3_aes_xts_cypher(struct esp32c3_aes_xts_s *aes, bool encrypt,
                           void *unitptr, const void *input, void *output,
                           uint32_t size);

/****************************************************************************
 * Name: esp32c3_aes_setkey
 *
 * Description:
 *   Configurate AES key.
 *
 * Input Parameters:
 *   aes     - AES object data pointer
 *   keyptr  - Key data pointer
 *   keybits - Key data bits
 *
 * Returned Value:
 *   OK is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32c3_aes_setkey(struct esp32c3_aes_s *aes, const void *keyptr,
                       uint16_t keybits);

/****************************************************************************
 * Name: esp32c3_aes_xts_setkey
 *
 * Description:
 *   Configurate AES XTS key.
 *
 * Input Parameters:
 *   aes     - AES object data pointer
 *   keyptr  - Key data pointer
 *   keybits - Key data bits
 *
 * Returned Value:
 *   OK is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32c3_aes_xts_setkey(struct esp32c3_aes_xts_s *aes, const void *keyptr,
                           uint16_t keybits);

/****************************************************************************
 * Name: esp32c3_aes_init
 *
 * Description:
 *   Initialize ESP32-C3 AES hardware driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32c3_aes_init(void);

/****************************************************************************
 * Name: aes_cypher
 ****************************************************************************/

int esp32c3_aes_cypher(void *out, const void *in, size_t size,
                       const void *iv, const void *key, size_t keysize,
                       int mode, int encrypt);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_ESP32C3_ESP32C3_AES_H */
