/****************************************************************************
 * include/nuttx/crypto/aes.h
 *
 *   Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *   Extracted from the CC3000 Host Driver Implementation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_CRYPTO_AES_H
#define __INCLUDE_NUTTX_CRYPTO_AES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AES128_KEY_SIZE    16

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct aes_state_s
{
  uint8_t expanded_key[176];
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: aes_encrypt
 *
 * Description:
 *   AES128 encryption:  Given AES128 key and 16 bytes plain text, cipher
 *   text of 16 bytes is computed. The AES implementation is in mode ECB
 *   (Electronic Code Book).
 *
 * Input Parameters:
 *  key   AES128 key of size 16 bytes
 *  state 16 bytes of plain text and cipher text
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void aes_encrypt(FAR uint8_t *state, FAR const uint8_t *key);

/****************************************************************************
 * Name: aes_decrypt
 *
 * Description:
 *   AES128 decryption: Given AES128 key and 16 bytes cipher text, plain
 *   text of 16 bytes is computed The AES implementation is in mode ECB
 *   (Electronic Code Book).
 *
 * Input Parameters:
 *  key   AES128 key of size 16 bytes
 *  state 16 bytes of plain text and cipher text
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void aes_decrypt(FAR uint8_t *state, FAR const uint8_t *key);

/****************************************************************************
 * Name: aes_setupkey
 *
 * Description:
 *   Configure the given AES context for operation with the selected key.
 *
 * Input Parameters:
 *  state  an AES context that can be used for AES operations
 *  key    a pointer to a 16-byte buffer holding the AES-128 key
 *  len    length of the key, must be 16
 *
 * TODO: Support other key lengths of 24 (AES-192) and 32 (AES-256)
 *
 * Returned Value:
 *   0 if OK
 *   -EINVAL if len is not 16
 *
 ****************************************************************************/

int aes_setupkey(FAR struct aes_state_s *state, FAR const uint8_t *key,
                 int len);

/****************************************************************************
 * Name: aes_encipher
 *
 * Description:
 *   Encipher some 16-byte blocks (without any operation mode) using the
 *   previously defined key. The function can be called multiple times with
 *   the same state parameter.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void aes_encipher(FAR struct aes_state_s *state, FAR uint8_t *blocks,
                  int nblk);

/****************************************************************************
 * Name: aes_decipher
 *
 * Description:
 *   Decipher some 16-byte blocks (without any operation mode) using the
 *   previously defined key. The function can be called multiple times with
 *   the same state parameter.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void aes_decipher(FAR struct aes_state_s *state, FAR uint8_t *blocks,
                  int nblk);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __INCLUDE_NUTTX_CRYPTO_AES_H */
