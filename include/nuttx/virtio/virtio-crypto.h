/****************************************************************************
 * include/nuttx/virtio/virtio-crypto.h
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

#ifndef __INCLUDE_NUTTX_VIRTIO_VIRTIO_CRYPTO_H
#define __INCLUDE_NUTTX_VIRTIO_VIRTIO_CRYPTO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Crypto feature bits */

#define VIRTIO_CRYPTO_F_REVISION_1                          0
#define VIRTIO_CRYPTO_F_CIPHER_STATELESS_MODE               1
#define VIRTIO_CRYPTO_F_HASH_STATELESS_MODE                 2
#define VIRTIO_CRYPTO_F_MAC_STATELESS_MODE                  3
#define VIRTIO_CRYPTO_F_AEAD_STATELESS_MODE                 4
#define VIRTIO_CRYPTO_F_AKCIPHER_STATELESS_MODE             5

/* CIPHER (Symmetric Key Cipher) service */

#define VIRTIO_CRYPTO_SERVICE_CIPHER                        0

/* HASH service */

#define VIRTIO_CRYPTO_SERVICE_HASH                          1

/* MAC (Message Authentication Codes) service */

#define VIRTIO_CRYPTO_SERVICE_MAC                           2

/* AEAD (Authenticated Encryption with Associated Data) service */

#define VIRTIO_CRYPTO_SERVICE_AEAD                          3

/* AKCIPHER (Asymmetric Key Cipher) service */

#define VIRTIO_CRYPTO_SERVICE_AKCIPHER                      4

/* CIPHER services */

#define VIRTIO_CRYPTO_NO_CIPHER                             0
#define VIRTIO_CRYPTO_CIPHER_ARC4                           1
#define VIRTIO_CRYPTO_CIPHER_AES_ECB                        2
#define VIRTIO_CRYPTO_CIPHER_AES_CBC                        3
#define VIRTIO_CRYPTO_CIPHER_AES_CTR                        4
#define VIRTIO_CRYPTO_CIPHER_DES_ECB                        5
#define VIRTIO_CRYPTO_CIPHER_DES_CBC                        6
#define VIRTIO_CRYPTO_CIPHER_3DES_ECB                       7
#define VIRTIO_CRYPTO_CIPHER_3DES_CBC                       8
#define VIRTIO_CRYPTO_CIPHER_3DES_CTR                       9
#define VIRTIO_CRYPTO_CIPHER_KASUMI_F8                      10
#define VIRTIO_CRYPTO_CIPHER_SNOW3G_UEA2                    11
#define VIRTIO_CRYPTO_CIPHER_AES_F8                         12
#define VIRTIO_CRYPTO_CIPHER_AES_XTS                        13
#define VIRTIO_CRYPTO_CIPHER_ZUC_EEA3                       14

/* HASH services */

#define VIRTIO_CRYPTO_NO_HASH                               0
#define VIRTIO_CRYPTO_HASH_MD5                              1
#define VIRTIO_CRYPTO_HASH_SHA1                             2
#define VIRTIO_CRYPTO_HASH_SHA_224                          3
#define VIRTIO_CRYPTO_HASH_SHA_256                          4
#define VIRTIO_CRYPTO_HASH_SHA_384                          5
#define VIRTIO_CRYPTO_HASH_SHA_512                          6
#define VIRTIO_CRYPTO_HASH_SHA3_224                         7
#define VIRTIO_CRYPTO_HASH_SHA3_256                         8
#define VIRTIO_CRYPTO_HASH_SHA3_384                         9
#define VIRTIO_CRYPTO_HASH_SHA3_512                         10
#define VIRTIO_CRYPTO_HASH_SHA3_SHAKE128                    11
#define VIRTIO_CRYPTO_HASH_SHA3_SHAKE256                    12

/* MAC services */

#define VIRTIO_CRYPTO_NO_MAC                                0
#define VIRTIO_CRYPTO_MAC_HMAC_MD5                          1
#define VIRTIO_CRYPTO_MAC_HMAC_SHA1                         2
#define VIRTIO_CRYPTO_MAC_HMAC_SHA_224                      3
#define VIRTIO_CRYPTO_MAC_HMAC_SHA_256                      4
#define VIRTIO_CRYPTO_MAC_HMAC_SHA_384                      5
#define VIRTIO_CRYPTO_MAC_HMAC_SHA_512                      6
#define VIRTIO_CRYPTO_MAC_CMAC_3DES                         25
#define VIRTIO_CRYPTO_MAC_CMAC_AES                          26
#define VIRTIO_CRYPTO_MAC_KASUMI_F9                         27
#define VIRTIO_CRYPTO_MAC_SNOW3G_UIA2                       28
#define VIRTIO_CRYPTO_MAC_GMAC_AES                          41
#define VIRTIO_CRYPTO_MAC_GMAC_TWOFISH                      42
#define VIRTIO_CRYPTO_MAC_CBCMAC_AES                        49
#define VIRTIO_CRYPTO_MAC_CBCMAC_KASUMI_F9                  50
#define VIRTIO_CRYPTO_MAC_XCBC_AES                          53
#define VIRTIO_CRYPTO_MAC_ZUC_EIA3                          54

/* AEAD services */

#define VIRTIO_CRYPTO_NO_AEAD                               0
#define VIRTIO_CRYPTO_AEAD_GCM                              1
#define VIRTIO_CRYPTO_AEAD_CCM                              2
#define VIRTIO_CRYPTO_AEAD_CHACHA20_POLY1305                3

/* AKCIPHER services */

#define VIRTIO_CRYPTO_NO_AKCIPHER                           0
#define VIRTIO_CRYPTO_AKCIPHER_RSA                          1
#define VIRTIO_CRYPTO_AKCIPHER_ECDSA                        2

#define VIRTIO_CRYPTO_S_HW_READY                            (1 << 0)

#define VIRTIO_CRYPTO_OPCODE(service, op)   (((service) << 8) | (op))

#define VIRTIO_CRYPTO_CIPHER_CREATE_SESSION \
       VIRTIO_CRYPTO_OPCODE(VIRTIO_CRYPTO_SERVICE_CIPHER, 0x02)
#define VIRTIO_CRYPTO_CIPHER_DESTROY_SESSION \
       VIRTIO_CRYPTO_OPCODE(VIRTIO_CRYPTO_SERVICE_CIPHER, 0x03)
#define VIRTIO_CRYPTO_HASH_CREATE_SESSION \
       VIRTIO_CRYPTO_OPCODE(VIRTIO_CRYPTO_SERVICE_HASH, 0x02)
#define VIRTIO_CRYPTO_HASH_DESTROY_SESSION \
       VIRTIO_CRYPTO_OPCODE(VIRTIO_CRYPTO_SERVICE_HASH, 0x03)
#define VIRTIO_CRYPTO_MAC_CREATE_SESSION \
       VIRTIO_CRYPTO_OPCODE(VIRTIO_CRYPTO_SERVICE_MAC, 0x02)
#define VIRTIO_CRYPTO_MAC_DESTROY_SESSION \
       VIRTIO_CRYPTO_OPCODE(VIRTIO_CRYPTO_SERVICE_MAC, 0x03)
#define VIRTIO_CRYPTO_AEAD_CREATE_SESSION \
       VIRTIO_CRYPTO_OPCODE(VIRTIO_CRYPTO_SERVICE_AEAD, 0x02)
#define VIRTIO_CRYPTO_AEAD_DESTROY_SESSION \
       VIRTIO_CRYPTO_OPCODE(VIRTIO_CRYPTO_SERVICE_AEAD, 0x03)
#define VIRTIO_CRYPTO_AKCIPHER_CREATE_SESSION \
       VIRTIO_CRYPTO_OPCODE(VIRTIO_CRYPTO_SERVICE_AKCIPHER, 0x04)
#define VIRTIO_CRYPTO_AKCIPHER_DESTROY_SESSION \
       VIRTIO_CRYPTO_OPCODE(VIRTIO_CRYPTO_SERVICE_AKCIPHER, 0x05)

#define VIRTIO_CRYPTO_CTRLQ_OP_SPEC_HDR_LEGACY              56

#define VIRTIO_CRYPTO_OP_ENCRYPT                            1
#define VIRTIO_CRYPTO_OP_DECRYPT                            2

#define VIRTIO_CRYPTO_SYM_ALG_CHAIN_ORDER_HASH_THEN_CIPHER  1
#define VIRTIO_CRYPTO_SYM_ALG_CHAIN_ORDER_CIPHER_THEN_HASH  2

/* Plain hash */

#define VIRTIO_CRYPTO_SYM_HASH_MODE_PLAIN                   1

/* Authenticated hash (mac) */

#define VIRTIO_CRYPTO_SYM_HASH_MODE_AUTH                    2

/* Nested hash */

#define VIRTIO_CRYPTO_SYM_HASH_MODE_NESTED                  3

#define VIRTIO_CRYPTO_ALG_CHAIN_SESS_OP_SPEC_HDR_SIZE       16

#define VIRTIO_CRYPTO_SYM_SESS_OP_SPEC_HDR_SIZE             48

/* No operation */

#define VIRTIO_CRYPTO_SYM_OP_NONE                           0

  /* Cipher only operation on the data */

#define VIRTIO_CRYPTO_SYM_OP_CIPHER                         1

/* Chain any cipher with any hash or mac operation. The order
 * depends on the value of alg_chain_order param
 */

#define VIRTIO_CRYPTO_SYM_OP_ALGORITHM_CHAINING             2

#define VIRTIO_CRYPTO_RSA_RAW_PADDING                       0
#define VIRTIO_CRYPTO_RSA_PKCS1_PADDING                     1

#define VIRTIO_CRYPTO_RSA_NO_HASH                           0
#define VIRTIO_CRYPTO_RSA_MD2                               1
#define VIRTIO_CRYPTO_RSA_MD3                               2
#define VIRTIO_CRYPTO_RSA_MD4                               3
#define VIRTIO_CRYPTO_RSA_MD5                               4
#define VIRTIO_CRYPTO_RSA_SHA1                              5
#define VIRTIO_CRYPTO_RSA_SHA256                            6
#define VIRTIO_CRYPTO_RSA_SHA384                            7
#define VIRTIO_CRYPTO_RSA_SHA512                            8
#define VIRTIO_CRYPTO_RSA_SHA224                            9

#define VIRTIO_CRYPTO_CURVE_UNKNOWN                         0
#define VIRTIO_CRYPTO_CURVE_NIST_P192                       1
#define VIRTIO_CRYPTO_CURVE_NIST_P224                       2
#define VIRTIO_CRYPTO_CURVE_NIST_P256                       3
#define VIRTIO_CRYPTO_CURVE_NIST_P384                       4
#define VIRTIO_CRYPTO_CURVE_NIST_P521                       5

#define VIRTIO_CRYPTO_AKCIPHER_KEY_TYPE_PUBLIC              1
#define VIRTIO_CRYPTO_AKCIPHER_KEY_TYPE_PRIVATE             2

#define VIRTIO_CRYPTO_AKCIPHER_SESS_ALGO_SPEC_HDR_SIZE      44

#define VIRTIO_CRYPTO_CIPHER_ENCRYPT \
  VIRTIO_CRYPTO_OPCODE(VIRTIO_CRYPTO_SERVICE_CIPHER, 0x00)
#define VIRTIO_CRYPTO_CIPHER_DECRYPT \
  VIRTIO_CRYPTO_OPCODE(VIRTIO_CRYPTO_SERVICE_CIPHER, 0x01)
#define VIRTIO_CRYPTO_HASH \
  VIRTIO_CRYPTO_OPCODE(VIRTIO_CRYPTO_SERVICE_HASH, 0x00)
#define VIRTIO_CRYPTO_MAC \
  VIRTIO_CRYPTO_OPCODE(VIRTIO_CRYPTO_SERVICE_MAC, 0x00)
#define VIRTIO_CRYPTO_AEAD_ENCRYPT \
  VIRTIO_CRYPTO_OPCODE(VIRTIO_CRYPTO_SERVICE_AEAD, 0x00)
#define VIRTIO_CRYPTO_AEAD_DECRYPT \
  VIRTIO_CRYPTO_OPCODE(VIRTIO_CRYPTO_SERVICE_AEAD, 0x01)
#define VIRTIO_CRYPTO_AKCIPHER_ENCRYPT \
  VIRTIO_CRYPTO_OPCODE(VIRTIO_CRYPTO_SERVICE_AKCIPHER, 0x00)
#define VIRTIO_CRYPTO_AKCIPHER_DECRYPT \
  VIRTIO_CRYPTO_OPCODE(VIRTIO_CRYPTO_SERVICE_AKCIPHER, 0x01)
#define VIRTIO_CRYPTO_AKCIPHER_SIGN \
  VIRTIO_CRYPTO_OPCODE(VIRTIO_CRYPTO_SERVICE_AKCIPHER, 0x02)
#define VIRTIO_CRYPTO_AKCIPHER_VERIFY \
  VIRTIO_CRYPTO_OPCODE(VIRTIO_CRYPTO_SERVICE_AKCIPHER, 0x03)

#define VIRTIO_CRYPTO_FLAG_SESSION_MODE                     1

#define VIRTIO_CRYPTO_DATAQ_OP_SPEC_HDR_LEGACY              48

#define VIRTIO_CRYPTO_SYM_DATA_REQ_HDR_SIZE                 40

#define VIRTIO_CRYPTO_SYM_DATE_REQ_HDR_STATELESS_SIZE       72

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct virtio_crypto_config_s
{
  uint32_t status;
  uint32_t max_dataqueues;
  uint32_t crypto_services;

  /* Detailed algorithms mask */

  uint32_t cipher_algo_l;
  uint32_t cipher_algo_h;
  uint32_t hash_algo;
  uint32_t mac_algo_l;
  uint32_t mac_algo_h;
  uint32_t aead_algo;

  /* Maximum length of cipher key in bytes */

  uint32_t max_cipher_key_len;

  /* Maximum length of authenticated key in bytes */

  uint32_t max_auth_key_len;
  uint32_t akcipher_algo;

  /* Maximum size of each crypto request’s content in bytes */

  uint64_t max_size;
} end_packed_struct;

enum VIRTIO_CRYPTO_STATUS
{
  VIRTIO_CRYPTO_OK = 0,
  VIRTIO_CRYPTO_ERR = 1,
  VIRTIO_CRYPTO_BADMSG = 2,
  VIRTIO_CRYPTO_NOTSUPP = 3,
  VIRTIO_CRYPTO_INVSESS = 4,
  VIRTIO_CRYPTO_NOSPC = 5,
  VIRTIO_CRYPTO_KEY_REJECTED = 6,
  VIRTIO_CRYPTO_MAX
};

begin_packed_struct struct virtio_crypto_ctrl_header_s
{
  uint32_t opcode;

  /* algo should be service-specific algorithms */

  uint32_t algo;
  uint32_t flag;
  uint32_t reserved;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_hash_create_session_flf_s
{
  /* Device read only portion */

  /* See VIRTIO_CRYPTO_HASH_* above */

  uint32_t algo;

  /* hash result length */

  uint32_t hash_result_len;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_mac_create_session_flf_s
{
  /* Device read only portion */

  /* See VIRTIO_CRYPTO_MAC_* above */

  uint32_t algo;

  /* hash result length */

  uint32_t hash_result_len;

  /* length of authenticated key */

  uint32_t auth_key_len;
  uint32_t padding;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_cipher_session_flf_s
{
  /* Device read only portion */

  /* See VIRTIO_CRYPTO_CIPHER* above */

  uint32_t algo;

  /* length of key */

  uint32_t key_len;

  /* encryption or decryption */

  uint32_t op;
  uint32_t padding;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_alg_chain_session_flf_s
{
  /* Device read only portion */

  uint32_t alg_chain_order;
  uint32_t hash_mode;
  struct virtio_crypto_cipher_session_flf_s cipher_hdr;

  /* fixed length fields, algo specific */

  union
    {
      struct virtio_crypto_hash_create_session_flf_s hash;
      struct virtio_crypto_mac_create_session_flf_s mac;
      uint8_t padding[VIRTIO_CRYPTO_ALG_CHAIN_SESS_OP_SPEC_HDR_SIZE];
    } algo_flf;

  /* length of the additional authenticated data (AAD) in bytes */

  uint32_t aad_len;
  uint32_t padding;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_sym_create_session_flf_s
{
  /* Device read only portion */

  union
    {
      struct virtio_crypto_cipher_session_flf_s cipher;
      struct virtio_crypto_alg_chain_session_flf_s chain;
      uint8_t padding[VIRTIO_CRYPTO_SYM_SESS_OP_SPEC_HDR_SIZE];
    } op_flf;

  uint32_t op_type;
  uint32_t padding;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_aead_create_session_flf_s
{
  /* Device read only portion */

  /* See VIRTIO_CRYPTO_AEAD_* above */

  uint32_t algo;

  /* length of key */

  uint32_t key_len;

  /* Authentication tag length */

  uint32_t tag_len;

  /* length of the additional authenticated data (AAD) in bytes */

  uint32_t aad_len;

  /* encrypt or decrypt, See above VIRTIO_CRYPTO_OP_* */

  uint32_t op;
  uint32_t padding;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_rsa_session_para_s
{
  uint32_t padding_algo;
  uint32_t hash_algo;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_ecdsa_session_para_s
{
  /* See VIRTIO_CRYPTO_CURVE_* above */

  uint32_t curve_id;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_akcipher_create_session_flf_s
{
  /* Device read only portion */

  /* See VIRTIO_CRYPTO_AKCIPHER_* above */

  uint32_t algo;
  uint32_t key_type;

  /* length of key */

  uint32_t key_len;

  union
    {
      struct virtio_crypto_rsa_session_para_s rsa;
      struct virtio_crypto_ecdsa_session_para_s ecdsa;
      uint8_t padding[VIRTIO_CRYPTO_AKCIPHER_SESS_ALGO_SPEC_HDR_SIZE];
    } algo_flf;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_create_session_input_s
{
  uint64_t session_id;
  uint32_t status;
  uint32_t padding;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_destroy_session_flf_s
{
  /* Device read only portion */

  uint64_t session_id;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_destroy_session_input_s
{
  /* Device write only portion */

  uint8_t status;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_op_ctrl_req_s
{
  /* Device read only portion */

  struct virtio_crypto_ctrl_header_s header;

  /* fixed length fields, opcode specific */

  union
    {
      struct virtio_crypto_sym_create_session_flf_s sym;
      struct virtio_crypto_hash_create_session_flf_s hash;
      struct virtio_crypto_mac_create_session_flf_s mac;
      struct virtio_crypto_aead_create_session_flf_s aead;
      struct virtio_crypto_akcipher_create_session_flf_s akcipher;
      struct virtio_crypto_destroy_session_flf_s destroy;
      uint8_t padding[VIRTIO_CRYPTO_CTRLQ_OP_SPEC_HDR_LEGACY];
    } op_flf;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_op_header_s
{
  uint32_t opcode;

  /* algo should be service-specific algorithms */

  uint32_t algo;

  /* session_id should be service-specific algorithms */

  uint64_t session_id;

  /* control flag to control the request */

  uint32_t flag;
  uint32_t padding;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_inhdr_s
{
  uint8_t status;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_hash_data_flf_s
{
  /* length of source data */

  uint32_t src_data_len;

  /* hash result length */

  uint32_t hash_result_len;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_hash_data_flf_stateless_s
{
  struct
    {
      /* See VIRTIO_CRYPTO_HASH_* above */

      uint32_t algo;
    } sess_para;

  /* length of source data */

  uint32_t src_data_len;

  /* hash result length */

  uint32_t hash_result_len;
  uint32_t reserved;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_mac_data_flf_s
{
  struct virtio_crypto_hash_data_flf_s hdr;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_mac_data_flf_stateless_s
{
  struct
    {
      /* See VIRTIO_CRYPTO_MAC_* above */

      uint32_t algo;

      /* length of authenticated key */

      uint32_t auth_key_len;
    } sess_para;

  /* length of source data */

  uint32_t src_data_len;

  /* hash result length */

  uint32_t hash_result_len;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_cipher_data_flf_s
{
  /* Byte Length of valid IV/Counter data pointed to by the below iv data.
   *
   * For block ciphers in CBC or F8 mode, or for Kasumi in F8 mode, or for
   *   SNOW3G in UEA2 mode, this is the length of the IV (which
   *   must be the same as the block length of the cipher).
   * For block ciphers in CTR mode, this is the length of the counter
   *   (which must be the same as the block length of the cipher).
   */

  uint32_t iv_len;

  /* length of source data */

  uint32_t src_data_len;

  /* length of dst data */

  uint32_t dst_data_len;
  uint32_t padding;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_alg_chain_data_flf_s
{
  uint32_t iv_len;

  /* Length of source data */

  uint32_t src_data_len;

  /* Length of destination data */

  uint32_t dst_data_len;

  /* Starting point for cipher processing in source data */

  uint32_t cipher_start_src_offset;

  /* Length of the source data that the cipher will be computed on */

  uint32_t len_to_cipher;

  /* Starting point for hash processing in source data */

  uint32_t hash_start_src_offset;

  /* Length of the source data that the hash will be computed on */

  uint32_t len_to_hash;

  /* Length of the additional auth data */

  uint32_t aad_len;

  /* Length of the hash result */

  uint32_t hash_result_len;
  uint32_t reserved;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_sym_data_flf_s
{
  /* Device read only portion */

  union
    {
      struct virtio_crypto_cipher_data_flf_s cipher;
      struct virtio_crypto_alg_chain_data_flf_s chain;
      uint8_t padding[VIRTIO_CRYPTO_SYM_DATA_REQ_HDR_SIZE];
    } op_type_flf;

  /* See above VIRTIO_CRYPTO_SYM_OP_* */

  uint32_t op_type;
  uint32_t padding;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_cipher_data_flf_stateless_s
{
  struct
    {
      /* See VIRTIO_CRYPTO_CIPHER* above */

      uint32_t algo;

      /* length of key */

      uint32_t key_len;

      /* See VIRTIO_CRYPTO_OP_* above */

      uint32_t op;
    } sess_para;

  /* Byte Length of valid IV/Counter data pointed to by the below iv data.
   */

  uint32_t iv_len;

  /* length of source data */

  uint32_t src_data_len;

  /* length of destination data */

  uint32_t dst_data_len;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_alg_chain_data_flf_stateless_s
{
  struct
    {
      /* See VIRTIO_CRYPTO_SYM_ALG_CHAIN_ORDER_* above */

      uint32_t alg_chain_order;

      /* length of the additional authenticated data in bytes */

      uint32_t aad_len;

      struct
        {
          /* See VIRTIO_CRYPTO_CIPHER* above */

          uint32_t algo;

          /* length of key */

          uint32_t key_len;

          /* See VIRTIO_CRYPTO_OP_* above */

          uint32_t op;
        } cipher;

      struct
        {
          /* See VIRTIO_CRYPTO_HASH_* or VIRTIO_CRYPTO_MAC_* above */

          uint32_t algo;

          /* length of authenticated key */

          uint32_t auth_key_len;

          /* See VIRTIO_CRYPTO_SYM_HASH_MODE_* above */

          uint32_t hash_mode;
        } hash;
    } sess_para;

  uint32_t iv_len;

  /* Length of source data */

  uint32_t src_data_len;

  /* Length of destination data */

  uint32_t dst_data_len;

  /* Starting point for cipher processing in source data */

  uint32_t cipher_start_src_offset;

  /* Length of the source data that the cipher will be computed on */

  uint32_t len_to_cipher;

  /* Starting point for hash processing in source data */

  uint32_t hash_start_src_offset;

  /* Length of the source data that the hash will be computed on */

  uint32_t len_to_hash;

  /* Length of the additional auth data */

  uint32_t aad_len;

  /* Length of the hash result */

  uint32_t hash_result_len;
  uint32_t reserved;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_sym_data_flf_stateless_s
{
  /* Device read only portion */

  union
    {
      struct virtio_crypto_cipher_data_flf_stateless_s cipher;
      struct virtio_crypto_alg_chain_data_flf_stateless_s chain;
      uint8_t padding[VIRTIO_CRYPTO_SYM_DATE_REQ_HDR_STATELESS_SIZE];
    } op_type_flf;

  /* Device write only portion */

  /* See above VIRTIO_CRYPTO_SYM_OP_* */

  uint32_t op_type;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_aead_data_flf_s
{
  /* Byte Length of valid IV data.
   *
   * For GCM mode, this is either 12 (for 96-bit IVs) or 16, in which
   *   case iv points to J0.
   * For CCM mode, this is the length of the nonce, which can be in the
   *   range 7 to 13 inclusive.
   */

  uint32_t iv_len;

  /* length of additional auth data */

  uint32_t aad_len;

  /* length of source data */

  uint32_t src_data_len;

  /* length of dst data */

  uint32_t dst_data_len;

  /* Authentication tag length */

  uint32_t tag_len;
  uint32_t reserved;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_aead_data_flf_stateless_s
{
  struct
    {
      /* See VIRTIO_CRYPTO_AEAD_* above */

      uint32_t algo;

      /* length of key */

      uint32_t key_len;

      /* encrypt or decrypt, See above VIRTIO_CRYPTO_OP_* */

      uint32_t op;
    } sess_para;

  /* Byte Length of valid IV data. */

  uint32_t iv_len;

  /* Authentication tag length */

  uint32_t tag_len;

  /* length of additional auth data */

  uint32_t aad_len;

  /* length of source data */

  uint32_t src_data_len;

  /* length of dst data, this should be at least src_data_len + tag_len */

  uint32_t dst_data_len;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_akcipher_data_flf_s
{
  /* length of source data */

  uint32_t src_data_len;

  /* length of dst data */

  uint32_t dst_data_len;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_akcipher_data_flf_stateless_s
{
  struct
    {
      /* See VIRTIO_CYRPTO_AKCIPHER* above */

      uint32_t algo;

      /* See VIRTIO_CRYPTO_AKCIPHER_KEY_TYPE_* above */

      uint32_t key_type;

      /* length of key */

      uint32_t key_len;

      /* algothrim specific parameters described above */

      union
        {
          struct virtio_crypto_rsa_session_para_s rsa;
          struct virtio_crypto_ecdsa_session_para_s ecdsa;
        } u;
    } sess_para;

  /* length of source data */

  uint32_t src_data_len;

  /* length of destination data */

  uint32_t dst_data_len;
} end_packed_struct;

begin_packed_struct struct virtio_crypto_op_data_req_s
{
  /* Device read only portion */

  struct virtio_crypto_op_header_s header;

  union
    {
      struct virtio_crypto_sym_data_flf_s sym;
      struct virtio_crypto_hash_data_flf_s hash;
      struct virtio_crypto_mac_data_flf_s mac;
      struct virtio_crypto_aead_data_flf_s aead;
      struct virtio_crypto_akcipher_data_flf_s akcipher;
      uint8_t padding[VIRTIO_CRYPTO_DATAQ_OP_SPEC_HDR_LEGACY];
    } op_flf;
} end_packed_struct;

#endif /* __INCLUDE_NUTTX_VIRTIO_VIRTIO_CRYPTO_H */
