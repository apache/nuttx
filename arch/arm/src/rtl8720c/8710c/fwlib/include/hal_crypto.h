/**************************************************************************//**
 * @file      hal_crypto.h
 * @brief     The HAL API implementation for the CRYPTO device.
 * @version   V1.00
 * @date      2019-08-28
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License. *
 *
 ******************************************************************************/

#ifndef __HAL_CRYPTO_H__
#define __HAL_CRYPTO_H__


#include <stddef.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

#include "basic_types.h"
#include "cmsis.h"
#include <arm_cmse.h>   /* Use CMSE intrinsics */

/**
 * @addtogroup hs_hal_crypto CRYPTO
 * @{
 */

static const unsigned char md5_null_msg_result[1][16] = {

    { 0xD4, 0x1D, 0x8C, 0xD9, 0x8F, 0x00, 0xB2, 0x04,
      0xE9, 0x80, 0x09, 0x98, 0xEC, 0xF8, 0x42, 0x7E }
};

static const unsigned char sha1_null_msg_result[1][20] = {

    { 0xDA, 0x39, 0xA3, 0xEE, 0x5E, 0x6B, 0x4B, 0x0D,
      0x32, 0x55, 0xBF, 0xEF, 0x95, 0x60, 0x18, 0x90,
      0xAF, 0xD8, 0x07, 0x09
    }
};

static const unsigned char sha2_224_null_msg_result[1][28] = {

    { 0xD1, 0x4A, 0x02, 0x8C, 0x2A, 0x3A, 0x2B, 0xC9,
      0x47, 0x61, 0x02, 0xBB, 0x28, 0x82, 0x34, 0xC4,
      0x15, 0xA2, 0xB0, 0x1F, 0x82, 0x8E, 0xA6, 0x2A,
      0xC5, 0xB3, 0xE4, 0x2F }
};

static const unsigned char sha2_256_null_msg_result[1][32] = {

    { 0xE3, 0xB0, 0xC4, 0x42, 0x98, 0xFC, 0x1C, 0x14,
      0x9A, 0xFB, 0xF4, 0xC8, 0x99, 0x6F, 0xB9, 0x24,
      0x27, 0xAE, 0x41, 0xE4, 0x64, 0x9B, 0x93, 0x4C,
      0xA4, 0x95, 0x99, 0x1B, 0x78, 0x52, 0xB8, 0x55  }
};

#ifndef CRYPTO_MAX_MSG_LENGTH
//#define CRYPTO_MAX_MSG_LENGTH		18432 // 18432  // 4096 //32768 // 64 // 16383
#define CRYPTO_MAX_MSG_LENGTH		65536 // 64k bytes
#endif

#ifndef CRC_MAX_MSG_LENGTH
#define CRC_MAX_MSG_LENGTH		    65535 // ((2^16) - 1) bytes
#endif


#ifndef CRYPTO_MAX_AAD_LENGTH
#define CRYPTO_MAX_AAD_LENGTH	    496  // 16*((2^5)-1)
#endif

#ifndef CRYPTO_MAX_DIGEST_LENGTH
#define CRYPTO_MAX_DIGEST_LENGTH	32  // SHA256 Digest length : 32
#endif

//
// IV length
//
#define CRYPTO_DES_IV_LENGTH        8
#define CRYPTO_AES_IV_LENGTH        16
#define CRYPTO_CHACHA_MSG_ALIGN     16


//
// Key length
//
#define CRYPTO_DES_KEY_LENGTH        8
#define CRYPTO_3DES_KEY_LENGTH       24
#define CRYPTO_AES128_KEY_LENGTH     16
#define CRYPTO_AES192_KEY_LENGTH     24
#define CRYPTO_AES256_KEY_LENGTH     32

//
// Error index
//
#define _ERRNO_CRYPTO_DESC_NUM_SET_OutRange 		            -2
#define _ERRNO_CRYPTO_BURST_NUM_SET_OutRange		            -3
/// Pointer which points to NULL Error.
#define _ERRNO_CRYPTO_NULL_POINTER					            -4
/// Crypto doesn't initialize Error.
#define _ERRNO_CRYPTO_ENGINE_NOT_INIT				            -5
/// The starting address is not 32 byte-aligned Error.
#define _ERRNO_CRYPTO_ADDR_NOT_32Byte_Aligned		            -6
/// Keylen out of range Error.
#define _ERRNO_CRYPTO_KEY_OutRange					            -7
/// Msglen out of range Error.
#define _ERRNO_CRYPTO_MSG_OutRange					            -8
/// Ivlen out of range Error.
#define _ERRNO_CRYPTO_IV_OutRange					            -9
/// AADlen out of range Error.
#define _ERRNO_CRYPTO_AAD_OutRange					            -10
#define _ERRNO_CRYPTO_AUTH_TYPE_NOT_MATCH			            -11
#define _ERRNO_CRYPTO_CIPHER_TYPE_NOT_MATCH 		            -12
#define _ERRNO_CRYPTO_KEY_IV_LEN_DIFF				            -13
/// AES Msglen is not 16 byte-aligned Error.
#define _ERRNO_CRYPTO_AES_MSGLEN_NOT_16Byte_Aligned	            -14
/// Chahca Msglen is not 16 byte-aligned Error.
#define _ERRNO_CRYPTO_CHACHA_MSGLEN_NOT_16Byte_Aligned	        -15
/// DES Msglen is not 8 byte-aligned Error.
#define _ERRNO_CRYPTO_DES_MSGLEN_NOT_8Byte_Aligned	            -16
#define _ERRNO_CRYPTO_HASH_FINAL_NO_UPDATE		                -17
#define _ERRNO_CRYPTO_HASH_SEQUENTIAL_HASH_WORNG_LENGTH         -18
#define _ERRNO_CRYPTO_CACHE_HANDLE			                    -19
#define _ERRNO_CRYPTO_CIPHER_DECRYPT_MSGLEN_NOT_8Byte_Aligned	-20
#define _ERRNO_CRYPTO_MIX_MODE_HASH_PAD_NULL_POINTER            -21
#define _ERRNO_CRYPTO_MIX_MODE_TAG_NULL_POINTER					-22
#define _ERRNO_CRYPTO_MIX_MODE_ENC_PAD_NULL_POINTER             -23


#if defined(CONFIG_BUILD_SECURE)
/**
 *  @brief      Non-secure callable function to enable/disable the clock of IPsec in RAM code.
 *  @param[in]  en  set the clock of IPsec state: 1=Enable, 0=Disable 
 *  @return     void
 */
void NS_ENTRY hal_crypto_engine_init_platform_nsc(const int en);
#elif defined(CONFIG_BUILD_NONSECURE)
void hal_crypto_engine_init_platform_nsc(const int en);
#endif


/**
 *  @brief To initialize the CRYPTO adapter.\n
 *         This function must be called before any CRYPTO operation.\n
 *         This function will do:
 *           - enable the CRYPTO hardware(Related clock,Crypto engine,Endian setting,DMA arbiter)
 *           - register the clean/invalidate D-cache functions.
 *           - register the interrupt handler.
 *           - enable interrupt and init value in Crypto adapter.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_engine_init(void);

/**
 *  @brief Deinitialize the CRYPTO adapter.\n
 *         It will do:
 *           - disable CRYPTO hardware function.
 *           - deregister the clean/invalidate D-cache functions.
 *           - enable CRYPTO interrupt mask.
 *           - disable interrupt and init value in Crypto adapter.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_engine_deinit(void);

//
// Authentication
//

// MD5
/**
 *  @brief MD5 message digest algorithm (hash function).
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[out] pDigest  the result of MD5 function
 *
 *  @note   Some details need to know before setting:
 *          - MD5 will generate 16bytes(128 bits) digest.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *          - The maximum input buffer length can't be over 64k bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_md5(IN const u8 *message, IN const u32 msglen, OUT u8 *pDigest);

/**
 *  @brief Initializes the MD5 function.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_md5_init(void);

/**
 *  @brief MD5 process buffer.
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[out] pDigest  the result of MD5 function
 *
 *  @note   Some details need to know before setting:
 *          - MD5 will generate 16bytes(128 bits) digest.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *          - The maximum input buffer length can't be over 64k bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO))
 */
int hal_crypto_md5_process(
    IN const u8 *message, IN const u32 msglen,
    OUT u8 *pDigest);

/**
 *  @brief Update MD5 with new buffer(Sequential hash to process buffer).\n
 *         If a buffer size is too large for rtl_crypto_md5_process() to process, then split the buffer into
 *         many fixed size buffers and the rest of buffer which size may be less than or equal to the fixed size.
 *         After that, repeated calls update() are equivalent to a single call process().
 *
 *  @note  Some details need to know before setting:
 *         - The rtl_crypto_md5_update() method can be called multiple times with new buffer
 *           until rtl_crypto_md5_final() is calles.
 *           Calling rtl_crypto_md5_update() after rtl_crypto_md5_final() will result in an error return.
 *         - The starting address of input buffer doesn't need to be 32 byte-aligned.
 *         - The maximum input buffer length can't be over 64k bytes.
 *
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_md5_update(IN const u8 *message, IN const u32 msglen);

/**
 *  @brief Get MD5 sequential hash final result.\n
 *         Return the digest of the buffers passed to the update() method so far.
 *  @param[out] pDigest  the result of MD5 function
 *
 *  @note   Some details need to know before setting:
 *          - MD5 will generate 16bytes(128 bits) digest.
 *          - The starting address of the result doesn't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_md5_final(OUT u8 *pDigest);

// SHA1
/**
 *  @brief SHA1 message digest algorithm (hash function).
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[out] pDigest  the result of SHA1 function
 *
 *  @note   Some details need to know before setting:
 *          - SHA1 will generate 20bytes(160 bits) digest.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *          - The maximum input buffer length can't be over 64k bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_sha1(IN const u8 *message, IN const u32 msglen, OUT u8 *pDigest);

/**
 *  @brief Initializes the SHA1 function.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_sha1_init(void);

/**
 *  @brief SHA1 process buffer.
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[out] pDigest  the result of SHA1 function
 *
 *  @note   Some details need to know before setting:
 *          - SHA1 will generate 20bytes(160 bits) digest.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *          - The maximum input buffer length can't be over 64k bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_sha1_process(
    IN const u8 *message, IN const u32 msglen,
    OUT u8 *pDigest);

/**
 *  @brief Update SHA1 with new buffer(Sequential hash to process buffer).\n
 *         If a buffer size is too large for rtl_crypto_sha1_process() to process, then split the buffer into
 *         many fixed size buffers and the rest of buffer which size may be less than or equal to the fixed size.
 *         After that, repeated calls update() are equivalent to a single call process().
 *
 *  @note  Some details need to know before setting:
 *         - The rtl_crypto_sha1_update() method can be called multiple times with new buffer
 *           until rtl_crypto_sha1_final() is calles.
 *           Calling rtl_crypto_sha1_update() after rtl_crypto_sha1_final() will result in an error return.
 *         - The starting address of input buffer doesn't need to be 32 byte-aligned.
 *         - The maximum input buffer length can't be over 64k bytes.
 *
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_sha1_update(IN const u8 *message, IN const u32 msglen);

/**
 *  @brief Get SHA1 sequential hash final result.\n
 *         Return the digest of the buffers passed to the update() method so far.
 *  @param[out] pDigest  the result of SHA1 function
 *
 *  @note   Some details need to know before setting:
 *          - SHA1 will generate 20bytes(160 bits) digest.
 *          - The starting address of the result doesn't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_sha1_final(OUT u8 *pDigest);

// SHA2-224
/**
 *  @brief SHA2_224 message digest algorithm (hash function).
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[out] pDigest  the result of SHA2_224 function
 *
 *  @note   Some details need to know before setting:
 *          - SHA2-224 will generate 28bytes(224 bits) digest.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *          - The maximum input buffer length can't be over 64k bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_sha2_224(IN const u8 *message, IN const u32 msglen, OUT u8 *pDigest);

/**
 *  @brief Initializes the SHA2_224 function.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_sha2_224_init(void);

/**
 *  @brief SHA2_224 process buffer.
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[out] pDigest  the result of SHA2_224 function
 *
 *  @note   Some details need to know before setting:
 *          - SHA2_224 will generate 28bytes(224 bits) digest.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *          - The maximum input buffer length can't be over 64k bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_sha2_224_process(
    IN const u8 *message, IN const u32 msglen,
    OUT u8 *pDigest);

/**
 *  @brief Update SHA2_224 with new buffer(Sequential hash to process buffer).\n
 *         If a buffer size is too large for rtl_crypto_sha2_224_process() to process, then split the buffer into
 *         many fixed size buffers and the rest of buffer which size may be less than or equal to the fixed size.
 *         After that, repeated calls update() are equivalent to a single call process().
 *
 *  @note  Some details need to know before setting:
 *         - The rtl_crypto_sha2_224_update() method can be called multiple times with new buffer
 *           until rtl_crypto_sha2_224_final() is calles.
 *           Calling rtl_crypto_sha2_224_update() after rtl_crypto_sha2_224_final() will result in an error return.
 *         - The starting address of input buffer doesn't need to be 32 byte-aligned.
 *         - The maximum input buffer length can't be over 64k bytes.
 *
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_sha2_224_update(IN const u8 *message, IN const u32 msglen);

/**
 *  @brief Get SHA2_224 sequential hash final result.\n
 *         Return the digest of the buffers passed to the update() method so far.
 *  @param[out] pDigest  the result of SHA2_224 function
 *
 *  @note   Some details need to know before setting:
 *          - SHA2_224 will generate 28bytes(224 bits) digest.
 *          - The starting address of the result doesn't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_sha2_224_final(OUT u8 *pDigest);

// SHA2-256
/**
 *  @brief SHA2_256 message digest algorithm (hash function).
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[out] pDigest  the result of SHA2_256 function
 *
 *  @note   Some details need to know before setting:
 *          - SHA2_256 will generate 32bytes(256 bits) digest.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *          - The maximum input buffer length can't be over 64k bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_sha2_256(IN const u8 *message, IN const u32 msglen, OUT u8 *pDigest);

/**
 *  @brief Initializes the SHA2_256 function.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_sha2_256_init(void);

/**
 *  @brief SHA2_256 process buffer.
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[out] pDigest  the result of SHA2_256 function
 *
 *  @note   Some details need to know before setting:
 *          - SHA2_256 will generate 32bytes(256 bits) digest.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *          - The maximum input buffer length can't be over 64k bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_sha2_256_process(
    IN const u8 *message, IN const u32 msglen,
    OUT u8 *pDigest);

/**
 *  @brief Update SHA2_256 with new buffer(Sequential hash to process buffer).\n
 *         If a buffer size is too large for rtl_crypto_sha2_256_process() to process, then split the buffer into
 *         many fixed size buffers and the rest of buffer which size may be less than or equal to the fixed size.
 *         After that, repeated calls update() are equivalent to a single call process().
 *
 *  @note  Some details need to know before setting:
 *         - The rtl_crypto_sha2_256_update() method can be called multiple times with new buffer
 *           until rtl_crypto_sha2_256_final() is calles.
 *           Calling rtl_crypto_sha2_256_update() after rtl_crypto_sha2_256_final() will result in an error return.
 *         - The starting address of input buffer doesn't need to be 32 byte-aligned.
 *         - The maximum input buffer length can't be over 64k bytes.
 *
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_sha2_256_update(IN const u8 *message, IN const u32 msglen);

/**
 *  @brief Get SHA2_256 sequential hash final result.\n
 *         Return the digest of the buffers passed to the update() method so far.
 *  @param[out] pDigest  the result of SHA2_256 function
 *
 *  @note   Some details need to know before setting:
 *          - SHA2_256 will generate 32bytes(256 bits) digest.
 *          - The starting address of the result doesn't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_sha2_256_final(OUT u8 *pDigest);

// HMAC-md5
/**
 *  @brief HMAC-MD5 message digest algorithm (hash function).
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[in]  key      HMAC secret key
 *  @param[in]  keylen   length of the HMAC key
 *  @param[out] pDigest  the result of HMAC-MD5 function
 *
 *  @note   Some details need to know before setting:
 *          - MD5 will generate 16bytes(128 bits) digest.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - The starting address of key need to be 32 byte-aligned.
 *          - The maximum HMAC key length can't be over 64 bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_hmac_md5(IN const u8 *message, IN const u32 msglen,
			IN const u8 *key, IN const u32 keylen, OUT u8 *pDigest);

/**
 *  @brief Initializes the HMAC-MD5 function.
 *  @param[in]  key      HMAC secret key
 *  @param[in]  keylen   length of the HMAC key
 *
 *  @note   Some details need to know before setting:
 *          - The starting address of key need to be 32 byte-aligned.
 *          - The maximum HMAC key length can't be over 64 bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_hmac_md5_init(IN const u8 *key, IN const u32 keylen);

/**
 *  @brief HMAC-MD5 process buffer.
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[out] pDigest  the result of HMAC-MD5 function
 *
 *  @note   Some details need to know before setting:
 *          - MD5 will generate 16bytes(128 bits) digest.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *          - The maximum input buffer length can't be over 64k bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_hmac_md5_process(IN const u8 *message, IN const u32 msglen, OUT u8 *pDigest);

/**
 *  @brief Update HMAC-MD5 with new buffer(Sequential hash to process buffer).\n
 *         If a buffer size is too large for rtl_crypto_hmac_md5_process() to process, then split the buffer into
 *         many fixed size buffers and the rest of buffer which size may be less than or equal to the fixed size.
 *         After that, repeated calls update() are equivalent to a single call process().
 *
 *  @note  Some details need to know before setting:
 *         - The rtl_crypto_hmac_md5_update() method can be called multiple times with new buffer
 *           until rtl_crypto_hmac_md5_final() is calles.
 *           Calling rtl_crypto_hmac_md5_update() after rtl_crypto_hmac_md5_final() will result in an error return.
 *         - The starting address of input buffer doesn't need to be 32 byte-aligned.
 *         - The maximum input buffer length can't be over 64k bytes.
 *
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_hmac_md5_update(IN const u8 *message, IN const u32 msglen);

/**
 *  @brief Get HMAC-MD5 sequential hash final result.\n
 *         Return the digest of the buffers passed to the update() method so far.
 *  @param[out] pDigest  the result of HMAC-MD5 function
 *
 *  @note   Some details need to know before setting:
 *          - MD5 will generate 16bytes(128 bits) digest.
 *          - The starting address of the result doesn't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_hmac_md5_final(OUT u8 *pDigest);

// HMAC-sha1
/**
 *  @brief HMAC-SHA1 message digest algorithm (hash function).
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[in]  key      HMAC secret key
 *  @param[in]  keylen   length of the HMAC key
 *  @param[out] pDigest  the result of HMAC-SHA1 function
 *
 *  @note   Some details need to know before setting:
 *          - SHA1 will generate 20bytes(160 bits) digest.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - The starting address of key need to be 32 byte-aligned.
 *          - The maximum HMAC key length can't be over 64 bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_hmac_sha1(IN const u8 *message, IN const u32 msglen,
			IN const u8 *key, IN const u32 keylen, OUT u8 *pDigest);

/**
 *  @brief Initializes the HMAC-SHA1 function.
 *  @param[in]  key      HMAC secret key
 *  @param[in]  keylen   length of the HMAC key
 *
 *  @note   Some details need to know before setting:
 *          - The starting address of key need to be 32 byte-aligned.
 *          - The maximum HMAC key length can't be over 64 bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO))
 */
int hal_crypto_hmac_sha1_init(IN const u8 *key, IN const u32 keylen);

/**
 *  @brief HMAC-SHA1 process buffer.
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[out] pDigest  the result of HMAC-SHA1 function
 *
 *  @note   Some details need to know before setting:
 *          - SHA1 will generate 20bytes(160 bits) digest.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *          - The maximum input buffer length can't be over 64k bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_hmac_sha1_process(IN const u8 *message, IN const u32 msglen, OUT u8 *pDigest);

/**
 *  @brief Update HMAC-SHA1 with new buffer(Sequential hash to process buffer).\n
 *         If a buffer size is too large for rtl_crypto_hmac_sha1_process() to process, then split the buffer into
 *         many fixed size buffers and the rest of buffer which size may be less than or equal to the fixed size.
 *         After that, repeated calls update() are equivalent to a single call process().
 *
 *  @note  Some details need to know before setting:
 *         - The rtl_crypto_hmac_sha1_update() method can be called multiple times with new buffer
 *           until rtl_crypto_hmac_sha1_final() is calles.
 *           Calling rtl_crypto_hmac_sha1_update() after rtl_crypto_hmac_sha1_final() will result in an error return.
 *         - The starting address of input buffer doesn't need to be 32 byte-aligned.
 *         - The maximum input buffer length can't be over 64k bytes.
 *
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_hmac_sha1_update(IN const u8 *message, IN const u32 msglen);

/**
 *  @brief Get HMAC-SHA1 sequential hash final result.\n
 *         Return the digest of the buffers passed to the update() method so far.
 *  @param[out] pDigest  the result of HMAC-SHA1 function
 *
 *  @note   Some details need to know before setting:
 *          - SHA1 will generate 20bytes(160 bits) digest.
 *          - The starting address of the result doesn't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_hmac_sha1_final(OUT u8 *pDigest);

// HMAC-sha2
// -- 224
/**
 *  @brief HMAC-SHA2-224 message digest algorithm (hash function).
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[in]  key      HMAC secret key
 *  @param[in]  keylen   length of the HMAC key
 *  @param[out] pDigest  the result of HMAC-SHA2-224 function
 *
 *  @note   Some details need to know before setting:
 *          - SHA2_224 will generate 28bytes(224 bits) digest.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - The starting address of key need to be 32 byte-aligned.
 *          - The maximum HMAC key length can't be over 64 bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_hmac_sha2_224( IN const u8 *message, IN const u32 msglen,
            IN const u8 *key, IN const u32 keylen, OUT u8 *pDigest);

/**
 *  @brief Initializes the HMAC-SHA2-224 function.
 *  @param[in]  key      HMAC secret key
 *  @param[in]  keylen   length of the HMAC key
 *
 *  @note   Some details need to know before setting:
 *          - The starting address of key need to be 32 byte-aligned.
 *          - The maximum HMAC key length can't be over 64 bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_hmac_sha2_224_init(IN const u8 *key, IN const u32 keylen);

/**
 *  @brief HMAC-SHA2-224 process buffer.
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[out] pDigest  the result of HMAC-SHA2-224 function
 *
 *  @note   Some details need to know before setting:
 *          - SHA2_224 will generate 28bytes(224 bits) digest.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *          - The maximum input buffer length can't be over 64k bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_hmac_sha2_224_process(IN const u8 *message, IN const u32 msglen, OUT u8 *pDigest);

/**
 *  @brief Update HMAC-SHA2-224 with new buffer(Sequential hash to process buffer).\n
 *         If a buffer size is too large for rtl_crypto_hmac_sha2_224_process() to process, then split the buffer into
 *         many fixed size buffers and the rest of buffer which size may be less than or equal to the fixed size.
 *         After that, repeated calls update() are equivalent to a single call process().
 *
 *  @note  Some details need to know before setting:
 *         - The rtl_crypto_hmac_sha2_224_update() method can be called multiple times with new buffer
 *           until rtl_crypto_hmac_sha2_224_final() is calles. Calling rtl_crypto_hmac_sha2_224_update()
 *           after rtl_crypto_hmac_sha2_224_final() will result in an error return.
 *         - The starting address of input buffer doesn't need to be 32 byte-aligned.
 *         - The maximum input buffer length can't be over 64k bytes.
 *
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_hmac_sha2_224_update(IN const u8 *message, IN const u32 msglen);

/**
 *  @brief Get HMAC-SHA2-224 sequential hash final result.\n
 *         Return the digest of the buffers passed to the update() method so far.
 *  @param[out] pDigest  the result of HMAC-SHA2-224 function
 *
 *  @note   Some details need to know before setting:
 *          - SHA2_224 will generate 28bytes(224 bits) digest.
 *          - The starting address of the result doesn't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_hmac_sha2_224_final(OUT u8 *pDigest);

// -- 256
/**
 *  @brief HMAC-SHA2-256 message digest algorithm (hash function).
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[in]  key      HMAC secret key
 *  @param[in]  keylen   length of the HMAC key
 *  @param[out] pDigest  the result of HMAC-SHA2-256 function
 *
 *  @note   Some details need to know before setting:
 *          - SHA2_256 will generate 32bytes(256 bits) digest.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - The starting address of key need to be 32 byte-aligned.
 *          - The maximum HMAC key length can't be over 64 bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_hmac_sha2_256( IN const u8 *message, IN const u32 msglen,
            IN const u8 *key, IN const u32 keylen, OUT u8 *pDigest);

/**
 *  @brief Initializes the HMAC-SHA2-256 function.
 *  @param[in]  key      HMAC secret key
 *  @param[in]  keylen   length of the HMAC key
 *
 *  @note   Some details need to know before setting:
 *          - The starting address of key need to be 32 byte-aligned.
 *          - The maximum HMAC key length can't be over 64 bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO))
 */
int hal_crypto_hmac_sha2_256_init(IN const u8 *key, IN const u32 keylen);

/**
 *  @brief HMAC-SHA2-256 process buffer.
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[out] pDigest  the result of HMAC-SHA2-256 function
 *
 *  @note   Some details need to know before setting:
 *          - SHA2_256 will generate 32bytes(256 bits) digest.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *          - The maximum input buffer length can't be over 64k bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_hmac_sha2_256_process(IN const u8 *message, IN const u32 msglen, OUT u8 *pDigest);

/**
 *  @brief Update HMAC-SHA2-256 with new buffer(Sequential hash to process buffer).\n
 *         If a buffer size is too large for rtl_crypto_hmac_sha2_256_process() to process, then split the buffer into
 *         many fixed size buffers and the rest of buffer which size may be less than or equal to the fixed size.
 *         After that, repeated calls update() are equivalent to a single call process().
 *
 *  @note  Some details need to know before setting:
 *         - The rtl_crypto_hmac_sha2_256_update() method can be called multiple times with new buffer
 *           until rtl_crypto_hmac_sha2_256_final() is calles. Calling rtl_crypto_hmac_sha2_256_update()
 *           after rtl_crypto_hmac_sha2_256_final() will result in an error return.
 *         - The starting address of input buffer doesn't need to be 32 byte-aligned.
 *         - The maximum input buffer length can't be over 64k bytes.
 *
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_hmac_sha2_256_update(IN const u8 *message, IN const u32 msglen);

/**
 *  @brief Get HMAC-SHA2-256 sequential hash final result.\n
 *         Return the digest of the buffers passed to the update() method so far.
 *  @param[out] pDigest  the result of HMAC-SHA2-256 function
 *
 *  @note   Some details need to know before setting:
 *          - SHA2_256 will generate 32bytes(256 bits) digest.
 *          - The starting address of the result doesn't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_hmac_sha2_256_final(OUT u8 *pDigest);

// AES-CBC
/**
 *  @brief Initializes the AES-CBC function with a secret key.
 *  @param[in]  key  secret key
 *  @param[in]  keylen   length of the key
 *
 *  @note   Some details need to know before setting:
 *          - AES keylen must be 16bytes(128bits), 24bytes(192bits) or 32bytes(256bits).
 *          - The starting address of key can't be NULL and need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_cbc_init(IN const u8 *key, IN const u32 keylen);

/**
 *  @brief AES-CBC buffer encryption.
 *  @param[in]  message  input buffer(Plaintext)
 *  @param[in]  msglen   input buffer length
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[in]  ivlen    length of the initial vector
 *  @param[out] pResult  the result of AES-CBC encrypt function(Ciphertext)
 *
 *  @note   Some details need to know before setting:
 *          - Usually ivlen is 16 bytes in AES.
 *          - The starting address of iv can't be NULL and need to be 32 byte-aligned.
 *          - The msglen must be 16 byte-aligned in AES.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - Encryption handles msglen bytes of plaintext, then it will generate the same size of ciphertext.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_cbc_encrypt(
    IN const u8 *message, 	IN const u32 msglen,
    IN const u8 *iv, 		IN const u32 ivlen,
    OUT u8 *pResult);

/**
 *  @brief AES-CBC buffer decryption.
 *  @param[in]  message  input buffer(Ciphertext)
 *  @param[in]  msglen   input buffer length
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[in]  ivlen    length of the initial vector
 *  @param[out] pResult  the result of AES-CBC decrypt function(Plaintext)
 *
 *  @note   Some details need to know before setting:
 *          - Usually ivlen is 16 bytes in AES.
 *          - The starting address of iv can't be NULL and need to be 32 byte-aligned.
 *          - Usually decrypted IV is the same as encrypted IV.
 *          - The msglen must be 16 byte-aligned in AES.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - Decryption handles msglen bytes of ciphertext, then it will generate the same size of plaintext.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_cbc_decrypt(
    IN const u8 *message, 	IN const u32 msglen,
    IN const u8 *iv, 		IN const u32 ivlen,
    OUT u8 *pResult);

// AES-ECB
/**
 *  @brief Initializes the AES-ECB function with a secret key.
 *  @param[in]  key  secret key
 *  @param[in]  keylen   length of the key
 *
 *  @note   Some details need to know before setting:
 *          - AES keylen must be 16bytes(128bits), 24bytes(192bits) or 32bytes(256bits).
 *          - The starting address of key can't be NULL and need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_ecb_init(IN const u8 *key, IN const u32 keylen);

/**
 *  @brief AES-ECB buffer encryption.
 *  @param[in]  message  input buffer(Plaintext)
 *  @param[in]  msglen   input buffer length
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[in]  ivlen    length of the initial vector
 *  @param[out] pResult  the result of AES-ECB encrypt function(Ciphertext)
 *
 *  @note   Some details need to know before setting:
 *          - Usually ECB mode doesn't need IV, so assign NULL to iv and ivlen must be 0.
 *          - The msglen must be 16 byte-aligned in AES.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - Encryption handles msglen bytes of plaintext, then it will generate the same size of ciphertext.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_ecb_encrypt(
    IN const u8 *message, 	IN const u32 msglen,
    IN const u8 *iv, 		IN const u32 ivlen,
    OUT u8 *pResult);

/**
 *  @brief AES-ECB buffer decryption.
 *  @param[in]  message  input buffer(Ciphertext)
 *  @param[in]  msglen   input buffer length
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[in]  ivlen    length of the initial vector
 *  @param[out] pResult  the result of AES-ECB decrypt function(Plaintext)
 *
 *  @note   Some details need to know before setting:
 *          - Usually ECB mode doesn't need IV, so assign NULL to iv and ivlen must be 0.
 *          - The msglen must be 16 byte-aligned in AES.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - Decryption handles msglen bytes of ciphertext, then it will generate the same size of plaintext.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_ecb_decrypt(
    IN const u8 *message, 	IN const u32 msglen,
    IN const u8 *iv, 		IN const u32 ivlen,
    OUT u8 *pResult);

// AES-CTR
/**
 *  @brief Initializes the AES-CTR function with a secret key.
 *  @param[in]  key  secret key
 *  @param[in]  keylen   length of the key
 *
 *  @note   Some details need to know before setting:
 *          - AES keylen must be 16bytes(128bits), 24bytes(192bits) or 32bytes(256bits).
 *          - The starting address of key can't be NULL and need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_ctr_init(IN const u8 *key, IN const u32 keylen);

/**
 *  @brief AES-CTR buffer encryption.
 *  @param[in]  message  input buffer(Plaintext)
 *  @param[in]  msglen   input buffer length
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[in]  ivlen    length of the initial vector
 *  @param[out] pResult  the result of AES-CTR encrypt function(Ciphertext)
 *
 *  @note   Some details need to know before setting:
 *          - Usually ivlen is 16 bytes in AES.
 *          - The starting address of iv can't be NULL and need to be 32 byte-aligned.
 *          - The msglen must be 16 byte-aligned in AES.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - Encryption handles msglen bytes of plaintext, then it will generate the same size of ciphertext.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_ctr_encrypt(
    IN const u8 *message, 	IN const u32 msglen,
    IN const u8 *iv, 		IN const u32 ivlen,
    OUT u8 *pResult);

/**
 *  @brief AES-CTR buffer decryption.
 *  @param[in]  message  input buffer(Ciphertext)
 *  @param[in]  msglen   input buffer length
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[in]  ivlen    length of the initial vector
 *  @param[out] pResult  the result of AES-CTR decrypt function(Plaintext)
 *
 *  @note   Some details need to know before setting:
 *          - Usually ivlen is 16 bytes in AES.
 *          - The starting address of iv can't be NULL and need to be 32 byte-aligned.
 *          - Usually decrypted IV is the same as encrypted IV.
 *          - The msglen must be 16 byte-aligned in AES.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - Decryption handles msglen bytes of ciphertext, then it will generate the same size of plaintext.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_ctr_decrypt(
    IN const u8 *message, 	IN const u32 msglen,
    IN const u8 *iv, 		IN const u32 ivlen,
    OUT u8 *pResult);

// AES-CFB
/**
 *  @brief Initializes the AES-CFB function with a secret key.
 *  @param[in]  key  secret key
 *  @param[in]  keylen   length of the key
 *
 *  @note   Some details need to know before setting:
 *          - AES keylen must be 16bytes(128bits), 24bytes(192bits) or 32bytes(256bits).
 *          - The starting address of key can't be NULL and need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_cfb_init(IN const u8 *key, IN const u32 keylen);

/**
 *  @brief AES-CFB buffer encryption.
 *  @param[in]  message  input buffer(Plaintext)
 *  @param[in]  msglen   input buffer length
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[in]  ivlen    length of the initial vector
 *  @param[out] pResult  the result of AES-CFB encrypt function(Ciphertext)
 *
 *  @note   Some details need to know before setting:
 *          - Usually ivlen is 16 bytes in AES.
 *          - The starting address of iv can't be NULL and need to be 32 byte-aligned.
 *          - The msglen must be 16 byte-aligned in AES.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - Encryption handles msglen bytes of plaintext, then it will generate the same size of ciphertext.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_cfb_encrypt(
    IN const u8 *message, 	IN const u32 msglen,
    IN const u8 *iv, 		IN const u32 ivlen,
    OUT u8 *pResult);

/**
 *  @brief AES-CFB buffer decryption.
 *  @param[in]  message  input buffer(Ciphertext)
 *  @param[in]  msglen   input buffer length
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[in]  ivlen    length of the initial vector
 *  @param[out] pResult  the result of AES-CFB decrypt function(Plaintext)
 *
 *  @note   Some details need to know before setting:
 *          - Usually ivlen is 16 bytes in AES.
 *          - The starting address of iv can't be NULL and need to be 32 byte-aligned.
 *          - Usually decrypted IV is the same as encrypted IV.
 *          - The msglen must be 16 byte-aligned in AES.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - Decryption handles msglen bytes of ciphertext, then it will generate the same size of plaintext.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_cfb_decrypt(
    IN const u8 *message, 	IN const u32 msglen,
    IN const u8 *iv, 		IN const u32 ivlen,
    OUT u8 *pResult);

// AES-OFB
/**
 *  @brief Initializes the AES-OFB function with a secret key.
 *  @param[in]  key  secret key
 *  @param[in]  keylen   length of the key
 *
 *  @note   Some details need to know before setting:
 *          - AES keylen must be 16bytes(128bits), 24bytes(192bits) or 32bytes(256bits).
 *          - The starting address of key can't be NULL and need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_ofb_init(IN const u8 *key, IN const u32 keylen);

/**
 *  @brief AES-OFB buffer encryption.
 *  @param[in]  message  input buffer(Plaintext)
 *  @param[in]  msglen   input buffer length
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[in]  ivlen    length of the initial vector
 *  @param[out] pResult  the result of AES-OFB encrypt function(Ciphertext)
 *
 *  @note   Some details need to know before setting:
 *          - Usually ivlen is 16 bytes in AES.
 *          - The starting address of iv can't be NULL and need to be 32 byte-aligned.
 *          - The msglen must be 16 byte-aligned in AES.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - Encryption handles msglen bytes of plaintext, then it will generate the same size of ciphertext.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_ofb_encrypt(
    IN const u8 *message, 	IN const u32 msglen,
    IN const u8 *iv, 		IN const u32 ivlen,
    OUT u8 *pResult);

/**
 *  @brief AES-OFB buffer decryption.
 *  @param[in]  message  input buffer(Ciphertext)
 *  @param[in]  msglen   input buffer length
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[in]  ivlen    length of the initial vector
 *  @param[out] pResult  the result of AES-OFB decrypt function(Plaintext)
 *
 *  @note   Some details need to know before setting:
 *          - Usually ivlen is 16 bytes in AES.
 *          - The starting address of iv can't be NULL and need to be 32 byte-aligned.
 *          - Usually decrypted IV is the same as encrypted IV.
 *          - The msglen must be 16 byte-aligned in AES.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - Decryption handles msglen bytes of ciphertext, then it will generate the same size of plaintext.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_ofb_decrypt(
    IN const u8 *message, 	IN const u32 msglen,
    IN const u8 *iv, 		IN const u32 ivlen,
    OUT u8 *pResult);

//AES-GHASH
/**
 *  @brief AES-GHASH buffer digest algorithm.
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[in]  key      secret key
 *  @param[in]  keylen   length of the key
 *  @param[out] pDigest  the result of AES-GHASH function
 *
 *  @note   Some details need to know before setting:
 *          - The starting address of key can't be NULL and need to be 32 byte-aligned.
 *          - The keylen must be 16 bytes.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_ghash(
    IN const u8 *message, IN const u32 msglen,
    IN const u8 *key, IN const u32 keylen,
    OUT u8 *pDigest);

/**
 *  @brief Initializes the AES-GHASH function with a secret key.
 *  @param[in]  key      secret key
 *  @param[in]  keylen   length of the key
 *
 *  @note   Some details need to know before setting:
 *          - The starting address of key can't be NULL and need to be 32 byte-aligned.
 *          - The keylen must be 16 bytes.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_ghash_init(IN const u8 *key, IN const u32 keylen);

/**
 *  @brief AES-GHASH process buffer.
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[out] pDigest  the result of AES-GHASH function
 *
 *  @note   Some details need to know before setting:
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_ghash_process(IN const u8 *message, IN const u32 msglen, OUT u8 *pDigest);

//AES-GMAC
/**
 *  @brief AES-GMAC buffer digest algorithm.
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[in]  key      secret key
 *  @param[in]  keylen   length of the key
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[in]  aad      additional data
 *  @param[in]  aadlen   length of additional data
 *  @param[out] pTag  buffer for holding the tag(Authentication code)
 *
 *  @note   Some details need to know before setting:
 *          - The starting address of key can't be NULL and need to be 32 byte-aligned.
 *          - AES keylen must be 16bytes(128bits), 24bytes(192bits) or 32bytes(256bits).
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *          - The starting address of iv can't be NULL and need to be 32 byte-aligned.
 *          - If assign an address to aad, the address need to be 32 byte-aligned.
 *          - The maximum aad length can't be over 496 bytes.
 *          - The starting address of pTag can't be NULL.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_gmac(
    IN const u8 *message, IN const u32 msglen,
    IN const u8 *key, IN const u32 keylen,
    IN const u8 *iv,
    IN const u8 *aad, IN const u32 aadlen, OUT u8 *pTag);

/**
 *  @brief Initializes the AES-GMAC function with a secret key.
 *  @param[in]  key      secret key
 *  @param[in]  keylen   length of the key
 *
 *  @note   Some details need to know before setting:
 *          - The starting address of key can't be NULL and need to be 32 byte-aligned.
 *          - AES keylen must be 16bytes(128bits), 24bytes(192bits) or 32bytes(256bits).
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_gmac_init(IN const u8 *key, IN const u32 keylen);

/**
 *  @brief AES-GMAC process buffer.
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[in]  aad      additional data
 *  @param[in]  aadlen   length of additional data
 *  @param[out] pTag  buffer for holding the tag(Authentication code)
 *
 *  @note   Some details need to know before setting:
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *          - The starting address of iv can't be NULL and need to be 32 byte-aligned.
 *          - If assign an address to aad, the address need to be 32 byte-aligned.
 *          - The maximum aad length can't be over 496 bytes.
 *          - The starting address of pTag can't be NULL.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_gmac_process(
    IN const u8 *message, IN const u32 msglen,
    IN const u8 *iv, IN const u8 *aad, IN const u32 aadlen, OUT u8 *pTag);

//AES-GCTR
/**
 *  @brief Initializes the AES-GCTR function with a secret key.
 *  @param[in]  key      secret key
 *  @param[in]  keylen   length of the key
 *
 *  @note   Some details need to know before setting:
 *          - The starting address of key can't be NULL and need to be 32 byte-aligned.
 *          - AES keylen must be 16bytes(128bits), 24bytes(192bits) or 32bytes(256bits).
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_gctr_init(IN const u8 *key, IN const u32 keylen);

/**
 *  @brief AES-GCTR buffer encryption.
 *  @param[in]  message  input buffer(Plaintext)
 *  @param[in]  msglen   input buffer length
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[out] pResult  the result of AES-GCTR encrypt function(Ciphertext)
 *
 *  @note   Some details need to know before setting:
 *          - The starting address of iv can't be NULL and need to be 32 byte-aligned.
 *          - The msglen must be 16 byte-aligned in AES.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - Encryption handles msglen bytes of plaintext, then it will generate the same size of ciphertext.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_gctr_encrypt(
    IN const u8 *message, 	IN const u32 msglen,
    IN const u8 *iv, OUT u8 *pResult);

/**
 *  @brief AES-GCTR buffer decryption.
 *  @param[in]  message  input buffer(Ciphertext)
 *  @param[in]  msglen   input buffer length
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[out] pResult  the result of AES-CTR decrypt function(Plaintext)
 *
 *  @note   Some details need to know before setting:
 *          - The starting address of iv can't be NULL and need to be 32 byte-aligned.
 *          - Usually decrypted IV is the same as encrypted IV.
 *          - The msglen must be 16 byte-aligned in AES.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - Decryption handles msglen bytes of ciphertext, then it will generate the same size of plaintext.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_gctr_decrypt(
    IN const u8 *message, 	IN const u32 msglen,
    IN const u8 *iv, OUT u8 *pResult);

// AES-GCM
/**
 *  @brief Initializes the AES-GCM function with a secret key.
 *  @param[in]  key  secret key
 *  @param[in]  keylen   length of the key
 *
 *  @note   Some details need to know before setting:
 *          - AES keylen must be 16bytes(128bits), 24bytes(192bits) or 32bytes(256bits).
 *          - The starting address of key can't be NULL and need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_gcm_init(IN const u8 *key, IN const u32 keylen);

/**
 *  @brief AES-GCM buffer encryption.
 *  @param[in]  message  input buffer(Plaintext)
 *  @param[in]  msglen   input buffer length
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[in]  aad      additional data
 *  @param[in]  aadlen   length of additional data
 *  @param[out] pResult  the result of AES-GCM encrypt function(Ciphertext)
 *  @param[out] pTag     buffer for holding the tag(Authentication code)
 *
 *  @note   Some details need to know before setting:
 *          - The starting address of iv can't be NULL and need to be 32 byte-aligned.
 *          - If assign an address to aad, the address need to be 32 byte-aligned.
 *          - The maximum aad length can't be over 496 bytes.
 *          - The starting address of pTag can't be NULL.
 *          - The msglen must be 16 byte-aligned in AES.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - Encryption handles msglen bytes of plaintext, then it will generate the same size of ciphertext
 *            and a 16bytes tag value.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_gcm_encrypt(
    IN const u8 *message, 	IN const u32 msglen,
    IN const u8 *iv,
    IN const u8 *aad,		IN const u32 aadlen,
    OUT u8 *pResult, OUT u8 *pTag);

/**
 *  @brief AES-GCM buffer decryption.
 *  @param[in]  message  input buffer(Ciphertext)
 *  @param[in]  msglen   input buffer length
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[in]  aad      additional data
 *  @param[in]  aadlen   length of additional data
 *  @param[out] pResult  the result of AES-GCM decrypt function(Plaintext)
 *  @param[out] pTag     buffer for holding the tag(Authentication code)
 *
 *  @note   Some details need to know before setting:
 *          - The starting address of iv can't be NULL and need to be 32 byte-aligned.
 *          - Usually decrypted IV is the same as encrypted IV.
 *          - If assign an address to aad, the address need to be 32 byte-aligned.
 *          - The maximum aad length can't be over 496 bytes.
 *          - The starting address of pTag can't be NULL.
 *          - The msglen must be 16 byte-aligned in AES.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - Decryption handles msglen bytes of ciphertext, then it will generate the same size of plaintext
 *            and a 16bytes tag value.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_aes_gcm_decrypt(
    IN const u8 *message, 	IN const u32 msglen,
    IN const u8 *iv,
    IN const u8 *aad,		IN const u32 aadlen,
    OUT u8 *pResult, OUT u8 *pTag);

#if defined(CONFIG_BUILD_NONSECURE)
//
//

// crc
//
/**
 *  @brief       Set the CRC basic parameters
 *  @param[in]   order CRC polynomial order
 *  @param[in]   polynom CRC polynomial coefficients
 *  @param[in]   crcinit CRC initial value
 *  @param[in]   crcxor CRC XOR output value
 *  @param[in]   refin CRC input swap value
 *  @param[in]   refout CRC output swap value
 *  @return      value ==  0    success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_crc_setting(int order, unsigned long polynom, unsigned long crcinit,
                           unsigned long crcxor, int refin, int refout);
/**
 *  @brief      Calculate CRC32 value using command mode.
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[out] pCrc     the result value of CRC32
 *
 *  @note   Some details need to know before setting:
 *          - The maximum input buffer length can't be over 65535 bytes.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_crc32_cmd(IN const u8 *message, IN const u32 msglen, OUT u32 *pCrc);

/**
 *  @brief      Calculate CRC32 value using DMA mode.
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[out] pCrc     the result value of CRC32
 *
 *  @note   Some details need to know before setting:
 *          - The maximum input buffer length can't be over 65535 bytes.
 *          - Input buffer which is assigned a 32 byte-aligned address can process more efficient than
 *            unaligned address.
 *          - The starting address of the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_crc32_dma(IN const u8 *message, IN const u32 msglen, OUT u32 *pCrc);

/**
 *  @brief      Calculate CRC value using command mode.
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[out] pCrc     the result value of CRC
 *
 *  @note   Some details need to know before setting:
 *          - The maximum input buffer length can't be over 65535 bytes.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_crc_cmd(IN const u8 *message, IN const u32 msglen, OUT u32 *pCrc);

/**
 *  @brief      Calculate CRC value using DMA mode.
 *  @param[in]  message  input buffer
 *  @param[in]  msglen   input buffer length
 *  @param[out] pCrc     the result value of CRC
 *
 *  @note   Some details need to know before setting:
 *          - The maximum input buffer length can't be over 65535 bytes.
 *          - Input buffer which is assigned a 32 byte-aligned address can process more efficient than
 *            unaligned address.
 *          - The starting address of the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_crc_dma(IN const u8 *message, IN const u32 msglen, OUT u32 *pCrc);
#endif

//MIX-MODE
/**
 *  @brief Initializes the Mix mode function(SSH/ESP/SSL) with secret key.
 *  @param[in]  cipher_type    Mix mode cipher type
 *  @param[in]  auth_type      Mix mode authentication type
 *  @param[in]  cipher_key     cipher key buffer
 *  @param[in]  cipher_keylen  the length of cipher key 
 *  @param[in]  auth_key       HMAC key buffer
 *  @param[in]  auth_keylen    length of the HMAC key
 *
 *  @note   Some details need to know before setting:
 *          - Ameba_zii cipher type only support AES
 *          - AES cipher keylen must be 16bytes(128bits), 24bytes(192bits) or 32bytes(256bits).
 *          - The starting address of cipher key can't be NULL and need to be 32 byte-aligned.
 *          - The maximum HMAC key length can't be over 64 bytes.
 *          - The starting address of cipher HMAC key need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_mix_mode_init(IN const u32 cipher_type, IN const u32 auth_type, 
                             IN const u8 *cipher_key, IN const u32 cipher_keylen,
                             IN const u8 *auth_key, IN const u32 auth_keylen);

/**
 *  @brief Mix-mode SSH buffer encryption and authentication.
 *  @param[in]  cipher_type    Mix mode cipher type
 *  @param[in]  message  input buffer(Plaintext)
 *  @param[in]  msglen   input buffer length
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[in]  ivlen    length of the initial vector
 *  @param[in]  aad      additional data buffer
 *  @param[in]  aadlen   length of additional data
 *  @param[in]  auth_type    Mix mode authentication type
 *  @param[out] pResult  the result buffer for mix mode cipher encrypt function(Ciphertext)
 *  @param[out] pTag     the digest buffer for mix mode authentication
 *
 *  @note   Some details need to know before setting:
 *          - The mix mode cipher type needs to be same as init function.
 *          - The mix mode auth type needs to be same as init function.
 *          - If assign an address to iv, the address need to be 32 byte-aligned.
 *          - If assign an address to aad, the address need to be 32 byte-aligned.
 *          - The maximum aad length can't be over 496 bytes.
 *          - The starting address of pTag can't be NULL.
 *          - The msglen must be 16 byte-aligned in AES.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_mix_ssh_encrypt(IN const u32 cipher_type,
                               IN const u8 *message, IN const u32 msglen,
                               IN const u8 *iv, IN const u32 ivlen,
                               IN const u8 *aad, IN const u32 aadlen,
                               IN const u32 auth_type,
                               OUT u8 *pResult, OUT u8 *pTag);

/**
 *  @brief Mix-mode SSH buffer decryption and authentication.
 *  @param[in]  cipher_type    Mix mode cipher type
 *  @param[in]  message  input buffer(Ciphertext)
 *  @param[in]  msglen   input buffer length
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[in]  ivlen    length of the initial vector
 *  @param[in]  aad      additional data buffer
 *  @param[in]  aadlen   length of additional data
 *  @param[in]  auth_type    Mix mode authentication type
 *  @param[out] pResult  the result buffer for mix mode cipher decrypt function(Plaintext)
 *  @param[out] pTag     the digest buffer for mix mode authentication
 *
 *  @note   Some details need to know before setting:
 *          - The mix mode cipher type needs to be same as init function.
 *          - The mix mode auth type needs to be same as init function.
 *          - Usually decrypted IV is the same as encrypted IV
 *          - If assign an address to iv, the address need to be 32 byte-aligned.
 *          - If assign an address to aad, the address need to be 32 byte-aligned.
 *          - The maximum aad length can't be over 496 bytes.
 *          - The starting address of pTag can't be NULL.
 *          - The msglen must be 16 byte-aligned in AES.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_mix_ssh_decrypt(IN const u32 cipher_type,
                               IN const u8 *message, IN const u32 msglen,
                               IN const u8 *iv, IN const u32 ivlen,
                               IN const u8 *aad, IN const u32 aadlen,
                               IN const u32 auth_type,
                               OUT u8 *pResult, OUT u8 *pTag);

/**
 *  @brief Mix-mode ESP buffer encryption and authentication.
 *  @param[in]  cipher_type    Mix mode cipher type
 *  @param[in]  message  input buffer(Plaintext)
 *  @param[in]  msglen   input buffer length
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[in]  ivlen    length of the initial vector
 *  @param[in]  aad      additional data buffer
 *  @param[in]  aadlen   length of additional data
 *  @param[in]  auth_type    Mix mode authentication type
 *  @param[out] pResult  the result buffer for mix mode cipher encrypt function(Ciphertext)
 *  @param[out] pTag     the digest buffer for mix mode authentication
 *
 *  @note   Some details need to know before setting:
 *          - The mix mode cipher type needs to be same as init function.
 *          - The mix mode auth type needs to be same as init function.
 *          - If assign an address to iv, the address need to be 32 byte-aligned.
 *          - If assign an address to aad, the address need to be 32 byte-aligned.
 *          - The maximum aad length can't be over 496 bytes.
 *          - The starting address of pTag can't be NULL.
 *          - The msglen must be 16 byte-aligned in AES.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_mix_esp_encrypt(IN const u32 cipher_type,
                               IN const u8 *message, IN const u32 msglen,
                               IN const u8 *iv, IN const u32 ivlen,
                               IN const u8 *aad, IN const u32 aadlen,
                               IN const u32 auth_type,
                               OUT u8 *pResult, OUT u8 *pTag);

/**
 *  @brief Mix-mode ESP buffer decryption and authentication.
 *  @param[in]  cipher_type    Mix mode cipher type
 *  @param[in]  message  input buffer(Ciphertext)
 *  @param[in]  msglen   input buffer length
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[in]  ivlen    length of the initial vector
 *  @param[in]  aad      additional data buffer
 *  @param[in]  aadlen   length of additional data
 *  @param[in]  auth_type    Mix mode authentication type
 *  @param[out] pResult  the result buffer for mix mode cipher decrypt function(Plaintext)
 *  @param[out] pTag     the digest buffer for mix mode authentication
 *
 *  @note   Some details need to know before setting:
 *          - The mix mode cipher type needs to be same as init function.
 *          - The mix mode auth type needs to be same as init function.
 *          - Usually decrypted IV is the same as encrypted IV
 *          - If assign an address to iv, the address need to be 32 byte-aligned.
 *          - If assign an address to aad, the address need to be 32 byte-aligned.
 *          - The maximum aad length can't be over 496 bytes.
 *          - The starting address of pTag can't be NULL.
 *          - The msglen must be 16 byte-aligned in AES.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_mix_esp_decrypt(IN const u32 cipher_type,
                               IN const u8 *message, IN const u32 msglen,
                               IN const u8 *iv, IN const u32 ivlen,
                               IN const u8 *aad, IN const u32 aadlen,
                               IN const u32 auth_type,
                               OUT u8 *pResult, OUT u8 *pTag);

/**
 *  @brief Mix-mode SSL_TLS buffer encryption.
 *  @param[in]  cipher_type    Mix mode cipher type
 *  @param[in]  message  input buffer(Plaintext)
 *  @param[in]  msglen   input buffer length
 *  @param[in]  iv       buffer holding the initial vector data
 *  @param[in]  ivlen    length of the initial vector
 *  @param[in]  aad      additional data buffer
 *  @param[in]  aadlen   length of additional data
 *  @param[in]  auth_type    Mix mode authentication type
 *  @param[out] pResult  the result buffer for mix mode cipher encrypt function(Ciphertext)
 *
 *  @note   Some details need to know before setting:
 *          - The mix mode cipher type needs to be same as init function.
 *          - The mix mode auth type needs to be same as init function.
 *          - If assign an address to iv, the address need to be 32 byte-aligned.
 *          - If assign an address to aad, the address need to be 32 byte-aligned.
 *          - The maximum aad length can't be over 496 bytes.
 *          - The msglen must be 16 byte-aligned in AES.
 *          - The maximum input buffer length can't be over 64k bytes.
 *          - The starting address of input buffer and the result don't need to be 32 byte-aligned.
 *
 *  @return      value == 0     success
 *  @return      value < 0      fail(Refer to ERRNO)
 */
int hal_crypto_mix_ssl_tls_encrypt(IN const u32 cipher_type,
                                   IN const u8 *message, IN const u32 msglen,
                                   IN const u8 *iv, IN const u32 ivlen,
                                   IN const u8 *aad, IN const u32 aadlen,
                                   IN const u32 auth_type,
                                   OUT u8 *pResult);

#if (CHIP_VER == CHIP_A_CUT) && (defined(CONFIG_BUILD_RAM))
int hal_crypto_auth_update_rtl8710c_patch(hal_crypto_adapter_t *pcrypto_adapter, IN const u32 auth_type,
                                          IN const u8 *message, IN const u32 msglen);

int hal_crypto_auth_final_rtl8710c_patch(hal_crypto_adapter_t *pcrypto_adapter, IN const u32 auth_type, OUT u8 *pDigest);
#endif

/** @} */ /* End of group hs_hal_crypto */

// debug

void rtl_crypto_set_debug(int val);

#ifdef __cplusplus
}
#endif


#endif // __HAL_CRYPTO_H__

