/**************************************************************************//**
 * @file      rtl8710c_sce_sce_type.h
 * @brief
 * @version   V1.00
 * @date      2017-11-1 13:26:50
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
 * limitations under the License.
 *
 ******************************************************************************/

#ifndef _RTL8710C_SCE_SCE_TYPE_H_
#define _RTL8710C_SCE_SCE_TYPE_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/// @cond DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_SCE_REG_TYPE

/**
 * @addtogroup hal_sce_reg SCE Register Type.
 * @ingroup hs_hal_sce
 * @{
 */

/**
  \brief Union type to access sce_sce_ctrl (@ 0x00000000).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000000) SCE Control Register                                       */

  struct {
    __IOM uint32_t sce_en     : 1;            /*!< [0..0] Enable the SCE 0: Disable 1: Enable                                */
    __IOM uint32_t sce_mode_type : 2;         /*!< [2..1] 0: Read mode; SCE decrypts the instructions from the
                                                   external memory and pass through the data which CPU or
                                                   other master ports write into the external memory. 1: Write
                                                   mode; SCE encrypts the data which CPU or others master
                                                   write to the external memory or flash. For the reading
                                                   operation, SCE pass through data to the master port. 2:
                                                   Simultaneous mode; SCE encrypts the reading and writing
                                                   data from the mater ports including CPU and DMA port. 3:
                                                   Reserved.                                                                 */
    __IOM uint32_t sce_block_size : 3;        /*!< [5..3] Every secure section is separated by SCE_BLOCK_SIZE.
                                                   And SCE will adopt the same key to encrypt or decrypt every
                                                   secure block. 0: 32 Bytes 1: 64 Bytes 2: 128 Bytes 3: 256
                                                   Bytes 4~7: Reserved                                                       */
    __IOM uint32_t sce_page_size : 2;         /*!< [7..6] Based on the SCE_PAGE_SIZE, SCE separates the whole memory
                                                   and assigns the number in sequence. This option provides
                                                   the supporting maximum memory size because hardware just
                                                   provides the 12 bits to record the page number. 0: 16K
                                                   1: 32K 2: 64K 3: Reserved                                                 */
  } b;                                        /*!< bit fields for sce_sce_ctrl */
} sce_sce_ctrl_t, *psce_sce_ctrl_t;

/**
  \brief Union type to access sce_sce_aes_key_pair1_w0 (@ 0x00000010).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000010) SCE AES Key of the Pair 1 Word0 Register                   */

  struct {
    __IOM uint32_t aes_key_w0 : 32;           /*!< [31..0] The key bit[31:0]. The user chose the pair 1 to protect
                                                   the code. When SCE want encrypt or decrypt the data, it
                                                   needs this 128bits key.                                                   */
  } b;                                        /*!< bit fields for sce_sce_aes_key_pair1_w0 */
} sce_sce_aes_key_pair1_w0_t, *psce_sce_aes_key_pair1_w0_t;

/**
  \brief Union type to access sce_sce_aes_key_pair1_w1 (@ 0x00000014).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000014) SCE AES Key of the Pair 1 Word1 Register                   */

  struct {
    __IOM uint32_t aes_key_w1 : 32;           /*!< [31..0] The key bit[63:32]. The user chose the pair 1 to protect
                                                   the code. When SCE want encrypt or decrypt the data, it
                                                   needs this 128bits key.                                                   */
  } b;                                        /*!< bit fields for sce_sce_aes_key_pair1_w1 */
} sce_sce_aes_key_pair1_w1_t, *psce_sce_aes_key_pair1_w1_t;

/**
  \brief Union type to access sce_sce_aes_key_pair1_w2 (@ 0x00000018).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000018) SCE AES Key of the Pair 1 Word2 Register                   */

  struct {
    __IOM uint32_t aes_key_w2 : 32;           /*!< [31..0] The key bit[95:64]. The user chose the pair 1 to protect
                                                   the code. When SCE want encrypt or decrypt the data, it
                                                   needs this 128bits key.                                                   */
  } b;                                        /*!< bit fields for sce_sce_aes_key_pair1_w2 */
} sce_sce_aes_key_pair1_w2_t, *psce_sce_aes_key_pair1_w2_t;

/**
  \brief Union type to access sce_sce_aes_key_pair1_w3 (@ 0x0000001C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000001C) SCE AES Key of the Pair 1 Word3 Register                   */

  struct {
    __IOM uint32_t aes_key_w3 : 32;           /*!< [31..0] The key bit[127:96]. The user chose the pair 1 to protect
                                                   the code. When SCE want encrypt or decrypt the data, it
                                                   needs this 128bits key.                                                   */
  } b;                                        /*!< bit fields for sce_sce_aes_key_pair1_w3 */
} sce_sce_aes_key_pair1_w3_t, *psce_sce_aes_key_pair1_w3_t;

/**
  \brief Union type to access sce_sce_aes_iv_pair1_w0 (@ 0x00000020).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000020) SCE IV Key Word0 of the Pair 1 Register                    */

  struct {
    __IOM uint32_t aes_iv_w0  : 32;           /*!< [31..0] The IV bit[31:0]. The user chose the pair 1 to protect
                                                   the code. When SCE want encrypt or decrypt the data, it
                                                   needs the initial vector generated by SCE_IV_KEY_PAIR_1.                  */
  } b;                                        /*!< bit fields for sce_sce_aes_iv_pair1_w0 */
} sce_sce_aes_iv_pair1_w0_t, *psce_sce_aes_iv_pair1_w0_t;

/**
  \brief Union type to access sce_sce_aes_iv_pair1_w1 (@ 0x00000024).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000024) SCE IV Key Word1 of the Pair 1 Register                    */

  struct {
    __IOM uint32_t aes_iv_w1  : 32;           /*!< [31..0] The IV bit[63:31]. The user chose the pair 1 to protect
                                                   the code. When SCE want encrypt or decrypt the data, it
                                                   needs the initial vector generated by SCE_IV_KEY_PAIR_1.                  */
  } b;                                        /*!< bit fields for sce_sce_aes_iv_pair1_w1 */
} sce_sce_aes_iv_pair1_w1_t, *psce_sce_aes_iv_pair1_w1_t;

/**
  \brief Union type to access sce_sce_aes_iv_pair1_w2 (@ 0x00000028).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000028) SCE IV Key Word2 of the Pair 1 Register                    */

  struct {
    __IOM uint32_t aes_iv_w2  : 32;           /*!< [31..0] The IV bit[95:64]. The user chose the pair 1 to protect
                                                   the code. When SCE want encrypt or decrypt the data, it
                                                   needs the initial vector generated by SCE_IV_KEY_PAIR_1.                  */
  } b;                                        /*!< bit fields for sce_sce_aes_iv_pair1_w2 */
} sce_sce_aes_iv_pair1_w2_t, *psce_sce_aes_iv_pair1_w2_t;

/**
  \brief Union type to access sce_sce_aes_iv_pair1_w3 (@ 0x0000002C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000002C) SCE IV Key Word3 of the Pair 1 Register                    */

  struct {
    __IOM uint32_t aes_iv_w3  : 32;           /*!< [31..0] The IV bit[127:96]. The user chose the pair 1 to protect
                                                   the code. When SCE want encrypt or decrypt the data, it
                                                   needs the initial vector generated by SCE_IV_KEY_PAIR_1.                  */
  } b;                                        /*!< bit fields for sce_sce_aes_iv_pair1_w3 */
} sce_sce_aes_iv_pair1_w3_t, *psce_sce_aes_iv_pair1_w3_t;

/**
  \brief Union type to access sce_sce_aes_key_pair2_w0 (@ 0x00000030).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000030) SCE AES Key of the Pair 2 Word0 Register                   */

  struct {
    __IOM uint32_t aes_key_w0 : 32;           /*!< [31..0] The key bit[31:0]. The user chose the pair 2 to protect
                                                   the code. When SCE want encrypt or decrypt the data, it
                                                   needs this 128bits key.                                                   */
  } b;                                        /*!< bit fields for sce_sce_aes_key_pair2_w0 */
} sce_sce_aes_key_pair2_w0_t, *psce_sce_aes_key_pair2_w0_t;

/**
  \brief Union type to access sce_sce_aes_key_pair2_w1 (@ 0x00000034).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000034) SCE AES Key of the Pair 2 Word1 Register                   */

  struct {
    __IOM uint32_t aes_key_w1 : 32;           /*!< [31..0] The key bit[63:32]. The user chose the pair 2 to protect
                                                   the code. When SCE want encrypt or decrypt the data, it
                                                   needs this 128bits key.                                                   */
  } b;                                        /*!< bit fields for sce_sce_aes_key_pair2_w1 */
} sce_sce_aes_key_pair2_w1_t, *psce_sce_aes_key_pair2_w1_t;

/**
  \brief Union type to access sce_sce_aes_key_pair2_w2 (@ 0x00000038).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000038) SCE AES Key of the Pair 2 Word2 Register                   */

  struct {
    __IOM uint32_t aes_key_w2 : 32;           /*!< [31..0] The key bit[95:64]. The user chose the pair 2 to protect
                                                   the code. When SCE want encrypt or decrypt the data, it
                                                   needs this 128bits key.                                                   */
  } b;                                        /*!< bit fields for sce_sce_aes_key_pair2_w2 */
} sce_sce_aes_key_pair2_w2_t, *psce_sce_aes_key_pair2_w2_t;

/**
  \brief Union type to access sce_sce_aes_key_pair2_w3 (@ 0x0000003C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000003C) SCE AES Key of the Pair 2 Word3 Register                   */

  struct {
    __IOM uint32_t aes_key_w3 : 32;           /*!< [31..0] The key bit[127:96]. The user chose the pair 2 to protect
                                                   the code. When SCE want encrypt or decrypt the data, it
                                                   needs this 128bits key.                                                   */
  } b;                                        /*!< bit fields for sce_sce_aes_key_pair2_w3 */
} sce_sce_aes_key_pair2_w3_t, *psce_sce_aes_key_pair2_w3_t;

/**
  \brief Union type to access sce_sce_aes_iv_pair2_w0 (@ 0x00000040).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000040) SCE IV Key Word0 of the Pair 2 Register                    */

  struct {
    __IOM uint32_t aes_iv_w0  : 32;           /*!< [31..0] The IV bit[31:0]. The user chose the pair 1 to protect
                                                   the code. When SCE want encrypt or decrypt the data, it
                                                   needs the initial vector generated by SCE_IV_KEY_PAIR_2.                  */
  } b;                                        /*!< bit fields for sce_sce_aes_iv_pair2_w0 */
} sce_sce_aes_iv_pair2_w0_t, *psce_sce_aes_iv_pair2_w0_t;

/**
  \brief Union type to access sce_sce_aes_iv_pair2_w1 (@ 0x00000044).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000044) SCE IV Key Word1 of the Pair 2 Register                    */

  struct {
    __IOM uint32_t aes_iv_w1  : 32;           /*!< [31..0] The IV bit[63:31]. The user chose the pair 1 to protect
                                                   the code. When SCE want encrypt or decrypt the data, it
                                                   needs the initial vector generated by SCE_IV_KEY_PAIR_2.                  */
  } b;                                        /*!< bit fields for sce_sce_aes_iv_pair2_w1 */
} sce_sce_aes_iv_pair2_w1_t, *psce_sce_aes_iv_pair2_w1_t;

/**
  \brief Union type to access sce_sce_aes_iv_pair2_w2 (@ 0x00000048).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000048) SCE IV Key Word2 of the Pair 2 Register                    */

  struct {
    __IOM uint32_t aes_iv_w2  : 32;           /*!< [31..0] The IV bit[95:64]. The user chose the pair 1 to protect
                                                   the code. When SCE want encrypt or decrypt the data, it
                                                   needs the initial vector generated by SCE_IV_KEY_PAIR_2.                  */
  } b;                                        /*!< bit fields for sce_sce_aes_iv_pair2_w2 */
} sce_sce_aes_iv_pair2_w2_t, *psce_sce_aes_iv_pair2_w2_t;

/**
  \brief Union type to access sce_sce_aes_iv_pair2_w3 (@ 0x0000004C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000004C) SCE IV Key Word3 of the Pair 2 Register                    */

  struct {
    __IOM uint32_t aes_iv_w3  : 32;           /*!< [31..0] The IV bit[127:96]. The user chose the pair 1 to protect
                                                   the code. When SCE want encrypt or decrypt the data, it
                                                   needs the initial vector generated by SCE_IV_KEY_PAIR_2.                  */
  } b;                                        /*!< bit fields for sce_sce_aes_iv_pair2_w3 */
} sce_sce_aes_iv_pair2_w3_t, *psce_sce_aes_iv_pair2_w3_t;

/**
  \brief Union type to access sce_sce_sect1 (@ 0x00000050).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000050) SCE Secure Section 1 Register                              */

  struct {
    __IOM uint32_t start_page : 12;           /*!< [11..0] If the users want to enable secure section, they need
                                                   configure the start page number. The definition of page
                                                   is defined in Chapter 4.                                                  */
    __IOM uint32_t end_page   : 12;           /*!< [23..12] If the users want to enable secure section, they need
                                                   configure the end page number. The definition of page is
                                                   defined in Chapter 4                                                      */
    __IOM uint32_t key_pair_id : 3;           /*!< [26..24] The users can chose one key pair from the two sets.
                                                   0: The key pair 1 1: The key pair 2                                       */
    __IOM uint32_t secure_en  : 1;            /*!< [27..27] data encryption control: 0: Data encryption of this
                                                   section is disabled. 1: Data encryption of this section
                                                   is enabled.                                                               */
    __IOM uint32_t remap_en   : 1;            /*!< [28..28] memory address remaping control 0: Address remapping
                                                   of this section is disabled. 1: Address remapping of this
                                                   section is enabled..                                                      */
  } b;                                        /*!< bit fields for sce_sce_sect1 */
} sce_sce_sect1_t, *psce_sce_sect1_t;

/**
  \brief Union type to access sce_sce_sect2 (@ 0x00000054).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000054) SCE Secure Section 2 Register                              */

  struct {
    __IOM uint32_t start_page : 12;           /*!< [11..0] If the users want to enable secure section, they need
                                                   configure the start page number. The definition of page
                                                   is defined in Chapter 4.                                                  */
    __IOM uint32_t end_page   : 12;           /*!< [23..12] If the users want to enable secure section, they need
                                                   configure the end page number. The definition of page is
                                                   defined in Chapter 4                                                      */
    __IOM uint32_t key_pair_id : 3;           /*!< [26..24] The users can chose one key pair from the two sets.
                                                   0: The key pair 1 1: The key pair 2                                       */
    __IOM uint32_t secure_en  : 1;            /*!< [27..27] data encryption control: 0: Data encryption of this
                                                   section is disabled. 1: Data encryption of this section
                                                   is enabled.                                                               */
    __IOM uint32_t remap_en   : 1;            /*!< [28..28] memory address remaping control 0: Address remapping
                                                   of this section is disabled. 1: Address remapping of this
                                                   section is enabled..                                                      */
  } b;                                        /*!< bit fields for sce_sce_sect2 */
} sce_sce_sect2_t, *psce_sce_sect2_t;

/**
  \brief Union type to access sce_sce_sect3 (@ 0x00000058).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000058) SCE Secure Section 3 Register                              */

  struct {
    __IOM uint32_t start_page : 12;           /*!< [11..0] If the users want to enable secure section, they need
                                                   configure the start page number. The definition of page
                                                   is defined in Chapter 4.                                                  */
    __IOM uint32_t end_page   : 12;           /*!< [23..12] If the users want to enable secure section, they need
                                                   configure the end page number. The definition of page is
                                                   defined in Chapter 4                                                      */
    __IOM uint32_t key_pair_id : 3;           /*!< [26..24] The users can chose one key pair from the two sets.
                                                   0: The key pair 1 1: The key pair 2                                       */
    __IOM uint32_t secure_en  : 1;            /*!< [27..27] data encryption control: 0: Data encryption of this
                                                   section is disabled. 1: Data encryption of this section
                                                   is enabled.                                                               */
    __IOM uint32_t remap_en   : 1;            /*!< [28..28] memory address remaping control 0: Address remapping
                                                   of this section is disabled. 1: Address remapping of this
                                                   section is enabled..                                                      */
  } b;                                        /*!< bit fields for sce_sce_sect3 */
} sce_sce_sect3_t, *psce_sce_sect3_t;

/**
  \brief Union type to access sce_sce_sect4 (@ 0x0000005C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000005C) SCE Secure Section 4 Register                              */

  struct {
    __IOM uint32_t start_page : 12;           /*!< [11..0] If the users want to enable secure section, they need
                                                   configure the start page number. The definition of page
                                                   is defined in Chapter 4.                                                  */
    __IOM uint32_t end_page   : 12;           /*!< [23..12] If the users want to enable secure section, they need
                                                   configure the end page number. The definition of page is
                                                   defined in Chapter 4                                                      */
    __IOM uint32_t key_pair_id : 3;           /*!< [26..24] The users can chose one key pair from the two sets.
                                                   0: The key pair 1 1: The key pair 2                                       */
    __IOM uint32_t secure_en  : 1;            /*!< [27..27] data encryption control: 0: Data encryption of this
                                                   section is disabled. 1: Data encryption of this section
                                                   is enabled.                                                               */
    __IOM uint32_t remap_en   : 1;            /*!< [28..28] memory address remaping control 0: Address remapping
                                                   of this section is disabled. 1: Address remapping of this
                                                   section is enabled..                                                      */
  } b;                                        /*!< bit fields for sce_sce_sect4 */
} sce_sce_sect4_t, *psce_sce_sect4_t;

/**
  \brief Union type to access sce_sce_sect5 (@ 0x00000060).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000060) SCE Secure Section 5 Register                              */

  struct {
    __IOM uint32_t start_page : 12;           /*!< [11..0] If the users want to enable secure section, they need
                                                   configure the start page number. The definition of page
                                                   is defined in Chapter 4.                                                  */
    __IOM uint32_t end_page   : 12;           /*!< [23..12] If the users want to enable secure section, they need
                                                   configure the end page number. The definition of page is
                                                   defined in Chapter 4                                                      */
    __IOM uint32_t key_pair_id : 3;           /*!< [26..24] The users can chose one key pair from the two sets.
                                                   0: The key pair 1 1: The key pair 2                                       */
    __IOM uint32_t secure_en  : 1;            /*!< [27..27] data encryption control: 0: Data encryption of this
                                                   section is disabled. 1: Data encryption of this section
                                                   is enabled.                                                               */
    __IOM uint32_t remap_en   : 1;            /*!< [28..28] memory address remaping control 0: Address remapping
                                                   of this section is disabled. 1: Address remapping of this
                                                   section is enabled..                                                      */
  } b;                                        /*!< bit fields for sce_sce_sect5 */
} sce_sce_sect5_t, *psce_sce_sect5_t;

/**
  \brief Union type to access sce_sce_sect6 (@ 0x00000064).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000064) SCE Secure Section 6 Register                              */

  struct {
    __IOM uint32_t start_page : 12;           /*!< [11..0] If the users want to enable secure section, they need
                                                   configure the start page number. The definition of page
                                                   is defined in Chapter 4.                                                  */
    __IOM uint32_t end_page   : 12;           /*!< [23..12] If the users want to enable secure section, they need
                                                   configure the end page number. The definition of page is
                                                   defined in Chapter 4                                                      */
    __IOM uint32_t key_pair_id : 3;           /*!< [26..24] The users can chose one key pair from the two sets.
                                                   0: The key pair 1 1: The key pair 2                                       */
    __IOM uint32_t secure_en  : 1;            /*!< [27..27] data encryption control: 0: Data encryption of this
                                                   section is disabled. 1: Data encryption of this section
                                                   is enabled.                                                               */
    __IOM uint32_t remap_en   : 1;            /*!< [28..28] memory address remaping control 0: Address remapping
                                                   of this section is disabled. 1: Address remapping of this
                                                   section is enabled..                                                      */
  } b;                                        /*!< bit fields for sce_sce_sect6 */
} sce_sce_sect6_t, *psce_sce_sect6_t;

/**
  \brief Union type to access sce_sce_sect7 (@ 0x00000068).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000068) SCE Secure Section 7 Register                              */

  struct {
    __IOM uint32_t start_page : 12;           /*!< [11..0] If the users want to enable secure section, they need
                                                   configure the start page number. The definition of page
                                                   is defined in Chapter 4.                                                  */
    __IOM uint32_t end_page   : 12;           /*!< [23..12] If the users want to enable secure section, they need
                                                   configure the end page number. The definition of page is
                                                   defined in Chapter 4                                                      */
    __IOM uint32_t key_pair_id : 3;           /*!< [26..24] The users can chose one key pair from the two sets.
                                                   0: The key pair 1 1: The key pair 2                                       */
    __IOM uint32_t secure_en  : 1;            /*!< [27..27] data encryption control: 0: Data encryption of this
                                                   section is disabled. 1: Data encryption of this section
                                                   is enabled.                                                               */
    __IOM uint32_t remap_en   : 1;            /*!< [28..28] memory address remaping control 0: Address remapping
                                                   of this section is disabled. 1: Address remapping of this
                                                   section is enabled..                                                      */
  } b;                                        /*!< bit fields for sce_sce_sect7 */
} sce_sce_sect7_t, *psce_sce_sect7_t;

/**
  \brief Union type to access sce_sce_sect8 (@ 0x0000006C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000006C) SCE Secure Section 8 Register                              */

  struct {
    __IOM uint32_t start_page : 12;           /*!< [11..0] If the users want to enable secure section, they need
                                                   configure the start page number. The definition of page
                                                   is defined in Chapter 4.                                                  */
    __IOM uint32_t end_page   : 12;           /*!< [23..12] If the users want to enable secure section, they need
                                                   configure the end page number. The definition of page is
                                                   defined in Chapter 4                                                      */
    __IOM uint32_t key_pair_id : 3;           /*!< [26..24] The users can chose one key pair from the two sets.
                                                   0: The key pair 1 1: The key pair 2                                       */
    __IOM uint32_t secure_en  : 1;            /*!< [27..27] data encryption control: 0: Data encryption of this
                                                   section is disabled. 1: Data encryption of this section
                                                   is enabled.                                                               */
    __IOM uint32_t remap_en   : 1;            /*!< [28..28] memory address remaping control 0: Address remapping
                                                   of this section is disabled. 1: Address remapping of this
                                                   section is enabled..                                                      */
  } b;                                        /*!< bit fields for sce_sce_sect8 */
} sce_sce_sect8_t, *psce_sce_sect8_t;

/**
  \brief Union type to access sce_sce_remap1 (@ 0x00000070).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000070) SCE Secure Section Remap 1 Register                        */

  struct {
    __IOM uint32_t phy_addr   : 32;           /*!< [31..0] The physical address to be remap to the address assigned
                                                   by the Secure Section register.                                           */
  } b;                                        /*!< bit fields for sce_sce_remap1 */
} sce_sce_remap1_t, *psce_sce_remap1_t;

/**
  \brief Union type to access sce_sce_remap2 (@ 0x00000074).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000074) SCE Secure Section Remap 2 Register                        */

  struct {
    __IOM uint32_t phy_addr   : 32;           /*!< [31..0] The physical address to be remap to the address assigned
                                                   by the Secure Section register.                                           */
  } b;                                        /*!< bit fields for sce_sce_remap2 */
} sce_sce_remap2_t, *psce_sce_remap2_t;

/**
  \brief Union type to access sce_sce_remap3 (@ 0x00000078).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000078) SCE Secure Section Remap 3 Register                        */

  struct {
    __IOM uint32_t phy_addr   : 32;           /*!< [31..0] The physical address to be remap to the address assigned
                                                   by the Secure Section register.                                           */
  } b;                                        /*!< bit fields for sce_sce_remap3 */
} sce_sce_remap3_t, *psce_sce_remap3_t;

/**
  \brief Union type to access sce_sce_remap4 (@ 0x0000007C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000007C) SCE Secure Section Remap 4 Register                        */

  struct {
    __IOM uint32_t phy_addr   : 32;           /*!< [31..0] The physical address to be remap to the address assigned
                                                   by the Secure Section register.                                           */
  } b;                                        /*!< bit fields for sce_sce_remap4 */
} sce_sce_remap4_t, *psce_sce_remap4_t;

/**
  \brief Union type to access sce_sce_remap5 (@ 0x00000080).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000080) SCE Secure Section Remap 5 Register                        */

  struct {
    __IOM uint32_t phy_addr   : 32;           /*!< [31..0] The physical address to be remap to the address assigned
                                                   by the Secure Section register.                                           */
  } b;                                        /*!< bit fields for sce_sce_remap5 */
} sce_sce_remap5_t, *psce_sce_remap5_t;

/**
  \brief Union type to access sce_sce_remap6 (@ 0x00000084).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000084) SCE Secure Section Remap 6 Register                        */

  struct {
    __IOM uint32_t phy_addr   : 32;           /*!< [31..0] The physical address to be remap to the address assigned
                                                   by the Secure Section register.                                           */
  } b;                                        /*!< bit fields for sce_sce_remap6 */
} sce_sce_remap6_t, *psce_sce_remap6_t;

/**
  \brief Union type to access sce_sce_remap7 (@ 0x00000088).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x00000088) SCE Secure Section Remap 7 Register                        */

  struct {
    __IOM uint32_t phy_addr   : 32;           /*!< [31..0] The physical address to be remap to the address assigned
                                                   by the Secure Section register.                                           */
  } b;                                        /*!< bit fields for sce_sce_remap7 */
} sce_sce_remap7_t, *psce_sce_remap7_t;

/**
  \brief Union type to access sce_sce_remap8 (@ 0x0000008C).
*/
typedef union {
  __IOM uint32_t w;                           /*!< (@ 0x0000008C) SCE Secure Section Remap 8 Register                        */

  struct {
    __IOM uint32_t phy_addr   : 32;           /*!< [31..0] The physical address to be remap to the address assigned
                                                   by the Secure Section register.                                           */
  } b;                                        /*!< bit fields for sce_sce_remap8 */
} sce_sce_remap8_t, *psce_sce_remap8_t;

/** @} */ /* End of group hal_sce_reg */
/// @endcond /* End of condition DOXYGEN_GENERAL_REG_TYPE || DOXYGEN_SCE_REG_TYPE */


#ifdef  __cplusplus
}
#endif

#endif    // end of #ifndef _RTL8710C_SCE_SCE_TYPE_H_

