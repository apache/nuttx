/**************************************************************************//**
 * @file     rtl8710c_sce.h
 * @brief    The maco, data type, enum definition for the SCE HAL.
 * @version  1.0
 * @date     31. July 2017
 *
 * @note
 *
 ******************************************************************************
 *
 * Copyright(c) 2007 - 2016 Realtek Corporation. All rights reserved.
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

/** @addtogroup hs_hal_sce SCE(Secure Code Engine)
  * @ingroup 8710c_hal
  * @{
  * @brief The Sceure Code Engine(SCE) HAL. This SCE hardware implements the function for the memory
  *        data encryption and/or decryption on the fly.It also implements the flash memory remapping function
  *        to map a contiguous flash memory to a specified virtual address.
  */

#ifndef RTL8710C_SCE_H
#define RTL8710C_SCE_H

#ifdef __cplusplus
extern "C" {
#endif


/// @cond DOXYGEN_REG_ENCLOSED

/** @addtogroup Device_SCE The Secure Code Engine registers.
  * @ingroup hs_hal_sce
  * @{
  */



/* =========================================================================================================================== */
/* ================                                            SCE                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief Secure Code Engine (SCE)
  */

typedef struct {                                /*!< (@ 0x00000000) SCE Structure                                              */

  union {
    __IOM uint32_t sce_ctrl;                    /*!< (@ 0x00000000) SCE Control Register                                       */

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
    } sce_ctrl_b;
  } ;
  __IM  uint32_t  RESERVED[3];

  union {
    __IOM uint32_t sce_aes_key_pair1_w0;        /*!< (@ 0x00000010) SCE AES Key of the Pair 1 Word0 Register                   */

    struct {
      __IOM uint32_t aes_key_w0 : 32;           /*!< [31..0] The key bit[31:0]. The user chose the pair 1 to protect
                                                     the code. When SCE want encrypt or decrypt the data, it
                                                     needs this 128bits key.                                                   */
    } sce_aes_key_pair1_w0_b;
  } ;

  union {
    __IOM uint32_t sce_aes_key_pair1_w1;        /*!< (@ 0x00000014) SCE AES Key of the Pair 1 Word1 Register                   */

    struct {
      __IOM uint32_t aes_key_w1 : 32;           /*!< [31..0] The key bit[63:32]. The user chose the pair 1 to protect
                                                     the code. When SCE want encrypt or decrypt the data, it
                                                     needs this 128bits key.                                                   */
    } sce_aes_key_pair1_w1_b;
  } ;

  union {
    __IOM uint32_t sce_aes_key_pair1_w2;        /*!< (@ 0x00000018) SCE AES Key of the Pair 1 Word2 Register                   */

    struct {
      __IOM uint32_t aes_key_w2 : 32;           /*!< [31..0] The key bit[95:64]. The user chose the pair 1 to protect
                                                     the code. When SCE want encrypt or decrypt the data, it
                                                     needs this 128bits key.                                                   */
    } sce_aes_key_pair1_w2_b;
  } ;

  union {
    __IOM uint32_t sce_aes_key_pair1_w3;        /*!< (@ 0x0000001C) SCE AES Key of the Pair 1 Word3 Register                   */

    struct {
      __IOM uint32_t aes_key_w3 : 32;           /*!< [31..0] The key bit[127:96]. The user chose the pair 1 to protect
                                                     the code. When SCE want encrypt or decrypt the data, it
                                                     needs this 128bits key.                                                   */
    } sce_aes_key_pair1_w3_b;
  } ;

  union {
    __IOM uint32_t sce_aes_iv_pair1_w0;         /*!< (@ 0x00000020) SCE IV Key Word0 of the Pair 1 Register                    */

    struct {
      __IOM uint32_t aes_iv_w0  : 32;           /*!< [31..0] The IV bit[31:0]. The user chose the pair 1 to protect
                                                     the code. When SCE want encrypt or decrypt the data, it
                                                     needs the initial vector generated by SCE_IV_KEY_PAIR_1.                  */
    } sce_aes_iv_pair1_w0_b;
  } ;

  union {
    __IOM uint32_t sce_aes_iv_pair1_w1;         /*!< (@ 0x00000024) SCE IV Key Word1 of the Pair 1 Register                    */

    struct {
      __IOM uint32_t aes_iv_w1  : 32;           /*!< [31..0] The IV bit[63:31]. The user chose the pair 1 to protect
                                                     the code. When SCE want encrypt or decrypt the data, it
                                                     needs the initial vector generated by SCE_IV_KEY_PAIR_1.                  */
    } sce_aes_iv_pair1_w1_b;
  } ;

  union {
    __IOM uint32_t sce_aes_iv_pair1_w2;         /*!< (@ 0x00000028) SCE IV Key Word2 of the Pair 1 Register                    */

    struct {
      __IOM uint32_t aes_iv_w2  : 32;           /*!< [31..0] The IV bit[95:64]. The user chose the pair 1 to protect
                                                     the code. When SCE want encrypt or decrypt the data, it
                                                     needs the initial vector generated by SCE_IV_KEY_PAIR_1.                  */
    } sce_aes_iv_pair1_w2_b;
  } ;

  union {
    __IOM uint32_t sce_aes_iv_pair1_w3;         /*!< (@ 0x0000002C) SCE IV Key Word3 of the Pair 1 Register                    */

    struct {
      __IOM uint32_t aes_iv_w3  : 32;           /*!< [31..0] The IV bit[127:96]. The user chose the pair 1 to protect
                                                     the code. When SCE want encrypt or decrypt the data, it
                                                     needs the initial vector generated by SCE_IV_KEY_PAIR_1.                  */
    } sce_aes_iv_pair1_w3_b;
  } ;

  union {
    __IOM uint32_t sce_aes_key_pair2_w0;        /*!< (@ 0x00000030) SCE AES Key of the Pair 2 Word0 Register                   */

    struct {
      __IOM uint32_t aes_key_w0 : 32;           /*!< [31..0] The key bit[31:0]. The user chose the pair 2 to protect
                                                     the code. When SCE want encrypt or decrypt the data, it
                                                     needs this 128bits key.                                                   */
    } sce_aes_key_pair2_w0_b;
  } ;

  union {
    __IOM uint32_t sce_aes_key_pair2_w1;        /*!< (@ 0x00000034) SCE AES Key of the Pair 2 Word1 Register                   */

    struct {
      __IOM uint32_t aes_key_w1 : 32;           /*!< [31..0] The key bit[63:32]. The user chose the pair 2 to protect
                                                     the code. When SCE want encrypt or decrypt the data, it
                                                     needs this 128bits key.                                                   */
    } sce_aes_key_pair2_w1_b;
  } ;

  union {
    __IOM uint32_t sce_aes_key_pair2_w2;        /*!< (@ 0x00000038) SCE AES Key of the Pair 2 Word2 Register                   */

    struct {
      __IOM uint32_t aes_key_w2 : 32;           /*!< [31..0] The key bit[95:64]. The user chose the pair 2 to protect
                                                     the code. When SCE want encrypt or decrypt the data, it
                                                     needs this 128bits key.                                                   */
    } sce_aes_key_pair2_w2_b;
  } ;

  union {
    __IOM uint32_t sce_aes_key_pair2_w3;        /*!< (@ 0x0000003C) SCE AES Key of the Pair 2 Word3 Register                   */

    struct {
      __IOM uint32_t aes_key_w3 : 32;           /*!< [31..0] The key bit[127:96]. The user chose the pair 2 to protect
                                                     the code. When SCE want encrypt or decrypt the data, it
                                                     needs this 128bits key.                                                   */
    } sce_aes_key_pair2_w3_b;
  } ;

  union {
    __IOM uint32_t sce_aes_iv_pair2_w0;         /*!< (@ 0x00000040) SCE IV Key Word0 of the Pair 2 Register                    */

    struct {
      __IOM uint32_t aes_iv_w0  : 32;           /*!< [31..0] The IV bit[31:0]. The user chose the pair 1 to protect
                                                     the code. When SCE want encrypt or decrypt the data, it
                                                     needs the initial vector generated by SCE_IV_KEY_PAIR_2.                  */
    } sce_aes_iv_pair2_w0_b;
  } ;

  union {
    __IOM uint32_t sce_aes_iv_pair2_w1;         /*!< (@ 0x00000044) SCE IV Key Word1 of the Pair 2 Register                    */

    struct {
      __IOM uint32_t aes_iv_w1  : 32;           /*!< [31..0] The IV bit[63:31]. The user chose the pair 1 to protect
                                                     the code. When SCE want encrypt or decrypt the data, it
                                                     needs the initial vector generated by SCE_IV_KEY_PAIR_2.                  */
    } sce_aes_iv_pair2_w1_b;
  } ;

  union {
    __IOM uint32_t sce_aes_iv_pair2_w2;         /*!< (@ 0x00000048) SCE IV Key Word2 of the Pair 2 Register                    */

    struct {
      __IOM uint32_t aes_iv_w2  : 32;           /*!< [31..0] The IV bit[95:64]. The user chose the pair 1 to protect
                                                     the code. When SCE want encrypt or decrypt the data, it
                                                     needs the initial vector generated by SCE_IV_KEY_PAIR_2.                  */
    } sce_aes_iv_pair2_w2_b;
  } ;

  union {
    __IOM uint32_t sce_aes_iv_pair2_w3;         /*!< (@ 0x0000004C) SCE IV Key Word3 of the Pair 2 Register                    */

    struct {
      __IOM uint32_t aes_iv_w3  : 32;           /*!< [31..0] The IV bit[127:96]. The user chose the pair 1 to protect
                                                     the code. When SCE want encrypt or decrypt the data, it
                                                     needs the initial vector generated by SCE_IV_KEY_PAIR_2.                  */
    } sce_aes_iv_pair2_w3_b;
  } ;

  union {
    __IOM uint32_t sce_sect1;                   /*!< (@ 0x00000050) SCE Secure Section 1 Register                              */

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
    } sce_sect1_b;
  } ;

  union {
    __IOM uint32_t sce_sect2;                   /*!< (@ 0x00000054) SCE Secure Section 2 Register                              */

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
    } sce_sect2_b;
  } ;

  union {
    __IOM uint32_t sce_sect3;                   /*!< (@ 0x00000058) SCE Secure Section 3 Register                              */

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
    } sce_sect3_b;
  } ;

  union {
    __IOM uint32_t sce_sect4;                   /*!< (@ 0x0000005C) SCE Secure Section 4 Register                              */

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
    } sce_sect4_b;
  } ;

  union {
    __IOM uint32_t sce_sect5;                   /*!< (@ 0x00000060) SCE Secure Section 5 Register                              */

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
    } sce_sect5_b;
  } ;

  union {
    __IOM uint32_t sce_sect6;                   /*!< (@ 0x00000064) SCE Secure Section 6 Register                              */

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
    } sce_sect6_b;
  } ;

  union {
    __IOM uint32_t sce_sect7;                   /*!< (@ 0x00000068) SCE Secure Section 7 Register                              */

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
    } sce_sect7_b;
  } ;

  union {
    __IOM uint32_t sce_sect8;                   /*!< (@ 0x0000006C) SCE Secure Section 8 Register                              */

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
    } sce_sect8_b;
  } ;

  union {
    __IOM uint32_t sce_remap1;                  /*!< (@ 0x00000070) SCE Secure Section Remap 1 Register                        */

    struct {
      __IOM uint32_t phy_addr   : 32;           /*!< [31..0] The physical address to be remap to the address assigned
                                                     by the Secure Section register.                                           */
    } sce_remap1_b;
  } ;

  union {
    __IOM uint32_t sce_remap2;                  /*!< (@ 0x00000074) SCE Secure Section Remap 2 Register                        */

    struct {
      __IOM uint32_t phy_addr   : 32;           /*!< [31..0] The physical address to be remap to the address assigned
                                                     by the Secure Section register.                                           */
    } sce_remap2_b;
  } ;

  union {
    __IOM uint32_t sce_remap3;                  /*!< (@ 0x00000078) SCE Secure Section Remap 3 Register                        */

    struct {
      __IOM uint32_t phy_addr   : 32;           /*!< [31..0] The physical address to be remap to the address assigned
                                                     by the Secure Section register.                                           */
    } sce_remap3_b;
  } ;

  union {
    __IOM uint32_t sce_remap4;                  /*!< (@ 0x0000007C) SCE Secure Section Remap 4 Register                        */

    struct {
      __IOM uint32_t phy_addr   : 32;           /*!< [31..0] The physical address to be remap to the address assigned
                                                     by the Secure Section register.                                           */
    } sce_remap4_b;
  } ;

  union {
    __IOM uint32_t sce_remap5;                  /*!< (@ 0x00000080) SCE Secure Section Remap 5 Register                        */

    struct {
      __IOM uint32_t phy_addr   : 32;           /*!< [31..0] The physical address to be remap to the address assigned
                                                     by the Secure Section register.                                           */
    } sce_remap5_b;
  } ;

  union {
    __IOM uint32_t sce_remap6;                  /*!< (@ 0x00000084) SCE Secure Section Remap 6 Register                        */

    struct {
      __IOM uint32_t phy_addr   : 32;           /*!< [31..0] The physical address to be remap to the address assigned
                                                     by the Secure Section register.                                           */
    } sce_remap6_b;
  } ;

  union {
    __IOM uint32_t sce_remap7;                  /*!< (@ 0x00000088) SCE Secure Section Remap 7 Register                        */

    struct {
      __IOM uint32_t phy_addr   : 32;           /*!< [31..0] The physical address to be remap to the address assigned
                                                     by the Secure Section register.                                           */
    } sce_remap7_b;
  } ;

  union {
    __IOM uint32_t sce_remap8;                  /*!< (@ 0x0000008C) SCE Secure Section Remap 8 Register                        */

    struct {
      __IOM uint32_t phy_addr   : 32;           /*!< [31..0] The physical address to be remap to the address assigned
                                                     by the Secure Section register.                                           */
    } sce_remap8_b;
  } ;
} SCE_Type;                                     /*!< Size = 144 (0x90)                                                         */


/** @} */ /* End of group Peripheral_SCE */

/// @endcond /* End of condition DOXYGEN_REG_ENCLOSED */

#include "rtl8710c_sce_type.h"

/// SCE control register offset.
#define REG_SCE_CTRL                (0x0)
/// SCE key pair 1 key data register offset.
#define REG_SCE_AES_KEY_PAIR1       (0x10 >> 2)
/// SCE key pair 1 IV data register offset.
#define REG_SCE_IV_PAIR1            (0x20 >> 2)
/// SCE key pair 2 key data register offset.
#define REG_SCE_AES_KEY_PAIR2       (0x30 >> 2)
/// SCE key pair 2 IV data register offset.
#define REG_SCE_IV_PAIR2            (0x40 >> 2)
/// SCE section 1 control register offset.
#define REG_SCE_SEC1                (0x50 >> 2)
/// SCE section 2 control register offset.
#define REG_SCE_SEC2                (0x54 >> 2)
/// SCE section 3 control register offset.
#define REG_SCE_SEC3                (0x58 >> 2)
/// SCE section 4 control register offset.
#define REG_SCE_SEC4                (0x5C >> 2)
/// SCE section 5 control register offset.
#define REG_SCE_SEC5                (0x60 >> 2)
/// SCE section 6 control register offset.
#define REG_SCE_SEC6                (0x64 >> 2)
/// SCE section 7 control register offset.
#define REG_SCE_SEC7                (0x68 >> 2)
/// SCE section 8 control register offset.
#define REG_SCE_SEC8                (0x6C >> 2)
/// SCE section 1 remapping address register offset.
#define REG_SCE_SEC1_REMAP          (0x70 >> 2)
/// SCE section 2 remapping address register offset.
#define REG_SCE_SEC2_REMAP          (0x74 >> 2)
/// SCE section 3 remapping address register offset.
#define REG_SCE_SEC3_REMAP          (0x78 >> 2)
/// SCE section 4 remapping address register offset.
#define REG_SCE_SEC4_REMAP          (0x7C >> 2)
/// SCE section 5 remapping address register offset.
#define REG_SCE_SEC5_REMAP          (0x80 >> 2)
/// SCE section 6 remapping address register offset.
#define REG_SCE_SEC6_REMAP          (0x84 >> 2)
/// SCE section 7 remapping address register offset.
#define REG_SCE_SEC7_REMAP          (0x88 >> 2)
/// SCE section 8 remapping address register offset.
#define REG_SCE_SEC8_REMAP          (0x8C >> 2)

/// the maximum section index of the SCE hardware. A SCE hardware support up to 8 sections of memory.
#define SCE_MAX_SECTION_ID          7
/// the maximum key pair index value. Since a SCE hardware has 2 key pairs. So the maximum key pair index is 1.
#define SCE_MAX_KEY_PAIR_ID         1

/// the length of SCE key data.
#define SCE_KEY_LEN                 16
/// the length of SCE IV(initial value) data.
#define SCE_IV_LEN                  16

/**
  \brief  Defines the memory type for the SCE function works with.
*/
enum sce_mem_type_e {
    SCE_TYPE_LPDDR      = 0,    ///< Low power DDR DRAM
    SCE_TYPE_PSRAM      = 1,    ///< Pseudo SRAM
    SCE_TYPE_FLASH      = 2     ///< Flash memory
};
typedef uint8_t sce_mem_type_t;

/**
  \brief  Defines the memory operation mode of the SCE function.
*/
enum sce_mode_select_e {
    ReadMode            = 0,    ///< Read only
    WriteMode           = 1,    ///< Write only
    ReadWriteMode       = 2     ///< supports both read and write
};
typedef uint8_t sce_mode_select_t;

/**
  \brief  Defines the memory page size for the SCE operation (encryption, decryption and remapping).
*/
enum sce_page_size_e {
    Page16KB            = 0,    ///< page size = 16K bytes
    Page32KB            = 1,    ///< page size = 32K bytes
    Page64KB            = 2     ///< page size = 64K bytes
};
typedef uint8_t sce_page_size_t;

/**
  \brief  Defines the memory block size for the SCE to do data encryption or decryption.
          The SCE use the same key for the data encryption and decryption in a block.
*/
enum sce_block_size_e {
    Block32Bytes        = 0,    ///< block size = 32 bytes
    Block64Bytes        = 1,    ///< block size = 64 bytes
    Block128Bytes       = 2,    ///< block size = 128 bytes
    Block256Bytes       = 3     ///< block size = 256 bytes
};
typedef uint8_t sce_block_size_t;

/**
  \brief  The data structure for the SCE common resource handling.
*/
typedef struct hal_sce_group_adaptor_s {
    uint8_t  flash_section_en; /*!<  bit map of flash section 1 ~ 8 enable */
//    uint8_t  exmem_section_en; /*!<  bit map of ext. mem 1 ~ 8 enable */
    uint8_t  flash_key_inited; /*!<  bit map of flash key 1 ~ 2 initialed */
//    uint8_t  exmem_key_inited; /*!<  bit map of ext. mem key 1 ~ 2 initialed */
} hal_sce_group_adaptor_t, *phal_sce_group_adaptor_t;

typedef struct hal_sce_check_info_s {
    uint32_t key_check[4] __attribute__((aligned(16)));
    uint32_t iv_check[4] __attribute__((aligned(16)));

} hal_sce_check_info_t, *phal_sce_check_info_t;

/// @cond DOXYGEN_ROM_HAL_API

/**
 * @addtogroup hs_hal_sce_rom_func SCE HAL ROM APIs.
 * @ingroup hs_hal_sce
 * @{
 */

void hal_sce_write_reg_rtl8710c (uint32_t addr, uint32_t value);
uint32_t hal_sce_read_reg_rtl8710c (uint32_t addr);
uint8_t hal_sce_comm_alloc_section_rtl8710c (void);
void hal_sce_comm_free_section_rtl8710c (uint8_t sec_id);
BOOLEAN hal_sce_comm_key_valid_rtl8710c (uint8_t pair_id);
void hal_sce_comm_set_key_valid_rtl8710c (uint8_t pair_id);
hal_status_t hal_sce_func_enable_rtl8710c (void);
hal_status_t hal_sce_func_disable_rtl8710c (void);
hal_status_t hal_sce_enable_rtl8710c (sce_page_size_t page_size,
                                        sce_block_size_t block_size, sce_mode_select_t rw_mode);
hal_status_t hal_sce_disable_rtl8710c (void);
void hal_sce_set_key_rtl8710c (uint8_t pair_id, uint8_t *key);
void hal_sce_set_iv_rtl8710c (uint8_t pair_id, uint8_t *iv);
void hal_sce_set_key_pair_rtl8710c (uint8_t pair_id, uint8_t *key, uint8_t *iv);
void hal_sce_read_key_pair_rtl8710c (uint8_t pair_id, uint8_t *key, uint8_t *iv);
uint8_t hal_sce_key_pair_search_rtl8710c (uint8_t *key, uint8_t *iv);
hal_status_t hal_sce_set_section_rtl8710c (uint8_t sec_id, uint8_t pair_id,
                                    uint32_t start_address, uint32_t end_address, uint8_t secure_en);
void hal_sce_section_disable_rtl8710c (uint8_t sec_id);
hal_status_t hal_sce_flash_remap_rtl8710c (uint32_t phy_addr, uint32_t vir_addr, uint32_t map_size,
                                             sce_page_size_t page_size_sel, uint8_t secure_en,
                                             sce_block_size_t block_size, uint8_t key_id);
hal_status_t hal_sce_set_mem_crypto_rtl8710c (uint32_t start_addr,
                                                uint32_t mem_size, uint8_t key_id);
hal_status_t hal_sce_remap_enable_rtl8710c (uint8_t sec_id, uint32_t phy_addr);
void hal_sce_reg_dump_rtl8710c (void);
hal_status_t hal_sce_cfg_rtl8710c (sce_page_size_t page_size,
                                     sce_block_size_t block_size, sce_mode_select_t rw_mode);
/// @endcond /* End of condition DOXYGEN_ROM_HAL_API */

typedef struct hal_sce_func_stubs_s {
    hal_sce_group_adaptor_t *psce_gpadp;
    void (*hal_sce_write_reg) (uint32_t addr, uint32_t value);
    uint32_t (*hal_sce_read_reg) (uint32_t addr);
    uint8_t (*hal_sce_comm_alloc_section) (void);
    void (*hal_sce_comm_free_section) (uint8_t sec_id);
    BOOLEAN (*hal_sce_comm_key_valid) (uint8_t pair_id);
    hal_status_t (*hal_sce_func_enable) (void);
    hal_status_t (*hal_sce_func_disable) (void);
    hal_status_t (*hal_sce_enable) (sce_page_size_t page_size,
                                    sce_block_size_t block_size, sce_mode_select_t rw_mode);
    hal_status_t (*hal_sce_disable) (void);
    hal_status_t (*hal_sce_cfg) (sce_page_size_t page_size,
                                 sce_block_size_t block_size, sce_mode_select_t rw_mode);
    void (*hal_sce_set_key) (uint8_t pair_id, uint8_t *key);
    void (*hal_sce_set_iv) (uint8_t pair_id, uint8_t *iv);
    void (*hal_sce_set_key_pair) (uint8_t pair_id, uint8_t *key, uint8_t *iv);
    void (*hal_sce_read_key_pair) (uint8_t pair_id, uint8_t *key, uint8_t *iv);
    uint8_t (*hal_sce_key_pair_search) (uint8_t *key, uint8_t *iv);
    hal_status_t (*hal_sce_set_section) (uint8_t sec_id, uint8_t pair_id,
                                        uint32_t start_address, uint32_t end_address, uint8_t secure_en);
    hal_status_t (*hal_sce_remap_enable) (uint8_t sec_id, uint32_t phy_addr);
    void (*hal_sce_section_disable) (uint8_t sec_id);
    hal_status_t (*hal_sce_flash_remap) (uint32_t phy_addr, uint32_t vir_addr, uint32_t map_size,
                                         sce_page_size_t page_size_sel, uint8_t secure_en,
                                         sce_block_size_t block_size, uint8_t key_id);
    hal_status_t (*hal_sce_set_mem_crypto) (uint32_t start_addr,
                                            uint32_t mem_size, uint8_t key_id);
    void (*hal_sce_reg_dump) (void);

    uint32_t reserved[4];
} hal_sce_func_stubs_t;

#ifdef __cplusplus
}
#endif

#endif /* RTL8710C_SCE_H */


/** @} */ /* End of group hal_sce */


