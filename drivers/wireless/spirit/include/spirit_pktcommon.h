/******************************************************************************
 * include/nuttx/wireless/spirit/include/spirit_pktcommon.h
 * Configuration and management of the common features of SPIRIT packets.
 *
 *   Copyright(c) 2015 STMicroelectronics
 *   Author: VMA division - AMS
 *   Version 3.2.2 08-July-2015
 *
 *   Adapted for NuttX by:
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#ifndef __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_PKTCOMMON_H
#define __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_PKTCOMMON_H

/* This module provides all the common functions and definitions used by the
 * packets modules.  Here are also defined all the generic enumeration types
 * that are redefined in the specific packets modules, but every enumeration
 * value is referred to this module. So the user who wants to configure the
 * preamble of a Basic, or a STack packet has to use the enumeration values
 * defined here.
 *
 * Example:
 *
 *   ...
 *
 *   spirit_pktbasic_set_preamblen(PKT_PREAMBLE_LENGTH_18BYTES);
 *
 *   ...
 */

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include "spirit_types.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

/* Macro used to compute the lower part of the packet length, to write in
 * the PCKTLEN0 and BUILD_PCKTLEN1 registers.  len is the length of the
 * packet payload.
 */

#define BUILD_PCKTLEN0(len)            ((len) & 0xff)
#define BUILD_PCKTLEN1(len)            ((len) >> 8)

/* Macros used in assertions */

#define IS_PKT_LENGTH_WIDTH_BITS(bits)  ((bits) <= 16)
#define IS_PKT_SEQ_NUMBER_RELOAD(seqn)  ((seqn) <= 3)

#define IS_PKT_PREAMBLE_LENGTH(len) \
  ((len == PKT_PREAMBLE_LENGTH_01BYTE)  || (len == PKT_PREAMBLE_LENGTH_02BYTES)  || \
   (len == PKT_PREAMBLE_LENGTH_03BYTES) || (len == PKT_PREAMBLE_LENGTH_04BYTES)  || \
   (len == PKT_PREAMBLE_LENGTH_05BYTES) || (len == PKT_PREAMBLE_LENGTH_06BYTES)  || \
   (len == PKT_PREAMBLE_LENGTH_07BYTES) || (len == PKT_PREAMBLE_LENGTH_08BYTES)  || \
   (len == PKT_PREAMBLE_LENGTH_09BYTES) || (len == PKT_PREAMBLE_LENGTH_10BYTES)  || \
   (len == PKT_PREAMBLE_LENGTH_11BYTES) || (len == PKT_PREAMBLE_LENGTH_12BYTES)  || \
   (len == PKT_PREAMBLE_LENGTH_13BYTES) || (len == PKT_PREAMBLE_LENGTH_14BYTES)  || \
   (len == PKT_PREAMBLE_LENGTH_15BYTES) || (len == PKT_PREAMBLE_LENGTH_16BYTES)  || \
   (len == PKT_PREAMBLE_LENGTH_17BYTES) || (len == PKT_PREAMBLE_LENGTH_18BYTES)  || \
   (len == PKT_PREAMBLE_LENGTH_19BYTES) || (len == PKT_PREAMBLE_LENGTH_20BYTES)  || \
   (len == PKT_PREAMBLE_LENGTH_21BYTES) || (len == PKT_PREAMBLE_LENGTH_22BYTES)  || \
   (len == PKT_PREAMBLE_LENGTH_23BYTES) || (len == PKT_PREAMBLE_LENGTH_24BYTES)  || \
   (len == PKT_PREAMBLE_LENGTH_25BYTES) || (len == PKT_PREAMBLE_LENGTH_26BYTES)  || \
   (len == PKT_PREAMBLE_LENGTH_27BYTES) || (len == PKT_PREAMBLE_LENGTH_28BYTES)  || \
   (len == PKT_PREAMBLE_LENGTH_29BYTES) || (len == PKT_PREAMBLE_LENGTH_30BYTES)  || \
   (len == PKT_PREAMBLE_LENGTH_31BYTES) || (len == PKT_PREAMBLE_LENGTH_32BYTES))
#define IS_PKT_SYNC_LENGTH(len) \
  ((len == PKT_SYNC_LENGTH_1BYTE)       || (len == PKT_SYNC_LENGTH_2BYTES)       || \
   (len == PKT_SYNC_LENGTH_3BYTES)      || (len == PKT_SYNC_LENGTH_4BYTES))
#define IS_PKT_FIX_VAR_LENGTH(len) \
  ((len == PKT_LENGTH_FIX)              || (len == PKT_LENGTH_VAR))
#define IS_PKT_CRC_MODE(mode) \
  ((mode == PKT_NO_CRC)                 || (mode == PKT_CRC_MODE_8BITS)          || \
   (mode == PKT_CRC_MODE_16BITS_1)      || (mode == PKT_CRC_MODE_16BITS_2)       || \
   (mode == PKT_CRC_MODE_24BITS))
#define IS_PKT_CONTROL_LENGTH(len) \
  ((len == PKT_CONTROL_LENGTH_0BYTES)   || (len == PKT_CONTROL_LENGTH_1BYTE)     || \
   (len == PKT_CONTROL_LENGTH_2BYTES)    || (len == PKT_CONTROL_LENGTH_3BYTES)   || \
   (len == PKT_CONTROL_LENGTH_4BYTES))

/******************************************************************************
 * Public Types
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Preamble length in bytes enumeration. */

enum pkt_preamblen_e
{
  PKT_PREAMBLE_LENGTH_01BYTE  = 0x00,        /* Preamble length 1 byte */
  PKT_PREAMBLE_LENGTH_02BYTES = 0x08,       /* Preamble length 2 bytes */
  PKT_PREAMBLE_LENGTH_03BYTES = 0x10,       /* Preamble length 3 bytes */
  PKT_PREAMBLE_LENGTH_04BYTES = 0x18,       /* Preamble length 4 bytes */
  PKT_PREAMBLE_LENGTH_05BYTES = 0x20,       /* Preamble length 5 bytes */
  PKT_PREAMBLE_LENGTH_06BYTES = 0x28,       /* Preamble length 6 bytes */
  PKT_PREAMBLE_LENGTH_07BYTES = 0x30,       /* Preamble length 7 bytes */
  PKT_PREAMBLE_LENGTH_08BYTES = 0x38,       /* Preamble length 8 bytes */
  PKT_PREAMBLE_LENGTH_09BYTES = 0x40,       /* Preamble length 9 bytes */
  PKT_PREAMBLE_LENGTH_10BYTES = 0x48,       /* Preamble length 10 bytes */
  PKT_PREAMBLE_LENGTH_11BYTES = 0x50,       /* Preamble length 11 bytes */
  PKT_PREAMBLE_LENGTH_12BYTES = 0x58,       /* Preamble length 12 bytes */
  PKT_PREAMBLE_LENGTH_13BYTES = 0x60,       /* Preamble length 13 bytes */
  PKT_PREAMBLE_LENGTH_14BYTES = 0x68,       /* Preamble length 14 bytes */
  PKT_PREAMBLE_LENGTH_15BYTES = 0x70,       /* Preamble length 15 bytes */
  PKT_PREAMBLE_LENGTH_16BYTES = 0x78,       /* Preamble length 16 bytes */
  PKT_PREAMBLE_LENGTH_17BYTES = 0x80,       /* Preamble length 17 bytes */
  PKT_PREAMBLE_LENGTH_18BYTES = 0x88,       /* Preamble length 18 bytes */
  PKT_PREAMBLE_LENGTH_19BYTES = 0x90,       /* Preamble length 19 bytes */
  PKT_PREAMBLE_LENGTH_20BYTES = 0x98,       /* Preamble length 20 bytes */
  PKT_PREAMBLE_LENGTH_21BYTES = 0xa0,       /* Preamble length 21 bytes */
  PKT_PREAMBLE_LENGTH_22BYTES = 0xa8,       /* Preamble length 22 bytes */
  PKT_PREAMBLE_LENGTH_23BYTES = 0xb0,       /* Preamble length 23 bytes */
  PKT_PREAMBLE_LENGTH_24BYTES = 0xb8,       /* Preamble length 24 bytes */
  PKT_PREAMBLE_LENGTH_25BYTES = 0xc0,       /* Preamble length 25 bytes */
  PKT_PREAMBLE_LENGTH_26BYTES = 0xc8,       /* Preamble length 26 bytes */
  PKT_PREAMBLE_LENGTH_27BYTES = 0xd0,       /* Preamble length 27 bytes */
  PKT_PREAMBLE_LENGTH_28BYTES = 0xd8,       /* Preamble length 28 bytes */
  PKT_PREAMBLE_LENGTH_29BYTES = 0xe0,       /* Preamble length 29 bytes */
  PKT_PREAMBLE_LENGTH_30BYTES = 0xe8,       /* Preamble length 30 bytes */
  PKT_PREAMBLE_LENGTH_31BYTES = 0xf0,       /* Preamble length 31 bytes */
  PKT_PREAMBLE_LENGTH_32BYTES = 0xf8        /* Preamble length 32 bytes */
};

/* Sync length in bytes enumeration. */

enum pkt_synlen_e
{
  PKT_SYNC_LENGTH_1BYTE       = 0x00,       /* Sync length 1 byte */
  PKT_SYNC_LENGTH_2BYTES      = 0x02,       /* Sync length 2 bytes */
  PKT_SYNC_LENGTH_3BYTES      = 0x04,       /* Sync length 3 bytes */
  PKT_SYNC_LENGTH_4BYTES      = 0x06,       /* Sync length 4 bytes */
};

/* Fixed or variable payload length enumeration. */

enum pkt_fixvar_len_e
{
  PKT_LENGTH_FIX              = 0x00,       /* Fixed payload length */
  PKT_LENGTH_VAR              = 0x01        /* Variable payload length */
};

/* CRC length in bytes enumeration. */

enum pkt_crcmode_e
{
  PKT_NO_CRC                  = 0x00,       /* No CRC */
  PKT_CRC_MODE_8BITS          = 0x20,       /* CRC length 8 bits - poly: 0x07 */
  PKT_CRC_MODE_16BITS_1       = 0x40,       /* CRC length 16 bits - poly: 0x8005 */
  PKT_CRC_MODE_16BITS_2       = 0x60,       /* CRC length 16 bits - poly: 0x1021  */
  PKT_CRC_MODE_24BITS         = 0x80,       /* CRC length 24 bits - poly: 0x864CFB */
};

/* Control length in bytes enumeration for SPIRIT packets. */

enum pkt_ctrllen_e
{
  PKT_CONTROL_LENGTH_0BYTES   = 0x00,      /* Control length 0 byte */
  PKT_CONTROL_LENGTH_1BYTE,                /* Control length 1 byte */
  PKT_CONTROL_LENGTH_2BYTES,               /* Control length 2 bytes */
  PKT_CONTROL_LENGTH_3BYTES,               /* Control length 3 bytes */
  PKT_CONTROL_LENGTH_4BYTES                /* Control length 4 bytes */
};

/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_pktcommon_set_controllen
 *
 * Description:
 *   Sets the CONTROL field length for SPIRIT packets.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   ctrllen - Length of CONTROL field in bytes.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_set_controllen(FAR struct spirit_library_s *spirit,
                                    enum pkt_ctrllen_e ctrllen);

/******************************************************************************
 * Name: spirit_pktcommon_get_controllen
 *
 * Description:
 *   Returns the CONTROL field length for SPIRIT packets.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Control field length.
 *
 ******************************************************************************/

uint8_t spirit_pktcommon_get_controllen(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_enable_crcfilter
 *
 * Description:
 *   Enables or Disables the filtering on CRC.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for CRC_CHECK.  This parameter can be S_ENABLE or
 *              S_DISABLE.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_enable_crcfilter(FAR struct spirit_library_s *spirit,
                                      enum spirit_functional_state_e newstate);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_PKTCOMMON_H */
