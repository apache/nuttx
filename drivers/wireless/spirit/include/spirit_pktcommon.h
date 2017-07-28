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
   (len == PKT_CONTROL_LENGTH_2BYTES)   || (len == PKT_CONTROL_LENGTH_3BYTES)    || \
   (len == PKT_CONTROL_LENGTH_4BYTES))
#define IS_PKT_SYNCWORD(word) \
  (((word) == PKT_SYNC_WORD_1)          || ((word) == PKT_SYNC_WORD_2)           || \
   ((word) == PKT_SYNC_WORD_3)          || ((word) == PKT_SYNC_WORD_4))
#define IS_PKT_MAXRETX(nretx) \
  (((nretx) == PKT_DISABLE_RETX)        || ((nretx) == PKT_N_RETX_1)             || \
   ((nretx) == PKT_N_RETX_2)            || ((nretx) == PKT_N_RETX_3)             || \
   ((nretx) == PKT_N_RETX_4)            || ((nretx) == PKT_N_RETX_5)             || \
   ((nretx) == PKT_N_RETX_6)            || ((nretx) == PKT_N_RETX_7)             || \
   ((nretx) == PKT_N_RETX_8)            || ((nretx) == PKT_N_RETX_9)             || \
   ((nretx) == PKT_N_RETX_10)           || ((nretx) == PKT_N_RETX_11)            || \
   ((nretx) == PKT_N_RETX_12)           || ((nretx) == PKT_N_RETX_13)            || \
   ((nretx) == PKT_N_RETX_14)           || ((nretx) == PKT_N_RETX_15))

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
  PKT_PREAMBLE_LENGTH_01BYTE  = 0x00,       /* Preamble length 1 byte */
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

/* Sync words enumeration for SPIRIT packets. */

enum spirit_pktsyncword_e
{
  PKT_SYNC_WORD_1             = 0x01,     /* Index of the 1st sync word */
  PKT_SYNC_WORD_2,                        /* Index of the 2nd sync word */
  PKT_SYNC_WORD_3,                        /* Index of the 3rd sync word */
  PKT_SYNC_WORD_4                         /* Index of the 4th sync word */
};

/* Max retransmissions number enumeration for SPIRIT packets. */

enum spirit_maxretx_e
{
  PKT_DISABLE_RETX            = 0x00,     /* No retrasmissions */
  PKT_N_RETX_1                = 0x10,     /* Max retrasmissions 1 */
  PKT_N_RETX_2                = 0x20,     /* Max retrasmissions 2 */
  PKT_N_RETX_3                = 0x30,     /* Max retrasmissions 3 */
  PKT_N_RETX_4                = 0x40,     /* Max retrasmissions 4 */
  PKT_N_RETX_5                = 0x50,     /* Max retrasmissions 5 */
  PKT_N_RETX_6                = 0x60,     /* Max retrasmissions 6 */
  PKT_N_RETX_7                = 0x70,     /* Max retrasmissions 7 */
  PKT_N_RETX_8                = 0x80,     /* Max retrasmissions 8 */
  PKT_N_RETX_9                = 0x90,     /* Max retrasmissions 9 */
  PKT_N_RETX_10               = 0xa0,     /* Max retrasmissions 10 */
  PKT_N_RETX_11               = 0xb0,     /* Max retrasmissions 11 */
  PKT_N_RETX_12               = 0xc0,     /* Max retrasmissions 12 */
  PKT_N_RETX_13               = 0xd0,     /* Max retrasmissions 13 */
  PKT_N_RETX_14               = 0xe0,     /* Max retrasmissions 14 */
  PKT_N_RETX_15               = 0xf0      /* Max retrasmissions 15 */
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
 * Name: spirit_pktcommon_set_preamblen
 *
 * Description:
 *   Sets the PREAMBLE field Length mode for SPIRIT packets.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   preamblen - Length of PREAMBLE field in bytes.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_set_preamblen(FAR struct spirit_library_s *spirit,
                                   enum pkt_preamblen_e preamblen);

/******************************************************************************
 * Name: spirit_pktcommon_get_preamblen
 *
 * Description:
 *   Returns the PREAMBLE field Length mode for SPIRIT packets.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Preamble field length in bytes.
 *
 ******************************************************************************/

uint8_t spirit_pktcommon_get_preamblen(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pkt_set_synclen
 *
 * Description:
 *   Sets the SYNC field Length for SPIRIT packets.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   synclen - Length of SYNC field in bytes.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pkt_set_synclen(FAR struct spirit_library_s *spirit,
                           enum pkt_synlen_e synclen);

/******************************************************************************
 * Name: spirit_pktcommon_get_synclen
 *
 * Description:
 *   Returns the SYNC field Length for SPIRIT packets.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Sync field length in bytes.
 *
 ******************************************************************************/

uint8_t spirit_pktcommon_get_synclen(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_set_fixvarlen
 *
 * Description:
 *   Sets fixed or variable payload length mode for SPIRIT packets.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   fixvarlen - Variable or fixed length.
 *                PKT_FIXED_LENGTH_VAR -> variable (the length is extracted
 *                  from the received packet).
 *                PKT_FIXED_LENGTH_FIX -> fix (the length is set by PCKTLEN0
 *                  and PCKTLEN1).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_set_fixvarlen(FAR struct spirit_library_s *spirit,
                                   enum pkt_fixvar_len_e fixvarlen);

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

/******************************************************************************
 * Name: spirit_pktcommon_isenabled_crcfilter
 *
 * Description:
 *   Returns the CRC filtering enable bit.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   CRC filtering state.
 *
 ******************************************************************************/

enum spirit_functional_state_e
  spirit_pktcommon_isenabled_crcfilter(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_set_crcmode
 *
 * Description:
 *   Sets the CRC mode for SPIRIT packets.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   crcmode - Length of CRC field in bytes.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_set_crcmode(FAR struct spirit_library_s *spirit,
                                 enum pkt_crcmode_e crcmode);

/******************************************************************************
 * Name: spirit_pktcommon_get_crcmode
 *
 * Description:
 *   Returns the CRC mode for SPIRIT packets.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Crc mode.
 *
 ******************************************************************************/

enum pkt_crcmode_e
  spirit_pktcommon_get_crcmode(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_enable_whitening
 *
 * Description:
 *   Enables or Disables WHITENING for SPIRIT packets.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for WHITENING mode.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_enable_whitening(FAR struct spirit_library_s *spirit,
                                      enum spirit_functional_state_e newstate);

/******************************************************************************
 * Name: spirit_pktcommon_enable_fec
 *
 * Description:
 *   Enables or Disables FEC for SPIRIT packets.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for FEC mode.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_enable_fec(FAR struct spirit_library_s *spirit,
                                enum spirit_functional_state_e newstate);

/******************************************************************************
 * Name: spirit_pktcommon_set_syncword
 *
 * Description:
 *   Sets a specific SYNC word for SPIRIT packets.
 *
 * Input Parameters:
 *   spirit     - Reference to a Spirit library state structure instance
 *   syncwordno - SYNC word number to be set.
 *   syncword   - SYNC word.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_set_syncword(FAR struct spirit_library_s *spirit,
                                  enum spirit_pktsyncword_e syncwordno,
                                  uint8_t syncword);

/******************************************************************************
 * Name: spirit_pktcommon_get_syncword
 *
 * Description:
 *   Returns a specific SYNC word for SPIRIT packets.
 *
 * Input Parameters:
 *   spirit     - Reference to a Spirit library state structure instance
 *   syncwordno - SYNC word number to be get.
 *
 * Returned Value:
 *   Sync word x.
 *
 ******************************************************************************/

uint8_t spirit_pktcommon_get_syncword(FAR struct spirit_library_s *spirit,
                                      enum spirit_pktsyncword_e syncwordno);

/******************************************************************************
 * Name: spirit_pktcommon_set_syncwords
 *
 * Description:
 *   Sets multiple SYNC words for SPIRIT packets.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   syncwords - SYNC words to be set with format: 0x|SYNC1|SYNC2|SYNC3|SYNC4|.
 *   synclen   - SYNC length in bytes. The 32bit word passed will be stored in
 *               the SYNCx registers from the MSb until the number of bytes in
 *               synclen has been stored.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_set_syncwords(FAR struct spirit_library_s *spirit,
                                   uint32_t syncwords, enum pkt_synlen_e synclen);

/******************************************************************************
 * Name: spirit_pktcommon_get_syncwords
 *
 * Description:
 *   Returns multiple SYNC words for SPIRIT packets.
 *
 * Input Parameters:
 *   spiri   - Reference to a Spirit library state structure instance
 *   synclen - SYNC length in bytes. The 32bit word passed will be stored in
 *             the SYNCx registers from the MSb until the number of bytes in
 *             synclen has been stored.
 *
 * Returned Value:
 *   Sync words. The format of the read 32 bit word is 0x|SYNC1|SYNC2|SYNC3|SYNC4|.
 *
 ******************************************************************************/

uint32_t spirit_pktcommon_get_syncwords(FAR struct spirit_library_s *spirit,
                                        enum pkt_synlen_e synclen);

/******************************************************************************
 * Name: spirit_pktcommon_get_varlen
 *
 * Description:
 *   Returns the variable length width (in number of bits).
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Variable length width in bits.
 *
 ******************************************************************************/

uint8_t spirit_pktcommon_get_varlen(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_set_txdestaddr
 *
 * Description:
 *   Sets the destination address for the Tx packet.
 *
 * Input Parameters:
 *   spirit     - Reference to a Spirit library state structure instance
 *   txdestaddr - Destination address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_set_txdestaddr(FAR struct spirit_library_s *spirit,
                                    uint8_t txdestaddr);

/******************************************************************************
 * Name: spirit_pktcommon_get_txdestaddr
 *
 * Description:
 *   Returns the settled destination address.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Transmitted destination address.
 *
 ******************************************************************************/

uint8_t spirit_pktcommon_get_txdestaddr(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_set_nodeaddress
 *
 * Description:
 *   Sets the node my address. When the filtering on my address is on, if the
 *   destination address extracted from the received packet is equal to the
 *   content of the my address, then the packet is accepted (this is the
 *   address of the node).
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   srcaddr - Address of the present node.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_set_nodeaddress(FAR struct spirit_library_s *spirit,
                                     uint8_t srcaddr);

/******************************************************************************
 * Name: spirit_pktcommon_get_nodeaddress
 *
 * Description:
 *   Returns the address of the present node.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   My address (address of this node).
 *
 ******************************************************************************/

uint8_t spirit_pktcommon_get_nodeaddress(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_set_bcastaddr
 *
 * Description:
 *   Sets the broadcast address. If the destination address extracted from the
 *   received packet is equal to the content of the BROADCAST_ADDR register,
 *   then the packet is accepted.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   bcastaddr - Broadcast address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_set_bcastaddr(FAR struct spirit_library_s *spirit,
                                   uint8_t bcastaddr);

/******************************************************************************
 * Name: spirit_pktcommon_get_bcastaddr
 *
 * Description:
 *   Returns the broadcast address.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Broadcast address.
 *
 ******************************************************************************/

uint8_t spirit_pktcommon_get_bcastaddr(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_set_mcastaddr
 *
 * Description:
 *   Sets the multicast address. When the multicast filtering is on, if the
 *   destination address extracted from the received packet is equal to the
 *   content of the MULTICAST_ADDR register, then the packet is accepted.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   mcastaddr - Multicast address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_set_mcastaddr(FAR struct spirit_library_s *spirit,
                                   uint8_t mcastaddr);

/******************************************************************************
 * Name: spirit_pktcommon_get_mcastaddr
 *
 * Description:
 *   Returns the multicast address.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Multicast address.
 *
 ******************************************************************************/

uint8_t spirit_pktcommon_get_mcastaddr(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_set_ctrlmask
 *
 * Description:
 *   Sets the control mask. The 1 bits of the CONTROL_MASK indicate the bits
 *   to be used in filtering. (All 0s no filtering)
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   mask Control mask.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_set_ctrlmask(FAR struct spirit_library_s *spirit,
                                  uint32_t mask);

/******************************************************************************
 * Name: spirit_pktcommon_get_ctrlmask
 *
 * Description:
 *   Returns the control mask. The 1 bits of the CONTROL_MASK indicate the
 *   bits to be used in filtering. (All 0s no filtering)
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Control mask.
 *
 ******************************************************************************/

uint32_t spirit_pktcommon_get_ctrlmask(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_set_ctrlref
 *
 * Description:
 *   Sets the control field reference. If the bits enabled by the CONTROL_MASK
 *   match the ones of the control fields extracted from the received packet
 *   then the packet is accepted.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   reference - Control reference.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_set_ctrlref(FAR struct spirit_library_s *spirit,
                                 uint32_t reference);

/******************************************************************************
 * Name: spirit_pktcommon_get_ctrlref
 *
 * Description:
 *   Returns the control field reference.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Control reference.
 *
 ******************************************************************************/

uint32_t spirit_pktcommon_get_ctrlref(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_set_txctrl
 *
 * Description:
 *   Sets the TX control field.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   txctrl - Tx control field.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_set_txctrl(FAR struct spirit_library_s *spirit,
                                uint32_t txctrl);

/******************************************************************************
 * Name: spirit_pktcommon_get_txctrl
 *
 * Description:
 *   Returns the Tx control field.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Control field of the transmitted packet.
 *
 ******************************************************************************/

uint32_t spirit_pktcommon_get_txctrl(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_enable_destaddr_filter
 *
 * Description:
 *   If enabled RX packet is accepted if its destination address matches with
 *   My address.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for DEST_VS_SOURCE_ADDRESS.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_enable_destaddr_filter(FAR struct spirit_library_s *spirit,
                                            enum spirit_functional_state_e newstate);

/******************************************************************************
 * Name: spirit_pktcommon_enable_mcastaddr_filter
 *
 * Description:
 *   If enabled RX packet is accepted if its destination address matches with
 *   multicast address.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for DEST_VS_MULTICAST_ADDRESS.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_enable_mcastaddr_filter(FAR struct spirit_library_s *spirit,
                                             enum spirit_functional_state_e newstate);

/******************************************************************************
 * Name: spirit_pktcommon_enable_bcastaddr_filter
 *
 * Description:
 *   If enabled RX packet is accepted if its destination address matches with
 *   broadcast address.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for DEST_VS_BROADCAST_ADDRESS.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_enable_bcastaddr_filter(FAR struct spirit_library_s *spirit,
                                             enum spirit_functional_state_e newstate);

/******************************************************************************
 * Name: spirit_pktcommon_isenabled_destaddr_filter
 *
 * Description:
 *   Returns the enable bit of the my address filtering.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   S_ENABLE or S_DISABLE.
 *
 ******************************************************************************/

enum spirit_functional_state_e
  spirit_pktcommon_isenabled_destaddr_filter(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_isenabled_mcastaddr_filter
 *
 * Description:
 *   Returns the enable bit of the multicast address filtering.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   S_ENABLE or S_DISABLE.
 *
 ******************************************************************************/

enum spirit_functional_state_e
  spirit_pktcommon_isenabled_mcastaddr_filter(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_isenabled_bcastaddr_filter
 *
 * Description:
 *   Returns the enable bit of the broadcast address filtering.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   S_ENABLE or S_DISABLE.
 *
 ******************************************************************************/

enum spirit_functional_state_e
  spirit_pktcommon_isenabled_bcastaddr_filter(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_get_rxdestaddr
 *
 * Description:
 *   Returns the destination address of the received packet.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Destination address of the received address.
 *
 ******************************************************************************/

uint8_t spirit_pktcommon_get_rxdestaddr(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_get_rxctrl
 *
 * Description:
 *   Returns the control field of the received packet.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Received control field.
 *
 ******************************************************************************/

uint32_t spirit_pktcommon_get_rxctrl(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_get_rxcrc
 *
 * Description:
 *   Returns the CRC field of the received packet.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   crc    - Array in which the CRC field has to be stored.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_get_rxcrc(FAR struct spirit_library_s *spirit,
                               FAR uint8_t *crc);

/******************************************************************************
 * Name: spirit_pktcommon_enable_rxautoack
 *
 * Description:
 *   Sets the AUTO ACKNOLEDGEMENT mechanism on the receiver. When the feature
 *   is enabled and a data packet has been correctly received, then an
 *   acknowledgement packet is sent back to the originator of the received
 *   packet. If the PIGGYBACKING bit is also set, payload data will be read
 *   from the FIFO; otherwise an empty packet is sent only containing the
 *   source and destination addresses and the sequence number of the packet
 *   being acknowledged.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   autoack   - New state for autoack.
 *   piggyback - New state for autoack.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_enable_rxautoack(FAR struct spirit_library_s *spirit,
                                      enum spirit_functional_state_e autoack,
                                      enum spirit_functional_state_e piggyback);

/******************************************************************************
 * Name: spirit_pktcommon_enable_txautoack
 *
 * Description:
 *   Sets the AUTO ACKNOLEDGEMENT mechanism on the transmitter. On the
 *   transmitter side, the NACK_TX field can be used to require or not an
 *   acknowledgment for each individual packet: if NACK_TX is set to "1" then
 *   acknowledgment will not be required; if NACK_TX is set to "0" then
 *   acknowledgment will be required.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for TX_AUTOACK.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_enable_txautoack(FAR struct spirit_library_s *spirit,
                                      enum spirit_functional_state_e newstate);

/******************************************************************************
 * Name: spirit_pktcommon_set_txseqno
 *
 * Description:
 *   Sets the TX sequence number to be used to start counting.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   seqno  - New value for Tx seq number reload.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_set_txseqno(FAR struct spirit_library_s *spirit,
                                 uint8_t seqno);

/******************************************************************************
 * Name: spirit_pktcommon_set_maxretx
 *
 * Description:
 *   Set the maximum number of TX retries (from 0 to 15).
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   seqno  - New value for Tx seq number reload.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_set_maxretx(FAR struct spirit_library_s *spirit,
                                 enum spirit_maxretx_e maxretx);

/******************************************************************************
 * Name: spirit_pktcommon_get_maxretx
 *
 * Description:
 *   Returns the max number of automatic retransmission.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Max number of retransmissions.
 *
 ******************************************************************************/

uint8_t spirit_pktcommon_get_maxretx(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_get_txackreq
 *
 * Description:
 *   Returns the TX ACK request
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Max number of retransmissions.
 *
 ******************************************************************************/

enum spirit_functional_state_e
  spirit_pktcommon_get_txackreq(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_get_rxsrcaddr
 *
 * Description:
 *   Returns the source address of the received packet.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Source address of the received packet.
 *
 ******************************************************************************/

uint8_t spirit_pktcommon_get_rxsrcaddr(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_get_rxseqno
 *
 * Description:
 *   Returns the sequence number of the received packet.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Received Sequence number.
 *
 ******************************************************************************/

uint8_t spirit_pktcommon_get_rxseqno(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_get_rxnak
 *
 * Description:
 *   Returns the Nack bit of the received packet
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Value of the Nack bit.
 *
 ******************************************************************************/

uint8_t spirit_pktcommon_get_rxnak(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_get_txseqno
 *
 * Description:
 *   Returns the sequence number of the transmitted packet.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Sequence number of the transmitted packet.
 *
 ******************************************************************************/

uint8_t spirit_pktcommon_get_txseqno(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_get_nretx
 *
 * Description:
 *   Returns the number of retransmission done on the transmitted packet.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Number of retransmissions done until now.
 *
 ******************************************************************************/

uint8_t spirit_pktcommon_get_nretx(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktcommon_enable_ctrl_filter
 *
 * Description:
 *   If enabled RX packet is accepted only if the masked control field matches
 *   the masked control field reference (CONTROL_MASK & CONTROL_FIELD_REF ==
 *   CONTROL_MASK & RX_CONTROL_FIELD).
 *
 *   NOTE: This filtering control is enabled by default but the control mask is
 *   by default set to 0.  As a matter of fact the user has to enable the
 *   control filtering bit after the packet initialization because the packet
 *   initialization routine disables it.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for Control filtering enable bit.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktcommon_enable_ctrl_filter(FAR struct spirit_library_s *spirit,
                                        enum spirit_functional_state_e newstate);

/******************************************************************************
 * Name: spirit_pktcommon_isenabled_ctrl_filter
 *
 * Description:
 *   Returns the enable bit of the control field filtering.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   S_ENABLE or S_DISABLE.
 *
 ******************************************************************************/

enum spirit_functional_state_e
  spirit_pktcommon_isenabled_ctrl_filter(FAR struct spirit_library_s *spirit);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_PKTCOMMON_H */
