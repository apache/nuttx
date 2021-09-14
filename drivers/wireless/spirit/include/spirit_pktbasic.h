/******************************************************************************
 * drivers/wireless/spirit/include/spirit_pktbasic.h
 *
 *   Copyright(c) 2015 STMicroelectronics
 *   Author: VMA division - AMS
 *   Version 3.2.2 08-July-2015
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#ifndef __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_PKTBASIC_H
#define __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_PKTBASIC_H

/* This module can be used to manage the configuration of Spirit Basic
 * packets.
 * The user can obtain a packet configuration filling the structure struct
 * spirit_pktbasic_init_s, defining in it some general parameters for the
 * Spirit Basic packet format.  Another structure the user can fill is struct
 * spirit_pktbasic_addr_s to define the addresses which will be used during
 * the communication.  In addition, functions to set the payload length and
 * the destination address are provided.
 *
 * Example:
 *
 * struct spirit_pktbasic_init_s g_pkbasic_init =
 * {
 *   PKT_PREAMBLE_LENGTH_08BYTES,       # preamble length in bytes
 *   PKT_SYNC_LENGTH_4BYTES,            # sync word length in bytes
 *   0x1A2635A8,                        # sync word
 *   PKT_LENGTH_VAR,                    # variable or fixed payload length
 *   7,                                 # length field width in bits
 *                                        (used only for variable length)
 *   PKT_NO_CRC,                        # CRC mode
 *   PKT_CONTROL_LENGTH_0BYTES,         # control field length
 *   S_ENABLE,                          # address field
 *   S_DISABLE,                         # FEC
 *   S_ENABLE                           # whitening
 * };
 *
 * struct spirit_pktbasic_addr_s g_pktbasic_addrinit =
 * {
 *   S_ENABLE,                 # enable/disable filtering on my address
 *   0x34,                     # my address (address of the current node)
 *   S_DISABLE,                # enable/disable filtering on multicast address
 *   0xee,                     # multicast address
 *   S_DISABLE,                # enable/disable filtering on broadcast address
 *   0xff                      # broadcast address
 * };
 *
 * ...
 *
 * spirit_pktbasic_initialize(spirit, &g_pkbasic_init);
 * spirit_pktbasic_addr_initialize(spirit, &g_pktbasic_addrinit);
 *
 * ...
 *
 * spirit_pktbasic_set_payloadlen(spirit, 20);
 * spirit_pktcommon_set_nodeaddress(spirit, 0x44);
 *
 * ...
 *
 * The module provides some other functions that can be used to modify or
 * read only some configuration parameters.
 */

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include "spirit_types.h"
#include "spirit_pktcommon.h"

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

#define IS_BASIC_LENGTH_WIDTH_BITS  IS_PKT_LENGTH_WIDTH_BITS

/* Macros used in assertions */

/******************************************************************************
 * Public Types
 ******************************************************************************/

/* SPIRIT Basic Packet Init structure definition. This structure allows
 * users to set the main options for the Basic packet.
 */

struct spirit_pktbasic_init_s
{
  uint32_t syncwords;    /* Specifies the sync words. This parameter is
                          * a uint32_t word with format:
                          * 0x|SYNC1|SYNC2|SYNC3|SYNC4 */
  uint8_t preamblen;     /* Specifies the preamble length. This parameter
                          * can be any value from enum pkt_preamblen_e */
  uint8_t synclen;       /* Specifies the sync word length.  The 32bit
                          * word passed (syncwords) will be stored in
                          * the SYNCx registers from the MSB until the
                          * number of bytes in synclen has been stored.
                          * This parameter can be any value of enum
                          * pkt_synlen_e */
  uint8_t fixvarlen;     /* Specifies if a fixed length of packet has to
                          * be used. This parameter can be any value of
                          * enum pkt_fixvar_len_e */
  uint8_t pktlenwidth;   /* Specifies the size of the length of packet
                          * in bits. This field is useful only if the
                          * field fixvarlen is set to BASIC_LENGTH_VAR.
                          * For Basic packets the length width is
                          * log2(max payload length + control length
                          * (0 to 4) + address length (0 or 1)). */
  uint8_t crcmode;       /* Specifies the CRC word length of packet.
                          * This parameter can be any value of enum
                          * pkt_crcmode_e */
  uint8_t ctrllen;       /* Specifies the length of a control field to
                          * be sent. This parameter can be any value
                          * from enum pkt_ctrllen_e */
  uint8_t txdestaddr;    /* Specifies if the destination address has to
                          * be sent. This parameter can be S_ENABLE or
                          * S_DISABLE */
  uint8_t fec;           /* Specifies if FEC has to be enabled. This
                          * parameter can be S_ENABLE or S_DISABLE */
  uint8_t datawhite;     /* Specifies if data whitening has to be
                          * enabled. This parameter can be S_ENABLE or
                          * S_DISABLE */
};

/* SPIRIT Basic Packet address structure definition. This structure allows
 * users to specify the node/multicast/broadcast addresses and the
 * correspondent filtering options.
 */

struct spirit_pktbasic_addr_s
{
  uint8_t destfilter;    /* If set RX packet is accepted if its destination
                          * address matches with srcaddr. This parameter
                          * can be S_ENABLE or S_DISABLE */
  uint8_t srcaddr;       /* Specifies the TX packet source address (address
                          * of this node).  */
  uint8_t mcastfilter;   /* If set RX packet is accepted if its destination
                          * address matches with mcastaddr. This parameter
                          * can be S_ENABLE or S_DISABLE */
  uint8_t mcastaddr;     /* Specifies the Multicast group address for this
                          * node. */
  uint8_t bcastfilter;   /* If set RX packet is accepted if its destination
                          * address matches with bcastaddr. This parameter
                          * can be S_ENABLE or S_DISABLE */
  uint8_t bcastaddr;     /* Specifies the Broadcast address for this node. */
};

/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_pktbasic_initialize
 *
 * Description:
 *   Initializes the Spirit Basic packet according to the specified parameters
 *   in the struct spirit_pktbasic_init_s.  Notice that this function sets the
 *   autofiltering option on CRC if it is set to any value different from
 *   BASIC_NO_CRC.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   pktbasic - Basic packet init structure.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktbasic_initialize(FAR struct spirit_library_s *spirit,
                        FAR const struct spirit_pktbasic_init_s *pktbasic);

/******************************************************************************
 * Name: spirit_pktbasic_get_setup
 *
 * Description:
 *   Returns the SPIRIT Basic packet structure according to the specified
 *   parameters in the registers.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   pktbasic - Basic packet init structure.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktbasic_get_setup(FAR struct spirit_library_s *spirit,
                              FAR struct spirit_pktbasic_init_s *pktbasic);

/******************************************************************************
 * Name: spirit_pktbasic_addr_initialize
 *
 * Description:
 *   Initializes the SPIRIT Basic packet addresses according to the specified
 *   parameters in the struct spirit_pktbasic_init_s struct.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   basicaddr - Basic packet addresses init structure.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktbasic_addr_initialize(FAR struct spirit_library_s *spirit,
                                FAR struct spirit_pktbasic_addr_s *basicaddr);

/******************************************************************************
 * Name: spirit_pktbasic_get_addrsetup
 *
 * Description:
 *   Returns the SPIRIT Basic packet addresses structure according to the
 *   specified parameters in the registers.
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   basicaddr - Basic packet addresses init structure.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktbasic_get_addrsetup(FAR struct spirit_library_s *spirit,
                            FAR struct spirit_pktbasic_addr_s *basicaddr);

/******************************************************************************
 * Name: spirit_pktbasic_set_format
 *
 * Description:
 *   Configures the Basic packet format as packet used by SPIRIT.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktbasic_set_format(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pkbasic_enable_addrlen
 *
 * Description:
 *   Sets the address length for SPIRIT Basic packets.
 *
 * Input Parameters:
 *   spirit     - Reference to a Spirit library state structure instance
 *   txdestaddr - Length of ADDRESS in bytes.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pkbasic_enable_addrlen(FAR struct spirit_library_s *spirit,
                                  enum spirit_functional_state_e txdestaddr);

/******************************************************************************
 * Name: spirit_pkbasic_isenabled_addrlen
 *
 * Description:
 *   Specifies if the Address field for SPIRIT Basic packets is enabled or
 *   disabled.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Notifies if the address field is enabled or disabled.
 *
 ******************************************************************************/

enum spirit_functional_state_e
  spirit_pkbasic_isenabled_addrlen(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktbasic_set_payloadlen
 *
 * Description:
 *   Sets the payload length for SPIRIT Basic packets. Since the packet length
 *   depends from the address and the control field size, this function reads
 *   the correspondent registers in order to determine the correct packet
 *   length to be written.
 *
 * Input Parameters:
 *   spirit     - Reference to a Spirit library state structure instance
 *   payloadlen - Payload length in bytes.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_pktbasic_set_payloadlen(FAR struct spirit_library_s *spirit,
                                   uint16_t payloadlen);

/******************************************************************************
 * Name: spirit_pktbase_get_payloadlen
 *
 * Description:
 *   Returns the payload length for SPIRIT Basic packets. Since the packet
 *   length depends from the address and the control field size, this function
 *   reads the correspondent registers in order to determine the correct
 *   payload length to be returned.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Payload length in bytes.
 *
 ******************************************************************************/

uint16_t spirit_pktbase_get_payloadlen(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktbasic_get_rxpktlen
 *
 * Description:
 *   Returns the packet length field of the received packet.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Rx Packet length.
 *
 ******************************************************************************/

uint16_t spirit_pktbasic_get_rxpktlen(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktbasic_set_varlen
 *
 * Description:
 *   Computes and sets the variable payload length for SPIRIT Basic packets.
 *
 * Input Parameters:
 *   spirit     - Reference to a Spirit library state structure instance
 *   payloadlen - Payload length in bytes.
 *   txdestaddr - Enable or Disable Address Field.
 *   ctrllen    - Control length in bytes.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_pktbasic_set_varlen(FAR struct spirit_library_s *spirit,
                               uint16_t payloadlen,
                               enum spirit_functional_state_e txdestaddr,
                               enum pkt_ctrllen_e ctrllen);

#endif /* __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_PKTBASIC_H*/
