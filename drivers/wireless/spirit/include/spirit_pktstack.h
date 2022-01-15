/******************************************************************************
 * drivers/wireless/spirit/include/spirit_pktstack.h
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

#ifndef __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_PKTSTACK_H
#define __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_PKTSTACK_H

/* This module can be used to manage the configuration of Spirit STack
 * packets, and it is quite similar to the Basic packets one since the
 * STack packets can be considered an extension of Basic.
 *
 * The user can obtain a packet configuration filling the structure
 * struct spirit_pktstack_init_s, defining in it some general parameters
 * for the Spirit STack packet format.
 *
 * Another structure the user can fill is struct spirit_pktstack_address_s
 * to define the addresses which will be used during the communication.
 * The structure struct spirit_pktstack_llp_s is provided in order to
 * configure the link layer protocol features like autoack,
 * autoretransmission or piggybacking.
 *
 * In addiiton, functions to set the payload length and the destination
 * address are provided.
 *
 * Example:
 *
 * struct spirit_pktstack_init_s g_pktstck_init =
 * {
 *   0x1a2635a8,                   # sync word
 *   PKT_PREAMBLE_LENGTH_08BYTES,  # preamble length in bytes
 *   PKT_SYNC_LENGTH_4BYTES,       # sync word length in bytes
 *   PKT_LENGTH_VAR,               # variable or fixed payload length
 *   7,                            # length field width in bits
 *                                  (used only for variable length)
 *   PKT_NO_CRC,                   # CRC mode
 *   PKT_CONTROL_LENGTH_0BYTES,    # control field length
 *   S_DISABLE,                    # FEC
 *   S_ENABLE                      # whitening
 * };
 *
 * struct spirit_pktstack_address_s g_pktstack_addrinit =
 * {
 *   S_ENABLE,            # enable/disable filtering on my address
 *   0x34,                # my address (address of the current node)
 *   S_DISABLE,           # enable/disable filtering on multicast address
 *   0xee,                # multicast address
 *   S_DISABLE,           # enable/disable filtering on broadcast address
 *   0xff                 # broadcast address
 * };
 *
 * struct spirit_pktstack_llp_s g_pktstack_llpinit =
 * {
 *   S_DISABLE,       # enable/disable the autoack feature
 *   S_DISABLE,       # enable/disable the piggybacking feature
 *   PKT_DISABLE_RETX # set the max number of retransmissions or disable them
 * };
 * ...
 *
 * spirit_pktstack_initialize(spirit, &g_pktstck_init);
 * spirit_pktstack_address_initialize(spirit, &g_pktstack_addrinit);
 * spirit_pktstack_llp_initialize(spirit, &g_pktstack_llpinit);
 *
 * ...
 *
 * spirit_pktstack_set_payloadlen(spirit, 20);
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
 * Public Types
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* SPIRIT STack Packet Init structure definition. This structure allows users
 * to set the main options for the STack packet.
 */

struct spirit_pktstack_init_s
{
  uint32_t syncwords;   /* Specifies the sync words. This parameter is a
                         * uint32_t word with format: 0x|SYNC1|SYNC2|SYNC3|SYNC4| */
  uint8_t  premblen;    /* Specifies the preamble length of packet. This
                         * parameter can be any value from enum pkt_premblen_e */
  uint8_t  synclen;     /* Specifies the sync word length of packet. This
                         * parameter can be any value of enum pkt_premblen_e */
  uint8_t  fixvarlen;   /* Specifies if a fixed length of packet has to be
                         * used. This parameter can be any value of enum
                         * pkt_fixvar_len_e */
  uint8_t  pktlenwidth; /* Specifies the size of the length of packet in
                         * bits. This field is useful only if the field
                         * fixvarlen is set to STACK_LENGTH_VAR. For STack
                         * packets the length width is log2( max payload
                         * length + control length (0 to 4) + address length
                         * (always 2)). */
  uint8_t  crcmode;     /* Specifies the CRC word length of packet.  This
                         * parameter can be any value of enum pkt_crcmode_e */
  uint8_t  ctrllen;     /* Specifies the length of a control field to be
                         * sent.  This parameter can be any value from
                         * enum pkt_ctrllen_e */
  uint8_t  fec;         /* Specifies if FEC has to be enabled. This parameter
                         * can be any value from enum spirit_functional_state_e */
  uint8_t  datawhite;   /* Specifies if data whitening has to be enabled.
                         * This parameter can be any value from enum
                         * spirit_functional_state_e */
};

/* SPIRIT STack packet address structure definition. This structure allows
 * users to specify the node/multicast/broadcast addresses and the
 * correspondent filtering options.
 */

struct spirit_pktstack_address_s
{
  uint8_t destfilter;   /* If set RX packet is accepted if its destination
                         * address matches with srcaddr. This parameter
                         * can be S_ENABLE or S_DISABLE */
  uint8_t srcaddr;      /* Specifies the TX packet source address (address
                         * of this node). */
  uint8_t mcastfilter;  /* If set RX packet is accepted if its destination
                         * address matches with mcastaddr.  This parameter
                         * can be S_ENABLE or S_DISABLE */
  uint8_t mcastaddr;    /* Specifies the Multicast group address for this
                         * node. */
  uint8_t bcastfilter;  /* If set RX packet is accepted if its destination
                         * address matches with bcastaddr.  This parameter
                         * can be S_ENABLE or S_DISABLE */
  uint8_t bcastaddr;    /* Specifies the Broadcast address for this node. */
};

/* SPIRIT STack packet LLP structure definition. This structure allows users
 * to configure all the LLP options for STack packets.
 */

struct spirit_pktstack_llp_s
{
  uint8_t autoack;      /* Specifies if the auto ACK feature is used or not.
                         * This parameter can be a value from enum
                         * spirit_functional_state_e */
  uint8_t piggyback;    /* Specifies if the piggybacking feature is used  or
                         * not. This parameter can be a value from enum
                         * spirit_functional_state_e */
  uint8_t maxretx;      /* Specifies the number of MAX-Retransmissions. This
                         * parameter can be a value from enum spirit_maxretx_e */
};

/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_pktstack_initialize
 *
 * Description:
 *   Initializes the SPIRIT STack packet according to the specified parameters
 *   in the struct spirit_pktstack_init_s.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   pktstack - STack packet init structure.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktstack_initialize(FAR struct spirit_library_s *spirit,
                           FAR const struct spirit_pktstack_init_s *pktstack);

/******************************************************************************
 * Name: spirit_pktstack_get_setup
 * Description:
 *   Returns the SPIRIT STack packet structure according to the specified
 *   parameters in the registers.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   pktstack - STack packet init structure.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktstack_get_setup(FAR struct spirit_library_s *spirit,
                              FAR struct spirit_pktstack_init_s *pktstack);

/******************************************************************************
 * Name: spirit_pktstack_address_initialize
 *
 * Description:
 *   Initializes the SPIRIT STack packet addresses according to the specified
 *   parameters in the PktStackAddresses struct.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   addrinit - STack packet addresses init structure.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktstack_address_initialize(FAR struct spirit_library_s *spirit,
                       FAR const struct spirit_pktstack_address_s *addrinit);

/******************************************************************************
 * Name: spirit_pktstack_get_addrsetup
 *
 * Description:
 *   Returns the SPIRIT STack packet addresses structure according to the
 *   specified parameters in the registers.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   addrinit - STack packet addresses init structure.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktstack_get_addrsetup(FAR struct spirit_library_s *spirit,
                              FAR struct spirit_pktstack_address_s *addrinit);

/******************************************************************************
 * Name: spirit_pktstack_llp_initialize
 *
 * Description:
 *   Initializes the SPIRIT STack packet LLP options according to the specified
 *   parameters in the struct spirit_pktstack_llp_s struct.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   llpinit - STack packet LLP init structure.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktstack_llp_initialize(FAR struct spirit_library_s *spirit,
                           FAR const struct spirit_pktstack_llp_s *llpinit);

/******************************************************************************
 * Name: spirit_pktstack_get_llpsetup
 *
 * Description:
 *   Returns the SPIRIT STack packet LLP options according to the specified
 *   values in the registers.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   llpinit - STack packet LLP structure.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktstack_get_llpsetup(FAR struct spirit_library_s *spirit,
                                 FAR struct spirit_pktstack_llp_s *llpinit);

/******************************************************************************
 * Name: spirit_pktstack_set_format
 *
 * Description:
 *   Configures the Stack packet format for SPIRIT.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktstack_set_format(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktstack_enable_addrlen
 *
 * Description:
 *   Sets the address length for SPIRIT STack packets (always 2).
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktstack_enable_addrlen(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktstack_set_payloadlen
 *
 * Description:
 *   Sets the payload length for SPIRIT STack packets. Since the packet length
 *   depends from the address (always 2 for this packet format) and the
 *   control field size, this function reads the control length register
 *   content in order to determine the correct packet length to be written.
 *
 * Input Parameters:
 *   spirit     - Reference to a Spirit library state structure instance
 *   payloadlen - Payload length in bytes.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktstack_set_payloadlen(FAR struct spirit_library_s *spirit,
                                   uint16_t payloadlen);

/******************************************************************************
 * Name: spirit_pktstack_get_payloadlen
 *
 * Description:
 *   Returns the payload length for SPIRIT STack packets. Since the packet
 *   length depends from the address and the control field size, this function
 *   reads the correspondent registers in order to determine the correct
 *   payload length to be returned.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Payload length.
 *
 ******************************************************************************/

uint16_t spirit_pktstack_get_payloadlen(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktstack_set_varlen
 *
 * Description:
 *   Computes and sets the variable payload length for SPIRIT STack packets.
 *
 * Input Parameters:
 *   spirit     - Reference to a Spirit library state structure instance
 *   payloadlen - Payload length in bytes.
 *   ctrllen    - Control length in bytes.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktstack_set_varlen(FAR struct spirit_library_s *spirit,
                               uint16_t payloadlen,
                               enum pkt_ctrllen_e ctrllen);

/******************************************************************************
 * Name: spirit_pkstack_set_rxsource_addrmask
 *
 * Description:
 *   Rx packet source mask. Used to mask the address of the accepted packets.
 *   If 0 -> no filtering.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   mask   - Rx source mask.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pkstack_set_rxsource_addrmask(FAR struct spirit_library_s *spirit,
                                         uint8_t mask);

/******************************************************************************
 * Name: spirit_pktstack_get_rxsource_addrmask
 *
 * Description:
 *   Returns the Rx packet source mask. Used to mask the address of the
 *   accepted packets. If 0 -> no filtering.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Rx source mask.
 *
 ******************************************************************************/

uint8_t
  spirit_pktstack_get_rxsource_addrmask(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pktstack_get_rxpktlen
 *
 * Description:
 *   Returns the packet length field of the received packet.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Packet length.
 *
 ******************************************************************************/

uint16_t spirit_pktstack_get_rxpktlen(FAR struct spirit_library_s *spirit);

/******************************************************************************
 * Name: spirit_pkstack_enable_rxsource_addrfilter
 *
 * Description:
 *   If enabled RX packet is accepted only if the masked source address field
 *   matches the masked source address field reference (SOURCE_MASK &
 *   SOURCE_FIELD_REF == SOURCE_MASK & RX_SOURCE_FIELD).
 *
 *   NOTE: This filtering control is enabled by default but the source
 *   address mask is by default set to 0.  As a matter of fact the user has
 *   to enable the source filtering bit after the packet initialization
 *   because the packet initialization routine disables it.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for Source address filtering enable bit.
 *         This parameter can be S_ENABLE or S_DISABLE.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pkstack_enable_rxsource_addrfilter(
                                      FAR struct spirit_library_s *spirit,
                                      enum spirit_functional_state_e newstate);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_WIRELESS_SPIRIT_INCLUDE_SPIRIT_PKTSTACK_H */
