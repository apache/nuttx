/******************************************************************************
 * drivers/wireless/spirit/lib/spirit_pktstack.c
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

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <sys/types.h>
#include <assert.h>

#include "spirit_spi.h"
#include "spirit_pktcommon.h"
#include "spirit_pktstack.h"

/******************************************************************************
 * Public Functions
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
                           FAR const struct spirit_pktstack_init_s *pktstack)
{
  uint8_t regval[4];
  uint8_t pktlenwidth;
  int ret;
  int i;

  /* Check the parameters */

  DEBUGASSERT(IS_PKT_PREAMBLE_LENGTH(pktstack->premblen));
  DEBUGASSERT(IS_PKT_SYNC_LENGTH(pktstack->synclen));
  DEBUGASSERT(IS_PKT_CRC_MODE(pktstack->crcmode));
  DEBUGASSERT(IS_PKT_LENGTH_WIDTH_BITS(pktstack->pktlenwidth));
  DEBUGASSERT(IS_PKT_FIX_VAR_LENGTH(pktstack->fixvarlen));
  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(pktstack->fec));
  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(pktstack->datawhite));
  DEBUGASSERT(IS_PKT_CONTROL_LENGTH(pktstack->ctrllen));

  /* Read the PROTOCOL1 register */

  ret = spirit_reg_read(spirit, PROTOCOL1_BASE, &regval[0], 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Mask a reserved bit */

  regval[0] &= ~0x20;

  /* Always (!) set the automatic packet filtering */

  regval[0] |= PROTOCOL1_AUTO_PCKT_FLT_MASK;

  /* Write the value to register */

  ret = spirit_reg_write(spirit, PROTOCOL1_BASE, &regval[0], 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the PCKT_FLT_OPTIONS register */

  ret = spirit_reg_read(spirit, PCKT_FLT_OPTIONS_BASE, &regval[0], 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Always reset the control and source filtering */

  regval[0] &= ~(PCKT_FLT_OPTIONS_SOURCE_FILTERING_MASK |
                 PCKT_FLT_OPTIONS_CONTROL_FILTERING_MASK);

  /* Write the value to register */

  ret = spirit_reg_write(spirit, PCKT_FLT_OPTIONS_BASE, &regval[0], 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Address and control length setting: source and destination address are
   * always present so ADDRESS_LENGTH=2.
   */

  regval[0] = 0x10 | (uint8_t)pktstack->ctrllen;

  /* Packet format and width length setting */

  pktlenwidth = pktstack->pktlenwidth;
  if (pktlenwidth == 0)
    {
      pktlenwidth = 1;
    }

  regval[1] = (uint8_t)PCKTCTRL3_PCKT_FRMT_STACK | (uint8_t)(pktlenwidth - 1);

  /* Preamble, sync and fixed or variable length setting */

  regval[2] = ((uint8_t)pktstack->premblen) | ((uint8_t)pktstack->synclen) |
              ((uint8_t)pktstack->fixvarlen);

  /* CRC length, whitening and FEC setting */

  regval[3] = (uint8_t) pktstack->crcmode;

  if (pktstack->datawhite == S_ENABLE)
    {
      regval[3] |= PCKTCTRL1_WHIT_MASK;
    }

  if (pktstack->fec == S_ENABLE)
    {
      regval[3] |= PCKTCTRL1_FEC_MASK;
    }

  /* Write registers */

  ret = spirit_reg_write(spirit, PCKTCTRL4_BASE, regval, 4);
  if (ret < 0)
    {
      return ret;
    }

  /* Sync words setting */

  for (i = 0; i < 4; i++)
    {
      if (i < 3 - (pktstack->synclen >> 1))
        {
          regval[i] = 0;
        }
      else
        {
          regval[i] = (uint8_t) (pktstack->syncwords >> (8 * i));
        }
    }

  /* Enables or disables the CRC check */

  if (pktstack->crcmode == PKT_NO_CRC)
    {
      ret = spirit_pktcommon_enable_crcfilter(spirit, S_DISABLE);
    }
  else
    {
      ret = spirit_pktcommon_enable_crcfilter(spirit, S_ENABLE);
    }

  if (ret < 0)
    {
      return ret;
    }

  /* Write registers */

  return spirit_reg_write(spirit, SYNC4_BASE, regval, 4);
}

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
                              FAR struct spirit_pktstack_init_s *pktstack)
{
  uint8_t regval[10];
  int ret;
  int i;

  /* Read registers */

  ret = spirit_reg_read(spirit, PCKTCTRL4_BASE, regval, 10);
  if (ret >= 0)
    {
      /* Length width */

      pktstack->pktlenwidth = (regval[1] & 0x0f) + 1;

      /* Control length */

      pktstack->ctrllen = regval[0] & 0x07;

      /* CRC mode */

      pktstack->crcmode = regval[3] & 0xe0;

      /* Whitening */

      pktstack->datawhite = (regval[3] >> 4) & 0x01;

      /* FEC */

      pktstack->fec = regval[3] & 0x01;

      /* FIX or VAR bit */

      pktstack->fixvarlen = regval[2] & 0x01;

      /* Preamble length */

      pktstack->premblen = regval[2] & 0xf8;

      /* Sync length */

      pktstack->synclen = regval[2] & 0x06;

      /* sync Words */

      pktstack->syncwords = 0;
      for (i = 0; i < 4; i++)
        {
          if (i > 2 - (pktstack->synclen >> 1))
            {
              pktstack->syncwords |= regval[i + 6] << (8 * i);
            }
        }
    }

  return OK;
}

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
                       FAR const struct spirit_pktstack_address_s *addrinit)
{
  uint8_t regval[3];
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(addrinit->destfilter));
  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(addrinit->mcastfilter));
  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(addrinit->bcastfilter));

  /* Read the filtering options ragister */

  ret = spirit_reg_read(spirit, PCKT_FLT_OPTIONS_BASE, &regval[0], 1);
  if (ret >= 0)
    {
      /* Enables or disables filtering on my address */

      if (addrinit->destfilter == S_ENABLE)
        {
          regval[0] |= PCKT_FLT_OPTIONS_DEST_VS_TX_ADDR_MASK;
        }
      else
        {
          regval[0] &= ~PCKT_FLT_OPTIONS_DEST_VS_TX_ADDR_MASK;
        }

      /* Enables or disables filtering on multicast address */

      if (addrinit->mcastfilter == S_ENABLE)
        {
          regval[0] |= PCKT_FLT_OPTIONS_DEST_VS_MULTICAST_ADDR_MASK;
        }
      else
        {
          regval[0] &= ~PCKT_FLT_OPTIONS_DEST_VS_MULTICAST_ADDR_MASK;
        }

      /* Enables or disables filtering on broadcast address */

      if (addrinit->bcastfilter == S_ENABLE)
        {
          regval[0] |= PCKT_FLT_OPTIONS_DEST_VS_BROADCAST_ADDR_MASK;
        }
      else
        {
          regval[0] &= ~PCKT_FLT_OPTIONS_DEST_VS_BROADCAST_ADDR_MASK;
        }

      /* Write values to the register */

      ret = spirit_reg_write(spirit, PCKT_FLT_OPTIONS_BASE, &regval[0], 1);
      if (ret >= 0)
        {
          /* Fill array with the addresses passed in the structure */

          regval[0] = addrinit->bcastaddr;
          regval[1] = addrinit->mcastaddr;
          regval[2] = addrinit->srcaddr;

          /* Write them to the addresses registers */

          ret = spirit_reg_write(spirit,
                                 PCKT_FLT_GOALS_BROADCAST_BASE, regval, 3);
        }
    }

  return ret;
}

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
                              FAR struct spirit_pktstack_address_s *addrinit)
{
  uint8_t regval[3];
  int ret;

  /* Read values from the PCKT_FLT_GOALS registers */

  ret = spirit_reg_read(spirit, PCKT_FLT_GOALS_BROADCAST_BASE, regval, 3);
  if (ret >= 0)
    {
      /* Fit the structure with the read addresses */

      addrinit->bcastaddr = regval[0];
      addrinit->mcastaddr = regval[1];
      addrinit->srcaddr   = regval[2];

      ret = spirit_reg_read(spirit, PCKT_FLT_OPTIONS_BASE, &regval[0], 1);
      if (ret >= 0)
        {
          /* Fit the structure with the read filtering bits */

          addrinit->bcastfilter =
            (enum spirit_functional_state_e)((regval[0] >> 1) & 0x01);
          addrinit->mcastfilter =
            (enum spirit_functional_state_e)((regval[0] >> 2) & 0x01);
          addrinit->destfilter =
            (enum spirit_functional_state_e)((regval[0] >> 3) & 0x01);
        }
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_pktstack_llp_initialize
 *
 * Description:
 *   Initializes the SPIRIT STack packet LLP options according to the
 *   specified parameters in the struct spirit_pktstack_llp_s struct.
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
                               FAR const struct spirit_pktstack_llp_s *llpinit)
{
  uint8_t regval[2];
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(llpinit->piggyback));
  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(llpinit->autoack));
  DEBUGASSERT(IS_PKT_MAXRETX(llpinit->maxretx));

  /* Check if piggybacking is enabled and autoack is disabled */

  DEBUGASSERT(!(llpinit->piggyback == S_ENABLE &&
                llpinit->autoack   == S_DISABLE));

  /* Piggybacking mechanism setting from the PROTOCOL1 register */

  ret = spirit_reg_read(spirit, PROTOCOL1_BASE, regval, 2);
  if (ret >= 0)
    {
      if (llpinit->piggyback == S_ENABLE)
        {
          regval[0] |= PROTOCOL1_PIGGYBACKING_MASK;
        }
      else
        {
          regval[0] &= ~PROTOCOL1_PIGGYBACKING_MASK;
        }

      /* RX and TX autoack mechanisms setting from the PROTOCOL0 register */

      if (llpinit->autoack == S_ENABLE)
        {
          regval[1] |= PROTOCOL0_AUTO_ACK_MASK;
        }
      else
        {
          regval[1] &= ~PROTOCOL0_AUTO_ACK_MASK;
        }

      /* Max number of retransmission setting */

      regval[1] &= ~PROTOCOL0_NMAX_RETX_MASK;
      regval[1] |= llpinit->maxretx;

      /* Write registers */

      ret = spirit_reg_write(spirit, PROTOCOL1_BASE, regval, 2);
    }

  return ret;
}

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
                                 FAR struct spirit_pktstack_llp_s *llpinit)
{
  uint8_t regval[2];
  int ret;

  /* Piggybacking mechanism setting from the PROTOCOL1 register */

  ret = spirit_reg_read(spirit, PROTOCOL1_BASE, regval, 2);
  if (ret >= 0)
    {
      /* Fit the structure with the read values */

      llpinit->piggyback = (regval[0] >> 6) & 0x01;
      llpinit->autoack   = (regval[1] >> 2) & 0x01;
      llpinit->maxretx   = regval[1] & PROTOCOL0_NMAX_RETX_MASK;
    }

  return ret;
}

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

int spirit_pktstack_set_format(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;
  int ret;

  /* Read the PCKTCTRL3 register value */

  ret = spirit_reg_read(spirit, PCKTCTRL3_BASE, &regval, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Build value to be written. Also set to 0 the direct RX mode bits */

  regval &= 0x0f;
  regval |= ((uint8_t)PCKTCTRL3_PCKT_FRMT_STACK);

  /* Write the value to the PCKTCTRL3 register. */

  ret = spirit_reg_write(spirit, PCKTCTRL3_BASE, &regval, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the PCKTCTRL1 register value */

  ret = spirit_reg_read(spirit, PCKTCTRL1_BASE, &regval, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Build the new value.  Set to 0 the direct TX mode bits */

  regval &= 0xf3;

  /* Write the PCKTCTRL1 value to register */

  ret = spirit_reg_write(spirit, PCKTCTRL1_BASE, &regval, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the PROTOCOL1 register */

  ret = spirit_reg_read(spirit, PROTOCOL1_BASE, &regval, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Mask a reserved bit */

  regval &= ~0x20;

  /* Write the value to the PROTOCOL1 register */

  return spirit_reg_write(spirit, PROTOCOL1_BASE, &regval, 1);
}

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

int spirit_pktstack_enable_addrlen(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;
  int ret;

  /* Read the PCKTCTRL4 register value */

  ret = spirit_reg_read(spirit, PCKTCTRL4_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the new value */

      regval &= ~PCKTCTRL4_ADDRESS_LEN_MASK;
      regval |= ((uint8_t) 0x10);

      /* Write the value to the PCKTCTRL4 register */

      ret = spirit_reg_write(spirit, PCKTCTRL4_BASE, &regval, 1);
    }

  return ret;
}

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
                                   uint16_t payloadlen)
{
  uint8_t regval[2];

  /* Computes the oversize (address + control) size */

  uint16_t oversize = 2 + (uint16_t)spirit_pktcommon_get_controllen(spirit);

  /* Computes PCKTLEN0 value from lPayloadLength */

  regval[1] = BUILD_PCKTLEN0(payloadlen + oversize);

  /* Computes PCKTLEN1 value from lPayloadLength */

  regval[0] = BUILD_PCKTLEN1(payloadlen + oversize);

  /* Write the value to the PCKTLENx registers */

  return spirit_reg_write(spirit, PCKTLEN1_BASE, regval, 2);
}

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

uint16_t spirit_pktstack_get_payloadlen(FAR struct spirit_library_s *spirit)
{
  uint8_t regval[2];
  uint16_t oversize;

  /* Computes the oversize (address + control) size */

  oversize = 2 + (uint16_t)spirit_pktcommon_get_controllen(spirit);

  /* Read the PCKTLEN1 registers value */

  spirit_reg_read(spirit, PCKTLEN1_BASE, regval, 2);

  /* Rebuild and return the payload length value */

  return ((((uint16_t) regval[1]) << 8) + (uint16_t) regval[0] - oversize);
}

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
                               enum pkt_ctrllen_e ctrllen)
{
  uint32_t pktlen;
  uint8_t regval;
  int ret;
  int i;

  /* packet length = payload length + address length (2) + control length */

  pktlen = payloadlen + 2 + ctrllen;

  /* Computes the number of bits */

  for (i = 0; i < 16; i++)
    {
      if (pktlen == 0)
        {
          break;
        }

      pktlen >>= 1;
    }

  if (i == 0)
    {
      i = 1;
    }

  /* Read the PCKTCTRL3 register value */

  ret = spirit_reg_read(spirit, PCKTCTRL3_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the register value */

      regval &= ~PCKTCTRL3_LEN_WID_MASK;
      regval |= (uint8_t)(i - 1);

      /* Write the PCKTCTRL3 register value */

      ret = spirit_reg_write(spirit, PCKTCTRL3_BASE, &regval, 1);
    }

  return ret;
}

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
                                         uint8_t mask)
{
  /* Write value to the register PCKT_FLT_GOALS_SOURCE_MASK */

  return spirit_reg_write(spirit, PCKT_FLT_GOALS_SOURCE_MASK_BASE, &mask, 1);
}

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

uint8_t spirit_pktstack_get_rxsource_addrmask(
                                      FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Write value to the PCKT_FLT_GOALS_SOURCE_MASK register */

  spirit_reg_read(spirit, PCKT_FLT_GOALS_SOURCE_MASK_BASE, &regval, 1);

  /* Return the read value */

  return regval;
}

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

uint16_t spirit_pktstack_get_rxpktlen(FAR struct spirit_library_s *spirit)
{
  uint8_t regval[2];
  uint16_t pktlen;

  /* Read the RX_PCKT_LENx registers value */

  spirit_reg_read(spirit, RX_PCKT_LEN1_BASE, regval, 2);

  /* Rebuild and return the the length field */

  pktlen = ((((uint16_t) regval[0]) << 8) + (uint16_t) regval[1]);

  /* Computes the oversize (address + control) size */

  pktlen -= 2 + (uint16_t)spirit_pktcommon_get_controllen(spirit);

  return pktlen;
}

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
                                      enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Modify the register value: set or reset the source bit filtering */

  ret = spirit_reg_read(spirit, PCKT_FLT_OPTIONS_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Set or reset the SOURCE ADDRESS filtering enabling bit */

      if (newstate == S_ENABLE)
        {
          regval |= PCKT_FLT_OPTIONS_SOURCE_FILTERING_MASK;
        }
      else
        {
          regval &= ~PCKT_FLT_OPTIONS_SOURCE_FILTERING_MASK;
        }

      /* Write the new value to the PCKT_FLT_OPTIONS register */

      ret = spirit_reg_write(spirit, PCKT_FLT_OPTIONS_BASE, &regval, 1);
    }

  return ret;
}
