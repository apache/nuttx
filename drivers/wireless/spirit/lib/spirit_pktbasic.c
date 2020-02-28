/******************************************************************************
 * drivers/wireless/spirit/lib/spirit_pktbasic.c
 * Configuration and management of SPIRIT Basic packets.
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

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <sys/types.h>
#include <assert.h>
#include <errno.h>

#include "spirit_pktbasic.h"
#include "spirit_spi.h"

/******************************************************************************
 * Public Functions
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
 *   pktpasic - Basic packet init structure.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktbasic_initialize(FAR struct spirit_library_s *spirit,
                               FAR const struct spirit_pktbasic_init_s *pktpasic)
{
  uint8_t regval[4];
  uint8_t pktlenwidth;
  uint8_t i;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_PKT_PREAMBLE_LENGTH(pktpasic->preamblen));
  DEBUGASSERT(IS_PKT_SYNC_LENGTH(pktpasic->synclen));
  DEBUGASSERT(IS_PKT_CRC_MODE(pktpasic->crcmode));
  DEBUGASSERT(IS_PKT_LENGTH_WIDTH_BITS(pktpasic->pktlenwidth));
  DEBUGASSERT(IS_PKT_FIX_VAR_LENGTH(pktpasic->fixvarlen));
  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(pktpasic->txdestaddr));
  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(pktpasic->fec));
  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(pktpasic->datawhite));
  DEBUGASSERT(IS_PKT_CONTROL_LENGTH(pktpasic->ctrllen));

  /* Reads the PROTOCOL1 register */

  ret = spirit_reg_read(spirit, PROTOCOL1_BASE, &regval[0], 1);

  /* Mask a reserved bit */

  regval[0] &= ~0x20;

  /* Always set the automatic packet filtering */

  regval[0] |= PROTOCOL1_AUTO_PCKT_FLT_MASK;

  /* Writes the value on register */

  ret = spirit_reg_write(spirit, PROTOCOL1_BASE, &regval[0], 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Reads the PCKT_FLT_OPTIONS register */

  ret = spirit_reg_read(spirit, PCKT_FLT_OPTIONS_BASE, &regval[0], 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Always reset the control and source filtering (also if it is not present
   * in basic).
   */

  regval[0] &= ~(PCKT_FLT_OPTIONS_SOURCE_FILTERING_MASK |
                 PCKT_FLT_OPTIONS_CONTROL_FILTERING_MASK);

  /* Writes the value on register */

  ret = spirit_reg_write(spirit, PCKT_FLT_OPTIONS_BASE, &regval[0], 1);
  if (ret < 0)
    {
      return ret;
    }

  if (pktpasic->txdestaddr == S_ENABLE)
    {
      regval[0] = 0x08;
    }
  else
    {
      regval[0] = 0x00;
    }

  /* Address and control length setting */

  regval[0] |= (uint8_t)pktpasic->ctrllen;

  /* Packet format and width length setting */

  pktlenwidth = pktpasic->pktlenwidth;
  if (pktlenwidth == 0)
    {
      pktlenwidth = 1;
    }

  regval[1] = (uint8_t)PCKTCTRL3_PCKT_FRMT_BASIC | (uint8_t)(pktlenwidth - 1);

  /* Preamble, sync and fixed or variable length setting */

  regval[2] = (uint8_t)pktpasic->preamblen | (uint8_t)pktpasic->synclen |
              (uint8_t)pktpasic->fixvarlen;

  /* CRC length, whitening and FEC setting */

  regval[3] = (uint8_t)pktpasic->crcmode;

  if (pktpasic->datawhite == S_ENABLE)
    {
      regval[3] |= PCKTCTRL1_WHIT_MASK;
    }

  if (pktpasic->fec == S_ENABLE)
    {
      regval[3] |= PCKTCTRL1_FEC_MASK;
    }

  /* Writes registers */

  ret = spirit_reg_write(spirit, PCKTCTRL4_BASE, regval, 4);
  if (ret < 0)
    {
      return ret;
    }

  /* Sync words setting */

  for (i = 0; i < 4; i++)
    {
      if (i < 3 - (pktpasic->synclen >> 1))
        {
          regval[i] = 0;
        }
      else
        {
          regval[i] = (uint8_t)(pktpasic->syncwords >> (8 * i));
        }
    }

  /* Sets CRC check bit */

  if (pktpasic->crcmode == PKT_NO_CRC)
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

  return spirit_reg_write(spirit, SYNC4_BASE, regval, 4);
}

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
                              FAR struct spirit_pktbasic_init_s *pktbasic)
{
  uint8_t regval[10];
  int ret;
  int i;

  /* Reads registers */

  ret = spirit_reg_read(spirit, PCKTCTRL4_BASE, regval, 10);
  if (ret < 0)
    {
      return ret;
    }

  /* Length width */

  pktbasic->pktlenwidth = (regval[1] & 0x0f) + 1;

  /* Address field */

  pktbasic->txdestaddr =
    (enum spirit_functional_state_e)((regval[0] >> 3) & 0x01);

  /* Control length */

  pktbasic->ctrllen = (enum pkt_ctrllen_e)(regval[0] & 0x07);

  /* CRC mode */

  pktbasic->crcmode = (enum pkt_crcmode_e)(regval[3] & 0xe0);

  /* Whitening */

  pktbasic->datawhite =
    (enum spirit_functional_state_e) ((regval[3] >> 4) & 0x01);

  /* FEC */

  pktbasic->fec = (enum spirit_functional_state_e)(regval[3] & 0x01);

  /* FIX or VAR bit */

  pktbasic->fixvarlen = (enum pkt_fixvar_len_e)(regval[2] & 0x01);

  /* Preamble length */

  pktbasic->preamblen = (enum pkt_preamblen_e)(regval[2] & 0xf8);

  /* Sync length */

  pktbasic->synclen = (enum pkt_synlen_e)(regval[2] & 0x06);

  /* Sync Words */

  pktbasic->syncwords = 0;
  for (i = 0; i < 4; i++)
    {
      if (i > 2 - (((uint8_t)pktbasic->synclen) >> 1))
        {
          pktbasic->syncwords |= (uint32_t) (regval[i + 6]) << (8 * i);
        }
    }

  return OK;
}

/******************************************************************************
 * Name: spirit_pktbasic_addr_initialize
 *
 * Description:
 *   Initializes the SPIRIT Basic packet addresses according to the specified
 *   parameters in the struct struct spirit_pktbasic_init_s struct.
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
                                    FAR struct spirit_pktbasic_addr_s *basicaddr)
{
  uint8_t regval[3];
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(basicaddr->destfilter));
  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(basicaddr->mcastfilter));
  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(basicaddr->bcastfilter));

  /* Reads the PCKT_FLT_OPTIONS ragister */

  ret = spirit_reg_read(spirit, PCKT_FLT_OPTIONS_BASE, &regval[0], 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Enables or disables filtering on my address */

  if (basicaddr->destfilter == S_ENABLE)
    {
      regval[0] |= PCKT_FLT_OPTIONS_DEST_VS_TX_ADDR_MASK;
    }
  else
    {
      regval[0] &= ~PCKT_FLT_OPTIONS_DEST_VS_TX_ADDR_MASK;
    }

  /* Enables or disables filtering on multicast address */

  if (basicaddr->mcastfilter == S_ENABLE)
    {
      regval[0] |= PCKT_FLT_OPTIONS_DEST_VS_MULTICAST_ADDR_MASK;
    }
  else
    {
      regval[0] &= ~PCKT_FLT_OPTIONS_DEST_VS_MULTICAST_ADDR_MASK;
    }

  /* Enables or disables filtering on broadcast address */

  if (basicaddr->bcastfilter == S_ENABLE)
    {
      regval[0] |= PCKT_FLT_OPTIONS_DEST_VS_BROADCAST_ADDR_MASK;
    }
  else
    {
      regval[0] &= ~PCKT_FLT_OPTIONS_DEST_VS_BROADCAST_ADDR_MASK;
    }

  /* Writes the new value on the PCKT_FLT_OPTIONS register */

  ret = spirit_reg_write(spirit, PCKT_FLT_OPTIONS_BASE, &regval[0], 1);

  /* Fills the array with the addresses passed in the structure */

  regval[0] = basicaddr->bcastaddr;
  regval[1] = basicaddr->mcastaddr;
  regval[2] = basicaddr->srcaddr;

  /* Writes values on the PCKT_FLT_GOALS registers */

  return spirit_reg_write(spirit, PCKT_FLT_GOALS_BROADCAST_BASE, regval, 3);
}

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
                                  FAR struct spirit_pktbasic_addr_s *basicaddr)
{
  uint8_t regval[3];
  int ret;

  /* Reads values on the PCKT_FLT_GOALS registers */

  ret = spirit_reg_read(spirit, PCKT_FLT_GOALS_BROADCAST_BASE, regval, 3);
  if (ret >= 0)
    {
      /* Fit the structure with the read addresses */

      basicaddr->bcastaddr = regval[0];
      basicaddr->mcastaddr = regval[1];
      basicaddr->srcaddr   = regval[2];

      ret = spirit_reg_read(spirit, PCKT_FLT_OPTIONS_BASE, &regval[0], 1);
      if (ret >= 0)
        {
          /* Fit the structure with the read filtering bits */

          basicaddr->bcastfilter =
            (enum spirit_functional_state_e)((regval[0] >> 1) & 0x01);
          basicaddr->mcastfilter =
            (enum spirit_functional_state_e)((regval[0] >> 2) & 0x01);
          basicaddr->destfilter =
            (enum spirit_functional_state_e)((regval[0] >> 3) & 0x01);
        }
    }

  return ret;
}

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

int spirit_pktbasic_set_format(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;
  int ret;

  /* Reads the register value */

  ret = spirit_reg_read(spirit, PCKTCTRL3_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the new value. Also set to 0 the direct RX mode bits */

      regval &= 0x0f;
      regval |= (uint8_t) PCKTCTRL3_PCKT_FRMT_BASIC;

      /* Write the value to the PCKTCTRL3 register */

      ret = spirit_reg_write(spirit, PCKTCTRL3_BASE, &regval, 1);
    }

  if (ret >= 0)
    {
      /* Reads the PCKTCTRL1_BASE register */

      ret = spirit_reg_read(spirit, PCKTCTRL1_BASE, &regval, 1);
      if (ret >= 0)
        {
          /* Build the new value. Set to 0 the direct TX mode bits */

          regval &= 0xf3;

          /* Write the value to the PCKTCTRL1 register */

          ret = spirit_reg_write(spirit, PCKTCTRL1_BASE, &regval, 1);
        }
    }

  if (ret >= 0)
    {
      /* Reads the PROTOCOL1 register */

      ret = spirit_reg_read(spirit, PROTOCOL1_BASE, &regval, 1);
      if (ret >= 0)
        {
          /* Mask a reserved bit */

          regval &= ~0x20;

          /* Write the value to the register */

          ret = spirit_reg_write(spirit, PROTOCOL1_BASE, &regval, 1);
        }
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_pkbasic_enable_addrlen
 *
 * Description:
 *   Sets the address length for SPIRIT Basic packets.
 *
 * Input Parameters:
 *   spirit     - Reference to a Spirit library state structure instance
 *   txdestaddr - Enable/disable address field
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pkbasic_enable_addrlen(FAR struct spirit_library_s *spirit,
                                  enum spirit_functional_state_e txdestaddr)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(txdestaddr));

  /* Reads the PCKTCTRL4 register value */

  ret = spirit_reg_read(spirit, PCKTCTRL4_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the address length for the register */

      if (txdestaddr == S_ENABLE)
        {
          regval |= 0x08;
        }
      else
        {
          regval &= 0x07;
        }

      /* Writes the new value on the PCKTCTRL4 register */

      ret = spirit_reg_write(spirit, PCKTCTRL4_BASE, &regval, 1);
    }

  return ret;
}

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
  spirit_pkbasic_isenabled_addrlen(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the PCKTCTRL4 register value */

  spirit_reg_read(spirit, PCKTCTRL4_BASE, &regval, 1);

  /* Returns the address field value */

  if ((regval & PCKTCTRL4_ADDRESS_LEN_MASK) != 0)
    {
      return S_ENABLE;
    }
  else
    {
      return S_DISABLE;
    }
}

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
                                   uint16_t payloadlen)
{
  uint8_t regval[2];
  uint16_t oversize = 0;

  /* Computes the oversize (address + control) size */

  if (spirit_pkbasic_isenabled_addrlen(spirit))
    {
      oversize = 1;
    }

  oversize += (uint16_t)spirit_pktcommon_get_controllen(spirit);

  /* Computes PCKTLEN0 value from payloadlen */

  regval[1] = BUILD_PCKTLEN0(payloadlen + oversize);

  /* Computes PCKTLEN1 value from payloadlen */

  regval[0] = BUILD_PCKTLEN1(payloadlen + oversize);

  /* Writes data on the PCKTLEN1/0 register */

  return spirit_reg_write(spirit, PCKTLEN1_BASE, regval, 2);
}

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

uint16_t spirit_pktbase_get_payloadlen(FAR struct spirit_library_s *spirit)
{
  uint8_t regval[2];
  uint16_t oversize = 0;

  /* Computes the oversize (address + control) size */

  if (spirit_pkbasic_isenabled_addrlen(spirit))
    {
      oversize = 1;
    }

  oversize += (uint16_t)spirit_pktcommon_get_controllen(spirit);

  /* Reads the packet length registers */

  spirit_reg_read(spirit, PCKTLEN1_BASE, regval, 2);

  /* Returns the packet length */

  return ((((uint16_t) regval[0]) << 8) + (uint16_t) regval[1]) - oversize;
}

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

uint16_t spirit_pktbasic_get_rxpktlen(FAR struct spirit_library_s *spirit)
{
  uint8_t regval[2];
  uint16_t oversize = 0;

  /* Computes the oversize (address + control) size */

  if (spirit_pkbasic_isenabled_addrlen(spirit))
    {
      oversize = 1;
    }

  oversize += (uint16_t)spirit_pktcommon_get_controllen(spirit);

  /* Reads the RX_PCKT_LENx registers value */

  spirit_reg_read(spirit, RX_PCKT_LEN1_BASE, regval, 2);

  /* Rebuild and return the length field */

  return (((((uint16_t) regval[0]) << 8) + (uint16_t)regval[1]) - oversize);
}

/******************************************************************************
 * Name: spirit_pktbasic_set_varlen
 *
 * Description:
 *   Computes and sets the variable payload length for SPIRIT Basic packets.
 *
 * Input Parameters:
 *   spirit     - Reference to a Spirit library state structure instance
 *   payloadlen - Payload length in bytes.
 *   txdestaddr - Enable or disable address field.
 *   ctrllen    - Control length in bytes.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_pktbasic_set_varlen(FAR struct spirit_library_s *spirit,
                               uint16_t payloadlen,
                               enum spirit_functional_state_e txdestaddr,
                               enum pkt_ctrllen_e ctrllen)
{
  uint32_t pktlen;
  uint8_t regval;
  uint8_t addrlen;
  int ret;
  int i;

  /* Sets the address length according to txdestaddr */

  if (txdestaddr == S_ENABLE)
    {
      addrlen = 1;
    }
  else
    {
      addrlen = 0;
    }

  /* packet length = payload length + address length + control length */

  pktlen = payloadlen + addrlen + ctrllen;

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

  /* Reads the PCKTCTRL3 register value */

  ret = spirit_reg_read(spirit, PCKTCTRL3_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build value for the length width */

      regval &= ~PCKTCTRL3_LEN_WID_MASK;
      regval |= (uint8_t)(i - 1);

      /* Write to the PCKTCTRL3 register value */

      ret = spirit_reg_write(spirit, PCKTCTRL3_BASE, &regval, 1);
    }

  return ret;
}
