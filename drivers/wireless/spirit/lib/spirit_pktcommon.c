/******************************************************************************
 * drivers/wireless/spirit/lib/spirit_pktcommon.c
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

#include <assert.h>

#include "spirit_pktcommon.h"
#include "spirit_spi.h"

/******************************************************************************
 * Public Functions
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
                                    enum pkt_ctrllen_e ctrllen)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_PKT_CONTROL_LENGTH(ctrllen));

  /* Read the PCKTCTRL4 register value */

  ret = spirit_reg_read(spirit, PCKTCTRL4_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Set the control length */

      regval &= ~PCKTCTRL4_CONTROL_LEN_MASK;
      regval |= (uint8_t)ctrllen;

      /* Write the new value on the PCKTCTRL4 register */

      ret = spirit_reg_write(spirit, PCKTCTRL4_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_pktcommon_get_controllen
 *
 * Description:
 *   Returns the CONTROL field length for SPIRIT packets.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Control field length.
 *
 ******************************************************************************/

uint8_t spirit_pktcommon_get_controllen(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the PCKTCTRL4 register value */

  spirit_reg_read(spirit, PCKTCTRL4_BASE, &regval, 1);

  /* Rebuild and return value */

  return (regval & PCKTCTRL4_CONTROL_LEN_MASK);
}

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
                                   enum pkt_preamblen_e preamblen)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_PKT_PREAMBLE_LENGTH(preamblen));

  /* Read the PCKTCTRL2 register value */

  ret = spirit_reg_read(spirit, PCKTCTRL2_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Set the preamble length */

      regval &= ~PCKTCTRL2_PREAMBLE_LENGTH_MASK;
      regval |= (uint8_t)preamblen;

      /* Write the new value to the PCKTCTRL2 register */

      ret = spirit_reg_write(spirit, PCKTCTRL2_BASE, &regval, 1);
    }

  return ret;
}

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

uint8_t spirit_pktcommon_get_preamblen(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the PCKTCTRL2 register value */

  spirit_reg_read(spirit, PCKTCTRL2_BASE, &regval, 1);

  /* Rebuild and return value */

  return ((regval & PCKTCTRL2_PREAMBLE_LENGTH_MASK) >> 3) + 1;
}

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
                           enum pkt_synlen_e synclen)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_PKT_SYNC_LENGTH(synclen));

  /* Read the PCKTCTRL2 register value */

  ret = spirit_reg_read(spirit, PCKTCTRL2_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Set the sync length */

      regval &= ~PCKTCTRL2_SYNC_LENGTH_MASK;
      regval |= (uint8_t)synclen;

      /* Write the new value to the PCKTCTRL2 register */

      ret = spirit_reg_write(spirit, PCKTCTRL2_BASE, &regval, 1);
    }

  return ret;
}

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

uint8_t spirit_pktcommon_get_synclen(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the PCKTCTRL2 register value */

  spirit_reg_read(spirit, PCKTCTRL2_BASE, &regval, 1);

  /* Rebuild and return value */

  return ((regval & PCKTCTRL2_SYNC_LENGTH_MASK) >> 1) + 1;
}

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
                                   enum pkt_fixvar_len_e fixvarlen)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_PKT_FIX_VAR_LENGTH(fixvarlen));

  /* Read the PCKTCTRL2 register value */

  ret = spirit_reg_read(spirit, PCKTCTRL2_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Set fixed or variable address mode */

      regval &= ~PCKTCTRL2_FIX_VAR_LEN_MASK;
      regval |= (uint8_t)fixvarlen;

      /* Write the new value to the PCKTCTRL2 register */

      ret = spirit_reg_write(spirit, PCKTCTRL2_BASE, &regval, 1);
    }

  return ret;
}

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
                                      enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Read the PCKT_FLT_OPTIONS register value */

  ret = spirit_reg_read(spirit, PCKT_FLT_OPTIONS_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Modify the register value: enable or disable the CRC filtering */

      if (newstate == S_ENABLE)
        {
          regval |= PCKT_FLT_OPTIONS_CRC_CHECK_MASK;
        }
      else
        {
          regval &= ~PCKT_FLT_OPTIONS_CRC_CHECK_MASK;
        }

      /* Write the PCKT_FLT_OPTIONS register value */

      ret = spirit_reg_write(spirit, PCKT_FLT_OPTIONS_BASE, &regval, 1);
    }

  return ret;
}

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
  spirit_pktcommon_isenabled_crcfilter(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the PCKT_FLT_OPTIONS register value */

  spirit_reg_read(spirit, PCKT_FLT_OPTIONS_BASE, &regval, 1);

  /* Check the CRC filtering bit */

  if (regval & PCKT_FLT_OPTIONS_CRC_CHECK_MASK)
    {
      return S_ENABLE;
    }
  else
    {
      return S_DISABLE;
    }
}

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
                                 enum pkt_crcmode_e crcmode)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_PKT_CRC_MODE(crcmode));

  /* Read the PCKTCTRL1 register value */

  ret = spirit_reg_read(spirit, PCKTCTRL1_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build data to write setting the CRC mode */

      regval &= ~PCKTCTRL1_CRC_MODE_MASK;
      regval |= (uint8_t)crcmode;

      /* Write the new value to the PCKTCTRL1 register */

      ret = spirit_reg_write(spirit, PCKTCTRL1_BASE, &regval, 1);
    }

  return ret;
}

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
  spirit_pktcommon_get_crcmode(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the PCKTCTRL1 register */

  spirit_reg_read(spirit, PCKTCTRL1_BASE, &regval, 1);

  /* Rebuild and return value */

  return (enum pkt_crcmode_e) (regval & 0xe0);
}

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
                                      enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Read the PCKTCTRL1 register value */

  ret = spirit_reg_read(spirit, PCKTCTRL1_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build data to write: set or reset the whitening enable bit */

      if (newstate == S_ENABLE)
        {
          regval |= PCKTCTRL1_WHIT_MASK;
        }
      else
        {
          regval &= ~PCKTCTRL1_WHIT_MASK;
        }

      /* Write the new value to the PCKTCTRL1 register */

      ret = spirit_reg_write(spirit, PCKTCTRL1_BASE, &regval, 1);
    }

  return ret;
}

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
                                enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Read the PCKTCTRL1 register value */

  ret = spirit_reg_read(spirit, PCKTCTRL1_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build data to write: set or reset the FEC enable bit */

      if (newstate == S_ENABLE)
        {
          regval |= PCKTCTRL1_FEC_MASK;
        }
      else
        {
          regval &= ~PCKTCTRL1_FEC_MASK;
        }

      /* Write data to the PCKTCTRL1 register */

      ret = spirit_reg_write(spirit, PCKTCTRL1_BASE, &regval, 1);
    }

  return ret;
}

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
                                  uint8_t syncword)
{
  uint8_t regaddr;

  /* Check the parameters */

  DEBUGASSERT(IS_PKT_SYNCWORD(syncwordno));

  /* Set the specified address */

  switch (syncwordno)
    {
    case PKT_SYNC_WORD_1:
      regaddr = SYNC1_BASE;
      break;

    case PKT_SYNC_WORD_2:
      regaddr = SYNC2_BASE;
      break;

    case PKT_SYNC_WORD_3:
      regaddr = SYNC3_BASE;
      break;

    default:
      regaddr = SYNC4_BASE;
      break;
    }

  /* Write value to the selected register */

  return spirit_reg_write(spirit, regaddr, &syncword, 1);
}

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
                                      enum spirit_pktsyncword_e syncwordno)
{
  uint8_t regaddr;
  uint8_t regval;

  /* Check the parameters */

  DEBUGASSERT(IS_PKT_SYNCWORD(syncwordno));

  /* Set the specified address */

  switch (syncwordno)
    {
    case PKT_SYNC_WORD_1:
      regaddr = SYNC1_BASE;
      break;

    case PKT_SYNC_WORD_2:
      regaddr = SYNC2_BASE;
      break;
    case PKT_SYNC_WORD_3:
      regaddr = SYNC3_BASE;
      break;

    default:
      regaddr = SYNC4_BASE;
      break;
    }

  /* Read the selected register value */

  spirit_reg_read(spirit, regaddr, &regval, 1);

  /* Returns the read value */

  return regval;
}

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
                                   uint32_t syncwords,
                                   enum pkt_synlen_e synclen)
{
  uint8_t regval[4];
  int i;

  /* Split the 32-bit value in 4 8-bit values */

  for (i = 0; i < 4; i++)
    {
      if (i < ((3 - synclen) >> 1))
        {
          regval[i] = 0;
        }
      else
        {
          regval[i] = (uint8_t)(syncwords >> (8 * i));
        }
    }

  /* Write SYNC value to the SYNCx registers */

  return spirit_reg_write(spirit, SYNC4_BASE, regval, 4);
}

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
 *   Sync words. The format of the read 32 bit word is
 *   0x|SYNC1|SYNC2|SYNC3|SYNC4|.
 *
 ******************************************************************************/

uint32_t spirit_pktcommon_get_syncwords(FAR struct spirit_library_s *spirit,
                                        enum pkt_synlen_e synclen)
{
  uint8_t regval[4];
  uint32_t syncword = 0;
  int i;

  /* Read the SYNCx registers value */

  spirit_reg_read(spirit, SYNC4_BASE, regval, 4);

  /* Rebuild the SYNC words */

  for (i = 0; i < 4; i++)
    {
      if (i > 2 - (synclen >> 1))
        {
          syncword |= regval[i] << (8 * i);
        }
    }

  /* Return SYNC words */

  return syncword;
}

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

uint8_t spirit_pktcommon_get_varlen(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the PCKTCTRL3 register value */

  spirit_reg_read(spirit, PCKTCTRL3_BASE, &regval, 1);

  /* Rebuild and return value */

  return (regval & PCKTCTRL3_LEN_WID_MASK) + 1;
}

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
                                    uint8_t txdestaddr)
{
  /* Write value to PCKT_FLT_GOALS_SOURCE_ADDR register */

  return spirit_reg_write(spirit,
                          PCKT_FLT_GOALS_SOURCE_ADDR_BASE, &txdestaddr, 1);
}

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

uint8_t spirit_pktcommon_get_txdestaddr(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read value from the PCKT_FLT_GOALS_SOURCE_ADDR register */

  spirit_reg_read(spirit, PCKT_FLT_GOALS_SOURCE_ADDR_BASE, &regval, 1);

  /* Return value */

  return regval;
}

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
                                     uint8_t srcaddr)
{
  /* Write value to the PCKT_FLT_GOALS_TX_ADDR register */

  return spirit_reg_write(spirit, PCKT_FLT_GOALS_TX_ADDR_BASE, &srcaddr, 1);
}

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

uint8_t spirit_pktcommon_get_nodeaddress(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read value from the PCKT_FLT_GOALS_TX_ADDR register */

  spirit_reg_read(spirit, PCKT_FLT_GOALS_TX_ADDR_BASE, &regval, 1);

  /* Return value */

  return regval;
}

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
                                   uint8_t bcastaddr)
{
  /* Write value to the PCKT_FLT_GOALS_BROADCAST register */

  return spirit_reg_write(spirit,
                          PCKT_FLT_GOALS_BROADCAST_BASE, &bcastaddr, 1);
}

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

uint8_t spirit_pktcommon_get_bcastaddr(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read value from the PCKT_FLT_GOALS_BROADCAST register */

  spirit_reg_read(spirit, PCKT_FLT_GOALS_BROADCAST_BASE, &regval, 1);

  /* Return value */

  return regval;
}

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
                                   uint8_t mcastaddr)
{
  /* Write value to the PCKT_FLT_GOALS_MULTICAST register */

  return spirit_reg_write(spirit,
                          PCKT_FLT_GOALS_MULTICAST_BASE, &mcastaddr, 1);
}

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

uint8_t spirit_pktcommon_get_mcastaddr(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read value from the PCKT_FLT_GOALS_MULTICAST register */

  spirit_reg_read(spirit, PCKT_FLT_GOALS_MULTICAST_BASE, &regval, 1);

  /* Return value */

  return regval;
}

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
                                  uint32_t mask)
{
  uint8_t regval[4];

  /* Split the 32-bit value in 4 8-bit values */

  regval[0] = (uint8_t)mask;
  regval[1] = (uint8_t)(mask >> 8);
  regval[2] = (uint8_t)(mask >> 16);
  regval[3] = (uint8_t)(mask >> 24);

  /* Write values to the CKT_FLT_GOALS_CONTROLx_MASK registers */

  return spirit_reg_write(spirit,
                          PCKT_FLT_GOALS_CONTROL0_MASK_BASE, regval, 4);
}

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

uint32_t spirit_pktcommon_get_ctrlmask(FAR struct spirit_library_s *spirit)
{
  uint8_t regval[4];
  uint32_t ctrlmask = 0;
  int i;

  /* Read the PCKT_FLT_GOALS_CONTROLx_MASK registers */

  spirit_reg_read(spirit, PCKT_FLT_GOALS_CONTROL0_MASK_BASE, regval, 4);

  /* Rebuild the control mask value on a 32-bit integer variable */

  for (i = 0; i < 4; i++)
    {
      ctrlmask |= (uint32_t)regval[i] << (8 * i);
    }

  /* Return value */

  return ctrlmask;
}

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
                                 uint32_t reference)
{
  uint8_t regval[4];

  /* Split the 32-bit value in 4 8-bit values */

  regval[0] = (uint8_t)reference;
  regval[1] = (uint8_t)(reference >> 8);
  regval[2] = (uint8_t)(reference >> 16);
  regval[3] = (uint8_t)(reference >> 24);

  /* Write values to the CKT_FLT_GOALS_CONTROLx_FIELD registers */

  return spirit_reg_write(spirit,
                          PCKT_FLT_GOALS_CONTROL0_FIELD_BASE, regval, 4);
}

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

uint32_t spirit_pktcommon_get_ctrlref(FAR struct spirit_library_s *spirit)
{
  uint8_t regval[4];
  uint32_t ctrlref = 0;
  int i;

  /* Read the PCKT_FLT_GOALS_CONTROLx_FIELD registers */

  spirit_reg_read(spirit, PCKT_FLT_GOALS_CONTROL0_FIELD_BASE, regval, 4);

  /* Rebuild the control mask value on a 32-bit integer variable */

  for (i = 0; i < 4; i++)
    {
      ctrlref |= ((uint32_t) regval[i]) << (8 * i);
    }

  /* Return value */

  return ctrlref;
}

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
                                uint32_t txctrl)
{
  uint8_t regval[4];

  /* Split the 32-bit value in 4 8-bit values */

  regval[3] = (uint8_t)txctrl;
  regval[2] = (uint8_t)(txctrl >> 8);
  regval[1] = (uint8_t)(txctrl >> 16);
  regval[0] = (uint8_t)(txctrl >> 24);

  /* Write value to the TX_CTRL_FIELDx register */

  return spirit_reg_write(spirit, TX_CTRL_FIELD3_BASE, regval, 4);
}

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

uint32_t spirit_pktcommon_get_txctrl(FAR struct spirit_library_s *spirit)
{
  uint8_t regval[4];
  uint32_t ctrlfield = 0;
  int i;

  /* Read the TX_CTRL_FIELDx registers */

  spirit_reg_read(spirit, TX_CTRL_FIELD3_BASE, regval, 4);

  /* Rebuild value: build a 32-bit value from the read bytes */

  for (i = 0; i < 4; i++)
    {
      ctrlfield |= ((uint32_t) regval[i]) << (8 * (3 - i));
    }

  /* Return value */

  return ctrlfield;
}

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

int spirit_pktcommon_enable_destaddr_filter(
                                    FAR struct spirit_library_s *spirit,
                                    enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Modify the register value: set or reset the TX source address control */

  ret = spirit_reg_read(spirit, PCKT_FLT_OPTIONS_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Set or reset the DESTINATION vs TX enabling bit */

      if (newstate == S_ENABLE)
        {
          regval |= PCKT_FLT_OPTIONS_DEST_VS_TX_ADDR_MASK;
        }
      else
        {
          regval &= ~PCKT_FLT_OPTIONS_DEST_VS_TX_ADDR_MASK;
        }

      /* Write the new value to the PCKT_FLT_OPTIONS register */

      ret = spirit_reg_write(spirit, PCKT_FLT_OPTIONS_BASE, &regval, 1);
    }

  return ret;
}

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

int spirit_pktcommon_enable_mcastaddr_filter(
                                    FAR struct spirit_library_s *spirit,
                                    enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Read the PCKT_FLT_OPTIONS register value */

  ret = spirit_reg_read(spirit, PCKT_FLT_OPTIONS_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Enable or disable the filtering option */

      if (newstate == S_ENABLE)
        {
          regval |= PCKT_FLT_OPTIONS_DEST_VS_MULTICAST_ADDR_MASK;
        }
      else
        {
          regval &= ~PCKT_FLT_OPTIONS_DEST_VS_MULTICAST_ADDR_MASK;
        }

      /* Write the new value to the PCKT_FLT_OPTIONS register */

      ret = spirit_reg_write(spirit, PCKT_FLT_OPTIONS_BASE, &regval, 1);
    }

  return ret;
}

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

int spirit_pktcommon_enable_bcastaddr_filter(
                                     FAR struct spirit_library_s *spirit,
                                     enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Read the register value */

  ret = spirit_reg_read(spirit, PCKT_FLT_OPTIONS_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Enable or disable the filtering option */

      if (newstate == S_ENABLE)
        {
          regval |= PCKT_FLT_OPTIONS_DEST_VS_BROADCAST_ADDR_MASK;
        }
      else
        {
          regval &= ~PCKT_FLT_OPTIONS_DEST_VS_BROADCAST_ADDR_MASK;
        }

      /* Write the new value to the PCKT_FLT_OPTIONS register */

      ret = spirit_reg_write(spirit, PCKT_FLT_OPTIONS_BASE, &regval, 1);
    }

  return ret;
}

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
  spirit_pktcommon_isenabled_destaddr_filter(
                                  FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the register value */

  spirit_reg_read(spirit, PCKT_FLT_OPTIONS_BASE, &regval, 1);

  /* Gets the enable/disable bit in form of enum spirit_functional_state_e
   * type
   */

  if (regval & 0x08)
    {
      return S_ENABLE;
    }
  else
    {
      return S_DISABLE;
    }
}

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
  spirit_pktcommon_isenabled_mcastaddr_filter(
                                      FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the register value */

  spirit_reg_read(spirit, PCKT_FLT_OPTIONS_BASE, &regval, 1);

  /* Get the enable/disable bit in form of enum spirit_functional_state_e
   * type
   */

  if (regval & 0x04)
    {
      return S_ENABLE;
    }
  else
    {
      return S_DISABLE;
    }
}

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
  spirit_pktcommon_isenabled_bcastaddr_filter(
                                       FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the register value */

  spirit_reg_read(spirit, PCKT_FLT_OPTIONS_BASE, &regval, 1);

  /* Get the enable/disable bit in form of enum spirit_functional_state_e
   * type
   */

  if (regval & 0x02)
    {
      return S_ENABLE;
    }
  else
    {
      return S_DISABLE;
    }
}

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

uint8_t spirit_pktcommon_get_rxdestaddr(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the RX_ADDR_FIELD0 register value */

  spirit_reg_read(spirit, RX_ADDR_FIELD0_BASE, &regval, 1);

  /* Return value */

  return regval;
}

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

uint32_t spirit_pktcommon_get_rxctrl(FAR struct spirit_library_s *spirit)
{
  uint8_t regval[4];
  uint32_t ctrlfield = 0;
  int i;

  /* Read the PCKT_FLT_GOALS_CONTROLx_MASK registers */

  spirit_reg_read(spirit, RX_CTRL_FIELD0_BASE, regval, 4);

  /* Rebuild the control mask value on a 32-bit integer variable */

  for (i = 0; i < 4; i++)
    {
      ctrlfield |= ((uint32_t)regval[i]) << (8 * i);
    }

  /* Returns value */

  return ctrlfield;
}

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
                               FAR uint8_t *crc)
{
  enum pkt_crcmode_e crcmode;
  uint8_t regval[3];
  uint8_t crclen;
  int ret;
  int i;

  /* Gets the CRC mode in enum pkt_crcmode_e enum */

  crcmode = spirit_pktcommon_get_crcmode(spirit);

  /* Cast to FAR uint8_t */

  crclen = (uint8_t)crcmode;

  /* Obtains the real length: see the @ref enum pkt_crcmode_e enumeration */

  crclen >>= 5;
  if (crclen >= 3)
    {
      crclen--;
    }

  /* Read the CRC_FIELDx registers value */

  ret = spirit_reg_read(spirit, CRC_FIELD2_BASE, regval, 3);
  if (ret >= 0)
    {
      /* Sets the array to be returned */

      for (i = 0; i < 3; i++)
        {
          if (i < crclen)
            {
              crc[i] = regval[2 - i];
            }
          else
            {
              crc[i] = 0;
            }
        }
    }

  return ret;
}

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
                                      enum spirit_functional_state_e piggyback)
{
  uint8_t regval[2];
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(autoack));
  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(piggyback));

  /* Check if piggybacking is enabled and autoack is disabled */

  DEBUGASSERT(!(piggyback == S_ENABLE && autoack == S_DISABLE));

  /* Read the PROTOCOL[1:0] registers value */

  ret = spirit_reg_read(spirit, PROTOCOL1_BASE, regval, 2);
  if (ret >= 0)
    {
      /* Sets the specified LLP option */

      /* Autoack setting */

      if (autoack == S_ENABLE)
        {
          regval[1] |= PROTOCOL0_AUTO_ACK_MASK;
        }
      else
        {
          regval[1] &= (~PROTOCOL0_AUTO_ACK_MASK);
        }

      /* Piggybacking setting */

      if (piggyback == S_ENABLE)
        {
          regval[0] |= PROTOCOL1_PIGGYBACKING_MASK;
        }
      else
        {
          regval[0] &= (~PROTOCOL1_PIGGYBACKING_MASK);
        }

      /* Write data to the PROTOCOL[1:0] registers */

      ret = spirit_reg_write(spirit, PROTOCOL1_BASE, regval, 2);
    }

  return ret;
}

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
                                      enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Read value from the PROTOCOL0 register */

  ret = spirit_reg_read(spirit, PROTOCOL0_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Enables or disables the ack requirement option */

      if (newstate == S_DISABLE)
        {
          regval |= PROTOCOL0_NACK_TX_MASK;
        }
      else
        {
          regval &= ~PROTOCOL0_NACK_TX_MASK;
        }

      /* Write value to the PROTOCOL0 register */

      ret = spirit_reg_write(spirit, PROTOCOL0_BASE, &regval, 1);
    }

  return ret;
}

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
                                 uint8_t seqno)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_PKT_SEQ_NUMBER_RELOAD(seqno));

  /* Read value from the PROTOCOL2 register */

  ret = spirit_reg_read(spirit, PROTOCOL2_BASE, &regval, 1);
  if (ret >= 0)
    {
      regval &= 0xe7;
      regval |= (seqno << 3);

      /* Write value to the PROTOCOL2 register */

      ret = spirit_reg_write(spirit, PROTOCOL2_BASE, &regval, 1);
    }

  return ret;
}

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
                                 enum spirit_maxretx_e maxretx)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_PKT_MAXRETX(maxretx));

  /* Read the PROTOCOL0 register value */

  ret = spirit_reg_read(spirit, PROTOCOL0_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the value to be written */

      regval &= ~PROTOCOL0_NMAX_RETX_MASK;
      regval |= maxretx;

      /* Write value to the PROTOCOL0 register */

      ret = spirit_reg_write(spirit, PROTOCOL0_BASE, &regval, 1);
    }

  return ret;
}

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

uint8_t spirit_pktcommon_get_maxretx(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the PROTOCOL0 register value */

  spirit_reg_read(spirit, PROTOCOL0_BASE, &regval, 1);

  /* Build the value to be written */

  return ((regval & PROTOCOL0_NMAX_RETX_MASK) >> 4);
}

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
  spirit_pktcommon_get_txackreq(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the PROTOCOL0 register value */

  spirit_reg_read(spirit, RX_PCKT_INFO_BASE, &regval, 1);

  /* Build the value to be written */

  return (enum spirit_functional_state_e)
          ((regval & TX_PCKT_INFO_NACK_RX) >> 2);
}

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

uint8_t spirit_pktcommon_get_rxsrcaddr(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the RX_ADDR_FIELD1 register value */

  spirit_reg_read(spirit, RX_ADDR_FIELD1_BASE, &regval, 1);

  /* Returns value */

  return regval;
}

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

uint8_t spirit_pktcommon_get_rxseqno(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the RX_PCKT_INFO register value */

  spirit_reg_read(spirit, RX_PCKT_INFO_BASE, &regval, 1);

  /* Obtains and returns the sequence number */

  return regval & 0x03;
}

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

uint8_t spirit_pktcommon_get_rxnak(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the RX_PCKT_INFO register value */

  spirit_reg_read(spirit, RX_PCKT_INFO_BASE, &regval, 1);

  /* Obtains and returns the RX nack bit */

  return (regval >> 2) & 0x01;
}

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

uint8_t spirit_pktcommon_get_txseqno(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the TX_PCKT_INFO register value */

  spirit_reg_read(spirit, TX_PCKT_INFO_BASE, &regval, 1);

  /* Obtains and returns the TX sequence number */

  return (regval >> 4) & 0x07;
}

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

uint8_t spirit_pktcommon_get_nretx(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the TX_PCKT_INFO register value */

  spirit_reg_read(spirit, TX_PCKT_INFO_BASE, &regval, 1);

  /* Obtains and returns the number of retransmission done */

  return (regval & 0x0f);
}

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
                                    enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Modify the register value: set or reset the control bit filtering */

  ret = spirit_reg_read(spirit, PCKT_FLT_OPTIONS_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Set or reset the CONTROL filtering enabling bit */

      if (newstate == S_ENABLE)
        {
          regval |= PCKT_FLT_OPTIONS_CONTROL_FILTERING_MASK;
        }
      else
        {
          regval &= ~PCKT_FLT_OPTIONS_CONTROL_FILTERING_MASK;
        }

      /* Write the new value to the PCKT_FLT_OPTIONS register */

      ret = spirit_reg_write(spirit, PCKT_FLT_OPTIONS_BASE, &regval, 1);
    }

  return ret;
}

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
  spirit_pktcommon_isenabled_ctrl_filter(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Read the register value */

  spirit_reg_read(spirit, PCKT_FLT_OPTIONS_BASE, &regval, 1);

  /* Gets the enable/disable bit in form of enum spirit_functional_state_e
   * type
   */

  if (regval & PCKT_FLT_OPTIONS_CONTROL_FILTERING_MASK)
    {
      return S_ENABLE;
    }
  else
    {
      return S_DISABLE;
    }
}
