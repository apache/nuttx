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
 *   in the struct pktbasic_init_s.  Notice that this function sets the
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
                               FAR const struct pktbasic_init_s *pktpasic)
{
  uint8_t regval[4];
  uint8_t pktlenwidth;
  uint8_t i;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_PKT_PREAMBLE_LENGTH(pktpasic->premblen));
  DEBUGASSERT(IS_PKT_SYNC_LENGTH(pktpasic->synclen));
  DEBUGASSERT(IS_PKT_CRC_MODE(pktpasic->crcmode));
  DEBUGASSERT(IS_PKT_LENGTH_WIDTH_BITS(pktpasic->pktlenwidth));
  DEBUGASSERT(IS_PKT_FIX_VAR_LENGTH(pktpasic->fixedvarlen));
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
   * in basic) */

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

  regval[2] = (uint8_t)pktpasic->premblen | (uint8_t)pktpasic->synclen |
              (uint8_t)pktpasic->fixedvarlen;

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
 * Name: spirit_pktbase_get_addrfield
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
  spirit_pktbase_get_addrfield(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the PCKTCTRL4 register value */

  (void)spirit_reg_read(spirit, PCKTCTRL4_BASE, &regval, 1);

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

  if (spirit_pktbase_get_addrfield(spirit))
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
 * Name: spirit_pktbasic_rxpktlen
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

uint16_t spirit_pktbasic_rxpktlen(FAR struct spirit_library_s *spirit)
{
  uint8_t regval[2];
  uint16_t oversize = 0;

  /* Computes the oversize (address + control) size */

  if (spirit_pktbase_get_addrfield(spirit))
    {
      oversize = 1;
    }

  oversize += (uint16_t)spirit_pktcommon_get_controllen(spirit);

  /* Reads the RX_PCKT_LENx registers value */

  (void)spirit_reg_read(spirit, RX_PCKT_LEN1_BASE, regval, 2);

  /* Rebuild and return the length field */

  return (((((uint16_t) regval[0]) << 8) + (uint16_t)regval[1]) - oversize);
}
