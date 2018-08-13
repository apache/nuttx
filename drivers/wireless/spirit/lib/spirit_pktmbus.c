/******************************************************************************
 * drivers/wireless/spirit/spirit_pktmbus.c
 * Configuration and management of SPIRIT MBUS packets.
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

#include "spirit_pktmbus.h"
#include "spirit_radio.h"
#include "spirit_spi.h"

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_pktmbus_initialize
 *
 * Description:
 *   Initializes the SPIRIT MBUS packet according to the specified parameters
 *   in the struct spirit_pktmbus_init_s structure.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   mbusinit - Pointer to a struct spirit_pktmbus_init_s structure that
 *              contains the configuration information for the specified
 *              SPIRIT MBUS PACKET FORMAT.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktmbus_initialize(FAR struct spirit_library_s *spirit,
                              FAR const struct spirit_pktmbus_init_s *mbusinit)
{
  uint8_t regval[3];

  /* Check the parameters */

  DEBUGASSERT(IS_MBUS_SUBMODE(mbusinit->submode));

  /* Packet format config */

  spirit_pktmbus_set_format(spirit);
  spirit_pktcommon_enable_crcfilter(spirit, S_DISABLE);
  spirit_radio_enable_csblanking(spirit, S_ENABLE);

  /* Preamble, postamble and submode config */

  regval[0] = mbusinit->preamblen;
  regval[1] = mbusinit->postamblen;
  regval[2] = (uint8_t) mbusinit->submode;

  /* Writes the new values on the MBUS_PRMBL registers */

  return spirit_reg_write(spirit, MBUS_PRMBL_BASE, regval, 3);
}

/******************************************************************************
 * Name: spirit_pktmbus_get_setup
 *
 * Description:
 *   Returns the SPIRIT MBUS packet structure according to the specified
 *   parameters in the registers.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   mbusinit - MBUS packet init structure.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktmbus_get_setup(FAR struct spirit_library_s *spirit,
                             FAR struct spirit_pktmbus_init_s *mbusinit)
{
  uint8_t regval[3];
  int ret;

  /* Reads the MBUS regs value */

  ret = spirit_reg_read(spirit, MBUS_PRMBL_BASE, regval, 3);
  if (ret >= 0)
    {
      /* Fit the structure */

      mbusinit->preamblen = regval[0];
      mbusinit->postamblen = regval[1];
      mbusinit->submode = (enum spirit_mbus_submode_e) (regval[2] & 0x0E);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_pktmbus_set_format
 *
 * Description:
 *   Configures the MBUS packet format as the one used by SPIRIT.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktmbus_set_format(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;
  int ret;

  /* Reads the PCKTCTRL3 register value */

  ret = spirit_reg_read(spirit, PCKTCTRL3_BASE, &regval, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Sets format bits. Also set to 0 the direct RX mode bits */

  regval &= 0x0F;
  regval |= ((uint8_t) PCKTCTRL3_PCKT_FRMT_MBUS);

  /* Writes value on the PCKTCTRL3 register */

  ret = spirit_reg_write(spirit, PCKTCTRL3_BASE, &regval, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Reads the PCKTCTRL1 register value */

  ret = spirit_reg_read(spirit, PCKTCTRL1_BASE, &regval, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Build the new value. Set to 0 the direct TX mode bits */

  regval &= 0xf3;

  /* Writes the value on the PCKTCTRL1 register */

  ret = spirit_reg_write(spirit, PCKTCTRL1_BASE, &regval, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Reads the PROTOCOL1 register */

  ret = spirit_reg_read(spirit, PROTOCOL1_BASE, &regval, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Mask a reserved bit */

  regval &= ~0x20;

  /* Writes the value on the PROTOCOL1 register */

  return spirit_reg_write(spirit, PROTOCOL1_BASE, &regval, 1);
}

/******************************************************************************
 * Name: spirit_pktmbus_set_preamble
 *
 * Description:
 *   Sets how many chip sequence “01” shall be added in the preamble respect
 *   to the minimum value as defined according to the specified sub-mode.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   preamble - The number of chip sequence.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktmbus_set_preamble(FAR struct spirit_library_s *spirit,
                                uint8_t preamble)
{
  /* Modifies the MBUS_PRMBL register value */

  return spirit_reg_write(spirit, MBUS_PRMBL_BASE, &preamble, 1);
}

/******************************************************************************
 * Name: spirit_pktmbus_get_preamble
 *
 * Description:
 *   Returns how many chip sequence "01" are added in the preamble respect to
 *   the minimum value as defined according to the specified sub-mode.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Preable in number of "01" chip sequences.
 *
 ******************************************************************************/

uint8_t spirit_pktmbus_get_preamble(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Modifies the MBUS_PRMBL register value */

  (void)spirit_reg_read(spirit, MBUS_PRMBL_BASE, &regval, 1);

  /* Return value */

  return regval;
}

/******************************************************************************
 * Name: spirit_pktmbus_set_postamble
 *
 * Description:
 *   Sets how many chip sequence “01” will be used in postamble
 *
 * Input Parameters:
 *   spirit    - Reference to a Spirit library state structure instance
 *   postamble - The number of chip sequence.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktmbus_set_postamble(FAR struct spirit_library_s *spirit,
                                 uint8_t postamble)
{
  /* Modifies the MBUS_PSTMBL register value */

  return spirit_reg_write(spirit, MBUS_PSTMBL_BASE, &postamble, 1);
}

/******************************************************************************
 * Name: spirit_pktmbus_get_postamble
 *
 * Description:
 *   Returns how many chip sequence "01" are used in the postamble
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Postamble in number of "01" chip sequences.
 *
 ******************************************************************************/

uint8_t spirit_pktmbus_get_postamble(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the MBUS_PSTMBL register */

  (void)spirit_reg_read(spirit, MBUS_PSTMBL_BASE, &regval, 1);

  /* Returns value */

  return regval;
}

/******************************************************************************
 * Name: spirit_pktmbus_set_submode
 *
 * Description:
 *   Sets the MBUS submode used.
 *
 * Input Parameters:
 *   spirit  - Reference to a Spirit library state structure instance
 *   submode - The submode used.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktmbus_set_submode(FAR struct spirit_library_s *spirit,
                               enum spirit_mbus_submode_e submode)
{
  /* Modifies the MBUS_CTRL register value */

  return spirit_reg_write(spirit, MBUS_CTRL_BASE, (FAR uint8_t *)submode, 1);
}

/******************************************************************************
 * Name: spirit_pktmbus_get_submode
 *
 * Description:
 *   Returns the MBUS submode used.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   MBUS submode.
 *
 ******************************************************************************/

enum spirit_mbus_submode_e
  spirit_pktmbus_get_submode(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the MBUS_CTRL register value */

  (void)spirit_reg_read(spirit, MBUS_CTRL_BASE, &regval, 1);

  /* Returns value */

  return (enum spirit_mbus_submode_e) regval;
}

/******************************************************************************
 * Name: spirit_pktmbus_set_payloadlen
 *
 * Description:
 *   Sets the payload length for SPIRIT MBUS packets.
 *
 * Input Parameters:
 *   spirit     - Reference to a Spirit library state structure instance
 *   payloadlen - Payload length in bytes.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_pktmbus_set_payloadlen(FAR struct spirit_library_s *spirit,
                                  uint16_t payloadlen)
{
  uint8_t regval[2];

  /* Computes PCKTLEN0 value from payloadlen */

  regval[1] = BUILD_PCKTLEN0(payloadlen);

  /* Computes PCKTLEN1 value from payloadlen */

  regval[0] = BUILD_PCKTLEN1(payloadlen);

  /* Writes data on the PCKTLEN1/0 register */

  return spirit_reg_write(spirit, PCKTLEN1_BASE, regval, 2);
}

/******************************************************************************
 * Name: spirit_pktmbus_get_payloadlen
 *
 * Description:
 *   Returns the payload length for SPIRIT MBUS packets.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Payload length in bytes.
 *
 ******************************************************************************/

uint16_t spirit_pktmbus_get_payloadlen(FAR struct spirit_library_s *spirit)
{
  uint8_t regval[2];

  /* Reads the packet length registers */

  (void)spirit_reg_read(spirit, PCKTLEN1_BASE, regval, 2);

  /* Returns the packet length */

  return ((((uint16_t)regval[0]) << 8) + (uint16_t)regval[1]);
}
