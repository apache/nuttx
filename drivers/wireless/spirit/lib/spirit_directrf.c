/******************************************************************************
 * drivers/wireless/spirit/lib/spirit_directrf.c
 * Configuration and management of SPIRIT direct transmission / receive modes.
 *
 *  Copyright(c) 2015 STMicroelectronics
 *  Author: VMA division - AMS
 *  Version 3.2.2 08-July-2015
 *
 *  Adapted for NuttX by:
 *  Author:  Gregory Nutt <gnutt@nuttx.org>
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

#include "spirit_directrf.h"
#include "spirit_spi.h"

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_directrf_set_rxmode
 *
 * Description:
 *   Sets the DirectRF RX mode of SPIRIT.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   directrx - Code of the desired mode.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_directrf_set_rxmode(FAR struct spirit_library_s *spirit,
                               enum spirit_directrx_e directrx)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_DIRECT_RX(directrx));

  /* Reads the register value */

  ret = spirit_reg_read(spirit, PCKTCTRL3_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the value to be stored */

      regval &= ~PCKTCTRL3_RX_MODE_MASK;
      regval |= (uint8_t)directrx;

      /* Write value to register */

      ret = spirit_reg_write(spirit, PCKTCTRL3_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_directrf_get_rxmode
 *
 * Description:
 *   Returns the DirectRF RX mode of SPIRIT.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Direct Rx mode.
 *
 ******************************************************************************/

enum spirit_directrx_e
  spirit_directrf_get_rxmode(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the register value and mask the RX_Mode field */

  (void)spirit_reg_read(spirit, PCKTCTRL3_BASE, &regval, 1);

  /* Rebuild and return value */

  return (enum spirit_directrx_e)(regval & 0x30);
}

/******************************************************************************
 * Name: spirit_directrf_set_txmode
 *
 * Description:
 *   Sets the TX mode of SPIRIT.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   directtx - Code of the desired source.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_directrf_set_txmode(FAR struct spirit_library_s *spirit,
                               enum spirit_directtx_e directtx)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_DIRECT_TX(directtx));

  /* Reads the register value */

  ret = spirit_reg_read(spirit, PCKTCTRL1_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the value to be stored */

      regval &= ~PCKTCTRL1_TX_SOURCE_MASK;
      regval |= (uint8_t) directtx;

      /* Write value to register */

      ret = spirit_reg_write(spirit, PCKTCTRL1_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_directrf_get_txmode
 *
 * Description:
 *   Returns the DirectRF TX mode of SPIRIT.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Direct Tx mode.
 *
 ******************************************************************************/

enum spirit_directtx_e
  spirit_directrf_get_txmode(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the register value and mask the RX_Mode field */

  (void)spirit_reg_read(spirit, PCKTCTRL1_BASE, &regval, 1);

  /* Returns value */

  return (enum spirit_directtx_e)(regval & 0x0c);
}
