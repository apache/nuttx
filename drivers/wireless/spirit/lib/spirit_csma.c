/******************************************************************************
 * drivers/wireless/spirit/spirit_csma.c
 * Configuration and management of SPIRIT CSMA.
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
#include <debug.h>

#include "spirit_csma.h"
#include "spirit_spi.h"

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_csma_initialize
 *
 * Description:
 *   Initializes the SPIRIT CSMA according to the specified parameters in the
 *   struct spirit_csma_init_s.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   csmainit - Reference to the Csma init structure.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value is returned on any failure.
 *
 ******************************************************************************/

int spirit_csma_initialize(FAR struct spirit_library_s *spirit,
                           FAR struct spirit_csma_init_s *csmainit)
{
  uint8_t regval[5];
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(csmainit->csmapersistent));
  DEBUGASSERT(IS_CCA_PERIOD(csmainit->multbit));
  DEBUGASSERT(IS_CSMA_LENGTH(csmainit->ccalen));
  DEBUGASSERT(IS_BU_COUNTER_SEED(csmainit->seed));
  DEBUGASSERT(IS_BU_PRESCALER(csmainit->prescaler));
  DEBUGASSERT(IS_CMAX_NB(csmainit->maxnb));

  /* CSMA BU counter seed (MSB) config */

  regval[0] = (uint8_t)(csmainit->seed >> 8);

  /* CSMA BU counter seed (LSB) config */

  regval[1] = (uint8_t)csmainit->seed;

  /* CSMA BU prescaler config and CCA period config */

  regval[2] = (csmainit->prescaler << 2) | csmainit->multbit;

  /* CSMA CCA length config and max number of back-off */

  regval[3] = (csmainit->ccalen | csmainit->maxnb);

  /* Reads the PROTOCOL1_BASE register value, to write the SEED_RELOAD and
   * CSMA_PERS_ON fields/
   */

  ret = spirit_reg_read(spirit, PROTOCOL1_BASE, &regval[4], 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Writes the new value for persistent mode */

  if (csmainit->csmapersistent == S_ENABLE)
    {
      regval[4] |= PROTOCOL1_CSMA_PERS_ON_MASK;
    }
  else
    {
      regval[4] &= ~PROTOCOL1_CSMA_PERS_ON_MASK;
    }

  /* Writes PROTOCOL1_BASE register */

  ret = spirit_reg_write(spirit, PROTOCOL1_BASE, &regval[4], 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Writes CSMA_CONFIGx_BASE registers */

  ret = spirit_reg_write(spirit, CSMA_CONFIG3_BASE, regval, 4);
  return ret;
}
