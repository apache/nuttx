/******************************************************************************
 * drivers/wireless/spirit/lib/spirit_linearfifo.c
 *
 *  Copyright(c) 2015 STMicroelectronics
 *  Author: VMA division - AMS
 *  Version 3.2.2 08-July-2015
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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

#include "spirit_linearfifo.h"
#include "spirit_spi.h"

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_fifo_get_rxcount
 *
 * Description:
 *   Returns the number of elements in the Rx FIFO.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Number of elements in the Rx FIFO.
 *
 ******************************************************************************/

uint8_t spirit_fifo_get_rxcount(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the register value */

  spirit_reg_read(spirit, LINEAR_FIFO_STATUS0_BASE, &regval, 1);

  /* Build and return value */

  return (regval & 0x7f);
}

/******************************************************************************
 * Name: spirit_fifo_get_txcount
 *
 * Description:
 *   Returns the number of elements in the Tx FIFO.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Number of elements in the Tx FIFO.
 *
 ******************************************************************************/

uint8_t spirit_fifo_get_txcount(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the number of elements in TX FIFO and return the value */

  spirit_reg_read(spirit, LINEAR_FIFO_STATUS1_BASE, &regval, 1);

  /* Build and return value */

  return (regval & 0x7f);
}

/******************************************************************************
 * Name: spirit_fifo_set_rxalmostfull
 *
 * Description:
 *   Sets the almost full threshold for the Rx FIFO. When the number of
 *   elements in RX FIFO reaches this value an interrupt can be generated to
 *   the MCU.
 *
 *   NOTE: The almost full threshold is encountered from the top of the FIFO.
 *   For example, if it is set to 7 the almost full FIFO irq will be raised
 *   when the number of elements is equals to 96-7 = 89.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   threshold almost full threshold.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_fifo_set_rxalmostfull(FAR struct spirit_library_s *spirit,
                                 uint8_t threshold)
{
  uint8_t regval;

  /* Check the parameters */

  DEBUGASSERT(IS_FIFO_THR(threshold));

  /* Build the register value */

  regval = threshold & 0x7f;

  /* Writes the Almost Full threshold for RX in the corresponding register */

  return spirit_reg_write(spirit, FIFO_CONFIG3_RXAFTHR_BASE, &regval, 1);
}

/******************************************************************************
 * Name: spirit_fifo_get_rxalmostfull
 *
 * Description:
 *   Returns the almost full threshold for RX FIFO.
 *
 *   NOTE: The almost full threshold is encountered from the top of the FIFO.
 *   For example, if it is 7 the almost full FIFO irq will be raised when the
 *   number of elements is equals to 96-7 = 89.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Almost full threshold for Rx FIFO.
 *
 ******************************************************************************/

uint8_t spirit_fifo_get_rxalmostfull(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the almost full threshold for RX FIFO and return the value */

  spirit_reg_read(spirit, FIFO_CONFIG3_RXAFTHR_BASE, &regval, 1);

  /* Build and return value */

  return (regval & 0x7f);
}

/******************************************************************************
 * Name: spirit_fifo_set_rxalmostempty
 *
 * Description:
 *   Sets the almost empty threshold for the Rx FIFO. When the number of
 *   elements in RX FIFO reaches this value an interrupt can be generated to
 *   the MCU.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   threshold almost empty threshold.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_fifo_set_rxalmostempty(FAR struct spirit_library_s *spirit,
                                  uint8_t threshold)
{
  uint8_t regval;

  /* Check the parameters */

  DEBUGASSERT(IS_FIFO_THR(threshold));

  /* Build the register value */

  regval = threshold & 0x7f;

  /* Writes the Almost Empty threshold for RX in the corresponding register */

  return spirit_reg_write(spirit, FIFO_CONFIG2_RXAETHR_BASE, &regval, 1);
}

/******************************************************************************
 * Name: spirit_fifo_get_rxalmostempty
 *
 * Description:
 *   Returns the almost empty threshold for Rx FIFO.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Almost empty threshold for Rx FIFO.
 *
 ******************************************************************************/

uint8_t spirit_fifo_get_rxalmostempty(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the almost empty threshold for RX FIFO and returns the value */

  spirit_reg_read(spirit, FIFO_CONFIG2_RXAETHR_BASE, &regval, 1);

  /* Build and return value */

  return (regval & 0x7f);
}

/******************************************************************************
 * Name: spirit_fifo_set_txalmostfull
 *
 * Description:
 *   Sets the almost full threshold for the Tx FIFO. When the number of
 *   elements in TX FIFO reaches this value an interrupt can be generated to
 *   the MCU.
 *
 *   NOTE: The almost full threshold is encountered from the top of the FIFO.
 *   For example, if it is set to 7 the almost full FIFO irq will be raised
 *   when the number of elements is equals to 96-7 = 89.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   threshold almost full threshold.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_fifo_set_txalmostfull(FAR struct spirit_library_s *spirit,
                                 uint8_t threshold)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_FIFO_THR(threshold));

  /* Reads the register value */

  ret = spirit_reg_read(spirit, FIFO_CONFIG1_TXAFTHR_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the register value */

      regval &= 0x80;
      regval |= threshold;

      /* Writes the Almost Full threshold for Tx in the corresponding
       * register
       */

      ret = spirit_reg_write(spirit, FIFO_CONFIG1_TXAFTHR_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_fifo_get_txalmostfull
 *
 * Description:
 *   Returns the almost full threshold for Tx FIFO.
 *
 *   NOTE: The almost full threshold is encountered from the top of the FIFO.
 *   For example, if it is set to 7 the almost full FIFO irq will be raised
 *   when the number of elements is equals to 96-7 = 89.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Almost full threshold for Tx FIFO.
 *
 ******************************************************************************/

uint8_t spirit_fifo_get_txalmostfull(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the almost full threshold for Tx FIFO and returns the value */

  spirit_reg_read(spirit, FIFO_CONFIG1_TXAFTHR_BASE, &regval, 1);

  /* Build and returns value */

  return (regval & 0x7f);
}

/******************************************************************************
 * Name: spirit_fifo_set_txalmostempty
 *
 * Description:
 *   Sets the almost empty threshold for the Tx FIFO. When the number of
 *   elements in Tx FIFO reaches this value an interrupt can can be generated
 *   to the MCU.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   threshold: almost empty threshold.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_fifo_set_txalmostempty(FAR struct spirit_library_s *spirit,
                                  uint8_t threshold)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_FIFO_THR(threshold));

  /* Reads the register value */

  ret = spirit_reg_read(spirit, FIFO_CONFIG0_TXAETHR_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Build the register value */

      regval &= 0x80;
      regval |= threshold;

      /* Writes the Almost Empty threshold for Tx in the corresponding
       * register
       */

      ret = spirit_reg_write(spirit, FIFO_CONFIG0_TXAETHR_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_fifo_get_txalmostempty
 *
 * Description:
 *   Returns the almost empty threshold for Tx FIFO.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Almost empty threshold for Tx FIFO.
 *
 ******************************************************************************/

uint8_t spirit_fifo_get_txalmostempty(FAR struct spirit_library_s *spirit)
{
  uint8_t regval;

  /* Reads the almost empty threshold for TX FIFO and returns the value */

  spirit_reg_read(spirit, FIFO_CONFIG0_TXAETHR_BASE, &regval, 1);

  /* Build and return value */

  return (regval & 0x7f);
}
