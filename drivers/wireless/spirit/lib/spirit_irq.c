/******************************************************************************
 * drivers/wireless/spirit/lib/spirit_irq.c
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

#include "spirit_types.h"
#include "spirit_spi.h"
#include "spirit_irq.h"

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_irq_disable_all
 *
 * Description:
 *   Ssets the IRQ mask registers to 0x00000000, disabling all IRQs.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_irq_disable_all(FAR struct spirit_library_s *spirit)
{
  uint8_t regval[4] =
  {
    0x00, 0x00, 0x00, 0x00
  };

  /* Writes the IRQ_MASK registers */

  return spirit_reg_write(spirit, IRQ_MASK3_BASE, regval, 4);
}

/******************************************************************************
 * Name: spirit_irq_set_mask
 *
 * Description:
 *   Enables/disables all the IRQs according to the user defined irqset
 *   structure.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   irqset - Pointer to a variable of type struct spirit_irqset_s, through
 *            which the user enable specific IRQs. This parameter is a
 *            pointer to a struct spirit_irqset_s.
 *
 *            For example suppose to enable only the two IRQ Low Battery Level
 *            and Tx Data Sent:
 *
 *              struct spirit_irqset_s g_irqset = {0};
 *              g_irqset.IRQ_LOW_BATT_LVL = 1;
 *              g_irqset.IRQ_TX_DATA_SENT = 1;
 *              spirit_irq_setmask(&g_irqset);
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

#ifndef CONFIG_ENDIAN_BIG
int spirit_irq_set_mask(FAR struct spirit_library_s *spirit,
                        FAR struct spirit_irqset_s *irqset)
{
  /* Writes the IRQ_MASK registers */

  return spirit_reg_write(spirit, IRQ_MASK3_BASE, (FAR uint8_t *)irqset, 4);
}
#endif

/******************************************************************************
 * Name: spirit_irq_enable
 *
 * Description:
 *   Enables or disables a specific IRQ.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   irq IRQ to enable or disable.
 *   newstate - new state for the IRQ.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_irq_enable(FAR struct spirit_library_s *spirit,
                      enum spirit_irq_e irq,
                      enum spirit_functional_state_e newstate)
{
  uint8_t regval[4];
  uint32_t dwirqs = 0;
  int ret;
  int i;
  int j;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_IRQ_LIST(irq));
  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the IRQ_MASK registers */

  ret = spirit_reg_read(spirit, IRQ_MASK3_BASE, regval, 4);
  if (ret < 0)
    {
      return ret;
    }

  /* Build the IRQ mask word */

  for (i = 0; i < 4; i++)
    {
      dwirqs += ((uint32_t) regval[i]) << (8 * (3 - i));
    }

  /* Rebuild the new mask according to user request */

  if (newstate == S_DISABLE)
    {
      dwirqs &= (~irq);
    }
  else
    {
      dwirqs |= (irq);
    }

  /* Build the array of bytes to write in the IRQ_MASK registers */

  for (j = 0; j < 4; j++)
    {
      regval[j] = (uint8_t)(dwirqs >> (8 * (3 - j)));
    }

  /* Writes the new IRQ mask in the corresponding registers */

  return spirit_reg_write(spirit, IRQ_MASK3_BASE, regval, 4);
}

/******************************************************************************
 * Name: spirit_irt_get_mask
 *
 * Description:
 *   Fills a pointer to a structure of struct spirit_irqset_s type with the
 *   content of the IRQ_MASK registers.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   pirqmask - Pointer to a variable of type struct spirit_irqset_s, through
 *              which the user can read which IRQs are enabled. All the
 *              bitfields equals to zero correspond to enabled IRQs, while
 *              all the bitfields equals to one correspond to disabled IRQs.
 *
 *              For example suppose that the Power On Reset and RX Data
 *              ready are the only enabled IRQs.
 *
 *                struct spirit_irqset_s g_irqmask;
 *                spirit_irq_get_pending(&g_irqmask);
 *
 *              Then g_irqmask.IRQ_POR and g_irqmask.IRQ_RX_DATA_READY are
 *              equal to 0 while all the other bitfields are equal to one.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

#ifndef CONFIG_ENDIAN_BIG
int spirit_irt_get_mask(FAR struct spirit_library_s *spirit,
                        FAR struct spirit_irqset_s *pirqmask)
{
  /* Reads IRQ_MASK registers */

  return spirit_reg_read(spirit, IRQ_MASK3_BASE, (FAR uint8_t *)pirqmask, 4);
}
#endif

/******************************************************************************
 * Name: spirit_irq_get_pending
 *
 * Description:
 *   Fills a pointer to a structure of struct spirit_irqset_s type with the
 *   content of the IRQ_STATUS registers.  NOTE:  Status bits will be cleared
 *   after they are read.
 *
 * Input Parameters:
 *   spirit     - Reference to a Spirit library state structure instance
 *   pirqstatus - A pointer to a variable of type struct spirit_irqset_s,
 *                through which the user can receive the status of all the
 *                IRQs. All the bitfields equals to one correspond to the
 *                raised interrupts.
 *
 *                For example suppose that the XO settling timeout is raised
 *                as well as the Sync word detection.
 *
 *                  struct spirit_irqset_s g_irqstatus;
 *                  spirit_irq_get_pending(&g_irqstatus);
 *
 *                Then g_irqstatus.IRQ_XO_COUNT_EXPIRED and
 *                g_irqstatus.IRQ_VALID_SYNC are equals to 1* while all the
 *                other bitfields are equals to zero.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

#ifndef CONFIG_ENDIAN_BIG
int spirit_irq_get_pending(FAR struct spirit_library_s *spirit,
                           FAR struct spirit_irqset_s *pirqstatus)
{
  /* Reads IRQ_STATUS registers */

  return spirit_reg_read(spirit, IRQ_STATUS3_BASE,
                        (FAR uint8_t *)pirqstatus, 4);
}
#endif

/******************************************************************************
 * Name: spirit_irq_clr_pending
 *
 * Description:
 *   Clear the IRQ status registers.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_irq_clr_pending(FAR struct spirit_library_s *spirit)
{
  uint8_t regval[4];

  /* Reads the IRQ_STATUS registers clearing all the flags */

  return spirit_reg_read(spirit, IRQ_STATUS3_BASE, regval, 4);
}

/******************************************************************************
 * Name: spirit_irq_is_pending
 *
 * Description:
 *   Checks if a specific IRQ has been generated.  The call resets all the
 *   IRQ status, so it can't be used in case of multiple raising interrupts.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   flag IRQ flag to be checked.
 *         This parameter can be any value of enum spirit_irq_e.
 *
 * Returned Value:
 *   true or false.
 *
 ******************************************************************************/

bool spirit_irq_is_pending(FAR struct spirit_library_s *spirit,
                           enum spirit_irq_e flag)
{
  uint8_t regval[4];
  uint32_t dwirqs = 0;
  int ret;
  int i;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_IRQ_LIST(flag));

  /* Read status registers */

  ret = spirit_reg_read(spirit, IRQ_STATUS3_BASE, regval, 4);
  if (ret < 0)
    {
      return ret;
    }

  /* Build the status word */

  for (i = 0; i < 4; i++)
    {
      dwirqs |= ((uint32_t)regval[i]) << (8 * (3 - i));
    }

  return ((dwirqs & flag) != 0);
}
