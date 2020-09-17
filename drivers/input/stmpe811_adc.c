/****************************************************************************
 * drivers/input/stmpe811_adc.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* References:
 *   "STMPE811 S-Touch advanced resistive touchscreen controller with 8-bit
 *    GPIO expander," Doc ID 14489 Rev 6, CD00186725, STMicroelectronics"
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/input/stmpe811.h>

#include "stmpe811.h"

#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_STMPE811) && \
   !defined(CONFIG_STMPE811_ADC_DISABLE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stmpe811_adcinitialize
 *
 * Description:
 *  Configure for ADC mode operation.  Set overall ADC ADC timing that
 *  applies to all pins.
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe811_instantiate
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stmpe811_adcinitialize(STMPE811_HANDLE handle)
{
  FAR struct stmpe811_dev_s *priv = (FAR struct stmpe811_dev_s *)handle;
  uint8_t regval;
  int ret;

  DEBUGASSERT(handle);

  /* Get exclusive access to the device structure */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      ierr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  /* Enable Clocking for ADC */

  regval = stmpe811_getreg8(priv, STMPE811_SYS_CTRL2);
  regval &= ~SYS_CTRL2_ADC_OFF;
  stmpe811_putreg8(priv, STMPE811_SYS_CTRL2, regval);

  /* Select Sample Time, bit number and ADC Reference */

  stmpe811_putreg8(priv, STMPE811_ADC_CTRL1, priv->config->ctrl1);

  /* Wait for 20 ms */

  up_mdelay(20);

  /* Select the ADC clock speed */

  stmpe811_putreg8(priv, STMPE811_ADC_CTRL2, priv->config->ctrl2);

  /* Mark ADC initialized */

  priv->flags |= STMPE811_FLAGS_ADC_INITIALIZED;
  nxsem_post(&priv->exclsem);
  return OK;
}

/****************************************************************************
 * Name: stmpe811_adcconfig
 *
 * Description:
 *  Configure a pin for ADC input.
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe811_instantiate
 *   pin       - The ADC pin number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stmpe811_adcconfig(STMPE811_HANDLE handle, int pin)
{
  FAR struct stmpe811_dev_s *priv = (FAR struct stmpe811_dev_s *)handle;
  uint8_t pinmask = STMPE811_GPIO_PIN(pin);
  uint8_t regval;
  int ret;

  DEBUGASSERT(handle && (unsigned)pin < STMPE811_ADC_NPINS);

  /* Get exclusive access to the device structure */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      ierr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  /* Make sure that the pin is not already in use */

  if ((priv->inuse & pinmask) != 0)
    {
      ierr("ERROR: PIN%d is already in-use\n", pin);
      nxsem_post(&priv->exclsem);
      return -EBUSY;
    }

  /* Clear the alternate function bit for the pin, making it an ADC input
   * (or perhaps an an external reference, depending on the state of the
   * ADC_CTRL1_REF_SEL bit).
   */

  regval  = stmpe811_getreg8(priv, STMPE811_GPIO_AF);
  regval &= ~pinmask;
  stmpe811_putreg8(priv, STMPE811_GPIO_AF, regval);

  /* Mark the pin as 'in use' */

  priv->inuse |= pinmask;
  nxsem_post(&priv->exclsem);
  return OK;
}

/****************************************************************************
 * Name: stmpe811_adcread
 *
 * Description:
 *  Read the converted analog input value from the select pin.
 *
 * Input Parameters:
 *   handle    - The handle previously returned by stmpe811_instantiate
 *   pin       - The ADC pin number
 *
 * Returned Value:
 *   The converted value (there is no error reporting mechanism).
 *
 ****************************************************************************/

uint16_t stmpe811_adcread(STMPE811_HANDLE handle, int pin)
{
  FAR struct stmpe811_dev_s *priv = (FAR struct stmpe811_dev_s *)handle;
  uint8_t pinmask = STMPE811_GPIO_PIN(pin);
  uint8_t regval;
  int i;
  int ret;

  DEBUGASSERT(handle && (unsigned)pin < 8);

  /* Get exclusive access to the device structure */

  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      ierr("ERROR: nxsem_wait failed: %d\n", ret);
      return ret;
    }

  /* Request AD conversion by setting the bit corresponding the pin in the
   * ADC CAPT register.
   */

  stmpe811_putreg8(priv, STMPE811_ADC_CAPT, pinmask);

  /* Wait for the conversion to complete.  The ADC CAPT register reads '1'
   * if conversion is completed. Reads '0' if conversion is in progress.
   * Try three times before giving up.
   */

  for (i = 0; i < 3; i++)
    {
      /* The worst case ADC conversion time is (nominally) 56.4 uS. The
       * following nxsig_usleep() looks nice but in reality, nxsig_usleep()
       * normal does not have that kind of precision (it will probably end
       * up waiting 10 MS).
       */

      nxsig_usleep(57);

      /* Check if the conversion is complete */

      regval = stmpe811_getreg8(priv, STMPE811_ADC_CAPT);
      if ((regval & pinmask) != 0)
        {
          break;
        }
    }

  /* At the completion of the conversion, return whatever we read from
   * from the channel register associated with the pin.
   */

  return stmpe811_getreg16(priv, STMPE811_ADC_DATACH(pin));
}

#endif /* CONFIG_INPUT && CONFIG_INPUT_STMPE811 && !CONFIG_STMPE811_ADC_DISABLE */
