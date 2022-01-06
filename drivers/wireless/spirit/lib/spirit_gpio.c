/******************************************************************************
 * drivers/wireless/spirit/lib/spirit_gpio.c
 *
 *  Copyright(c) 2015 STMicroelectronics
 *  Author: VMA division - AMS
 *  Version 3.2.2 08-July-2015
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
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <stdint.h>
#include <assert.h>

#include "spirit_gpio.h"
#include "spirit_spi.h"

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/******************************************************************************
 * Name: spirit_gpio_initialize
 *
 * Description:
 *   Initializes the Spirit GPIOx according to the specified parameters in
 *   the gpioinit parameter.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   gpioinit - A pointer to a struct spirit_gpio_init_s structure that
 *              contains the configuration information for the specified
 *              SPIRIT GPIO.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_gpio_initialize(FAR struct spirit_library_s *spirit,
                           FAR const struct spirit_gpio_init_s *gpioinit)
{
  uint8_t regval = 0x00;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_GPIO(gpioinit->gpiopin));
  DEBUGASSERT(IS_SPIRIT_GPIO_MODE(gpioinit->gpiomode));
  DEBUGASSERT(IS_SPIRIT_GPIO_IO(gpioinit->gpioio));

  regval = ((uint8_t)(gpioinit->gpiomode) | (uint8_t)(gpioinit->gpioio));
  return spirit_reg_write(spirit, gpioinit->gpiopin, &regval, 1);
}

/******************************************************************************
 * Name: spirit_gpio_enable_tempsensor
 *
 * Description:
 *   Enables or Disables the output of temperature sensor on SPIRIT GPIO_0.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - Bew state for temperature sensor.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_gpio_enable_tempsensor(FAR struct spirit_library_s *spirit,
                                  enum spirit_functional_state_e newstate)
{
  uint8_t regval      = 0;
  uint8_t gpio0regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the ANA_FUNC_CONF0 register and mask the result to enable or disable
   * the temperature sensor.
   */

  ret = spirit_reg_read(spirit, ANA_FUNC_CONF0_BASE, &regval, 1);
  if (ret >= 0)
    {
      if (newstate == S_ENABLE)
        {
          regval |= TEMPERATURE_SENSOR_MASK;
        }
      else
        {
          regval &= (~TEMPERATURE_SENSOR_MASK);
          gpio0regval = 0x0a;       /* Default value */
        }

      ret = spirit_reg_write(spirit, ANA_FUNC_CONF0_BASE, &regval, 1);
      if (ret >= 0)
        {
          /* Sets the SPIRIT GPIO_0 according to input request */

          ret = spirit_reg_write(spirit, GPIO0_CONF_BASE, &gpio0regval, 1);
        }
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_gpio_set_outputlevel
 *
 * Description:
 *   Forces SPIRIT GPIO_x configured as digital output, to VDD or GND.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   gpio   - Specifies the GPIO to be configured.  This parameter can be one
 *            of following values:
 *
 *              SPIRIT_GPIO_0: SPIRIT GPIO_0
 *              SPIRIT_GPIO_1: SPIRIT GPIO_1
 *              SPIRIT_GPIO_2: SPIRIT GPIO_2
 *              SPIRIT_GPIO_3: SPIRIT GPIO_3
 *
 *   level  - Specifies the level.  This parameter can be: HIGH or LOW.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_gpio_set_outputlevel(FAR struct spirit_library_s *spirit,
                                enum spirit_gpio_pin_e gpio,
                                enum spirit_outputlevel_e level)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_GPIO(gpio));
  DEBUGASSERT(IS_SPIRIT_GPIO_LEVEL(level));

  /* Reads the SPIRIT_GPIOx register and mask the GPIO_SELECT field */

  ret = spirit_reg_read(spirit, gpio, &regval, 1);
  if (ret >= 0)
    {
      regval &= 0x04;

      /* Sets the value of the SPIRIT GPIO register according to the
       * specified level.
       */

      if (level == HIGH)
        {
          regval |= (uint8_t)SPIRIT_GPIO_DIG_OUT_VDD |
                    (uint8_t)SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP;
        }
     else
        {
          regval |= (uint8_t)SPIRIT_GPIO_DIG_OUT_GND |
                    (uint8_t)SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP;
        }

      /* Write to the SPIRIT GPIO register */

      ret = spirit_reg_write(spirit, gpio, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_gpio_get_outputlevel
 *
 * Description:
 *   Returns output value (VDD or GND) of SPIRIT GPIO_x, when it is configured
 *   as digital output.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *   gpio   - Specifies the GPIO to be read.  This parameter can be one of
 *            following values:
 *
 *              SPIRIT_GPIO_0: SPIRIT GPIO_0
 *              SPIRIT_GPIO_1: SPIRIT GPIO_1
 *              SPIRIT_GPIO_2: SPIRIT GPIO_2
 *              SPIRIT_GPIO_3: SPIRIT GPIO_3
 *
 * Returned Value:
 *   Logical level of selected GPIO configured as digital output. This
 *   parameter can be: HIGH or LOW.
 *
 ******************************************************************************/

enum spirit_outputlevel_e
  spirit_gpio_get_outputlevel(FAR struct spirit_library_s *spirit,
                              enum spirit_gpio_pin_e gpio)
{
  enum spirit_outputlevel_e level;
  uint8_t regval = 0;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_GPIO(gpio));

  /* Reads the SPIRIT_GPIOx register */

  spirit_reg_read(spirit, gpio, &regval, 1);

  /* Mask the GPIO_SELECT field and returns the value according */

  regval &= 0xf8;
  if (regval == SPIRIT_GPIO_DIG_OUT_VDD)
    {
      level = HIGH;
    }
  else
    {
      level = LOW;
    }

  return level;
}

/******************************************************************************
 * Name: spirit_gpio_enable_clockoutput
 *
 * Description:
 *   Enables or Disables the MCU clock output.
 *
 * Input Parameters:
 *   spirit   - Reference to a Spirit library state structure instance
 *   newstate - New state for the MCU clock output.  This parameter can be:
 *              S_ENABLE or S_DISABLE.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_gpio_enable_clockoutput(FAR struct spirit_library_s *spirit,
                                   enum spirit_functional_state_e newstate)
{
  uint8_t regval;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_FUNCTIONAL_STATE(newstate));

  /* Reads the MCU_CK_CONF register and mask the result to enable or disable
   * the clock output.
   */

  ret = spirit_reg_read(spirit, MCU_CK_CONF_BASE, &regval, 1);
  if (ret >= 0)
    {
      if (newstate)
        {
          regval |= MCU_CK_ENABLE;
        }
      else
        {
          regval &= (~MCU_CK_ENABLE);
        }

      /* Write to the MCU_CK_CONF register */

      ret = spirit_reg_write(spirit, MCU_CK_CONF_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_gpio_clockoutput_initialize
 *
 * Description:
 *   Initializes the SPIRIT Clock Output according to the specified parameters
 *   in the xClockOutputInitStruct.
 *
 *   NOTE:
 *   The function spirit_gpio_enable_clockoutput() must be called in order to
 *   enable or disable the MCU clock dividers.
 *
 * Input Parameters:
 *   spirit      - Reference to a Spirit library state structure instance
 *   clockoutput - Pointer to a struct spirit_clockoutput_init_s structure
 *                 that contains the configuration information for the SPIRIT
 *                 Clock Output.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_gpio_clockoutput_initialize(
                   FAR struct spirit_library_s *spirit,
                   FAR const struct spirit_clockoutput_init_s *clockoutput)
{
  uint8_t regval = 0;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_CLOCK_OUTPUT_XO(clockoutput->xoprescaler));
  DEBUGASSERT(IS_SPIRIT_CLOCK_OUTPUT_RCO(clockoutput->rcoprescaler));
  DEBUGASSERT(IS_SPIRIT_CLOCK_OUTPUT_EXTRA_CYCLES(clockoutput->xtracycles));

  /* Calculates the register value to write according to the specified
   * configuration.
   */

  regval = ((uint8_t)(clockoutput->xoprescaler) |
            (uint8_t)(clockoutput->rcoprescaler) |
            (uint8_t)(clockoutput->xtracycles));

  /* Write to the the MCU_CLOCK register */

  return spirit_reg_write(spirit, MCU_CK_CONF_BASE, &regval, 1);
}

/******************************************************************************
 * Name: spirit_gpio_set_xoprescaler
 *
 * Description:
 *   Sets the XO ratio as clock output.
 *
 * Input Parameters:
 *   spirit      - Reference to a Spirit library state structure instance
 *   xoprescaler - the XO prescaler to be used as clock output.  This
 *                 parameter can be any value from enum
 *                 spirit_clockoutput_xoprescaler_e .
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_gpio_set_xoprescaler(FAR struct spirit_library_s *spirit,
                        enum spirit_clockoutput_xoprescaler_e xoprescaler)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_CLOCK_OUTPUT_XO(xoprescaler));

  /* Reads the MCU_CLK_CONFIG register */

  ret = spirit_reg_read(spirit, MCU_CK_CONF_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Mask the XO_RATIO field and writes the new value */

      regval &= 0x61;
      regval |= (uint8_t)xoprescaler;

      /* Write to the new XO prescaler in the MCU_CLOCK register */

      ret = spirit_reg_write(spirit, MCU_CK_CONF_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_gpio_get_xoprescaler
 *
 * Description:
 *   Returns the settled XO prescaler as clock output.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Settled XO prescaler used for clock output.
 *
 ******************************************************************************/

enum spirit_clockoutput_xoprescaler_e
  spirit_gpio_get_xoprescaler(FAR struct spirit_library_s *spirit)
{
  uint8_t regval = 0x00;

  /* Reads the MCU_CLK_CONFIG register */

  spirit_reg_read(spirit, MCU_CK_CONF_BASE, &regval, 1);

  /* Mask the XO_RATIO field and return the value */

  return ((enum spirit_clockoutput_xoprescaler_e)(regval & 0x1e));
}

/******************************************************************************
 * Name: spirit_gpio_set_rcoprescaler
 *
 * Description:
 *   Sets the RCO ratio as clock output
 *
 * Input Parameters:
 *   spirit       - Reference to a Spirit library state structure instance
 *   rcoprescaler - The RCO prescaler to be used as clock output. This
 *                  parameter can be any value from enum
 *                  spirit_clockoutput_rcoprescaler_e .
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_gpio_set_rcoprescaler(FAR struct spirit_library_s *spirit,
                         enum spirit_clockoutput_rcoprescaler_e rcoprescaler)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_CLOCK_OUTPUT_RCO(rcoprescaler));

  /* Reads the MCU_CLK_CONFIG register */

  ret = spirit_reg_read(spirit, MCU_CK_CONF_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Mask the RCO_RATIO field and writes the new value */

      regval &= 0xfe;
      regval |= (uint8_t)rcoprescaler;

      /* Write to the new RCO prescaler in the MCU_CLOCK register */

      ret = spirit_reg_write(spirit, MCU_CK_CONF_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_gpio_get_rcoprescaler
 *
 * Description:
 *   Returns the settled RCO prescaler as clock output.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Settled RCO prescaler used for clock output.
 *
 ******************************************************************************/

enum spirit_clockoutput_rcoprescaler_e
  spirit_gpio_get_rcoprescaler(FAR struct spirit_library_s *spirit)
{
  uint8_t regval = 0;

  /* Reads the MCU_CLK_CONFIG register */

  spirit_reg_read(spirit, MCU_CK_CONF_BASE, &regval, 1);

  /* Mask the RCO_RATIO field and returns the value */

  return ((enum spirit_clockoutput_rcoprescaler_e)(regval & 0x01));
}

/******************************************************************************
 * Name: spirit_gpio_set_extracycles
 *
 * Description:
 *   Sets the RCO ratio as clock output.
 *
 * Input Parameters:
 *   spirit     - Reference to a Spirit library state structure instance
 *   xtracycles - The number of extra clock cycles provided before switching
 *                to STANDBY state. This parameter can be any value of enum
 *                spirit_extra_clockcycles_e.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int spirit_gpio_set_extracycles(FAR struct spirit_library_s *spirit,
                                enum spirit_extra_clockcycles_e xtracycles)
{
  uint8_t regval = 0;
  int ret;

  /* Check the parameters */

  DEBUGASSERT(IS_SPIRIT_CLOCK_OUTPUT_EXTRA_CYCLES(xtracycles));

  /* Reads the MCU_CLK_CONFIG register */

  ret = spirit_reg_read(spirit, MCU_CK_CONF_BASE, &regval, 1);
  if (ret >= 0)
    {
      /* Mask the CLOCK_TAIL field and writes the new value */

      regval &= 0x9f;
      regval |= (uint8_t)xtracycles;

      /* Write to the new number of extra clock cycles in the MCU_CLOCK
       * register.
       */

      ret = spirit_reg_write(spirit, MCU_CK_CONF_BASE, &regval, 1);
    }

  return ret;
}

/******************************************************************************
 * Name: spirit_gpio_get_extracycles
 *
 * Description:
 *   Returns the settled RCO prescaler as clock output.
 *
 * Input Parameters:
 *   spirit - Reference to a Spirit library state structure instance
 *
 * Returned Value:
 *   Settled number of extra clock cycles provided before switching to STANDBY
 *   state.
 *
 ******************************************************************************/

enum spirit_extra_clockcycles_e
  spirit_gpio_get_extracycles(FAR struct spirit_library_s *spirit)
{
  uint8_t regval = 0;

  /* Reads the MCU_CLK_CONFIG register */

  spirit_reg_read(spirit, MCU_CK_CONF_BASE, &regval, 1);

  /* Mask the CLOCK_TAIL field and returns the value */

  return ((enum spirit_extra_clockcycles_e)(regval & 0x60));
}
