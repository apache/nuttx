/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_gpio_cfg.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>

#include "tlsr82_analog.h"
#include "tlsr82_gpio.h"
#include "tlsr82_gpio_cfg.h"
#include "tlsr82_gpio_default.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MUX_NONE             TLSR82_MUX_NONE
#define MUX_UART             TLSR82_MUX_UART
#define MUX_PWM              TLSR82_MUX_PWM
#define MUX_DMIC             TLSR82_MUX_DMIC
#define MUX_I2C              TLSR82_MUX_I2C
#define MUX_SPI              TLSR82_MUX_SPI
#define MUX_U7816            TLSR82_MUX_U7816
#define MUX_SDM              TLSR82_MUX_SDM
#define MUX_I2S              TLSR82_MUX_I2S
#define MUX_ATSEL            TLSR82_MUX_ATSEL
#define MUX_SWS              TLSR82_MUX_SWS
#define MUX_AMP              TLSR82_MUX_AMP
#define MUX_USB              TLSR82_MUX_USB
#define MUX_ADC              TLSR82_MUX_ADC
#define MUX_LPC              TLSR82_MUX_LPC
#define MUX_INPUT            TLSR82_MUX_INPUT
#define MUX_OUTPUT           TLSR82_MUX_OUTPUT

#define I2C_SPI              (MUX_I2C | MUX_SPI)
#define UART7816             (MUX_UART | MUX_U7816)
#define ADC_LPC              (MUX_ADC | MUX_LPC)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_TLSR82_GPIO_VALIDATION
const static struct tlsr82_gpio_mux_s gpio_mux_tbl[GPIO_NUM] =
{
  {MUX_DMIC, MUX_PWM  , MUX_UART , MUX_NONE}, /* -- PA0 */
  {MUX_DMIC, MUX_U7816, MUX_I2S  , MUX_NONE}, /*    PA1 */
  {MUX_SPI , MUX_UART , MUX_PWM  , MUX_NONE}, /*    PA2 */
  {I2C_SPI , MUX_UART , MUX_PWM  , MUX_NONE}, /*    PA3 */
  {I2C_SPI , MUX_UART , MUX_PWM  , MUX_NONE}, /*    PA4 */
  {MUX_SWS , MUX_NONE , MUX_NONE , MUX_NONE}, /*    PA5 */
  {MUX_SWS , MUX_NONE , MUX_NONE , MUX_NONE}, /*    PA6 */
  {MUX_SWS , MUX_UART , MUX_NONE , MUX_NONE}, /*    PA7 */
  {MUX_PWM , MUX_UART , MUX_ATSEL, ADC_LPC }, /* -- PB0 */
  {MUX_PWM , MUX_UART , MUX_ATSEL, ADC_LPC }, /*    PB1 */
  {MUX_PWM , MUX_UART , MUX_AMP  , ADC_LPC }, /*    PB2 */
  {MUX_PWM , MUX_UART , MUX_AMP  , ADC_LPC }, /*    PB3 */
  {MUX_SDM , MUX_PWM  , MUX_NONE , ADC_LPC }, /*    PB4 */
  {MUX_SDM , MUX_UART , MUX_NONE , ADC_LPC }, /*    PB5 */
  {MUX_SDM , I2C_SPI  , MUX_UART , ADC_LPC }, /*    PB6 */
  {MUX_SDM , MUX_SPI  , MUX_UART , ADC_LPC }, /*    PB7 */
  {MUX_I2C , MUX_PWM  , MUX_UART , MUX_NONE}, /* -- PC0 */
  {MUX_I2C , MUX_PWM  , MUX_PWM  , MUX_NONE}, /*    PC1 */
  {MUX_PWM , UART7816 , MUX_I2C  , MUX_NONE}, /*    PC2 */
  {MUX_PWM , MUX_UART , MUX_I2C  , MUX_NONE}, /*    PC3 */
  {MUX_PWM , MUX_UART , MUX_PWM  , MUX_ADC }, /*    PC4 */
  {MUX_PWM , MUX_UART , MUX_ATSEL, MUX_ADC }, /*    PC5 */
  {MUX_AMP , MUX_ATSEL, MUX_PWM  , MUX_NONE}, /*    PC6 */
  {MUX_AMP , MUX_ATSEL, MUX_PWM  , MUX_NONE}, /*    PC7 */
  {MUX_AMP , MUX_NONE , UART7816 , MUX_NONE}, /* -- PD0 */
  {MUX_AMP , MUX_NONE , MUX_UART , MUX_NONE}, /*    PD1 */
  {MUX_SPI , MUX_I2S  , MUX_PWM  , MUX_NONE}, /*    PD2 */
  {MUX_PWM , MUX_I2S  , UART7816 , MUX_NONE}, /*    PD3 */
  {MUX_SWS , MUX_I2S  , MUX_PWM  , MUX_NONE}, /*    PD4 */
  {MUX_PWM , MUX_NONE , MUX_PWM  , MUX_NONE}, /*    PD5 */
  {MUX_SPI , MUX_UART , MUX_ATSEL, MUX_NONE}, /*    PD6 */
  {I2C_SPI , MUX_I2S  , UART7816 , MUX_NONE}, /*    PD7 */
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tlsr82_gpio_cfg_check
 *
 * Description:
 *   Check weather the mux function is supported or not by the input pin.
 *
 * Input Parameters:
 *   cfg - the pin config information, include the group and pin num
 *         information.
 *   mux - mux function.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

int tlsr82_gpio_cfg_check(uint32_t cfg, uint32_t mux)
{
#ifdef CONFIG_TLSR82_GPIO_VALIDATION
  int muxid;
  uint32_t mux3;
  uint32_t cfg_af;
  uint16_t pinnum;

  DEBUGASSERT(GPIO_VALID(cfg));

  pinnum = GPIO_PIN2NUM(cfg);
  cfg_af = GPIO_GET(AF, cfg);

  /* ADC and LPC is not configed by gpio mux register */

  if ((mux == MUX_ADC) || (mux == MUX_LPC))
    {
      mux3 = gpio_mux_tbl[pinnum].muxs[3];
      return (mux3 & mux) ? 0 : -1;
    }

  /* If af is not mux0 ~ mux3, return -2 */

  if (cfg_af < GPIO_VAL(AF, MUX0))
    {
      return -2;
    }

  /* Get the muxid [0, 3], if mux is not supported by
   * this gpio pin, return -1, otherwise return 0.
   */

  muxid = cfg_af - GPIO_VAL(AF, MUX0);
  if ((gpio_mux_tbl[pinnum].muxs[muxid] & mux) == 0)
    {
      return -1;
    }
#endif

  return 0;
}

/****************************************************************************
 * Name: tlsr82_gpio_init
 *
 * Description:
 *   GPIO config initialization.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void tlsr82_gpio_init(void)
{
  /* Group A */

  GPIO_SET_PA_IE_REG  = GPIO_DEFAULT_IE_GET(A);
  GPIO_SET_PA_OEN_REG = GPIO_DEFAULT_OEN_GET(A);
  GPIO_SET_PA_OUT_REG = GPIO_DEFAULT_OUT_GET(A);
  GPIO_SET_PA_DS_REG  = GPIO_DEFAULT_DS_GET(A);
  GPIO_SET_PA_ACT_REG = GPIO_DEFAULT_ACT_GET(A);

  /* Group B */

  GPIO_SET_PB_IE_REG  = GPIO_DEFAULT_IE_GET(B);
  GPIO_SET_PB_OEN_REG = GPIO_DEFAULT_OEN_GET(B);
  GPIO_SET_PB_OUT_REG = GPIO_DEFAULT_OUT_GET(B);
  GPIO_SET_PB_DS_REG  = GPIO_DEFAULT_DS_GET(B);
  GPIO_SET_PB_ACT_REG = GPIO_DEFAULT_ACT_GET(B);

  /* Group C */

  tlsr82_analog_write(ANALOG_PC_IE_ADDR, GPIO_DEFAULT_IE_GET(C));
  GPIO_SET_PC_OEN_REG = GPIO_DEFAULT_OEN_GET(C);
  GPIO_SET_PC_OUT_REG = GPIO_DEFAULT_OUT_GET(C);
  tlsr82_analog_write(ANALOG_PC_DS_ADDR, GPIO_DEFAULT_DS_GET(C));
  GPIO_SET_PC_ACT_REG = GPIO_DEFAULT_ACT_GET(C);

  /* Group D */

  GPIO_SET_PD_IE_REG  = GPIO_DEFAULT_IE_GET(D);
  GPIO_SET_PD_OEN_REG = GPIO_DEFAULT_OEN_GET(D);
  GPIO_SET_PD_OUT_REG = GPIO_DEFAULT_OUT_GET(D);
  GPIO_SET_PD_DS_REG  = GPIO_DEFAULT_DS_GET(D);
  GPIO_SET_PD_ACT_REG = GPIO_DEFAULT_ACT_GET(D);

  /* Group E */

  GPIO_SET_PE_IE_REG  = GPIO_DEFAULT_IE_GET_E;
  GPIO_SET_PE_OEN_REG = GPIO_DEFAULT_OEN_GET_E;
  GPIO_SET_PE_OUT_REG = GPIO_DEFAULT_OUT_GET_E;
  GPIO_SET_PE_DS_REG  = GPIO_DEFAULT_DS_GET_E;
  GPIO_SET_PE_ACT_REG = GPIO_DEFAULT_ACT_GET_E;
}
