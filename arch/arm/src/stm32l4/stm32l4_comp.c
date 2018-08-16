/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_comp.c
 *
 *   Copyright (c) 2017 Gregory Nutt. All rights reserved.
 *
 * Based on COMP driver from the Motorola MDK:
 *
 *   Copyright (c) 2016 Motorola Mobility, LLC. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <debug.h>
#include <string.h>

#include <nuttx/analog/comp.h>

#include "chip.h"
#include "stm32l4_comp.h"
#include "stm32l4_exti.h"
#include "stm32l4_gpio.h"
#include "up_arch.h"

#include <errno.h>

#if !(defined(CONFIG_STM32L4_STM32L4X3) || defined(CONFIG_STM32L4_STM32L4X5) || \
      defined(CONFIG_STM32L4_STM32L4X6) || defined(CONFIG_STM32L4_STM32L4XR))
#  error "Unrecognized STM32 chip"
#endif

#ifdef CONFIG_STM32L4_COMP

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* COMP Register access */

static inline void modify_csr(FAR const struct stm32l4_comp_config_s *cfg,
                              uint32_t clearbits, uint32_t setbits);
static inline uint32_t get_csr(const struct stm32l4_comp_config_s *cfg);
static void stm32l4_compenable(FAR struct stm32l4_comp_config_s *cfg,
                               bool en);
static int stm32l4_compconfig(FAR const struct comp_dev_s *dev);

/* COMP Driver Methods */

static void comp_shutdown(FAR struct comp_dev_s *dev);
static int comp_setup(FAR struct comp_dev_s *dev);
static int comp_read(FAR struct comp_dev_s *dev);
static int comp_ioctl(FAR struct comp_dev_s *dev, int cmd,
                      unsigned long arg);
static int comp_bind(FAR struct comp_dev_s *dev,
                     FAR const struct comp_callback_s *callback);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct comp_ops_s g_compops =
{
  .ao_shutdown  = comp_shutdown,
  .ao_setup     = comp_setup,
  .ao_read      = comp_read,
  .ao_ioctl     = comp_ioctl,
  .ao_bind      = comp_bind,
};

static struct stm32l4_comp_config_s g_comp1priv =
{
  .interrupt    =
  {
    .cb         = NULL, /* will be bound to upper-half driver */
    .rising     = true,
    .falling    = false
  },
  .inp          = STM32L4_COMP_INP_PIN_2,
  .inm          = STM32L4_COMP_INM_VREF,
  .hyst         = STM32L4_COMP_HYST_LOW,
  .speed        = STM32L4_COMP_SPEED_MEDIUM,
  .inverted     = false,
  .csr          = STM32L4_COMP1_CSR,
};

static struct comp_dev_s g_comp1dev =
{
  .ad_ops       = &g_compops,
  .ad_priv      = &g_comp1priv,
};

static struct stm32l4_comp_config_s g_comp2priv =
{
  .interrupt    =
  {
    .cb         = NULL, /* will be bound to upper-half driver */
    .rising     = true,
    .falling    = false
  },
  .inp          = STM32L4_COMP_INP_PIN_1,
  .inm          = STM32L4_COMP_INM_DAC_1,
  .hyst         = STM32L4_COMP_HYST_LOW,
  .speed        = STM32L4_COMP_SPEED_MEDIUM,
  .inverted     = false,
  .csr          = STM32L4_COMP2_CSR,
};

static struct comp_dev_s g_comp2dev =
{
  .ad_ops       = &g_compops,
  .ad_priv      = &g_comp2priv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modify_csr
 ****************************************************************************/

static inline void modify_csr(FAR const struct stm32l4_comp_config_s *cfg,
                              uint32_t clearbits, uint32_t setbits)
{
  modifyreg32(cfg->csr, clearbits, setbits);
}

/****************************************************************************
 * Name: get_csr
 ****************************************************************************/

static inline uint32_t get_csr(const struct stm32l4_comp_config_s *cfg)
{
  return getreg32(cfg->csr);
}

/****************************************************************************
 * Name: comp_setup
 *
 * Description:
 *   Configure the COMP. This method is called the first time that the COMP
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching COMP interrupts.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *
 * Returned Value:
 *
 ****************************************************************************/

static int comp_setup(FAR struct comp_dev_s *dev)
{
  int ret;

  /* Configure selected comparator */

  ret = stm32l4_compconfig(dev);
  if (ret < 0)
    {
      aerr("ERROR: Failed to initialize COMP: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: comp_shutdown
 *
 * Description:
 *   Disable the COMP.  This method is called when the COMP device is closed.
 *   This method reverses the operation the setup method.
 *   Works only if COMP device is not locked.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void comp_shutdown(FAR struct comp_dev_s *dev)
{
  FAR struct stm32l4_comp_config_s *cfg;

  cfg = dev->ad_priv;
  stm32l4_compenable(cfg, false);
}

/****************************************************************************
 * Name: comp_read
 *
 * Description:
 *  Get the COMP output state.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *
 * Returned Value:
 *   0 if output is low (non-inverting input below inverting input),
 *   1 if output is high (non inverting input above inverting input).
 *
 ****************************************************************************/

static int comp_read(FAR struct comp_dev_s *dev)
{
  FAR struct stm32l4_comp_config_s *cfg;
  uint32_t regval;

  cfg = dev->ad_priv;
  regval = get_csr(cfg);

  return (((regval & COMP_CSR_VALUE) == 0) ? 0 : 1);
}

/****************************************************************************
 * Name: comp_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *   cmd - command
 *   arg - arguments passed with command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int comp_ioctl(FAR struct comp_dev_s *dev, int cmd, unsigned long arg)
{
#warning "Missing logic"
  return -ENOTTY;
}

/****************************************************************************
 * Name: comp_bind
 *
 * Description:
 *   Bind upper half callback.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int comp_bind(FAR struct comp_dev_s *dev,
                     FAR const struct comp_callback_s *callback)
{
  FAR struct stm32l4_comp_config_s *priv =
    (FAR struct stm32l4_comp_config_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->interrupt.cb = callback;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_compenable
 *
 * Description:
 *   Enable/disable comparator
 *
 * Input Parameters:
 *  cmp - comparator
 *  cfg - enable/disable flag
 *
 ****************************************************************************/

static void stm32l4_compenable(FAR struct stm32l4_comp_config_s *cfg, bool en)
{
  uint32_t clearbits = en ? 0 : COMP_CSR_EN;
  uint32_t setbits = en ? COMP_CSR_EN : 0;

  modify_csr(cfg, clearbits, setbits);
}

static int stm32l4_exti_comp_isr(int irq, void *context, FAR void *arg)
{
  FAR struct comp_dev_s *dev = (FAR struct comp_dev_s *)arg;
  struct stm32l4_comp_config_s *cfg = dev->ad_priv;

  DEBUGASSERT(cfg->interrupt.cb &&
              (cfg->interrupt.rising || cfg->interrupt.falling));

  ainfo("isr: %d\n", (cfg->csr == STM32L4_COMP1_CSR ? 0 : 1));

  cfg->interrupt.cb->au_notify(dev, comp_read(dev));

  return 0;
}

/****************************************************************************
 * Name: stm32l4_compconfig
 *
 * Description:
 *   Configure comparator and I/Os used as comparators inputs
 *
 * Input Parameters:
 *  cfg - configuration
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int stm32l4_compconfig(FAR const struct comp_dev_s *dev)
{
  FAR struct stm32l4_comp_config_s *cfg;
  uint32_t regval = 0;
  uint32_t mask = 0;
  uint32_t clearbits;
  uint32_t setbits;
  int ret;
  int cmp;

  cfg = dev->ad_priv;
  cmp = cfg->csr == STM32L4_COMP1_CSR ? STM32L4_COMP1 : STM32L4_COMP2;

  /* Input plus */

  mask |= COMP_CSR_INPSEL_MASK;
  switch (cfg->inp)
    {
    case STM32L4_COMP_INP_PIN_1:
      stm32l4_configgpio(cmp == STM32L4_COMP1 ? GPIO_COMP1_INP_1 : GPIO_COMP2_INP_1);
      regval |= COMP_CSR_INPSEL_PIN1;
      break;

    case STM32L4_COMP_INP_PIN_2:
      stm32l4_configgpio(cmp == STM32L4_COMP1 ? GPIO_COMP1_INP_2 : GPIO_COMP2_INP_2);
      regval |= COMP_CSR_INPSEL_PIN2;
      break;

#if defined(CONFIG_STM32L4_STM32L4X3)
    case STM32L4_COMP_INP_PIN_3:
      stm32l4_configgpio(cmp == STM32L4_COMP1 ? GPIO_COMP1_INP_3 : GPIO_COMP2_INP_3);
      regval |= COMP_CSR_INPSEL_PIN3;
      break;
#endif

    default:
      return -EINVAL;
    }

  /* Input minus */

  mask |= COMP_CSR_INMSEL_MASK;
  switch (cfg->inm)
    {
    case STM32L4_COMP_INM_1_4_VREF:
      regval |= COMP_CSR_INMSEL_25PCT;
      mask   |= (COMP_CSR_SCALEN | COMP_CSR_BRGEN);
      regval |= (COMP_CSR_SCALEN | COMP_CSR_BRGEN);
      break;

    case STM32L4_COMP_INM_1_2_VREF:
      regval |= COMP_CSR_INMSEL_50PCT;
      mask   |= (COMP_CSR_SCALEN | COMP_CSR_BRGEN);
      regval |= (COMP_CSR_SCALEN | COMP_CSR_BRGEN);
      break;

    case STM32L4_COMP_INM_3_4_VREF:
      regval |= COMP_CSR_INMSEL_75PCT;
      mask   |= (COMP_CSR_SCALEN | COMP_CSR_BRGEN);
      regval |= (COMP_CSR_SCALEN | COMP_CSR_BRGEN);
      break;

    case STM32L4_COMP_INM_VREF:
      regval |= COMP_CSR_INMSEL_VREF;
      mask   |= (COMP_CSR_SCALEN | COMP_CSR_BRGEN);
      regval |= COMP_CSR_SCALEN;
      break;

    case STM32L4_COMP_INM_DAC_1:
      regval |= COMP_CSR_INMSEL_DAC1;
      break;

    case STM32L4_COMP_INM_DAC_2:
      regval |= COMP_CSR_INMSEL_DAC2;
      break;

  case STM32L4_COMP_INM_PIN_1:
      stm32l4_configgpio(cmp == STM32L4_COMP1 ? GPIO_COMP1_INM_1 : GPIO_COMP2_INM_1);
      regval |= COMP_CSR_INMSEL_PIN1;
      break;

  case STM32L4_COMP_INM_PIN_2:
      stm32l4_configgpio(cmp == STM32L4_COMP1 ? GPIO_COMP1_INM_2 : GPIO_COMP2_INM_2);
#if defined(CONFIG_STM32L4_STM32L4X5) || defined(CONFIG_STM32L4_STM32L4X6) || \
    defined(CONFIG_STM32L4_STM32L4XR)
      regval |= COMP_CSR_INMSEL_PIN2;
#else
      regval |= COMP_CSR_INMSEL_INMESEL;
      mask   |= COMP_CSR_INMESEL_MASK;
      regval |= COMP_CSR_INMESEL_PIN2;
#endif
      break;

#if defined(CONFIG_STM32L4_STM32L4X3)
    case STM32L4_COMP_INM_PIN_3:
      stm32l4_configgpio(cmp == STM32L4_COMP1 ? GPIO_COMP1_INM_3 : GPIO_COMP2_INM_3);
      regval |= COMP_CSR_INMSEL_INMESEL;
      mask   |= COMP_CSR_INMESEL_MASK;
      regval |= COMP_CSR_INMESEL_PIN3;
      break;

    case STM32L4_COMP_INM_PIN_4:
      stm32l4_configgpio(cmp == STM32L4_COMP1 ? GPIO_COMP1_INM_4 : GPIO_COMP2_INM_4);
      regval |= COMP_CSR_INMSEL_INMESEL;
      mask   |= COMP_CSR_INMESEL_MASK;
      regval |= COMP_CSR_INMESEL_PIN4;
      break;

    case STM32L4_COMP_INM_PIN_5:
      stm32l4_configgpio(cmp == STM32L4_COMP1 ? GPIO_COMP1_INM_5 : GPIO_COMP2_INM_5);
      regval |= COMP_CSR_INMSEL_INMESEL;
      mask   |= COMP_CSR_INMESEL_MASK;
      regval |= COMP_CSR_INMESEL_PIN5;
      break;

#endif
    default:
      return -EINVAL;
    }

  /* Hysteresis */

  mask |= COMP_CSR_HYST_MASK;
  switch (cfg->hyst)
    {
    case STM32L4_COMP_HYST_NONE:
      regval |= COMP_CSR_HYST_NONE;
      break;

    case STM32L4_COMP_HYST_LOW:
      regval |= COMP_CSR_HYST_LOW;
      break;

    case STM32L4_COMP_HYST_MEDIUM:
      regval |= COMP_CSR_HYST_MEDIUM;
      break;

    case STM32L4_COMP_HYST_HIGH:
      regval |= COMP_CSR_HYST_HIGH;
      break;

    default:
      return -EINVAL;
    }

  /* Power/speed Mode */

  mask |= COMP_CSR_PWRMODE_MASK;
  switch(cfg->speed)
    {
    case STM32L4_COMP_SPEED_HIGH:
      regval |= COMP_CSR_PWRMODE_HIGH;
      break;

    case STM32L4_COMP_SPEED_MEDIUM:
      regval |= COMP_CSR_PWRMODE_MEDIUM;
      break;

    case STM32L4_COMP_SPEED_LOW:
      regval |= COMP_CSR_PWRMODE_LOW;
      break;

    default:
      return -EINVAL;
    }

  /* Polarity */

  mask |= COMP_CSR_POLARITY_MASK;
  if (cfg->inverted)
    {
        regval |= COMP_CSR_POLARITY_INVERT;
    }

  /* Disable blanking */

  mask   |= COMP_CSR_BLANK_MASK;
  regval |= COMP_CSR_BLANK_NONE;

  clearbits = regval ^ mask;
  setbits = regval;

  modify_csr(cfg, clearbits, setbits);

  /* Enable */

  stm32l4_compenable(cfg, true);

  /* Enable interrupt */

  if (cfg->interrupt.cb && (cfg->interrupt.rising || cfg->interrupt.falling))
    {
      ret = stm32l4_exti_comp(cmp, cfg->interrupt.rising, cfg->interrupt.falling,
                              0, stm32l4_exti_comp_isr, (void *)dev);
      if (ret < 0)
        {
          aerr("stm32l4_exti_comp failed ret = %d\n", ret);
          return ERROR;
        }
    }

  ainfo("comp%d configured\n", cmp);

  return 0;
}

/****************************************************************************
 * Name: stm32l4_compinitialize
 *
 * Description:
 *   Initialize the COMP.
 *
 * Input Parameters:
 *   intf - The COMP interface number.
 *
 * Returned Value:
 *   Valid COMP device structure reference on success; a NULL on failure.
 *
 * Assumptions:
 *   1. Clock to the COMP block has enabled,
 *   2. Board-specific logic has already configured
 *
 ****************************************************************************/

FAR struct comp_dev_s* stm32l4_compinitialize(int intf,
                                              FAR const struct stm32l4_comp_config_s *cfg)
{
  FAR struct comp_dev_s *dev;

  switch (intf)
    {
    case 1:
      ainfo("COMP1 selected\n");
      dev = &g_comp1dev;
      break;

    case 2:
      ainfo("COMP2 selected\n");
      dev = &g_comp2dev;
      break;

    default:
      aerr("ERROR: No COMP interface defined\n");
      return NULL;
    }

  if (cfg)
    {
      memcpy(dev->ad_priv, cfg, sizeof(*cfg));
    }

  return dev;
}

#endif /* CONFIG_STM32L4_COMP */
