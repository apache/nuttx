/****************************************************************************
 * arch/arm/src/stm32/stm32_comp_v2.c
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some COMP peripheral must be enabled and the device must be supported */

#define DEVICE_NOT_SUPPORTED

#if defined(CONFIG_STM32_COMP)

#ifndef CONFIG_STM32_SYSCFG
#  error "SYSCFG clock enable must be set"
#endif

#if defined(CONFIG_STM32_STM32G43XX)
#  undef DEVICE_NOT_SUPPORTED
#  if defined(CONFIG_STM32_COMP5) || defined(CONFIG_STM32_COMP6) || \
      defined(CONFIG_STM32_COMP7)
#    error "STM32G43XX supports only COMP1, COMP2, COMP3 and COMP4"
#  endif
#endif

#if defined(DEVICE_NOT_SUPPORTED)
#  error "Device not supported"
#endif

#if defined(CONFIG_STM32_COMP1_OUT) || defined(CONFIG_STM32_COMP2_OUT) || \
    defined(CONFIG_STM32_COMP3_OUT) || defined(CONFIG_STM32_COMP4_OUT) || \
    defined(CONFIG_STM32_COMP5_OUT) || defined(CONFIG_STM32_COMP6_OUT) || \
    defined(CONFIG_STM32_COMP7_OUT)
#  define COMP_OUT_GPIO
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the configuration of one COMP device */

struct stm32_comp_s
{
  uint8_t inm;                  /* Inverting input selection */
  uint32_t gpio_inm;            /* Inverting input pin */
  uint8_t inp;                  /* Non inverting input selection */
  uint32_t gpio_inp;            /* Non-inverting input pin */
  uint8_t pol;                  /* Output polarity */
  uint8_t hyst;                 /* Comparator hysteresis */
  uint8_t blanking;             /* Blanking source */
  uint8_t lock;                 /* Comparator Lock */
  uint32_t csr;                 /* Control and status register */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* COMP Register access */

static inline void     comp_modify_csr(struct stm32_comp_s *priv,
                                       uint32_t clearbits, uint32_t setbits);
static inline uint32_t comp_getreg_csr(struct stm32_comp_s *priv);
static inline void     comp_putreg_csr(struct stm32_comp_s *priv,
                                       uint32_t value);

/* COMP Driver Methods */

#if defined (CONFIG_COMP)
static void comp_shutdown(struct comp_dev_s *dev);
static int  comp_setup(struct comp_dev_s *dev);
static int  comp_read(struct comp_dev_s *dev);
static int  comp_ioctl(struct comp_dev_s *dev, int cmd,
                       unsigned long arg);
#endif

static int  comp_config(struct stm32_comp_s *priv);
static int  comp_enable(struct stm32_comp_s *priv, bool enable);
static bool comp_lock_get(struct stm32_comp_s *priv);
static int  comp_lock_set(struct stm32_comp_s *priv, bool lock);

static int  comp_config_inmpin(struct stm32_comp_s *priv);
static int  comp_config_inppin(struct stm32_comp_s *priv);
#if defined(COMP_OUT_GPIO)
static int  comp_config_outpin(struct stm32_comp_s *priv);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_COMP
static const struct comp_ops_s g_compops =
{
  .ao_shutdown  = comp_shutdown,
  .ao_setup     = comp_setup,
  .ao_read      = comp_read,
  .ao_ioctl     = comp_ioctl,
};
#endif

#ifdef CONFIG_STM32_COMP1
static struct stm32_comp_s g_comp1priv =
{
  .inm       = CONFIG_STM32_COMP1_INM,
  .inp       = CONFIG_STM32_COMP1_INP,
  .pol       = CONFIG_STM32_COMP1_POL,
  .hyst      = CONFIG_STM32_COMP1_HYST,
  .blanking  = CONFIG_STM32_COMP1_BLANKSEL,
  .lock      = CONFIG_STM32_COMP1_LOCK,
  .gpio_inp  = GPIO_COMP1_INP,
  .csr       = STM32_COMP1_CSR
};

static struct comp_dev_s g_comp1dev =
{
#ifdef CONFIG_COMP
  .ad_ops  = &g_compops,
#endif
  .ad_priv = &g_comp1priv,
};
#endif

#ifdef CONFIG_STM32_COMP2
static struct stm32_comp_s g_comp2priv =
{
  .inm       = CONFIG_STM32_COMP2_INM,
  .inp       = CONFIG_STM32_COMP2_INP,
  .pol       = CONFIG_STM32_COMP2_POL,
  .hyst      = CONFIG_STM32_COMP2_HYST,
  .blanking  = CONFIG_STM32_COMP2_BLANKSEL,
  .lock      = CONFIG_STM32_COMP2_LOCK,
  .gpio_inp  = GPIO_COMP2_INP,
  .csr       = STM32_COMP2_CSR
};

static struct comp_dev_s g_comp2dev =
{
#ifdef CONFIG_COMP
  .ad_ops  = &g_compops,
#endif
  .ad_priv = &g_comp2priv,
};
#endif

#ifdef CONFIG_STM32_COMP3
static struct stm32_comp_s g_comp3priv =
{
  .inm       = CONFIG_STM32_COMP3_INM,
  .inp       = CONFIG_STM32_COMP3_INP,
  .pol       = CONFIG_STM32_COMP3_POL,
  .hyst      = CONFIG_STM32_COMP3_HYST,
  .blanking  = CONFIG_STM32_COMP3_BLANKSEL,
  .lock      = CONFIG_STM32_COMP3_LOCK,
  .gpio_inp  = GPIO_COMP3_INP,
  .csr       = STM32_COMP3_CSR
};

static struct comp_dev_s g_comp3dev =
{
#ifdef CONFIG_COMP
  .ad_ops  = &g_compops,
#endif
  .ad_priv = &g_comp3priv,
};
#endif

#ifdef CONFIG_STM32_COMP4
static struct stm32_comp_s g_comp4priv =
{
  .inm       = CONFIG_STM32_COMP4_INM,
  .inp       = CONFIG_STM32_COMP4_INP,
  .pol       = CONFIG_STM32_COMP4_POL,
  .hyst      = CONFIG_STM32_COMP4_HYST,
  .blanking  = CONFIG_STM32_COMP4_BLANKSEL,
  .lock      = CONFIG_STM32_COMP4_LOCK,
  .gpio_inp  = GPIO_COMP4_INP,
  .csr       = STM32_COMP4_CSR
};

static struct comp_dev_s g_comp4dev =
{
#ifdef CONFIG_COMP
  .ad_ops  = &g_compops,
#endif
  .ad_priv = &g_comp4priv,
};
#endif

#ifdef CONFIG_STM32_COMP5
static struct stm32_comp_s g_comp5priv =
{
  .inm       = CONFIG_STM32_COMP5_INM,
  .inp       = CONFIG_STM32_COMP5_INP,
  .pol       = CONFIG_STM32_COMP5_POL,
  .hyst      = CONFIG_STM32_COMP5_HYST,
  .blanking  = CONFIG_STM32_COMP5_BLANKSEL,
  .lock      = CONFIG_STM32_COMP5_LOCK,
  .gpio_inp  = GPIO_COMP5_INP,
  .csr       = STM32_COMP5_CSR
};

static struct comp_dev_s g_comp5dev =
{
#ifdef CONFIG_COMP
  .ad_ops  = &g_compops,
#endif
  .ad_priv = &g_comp5priv,
};
#endif

#ifdef CONFIG_STM32_COMP6
static struct stm32_comp_s g_comp6priv =
{
  .inm       = CONFIG_STM32_COMP6_INM,
  .inp       = CONFIG_STM32_COMP6_INP,
  .pol       = CONFIG_STM32_COMP6_POL,
  .hyst      = CONFIG_STM32_COMP6_HYST,
  .blanking  = CONFIG_STM32_COMP6_BLANKSEL,
  .lock      = CONFIG_STM32_COMP6_LOCK,
  .gpio_inp  = GPIO_COMP6_INP,
  .csr       = STM32_COMP6_CSR
};

static struct comp_dev_s g_comp6dev =
{
#ifdef CONFIG_COMP
  .ad_ops  = &g_compops,
#endif
  .ad_priv = &g_comp6priv,
};
#endif

#ifdef CONFIG_STM32_COMP7
static struct stm32_comp_s g_comp7priv =
{
  .inm       = CONFIG_STM32_COMP7_INM,
  .inp       = CONFIG_STM32_COMP7_INP,
  .pol       = CONFIG_STM32_COMP7_POL,
  .hyst      = CONFIG_STM32_COMP7_HYST,
  .blanking  = CONFIG_STM32_COMP7_BLANKSEL,
  .lock      = CONFIG_STM32_COMP7_LOCK,
  .gpio_inp  = GPIO_COMP7_INP,
  .csr       = STM32_COMP7_CSR
};

static struct comp_dev_s g_comp7dev =
{
#ifdef CONFIG_COMP
  .ad_ops  = &g_compops,
#endif
  .ad_priv = &g_comp7priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: comp_modify_csr
 *
 * Description:
 *   Modify the value of a 32-bit COMP CSR register (not atomic).
 *
 * Input Parameters:
 *   priv   - A reference to the COMP structure
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void comp_modify_csr(struct stm32_comp_s *priv,
                                   uint32_t clearbits, uint32_t setbits)
{
  uint32_t csr = priv->csr;

  modifyreg32(csr, clearbits, setbits);
}

/****************************************************************************
 * Name: comp_getreg_csr
 *
 * Description:
 *   Read the value of an COMP CSR register
 *
 * Input Parameters:
 *   priv   - A reference to the COMP structure
 *
 * Returned Value:
 *   The current contents of the COMP CSR register
 *
 ****************************************************************************/

static inline uint32_t comp_getreg_csr(struct stm32_comp_s *priv)
{
  uint32_t csr = priv->csr;

  return getreg32(csr);
}

/****************************************************************************
 * Name: comp_putreg_csr
 *
 * Description:
 *   Write a value to an COMP register.
 *
 * Input Parameters:
 *   priv   - A reference to the COMP structure
 *   value  - The value to write to the COMP CSR register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void comp_putreg_csr(struct stm32_comp_s *priv,
                                   uint32_t value)
{
  uint32_t csr = priv->csr;

  putreg32(value, csr);
}

/****************************************************************************
 * Name: comp_lock_get
 *
 * Description:
 *   Get COMP lock bit state
 *
 * Input Parameters:
 *   priv   - A reference to the COMP structure
 *
 * Returned Value:
 *   True if COMP locked, false if not locked
 *
 ****************************************************************************/

static bool comp_lock_get(struct stm32_comp_s *priv)
{
  uint32_t regval;

  regval = comp_getreg_csr(priv);

  return (((regval & COMP_CSR_LOCK) == 0) ? false : true);
}

/****************************************************************************
 * Name: comp_lock_set
 *
 * Description:
 *   Lock comparator CSR register
 *
 * Input Parameters:
 *   priv   - A reference to the COMP structure
 *   enable - lock flag
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int comp_lock_set(struct stm32_comp_s *priv, bool lock)
{
  if (comp_lock_get(priv))
    {
      if (lock == false)
        {
          aerr("ERROR: COMP LOCK can be cleared only by a system reset\n");

          return -EPERM;
        }
    }
  else
    {
      if (lock == true)
        {
          comp_modify_csr(priv, 0, COMP_CSR_LOCK);

          priv->lock = 1;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: comp_config_inmpin
 *
 * Description:
 *   Configure comparator inverting input pin. The GPIO that COMPx inverting
 *   input will be assigned is dependent of comparator number and must be
 *   defined in board.h file. See table 196 in RM0440.
 *
 * Input Parameters:
 *   priv   - A reference to the COMP structure
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int comp_config_inmpin(struct stm32_comp_s *priv)
{
#  if defined(CONFIG_STM32_COMP1)
  if (priv->csr == STM32_COMP1_CSR)
    {
      stm32_configgpio(GPIO_COMP1_INM);
    }
#  endif

#  if defined(CONFIG_STM32_COMP2)
  if (priv->csr == STM32_COMP2_CSR)
    {
      stm32_configgpio(GPIO_COMP2_INM);
    }
#  endif

#  if defined(CONFIG_STM32_COMP3)
  if (priv->csr == STM32_COMP3_CSR)
    {
      stm32_configgpio(GPIO_COMP3_INM);
    }
#  endif

#  if defined(CONFIG_STM32_COMP4)
  if (priv->csr == STM32_COMP4_CSR)
    {
      stm32_configgpio(GPIO_COMP4_INM);
    }
#  endif

#  if defined(CONFIG_STM32_COMP5)
  if (priv->csr == STM32_COMP5_CSR)
    {
      stm32_configgpio(GPIO_COMP5_INM);
    }
#  endif

#  if defined(CONFIG_STM32_COMP6)
  if (priv->csr == STM32_COMP6_CSR)
    {
      stm32_configgpio(GPIO_COMP6_INM);
    }
#  endif

#  if defined(CONFIG_STM32_COMP7)
  if (priv->csr == STM32_COMP7_CSR)
    {
      stm32_configgpio(GPIO_COMP7_INM);
    }
#  endif

  return OK;
}

/****************************************************************************
 * Name: comp_config_inppin
 *
 * Description:
 *   Configure comparator non-inverting input pin. The IO pin that COMPx
 *   non-inverting input will be assigned is dependent of comparator number
 *   and must be defined in board.h file.
 *
 * Input Parameters:
 *   priv   - A reference to the COMP structure
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int comp_config_inppin(struct stm32_comp_s *priv)
{
#  if defined(CONFIG_STM32_COMP1)
  if (priv->csr == STM32_COMP1_CSR)
    {
      stm32_configgpio(GPIO_COMP1_INP);
    }
#  endif

#  if defined(CONFIG_STM32_COMP2)
  if (priv->csr == STM32_COMP2_CSR)
    {
      stm32_configgpio(GPIO_COMP2_INP);
    }
#  endif

#  if defined(CONFIG_STM32_COMP3)
  if (priv->csr == STM32_COMP3_CSR)
    {
      stm32_configgpio(GPIO_COMP3_INP);
    }
#  endif

#  if defined(CONFIG_STM32_COMP4)
  if (priv->csr == STM32_COMP4_CSR)
    {
      stm32_configgpio(GPIO_COMP4_INP);
    }
#  endif

#  if defined(CONFIG_STM32_COMP5)
  if (priv->csr == STM32_COMP5_CSR)
    {
      stm32_configgpio(GPIO_COMP5_INP);
    }
#  endif

#  if defined(CONFIG_STM32_COMP6)
  if (priv->csr == STM32_COMP6_CSR)
    {
      stm32_configgpio(GPIO_COMP6_INP);
    }
#  endif

#  if defined(CONFIG_STM32_COMP7)
  if (priv->csr == STM32_COMP7_CSR)
    {
      stm32_configgpio(GPIO_COMP7_INP);
    }
#  endif

  return OK;
}

/****************************************************************************
 * Name: comp_config_outpin
 *
 * Description:
 *   Configure comparator output GPIO pin.
 *
 * Input Parameters:
 *   priv   - A reference to the COMP structure
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

#if defined(COMP_OUT_GPIO)
static int comp_config_outpin(struct stm32_comp_s *priv)
{
#  if defined(CONFIG_STM32_COMP1_OUT)
  if (priv->csr == STM32_COMP1_CSR)
    {
      stm32_configgpio(GPIO_COMP1_OUT);
    }
#  endif

#  if defined(CONFIG_STM32_COMP2_OUT)
  if (priv->csr == STM32_COMP2_CSR)
    {
      ainfo("\tOUT assigned to: GPIO\n");
      stm32_configgpio(GPIO_COMP2_OUT);
    }
#  endif

#  if defined(CONFIG_STM32_COMP3_OUT)
  if (priv->csr == STM32_COMP3_CSR)
    {
      stm32_configgpio(GPIO_COMP3_OUT);
    }
#  endif

#  if defined(CONFIG_STM32_COMP4_OUT)
  if (priv->csr == STM32_COMP4_CSR)
    {
      stm32_configgpio(GPIO_COMP4_OUT);
    }
#  endif

#  if defined(CONFIG_STM32_COMP5_OUT)
  if (priv->csr == STM32_COMP5_CSR)
    {
      stm32_configgpio(GPIO_COMP5_OUT);
    }
#  endif

#  if defined(CONFIG_STM32_COMP6_OUT)
  if (priv->csr == STM32_COMP6_CSR)
    {
      stm32_configgpio(GPIO_COMP6_OUT);
    }
#  endif

#  if defined(CONFIG_STM32_COMP7_OUT)
  if (priv->csr == STM32_COMP7_CSR)
    {
      stm32_configgpio(GPIO_COMP7_OUT);
    }
#  endif

  return OK;
}
#endif /* COMP_OUT_GPIO */

/****************************************************************************
 * Name: comp_config
 *
 * Description:
 *   Configure comparator and used I/Os. The pin configuration and the input
 *   assignments are COMP index dependent.
 *
 * Input Parameters:
 *   priv   - A reference to the COMP structure
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 * REVISIT: Where to config comparator output pin ?
 *
 ****************************************************************************/

static int comp_config(struct stm32_comp_s *priv)
{
  uint32_t regval = 0;
  uint32_t value = 0;

  /* Configure COMPx inverting input. */

  value = priv->inm << COMP_CSR_INMSEL_SHIFT;

  switch (priv->inm)
    {
      case COMP_INM_1_4_VREF:
      case COMP_INM_1_2_VREF:
      case COMP_INM_3_4_VREF:

        value |= COMP_CSR_BRGEN;    /* scaler resistor bridge enable */

      case COMP_INM_VREF:

        value |= COMP_CSR_SCALEN;    /* VREFINT scaler enable */
        break;

      case COMP_INM_DAC_1:
      case COMP_INM_DAC_2:

        break;

      case COMP_INM_PIN_1:
      case COMP_INM_PIN_2:

        comp_config_inmpin(priv);
        break;

      default:
        return -EINVAL;
    }

  regval |= value;

  /* Configure COMPx non-inverting input. */

  ainfo("\tINP assigned to GPIO%d\n", priv->inp);

  value = priv->inp << COMP_CSR_INPSEL_SHIFT;
  regval |= value;

  comp_config_inppin(priv);

  /* Configure COMPx polarity */

  if (priv->pol == COMP_POL_INVERTED)
    {
      value = COMP_CSR_POL;
      regval |= value;
    }

  /* Configure COMPx hysteresis */

  switch (priv->hyst)
    {
      case COMP_HYST_DIS:
      case COMP_HYST_10MV:
      case COMP_HYST_20MV:
      case COMP_HYST_30MV:
      case COMP_HYST_40MV:
      case COMP_HYST_50MV:
      case COMP_HYST_60MV:
      case COMP_HYST_70MV:

        value = priv->hyst << COMP_CSR_HYST_SHIFT;
        regval |= value;
        break;

      default:
        return -EINVAL;
    }

  /* Configure COMPx blanking signal source */

  switch (priv->blanking)
    {
      case COMP_BLANKING_DIS:
      case COMP_BLANKING_TIMX_OCY_1:
      case COMP_BLANKING_TIMX_OCY_2:
      case COMP_BLANKING_TIMX_OCY_3:
      case COMP_BLANKING_TIMX_OCY_4:
      case COMP_BLANKING_TIMX_OCY_5:
      case COMP_BLANKING_TIMX_OCY_6:
      case COMP_BLANKING_TIMX_OCY_7:

        value = priv->blanking << COMP_CSR_BLANKING_SHIFT;
        regval |= value;
        break;

      default:
        return -EINVAL;
    }

  /* Set Comparator output selection */

#if defined(COMP_OUT_GPIO)
  comp_config_outpin(priv);
#endif

  /* Save CSR register */

  comp_putreg_csr(priv, regval);

  /* Enable Comparator */

  comp_enable(priv, true);

  /* Lock Comparator if needed */

  if (priv->lock)
    {
      comp_lock_set(priv, true);
    }

  return OK;
}

/****************************************************************************
 * Name: comp_enable
 *
 * Description:
 *   Enable/disable comparator
 *
 * Input Parameters:
 *   priv   - A reference to the COMP structure
 *   enable - enable/disable flag
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int comp_enable(struct stm32_comp_s *priv, bool enable)
{
  bool lock;

  ainfo("enable: %d\n", enable ? 1 : 0);

  lock = comp_lock_get(priv);

  if (lock)
    {
      aerr("ERROR: Comparator locked!\n");

      return -EPERM;
    }
  else
    {
      if (enable)
        {
          /* Enable the COMP */

          comp_modify_csr(priv, 0, COMP_CSR_COMPEN);
        }
      else
        {
          /* Disable the COMP */

          comp_modify_csr(priv, COMP_CSR_COMPEN, 0);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: comp_setup
 *
 * Description:
 *   Configure the COMP. This method is called the first time that the COMP
 *   device is opened. This will occur when the port is first opened. This
 *   setup includes configuring and attaching COMP interrupts.
 *   Interrupts are all disabled upon return.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef CONFIG_COMP
static int comp_setup(struct comp_dev_s *dev)
{
#warning "Missing logic"

  return OK;
}
#endif

/****************************************************************************
 * Name: comp_shutdown
 *
 * Description:
 *   Disable the COMP. This method is called when the COMP device is closed.
 *   This method reverses the operation the setup method.
 *   Works only if COMP device is not locked.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_COMP
static void comp_shutdown(struct comp_dev_s *dev)
{
#  warning "Missing logic"
}
#endif

/****************************************************************************
 * Name: comp_read
 *
 * Description:
 *  Get the COMP output state.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   0 if output is low (non-inverting input below inverting input),
 *   1 if output is high (non inverting input above inverting input).
 *
 ****************************************************************************/

#ifdef CONFIG_COMP
static int comp_read(struct comp_dev_s *dev)
{
  struct stm32_comp_s *priv;
  uint32_t regval;

  priv = dev->ad_priv;
  regval = comp_getreg_csr(priv);

  return (((regval & COMP_CSR_VALUE) == 0) ? 0 : 1);
}
#endif

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

#ifdef CONFIG_COMP
static int comp_ioctl(struct comp_dev_s *dev, int cmd, unsigned long arg)
{
#warning "Missing logic"
  return -ENOTTY;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_compinitialize
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

struct comp_dev_s *stm32_compinitialize(int intf)
{
  struct comp_dev_s   *dev;
  struct stm32_comp_s *comp;
  int ret;

  switch (intf)
    {
#ifdef CONFIG_STM32_COMP1
    case 1:
      ainfo("COMP1 selected\n");
      dev = &g_comp1dev;
      break;
#endif

#ifdef CONFIG_STM32_COMP2
    case 2:
      ainfo("COMP2 selected\n");
      dev = &g_comp2dev;
      break;
#endif

#ifdef CONFIG_STM32_COMP3
    case 3:
      ainfo("COMP3 selected\n");
      dev = &g_comp3dev;
      break;
#endif

#ifdef CONFIG_STM32_COMP4
    case 4:
      ainfo("COMP4 selected\n");
      dev = &g_comp4dev;
      break;
#endif

#ifdef CONFIG_STM32_COMP5
    case 5:
      ainfo("COMP5 selected\n");
      dev = &g_comp5dev;
      break;
#endif

#ifdef CONFIG_STM32_COMP6
    case 6:
      ainfo("COMP6 selected\n");
      dev = &g_comp6dev;
      break;
#endif

#ifdef CONFIG_STM32_COMP7
    case 7:
      ainfo("COMP7 selected\n");
      dev = &g_comp7dev;
      break;
#endif

    default:
      aerr("ERROR: No COMP interface defined\n");
      return NULL;
    }

  /* Configure selected comparator */

  comp = dev->ad_priv;

  ret = comp_config(comp);
  if (ret < 0)
    {
      aerr("ERROR: Failed to initialize COMP%d: %d\n", intf, ret);
      return NULL;
    }

  return dev;
}

#endif /* CONFIG_STM32_COMP */
