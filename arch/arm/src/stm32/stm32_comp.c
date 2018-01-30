/****************************************************************************
 * arch/arm/src/stm32/stm32_comp.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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

#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/analog/comp.h>
#include <nuttx/analog/ioctl.h>

#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_comp.h"

/* Some COMP peripheral must be enabled */
/* Up to 7 comparators in STM32F3 Series */

#if defined(CONFIG_STM32_COMP1) || defined(CONFIG_STM32_COMP2) || \
    defined(CONFIG_STM32_COMP3) || defined(CONFIG_STM32_COMP4) || \
    defined(CONFIG_STM32_COMP5) || defined(CONFIG_STM32_COMP6) || \
    defined(CONFIG_STM32_COMP7)

#ifndef CONFIG_STM32_SYSCFG
#  error "SYSCFG clock enable must be set"
#endif

/* @TODO: support for STM32F30XX and STM32F37XX comparators */

#if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F33XX) || \
    defined(CONFIG_STM32_STM32F37XX)

/* Currently only STM32F33XX supported */

#if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX)
#  error "Not supported yet"
#endif

#if defined(CONFIG_STM32_STM32F33XX)
#  if defined(CONFIG_STM32_COMP1) || defined(CONFIG_STM32_COMP3) || \
      defined(CONFIG_STM32_COMP5) || defined(CONFIG_STM32_COMP7)
#    error "STM32F33 supports only COMP2, COMP4 and COMP6"
#  endif
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* COMP2 default configuration **********************************************/

#ifdef CONFIG_STM32_COMP2
#  ifndef COMP2_BLANLKING
#    define COMP2_BLANKING COMP_BLANKING_DEFAULT
#  endif
#  ifndef COMP2_POL
#    define COMP2_POL COMP_BLANKING_DEFAULT
#  endif
#  ifndef COMP2_INM
#    define COMP2_INM COMP_INM_DEFAULT
#  endif
#  ifndef COMP2_OUTSEL
#    define COMP2_OUTSEL COMP_OUTSEL_DEFAULT
#  endif
#  ifndef COMP2_LOCK
#    define COMP2_LOCK COMP_LOCK_DEFAULT
#  endif
#  ifndef GPIO_COMP2_INM
#    warning "GPIO_COMP2_INM not selected. Set default value to GPIO_COMP2_INM1"
#    define GPIO_COMP2_INM GPIO_COMP4_INM_1
#  endif
#endif

/* COMP4 default configuration **********************************************/

#ifdef CONFIG_STM32_COMP4
#  ifndef COMP4_BLANLKING
#    define COMP4_BLANKING COMP_BLANKING_DEFAULT
#  endif
#  ifndef COMP4_POL
#    define COMP4_POL COMP_BLANKING_DEFAULT
#  endif
#  ifndef COMP4_INM
#    define COMP4_INM COMP_INM_DEFAULT
#  endif
#  ifndef COMP4_OUTSEL
#    define COMP4_OUTSEL COMP_OUTSEL_DEFAULT
#  endif
#  ifndef COMP4_LOCK
#    define COMP4_LOCK COMP_LOCK_DEFAULT
#  endif
#  ifndef GPIO_COMP4_INM
#    warning "GPIO_COMP4_INM not selected. Set default value to GPIO_COMP4_INM1"
#    define GPIO_COMP4_INM GPIO_COMP4_INM_1
#  endif
#endif

/* COMP6 default configuration **********************************************/

#ifdef CONFIG_STM32_COMP6
#  ifndef COMP6_BLANLKING
#    define COMP6_BLANKING COMP_BLANKING_DEFAULT
#  endif
#  ifndef COMP6_POL
#    define COMP6_POL COMP_BLANKING_DEFAULT
#  endif
#  ifndef COMP6_INM
#    define COMP6_INM COMP_INM_DEFAULT
#  endif
#  ifndef COMP6_OUTSEL
#    define COMP6_OUTSEL COMP_OUTSEL_DEFAULT
#  endif
#  ifndef COMP6_LOCK
#    define COMP6_LOCK COMP_LOCK_DEFAULT
#  endif
#  ifndef GPIO_COMP6_INM
#    warning "GPIO_COMP6_INM not selected. Set default value to GPIO_COMP6_INM1"
#    define GPIO_COMP6_INM GPIO_COMP6_INM_1
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the configuration of one COMP device */

struct stm32_comp_s
{
  uint8_t blanking;             /* Blanking source */
  uint8_t pol;                  /* Output polarity */
  uint8_t inm;                  /* Inverting input selection */
  uint8_t out;                  /* Comparator output */
  uint8_t lock;                 /* Comparator Lock */
  uint32_t csr;                 /* Control and status register */
#ifndef CONFIG_STM32_STM32F33XX
  uint8_t mode;                 /* Comparator mode */
  uint8_t hyst;                 /* Comparator hysteresis */
                                /* @TODO: Window mode + INP selection */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* COMP Register access */

static inline void comp_modify_csr(FAR struct stm32_comp_s *priv,
                                   uint32_t clearbits, uint32_t setbits);
static inline uint32_t comp_getreg_csr(FAR struct stm32_comp_s *priv);
static inline void comp_putreg_csr(FAR struct stm32_comp_s *priv,
                                   uint32_t value);
static bool stm32_complock_get(FAR struct stm32_comp_s *priv);
static int stm32_complock(FAR struct stm32_comp_s *priv, bool lock);

/* COMP Driver Methods */

static void comp_shutdown(FAR struct comp_dev_s *dev);
static int comp_setup(FAR struct comp_dev_s *dev);
static int comp_read(FAR struct comp_dev_s *dev);
static int comp_ioctl(FAR struct comp_dev_s *dev, int cmd, unsigned long arg);

/* Initialization */

static int stm32_compconfig(FAR struct stm32_comp_s *priv);
static int stm32_compenable(FAR struct stm32_comp_s *priv, bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct comp_ops_s g_compops =
{
  .ao_shutdown  = comp_shutdown,
  .ao_setup     = comp_setup,
  .ao_read      = comp_read,
  .ao_ioctl     = comp_ioctl,
};

#ifdef CONFIG_STM32_COMP1
static struct stm32_comp_s g_comp1priv =
{
  .blanking  = COMP1_BLANKING,
  .pol  = COMP1_POL,
  .inm  = COMP1_INM,
  .out  = COMP1_OUTSEL,
  .lock = COMP1_LOCK,
  .csr  = STM32_COMP1_CSR,
#ifndef CONFIG_STM32_STM32F33XX
  .mode = COMP1_MODE,
  .hyst = COMP1_HYST,
#endif
};

static struct comp_dev_s g_comp1dev =
{
  .ad_ops  = &g_compops,
  .ad_priv = &g_comp1priv,
};
#endif

#ifdef CONFIG_STM32_COMP2
static struct stm32_comp_s g_comp2priv =
{
  .blanking  = COMP2_BLANKING,
  .pol  = COMP2_POL,
  .inm  = COMP2_INM,
  .out  = COMP2_OUTSEL,
  .lock = COMP2_LOCK,
  .csr  = STM32_COMP2_CSR,
#ifndef CONFIG_STM32_STM32F33XX
  .mode = COMP2_MODE,
  .hyst = COMP2_HYST,
#endif
};

static struct comp_dev_s g_comp2dev =
{
  .ad_ops  = &g_compops,
  .ad_priv = &g_comp2priv,
};
#endif

#ifdef CONFIG_STM32_COMP3
static struct stm32_comp_s g_comp3priv =
{
  .blanking  = COMP3_BLANKING,
  .pol  = COMP3_POL,
  .inm  = COMP3_INM,
  .out  = COMP3_OUTSEL,
  .lock = COMP3_LOCK,
  .csr  = STM32_COMP3_CSR,
#ifndef CONFIG_STM32_STM32F33XX
  .mode = COMP3_MODE,
  .hyst = COMP3_HYST,
#endif
};

static struct comp_dev_s g_comp3dev =
{
  .ad_ops  = &g_compops,
  .ad_priv = &g_comp3priv,
};
#endif

#ifdef CONFIG_STM32_COMP4
static struct stm32_comp_s g_comp4priv =
{
  .blanking  = COMP4_BLANKING,
  .pol  = COMP4_POL,
  .inm  = COMP4_INM,
  .out  = COMP4_OUTSEL,
  .lock = COMP4_LOCK,
  .csr  = STM32_COMP4_CSR,
#ifndef CONFIG_STM32_STM32F33XX
  .mode = COMP4_MODE,
  .hyst = COMP4_HYST,
#endif
};

static struct comp_dev_s g_comp4dev =
{
  .ad_ops  = &g_compops,
  .ad_priv = &g_comp4priv,
};
#endif

#ifdef CONFIG_STM32_COMP5
static struct stm32_comp_s g_comp5priv =
{
  .blanking  = COMP5_BLANKING,
  .pol  = COMP5_POL,
  .inm  = COMP5_INM,
  .out  = COMP5_OUTSEL,
  .lock = COMP5_LOCK,
  .csr  = STM32_COMP5_CSR,
#ifndef CONFIG_STM32_STM32F33XX
  .mode = COMP5_MODE,
  .hyst = COMP5_HYST,
#endif
};

static struct comp_dev_s g_comp5dev =
{
  .ad_ops  = &g_compops,
  .ad_priv = &g_comp5priv,
};
#endif

#ifdef CONFIG_STM32_COMP6
static struct stm32_comp_s g_comp6priv =
{
  .blanking  = COMP6_BLANKING,
  .pol  = COMP6_POL,
  .inm  = COMP6_INM,
  .out  = COMP6_OUTSEL,
  .lock = COMP6_LOCK,
  .csr  = STM32_COMP6_CSR,
#ifndef CONFIG_STM32_STM32F33XX
  .mode = COMP6_MODE,
  .hyst = COMP6_HYST,
#endif
};

static struct comp_dev_s g_comp6dev =
{
  .ad_ops  = &g_compops,
  .ad_priv = &g_comp6priv,
};
#endif

#ifdef CONFIG_STM32_COMP7
static struct stm32_comp_s g_comp7priv =
{
  .blanking  = COMP7_BLANKING,
  .pol  = COMP7_POL,
  .inm  = COMP7_INM,
  .out  = COMP7_OUTSEL,
  .lock = COMP7_LOCK,
  .csr  = STM32_COMP7_CSR,
#ifndef CONFIG_STM32_STM32F33XX
  .mode = COMP7_MODE,
  .hyst = COMP7_HYST,
#endif
};

static struct comp_dev_s g_comp7dev =
{
  .ad_ops  = &g_compops,
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

static inline void comp_modify_csr(FAR struct stm32_comp_s *priv,
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

static inline uint32_t comp_getreg_csr(FAR struct stm32_comp_s *priv)
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

static inline void comp_putreg_csr(FAR struct stm32_comp_s *priv,
                                   uint32_t value)
{
  uint32_t csr = priv->csr;

  putreg32(value, csr);
}

/****************************************************************************
 * Name: stm32_comp_complock_get
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

static bool stm32_complock_get(FAR struct stm32_comp_s *priv)
{
  uint32_t regval;

  regval = comp_getreg_csr(priv);

  return (((regval & COMP_CSR_LOCK) == 0) ? false : true);
}

/****************************************************************************
 * Name: stm32_complock
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

static int stm32_complock(FAR struct stm32_comp_s *priv, bool lock)
{
  bool current;

  current = stm32_complock_get(priv);

  if (current)
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

          priv->lock = COMP_LOCK_RO;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_compconfig
 *
 * Description:
 *   Configure comparator and used I/Os
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

static int stm32_compconfig(FAR struct stm32_comp_s *priv)
{
  uint32_t regval = 0;
  int index;

  /* Get comparator index */

  switch (priv->csr)
    {
#ifdef CONFIG_STM32_COMP1
    case STM32_COMP1_CSR:
      index = 1;
      break;
#endif

#ifdef CONFIG_STM32_COMP2
    case STM32_COMP2_CSR:
      index = 2;
      break;
#endif

#ifdef CONFIG_STM32_COMP3
    case STM32_COMP3_CSR:
      index = 3;
      break;
#endif

#ifdef CONFIG_STM32_COMP4
    case STM32_COMP4_CSR:
      index = 4;
      break;
#endif

#ifdef CONFIG_STM32_COMP5
    case STM32_COMP5_CSR:
      index = 5;
      break;
#endif

#ifdef CONFIG_STM32_COMP6
    case STM32_COMP6_CSR:
      index = 6;
      break;
#endif

#ifdef CONFIG_STM32_COMP7
    case STM32_COMP7_CSR:
      index = 7;
      break;
#endif

    default:
      return -EINVAL;
    }

  /* Configure non inverting input */

  switch (index)
    {
#ifdef CONFIG_STM32_COMP1
    case 1:
      stm32_configgpio(GPIO_COMP1_INP);
      break;
#endif

#ifdef CONFIG_STM32_COMP2
    case 2:
      stm32_configgpio(GPIO_COMP2_INP);
      break;
#endif

#ifdef CONFIG_STM32_COMP3
    case 3:
      stm32_configgpio(GPIO_COMP3_INP);
      break;
#endif

#ifdef CONFIG_STM32_COMP4
    case 4:
      stm32_configgpio(GPIO_COMP4_INP);
      break;
#endif

#ifdef CONFIG_STM32_COMP5
    case 5:
      stm32_configgpio(GPIO_COMP5_INP);
      break;
#endif

#ifdef CONFIG_STM32_COMP6
    case 6:
      stm32_configgpio(GPIO_COMP6_INP);
      break;
#endif

#ifdef CONFIG_STM32_COMP7
    case 7:
      stm32_configgpio(GPIO_COMP7_INP);
      break;
#endif

    default:
      return -EINVAL;
    }

  /* Set Comparator inverting input */

  switch (priv->inm)
    {
    case COMP_INMSEL_1P4VREF:
      regval |= COMP_CSR_INMSEL_1P4VREF;
      break;

    case COMP_INMSEL_1P2VREF:
      regval |= COMP_CSR_INMSEL_1P2VREF;
      break;

    case COMP_INMSEL_3P4VREF:
      regval |= COMP_CSR_INMSEL_3P4VREF;
      break;

    case COMP_INMSEL_VREF:
      regval |= COMP_CSR_INMSEL_VREF;
      break;

    case COMP_INMSEL_DAC1CH1:
      regval |= COMP_CSR_INMSEL_DAC1CH1;
      break;

    case COMP_INMSEL_DAC1CH2:
      regval |= COMP_CSR_INMSEL_DAC1CH2;
      break;

    case COMP_INMSEL_PIN:
      {
        /* INMSEL PIN configuration dependent on COMP index */

        switch (index)
          {
            /* TODO: Inverting input pin configuration for COMP1/3/5/7 */

#ifdef CONFIG_STM32_COMP2
          case 2:
            {
              /* COMP2_INM can be PA2 or PA4 */

              stm32_configgpio(GPIO_COMP2_INM);
              regval |= (GPIO_COMP2_INM == GPIO_COMP2_INM_1 ? COMP_CSR_INMSEL_PA2 : COMP_CSR_INMSEL_PA4);
              break;
            }
#endif
#ifdef CONFIG_STM32_COMP4
          case 4:
            {
              /* COMP4_INM can be PB2 or PA4 */

              stm32_configgpio(GPIO_COMP4_INM);
              regval |= (GPIO_COMP4_INM == GPIO_COMP4_INM_1 ? COMP_CSR_INMSEL_PB2 : COMP_CSR_INMSEL_PA4);
              break;
            }
#endif
#ifdef CONFIG_STM32_COMP6
          case 6:
            {
              /* COMP6_INM can be PB15 or PA4 */

              stm32_configgpio(GPIO_COMP6_INM);
              regval |= (GPIO_COMP6_INM == GPIO_COMP6_INM_1 ? COMP_CSR_INMSEL_PB15 : COMP_CSR_INMSEL_PA4);
              break;
            }
#endif
          default :
            return -EINVAL;
          }

        break;
      }

    default:
      return -EINVAL;
    }

  /* Set Comparator output selection */

  switch (priv->out)
    {
    case COMP_OUTSEL_NOSEL:
      regval |= COMP_CSR_OUTSEL_NOSEL;
      break;

    case COMP_OUTSEL_BRKACTH:
      regval |= COMP_CSR_OUTSEL_BRKACTH;
      break;

    case COMP_OUTSEL_BRK2:
      regval |= COMP_CSR_OUTSEL_BRK2;
      break;

    case COMP_OUTSEL_T1OCC:
      regval |= COMP_CSR_OUTSEL_T1OCC;
      break;

    case COMP_OUTSEL_T3CAP3:
      regval |= COMP_CSR_OUTSEL_T3CAP3;
      break;

    case COMP_OUTSEL_T2CAP2:
      regval |= COMP_CSR_OUTSEL_T2CAP2;
      break;

    case COMP_OUTSEL_T1CAP1:
      regval |= COMP_CSR_OUTSEL_T1CAP1;
      break;

    case COMP_OUTSEL_T2CAP4:
      regval |= COMP_CSR_OUTSEL_T2CAP4;
      break;

    case COMP_OUTSEL_T15CAP2:
      regval |= COMP_CSR_OUTSEL_T15CAP2;
      break;

    case COMP_OUTSEL_T2OCC:
      if (index == 2)
        {
          regval |= COMP2_CSR_OUTSEL_T2OCC;
        }
      else if (index == 6)
        {
          regval |= COMP6_CSR_OUTSEL_T2OCC;
        }

      break;

    case COMP_OUTSEL_T16OCC:
      regval |= COMP_CSR_OUTSEL_T16OCC;
      break;

    case COMP_OUTSEL_T3CAP1:
      regval |= COMP_CSR_OUTSEL_T3CAP1;
      break;

    case COMP_OUTSEL_T15OCC:
      regval |= COMP_CSR_OUTSEL_T15OCC;
      break;

    case COMP_OUTSEL_T16CAP1:
      regval |= COMP_CSR_OUTSEL_T16CAP1;
      break;

    case COMP_OUTSEL_T3OCC:
      regval |= COMP_CSR_OUTSEL_T3OCC;
      break;

    default:
      return -EINVAL;
    }

  /* Set Comparator output polarity */

  regval |= (priv->pol == COMP_POL_INVERTED ? COMP_CSR_POL : 0);

  /* Set Comparator output blanking source */

  switch (priv->blanking)
    {
    case COMP_BLANKING_DIS:
      regval |= COMP_CSR_BLANKING_DIS;
      break;

    case COMP_BLANKING_T1OC5:
      regval |= COMP_CSR_BLANKING_T1OC5;
      break;

    case COMP_BLANKING_T3OC4:
      regval |= COMP_CSR_BLANKING_T3OC4;
      break;

    case COMP_BLANKING_T2OC3:
      regval |= COMP_CSR_BLANKING_T2OC3;
      break;

    case COMP_BLANKING_T15OC1:
      regval |= COMP_CSR_BLANKING_T15OC1;
      break;

    case COMP_BLANKING_T2OC4:
      regval |= COMP_CSR_BLANKING_T2OC4;
      break;

    case COMP_BLANKING_T15OC2:
      regval |= COMP_CSR_BLANKING_T15OC1;
      break;

    default:
      return -EINVAL;
    }

  /* Save CSR register */

  comp_putreg_csr(priv, regval);

  /* Enable Comparator */

  stm32_compenable(priv, true);

  /* Lock Comparator if needed */

  if (priv->lock == COMP_LOCK_RO)
    {
      stm32_complock(priv, true);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_compenable
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

static int stm32_compenable(FAR struct stm32_comp_s *priv, bool enable)
{
  bool lock;

  ainfo("enable: %d\n", enable ? 1 : 0);

  lock = stm32_complock_get(priv);

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
 * Name: adc_setup
 *
 * Description:
 *   Configure the COMP. This method is called the first time that the COMP
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching COMP interrupts.
 *   Interrupts are all disabled upon return.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int comp_setup(FAR struct comp_dev_s *dev)
{
#warning "Missing logic"
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
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void comp_shutdown(FAR struct comp_dev_s *dev)
{
#warning "Missing logic"
}

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

static int comp_read(FAR struct comp_dev_s *dev)
{
  FAR struct stm32_comp_s *priv;
  uint32_t regval;

  priv = dev->ad_priv;
  regval = comp_getreg_csr(priv);

  return (((regval & COMP_CSR_OUT) == 0) ? 0 : 1);
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
 *   Valid COMP device structure reference on succcess; a NULL on failure.
 *
 * Assumptions:
 *   1. Clock to the COMP block has enabled,
 *   2. Board-specific logic has already configured
 *
 ****************************************************************************/

FAR struct comp_dev_s* stm32_compinitialize(int intf)
{
  FAR struct comp_dev_s  *dev;
  FAR struct stm32_comp_s *comp;
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

  ret = stm32_compconfig(comp);
  if (ret < 0)
    {
      aerr("ERROR: Failed to initialize COMP%d: %d\n", intf, ret);
      return NULL;
    }

  return dev;
}

#endif  /* CONFIG_STM32_STM32F30XX || CONFIG_STM32_STM32F33XX ||
         * CONFIG_STM32_STM32F37XX*/

#endif  /* CONFIG_STM32_COMP2 || CONFIG_STM32_COMP4 ||
         * CONFIG_STM32_COMP6 */
