/****************************************************************************
 * arch/arm/src/stm32/stm32_opamp.c
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
#include <nuttx/analog/opamp.h>
#include <nuttx/analog/ioctl.h>

#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_opamp.h"

/* OPAMP "upper half" support must be enabled */

#ifdef CONFIG_STM32_OPAMP

/* Some OPAMP peripheral must be enabled */
/* Up to 4 OPAMPs in STM32F3 Series */

#if defined(CONFIG_STM32_OPAMP1) || defined(CONFIG_STM32_OPAMP2) || \
    defined(CONFIG_STM32_OPAMP3) || defined(CONFIG_STM32_OPAMP4)

#ifndef CONFIG_STM32_SYSCFG
#  error "SYSCFG clock enable must be set"
#endif

/* @TODO: support for STM32F30XX opamps */

#if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F33XX)

/* Currently only STM32F33XX supported */

#if defined(CONFIG_STM32_STM32F30XX)
#  error "Not supported yet"
#endif

#if defined(CONFIG_STM32_STM32F33XX)
#  if defined(CONFIG_STM32_OPAMP1) || defined(CONFIG_STM32_OPAMP3) ||     \
      defined(CONFIG_STM32_OPAMP4)
#    error "STM32F33 supports only OPAMP2"
#  endif
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* OPAMPs default configuration *********************************************/

#ifdef CONFIG_STM32_OPAMP1
#  ifndef OPAMP1_MODE
#    define OPAMP1_MODE OPAMP_MODE_DEFAULT
#  endif
#  ifndef OPAMP1_MUX
#    define OPAMP1_MUX OPAMP_MUX_DEFAULT
#  endif
#  ifndef OPAMP1_USERCAL
#    define OPAMP1_USERCAL OPAMP_USERCAL_DEFAULT
#  endif
#  ifndef OPAMP1_LOCK
#    define OPAMP1_LOCK OPAMP_LOCK_DEFAULT
#  endif
#  ifndef OPAMP1_GAIN
#    define OPAMP1_GAIN OPAMP_GAIN_DEFAULT
#  endif
#endif
#ifdef CONFIG_STM32_OPAMP2
#  ifndef OPAMP2_MODE
#    define OPAMP2_MODE OPAMP_MODE_DEFAULT
#  endif
#  ifndef OPAMP2_MUX
#    define OPAMP2_MUX OPAMP_MUX_DEFAULT
#  endif
#  ifndef OPAMP2_USERCAL
#    define OPAMP2_USERCAL OPAMP_USERCAL_DEFAULT
#  endif
#  ifndef OPAMP2_LOCK
#    define OPAMP2_LOCK OPAMP_LOCK_DEFAULT
#  endif
#  ifndef OPAMP2_GAIN
#    define OPAMP2_GAIN OPAMP_GAIN_DEFAULT
#  endif
#endif
#ifdef CONFIG_STM32_OPAMP3
#  ifndef OPAMP3_MODE
#    define OPAMP3_MODE OPAMP_MODE_DEFAULT
#  endif
#  ifndef OPAMP3_MUX
#    define OPAMP3_MUX OPAMP_MUX_DEFAULT
#  endif
#  ifndef OPAMP3_USERCAL
#    define OPAMP3_USERCAL OPAMP_USERCAL_DEFAULT
#  endif
#  ifndef OPAMP3_LOCK
#    define OPAMP3_LOCK OPAMP_LOCK_DEFAULT
#  endif
#  ifndef OPAMP3_GAIN
#    define OPAMP3_GAIN OPAMP_GAIN_DEFAULT
#  endif
#endif
#ifdef CONFIG_STM32_OPAMP4
#  ifndef OPAMP4_MODE
#    define OPAMP4_MODE OPAMP_MODE_DEFAULT
#  endif
#  ifndef OPAMP4_MUX
#    define OPAMP4_MUX OPAMP_MUX_DEFAULT
#  endif
#  ifndef OPAMP4_USERCAL
#    define OPAMP4_USERCAL OPAMP_USERCAL_DEFAULT
#  endif
#  ifndef OPAMP4_LOCK
#    define OPAMP4_LOCK OPAMP_LOCK_DEFAULT
#  endif
#  ifndef OPAMP4_GAIN
#    define OPAMP4_GAIN OPAMP_GAIN_DEFAULT
#  endif
#endif

/* Some assertions  *******************************************************/

/* Check OPAMPs inputs selection  */

#ifdef CONFIG_STM32_OPAMP1
#  if (OPAMP1_MODE == OPAMP_MODE_FOLLOWER)
#    define OPAMP1_VMSEL OPAMP1_VMSEL_FOLLOWER
#  endif
#  if (OPAMP1_MODE == OPAMP_MODE_PGA)
#    define OPAMP1_VMSEL OPAMP1_VMSEL_PGA
#  endif
#  if (OPAMP1_MODE == OPAMP_MODE_STANDALONE)
#    ifndef OPAMP1_VMSEL
#      error "OPAMP1_VMSEL must be selected in standalone mode!"
#    endif
#  endif
#  ifndef OPAMP1_VPSEL
#    error "OPAMP1_VPSEL must be slected in standalone mode!"
#  endif
#endif
#ifdef CONFIG_STM32_OPAMP2
#  if (OPAMP2_MODE == OPAMP_MODE_FOLLOWER)
#    define OPAMP2_VMSEL OPAMP2_VMSEL_FOLLOWER
#  endif
#  if (OPAMP2_MODE == OPAMP_MODE_PGA)
#    define OPAMP2_VMSEL OPAMP2_VMSEL_PGA
#  endif
#  if (OPAMP2_MODE == OPAMP_MODE_STANDALONE)
#    ifndef OPAMP2_VMSEL
#      error "OPAMP2_VMSEL must be selected in standalone mode!"
#    endif
#  endif
#  ifndef OPAMP2_VPSEL
#    error "OPAMP2_VPSEL must be slected in standalone mode!"
#  endif
#endif
#ifdef CONFIG_STM32_OPAMP3
#  if (OPAMP3_MODE == OPAMP_MODE_FOLLOWER)
#    define OPAMP3_VMSEL OPAMP3_VMSEL_FOLLOWER
#  endif
#  if (OPAMP3_MODE == OPAMP_MODE_PGA)
#    define OPAMP3_VMSEL OPAMP3_VMSEL_PGA
#  endif
#  if (OPAMP3_MODE == OPAMP_MODE_STANDALONE)
#    ifndef OPAMP3_VMSEL
#      error "OPAMP3_VMSEL must be selected in standalone mode!"
#    endif
#  endif
#  ifndef OPAMP3_VPSEL
#    error "OPAMP3_VPSEL must be slected in standalone mode!"
#  endif
#endif
#ifdef CONFIG_STM32_OPAMP4
#  if (OPAMP4_MODE == OPAMP_MODE_FOLLOWER)
#    define OPAMP4_VMSEL OPAMP4_VMSEL_FOLLOWER
#  endif
#  if (OPAMP4_MODE == OPAMP_MODE_PGA)
#    define OPAMP4_VMSEL OPAMP4_VMSEL_PGA
#  endif
#  if (OPAMP4_MODE == OPAMP_MODE_STANDALONE)
#    ifndef OPAMP4_VMSEL
#      error "OPAMP4_VMSEL must be selected in standalone mode!"
#    endif
#  endif
#  ifndef OPAMP4_VPSEL
#    error "OPAMP4_VPSEL must be slected in standalone mode!"
#  endif
#endif

/*  When OPAMP MUX enabled, make sure that secondary selection inputs are configured */

#ifdef CONFIG_STM32_OPAMP1
#  if (OPAMP1_MUX == OPAMP_MUX_ENABLE)
#    if !defined(OPAMP1_VMSSEL) || !defined(OPAMP1_VPSSEL)
#      error "OPAMP1_VMSSEL and OPAMP1_VPSSEL must be selected when OPAMP1 MUX enabled!"
#    endif
#  endif
#endif
#ifdef CONFIG_STM32_OPAMP2
#  if (OPAMP2_MUX == OPAMP_MUX_ENABLE)
#    if !defined(OPAMP2_VMSSEL) || !defined(OPAMP2_VPSSEL)
#      error "OPAMP2_VMSSEL and OPAMP2_VPSSEL must be selected when OPAMP2 MUX enabled!"
#    endif
#  endif
#endif
#ifdef CONFIG_STM32_OPAMP3
#  if (OPAMP3_MUX == OPAMP_MUX_ENABLE)
#    if !defined(OPAMP3_VMSSEL) || !defined(OPAMP3_VPSSEL)
#      error "OPAMP3_VMSSEL and OPAMP3_VPSSEL must be selected when OPAMP3 MUX enabled!"
#    endif
#  endif
#endif
#ifdef CONFIG_STM32_OPAMP4
#  if (OPAMP4_MUX == OPAMP_MUX_ENABLE)
#    if !defined(OPAMP4_VMSSEL) || !defined(OPAMP4_VPSSEL)
#      error "OPAMP4_VMSSEL and OPAMP4_VPSSEL must be selected when OPAMP4 MUX enabled!"
#    endif
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the configuration of one OPAMP device */

struct stm32_opamp_s
{
  uint32_t csr;                 /* Control and status register */

  uint8_t lock:1;               /* OPAMP lock */
  uint8_t mux:1;                /* Timer controlled MUX mode */
  uint8_t mode:2;               /* OPAMP mode */
  uint8_t gain:4;               /* OPAMP gain in PGA mode */

  uint8_t vm_sel:2;             /* Inverting input selection */
  uint8_t vp_sel:2;             /* Non inverting input selection */
  uint8_t vms_sel:2;            /* Inverting input secondary selection (MUX mode) */
  uint8_t vps_sel:2;            /* Non inverting input secondary selection (Mux mode) */

  uint16_t trim_n:5;            /* Offset trimming value (NMOS) */
  uint16_t trim_p:5;            /* Offset trimming value (PMOS) */
  uint16_t _reserved:6;         /* reserved for calibration */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* OPAMP Register access */

static inline void opamp_modify_csr(FAR struct stm32_opamp_s *priv,
                                   uint32_t clearbits, uint32_t setbits);
static inline uint32_t opamp_getreg_csr(FAR struct stm32_opamp_s* priv);
static inline void opamp_putreg_csr(FAR struct stm32_opamp_s* priv,
                                    uint32_t value);
static bool stm32_opamplock_get(FAR struct stm32_opamp_s *priv);
static int stm32_opamplock(FAR struct stm32_opamp_s *priv, bool lock);

/* Initialization */

static int stm32_opampconfig(FAR struct stm32_opamp_s *priv);
static int stm32_opampenable(FAR struct stm32_opamp_s *priv, bool enable);
static int stm32_opampgain_set(FAR struct stm32_opamp_s *priv, uint8_t gain);
#if 0
static int stm32_opampcalibrate(FAR struct stm32_opamp_s *priv);
#endif

/* OPAMP Driver Methods */

static void opamp_shutdown(FAR struct opamp_dev_s *dev);
static int opamp_setup(FAR struct opamp_dev_s *dev);
static int opamp_ioctl(FAR struct opamp_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct opamp_ops_s g_opampops =
{
  .ao_shutdown  = opamp_shutdown,
  .ao_setup     = opamp_setup,
  .ao_ioctl     = opamp_ioctl
};

#ifdef CONFIG_STM32_OPAMP1
static struct stm32_opamp_s g_opamp1priv =
{
  .csr = STM32_OPAMP1_CSR,
  .lock = OPAMP1_LOCK,
  .mux = OPAMP1_MUX,
  .mode = OPAMP1_MODE,
  .vm_sel = OPAMP1_VMSEL,
  .vp_sel = OPAMP1_VPSEL,
#if OPAMP1_MUX == OPAMP_MUX_ENABLE
  .vms_sel = OPAMP1_VMSSEL,
  .vps_sel = OPAMP1_VPSSEL,
#endif
  .gain = OPAMP1_GAIN
};

static struct opamp_dev_s g_opamp1dev =
{
  .ad_ops = &g_opampops,
  .ad_priv = &g_opamp1priv
};
#endif

#ifdef CONFIG_STM32_OPAMP2
static struct stm32_opamp_s g_opamp2priv =
{
  .csr = STM32_OPAMP2_CSR,
  .lock = OPAMP2_LOCK,
  .mux = OPAMP2_MUX,
  .mode = OPAMP2_MODE,
  .vm_sel = OPAMP2_VMSEL,
  .vp_sel = OPAMP2_VPSEL,
#if OPAMP2_MUX == OPAMP_MUX_ENABLE
  .vms_sel = OPAMP2_VMSSEL,
  .vps_sel = OPAMP2_VPSSEL,
#endif
  .gain = OPAMP2_GAIN
};

static struct opamp_dev_s g_opamp2dev =
  {
    .ad_ops = &g_opampops,
    .ad_priv = &g_opamp2priv
  };
#endif

#ifdef CONFIG_STM32_OPAMP3
static struct stm32_opamp_s g_opamp3priv =
{
  .csr = STM32_OPAMP3_CSR,
  .lock = OPAMP3_LOCK,
  .mux = OPAMP3_MUX,
  .mode = OPAMP3_MODE,
  .vm_sel = OPAMP3_VMSEL,
  .vp_sel = OPAMP3_VPSEL,
#if OPAMP3_MUX == OPAMP_MUX_ENABLE
  .vms_sel = OPAMP3_VMSSEL,
  .vps_sel = OPAMP3_VPSSEL,
#endif
  .gain = OPAMP3_GAIN
};

static struct opamp_dev_s g_opamp3dev =
{
  .ad_ops = &g_opampops,
  .ad_priv = &g_opamp3priv
};
#endif

#ifdef CONFIG_STM32_OPAMP4
static struct stm32_opamp_s g_opamp4priv =
{
  .csr = STM32_OPAMP4_CSR,
  .lock = OPAMP4_LOCK,
  .mux = OPAMP4_MUX,
  .mode = OPAMP4_MODE,
  .vm_sel = OPAMP4_VMSEL,
  .vp_sel = OPAMP4_VPSEL,
#if OPAMP4_MUX == OPAMP_MUX_ENABLE
  .vms_sel = OPAMP4_VMSSEL,
  .vps_sel = OPAMP4_VPSSEL,
#endif
  .gain = OPAMP4_GAIN
};

static struct opamp_dev_s g_opamp4dev =
{
  .ad_ops = &g_opampops,
  .ad_priv = &g_opamp4priv
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: opamp_modify_csr
 *
 * Description:
 *   Modify the value of a 32-bit OPAMP CSR register (not atomic).
 *
 * Input Parameters:
 *   priv   - A reference to the OPAMP structure
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void opamp_modify_csr(FAR struct stm32_opamp_s *priv,
                                   uint32_t clearbits, uint32_t setbits)
{
  uint32_t csr = priv->csr;

  modifyreg32(csr, clearbits, setbits);
}

/****************************************************************************
 * Name: opamp_getreg_csr
 *
 * Description:
 *   Read the value of an OPAMP CSR register
 *
 * Input Parameters:
 *   priv   - A reference to the OPAMP structure
 *
 * Returned Value:
 *   The current contents of the OPAMP CSR register
 *
 ****************************************************************************/

static inline uint32_t opamp_getreg_csr(FAR struct stm32_opamp_s *priv)
{
  uint32_t csr = priv->csr;

  return getreg32(csr);
}

/****************************************************************************
 * Name: opamp_putreg_csr
 *
 * Description:
 *   Write a value to an OPAMP register.
 *
 * Input Parameters:
 *   priv   - A reference to the OPAMP structure
 *   value  - The value to write to the OPAMP CSR register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void opamp_putreg_csr(FAR struct stm32_opamp_s *priv,
                                   uint32_t value)
{
  uint32_t csr = priv->csr;

  putreg32(value, csr);
}

/****************************************************************************
 * Name: stm32_opamp_opamplock_get
 *
 * Description:
 *   Get OPAMP lock bit state
 *
 * Input Parameters:
 *   priv   - A reference to the OPAMP structure
 *
 * Returned Value:
 *   True if OPAMP locked, false if not locked
 *
 ****************************************************************************/

static bool stm32_opamplock_get(FAR struct stm32_opamp_s *priv)
{
  uint32_t regval;

  regval = opamp_getreg_csr(priv);

  return (((regval & OPAMP_CSR_LOCK) == 0) ? false : true);
}

/****************************************************************************
 * Name: stm32_opamplock
 *
 * Description:
 *   Lock OPAMP CSR register
 *
 * Input Parameters:
 *   priv   - A reference to the OPAMP structure
 *   enable - lock flag
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int stm32_opamplock(FAR struct stm32_opamp_s *priv, bool lock)
{
  bool current;

  current = stm32_opamplock_get(priv);

  if (current)
    {
      if (lock == false)
        {
          aerr("ERROR: OPAMP LOCK can be cleared only by a system reset\n");

          return -EPERM;
        }
    }
  else
    {
      if (lock == true)
        {
          opamp_modify_csr(priv, 0, OPAMP_CSR_LOCK);

          priv->lock = OPAMP_LOCK_RO;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_opampconfig
 *
 * Description:
 *   Configure OPAMP and used I/Os
 *
 * Input Parameters:
 *   priv   - A reference to the OPAMP structure
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int stm32_opampconfig(FAR struct stm32_opamp_s *priv)
{
  uint32_t regval = 0;
  int index;

  /* Get OPAMP index */

  switch (priv->csr)
    {
#ifdef CONFIG_STM32_OPAMP1
    case STM32_OPAMP1_CSR:
      index = 1;
      break;
#endif

#ifdef CONFIG_STM32_OPAMP2
    case STM32_OPAMP2_CSR:
      index = 2;
      break;
#endif

#ifdef CONFIG_STM32_OPAMP3
    case STM32_OPAMP3_CSR:
      index = 3;
      break;
#endif

#ifdef CONFIG_STM32_OPAMP4
    case STM32_OPAMP4_CSR:
      index = 4;
      break;
#endif

    default:
      return -EINVAL;
    }

  /* Configure non inverting input */

  switch (index)
    {
#ifdef CONFIG_STM32_OPAMP1
    case 1:
      {
        switch (priv->vp_sel)
          {
          case OPAMP1_VPSEL_PA7:
            stm32_configgpio(GPIO_OPAMP1_VINP_1);
            regval |= OPAMP_CSR_VPSEL_PA7;
            break;

          case OPAMP1_VPSEL_PA5:
            stm32_configgpio(GPIO_OPAMP1_VINP_2);
            regval |= OPAMP_CSR_VPSEL_PA5;
            break;

          case OPAMP1_VPSEL_PA3:
            stm32_configgpio(GPIO_OPAMP1_VINP_3);
            regval |= OPAMP_CSR_VPSEL_PA3;
            break;

          case OPAMP1_VPSEL_PA1:
            stm32_configgpio(GPIO_OPAMP1_VINP_4);
            regval |= OPAMP_CSR_VPSEL_PA1;
            break;

          default:
            return -EINVAL;
          }
        break;
      }
#endif

#ifdef CONFIG_STM32_OPAMP2
    case 2:
      {
        switch (priv->vp_sel)
          {
#ifndef CONFIG_STM32_STM32F33XX
          case OPAMP2_VPSEL_PD14:
            stm32_configgpio(GPIO_OPAMP2_VINP_1);
            regval |= OPAMP_CSR_VPSEL_PD14;
            break;
#endif
          case OPAMP2_VPSEL_PB14:
            stm32_configgpio(GPIO_OPAMP2_VINP_2);
            regval |= OPAMP_CSR_VPSEL_PB14;
            break;

          case OPAMP2_VPSEL_PB0:
            stm32_configgpio(GPIO_OPAMP2_VINP_3);
            regval |= OPAMP_CSR_VPSEL_PB0;
            break;

          case OPAMP2_VPSEL_PA7:
            stm32_configgpio(GPIO_OPAMP2_VINP_4);
            regval |= OPAMP_CSR_VPSEL_PA7;
            break;

          default:
            return -EINVAL;
          }
        break;
      }
#endif

#ifdef CONFIG_STM32_OPAMP3
    case 3:
      {
        switch (priv->vp_sel)
          {
          case OPAMP3_VPSEL_PB13:
            stm32_configgpio(GPIO_OPAMP3_VINP_1);
            regval |= OPAMP_CSR_VPSEL_PB13;
            break;

          case OPAMP3_VPSEL_PA5:
            stm32_configgpio(GPIO_OPAMP3_VINP_2);
            regval |= OPAMP_CSR_VPSEL_PA5;
            break;

          case OPAMP3_VPSEL_PA1:
            stm32_configgpio(GPIO_OPAMP3_VINP_3);
            regval |= OPAMP_CSR_VPSEL_PA1;
            break;

          case OPAMP3_VPSEL_PB0:
            stm32_configgpio(GPIO_OPAMP3_VINP_4);
            regval |= OPAMP_CSR_VPSEL_PB0;
            break;

          default:
            return -EINVAL;
          }
        break;
      }
#endif

#ifdef CONFIG_STM32_OPAMP4
    case 4:
      {
        switch (priv->vp_sel)
          {
          case OPAMP4_VPSEL_PD11:
            stm32_configgpio(GPIO_OPAMP4_VINP_1);
            regval |= OPAMP_CSR_VPSEL_PD11;
            break;

          case OPAMP4_VPSEL_PB11:
            stm32_configgpio(GPIO_OPAMP4_VINP_2);
            regval |= OPAMP_CSR_VPSEL_PB11;
            break;

          case OPAMP4_VPSEL_PA4:
            stm32_configgpio(GPIO_OPAMP4_VINP_3);
            regval |= OPAMP_CSR_VPSEL_PA4;
            break;

          case OPAMP4_VPSEL_PB13:
            stm32_configgpio(GPIO_OPAMP4_VINP_4;
            regval |= OPAMP_CSR_VPSEL_PB13;
            break;

          default:
            return -EINVAL;
          }
        break;
      }
#endif

    default:
      return -EINVAL;
    }

  /* Configure inverting input */

    switch (index)
    {
#ifdef CONFIG_STM32_OPAMP1
    case 1:
      {
        switch (priv->vm_sel)
          {
          case OPAMP1_VSEL_PC5:
            stm32_configgpio(GPIO_OPAMP1_VINM_1);
            regval |= OPAMP_CSR_VMSEL_PC5;
            break;

          case OPAMP1_VMSEL_PA3:
            stm32_configgpio(GPIO_OPAMP1_VINM_2);
            regval |= OPAMP_CSR_VMSEL_PA3;
            break;

          case OPAMP1_VMSEL_PGAMODE:
            regval |= OPAMP_CSR_VMSEL_PGA;
            break;

          case OPAMP1_VMSEL_FOLLOWER:
            regval |= OPAMP_CSR_VMSEL_FOLLOWER;
            break;

          default:
            return -EINVAL;
          }
        break;
      }
#endif

#ifdef CONFIG_STM32_OPAMP2
    case 2:
      {
        switch (priv->vm_sel)
          {
          case OPAMP2_VMSEL_PC5:
            stm32_configgpio(GPIO_OPAMP2_VINM_1);
            regval |= OPAMP_CSR_VMSEL_PC5;
            break;

          case OPAMP2_VMSEL_PA5:
            stm32_configgpio(GPIO_OPAMP2_VINM_2);
            regval |= OPAMP_CSR_VMSEL_PA5;
            break;

          case OPAMP2_VMSEL_PGAMODE:
            regval |= OPAMP_CSR_VMSEL_PGA;
            break;

          case OPAMP2_VMSEL_FOLLOWER:
            regval |= OPAMP_CSR_VMSEL_FOLLOWER;
            break;

          default:
            return -EINVAL;
          }
        break;
      }
#endif

#ifdef CONFIG_STM32_OPAMP3
    case 3:
      {
        switch (priv->vm_sel)
          {
          case OPAMP3_VMSEL_PB10:
            stm32_configgpio(GPIO_OPAMP3_VINM_1);
            regval |= OPAMP_CSR_VMSEL_PB10;
            break;

          case OPAMP3_VMSEL_PB2:
            stm32_configgpio(GPIO_OPAMP3_VINM_2);
            regval |= OPAMP_CSR_VMSEL_PB2;
            break;

          case OPAMP3_VMSEL_PGAMODE:
            regval |= OPAMP_CSR_VMSEL_PGA;
            break;

          case OPAMP3_VMSEL_FOLLOWER:
            regval |= OPAMP_CSR_VMSEL_FOLLOWER;
            break;

          default:
            return -EINVAL;
          }
        break;
      }
#endif

#ifdef CONFIG_STM32_OPAMP4
    case 4:
      {
        switch (priv->vm_sel)
          {
          case OPAMP4_VMSEL_PB10:
            stm32_configgpio(GPIO_OPAMP4_VINM_1);
            regval |= OPAMP_CSR_VMSEL_PB10;
            break;

          case OPAMP4_VMSEL_PD8:
            stm32_configgpio(GPIO_OPAMP4_VINM_2);
            regval |= OPAMP_CSR_VMSEL_PD8;
            break;

          case OPAMP4_VMSEL_PGAMODE:
            regval |= OPAMP_CSR_VMSEL_PGA;
            break;

          case OPAMP4_VMSEL_FOLLOWER:
            regval |= OPAMP_CSR_VMSEL_FOLLOWER;
            break;

          default:
            return -EINVAL;
          }
        break;
      }
#endif

    default:
      return -EINVAL;
    }

    if (priv->mux == 1)
      {
        /* Enable Timer controled Mux mode */

        regval |= OPAMP_CSR_TCMEN;

        /* Configure non inverting secondary input */

        switch (index)
          {
#ifdef CONFIG_STM32_OPAMP1
          case 1:
            {
              switch (priv->vps_sel)
                {
                case OPAMP1_VPSEL_PA7:
                  stm32_configgpio(GPIO_OPAMP1_VINP_1);
                  regval |= OPAMP_CSR_VPSSEL_PA7;
                  break;

                case OPAMP1_VPSEL_PA5:
                  stm32_configgpio(GPIO_OPAMP1_VINP_2);
                  regval |= OPAMP_CSR_VPSSEL_PA5;
                  break;

                case OPAMP1_VPSEL_PA3:
                  stm32_configgpio(GPIO_OPAMP1_VINP_3);
                  regval |= OPAMP_CSR_VPSSEL_PA3;
                  break;

                case OPAMP1_VPSEL_PA1:
                  stm32_configgpio(GPIO_OPAMP1_VINP_4);
                  regval |= OPAMP_CSR_VPSSEL_PA1;
                  break;

                default:
                  return -EINVAL;
                }
              break;
            }
#endif

#ifdef CONFIG_STM32_OPAMP2
          case 2:
            {
              switch (priv->vps_sel)
                {
#ifndef CONFIG_STM32_STM32F33XX
                case OPAMP2_VPSEL_PD14:
                  stm32_configgpio(GPIO_OPAMP2_VINP_1);
                  regval |= OPAMP_CSR_VPSSEL_PD14;
                  break;
#endif
                case OPAMP2_VPSEL_PB14:
                  stm32_configgpio(GPIO_OPAMP2_VINP_2);
                  regval |= OPAMP_CSR_VPSSEL_PB14;
                  break;

                case OPAMP2_VPSEL_PB0:
                  stm32_configgpio(GPIO_OPAMP2_VINP_3);
                  regval |= OPAMP_CSR_VPSSEL_PB0;
                  break;

                case OPAMP2_VPSEL_PA7:
                  stm32_configgpio(GPIO_OPAMP2_VINP_4);
                  regval |= OPAMP_CSR_VPSSEL_PA7;
                  break;

                default:
                  return -EINVAL;
                }
              break;
            }
#endif

#ifdef CONFIG_STM32_OPAMP3
          case 3:
            {
              switch (priv->vps_sel)
                {
                case OPAMP3_VPSEL_PB13:
                  stm32_configgpio(GPIO_OPAMP3_VINP_1);
                  regval |= OPAMP_CSR_VPSSEL_PB13;
                  break;

                case OPAMP3_VPSEL_PA5:
                  stm32_configgpio(GPIO_OPAMP3_VINP_2);
                  regval |= OPAMP_CSR_VPSSEL_PA5;
                  break;

                case OPAMP3_VPSEL_PA1:
                  stm32_configgpio(GPIO_OPAMP3_VINP_3);
                  regval |= OPAMP_CSR_VPSSEL_PA1;
                  break;

                case OPAMP3_VPSEL_PB0:
                  stm32_configgpio(GPIO_OPAMP3_VINP_4);
                  regval |= OPAMP_CSR_VPSSEL_PB0;
                  break;

                default:
                  return -EINVAL;
                }
              break;
            }
#endif

#ifdef CONFIG_STM32_OPAMP4
          case 4:
            {
              switch (priv->vps_sel)
                {
                case OPAMP4_VPSEL_PD11:
                  stm32_configgpio(GPIO_OPAMP4_VINP_1);
                  regval |= OPAMP_CSR_VPSSEL_PD11;
                  break;

                case OPAMP4_VPSEL_PB11:
                  stm32_configgpio(GPIO_OPAMP4_VINP_2);
                  regval |= OPAMP_CSR_VPSSEL_PB11;
                  break;

                case OPAMP4_VPSEL_PA4:
                  stm32_configgpio(GPIO_OPAMP4_VINP_3);
                  regval |= OPAMP_CSR_VPSSEL_PA4;
                  break;

                case OPAMP4_VPSEL_PB13:
                  stm32_configgpio(GPIO_OPAMP4_VINP_4);
                  regval |= OPAMP_CSR_VPSSEL_PB13;
                  break;

                default:
                  return -EINVAL;
                }
              break;
            }
#endif

              default:
                return -EINVAL;
            }

            /* Configure inverting secondary input */

            switch (index)
              {
#ifdef CONFIG_STM32_OPAMP1
              case 1:
                {
                  switch (priv->vms_sel)
                    {
                    case OPAMP1_VSEL_PC5:
                      stm32_configgpio(GPIO_OPAMP1_VINM_1);
                      regval &= ~OPAMP_CSR_VMSSEL;
                      break;

                    case OPAMP1_VMSEL_PA3:
                      stm32_configgpio(GPIO_OPAMP1_VINM_2);
                      regval |= OPAMP_CSR_VMSSEL;
                      break;

                    default:
                      return -EINVAL;
                    }
                  break;
                }
#endif

#ifdef CONFIG_STM32_OPAMP2
              case 2:
                {
                  switch (priv->vms_sel)
                    {
                    case OPAMP2_VMSEL_PC5:
                      stm32_configgpio(GPIO_OPAMP2_VINM_1);
                      regval &= ~OPAMP_CSR_VMSSEL;
                      break;

                    case OPAMP2_VMSEL_PA5:
                      stm32_configgpio(GPIO_OPAMP2_VINM_2);
                      regval |= OPAMP_CSR_VMSSEL;
                      break;

                    default:
                      return -EINVAL;
                    }
                  break;
                }
#endif

#ifdef CONFIG_STM32_OPAMP3
              case 3:
                {
                  switch (priv->vms_sel)
                    {
                    case OPAMP3_VMSEL_PB10:
                      stm32_configgpio(GPIO_OPAMP3_VINM_1);
                      regval &= ~OPAMP_CSR_VMSSEL;
                      break;

                    case OPAMP3_VMSEL_PB2:
                      stm32_configgpio(GPIO_OPAMP3_VINM_2);
                      regval |= OPAMP_CSR_VMSSEL;
                      break;

                    default:
                      return -EINVAL;
                    }
                  break;
                }
#endif

#ifdef CONFIG_STM32_OPAMP4
              case 4:
                {
                  switch (priv->vms_sel)
                    {
                    case OPAMP4_VMSEL_PB10:
                      stm32_configgpio(GPIO_OPAMP4_VINM_1);
                      regval &= ~OPAMP_CSR_VMSSEL;
                      break;

                    case OPAMP4_VMSEL_PD8:
                      stm32_configgpio(GPIO_OPAMP4_VINM_2);
                      regval |= OPAMP_CSR_VMSSEL;
                      break;

                    default:
                      return -EINVAL;
                    }
                  break;
                }
#endif

              default:
                return -EINVAL;
              }
          }

    /* Save CSR register */

    opamp_putreg_csr(priv, regval);

    /* Configure defaul gain in PGA mode */

    stm32_opampgain_set(priv, priv->gain);

    /* Enable OPAMP */

    stm32_opampenable(priv, true);

    /* TODO: OPAMP user calibration */
    /* stm32_opampcalibrate(priv); */


    /* Lock OPAMP if needed */

    if (priv->lock == OPAMP_LOCK_RO)
      {
        stm32_opamplock(priv, true);
      }

    return OK;
}

/****************************************************************************
 * Name: stm32_opampenable
 *
 * Description:
 *   Enable/disable OPAMP
 *
 * Input Parameters:
 *   priv   - A reference to the OPAMP structure
 *   enable - enable/disable flag
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int stm32_opampenable(FAR struct stm32_opamp_s *priv, bool enable)
{
  bool lock;

  ainfo("enable: %d\n", enable ? 1 : 0);

  lock = stm32_opamplock_get(priv);

  if (lock)
    {
      aerr("ERROR: OPAMP locked!\n");

      return -EPERM;
    }
  else
    {
      if (enable)
        {
          /* Enable the OPAMP */

          opamp_modify_csr(priv, 0, OPAMP_CSR_OPAMPEN);
        }
      else
        {
          /* Disable the OPAMP */

          opamp_modify_csr(priv, OPAMP_CSR_OPAMPEN, 0);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_opampgain_set
 *
 * Description:
 *   Set OPAMP gain
 *
 * Input Parameters:
 *   priv   - A reference to the OPAMP structure
 *   gain   - OPAMP gain
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int stm32_opampgain_set(FAR struct stm32_opamp_s *priv, uint8_t gain)
{
  bool lock;
  uint32_t regval = 0;

  lock = stm32_opamplock_get(priv);

  if (lock)
    {
      aerr("ERROR: OPAMP locked!\n");
      return -EPERM;
    }

  regval = opamp_getreg_csr(priv);

  switch (gain)
    {
    case OPAMP_GAIN_2:
      regval |= OPAMP_CSR_PGAGAIN_2;
      break;
    case OPAMP_GAIN_4:
      regval |= OPAMP_CSR_PGAGAIN_4;
      break;
    case OPAMP_GAIN_8:
      regval |= OPAMP_CSR_PGAGAIN_8;
      break;
    case OPAMP_GAIN_2_VM0:
      regval |= OPAMP_CSR_PGAGAIN_2VM0;
      break;
    case OPAMP_GAIN_4_VM0:
      regval |= OPAMP_CSR_PGAGAIN_4VM0;
      break;
    case OPAMP_GAIN_8_VM0:
      regval |= OPAMP_CSR_PGAGAIN_8VM0;
      break;
    case OPAMP_GAIN_16_VM0:
      regval |= OPAMP_CSR_PGAGAIN_16VM0;
      break;
    case OPAMP_GAIN_2_VM1:
      regval |= OPAMP_CSR_PGAGAIN_2VM1;
      break;
    case OPAMP_GAIN_4_VM1:
      regval |= OPAMP_CSR_PGAGAIN_4VM1;
      break;
    case OPAMP_GAIN_8_VM1:
      regval |= OPAMP_CSR_PGAGAIN_8VM1;
      break;
    case OPAMP_GAIN_16_VM1:
      regval |= OPAMP_CSR_PGAGAIN_16VM1;
      break;
    default:
      aerr("ERROR: Unsupported OPAMP gain\n");
      return -EINVAL;
    }

  /* Update gain in OPAMP device structure */

  priv->gain = gain;

  return OK;

}

#if 0
static int stm32_opampcalibrate(FAR struct stm32_opamp_s *priv)
{
#warning "Missing logic"
  return OK;
}
#endif

/****************************************************************************
 * Name: opamp_shutdown
 *
 * Description:
 *   Disable the OPAMP.  This method is called when the OPAMP device is closed.
 *   This method reverses the operation the setup method.
 *   Works only if OPAMP device is not locked.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void opamp_shutdown(FAR struct opamp_dev_s *dev)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: opamp_setup
 *
 * Description:
 *   Configure the OPAMP. This method is called the first time that the OPAMP
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching OPAMP interrupts.
 *   Interrupts are all disabled upon return.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int opamp_setup(FAR struct opamp_dev_s *dev)
{
#warning "Missing logic"
  return OK;
}

/****************************************************************************
 * Name: opamp_ioctl
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
 *
 ****************************************************************************/

static int opamp_ioctl(FAR struct opamp_dev_s* dev, int cmd, unsigned long arg)
{
#warning "Missing logic"
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_opampinitialize
 *
 * Description:
 *   Initialize the OPAMP.
 *
 * Input Parameters:
 *   intf - The OPAMP interface number.
 *
 * Returned Value:
 *   Valid OPAMP device structure reference on succcess; a NULL on failure.
 *
 * Assumptions:
 *   1. Clock to the OPAMP block has enabled,
 *   2. Board-specific logic has already configured
 *
 ****************************************************************************/

FAR struct opamp_dev_s* stm32_opampinitialize(int intf)
{
  FAR struct opamp_dev_s *dev;
  FAR struct stm32_opamp_s *opamp;
  int ret;

  switch (intf)
    {
#ifdef CONFIG_STM32_OPAMP1
    case 1:
      ainfo("OPAMP1 selected\n");
      dev = &g_opamp1dev;
      break;
#endif

#ifdef CONFIG_STM32_OPAMP2
    case 2:
      ainfo("OPAMP2 selected\n");
      dev = &g_opamp2dev;
      break;
#endif

#ifdef CONFIG_STM32_OPAMP3
    case 3:
      ainfo("OPAMP3 selected\n");
      dev = &g_opamp3dev;
      break;
#endif

#ifdef CONFIG_STM32_OPAMP4
    case 4:
      ainfo("OPAMP4 selected\n");
      dev = &g_opamp4dev;
      break;
#endif

    default:
      aerr("ERROR: No OPAMP interface defined\n");
      return NULL;
    }

  /* Configure selected OPAMP */

  opamp = dev->ad_priv;

  ret = stm32_opampconfig(opamp);
  if (ret < 0)
    {
      aerr("ERROR: Failed to initialize OPAMP%d: %d\n", intf, ret);
      return NULL;
    }

  return dev;
}

#endif  /* CONFIG_STM32_STM32F30XX || CONFIG_STM32_STM32F33XX*/

#endif  /* CONFIG_STM32_OPAMP1 || CONFIG_STM32_OPAMP2 ||
         * CONFIG_STM32_OPAMP3 || CONFIG_STM32_OPAMP4
         */

#endif /* CONFIG_STM32_OPAMP */
