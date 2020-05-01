/****************************************************************************
 * arch/arm/src/imxrt/imxrt_enc.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author:  Nicholas Chin <nicholaschin1995@gmail.com>
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/sensors/qencoder.h>
#include <nuttx/semaphore.h>

#include "chip.h"
#include "arm_arch.h"

#include "imxrt_periphclks.h"

#include "imxrt_enc.h"

/* This functionality is dependent on Qencoder Sensor support */

#ifndef CONFIG_SENSORS_QENCODER
#  undef CONFIG_IMXRT_ENC
#  error "Qencoder Sensor support is not enabled (CONFIG_SENSORS_QENCODER)"
#endif

#ifdef CONFIG_IMXRT_ENC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

/* Non-standard debug that may be used to enable the imxrt qe's built-in test
 * features
 */

#ifndef CONFIG_DEBUG_FEATURES
#  undef CONFIG_DEBUG_SENSORS
#endif

/* ENC1 Macros */

#if defined(CONFIG_IMXRT_ENC1)

#ifndef CONFIG_ENC1_DIR
#  define CONFIG_ENC1_DIR 0
#endif

#ifndef CONFIG_ENC1_HIP
#  define CONFIG_ENC1_HIP 0
#endif

#ifndef CONFIG_ENC1_HNE
# define CONFIG_ENC1_HNE 0
#endif

#ifndef CONFIG_ENC1_XIP
#  define CONFIG_ENC1_XIP 0
#endif

#ifndef CONFIG_ENC1_XNE
# define CONFIG_ENC1_XNE 0
#endif

#ifndef CONFIG_ENC1_MOD
#  define CONFIG_ENC1_MOD 0
#endif

#ifndef CONFIG_ENC1_MODULUS
# define CONFIG_ENC1_MODULUS 0
#endif

#if defined(CONFIG_DEBUG_SENSORS)

#ifndef CONFIG_ENC1_TST_DIR
#  define CONFIG_ENC1_TST_DIR 0
#endif

#endif /* CONFIG_DEBUG_SENSORS */
#endif /* CONFIG_IMXRT_ENC1 */

/* ENC2 Macros */

#if defined(CONFIG_IMXRT_ENC2)

#ifndef CONFIG_ENC2_DIR
#  define CONFIG_ENC2_DIR 0
#endif

#ifndef CONFIG_ENC2_HIP
#  define CONFIG_ENC2_HIP 0
#endif

#ifndef CONFIG_ENC2_HNE
# define CONFIG_ENC2_HNE 0
#endif

#ifndef CONFIG_ENC2_XIP
#  define CONFIG_ENC2_XIP 0
#endif

#ifndef CONFIG_ENC2_XNE
# define CONFIG_ENC2_XNE 0
#endif

#ifndef CONFIG_ENC2_MOD
#  define CONFIG_ENC2_MOD 0
#endif

#ifndef CONFIG_ENC2_MODULUS
# define CONFIG_ENC2_MODULUS 0
#endif

#if defined(CONFIG_DEBUG_SENSORS)

#ifndef CONFIG_ENC2_TST_DIR
#  define CONFIG_ENC2_TST_DIR 0
#endif

#endif /* CONFIG_DEBUG_SENSORS */
#endif /* CONFIG_IMXRT_ENC2 */

/* ENC3 Macros */

#if defined(CONFIG_IMXRT_ENC3)

#ifndef CONFIG_ENC3_DIR
#  define CONFIG_ENC3_DIR 0
#endif

#ifndef CONFIG_ENC3_HIP
#  define CONFIG_ENC3_HIP 0
#endif

#ifndef CONFIG_ENC3_HNE
# define CONFIG_ENC3_HNE 0
#endif

#ifndef CONFIG_ENC3_XIP
#  define CONFIG_ENC3_XIP 0
#endif

#ifndef CONFIG_ENC3_XNE
# define CONFIG_ENC3_XNE 0
#endif

#ifndef CONFIG_ENC3_MOD
#  define CONFIG_ENC3_MOD 0
#endif

#ifndef CONFIG_ENC3_MODULUS
# define CONFIG_ENC3_MODULUS 0
#endif

#if defined(CONFIG_DEBUG_SENSORS)

#ifndef CONFIG_ENC3_TST_DIR
#  define CONFIG_ENC3_TST_DIR 0
#endif

#endif /* CONFIG_DEBUG_SENSORS */
#endif /* CONFIG_IMXRT_ENC3 */

/* ENC4 Macros */

#if defined(CONFIG_IMXRT_ENC4)

#ifndef CONFIG_ENC4_DIR
#  define CONFIG_ENC4_DIR 0
#endif

#ifndef CONFIG_ENC4_HIP
#  define CONFIG_ENC4_HIP 0
#endif

#ifndef CONFIG_ENC4_HNE
# define CONFIG_ENC4_HNE 0
#endif

#ifndef CONFIG_ENC4_XIP
#  define CONFIG_ENC4_XIP 0
#endif

#ifndef CONFIG_ENC4_XNE
# define CONFIG_ENC4_XNE 0
#endif

#ifndef CONFIG_ENC4_MOD
#  define CONFIG_ENC4_MOD 0
#endif

#ifndef CONFIG_ENC4_MODULUS
# define CONFIG_ENC4_MODULUS 0
#endif

#if defined(CONFIG_DEBUG_SENSORS)

#ifndef CONFIG_ENC4_TST_DIR
#  define CONFIG_ENC4_TST_DIR 0
#endif

#endif /* CONFIG_DEBUG_SENSORS */
#endif /* CONFIG_IMXRT_ENC4 */

/* For extracting CTRL register flag values from config struct */

#define HIP_SHIFT (0)
#define HNE_SHIFT (1)
#define XIP_SHIFT (2)
#define XNE_SHIFT (3)
#define REV_SHIFT (4)
#define MOD_SHIFT (5)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Constant configuration structure that is retained in FLASH */

struct imxrt_qeconfig_s
{
  uint32_t  base;           /* Register base address */
  uint32_t  init_val;       /* Value to initialize position counters to */
  uint32_t  modulus;        /* Modulus to use when modulo counting is enabled */
  uint16_t  in_filt_per;    /* Period for input filter sampling in # of periph
                             * clock cycles
                             */
  uint16_t  in_filt_cnt;    /* # of consecutive input filter samples that must
                             * agree
                             */
  uint16_t  init_flags;     /* Flags to control which signals and edge transitions
                             * will reinitialize the position counter. Bits 4-0:
                             * [MOD, REV, XNE, XIP, HNE, HIP]
                             */

#ifdef CONFIG_DEBUG_SENSORS
  bool      tst_dir_adv;    /* Whether to generate down/up test signals */
  uint8_t   tst_period;     /* Period of PHASE pulses in # of periph clock cycles */
#endif
};

/* ENC Device Private Data */

struct imxrt_enc_lowerhalf_s
{
  /* The first field of this state structure must be a pointer to the lower-
   * half callback structure:
   */

  FAR const struct qe_ops_s *ops;             /* Lower half callback structure */

  /* IMXRT driver-specific fields: */

  FAR const struct imxrt_qeconfig_s *config;  /* static configuration */
  sem_t sem_excl;                             /* Mutual exclusion semaphore to
                                               * ensure atomic 32-bit reads.
                                               */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions */

static inline uint16_t imxrt_enc_getreg16
                        (FAR struct imxrt_enc_lowerhalf_s *priv, int offset);
static inline void imxrt_enc_putreg16(FAR struct imxrt_enc_lowerhalf_s *priv,
              int offset,  uint16_t value);
static inline void imxrt_enc_modifyreg16
                    (FAR struct imxrt_enc_lowerhalf_s *priv, int offset,
                    uint16_t clearbits, uint16_t setbits);

static void imxrt_enc_clock_enable (uint32_t base);
static void imxrt_enc_clock_disable (uint32_t base);

static inline int  imxrt_enc_sem_wait(
    FAR struct imxrt_enc_lowerhalf_s *priv);
static inline void imxrt_enc_sem_post(
    FAR struct imxrt_enc_lowerhalf_s *priv);

static int imxrt_enc_reconfig(FAR struct imxrt_enc_lowerhalf_s *priv,
              uint16_t args);
static void imxrt_enc_set_initial_val(FAR struct imxrt_enc_lowerhalf_s *priv,
              uint32_t value);
static void imxrt_enc_modulo_enable(FAR struct imxrt_enc_lowerhalf_s *priv,
              uint32_t modulus);
static void imxrt_enc_modulo_disable(FAR struct imxrt_enc_lowerhalf_s *priv);

#ifdef CONFIG_DEBUG_SENSORS
static int imxrt_enc_test_gen(FAR struct imxrt_enc_lowerhalf_s *priv,
              uint16_t value);
#endif

/* Lower-half Quadrature Encoder Driver Methods */

static int imxrt_setup(FAR struct qe_lowerhalf_s *lower);
static int imxrt_shutdown(FAR struct qe_lowerhalf_s *lower);
static int imxrt_position(FAR struct qe_lowerhalf_s *lower,
                          FAR int32_t *pos);
static int imxrt_reset(FAR struct qe_lowerhalf_s *lower);
static int imxrt_ioctl(FAR struct qe_lowerhalf_s *lower, int cmd,
              unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The lower half callback structure */

static const struct qe_ops_s g_qecallbacks =
{
  .setup    = imxrt_setup,
  .shutdown = imxrt_shutdown,
  .position = imxrt_position,
  .reset    = imxrt_reset,
  .ioctl    = imxrt_ioctl,
};

/* Per-timer state structures */

#ifdef CONFIG_IMXRT_ENC1
static const struct imxrt_qeconfig_s imxrt_enc1_config =
{
  .base        = IMXRT_ENC1_BASE,
  .init_val    = CONFIG_ENC1_INITVAL,
  .modulus     = CONFIG_ENC1_MODULUS,
  .in_filt_per = CONFIG_ENC1_FILTPER,
  .in_filt_cnt = CONFIG_ENC1_FILTCNT,
  .init_flags  = CONFIG_ENC1_HIP << HIP_SHIFT |
                 CONFIG_ENC1_HNE << HNE_SHIFT |
                 CONFIG_ENC1_XIP << XIP_SHIFT |
                 CONFIG_ENC1_XNE << XNE_SHIFT |
                 CONFIG_ENC1_DIR << REV_SHIFT |
                 CONFIG_ENC1_MOD << MOD_SHIFT,

#ifdef CONFIG_DEBUG_SENSORS
  .tst_dir_adv = CONFIG_ENC1_TST_DIR,
  .tst_period  = CONFIG_ENC1_TST_PER,
#endif
};

static struct imxrt_enc_lowerhalf_s imxrt_enc1_priv =
{
  .ops = &g_qecallbacks,
  .config = &imxrt_enc1_config,
};
#endif

#ifdef CONFIG_IMXRT_ENC2
static const struct imxrt_qeconfig_s imxrt_enc2_config =
{
  .base        = IMXRT_ENC2_BASE,
  .init_val    = CONFIG_ENC2_INITVAL,
  .modulus     = CONFIG_ENC2_MODULUS,
  .in_filt_per = CONFIG_ENC2_FILTPER,
  .in_filt_cnt = CONFIG_ENC2_FILTCNT,
  .init_flags  = CONFIG_ENC2_HIP << HIP_SHIFT |
                 CONFIG_ENC2_HNE << HNE_SHIFT |
                 CONFIG_ENC2_XIP << XIP_SHIFT |
                 CONFIG_ENC2_XNE << XNE_SHIFT |
                 CONFIG_ENC2_DIR << REV_SHIFT |
                 CONFIG_ENC2_MOD << MOD_SHIFT,

#ifdef CONFIG_DEBUG_SENSORS
  .tst_dir_adv = CONFIG_ENC2_TST_DIR,
  .tst_period  = CONFIG_ENC2_TST_PER,
#endif
};

static struct imxrt_enc_lowerhalf_s imxrt_enc2_priv =
{
  .ops    = &g_qecallbacks,
  .config = &imxrt_enc2_config,
};
#endif

#ifdef CONFIG_IMXRT_ENC3
static const struct imxrt_qeconfig_s imxrt_enc3_config =
{
  .base        = IMXRT_ENC3_BASE,
  .init_val    = CONFIG_ENC3_INITVAL,
  .modulus     = CONFIG_ENC3_MODULUS,
  .in_filt_per = CONFIG_ENC3_FILTPER,
  .in_filt_cnt = CONFIG_ENC3_FILTCNT,
  .init_flags  = CONFIG_ENC3_HIP << HIP_SHIFT |
                 CONFIG_ENC3_HNE << HNE_SHIFT |
                 CONFIG_ENC3_XIP << XIP_SHIFT |
                 CONFIG_ENC3_XNE << XNE_SHIFT |
                 CONFIG_ENC3_DIR << REV_SHIFT |
                 CONFIG_ENC3_MOD << MOD_SHIFT,

#ifdef CONFIG_DEBUG_SENSORS
  .tst_dir_adv = CONFIG_ENC3_TST_DIR,
  .tst_period  = CONFIG_ENC3_TST_PER,
#endif
};

static struct imxrt_enc_lowerhalf_s imxrt_enc3_priv =
{
  .ops    = &g_qecallbacks,
  .config = &imxrt_enc3_config,
};
#endif

#ifdef CONFIG_IMXRT_ENC4
static const struct imxrt_qeconfig_s imxrt_enc4_config =
{
  .base        = IMXRT_ENC4_BASE,
  .init_val    = CONFIG_ENC4_INITVAL,
  .modulus     = CONFIG_ENC4_MODULUS,
  .in_filt_per = CONFIG_ENC4_FILTPER,
  .in_filt_cnt = CONFIG_ENC4_FILTCNT,
  .init_flags  = CONFIG_ENC4_HIP << HIP_SHIFT |
                 CONFIG_ENC4_HNE << HNE_SHIFT |
                 CONFIG_ENC4_XIP << XIP_SHIFT |
                 CONFIG_ENC4_XNE << XNE_SHIFT |
                 CONFIG_ENC4_DIR << REV_SHIFT |
                 CONFIG_ENC4_MOD << MOD_SHIFT,

#ifdef CONFIG_DEBUG_SENSORS
  .tst_dir_adv = CONFIG_ENC4_TST_DIR,
  .tst_period  = CONFIG_ENC4_TST_PER,
#endif
};

static struct imxrt_enc_lowerhalf_s imxrt_enc4_priv =
{
  .ops    = &g_qecallbacks,
  .config = &imxrt_enc4_config,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_enc_getreg16
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ****************************************************************************/

static inline uint16_t imxrt_enc_getreg16
                        (FAR struct imxrt_enc_lowerhalf_s *priv, int offset)
{
  return getreg16(priv->config->base + offset);
}

/****************************************************************************
 * Name: imxrt_enc_putreg16
 *
 * Description:
 *  Put a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void imxrt_enc_putreg16(FAR struct imxrt_enc_lowerhalf_s *priv,
                                      int offset, uint16_t value)
{
  putreg16(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: imxrt_enc_modifyreg16
 *
 * Description:
 *   Modify a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void imxrt_enc_modifyreg16
                    (FAR struct imxrt_enc_lowerhalf_s *priv, int offset,
                    uint16_t clearbits, uint16_t setbits)
{
  modifyreg16(priv->config->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: imxrt_enc_clock_enable
 *
 * Description:
 *   Ungate ENC clock
 *
 ****************************************************************************/

void imxrt_enc_clock_enable (uint32_t base)
{
  if (base == IMXRT_ENC1_BASE)
    {
      imxrt_clockall_enc1();
    }
  else if (base == IMXRT_ENC2_BASE)
    {
      imxrt_clockall_enc2();
    }

#if (defined(CONFIG_ARCH_FAMILY_IMXRT105x) || \
     defined(CONFIG_ARCH_FAMILY_IMXRT106x))
  else if (base == IMXRT_ENC3_BASE)
    {
      imxrt_clockall_enc3();
    }
  else if (base == IMXRT_ENC4_BASE)
    {
      imxrt_clockall_enc4();
    }
#endif /* CONFIG_ARCH_FAMILY_IMXRT105x || CONFIG_ARCH_FAMILY_IMXRT106x */
}

/****************************************************************************
 * Name: imxrt_enc_clock_disable
 *
 * Description:
 *   Gate ENC clock
 *
 ****************************************************************************/

void imxrt_enc_clock_disable (uint32_t base)
{
  if (base == IMXRT_ENC1_BASE)
    {
      imxrt_clockoff_enc1();
    }
  else if (base == IMXRT_ENC2_BASE)
    {
      imxrt_clockoff_enc2();
    }

#if (defined(CONFIG_ARCH_FAMILY_IMXRT105x) || \
     defined(CONFIG_ARCH_FAMILY_IMXRT106x))
  else if (base == IMXRT_ENC3_BASE)
    {
      imxrt_clockoff_enc3();
    }
  else if (base == IMXRT_ENC4_BASE)
    {
      imxrt_clockoff_enc4();
    }
#endif /* CONFIG_ARCH_FAMILY_IMXRT105x || CONFIG_ARCH_FAMILY_IMXRT106x */
}

/****************************************************************************
 * Name: imxrt_enc_sem_wait
 *
 * Description:
 *   Take exclusive access to the position register, waiting as necessary
 *
 ****************************************************************************/

static inline int imxrt_enc_sem_wait(FAR struct imxrt_enc_lowerhalf_s *priv)
{
  return nxsem_wait_uninterruptible(&priv->sem_excl);
}

/****************************************************************************
 * Name: imxrt_enc_sem_post
 *
 * Description:
 *   Release the mutual exclusion semaphore
 *
 ****************************************************************************/

static inline void imxrt_enc_sem_post(struct imxrt_enc_lowerhalf_s *priv)
{
  nxsem_post(&priv->sem_excl);
}

/****************************************************************************
 * Name: imxrt_enc_reconfig
 *
 * Description:
 *   Sets control bits in the IMXRT quadrature encoder peripheral.
 *
 * Input Parameters:
 *   priv - A reference to the IMXRT enc lowerhalf structure
 *   args - Bit encoded control parameters. As Follows:
 *              Bits 8-7: Reserved
 *              Bits 6-0: [OUTCTL, REVMOD, REV, XNE, XIP, HNE, HIP]
 *
 * Returns: 0 on success. Negated errno on failure.
 *
 ****************************************************************************/

static int imxrt_enc_reconfig(FAR struct imxrt_enc_lowerhalf_s *priv,
                                uint16_t args)
{
  uint16_t clear = 0;
  uint16_t set = 0;

  if (args & (2 << 7))
    {
      return -EINVAL;
    }

  if ((args >> HIP_SHIFT) & 1)
    {
      set |= ENC_CTRL_HIP;
    }
  else
    {
      clear |= ENC_CTRL_HIP;
    }

  if ((args >> HNE_SHIFT) & 1)
    {
      set |= ENC_CTRL_HNE;
    }
  else
    {
      clear |= ENC_CTRL_HNE;
    }

  if ((args >> XIP_SHIFT) & 1)
    {
      set |= ENC_CTRL_XIP;
    }
  else
    {
      clear |= ENC_CTRL_XIP;
    }

  if ((args >> XNE_SHIFT) & 1)
    {
      set |= ENC_CTRL_XNE;
    }
  else
    {
      clear |= ENC_CTRL_XNE;
    }

  if ((args >> REV_SHIFT) & 1)
    {
      set |= ENC_CTRL_REV;
    }
  else
    {
      clear |= ENC_CTRL_REV;
    }

  imxrt_enc_modifyreg16(priv, IMXRT_ENC_CTRL_OFFSET, clear, set);

  clear = 0;
  set = 0;

  if ((args >> 5) & 1)
    {
      set |= ENC_CTRL2_REVMOD;
    }
  else
    {
      clear |= ENC_CTRL2_REVMOD;
    }

  if ((args >> 6) & 1)
    {
      set |= ENC_CTRL2_OUTCTL;
    }
  else
    {
      clear |= ENC_CTRL2_OUTCTL;
    }

  imxrt_enc_modifyreg16(priv, IMXRT_ENC_CTRL2_OFFSET, clear, set);

  return OK;
}

/****************************************************************************
 * Name: imxrt_enc_set_initial_val
 *
 * Description:
 *   Sets the value of UINIT and LINIT registers.
 *
 * Input Parameters:
 *   priv - A reference to the IMXRT enc lower-half structure
 *   value - New initial value that the position counters will take upon
 *           reset or roll-over.
 *
 ****************************************************************************/

static void imxrt_enc_set_initial_val(FAR struct imxrt_enc_lowerhalf_s *priv,
                                      uint32_t value)
{
  imxrt_enc_putreg16(priv, IMXRT_ENC_LINIT_OFFSET, value & 0xffff);
  imxrt_enc_putreg16(priv, IMXRT_ENC_UINIT_OFFSET, (value >> 16) & 0xffff);
}

/****************************************************************************
 * Name: imxrt_enc_modulo_enable
 *
 * Description:
 *   Enables modulo counting and sets the modulus for counting.
 *
 * Input Parameters:
 *   priv - A reference to the IMXRT enc lower-half structure
 *   modulus - The maximum position counter value before roll-over.
 *
 ****************************************************************************/

static void imxrt_enc_modulo_enable(FAR struct imxrt_enc_lowerhalf_s *priv,
                                    uint32_t modulus)
{
  imxrt_enc_putreg16(priv, IMXRT_ENC_LMOD_OFFSET, modulus & 0xffff);
  imxrt_enc_putreg16(priv, IMXRT_ENC_UMOD_OFFSET, (modulus >> 16) & 0xffff);

  imxrt_enc_modifyreg16(priv, IMXRT_ENC_CTRL2_OFFSET, 0, ENC_CTRL2_MOD);
}

/****************************************************************************
 * Name: imxrt_enc_modulo_disable
 *
 * Description:
 *   Disables modulo counting.
 *
 * Input Parameters:
 *   priv - A reference to the IMXRT enc lowerhalf structure
 *
 ****************************************************************************/

static void imxrt_enc_modulo_disable(FAR struct imxrt_enc_lowerhalf_s *priv)
{
  imxrt_enc_modifyreg16(priv, IMXRT_ENC_CTRL2_OFFSET, ENC_CTRL2_MOD, 0);
}

#ifdef CONFIG_DEBUG_SENSORS

/****************************************************************************
 * Name: imxrt_enc_test_gen
 *
 * Description:
 *   Generates PHASEA and PHASEB pulses to test the peripheral.
 *
 * Input Parameters:
 *   priv - A reference to the IMXRT enc lowerhalf structure
 *   value - Bit encoded variable to indicate how many pulse advances to
 *           generate and which direction to generate them.
 *              Bits 15-9: Reserved.
 *              Bit 8: QDN. Generate negative/ positive advances.
 *              Bits 7-0: TEST_COUNT. Number of advances to generate.
 *
 * Returns: 0 on success. Negated errno on failure.
 *
 ****************************************************************************/

static int imxrt_enc_test_gen(FAR struct imxrt_enc_lowerhalf_s *priv,
                                uint16_t value)
{
  if (value >> 9)
    {
      return -EINVAL;
    }

  if (value & (1 << 8))
    {
      imxrt_enc_modifyreg16(priv, IMXRT_ENC_TST_OFFSET, 0, ENC_TST_QDN);
    }
  else
    {
      imxrt_enc_modifyreg16(priv, IMXRT_ENC_TST_OFFSET, ENC_TST_QDN, 0);
    }

  imxrt_enc_modifyreg16(priv, IMXRT_ENC_TST_OFFSET, 0,
                        (value & ENC_TST_COUNT_MASK) << ENC_TST_COUNT_SHIFT);

  return OK;
}

#endif /* CONFIG_DEBUG_SENSORS */

/****************************************************************************
 * Device Driver Operations
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   The initial position value is set to the user-specified INIT register
 *   values.
 *
 ****************************************************************************/

static int imxrt_setup(FAR struct qe_lowerhalf_s *lower)
{
  FAR struct imxrt_enc_lowerhalf_s *priv =
    (FAR struct imxrt_enc_lowerhalf_s *)lower;
  FAR const struct imxrt_qeconfig_s *config = priv->config;
  uint32_t regval;
  int ret;

  ret = imxrt_enc_sem_wait(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Ungate the clock */

  imxrt_enc_clock_enable(config->base);

  /* Initial value registers */

  imxrt_enc_putreg16(priv, IMXRT_ENC_LINIT_OFFSET,
                     config->init_val & 0xffff);
  imxrt_enc_putreg16(priv, IMXRT_ENC_UINIT_OFFSET,
                        (config->init_val >> 16) & 0xffff);

  /* Modulus registers */

  imxrt_enc_putreg16(priv, IMXRT_ENC_LMOD_OFFSET, config->modulus & 0xffff);
  imxrt_enc_putreg16(priv, IMXRT_ENC_UMOD_OFFSET,
                        (config->modulus >> 16) & 0xffff);

  /* Input Filter registers */

  regval = (config->in_filt_per  & ENC_FILT_PER_MASK) << ENC_FILT_PER_SHIFT |
           (config->in_filt_cnt & ENC_FILT_CNT_MASK) << ENC_FILT_CNT_SHIFT;
  imxrt_enc_putreg16(priv, IMXRT_ENC_FILT_OFFSET, regval);

  /* Test Registers */

#ifdef CONFIG_DEBUG_SENSORS
  regval = ENC_TST_TCE | ENC_TST_TEN;
  regval |= config->tst_dir_adv ? ENC_TST_QDN : 0;
  regval |= (config->tst_period & ENC_TST_PERIOD_MASK) <<
            ENC_TST_PERIOD_SHIFT;
  imxrt_enc_putreg16(priv, IMXRT_ENC_TST_OFFSET, regval);
#endif

  /* Control and Control 2 register */

  regval = ENC_CTRL_SWIP;
  regval |= ((config->init_flags >> REV_SHIFT) & 1) ? ENC_CTRL_REV : 0;
  regval |= ((config->init_flags >> HIP_SHIFT) & 1) ? ENC_CTRL_HIP : 0;
  regval |= ((config->init_flags >> HNE_SHIFT) & 1) ? ENC_CTRL_HNE : 0;
  regval |= ((config->init_flags >> XIP_SHIFT) & 1) ? ENC_CTRL_XIP : 0;
  regval |= ((config->init_flags >> XNE_SHIFT) & 1) ? ENC_CTRL_XNE : 0;
  imxrt_enc_putreg16(priv, IMXRT_ENC_CTRL_OFFSET, regval);

  regval = ((config->init_flags >> MOD_SHIFT) & 1) ? ENC_CTRL2_MOD : 0;
  imxrt_enc_putreg16(priv, IMXRT_ENC_CTRL2_OFFSET, regval);

  imxrt_enc_sem_post(priv);
  return OK;
}

/****************************************************************************
 * Name: imxrt_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   should stop data collection, and put the system into the lowest possible
 *   power usage state
 *
 ****************************************************************************/

static int imxrt_shutdown(FAR struct qe_lowerhalf_s *lower)
{
  FAR struct imxrt_enc_lowerhalf_s *priv =
    (FAR struct imxrt_enc_lowerhalf_s *)lower;
  int ret;

  /* Ensure any in-progress operations are done. */

  ret = imxrt_enc_sem_wait(priv);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_DEBUG_SENSORS
  imxrt_enc_putreg16(priv, IMXRT_ENC_TST_OFFSET, 0);
#endif

  imxrt_enc_putreg16(priv, IMXRT_ENC_FILT_OFFSET, 0);
  imxrt_enc_putreg16(priv, IMXRT_ENC_LINIT_OFFSET, 0);
  imxrt_enc_putreg16(priv, IMXRT_ENC_UINIT_OFFSET, 0);
  imxrt_enc_putreg16(priv, IMXRT_ENC_REV_OFFSET, 0);
  imxrt_enc_putreg16(priv, IMXRT_ENC_POSD_OFFSET, 0);

  /* Write to only SWIP to blank CTRL and also reinit POS to 0 */

  imxrt_enc_putreg16(priv, IMXRT_ENC_CTRL_OFFSET, ENC_CTRL_SWIP);

  /* Gate the clock */

  imxrt_enc_clock_disable(priv->config->base);

  imxrt_enc_sem_post(priv);
  return OK;
}

/****************************************************************************
 * Name: imxrt_position
 *
 * Description:
 *   Return the current position measurement.
 *
 ****************************************************************************/

static int imxrt_position(FAR struct qe_lowerhalf_s *lower, FAR int32_t *pos)
{
  FAR struct imxrt_enc_lowerhalf_s *priv =
    (FAR struct imxrt_enc_lowerhalf_s *)lower;
  uint16_t lpos;
  uint16_t upos;
  int i;
  int ret;

  ret = imxrt_enc_sem_wait(priv);
  if (ret < 0)
    {
      return ret;
    }

  lpos = imxrt_enc_getreg16(priv, IMXRT_ENC_LPOS_OFFSET);

  /**************************************************************************
   * When a position register is read, it triggers a snapshot into the
   * position hold registers.
   *
   * If the core clock is faster than the peripheral clock, we might need to
   * wait until the snapshot registers latch properly. There is no interrupt
   * to signal that the snapshot is done, so we have to poll LPOSH until it
   * equals our reading. We will poll for at most, 2 peripheral clock cycles.
   * Since IPG_PODF max is 4, at most  we'll poll 8 core clock cycles.
   **************************************************************************/

  for (i = 8;
       lpos != imxrt_enc_getreg16(priv, IMXRT_ENC_LPOSH_OFFSET) && i > 0;
       i--)
    {
    }

  if (lpos != imxrt_enc_getreg16(priv, IMXRT_ENC_LPOSH_OFFSET))
    {
      imxrt_enc_sem_post(priv);
      return -EAGAIN;
    }

  upos = imxrt_enc_getreg16(priv, IMXRT_ENC_UPOSH_OFFSET);

  imxrt_enc_sem_post(priv);

  *pos = (int32_t)((upos << 16) | lpos);
  return OK;
}

/****************************************************************************
 * Name: imxrt_reset
 *
 * Description:
 *   Reset the position measurement to the value of the INIT registers.
 *
 ****************************************************************************/

static int imxrt_reset(FAR struct qe_lowerhalf_s *lower)
{
  FAR struct imxrt_enc_lowerhalf_s *priv =
    (FAR struct imxrt_enc_lowerhalf_s *)lower;
  int ret;

  /* Write a 1 to the SWIP bit to load UINIT and LINIT into UPOS and LPOS */

  ret = imxrt_enc_sem_wait(priv);
  if (ret < 0)
    {
      return ret;
    }

  imxrt_enc_modifyreg16(priv, IMXRT_ENC_CTRL_OFFSET, 0, ENC_CTRL_SWIP);
  imxrt_enc_sem_post(priv);

  return OK;
}

/****************************************************************************
 * Name: imxrt_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 ****************************************************************************/

static int imxrt_ioctl(FAR struct qe_lowerhalf_s *lower, int cmd,
              unsigned long arg)
{
  struct imxrt_enc_lowerhalf_s *priv = (struct imxrt_enc_lowerhalf_s *)lower;
  switch (cmd)
    {
      /* QEIOC_POSDIFF: returns the content of the Position Difference register */

      case QEIOC_POSDIFF:
        *((uint16_t *)arg) = imxrt_enc_getreg16(priv, IMXRT_ENC_POSD_OFFSET);
        break;

      /* QEIOC_REVOLUTION: returns the content of the Position Difference register */

      case QEIOC_REVOLUTION:
        *((uint16_t *)arg) = imxrt_enc_getreg16(priv, IMXRT_ENC_REV_OFFSET);
        break;
      case QEIOC_RECONFIG:
        return imxrt_enc_reconfig(priv, (uint16_t)arg);
      case QEIOC_INITTO:
        imxrt_enc_set_initial_val(priv, (uint32_t)arg);
        break;
      case QEIOC_RESETAT:
        imxrt_enc_modulo_enable(priv, (uint32_t)arg);
        break;
      case QEIOC_RESETATMAX:
        imxrt_enc_modulo_disable(priv);
        break;

#ifdef CONFIG_DEBUG_SENSORS
      case QEIOC_TEST_GEN:
        return imxrt_enc_test_gen(priv, (uint16_t)arg);
#endif /* CONFIG_DEBUG_SENSORS */

      default:
        return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_qeinitialize
 *
 * Description:
 *   Initialize a quadrature encoder interface.  This function must be called
 *   from board-specific logic..
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/qe0"
 *   enc     - The encoder peripheral to use.  'enc' must be an element of
 *             {1,2,3,4}
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

int imxrt_qeinitialize(FAR const char *devpath, int enc)
{
  struct imxrt_enc_lowerhalf_s * priv = NULL;

  switch (enc)
    {
#ifdef CONFIG_IMXRT_ENC1
    case 1:
      priv = (struct imxrt_enc_lowerhalf_s *)&imxrt_enc1_priv;
      break;
#endif
#ifdef CONFIG_IMXRT_ENC2
    case 2:
      priv = (struct imxrt_enc_lowerhalf_s *)&imxrt_enc2_priv;
      break;
#endif
#ifdef CONFIG_IMXRT_ENC3
    case 3:
      priv = (struct imxrt_enc_lowerhalf_s *)&imxrt_enc3_priv;
      break;
#endif
#ifdef CONFIG_IMXRT_ENC4
    case 4:
      priv = (struct imxrt_enc_lowerhalf_s *)&imxrt_enc4_priv;
      break;
#endif
    default:
      return -ENODEV;
    }

  /* Initialize private data */

  nxsem_init(&priv->sem_excl, 0, 1);

  /* Register the upper-half driver */

  int ret = qe_register(devpath, (FAR struct qe_lowerhalf_s *)priv);
  if (ret < 0)
    {
      snerr("ERROR: qe_register failed: %d\n", ret);
    }

  return ret;
}

#endif /* CONFIG_IMXRT_ENC */
