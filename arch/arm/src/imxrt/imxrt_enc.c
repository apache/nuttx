/****************************************************************************
 * arch/arm/src/imxrt/imxrt_enc.c
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

#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/sensors/qencoder.h>
#include <nuttx/mutex.h>

#include "chip.h"
#include "arm_internal.h"
#include "imxrt_periphclks.h"

#include "imxrt_enc.h"
#include "hardware/imxrt_enc.h"

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
#  define CONFIG_ENC1_HNE 0
#endif

#ifndef CONFIG_ENC1_XIE
#  define CONFIG_ENC1_XIE 0
#endif

#ifndef CONFIG_ENC1_XIP
#  define CONFIG_ENC1_XIP 0
#endif

#ifndef CONFIG_ENC1_XNE
#  define CONFIG_ENC1_XNE 0
#endif

#ifndef CONFIG_ENC1_MOD
#  define CONFIG_ENC1_MOD 0
#endif

#ifndef CONFIG_ENC1_MODULUS
#  define CONFIG_ENC1_MODULUS 0
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
#  define CONFIG_ENC2_HNE 0
#endif

#ifndef CONFIG_ENC2_XIE
#  define CONFIG_ENC2_XIE 0
#endif

#ifndef CONFIG_ENC2_XIP
#  define CONFIG_ENC2_XIP 0
#endif

#ifndef CONFIG_ENC2_XNE
#  define CONFIG_ENC2_XNE 0
#endif

#ifndef CONFIG_ENC2_MOD
#  define CONFIG_ENC2_MOD 0
#endif

#ifndef CONFIG_ENC2_MODULUS
#  define CONFIG_ENC2_MODULUS 0
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
#  define CONFIG_ENC3_HNE 0
#endif

#ifndef CONFIG_ENC3_XIE
#  define CONFIG_ENC3_XIE 0
#endif

#ifndef CONFIG_ENC3_XIP
#  define CONFIG_ENC3_XIP 0
#endif

#ifndef CONFIG_ENC3_XNE
#  define CONFIG_ENC3_XNE 0
#endif

#ifndef CONFIG_ENC3_MOD
#  define CONFIG_ENC3_MOD 0
#endif

#ifndef CONFIG_ENC3_MODULUS
#  define CONFIG_ENC3_MODULUS 0
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
#  define CONFIG_ENC4_HNE 0
#endif

#ifndef CONFIG_ENC4_XIE
#  define CONFIG_ENC4_XIE 0
#endif

#ifndef CONFIG_ENC4_XIP
#  define CONFIG_ENC4_XIP 0
#endif

#ifndef CONFIG_ENC4_XNE
#  define CONFIG_ENC4_XNE 0
#endif

#ifndef CONFIG_ENC4_MOD
#  define CONFIG_ENC4_MOD 0
#endif

#ifndef CONFIG_ENC4_MODULUS
#  define CONFIG_ENC4_MODULUS 0
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
#define XIE_SHIFT (6)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Constant configuration structure that is retained in FLASH */

struct imxrt_qeconfig_s
{
  uint32_t  base;           /* Register base address */
  uint32_t  irq;            /* Encoder interrupt */
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

struct imxrt_qedata_s
{
  int32_t  index_pos;      /* Last position of index occurance */
  uint32_t index_cnt;      /* Number of index occurance */
};

/* ENC Device Private Data */

struct imxrt_enc_lowerhalf_s
{
  /* The first field of this state structure must be a pointer to the lower-
   * half callback structure:
   */

  const struct qe_ops_s *ops;             /* Lower half callback structure */

  /* IMXRT driver-specific fields: */

  const struct imxrt_qeconfig_s *config;  /* static configuration */
  struct qe_index_s *data;
  mutex_t lock;                           /* Mutual exclusion mutex to
                                           * ensure atomic 32-bit reads.
                                           */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions */

static inline uint16_t imxrt_enc_getreg16
                        (struct imxrt_enc_lowerhalf_s *priv, int offset);
static inline void imxrt_enc_putreg16(struct imxrt_enc_lowerhalf_s *priv,
                                      int offset,  uint16_t value);
static inline void imxrt_enc_modifyreg16
                    (struct imxrt_enc_lowerhalf_s *priv, int offset,
                    uint16_t clearbits, uint16_t setbits);

static void imxrt_enc_clock_enable (uint32_t base);
static void imxrt_enc_clock_disable (uint32_t base);

static int imxrt_enc_reconfig(struct imxrt_enc_lowerhalf_s *priv,
                              uint16_t args);
static void imxrt_enc_set_initial_val(struct imxrt_enc_lowerhalf_s *priv,
                                      uint32_t value);
static void imxrt_enc_modulo_enable(struct imxrt_enc_lowerhalf_s *priv,
                                    uint32_t modulus);
static void imxrt_enc_modulo_disable(struct imxrt_enc_lowerhalf_s *priv);

static int imxrt_enc_index(int irq, void *context, void *arg);

#ifdef CONFIG_DEBUG_SENSORS
static int imxrt_enc_test_gen(struct imxrt_enc_lowerhalf_s *priv,
                              uint16_t value);
#endif

/* Lower-half Quadrature Encoder Driver Methods */

static int imxrt_setup(struct qe_lowerhalf_s *lower);
static int imxrt_shutdown(struct qe_lowerhalf_s *lower);
static int imxrt_position(struct qe_lowerhalf_s *lower,
                          int32_t *pos);
static int imxrt_reset(struct qe_lowerhalf_s *lower);
static int imxrt_ioctl(struct qe_lowerhalf_s *lower, int cmd,
                       unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The lower half callback structure */

static const struct qe_ops_s g_qecallbacks =
{
  .setup     = imxrt_setup,
  .shutdown  = imxrt_shutdown,
  .position  = imxrt_position,
  .setposmax = NULL,            /* not supported yet */
  .reset     = imxrt_reset,
  .setindex  = NULL,            /* not supported yet */
  .ioctl     = imxrt_ioctl,
};

/* Per-timer state structures */

#ifdef CONFIG_IMXRT_ENC1
static const struct imxrt_qeconfig_s imxrt_enc1_config =
{
  .base        = IMXRT_ENC1_BASE,
  .irq         = IMXRT_IRQ_ENC1,
  .init_val    = CONFIG_ENC1_INITVAL,
  .modulus     = CONFIG_ENC1_MODULUS,
  .in_filt_per = CONFIG_ENC1_FILTPER,
  .in_filt_cnt = CONFIG_ENC1_FILTCNT,
  .init_flags  = CONFIG_ENC1_HIP << HIP_SHIFT |
                 CONFIG_ENC1_HNE << HNE_SHIFT |
                 CONFIG_ENC1_XIE << XIE_SHIFT |
                 CONFIG_ENC1_XIP << XIP_SHIFT |
                 CONFIG_ENC1_XNE << XNE_SHIFT |
                 CONFIG_ENC1_DIR << REV_SHIFT |
                 CONFIG_ENC1_MOD << MOD_SHIFT,

#ifdef CONFIG_DEBUG_SENSORS
  .tst_dir_adv = CONFIG_ENC1_TST_DIR,
  .tst_period  = CONFIG_ENC1_TST_PER,
#endif
};

static struct qe_index_s imxrt_enc1_data =
{
  .qenc_pos = 0,
  .indx_pos = 0,
  .indx_cnt = 0,
};

static struct imxrt_enc_lowerhalf_s imxrt_enc1_priv =
{
  .ops    = &g_qecallbacks,
  .config = &imxrt_enc1_config,
  .data   = &imxrt_enc1_data,
  .lock   = NXMUTEX_INITIALIZER,
};
#endif

#ifdef CONFIG_IMXRT_ENC2
static const struct imxrt_qeconfig_s imxrt_enc2_config =
{
  .base        = IMXRT_ENC2_BASE,
  .irq         = IMXRT_IRQ_ENC2,
  .init_val    = CONFIG_ENC2_INITVAL,
  .modulus     = CONFIG_ENC2_MODULUS,
  .in_filt_per = CONFIG_ENC2_FILTPER,
  .in_filt_cnt = CONFIG_ENC2_FILTCNT,
  .init_flags  = CONFIG_ENC2_HIP << HIP_SHIFT |
                 CONFIG_ENC2_HNE << HNE_SHIFT |
                 CONFIG_ENC2_XIE << XIE_SHIFT |
                 CONFIG_ENC2_XIP << XIP_SHIFT |
                 CONFIG_ENC2_XNE << XNE_SHIFT |
                 CONFIG_ENC2_DIR << REV_SHIFT |
                 CONFIG_ENC2_MOD << MOD_SHIFT,

#ifdef CONFIG_DEBUG_SENSORS
  .tst_dir_adv = CONFIG_ENC2_TST_DIR,
  .tst_period  = CONFIG_ENC2_TST_PER,
#endif
};

static struct qe_index_s imxrt_enc2_data =
{
  .qenc_pos = 0,
  .indx_pos = 0,
  .indx_cnt = 0,
};

static struct imxrt_enc_lowerhalf_s imxrt_enc2_priv =
{
  .ops    = &g_qecallbacks,
  .config = &imxrt_enc2_config,
  .data   = &imxrt_enc2_data,
  .lock   = NXMUTEX_INITIALIZER,
};
#endif

#ifdef CONFIG_IMXRT_ENC3
static const struct imxrt_qeconfig_s imxrt_enc3_config =
{
  .base        = IMXRT_ENC3_BASE,
  .irq         = IMXRT_IRQ_ENC3,
  .init_val    = CONFIG_ENC3_INITVAL,
  .modulus     = CONFIG_ENC3_MODULUS,
  .in_filt_per = CONFIG_ENC3_FILTPER,
  .in_filt_cnt = CONFIG_ENC3_FILTCNT,
  .init_flags  = CONFIG_ENC3_HIP << HIP_SHIFT |
                 CONFIG_ENC3_HNE << HNE_SHIFT |
                 CONFIG_ENC3_XIE << XIE_SHIFT |
                 CONFIG_ENC3_XIP << XIP_SHIFT |
                 CONFIG_ENC3_XNE << XNE_SHIFT |
                 CONFIG_ENC3_DIR << REV_SHIFT |
                 CONFIG_ENC3_MOD << MOD_SHIFT,

#ifdef CONFIG_DEBUG_SENSORS
  .tst_dir_adv = CONFIG_ENC3_TST_DIR,
  .tst_period  = CONFIG_ENC3_TST_PER,
#endif
};

static struct qe_index_s imxrt_enc3_data =
{
  .qenc_pos = 0,
  .indx_pos = 0,
  .indx_cnt = 0,
};

static struct imxrt_enc_lowerhalf_s imxrt_enc3_priv =
{
  .ops    = &g_qecallbacks,
  .config = &imxrt_enc3_config,
  .data   = &imxrt_enc3_data,
  .lock   = NXMUTEX_INITIALIZER,
};
#endif

#ifdef CONFIG_IMXRT_ENC4
static const struct imxrt_qeconfig_s imxrt_enc4_config =
{
  .base        = IMXRT_ENC4_BASE,
  .irq         = IMXRT_IRQ_ENC4,
  .init_val    = CONFIG_ENC4_INITVAL,
  .modulus     = CONFIG_ENC4_MODULUS,
  .in_filt_per = CONFIG_ENC4_FILTPER,
  .in_filt_cnt = CONFIG_ENC4_FILTCNT,
  .init_flags  = CONFIG_ENC4_HIP << HIP_SHIFT |
                 CONFIG_ENC4_HNE << HNE_SHIFT |
                 CONFIG_ENC4_XIE << XIE_SHIFT |
                 CONFIG_ENC4_XIP << XIP_SHIFT |
                 CONFIG_ENC4_XNE << XNE_SHIFT |
                 CONFIG_ENC4_DIR << REV_SHIFT |
                 CONFIG_ENC4_MOD << MOD_SHIFT,

#ifdef CONFIG_DEBUG_SENSORS
  .tst_dir_adv = CONFIG_ENC4_TST_DIR,
  .tst_period  = CONFIG_ENC4_TST_PER,
#endif
};

static struct qe_index_s imxrt_enc4_data =
{
  .qenc_pos = 0,
  .indx_pos = 0,
  .indx_cnt = 0,
};

static struct imxrt_enc_lowerhalf_s imxrt_enc4_priv =
{
  .ops    = &g_qecallbacks,
  .config = &imxrt_enc4_config,
  .data   = &imxrt_enc4_data,
  .lock   = NXMUTEX_INITIALIZER,
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
                        (struct imxrt_enc_lowerhalf_s *priv, int offset)
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

static inline void imxrt_enc_putreg16(struct imxrt_enc_lowerhalf_s *priv,
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
                    (struct imxrt_enc_lowerhalf_s *priv, int offset,
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

void imxrt_enc_clock_enable(uint32_t base)
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

void imxrt_enc_clock_disable(uint32_t base)
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

static int imxrt_enc_reconfig(struct imxrt_enc_lowerhalf_s *priv,
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

  if ((args >> XIE_SHIFT) & 1)
    {
      set |= ENC_CTRL_XIE;
    }
  else
    {
      clear |= ENC_CTRL_XIE;
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

static void imxrt_enc_set_initial_val(struct imxrt_enc_lowerhalf_s *priv,
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

static void imxrt_enc_modulo_enable(struct imxrt_enc_lowerhalf_s *priv,
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

static void imxrt_enc_modulo_disable(struct imxrt_enc_lowerhalf_s *priv)
{
  imxrt_enc_modifyreg16(priv, IMXRT_ENC_CTRL2_OFFSET, ENC_CTRL2_MOD, 0);
}

/****************************************************************************
 * Name: imxrt_enc_index
 *
 * Description:
 *   Get the index position and increments index count.
 *
 ****************************************************************************/

static int imxrt_enc_index(int irq, void *context, void *arg)
{
  struct imxrt_enc_lowerhalf_s *priv =
    (struct imxrt_enc_lowerhalf_s *)arg;
  const struct imxrt_qeconfig_s *config = priv->config;
  struct qe_index_s *data = priv->data;
  uint16_t regval = getreg16(config->base + IMXRT_ENC_CTRL_OFFSET);

  if ((regval & ENC_CTRL_XIRQ) != 0)
    {
      /* Clear the interrupt */

      regval |= ENC_CTRL_XIRQ;
      putreg16(regval, config->base + IMXRT_ENC_CTRL_OFFSET);

      /* Get index position */

      imxrt_position(arg, &data->indx_pos);

      /* Increment index count */

      priv->data->indx_cnt += 1;
    }

  return OK;
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

static int imxrt_enc_test_gen(struct imxrt_enc_lowerhalf_s *priv,
                              uint16_t value)
{
  if (value >> 9)
    {
      return -EINVAL;
    }

  if (value == 0)
    {
      imxrt_enc_modifyreg16(priv, IMXRT_ENC_TST_OFFSET,
                            ENC_TST_TCE | ENC_TST_TEN, 0);
      return OK;
    }

  if (value & (1 << 8))
    {
      imxrt_enc_modifyreg16(priv, IMXRT_ENC_TST_OFFSET, 0, ENC_TST_QDN
                            | ENC_TST_TCE | ENC_TST_TEN);
    }
  else
    {
      imxrt_enc_modifyreg16(priv, IMXRT_ENC_TST_OFFSET, ENC_TST_QDN,
                            ENC_TST_TCE | ENC_TST_TEN);
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

static int imxrt_setup(struct qe_lowerhalf_s *lower)
{
  struct imxrt_enc_lowerhalf_s *priv =
    (struct imxrt_enc_lowerhalf_s *)lower;
  const struct imxrt_qeconfig_s *config = priv->config;
  uint32_t regval;
  int ret;

  ret = nxmutex_lock(&priv->lock);
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
  regval = config->tst_dir_adv ? ENC_TST_QDN : 0;
  regval |= (config->tst_period & ENC_TST_PERIOD_MASK) <<
            ENC_TST_PERIOD_SHIFT;
  imxrt_enc_putreg16(priv, IMXRT_ENC_TST_OFFSET, regval);
#endif

  if (((config->init_flags >> XIE_SHIFT) & 1) != 0)
    {
      ret = irq_attach(config->irq, imxrt_enc_index, priv);
      if (ret < 0)
        {
          snerr("ERROR: irq_attach failed: %d\n", ret);
          return ret;
        }

      up_enable_irq(config->irq);
    }

  /* Control and Control 2 register */

  regval = ENC_CTRL_SWIP;
  regval |= ((config->init_flags >> REV_SHIFT) & 1) ? ENC_CTRL_REV : 0;
  regval |= ((config->init_flags >> HIP_SHIFT) & 1) ? ENC_CTRL_HIP : 0;
  regval |= ((config->init_flags >> HNE_SHIFT) & 1) ? ENC_CTRL_HNE : 0;
  regval |= ((config->init_flags >> XIP_SHIFT) & 1) ? ENC_CTRL_XIP : 0;
  regval |= ((config->init_flags >> XIE_SHIFT) & 1) ? ENC_CTRL_XIE : 0;
  regval |= ((config->init_flags >> XNE_SHIFT) & 1) ? ENC_CTRL_XNE : 0;
  imxrt_enc_putreg16(priv, IMXRT_ENC_CTRL_OFFSET, regval);

  regval = ((config->init_flags >> MOD_SHIFT) & 1) ? ENC_CTRL2_MOD : 0;
  imxrt_enc_putreg16(priv, IMXRT_ENC_CTRL2_OFFSET, regval);

  nxmutex_unlock(&priv->lock);
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

static int imxrt_shutdown(struct qe_lowerhalf_s *lower)
{
  struct imxrt_enc_lowerhalf_s *priv =
    (struct imxrt_enc_lowerhalf_s *)lower;
  int ret;

  /* Ensure any in-progress operations are done. */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_DEBUG_SENSORS
  imxrt_enc_putreg16(priv, IMXRT_ENC_TST_OFFSET, 0);
#endif

  /* Disable interrupts if used */

  if (((priv->config->init_flags >> XIE_SHIFT) & 1) != 0)
    {
      up_disable_irq(priv->config->irq);
      irq_detach(priv->config->irq);
    }

  imxrt_enc_putreg16(priv, IMXRT_ENC_FILT_OFFSET, 0);
  imxrt_enc_putreg16(priv, IMXRT_ENC_LINIT_OFFSET, 0);
  imxrt_enc_putreg16(priv, IMXRT_ENC_UINIT_OFFSET, 0);
  imxrt_enc_putreg16(priv, IMXRT_ENC_REV_OFFSET, 0);
  imxrt_enc_putreg16(priv, IMXRT_ENC_POSD_OFFSET, 0);

  /* Write to only SWIP to blank CTRL and also reinit POS to 0 */

  imxrt_enc_putreg16(priv, IMXRT_ENC_CTRL_OFFSET, ENC_CTRL_SWIP);

  /* Gate the clock */

  imxrt_enc_clock_disable(priv->config->base);

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: imxrt_position
 *
 * Description:
 *   Return the current position measurement.
 *
 ****************************************************************************/

static int imxrt_position(struct qe_lowerhalf_s *lower, int32_t *pos)
{
  struct imxrt_enc_lowerhalf_s *priv =
    (struct imxrt_enc_lowerhalf_s *)lower;
  uint16_t lpos;
  uint16_t upos;
  int i;
  int ret;

  ret = nxmutex_lock(&priv->lock);
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
      nxmutex_unlock(&priv->lock);
      return -EAGAIN;
    }

  upos = imxrt_enc_getreg16(priv, IMXRT_ENC_UPOSH_OFFSET);

  nxmutex_unlock(&priv->lock);

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

static int imxrt_reset(struct qe_lowerhalf_s *lower)
{
  struct imxrt_enc_lowerhalf_s *priv =
    (struct imxrt_enc_lowerhalf_s *)lower;
  int ret;

  /* Write a 1 to the SWIP bit to load UINIT and LINIT into UPOS and LPOS */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  imxrt_enc_modifyreg16(priv, IMXRT_ENC_CTRL_OFFSET, 0, ENC_CTRL_SWIP);
  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: imxrt_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 ****************************************************************************/

static int imxrt_ioctl(struct qe_lowerhalf_s *lower, int cmd,
                       unsigned long arg)
{
  struct imxrt_enc_lowerhalf_s *priv = (struct imxrt_enc_lowerhalf_s *)lower;
  struct qe_index_s *data = priv->data;
  switch (cmd)
    {
      /* QEIOC_POSDIFF:
       * returns the content of the Position Difference register
       */

      case QEIOC_POSDIFF:
        *((uint16_t *)arg) = imxrt_enc_getreg16(priv, IMXRT_ENC_POSD_OFFSET);
        break;

      /* QEIOC_REVOLUTION:
       * returns the content of the Position Difference register
       */

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
      case QEIOC_GETINDEX:
        imxrt_position(lower, &data->qenc_pos);
        *((struct qe_index_s *)arg) = *data;
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

int imxrt_qeinitialize(const char *devpath, int enc)
{
  struct imxrt_enc_lowerhalf_s *priv = NULL;

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

  /* Register the upper-half driver */

  int ret = qe_register(devpath, (struct qe_lowerhalf_s *)priv);
  if (ret < 0)
    {
      snerr("ERROR: qe_register failed: %d\n", ret);
    }

  return ret;
}

#endif /* CONFIG_IMXRT_ENC */
