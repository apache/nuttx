/****************************************************************************
 * arch/xtensa/src/esp32/esp32_qencoder.c
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
#include <inttypes.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/sensors/qencoder.h>

#include <arch/board/board.h>

#include "chip.h"
#include "esp32_gpio.h"
#include "esp32_irq.h"

#include "xtensa.h"
#include "hardware/esp32_soc.h"
#include "hardware/esp32_dport.h"
#include "hardware/esp32_pinmap.h"
#include "hardware/esp32_gpio_sigmap.h"
#include "hardware/esp32_pcnt.h"
#include "esp32_qencoder.h"

#ifdef CONFIG_SENSORS_QENCODER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Input filter *************************************************************/

#ifdef CONFIG_ESP32_PCNT_U0_QE
#  ifndef CONFIG_ESP32_PCNT_U0_FILTER_EN
#    warning "Glitch Filter is recommended for Quadrature Encoder on PCNT_U0"
#  endif
#endif

#ifdef CONFIG_ESP32_PCNT_U1_QE
#  ifndef CONFIG_ESP32_PCNT_U1_FILTER_EN
#    warning "Glitch Filter is recommended for Quadrature Encoder on PCNT_U1"
#  endif
#endif

#ifdef CONFIG_ESP32_PCNT_U2_QE
#  ifndef CONFIG_ESP32_PCNT_U2_FILTER_EN
#    warning "Glitch Filter is recommended for Quadrature Encoder on PCNT_U2"
#  endif
#endif

#ifdef CONFIG_ESP32_PCNT_U3_QE
#  ifndef CONFIG_ESP32_PCNT_U3_FILTER_EN
#    warning "Glitch Filter is recommended for Quadrature Encoder on PCNT_U3"
#  endif
#endif

#ifdef CONFIG_ESP32_PCNT_U4_QE
#  ifndef CONFIG_ESP32_PCNT_U4_FILTER_EN
#    warning "Glitch Filter is recommended for Quadrature Encoder on PCNT_U4"
#  endif
#endif

#ifdef CONFIG_ESP32_PCNT_U5_QE
#  ifndef CONFIG_ESP32_PCNT_U5_FILTER_EN
#    warning "Glitch Filter is recommended for Quadrature Encoder on PCNT_U5"
#  endif
#endif

#ifdef CONFIG_ESP32_PCNT_U6_QE
#  ifndef CONFIG_ESP32_PCNT_U6_FILTER_EN
#    warning "Glitch Filter is recommended for Quadrature Encoder on PCNT_U6"
#  endif
#endif

#ifdef CONFIG_ESP32_PCNT_U7_QE
#  ifndef CONFIG_ESP32_PCNT_U7_FILTER_EN
#    warning "Glitch Filter is recommended for Quadrature Encoder on PCNT_U7"
#  endif
#endif

/* Debug ********************************************************************/

/* Non-standard debug that may be enabled just for testing the quadrature
 * encoder
 */

#ifndef CONFIG_DEBUG_FEATURES
#  undef CONFIG_DEBUG_SENSORS
#endif

#ifdef CONFIG_DEBUG_SENSORS
#  ifdef CONFIG_DEBUG_INFO
#    define qe_dumpgpio(p,m)    esp32_dumpgpio(p,m)
#  else
#    define qe_dumpgpio(p,m)
#  endif
#else
#  define qe_dumpgpio(p,m)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Constant configuration structure that is retained in FLASH */

struct esp32_qeconfig_s
{
  uint8_t   pcntid;        /* PCNT ID {0,1,2,3,4,5,6,7} */
  uint8_t   ch0_gpio;      /* Channel 0 gpio pin (Edge/Pulse) */
  uint8_t   ch1_gpio;      /* Channel 1 gpio pin (Level/Ctrl) */
  uint32_t  ch0_pulse_sig; /* ch0 pulse signal index */
  uint32_t  ch0_ctrl_sig;  /* ch0 ctrl signal index */
  uint32_t  ch1_pulse_sig; /* ch1 pulse signal index */
  uint32_t  ch1_ctrl_sig;  /* ch1 ctrl signal index */
  uint16_t  filter_thres;  /* Filter threshold for this PCNT Unit */
};

/* NOTE: we are using Quadrature Encoder in X4 mode on ESP32 PCNT, then
 * instead of using 'pulse_gpio' and 'ctrl_gpio' names, we only use ch0_gpio
 * and ch1_gpio names. It avoid confusion, since the same signal that is used
 * on pin 'pulse' of CH0 is also connected to 'ctrl' pin of the CH1 and
 * 'ctrl' pin of CH0 is also connected on 'pulse' pin of CH1.
 */

/* Overall, RAM-based state structure */

struct esp32_lowerhalf_s
{
  /* The first field of this state structure must be a pointer to the
   * lower-half callback structure:
   */

  const struct qe_ops_s *ops; /* Lower half callback structure */

  /* ESP32 driver-specific fields: */

  const struct esp32_qeconfig_s *config; /* static configuration */

  bool inuse; /* True: The lower-half driver is in-use */

  volatile int32_t position; /* The current position offset */

  spinlock_t lock; /* Device specific lock. */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions */

#if defined(CONFIG_DEBUG_SENSORS) && defined(CONFIG_DEBUG_INFO)
static void esp32_dumpregs(struct esp32_lowerhalf_s *priv,
                           const char *msg);
#else
#  define esp32_dumpregs(priv,msg)
#endif

static struct esp32_lowerhalf_s *esp32_pcnt2lower(int pcnt);

/* Interrupt handling */

#if 0 /* FIXME: To be implement */
static int esp32_interrupt(int irq, void *context, void *arg);
#endif

/* Lower-half Quadrature Encoder Driver Methods */

static int esp32_setup(struct qe_lowerhalf_s *lower);
static int esp32_shutdown(struct qe_lowerhalf_s *lower);
static int esp32_position(struct qe_lowerhalf_s *lower,
                          int32_t *pos);
static int esp32_setposmax(struct qe_lowerhalf_s *lower, uint32_t pos);
static int esp32_reset(struct qe_lowerhalf_s *lower);
static int esp32_setindex(struct qe_lowerhalf_s *lower, uint32_t pos);
static int esp32_ioctl(struct qe_lowerhalf_s *lower, int cmd,
                       unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The lower half callback structure */

static const struct qe_ops_s g_qecallbacks =
{
  .setup     = esp32_setup,
  .shutdown  = esp32_shutdown,
  .position  = esp32_position,
  .setposmax = esp32_setposmax,
  .reset     = esp32_reset,
  .setindex  = esp32_setindex,
  .ioctl     = esp32_ioctl,
};

/* Per-pcnt state structures */

#ifdef CONFIG_ESP32_PCNT_U0_QE
static const struct esp32_qeconfig_s g_pcnt0config =
{
  .pcntid        = 0,
  .ch0_gpio      = CONFIG_ESP32_PCNT_U0_CH0_EDGE_PIN,
  .ch1_gpio      = CONFIG_ESP32_PCNT_U0_CH1_LEVEL_PIN,
  .ch0_pulse_sig = PCNT_SIG_CH0_IN0_IDX,
  .ch1_pulse_sig = PCNT_SIG_CH1_IN0_IDX,
  .ch0_ctrl_sig  = PCNT_CTRL_CH0_IN0_IDX,
  .ch1_ctrl_sig  = PCNT_CTRL_CH1_IN0_IDX,
  .filter_thres  = CONFIG_ESP32_PCNT_U0_FILTER_THRES,
};

static struct esp32_lowerhalf_s g_pcnt0lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_pcnt0config,
  .inuse    = false,
};
#endif

#ifdef CONFIG_ESP32_PCNT_U1_QE
static const struct esp32_qeconfig_s g_pcnt1config =
{
  .pcntid        = 1,
  .ch0_gpio      = CONFIG_ESP32_PCNT_U1_CH0_EDGE_PIN,
  .ch1_gpio      = CONFIG_ESP32_PCNT_U1_CH1_LEVEL_PIN,
  .ch0_pulse_sig = PCNT_SIG_CH0_IN1_IDX,
  .ch1_pulse_sig = PCNT_SIG_CH1_IN1_IDX,
  .ch0_ctrl_sig  = PCNT_CTRL_CH0_IN1_IDX,
  .ch1_ctrl_sig  = PCNT_CTRL_CH1_IN1_IDX,
  .filter_thres  = CONFIG_ESP32_PCNT_U1_FILTER_THRES,
};

static struct esp32_lowerhalf_s g_pcnt1lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_pcnt1config,
  .inuse    = false,
};
#endif

#ifdef CONFIG_ESP32_PCNT_U2_QE
static const struct esp32_qeconfig_s g_pcnt2config =
{
  .pcntid       = 2,
  .ch0_gpio     = CONFIG_ESP32_PCNT_U2_CH0_EDGE_PIN,
  .ch1_gpio     = CONFIG_ESP32_PCNT_U2_CH1_LEVEL_PIN,
  .ch0_pulse_sig = PCNT_SIG_CH0_IN2_IDX,
  .ch1_pulse_sig = PCNT_SIG_CH1_IN2_IDX,
  .ch0_ctrl_sig  = PCNT_CTRL_CH0_IN2_IDX,
  .ch1_ctrl_sig  = PCNT_CTRL_CH1_IN2_IDX,
  .filter_thres = CONFIG_ESP32_PCNT_U2_FILTER_THRES,
};

static struct esp32_lowerhalf_s g_pcnt2lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_pcnt2config,
  .inuse    = false,
};
#endif

#ifdef CONFIG_ESP32_PCNT_U3_QE
static const struct esp32_qeconfig_s g_pcnt3config =
{
  .pcntid        = 3,
  .ch0_gpio      = CONFIG_ESP32_PCNT_U3_CH0_EDGE_PIN,
  .ch1_gpio      = CONFIG_ESP32_PCNT_U3_CH1_LEVEL_PIN,
  .ch0_pulse_sig = PCNT_SIG_CH0_IN3_IDX,
  .ch1_pulse_sig = PCNT_SIG_CH1_IN3_IDX,
  .ch0_ctrl_sig  = PCNT_CTRL_CH0_IN3_IDX,
  .ch1_ctrl_sig  = PCNT_CTRL_CH1_IN3_IDX,
  .filter_thres  = CONFIG_ESP32_PCNT_U3_FILTER_THRES,
};

static struct esp32_lowerhalf_s g_pcnt3lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_pcnt3config,
  .inuse    = false,
};
#endif

#ifdef CONFIG_ESP32_PCNT_U4_QE
static const struct esp32_qeconfig_s g_pcnt4config =
{
  .pcntid        = 4,
  .ch0_gpio      = CONFIG_ESP32_PCNT_U4_CH0_EDGE_PIN,
  .ch1_gpio      = CONFIG_ESP32_PCNT_U4_CH1_LEVEL_PIN,
  .ch0_pulse_sig = PCNT_SIG_CH0_IN4_IDX,
  .ch1_pulse_sig = PCNT_SIG_CH1_IN4_IDX,
  .ch0_ctrl_sig  = PCNT_CTRL_CH0_IN4_IDX,
  .ch1_ctrl_sig  = PCNT_CTRL_CH1_IN4_IDX,
  .filter_thres  = CONFIG_ESP32_PCNT_U4_FILTER_THRES,
};

static struct esp32_lowerhalf_s g_pcnt4lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_pcnt4config,
  .inuse    = false,
};
#endif

#ifdef CONFIG_ESP32_PCNT_U5_QE
static const struct esp32_qeconfig_s g_pcnt5config =
{
  .pcntid       = 5,
  .ch0_gpio     = CONFIG_ESP32_PCNT_U5_CH0_EDGE_PIN,
  .ch1_gpio     = CONFIG_ESP32_PCNT_U5_CH1_LEVEL_PIN,
  .ch0_pulse_sig = PCNT_SIG_CH0_IN5_IDX,
  .ch1_pulse_sig = PCNT_SIG_CH1_IN5_IDX,
  .ch0_ctrl_sig  = PCNT_CTRL_CH0_IN5_IDX,
  .ch1_ctrl_sig  = PCNT_CTRL_CH1_IN5_IDX,
  .filter_thres = CONFIG_ESP32_PCNT_U5_FILTER_THRES,
};

static struct esp32_lowerhalf_s g_pcnt5lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_pcnt5config,
  .inuse    = false,
};
#endif

#ifdef CONFIG_ESP32_PCNT_U6_QE
static const struct esp32_qeconfig_s g_pcnt6config =
{
  .pcntid        = 6,
  .ch0_gpio      = CONFIG_ESP32_PCNT_U6_CH0_EDGE_PIN,
  .ch1_gpio      = CONFIG_ESP32_PCNT_U6_CH1_LEVEL_PIN,
  .ch0_pulse_sig = PCNT_SIG_CH0_IN6_IDX,
  .ch1_pulse_sig = PCNT_SIG_CH1_IN6_IDX,
  .ch0_ctrl_sig  = PCNT_CTRL_CH0_IN6_IDX,
  .ch1_ctrl_sig  = PCNT_CTRL_CH1_IN6_IDX,
  .filter_thres  = CONFIG_ESP32_PCNT_U6_FILTER_THRES,
};

static struct esp32_lowerhalf_s g_pcnt6lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_pcnt6config,
  .inuse    = false,
};
#endif

#ifdef CONFIG_ESP32_PCNT_U7_QE
static const struct esp32_qeconfig_s g_pcnt7config =
{
  .pcntid        = 7,
  .ch0_gpio      = CONFIG_ESP32_PCNT_U7_CH0_EDGE_PIN,
  .ch1_gpio      = CONFIG_ESP32_PCNT_U7_CH1_LEVEL_PIN,
  .ch0_pulse_sig = PCNT_SIG_CH0_IN7_IDX,
  .ch1_pulse_sig = PCNT_SIG_CH1_IN7_IDX,
  .ch0_ctrl_sig  = PCNT_CTRL_CH0_IN7_IDX,
  .ch1_ctrl_sig  = PCNT_CTRL_CH1_IN7_IDX,
  .filter_thres  = CONFIG_ESP32_PCNT_U7_FILTER_THRES,
};

static struct esp32_lowerhalf_s g_pcnt7lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_pcnt7config,
  .inuse    = false,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input Parameters:
 *   priv - A reference to the QENCODER block status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_SENSORS) && defined(CONFIG_DEBUG_INFO)
static void esp32_dumpregs(struct esp32_lowerhalf_s *priv,
                           const char *msg)
{
  sninfo("%s:\n", msg);
  sninfo("  PCNT_U0_CONF0_REG: %08x PCNT_U1_CONF0_REG:  %08x\n",
         getreg32(PCNT_CONF0_U(0)),
         getreg32(PCNT_CONF0_U(1)));
  sninfo("  PCNT_U2_CONF0_REG: %08x PCNT_U3_CONF0_REG:  %08x\n",
         getreg32(PCNT_CONF0_U(2)),
         getreg32(PCNT_CONF0_U(3)));
  sninfo("  PCNT_U4_CONF0_REG: %08x PCNT_U5_CONF0_REG:  %08x\n",
         getreg32(PCNT_CONF0_U(4)),
         getreg32(PCNT_CONF0_U(5)));
  sninfo("  PCNT_U6_CONF0_REG: %08x PCNT_U7_CONF0_REG:  %08x\n",
         getreg32(PCNT_CONF0_U(6)),
         getreg32(PCNT_CONF0_U(7)));
  sninfo("  PCNT_U0_CONF1_REG: %08x PCNT_U1_CONF1_REG:  %08x\n",
         getreg32(PCNT_CONF1_U(0)),
         getreg32(PCNT_CONF1_U(1)));
  sninfo("  PCNT_U2_CONF1_REG: %08x PCNT_U3_CONF1_REG:  %08x\n",
         getreg32(PCNT_CONF1_U(2)),
         getreg32(PCNT_CONF1_U(3)));
  sninfo("  PCNT_U4_CONF1_REG: %08x PCNT_U5_CONF1_REG:  %08x\n",
         getreg32(PCNT_CONF1_U(4)),
         getreg32(PCNT_CONF1_U(5)));
  sninfo("  PCNT_U6_CONF1_REG: %08x PCNT_U7_CONF1_REG:  %08x\n",
         getreg32(PCNT_CONF1_U(6)),
         getreg32(PCNT_CONF1_U(7)));
  sninfo("  PCNT_U0_CONF2_REG: %08x PCNT_U1_CONF2_REG:  %08x\n",
         getreg32(PCNT_CONF2_U(0)),
         getreg32(PCNT_CONF2_U(1)));
  sninfo("  PCNT_U2_CONF2_REG: %08x PCNT_U3_CONF2_REG:  %08x\n",
         getreg32(PCNT_CONF2_U(2)),
         getreg32(PCNT_CONF2_U(3)));
  sninfo("  PCNT_U4_CONF2_REG: %08x PCNT_U5_CONF2_REG:  %08x\n",
         getreg32(PCNT_CONF2_U(4)),
         getreg32(PCNT_CONF2_U(5)));
  sninfo("  PCNT_U6_CONF2_REG: %08x PCNT_U7_CONF2_REG:  %08x\n",
         getreg32(PCNT_CONF2_U(6)),
         getreg32(PCNT_CONF2_U(7)));
  sninfo("  PCNT_U0_CNT_REG: %08x PCNT_U1_CNT_REG:  %08x\n",
         getreg32(PCNT_CNT_U(0)),
         getreg32(PCNT_CNT_U(1)));
  sninfo("  PCNT_U2_CNT_REG: %08x PCNT_U3_CNT_REG:  %08x\n",
         getreg32(PCNT_CNT_U(2)),
         getreg32(PCNT_CNT_U(3)));
  sninfo("  PCNT_U4_CNT_REG: %08x PCNT_U5_CNT_REG:  %08x\n",
         getreg32(PCNT_CNT_U(4)),
         getreg32(PCNT_CNT_U(5)));
  sninfo("  PCNT_U6_CNT_REG: %08x PCNT_U7_CNT_REG:  %08x\n",
         getreg32(PCNT_CNT_U(6)),
         getreg32(PCNT_CNT_U(7)));
  sninfo("  PCNT_CTRL_REF: %08x\n",
         getreg32(PCNT_CTRL_REG);
}
#endif

/****************************************************************************
 * Name: esp32_pcnt2lower
 *
 * Description:
 *   Map a PCNT number to a device structure
 *
 ****************************************************************************/

static struct esp32_lowerhalf_s *esp32_pcnt2lower(int pcnt)
{
  switch (pcnt)
    {
#ifdef CONFIG_ESP32_PCNT_U0_QE
    case 0:
      return &g_pcnt0lower;
#endif
#ifdef CONFIG_ESP32_PCNT_U1_QE
    case 1:
      return &g_pcnt1lower;
#endif
#ifdef CONFIG_ESP32_PCNT_U2_QE
    case 2:
      return &g_pcnt2lower;
#endif
#ifdef CONFIG_ESP32_PCNT_U3_QE
    case 3:
      return &g_pcnt3lower;
#endif
#ifdef CONFIG_ESP32_PCNT_U4_QE
    case 4:
      return &g_pcnt4lower;
#endif
#ifdef CONFIG_ESP32_PCNT_U5_QE
    case 5:
      return &g_pcnt5lower;
#endif
#ifdef CONFIG_ESP32_PCNT_U6_QE
    case 6:
      return &g_pcnt6lower;
#endif
#ifdef CONFIG_ESP32_PCNT_U7_QE
    case 7:
      return &g_pcnt7lower;
#endif
    default:
      return NULL;
    }
}

/****************************************************************************
 * Name: esp32_interrupt
 *
 * Description:
 *   Common timer interrupt handling.  NOTE: Only 16-bit timers require timer
 *   interrupts.
 *
 ****************************************************************************/

#if 0 /* FIXME: To be implement */
static int esp32_interrupt(int irq, void *context, void *arg)
{
  struct esp32_lowerhalf_s *priv = (struct esp32_lowerhalf_s *)arg;
  uint16_t regval;

  DEBUGASSERT(priv != NULL);

  return OK;
}
#endif

/****************************************************************************
 * Name: esp32_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   The initial position value should be zero. *
 *
 ****************************************************************************/

static int esp32_setup(struct qe_lowerhalf_s *lower)
{
  struct esp32_lowerhalf_s *priv = (struct esp32_lowerhalf_s *)lower;
  irqstate_t flags;
  uint32_t regval;

  /* Protected access to the registers */

  flags = spin_lock_irqsave(&priv->lock);

  esp32_dumpregs(priv, "Before setup");

  /* Enable the PCNT Clock and Reset the peripheral */

  modifyreg32(DPORT_PERIP_CLK_EN_REG, 0, DPORT_PCNT_CLK_EN);
  modifyreg32(DPORT_PERIP_RST_EN_REG, 0, DPORT_PCNT_RST);
  modifyreg32(DPORT_PERIP_RST_EN_REG, DPORT_PCNT_RST, 0);

  /* Disable all events */

  putreg32(0, PCNT_CONF0_U(priv->config->pcntid));

  /* Configure the limits range PCNT_CNT_L_LIM | PCNT_CNT_H_LIM */

  regval  = INT16_MIN << 16;
  regval |= INT16_MAX;
  putreg32(regval, PCNT_CONF2_U(priv->config->pcntid));

  /* Setup POS/NEG/LCTRL/HCTRL/FILTER modes */

  regval  = priv->config->filter_thres;
  regval |= PCNT_COUNT_INC << PCNT_CH0_NEG_MODE_U0_S;      /* Ignore Falling-Edge */
  regval |= PCNT_COUNT_DEC << PCNT_CH0_POS_MODE_U0_S;      /* Increase on Rising-Edge */
  regval |= PCNT_MODE_REVERSE << PCNT_CH0_LCTRL_MODE_U0_S; /* Rising A with B in HIGH = CW step */
  regval |= PCNT_MODE_KEEP << PCNT_CH0_HCTRL_MODE_U0_S;    /* Rising A with B in LOW = CCW step */

  putreg32(regval, PCNT_CONF0_U(priv->config->pcntid));

  regval |= PCNT_COUNT_DEC << PCNT_CH1_NEG_MODE_U0_S; /* Ignore Falling-Edge */
  regval |= PCNT_COUNT_INC << PCNT_CH1_POS_MODE_U0_S; /* Increase on Rising-Edge */

  putreg32(regval, PCNT_CONF0_U(priv->config->pcntid));

  /* Configure GPIO pins as Input with Pull-Up enabled */

  esp32_configgpio(priv->config->ch0_gpio, INPUT_FUNCTION_3 | PULLUP);
  esp32_configgpio(priv->config->ch1_gpio, INPUT_FUNCTION_3 | PULLUP);

  /* Connect Channel A (ch0_gpio) and Channel B (ch1_gpio) crossed for X4 */

  esp32_gpio_matrix_in(priv->config->ch0_gpio,
                       priv->config->ch0_pulse_sig, 0);
  esp32_gpio_matrix_in(priv->config->ch1_gpio,
                       priv->config->ch0_ctrl_sig, 0);

  esp32_gpio_matrix_in(priv->config->ch1_gpio,
                       priv->config->ch1_pulse_sig, 0);
  esp32_gpio_matrix_in(priv->config->ch0_gpio,
                       priv->config->ch1_ctrl_sig, 0);

  /* Clear the Reset bit to nable the Pulse Counter */

  regval = getreg32(PCNT_CTRL_REG);
  regval &= ~(1 << (2 * priv->config->pcntid));
  putreg32(regval, PCNT_CTRL_REG);

  esp32_dumpregs(priv, "After setup");

  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Name: esp32_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   should stop data collection, free any resources, disable timer hardware,
 *   and put the system into the lowest possible power usage state
 *
 ****************************************************************************/

static int esp32_shutdown(struct qe_lowerhalf_s *lower)
{
  struct esp32_lowerhalf_s *priv = (struct esp32_lowerhalf_s *)lower;

  /* Disable PCNT clock */

  modifyreg32(DPORT_PERIP_CLK_EN_REG, DPORT_PCNT_CLK_EN, 0);

  /* Make sure initial position is 0 */

  priv->position = 0;

  return OK;
}

/****************************************************************************
 * Name: esp32_position
 *
 * Description:
 *   Return the current position measurement.
 *
 ****************************************************************************/

static int esp32_position(struct qe_lowerhalf_s *lower, int32_t *pos)
{
  struct esp32_lowerhalf_s *priv = (struct esp32_lowerhalf_s *)lower;
  irqstate_t flags;
  int32_t position;
  int16_t count;

  DEBUGASSERT(lower && priv->inuse);

  flags = spin_lock_irqsave(&priv->lock);

  position = priv->position;
  count = (int16_t)(getreg32(PCNT_CNT_U(priv->config->pcntid)) & 0xffff);

  /* Return the position measurement */

  *pos = position + count;

  spin_unlock_irqrestore(&priv->lock, flags);

  return OK;
}

/****************************************************************************
 * Name: esp32_setposmax
 *
 * Description:
 *   Set the maximum encoder position.
 *
 ****************************************************************************/

static int esp32_setposmax(struct qe_lowerhalf_s *lower, uint32_t pos)
{
#ifdef CONFIG_ESP32_QENCODER_DISABLE_EXTEND16BTIMERS
  struct esp32_lowerhalf_s *priv = (struct esp32_lowerhalf_s *)lower;

#if defined(HAVE_MIXEDWIDTH_TIMERS)
  if (priv->config->width == 32)
    {
      esp32_putreg32(priv, ESP32_GTIM_ARR_OFFSET, pos);
    }
  else
    {
      esp32_putreg16(priv, ESP32_GTIM_ARR_OFFSET, pos);
    }
#elif defined(HAVE_32BIT_TIMERS)
  esp32_putreg32(priv, ESP32_GTIM_ARR_OFFSET, pos);
#else
  esp32_putreg16(priv, ESP32_GTIM_ARR_OFFSET, pos);
#endif

  return OK;
#else
  return -ENOTTY;
#endif
}

/****************************************************************************
 * Name: esp32_reset
 *
 * Description:
 *   Reset the position measurement to zero.
 *
 ****************************************************************************/

static int esp32_reset(struct qe_lowerhalf_s *lower)
{
  struct esp32_lowerhalf_s *priv = (struct esp32_lowerhalf_s *)lower;
  irqstate_t flags;
  uint32_t regval;

  sninfo("Resetting position to zero\n");
  DEBUGASSERT(lower && priv->inuse);

  /* Reset this Pulse Counter Unit. */

  flags = spin_lock_irqsave(&priv->lock);

  regval  = getreg32(PCNT_CTRL_REG);
  regval |= PCNT_CNT_RST_U(priv->config->pcntid);
  putreg32(regval, PCNT_CTRL_REG);

  priv->position = 0;

  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: esp32_setindex
 *
 * Description:
 *   Set the index pin position
 *
 ****************************************************************************/

static int esp32_setindex(struct qe_lowerhalf_s *lower, uint32_t pos)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: esp32_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 ****************************************************************************/

static int esp32_ioctl(struct qe_lowerhalf_s *lower, int cmd,
                       unsigned long arg)
{
  /* No ioctl commands supported */

  /* TODO add an IOCTL to control the encoder pulse count prescaler */

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_qeinitialize
 *
 * Description:
 *   Initialize a quadrature encoder interface.  This function must be
 *   called from board-specific logic.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/qe0"
 *   pcnt    - The PCNT number to used.  'pcnt' must be an element of
 *             {0,1,2,3,4,5,6,7,8}
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

int esp32_qeinitialize(const char *devpath, int pcnt)
{
  struct esp32_lowerhalf_s *priv;
  int ret;

  /* Find the pre-allocated timer state structure corresponding to this
   * timer
   */

  priv = esp32_pcnt2lower(pcnt);
  if (priv == NULL)
    {
      snerr("ERROR: PCNT%d support not configured\n", pcnt);
      return -ENXIO;
    }

  /* Make sure that it is available */

  if (priv->inuse)
    {
      snerr("ERROR: PCNT%d is in-use\n", pcnt);
      return -EBUSY;
    }

  /* Register the upper-half driver */

  ret = qe_register(devpath, (struct qe_lowerhalf_s *)priv);
  if (ret < 0)
    {
      snerr("ERROR: qe_register failed: %d\n", ret);
      return ret;
    }

  /* Make sure that the PCNT is in the shutdown state */

  esp32_shutdown((struct qe_lowerhalf_s *)priv);

  /* The driver is now in-use */

  priv->inuse = true;

  return OK;
}

#endif /* CONFIG_SENSORS_QENCODER */
