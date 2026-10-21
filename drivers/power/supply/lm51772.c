/****************************************************************************
 * drivers/power/supply/lm51772.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/lm51772.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Time for VCC2 and the power stage supplies to settle after EN goes high */

#define LM51772_ENABLE_DELAY_US  15000

/* Output voltage target: 12 bits, step set by SEL_FB_DIV20 */

#define LM51772_VOUT_MASK        0x0fff
#define LM51772_VOUT_STEP_DIV20  20  /* mV */
#define LM51772_VOUT_STEP_DIV10  10  /* mV */

/* Output current limit register: LM51772_ILIM_THRESHOLD() in the header.
 * The register saturates at 0x8c (70 mV across the sense resistor): a limit
 * above it is refused instead of being silently lowered by the chip.
 */

#define LM51772_ILIM_MAX         0x8c

/* Current limit in amperes of an ILIM_THRESHOLD value */

#define LM51772_ILIM_A(val, mohm) ((float)(val) * 0.5f / (float)(mohm))

/* I2C address when the board does not set one (ADDR pin open) */

#define LM51772_ADDR_DEFAULT     0x6a

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lm51772_s
{
  struct smps_s           smps;        /* Upper half data, must be first */
  struct lm51772_config_s config;
  struct i2c_config_s     i2c;         /* I2C frequency and address */
  mutex_t                 lock;        /* Serializes register sequences */
  float                   init_ilim;   /* A, current limit of the init list */
  uint8_t                 vout_step;   /* mV, known once configured */
  bool                    configured;  /* Powered and registers written */
  bool                    running;     /* Power stage on */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int lm51772_setup(FAR struct smps_dev_s *dev);
static int lm51772_shutdown(FAR struct smps_dev_s *dev);
static int lm51772_stop(FAR struct smps_dev_s *dev);
static int lm51772_start(FAR struct smps_dev_s *dev);
static int lm51772_params_set(FAR struct smps_dev_s *dev,
                              FAR struct smps_params_s *param);
static int lm51772_mode_set(FAR struct smps_dev_s *dev, uint8_t mode);
static int lm51772_limits_set(FAR struct smps_dev_s *dev,
                              FAR struct smps_limits_s *limits);
static int lm51772_fault_set(FAR struct smps_dev_s *dev, uint8_t fault);
static int lm51772_state_get(FAR struct smps_dev_s *dev,
                             FAR struct smps_state_s *state);
static int lm51772_fault_get(FAR struct smps_dev_s *dev,
                             FAR uint8_t *fault);
static int lm51772_fault_clean(FAR struct smps_dev_s *dev, uint8_t fault);
static int lm51772_ioctl(FAR struct smps_dev_s *dev, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct smps_ops_s g_lm51772_ops =
{
  .setup       = lm51772_setup,
  .shutdown    = lm51772_shutdown,
  .stop        = lm51772_stop,
  .start       = lm51772_start,
  .params_set  = lm51772_params_set,
  .mode_set    = lm51772_mode_set,
  .limits_set  = lm51772_limits_set,
  .fault_set   = lm51772_fault_set,
  .state_get   = lm51772_state_get,
  .fault_get   = lm51772_fault_get,
  .fault_clean = lm51772_fault_clean,
  .ioctl       = lm51772_ioctl
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm51772_reg_write / lm51772_reg_read
 ****************************************************************************/

static int lm51772_reg_write(FAR struct lm51772_s *priv, uint8_t reg,
                             uint8_t val)
{
  uint8_t buf[2];

  buf[0] = reg;
  buf[1] = val;

  return i2c_write(priv->config.i2c, &priv->i2c, buf, 2);
}

static int lm51772_reg_read(FAR struct lm51772_s *priv, uint8_t reg,
                            FAR uint8_t *val)
{
  return i2c_writeread(priv->config.i2c, &priv->i2c, &reg, 1, val, 1);
}

/****************************************************************************
 * Name: lm51772_reg_modify
 *
 * Description:
 *   Read a register, clear the bits in clr, set the bits in set, write it
 *   back.
 *
 ****************************************************************************/

static int lm51772_reg_modify(FAR struct lm51772_s *priv, uint8_t reg,
                              uint8_t clr, uint8_t set)
{
  uint8_t val;
  int ret;

  ret = lm51772_reg_read(priv, reg, &val);
  if (ret < 0)
    {
      return ret;
    }

  return lm51772_reg_write(priv, reg, (val & ~clr) | set);
}

/****************************************************************************
 * Name: lm51772_vout_reg
 *
 * Description:
 *   Convert an output voltage in volts to the target register value, with
 *   the step of the chip.
 *
 ****************************************************************************/

static uint32_t lm51772_vout_reg(FAR struct lm51772_s *priv, float v_out)
{
  uint32_t mv = (uint32_t)(v_out * 1000.0f + 0.5f);

  return mv / priv->vout_step;
}

/****************************************************************************
 * Name: lm51772_ilim_reg
 *
 * Description:
 *   Convert an output current limit in amperes to the ILIM_THRESHOLD
 *   value, or a negated errno if it does not fit in the register.
 *
 ****************************************************************************/

static int lm51772_ilim_reg(FAR struct lm51772_s *priv, float i_out)
{
  uint32_t ma = (uint32_t)(i_out * 1000.0f + 0.5f);
  uint32_t val = LM51772_ILIM_THRESHOLD(ma, priv->config.r_isns_mohm);

  return val > LM51772_ILIM_MAX ? -EINVAL : (int)val;
}

/****************************************************************************
 * Name: lm51772_check_ratings
 *
 * Description:
 *   Check an output voltage and current limit against the absolute
 *   maximum ratings of the board.  An i_out of 0 keeps the current limit of
 *   the init list, or the maximum current if there is none.
 *
 ****************************************************************************/

static int lm51772_check_ratings(FAR struct lm51772_s *priv, float v_out,
                                 float i_out)
{
  FAR const struct lm51772_config_s *cfg = &priv->config;
  float i_lim = i_out;

  if ((cfg->v_out_min > 0.0f && v_out < cfg->v_out_min) ||
      (cfg->v_out_max > 0.0f && v_out > cfg->v_out_max) ||
      (cfg->i_out_max > 0.0f && i_out > cfg->i_out_max))
    {
      return -EINVAL;
    }

  if (cfg->p_out_max > 0.0f)
    {
      if (i_lim <= 0.0f)
        {
          i_lim = priv->init_ilim > 0.0f ? priv->init_ilim : cfg->i_out_max;
        }

      if (v_out * i_lim > cfg->p_out_max)
        {
          return -EINVAL;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: lm51772_apply_params
 *
 * Description:
 *   Write the output voltage target and, if set, the current limit.  The
 *   chip must be configured and its voltage step known.
 *
 ****************************************************************************/

static int lm51772_apply_params(FAR struct lm51772_s *priv)
{
  uint32_t vout = lm51772_vout_reg(priv, priv->smps.param.v_out);
  int ilim = -1;  /* Negative: keep the limit of the chip */
  int ret;

  if (vout == 0 || vout > LM51772_VOUT_MASK)
    {
      return -EINVAL;
    }

  if (priv->smps.param.i_out > 0.0f)
    {
      ilim = lm51772_ilim_reg(priv, priv->smps.param.i_out);
      if (ilim < 0)
        {
          return ilim;
        }
    }

  ret = lm51772_reg_write(priv, LM51772_REG_VOUT_TARGET1_LSB,
                          vout & 0xff);
  if (ret < 0)
    {
      return ret;
    }

  ret = lm51772_reg_write(priv, LM51772_REG_VOUT_TARGET1_MSB,
                          (vout >> 8) & 0x0f);
  if (ret < 0 || ilim < 0)
    {
      return ret;
    }

  return lm51772_reg_write(priv, LM51772_REG_ILIM_THRESHOLD, ilim);
}

/****************************************************************************
 * Name: lm51772_configure
 *
 * Description:
 *   Enable the controller (EN high), check that it answers and write the
 *   register list and the parameters.  The power stage does
 *   not switch yet.  Called with the lock held.
 *
 ****************************************************************************/

static int lm51772_configure(FAR struct lm51772_s *priv)
{
  uint8_t val;
  size_t i;
  int ret;

  priv->config.enable(priv->config.arg, true);
  nxsig_usleep(LM51772_ENABLE_DELAY_US);

  ret = lm51772_reg_read(priv, LM51772_REG_STATUS_BYTE, &val);
  if (ret < 0)
    {
      _err("ERROR: not answering at 0x%02x: %d\n", priv->i2c.address, ret);
      goto errout;
    }

  for (i = 0; i < priv->config.ninit; i++)
    {
      val = priv->config.init[i].val;

      /* The power stage starts in lm51772_start() */

      if (priv->config.init[i].reg == LM51772_REG_MFR_SPECIFIC_D0)
        {
          val &= ~LM51772_D0_CONV_EN;
        }

      ret = lm51772_reg_write(priv, priv->config.init[i].reg, val);
      if (ret < 0)
        {
          goto errout;
        }
    }

  ret = lm51772_reg_read(priv, LM51772_REG_MFR_SPECIFIC_D8, &val);
  if (ret < 0)
    {
      goto errout;
    }

  priv->vout_step = (val & LM51772_D8_SEL_FB_DIV20) != 0 ?
                    LM51772_VOUT_STEP_DIV20 : LM51772_VOUT_STEP_DIV10;

  ret = lm51772_apply_params(priv);
  if (ret < 0)
    {
      goto errout;
    }

  /* Drop the faults latched while the input was settling (VIN_UV) */

  ret = lm51772_reg_write(priv, LM51772_REG_CLEAR_FAULTS, 0);
  if (ret < 0)
    {
      goto errout;
    }

  priv->configured = true;
  return OK;

errout:
  priv->config.enable(priv->config.arg, false);
  return ret;
}

/****************************************************************************
 * Name: lm51772_stop_locked
 *
 * Description:
 *   Stop switching and discharge the output.  Both writes are tried even
 *   if the first fails.
 *
 ****************************************************************************/

static int lm51772_stop_locked(FAR struct lm51772_s *priv)
{
  int ret;
  int ret2;

  ret  = lm51772_reg_modify(priv, LM51772_REG_MFR_SPECIFIC_D0,
                            LM51772_D0_CONV_EN, 0);
  ret2 = lm51772_reg_modify(priv, LM51772_REG_USB_PD_CONTROL_0,
                            LM51772_PD0_CONV_EN2, LM51772_PD0_FORCE_DISCH);
  priv->running = false;

  return ret < 0 ? ret : ret2;
}

/****************************************************************************
 * Name: lm51772_faults
 *
 * Description:
 *   Map STATUS_BYTE to smps_fault_e bits.  BUSY and OFF are not faults.
 *
 ****************************************************************************/

static uint8_t lm51772_faults(uint8_t status)
{
  uint8_t fault = 0;

  if ((status & LM51772_STATUS_VOUT) != 0)
    {
      fault |= SMPS_FAULT_OVERVOLTAGE;
    }

  if ((status & LM51772_STATUS_IOUT) != 0)
    {
      fault |= SMPS_FAULT_OVERCURRENT;
    }

  if ((status & LM51772_STATUS_TEMPERATURE) != 0)
    {
      fault |= SMPS_FAULT_OVERTEMP;
    }

  if ((status & (LM51772_STATUS_INPUT | LM51772_STATUS_CML |
                 LM51772_STATUS_OTHER)) != 0)
    {
      fault |= SMPS_FAULT_OTHER;
    }

  return fault;
}

/****************************************************************************
 * Name: lm51772_status_get
 *
 * Description:
 *   Read the faults.  Without power in the controller there is nothing to
 *   read and there are no faults.
 *
 ****************************************************************************/

static int lm51772_status_get(FAR struct lm51772_s *priv,
                              FAR uint8_t *fault)
{
  uint8_t status;
  int ret;

  *fault = 0;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->configured)
    {
      ret = lm51772_reg_read(priv, LM51772_REG_STATUS_BYTE, &status);
      if (ret >= 0)
        {
          *fault = lm51772_faults(status);
        }
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: lm51772_setup
 *
 * Description:
 *   Called by the upper half on the first open, with interrupts disabled,
 *   so no I2C here: the controller is brought up in lm51772_start().
 *
 ****************************************************************************/

static int lm51772_setup(FAR struct smps_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: lm51772_shutdown
 *
 * Description:
 *   Called by the upper half on the last close, with interrupts disabled:
 *   disable the controller (EN low).  This stops the converter, so a
 *   program that starts it has to keep the device open while it runs.
 *   The chip may keep its registers (they depend on nRST), so they are
 *   written again at the next start.
 *
 ****************************************************************************/

static int lm51772_shutdown(FAR struct smps_dev_s *dev)
{
  FAR struct lm51772_s *priv = dev->lower;

  priv->config.enable(priv->config.arg, false);
  priv->configured = false;
  priv->running    = false;

  return OK;
}

/****************************************************************************
 * Name: lm51772_stop
 ****************************************************************************/

static int lm51772_stop(FAR struct smps_dev_s *dev)
{
  FAR struct lm51772_s *priv = dev->lower;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->configured)
    {
      ret = lm51772_stop_locked(priv);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: lm51772_start
 ****************************************************************************/

static int lm51772_start(FAR struct smps_dev_s *dev)
{
  FAR struct lm51772_s *priv = dev->lower;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->running)
    {
      goto out;
    }

  if (priv->smps.param.v_out <= 0.0f)
    {
      ret = -EPERM;
      goto out;
    }

  if (!priv->configured)
    {
      ret = lm51772_configure(priv);
      if (ret < 0)
        {
          goto out;
        }
    }

  ret = lm51772_reg_modify(priv, LM51772_REG_MFR_SPECIFIC_D0, 0,
                           LM51772_D0_CONV_EN);
  if (ret >= 0)
    {
      ret = lm51772_reg_modify(priv, LM51772_REG_USB_PD_CONTROL_0,
                               LM51772_PD0_FORCE_DISCH,
                               LM51772_PD0_CONV_EN2);
    }

  if (ret < 0)
    {
      lm51772_stop_locked(priv);
      goto out;
    }

  /* Drop the faults latched while the output was still zero */

  lm51772_reg_write(priv, LM51772_REG_CLEAR_FAULTS, 0);
  priv->running = true;

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: lm51772_params_set
 *
 * Description:
 *   Store the output voltage and the output current limit.  They are
 *   written to the chip right away if it is configured, or at the start
 *   otherwise, when the range of the voltage is checked against the step
 *   of the chip.  An i_out of 0 keeps the limit of the chip.
 *
 ****************************************************************************/

static int lm51772_params_set(FAR struct smps_dev_s *dev,
                              FAR struct smps_params_s *param)
{
  FAR struct lm51772_s *priv = dev->lower;
  struct smps_params_s old;
  int ret = OK;

  if (param->v_out <= 0.0f || param->i_out < 0.0f)
    {
      return -EINVAL;
    }

  ret = lm51772_check_ratings(priv, param->v_out, param->i_out);
  if (ret < 0)
    {
      return ret;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  old = priv->smps.param;
  priv->smps.param = *param;

  if (param->i_out > 0.0f)
    {
      ret = lm51772_ilim_reg(priv, param->i_out);
    }

  if (ret >= 0 && priv->configured)
    {
      ret = lm51772_apply_params(priv);
    }

  if (ret < 0)
    {
      priv->smps.param = old;
    }

  nxmutex_unlock(&priv->lock);
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: lm51772_mode_set
 *
 * Description:
 *   The controller regulates the output voltage: only constant voltage.
 *
 ****************************************************************************/

static int lm51772_mode_set(FAR struct smps_dev_s *dev, uint8_t mode)
{
  FAR struct lm51772_s *priv = dev->lower;

  if (mode != SMPS_OPMODE_CV)
    {
      return -ENOTSUP;
    }

  priv->smps.opmode = mode;
  return OK;
}

/****************************************************************************
 * Name: lm51772_limits_set
 *
 * Description:
 *   Keep the limits for the upper half checks (params against limits).
 *   The output voltage limit is required.  Limits above the absolute
 *   maximum ratings of the board are refused, and a current or power
 *   limit left at 0 (none for the upper half) becomes the rating.
 *
 ****************************************************************************/

static int lm51772_limits_set(FAR struct smps_dev_s *dev,
                              FAR struct smps_limits_s *limits)
{
  FAR struct lm51772_s *priv = dev->lower;
  FAR const struct lm51772_config_s *cfg = &priv->config;

  if (limits->v_out <= 0.0f ||
      (cfg->v_out_max > 0.0f && limits->v_out > cfg->v_out_max) ||
      (cfg->i_out_max > 0.0f && limits->i_out > cfg->i_out_max) ||
      (cfg->p_out_max > 0.0f && limits->p_out > cfg->p_out_max))
    {
      return -EINVAL;
    }

  priv->smps.limits = *limits;

  if (limits->i_out <= 0.0f)
    {
      priv->smps.limits.i_out = cfg->i_out_max;
    }

  if (limits->p_out <= 0.0f)
    {
      priv->smps.limits.p_out = cfg->p_out_max;
    }

  return OK;
}

/****************************************************************************
 * Name: lm51772_fault_set
 ****************************************************************************/

static int lm51772_fault_set(FAR struct smps_dev_s *dev, uint8_t fault)
{
  return -ENOTSUP;
}

/****************************************************************************
 * Name: lm51772_state_get
 *
 * Description:
 *   The state and the faults come from the controller.  The feedback is
 *   left empty: the board measures the load current and the battery
 *   voltage with the ADC, and the converter has no other telemetry.
 *
 ****************************************************************************/

static int lm51772_state_get(FAR struct smps_dev_s *dev,
                             FAR struct smps_state_s *state)
{
  FAR struct lm51772_s *priv = dev->lower;
  int ret;

  memset(state, 0, sizeof(*state));

  ret = lm51772_status_get(priv, &state->fault);
  if (ret < 0)
    {
      return ret;
    }

  state->state = state->fault != 0 ? SMPS_STATE_FAULT :
                 priv->running ? SMPS_STATE_RUN : SMPS_STATE_IDLE;

  return OK;
}

/****************************************************************************
 * Name: lm51772_fault_get
 ****************************************************************************/

static int lm51772_fault_get(FAR struct smps_dev_s *dev, FAR uint8_t *fault)
{
  return lm51772_status_get(dev->lower, fault);
}

/****************************************************************************
 * Name: lm51772_fault_clean
 ****************************************************************************/

static int lm51772_fault_clean(FAR struct smps_dev_s *dev, uint8_t fault)
{
  FAR struct lm51772_s *priv = dev->lower;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* The chip clears all the latched faults at once */

  if (priv->configured)
    {
      ret = lm51772_reg_write(priv, LM51772_REG_CLEAR_FAULTS, 0);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: lm51772_ioctl
 ****************************************************************************/

static int lm51772_ioctl(FAR struct smps_dev_s *dev, int cmd,
                         unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm51772_register
 *
 * Description:
 *   Register a LM51772 as an SMPS device.  The controller stays powered
 *   down until the first PWRIOC_START.
 *
 ****************************************************************************/

int lm51772_register(FAR const char *path,
                     FAR const struct lm51772_config_s *config)
{
  FAR struct smps_dev_s *dev;
  FAR struct lm51772_s *priv;
  size_t i;
  int ret;

  DEBUGASSERT(path != NULL && config != NULL);
  DEBUGASSERT(config->i2c != NULL && config->enable != NULL);
  DEBUGASSERT(config->r_isns_mohm > 0);

  priv = kmm_zalloc(sizeof(struct lm51772_s));
  dev  = kmm_zalloc(sizeof(struct smps_dev_s));
  if (priv == NULL || dev == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  priv->config      = *config;

  /* The current limit the chip keeps when the parameters have none */

  for (i = 0; i < config->ninit; i++)
    {
      if (config->init[i].reg == LM51772_REG_ILIM_THRESHOLD)
        {
          priv->init_ilim = LM51772_ILIM_A(config->init[i].val,
                                           config->r_isns_mohm);
        }
    }

  priv->i2c.address = config->addr != 0 ? config->addr :
                      LM51772_ADDR_DEFAULT;
  priv->i2c.addrlen = 7;
  priv->i2c.frequency = config->frequency;
  nxmutex_init(&priv->lock);

  dev->ops  = &g_lm51772_ops;
  dev->priv = &priv->smps;

  ret = smps_register(path, dev, priv);
  if (ret < 0)
    {
      nxmutex_destroy(&priv->lock);
      goto errout;
    }

  return OK;

errout:
  kmm_free(priv);
  kmm_free(dev);
  return ret;
}
