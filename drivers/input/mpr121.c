/****************************************************************************
 * drivers/input/mpr121.c
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

#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <nuttx/debug.h>
#include <errno.h>
#include <poll.h>
#include <fcntl.h>

#include <nuttx/input/mpr121.h>
#include <nuttx/fs/fs.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/keyboard.h>
#include <nuttx/input/kbd_codec.h>

#include "mpr121.h"

/* This driver is for MPR121 Capacitive Keypad
 * usually found at Aliexpress. This keypad has
 * 12 keys numbered from 0 to 11, starting from
 * 0 at bottom left and ending with 11 at top
 * right:
 *
 * +---+---+---+
 * | 3 | 7 | 11|
 * +---+---+---+
 * | 2 | 6 | 10|
 * +---+---+---+
 * | 1 | 5 | 9 |
 * +---+---+---+
 * | 0 | 4 | 8 |
 * +---+---+---+
 *   o o o o o
 *   | | | | |
 *   | | | | +- VCC
 *   | | | +--- IRQ
 *   | | +----- SCL
 *   | +------- SDA
 *   +--------- GND
 *
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mpr121_dev_s
{
  FAR const struct mpr121_config_s *config;  /* Board configuration data */

  mutex_t lock;           /* Exclusive access to device */
  struct work_s work;     /* Work queue to process keys after IRQ */

  /* Current and previous state of the matrix (bitfield) */

  FAR uint8_t *state; /* Current state bitmap */

  /* Keyboard lower-half registration */

  struct keyboard_lowerhalf_s lower;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Work queue */

static void    mpr121_worker(FAR void *arg);

/****************************************************************************
 * Name: mpr121_int_handler
 *
 * Description:
 *   Interrupt handler (ISR) for MPR121 IRQ pin.
 *
 ****************************************************************************/

static int mpr121_int_handler(int irq, FAR void *context, FAR void *arg)
{
  int ret;

  FAR struct mpr121_dev_s *priv = (FAR struct mpr121_dev_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Transfer processing to the worker thread.  Since MPR121 interrupts
   * are disabled while the work is pending, no special action should be
   * required to protect the work queue.
   */

  DEBUGASSERT(priv->work.worker == NULL);
  ret = work_queue(LPWORK, &priv->work, mpr121_worker, priv, 0);
  if (ret != 0)
    {
      ierr("ERROR: Failed to queue work: %d\n", ret);
    }

  return OK;
}

/****************************************************************************
 * Name: mpr121_getreg8
 *
 * Description:
 *   Read from an 8-bit mpr121 register
 *
 ****************************************************************************/

static uint8_t mpr121_getreg8(FAR struct mpr121_dev_s *priv,
                              uint8_t regaddr)
{
  struct i2c_msg_s msg[2];
  uint8_t regval;
  int ret;

  /* Setup 8-bit mpr121 address write message */

  msg[0].frequency = 400000;                   /* I2C frequency 400 KHz */
  msg[0].addr      = priv->config->i2c_addr;   /* 7-bit address */
  msg[0].flags     = 0;                        /* Write transaction, START */
  msg[0].buffer    = &regaddr;                 /* Transfer from this addr */
  msg[0].length    = 1;                        /* Send 1 byte following the addr
                                                * (no STOP) */

  /* Set up the 8-bit mpr121 data read message */

  msg[1].frequency = 400000;                   /* I2C frequency 400 KHz */
  msg[1].addr      = priv->config->i2c_addr;   /* 7-bit address */
  msg[1].flags     = I2C_M_READ;               /* Read transac., Re-START */
  msg[1].buffer    = &regval;                  /* Transfer to this address */
  msg[1].length    = 1;                        /* Recv 1 byte following the addr
                                                * (then STOP) */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->config->i2c_dev, msg, 2);
  if (ret < 0)
    {
      ierr("ERROR: I2C_TRANSFER failed: %d\n", ret);
      return 0;
    }

#ifdef CONFIG_MPR121_KEYPAD_REGDEBUG
  _err("%02x->%02x\n", regaddr, regval);
#endif
  return regval;
}

/****************************************************************************
 * Name: mpr121_putreg8
 *
 * Description:
 *   Write a value to an 8-bit mpr121 register
 *
 ****************************************************************************/

static int mpr121_putreg8(FAR struct mpr121_dev_s *priv,
                           uint8_t regaddr, uint8_t regval)
{
  /* 8-bit data read sequence:
   *
   * Start-I2C_Write_Address-MPR121_Reg_Address-MPR121_Write_Data-STOP
   */

  struct i2c_msg_s msg;
  uint8_t txbuffer[2];
  int ret;

#ifdef CONFIG_MPR121_REGDEBUG
  _err("%02x<-%02x\n", regaddr, regval);
#endif

  /* Setup to the data to be transferred.  Two bytes:  The mpr121 register
   * address followed by one byte of data.
   */

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  /* Setup 8-bit mpr121 address write message */

  msg.frequency = 400000;                   /* I2C frequency 400 KHz */
  msg.addr      = priv->config->i2c_addr;   /* 7-bit address */
  msg.flags     = 0;                        /* Write transaction, beginning with START */
  msg.buffer    = txbuffer;                 /* Transfer from this address */
  msg.length    = 2;                        /* Send two byte following the address
                                             * (then STOP) */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->config->i2c_dev, &msg, 1);
  if (ret < 0)
    {
      ierr("ERROR: I2C_TRANSFER failed: %d\n", ret);
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: mpr121_get_state
 *
 * Description:
 *   Get the current state of a key at position (key_idx)
 *
 ****************************************************************************/

static bool mpr121_get_state(FAR struct mpr121_dev_s *priv,
                             uint8_t key_idx)
{
  uint8_t byte_idx = key_idx / 8;
  uint8_t bit_idx = key_idx % 8;

  return (priv->state[byte_idx] >> bit_idx) & 1;
}

/****************************************************************************
 * Name: mpr121_set_state
 *
 * Description:
 *   Set the current state of a key at position (key_idx)
 *
 ****************************************************************************/

static void mpr121_set_state(FAR struct mpr121_dev_s *priv,
                             uint8_t key_idx, bool pressed)
{
  uint8_t byte_idx = key_idx / 8;
  uint8_t bit_idx = key_idx % 8;

  if (pressed)
    {
      priv->state[byte_idx] |= (1 << bit_idx);
    }
  else
    {
      priv->state[byte_idx] &= ~(1 << bit_idx);
    }
}

/****************************************************************************
 * Name: mpr121_worker
 *
 * Description:
 *   The worker is executed every time the MPR121 issue an IRQ (goes low)
 *
 ****************************************************************************/

static void mpr121_worker(FAR void *arg)
{
  FAR struct mpr121_dev_s *priv = (FAR struct mpr121_dev_s *)arg;
  uint32_t keycode;
  uint16_t status;
  uint8_t lo;
  uint8_t hi;
  int ret;
  int i;

  DEBUGASSERT(priv != NULL);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return;
    }

  /* Read the status touched keys */

  lo = mpr121_getreg8(priv, MPR121_ELE0_7_TS);
  hi = mpr121_getreg8(priv, MPR121_ELE8_11_TS);
  iinfo("Pressed: LO = 0x%02X | HI = 0x%02X", lo, hi);

  /* Only lower 4 bits of hi byte are valid (ELE8..ELE11) */

  status = (uint16_t)lo | ((uint16_t)(hi & 0x0f) << 8);

  /* Sweep all the previous saved keys and generate event
   * for each new key pressed or released
   */

  for (i = 0; i < MPR121_NUMKEY; i++)
    {
      if (mpr121_get_state(priv, i) != (bool)(status & 1 << i))
        {
          /* Generate keyboard event */

          keycode = priv->config->keymap[i];
          keyboard_event(&priv->lower, (uint16_t)keycode,
                 (status & 1 << i) ? KEYBOARD_PRESS : KEYBOARD_RELEASE);

          iinfo("Key [%d]: %s (code %lu)\n", i,
                (status & 1 << i) ? "PRESS" : "RELEASE",
                (unsigned long)keycode);
        }

      mpr121_set_state(priv, i, (bool)(status & 1 << i));
    }

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpr121_register
 *
 * Description:
 *   Configure and register a keyboard matrix device.
 *
 ****************************************************************************/

int mpr121_register(FAR const struct mpr121_config_s *config,
                    FAR const char *devpath)
{
  FAR struct mpr121_dev_s *priv;
  int i;
  int ret;
  uint8_t state_size;
  uint8_t regval;

  /* Validate configuration */

  DEBUGASSERT(config != NULL);
  DEBUGASSERT(devpath != NULL);
  DEBUGASSERT(config->keymap != NULL);

  iinfo("Registering MPR121 Capacitive Keypad as %s\n", devpath);

  /* Allocate driver instance */

  priv = kmm_zalloc(sizeof(struct mpr121_dev_s));
  if (!priv)
    {
      ierr("ERROR: kmm_zalloc(%zu) failed\n", sizeof(struct mpr121_dev_s));
      return -ENOMEM;
    }

  /* Calculate bitmap sizes */

  state_size = (MPR121_NUMKEY + 7) / 8;

  /* Allocate state bitmap */

  priv->state = kmm_zalloc(state_size);
  if (!priv->state)
    {
      ierr("ERROR: Failed to allocate state bitmap\n");
      kmm_free(priv);
      return -ENOMEM;
    }

  /* Initialize device structure */

  priv->config = config;

  nxmutex_init(&priv->lock);

  /* Soft Reset the MPR121 */

  ret = mpr121_putreg8(priv, MPR121_SRST, 0x63);
  if (ret < 0)
    {
      ierr("Failed to reset MPR121\n");
      goto errout_with_priv;
    }

  /* Force Stop Mode, although it should be in this mode after reset */

  ret = mpr121_putreg8(priv, MPR121_ECR, 0x00);
  if (ret < 0)
    {
      ierr("Failed to force MPR121 into Stop Mode\n");
      goto errout_with_priv;
    }

  /* Probe the device, AFE Conf1 = 0x10 and Conf2 = 0x24 */

  regval = mpr121_getreg8(priv, MPR121_AFE1);
  if (regval != 0x10)
    {
      ierr("Error MPR121_AFE1 = 0x%02X instead of 0x10\n", regval);
      goto errout_with_priv;
    }

  regval = mpr121_getreg8(priv, MPR121_AFE2);
  if (regval != 0x24)
    {
      ierr("Error MPR121_AFE2 = 0x%02X instead of 0x24\n", regval);
      goto errout_with_priv;
    }

  /* Setup the Baseline Tracking: MHD / NHD / NCL / FDL (see AN3944) */

  /* Rising */

  ret  = mpr121_putreg8(priv, MPR121_MHDR, 0x01);
  ret |= mpr121_putreg8(priv, MPR121_NHDR, 0x01);
  ret |= mpr121_putreg8(priv, MPR121_NCLR, 0x0e);
  ret |= mpr121_putreg8(priv, MPR121_FDLR, 0x00);

  /* Falling */

  ret |= mpr121_putreg8(priv, MPR121_MHDF, 0x01);
  ret |= mpr121_putreg8(priv, MPR121_NHDF, 0x05);
  ret |= mpr121_putreg8(priv, MPR121_NCLF, 0x01);
  ret |= mpr121_putreg8(priv, MPR121_FDLF, 0x00);

  /* Touched */

  ret |= mpr121_putreg8(priv, MPR121_NHDT, 0x00);
  ret |= mpr121_putreg8(priv, MPR121_NCLT, 0x00);
  ret |= mpr121_putreg8(priv, MPR121_FDLT, 0x00);

  if (ret < 0)
    {
      ierr("Failed to setup the baseline tracking\n");
      goto errout_with_priv;
    }

  /* Configure each electrode Touch / Release thresholds (all 12 channels)
   *
   * Touch threshold > Release threshold always, with hysteresis recommended
   * at ~50 % of touch threshold.
   */

  for (i = 0; i < MPR121_NUMKEY; i++)
    {
      ret = mpr121_putreg8(priv, MPR121_REG_TOUCH_THRESH(i),
                           MPR121_TOUCH_THRESHOLD);
      if (ret < 0)
        {
          ierr("Failed to set the touch threshold\n");
          goto errout_with_priv;
        }

      ret = mpr121_putreg8(priv, MPR121_REG_REL_THRESH(i),
                           MPR121_RELEASE_THRESHOLD);
      if (ret < 0)
        {
          ierr("Failed to set the release threshold\n");
          goto errout_with_priv;
        }
    }

  /* Configure Debounce = 2 consecutive samples required for touch / release
   *
   * Bits [2:0] = touch debounce
   * bits [6:4] = release debounce
   */

  ret = mpr121_putreg8(priv, MPR121_DT_DR, 0x22);
  if (ret < 0)
    {
      ierr("Failed to setup the debounce\n");
      goto errout_with_priv;
    }

  /* Configure AFE (Analog Front End) & Filter configuration
   *
   * We are keeping the original values here:
   * AFE1: FFI = 6 samples, CDC = 16 uA
   * AFE2: CDT = 0.5 us, SFI = 4 samples, ESI = 1 ms
   * Note: Increase ESI to reduce noise, but also reduces response time
   */

  ret  = mpr121_putreg8(priv, MPR121_AFE1, MPR121_AFE1_VALUE);
  ret |= mpr121_putreg8(priv, MPR121_AFE2, MPR121_AFE2_VALUE);
  if (ret < 0)
    {
      ierr("Failed to configure the Analog Front End and Filter\n");
      goto errout_with_priv;
    }

  /* Auto-Configuration (see AN3890), optional but recommended!
   *
   * When we know the Vdd we can automatically tunes CDC/CDT per electrode:
   *
   * AUTOCONF0: ACE=1 (enable), ARE=1 (retry), BVA=11 (baseline=5MSB)
   *            FFI same as AFE1 bits [7:6] => 0x0B
   * AUTOCONF1: ACFIE=0, ARFIE=0, OORIE=0, SCTS=0 => 0x00
   */

  ret  = mpr121_putreg8(priv, MPR121_AUTOCONF0, 0x0b);
  ret |= mpr121_putreg8(priv, MPR121_AUTOCONF1, 0x00);
  ret |= mpr121_putreg8(priv, MPR121_UPLIMIT, MPR121_UPLIMIT_VALUE);
  ret |= mpr121_putreg8(priv, MPR121_LOWLIMIT, MPR121_LOWLIMIT_VALUE);
  ret |= mpr121_putreg8(priv, MPR121_TARGETLIMIT, MPR121_TARGETLIMIT_VALUE);
  if (ret < 0)
    {
      ierr("Failed to setup the Auto Configuration limits\n");
      goto errout_with_priv;
    }

  /* Enter Run Mode writing the ECR register
   *
   * ECR:
   *     CL         [7:6] = 10 => baseline loaded from 5 upper bits data
   *     ELEPROX_EN [5:4] = 00 => proximity disabled
   *     ELE_EN     [3:0] = 1100 => ELE0..ELE11 (12 electrodes active)
   */

  ret = mpr121_putreg8(priv, MPR121_ECR, MPR121_ECR_VALUE);
  if (ret < 0)
    {
      ierr("Failed to enter in Run Mode\n");
      goto errout_with_priv;
    }

  /* Register as keyboard device */

  ret = keyboard_register(&priv->lower, devpath,
                          CONFIG_INPUT_MPR121_KEYPAD_BUFSIZE);
  if (ret < 0)
    {
      ierr("ERROR: keyboard_register() failed: %d\n", ret);
      goto errout_with_priv;
    }

  iinfo("MPR121 Capacitive Keypad registered as %s\n", devpath);

  /* Attach to the interrupt */

  priv->config->irq_attach(priv->config, mpr121_int_handler, priv);

  return OK;

errout_with_priv:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv->state);
  kmm_free(priv);
  return ret;
}
