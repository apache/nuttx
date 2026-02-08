/****************************************************************************
 * drivers/input/kmatrix.c
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
#include <debug.h>
#include <errno.h>
#include <poll.h>
#include <fcntl.h>

#include <nuttx/input/kmatrix.h>
#include <nuttx/fs/fs.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/input/keyboard.h>
#include <nuttx/input/kbd_codec.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct kmatrix_dev_s
{
  FAR const struct kmatrix_config_s *config;  /* Board configuration data */

  mutex_t lock;           /* Exclusive access to device */
  struct work_s work;     /* Work queue for polling */
  uint16_t poll_interval; /* Polling interval in milliseconds */

  /* Current and previous state of the matrix (bitfield) */

  FAR uint8_t *state;     /* Current state bitmap */
  FAR uint8_t *debounce;  /* Debounce counter */

  /* Keyboard lower-half registration */

  struct keyboard_lowerhalf_s lower;
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

static void kmatrix_scan_worker(FAR void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kmatrix_get_state
 *
 * Description:
 *   Get the current state of a key at position (row, col)
 *
 ****************************************************************************/

static bool kmatrix_get_state(FAR struct kmatrix_dev_s *priv,
                               uint8_t row, uint8_t col)
{
  uint16_t idx = row * priv->config->ncols + col;
  uint16_t byte_idx = idx / 8;
  uint8_t bit_idx = idx % 8;

  return (priv->state[byte_idx] >> bit_idx) & 1;
}

/****************************************************************************
 * Name: kmatrix_set_state
 *
 * Description:
 *   Set the current state of a key at position (row, col)
 *
 ****************************************************************************/

static void kmatrix_set_state(FAR struct kmatrix_dev_s *priv,
                               uint8_t row, uint8_t col, bool pressed)
{
  uint16_t idx = row * priv->config->ncols + col;
  uint16_t byte_idx = idx / 8;
  uint8_t bit_idx = idx % 8;

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
 * Name: kmatrix_inc_debounce
 *
 * Description:
 *   Increment debounce counter for a key
 *
 ****************************************************************************/

static void kmatrix_inc_debounce(FAR struct kmatrix_dev_s *priv,
                                 uint8_t row, uint8_t col)
{
  uint16_t idx = row * priv->config->ncols + col;

  if (priv->debounce[idx] < CONFIG_INPUT_KMATRIX_DEBOUNCE)
    {
      priv->debounce[idx]++;
    }
}

/****************************************************************************
 * Name: kmatrix_reset_debounce
 *
 * Description:
 *   Reset debounce counter for a key
 *
 ****************************************************************************/

static void kmatrix_reset_debounce(FAR struct kmatrix_dev_s *priv,
                                   uint8_t row, uint8_t col)
{
  uint16_t idx = row * priv->config->ncols + col;

  priv->debounce[idx] = 0;
}

/****************************************************************************
 * Name: kmatrix_scan_worker
 *
 * Description:
 *   Periodic worker that scans the keyboard matrix and detects key presses
 *   and releases.
 *
 ****************************************************************************/

static void kmatrix_scan_worker(FAR void *arg)
{
  FAR struct kmatrix_dev_s *priv = (FAR struct kmatrix_dev_s *)arg;
  uint8_t row;
  uint8_t col;
  bool pressed;
  bool old_state;
  uint32_t keycode;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return;
    }

  /* Scan each row */

  for (row = 0; row < priv->config->nrows; row++)
    {
      /* Activate this row */

      priv->config->row_set(priv->config->rows[row], true);

      /* Read each column */

      for (col = 0; col < priv->config->ncols; col++)
        {
          pressed = priv->config->col_get(priv->config->cols[col]);
          old_state = kmatrix_get_state(priv, row, col);

          /* Check if state changed */

          if (pressed != old_state)
            {
              kmatrix_inc_debounce(priv, row, col);

              /* After debounce threshold is reached, update state */

              if (priv->debounce[row * priv->config->ncols + col] >=
                  CONFIG_INPUT_KMATRIX_DEBOUNCE)
                {
                  kmatrix_set_state(priv, row, col, pressed);
                  kmatrix_reset_debounce(priv, row, col);

                  /* Generate keyboard event */

                  keycode = priv->config->keymap[
                    row * priv->config->ncols + col];
                  keyboard_event(&priv->lower, (uint16_t)keycode,
                                 pressed ? KEYBOARD_PRESS :
                                           KEYBOARD_RELEASE);

                  iinfo("Key [%d,%d]: %s (code %lu)\n", row, col,
                        pressed ? "PRESS" : "RELEASE",
                        (unsigned long)keycode);
                }
            }
          else
            {
              kmatrix_reset_debounce(priv, row, col);
            }
        }

      /* Deactivate this row */

      priv->config->row_set(priv->config->rows[row], false);
    }

  nxmutex_unlock(&priv->lock);

  /* Reschedule the worker */

  work_queue(LPWORK, &priv->work, kmatrix_scan_worker, priv,
             MSEC2TICK(priv->poll_interval));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kmatrix_register
 *
 * Description:
 *   Configure and register a keyboard matrix device.
 *
 ****************************************************************************/

int kmatrix_register(FAR const struct kmatrix_config_s *config,
                     FAR const char *devpath)
{
  FAR struct kmatrix_dev_s *priv;
  int ret;
  uint16_t state_size;
  uint16_t debounce_size;
  uint16_t keys;

  iinfo("Registering keypad matrix: %dx%d at %s\n", config->nrows,
        config->ncols, devpath);

  /* Validate configuration */

  DEBUGASSERT(config != NULL);
  DEBUGASSERT(devpath != NULL);
  DEBUGASSERT(config->rows != NULL);
  DEBUGASSERT(config->cols != NULL);
  DEBUGASSERT(config->keymap != NULL);
  DEBUGASSERT(config->config_row != NULL);
  DEBUGASSERT(config->config_col != NULL);
  DEBUGASSERT(config->row_set != NULL);
  DEBUGASSERT(config->col_get != NULL);

  /* Allocate driver instance */

  priv = kmm_zalloc(sizeof(struct kmatrix_dev_s));
  if (!priv)
    {
      ierr("ERROR: kmm_zalloc(%zu) failed\n", sizeof(struct kmatrix_dev_s));
      return -ENOMEM;
    }

  /* Calculate bitmap sizes */

  keys = config->nrows * config->ncols;
  state_size = (keys + 7) / 8;
  debounce_size = keys;

  /* Allocate state and debounce bitmaps */

  priv->state = kmm_zalloc(state_size);
  if (!priv->state)
    {
      ierr("ERROR: Failed to allocate state bitmap\n");
      kmm_free(priv);
      return -ENOMEM;
    }

  priv->debounce = kmm_zalloc(debounce_size);
  if (!priv->debounce)
    {
      ierr("ERROR: Failed to allocate debounce bitmap\n");
      kmm_free(priv->state);
      kmm_free(priv);
      return -ENOMEM;
    }

  /* Initialize device structure */

  priv->config         = config;
  priv->poll_interval  = config->poll_interval_ms > 0 ?
                         config->poll_interval_ms :
                         CONFIG_INPUT_KMATRIX_POLL_MS;

  nxmutex_init(&priv->lock);

  /* Configure all GPIO pins */

  for (int i = 0; i < config->nrows; i++)
    {
      config->config_row(config->rows[i]);
    }

  for (int i = 0; i < config->ncols; i++)
    {
      config->config_col(config->cols[i]);
    }

  /* Register as keyboard device */

  ret = keyboard_register(&priv->lower, devpath,
                          CONFIG_INPUT_KMATRIX_BUFSIZE);
  if (ret < 0)
    {
      ierr("ERROR: keyboard_register() failed: %d\n", ret);
      goto errout_with_priv;
    }

  /* Start the scanning worker */

  work_queue(LPWORK, &priv->work, kmatrix_scan_worker, priv,
             MSEC2TICK(priv->poll_interval));

  iinfo("Keypad matrix registered as %s\n", devpath);
  return OK;

errout_with_priv:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv->debounce);
  kmm_free(priv->state);
  kmm_free(priv);
  return ret;
}
