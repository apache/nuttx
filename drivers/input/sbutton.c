/****************************************************************************
 * drivers/input/sbutton.c
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

#include <nuttx/input/sbutton.h>
#include <nuttx/fs/fs.h>
#include <nuttx/clock.h>
#include <nuttx/ascii.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/input/keyboard.h>
#include <nuttx/input/kbd_codec.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This format is used to construct the /dev/kbd[n] device driver path. It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT      "/dev/kbd%d"
#define DEV_NAMELEN     12

#define KEY_PRESS       true
#define KEY_RELEASE     false

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sbutton_dev_s
{
  FAR const struct sbutton_config_s *config;  /* Board configuration data */

  mutex_t lock;         /* Exclusive access to dev */
  clock_t start;        /* Clock tick when the key was pressed */
  clock_t end;          /* Clock tick when the key was released */
  bool    pressed;      /* Keep previous status of button */
  struct  work_s work;  /* Supports the interrupt handling "bottom half" */

  /* Keyboard lowerhalf of the registered keyboard */

  struct keyboard_lowerhalf_s lower;
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

static int sbutton_interrupt(int irq, FAR void *context, FAR void *arg);
static void sbutton_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sbutton_worker
 ****************************************************************************/

static void sbutton_worker(FAR void *arg)
{
  FAR struct sbutton_dev_s *priv = (FAR struct sbutton_dev_s *)arg;
  uint8_t                   state;
  int                       ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return;
    }

  /* Read the status of the button */

  state = priv->config->status();

  /* If the user just pressed the button, start measuring */

  if (state == KEY_PRESS && !priv->pressed)
    {
      iinfo("Button pressed\n");

      priv->pressed = true;
      priv->start   = clock_systime_ticks();
    }
  else
    {
      if (state == KEY_RELEASE && priv->pressed)
        {
          uint32_t elapsed;

          iinfo("Button released\n");

          priv->pressed = false;

          priv->end = clock_systime_ticks();
          elapsed = priv->end - priv->start;

          /* Debounce to avoid getting wrong press/release event */

          if (elapsed < MSEC2TICK(CONFIG_INPUT_SBUTTON_KEY_DEBOUNCE_MS))
            {
              iwarn("Button event too short, ignoring it\n");
            }
          else
            {
              if (elapsed < MSEC2TICK(CONFIG_INPUT_SBUTTON_KEY_THRESH_MS))
                {
                  iinfo("KEY_1\n");
                  keyboard_event(&priv->lower, CONFIG_INPUT_SBUTTON_KEY1,
                                 KEYBOARD_PRESS);
                }
              else
                {
                  iinfo("KEY_2\n");
                  keyboard_event(&priv->lower, CONFIG_INPUT_SBUTTON_KEY2,
                                 KEYBOARD_PRESS);
                }
            }
        }
    }

  /* Unlock and return */

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: sbutton_interrupt
 ****************************************************************************/

static int sbutton_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct sbutton_dev_s *priv = (FAR struct sbutton_dev_s *)arg;
  int                        ret;

  /* Let the event worker know that it has an interrupt event to handle
   * It is possible that we will already have work scheduled from a
   * previous interrupt event.  That is OK we will service all the events
   * in the same work job.
   */

  if (work_available(&priv->work))
    {
      ret = work_queue(HPWORK, &priv->work, sbutton_worker, priv, 0);
      if (ret != 0)
        {
          ierr("ERROR: Failed to queue work: %d\n", ret);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sbutton_register
 *
 * Description:
 *   Configure the Single Button Multi Key Keyboard to use the provided
 *   instance.  This will register the driver as /dev/kbdN where N is the
 *   minor device number.
 *
 * Input Parameters:
 *   config      - Persistent board configuration data
 *   kbdminor    - The keyboard input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int sbutton_register(FAR const struct sbutton_config_s *config,
                     char kbdminor)
{
  FAR struct sbutton_dev_s *priv;
  char                      kbddevname[DEV_NAMELEN];
  int                       ret;

  /* Debug Sanity Checks */

  DEBUGASSERT(config != NULL);
  DEBUGASSERT(config->attach != NULL);
  DEBUGASSERT(config->enable != NULL);
  DEBUGASSERT(config->clear  != NULL);

  priv = kmm_zalloc(sizeof(struct sbutton_dev_s));
  if (!priv)
    {
      ierr("ERROR: kmm_zalloc(%d) failed\n", sizeof(struct sbutton_dev_s));
      return -ENOMEM;
    }

  /* Initialize the device driver instance */

  priv->config    = config;  /* Save the board configuration */
  priv->pressed   = false;

  nxmutex_init(&priv->lock);   /* Initialize device mutex */

  config->clear(config);
  config->enable(config, false);

  /* Attach the interrupt handler */

  ret = config->attach(config, sbutton_interrupt, priv);
  if (ret < 0)
    {
      ierr("ERROR: Failed to attach interrupt\n");
      goto errout_with_priv;
    }

  /* Start servicing events */

  priv->config->enable(priv->config, true);

  snprintf(kbddevname, sizeof(kbddevname), DEV_FORMAT, kbdminor);

  /* Register the device as a keyboard device */

  ret = keyboard_register(&priv->lower, kbddevname,
                          CONFIG_INPUT_SBUTTON_BUFSIZE);
  if (ret < 0)
    {
      ierr("ERROR: keyboard_register() failed: %d\n", ret);
      goto errout_with_priv;
    }

  return OK;

errout_with_priv:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}
