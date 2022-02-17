/****************************************************************************
 * drivers/input/uinput.c
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <errno.h>
#include <stdio.h>

#include <nuttx/input/buttons.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/uinput.h>
#include <nuttx/kmalloc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UINPUT_NAME_SIZE 32

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct uinput_button_lowerhalf_s
{
  struct btn_lowerhalf_s lower;
  btn_buttonset_t        buttons;
  btn_handler_t          handler;
  FAR void              *arg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_INPUT_TOUCHSCREEN
static ssize_t uinput_touch_write(FAR struct touch_lowerhalf_s *lower,
                                  FAR const char *buffer, size_t buflen);
#endif

#ifdef CONFIG_INPUT_BUTTONS
static ssize_t uinput_button_write(FAR const struct btn_lowerhalf_s *lower,
                                   FAR const char *buffer, size_t buflen);

static btn_buttonset_t uinput_button_supported(FAR const struct
                                               btn_lowerhalf_s *lower);

static btn_buttonset_t uinput_button_buttons(FAR const struct
                                             btn_lowerhalf_s *lower);

static void uinput_button_enable(FAR const struct btn_lowerhalf_s *lower,
                                 btn_buttonset_t press,
                                 btn_buttonset_t release,
                                 btn_handler_t handler,
                                 FAR void *arg);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_INPUT_TOUCHSCREEN

/****************************************************************************
 * Name: uinput_touch_write
 ****************************************************************************/

static ssize_t uinput_touch_write(FAR struct touch_lowerhalf_s *lower,
                                  FAR const char *buffer, size_t buflen)
{
  FAR const struct touch_sample_s *sample;

  sample = (FAR const struct touch_sample_s *)buffer;
  if (sample == NULL)
    {
      return -EINVAL;
    }

  touch_event(lower->priv, sample);
  return buflen;
}
#endif /* CONFIG_INPUT_TOUCHSCREEN */

#ifdef CONFIG_INPUT_BUTTONS

/****************************************************************************
 * Name: uinput_button_write
 ****************************************************************************/

/****************************************************************************
 * Name: uinput_button_write
 ****************************************************************************/

static ssize_t uinput_button_write(FAR const struct btn_lowerhalf_s *lower,
                                   FAR const char *buffer, size_t buflen)
{
  FAR struct uinput_button_lowerhalf_s *ubtn_lower =
             (FAR struct uinput_button_lowerhalf_s *)lower;

  if (buflen != sizeof(btn_buttonset_t))
    {
      return -EINVAL;
    }

  ubtn_lower->buttons = *(btn_buttonset_t *)buffer;
  ubtn_lower->handler(&ubtn_lower->lower, ubtn_lower->arg);

  return buflen;
}

/****************************************************************************
 * Name: uinput_button_supported
 ****************************************************************************/

static btn_buttonset_t uinput_button_supported(FAR const struct
                                               btn_lowerhalf_s *lower)
{
  return ~0u;
}

/****************************************************************************
 * Name: uinput_button_buttons
 ****************************************************************************/

static btn_buttonset_t
uinput_button_buttons(FAR const struct btn_lowerhalf_s *lower)
{
  FAR struct uinput_button_lowerhalf_s *ubtn_lower =
             (FAR struct uinput_button_lowerhalf_s *)lower;

  return ubtn_lower->buttons;
}

/****************************************************************************
 * Name: uinput_button_enable
 ****************************************************************************/

static void uinput_button_enable(FAR const struct btn_lowerhalf_s *lower,
                                 btn_buttonset_t press,
                                 btn_buttonset_t release,
                                 btn_handler_t handler, FAR void *arg)
{
  FAR struct uinput_button_lowerhalf_s *ubtn_lower =
    (FAR struct uinput_button_lowerhalf_s *)lower;

  ubtn_lower->arg     = arg;
  ubtn_lower->handler = handler;
}

#endif /* CONFIG_INPUT_BUTTONS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uinput_touch_initialize
 *
 * Description:
 *   Initialized the uinput touchscreen device
 *
 * Input Parameters:
 *   name:      Touchscreen devices name
 *   maxpoint:  Maximum number of touch points supported.
 *   buff_nums: Number of the touch points structure.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_INPUT_TOUCHSCREEN
int uinput_touch_initialize(FAR const char *name, int maxpoint, int buffnums)
{
  char devname[UINPUT_NAME_SIZE];
  FAR struct touch_lowerhalf_s *lower;
  int ret;

  lower = kmm_zalloc(sizeof(struct touch_lowerhalf_s));
  if (!lower)
    {
      return -ENOMEM;
    }

  /* Regiest Touchscreen device */

  lower->write    = uinput_touch_write;
  lower->maxpoint = maxpoint;

  snprintf(devname, UINPUT_NAME_SIZE, "/dev/%s", name);
  ret = touch_register(lower, devname, buffnums);
  if (ret < 0)
    {
      kmm_free(lower);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: uinput_button_initialize
 *
 * Description:
 *   Initialized the uinput button device
 *
 * Input Parameters:
 *   name:      Button devices name
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_INPUT_BUTTONS
int uinput_button_initialize(FAR const char *name)
{
  char devname[UINPUT_NAME_SIZE];
  FAR struct uinput_button_lowerhalf_s *ubtn_lower;
  int ret;

  ubtn_lower = kmm_zalloc(sizeof(struct uinput_button_lowerhalf_s));
  ubtn_lower->lower.bl_buttons   = uinput_button_buttons;
  ubtn_lower->lower.bl_enable    = uinput_button_enable;
  ubtn_lower->lower.bl_supported = uinput_button_supported;
  ubtn_lower->lower.bl_write     = uinput_button_write;

  snprintf(devname, UINPUT_NAME_SIZE, "/dev/%s", name);
  ret = btn_register(devname, &ubtn_lower->lower);
  if (ret < 0)
    {
      kmm_free(ubtn_lower);
    }

  return ret;
}
#endif /* CONFIG_INPUT_BUTTONS */
