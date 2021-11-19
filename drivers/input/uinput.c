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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/uinput.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UINPUT_NAME_SIZE 32

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t uinput_touch_write(FAR struct touch_lowerhalf_s *lower,
                                  FAR const char *buffer, size_t buflen);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

int uinput_touch_initialize(FAR const char *name, int maxpoint, int buffnums)
{
  char devname[UINPUT_NAME_SIZE];
  FAR struct touch_lowerhalf_s *lower;

  lower = kmm_malloc(sizeof(struct touch_lowerhalf_s));
  if (!lower)
    {
      return -ENOMEM;
    }

  /* Regiest Touchscreen device */

  lower->write    = uinput_touch_write;
  lower->maxpoint = maxpoint;
  snprintf(devname, UINPUT_NAME_SIZE, "/dev/%s", name);

  return touch_register(lower, devname, buffnums);
}
