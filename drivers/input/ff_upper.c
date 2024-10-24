/****************************************************************************
 * drivers/input/ff_upper.c
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

#include <nuttx/fs/fs.h>
#include <nuttx/input/ff.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ff_effect_s
{
  struct ff_effect data;
  FAR struct file *owner;
};

/* This structure is for force feedback device upper half driver */

struct ff_upperhalf_s
{
  /* A pointer of lower half instance */

  FAR struct ff_lowerhalf_s *lower;

  /* Manages exclusive access to this structure */

  mutex_t lock;

  /* Maximum number of effects supported by device. */

  int max_effects;

  /* The pointer to an array of effects context currently loaded into device.
   * when file handle owning an effect gets closed the effect is
   * automatically erased.
   */

  FAR struct ff_effect_s *effects;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ff_open(FAR struct file *filep);
static int ff_close(FAR struct file *filep);
static ssize_t ff_write(FAR struct file *filep, FAR const char *buffer,
                        size_t buflen);
static int ff_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ff_fops =
{
  ff_open,  /* open */
  ff_close, /* close */
  NULL,     /* read */
  ff_write, /* write */
  NULL,     /* seek */
  ff_ioctl, /* ioctl */
  NULL,     /* mmap */
  NULL,     /* truncate */
  NULL      /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ff_check_effect_access
 *
 * Description:
 *   Check that the effect_id is a valid effect and whether the user
 *   is the owner.
 *
 ****************************************************************************/

static int ff_check_effect_access(FAR struct ff_upperhalf_s *upper,
                                  int effect_id, FAR struct file * filep)
{
  if (effect_id < 0 || effect_id >= upper->max_effects ||
      upper->effects[effect_id].owner == NULL)
    {
      return -EINVAL;
    }

  if (filep != NULL && upper->effects[effect_id].owner != filep)
    {
      return -EACCES;
    }

  return 0;
}

/****************************************************************************
 * Name: ff_check_effects_compatible
 *
 * Description:
 *   Checks whether 2 effects can be combined together.
 *
 ****************************************************************************/

static inline bool ff_check_effects_compatible(FAR struct ff_effect *e1,
                                               FAR struct ff_effect *e2)
{
  return e1->type == e2->type &&
         (e1->type != FF_PERIODIC ||
          e1->u.periodic.waveform == e2->u.periodic.waveform);
}

/****************************************************************************
 * Name: ff_compat_effect
 *
 * Description:
 *   Convert an effect into compatible one.
 *
 ****************************************************************************/

static int ff_compat_effect(FAR struct ff_lowerhalf_s *lower,
                            FAR struct ff_effect *effect)
{
  switch (effect->type)
    {
      case FF_RUMBLE:
        {
          int magnitude;

          if (!test_bit(FF_PERIODIC, lower->ffbit))
            {
              return -EINVAL;
            }

          /* Calculate magnitude of sine wave as average of rumble's
           * 2/3 of strong magnitude and 1/3 of weak magnitude
           */

          magnitude = effect->u.rumble.strong_magnitude / 3 +
                      effect->u.rumble.weak_magnitude / 6;

          effect->type = FF_PERIODIC;
          effect->u.periodic.waveform = FF_SINE;
          effect->u.periodic.period = 50;
          effect->u.periodic.magnitude = magnitude;
          effect->u.periodic.offset = 0;
          effect->u.periodic.phase = 0;
          effect->u.periodic.envelope.attack_length = 0;
          effect->u.periodic.envelope.attack_level = 0;
          effect->u.periodic.envelope.fade_length = 0;
          effect->u.periodic.envelope.fade_level = 0;

          return 0;
        }

      default:
        return 0;
    }
}

/****************************************************************************
 * Name: ff_erase_effect
 *
 * Description:
 *   Erases the effect if the requester is also the effect owner. The mutex
 *   should already be locked before calling this function.
 *
 ****************************************************************************/

static int ff_erase_effect(FAR struct ff_upperhalf_s *upper, int effect_id,
                           FAR struct file *filep)
{
  FAR struct ff_lowerhalf_s *lower = upper->lower;
  irqstate_t flags;
  int ret;

  ret = ff_check_effect_access(upper, effect_id, filep);
  if (ret < 0)
    {
      return ret;
    }

  flags = enter_critical_section();
  lower->playback(lower, effect_id, 0);
  leave_critical_section(flags);

  upper->effects[effect_id].owner = NULL;
  if (lower->erase != NULL)
    {
      ret = lower->erase(lower, effect_id);
      if (ret < 0)
        {
          upper->effects[effect_id].owner = filep;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: ff_upload
 *
 * Description:
 *   Upload effect into force-feedback device.
 *
 ****************************************************************************/

static int ff_upload(FAR struct ff_upperhalf_s *upper,
                     FAR struct ff_effect *effect, FAR struct file *filep)
{
  FAR struct ff_lowerhalf_s *lower = upper->lower;
  FAR struct ff_effect *old;
  int ret = 0;
  int id;

  if (effect->type > FF_EFFECT_MAX)
    {
      return -EINVAL;
    }

  if (effect->type == FF_PERIODIC &&
      (effect->u.periodic.waveform < FF_WAVEFORM_MIN ||
       effect->u.periodic.waveform > FF_WAVEFORM_MAX ||
       !test_bit(effect->u.periodic.waveform, lower->ffbit)))
    {
      return -EINVAL;
    }

  if (!test_bit(effect->type, lower->ffbit))
    {
      ret = ff_compat_effect(lower, effect);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (effect->id == -1)
    {
      for (id = 0; id < upper->max_effects; id++)
        {
          if (upper->effects[id].owner == NULL)
            {
              break;
            }
        }

      if (id >= upper->max_effects)
        {
          return -ENOSPC;
        }

      effect->id = id;
      old = NULL;
    }
  else
    {
      id = effect->id;
      ret = ff_check_effect_access(upper, id, filep);
      if (ret < 0)
        {
          return ret;
        }

      old = &upper->effects[id].data;
      if (!ff_check_effects_compatible(effect, old))
        {
          return -EINVAL;
        }
    }

  ret = lower->upload(lower, effect, old);
  if (ret < 0)
    {
      return ret;
    }

  upper->effects[id].data = *effect;
  upper->effects[id].owner = filep;

  return ret;
}

/****************************************************************************
 * Name: ff_open
 ****************************************************************************/

static int ff_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: ff_close
 ****************************************************************************/

static int ff_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: ff_write
 ****************************************************************************/

static ssize_t ff_write(FAR struct file *filep, FAR const char *buffer,
                        size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ff_upperhalf_s *upper = inode->i_private;
  FAR struct ff_event_s *event = (FAR struct ff_event_s *)buffer;

  DEBUGASSERT(buflen == sizeof(*event));

  nxmutex_lock(&upper->lock);
  ff_event(upper->lower, event->code, event->value);
  nxmutex_unlock(&upper->lock);

  return buflen;
}

/****************************************************************************
 * Name: ff_ioctl
 ****************************************************************************/

static int ff_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ff_upperhalf_s *upper = inode->i_private;
  int ret = OK;

  nxmutex_lock(&upper->lock);
  switch (cmd)
    {
      case EVIOCGBIT:
        {
          memcpy((FAR unsigned long *)(uintptr_t)arg, upper->lower->ffbit,
                 sizeof(upper->lower->ffbit));
        }
        break;

      case EVIOCSFF:
        {
          ret = ff_upload(upper, (FAR struct ff_effect *)(uintptr_t)arg,
                          filep);
        }
        break;

      case EVIOCRMFF:
        {
          ret = ff_erase_effect(upper, (int)arg, filep);
        }
        break;

      case EVIOCGEFFECTS:
        {
          *(FAR int *)(uintptr_t)arg = upper->max_effects;
        }
        break;

      case EVIOCSETCALIBDATA:
        {
          if (upper->lower->set_calibvalue == NULL)
            {
              ret = -ENOTSUP;
              break;
            }

          ret = upper->lower->set_calibvalue(upper->lower, arg);
        }
        break;

      case EVIOCCALIBRATE:
        {
          if (upper->lower->calibrate == NULL)
            {
              ret = -ENOTSUP;
              break;
            }

          ret = upper->lower->calibrate(upper->lower, arg);
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&upper->lock);
  return ret;
}

/****************************************************************************
 * Public Function
 ****************************************************************************/

/****************************************************************************
 * Name: ff_event
 *
 * Description:
 *   The lower half driver pushes force feedback events through this
 *   interface, provided by force feedback upper half.
 *
 * Arguments:
 *   lower - lower half driver handle.
 *   code  - event code.
 *   value - event value.
 *
 ****************************************************************************/

int ff_event(FAR struct ff_lowerhalf_s *lower, uint32_t code, int value)
{
  irqstate_t flags;
  int ret = OK;

  switch (code)
    {
      case FF_GAIN:
        {
          if (!test_bit(FF_GAIN, lower->ffbit) || value > 0xffffu)
            {
              break;
            }

          flags = enter_critical_section();
          lower->set_gain(lower, value);
          leave_critical_section(flags);
        }
        break;

      case FF_AUTOCENTER:
        {
          if (!test_bit(FF_AUTOCENTER, lower->ffbit) || value > 0xffffu)
            {
              break;
            }

          flags = enter_critical_section();
          lower->set_autocenter(lower, value);
          leave_critical_section(flags);
        }
        break;

      default:
        {
          if (ff_check_effect_access(lower->priv, code, NULL) == 0)
            {
              flags = enter_critical_section();
              ret = lower->playback(lower, code, value);
              leave_critical_section(flags);
            }
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: ff_register
 *
 * Description:
 *   This function registers a force feedback device, the upper half binds
 *   with hardware device through the lower half instance.
 *
 * Arguments:
 *   lower  - A pointer of lower half instance.
 *   path   - The path of force feedback device. such as "/dev/input_ff0".
 *   max_effects - Maximum number of effects supported by device.
 *
 * Return:
 *   OK if the driver was successfully registered; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int ff_register(FAR struct ff_lowerhalf_s *lower, FAR const char *path,
                int max_effects)
{
  FAR struct ff_upperhalf_s *upper;
  int ret;

  if (lower == NULL || max_effects == 0 || max_effects > FF_MAX_EFFECTS)
    {
      return -EINVAL;
    }

  upper = kmm_zalloc(sizeof(struct ff_upperhalf_s) +
                     sizeof(struct ff_effect_s) * max_effects);
  if (upper == NULL)
    {
      return -ENOMEM;
    }

  upper->lower       = lower;
  lower->priv        = upper;
  upper->max_effects = max_effects;
  upper->effects     = (FAR struct ff_effect_s *)(upper + 1);
  nxmutex_init(&upper->lock);

  ret = register_driver(path, &g_ff_fops, 0666, upper);
  if (ret < 0)
    {
      nxmutex_destroy(&upper->lock);
      kmm_free(upper);
    }

  return ret;
}

/****************************************************************************
 * Name: ff_unregister
 *
 * Description:
 *   This function is used to force feedback driver to unregister and
 *   release the occupied resources.
 *
 * Arguments:
 *   lower - A pointer to an insatnce of force feedback lower half driver.
 *   path  - The path of force feedback device. such as "/dev/input0"
 *
 ****************************************************************************/

void ff_unregister(FAR struct ff_lowerhalf_s *lower, FAR const char *path)
{
  FAR struct ff_upperhalf_s *upper;

  if (unregister_driver(path) < 0)
    {
      return;
    }

  if (lower->destroy != NULL)
    {
      lower->destroy(lower);
    }

  upper = lower->priv;
  nxmutex_destroy(&upper->lock);
  kmm_free(upper);
}
