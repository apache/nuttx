/****************************************************************************
 * drivers/input/ff_dummy.c
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

#include <debug.h>
#include <stdio.h>

#include <nuttx/bits.h>
#include <nuttx/kmalloc.h>
#include <nuttx/input/ff.h>

#define FF_DEVNAME_FMT      "/dev/lra%d"
#define FF_DEVNAME_MAX      32
#define FF_EFFECT_COUNT_MAX 5

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct ff_dummy_dev_s
{
  struct ff_lowerhalf_s lower;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ff_dummy_haptics_upload_effect(FAR struct ff_lowerhalf_s *lower,
                                          FAR struct ff_effect *effect,
                                          FAR struct ff_effect *old)
{
  iinfo("called: effect_id = %d \n", effect->id);
  return OK;
}

static int ff_dummy_haptics_playback(struct ff_lowerhalf_s *lower,
                                     int effect_id, int val)
{
  iinfo("called: effect_id = %d val = %d\n", effect_id, val);
  return OK;
}

static int ff_dummy_haptics_erase(FAR struct ff_lowerhalf_s *lower,
                                  int effect_id)
{
  iinfo("called: effect_id = %d\n", effect_id);
  return OK;
}

static void ff_dummy_haptics_set_gain(FAR struct ff_lowerhalf_s *lower,
                                      uint16_t gain)
{
  iinfo("called: gain = %d\n", gain);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ff_dummy_initialize
 *
 * Description:
 *   This function gengrate a vibrator node under /dev/ named lran. Which
 *   indicates a dummy vibrator device.
 *
 * Input Parameters:
 *   devno - The user specifies device number, from 0. If the
 *           devno alerady exists, -EEXIST will be returned.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ff_dummy_initialize(int devno)
{
  FAR struct ff_lowerhalf_s *lower;
  FAR struct ff_dummy_dev_s *dev;
  char path[FF_DEVNAME_MAX];
  int ret;

  dev = kmm_zalloc(sizeof(struct ff_dummy_dev_s));
  if (NULL == dev)
    {
      ierr("failed to alloc memory for ff dummy\n\n");
      return -ENOMEM;
    }

  lower           = &dev->lower;
  lower->upload   = ff_dummy_haptics_upload_effect;
  lower->playback = ff_dummy_haptics_playback;
  lower->set_gain = ff_dummy_haptics_set_gain;
  lower->erase    = ff_dummy_haptics_erase;

  /* set dummy device capabilities */

  set_bit(FF_CUSTOM, lower->ffbit);
  set_bit(FF_GAIN, lower->ffbit);
  set_bit(FF_CONSTANT, lower->ffbit);
  set_bit(FF_PERIODIC, lower->ffbit);

  snprintf(path, FF_DEVNAME_MAX, FF_DEVNAME_FMT, devno);
  ret = ff_register(lower, path, FF_EFFECT_COUNT_MAX);
  if (ret >= 0)
    {
      return ret;
    }

  ierr("Failed to register driver:%d\n", ret);
  kmm_free(dev);
  return ret;
}
