/****************************************************************************
 * include/nuttx/input/ff.h
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

#ifndef __INCLUDE_NUTTX_INPUT_FF_H
#define __INCLUDE_NUTTX_INPUT_FF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/bits.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Force feedback effect types. */

#define FF_RUMBLE         0x00
#define FF_PERIODIC       0x01
#define FF_CONSTANT       0x02
#define FF_SPRING         0x03
#define FF_FRICTION       0x04
#define FF_DAMPER         0x05
#define FF_INERTIA        0x06
#define FF_RAMP           0x07

#define FF_EFFECT_MIN     FF_RUMBLE
#define FF_EFFECT_MAX     FF_RAMP

/* Force feedback periodic effect types. */

#define FF_SQUARE         0x08
#define FF_TRIANGLE       0x09
#define FF_SINE           0x0a
#define FF_SAW_UP         0x0b
#define FF_SAW_DOWN       0x0c
#define FF_CUSTOM         0x0d

#define FF_WAVEFORM_MIN   FF_SQUARE
#define FF_WAVEFORM_MAX   FF_CUSTOM

/* Set ff device properties. */

#define FF_GAIN           0x0e
#define FF_AUTOCENTER     0x0f

/* Total number of effects should never exceed FF_MAX_EFFECTS. */

#define FF_MAX_EFFECTS    FF_GAIN

#define FF_MAX            0x7f
#define FF_CNT            (FF_MAX + 1)

/* Values describing the status of a force-feedback effect. */

#define FF_STATUS_STOPPED 0x00
#define FF_STATUS_PLAYING 0x01
#define FF_STATUS_MAX     0x01

/* IOCTL commands unique to the force feedback device */

/* This cmd use to querying device capabilities.
 * Arg: pointer to address of
 *      "unsigned long features[BITS_TO_LONGS(FF_CNT)]".
 */

#define EVIOCGBIT         _FFIOC(0)

/* This cmd use to send a force effect to a force feedback device.
 * Arg: pointer to address of struct ff_effect.
 */

#define EVIOCSFF          _FFIOC(1)

/* This cmd use to erase a force effect.
 * Arg: int value, the effect id.
 */

#define EVIOCRMFF         _FFIOC(2)

/* This cmd use to report number of effects playable at the same time.
 * Arg: pointer to address of int value, return the number of effects.
 */

#define EVIOCGEFFECTS     _FFIOC(3)

/* This cmd use to calibrate the device and return the calibration value.
 * Arg: pointer to address of integer array value, return the calibration
 * value.
 */

#define EVIOCCALIBRATE    _FFIOC(4)

/* This cmd use to set calibration value for the device.
 * Arg: pointer to address of the calibration value which should be set.
 */

#define EVIOCSETCALIBDATA _FFIOC(5)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Structures used in ioctls to upload effects to a device
 * They are pieces of a bigger structure (called ff_effect).
 */

/* All duration values are expressed in ms. Values above 32767 ms (0x7fff)
 * should not be used and have unspecified results.
 */

/* struct ff_replay - defines scheduling of the force-feedback effect */

struct ff_replay
{
  /* Duration of the effect */

  uint16_t length;

  /* Delay before effect should start playing */

  uint16_t delay;
};

/* struct ff_trigger - defines what triggers the force-feedback effect */

struct ff_trigger
{
  /* Number of the button triggering the effect */

  uint16_t button;

  /* Controls how soon the effect can be re-triggered */

  uint16_t interval;
};

/* struct ff_envelope - generic force-feedback effect envelope
 *
 * The attack_level and fade_level are absolute values; when applying
 * envelope force-feedback core will convert to positive/negative
 * value based on polarity of the default level of the effect.
 * Valid range for the attack and fade levels is 0x0000 - 0x7fff
 */

struct ff_envelope
{
  /* Duration of the attack (ms). */

  uint16_t attack_length;

  /* Level at the beginning of the attack. */

  uint16_t attack_level;

  /* Duration of fade (ms). */

  uint16_t fade_length;

  /* Level at the end of fade. */

  uint16_t fade_level;
};

/* struct ff_constant_effect - defines parameters of a constant
 * force-feedback effect.
 */

struct ff_constant_effect
{
  /* Strength of the effect; may be negative. */

  int16_t level;

  /* Envelope data. */

  struct ff_envelope envelope;
};

/* struct ff_ramp_effect - defines parameters of a ramp force-feedback
 * effect.
 */

struct ff_ramp_effect
{
  /* Beginning strength of the effect; may be negative. */

  int16_t start_level;

  /* Final strength of the effect; may be negative. */

  int16_t end_level;

  /* Envelope data. */

  struct ff_envelope envelope;
};

/* struct ff_condition_effect - defines a spring or friction force-feedback
 * effect
 */

struct ff_condition_effect
{
  /* Maximum level when joystick moved all way to the right. */

  uint16_t right_saturation;

  /* Same for the left side. */

  uint16_t left_saturation;

  /* Controls how fast the force grows when the joystick moves
   * to the right.
   */

  int16_t right_coeff;

  /* Same for the left side */

  int16_t left_coeff;

  /* Size of the dead zone, where no force is produced */

  uint16_t deadband;

  /* Position of the dead zone */

  int16_t center;
};

/* struct ff_periodic_effect - defines parameters of a periodic
 * force-feedback effect.
 *
 * Known waveforms - FF_SQUARE, FF_TRIANGLE, FF_SINE, FF_SAW_UP,
 * FF_SAW_DOWN, FF_CUSTOM. The exact syntax FF_CUSTOM is undefined
 * for the time being as no driver supports it yet.
 *
 * Note: the data pointed by custom_data is copied by the driver.
 * You can therefore dispose of the memory after the upload/update.
 */

struct ff_periodic_effect
{
  /* Kind of the effect (wave). */

  uint16_t waveform;

  /* Period of the wave (ms). */

  uint16_t period;

  /* Peak value. */

  int16_t  magnitude;

  /* Mean value of the wave (roughly). */

  int16_t  offset;

  /* 'horizontal' shift. */

  uint16_t phase;

  /* Envelope data. */

  struct ff_envelope envelope;

  /* Number of samples (FF_CUSTOM only). */

  uint32_t custom_len;

  /* Buffer of samples (FF_CUSTOM only). */

  FAR int16_t *custom_data;
};

/* struct ff_rumble_effect - defines parameters of a periodic
 * force-feedback effect
 *
 * Some rumble pads have two motors of different weight. Strong_magnitude
 * represents the magnitude of the vibration generated by the heavy one.
 */

struct ff_rumble_effect
{
  /* Magnitude of the heavy motor. */

  uint16_t strong_magnitude;

  /* Magnitude of the light one */

  uint16_t weak_magnitude;
};

/* struct ff_effect - defines force feedback effect
 * This structure is sent through ioctl from the application to the driver.
 * To create a new effect application should set its @id to -1; the kernel
 * will return assigned @id which can later be used to update or delete
 * this effect.
 */

struct ff_effect
{
  /* Type of the effect (FF_CONSTANT, FF_PERIODIC, FF_RAMP, FF_SPRING,
   * FF_FRICTION, FF_DAMPER, FF_RUMBLE, FF_INERTIA, or FF_CUSTOM).
   */

  uint16_t type;

  /* An unique id assigned to an effect. */

  int16_t  id;

  /* Direction of the effect is encoded as follows:
   *      0 deg -> 0x0000 (down)
   *      90 deg -> 0x4000 (left)
   *      180 deg -> 0x8000 (up)
   *      270 deg -> 0xC000 (right)
   */

  uint16_t direction;

  /* Trigger conditions (struct ff_trigger). */

  struct ff_trigger trigger;

  /* Scheduling of the effect (struct ff_replay). */

  struct ff_replay  replay;

  /* Effect-specific structure (one of ff_constant_effect,
   * ff_ramp_effect, ff_periodic_effect, ff_rumble_effect) further
   * defining effect parameters.
   */

  union
    {
      struct ff_constant_effect constant;
      struct ff_ramp_effect ramp;
      struct ff_periodic_effect periodic;
      struct ff_condition_effect condition[2];
      struct ff_rumble_effect rumble;
    } u;
};

/* The structure is used to description the event by userspace specified. */

struct ff_event_s
{
  uint32_t code;  /* Event code, eg: FF_GAIN, FF_AUTOCENTER, effect id */
  int value;      /* Event value corresponding to Event code */
};

/* struct ff_lowerhalf_s - force-feedback device lower half driver
 *
 * Every force-feedback device must implement upload() and playback()
 * methods; erase() is optional. set_gain() and set_autocenter() need
 * only be implemented if driver sets up FF_GAIN and FF_AUTOCENTER
 * bits.
 *
 * Note that playback(), set_gain() and set_autocenter() are called
 * with lock held and interrupts off and thus may not sleep.
 */

struct ff_lowerhalf_s
{
  /* Called to upload an new effect into device. */

  CODE int (*upload)(FAR struct ff_lowerhalf_s *lower,
                     FAR struct ff_effect *effect,
                     FAR struct ff_effect *old);

  /* Called to erase an effect from device. */

  CODE int (*erase)(FAR struct ff_lowerhalf_s *lower, int effect_id);

  /* Called to request device to start playing specified effect. */

  CODE int (*playback)(FAR struct ff_lowerhalf_s *lower, int effect_id,
                       int value);

  /* Called to set specified gain. */

  CODE void (*set_gain)(FAR struct ff_lowerhalf_s *lower, uint16_t gain);

  /* Called to auto-center device. */

  CODE void (*set_autocenter)(FAR struct ff_lowerhalf_s *lower,
                              uint16_t magnitude);

  /* Called by ff upper half when device is being destroyed. */

  CODE void (*destroy)(FAR struct ff_lowerhalf_s *lower);

  /* The calibration value to be written in or the non-volatile memory of the
   * device or dedicated registers. At each power-on, so that the values read
   * from the device are already corrected. When the device is calibrated,
   * the absolute accuracy will be better than before.
   * Note: the parameters associated with calibration value, maxinum 32-byte.
   */

  CODE int (*set_calibvalue)(FAR struct ff_lowerhalf_s *lower,
                             unsigned long arg);

  /* This operation can trigger the calibration operation, and if the
   * calibration operation is short-lived, the calibration result value can
   * be obtained at the same time, the calibration value to be written in
   * the non-volatile memory of the device or dedicated registers. When the
   * upper-level application calibration is completed, the current
   * calibration value of the device needs to be obtained and backed up,
   * so that the last calibration value can be directly obtained after
   * power-on.
   * Note: the parameters associated with calibration value, maxinum 32-byte.
   */

  CODE int (*calibrate)(FAR struct ff_lowerhalf_s *lower,
                        unsigned long arg);

  /* The bitmap of force feedback capabilities truly supported by device */

  unsigned long ffbit[BITS_TO_LONGS(FF_CNT)];

  /* The private opaque pointer to be passed to upper-layer during callback */

  FAR void *priv;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ff_event
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
 * Return:
 *   OK if the driver was successfully process event; A negated errno value
 *   is returned on any failure.
 *
 ****************************************************************************/

int ff_event(FAR struct ff_lowerhalf_s *lower, uint32_t code, int value);

/****************************************************************************
 * Name: ff_register
 *
 * Description:
 *   This function registers a force feedback device, the upper half binds
 *   with hardware device through the lower half instance.
 *
 * Arguments:
 *   lower  - A pointer of lower half instance.
 *   path   - The path of force feedback device. such as "/dev/input0".
 *   max_effects - Maximum number of effects supported by device.
 *
 * Return:
 *   OK if the driver was successfully registered; A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int ff_register(FAR struct ff_lowerhalf_s *lower, FAR const char *path,
                int max_effects);

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

void ff_unregister(FAR struct ff_lowerhalf_s *lower, FAR const char *path);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_INPUT_FF_H */
