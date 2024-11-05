/****************************************************************************
 * drivers/input/goldfish_events.c
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/queue.h>
#include <nuttx/kmalloc.h>

#include <nuttx/input/goldfish_events.h>
#include <nuttx/input/mouse.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/input/keyboard.h>
#include <nuttx/input/kbd_codec.h>
#include <nuttx/input/virtio-input-event-codes.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#ifndef putreg32
#define putreg32(v, x) (*(FAR volatile uint32_t *)(x) = (v))
#endif

#ifndef getreg32
#define getreg32(x) (*(FAR volatile uint32_t *)(x))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum
{
  GOLDFISH_EVENTS_READ         = 0x00,    /* Read next event type, code or value. */
  GOLDFISH_EVENTS_SET_PAGE     = 0x00,    /* Set page index. */
  GOLDFISH_EVENTS_LEN          = 0x04,    /* Read length of page data. */
  GOLDFISH_EVENTS_DATA         = 0x08,    /* Read page data. */

  GOLDFISH_EVENTS_PAGE_NAME    = 0x0000,  /* Keyboard charmap name. */
  GOLDFISH_EVENTS_PAGE_EVBITS  = 0x10000, /* Event code supported sets. */
  GOLDFISH_EVENTS_PAGE_ABSDATA = 0x20003  /* (really 0x20000 + EV_ABS) EV_ABS min/max values. */
};

struct goldfish_input_event
{
  uint32_t type;
  uint32_t code;
  uint32_t value;
};

struct goldfish_events_s
{
  FAR void                      *base;
  int                           irq;
  struct work_s                 work;           /* Supports the interrupt handling "bottom half" */
  struct mouse_lowerhalf_s      mouselower;     /* Mouse device lowerhalf instance */
  struct mouse_report_s         mousesample;    /* Mouse event */
  struct keyboard_lowerhalf_s   keyboardlower;  /* Keyboard device lowerhalf instance */
  struct touch_lowerhalf_s      touchlower;     /* Touchpad device lowerhalf instance */
  struct touch_sample_s         touchsample;    /* Touchpad event */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int
goldfish_events_interrupt(int irq, FAR void *dev_id, FAR void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: goldfish_events_send_keyboard_event
 ****************************************************************************/

static bool
goldfish_events_send_keyboard_event(FAR struct goldfish_events_s *events,
                                    FAR struct goldfish_input_event *evt)
{
  if (evt->type == EV_KEY)
    {
      keyboard_event(&(events->keyboardlower),
                     keyboard_translate_virtio_code(evt->code),
                     !evt->value);
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: goldfish_events_send_mouse_event
 ****************************************************************************/

static bool
goldfish_events_send_mouse_event(FAR struct goldfish_events_s *events,
                                 FAR struct goldfish_input_event *evt)
{
  if (evt->type == EV_REL)
    {
      switch (evt->code)
        {
          case REL_X:
            events->mousesample.x = evt->value;
            return true;

          case REL_Y:
            events->mousesample.y = evt->value;
            return true;

        #ifdef CONFIG_INPUT_MOUSE_WHEEL
          case REL_WHEEL:
            events->mousesample.wheel = evt->value;
            return true;
        #endif
        }
    }
  else if (evt->type == EV_KEY && evt->value != 0)
    {
      switch (evt->code)
        {
          case BTN_LEFT:
            events->mousesample.buttons |= MOUSE_BUTTON_1;
            return true;

          case BTN_RIGHT:
            events->mousesample.buttons |= MOUSE_BUTTON_2;
            return true;

          case BTN_MIDDLE:
            events->mousesample.buttons |= MOUSE_BUTTON_3;
            return true;
        }
    }

  else if (evt->type == EV_SYN && evt->code == SYN_REPORT &&
           ((events->mousesample.x != 0) ||
           (events->mousesample.y != 0)
#ifdef CONFIG_INPUT_MOUSE_WHEEL
           || (events->mousesample.wheel != 0)
#endif
            ))
    {
      mouse_event(events->mouselower.priv, &events->mousesample);
      memset(&events->mousesample, 0, sizeof(events->mousesample));
    }

  return false;
}

/****************************************************************************
 * Name: goldfish_events_send_touch_event
 ****************************************************************************/

static bool
goldfish_events_send_touch_event(FAR struct goldfish_events_s *events,
                                 FAR struct goldfish_input_event *evt)
{
  if (evt->type == EV_ABS)
    {
      switch (evt->code)
        {
          case ABS_PRESSURE:
            events->touchsample.point[0].flags |= TOUCH_PRESSURE_VALID;
            events->touchsample.point[0].pressure = evt->value;
            return true;

          case ABS_X:
            events->touchsample.point[0].flags |= TOUCH_POS_VALID;
            events->touchsample.point[0].x = evt->value;
            return true;

          case ABS_Y:
            events->touchsample.point[0].flags |= TOUCH_POS_VALID;
            events->touchsample.point[0].y = evt->value;
            return true;
        }
    }
  else if (evt->type == EV_KEY)
    {
      if (evt->code == BTN_TOUCH)
        {
          if (evt->value)
            {
              events->touchsample.point[0].flags |= TOUCH_DOWN;
            }
          else
            {
              events->touchsample.point[0].flags |= TOUCH_UP;
            }

          return true;
        }
    }
  else if (evt->type == EV_SYN && evt->code == SYN_REPORT &&
           ((events->touchsample.point[0].pressure != 0) ||
           (events->touchsample.point[0].x != 0) ||
           (events->touchsample.point[0].y != 0)))
    {
      events->touchsample.npoints = 1;
      events->touchsample.point[0].timestamp = touch_get_time();

      touch_event(events->touchlower.priv, &events->touchsample);
      memset(&events->touchsample, 0, sizeof(events->touchsample));
    }

  return false;
}

/****************************************************************************
 * Name: goldfish_events_worker
 ****************************************************************************/

static void goldfish_events_worker(FAR void *arg)
{
  FAR struct goldfish_events_s *events = (FAR struct goldfish_events_s *)arg;
  struct goldfish_input_event evt;

  up_enable_irq(events->irq);

  evt.type = getreg32(events->base + GOLDFISH_EVENTS_READ);
  putreg32(GOLDFISH_EVENTS_PAGE_ABSDATA,
           events->base + GOLDFISH_EVENTS_SET_PAGE);
  evt.code = getreg32(events->base + GOLDFISH_EVENTS_READ);
  putreg32(GOLDFISH_EVENTS_PAGE_ABSDATA,
           events->base + GOLDFISH_EVENTS_SET_PAGE);
  evt.value = getreg32(events->base + GOLDFISH_EVENTS_READ);
  putreg32(GOLDFISH_EVENTS_PAGE_ABSDATA,
           events->base + GOLDFISH_EVENTS_SET_PAGE);

  iinfo("goldfish_events_interrupt events(%" PRIu32 ", %" PRIu32 ", \
        %" PRIu32 ").\n", evt.type, evt.code, evt.value);

  if (goldfish_events_send_touch_event(events, &evt))
    {
      return;
    }

  if (goldfish_events_send_mouse_event(events, &evt))
    {
      return;
    }

  goldfish_events_send_keyboard_event(events, &evt);
}

/****************************************************************************
 * Name: goldfish_events_interrupt
 ****************************************************************************/

static int
goldfish_events_interrupt(int irq, FAR void *dev_id, FAR void *arg)
{
  FAR struct goldfish_events_s *events = arg;

  work_queue(HPWORK, &events->work, goldfish_events_worker, events, 0);
  up_disable_irq(events->irq);

  return 0;
}

/****************************************************************************
 * Name: goldfish_drivers_register
 ****************************************************************************/

static void goldfish_drivers_register(FAR struct goldfish_events_s *events)
{
  putreg32(GOLDFISH_EVENTS_PAGE_EVBITS | EV_KEY,
           events->base + GOLDFISH_EVENTS_SET_PAGE);
  if (getreg32(events->base + GOLDFISH_EVENTS_LEN))
    {
      putreg32(GOLDFISH_EVENTS_PAGE_ABSDATA,
               events->base + GOLDFISH_EVENTS_SET_PAGE);
      if (getreg32(events->base + GOLDFISH_EVENTS_LEN))
        {
          keyboard_register(&(events->keyboardlower),
                            "/dev/kbd0",
                            CONFIG_INPUT_GOLDFISH_NBUFFER);
        }
    }

  putreg32(GOLDFISH_EVENTS_PAGE_EVBITS | EV_REL,
           events->base + GOLDFISH_EVENTS_SET_PAGE);
  if (getreg32(events->base + GOLDFISH_EVENTS_LEN))
    {
      putreg32(GOLDFISH_EVENTS_PAGE_ABSDATA,
               events->base + GOLDFISH_EVENTS_SET_PAGE);
      if (getreg32(events->base + GOLDFISH_EVENTS_LEN))
        {
          mouse_register(&(events->mouselower),
                         "/dev/mouse0",
                         CONFIG_INPUT_GOLDFISH_NBUFFER);
        }
    }

  putreg32(GOLDFISH_EVENTS_PAGE_EVBITS | EV_ABS,
           events->base + GOLDFISH_EVENTS_SET_PAGE);
  if (getreg32(events->base + GOLDFISH_EVENTS_LEN))
    {
      putreg32(GOLDFISH_EVENTS_PAGE_ABSDATA,
               events->base + GOLDFISH_EVENTS_SET_PAGE);
      if (getreg32(events->base + GOLDFISH_EVENTS_LEN))
        {
          touch_register(&(events->touchlower),
                         "/dev/input0",
                         CONFIG_INPUT_GOLDFISH_NBUFFER);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: goldfish_events_register
 ****************************************************************************/

int goldfish_events_register(FAR void *base, int irq)
{
  FAR struct goldfish_events_s *events;
  int ret;

  events = (FAR struct goldfish_events_s *)kmm_zalloc(sizeof(*events));
  if (events == NULL)
    {
      return -ENOMEM;
    }

  events->base = base;
  events->irq = irq;

  ret = irq_attach(events->irq, goldfish_events_interrupt, events);
  if (ret < 0)
    {
      goto err_free_events;
    }

  /* Register lower half drivers */

  goldfish_drivers_register(events);
  up_enable_irq(events->irq);

  return 0;

err_free_events:
  kmm_free(events);
  return ret;
}
