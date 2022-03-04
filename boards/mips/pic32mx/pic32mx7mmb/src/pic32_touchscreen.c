/****************************************************************************
 * boards/mips/pic32mx/pic32mx7mmb/src/pic32_touchscreen.c
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

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/semaphore.h>

#include <arch/board/board.h>
#include "mips_arch.h"
#include "mips_internal.h"

#include "pic32mx.h"
#include "pic32mx_adc.h"
#include "pic32mx_ioport.h"
#include "pic32mx7mmb.h"

#ifdef CONFIG_INPUT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Reference counting is partially implemented, but not needed in the current
 * design.
 */

#undef CONFIG_TOUCHSCREEN_REFCNT

/* Should we try again on bad samples? */

#undef CONFIG_TOUCHSCREEN_RESAMPLE

/* Work queue support is required */

#ifndef CONFIG_SCHED_WORKQUEUE
#  warning "Work queue support is required (CONFIG_SCHED_WORKQUEUE=y)
#endif

/* CONFIG_TOUCHSCREEN_THRESHX and CONFIG_TOUCHSCREEN_THRESHY
 *   Touchscreen data comes in a a very high rate.  New touch positions
 *   will only be reported when the X or Y data changes by these thresholds.
 *   This trades reduces data rate for some loss in dragging accuracy.  The
 *   touchscreen is configure for 10-bit values so the raw ranges are 0-1023.
 *   So for example, if your display is 320x240, then THRESHX=3 and THRESHY=4
 *   would correspond to one pixel.  Default: 4
 */

#ifndef CONFIG_TOUCHSCREEN_THRESHX
#  define CONFIG_TOUCHSCREEN_THRESHX 4
#endif

#ifndef CONFIG_TOUCHSCREEN_THRESHY
#  define CONFIG_TOUCHSCREEN_THRESHY 4
#endif

/* Driver support ***********************************************************/

/* This format is used to construct the /dev/input[n] device driver path.
 * It is defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT   "/dev/input%d"
#define DEV_NAMELEN  16

/* PIC32MX7MMB Touchscreen Hardware Definitions *****************************/

/*   ----- ------ --------------------
 *   GPIO  ADC IN TFT Signal Name
 *   ----- ------ --------------------
 *   RB10   AN10   LCD-YD
 *   RB11   AN11   LCD-XR
 *   RB12   AN12   LCD-YU
 *   RB13   AN13   LCD-XL
 */

#define LCD_XPLUS_PIN  (11)
#define LCD_YPLUS_PIN  (12)
#define LCD_XMINUS_PIN (13)
#define LCD_YMINUS_PIN (10)

#define LCD_XPLUS_BIT  (1 << LCD_XPLUS_PIN)
#define LCD_YPLUS_BIT  (1 << LCD_YPLUS_PIN)
#define LCD_XMINUS_BIT (1 << LCD_XMINUS_PIN)
#define LCD_YMINUS_BIT (1 << LCD_YMINUS_PIN)
#define LCD_ALL_BITS   (LCD_XPLUS_BIT | LCD_YPLUS_BIT | LCD_XMINUS_BIT | LCD_YMINUS_BIT)

/* Conversions are performed as 10-bit samples represented as 16-bit
 * unsigned integers:
 */

#define MAX_ADC        (1023)

/* A measured value has to be within this range to be considered */

#define UPPER_THRESHOLD        (MAX_ADC-1)
#define LOWER_THRESHOLD        (1)

/* Delays *******************************************************************/

/* All values will be increased by one system timer tick (probably 10MS). */

#define TC_PENUP_POLL_TICKS   MSEC2TICK(100) /* IDLE polling rate: 100 MSec */
#define TC_PENDOWN_POLL_TICKS MSEC2TICK(60)  /* Active polling rate: 60 MSec */
#define TC_DEBOUNCE_TICKS     MSEC2TICK(30)  /* Delay before re-sampling: 30 MSec */
#define TC_SAMPLE_TICKS       MSEC2TICK(4)   /* Delay for A/D sampling: 4 MSec */
#define TC_RESAMPLE_TICKS     TC_SAMPLE_TICKS

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This enumeration describes the state of touchscreen state machine */

enum tc_state_e
{
  TC_READY = 0,                        /* Ready to begin next sample */
  TC_YMPENDOWN,                        /* Allowing time for the Y- pen down sampling */
  TC_DEBOUNCE,                         /* Allowing a debounce time for the first sample */
  TC_RESAMPLE,                         /* Restart sampling on a bad measurement */
  TC_YMSAMPLE,                         /* Allowing time for the Y- sampling */
  TC_YPSAMPLE,                         /* Allowing time for the Y+ sampling */
  TC_XPSAMPLE,                         /* Allowing time for the X+ sampling */
  TC_XMSAMPLE,                         /* Allowing time for the X- sampling */
  TC_PENDOWN,                          /* Conversion is complete -- pen down */
  TC_PENUP                             /* Conversion is complete -- pen up */
};

/* This describes the state of one contact */

enum tc_contact_e
{
  CONTACT_NONE = 0,                    /* No contact */
  CONTACT_DOWN,                        /* First contact */
  CONTACT_MOVE,                        /* Same contact, possibly different position */
  CONTACT_UP,                          /* Contact lost */
};

/* This structure describes the results of one touchscreen sample */

struct tc_sample_s
{
  uint8_t  id;                         /* Sampled touch point ID */
  uint8_t  contact;                    /* Contact state (see enum tc_contact_e) */
  bool     valid;                      /* True: x,y contain valid, sampled data */
  uint16_t x;                          /* Thresholded X position */
  uint16_t y;                          /* Thresholded Y position */
};

/* This structure describes the state of one touchscreen driver instance */

struct tc_dev_s
{
#ifdef CONFIG_TOUCHSCREEN_REFCNT
  uint8_t crefs;                       /* Number of times the device has been opened */
#endif
  uint8_t state;                       /* See enum tc_state_e */
  uint8_t nwaiters;                    /* Number of threads waiting for touchscreen data */
  uint8_t id;                          /* Current touch point ID */
  volatile bool penchange;             /* An unreported event is buffered */
  uint16_t value;                      /* Partial sample value (Y+ or X-) */
  uint16_t newy;                       /* New, un-thresholded Y value */
  sem_t devsem;                        /* Manages exclusive access to this structure */
  sem_t waitsem;                       /* Used to wait for the availability of data */
  struct tc_sample_s sample;           /* Last sampled touch point data */
  struct work_s work;                  /* Supports the state machine delayed processing */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

  struct pollfd *fds[CONFIG_TOUCHSCREEN_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void tc_adc_sample(int pin);
static uint16_t tc_adc_convert(void);
static void tc_yminus_sample(void);
static void tc_yplus_sample(void);
static void tc_xplus_sample(void);
static void tc_xminus_sample(void);
static inline bool tc_valid_sample(uint16_t sample);

static void tc_notify(FAR struct tc_dev_s *priv);
static int tc_sample(FAR struct tc_dev_s *priv,
                     FAR struct tc_sample_s *sample);
static int tc_waitsample(FAR struct tc_dev_s *priv,
                         FAR struct tc_sample_s *sample);
static void tc_worker(FAR void *arg);

/* Character driver methods */

static int tc_open(FAR struct file *filep);
static int tc_close(FAR struct file *filep);
static ssize_t tc_read(FAR struct file *filep, FAR char *buffer, size_t len);
static int tc_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int tc_poll(FAR struct file *filep, struct pollfd *fds, bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations tc_fops =
{
  tc_open,    /* open */
  tc_close,   /* close */
  tc_read,    /* read */
  NULL,       /* write */
  NULL,       /* seek */
  tc_ioctl,   /* ioctl */
  tc_poll     /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL      /* unlink */
#endif
};

/* If only a single touchscreen device is supported, then the driver state
 * structure may as well be pre-allocated.
 */

#ifndef CONFIG_TOUCHSCREEN_MULTIPLE
static struct tc_dev_s g_touchscreen;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc_adc_sample
 *
 * Description:
 *   Perform A/D sampling.
 *   Time must be allowed between the start of sampling
 *   and conversion (approx. 100Ms).
 *
 ****************************************************************************/

static void tc_adc_sample(int pin)
{
  /* Configure the pins for as an analog input. AD1PCFG specifies the
   * configuration of device pins to be used as analog inputs. A pin
   * is configured as an analog input when the corresponding PCFGn bit
   * is 0.
   */

  putreg32(ADC_CFG(pin), PIC32MX_ADC_CFGCLR);

  /* Set SAMP=0, format 16-bit unsigned integer, manual conversion,
   * SAMP=1 will trigger.
   */

  putreg32(0, PIC32MX_ADC_CON1);

  /* Select the pin as the MUXA CH0 input (positive) */

  putreg32(ADC_CHS_CH0SA(pin), PIC32MX_ADC_CHS);

  /* No input scan */

  putreg32(0, PIC32MX_ADC_CSSL);

  /* Manual sample, TAD = internal, 6 TPB */

  putreg32(ADC_CON3_ADCS(6) | ADC_CON3_SAMC(0), PIC32MX_ADC_CON3);

  /* No interrupts, no scan, internal voltage reference */

  putreg32(ADC_CON2_VCFG_AVDDAVSS, PIC32MX_ADC_CON2);

  /* Turn on the ADC */

  putreg32(ADC_CON1_ON, PIC32MX_ADC_CON1SET);

  /* Start sampling */

  putreg32(ADC_CON1_SAMP, PIC32MX_ADC_CON1SET);
}

/****************************************************************************
 * Name: tc_adc_convert
 *
 * Description:
 *   Begin A/D conversion.
 *   Time must be allowed between the start of sampling
 *   and conversion (approx. 100Ms).
 *
 * Assumptions:
 * 1) All output pins configured as outputs:
 * 2) Appropriate pins are driven high and low
 *
 ****************************************************************************/

static uint16_t tc_adc_convert(void)
{
  uint32_t retval;

  /* Start conversion */

  putreg32(ADC_CON1_SAMP, PIC32MX_ADC_CON1CLR);

  /* Wait for the conversion to complete */

  while ((getreg32(PIC32MX_ADC_CON1) & ADC_CON1_DONE) == 0);

  /* Then read the converted ADC value */

  retval = getreg32(PIC32MX_ADC_BUF0);

  /* Disable the ADC */

  putreg32(ADC_CON1_ON, PIC32MX_ADC_CON1CLR);

  /* Reset all pins to digital function */

  putreg32(LCD_ALL_BITS, PIC32MX_ADC_CFGSET);
  return (uint16_t)retval;
}

/****************************************************************************
 * Name: tc_yminus_sample
 *
 * Description:
 *   Initiate sampling on Y-
 *
 ****************************************************************************/

static void tc_yminus_sample(void)
{
  /* Configure X- as an input and X+, Y+, and Y- as outputs */

  putreg32(LCD_XPLUS_BIT | LCD_YPLUS_BIT | LCD_YMINUS_BIT,
           PIC32MX_IOPORTB_TRISCLR);
  putreg32(LCD_XMINUS_BIT, PIC32MX_IOPORTB_TRISSET);

  /* Energize the X plate: Y+ and Y- high, X+ low */

  putreg32(LCD_XPLUS_BIT, PIC32MX_IOPORTB_PORTCLR);
  putreg32(LCD_YPLUS_BIT | LCD_YMINUS_BIT, PIC32MX_IOPORTB_PORTSET);

  /* Start the Y axis sampling */

  tc_adc_sample(LCD_XMINUS_PIN);
}

/****************************************************************************
 * Name: tc_yplus_sample
 *
 * Description:
 *   Initiate sampling on Y+
 *
 ****************************************************************************/

static void tc_yplus_sample(void)
{
  /* Configure X+ as an input and X-, Y+, and Y- as outputs */

  putreg32(LCD_XMINUS_BIT | LCD_YPLUS_BIT | LCD_YMINUS_BIT,
           PIC32MX_IOPORTB_TRISCLR);
  putreg32(LCD_XPLUS_BIT, PIC32MX_IOPORTB_TRISSET);

  /* Energize the X plate: Y+ and Y- High, X- low (X+ is an input) */

  putreg32(LCD_XMINUS_BIT, PIC32MX_IOPORTB_PORTCLR);
  putreg32(LCD_YPLUS_BIT | LCD_YMINUS_BIT, PIC32MX_IOPORTB_PORTSET);

  /* Start the Y axis sampling */

  tc_adc_sample(LCD_XPLUS_PIN);
}

/****************************************************************************
 * Name: tc_xplus_sample
 *
 * Description:
 *   Initiate sampling on X+
 *
 ****************************************************************************/

static void tc_xplus_sample(void)
{
  /* Configure Y+ as an input and X+, X-, and Y- as outputs */

  putreg32(LCD_XPLUS_BIT | LCD_XMINUS_BIT | LCD_YMINUS_BIT,
           PIC32MX_IOPORTB_TRISCLR);
  putreg32(LCD_YPLUS_BIT, PIC32MX_IOPORTB_TRISSET);

  /* Energize the Y plate: X+ and X- high, Y- low (Y+ is an input) */

  putreg32(LCD_YMINUS_BIT, PIC32MX_IOPORTB_PORTCLR);
  putreg32(LCD_XPLUS_BIT | LCD_XMINUS_BIT, PIC32MX_IOPORTB_PORTSET);

  /* Read the X axis value */

  tc_adc_sample(LCD_YPLUS_PIN);
}

/****************************************************************************
 * Name: tc_xminus_sample
 *
 * Description:
 *   Initiate sampling on X-
 *
 ****************************************************************************/

static void tc_xminus_sample(void)
{
  /* Configure Y- as an input and X+, Y+, and X- as outputs */

  putreg32(LCD_XPLUS_BIT | LCD_XMINUS_BIT | LCD_YPLUS_BIT,
           PIC32MX_IOPORTB_TRISCLR);
  putreg32(LCD_YMINUS_BIT, PIC32MX_IOPORTB_TRISSET);

  /* Energize the Y plate: X+ and X- high, Y+ low (Y- is an input) */

  putreg32(LCD_YPLUS_BIT, PIC32MX_IOPORTB_PORTCLR);
  putreg32(LCD_XPLUS_BIT | LCD_XMINUS_BIT, PIC32MX_IOPORTB_PORTSET);

  /* Start X axis sampling */

  tc_adc_sample(LCD_YMINUS_PIN);
}

/****************************************************************************
 * Name: tc_valid_sample
 ****************************************************************************/

static inline bool tc_valid_sample(uint16_t sample)
{
  return (sample > LOWER_THRESHOLD /* && sample < UPPER_THRESHOLD */);
}

/****************************************************************************
 * Name: tc_notify
 ****************************************************************************/

static void tc_notify(FAR struct tc_dev_s *priv)
{
  int i;

  /* If there are threads waiting on poll() for touchscreen data to become
   * available, then wake them up now.  NOTE: we wake up all waiting threads
   * because we do not know that they are going to do.
   * If they all try to read the data, then some make end up blocking after
   * all.
   */

  for (i = 0; i < CONFIG_TOUCHSCREEN_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          iinfo("Report events: %02x\n", fds->revents);
          nxsem_post(fds->sem);
        }
    }

  /* If there are threads waiting for read data, then signal one of them
   * that the read data is available.
   */

  if (priv->nwaiters > 0)
    {
      /* After posting this semaphore, we need to exit because the
       * touchscreen is no longer available.
       */

      nxsem_post(&priv->waitsem);
    }
}

/****************************************************************************
 * Name: tc_sample
 *
 * Assumptions:  pre-emption is disabled
 *
 ****************************************************************************/

static int tc_sample(FAR struct tc_dev_s *priv,
                          FAR struct tc_sample_s *sample)
{
  int ret = -EAGAIN;

  /* Is there new touchscreen sample data available? */

  if (priv->penchange)
    {
      /* Yes.. the state has changed in some way.  Return a copy of the
       * sampled data.
       */

      memcpy(sample, &priv->sample, sizeof(struct tc_sample_s));

      /* Now manage state transitions */

      if (sample->contact == CONTACT_UP)
        {
          /* Next.. no contact.  Increment the ID so that next contact ID
           * will be unique.  X/Y positions are no longer valid.
           */

          priv->sample.contact = CONTACT_NONE;
          priv->sample.valid   = false;
          priv->id++;
        }
      else if (sample->contact == CONTACT_DOWN)
        {
          /* First report -- next report will be a movement */

          priv->sample.contact = CONTACT_MOVE;
        }

      priv->penchange = false;
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: tc_waitsample
 ****************************************************************************/

static int tc_waitsample(FAR struct tc_dev_s *priv,
                              FAR struct tc_sample_s *sample)
{
  int ret;

  /* Pre-emption must be disabled when this is called to prevent sampled
   * data from changing until it has been reported.
   */

  sched_lock();

  /* Now release the semaphore that manages mutually exclusive access to
   * the device structure.  This may cause other tasks to become ready to
   * run, but they cannot run yet because pre-emption is disabled.
   */

  nxsem_post(&priv->devsem);

  /* Try to get the a sample... if we cannot, then wait on the semaphore
   * that is posted when new sample data is available.
   */

  while (tc_sample(priv, sample) < 0)
    {
      /* Wait for a change in the touchscreen state */

      priv->nwaiters++;
      ret = nxsem_wait(&priv->waitsem);
      priv->nwaiters--;

      if (ret < 0)
        {
          goto errout;
        }
    }

  /* Re-acquire the semaphore that manages mutually exclusive access to
   * the device structure. We may have to wait here. But we have our sample.
   * Interrupts and pre-emption will be re-enabled while we wait.
   */

  ret = nxsem_wait(&priv->devsem);

errout:
  /* Restore pre-emption.  We might get suspended here but that is okay
   * because we already have our sample.  Note:  this means that if there
   * were two threads reading from the touchscreen for some reason, the data
   * might be read out of order.
   */

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: tc_worker
 ****************************************************************************/

static void tc_worker(FAR void *arg)
{
  FAR struct tc_dev_s *priv = (FAR struct tc_dev_s *)arg;
  uint32_t delay;
  uint16_t value;
  uint16_t newx;
  int16_t xdiff;
  int16_t ydiff;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Perform the next action based on the state of the conversions */

  switch (priv->state)
    {
    /* The touchscreen is IDLE and we are ready to begin the next sample */

    case TC_READY:
      {
        /* Start Y- sampling */

        tc_yminus_sample();

        /* Allow time for the Y- pend down sampling */

        priv->state = TC_YMPENDOWN;
        delay       = TC_SAMPLE_TICKS;
      }
      break;

    /* The Y- sampling time has elapsed and the Y- value should be ready
     * for conversion
     */

    case TC_YMPENDOWN:
      {
        /* Convert the Y- sample value */

        value = tc_adc_convert();

        /* A converted value at the minimum would mean that there is no touch
         * and that the sampling period is complete.
         */

        if (!tc_valid_sample(value))
          {
            priv->state = TC_PENUP;
          }
        else
          {
            /* Allow time for touch inputs to stabilize */

            priv->state = TC_DEBOUNCE;
            delay       = TC_DEBOUNCE_TICKS;
          }
      }
      break;

    /* The debounce time period has elapsed and we are ready to re-sample
     * the touchscreen.
     */

    case TC_RESAMPLE:
    case TC_DEBOUNCE:
      {
        /* (Re-)start Y- sampling */

        tc_yminus_sample();

        /* Allow time for the Y- sampling */

        priv->state = TC_YMSAMPLE;
        delay       = TC_SAMPLE_TICKS;
      }
      break;

    /* The Y- sampling period has elapsed and we are ready to perform the
     * conversion.
     */

    case TC_YMSAMPLE:
      {
        /* Convert and save the Y- sample value */

        value = tc_adc_convert();

        /* A converted value at the minimum would mean that there is no touch
         * and that the sampling period is complete.  At converted value at
         * the maximum value is probably bad too.
         */

        if (!tc_valid_sample(value))
          {
            priv->state = TC_PENUP;
          }
        else
          {
            /* Save the Y- sample and start Y+ sampling */

            priv->value = value;
            tc_yplus_sample();

            /* Allow time for the Y+ sampling */

            priv->state = TC_YPSAMPLE;
            delay       = TC_SAMPLE_TICKS;
          }
      }
      break;

    /* The Y+ sampling period has elapsed and we are ready to perform the
     * conversion.
     */

    case TC_YPSAMPLE:                         /* Allowing time for the Y+ sampling */
      {
        /* Read the Y+ axis position */

        value = tc_adc_convert();

        /* A converted value at the minimum would mean that we lost the
         * contact before all of the conversions were completed.
         *  At converted value at the maximum value is probably bad too.
         */

        if (!tc_valid_sample(value))
          {
#ifdef CONFIG_TOUCHSCREEN_RESAMPLE
            priv->state = TC_RESAMPLE;
            delay       = TC_RESAMPLE_TICKS;
#else
            priv->state = TC_PENUP;
#endif
          }
        else
          {
            value      = MAX_ADC - value;
            priv->newy = (value + priv->value) >> 1;
            iinfo("Y-=%d Y+=%d[%d] Y=%d\n", priv->value,
                   value, MAX_ADC - value, priv->newy);

            /* Start X+ sampling */

            tc_xplus_sample();

            /* Allow time for the X+ sampling */

            priv->state = TC_XPSAMPLE;
            delay       = TC_SAMPLE_TICKS;
          }
      }
      break;

    /* The X+ sampling period has elapsed and we are ready to perform the
     * conversion.
     */

    case TC_XPSAMPLE:
      {
        /* Convert the X+ sample value */

        value = tc_adc_convert();

        /* A converted value at the minimum would mean that we lost the
         * contact before all of the conversions were completed.
         * At converted value at the maximum value is probably bad too.
         */

        if (!tc_valid_sample(value))
          {
#ifdef CONFIG_TOUCHSCREEN_RESAMPLE
            priv->state = TC_RESAMPLE;
            delay       = TC_RESAMPLE_TICKS;
#else
            priv->state = TC_PENUP;
#endif
          }
        else
          {
            /* Save the X+ sample value */

            priv->value = value;

            /* Start X- sampling */

            tc_xminus_sample();

            /* Allow time for the X- pend down sampling */

            priv->state = TC_XMSAMPLE;
            delay       = TC_SAMPLE_TICKS;
          }
      }
      break;

    /* The X+ sampling period has elapsed and we are ready to perform the
     * conversion.
     */

    case TC_XMSAMPLE:                         /* Allowing time for the X- sampling */
      {
        /* Read the converted X- axis position */

        value = tc_adc_convert();

        /* A converted value at the minimum would mean that we lost the
         * contact before all of the conversions were completed.
         * At converted value at the maximum value is probably bad too.
         */

        if (!tc_valid_sample(value))
          {
#ifdef CONFIG_TOUCHSCREEN_RESAMPLE
            priv->state = TC_RESAMPLE;
            delay       = TC_RESAMPLE_TICKS;
#else
            priv->state = TC_PENUP;
#endif
          }
        else
          {
            /* Calculate the X- axis position */

            value = MAX_ADC - value;
            newx  = (value + priv->value) >> 1;
            iinfo("X+=%d X-=%d[%d] X=%d\n", priv->value,
                   value, MAX_ADC - value, newx);

            /* Samples are available */

            priv->state = TC_PENDOWN;
          }
      }
      break;
    }

  /* Check for terminal conditions.. */

  /* Check if the sampling resulted in a pen up decision.  If so, we need to
   * handle the change from pen down to pen up.
   */

  if (priv->state == TC_PENUP)
    {
      /* Ignore if the pen was already down (CONTACT_NONE == pen up and
       *  already reported.  CONTACT_UP == pen up, but not reported)
       */

      if (priv->sample.contact != CONTACT_NONE)
        {
          /* The pen is up.  We know from the above test, that this is a
           * loss of contact condition.  This will be changed to CONTACT_NONE
           * after the loss of contact is sampled.
           */

          priv->sample.contact = CONTACT_UP;

          /* Indicate the availability of new sample data for this ID */

          priv->sample.id = priv->id;
          priv->penchange = true;

          /* Notify any waiters that nes touchscreen data is available */

          tc_notify(priv);
        }

      /* Set up for the next poll */

      priv->sample.valid = false;
      priv->state        = TC_READY;
      delay              = TC_PENUP_POLL_TICKS;
    }

  /* Check if the sampling resulted in a pen down decision. */

  else if (priv->state == TC_PENDOWN)
    {
      /* It is a pen down event.  If the last loss-of-contact event has not
       * been processed yet, then we have to ignore the pen down event
       * (or else it will look like a drag event)
       */

      if (priv->sample.contact != CONTACT_UP)
        {
          /* Perform a thresholding operation so that the results will be
           * more stable.
           * If the difference from the last sample is small, then ignore
           * the event.
           */

          xdiff = (int16_t)priv->sample.x - (int16_t)newx;
          if (xdiff < 0)
            {
              xdiff = -xdiff;
            }

          ydiff = (int16_t)priv->sample.y - (int16_t)priv->newy;
          if (ydiff < 0)
            {
              ydiff = -ydiff;
            }

          if (xdiff >= CONFIG_TOUCHSCREEN_THRESHX ||
              ydiff >= CONFIG_TOUCHSCREEN_THRESHY)
            {
              /* There is some change above the threshold...
               * Report the change.
               */

              priv->sample.x     = newx;
              priv->sample.y     = priv->newy;
              priv->sample.valid = true;

              /* If this is the first (acknowledged) penddown report, then
               * report this as the first contact.
               * If contact == CONTACT_DOWN, it will be set to set to
               * CONTACT_MOVE after the contact is first sampled.
               */

              if (priv->sample.contact != CONTACT_MOVE)
                {
                  /* First contact */

                  priv->sample.contact = CONTACT_DOWN;
                }

              /* Indicate the availability of new sample data for this ID */

              priv->sample.id = priv->id;
              priv->penchange = true;

              /* Notify any waiters that nes touchscreen data is available */

              tc_notify(priv);
            }
        }

      /* Set up for the next poll */

      priv->state = TC_READY;
      delay       = TC_PENDOWN_POLL_TICKS;
    }

  /* Set up the next sample event */

  ret = work_queue(HPWORK, &priv->work, tc_worker, priv, delay);
  DEBUGASSERT(ret == 0);
}

/****************************************************************************
 * Name: tc_open
 ****************************************************************************/

static int tc_open(FAR struct file *filep)
{
#ifdef CONFIG_TOUCHSCREEN_REFCNT
  FAR struct inode         *inode;
  FAR struct tc_dev_s *priv;
  uint8_t                   tmp;
  int                       ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct tc_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Increment the reference count */

  tmp = priv->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_sem;
    }

  /* When the reference increments to 1, this is the first open event
   * on the driver.. and an opportunity to do any one-time initialization.
   */

  /* Save the new open count on success */

  priv->crefs = tmp;

errout_with_sem:
  nxsem_post(&priv->devsem);
  return ret;
#else
  return OK;
#endif
}

/****************************************************************************
 * Name: tc_close
 ****************************************************************************/

static int tc_close(FAR struct file *filep)
{
#ifdef CONFIG_TOUCHSCREEN_REFCNT
  FAR struct inode         *inode;
  FAR struct tc_dev_s *priv;
  int                       ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct tc_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the reference count unless it would decrement a negative
   * value.  When the count decrements to zero, there are no further
   * open references to the driver.
   */

  if (priv->crefs >= 1)
    {
      priv->crefs--;
    }

  nxsem_post(&priv->devsem);
#endif
  return OK;
}

/****************************************************************************
 * Name: tc_read
 ****************************************************************************/

static ssize_t tc_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode          *inode;
  FAR struct tc_dev_s  *priv;
  FAR struct touch_sample_s *report;
  struct tc_sample_s    sample;
  int                        ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct tc_dev_s *)inode->i_private;

  /* Verify that the caller has provided a buffer large enough to receive
   * the touch data.
   */

  if (len < SIZEOF_TOUCH_SAMPLE_S(1))
    {
      /* We could provide logic to break up a touch report into segments and
       * handle smaller reads... but why?
       */

      return -ENOSYS;
    }

  /* Get exclusive access to the driver data structure */

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Try to read sample data. */

  ret = tc_sample(priv, &sample);
  if (ret < 0)
    {
      /* Sample data is not available now.  We would ave to wait to get
       * receive sample data.  If the user has specified the O_NONBLOCK
       * option, then just return an error.
       */

      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          goto errout;
        }

      /* Wait for sample data */

      ret = tc_waitsample(priv, &sample);
      if (ret < 0)
        {
          /* We might have been awakened by a signal */

          goto errout;
        }
    }

  /* In any event, we now have sampled touchscreen data that we can report
   * to the caller.
   */

  report = (FAR struct touch_sample_s *)buffer;
  memset(report, 0, SIZEOF_TOUCH_SAMPLE_S(1));
  report->npoints            = 1;
  report->point[0].id        = sample.id;
  report->point[0].x         = sample.x;
  report->point[0].y         = sample.y;

  /* Report the appropriate flags */

  if (sample.contact == CONTACT_UP)
    {
      /* Pen is now up.  Is the positional data valid?  This is important to
       * know because the release will be sent to the window based on its
       * last positional data.
       */

      if (sample.valid)
        {
          report->point[0].flags  = TOUCH_UP | TOUCH_ID_VALID |
                                    TOUCH_POS_VALID | TOUCH_PRESSURE_VALID;
        }
      else
        {
          report->point[0].flags  = TOUCH_UP | TOUCH_ID_VALID;
        }
    }
  else
    {
      if (sample.contact == CONTACT_DOWN)
        {
          /* First contact */

          report->point[0].flags  = TOUCH_DOWN | TOUCH_ID_VALID |
                                    TOUCH_POS_VALID;
        }
      else /* if (sample->contact == CONTACT_MOVE) */
        {
          /* Movement of the same contact */

          report->point[0].flags  = TOUCH_MOVE | TOUCH_ID_VALID |
                                    TOUCH_POS_VALID;
        }
    }

  ret = SIZEOF_TOUCH_SAMPLE_S(1);

errout:
  nxsem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: tc_ioctl
 ****************************************************************************/

static int tc_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
#if 1
  iinfo("cmd: %d arg: %ld\n", cmd, arg);
  return -ENOTTY; /* None yet supported */
#else
  FAR struct inode         *inode;
  FAR struct tc_dev_s *priv;
  int                       ret;

  iinfo("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct tc_dev_s *)inode->i_private;

  /* Get exclusive access to the driver data structure */

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Process the IOCTL by command */

  switch (cmd)
    {
      /* ADD IOCTL COMMAND CASES HERE */

      default:
        ret = -ENOTTY;
        break;
    }

  nxsem_post(&priv->devsem);
  return ret;
#endif
}

/****************************************************************************
 * Name: tc_poll
 ****************************************************************************/

static int tc_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode         *inode;
  FAR struct tc_dev_s *priv;
  int                       ret;
  int                       i;

  iinfo("setup: %d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct tc_dev_s *)inode->i_private;

  /* Are we setting up the poll?  Or tearing it down? */

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ierr("ERROR: Missing POLLIN: revents: %08x\n", fds->revents);
          ret = -EDEADLK;
          goto errout;
        }

      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_TOUCHSCREEN_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_TOUCHSCREEN_NPOLLWAITERS)
        {
          ierr("ERROR: No available slot found: %d\n", i);
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? */

      if (priv->penchange)
        {
          tc_notify(priv);
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup */

      *slot                = NULL;
      fds->priv            = NULL;
    }

errout:
  nxsem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mx_tsc_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.  This function will register the driver as
 *   /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor   - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int pic32mx_tsc_setup(int minor)
{
  FAR struct tc_dev_s *priv;
  char devname[DEV_NAMELEN];
  int ret;

  iinfo("minor: %d\n", minor);
  DEBUGASSERT(minor >= 0 && minor < 100);

  /* Configure all touchscreen pins as inputs, undriven */

  putreg32(LCD_ALL_BITS, PIC32MX_IOPORTB_TRISSET);

  /* Configure all pins for as digital. AD1PCFG specifies the configuration
   * of device pins to be used as analog inputs. A pin is configured as an
   * analog input when the corresponding PCFGn bit is 0.
   */

  putreg32(LCD_ALL_BITS, PIC32MX_ADC_CFGSET);

  /* Create and initialize a touchscreen device driver instance */

#ifndef CONFIG_TOUCHSCREEN_MULTIPLE
  priv = &g_touchscreen;
#else
  priv = (FAR struct tc_dev_s *)kmm_malloc(sizeof(struct tc_dev_s));
  if (!priv)
    {
      ierr("ERROR: kmm_malloc(%d) failed\n", sizeof(struct tc_dev_s));
      return -ENOMEM;
    }
#endif

  /* Initialize the touchscreen device driver instance */

  memset(priv, 0, sizeof(struct tc_dev_s));
  nxsem_init(&priv->devsem,  0, 1); /* Initialize device structure semaphore */
  nxsem_init(&priv->waitsem, 0, 0); /* Initialize pen event wait semaphore */

  /* Register the device as an input device */

  snprintf(devname, DEV_NAMELEN, DEV_FORMAT, minor);
  iinfo("Registering %s\n", devname);

  ret = register_driver(devname, &tc_fops, 0666, priv);
  if (ret < 0)
    {
      ierr("ERROR: register_driver() failed: %d\n", ret);
      goto errout_with_priv;
    }

  /* Schedule work to perform the initial sampling and to set the data
   * availability conditions.
   */

  priv->state = TC_READY;
  ret = work_queue(HPWORK, &priv->work, tc_worker, priv, 0);
  if (ret != 0)
    {
      ierr("ERROR: Failed to queue work: %d\n", ret);
      goto errout_with_priv;
    }

  /* And return success (?) */

  return OK;

errout_with_priv:
  nxsem_destroy(&priv->devsem);
#ifdef CONFIG_TOUCHSCREEN_MULTIPLE
  kmm_free(priv);
#endif
  return ret;
}

#endif /* CONFIG_INPUT */
