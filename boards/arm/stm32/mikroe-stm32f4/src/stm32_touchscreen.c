/****************************************************************************
 * boards/arm/stm32/mikroe-stm32f4/src/stm32_touchscreen.c
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

#include <nuttx/irq.h>
#include <nuttx/board.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/fs.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/semaphore.h>

#include <arch/board/board.h>
#include "arm_internal.h"
#include "stm32_adc.h"
#include "stm32_gpio.h"
#include "mikroe-stm32f4.h"

#ifdef CONFIG_INPUT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Reference counting is partially implemented, but not needed in the current
 * design.
 */

#define CONFIG_TOUCHSCREEN_REFCNT
/* Should we try again on bad samples? */

#undef CONFIG_TOUCHSCREEN_RESAMPLE

/* TP uses ADC Channel #2 in a dedicated mode.  Ensure ADC2 not selected for
 * general use via the menuconfig
 */

#ifndef CONFIG_STM32_ADC2
#  error   Touchpanel Input (CONFIG_INPUT=y) requires enablinga ADC2 (CONFIG_STM32_ADC2=y)
#endif

/* Work queue support is required */

#ifndef CONFIG_SCHED_WORKQUEUE
#  warning Work queue support is required (CONFIG_SCHED_WORKQUEUE=y)
#endif

/* CONFIG_TOUCHSCREEN_THRESHX and CONFIG_TOUCHSCREEN_THRESHY
 *   Touchscreen data comes in a a very high rate.  New touch positions
 *   will only be reported when the X or Y data changes by these thresholds.
 *   This trades reduces data rate for some loss in dragging accuracy.  The
 *   touchscreen is configure for 12-bit values so the raw ranges are 0-4096.
 *   So for example, if your display is 320x240, then THRESHX=3 and THRESHY=4
 *   would correspond to one pixel.  Default: 4
 */

#ifndef CONFIG_TOUCHSCREEN_THRESHX
#  define CONFIG_TOUCHSCREEN_THRESHX 12
#endif

#ifndef CONFIG_TOUCHSCREEN_THRESHY
#  define CONFIG_TOUCHSCREEN_THRESHY 12
#endif

#ifndef CONFIG_TOUCHSCREEN_AVG_SAMPLES
#  define CONFIG_TOUCHSCREEN_AVG_SAMPLES 2
#endif

#ifndef CONFIG_TOUCHSCREEN_NPOLLWAITERS
#  define CONFIG_TOUCHSCREEN_NPOLLWAITERS 2
#endif

/* Driver support ***********************************************************/

/* This format is used to construct the /dev/input[n] device driver path.  It
 * is defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT   "/dev/input%d"
#define DEV_NAMELEN  16

/* Mikroe-STM32M4 Touchscreen Hardware Definitions **************************
 * PIN CONFIGURATIONS      SIGNAL NAME          ON-BOARD CONNECTIONS
 * --- ---------------------------------- -------------------- --------------
 *  35 PB0                 LCD-YD               YD Analog input
 *  36 PB1                 LCD-XL               XL Analog input
 *  95 PB8                 DRIVEA               Drives XR, XL and YU
 *  96 PB9                 DRIVEB               Drives YD
 */

#define LCD_YD_PIN           (0)
#define LCD_XL_PIN           (1)
#define LCD_YD_CHANNEL       (8)
#define LCD_XL_CHANNEL       (9)
#define LCD_DRIVEA_PIN       (8)
#define LCD_DRIVEB_PIN       (9)

#define LCD_DRIVEA_BIT       (1 << LCD_DRIVEA_PIN)
#define LCD_DRIVEB_BIT       (1 << LCD_DRIVEB_PIN)
#define LCD_SAMPX_BITS       (LCD_DRIVEA_BIT | (LCD_DRIVEB_BIT << 16))
#define LCD_SAMPY_BITS       (LCD_DRIVEB_BIT | (LCD_DRIVEA_BIT << 16))
#define LCD_TP_PORT_SETRESET  STM32_GPIOB_BSRR

#define TC_ADC_BASE           STM32_ADC2_BASE      /* ADC Channel base for TP */
#define ADC_CR1_ALLINTS       (ADC_CR1_AWDIE | ADC_CR1_EOCIE | ADC_CR1_JEOCIE)

/* Conversions are performed as 10-bit samples represented as 16-bit */

#define MAX_ADC               (4096)

/* A measured value has to be within this range to be considered */

#define UPPER_THRESHOLD       (MAX_ADC-1)
#define LOWER_THRESHOLD       (362)

/* Delays *******************************************************************/

/* All values will be increased by one system timer tick (probably 10MS). */

#define TC_PENUP_POLL_TICKS   MSEC2TICK(70)  /* IDLE polling rate: 70 MSec */
#define TC_PENDOWN_POLL_TICKS MSEC2TICK(40)  /* Active polling rate: 40 MSec */
#define TC_DEBOUNCE_TICKS     MSEC2TICK(16)  /* Delay before re-sampling: 16 MSec */
#define TC_SAMPLE_TICKS       MSEC2TICK(4)   /* Delay for A/D sampling: 4 MSec */
#define TC_SETTLE_TICKS       MSEC2TICK(10)  /* Delay for A/D settling: 10 MSec */
#define TC_RESAMPLE_TICKS     TC_SAMPLE_TICKS

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This enumeration describes the state of touchscreen state machine */

enum tc_state_e
{
  TC_READY = 0,                        /* Ready to begin next sample */
  TC_READY_SETTLE,                     /* Allowing time for Y DRIVE to settle */
  TC_YPENDOWN,                         /* Allowing time for the Y pen down sampling */
  TC_DEBOUNCE,                         /* Allowing a debounce time for the first sample */
  TC_RESAMPLE,                         /* Restart sampling on a bad measurement */
  TC_YSAMPLE,                          /* Allowing time for the Y sampling */
  TC_XSETTLE,                          /* Allowing time for the X to settle after changing DRIVE */
  TC_XSAMPLE,                          /* Allowing time for the X sampling */
  TC_XRESAMPLE,                        /* Allow time to resample X */
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
  uint8_t sampcount;                   /* Count of samples for average so far */
  uint8_t resamplecount;               /* Countdown to PENUP */
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

static void tc_adc_init(void);
static void tc_adc_start_sample(int pin);
static uint16_t tc_adc_read_sample(void);
static void tc_y_sample(void);
static void tc_x_sample(void);
static inline bool tc_valid_sample(uint16_t sample);

static void tc_notify(struct tc_dev_s *priv);
static int tc_sample(struct tc_dev_s *priv,
                     struct tc_sample_s *sample);
static int tc_waitsample(struct tc_dev_s *priv,
                         struct tc_sample_s *sample);
static void tc_worker(void *arg);

/* Character driver methods */

static int tc_open(struct file *filep);
static int tc_close(struct file *filep);
static ssize_t tc_read(struct file *filep, char *buffer, size_t len);
static int tc_ioctl(struct file *filep, int cmd, unsigned long arg);
static int tc_poll(struct file *filep, struct pollfd *fds, bool setup);

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
static bool g_touchinitdone = false;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc_adc_getreg
 *
 * Description:
 *   Read the value of an TC ADC channel (#2) register.
 *
 * Input Parameters:
 *   offset - The offset to the register to read
 *   value
 *
 * Returned Value:
 *
 ****************************************************************************/

static inline uint32_t tc_adc_getreg(int offset)
{
  return getreg32(TC_ADC_BASE + offset);
}

/****************************************************************************
 * Name: tc_adc_putreg
 *
 * Description:
 *   Set the value of an ADC register.
 *
 * Input Parameters:
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *
 ****************************************************************************/

static inline void tc_adc_putreg(int offset, uint32_t value)
{
  putreg32(value, TC_ADC_BASE + offset);
}

/****************************************************************************
 * Name: tc_adc_init
 *
 * Description:
 *   Initialize ADC Channel #2 for use with the touch panel.  The touch panel
 *   uses Channels 8 and 9 (PB0 and PB1) to read the X and Y axis touch
 *   positions.
 *
 ****************************************************************************/

static void tc_adc_init(void)
{
  irqstate_t flags;
  uint32_t regval;

  /* Do an rcc reset to reset the ADC peripheral */

  /* Disable interrupts.  This is necessary because the APB2RTSR register
   * is used by several different drivers.
   */

  flags = enter_critical_section();

  /* Enable  ADC reset state */

  regval = getreg32(STM32_RCC_APB2RSTR);
  regval |= RCC_APB2RSTR_ADCRST;
  putreg32(regval, STM32_RCC_APB2RSTR);

  /* Release ADC from reset state */

  regval &= ~RCC_APB2RSTR_ADCRST;
  putreg32(regval, STM32_RCC_APB2RSTR);

  /* Initialize the watchdog high threshold register */

  tc_adc_putreg(STM32_ADC_HTR_OFFSET, 0x00000fff);

  /* Initialize the watchdog low threshold register */

  tc_adc_putreg(STM32_ADC_LTR_OFFSET, 0x00000000);

  /* Initialize the same sample time for each ADC 55.5 cycles
   *
   * During sample cycles channel selection bits must remain unchanged.
   *
   *   000:   1.5 cycles
   *   001:   7.5 cycles
   *   010:  13.5 cycles
   *   011:  28.5 cycles
   *   100:  41.5 cycles
   *   101:  55.5 cycles
   *   110:  71.5 cycles
   *   111: 239.5 cycles
   */

  tc_adc_putreg(STM32_ADC_SMPR1_OFFSET, 0x00b6db6d);
  tc_adc_putreg(STM32_ADC_SMPR2_OFFSET, 0x00b6db6d);

  /* ADC CR1 Configuration */

  regval  = tc_adc_getreg(STM32_ADC_CR1_OFFSET);

  /* Initialize the Analog watchdog enable */

  regval &= ~ADC_CR1_AWDEN;
  regval |= (LCD_YD_CHANNEL << ADC_CR1_AWDCH_SHIFT);

  /* Enable interrupt flags */

  /* regval |= ADC_CR1_ALLINTS; */

  /* Disable Overrun interrupt */

  regval &= ~ADC_CR1_OVRIE;

  /* Set the resolution of the conversion.  We only need 10 bits. */

  regval |= ADC_CR1_RES_12BIT;

  tc_adc_putreg(STM32_ADC_CR1_OFFSET, regval);

  /* ADC CR2 Configuration */

  regval  = tc_adc_getreg(STM32_ADC_CR2_OFFSET);

  /* Clear CONT, continuous mode disable.  We will perform single
   * sampling on one channel at a time.
   */

  regval &= ~ADC_CR2_CONT;

  /* Set ALIGN (Right = 0) */

  regval &= ~ADC_CR2_ALIGN;

  /* External trigger disable.  We will do SW triggering */

  regval &= ~ADC_CR2_EXTEN_MASK;

  tc_adc_putreg(STM32_ADC_CR2_OFFSET, regval);

  /* Configuration of the channel conversion - start with Y sampling */

  regval = tc_adc_getreg(STM32_ADC_SQR3_OFFSET) & ADC_SQR3_RESERVED;
  regval |= LCD_YD_CHANNEL;
  tc_adc_putreg(STM32_ADC_SQR3_OFFSET, regval);

  /* Set the number of conversions = 1 */

  regval = tc_adc_getreg(STM32_ADC_SQR1_OFFSET) & ADC_SQR1_RESERVED;
  regval |= 0 << ADC_SQR1_L_SHIFT;
  tc_adc_putreg(STM32_ADC_SQR1_OFFSET, regval);

  /* ADC CCR configuration */

  regval  = getreg32(STM32_ADC_CCR);
  regval &= ~(ADC_CCR_MULTI_MASK | ADC_CCR_DELAY_MASK | ADC_CCR_DDS |
              ADC_CCR_DMA_MASK | ADC_CCR_ADCPRE_MASK | ADC_CCR_VBATEN |
              ADC_CCR_TSVREFE);
  regval |=  (ADC_CCR_MULTI_NONE | ADC_CCR_DMA_DISABLED |
              ADC_CCR_ADCPRE_DIV2);
  putreg32(regval, STM32_ADC_CCR);

  /* Set ADON to wake up the ADC from Power Down state. */

  regval  = tc_adc_getreg(STM32_ADC_CR2_OFFSET);
  regval |= ADC_CR2_ADON;
  tc_adc_putreg(STM32_ADC_CR2_OFFSET, regval);

  /* Restore the IRQ state */

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: tc_adc_start_sample
 *
 * Description:
 *   Perform A/D sampling.    Time must be allowed between the start of
 *   sampling and conversion (approx. 100Ms).
 *
 ****************************************************************************/

static void tc_adc_start_sample(int channel)
{
  uint32_t regval;

  /* Configure the specified channel for ADC conversion.  */

  regval = tc_adc_getreg(STM32_ADC_SQR3_OFFSET) & ADC_SQR3_RESERVED;
  regval |= channel;
  tc_adc_putreg(STM32_ADC_SQR3_OFFSET, regval);

  /* Configure the Watchdog for this channel */

  regval = tc_adc_getreg(STM32_ADC_CR1_OFFSET) & ADC_CR1_AWDCH_MASK;
  regval |= (channel << ADC_CR1_AWDCH_SHIFT);
  tc_adc_putreg(STM32_ADC_CR1_OFFSET, regval);

  /* Start the conversion */

  regval = tc_adc_getreg(STM32_ADC_CR2_OFFSET);
  regval |= ADC_CR2_SWSTART;
  tc_adc_putreg(STM32_ADC_CR2_OFFSET, regval);
}

/****************************************************************************
 * Name: tc_adc_read_sample
 *
 * Description:
 *   Begin A/D conversion.  Time must be allowed between the start of
 *   sampling and conversion (approx. 100Ms).
 *
 * Assumptions:
 * 1) All output pins configured as outputs:
 * 2) Appropriate pins are driven high and low
 *
 ****************************************************************************/

static uint16_t tc_adc_read_sample(void)
{
  uint16_t retval;
  uint32_t adcsr;
  uint16_t count = 0;

  /* Validate the conversion is complete */

  adcsr = tc_adc_getreg(STM32_ADC_SR_OFFSET);
  while ((adcsr & ADC_SR_EOC) == 0)
    {
      adcsr = tc_adc_getreg(STM32_ADC_SR_OFFSET);
      count++;
    }

  /* Read the sample */

  retval = tc_adc_getreg(STM32_ADC_DR_OFFSET);
  retval &= ADC_DR_RDATA_MASK;

  if (count > 0)
    {
      iinfo("Count = %d\n", count);
    }

  return retval;
}

/****************************************************************************
 * Name: tc_y_sample
 *
 * Description:
 *   Initiate sampling on Y
 *
 ****************************************************************************/

static void tc_y_sample(void)
{
  /* Start the Y axis sampling */

  tc_adc_start_sample(LCD_XL_CHANNEL);
}

/****************************************************************************
 * Name: tc_x_sample
 *
 * Description:
 *   Initiate sampling on X
 *
 ****************************************************************************/

static void tc_x_sample(void)
{
  /* Start the X axis sampling */

  tc_adc_start_sample(LCD_YD_CHANNEL);
}

/****************************************************************************
 * Name: tc_valid_sample
 ****************************************************************************/

static inline bool tc_valid_sample(uint16_t sample)
{
  return (sample > LOWER_THRESHOLD);
}

/****************************************************************************
 * Name: tc_notify
 ****************************************************************************/

static void tc_notify(struct tc_dev_s *priv)
{
  int i;

  /* If no threads have the driver open, then just dump the state */

#ifdef CONFIG_TOUCHSCREEN_REFCNT
  if ((priv->crefs == 0) && priv->sample.contact == CONTACT_UP)
    {
      priv->sample.contact = CONTACT_NONE;
      priv->sample.valid   = false;
      priv->id++;
      return;
    }
#endif

  /* If there are threads waiting on poll() for touchscreen data to become
   * available, then wake them up now.  NOTE: we wake up all waiting threads
   * because we do not know that they are going to do.  If they all try to
   * read the data, then some make end up blocking after all.
   */

  for (i = 0; i < CONFIG_TOUCHSCREEN_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          iinfo("Report events: %08" PRIx32 "\n", fds->revents);
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

static int tc_sample(struct tc_dev_s *priv,
                          struct tc_sample_s *sample)
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

static int tc_waitsample(struct tc_dev_s *priv,
                              struct tc_sample_s *sample)
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
   * the device structure.  We may have to wait here.  But we have our
   * sample. Interrupts and pre-emption will be re-enabled while we wait.
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

static void tc_worker(void *arg)
{
  struct tc_dev_s *priv = (struct tc_dev_s *)arg;
  uint32_t delay = TC_PENUP_POLL_TICKS;
  uint16_t value;
  uint16_t newx = 0;
  int16_t xdiff;
  int16_t ydiff;

  DEBUGASSERT(priv != NULL);

  /* Perform the next action based on the state of the conversions */

  switch (priv->state)
    {
    /* The touchscreen is IDLE and we are ready to begin the next sample */

    case TC_READY:
      {
        /* Select DRIVE for Y sampling */

        /* Configure XL, XR with drive voltages and disable YU drive.  Note
         * that this is configuring the DRIVEA and DRIVEB outputs to enable
         * the on-board transistor drive logic to energize the touch panel.
         */

        *((uint32_t *) LCD_TP_PORT_SETRESET) = LCD_SAMPY_BITS;

        /* Allow time for the Y DRIVE to settle */

        priv->resamplecount = 0;
        priv->sampcount = 0;
        priv->value = 0;
        priv->state = TC_READY_SETTLE;
        delay       = TC_SETTLE_TICKS;
      }
      break;

    case TC_READY_SETTLE:
      {
        /* Start Y sampling */

        tc_y_sample();

        /* Allow time for the Y pend down sampling */

        priv->state = TC_YPENDOWN;
        delay       = TC_SAMPLE_TICKS;
      }
      break;

    /* The Y sampling time has elapsed and the Y value should be ready
     * for conversion
     */

    case TC_YPENDOWN:
      {
        /* Convert the Y sample value */

        value = tc_adc_read_sample();

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
      {
        /* Select DRIVE for Y sampling */

        /* Configure XL, XR with drive voltages and disable YU drive.  Note
         * that this is configuring the DRIVEA and DRIVEB outputs to enable
         * the on-board transistor drive logic to energize the touch panel.
         */

        *((uint32_t *) LCD_TP_PORT_SETRESET) = LCD_SAMPY_BITS;

        /* Allow time for the Y DRIVE to settle */

        priv->state = TC_DEBOUNCE;
        delay       = TC_SETTLE_TICKS;
      }
      break;

    case TC_DEBOUNCE:
      {
        /* (Re-)start Y sampling */

        tc_y_sample();

        /* Allow time for the Y sampling */

        priv->state = TC_YSAMPLE;
        delay       = TC_SAMPLE_TICKS;
      }
      break;

    /* The Y sampling period has elapsed and we are ready to perform the
     * conversion.
     */

    case TC_YSAMPLE:                          /* Allowing time for the Y sampling */
      {
        /* Read the Y axis position */

        value = tc_adc_read_sample();

        /* A converted value at the minimum would mean that we lost the
         * contact before all of the conversions were completed.  At
         * converted value at the maximum value is probably bad too.
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
            value       = MAX_ADC - value;
            priv->value += value;
            if (++priv->sampcount < CONFIG_TOUCHSCREEN_AVG_SAMPLES)
              {
                priv->state = TC_READY_SETTLE;
                delay = 1;
                break;
              }

            priv->newy  = value / CONFIG_TOUCHSCREEN_AVG_SAMPLES;
            priv->value = 0;
            priv->sampcount = 0;
            iinfo("Y=%d\n", priv->newy);

            /* Configure YU and YD with drive voltages and disable XR drive.
             * Note that this is configuring the DRIVEA and DRIVEB outputs
             * to enable the on-board transistor drive logic to energize the
             * touch panel.
             */

            *((uint32_t *) LCD_TP_PORT_SETRESET) = LCD_SAMPX_BITS;

            /* Allow time for the X sampling */

            priv->state = TC_XSETTLE;
            delay       = TC_SETTLE_TICKS;
          }
      }
      break;

    case TC_XRESAMPLE:                /* Perform X resampling */
      {
        if (priv->resamplecount-- == 0)
          {
            priv->state = TC_PENUP;
            break;
          }
      }

    case TC_XSETTLE:                  /* Allowing time X to settle after changing DRIVE */
      {
        /* The X Drive settling time has elaspsed and it's time to start
         * the conversion
         */

        /* Start X sampling */

        tc_x_sample();

        /* Allow time for the X sampling */

        priv->state = TC_XSAMPLE;
        delay       = TC_SAMPLE_TICKS;
      }
      break;

    case TC_XSAMPLE:                 /* Allowing time for the X sampling */
      {
        /* Read the converted X axis position */

        value = tc_adc_read_sample();

        /* A converted value at the minimum would mean that we lost the
         * contact before all of the conversions were completed.  At
         * converted value at the maximum value is probably bad too.
         */

        if (!tc_valid_sample(value))
          {
#ifdef CONFIG_TOUCHSCREEN_RESAMPLE
            priv->state = TC_XRESAMPLE;
            if (priv->resamplecount == 0)
                priv->resamplecount = 1;
            delay       = TC_RESAMPLE_TICKS;
#else
            priv->state = TC_PENUP;
#endif
          }
        else
          {
            /* Calculate the X axis position */

            priv->value += value;
            if (++priv->sampcount < CONFIG_TOUCHSCREEN_AVG_SAMPLES)
              {
                priv->state = TC_XSETTLE;
                delay = 1;
                break;
              }

            newx  = value / CONFIG_TOUCHSCREEN_AVG_SAMPLES;
            iinfo("X=%d\n", newx);

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
       * already reported.  CONTACT_UP == pen up, but not reported)
       */

      if (priv->sample.contact != CONTACT_NONE &&
          priv->sample.contact != CONTACT_UP)
        {
          /* The pen is up.  We know from the above test, that this is a
           * loss of contact condition.  This will be changed to CONTACT_NONE
           * after the loss of contact is sampled.
           */

          priv->sample.contact = CONTACT_UP;

          /* Indicate the availability of new sample data for this ID */

          priv->sample.id = priv->id;
          priv->penchange = true;

          /* Notify any waiters that new touchscreen data is available */

          iinfo("1:X=%d, Y=%d\n", priv->sample.x, priv->sample.y);

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
       * been processed yet, then we have to ignore the pen down event (or
       * else it will look like a drag event)
       */

      if (priv->sample.contact != CONTACT_UP)
        {
          /* Perform a thresholding operation so that the results will be
           * more stable. If the difference from the last sample is small,
           * then ignore the event.
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

#ifdef CONFIG_LCD_LANDSCAPE
              priv->sample.x     = MAX_ADC - priv->newy;
              priv->sample.y     = newx;
#else
              priv->sample.x     = newx;
              priv->sample.y     = priv->newy;
#endif
              priv->sample.valid = true;

              /* If this is the first (acknowledged) penddown report, then
               * report this as the 1st contact. If contact == CONTACT_DOWN,
               * it will be set to set to CONTACT_MOVE after the contact is
               * first sampled.
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

              iinfo("2:X=%d, Y=%d\n", priv->sample.x, priv->sample.y);

              tc_notify(priv);
            }
        }

      /* Set up for the next poll */

      priv->state = TC_READY;
      delay       = TC_PENDOWN_POLL_TICKS;
    }

  /* Set up the next sample event */

  work_queue(HPWORK, &priv->work, tc_worker, priv, delay);
}

/****************************************************************************
 * Name: tc_open
 ****************************************************************************/

static int tc_open(struct file *filep)
{
#ifdef CONFIG_TOUCHSCREEN_REFCNT
  struct inode         *inode;
  struct tc_dev_s      *priv;
  uint8_t               tmp;
  int                   ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (struct tc_dev_s *)inode->i_private;

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

static int tc_close(struct file *filep)
{
#ifdef CONFIG_TOUCHSCREEN_REFCNT
  struct inode         *inode;
  struct tc_dev_s      *priv;
  int                   ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (struct tc_dev_s *)inode->i_private;

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

static ssize_t tc_read(struct file *filep, char *buffer, size_t len)
{
  struct inode          *inode;
  struct tc_dev_s       *priv;
  struct touch_sample_s *report;
  struct tc_sample_s    sample;
  int                   ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (struct tc_dev_s *)inode->i_private;

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

  report = (struct touch_sample_s *)buffer;
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

static int tc_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if 1
  iinfo("cmd: %d arg: %ld\n", cmd, arg);
  return -ENOTTY; /* None yet supported */
#else
  struct inode *inode;
  struct tc_dev_s *priv;
  int ret;

  iinfo("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (struct tc_dev_s *)inode->i_private;

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

static int tc_poll(struct file *filep, struct pollfd *fds,
                        bool setup)
{
  struct inode         *inode;
  struct tc_dev_s      *priv;
  int                   ret;
  int                   i;

  iinfo("setup: %d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (struct tc_dev_s *)inode->i_private;

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
          ierr("ERROR: Missing POLLIN: revents: %08" PRIx32 "\n",
               fds->revents);
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
 * Name: stm32_tsc_setup
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

int stm32_tsc_setup(int minor)
{
  struct tc_dev_s *priv;
  char devname[DEV_NAMELEN];
#ifdef CONFIG_TOUCHSCREEN_MULTIPLE
  irqstate_t flags;
#endif
  int ret;

  iinfo("minor: %d\n", minor);
  DEBUGASSERT(minor >= 0 && minor < 100);

  /* If we only have one touchscreen, check if we already did init */

#ifndef CONFIG_TOUCHSCREEN_MULTIPLE
  if (g_touchinitdone)
    {
      return OK;
    }
#endif

  /* Configure the touchscreen DRIVEA and DRIVEB pins for output */

  stm32_configgpio(GPIO_TP_DRIVEA);
  stm32_configgpio(GPIO_TP_DRIVEB);

  /* Configure Analog inputs for sampling X and Y coordinates */

  stm32_configgpio(GPIO_TP_XL);
  stm32_configgpio(GPIO_TP_YD);

  tc_adc_init();

  /* Create and initialize a touchscreen device driver instance */

#ifndef CONFIG_TOUCHSCREEN_MULTIPLE
  priv = &g_touchscreen;
#else
  priv = (struct tc_dev_s *)kmm_malloc(sizeof(struct tc_dev_s));
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

  snprintf(devname, sizeof(devname), DEV_FORMAT, minor);
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

#ifndef CONFIG_TOUCHSCREEN_MULTIPLE
  g_touchinitdone = true;
#endif

  return OK;

errout_with_priv:
  nxsem_destroy(&priv->devsem);
#ifdef CONFIG_TOUCHSCREEN_MULTIPLE
  kmm_free(priv);
#endif
  return ret;
}

#endif /* CONFIG_INPUT */
