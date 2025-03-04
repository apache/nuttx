/****************************************************************************
 * drivers/analog/hx711.c
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
#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>
#include <nuttx/irq.h>
#include <sys/param.h>

#include <ctype.h>
#include <stdio.h>

#include <nuttx/analog/hx711.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVNAME_FMT "/dev/hx711_%d"
#define DEVNAME_FMTLEN (11 + 3 + 1)

/* hx711 is a 24 bit ADC, but in case they decide to do like a
 * hx771s(uperb) with 32 bit resolution, here is easy to change def
 */

#define HX711_BITS_PER_READ 24

#define HX711_TARE_MAX_LOOP 64
#define HX711_TARE_NSAMPLES 5

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct hx711_dev_s
{
  FAR struct hx711_lower_s *lower;
  mutex_t excl;
  sem_t hx711_ready;
  int crefs;
  int unlinked;
  unsigned char minor;

  int val_per_unit;
  long tare;
  unsigned char average;
  unsigned char gain;
  char channel;
  signed char sign;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int hx711_open(FAR struct file *filep);
static int hx711_close(FAR struct file *filep);
static int hx711_unlink(FAR struct inode *inode);
static int hx711_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static ssize_t hx711_read(FAR struct file *filep,
                          FAR char *buf, size_t buflen);
static int32_t hx711_single_read(FAR struct hx711_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_hx711_fops =
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  .unlink = hx711_unlink,
#endif /* CONFIG_DISABLE_PSEUDOFS_OPERATIONS */
  .open   = hx711_open,
  .close  = hx711_close,
  .read   = hx711_read,
  .ioctl  = hx711_ioctl
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hx711_tare
 *
 * Description:
 *   Tares the scale. Function will read some number of samples and will
 *   check if these readings are stable (more or less the same within
 *   specified precision). If operation is a success, next call to read()
 *   will return value close to 0 (if no force is applied to tensometer).
 *
 * Input Parameters:
 *   dev - hx711 instance to tare
 *   precision - precision with which to tare the scale. If set to 100
 *               function will set new tare if min-max values read are
 *               less than 100
 *
 * Returned Value:
 *   OK - on success
 *   -EIO - no communication with the hx711
 *   -ETIME - scale was not stable for HX711_TARE_NSAMPLES loops
 *
 ****************************************************************************/

static int hx711_tare(FAR struct hx711_dev_s *dev, float precision)
{
  int32_t samples[HX711_TARE_NSAMPLES];
  int i;
  int j;
  int min;
  int max;
  long tare;
  int prec;
  long taresave;
  signed char signsave;

  /* If value per unit is defined, we assume precision is specified
   * in units, calculate raw value for precision
   */

  prec = dev->val_per_unit > 0 ? precision * dev->val_per_unit : precision;

  /* Save old tare value and sign, which we will restore when we
   * have an error
   */

  taresave = dev->tare;
  signsave = dev->sign;

  /* Reset tare value and sign during taring */

  dev->tare = 0;
  dev->sign = 1;

  for (i = 0; i != HX711_TARE_NSAMPLES; i++)
    {
      samples[i] = hx711_single_read(dev);
      if (samples[i] == INT32_MIN)
        {
          dev->tare = taresave;
          dev->sign = signsave;
          return -EIO;
        }
    }

  for (i = 0; i != HX711_TARE_MAX_LOOP; i++)
    {
      /* Check if scale reading is stable */

      min = INT_MAX;
      max = INT_MIN;
      for (j = 0; j != HX711_TARE_NSAMPLES; j++)
        {
          min = samples[j] < min ? samples[j] : min;
          max = samples[j] > max ? samples[j] : max;
        }

      if (max - min <= prec)
        {
          /* Scale readings are stable within specified precision.
           * Use average of these readings to set new tare value.
           */

          for (tare = j = 0; j != HX711_TARE_NSAMPLES; j++)
            {
              tare += samples[j];
            }

          tare /= HX711_TARE_NSAMPLES;
          dev->tare = tare;
          dev->sign = signsave;
          return OK;
        }

      /* Reading is not yet stable, perform next read and check
       * stability again
       */

      samples[i % HX711_TARE_NSAMPLES] = hx711_single_read(dev);
      if (samples[i % HX711_TARE_NSAMPLES] == INT32_MIN)
        {
          dev->tare = taresave;
          dev->sign = signsave;
          return -EIO;
        }
    }

  /* If we get here, we couldn't get stable readings within specified
   * limit
   */

  dev->tare = taresave;
  dev->sign = signsave;
  return -ETIME;
}

/****************************************************************************
 * Name: hx711_ioctl
 *
 * Description:
 *   Perform device specific operations.
 *
 * Input Parameters:
 *   filep - file on vfs associated with the driver.
 *   cmd - command to perform
 *   arg - argument for the cmd
 *
 * Returned Value:
 *   Returns OK on success or negated errno on failure.
 *
 ****************************************************************************/

static int hx711_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct hx711_dev_s *dev;
  int ret;

  dev = filep->f_inode->i_private;

  /* Get exclusive access to the hx711 driver state */

  ret = nxmutex_lock(&dev->excl);
  if (ret < 0)
    {
      return ret;
    }

  ret = OK;
  switch (cmd)
    {
    case HX711_SET_AVERAGE:
      if (arg < 1 || arg > HX711_MAX_AVG_SAMPLES)
        {
          /* Averaging more than HX711_MAX_AVG_SAMPLES samples could
           * overflow averaging variable leading to invalid reading.
           */

          ret = -EINVAL;
          break;
        }

      dev->average = arg;
      break;

    case HX711_SET_CHANNEL:
      if (arg != 'a' || arg != 'b')
        {
          /* Only channel a or b are available */

          ret = -EINVAL;
          break;
        }

      dev->channel = arg;

      if (dev->channel == 'b')
        {
          /* Only valid gain for channel b is 32, adjust */

          dev->gain = 32;
        }

      if (dev->channel == 'a')
        {
          /* If we are switching from channel 'b', gain will be 32,
           * which is invalid value for channel 'a'. If current gain
           * is not valid for channel 'a', set default value of 128
           */

          if (dev->gain != 128 && dev->gain != 64)
            {
              dev->gain = 128;
            }
        }

      /* Channel setting will be applied after next read from hx711,
       * we have to do one dummy read, so that user can immediately
       * read from new channel
       */

      if (hx711_single_read(dev) == INT32_MIN)
        {
          ret = -EIO;
        }

      break;

    case HX711_SET_GAIN:
      if (dev->channel == 'a' && (arg != 128 || arg != 64))
        {
          /* For channel 'a' only gain of value 128 and 64 are valid */

          ret = -EINVAL;
          break;
        }
      else if (dev->channel == 'b' && arg != 32)
        {
          /* For channel 'b' only gain of 32 is valid */

          ret = -EINVAL;
          break;
        }

      dev->gain = arg;

      break;

    case HX711_SET_VAL_PER_UNIT:
      dev->val_per_unit = arg;
      break;

    case HX711_GET_AVERAGE:
        {
          FAR unsigned *ptr = (FAR unsigned *)((uintptr_t)arg);
          if (ptr == NULL)
            {
              ret = -EINVAL;
              break;
            }

          *ptr = dev->average;
          break;
        }

    case HX711_GET_CHANNEL:
        {
          FAR char *ptr = (FAR char *)((uintptr_t)arg);
          if (ptr == NULL)
            {
              ret = -EINVAL;
              break;
            }

          *ptr = dev->channel;
          break;
        }

    case HX711_GET_GAIN:
        {
          FAR unsigned char *ptr = (FAR unsigned char *)((uintptr_t)arg);
          if (ptr == NULL)
            {
              ret = -EINVAL;
              break;
            }

          *ptr = dev->gain;
          break;
        }

    case HX711_GET_VAL_PER_UNIT:
        {
          FAR unsigned *ptr = (FAR unsigned *)((uintptr_t)arg);
          if (ptr == NULL)
            {
              ret = -EINVAL;
              break;
            }

          *ptr = dev->val_per_unit;
          break;
        }

    case HX711_TARE:
        {
          FAR float *precision = (FAR float *)((uintptr_t)arg);
          if (precision == NULL)
            {
              ret = -EINVAL;
              break;
            }

          ret = hx711_tare(dev, *precision);
          break;
        }

    case HX711_SET_SIGN:
        {
          FAR int *sign = (FAR int *)((uintptr_t)arg);
          if (sign == NULL || (*sign != 1 && *sign != -1))
            {
              ret = EINVAL;
              break;
            }

          dev->sign = *sign;
          break;
        }

    default:
      ret = EINVAL;
    }

  nxmutex_unlock(&dev->excl);
  return ret;
}

/****************************************************************************
 * Name: hx711_data_interrupt
 *
 * Description:
 *   Function is called when we are waiting for hx711 to be ready and once
 *   data line goes from HIGH to LOW state.
 *
 * Input Parameters:
 *   arg - hx711 device instance
 *
 ****************************************************************************/

static int hx711_data_interrupt(int irq, FAR void *context, FAR void *arg)
{
  UNUSED(irq);
  UNUSED(context);
  FAR struct hx711_dev_s *dev = arg;

  nxsem_post(&dev->hx711_ready);
  return 0;
}

/****************************************************************************
 * Name: hx711_wait_ready
 *
 * Description:
 *   Waits for conversion to be ready to read.
 *
 * Input Parameters:
 *   dev - hx711 device instance
 *
 * Returned Value:
 *  Function returns OK when chip is ready for reading, or -EIO, which
 *  means there is problem communicating with the device.
 *
 ****************************************************************************/

static int hx711_wait_ready(FAR struct hx711_dev_s *dev)
{
  int ret;
  struct timespec tp;

  /* It is possible that there was no read() call for long enough
   * that hx711 is already ready, if that is the case just quickly return
   */

  if (dev->lower->data_read(dev->minor) == 0)
    {
      return OK;
    }

  /* Install data line interrupt, so we know when hx711 is ready.
   * This can even be 100ms between sampling, and up to 500ms when
   * hx711 goes out of low power mode
   */

  if ((ret = dev->lower->data_irq(dev->minor, hx711_data_interrupt, dev)))
    {
      return ret;
    }

  /* During waiting for ready signal, clock should be low */

  dev->lower->clock_set(dev->minor, 0);

  clock_gettime(CLOCK_MONOTONIC, &tp);
  tp.tv_sec += 1;

  if ((ret = nxsem_timedwait(&dev->hx711_ready, &tp)))
    {
      /* Chip not ready for long time. This probably mean that the
       * hx711 chip is not properly (if at all) connected.
       */

      dev->lower->data_irq(dev->minor, NULL, NULL);
      return -EIO;
    }

  /* hx711 is ready */

  dev->lower->data_irq(dev->minor, NULL, NULL);
  return OK;
}

/****************************************************************************
 * Name: hx711_delay
 *
 * Description:
 *   hx711 datasheet specifies that time between clock changes should be
 *   between 0.2us and 50us, with typical value of 1us. On slow MCUs this
 *   is not a problem, as all operations between clocking take longer than
 *   that time, but on fast CHIP, clocking without delay will cause data
 *   lose.
 *
 ****************************************************************************/

static void hx711_delay(void)
{
#ifdef CONFIG_ADC_HX711_ADD_DELAY
  up_delay(1);
#endif
}

/****************************************************************************
 * Name: hx711_single_read
 *
 * Description:
 *   Reads single, 24bit adc data from hx711. Function will perform
 *   conversion form 24bit 2's complement to 32bit 2's complement.
 *
 * Input Parameters:
 *   dev - hx711 instance to perform read from.
 *
 * Returned Value:
 *   Read value from hx711. Returned value is stored on 24 bits of
 *   int32_t type. If there was error during read, function will
 *   return INT32_MIN.
 *
 ****************************************************************************/

static int32_t hx711_single_read(FAR struct hx711_dev_s *dev)
{
  int32_t value;
  int i;
  int pulses;
  int flags;
  int ret;

  /* Wait for conversion to be finished */

  if ((ret = hx711_wait_ready(dev)))
    {
      /* Timeout while waiting for chip, assuming chip is not connected */

      nxmutex_unlock(&dev->excl);
      return INT32_MIN;
    }

  /* Even though we are clocking the hx711, we must perform whole readout
   * without interruption. This is because, if we set clock pin to HIGH,
   * hx711 will go into low power mode in 60us unless we set clock to LOW
   * within that time.
   */

  flags = enter_critical_section();

  for (value = i = 0; i != HX711_BITS_PER_READ; i++)
    {
      dev->lower->clock_set(dev->minor, 1);
      hx711_delay();

      /* Data is sent MSB first */

      value |= dev->lower->data_read(dev->minor);
      value <<= 1;
      dev->lower->clock_set(dev->minor, 0);
      hx711_delay();
    }

  /* Next few clock pulses will determine type of next conversion
   * hx711 will perform. We gotta do this in the same critical
   * section block as read.
   *
   * 1 pulse  - Channel A, Gain 128
   * 2 pulses - Channel B, Gain 32
   * 3 pulses - Channel A, Gain 64
   */

  if (dev->channel == 'b')
    {
      /* Channel B has static gain of 32 */

      pulses = 2;
    }
  else
    {
      /* channel A has 2 possible gains, either 128 or 64. */

      pulses = dev->gain == 128 ? 1 : 3;
    }

  for (i = 0; i != pulses; i++)
    {
      dev->lower->clock_set(dev->minor, 1);
      hx711_delay();
      dev->lower->clock_set(dev->minor, 0);
      hx711_delay();
    }

  leave_critical_section(flags);

  /* Data is sent in standard 2's complement, but we just stored
   * 24bit integer in a 32bit integer. For positives reading this
   * makes no difference, but if we have just returned 24bit negative
   * number in 32bit integer, we would end up with positive (and false)
   * reading.
   *
   * If number is negative, convert it to 32bit negative.
   */

  if (value & 0x800000)
    {
      value |= 0xff000000;
    }

  /* Apply tare value and sign at the end */

  return dev->sign * (value - dev->tare);
}

/****************************************************************************
 * Name: hx711_read
 *
 * Description:
 *   Performs read from the hx711 device. Only a single value can be read
 *   with single call, but when averaging is enabled, driver will read
 *   configured number of points and will return single, average value
 *   of them all.
 *
 * Input Parameters:
 *   filep - file on vfs associated with the driver.
 *   buf - pointer to 32bit integer where value will be stored.
 *   buflen - size of buf, must be equal to 4 (sizeof(int32_t))
 *
 * Returned Value:
 *   On success 4 is returned (sizeof(int32_t)), as in number of bytes
 *   copied to userspace. On failure, negated errno is returned.
 *
 ****************************************************************************/

static ssize_t hx711_read(FAR struct file *filep,
                          FAR char *buf, size_t buflen)
{
  FAR struct hx711_dev_s *dev;
  int ret;
  int32_t value; /* 24bit value from hx711 will be stored here */
  int32_t average;
  int i;

  value = 0;
  dev = filep->f_inode->i_private;

  if (buflen == 0)
    {
      return 0;
    }

  if (buflen < sizeof(int32_t))
    {
      return -EINVAL;
    }

  /* Get exclusive access to the hx711 driver state */

  ret = nxmutex_lock(&dev->excl);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 1; i <= (int)dev->average; i++)
    {
      value = hx711_single_read(dev);
      if (value == INT32_MIN)
        {
          /* There was error while reading sample. */

          nxmutex_unlock(&dev->excl);
          return -EIO;
        }

      average = (average * (i - 1) + value) / i;
    }

  /* We are done with the device, so free mutex for next possible client */

  nxmutex_unlock(&dev->excl);

  /* If user specified value per unit, we convert raw data into units */

  if (dev->val_per_unit > 0)
    {
      average /= dev->val_per_unit;
    }

  /* Copy data back to userspace and exit */

  if (buflen == sizeof(int32_t))
    {
      /* int32 was passed, assuming binary operation from C code */

      memcpy(buf, &average, sizeof(average));
      return sizeof(int32_t);
    }
  else
    {
      /* Something else passed, assuming it's shell operation. If it's
       * called from C, it's assumed user wants c-string.
       */

      ret = snprintf(buf, buflen, "%"PRIi32"\n", average);

      /* snprintf returns number of bytes written (or that would have
       * been written) without null byte, but we return number of bytes
       * written including that byte, hence +1.
       */

      ret += 1;

      /* If buflen is not big enough, snprintf() will return number
       * of bytes that would have been written to buf if enough space
       * had been available and not number of bytes actually written.
       * We must return number of bytes actually written, so we take
       * smaller value.
       */

      return MIN(ret, (int)buflen);
    }
}

/****************************************************************************
 * Name: hx711_cleanup
 *
 * Description:
 *   Called when last user closed hx711 dsevice and that device is (or was)
 *   unlinked.
 *
 * Input Parameters:
 *   dev - hx711 device instance.
 *
 ****************************************************************************/

static void hx711_cleanup(FAR struct hx711_dev_s *dev)
{
  /* Put chip into sleep state by setting clock to HIGH */

  dev->lower->clock_set(dev->minor, 1);

  if (dev->lower->cleanup)
    {
      dev->lower->cleanup(dev->minor);
    }

  nxmutex_destroy(&dev->excl);
  nxsem_destroy(&dev->hx711_ready);
  kmm_free(dev);
}

/****************************************************************************
 * Name: hx711_open
 *
 * Description:
 *   Open driver for use by userspace application.
 *
 * Input Parameters:
 *   filep - pointer to a file structure to open
 *
 * Returned Value:
 *   OK on success, or negated errno on failure
 *
 ****************************************************************************/

static int hx711_open(FAR struct file *filep)
{
  FAR struct hx711_dev_s *dev;
  int ret;

  dev = filep->f_inode->i_private;

  /* Get exclusive access to the hx711 driver state */

  ret = nxmutex_lock(&dev->excl);
  if (ret < 0)
    {
      return ret;
    }

  /* Increment the count of open references on the driver */

  dev->crefs++;
  DEBUGASSERT(dev->crefs > 0);

  nxmutex_unlock(&dev->excl);
  return OK;
}

/****************************************************************************
 * Name: hx711_close
 *
 * Description:
 *   Closes the driver device. If this is last reference and file has been
 *   unlinked, we will also free resources allocated by ipcc_register()
 *
 * Input Parameters:
 *   filep - pointer to a file structure to close.
 *
 * Returned Value:
 *   OK on success, or negated errno on failure.
 *
 ****************************************************************************/

static int hx711_close(FAR struct file *filep)
{
  FAR struct hx711_dev_s *dev;
  int ret;

  dev = filep->f_inode->i_private;

  /* Get exclusive access to the hx711 driver state */

  ret = nxmutex_lock(&dev->excl);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the count of open references on the driver */

  DEBUGASSERT(dev->crefs > 0);
  dev->crefs--;

  if (dev->crefs <= 0 && dev->unlinked)
    {
      /* If count ref is zero and file has been unlinked, it
       * means nobody uses the driver and seems like nobody
       * wants to use it anymore, so free up resources. This
       * also means we are last holders of excl mutex, which
       * will be destroyed in cleanup function, so we don't
       * have to unlock it here.
       */

      hx711_cleanup(dev);
      return OK;
    }

  nxmutex_unlock(&dev->excl);
  return OK;
}

/****************************************************************************
 * Name: hx711_unlink
 *
 * Description:
 *   Action to take upon file unlinking. Function will free resources if
 *   noone is using the driver when unlinking occured. If driver is still
 *   in use, it will be marked as unlinked and resource freeing will take
 *   place in hx711_close() function instead, once last reference is closed.
 *
 * Input Parameters:
 *   inode - driver inode that is being unlinked.
 *
 * Returned Value:
 *   OK on successfull close, or negated errno on failure.
 *
 ****************************************************************************/

static int hx711_unlink(FAR struct inode *inode)
{
  FAR struct hx711_dev_s *dev;
  int ret;

  dev = inode->i_private;

  /* Get exclusive access to the hx711 driver state */

  ret = nxmutex_lock(&dev->excl);
  if (ret < 0)
    {
      return ret;
    }

  /* Is anyone still using the driver? */

  if (dev->crefs <= 0)
    {
      /* No, we are free to free resources */

      hx711_cleanup(dev);
      return OK;
    }

  /* Yes, someone is still using the driver, just mark file
   * as unlinked and free resources in hx711_close() once last
   * reference is closed.
   */

  dev->unlinked = true;
  nxmutex_unlock(&dev->excl);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hx711_register
 *
 * Description:
 *   Register new hx711 device in /dev/hx711_%d. Multiple hx711 can be
 *   supported by providing different minor number. When driver calls
 *   platform specific function, minor number is passed back, so platform
 *   can know which hx711 is manipulated.
 *
 * Input Parameters:
 *   minor - unique number identifying hx711 chip.
 *   lower - provided by platform code to manipulate hx711 with platform
 *           dependant functions>
 *
 * Returned Value:
 *   OK on success, or negated errno on failure
 *
 ****************************************************************************/

int hx711_register(unsigned char minor, FAR struct hx711_lower_s *lower)
{
  FAR struct hx711_dev_s *dev;
  char devname[DEVNAME_FMTLEN];
  int ret;

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  snprintf(devname, sizeof(devname), DEVNAME_FMT, minor);
  ret = register_driver(devname, &g_hx711_fops, 0666, dev);
  if (ret)
    {
      kmm_free(dev);
      return ret;
    }

  dev->channel = 'a';
  dev->gain = 128;
  dev->lower = lower;
  dev->average = 1;
  dev->sign = 1;
  nxmutex_init(&dev->excl);
  nxsem_init(&dev->hx711_ready, 0, 0);

  /* Put chip into working state by setting clock to LOW */

  dev->lower->clock_set(dev->minor, 0);

  return OK;
}
