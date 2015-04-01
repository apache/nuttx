/****************************************************************************
 * drivers/timers/rtc.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/timers/rtc.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rtc_upperhalf_s
{
  FAR struct rtc_lowerhalf_s *lower;  /* Contained lower half driver */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  uint8_t crefs;                      /* Number of open references */
  bool unlinked;                      /* True if the driver has been unlinked */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Internal logic */

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static void    rtc_destroy(FAR struct rtc_upperhalf_s *upper);
#endif

/* Character driver methods */

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     rtc_open(FAR struct file *filep);
static int     rtc_close(FAR struct file *filep);
#endif

static ssize_t rtc_read(FAR struct file *filep, FAR char *, size_t);
static ssize_t rtc_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     rtc_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     rtc_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations rtc_fops =
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  rtc_open,      /* open */
  rtc_close,     /* close */
#else
  0,             /* open */
  0,             /* close */
#endif
  rtc_read,      /* read */
  rtc_write,     /* write */
  0,             /* seek */
  rtc_ioctl,     /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0,             /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  rtc_unlink     /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtc_destory
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static void rtc_destroy(FAR struct rtc_upperhalf_s *upper)
{
  /* If the lower half driver provided a destroy method, then call that
   * method now in order order to clean up resources used by the lower-half
   * driver.
   */

  DEBUGASSERT(upper->lower && upper->lower->ops);
  if (upper->lower->ops->destroy)
    {
      upper->lower->ops->destroy(upper->lower);
    }

  /* And free our container */

  kmm_free(upper);
}
#endif

/****************************************************************************
 * Name: rtc_open
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int rtc_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct rtc_upperhalf_s *upper;

  /* Get the reference to our internal state structure from the inode
   * structure.
   */

  DEBUGASSERT(filep);
  inode = filep->f_inode;
  DEBUGASSERT(inode && inode->i_private);
  upper = inode->i_private;

  /* Increment the count of open references on the RTC driver */

  upper->crefs++;
  DEBUGASSERT(upper->crefs > 0);
  return OK;
}
#endif

/****************************************************************************
 * Name: rtc_close
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int rtc_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct rtc_upperhalf_s *upper;

  /* Get the reference to our internal state structure from the inode
   * structure.
   */

  DEBUGASSERT(filep);
  inode = filep->f_inode;
  DEBUGASSERT(inode && inode->i_private);
  upper = inode->i_private;

  /* Decrement the count of open references on the RTC driver */

  DEBUGASSERT(upper->crefs > 0);
  upper->crefs--;

  /* If the count has decremented to zero and the driver has been unlinked,
   * then commit Hara-Kiri now.
   */

  if (upper->crefs == 0 && upper->unlinked)
    {
      rtc_destroy(upper);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: rtc_read
 ****************************************************************************/

static ssize_t rtc_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  return 0; /* Return EOF */
}

/****************************************************************************
 * Name: rtc_write
 ****************************************************************************/

static ssize_t rtc_write(FAR struct file *filep, FAR const char *buffer, size_t len)
{
  return len; /* Say that everything was written */
}

/****************************************************************************
 * Name: rtc_ioctl
 ****************************************************************************/

static int rtc_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct rtc_upperhalf_s *upper;
  FAR const struct rtc_ops_s *ops;
  int ret = -ENOSYS;

  /* Get the reference to our internal state structure from the inode
   * structure.
   */

  DEBUGASSERT(filep);
  inode = filep->f_inode;
  DEBUGASSERT(inode && inode->i_private);
  upper = inode->i_private;
  DEBUGASSERT(upper->lower && upper->lower->ops);

  /* We simply forward all ioctl() commands to the lower half.  The upper
   * half is nothing more than a thin driver shell over the lower level
   * RTC implementation.
   */

  ops = upper->lower->ops;
  switch (cmd)
    {
    /* RTC_RD_TIME returns the current RTC time.
     *
     * Argument: A writeable reference to a struct rtc_time to receive the
     *           RTC's time.
     */

    case RTC_RD_TIME:
      {
        FAR struct rtc_time *rtctime = (FAR struct rtc_time *)((uintptr_t)arg);

        if (ops->rdtime)
          {
            ret = ops->rdtime(upper->lower, rtctime);
          }
      }
      break;

    /* RTC_SET_TIME sets the RTC's time
     *
     * Argument: A read-only reference to a struct rtc_time containing the
     *           the new time to be set.
     */

    case RTC_SET_TIME:
      {
        FAR const struct rtc_time *rtctime =
          (FAR const struct rtc_time *)((uintptr_t)arg);

        if (ops->settime)
          {
            ret = ops->settime(upper->lower, rtctime);
          }
      }
      break;

#ifdef CONFIG_RTC_ALARM
    /* RTC_ALM_READ reads the alarm time (for RTCs that support alarms)
     *
     * Argument: A writeable reference to a struct rtc_time to receive the
     *           RTC's alarm time.
     */

    case RTC_ALM_READ:
      {
        FAR struct rtc_time *almtime = (FAR struct rtc_time *)((uintptr_t)arg);

        if (ops->almread)
          {
            ret = ops->almread(upper->lower, almtime);
          }
      }
      break;

    /* RTC_ALM_SET sets the alarm time (for RTCs that support alarms).
     *
     * Argument: A read-only reference to a struct rtc_time containing the
     *           new alarm time to be set.
     */

    case RTC_ALM_SET:
      {
        FAR const struct rtc_time *almtime =
          (FAR const struct rtc_time *)((uintptr_t)arg);

        if (ops->almset)
          {
            ret = ops->almset(upper->lower, almtime);
          }
      }
      break;
#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_PERIODIC
    /* RTC_IRQP_READ read the frequency for periodic interrupts (for RTCs
     * that support periodic interrupts)
     *
     * Argument: A pointer to a writeable unsigned long value in which to
     *           receive the frequency value.
     */

    case RTC_IRQP_READ:
      {
        FAR unsigned long *irqpfreq = (FAR unsigned long *)((uintptr_t)arg);

        if (ops->irqpread)
          {
            ret = ops->irqpread(upper->lower, irqpfreq);
          }
      }
      break;

    /* RTC_IRQP_SET set the frequency for periodic interrupts (for RTCs that
     * support periodic interrupts)
     *
     * Argument: An unsigned long value providing the new periodic frequency
     */

    case RTC_IRQP_SET:
      {
        if (ops->irqpset)
          {
            ret = ops->irqpset(upper->lower, arg);
          }
      }
      break;
#endif /* CONFIG_RTC_PERIODIC */

#ifdef CONFIG_RTC_ALARM
    /* RTC_AIE_ON enable alarm interrupts (for RTCs that support alarms)
     *
     * Argument: None
     */

    case RTC_AIE_ON:
      {
        if (ops->aie)
          {
            ret = ops->aie(upper->lower, true);
          }
      }
      break;

    /* RTC_AIE_OFF disable the alarm interrupt (for RTCs that support
     * alarms)
     *
     * Argument: None
     */

    case RTC_AIE_OFF:
      {
        if (ops->aie)
          {
            ret = ops->aie(upper->lower, false);
          }
      }
      break;
#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_ONESEC
    /* RTC_UIE_ON enable the interrupt on every clock update (for RTCs that
     * support this once-per-second interrupt).
     *
     * Argument: None
     */

    case RTC_UIE_ON:
      {
        if (ops->uie)
          {
            ret = ops->uie(upper->lower, true);
          }
      }
      break;

    /* RTC_UIE_OFF disable the interrupt on every clock update (for RTCs
     * that support this once-per-second interrupt).
     *
     * Argument: None
     */

    case RTC_UIE_OFF:
      {
        if (ops->uie)
          {
            ret = ops->uie(upper->lower, false);
          }
      }
      break;
#endif /* CONFIG_RTC_ONESEC */

#ifdef CONFIG_RTC_PERIODIC
    /* RTC_PIE_ON enable the periodic interrupt (for RTCs that support these
     * periodic interrupts).
     *
     * Argument: None
     */

    case RTC_PIE_ON:
      {
        if (ops->pie)
          {
            ret = ops->pie(upper->lower, true);
          }
      }
      break;

    /* RTC_PIE_OFF disable the periodic interrupt (for RTCs that support
     * these periodic interrupts).
     *
     * Argument: None
     */

    case RTC_PIE_OFF:
      {
        if (ops->pie)
          {
            ret = ops->pie(upper->lower, false);
          }
      }
      break;
#endif /* CONFIG_RTC_PERIODIC */

#ifdef CONFIG_RTC_EPOCHYEAR
   /* RTC_EPOCH_READ read the Epoch.
    *
    * Argument: A reference to a writeable unsigned low variable that will
    *           receive the Epoch value.
    */

    case RTC_EPOCH_READ:
      {
        FAR unsigned long *epoch = (FAR unsigned long *)((uintptr_t)arg);

        if (ops->rdepoch)
          {
            ret = ops->rdepoch(upper->lower, epoch);
          }
      }
      break;

    /* RTC_EPOCH_SET set the Epoch
     *
     * Argument: An unsigned long value containing the new Epoch value to be
     *           set.
     */

    case RTC_EPOCH_SET:
      {
        if (ops->setepoch)
          {
            ret = ops->setepoch(upper->lower, arg);
          }
      }
      break;
#endif /* CONFIG_RTC_EPOCHYEAR */

#ifdef CONFIG_RTC_ALARM
    /* RTC_WKALM_RD read the current alarm
     *
     * Argument: A writeable reference to struct rtc_wkalrm to receive the
     *           current alarm settings.
     */

    case RTC_WKALM_RD:
      {
        FAR struct rtc_wkalrm *wkalrm = (FAR struct rtc_wkalrm *)((uintptr_t)arg);

        if (ops->rdwkalm)
          {
            ret = ops->rdwkalm(upper->lower, wkalrm);
          }
      }
      break;

    /* RTC_WKALM_SET set the alarm.
     *
     * Argument: A read-only reference to struct rtc_wkalrm containing the
     *           new alarm settings.
     */

    case RTC_WKALM_SET:
      {
        FAR const struct rtc_wkalrm *wkalrm =
          (FAR const struct rtc_wkalrm *)((uintptr_t)arg);

        if (ops->setwkalm)
          {
            ret = ops->setwkalm(upper->lower, wkalrm);
          }
      }
      break;
#endif /* CONFIG_RTC_ALARM */

    /* Forward any unrecognized IOCTLs to the lower half driver... they
     * may represent some architecture-specific command.
     */

    default:
      {
        ret = -ENOTTY;
#ifdef CONFIG_RTC_IOCTL
        if (ops->ioctl)
          {
            ret = ops->ioctl(upper->lower, cmd, arg);
          }
#endif
      }
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: rtc_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int rtc_unlink(FAR struct inode *inode)
{
  FAR struct rtc_upperhalf_s *upper;

  /* Get the reference to our internal state structure from the inode
   * structure.
   */

  DEBUGASSERT(inode && inode->i_private);
  upper = inode->i_private;

  /* Indicate that the driver has been unlinked */

  upper->unlinked = true;

  /* If there are no further open references to the driver, then commit
   * Hara-Kiri now.
   */

  if (upper->crefs == 0)
    {
      rtc_destroy(upper);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtc_initialize
 *
 * Description:
 *   Create an RTC driver by binding to the lower half RTC driver instance
 *   provided to this function.  The resulting RTC driver will be registered
 *   at /dev/rtcN where N is the minor number provided to this function.
 *
 * Input parameters:
 *   minor - The minor number of the RTC device.  The N in /dev/rtcN
 *   lower - The lower half driver instance.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ****************************************************************************/

int rtc_initialize(int minor, FAR struct rtc_lowerhalf_s *lower)
{
  FAR struct rtc_upperhalf_s *upper;
  char devpath[16];
  int ret;

  DEBUGASSERT(lower && lower->ops && minor >= 0 && minor < 1000);

  /* Allocate an upper half container structure */

  upper = (FAR struct rtc_upperhalf_s *)kmm_malloc(sizeof(struct rtc_upperhalf_s));
  if (!upper)
    {
      return -ENOMEM;
    }

  /* Initialize the upper half container */

  upper->lower = lower;     /* Contain lower half driver */

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  upper->crefs = 0;         /* No open references */
  upper->unlinked = false;  /* Driver is not  unlinked */
#endif

  /* Create the driver name.  There is space for the a minor number up to  6
   * characters
   */

  snprintf(devpath, 16, "/dev/rtc%d", minor);

  /* And, finally, register the new RTC driver */

  ret = register_driver(devpath, &rtc_fops, 0666, upper);
  if (ret < 0)
    {
      kmm_free(upper);
      return ret;
    }

  return OK;
}
