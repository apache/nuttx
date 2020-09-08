/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_rtc.c
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

#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>
#include <nuttx/compiler.h>
#include <arch/board/board.h>
#include <rx65n_rtc.h>
#include "up_arch.h"

#include "nuttx/compiler.h"
#ifdef CONFIG_RX65N_RTC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

# define rx65n_getreg(addr)      getreg8(addr)
# define rx65n_putreg(val,addr)  putreg8(val,addr)

/* Configuration ************************************************************/

#ifdef CONFIG_RTC_HIRES
#  ifndef CONFIG_RTC_FREQUENCY
#    error "CONFIG_RTC_FREQUENCY is required for CONFIG_RTC_HIRES"
#  endif
#else
#  ifndef CONFIG_RTC_FREQUENCY
#    define CONFIG_RTC_FREQUENCY 1
#  endif
#  if CONFIG_RTC_FREQUENCY != 1
#    error "Only lo-res CONFIG_RTC_FREQUENCY of 1Hz is supported"
#  endif
#  endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

void up_enable_irq(int irq);
void up_disable_irq(int irq);
void  rtc_prd_interrupt(void);
static uint32_t rtc_dec2bcd(uint8_t value);
#if defined (CONFIG_RTC_HIRES) || defined (CONFIG_RTC_ALARM) || defined (CONFIG_RTC_DATETIME)
static int rtc_bcd2dec(uint32_t value);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Callback to use when the alarm expires */

#ifdef CONFIG_RTC_ALARM
struct alm_cbinfo_s
{
  volatile alm_callback_t ac_cb; /* Client callback function */
  volatile FAR void *ac_arg;     /* Argument to pass with the callback function */
};
#endif

/* Callback to use when the periodic interrupt expires */

#ifdef CONFIG_RTC_PERIODIC
struct prd_cbinfo_s
{
  volatile periodiccb_t prd_cb; /* Client callback function */
  volatile FAR void *prd_arg;   /* Argument to pass with the callback function */
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
/* Callback to use when an EXTI is activated  */

static struct alm_cbinfo_s g_alarmcb;
#endif

#ifdef CONFIG_RTC_PERIODIC
static struct prd_cbinfo_s g_periodiccb;
#endif

/* Callback to use when the cary interrupt expires */

#ifdef CONFIG_RX65N_CARRY
static carrycb_t g_carrycb;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_rtc_enabled is set true after the RTC has successfully initialized */

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtc_dumpregs
 *
 * Description:
 *    Disable RTC write protection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_RTC_INFO
static void rtc_dumpregs(FAR const char *msg)
{
  rtcinfo("%s:\n", msg);
  rtcinfo("  64-Hz Counter: %08x\n", getreg8(RX65N_RTC_R64CNT));
  rtcinfo("  Second Counter: %08x\n", getreg8(RX65N_RTC_RSECCNT));
  rtcinfo("  Minute Counter: %08x\n", getreg8(RX65N_RTC_RMINCNT));
  rtcinfo("  Hour Counter: %08x\n", getreg8(RX65N_RTC_RHRCNT));
  rtcinfo("  Day-of-Week Counter: %08x\n", getreg8(RX65N_RTC_RWKCNT));
  rtcinfo("  Date Counter: %08x\n", getreg8(RX65N_RTC_RDAYCNT));
  rtcinfo("  Month Counter: %08x\n", getreg8(RX65N_RTC_RMONCNT));
  rtcinfo("  Year Counter: %08x\n", getreg8(RX65N_RTC_RYRCNT));
  rtcinfo(" Second Alarm Register: %08x\n", getreg8(RX65N_RTC_RSECAR));
  rtcinfo(" Minute Alarm Register: %08x\n", getreg8(RX65N_RTC_RMINAR));
  rtcinfo(" Hour Alarm Register: %08x\n", getreg8(RX65N_RTC_RHRAR));
  rtcinfo(" Day-of-Week Alarm Register: %08x\n", getreg8(RX65N_RTC_RWKAR));
  rtcinfo(" Date Alarm Register: %08x\n", getreg8(RX65N_RTC_RDAYAR));
  rtcinfo(" Month Alarm Register: %08x\n", getreg8(RX65N_RTC_RMONAR));
  rtcinfo(" Year Alarm Register: %08x\n", getreg8(RX65N_RTC_RYRAR));
  rtcinfo(" Year Alarm Enable Register: %08x\n", getreg8(RX65N_RTC_RYRAREN));
  rtcinfo(" RTC Control Register 1: %08x\n", getreg8(RX65N_RTC_RCR1));
  rtcinfo(" RTC Control Register 2: %08x\n", getreg8(RX65N_RTC_RCR2));
  rtcinfo(" RTC Control Register 3: %08x\n", getreg8(RX65N_RTC_RCR3));
  rtcinfo(" RTC Control Register 4: %08x\n", getreg8(RX65N_RTC_RCR4));
}
#else
#  define rtc_dumpregs(msg)
#endif

/****************************************************************************
 * Name: rtc_dumptime
 *
 * Description:
 *    Disable RTC write protection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_RTC_INFO
static void rtc_dumptime(FAR struct tm *tp, FAR const char *msg)
{
  rtcinfo("%s:\n", msg);
  rtcinfo("  tm_sec: %08x\n", tp->tm_sec);
  rtcinfo("  tm_min: %08x\n", tp->tm_min);
  rtcinfo(" tm_hour: %08x\n", tp->tm_hour);
  rtcinfo(" tm_mday: %08x\n", tp->tm_mday);
  rtcinfo("  tm_mon: %08x\n", tp->tm_mon);
  rtcinfo(" tm_year: %08x\n", tp->tm_year);
}
#else
#  define rtc_dumptime(tp, msg)
#endif

/****************************************************************************
 * Name: rtc_dec2bcd
 *
 * Description:
 *   Converts decimal value to BCD format
 *
 * Input Parameters:
 *   value - The byte to be converted.
 *
 * Returned Value:
 *   The value in BCD representation
 *
 ****************************************************************************/

static uint32_t rtc_dec2bcd(uint8_t value)
{
  return (uint8_t) ((((value / 10) << 4) & 0xf0) | (value % 10));
}

/****************************************************************************
 * Name: rtc_bcd2dec
 *
 * Description:
 *   Convert from 2 digit BCD to decimal.
 *
 * Input Parameters:
 *   value - The BCD value to be converted.
 *
 * Returned Value:
 *   The value in binary representation
 *
 ****************************************************************************/

#if defined (CONFIG_RTC_HIRES) || defined (CONFIG_RTC_ALARM) || defined (CONFIG_RTC_DATETIME)
static int rtc_bcd2dec(uint32_t value)
{
  return (int) ((((value & 0xf0) >> 4) * 10) + (value & 0x0f));
}
#endif

/****************************************************************************
 * Name: rtc_interrupt
 *
 * Description:
 *    RTC interrupt service routine
 *
 * Input Parameters:
 *   irq - The IRQ number that generated the interrupt
 *   context - Architecture specific register save information.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int rtc_alm_interrupt(int irq, void *context, FAR void *arg)
{
  FAR struct alm_cbinfo_s *cbinfo;
  alm_callback_t cb;
  uint8_t source = rx65n_getreg(RX65N_RTC_RCR1);
  if ((source & RTC_ALARM_INT_ENABLE) != 0)
    {
      /* Alarm callback */

      cbinfo = &g_alarmcb;
          cb = cbinfo->ac_cb;
          arg = (FAR void *)cbinfo->ac_arg;
          cbinfo->ac_cb  = NULL;
      cbinfo->ac_arg = NULL;
      cb(arg, 0);
    }

  up_disable_irq(RX65N_ALM_IRQ);
  return 0;
}
#endif

#ifdef CONFIG_RTC_PERIODIC
static int rtc_periodic_interrupt(int irq, void *context, FAR void *arg)
{
  FAR struct prd_cbinfo_s *cbinfo;
  periodiccb_t cb;
  uint8_t source = rx65n_getreg(RX65N_RTC_RCR1);
  if ((source & RTC_PERIOD_INT_ENABLE) != 0)
    {
      /* Periodic callback */

      cbinfo = &g_periodiccb;
      cb = cbinfo->prd_cb;
      arg = (FAR void *)cbinfo->prd_arg;
      cb(arg, 0);
    }

  return 0;
}
#endif

#ifdef CONFIG_RX65N_CARRY
static int rtc_carry_interrupt(int irq, void *context, FAR void *arg)
{
  uint8_t source = rx65n_getreg(RX65N_RTC_RCR1);
  if ((source & RTC_CARRY_INT_ENABLE) != 0)
    {
      /* Carry callback */

      g_carrycb();
    }

  return 0;
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.
 *   This function is
 *   called once during the OS initialization sequence
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_initialize(void)
{
  uint8_t regval;
  uint32_t temp_byte;
  rtc_dumpregs("On reset");

  /* Disable ALM, PRD and CUP interrupts */

  up_disable_irq(RX65N_ALM_IRQ);
  up_disable_irq(RX65N_PRD_IRQ);
  up_disable_irq(RX65N_INTB176_IRQ);

  /* Set RTC clock source as sub clock */

  regval = getreg8(RX65N_RTC_RCR4);
  regval |= RTC_RCR4_RCKSEL;
  rx65n_putreg(regval, RX65N_RTC_RCR4);

  /* Set sub-clock oscillator */

  regval = getreg8(RX65N_RTC_RCR3);
  regval |= RTC_RCR3_RTCEN;
  rx65n_putreg(regval, RX65N_RTC_RCR3);

  while (1U != RTC.RCR3.BIT.RTCEN)
    {
      /* waiting for sub-clock oscillator is operating */
    }

  /* Stop all counters */

  rx65n_putreg(0, RX65N_RTC_RCR2);
  while (0U != RTC.RCR2.BIT.START)
    {
      /* Ensure the clock is stopped while configuring it. */
    }

  /* Select count mode */

  regval = getreg8(RX65N_RTC_RCR2);
  regval |= RTC_RCR2_CNTMD;
  rx65n_putreg(regval, RX65N_RTC_RCR2);

  while (0U != RTC.RCR2.BIT.CNTMD)
    {
      /* Wait for the calendar count mode complete setting */
    }

  /* Execute RTC software reset */

  regval = getreg8(RX65N_RTC_RCR2);
  regval |= RTC_RCR2_RESET;
  rx65n_putreg(regval, RX65N_RTC_RCR2);

  while (0U != RTC.RCR2.BIT.RESET)
    {
      /* Wait for the reset to complete */
    }

  /* Stop RTC counter */

  regval = getreg8(RX65N_RTC_RCR2);
  regval &= ~(RTC_RCR2_START);
  rx65n_putreg(regval, RX65N_RTC_RCR2);

  while (0U != RTC.RCR2.BIT.START)
    {
      /* Ensure the clock is stopped while configuring it. */
    }

  /* Clear ALM,PRD,CUP IR */

  IR(RTC, ALM) = 0U;
  IR(RTC, PRD) = 0U;
  IR(PERIB, INTB176) = 0U;

  /* After a reset is generated, write to the RTC register
   * when six cycles of the count source have elapsed
   */

  up_udelay(RX65N_RTC_WAIT_PERIOD);

  /* Start the counter and set 24hr mode */

  regval = getreg8(RX65N_RTC_RCR2);
  regval |= (RTC_RCR2_HR24);
  rx65n_putreg(regval, RX65N_RTC_RCR2);

  /* Setting RADJ register */

  regval = getreg8(RX65N_RTC_RADJ);
  regval |= (RTC_RADJ_INITVALUE);
  rx65n_putreg(regval, RX65N_RTC_RADJ);

  /* Setting AADJE and AADJP register */

  regval = getreg8(RX65N_RTC_RCR2);
  regval |= (RTC_RCR2_AADJE | RTC_RCR2_AADJP);
  rx65n_putreg(regval, RX65N_RTC_RCR2);
  g_rtc_enabled = true;
  UNUSED(temp_byte);
  return OK;
}

/****************************************************************************
 * Name: up_rtc_gettime
 *
 * Description:
 *   Get the current date and time from the date/time RTC.
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#if defined(CONFIG_RTC_HIRES)
int up_rtc_gettime(FAR struct timespec *tp)
{
  uint8_t weekcnt;
  uint8_t daycnt;
  uint8_t monthcnt;
  uint8_t yearcnt;
  uint8_t seccnt;
  uint8_t mincnt;
  uint8_t hrcnt;
  uint8_t tmp_week;
  uint8_t tmp_day;
  uint8_t tmp_month;
  uint8_t tmp_year;
  uint16_t bcd_years;
  uint8_t regval;
  struct tm t;

  if (RTC.RCR2.BIT.START == 0)
    {
          RTC.RCR2.BIT.START = 1;
    }

  do
    {
      weekcnt  = getreg8(RX65N_RTC_RWKCNT);
      daycnt   = getreg8(RX65N_RTC_RDAYCNT);
      monthcnt = getreg8(RX65N_RTC_RMONCNT);
      yearcnt  = getreg8(RX65N_RTC_RYRCNT);
      seccnt   = getreg8(RX65N_RTC_RSECCNT);
      mincnt   = getreg8(RX65N_RTC_RMINCNT);
      hrcnt    = getreg8(RX65N_RTC_RHRCNT);
      tmp_week = getreg8(RX65N_RTC_RWKCNT);
      tmp_day  = getreg8(RX65N_RTC_RDAYCNT);
      tmp_month = getreg8(RX65N_RTC_RMONCNT);
      tmp_year  = getreg8(RX65N_RTC_RYRCNT);
    }

  while (tmp_week != weekcnt && tmp_day != daycnt &&
         tmp_month != monthcnt && tmp_year != yearcnt);

  /* Disable ICU CUP interrupt */

  up_disable_irq(CONFIG_RX65N_PERIB);

  /* Enable RTC CUP interrupt */

  regval = getreg8(RX65N_RTC_RCR1);
  regval |= (RTC_RCR1_CUP);
  rx65n_putreg(regval, RX65N_RTC_RCR1);

  do
    {
      /* Clear carry flag in ICU */

      IR(PERIB, INTB176) = 0U;

      /* Read and convert RTC registers;
       * mask off unknown bits and hour am/pm.
       */

      /* Seconds. (0-59) */

      t.tm_sec  = rtc_bcd2dec((uint8_t) (RTC.RSECCNT.BYTE & 0x7fu));
      t.tm_min  = rtc_bcd2dec((uint8_t) (RTC.RMINCNT.BYTE & 0x7fu));
      t.tm_hour = rtc_bcd2dec((uint8_t) (RTC.RHRCNT.BYTE & 0x3fu));
      t.tm_mday = rtc_bcd2dec(RTC.RDAYCNT.BYTE);
      t.tm_mon  = rtc_bcd2dec(RTC.RMONCNT.BYTE) - 1;

      /* Years since 2000 */

      bcd_years = (uint16_t) RTC.RYRCNT.WORD;

      t.tm_year = rtc_bcd2dec((uint8_t) (bcd_years & 0xff)) + 100;

          tp->tv_sec = mktime(&t);
          tp->tv_nsec = 0;
    }

  while (1 == IR(PERIB, INTB176));
  UNUSED(hrcnt);
  UNUSED(mincnt);
  UNUSED(seccnt);
  return OK;
}
#endif

int rx65n_rtc_setdatetime(FAR const struct tm *tp)
{
  int i;
  volatile uint8_t dummy_byte;
  volatile uint16_t dummy_word;

  /* Break out the time values (note that the time is set only to units of
   * seconds)
   */

  /* (void)gmtime_r(&tp->tv_sec, &tp); */

  rtc_dumptime(&tp, "Setting time");

  /* Then write the broken out values to the RTC */

  /* Convert the struct tm format to RTC time register fields.
   *
   *   struct tm       TIMR register
   *   tm_sec    0-61* SEC    (0-59)
   *   tm_min    0-59  MIN    (0-59)
   *   tm_hour   0-23  HOUR   (0-23)
   *
   *  *To allow for leap seconds.  But these never actuall happen.
   */

  /* Stop all counters */

  RTC.RCR2.BIT.START = 0U;
  while (0U != RTC.RCR2.BIT.START)
    {
      /* Ensure the clock is stopped while configuring it. */
    }

  /* Execute RTC software reset */

  RTC.RCR2.BIT.RESET = 1U;
  while (1U != RTC.RCR2.BIT.RESET)
    {
      /* Wait for the reset to complete */
    }

  RTC.RCR2.BIT.HR24 = 1;

  /* Set time */

  /* Set seconds. (0-59) */

  RTC.RSECCNT.BYTE = rtc_dec2bcd((uint8_t)tp->tm_sec);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = RTC.RSECCNT.BYTE;
    }

  /* Set minutes (0-59) */

  RTC.RMINCNT.BYTE = rtc_dec2bcd((uint8_t) tp->tm_min);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = RTC.RMINCNT.BYTE;
    }

  /* Set hours. (0-23) */

  RTC.RHRCNT.BYTE = rtc_dec2bcd((uint8_t) tp->tm_hour);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = RTC.RHRCNT.BYTE;
    }

  /* Set the date */

  /* Day of the week (0-6, 0=Sunday) */

#if defined(CONFIG_LIBC_LOCALTIME) || defined(CONFIG_TIME_EXTENDED)
  RTC.RWKCNT.BYTE = rtc_dec2bcd((uint8_t) tp->tm_wday);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = RTC.RWKCNT.BYTE;
    }
#endif

  /* Day of the month (1-31) */

  RTC.RDAYCNT.BYTE = rtc_dec2bcd((uint8_t) tp->tm_mday);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = RTC.RDAYCNT.BYTE;
    }

  /* Month. (1-12, 1=January) */

  RTC.RMONCNT.BYTE = rtc_dec2bcd((uint8_t) (tp->tm_mon + 1));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = RTC.RMONCNT.BYTE;
    }

  /* Year. (00-99) */

  RTC.RYRCNT.WORD = (uint16_t) (rtc_dec2bcd((uint8_t)
                               ((tp->tm_year + 1900) % 100)));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_word = RTC.RYRCNT.WORD;
    }

  RTC.RCR2.BIT.START = 1U;

  rtc_dumpregs("New time setting");
  UNUSED(dummy_word);
  UNUSED(dummy_byte);
  return OK;
}

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations
 *   must be able to
 *   set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_settime(FAR const struct timespec *tp)
{
  FAR struct tm newtime;
  int i;
  volatile uint8_t dummy_byte;
  volatile uint16_t dummy_word;

  /* Break out the time values (note that the time is set only to units of
   * seconds)
   */

  (void)gmtime_r(&tp->tv_sec, &newtime);
  rtc_dumptime(&newtime, "Setting time");

  /* Then write the broken out values to the RTC */

  /* Convert the struct tm format to RTC time register fields.
   *
   *   struct tm       TIMR register
   *   tm_sec    0-61* SEC    (0-59)
   *   tm_min    0-59  MIN    (0-59)
   *   tm_hour   0-23  HOUR   (0-23)
   *
   *  *To allow for leap seconds.  But these never actuall happen.
   */

  /* Stop all counters */

  RTC.RCR2.BIT.START = 0U;
  while (0U != RTC.RCR2.BIT.START)
    {
      /* Ensure the clock is stopped while configuring it. */
    }

  /* Execute RTC software reset */

  RTC.RCR2.BIT.RESET = 1U;
  while (1U != RTC.RCR2.BIT.RESET)
    {
      /* Wait for the reset to complete */
    }

  RTC.RCR2.BIT.HR24 = 1;

  /* Set time */

  /* Set seconds. (0-59) */

  RTC.RSECCNT.BYTE = rtc_dec2bcd((uint8_t)newtime.tm_sec);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = RTC.RSECCNT.BYTE;
    }

  /* Set minutes (0-59) */

  RTC.RMINCNT.BYTE = rtc_dec2bcd((uint8_t) newtime.tm_min);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = RTC.RMINCNT.BYTE;
    }

  /* Set hours. (0-23) */

  RTC.RHRCNT.BYTE = rtc_dec2bcd((uint8_t) newtime.tm_hour);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = RTC.RHRCNT.BYTE;
    }

  /* Set the date */

  /* Day of the week (0-6, 0=Sunday) */

  RTC.RWKCNT.BYTE = rtc_dec2bcd((uint8_t) newtime.tm_wday);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = RTC.RWKCNT.BYTE;
    }

  /* Day of the month (1-31) */

  RTC.RDAYCNT.BYTE = rtc_dec2bcd((uint8_t) newtime.tm_mday);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = RTC.RDAYCNT.BYTE;
    }

  /* Month. (1-12, 1=January) */

  RTC.RMONCNT.BYTE = rtc_dec2bcd((uint8_t) (newtime.tm_mon + 1));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = RTC.RMONCNT.BYTE;
    }

  /* Year. (00-99) */

  RTC.RYRCNT.WORD = (uint16_t) (rtc_dec2bcd((uint8_t)
                               ((newtime.tm_year + 1900) % 100)));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_word = RTC.RYRCNT.WORD;
    }

  RTC.RCR2.BIT.START = 1U;

  rtc_dumpregs("New time setting");
  UNUSED(dummy_word);
  UNUSED(dummy_byte);
  return OK;
}

/****************************************************************************
 * Name: rx65n_rtc_getalarmdatetime
 *
 * Description:
 *   Get the current date and time for a RTC alarm.
 *
 * Input Parameters:
 *   reg - RTC alarm register
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int rx65n_rtc_getalarmdatetime(FAR struct tm *tp)
{
  uint8_t bcd_years;
  DEBUGASSERT(tp != NULL);

  tp->tm_sec  = rtc_bcd2dec((uint8_t) (RTC.RSECAR.BYTE & 0x7fu));
  tp->tm_min  = rtc_bcd2dec((uint8_t) (RTC.RMINAR.BYTE & 0x7fu));
  tp->tm_hour = rtc_bcd2dec((uint8_t) (RTC.RHRAR.BYTE & 0x3fu));
  tp->tm_mday = rtc_bcd2dec(RTC.RDAYAR.BYTE & 0x3fu);
  tp->tm_mon  = rtc_bcd2dec(RTC.RMONAR.BYTE & 0x1fu) - 1;

  /* Years since 2000 */

  bcd_years = (uint8_t) RTC.RYRAR.WORD;

  tp->tm_year = rtc_bcd2dec((uint8_t) (bcd_years & 0xff)) + 100;
  return 0;
}

#endif
/****************************************************************************
 * Name: rx65n_rtc_rdalarm
 *
 * Description:
 *   Query an alarm configured in hardware.
 *
 * Input Parameters:
 *  alminfo - Information about the alarm configuration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int rx65n_rtc_rdalarm(FAR struct alm_rdalarm_s *alminfo)
{
  int ret = -EINVAL;
  DEBUGASSERT(alminfo != NULL);
  ret = rx65n_rtc_getalarmdatetime((struct tm *)alminfo->ar_time);
  return ret;
}
#endif

/****************************************************************************
 * Name: rx65n_rtc_setalarm
 *
 * Description:
 *   Set up an alarm.
 *
 * Input Parameters:
 *   tp - the time to set the alarm
 *   callback - the function to call when the alarm expires.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int rx65n_rtc_setalarm(FAR struct alm_setalarm_s *alminfo)
{
  irqstate_t flags;
  uint8_t dummy_byte;
  uint8_t dummy_word;
  uint8_t i;
  int ret = -EBUSY;

  /* Is there already something waiting on the ALARM? */

  flags = enter_critical_section();

  /* Save the callback info */

  g_alarmcb.ac_cb  = alminfo->as_cb;
  g_alarmcb.ac_arg = alminfo->as_arg;

  IEN(RTC, ALM) = 0U;

  /* Attach the Alarm Interrupt */

  irq_attach(RX65N_ALM_IRQ, rtc_alm_interrupt, NULL);

  /* Start RTC counter */

  RTC.RCR2.BIT.START = 1U;
  while (1U != RTC.RCR2.BIT.START)
    {
      /* Wait for the register modification to complete */
    }

  /* Set time */

  /* Set seconds. (0-59) */

  RTC.RSECAR.BYTE |= 0x80u;

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
       dummy_byte = RTC.RSECAR.BYTE;
    }

  RTC.RSECAR.BYTE |=  rtc_dec2bcd((uint8_t)alminfo->as_time.tm_sec);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = RTC.RSECAR.BYTE;
    }

  /* Set minutes (0-59) */

  RTC.RMINAR.BYTE |= 0x80u;

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
       dummy_byte = RTC.RMINAR.BYTE;
    }

  RTC.RMINAR.BYTE |= rtc_dec2bcd((uint8_t) alminfo->as_time.tm_min);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
       dummy_byte = RTC.RMINAR.BYTE;
    }

  /* Set hours. (0-23) */

  RTC.RHRAR.BYTE |= 0x80u;

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = RTC.RHRAR.BYTE;
    }

  RTC.RHRAR.BYTE |= rtc_dec2bcd((uint8_t) alminfo->as_time.tm_hour);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = RTC.RHRAR.BYTE;
    }

  /* Day of the month (1-31) */

  RTC.RDAYAR.BYTE |= 0x80u;

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
       dummy_byte = RTC.RDAYAR.BYTE;
    }

  RTC.RDAYAR.BYTE |= rtc_dec2bcd((uint8_t) alminfo->as_time.tm_mday);

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
       dummy_byte = RTC.RDAYAR.BYTE;
    }

  /* Month. (1-12, 1=January) */

  RTC.RMONAR.BYTE |= 0x80u;

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = RTC.RMONAR.BYTE;
    }

  RTC.RMONAR.BYTE |= rtc_dec2bcd((uint8_t) ((alminfo->as_time.tm_mon)+ 1));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_byte = RTC.RMONAR.BYTE;
    }

  /* Year. (00-99) */

  RTC.RYRAR.WORD = (uint16_t) (rtc_dec2bcd((uint8_t)
                   ((alminfo->as_time.tm_year) + 1900)));

  /* WAIT_LOOP */

  for (i = 0; i < RTC_DUMMY_READ; i++)
    {
      dummy_word = RTC.RYRAR.WORD;
    }

  rtc_dumpregs("New alarm setting");

  /* Enable RTC ALARM interrupt */

  RTC.RCR1.BIT.AIE = 1U;

  /* Clear IR flag of ICU ALARM interrupt */

  IR(RTC, ALM) = 0U;

  /* Enable alarm interrupts */

  IEN(RTC, ALM) = 1U;

  /* Set Priority of ALM interrupt */

  IPR(RTC, ALM) = _0F_RTC_PRIORITY_LEVEL15;
  ret = OK;
  leave_critical_section(flags);
  UNUSED(dummy_byte);
  UNUSED(dummy_word);
  return ret;
}
#endif

#ifdef CONFIG_RTC_PERIODIC
int rx65n_rtc_setperiodic(FAR const struct timespec *period,
                          periodiccb_t callback)
{
  irqstate_t flags;
  volatile uint8_t regval;
  uint8_t prd;
  flags = enter_critical_section();

  /* No.. Save the callback function pointer */

  g_periodiccb.prd_cb = callback;
  prd = period->tv_sec;

  /* Disable ICU PRD interrupt */

  IEN(RTC, PRD) = 0U;

  /* Clear IR flag of PRD interrupt */

  IR(RTC, PRD) = 0U;

  /* Set RTC control register 1 */

  regval = getreg8(RX65N_RTC_RCR1);
  regval |= 0x04 | (prd << 4);
  rx65n_putreg(regval, RX65N_RTC_RCR1);

  irq_attach(RX65N_PRD_IRQ, rtc_periodic_interrupt, NULL);

  /* Start RTC counter */

  RTC.RCR2.BIT.START = 1U;

  /* Enable ICU PRD interrupt */

  IEN(RTC, PRD) = 1U;

  /* Set PRD priority level */

  IPR(RTC, PRD) = _0F_RTC_PRIORITY_LEVEL15;
  return OK;
  leave_critical_section(flags);
}
#endif

#ifdef CONFIG_RX65N_CARRY
void rx65n_rtc_set_carry(carrycb_t callback)
{
  irqstate_t flags;
  flags = enter_critical_section();

  /* No.. Save the callback function pointer */

  g_carrycb = callback;

  /* Clear IR flag of CUP interrupt */

  IR(PERIB, INTB176) = 0U;
  irq_attach(RX65N_INTB176_IRQ, rtc_carry_interrupt, NULL);

  RTC.RCR2.BIT.START = 1U;

  /* Enable ICU CUP interrupt */

  IEN(PERIB, INTB176) = 1U;

  /* Set CUP priority level */

  ICU.SLIBR176.BYTE = 0x31u;
  IPR(PERIB, INTB176) = 15;
  RTC.RCR1.BIT.CIE = 1U;

  leave_critical_section(flags);
}

#endif
#ifdef CONFIG_RTC_ALARM
int rx65n_rtc_cancelalarm(void)
{
  irqstate_t flags;
  int ret = -ENODATA;

  flags = enter_critical_section();

  /* Cancel the global callback function */

  g_alarmcb.ac_cb = NULL;
  g_alarmcb.ac_arg = NULL;

  /* Unset the alarm */

  rx65n_putreg(0x0, RX65N_RTC_RSECAR);
  rx65n_putreg(0x0, RX65N_RTC_RMINAR);
  rx65n_putreg(0x0, RX65N_RTC_RHRAR);
  rx65n_putreg(0x0, RX65N_RTC_RWKAR);
  rx65n_putreg(0x0, RX65N_RTC_RDAYAR);
  rx65n_putreg(0x0, RX65N_RTC_RMONAR);
  rx65n_putreg(0x0, RX65N_RTC_RYRAR);
  ret = OK;

  leave_critical_section(flags);

  return ret;
}

#endif

#ifdef CONFIG_RTC_PERIODIC
int rx65n_rtc_cancelperiodic(void)
{
  /* Disable ICU PRD interrupt */

  IEN(RTC, PRD) = 0U;

  /* Clear IR flag of PRD interrupt */

  IR(RTC, PRD) = 0U;

  /* Disable RTC PRD interrupt */

  RTC.RCR1.BIT.PIE = 0U;
  while (0U != RTC.RCR1.BIT.PIE)
    {
      /* Wait for this write to complete. */
    }

  return OK;
}

#endif

#if defined(CONFIG_RX65N_CARRY)

int rx65n_rtc_cancelcarry(void)
{
  /* Clear IR flag of CUP interrupt */

  IR(PERIB, INTB176) = 0U;

  /* Disable ICU CUP interrupt */

  IEN(PERIB, INTB176) = 0U;

  /* Disable RTC CUP interrupt */

  RTC.RCR1.BIT.CIE = 0U;
  return OK;
}

#endif
/****************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used
 *   by the RTOS during initialization to set up the system time
 *   when CONFIG_RTC  and CONFIG_RTC_DATETIME
 *   are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: Some date/time RTC hardware is capability of sub-second accuracy.
 *   That sub-second accuracy is lost in this interface.  However,
 *   since the system time is reinitialized on each power-up/reset,
 *   there will be no timing inaccuracy in the long run.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_DATETIME
int up_rtc_getdatetime(FAR struct tm *tp)
{
  uint8_t weekcnt;
  uint8_t daycnt;
  uint8_t monthcnt;
  uint8_t yearcnt;
  uint8_t seccnt;
  uint8_t mincnt;
  uint8_t hrcnt;
  uint8_t tmp_week;
  uint8_t tmp_day;
  uint8_t tmp_month;
  uint8_t tmp_year;
  uint16_t bcd_years;
  uint8_t regval;

  /* Sample the data time registers.  There is a race condition here...
   * If we sample the time just before midnight on December 31,
   * the date could be wrong because the day rolled over while were sampling.
   */

  if (RTC.RCR2.BIT.START == 0)
    {
          RTC.RCR2.BIT.START = 1;
    }

  do
    {
      weekcnt  = getreg8(RX65N_RTC_RWKCNT);
      daycnt   = getreg8(RX65N_RTC_RDAYCNT);
      monthcnt = getreg8(RX65N_RTC_RMONCNT);
      yearcnt  = getreg8(RX65N_RTC_RYRCNT);
      seccnt   = getreg8(RX65N_RTC_RSECCNT);
      mincnt   = getreg8(RX65N_RTC_RMINCNT);
      hrcnt    = getreg8(RX65N_RTC_RHRCNT);
      tmp_week = getreg8(RX65N_RTC_RWKCNT);
      tmp_day  = getreg8(RX65N_RTC_RDAYCNT);
      tmp_month = getreg8(RX65N_RTC_RMONCNT);
      tmp_year  = getreg8(RX65N_RTC_RYRCNT);
    }

  while (tmp_week != weekcnt && tmp_day != daycnt &&
         tmp_month != monthcnt && tmp_year != yearcnt);

  rtc_dumpregs("Reading Time");

  /* Convert the RTC time register fields to struct tm format.
   *
   *   struct tm       TIMR register
   *   tm_sec    0-61* SEC    (0-59)
   *   tm_min    0-59  MIN    (0-59)
   *   tm_hour   0-23  HOUR   (0-23)
   *
   *  *To allow for leap seconds.  But these never actuall happen.
   */

  /* Disable ICU CUP interrupt */

  up_disable_irq(CONFIG_RX65N_PERIB);

  /* Enable RTC CUP interrupt */

  regval = getreg8(RX65N_RTC_RCR1);
  regval |= (RTC_RCR1_CUP);
  rx65n_putreg(regval, RX65N_RTC_RCR1);

  do
    {
      /* Clear carry flag in ICU */

      IR(PERIB, INTB176) = 0U;

      /* Read and convert RTC registers;
       * mask off unknown bits and hour am/pm.
       */

      /* Seconds. (0-59) */

      tp->tm_sec = rtc_bcd2dec((uint8_t) (RTC.RSECCNT.BYTE & 0x7fu));

      /* Minutes. (0-59) */

      tp->tm_min = rtc_bcd2dec((uint8_t) (RTC.RMINCNT.BYTE & 0x7fu));

      /* Hours. (0-23) */

      tp->tm_hour = rtc_bcd2dec((uint8_t) (RTC.RHRCNT.BYTE & 0x3fu));

      /* Day of the month (1-31) */

      tp->tm_mday = rtc_bcd2dec(RTC.RDAYCNT.BYTE);

      /* Months since January (0-11) */

      tp->tm_mon =  rtc_bcd2dec(RTC.RMONCNT.BYTE) - 1;

      /* Years since 2000 */

      bcd_years = (uint16_t) RTC.RYRCNT.WORD;

      /* years years since 1900 (100-199) */

      tp->tm_year = rtc_bcd2dec((uint8_t) (bcd_years & 0xff)) + 100;

      /* Days since Sunday (0-6) */

      tp->tm_wday = (int) (RTC.RWKCNT.BYTE & 0x07u);
      rtc_dumptime(tp, "Returning");
    }

  while (1 == IR(PERIB, INTB176));
  UNUSED(hrcnt);
  UNUSED(mincnt);
  UNUSED(seccnt);
  return OK;
}

#endif
#endif /* CONFIG_RX65N_RTC */
