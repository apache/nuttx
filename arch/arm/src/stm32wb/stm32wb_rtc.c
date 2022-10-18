/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_rtc.c
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

#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/time.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32wb_rcc.h"
#include "stm32wb_pwr.h"
#include "stm32wb_rtc.h"
#include "stm32wb_exti.h"

#ifdef CONFIG_STM32WB_RTC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* This RTC implementation supports
 *  - date/time RTC hardware
 *  - extended functions Alarm A and B
 * */

#ifndef CONFIG_RTC_DATETIME
#  error "CONFIG_RTC_DATETIME must be set to use this driver"
#endif

#ifdef CONFIG_RTC_HIRES
#  error "CONFIG_RTC_HIRES must NOT be set with this driver"
#endif

#ifndef CONFIG_STM32WB_PWR
#  error "CONFIG_STM32WB_PWR must selected to use this driver"
#endif

/* Constants ****************************************************************/

#define SYNCHRO_TIMEOUT               (0x00020000)
#define INITMODE_TIMEOUT              (0x00010000)

#define RTC_ALRMR_ENABLE              (0x00000000)

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
typedef unsigned int rtc_alarmreg_t;

struct alm_cbinfo_s
{
  volatile alm_callback_t ac_cb; /* Client callback function */
  volatile void *ac_arg;         /* Argument to pass with the callback function */
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
/* Callback to use when an EXTI is activated  */

static struct alm_cbinfo_s g_alarmcb[RTC_ALARM_LAST];
static bool g_alarm_enabled;  /* True: Alarm interrupts are enabled */
#endif

#ifdef CONFIG_RTC_PERIODIC
static wakeupcb_t g_wakeupcb;
static bool g_wakeup_enabled;  /* True: Wakeup interrupts are enabled */
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_rtc_enabled is set true after the RTC has successfully initialized */

volatile bool g_rtc_enabled;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int rtchw_check_alrawf(void);
static int rtchw_set_alrmar(rtc_alarmreg_t alarmreg);
#if CONFIG_RTC_NALARMS > 1
static int rtchw_check_alrbwf(void);
static int rtchw_set_alrmbr(rtc_alarmreg_t alarmreg);
#endif
static inline void rtc_enable_alarm(void);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtc_dumpregs
 ****************************************************************************/

#ifdef CONFIG_DEBUG_RTC_INFO
static void rtc_dumpregs(const char *msg)
{
  rtcinfo("%s:\n", msg);
  rtcinfo("      TR: %08x\n", getreg32(STM32WB_RTC_TR));
  rtcinfo("      DR: %08x\n", getreg32(STM32WB_RTC_DR));
  rtcinfo("      CR: %08x\n", getreg32(STM32WB_RTC_CR));
  rtcinfo("     ISR: %08x\n", getreg32(STM32WB_RTC_ISR));
  rtcinfo("    PRER: %08x\n", getreg32(STM32WB_RTC_PRER));
  rtcinfo("    WUTR: %08x\n", getreg32(STM32WB_RTC_WUTR));
  rtcinfo("  ALRMAR: %08x\n", getreg32(STM32WB_RTC_ALRMAR));
  rtcinfo("  ALRMBR: %08x\n", getreg32(STM32WB_RTC_ALRMBR));
  rtcinfo("  SHIFTR: %08x\n", getreg32(STM32WB_RTC_SHIFTR));
  rtcinfo("    TSTR: %08x\n", getreg32(STM32WB_RTC_TSTR));
  rtcinfo("    TSDR: %08x\n", getreg32(STM32WB_RTC_TSDR));
  rtcinfo("   TSSSR: %08x\n", getreg32(STM32WB_RTC_TSSSR));
  rtcinfo("    CALR: %08x\n", getreg32(STM32WB_RTC_CALR));
  rtcinfo("  TAMPCR: %08x\n", getreg32(STM32WB_RTC_TAMPCR));
  rtcinfo("ALRMASSR: %08x\n", getreg32(STM32WB_RTC_ALRMASSR));
  rtcinfo("ALRMBSSR: %08x\n", getreg32(STM32WB_RTC_ALRMBSSR));
  rtcinfo("      OR: %08x\n", getreg32(STM32WB_RTC_OR));
  rtcinfo("MAGICREG: %08x\n", getreg32(RTC_MAGIC_REG));
}
#else
#  define rtc_dumpregs(msg)
#endif

/****************************************************************************
 * Name: rtc_dumptime
 ****************************************************************************/

#ifdef CONFIG_DEBUG_RTC_INFO
static void rtc_dumptime(const struct tm *tp, const char *msg)
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
 * Name: rtc_wprunlock
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

static void rtc_wprunlock(void)
{
  /* Enable write access to the backup domain. */

  stm32wb_pwr_enablebkp(true);

  /* The following steps are required to unlock the write protection on
   * all the RTC registers (except for RTC_ISR[13:8], RTC_TAFCR, and
   * RTC_BKPxR).
   *
   * 1. Write 0xca into the RTC_WPR register.
   * 2. Write 0x53 into the RTC_WPR register.
   *
   * Writing a wrong key re-activates the write protection.
   */

  putreg32(RTC_WPR_KEY1, STM32WB_RTC_WPR);
  putreg32(RTC_WPR_KEY2, STM32WB_RTC_WPR);
}

/****************************************************************************
 * Name: rtc_wprlock
 *
 * Description:
 *    Enable RTC write protection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rtc_wprlock(void)
{
  /* Writing any wrong key re-activates the write protection. */

  putreg32(0xff, STM32WB_RTC_WPR);

  /* Disable write access to the backup domain. */

  stm32wb_pwr_enablebkp(false);
}

/****************************************************************************
 * Name: rtc_synchwait
 *
 * Description:
 *   Waits until the RTC Time and Date registers (RTC_TR and RTC_DR) are
 *   synchronized with RTC APB clock.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

static int rtc_synchwait(void)
{
  volatile uint32_t timeout;
  uint32_t regval;
  int ret;

  /* Clear Registers synchronization flag (RSF) */

  regval  = getreg32(STM32WB_RTC_ISR);
  regval &= ~RTC_ISR_RSF;
  putreg32(regval, STM32WB_RTC_ISR);

  /* Now wait the registers to become synchronised */

  ret = -ETIMEDOUT;
  for (timeout = 0; timeout < SYNCHRO_TIMEOUT; timeout++)
    {
      regval = getreg32(STM32WB_RTC_ISR);
      if ((regval & RTC_ISR_RSF) != 0)
        {
          /* Synchronized */

          ret = OK;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: rtc_enterinit
 *
 * Description:
 *   Enter RTC initialization mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

static int rtc_enterinit(void)
{
  volatile uint32_t timeout;
  uint32_t regval;
  int ret;

  /* Check if the Initialization mode is already set */

  regval = getreg32(STM32WB_RTC_ISR);

  ret = OK;
  if ((regval & RTC_ISR_INITF) == 0)
    {
      /* Set the Initialization mode */

      putreg32(RTC_ISR_INIT, STM32WB_RTC_ISR);

      /* Wait until the RTC is in the INIT state (or a timeout occurs) */

      ret = -ETIMEDOUT;
      for (timeout = 0; timeout < INITMODE_TIMEOUT; timeout++)
        {
          regval = getreg32(STM32WB_RTC_ISR);
          if ((regval & RTC_ISR_INITF) != 0)
            {
              ret = OK;
              break;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: rtc_exitinit
 *
 * Description:
 *   Exit RTC initialization mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

static void rtc_exitinit(void)
{
  uint32_t regval;

  regval = getreg32(STM32WB_RTC_ISR);
  regval &= ~(RTC_ISR_INIT);
  putreg32(regval, STM32WB_RTC_ISR);
}

/****************************************************************************
 * Name: rtc_bin2bcd
 *
 * Description:
 *   Converts a 2 digit binary to BCD format
 *
 * Input Parameters:
 *   value - The byte to be converted.
 *
 * Returned Value:
 *   The value in BCD representation
 *
 ****************************************************************************/

static uint32_t rtc_bin2bcd(int value)
{
  uint32_t msbcd = 0;

  while (value >= 10)
    {
      msbcd++;
      value -= 10;
    }

  return (msbcd << 4) | value;
}

/****************************************************************************
 * Name: rtc_bin2bcd
 *
 * Description:
 *   Convert from 2 digit BCD to binary.
 *
 * Input Parameters:
 *   value - The BCD value to be converted.
 *
 * Returned Value:
 *   The value in binary representation
 *
 ****************************************************************************/

static int rtc_bcd2bin(uint32_t value)
{
  uint32_t tens = (value >> 4) * 10;
  return (int)(tens + (value & 0x0f));
}

/****************************************************************************
 * Name: rtc_resume
 *
 * Description:
 *   Called when the RTC was already initialized on a previous power cycle.
 *   This just brings the RTC back into full operation.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

static void rtc_resume(void)
{
#ifdef CONFIG_RTC_ALARM
  uint32_t regval;

  /* Clear the RTC alarm flags */

  regval  = getreg32(STM32WB_RTC_ISR);
  regval &= ~(RTC_ISR_ALRAF | RTC_ISR_ALRBF);
  putreg32(regval, STM32WB_RTC_ISR);

  /* Clear the EXTI Line 17 Pending bit (Connected internally to RTC Alarm) */

  putreg32(EXTI_PR1_PIF(EXTI_EVT_RTCALARM), STM32WB_EXTI_PR1);
#endif
}

/****************************************************************************
 * Name: stm32wb_rtc_alarm_handler
 *
 * Description:
 *   RTC ALARM interrupt service routine through the EXTI line
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
static int stm32wb_rtc_alarm_handler(int irq, void *context,
                                     void *rtc_handler_arg)
{
  struct alm_cbinfo_s *cbinfo;
  alm_callback_t cb;
  void *arg;
  uint32_t isr;
  uint32_t cr;
  int ret = OK;

  /* Enable write access to the backup domain (RTC registers, RTC
   * backup data registers and backup SRAM).
   */

  stm32wb_pwr_enablebkp(true);

  /* Check for EXTI from Alarm A or B and handle according */

  cr  = getreg32(STM32WB_RTC_CR);
  if ((cr & RTC_CR_ALRAIE) != 0)
    {
      isr  = getreg32(STM32WB_RTC_ISR);
      if ((isr & RTC_ISR_ALRAF) != 0)
        {
          cbinfo = &g_alarmcb[RTC_ALARMA];
          if (cbinfo->ac_cb != NULL)
            {
              /* Alarm A callback */

              cb  = cbinfo->ac_cb;
              arg = (void *)cbinfo->ac_arg;

              cbinfo->ac_cb  = NULL;
              cbinfo->ac_arg = NULL;

              cb(arg, RTC_ALARMA);
            }

          /* note, bits 8-13 do /not/ require the write enable procedure */

          isr  = getreg32(STM32WB_RTC_ISR);
          isr &= ~RTC_ISR_ALRAF;
          putreg32(isr, STM32WB_RTC_ISR);
        }
    }

#if CONFIG_RTC_NALARMS > 1
  cr  = getreg32(STM32WB_RTC_CR);
  if ((cr & RTC_CR_ALRBIE) != 0)
    {
      isr  = getreg32(STM32WB_RTC_ISR);
      if ((isr & RTC_ISR_ALRBF) != 0)
        {
          cbinfo = &g_alarmcb[RTC_ALARMB];
          if (cbinfo->ac_cb != NULL)
            {
              /* Alarm B callback */

              cb  = cbinfo->ac_cb;
              arg = (void *)cbinfo->ac_arg;

              cbinfo->ac_cb  = NULL;
              cbinfo->ac_arg = NULL;

              cb(arg, RTC_ALARMB);
            }

          /* note, bits 8-13 do /not/ require the write enable procedure */

          isr  = getreg32(STM32WB_RTC_ISR);
          isr &= ~RTC_ISR_ALRBF;
          putreg32(isr, STM32WB_RTC_ISR);
        }
    }
#endif

  /* Disable write access to the backup domain (RTC registers, RTC backup
   * data registers and backup SRAM).
   */

  stm32wb_pwr_enablebkp(false);

  return ret;
}
#endif

/****************************************************************************
 * Name: rtchw_check_alrXwf X= a or B
 *
 * Description:
 *   Check registers
 *
 * Input Parameters:
 * None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int rtchw_check_alrawf(void)
{
  volatile uint32_t timeout;
  uint32_t regval;
  int ret = -ETIMEDOUT;

  /* Check RTC_ISR ALRAWF for access to alarm register,
   * Can take 2 RTCCLK cycles or timeout
   * CubeMX use GetTick.
   */

  for (timeout = 0; timeout < INITMODE_TIMEOUT; timeout++)
    {
      regval = getreg32(STM32WB_RTC_ISR);
      if ((regval & RTC_ISR_ALRAWF) != 0)
        {
          ret = OK;
          break;
        }
    }

  return ret;
}
#endif

#if defined(CONFIG_RTC_ALARM) && CONFIG_RTC_NALARMS > 1
static int rtchw_check_alrbwf(void)
{
  volatile uint32_t timeout;
  uint32_t regval;
  int ret = -ETIMEDOUT;

  /* Check RTC_ISR ALRBWF for access to alarm register,
   * can take 2 RTCCLK cycles or timeout
   * CubeMX use GetTick.
   */

  for (timeout = 0; timeout < INITMODE_TIMEOUT; timeout++)
    {
      regval = getreg32(STM32WB_RTC_ISR);
      if ((regval & RTC_ISR_ALRBWF) != 0)
        {
          ret = OK;
          break;
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: stm32wb_rtchw_set_alrmXr X is a or b
 *
 * Description:
 *   Set the alarm (A or B) hardware registers, using the required hardware
 *   access protocol
 *
 * Input Parameters:
 *   alarmreg - the register
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int rtchw_set_alrmar(rtc_alarmreg_t alarmreg)
{
  int isr;
  int ret = -EBUSY;

  /* Disable the write protection for RTC registers */

  rtc_wprunlock();

  /* Disable RTC alarm A & Interrupt A */

  modifyreg32(STM32WB_RTC_CR, (RTC_CR_ALRAE | RTC_CR_ALRAIE), 0);

  /* Ensure Alarm A flag reset; this is edge triggered */

  isr  = getreg32(STM32WB_RTC_ISR) & ~RTC_ISR_ALRAF;
  putreg32(isr, STM32WB_RTC_ISR);

  /* Wait for Alarm A to be writable */

  ret = rtchw_check_alrawf();
  if (ret != OK)
    {
      goto errout_with_wprunlock;
    }

  /* Set the RTC Alarm A register */

  putreg32(alarmreg, STM32WB_RTC_ALRMAR);
  putreg32(0, STM32WB_RTC_ALRMASSR);
  rtcinfo("  ALRMAR: %08" PRIx32 "\n", getreg32(STM32WB_RTC_ALRMAR));

  /* Enable RTC alarm A */

  modifyreg32(STM32WB_RTC_CR, 0, (RTC_CR_ALRAE | RTC_CR_ALRAIE));

errout_with_wprunlock:
  rtc_wprlock();
  return ret;
}
#endif

#if defined(CONFIG_RTC_ALARM) && CONFIG_RTC_NALARMS > 1
static int rtchw_set_alrmbr(rtc_alarmreg_t alarmreg)
{
  int isr;
  int ret = -EBUSY;

  /* Disable the write protection for RTC registers */

  rtc_wprunlock();

  /* Disable RTC alarm B & Interrupt B */

  modifyreg32(STM32WB_RTC_CR, (RTC_CR_ALRBE | RTC_CR_ALRBIE), 0);

  /* Ensure Alarm B flag reset; this is edge triggered */

  isr  = getreg32(STM32WB_RTC_ISR) & ~RTC_ISR_ALRBF;
  putreg32(isr, STM32WB_RTC_ISR);

  /* Wait for Alarm B to be writable */

  ret = rtchw_check_alrbwf();
  if (ret != OK)
    {
      goto rtchw_set_alrmbr_exit;
    }

  /* Set the RTC Alarm B register */

  putreg32(alarmreg, STM32WB_RTC_ALRMBR);
  putreg32(0, STM32WB_RTC_ALRMBSSR);
  rtcinfo("  ALRMBR: %08" PRIx32 "\n", getreg32(STM32WB_RTC_ALRMBR));

  /* Enable RTC alarm B */

  modifyreg32(STM32WB_RTC_CR, 0, (RTC_CR_ALRBE | RTC_CR_ALRBIE));

rtchw_set_alrmbr_exit:
  rtc_wprlock();
  return ret;
}
#endif

/****************************************************************************
 * Name: rtc_enable_alarm
 *
 * Description:
 *   Enable ALARM interrupts
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static inline void rtc_enable_alarm(void)
{
  /* Is the alarm already enabled? */

  if (!g_alarm_enabled)
    {
      /* Configure RTC interrupt to catch alarm interrupts. All RTC
       * interrupts are connected to the EXTI controller.  To enable the
       * RTC Alarm interrupt, the following sequence is required:
       *
       * 1. Configure and enable the EXTI line 17 (RTC ALARM) in interrupt
       *    mode and select the rising edge sensitivity.
       *    EXTI line 18 RTC Tamper or Timestamp or CSS_LSE
       *    EXTI line 19 RTC Wakeup
       * 2. Configure and enable the RTC_Alarm IRQ channel in the NVIC.
       * 3. Configure the RTC to generate RTC alarms (Alarm A or Alarm B).
       */

      stm32wb_exti_alarm(true, false, true, stm32wb_rtc_alarm_handler, NULL);
      g_alarm_enabled = true;
    }
}
#endif

/****************************************************************************
 * Name: stm32wb_rtc_getalarmdatetime
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
static int stm32wb_rtc_getalarmdatetime(rtc_alarmreg_t reg,
                                        struct tm *tp)
{
  uint32_t data;
  uint32_t tmp;

  DEBUGASSERT(tp != NULL);

  /* Sample the data time register. */

  data = getreg32(reg);

  /* Convert the RTC time to fields in struct tm format.  All of the STM32WB
   * ranges of values correspond between struct tm and the time register.
   */

  tmp = (data & (RTC_ALRMR_SU_MASK | RTC_ALRMR_ST_MASK)) >>
        RTC_ALRMR_SU_SHIFT;
  tp->tm_sec = rtc_bcd2bin(tmp);

  tmp = (data & (RTC_ALRMR_MNU_MASK | RTC_ALRMR_MNT_MASK)) >>
        RTC_ALRMR_MNU_SHIFT;
  tp->tm_min = rtc_bcd2bin(tmp);

  tmp = (data & (RTC_ALRMR_HU_MASK | RTC_ALRMR_HT_MASK)) >>
        RTC_ALRMR_HU_SHIFT;
  tp->tm_hour = rtc_bcd2bin(tmp);

  tmp = (data & (RTC_ALRMR_DU_MASK | RTC_ALRMR_DT_MASK)) >>
        RTC_ALRMR_DU_SHIFT;
  tp->tm_mday = rtc_bcd2bin(tmp);

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_rtc_is_initialized
 *
 * Description:
 *    Returns 'true' if the RTC has been initialized
 *    Returns 'false' if the RTC has never been initialized since first
 *    time power up, and the counters are stopped until it is first
 *    initialized.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns true if RTC has been initialized.
 *
 ****************************************************************************/

bool stm32wb_rtc_is_initialized(void)
{
  uint32_t regval;

  regval = getreg32(RTC_MAGIC_REG);

  return regval == RTC_MAGIC || regval == RTC_MAGIC_TIME_SET;
}

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.  This
 *   function is called once during the OS initialization sequence
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
  bool init_stat;
  uint32_t regval;

  rtc_dumpregs("Before Initialization");

  /* See if the clock has already been initialized; since it is battery
   * backed, we don't need or want to re-initialize on each reset.
   */

  init_stat = stm32wb_rtc_is_initialized();
  if (!init_stat)
    {
      /* Enable write access to the backup domain (RTC registers, RTC
       * backup data registers and backup SRAM).
       */

      stm32wb_pwr_enablebkp(true);

#if defined(CONFIG_STM32WB_RTC_HSECLOCK)
      modifyreg32(STM32WB_RCC_BDCR, RCC_BDCR_RTCSEL_MASK,
                  RCC_BDCR_RTCSEL_HSE);
#elif defined(CONFIG_STM32WB_RTC_LSICLOCK)
      modifyreg32(STM32WB_RCC_BDCR, RCC_BDCR_RTCSEL_MASK,
                  RCC_BDCR_RTCSEL_LSI);
#elif defined(CONFIG_STM32WB_RTC_LSECLOCK)
      modifyreg32(STM32WB_RCC_BDCR, RCC_BDCR_RTCSEL_MASK,
                  RCC_BDCR_RTCSEL_LSE);
#else
#  error "No clock for RTC!"
#endif

      /* Enable the RTC Clock by setting the RTCEN bit in the RCC register */

      modifyreg32(STM32WB_RCC_BDCR, 0, RCC_BDCR_RTCEN);

      /* Disable the write protection for RTC registers */

      rtc_wprunlock();

      /* Set Initialization mode */

      if (rtc_enterinit() != OK)
        {
          /* Enable the write protection for RTC registers */

          rtc_wprlock();

          /* Disable write access to the backup domain (RTC registers,
           * RTC backup data registers and backup SRAM).
           */

          stm32wb_pwr_enablebkp(false);

          rtc_dumpregs("After Failed Initialization");

          return ERROR;
        }
      else
        {
          /* Clear RTC_CR FMT, OSEL and POL Bits */

          regval = getreg32(STM32WB_RTC_CR);
          regval &= ~(RTC_CR_FMT | RTC_CR_OSEL_MASK | RTC_CR_POL);
          putreg32(regval, STM32WB_RTC_CR);

          /* Configure RTC pre-scaler with the required values */

#ifdef CONFIG_STM32WB_RTC_HSECLOCK
          /* The HSE is divided by 32 prior to the prescaler we set here.
           * 1953
           * NOTE: max HSE/32 is 4 MHz if it is to be used with RTC
           */

          /* For a 1 MHz clock this yields 0.9999360041 Hz on the second
           * timer - which is pretty close.
           */

          putreg32(((uint32_t)7812 << RTC_PRER_PREDIV_S_SHIFT) |
                   ((uint32_t)0x7f << RTC_PRER_PREDIV_A_SHIFT),
                   STM32WB_RTC_PRER);
#elif defined(CONFIG_STM32WB_RTC_LSICLOCK)
          /* Suitable values for 32.000 KHz LSI clock (29.5 - 34 KHz,
           * though)
           */

          putreg32(((uint32_t)0xf9 << RTC_PRER_PREDIV_S_SHIFT) |
                   ((uint32_t)0x7f << RTC_PRER_PREDIV_A_SHIFT),
                   STM32WB_RTC_PRER);
#else /* defined(CONFIG_STM32WB_RTC_LSECLOCK) */
          /* Correct values for 32.768 KHz LSE clock */

          putreg32(((uint32_t)0xff << RTC_PRER_PREDIV_S_SHIFT) |
                   ((uint32_t)0x7f << RTC_PRER_PREDIV_A_SHIFT),
                   STM32WB_RTC_PRER);
#endif

          /* Exit Initialization mode */

          rtc_exitinit();

          /* Wait for the RTC Time and Date registers to be synchronized
           * with RTC APB clock.
           */

          rtc_synchwait();

          /* Keep the fact that the RTC is initialized */

          putreg32(RTC_MAGIC, RTC_MAGIC_REG);

          /* Enable the write protection for RTC registers */

          rtc_wprlock();

          /* Disable write access to the backup domain (RTC registers,
           * RTC backup data registers and backup SRAM).
           */

          stm32wb_pwr_enablebkp(false);
        }
    }
  else
    {
      /* Enable write access to the backup domain (RTC registers, RTC
       * backup data registers and backup SRAM).
       */

      stm32wb_pwr_enablebkp(true);

      /* Write protection for RTC registers does not need to be disabled. */

      rtc_resume();

      /* Disable write access to the backup domain (RTC registers, RTC backup
       * data registers and backup SRAM).
       */

      stm32wb_pwr_enablebkp(false);
    }

  g_rtc_enabled = true;
  rtc_dumpregs("After Initialization");

  return OK;
}

/****************************************************************************
 * Name: stm32wb_rtc_getdatetime_with_subseconds
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS
 *   during initialization to set up the system time when CONFIG_RTC and
 *   CONFIG_RTC_DATETIME are selected.
 *
 *   Sub-second accuracy is returned through 'nsec'.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *   nsec - The location to return the subsecond time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int stm32wb_rtc_getdatetime_with_subseconds(struct tm *tp,
                                            long *nsec)
{
#ifdef CONFIG_STM32WB_HAVE_RTC_SUBSECONDS
  uint32_t ssr;
#endif
  uint32_t dr;
  uint32_t tr;
  uint32_t tmp;

  /* Sample the data time registers.  There is a race condition here... If
   * we sample the time just before midnight on December 31, the date could
   * be wrong because the day rolled over while were sampling. Thus loop for
   * checking overflow here is needed.  There is a race condition with
   * subseconds too. If we sample TR register just before second rolling
   * and subseconds are read at wrong second, we get wrong time.
   */

  do
    {
      dr  = getreg32(STM32WB_RTC_DR);
      tr  = getreg32(STM32WB_RTC_TR);
#ifdef CONFIG_STM32WB_HAVE_RTC_SUBSECONDS
      ssr = getreg32(STM32WB_RTC_SSR);
      tmp = getreg32(STM32WB_RTC_TR);
      if (tmp != tr)
        {
          continue;
        }
#endif

      tmp = getreg32(STM32WB_RTC_DR);
    }
  while (tmp != dr);

  rtc_dumpregs("Reading Time");

  /* Convert the RTC time to fields in struct tm format. All of the STM32WB
   * ranges of values correspond between struct tm and the time register.
   */

  tmp = (tr & (RTC_TR_SU_MASK | RTC_TR_ST_MASK)) >> RTC_TR_SU_SHIFT;
  tp->tm_sec = rtc_bcd2bin(tmp);

  tmp = (tr & (RTC_TR_MNU_MASK | RTC_TR_MNT_MASK)) >> RTC_TR_MNU_SHIFT;
  tp->tm_min = rtc_bcd2bin(tmp);

  tmp = (tr & (RTC_TR_HU_MASK | RTC_TR_HT_MASK)) >> RTC_TR_HU_SHIFT;
  tp->tm_hour = rtc_bcd2bin(tmp);

  /* Now convert the RTC date to fields in struct tm format:
   * Days: 1-31 match in both cases.
   * Month: STM32 is 1-12, struct tm is 0-11.
   * Years: STM32 is 00-99, struct tm is years since 1900.
   * WeekDay: STM32 is 1 = Mon - 7 = Sun
   *
   * Issue:  I am not sure what the STM32 years mean.  Are these the
   * years 2000-2099?  I'll assume so.
   */

  tmp = (dr & (RTC_DR_DU_MASK | RTC_DR_DT_MASK)) >> RTC_DR_DU_SHIFT;
  tp->tm_mday = rtc_bcd2bin(tmp);

  tmp = (dr & (RTC_DR_MU_MASK | RTC_DR_MT)) >> RTC_DR_MU_SHIFT;
  tp->tm_mon = rtc_bcd2bin(tmp) - 1;

  tmp = (dr & (RTC_DR_YU_MASK | RTC_DR_YT_MASK)) >> RTC_DR_YU_SHIFT;
  tp->tm_year = rtc_bcd2bin(tmp) + 100;

  tmp = (dr & RTC_DR_WDU_MASK) >> RTC_DR_WDU_SHIFT;
  tp->tm_wday = tmp % 7;
  tp->tm_yday = tp->tm_mday - 1 +
                clock_daysbeforemonth(tp->tm_mon,
                                      clock_isleapyear(tp->tm_year + 1900));
  tp->tm_isdst = 0;

  /* Return RTC sub-seconds if a non-NULL value
   * of nsec has been provided to receive the sub-second value.
   */

#ifdef CONFIG_STM32WB_HAVE_RTC_SUBSECONDS
  if (nsec)
    {
      uint32_t prediv_s;
      uint32_t usecs;

      prediv_s   = getreg32(STM32WB_RTC_PRER) & RTC_PRER_PREDIV_S_MASK;
      prediv_s >>= RTC_PRER_PREDIV_S_SHIFT;

      ssr &= RTC_SSR_MASK;

      /* Maximum prediv_s is 0x7fff, thus we can multiply by 100000 and
       * still fit 32-bit unsigned integer.
       */

      usecs = (((prediv_s - ssr) * 100000) / (prediv_s + 1)) * 10;
      *nsec = usecs * 1000;
    }
#else
  DEBUGASSERT(nsec == NULL);
#endif

  rtc_dumptime(tp, "Returning");
  return OK;
}

/****************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS
 *   during initialization to set up the system time when CONFIG_RTC and
 *   CONFIG_RTC_DATETIME are selected.
 *
 *   NOTE: Some date/time RTC hardware is capability of sub-second accuracy.
 *   That sub-second accuracy is lost in this interface.  However, since the
 *   system time is reinitialized on each power-up/reset, there will be no
 *   timing inaccuracy in the long run.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_getdatetime(struct tm *tp)
{
  return stm32wb_rtc_getdatetime_with_subseconds(tp, NULL);
}

/****************************************************************************
 * Name: up_rtc_getdatetime_with_subseconds
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS
 *   during initialization to set up the system time when CONFIG_RTC and
 *   CONFIG_RTC_DATETIME are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: This interface exposes sub-second accuracy capability of RTC
 *   hardware.  This interface allow maintaining timing accuracy when system
 *   time needs constant resynchronization with RTC, for example with board
 *   level power-save mode utilizing deep-sleep modes such as STOP on STM32WB
 *   MCUs.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *   nsec - The location to return the subsecond time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_RTC_SUBSECONDS
#  ifndef CONFIG_STM32WB_HAVE_RTC_SUBSECONDS
#    error "Invalid config, enable CONFIG_STM32WB_HAVE_RTC_SUBSECONDS."
#  endif
int up_rtc_getdatetime_with_subseconds(struct tm *tp, long *nsec)
{
  return stm32wb_rtc_getdatetime_with_subseconds(tp, nsec);
}
#endif

/****************************************************************************
 * Name: stm32wb_rtc_setdatetime
 *
 * Description:
 *   Set the RTC to the provided time. RTC implementations which provide
 *   up_rtc_getdatetime() (CONFIG_RTC_DATETIME is selected) should provide
 *   this function.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int stm32wb_rtc_setdatetime(const struct tm *tp)
{
  uint32_t tr;
  uint32_t dr;
  int ret;

  rtc_dumptime(tp, "Setting time");

  /* Then write the broken out values to the RTC */

  /* Convert the struct tm format to RTC time register fields.
   * All of the ranges of values correspond between struct tm and the time
   * register.
   */

  tr = (rtc_bin2bcd(tp->tm_sec)  << RTC_TR_SU_SHIFT) |
       (rtc_bin2bcd(tp->tm_min)  << RTC_TR_MNU_SHIFT) |
       (rtc_bin2bcd(tp->tm_hour) << RTC_TR_HU_SHIFT);
  tr &= ~RTC_TR_RESERVED_BITS;

  /* Now convert the fields in struct tm format to the RTC date register
   * fields:
   * Days: 1-31 match in both cases.
   * Month: STM32 is 1-12, struct tm is 0-11.
   * Years: STM32 is 00-99, struct tm is years since 1900.
   * WeekDay: STM32 is 1 = Mon - 7 = Sun
   * Issue:  I am not sure what the STM32 years mean.  Are these the
   * years 2000-2099?  I'll assume so.
   */

  dr = (rtc_bin2bcd(tp->tm_mday) << RTC_DR_DU_SHIFT) |
       ((rtc_bin2bcd(tp->tm_mon + 1))  << RTC_DR_MU_SHIFT) |
       ((tp->tm_wday == 0 ? 7 : (tp->tm_wday & 7))  << RTC_DR_WDU_SHIFT) |
       ((rtc_bin2bcd(tp->tm_year - 100)) << RTC_DR_YU_SHIFT);

  dr &= ~RTC_DR_RESERVED_BITS;

  /* Disable the write protection for RTC registers */

  rtc_wprunlock();

  /* Set Initialization mode */

  ret = rtc_enterinit();
  if (ret == OK)
    {
      /* Set the RTC TR and DR registers */

      putreg32(tr, STM32WB_RTC_TR);
      putreg32(dr, STM32WB_RTC_DR);

      /* Exit Initialization mode and wait for the RTC Time and Date
       * registers to be synchronized with RTC APB clock.
       */

      rtc_exitinit();
      ret = rtc_synchwait();
    }

  /* Remember that the RTC is initialized and had its time set. */

  if (getreg32(RTC_MAGIC_REG) != RTC_MAGIC_TIME_SET)
    {
      stm32wb_pwr_enablebkp(true);
      putreg32(RTC_MAGIC_TIME_SET, RTC_MAGIC_REG);
      stm32wb_pwr_enablebkp(false);
    }

  /* Re-enable the write protection for RTC registers */

  rtc_wprlock();
  rtc_dumpregs("New time setting");
  return ret;
}

/****************************************************************************
 * Name: stm32wb_rtc_havesettime
 *
 * Description:
 *   Check if RTC time has been set.
 *
 * Returned Value:
 *   Returns true if RTC date-time have been previously set.
 *
 ****************************************************************************/

bool stm32wb_rtc_havesettime(void)
{
  return getreg32(RTC_MAGIC_REG) == RTC_MAGIC_TIME_SET;
}

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be able
 *   to set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_settime(const struct timespec *tp)
{
  struct tm newtime;

  /* Break out the time values (not that the time is set only to units of
   * seconds)
   */

  gmtime_r(&tp->tv_sec, &newtime);
  return stm32wb_rtc_setdatetime(&newtime);
}

/****************************************************************************
 * Name: stm32wb_rtc_setalarm
 *
 * Description:
 *   Set an alarm to an absolute time using associated hardware.
 *
 * Input Parameters:
 *  alminfo - Information about the alarm configuration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int stm32wb_rtc_setalarm(struct alm_setalarm_s *alminfo)
{
  struct alm_cbinfo_s *cbinfo;
  rtc_alarmreg_t alarmreg;
  int ret = -EINVAL;

  DEBUGASSERT(alminfo != NULL);
  DEBUGASSERT(RTC_ALARM_LAST > alminfo->as_id);

  /* Make sure the alarm interrupt is enabled at the NVIC */

  rtc_enable_alarm();

  /* REVISIT:  Should test that the time is in the future */

  rtc_dumptime(&alminfo->as_time, "New alarm time");

  /* Break out the values to the HW alarm register format.  The values in
   * all STM32WB fields match the fields of struct tm in this case.  Notice
   * that the alarm is limited to one month.
   */

  alarmreg = (rtc_bin2bcd(alminfo->as_time.tm_sec)  << RTC_ALRMR_SU_SHIFT) |
             (rtc_bin2bcd(alminfo->as_time.tm_min)  << RTC_ALRMR_MNU_SHIFT) |
             (rtc_bin2bcd(alminfo->as_time.tm_hour) << RTC_ALRMR_HU_SHIFT) |
             (rtc_bin2bcd(alminfo->as_time.tm_mday) << RTC_ALRMR_DU_SHIFT);

  /* Set the alarm in hardware and enable interrupts from the RTC */

  switch (alminfo->as_id)
    {
      case RTC_ALARMA:
        {
          cbinfo         = &g_alarmcb[RTC_ALARMA];
          cbinfo->ac_cb  = alminfo->as_cb;
          cbinfo->ac_arg = alminfo->as_arg;

          ret = rtchw_set_alrmar(alarmreg | RTC_ALRMR_ENABLE);
          if (ret < 0)
            {
              cbinfo->ac_cb  = NULL;
              cbinfo->ac_arg = NULL;
            }
        }
        break;

#if CONFIG_RTC_NALARMS > 1
      case RTC_ALARMB:
        {
          cbinfo         = &g_alarmcb[RTC_ALARMB];
          cbinfo->ac_cb  = alminfo->as_cb;
          cbinfo->ac_arg = alminfo->as_arg;

          ret = rtchw_set_alrmbr(alarmreg | RTC_ALRMR_ENABLE);
          if (ret < 0)
            {
              cbinfo->ac_cb  = NULL;
              cbinfo->ac_arg = NULL;
            }
        }
        break;
#endif

      default:
        rtcerr("ERROR: Invalid ALARM%d\n", alminfo->as_id);
        break;
    }

  rtc_dumpregs("After alarm setting");

  return ret;
}
#endif

/****************************************************************************
 * Name: stm32wb_rtc_cancelalarm
 *
 * Description:
 *   Cancel an alarm.
 *
 * Input Parameters:
 *  alarmid - Identifies the alarm to be cancelled
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
int stm32wb_rtc_cancelalarm(enum alm_id_e alarmid)
{
  int ret = -EINVAL;

  DEBUGASSERT(RTC_ALARM_LAST > alarmid);

  /* Cancel the alarm in hardware and disable interrupts */

  switch (alarmid)
    {
      case RTC_ALARMA:
        {
          /* Cancel the global callback function */

           g_alarmcb[alarmid].ac_cb  = NULL;
           g_alarmcb[alarmid].ac_arg = NULL;

          /* Disable the write protection for RTC registers */

          rtc_wprunlock();

          /* Disable RTC alarm and interrupt */

          modifyreg32(STM32WB_RTC_CR, (RTC_CR_ALRAE | RTC_CR_ALRAIE), 0);

          ret = rtchw_check_alrawf();
          if (ret < 0)
            {
              goto errout_with_wprunlock;
            }

          /* Unset the alarm */

          putreg32(0xffffffff, STM32WB_RTC_ALRMAR);
          modifyreg32(STM32WB_RTC_ISR, RTC_ISR_ALRAF, 0);
          rtc_wprlock();
          ret = OK;
        }
        break;

#if CONFIG_RTC_NALARMS > 1
      case RTC_ALARMB:
        {
          /* Cancel the global callback function */

           g_alarmcb[alarmid].ac_cb  = NULL;
           g_alarmcb[alarmid].ac_arg = NULL;

          /* Disable the write protection for RTC registers */

          rtc_wprunlock();

          /* Disable RTC alarm and interrupt */

          modifyreg32(STM32WB_RTC_CR, (RTC_CR_ALRBE | RTC_CR_ALRBIE), 0);

          ret = rtchw_check_alrbwf();
          if (ret < 0)
            {
              goto errout_with_wprunlock;
            }

          /* Unset the alarm */

          putreg32(0xffffffff, STM32WB_RTC_ALRMBR);
          modifyreg32(STM32WB_RTC_ISR, RTC_ISR_ALRBF, 0);
          rtc_wprlock();
          ret = OK;
        }
        break;
#endif

      default:
        rtcerr("ERROR: Invalid ALARM%d\n", alarmid);
        break;
    }

  return ret;

errout_with_wprunlock:
  rtc_wprlock();
  return ret;
}
#endif

/****************************************************************************
 * Name: stm32wb_rtc_rdalarm
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
int stm32wb_rtc_rdalarm(struct alm_rdalarm_s *alminfo)
{
  rtc_alarmreg_t alarmreg;
  int ret = -EINVAL;

  DEBUGASSERT(alminfo != NULL);
  DEBUGASSERT(RTC_ALARM_LAST > alminfo->ar_id);

  switch (alminfo->ar_id)
    {
      case RTC_ALARMA:
        {
          alarmreg = STM32WB_RTC_ALRMAR;
          ret = stm32wb_rtc_getalarmdatetime(alarmreg,
                                             (struct tm *)alminfo->ar_time);
        }
        break;

#if CONFIG_RTC_NALARMS > 1
      case RTC_ALARMB:
        {
          alarmreg = STM32WB_RTC_ALRMBR;
          ret = stm32wb_rtc_getalarmdatetime(alarmreg,
                                             (struct tm *)alminfo->ar_time);
        }
        break;
#endif

      default:
        rtcerr("ERROR: Invalid ALARM%d\n", alminfo->ar_id);
        break;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: stm32wb_rtc_wakeup_handler
 *
 * Description:
 *   RTC WAKEUP interrupt service routine through the EXTI line
 *
 * Input Parameters:
 *   irq - The IRQ number that generated the interrupt
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_PERIODIC
static int stm32wb_rtc_wakeup_handler(int irq, void *context, void *arg)
{
  uint32_t regval = 0;

  stm32wb_pwr_enablebkp(true);

  regval = getreg32(STM32WB_RTC_ISR);
  regval &= ~RTC_ISR_WUTF;
  putreg32(regval, STM32WB_RTC_ISR);

  stm32wb_pwr_enablebkp(false);

  if (g_wakeupcb != NULL)
    {
      g_wakeupcb();
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: rtc_enable_wakeup
 *
 * Description:
 *   Enable periodic wakeup interrupts
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_PERIODIC
static inline void rtc_enable_wakeup(void)
{
  if (!g_wakeup_enabled)
    {
      stm32wb_exti_wakeup(true, false, true, stm32wb_rtc_wakeup_handler,
                          NULL);
      g_wakeup_enabled = true;
    }
}
#endif

/****************************************************************************
 * Name: rtc_set_wcksel
 *
 * Description:
 *    Sets RTC wakeup clock selection value
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_PERIODIC
static inline void rtc_set_wcksel(unsigned int wucksel)
{
  uint32_t regval = 0;

  regval = getreg32(STM32WB_RTC_CR);
  regval &= ~RTC_CR_WUCKSEL_MASK;
  regval |= wucksel;
  putreg32(regval, STM32WB_RTC_CR);
}
#endif

/****************************************************************************
 * Name: stm32wb_rtc_setperiodic
 *
 * Description:
 *   Set a periodic RTC wakeup
 *
 * Input Parameters:
 *  period   - Time to sleep between wakeups
 *  callback - Function to call when the period expires.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_PERIODIC
int stm32wb_rtc_setperiodic(const struct timespec *period,
                            wakeupcb_t callback)
{
  unsigned int wutr_val;
  int ret;
  int timeout;
  uint32_t regval;
  uint32_t secs;
  uint32_t millisecs;

#if defined(CONFIG_STM32WB_RTC_HSECLOCK)
#  error "Periodic wakeup not available for HSE"
#elif defined(CONFIG_STM32WB_RTC_LSICLOCK)
#  error "Periodic wakeup not available for LSI (and it is too inaccurate!)"
#elif defined(CONFIG_STM32WB_RTC_LSECLOCK)
  const uint32_t rtc_div16_max_msecs = 16 * 1000 * 0xffffu /
                                       STM32WB_LSE_FREQUENCY;
#else
#  error "No clock for RTC!"
#endif

  /* Lets use RTC wake-up with 0.001 sec to ~18 hour range.
   *
   * TODO: scale to higher periods, with necessary losing some precision.
   * We currently go for subseconds accuracy instead of maximum period.
   */

  if (period->tv_sec > 0xffffu ||
     (period->tv_sec == 0xffffu && period->tv_nsec > 0))
    {
      /* More than max. */

      secs = 0xffffu;
      millisecs = secs * 1000;
    }
  else
    {
      secs = period->tv_sec;
      millisecs = secs * 1000 + period->tv_nsec / NSEC_PER_MSEC;
    }

  if (millisecs == 0)
    {
      return -EINVAL;
    }

  /* Make sure the alarm interrupt is enabled at the NVIC */

  rtc_enable_wakeup();

  rtc_wprunlock();

  /* Clear WUTE in RTC_CR to disable the wakeup timer */

  regval = getreg32(STM32WB_RTC_CR);
  regval &= ~RTC_CR_WUTE;
  putreg32(regval, STM32WB_RTC_CR);

  /* Poll WUTWF until it is set in RTC_ISR (takes around 2 RTCCLK
   * clock cycles)
   */

  ret = -ETIMEDOUT;
  for (timeout = 0; timeout < SYNCHRO_TIMEOUT; timeout++)
    {
      regval = getreg32(STM32WB_RTC_ISR);
      if ((regval & RTC_ISR_WUTWF) != 0)
        {
          /* Synchronized */

          ret = OK;
          break;
        }
    }

  /* Set callback function pointer. */

  g_wakeupcb = callback;

  if (millisecs <= rtc_div16_max_msecs)
    {
      unsigned int ticks;

      /* Select wake-up with 32768/16 hz counter. */

      rtc_set_wcksel(RTC_CR_WUCKSEL_RTCDIV16);

      /* Get number of ticks. */

      ticks = millisecs * STM32WB_LSE_FREQUENCY / (16 * 1000);

      /* Wake-up is after WUT+1 ticks. */

      wutr_val = ticks - 1;
    }
  else
    {
      /* Select wake-up with 1hz counter. */

      rtc_set_wcksel(RTC_CR_WUCKSEL_CKSPRE);

      /* Wake-up is after WUT+1 ticks. */

      wutr_val = secs - 1;
    }

  /* Program the wakeup auto-reload value WUT[15:0], and the wakeup clock
   * selection.
   */

  putreg32(wutr_val, STM32WB_RTC_WUTR);

  regval = getreg32(STM32WB_RTC_CR);
  regval |= RTC_CR_WUTIE | RTC_CR_WUTE;
  putreg32(regval, STM32WB_RTC_CR);

  /* Just in case resets the WUTF flag in RTC_ISR */

  regval = getreg32(STM32WB_RTC_ISR);
  regval &= ~RTC_ISR_WUTF;
  putreg32(regval, STM32WB_RTC_ISR);

  rtc_wprlock();

  return ret;
}
#endif

/****************************************************************************
 * Name: stm32wb_rtc_cancelperiodic
 *
 * Description:
 *   Cancel a periodic wakeup
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_PERIODIC
int stm32wb_rtc_cancelperiodic(void)
{
  int ret = OK;
  int timeout = 0;
  uint32_t regval = 0;

  rtc_wprunlock();

  /* Clear WUTE and WUTIE in RTC_CR to disable the wakeup timer */

  regval = getreg32(STM32WB_RTC_CR);
  regval &= ~(RTC_CR_WUTE | RTC_CR_WUTIE);
  putreg32(regval, STM32WB_RTC_CR);

  /* Poll WUTWF until it is set in RTC_ISR (takes around 2 RTCCLK
   * clock cycles)
   */

  ret = -ETIMEDOUT;
  for (timeout = 0; timeout < SYNCHRO_TIMEOUT; timeout++)
    {
      regval = getreg32(STM32WB_RTC_ISR);
      if ((regval & RTC_ISR_WUTWF) != 0)
        {
          /* Synchronized */

          ret = OK;
          break;
        }
    }

  /* Clears RTC_WUTR register */

  regval = getreg32(STM32WB_RTC_WUTR);
  regval &= ~RTC_WUTR_MASK;
  putreg32(regval, STM32WB_RTC_WUTR);

  rtc_wprlock();

  return ret;
}
#endif

#endif /* CONFIG_STM32WB_RTC */
