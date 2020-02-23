/****************************************************************************
 * arch/arm/include/cxd56xx/pm.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_PM_H
#define __ARCH_ARM_INCLUDE_CXD56XX_PM_H

/*-----------------------------------------------------------------------------
 * include files
 *---------------------------------------------------------------------------*/

#include <queue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Boot Cause definitions */

#define PM_BOOT_POR_NORMAL      (0x00000000ul) /** Power On Reset like as battery attached */
#define PM_BOOT_POR_DEADBATT    (0x00000001ul) /** Battery charged from DeadBattery state */
#define PM_BOOT_WDT_REBOOT      (0x00000002ul) /** System WDT expired or Explicitly Self Reboot */
#define PM_BOOT_WDT_RESET       (0x00000004ul) /** Chip WDT expired (might be used in HV-only system) */
#define PM_BOOT_DEEP_WKUPL      (0x00000008ul) /** In DeepSleep state, Detected WKUPL signal */
#define PM_BOOT_DEEP_WKUPS      (0x00000010ul) /** In DeepSleep state, Detected WKUPS signal */
#define PM_BOOT_DEEP_RTC        (0x00000020ul) /** In DeepSleep state, RTC Alarm expired */
#define PM_BOOT_DEEP_USB_ATTACH (0x00000040ul) /** In DeepSleep state, USB Connected */
#define PM_BOOT_DEEP_OTHERS     (0x00000080ul) /** In DeepSleep state, Reserved others cause occurred */
#define PM_BOOT_COLD_SCU_INT    (0x00000100ul) /** In ColdSleep state, Detected SCU Interrupt */
#define PM_BOOT_COLD_RTC        (0x00001e00ul) /** In ColdSleep state, RTC Alarm Interrupt */
#define PM_BOOT_COLD_RTC_ALM0   (0x00000200ul) /** In ColdSleep state, RTC Alarm0 expired */
#define PM_BOOT_COLD_RTC_ALM1   (0x00000400ul) /** In ColdSleep state, RTC Alarm1 expired */
#define PM_BOOT_COLD_RTC_ALM2   (0x00000800ul) /** In ColdSleep state, RTC Alarm2 expired */
#define PM_BOOT_COLD_RTC_ALMERR (0x00001000ul) /** In ColdSleep state, RTC Alarm Error occurred */
#define PM_BOOT_COLD_GPIO       (0x0fff0000ul) /** In ColdSleep state, Detected GPIO interrupt */
#define PM_BOOT_COLD_SEN_INT    (0x10000000ul) /** In ColdSleep state, Detected SEN_INT Interrupt */
#define PM_BOOT_COLD_PMIC_INT   (0x20000000ul) /** In ColdSleep state, Detected PMIC Interrupt */
#define PM_BOOT_COLD_USB_DETACH (0x40000000ul) /** In ColdSleep state, USB Disconnected */
#define PM_BOOT_COLD_USB_ATTACH (0x80000000ul) /** In ColdSleep state, USB Connected */

/* SRAM power status definitions */

#define PMCMD_RAM_OFF 0  /* Power off */
#define PMCMD_RAM_RET 1  /* Retention */
#define PMCMD_RAM_ON  3  /* Power on */

/* FrequencyLock request flag definitions */

#define PM_CPUFREQLOCK_FLAG_HV (0x0001) /* request HV */
#define PM_CPUFREQLOCK_FLAG_LV (0x4000) /* request LV */

/* FrequencyLock identifier tag helper macro function */

#define PM_CPUFREQLOCK_TAG(prefix1, prefix2, num) \
          (((prefix1) << 24) + ((prefix2) << 16) + (num))

/* FrequencyLock initializer macro function */

#  define PM_CPUFREQLOCK_INIT(_tag, _flag) \
{ \
  .count = 0, \
  .info = _tag, \
  .flag = _flag, \
}

/* WakeLock identifier tag helper macro function */

#define PM_CPUWAKELOCK_TAG(prefix1, prefix2, num) \
          (((prefix1) << 24) + ((prefix2) << 16) + (num))

/* WakeLock initializer macro function */

#define PM_CPUWAKELOCK_INIT(_tag) \
{ \
  .count = 0, \
  .info = _tag, \
}

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* slee mode definitions */

enum pm_sleepmode_e
{
  PM_SLEEP_DEEP,
  PM_SLEEP_COLD,
};

/* FreqLock structure */

struct pm_cpu_freqlock_s
{
  struct sq_entry_s sq_entry;
  int      count;
  uint32_t info;
  int      flag;
};

/* WakeLock structure */

struct pm_cpu_wakelock_s
{
  struct sq_entry_s sq_entry;
  int      count;
  uint32_t info;
};

/* Definitions for pmic notify */

enum pmic_notify_e
{
  PMIC_NOTIFY_ALARM = 0,
  PMIC_NOTIFY_WKUPS,
  PMIC_NOTIFY_WKUPL,
  PMIC_NOTIFY_LOWBATT,
  PMIC_NOTIFY_MAX
};

/* callback function for pmic notify */

typedef void (*pmic_notify_t)(void *arg);

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
 * Name: up_pmstatdump
 *
 * Description:
 *   Print architecture specific power status
 *
 ****************************************************************************/

int up_pmramctrl(int cmd, uintptr_t addr, size_t size);

#ifdef CONFIG_DEBUG_PM
/****************************************************************************
 * Name: up_pmstatdump
 *
 * Description:
 *   Print architecture specific power status
 *
 ****************************************************************************/

void up_pmstatdump(void);
#else
#  define up_pmstatdump()
#endif

/****************************************************************************
 * Name: up_pm_acquire_freqlock
 *
 * Description:
 *   Acquire the specified freqlock. If the higher freqlock is acquired, the
 *   system can clockup until it is released.
 *
 * Parameter:
 *   lock - the pointer of a wakelock variable
 *
 ****************************************************************************/

void up_pm_acquire_freqlock(struct pm_cpu_freqlock_s *lock);

/****************************************************************************
 * Name: up_pm_release_freqlock
 *
 * Description:
 *   Release the specified freqlock. If the freqlock are released, the system
 *   can drop to the lower clock mode for power saving.
 *
 * Parameter:
 *   lock - the pointer of a freqlock variable
 *
 ****************************************************************************/

void up_pm_release_freqlock(struct pm_cpu_freqlock_s *lock);

/****************************************************************************
 * Name: up_pm_get_freqlock_count
 *
 * Description:
 *   Get the locked count of the specified freqlock
 *
 * Parameter:
 *   lock - the pointer of a freqlock variable
 *
 * Return:
 *   the locked count of the specified freqlock
 *
 ****************************************************************************/

int up_pm_get_freqlock_count(struct pm_cpu_freqlock_s *lock);

/****************************************************************************
 * Name: up_pm_acquire_wakelock
 *
 * Description:
 *   Acquire the specified wakelock. If any wakelock is acquired, CPU can't
 *   enter to the hot sleep state.
 *
 * Parameter:
 *   lock - the pointer of a wakelock variable
 *
 ****************************************************************************/

void up_pm_acquire_wakelock(struct pm_cpu_wakelock_s *lock);

/****************************************************************************
 * Name: up_pm_release_wakelock
 *
 * Description:
 *   Release the specified wakelock. If all of the wakelock are released,
 *   CPU can enter to the hot sleep state.
 *
 * Parameter:
 *   lock - the pointer of a wakelock variable
 *
 ****************************************************************************/

void up_pm_release_wakelock(struct pm_cpu_wakelock_s *lock);

/****************************************************************************
 * Name: up_pm_count_acquire_wakelock
 *
 * Description:
 *   Count the total number of wakelock
 *
 * Return:
 *   the total number of wakelock
 *
 ****************************************************************************/

int up_pm_count_acquire_wakelock(void);

/****************************************************************************
 * Name: up_pm_get_bootcause
 *
 * Description:
 *   Get the system boot cause. This boot cause indicates the cause why the
 *   system is launched from the state of power-off, deep sleep or cold sleep.
 *   Each boot cause is defined as PM_BOOT_XXX.
 *
 * Return:
 *   Boot cause
 *
 ****************************************************************************/

uint32_t up_pm_get_bootcause(void);

/****************************************************************************
 * Name: up_pm_get_bootmask
 *
 * Description:
 *   Get the system boot mask. This boot mask indicates whether the specified
 *   bit is enabled or not as the boot cause. If a bit of boot mask is set,
 *   the boot cause is enabled. Each boot mask is defined as PM_BOOT_XXX.
 *
 * Return:
 *   Boot mask
 *
 ****************************************************************************/

uint32_t up_pm_get_bootmask(void);

/****************************************************************************
 * Name: up_pm_set_bootmask
 *
 * Description:
 *   Enable the boot cause of the specified bit.
 *
 * Parameter:
 *   mask - OR of Boot mask defined as PM_BOOT_XXX
 *
 * Return:
 *   Updated boot mask
 *
 ****************************************************************************/

uint32_t up_pm_set_bootmask(uint32_t mask);

/****************************************************************************
 * Name: up_pm_clr_bootmask
 *
 * Description:
 *   Disable the boot cause of the specified bit.
 *
 * Parameter:
 *   mask - OR of Boot mask defined as PM_BOOT_XXX
 *
 * Return:
 *   Updated boot mask
 *
 ****************************************************************************/

uint32_t up_pm_clr_bootmask(uint32_t mask);

/****************************************************************************
 * Name: up_pm_sleep
 *
 * Description:
 *   Enter sleep mode. This function never returns.
 *
 * Parameter:
 *   mode - PM_SLEEP_DEEP or PM_SLEEP_COLD
 *
 ****************************************************************************/

int up_pm_sleep(enum pm_sleepmode_e mode);

/****************************************************************************
 * Name: up_pm_reboot
 *
 * Description:
 *   System reboot. This function never returns.
 *
 ****************************************************************************/

int up_pm_reboot(void);

/****************************************************************************
 * Name: up_pmic_set_notify
 *
 * Description:
 *   Register a callback for pmic interrupt
 *
 * Input Parameter:
 *   kind - A kind of pmic interrupt defined as pmic_notify_e
 *   cb - A callback function for a kind of pmic interrupt
 *
 * Returned Value:
 *   Return 0 on success. Otherwise, return a negated errno.
 *
 ****************************************************************************/

#ifdef CONFIG_CXD56_PMIC_INT
int up_pmic_set_notify(int kind, pmic_notify_t cb);
#else
#  define up_pmic_set_notify(kind, cb)
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_INCLUDE_CXD56XX_PM_H */
