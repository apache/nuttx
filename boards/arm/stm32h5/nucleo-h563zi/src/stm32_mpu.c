/****************************************************************************
 * boards/arm/stm32h5/nucleo-h563zi/src/stm32_mpu.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>

#include "hardware/stm32_memorymap.h"
#include "mpu.h"
#include "stm32_mpuinit.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mpu_configure_otp
 *
 * Description:
 *   Configure the OTP flash region as non-cacheable, non-executable, non-
 *   shareable, and read-only.
 *
 ****************************************************************************/

void stm32_mpu_configure_otp(void)
{
  mpu_configure_region(STM32_OTP_BASE, 4096,
      MPU_RBAR_XN | MPU_RBAR_SH_NO | MPU_RBAR_AP_RORO,
      MPU_RLAR_NONCACHEABLE);
}
