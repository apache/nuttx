/****************************************************************************
 * arch/arm/src/n32h7/n32_usbhs.h
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

/* N32H7 MOD:
 * This file is the N32H76x counterpart of the STM32H7 stm32_otg.h driver
 * common header.  It provides the shared definitions used by the N32H7
 * USBHS device (n32_usbhsdev.c) and host (n32_usbhshost.c) drivers.
 *
 * Register offsets and bitfields are NOT duplicated here; they live in
 * hardware/n32h7_usbhs.h.  This header only composes the instance base
 * address with the hardware offsets and supplies the small set of
 * driver-local constants that the ported Synopsys OTG core logic needs.
 */

#ifndef __ARCH_ARM_SRC_N32H7_N32_USBHS_H
#define __ARCH_ARM_SRC_N32H7_N32_USBHS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "chip.h"
#include "hardware/n32h7_usbhs.h"
#include "hardware/n32h76x_rcc.h"
#include "hardware/n32h76x_pwr.h"

#if defined(CONFIG_N32H7_USBHS1_DEV)  || defined(CONFIG_N32H7_USBHS2_DEV) || \
    defined(CONFIG_N32H7_USBHS1_HOST) || defined(CONFIG_N32H7_USBHS2_HOST)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Addresses *******************************************************/

/* Core global registers */

#define N32_USBHS_GCTRLSTS        (N32_USBHS_BASE + N32_USBHS_GCTRLSTS_OFF)
#define N32_USBHS_GAHBCFG         (N32_USBHS_BASE + N32_USBHS_GAHBCFG_OFF)
#define N32_USBHS_GCFG            (N32_USBHS_BASE + N32_USBHS_GCFG_OFF)
#define N32_USBHS_GRSTCTRL        (N32_USBHS_BASE + N32_USBHS_GRSTCTRL_OFF)
#define N32_USBHS_GINTSTS         (N32_USBHS_BASE + N32_USBHS_GINTSTS_OFF)
#define N32_USBHS_GINTEN          (N32_USBHS_BASE + N32_USBHS_GINTEN_OFF)
#define N32_USBHS_GRXSTS          (N32_USBHS_BASE + N32_USBHS_GRXSTS_OFF)
#define N32_USBHS_GRXSTSP         (N32_USBHS_BASE + N32_USBHS_GRXSTSP_OFF)
#define N32_USBHS_GRXFSIZ         (N32_USBHS_BASE + N32_USBHS_GRXFSIZ_OFF)
#define N32_USBHS_GNPTXFSIZ       (N32_USBHS_BASE + N32_USBHS_GNPTXFSIZ_OFF)
#define N32_USBHS_GNPTXFSTS       (N32_USBHS_BASE + N32_USBHS_GNPTXFSTS_OFF)
#define N32_USBHS_CID             (N32_USBHS_BASE + N32_USBHS_CID_OFF)
#define N32_USBHS_GPD             (N32_USBHS_BASE + N32_USBHS_GPD_OFF)
#define N32_USBHS_HPTXFSIZ        (N32_USBHS_BASE + N32_USBHS_HPTXFSIZ_OFF)

#define N32_USBHS_DINEPPTXFSIZ(n) (N32_USBHS_BASE + \
                                   N32_USBHS_DINEPPTXFSIZ_OFF + (((n)-1) << 2))

/* N32H7 MOD:
 * In device mode the IN EP0 TxFIFO size register shares the 0x028 offset
 * with GNPTXFSIZ (same physical register, like DIEPTXF0/HNPTXFSIZ on the
 * STM32H7 OTG core).
 */

#define N32_USBHS_DINEPTXF0       (N32_USBHS_BASE + N32_USBHS_GNPTXFSIZ_OFF)

/* Host mode registers */

#define N32_USBHS_HCFG            (N32_USBHS_BASE + N32_USBHS_HCFG_OFF)
#define N32_USBHS_HFRI            (N32_USBHS_BASE + N32_USBHS_HFRI_OFF)
#define N32_USBHS_HFNUM           (N32_USBHS_BASE + N32_USBHS_HFNUM_OFF)
#define N32_USBHS_HPTXFQSTS       (N32_USBHS_BASE + N32_USBHS_HPTXFQSTS_OFF)
#define N32_USBHS_HACHINT         (N32_USBHS_BASE + N32_USBHS_HACHINT_OFF)
#define N32_USBHS_HACHINTEN       (N32_USBHS_BASE + N32_USBHS_HACHINTEN_OFF)
#define N32_USBHS_HPCS            (N32_USBHS_BASE + N32_USBHS_HPCS_OFF)

/* Host channel registers */

#define N32_USBHS_HCHCTRL(n)      (N32_USBHS_BASE + N32_USBHS_HCHCTRL_OFF(n))
#define N32_USBHS_HCSCTRL(n)      (N32_USBHS_BASE + N32_USBHS_HCSCTRL_OFF(n))
#define N32_USBHS_HCHINTSTS(n)    (N32_USBHS_BASE + N32_USBHS_HCHINTSTS_OFF(n))
#define N32_USBHS_HCHINTEN(n)     (N32_USBHS_BASE + N32_USBHS_HCHINTEN_OFF(n))
#define N32_USBHS_HCHTXSIZ(n)     (N32_USBHS_BASE + N32_USBHS_HCHTXSIZ_OFF(n))
#define N32_USBHS_HCHDMADD(n)     (N32_USBHS_BASE + N32_USBHS_HCHDMADD_OFF(n))

/* Device mode registers */

#define N32_USBHS_DCFG            (N32_USBHS_BASE + N32_USBHS_DCFG_OFF)
#define N32_USBHS_DCTRL           (N32_USBHS_BASE + N32_USBHS_DCTRL_OFF)
#define N32_USBHS_DSTS            (N32_USBHS_BASE + N32_USBHS_DSTS_OFF)
#define N32_USBHS_DINEPINTEN      (N32_USBHS_BASE + N32_USBHS_DINEPINTEN_OFF)
#define N32_USBHS_DOUTEPINTEN     (N32_USBHS_BASE + N32_USBHS_DOUTEPINTEN_OFF)
#define N32_USBHS_DAEPINTSTS      (N32_USBHS_BASE + N32_USBHS_DAEPINTSTS_OFF)
#define N32_USBHS_DAEPINTEN       (N32_USBHS_BASE + N32_USBHS_DAEPINTEN_OFF)
#define N32_USBHS_DTHRCTRL        (N32_USBHS_BASE + N32_USBHS_DTHRCTRL_OFF)
#define N32_USBHS_DINEPFEINTEN    (N32_USBHS_BASE + N32_USBHS_DINEPFEINTEN_OFF)

/* Device IN/OUT endpoint registers */

#define N32_USBHS_DINEPCTRL(n)    (N32_USBHS_BASE + N32_USBHS_DINEPCTRL_OFF(n))
#define N32_USBHS_DINEPINTSTS(n)  (N32_USBHS_BASE + N32_USBHS_DINEPINTSTS_OFF(n))
#define N32_USBHS_DINEPTXSIZ(n)   (N32_USBHS_BASE + N32_USBHS_DINEPTXSIZ_OFF(n))
#define N32_USBHS_DINEPTXFSTS(n)  (N32_USBHS_BASE + N32_USBHS_DINEPTXFSTS_OFF(n))
#define N32_USBHS_DOUTEPCTRL(n)   (N32_USBHS_BASE + N32_USBHS_DOUTEPCTRL_OFF(n))
#define N32_USBHS_DOUTEPINTSTS(n) (N32_USBHS_BASE + N32_USBHS_DOUTEPINTSTS_OFF(n))
#define N32_USBHS_DOUTEPTXSIZ(n)  (N32_USBHS_BASE + N32_USBHS_DOUTEPTXSIZ_OFF(n))

/* Power and clock gating control register */

#define N32_USBHS_PWRCTRL         (N32_USBHS_BASE + N32_USBHS_PWRCTRL_OFF)
#define N32_USBHS_PWRCTRL1        (N32_USBHS_BASE + N32_USBHS_PWRCTRL1_OFF)

/* Wrapper Registers */
#define N32_USBHS_WRPCTRL         (N32_USBHS_WRAPPER_BASE + N32_USBHS_WRPCTRL_OFF)
#define N32_USBHS_WRPCFG          (N32_USBHS_WRAPPER_BASE + N32_USBHS_WRPCFG_OFF)

/* N32H7 MOD:
 * Data FIFO access (push/pop) addresses.  The Synopsys-compatible core
 * maps one 4Kb FIFO debug/push-pop region per endpoint (device mode) or
 * per host channel starting at offset 0x1000.  hardware/n32h7_usbhs.h
 * does not carry FIFO window definitions, so they are provided here.
 */

#define N32_USBHS_DFIFO_DEP(n)    (N32_USBHS_BASE + 0x1000UL + \
                                   ((uint32_t)(n) << 12))
#define N32_USBHS_DFIFO_HCH(n)    (N32_USBHS_BASE + 0x1000UL + \
                                   ((uint32_t)(n) << 12))

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: n32_usbhs_enableclock
 *
 * Description:
 *   Enable the AHB clock of the USBHS instance selected by the including
 *   driver file (via N32_USBHS_BASE) and un-gate the USBHS core memory
 *   power.
 *
 * N32H7 MOD:
 *   Replaces the STM32 RCC OTG clock enable.  Per n32h76x_rcc.h the
 *   USBHS1 core clock is RCC_AHB2EN1_M7USB1EN and the USBHS2 core clock
 *   is RCC_AHB1EN1_M7USB2EN.  The USBHS core memory power gate is
 *   controlled through N32_PWR_IP_MEMPWR_CR (PWR_IPMEMCTRL_USBx_PGEN);
 *   there is no USB voltage regulator / VDD33 detector as on STM32H7
 *   (PWR_CR3_USBREGEN / PWR_CR3_USB33DEN do not exist on N32H76x).
 *
 *   Defined static inline in this header so that both n32_usbhsdev.c and
 *   n32_usbhshost.c can use it without duplicate symbols when device and
 *   host roles are enabled on different instances in the same build.
 *
 ****************************************************************************/

static inline void n32_usbhs_enableclock(void)
{
  uint32_t regval;

  /* Enable the USBHS core memory power gate and wait until ready */

  if (N32_USBHS_BASE == N32_USBCTRL1_BASE)
    {
      /* Enable the HSC1 Power Gate */

      regval = getreg32(N32_PWR_SYS_PWR_CR3);
      regval |= PWR_SYSCTRL3_HSC1_PGEN;
      putreg32(regval, N32_PWR_SYS_PWR_CR3);

      while ((getreg32(N32_PWR_SYS_PWR_CR3) &
             PWR_SYSCTRL3_HSC1_PWRRDY) == 0);

      regval = getreg32(N32_PWR_SYS_PWR_CR3);
      regval |= (PWR_SYSCTRL3_HSC1_FUCEN | PWR_SYSCTRL3_HSC1_ISNEN);
      putreg32(regval, N32_PWR_SYS_PWR_CR3);

      /* Enable the USBHS1 peripheral clock */

      regval  = getreg32(N32_RCC_AHB2EN1);
      regval |= RCC_AHB2EN1_M7USB1EN;
      putreg32(regval, N32_RCC_AHB2EN1);

      regval  = getreg32(N32_PWR_IP_MEMPWR_CR);
      regval &= ~PWR_IPMEMCTRL_USB1_PGEN;
      putreg32(regval, N32_PWR_IP_MEMPWR_CR);

      while ((getreg32(N32_PWR_IP_MEMPWR_CSR) &
             PWR_IPMEMSTS_USB1_PRDY) == 0);

      /* Reset the USBHS1 core */

      regval = getreg32(N32_RCC_AHB2RST1);
      regval |= RCC_AHB2RST1_USB1RST;
      putreg32(regval, N32_RCC_AHB2RST1);
      regval &= ~RCC_AHB2RST1_USB1RST;
      putreg32(regval, N32_RCC_AHB2RST1);
    }
  else
    {
      /* Enable the HSC2 Power Gate */

      regval = getreg32(N32_PWR_SYS_PWR_CR3);
      regval |= PWR_SYSCTRL3_HSC2_PGEN;
      putreg32(regval, N32_PWR_SYS_PWR_CR3);

      while ((getreg32(N32_PWR_SYS_PWR_CR3) &
             PWR_SYSCTRL3_HSC2_PWRRDY) == 0);

      regval = getreg32(N32_PWR_SYS_PWR_CR3);
      regval |= (PWR_SYSCTRL3_HSC2_FUCEN | PWR_SYSCTRL3_HSC2_ISNEN);
      putreg32(regval, N32_PWR_SYS_PWR_CR3);

      /* Enable the USBHS2 peripheral clock */

      regval  = getreg32(N32_RCC_AHB1EN1);
      regval |= RCC_AHB1EN1_M7USB2EN;
      putreg32(regval, N32_RCC_AHB1EN1);

      regval  = getreg32(N32_PWR_IP_MEMPWR_CR);
      regval &= ~PWR_IPMEMCTRL_USB2_PGEN;
      putreg32(regval, N32_PWR_IP_MEMPWR_CR);

      while ((getreg32(N32_PWR_IP_MEMPWR_CSR) &
             PWR_IPMEMSTS_USB2_PRDY) == 0);

      /* Reset the USBHS2 core */

      regval = getreg32(N32_RCC_AHB1RST1);
      regval |= RCC_AHB1RST1_USB2RST;
      putreg32(regval, N32_RCC_AHB1RST1);
      regval &= ~RCC_AHB1RST1_USB2RST;
      putreg32(regval, N32_RCC_AHB1RST1);
    }
}

/****************************************************************************
 * Name: n32_usbhs_disableclock
 *
 * Description:
 *   Disable the AHB clock of the USBHS instance selected by the including
 *   driver file and gate the USBHS core memory power.
 ****************************************************************************/

static inline void n32_usbhs_disableclock(void)
{
  uint32_t regval;

  if (N32_USBHS_BASE == N32_USBCTRL1_BASE)
    {
      regval  = getreg32(N32_RCC_AHB2EN1);
      regval &= ~RCC_AHB2EN1_M7USB1EN;
      putreg32(regval, N32_RCC_AHB2EN1);

      regval  = getreg32(N32_PWR_IP_MEMPWR_CR);
      regval |= PWR_IPMEMCTRL_USB1_PGEN;
      putreg32(regval, N32_PWR_IP_MEMPWR_CR);
    }
  else
    {
      regval  = getreg32(N32_RCC_AHB1EN1);
      regval &= ~RCC_AHB1EN1_M7USB2EN;
      putreg32(regval, N32_RCC_AHB1EN1);

      regval  = getreg32(N32_PWR_IP_MEMPWR_CR);
      regval |= PWR_IPMEMCTRL_USB2_PGEN;
      putreg32(regval, N32_PWR_IP_MEMPWR_CR);
    }
}

/****************************************************************************
 * Name: n32_usbhs_phy_initialize
 *
 * Description:
 *   Initialize the USBHS PHY interface.
 *
 * N32H7 MOD:
 *   The N32H76x USBHS core has no on-chip HS PHY; an external ULPI PHY is
 *   used instead.  The STM32H7 wrapper (WRPCTRL/WRPCFG) internal PHY PLL
 *   configuration is therefore not required and is intentionally skipped.
 *   The FS serial transceiver select (GCFG.PHYSEL) is only programmed by
 *   the drivers when the internal FS PHY configuration is selected
 *   (CONFIG_N32H7_USBHS_FS); for the default external ULPI configuration
 *   PHYSEL remains cleared.
 ****************************************************************************/

static inline void n32_usbhs_phy_initialize(void)
{
  /* N32H7 MOD:
   * External ULPI PHY is used.
   * Skip internal PHY wrapper (WRPCTRL/WRPCFG) PLL configuration.
   * Any board-level ULPI PHY reset is performed via n32_usbulpireset().
   */
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: n32_usbhshost_initialize
 *
 * Description:
 *   Initialize USB host device controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than USB host controller, then
 *     this identifies which controller is being initialized.  Normally,
 *     this is just zero.
 *
 * Returned Value:
 *   And instance of the USB host interface.  The controlling task should
 *   use this interface to (1) call the wait() method to wait for a device
 *   to be connected, and (2) call the enumerate() method to bind the device
 *   to a class driver.
 *
 * Assumptions:
 * - This function should called in the initialization sequence in order
 *   to initialize the USB device functionality.
 * - Class drivers should be initialized prior to calling this function.
 *   Otherwise, there is a race condition if the device is already connected.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST
struct usbhost_connection_s;
struct usbhost_connection_s *n32_usbhshost_initialize(int controller);
#endif

/****************************************************************************
 * Name:  n32_usbsuspend
 *
 * Description:
 *   Board logic must provide the n32_usbsuspend logic if the USBHS
 *   device driver is used.  This function is called whenever the USB enters
 *   or leaves suspend mode. This is an opportunity for the board logic to
 *   shutdown clocks, power, etc. while the USB is suspended.
 *
 ****************************************************************************/

struct usbdev_s;
extern void n32_usbsuspend(struct usbdev_s *dev, bool resume);

/****************************************************************************
 * Name:  n32_usbulpireset
 *
 * Description:
 *   Reset external ULPI PHY.
 *
 ****************************************************************************/

#ifndef CONFIG_N32H7_USBHS_FS
struct usbdev_s;
void n32_usbulpireset(struct usbdev_s *dev);
#endif

/****************************************************************************
 * Name: n32_usbhs_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.  This function must be
 *   provided by each board that implements the N32H7 USBHS host interface.
 *
 *   On-chip 5V VBUS generation is not supported.  A charge pump or a basic
 *   power switch must be added externally to drive the 5V VBUS line; it
 *   can be driven by any GPIO output.  When the application powers on
 *   VBUS using the chosen GPIO, it must also set the port power bit
 *   (HPCS.PPWR).
 *
 * Input Parameters:
 *   iface  - For future growth to handle multiple USB host interfaces.
 *            Should be zero.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST
extern void n32_usbhs_vbusdrive(int iface, bool enable);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_N32H7_USBHS1_DEV || CONFIG_N32H7_USBHS2_DEV || ... */
#endif /* __ARCH_ARM_SRC_N32H7_N32_USBHS_H */
