/************************************************************************************
 * configs/stm32f429i-disco/src/stm32_boot.c
 *
 *   Copyright (C) 2011-2012, 2015-2016 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "stm32f429i-disco.h"
#include "stm32_ccm.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/
/* Should we initialize the NX server using nx_start?  This is done for NxWidgets
 * (CONFIG_NXWIDGETS=y) and if the NxWidget::CNxServer class expects the RTOS do the
 * the NX initialization (CONFIG_NXWIDGET_SERVERINIT=n).  This combination of
 * settings is normally only used in the kernel build mode* (CONFIG_BUILD_PROTECTED)
 * when NxWidgets is unable to initialize NX from user-space.
 */

#undef HAVE_NXSTART

#if !defined(CONFIG_NX_MULTIUSER)
#  undef CONFIG_NX_START
#endif

#if defined(CONFIG_NXWIDGETS) && !defined(CONFIG_NXWIDGET_SERVERINIT)
#   define HAVE_NXSTART
#   include <nuttx/nx/nx.h>
#endif

/* Should we initialize the touchscreen for the NxWM (CONFIG_NXWM=y)?  This
 * is done if we have a touchscreen (CONFIG_INPUT_STMPE811=y), NxWM uses the
 * touchscreen (CONFIG_NXWM_TOUCHSCREEN=y), and if we were asked to
 * initialize the touchscreen for NxWM (NXWM_TOUCHSCREEN_DEVINIT=n). This
 * combination of settings is normally only used in the kernel build mode
 * (CONFIG_BUILD_PROTECTED) when NxWidgets is unable to initialize NX from
 * user-space.
 */

#undef HAVE_TCINIT

#if defined(CONFIG_NXWM_TOUCHSCREEN)
#  if !defined(CONFIG_NXWM_TOUCHSCREEN_DEVNO)
#    error CONFIG_NXWM_TOUCHSCREEN_DEVNO is not defined
#  elif defined(CONFIG_INPUT_STMPE811)
#    if !defined(CONFIG_NXWM_TOUCHSCREEN_DEVINIT)
#      define HAVE_TCINIT
#      include <nuttx/input/touchscreen.h>
#    endif
#  else
#    if !defined(CONFIG_NXWM_TOUCHSCREEN_DEVINIT) && defined(CONFIG_BUILD_PROTECTED)
#      error CONFIG_INPUT_STMPE811=y is needed
#    endif
#  endif
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void stm32_boardinitialize(void)
{
#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || \
    defined(CONFIG_STM32_SPI3) || defined(CONFIG_STM32_SPI4) || \
    defined(CONFIG_STM32_SPI5)
  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak function
   * stm32_spidev_initialize() has been brought into the link.
   */

  if (stm32_spidev_initialize)
    {
      stm32_spidev_initialize();
    }
#endif

#ifdef CONFIG_STM32_OTGHS
  /* Initialize USB if the 1) OTG HS controller is in the configuration and 2)
   * disabled, and 3) the weak function stm32_usbinitialize() has been brought
   * into the build. Presumably either CONFIG_USBDEV or CONFIG_USBHOST is also
   * selected.
   */

  if (stm32_usbinitialize)
    {
      stm32_usbinitialize();
    }
#endif

#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  board_autoled_initialize();
#endif

#ifdef CONFIG_STM32_FSMC
  stm32_enablefsmc();
#endif

#ifdef HAVE_CCM_HEAP
  /* Initialize CCM allocator */

  ccm_initialize();
#endif
}

/****************************************************************************
 * Name: board_initialize
 *
 * Description:
 *   If CONFIG_BOARD_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_initialize().  board_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_INITIALIZE
void board_initialize(void)
{
#ifdef CONFIG_STM32F429I_DISCO_ILI9341_FBIFACE
  /* Initialize the framebuffer driver */

  up_fbinitialize(0);
#endif

#ifdef CONFIG_STM32F429I_DISCO_ILI9341_LCDIFACE
  /* Initialize the SPI-based LCD early */

  board_lcd_initialize();
#endif

#if defined(CONFIG_NSH_LIBRARY) && !defined(CONFIG_LIB_BOARDCTL)
  /* Perform NSH initialization here instead of from the NSH.  This
   * alternative NSH initialization is necessary when NSH is ran in user-space
   * but the initialization function must run in kernel space.
   */

  (void)board_app_initialize(0);
#endif
}
#endif
