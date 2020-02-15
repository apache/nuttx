/****************************************************************************
 * boards/arm/stm32/clicker2-stm32/src/clicker2-stm32.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#ifndef __BOARDS_ARM_STM32_CLICKER2_STM32_SRC_CLICKER2_H
#define __BOARDS_ARM_STM32_CLICKER2_STM32_SRC_CLICKER2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Assume that we support everything until convinced otherwise */

#define HAVE_USBDEV       1
#define HAVE_USBMONITOR   1
#define HAVE_MMCSD        1
#define HAVE_AUTOMOUNTER  1

/* MMCSD */

/* Only support uSD click board */

#if !defined(CONFIG_CLICKER2_STM32_MB1_MMCSD) && \
    !defined(CONFIG_CLICKER2_STM32_MB2_MMCSD)
#  undef HAVE_MMCSD
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(HAVE_MMCSD) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No MMC/SD support
#  undef HAVE_MMCSD
#endif

#ifdef HAVE_MMCSD

/* Default slot number */

#  ifdef CONFIG_CLICKER2_STM32_MB1_MMCSD
#    ifdef CONFIG_NSH_MMCSDSLOTNO
#      define MB1_MMCSD_SLOTNO CONFIG_NSH_MMCSDSLOTNO
#    else
#      define MB1_MMCSD_SLOTNO 0
#    endif
#    ifdef CONFIG_CLICKER2_STM32_MB2_MMCSD
#      define MB2_MMCSD_SLOTNO MB1_MMCSD_SLOTNO+1
#    endif
#  elif defined(CONFIG_CLICKER2_STM32_MB2_MMCSD)
#    ifdef CONFIG_NSH_MMCSDSLOTNO
#      define MB2_MMCSD_SLOTNO CONFIG_NSH_MMCSDSLOTNO
#    else
#      define MB2_MMCSD_SLOTNO 0
#    endif
#  endif

/* Default minor device number */

#  ifdef CONFIG_CLICKER2_STM32_MB1_MMCSD
#    ifdef CONFIG_NSH_MMCSDMINOR
#      define MB1_MMCSD_MINOR CONFIG_NSH_MMCSDMINOR
#    else
#      define MB1_MMCSD_MINOR 0
#    endif
#    ifdef CONFIG_CLICKER2_STM32_MB2_MMCSD
#      define MB2_MMCSD_MINOR MB1_MMCSD_MINOR+1
#    endif
#  elif defined(CONFIG_CLICKER2_STM32_MB2_MMCSD)
#    ifdef CONFIG_NSH_MMCSDMINOR
#      define MB2_MMCSD_MINOR CONFIG_NSH_MMCSDMINOR
#    else
#      define MB2_MMCSD_MINOR 0
#    endif
#  endif

#endif /* HAVE_MMCSD */

#ifndef CONFIG_FS_AUTOMOUNTER
#  undef HAVE_AUTOMOUNTER
#endif

/* Can't support USB device feature if USB OTG FS is not enabled */

#ifndef CONFIG_STM32_OTGFS
#  undef HAVE_USBDEV
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB device monitor if USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#  undef HAVE_USBMONITOR
#endif

/* Check if we should enable the USB monitor before starting NSH */

#if !defined(CONFIG_USBDEV_TRACE) || !defined(CONFIG_USBMONITOR)
#  undef HAVE_USBMONITOR
#endif

/* Mickroe Clicker2 STM32 GPIOs *********************************************/
/* LEDs
 *
 * The Mikroe Clicker2 STM32 has two user controllable LEDs:
 *
 *   LD1 - PE12, Active high output illuminates
 *   LD2 - PE15, Active high output illuminates
 */

#define GPIO_LED1         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN12)
#define GPIO_LED2         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN15)

/* BUTTONs
 *
 * The Mikroe Clicker2 STM32 has two buttons available to software:
 *
 *   T2 - PE0, Low sensed when pressed
 *   T3 - PA10, Low sensed when pressed
 *
 * NOTE that all have EXTI interrupts configured
 */

#define GPIO_BTN_T2       (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN0)
#define GPIO_BTN_T3       (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN10)

/* USB OTG FS
 *
 * USB device.  VBUS sensing is provided:
 *
 *   PA9  OTG_FS_VBUS VBUS sensing (USB-DET)
 *
 * USB host does not appear to be supported.  My interpretation is that power
 * is provided via LTC3586 which can be driven either from USB VBUS or from PWR-EN
 * (controlled by SW1).  But I don't see any capability to drive VBUS power.
 *
 * Overcurrent and battery status are provided by the LTC3586, but not USB power.
 *
 *   PC6  Overcurrent detection (PC6-FAULT)
 *   PD4  Battery status (PD4-BATSTAT)
 */

/* USB device */

/* USB device */

#define GPIO_OTGFS_VBUS   (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTA|GPIO_PIN9)

/* Power status */

#define GPIO_PWR_FAULT   (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTC|GPIO_PIN6)
#define GPIO_PWR_BATSTAT (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTD|GPIO_PIN4)

/* mikroBUS *************************************************************************/

/* U[S]ARTs
 *
 *   USART2 - mikroBUS1
 *   USART3 - mikroBUS2
 */

/* SPI Chip Selects
 *
 *   mikroBUS1 Chipselect: PE8-MB1_CS  (SPI3)
 *   mikroBUS2 Chipselect: PE11-MB2_CS (SPI2)
 */

#define GPIO_MB1_CS      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                          GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN8)
#define GPIO_MB2_CS      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                          GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN11)

/* I2C
 *
 *   mikroBUS1 I2C: PA8-I2C3_SCL, PC9-I2C3_SDA  (I2C3)
 *   mikroBUS2 I2C: PB10-I2C2_SCL, PB11-I2C2_SDA ()
 */

/* Analog
 *
 *   mikroBUS1 ADC: PA2-MB1_AN
 *   mikroBUS1 ADC: PA3-MB2_AN
 */

/* PWM
 *
 *   mikroBUS1 ADC: PE9-MB1-PWM  (TIM1, channel 1)
 *   mikroBUS1 ADC: PD12-MB2-PWM (TIM4, channel 1)
 */

/* Reset
 *
 *   mikroBUS1 Reset: PE7-MB1_RST
 *   mikroBUS2 Reset: PE13-MB2_RST
 */

#define GPIO_MB1_RST     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                          GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN7)
#define GPIO_MB2_RST     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                          GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN13)

/* Interrupts
 *
 *   mikroBUS1 Interrupt: PE10-MB1_INT
 *   mikroBUS2 Interrupt: PE14-MB2_INT
 *
 * I assume that the interrupt lines are active low.  No pull-ups are provided on
 * board so pull-ups are provided in the pin configurations.
 */

#define GPIO_MB1_INT     (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTE|GPIO_PIN10)
#define GPIO_MB2_INT     (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTE|GPIO_PIN14)

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Mikroe Clicker2 STM32
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_usb_configure
 *
 * Description:
 *   Called from stm32_boardinitialize very early in inialization to setup USB-related
 *   GPIO pins for the Mikroe Clicker2 STM32 board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_OTGFS
void stm32_usb_configure(void);
#endif

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int stm32_adc_setup(void);
#endif

/****************************************************************************
 * Name: stm32_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

#ifdef CONFIG_CAN
int stm32_can_setup(void);
#endif

/****************************************************************************
 * Name: stm32_mrf24j40_initialize
 *
 * Description:
 *   Initialize the MRF24J40 device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#if defined(CONFIG_CLICKER2_STM32_MB1_BEE) || defined(CONFIG_CLICKER2_STM32_MB2_BEE)
int stm32_mrf24j40_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_xbee_initialize
 *
 * Description:
 *   Initialize the XBee device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#if defined(CONFIG_CLICKER2_STM32_MB1_XBEE) || defined(CONFIG_CLICKER2_STM32_MB2_XBEE)
int stm32_xbee_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_mmcsd_initialize
 *
 * Description:
 *   Initialize the MMCSD device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef HAVE_MMCSD
int stm32_mmcsd_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_cardinserted
 *
 * Description:
 *   Check if SD card is inserted in slotno
 *
 * Returned Value:
 *   true if card is inserted, false if not
 *
 ****************************************************************************/

#ifdef HAVE_MMCSD
bool stm32_cardinserted(int slotno);
#endif

/****************************************************************************
 * Name:  stm32_automount_initialize
 *
 * Description:
 *   Configure auto-mounters for each enabled MikroBus MMCSD
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    None
 *
 ****************************************************************************/

#ifdef HAVE_AUTOMOUNTER
int stm32_automount_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_automount_event
 *
 * Description:
 *   The MMCSD card detection logic has detected an insertion or removal event.  It
 *   has already scheduled the MMC/SD block driver operations.  Now we need to
 *   schedule the auto-mount event which will occur with a substantial delay to make
 *   sure that everything has settle down.
 *
 * Input Parameters:
 *   slotno - Identifies the MB slot: MB1_MMCSD_SLOTNO or MB2_MMCSD_SLOTNO.
 *   inserted - True if the card is inserted in the slot.  False otherwise.
 *
 *  Returned Value:
 *    None
 *
 *  Assumptions:
 *    Interrupts are disabled.
 *
 ****************************************************************************/

#ifdef HAVE_AUTOMOUNTER
void stm32_automount_event(int slotno, bool inserted);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_CLICKER2_STM32_SRC_CLICKER2_H */
