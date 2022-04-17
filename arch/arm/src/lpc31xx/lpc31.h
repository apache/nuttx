/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31.h
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

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "arm_internal.h"
#include "chip.h"
#include "lpc31_ioconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* NVIC priority levels *****************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
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
 * Inline Functions
 ****************************************************************************/

/* Configure a pin as an input */

static inline void gpio_configinput(uint32_t ioconfig, uint32_t bit)
{
  uint32_t regaddr;

  regaddr = ioconfig + LPC31_IOCONFIG_MODE0RESET_OFFSET;
  putreg32(bit, regaddr);

  regaddr = ioconfig + LPC31_IOCONFIG_MODE1RESET_OFFSET;
  putreg32(bit, regaddr);
}

/* Return the current state of an input GPIO pin */

static inline bool lpc31_gpioread(uint32_t ioconfig, uint32_t bit)
{
  uint32_t regaddr = ioconfig + LPC31_IOCONFIG_PINS_OFFSET;
  return (getreg32(regaddr) & bit) != 0;
}

/* Configure the pin so that it is driven by the device */

static inline void gpio_configdev(uint32_t ioconfig, uint32_t bit)
{
  uint32_t regaddr;

  regaddr = ioconfig + LPC31_IOCONFIG_MODE1RESET_OFFSET;
  putreg32(bit, regaddr);

  regaddr = ioconfig + LPC31_IOCONFIG_MODE0SET_OFFSET;
  putreg32(bit, regaddr);
}

/* Configure a pin as a low output */

static inline void gpio_outputlow(uint32_t ioconfig, uint32_t bit)
{
  uint32_t regaddr;

  regaddr = ioconfig + LPC31_IOCONFIG_MODE1SET_OFFSET;
  putreg32(bit, regaddr);

  regaddr = ioconfig + LPC31_IOCONFIG_MODE0RESET_OFFSET;
  putreg32(bit, regaddr);
}

/* Configure a pin as a high output */

static inline void gpio_outputhigh(uint32_t ioconfig, uint32_t bit)
{
  uint32_t regaddr;

  regaddr = ioconfig + LPC31_IOCONFIG_MODE1SET_OFFSET;
  putreg32(bit, regaddr);

  regaddr = ioconfig + LPC31_IOCONFIG_MODE0SET_OFFSET;
  putreg32(bit, regaddr);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lpc31_lowsetup
 *
 * Description:
 *   Called early in arm_boot.
 *   Performs chip-common low level initialization.
 *
 ****************************************************************************/

void lpc31_lowsetup(void);

/****************************************************************************
 * Name: lpc31_clockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h
 *
 ****************************************************************************/

void lpc31_clockconfig(void);

/****************************************************************************
 * Name: lpc31_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s; /* Forward reference */
struct spi_dev_s *lpc31_spibus_initialize(int port);

/****************************************************************************
 * Name:  lpc31_spiselect and lpc31_spistatus
 *
 * Description:
 *   The external functions, lpc31_spiselect, lpc31_spistatus, and
 *   lpc31_spicmddata must be provided by board-specific logic.  These are
 *   implementations of the select, status, and cmddata methods of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h). All
 *   other methods (including lpc31_spibus_initialize()) are provided by
 *   common LPC31XX logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in lpc31_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide lpc31_spiselect() and lpc31_spistatus() functions in your
 *      board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board
 *      is configured.
 *   3. If CONFIG_SPI_CMDDATA is selected in your NuttX configuration,
 *      provide the lpc31_spicmddata() function in your board-specific logic.
 *      This function will perform cmd/data selection operations using GPIOs
 *      in the way your board is configured.
 *   4. Add a calls to lpc31_spibus_initialize() in your low level
 *      application initialization logic
 *   5. The handle returned by lpc31_spibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver
 *      to the SPI MMC/SD driver).
 *
 ****************************************************************************/

void  lpc31_spiselect(struct spi_dev_s *dev,
                      uint32_t devid, bool selected);
uint8_t lpc31_spistatus(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int lpc31_spicmddata(struct spi_dev_s *dev,
                     uint32_t devid, bool cmd);
#endif

/****************************************************************************
 * Name:  lpc31_usbpullup
 *
 * Description:
 *   If USB is supported and the board supports a pullup via GPIO (for USB
 *   software connect and disconnect), then the board software must provide
 *   lpc31_pullup. See include/nuttx/usb/usbdev.h for additional description
 *   of this method. Alternatively, if no pull-up GPIO the following EXTERN
 *   can be redefined to be NULL.
 *
 ****************************************************************************/

#if defined(CONFIG_LPC31_USBOTG) && defined(CONFIG_USBDEV)
struct usbdev_s;
int lpc31_usbpullup(struct usbdev_s *dev,  bool enable);
#endif

/****************************************************************************
 * Name:  lpc31_usbsuspend
 *
 * Description:
 *   Board logic must provide the lpc31_usbsuspend logic if the USBDEV driver
 *   is used.  This function is called whenever the USB enters or leaves
 *   suspend mode. This is an opportunity for the board logic to shutdown
 *   clocks, power, etc. while the USB is suspended.
 *
 ****************************************************************************/

#if defined(CONFIG_LPC31_USBOTG) && defined(CONFIG_USBDEV)
struct usbdev_s;
void lpc31_usbsuspend(struct usbdev_s *dev, bool resume);
#endif

/****************************************************************************
 * Name: lpc31_ehci_initialize
 *
 * Description:
 *   Initialize USB EHCI host controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than one EHCI interface, then
 *     this identifies which controller is being initializeed.  Normally,
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

#if defined(CONFIG_LPC31_USBOTG) && defined(CONFIG_USBHOST)
struct usbhost_connection_s;
struct usbhost_connection_s *lpc31_ehci_initialize(int controller);
#endif

/****************************************************************************
 * Name: lpc31_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.  This function must be
 *   provided by each platform that implements the EHCI host interface
 *
 * Input Parameters:
 *   rhport - Selects root hub port to be powered host interface.  This is
 *            not used with the LPC31 since it supports only a single root
 *            hub port.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_LPC31_USBOTG) && defined(CONFIG_USBHOST)
void lpc31_usbhost_vbusdrive(int rhport, bool enable);
#endif

/****************************************************************************
 * Name: sdio_initialize
 *
 * Description:
 *   Initialize SDIO for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Value:
 *   A reference to an SDIO interface structure.
 *   NULL is returned on failures.
 *
 ****************************************************************************/

struct sdio_dev_s; /* See include/nuttx/sdio.h */
struct sdio_dev_s *sdio_initialize(int slotno);

/****************************************************************************
 * Name: sdio_mediachange
 *
 * Description:
 *   Called by board-specific logic -- possibly from an interrupt handler --
 *   in order to signal to the driver that a card has been inserted or
 *   removed from the slot
 *
 * Input Parameters:
 *   dev        - An instance of the SDIO driver device state structure.
 *   cardinslot - true is a card has been detected in the slot; false if a
 *                card has been removed from the slot.  Only transitions
 *                (inserted->removed or removed->inserted should be reported)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sdio_mediachange(struct sdio_dev_s *dev, bool cardinslot);

/****************************************************************************
 * Name: sdio_wrprotect
 *
 * Description:
 *   Called by board-specific logic to report if the card in the slot is
 *   mechanically write protected.
 *
 * Input Parameters:
 *   dev       - An instance of the SDIO driver device state structure.
 *   wrprotect - true is a card is writeprotected.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sdio_wrprotect(struct sdio_dev_s *dev, bool wrprotect);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_H */
