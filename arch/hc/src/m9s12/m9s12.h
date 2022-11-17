/****************************************************************************
 * arch/hc/src/m9s12/m9s12.h
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

#ifndef __ARCH_HC_SRC_M9S12_M9S12_H
#define __ARCH_HC_SRC_M9S12_M9S12_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#  include <nuttx/spi/spi.h>
#endif

#include "hc_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO management macros:
 *
 * The GPIO configuration is represented by a 16-bit value encoded as
 * follows:
 *
 *   xIIO UURV DMGG GPPP
 *    ||| |||| |||    `-Pin number
 *    ||| |||| || `- Port number
 *    ||| |||| | `- PIM Ports
 *    ||| |||| `- Direction
 *    ||| |||`- Initial value of output
 *    ||| ||`- Reduced drive
 *    ||| |`- Polarity
 *    ||| `- Pull up (or down)
 *    ||`- Wired OR open drain
 *    |`- Interrupt or rising/falling (polarity)
 *    `- Interrupt
 *
 * NOTE: MEBI ports E and K can have special configurations as controlled by
 * the PEAR and MODE registers.  Those special configurations are not managed
 * by the logic below; that logic is only intended to support general GPIO
 * pin usage.
 */

/* Interrupts:
 *
 *   xIIx xxxx xxxx xxxx
 *
 * For PIM ports G, H, and J.  NOTE:  If pull up/down is also selected, then
 * it must be consistent with the selected interrupt edge (because both are
 * controlled by the same polarity register):
 *
 *   Rising edge <-> Pull down
 *   Falling Edge <-> Pull up
 *
 * Selecting input also selects direction == input (it is unless to specify
 * GPIO_INPUT and an error if GPIO_OUTPUT is also specified)
 */

#define GPIO_INT_SHIFT      (13)
#define GPIO_INT_MASK       (3 << GPIO_PULLUP_SHIFT)
#  define GPIO_INT_POLARITY (1 << GPIO_PULLUP_SHIFT)
#  define GPIO_INT_ENABLE   (2 << GPIO_PULLUP_SHIFT)
#  define GPIO_INT_FALLING  (2 << GPIO_PULLUP_SHIFT)
#  define GPIO_INT_RISING   (3 << GPIO_PULLUP_SHIFT)

/* Wired OR open-drain:
 *
 *   xxxO xxxx xxxx xxxx
 *
 * Only PIM ports S and L
 */

#define GPIO_OPENDRAIN (1 << 12)

/* Pull up (or down):
 *
 *   xxxx UUxx xxxx xxxx
 *
 * For PIM ports (T,S,G,H,J,L), selection is per-pin
 * For MEBI ports (A,B,E,K), selection is per-port, polarity is ignored
 */

#define GPIO_PULLUP_SHIFT    (10)
#define GPIO_PULLUP_MASK     (3 << GPIO_PULLUP_SHIFT)
#  define GPIO_PULL_POLARITY (1 << GPIO_PULLUP_SHIFT)
#  define GPIO_PULL_ENABLE   (2 << GPIO_PULLUP_SHIFT)
#  define GPIO_PULLUP        (2 << GPIO_PULLUP_SHIFT)
#  define GPIO_PULLDOWN      (3 << GPIO_PULLUP_SHIFT)

/* Reduced drive:
 *
 *   xxxx xxRx xxxx xxxx
 *
 * For PIM ports (T,S,G,H,J,L), selection is per-pin
 * For MEBI ports (A,B,E,K), selection is per-port
 */

#define GPIO_REDUCED (1 << 9)

/* Initial value of output:
 *
 *   xxxx xxxV xxxx xxxx
 *
 * For PIM ports (T,S,G,H,J,L), selection is per-pin
 * For MEBI ports (A,B,E,K), selection is per-port
 */

#define GPIO_OUTPUT_VALUE (1 << 8)
#define GPIO_OUTPUT_LOW   (0)
#define GPIO_OUTPUT_HIGH  GPIO_OUTPUT_VALUE

/* Data direction (All ports -- A,B,E,K,T,S,G,H,J,L)
 *
 *   xxxx xxxx Dxxx xxxx
 *
 */

#define GPIO_DIRECTION (1 << 7)
#  define GPIO_INPUT   (0)
#  define GPIO_OUTPUT   GPIO_DIRECTION

/* Port selection
 *
 *   xxxx xxxx xGGG Gxxx
 *
 * Ports A, B, E, and K reside in the MEBI block
 * Ports T,S,G,H,J, and L reside in the PIM block.
 */

#define GPIO_PORT_SHIFT 3
#define GPIO_PORT_MASK  (15 << GPIO_PORT_SHIFT)
#  define GPIO_PORT_A   (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORT_B   (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORT_E   (2 << GPIO_PORT_SHIFT)
#  define GPIO_PORT_K   (3 << GPIO_PORT_SHIFT)

#  define GPIO_PORT_PIM (8 << GPIO_PORT_SHIFT)
#  define GPIO_PORT_T   (8 << GPIO_PORT_SHIFT)
#  define GPIO_PORT_S   (9 << GPIO_PORT_SHIFT
#  define GPIO_PORT_G   (10 << GPIO_PORT_SHIFT)
#  define GPIO_PORT_H   (11 << GPIO_PORT_SHIFT)
#  define GPIO_PORT_J   (12 << GPIO_PORT_SHIFT)
#  define GPIO_PORT_L   (13 << GPIO_PORT_SHIFT)

/* Pin numbers
 *
 *   xxxx xxxx xxxx xPPP
 */

#define GPIO_PIN_SHIFT  (0)
#define GPIO_PIN_MASK   (7 << GPIO_PORT_SHIFT)
#  define GPIO_PIN_0    (0 << GPIO_PORT_SHIFT)
#  define GPIO_PIN_1    (1 << GPIO_PORT_SHIFT)
#  define GPIO_PIN_2    (2 << GPIO_PORT_SHIFT)
#  define GPIO_PIN_3    (3 << GPIO_PORT_SHIFT)
#  define GPIO_PIN_4    (4 << GPIO_PORT_SHIFT)
#  define GPIO_PIN_5    (5 << GPIO_PORT_SHIFT)
#  define GPIO_PIN_6    (6 << GPIO_PORT_SHIFT)
#  define GPIO_PIN_7    (7 << GPIO_PORT_SHIFT)

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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: hcs12_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

void hcs12_gpioirqinitialize(void);

/****************************************************************************
 * Name: hcs12_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int hcs12_configgpio(uint16_t cfgset);

/****************************************************************************
 * Name: hcs12_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void hcs12_gpiowrite(uint16_t pinset, bool value);

/****************************************************************************
 * Name: hcs12_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool hcs12_gpioread(uint16_t pinset);

/****************************************************************************
 * Name: hcs12_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_HCS12_GPIOIRQ
void hcs12_gpioirqenable(int irq);
#else
#  define hcs12_gpioirqenable(irq)
#endif

/****************************************************************************
 * Name: hcs12_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_HCS12_GPIOIRQ
void hcs12_gpioirqdisable(int irq);
#else
#  define hcs12_gpioirqdisable(irq)
#endif

/****************************************************************************
 * Function:  hcs12_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided
 *   pinset.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int hcs12_dumpgpio(uint16_t pinset, const char *msg);
#else
#  define hcs12_dumpgpio(p,m)
#endif

/****************************************************************************
 * Function: hcs12_ethinitialize
 *
 * Description:
 *   Initialize the Ethernet driver for one interface.  If the STM32 chip
 *   supports multiple Ethernet controllers, then bould specific logic
 *   must implement hc_netinitialize() and call this function to initialize
 *   the desiresed interfaces.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#if STM32_NTHERNET > 1
int hcs12_ethinitialize(int intf);
#endif

/****************************************************************************
 * Name: hcs12_spibus_initialize
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

struct spi_dev_s;  /* Forward reference */
FAR struct spi_dev_s *hcs12_spibus_initialize(int port);

/****************************************************************************
 * Name:  hcs12_spiselect and hcs12_spistatus
 *
 * Description:
 *   The external functions, hcs12_spiselect and hcs12_spistatus must be
 *   provided by board-specific logic.  They are implementations of the
 *   select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including mps12_spibus_initialize()) are provided by
 *   common STM32 logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in hcs12_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide hcs12_spiselect() and hcs12_spistatus() functions in your
 *      board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to mps12_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by mps12_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

void  hcs12_spiselect(FAR struct spi_dev_s *dev,
                      uint32_t devid, bool selected);
uint8_t hcs12_spistatus(FAR struct spi_dev_s *dev, uint32_t devid);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_HC_SRC_M9S12_M9S12_H */
