/****************************************************************************
 * include/nuttx/wireless/cc3000/include/cc3000_upif.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
 *
 * This is the definition for the interface to the bottom half of the cc3000 driver.
 * it provides the wiring of the board's GPIO and spi to the cc3000 driver.
 *
 * References:
 *   CC30000 from Texas Instruments http://processors.wiki.ti.com/index.php/CC3000
 *
 * See also:
 *     http://processors.wiki.ti.com/index.php/CC3000_Host_Driver_Porting_Guide
 *     http://processors.wiki.ti.com/index.php/CC3000_Host_Programming_Guide
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

#ifndef __INCLUDE_NUTTX_WIRELESS_CC3000_INCLUDE_CC3000_UPIFL_H
#define __INCLUDE_NUTTX_WIRELESS_CC3000_INCLUDE_CC3000_UPIFL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>

#if defined(CONFIG_DRIVERS_WIRELESS) && defined(CONFIG_WL_CC3000)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
/* SPI Frequency.  Default:  16MHz */

#ifndef CONFIG_CC3000_SPI_FREQUENCY
#  define CONFIG_CC3000_SPI_FREQUENCY 16000000
#endif

/* Maximum number of threads than can be waiting for POLL events */

#ifndef CONFIG_CC3000_NPOLLWAITERS
#  define CONFIG_CC3000_NPOLLWAITERS 2
#endif

#ifndef CONFIG_CC3000_SPI_MODE
/* CPOL = 0, CPHA = 1 Sample Data Falling Edge of Clock
 * See http://processors.wiki.ti.com/index.php/CC3000_Serial_Port_Interface_(SPI)
 */

#  define CONFIG_CC3000_SPI_MODE SPIDEV_MODE1
#endif

/* Check for some required settings.  This can save the user a lot of time
 * in getting the right configuration.
 */

#ifdef CONFIG_DISABLE_SIGNALS
#  error "Signals are required.  CONFIG_DISABLE_SIGNALS must not be selected."
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the CC3000
 * driver.  This structure provides information about the configuration
 * of the CC3000 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

struct cc3000_config_s
{
  /* Device characterization */

  uint32_t spi_frequency; /* SPI frequency */
  uint32_t spi_mode;      /* SPI mode */
  size_t   max_rx_size;   /* Size allocated by driver for rx buffer */

  /* If multiple CC3000 devices are supported, then an IRQ number must
   * be provided for each so that their interrupts can be distinguished.
   */

#ifndef CONFIG_CC3000_MULTIPLE
  int      irq;           /* IRQ number received by interrupt handler. */
#endif

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the CC3000 driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.  If possible,
   * interrupts should be configured on falling edges to detect the Ready Condition
   * At T2: The normal master SPI write sequence is SPI_CS low, followed by SPI_IRQ low
   * CC3000 to host, indicating that the CC3000 core module is ready to accept data.
   * T2 duration is approximately 7 ms.
   *
   *   irq_attach       - Attach the CC3000 interrupt handler to the GPIO interrupt
   *   irq_enable       - Enable or disable the GPIO interrupt
   *   irq_clear        - Acknowledge/clear any pending GPIO interrupt
   *   power_enable     - Enable or disable Module enable.
   *   chip_chip_select - The Chip Select
   *   irq_read         - Return the state of the interrupt GPIO input
   *   probe            - Debug support
   */

  int  (*irq_attach)(FAR struct cc3000_config_s *state, xcpt_t isr, FAR void *arg);
  void (*irq_enable)(FAR struct cc3000_config_s *state, bool enable);
  void (*irq_clear)(FAR struct cc3000_config_s *state);
  void (*power_enable)(FAR struct cc3000_config_s *state,bool enable);
  void (*chip_chip_select)(FAR struct cc3000_config_s *state,bool enable);
  bool (*irq_read)(FAR struct cc3000_config_s *state);
#ifdef CONFIG_CC3000_PROBES
  bool (*probe)(FAR struct cc3000_config_s *state, int n, bool s);
#endif
};

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
 * Name: cc3000_register
 *
 * Description:
 *   Configure the CC3000 to use the provided SPI device instance.  This
 *   will register the driver as /dev/inputN where N is the minor device
 *   number
 *
 * Input Parameters:
 *   spi     - An SPI driver instance
 *   config  - Persistent board configuration data
 *   minor   - The CC000 device minor number
 *
 * Returned Value:
 *    Pointer to newly allocated cc3000 device structure or NULL on error
 *    (errno is set accordingly in this case).
 *
 ****************************************************************************/

int cc3000_register(FAR struct spi_dev_s *spi,
                    FAR struct cc3000_config_s *config, int minor);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DRIVERS_WIRELESS && CONFIG_INPUT_CC3000 */
#endif /* __INCLUDE_NUTTX_WIRELESS_CC3000_INCLUDE_CC3000_UPIFL_H */
