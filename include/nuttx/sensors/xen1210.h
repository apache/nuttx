/********************************************************************************************
 * include/nuttx/input/xen1210.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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
 ********************************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_XEN1210_H
#define __INCLUDE_NUTTX_SENSORS_XEN1210_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#if defined(CONFIG_SENSORS_XEN1210)

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Configuration ****************************************************************************/
/* Prerequisites:
 *
 * CONFIG_SCHED_WORKQUEUE - Work queue support is required
 *
 * Settings that effect the driver: CONFIG_DISABLE_POLL
 *
 * CONFIG_SENSORS_XEN1210
 *   Enables support for the XEN1210 driver
 */

#ifdef CONFIG_DISABLE_SIGNALS
#  error "Signals are required.  CONFIG_DISABLE_SIGNALS must not be selected."
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Work queue support required.  CONFIG_SCHED_WORKQUEUE must be selected."
#endif

/* The XEN1210 interfaces with the target CPU via a SPI interface. */

/* SPI **************************************************************************************/
/* The device always operates in mode 1 */

#define XEN1210_SPI_MODE            SPIDEV_MODE1 /* Mode 1 */

/* SPI frequency */

#define XEN1210_SPI_MAXFREQUENCY    100000       /* 100KHz */

/* XEN1210 Commands  ************************************************************************/
/* Operation Commands */

#define XEN1210_RESET               0x10  /* System-Reset command */
#define XEN1210_POWERON             0x20  /* Power-ON command */
#define XEN1210_POWEROFF            0x40  /* Power-OFF command */
#define XEN1210_SINGLESHOT          0x60  /* Single-Shot command */

/* Settings Commands */

#define XEN1210_TIMING              0x01  /* Timing and biasing settings */
#define XEN1210_TEST                0x02  /* Testing */

/* Timming Cycles */

#define XEN1210_1K_CYCLES           0x1113 /* 1024 cycles used be internale ADC */
#define XEN1210_2K_CYCLES           0x2113 /* 2048 cycles used be internale ADC */
#define XEN1210_4K_CYCLES           0x3113 /* 4096 cycles used be internale ADC */
#define XEN1210_8K_CYCLES           0x4113 /* 8192 cycles used be internale ADC */
#define XEN1210_16K_CYCLES          0x5113 /* 16384 cycles used be internale ADC */
#define XEN1210_32K_CYCLES          0x6113 /* 32768 cycles used be internale ADC */
#define XEN1210_64K_CYCLES          0x7113 /* 64536 cycles used be internale ADC */

/* Test setting */

#define XEN1210_TESTVALUE           0x3A00 /* Default value to be used with Test command */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/* Form of the GPIO "interrupt handler" callback. Callbacks do not occur from an interrupt
 * handler but rather from the context of the worker thread with interrupts enabled.
 */

struct xen1210_config_s;
typedef void (*xen1210_handler_t)(FAR struct xen1210_config_s *config, FAR void *arg);

/* A reference to a structure of this type must be passed to the XEN1210 driver when the
 * driver is instantiated. This structure provides information about the configuration of
 * the XEN1210 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied by the driver
 * and is presumed to persist while the driver is active. The memory must be writeable
 * because, under certain circumstances, the driver may modify the frequency.
 */

struct xen1210_config_s
{
  /* Device characterization */

  uint32_t frequency;  /* SPI frequency */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the XEN1210 driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.
   *
   * attach  - Attach the XEN1210 interrupt handler to the GPIO interrupt
   * enable  - Enable or disable the GPIO interrupt
   * clear   - Acknowledge/clear any pending GPIO interrupt
   */

  int  (*attach)(FAR struct xen1210_config_s *state, xen1210_handler_t handler,
                 FAR void *arg);
  void (*enable)(FAR struct xen1210_config_s *state, bool enable);
  void (*clear)(FAR struct xen1210_config_s *state);
};

typedef FAR void *XEN1210_HANDLE;

struct spi_dev_s;

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/********************************************************************************************
 * Name: xen1210_instantiate
 *
 * Description:
 *   Instantiate and configure the XEN1210 device driver to use the provided I2C or SPI
 *   device instance.
 *
 * Input Parameters:
 *   dev     - A SPI driver instance
 *   config  - Persistent board configuration data
 *
 * Returned Value:
 *   A non-zero handle is returned on success.  This handle may then be used to configure
 *   the XEN1210 driver as necessary.  A NULL handle value is returned on failure.
 *
 ********************************************************************************************/

XEN1210_HANDLE xen1210_instantiate(FAR struct spi_dev_s *dev,
                                   FAR struct xen1210_config_s *config);

/********************************************************************************************
 * Name: xen1210_register
 *
 * Description:
 *  This function will register the magnetometer driver as /dev/magN
 *  where N is the minor device number
 *
 * Input Parameters:
 *   handle    - The handle previously returned by xen1210_instantiate
 *   minor     - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is returned to indicate
 *   the nature of the failure.
 *
 ********************************************************************************************/

int xen1210_register(XEN1210_HANDLE handle, int minor);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_XEN1210 */
#endif /* __INCLUDE_NUTTX_SENSORS_XEN1210_H */
