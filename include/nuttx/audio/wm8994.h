/****************************************************************************
 * include/nuttx/audio/wm8994.h
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

#ifndef __INCLUDE_NUTTX_AUDIO_WM8994_H
#define __INCLUDE_NUTTX_AUDIO_WM8994_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/irq.h>

#ifdef CONFIG_AUDIO_WM8994

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************
 *
 * CONFIG_AUDIO_WM8994 - Enables WM8994 support
 * CONFIG_WM8994_INITVOLUME - The initial volume level in the range {0..1000}
 * CONFIG_WM8994_INFLIGHT - Maximum number of buffers that the WM8994 driver
 *   will send to the I2S driver before any have completed.
 * CONFIG_WM8994_MSG_PRIO - Priority of messages sent to the WM8994 worker
 *   thread.
 * CONFIG_WM8994_BUFFER_SIZE - Preferred buffer size
 * CONFIG_WM8994_NUM_BUFFERS - Preferred number of buffers
 * CONFIG_WM8994_WORKER_STACKSIZE - Stack size to use when creating the
 *   WM8994 worker thread.
 * CONFIG_WM8994_REGDUMP - Enable logic to dump all WM8994 registers to
 *   the SYSLOG device.
 */

/* Pre-requisites */

#ifndef CONFIG_AUDIO
#  error CONFIG_AUDIO is required for audio subsystem support
#endif

#ifndef CONFIG_I2S
#  error CONFIG_I2S is required by the WM8994 driver
#endif

#ifndef CONFIG_I2C
#  error CONFIG_I2C is required by the WM8994 driver
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error CONFIG_SCHED_WORKQUEUE is required by the WM8994 driver
#endif

/* Default configuration values */

#ifndef CONFIG_WM8994_INITVOLUME
#  define CONFIG_WM8994_INITVOLUME       250
#endif

#ifndef CONFIG_WM8994_INFLIGHT
#  define CONFIG_WM8994_INFLIGHT          2
#endif

#if CONFIG_WM8994_INFLIGHT > 255
#  error CONFIG_WM8994_INFLIGHT must fit in a uint8_t
#endif

#ifndef CONFIG_WM8994_MSG_PRIO
#  define CONFIG_WM8994_MSG_PRIO          1
#endif

#ifndef CONFIG_WM8994_BUFFER_SIZE
#  define CONFIG_WM8994_BUFFER_SIZE       8192
#endif

#ifndef CONFIG_WM8994_NUM_BUFFERS
#  define CONFIG_WM8994_NUM_BUFFERS       4
#endif

#ifndef CONFIG_WM8994_WORKER_STACKSIZE
#  define CONFIG_WM8994_WORKER_STACKSIZE  768
#endif

/* Helper macros ************************************************************/

#define WM8994_ATTACH(s,isr,arg) ((s)->attach(s,isr,arg))
#define WM8994_DETACH(s)         ((s)->attach(s,NULL,NULL))
#define WM8994_ENABLE(s)         ((s)->enable(s,true))
#define WM8994_DISABLE(s)        ((s)->enable(s,false))
#define WM8994_RESTORE(s,e)      ((s)->enable(s,e))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the type of the WM8994 interrupt handler.  The lower level code
 * will intercept the interrupt and provide the upper level with the private
 * data that was provided when the interrupt was attached.
 */

struct wm8994_lower_s; /* Forward reference.  Defined below */

typedef CODE int (*wm8994_handler_t)(FAR const struct wm8994_lower_s *lower,
                                     FAR void *arg);

/* A reference to a structure of this type must be passed to the WM8994
 * driver.  This structure provides information about the configuration
 * of the WM8994 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

struct wm8994_lower_s
{
  /* I2C characterization */

  uint32_t frequency;  /* Initial I2C frequency */
  uint8_t  address;    /* 7-bit I2C address (only bits 0-6 used) */

  /* Clocking is provided via MCLK.  The WM8994 driver will need to know
   * the frequency of MCLK in order to generate the correct bitrates.
   */

  uint32_t mclk;       /* W8994 Master clock frequency */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the WM8994 driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.  If possible,
   * interrupts should be configured on both rising and falling edges
   * so that contact and loss-of-contact events can be detected.
   *
   * attach  - Attach or detach the WM8994 interrupt handler to the GPIO
   *           interrupt
   * enable  - Enable or disable the GPIO interrupt.  Returns the
   *           previous interrupt state.
   */

  CODE int  (*attach)(FAR const struct wm8994_lower_s *lower,
                      wm8994_handler_t isr, FAR void *arg);
  CODE bool (*enable)(FAR const struct wm8994_lower_s *lower, bool enable);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
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
 * Name: wm8994_initialize
 *
 * Description:
 *   Initialize the WM8994 device.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   i2s     - An I2S driver instance
 *   lower   - Persistent board configuration data
 *
 * Returned Value:
 *   A new lower half audio interface for the WM8994 device is returned on
 *   success; NULL is returned on failure.
 *
 ****************************************************************************/

struct i2c_master_s;      /* Forward reference. Defined in include/nuttx/i2c/i2c_master.h */
struct i2s_dev_s;         /* Forward reference. Defined in include/nuttx/audio/i2s.h */
struct audio_lowerhalf_s; /* Forward reference. Defined in nuttx/audio/audio.h */

FAR struct audio_lowerhalf_s *
  wm8994_initialize(FAR struct i2c_master_s *i2c, FAR struct i2s_dev_s *i2s,
                    FAR const struct wm8994_lower_s *lower);

/****************************************************************************
 * Name: wm8994_dump_registers
 *
 * Description:
 *   Dump the contents of all WM8994 registers to the syslog device
 *
 * Input Parameters:
 *   dev - The device instance returned by wm8994_initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_WM8994_REGDUMP
void wm8994_dump_registers(FAR struct audio_lowerhalf_s *dev,
                           FAR const char *msg);
#else
  /* This eliminates the need for any conditional compilation in the
   * including file.
   */

#  define wm8994_dump_registers(d,m)
#endif

/****************************************************************************
 * Name: wm8994_clock_analysis
 *
 * Description:
 *   Analyze the settings in the clock chain and dump to syslog.
 *
 * Input Parameters:
 *   dev - The device instance returned by wm8994_initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_WM8994_CLKDEBUG
void wm8994_clock_analysis(FAR struct audio_lowerhalf_s *dev,
                           FAR const char *msg);
#else
  /* This eliminates the need for any conditional compilation in the
   * including file.
   */

#  define wm8994_clock_analysis(d,m)
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_AUDIO_WM8994 */
#endif /* __INCLUDE_NUTTX_AUDIO_WM8994_H */
