/****************************************************************************
 * include/nuttx/audio/cs43l22.h
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

#ifndef __INCLUDE_NUTTX_AUDIO_CS43L22_H
#define __INCLUDE_NUTTX_AUDIO_CS43L22_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/irq.h>

#ifdef CONFIG_AUDIO_CS43L22

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************
 *
 * CONFIG_AUDIO_CS43L22 - Enables CS43L22 support
 * CONFIG_CS43L22_INITVOLUME - The initial volume level
 *                             in the range {0..1000}
 * CONFIG_CS43L22_INFLIGHT - Maximum number of buffers that the CS43L22
 *   driver will send to the I2S driver before any have completed.
 * CONFIG_CS43L22_MSG_PRIO - Priority of messages sent to the CS43L22
 *   worker thread.
 * CONFIG_CS43L22_BUFFER_SIZE - Preferred buffer size
 * CONFIG_CS43L22_NUM_BUFFERS - Preferred number of buffers
 * CONFIG_CS43L22_WORKER_STACKSIZE - Stack size to use when creating the the
 *   CS43L22 worker thread.
 * CONFIG_CS43L22_REGDUMP - Enable logic to dump all CS43L22 registers to
 *   the SYSLOG device.
 */

/* Pre-requisites */

#ifndef CONFIG_AUDIO
#  error CONFIG_AUDIO is required for audio subsystem support
#endif

#ifndef CONFIG_I2S
#  error CONFIG_I2S is required by the CS43L22 driver
#endif

#ifndef CONFIG_I2C
#  error CONFIG_I2C is required by the CS43L22 driver
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error CONFIG_SCHED_WORKQUEUE is required by the CS43L22 driver
#endif

/* Default configuration values */

#ifndef CONFIG_CS43L22_INITVOLUME
#  define CONFIG_CS43L22_INITVOLUME       400
#endif

#ifndef CONFIG_CS43L22_INFLIGHT
#  define CONFIG_CS43L22_INFLIGHT          2
#endif

#if CONFIG_CS43L22_INFLIGHT > 255
#  error CONFIG_CS43L22_INFLIGHT must fit in a uint8_t
#endif

#ifndef CONFIG_CS43L22_MSG_PRIO
#  define CONFIG_CS43L22_MSG_PRIO          1
#endif

#ifndef CONFIG_CS43L22_BUFFER_SIZE
#  define CONFIG_CS43L22_BUFFER_SIZE       8192
#endif

#ifndef CONFIG_CS43L22_NUM_BUFFERS
#  define CONFIG_CS43L22_NUM_BUFFERS       4
#endif

#ifndef CONFIG_CS43L22_WORKER_STACKSIZE
#  define CONFIG_CS43L22_WORKER_STACKSIZE  768
#endif

/* Helper macros ************************************************************/

#define CS43L22_ATTACH(s,isr,arg) ((s)->attach(s,isr,arg))
#define CS43L22_DETACH(s)         ((s)->attach(s,NULL,NULL))
#define CS43L22_ENABLE(s)         ((s)->enable(s,true))
#define CS43L22_DISABLE(s)        ((s)->enable(s,false))
#define CS43L22_RESTORE(s,e)      ((s)->enable(s,e))
#define CS43L22_HW_RESET(s)       ((s)->reset(s))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the type of the CS43L22 interrupt handler.  The lower level code
 * will intercept the interrupt and provide the upper level with the private
 * data that was provided when the interrupt was attached.
 */

struct cs43l22_lower_s; /* Forward reference.  Defined below */

typedef CODE int (*cs43l22_handler_t)
                 (FAR const struct cs43l22_lower_s *lower,
                  FAR void *arg);

/* A reference to a structure of this type must be passed to the CS43L22
 * driver.  This structure provides information about the configuration
 * of the CS43L22 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

struct cs43l22_lower_s
{
  /* I2C characterization */

  uint32_t frequency;  /* Initial I2C frequency */
  uint8_t  address;    /* 7-bit I2C address (only bits 0-6 used) */

  /* Clocking is provided via MCLK.  The CS43L22 driver will need to know
   * the frequency of MCLK in order to generate the correct bitrates.
   */

  uint32_t mclk;       /* CS43L22 Master clock frequency */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the CS43L22 driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.  If possible,
   * interrupts should be configured on both rising and falling edges
   * so that contact and loss-of-contact events can be detected.
   *
   * attach  - Attach or detach the CS43L22 interrupt handler to the GPIO
   *           interrupt
   * enable  - Enable or disable the GPIO interrupt.  Returns the
   *           previous interrupt state.
   * reset   - HW reset of the CS43L22 chip
   */

  CODE int  (*attach)(FAR const struct cs43l22_lower_s *lower,
                      cs43l22_handler_t isr, FAR void *arg);
  CODE bool (*enable)(FAR const struct cs43l22_lower_s *lower, bool enable);
  CODE void (*reset)(FAR const struct cs43l22_lower_s *lower);
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
 * Name: cs43l22_initialize
 *
 * Description:
 *   Initialize the CS43L22 device.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   i2s     - An I2S driver instance
 *   lower   - Persistent board configuration data
 *
 * Returned Value:
 *   A new lower half audio interface for the CS43L22 device is returned on
 *   success; NULL is returned on failure.
 *
 ****************************************************************************/

struct i2c_master_s;      /* Forward reference. Defined in include/nuttx/i2c/i2c_master.h */
struct i2s_dev_s;         /* Forward reference. Defined in include/nuttx/audio/i2s.h */
struct audio_lowerhalf_s; /* Forward reference. Defined in nuttx/audio/audio.h */

FAR struct audio_lowerhalf_s *
  cs43l22_initialize(FAR struct i2c_master_s *i2c, FAR struct i2s_dev_s *i2s,
                    FAR const struct cs43l22_lower_s *lower);

/****************************************************************************
 * Name: cs43l22_dump_registers
 *
 * Description:
 *   Dump the contents of all CS43L22 registers to the syslog device
 *
 * Input Parameters:
 *   dev - The device instance returned by cs43l22_initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_CS43L22_REGDUMP
void cs43l22_dump_registers(FAR struct audio_lowerhalf_s *dev,
                           FAR const char *msg);
#else
  /* This eliminates the need for any conditional compilation in the
   * including file.
   */

#  define cs43l22_dump_registers(d,m)
#endif

/****************************************************************************
 * Name: cs43l22_clock_analysis
 *
 * Description:
 *   Analyze the settings in the clock chain and dump to syslog.
 *
 * Input Parameters:
 *   dev - The device instance returned by cs43l22_initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_CS43L22_CLKDEBUG
void cs43l22_clock_analysis(FAR struct audio_lowerhalf_s *dev,
                           FAR const char *msg);
#else
  /* This eliminates the need for any conditional compilation in the
   * including file.
   */

#  define cs43l22_clock_analysis(d,m)
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_AUDIO_CS43L22 */
#endif /* __INCLUDE_NUTTX_AUDIO_CS43L22_H */
