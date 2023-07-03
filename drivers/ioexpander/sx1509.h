/****************************************************************************
 * drivers/ioexpander/sx1509.h
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

#ifndef __DRIVERS_IOEXPANDER_SX1509_H
#define __DRIVERS_IOEXPANDER_SX1509_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/sx1509.h>
#ifdef CONFIG_SX1509_LED_ENABLE
#  include <nuttx/leds/userled.h>
#endif

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SX1509 Definitions *******************************************************/

/* I2C frequency */

#define SX1509_I2C_MAXFREQUENCY        400000       /* 400KHz */

/* SX1509 *******************************************************************/

/* Device and IO Banks */

#define SX1509_REGINPUTDISABLE_B      0x00
#define SX1509_REGINPUTDISABLE_A      0x01
#define SX1509_REGLONGSLEW_B          0x02
#define SX1509_REGLONGSLEW_A          0x03
#define SX1509_REGLOWDRIVE_B          0x04
#define SX1509_REGLOWDRIVE_A          0x05
#define SX1509_REGPULLUP_B            0x06
#define SX1509_REGPULLUP_A            0x07
#define SX1509_REGPULLDOWN_B          0x08
#define SX1509_REGPULLDOWN_A          0x09
#define SX1509_REGOPENDRAIN_B         0x0a
#define SX1509_REGOPENDRAIN_A         0x0b
#define SX1509_REGPOLARITY_B          0x0c
#define SX1509_REGPOLARITY_A          0x0d
#define SX1509_REGDIR_B               0x0e
#define SX1509_REGDIR_A               0x0f
#define SX1509_REGDATA_B              0x10
#define SX1509_REGDATA_A              0x11
#define SX1509_REGINTERRUPTMASK_B     0x12
#define SX1509_REGINTERRUPTMASK_A     0x13
#define SX1509_REGSENSEHIGH_B         0x14
#define SX1509_REGSENSELOW_B          0x15
#define SX1509_REGSENSEHIGH_A         0x16
#define SX1509_REGSENSELOW_A          0x17
#define SX1509_REGINTERRUPTSOURCE_B   0x18
#define SX1509_REGINTERRUPTSOURCE_A   0x19
#define SX1509_REGEVENTSTATUS_B       0x1a
#define SX1509_REGEVENTSTATUS_A       0x1b
#define SX1509_REGLEVELSHIFTER_1      0x1c
#define SX1509_REGLEVELSHIFTER_2      0x1d
#define SX1509_REGCLOCK               0x1e
#define SX1509_REGMISC                0x1f
#define SX1509_REGLEDDRIVERENABLE_B   0x20
#define SX1509_REGLEDDRIVERENABLE_A   0x21

/* Debounce and Keypad Engine */

#define SX1509_REGDEBOUNCE_CONFIG     0x22
#define SX1509_REGDEBOUNCEENABLE_B    0x23
#define SX1509_REGDEBOUNCEENABLE_A    0x24
#define SX1509_REGKEYCONFIG_1         0x25
#define SX1509_REGKEYCONFIG_2         0x26
#define SX1509_REGKEYDATA_1           0x27
#define SX1509_REGKEYDATA_2           0x28

/* LED Driver (PWM, blinking, breathing) */

#define SX1509_REGTON_0               0x29
#define SX1509_REGION_0               0x2a
#define SX1509_REGOFF_0               0x2b
#define SX1509_REGTON_1               0x2c
#define SX1509_REGION_1               0x2d
#define SX1509_REGOFF_1               0x2e
#define SX1509_REGTON_2               0x2f
#define SX1509_REGION_2               0x30
#define SX1509_REGOFF_2               0x31
#define SX1509_REGTON_3               0x32
#define SX1509_REGION_3               0x33
#define SX1509_REGOFF_3               0x34
#define SX1509_REGTON_4               0x35
#define SX1509_REGION_4               0x36
#define SX1509_REGOFF_4               0x37
#define SX1509_TRISE_4                0x38
#define SX1509_TFALL_4                0x39
#define SX1509_REGTON_5               0x3a
#define SX1509_REGION_5               0x3b
#define SX1509_REGOFF_5               0x3c
#define SX1509_TRISE_5                0x3d
#define SX1509_TFALL_5                0x3e
#define SX1509_REGTON_6               0x3f
#define SX1509_REGION_6               0x40
#define SX1509_REGOFF_6               0x41
#define SX1509_TRISE_6                0x42
#define SX1509_TFALL_6                0x43
#define SX1509_REGTON_7               0x44
#define SX1509_REGION_7               0x45
#define SX1509_REGOFF_7               0x46
#define SX1509_TRISE_7                0x47
#define SX1509_TFALL_7                0x48
#define SX1509_REGTON_8               0x49
#define SX1509_REGION_8               0x4a
#define SX1509_REGOFF_8               0x4b
#define SX1509_REGTON_9               0x4c
#define SX1509_REGION_9               0x4d
#define SX1509_REGOFF_9               0x4e
#define SX1509_REGTON_10              0x4f
#define SX1509_REGION_10              0x50
#define SX1509_REGOFF_10              0x51
#define SX1509_REGTON_11              0x52
#define SX1509_REGION_11              0x53
#define SX1509_REGOFF_11              0x54
#define SX1509_REGTON_12              0x55
#define SX1509_REGION_12              0x56
#define SX1509_REGOFF_12              0x57
#define SX1509_TRISE_12               0x58
#define SX1509_TFALL_12               0x59
#define SX1509_REGTON_13              0x5a
#define SX1509_REGION_13              0x5b
#define SX1509_REGOFF_13              0x5c
#define SX1509_TRISE_13               0x5d
#define SX1509_TFALL_13               0x5e
#define SX1509_REGTON_14              0x5f
#define SX1509_REGION_14              0x60
#define SX1509_REGOFF_14              0x61
#define SX1509_TRISE_14               0x62
#define SX1509_TFALL_14               0x63
#define SX1509_REGTON_15              0x64
#define SX1509_REGION_15              0x65
#define SX1509_REGOFF_15              0x66
#define SX1509_TRISE_15               0x67
#define SX1509_TFALL_15               0x68

/* Miscellaneous */

#define SX1509_REGHIGHINPUT_B         0x69
#define SX1509_REGHIGHINPUT_A         0x6a

/* Software Reset */

#define SX1509_REGRESET               0x7d

/* Test */

#define SX1509_REGTEST1               0x7e
#define SX1509_REGTEST2               0x7f

/* RegSense */

#define SX1509_SENSE_MASK             (0x3)
#define SX1509_SENSE_NONE             (0)
#define SX1509_SENSE_RISING           (1)
#define SX1509_SENSE_FALLING          (2)
#define SX1509_SENSE_BOTH             (3)

/* RegClock */

#define SX1509_OSC_OFF                (0 << 5)
#define SX1509_OSC_EXT                (1 << 5)
#define SX1509_OSC_INT                (2 << 5)

/* RegMisc */

#define SX1509_LEDCLK_SHIFT           (4)

/* RegTOnX */

#define SX1509_TON_SHIFT              (0)
#define SX1509_TON_1_MAX              (15)
#define SX1509_TON_2_MAX              (15)

/* RegOffX */

#define SX1509_IOFF_SHIFT             (0)
#define SX1509_TOFF_SHIFT             (3)
#define SX1509_TOFF_1_MAX             (15)
#define SX1509_TOFF_2_MAX             (15)

/* Driver definitions */

#define SX1509_NR_GPIO_MAX            16
#define SX1509_POLLDELAY              (CONFIG_SX1509_INT_POLLDELAY / USEC_PER_TICK)
#define SX1509_INTOSC_FREQ            (2000000)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
/* This type represents on registered pin interrupt callback */

struct sx1509_callback_s
{
  ioe_pinset_t    pinset;           /* Set of pin interrupts that will generate
                                     * the callback. */
  ioe_callback_t  cbfunc;           /* The saved callback function pointer */
  FAR void       *cbarg;            /* Callback argument */
};
#endif

/* This structure represents the state of the SX1509 driver */

struct sx1509_dev_s
{
  struct ioexpander_dev_s     dev;    /* Nested structure to allow casting
                                       * as public gpio expander.
                                       */
#ifdef CONFIG_SX1509_MULTIPLE
  /* Supports a singly linked list of drivers */

  FAR struct sx1509_dev_s    *flink;
#endif

  FAR struct sx1509_config_s *config; /* Board configuration data */
  FAR struct i2c_master_s    *i2c;    /* Saved I2C driver instance */
  mutex_t                     lock;   /* Mutual exclusion */

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#  ifdef CONFIG_SX1509_INT_POLL

  /* Timer used to poll for missed interrupts */

  struct wdog_s               wdog;
#  endif

  /* Supports the interrupt handling "bottom half" */

  struct work_s               work;

  /* Saved callback information for each I/O expander client */

  struct sx1509_callback_s cb[CONFIG_SX1509_INT_NCALLBACKS];
#endif

#ifdef CONFIG_SX1509_LED_ENABLE
  /* Lower half LED driver */

  struct userled_lowerhalf_s userleds;

  /* Pin number to LED number map. If set to -1 then no LED assigned. */

  int8_t pin2led[SX1509_NR_GPIO_MAX];

  /* LED driver clock frequency */

  uint32_t led_freq;

  /* LED driver timings */

  uint32_t t_on_1_ms;
  uint32_t t_on_2_ms;

  uint32_t t_off_1_ms;
  uint32_t t_off_2_ms;
#endif
};

#endif /* __DRIVERS_IOEXPANDER_SX1509_H */
