/********************************************************************************************
 * drivers/ioexpander/tca64.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
 *
 * References:
 *   "16-bit I2C-bus and SMBus I/O port with interrupt product datasheet",
 *   Rev. 08 - 22 October 2009, NXP
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

#ifndef __DRIVERS_IOEXPANDER_TCA64XX_H
#define __DRIVERS_IOEXPANDER_TCA64XX_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include <semaphore.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>

#include <nuttx/wqueue.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/tca64xx.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/irq.h>

#if defined(CONFIG_IOEXPANDER) && defined(CONFIG_IOEXPANDER_TCA64XX)

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Configuration ****************************************************************************/
/* Prerequisites:
 *   CONFIG_I2C
 *     I2C support is required
 *   CONFIG_IOEXPANDER
 *     Enables I/O expander support
 *
 * Other settings that effect the driver: CONFIG_DISABLE_POLL
 *
 * CONFIG_IOEXPANDER_TCA64XX
 *   Enables support for the TCA64XX driver (Needs CONFIG_INPUT)
 * CONFIG_TCA64XX_MULTIPLE
 *   Can be defined to support multiple TCA64XX devices on board.
 * CONFIG_TCA64XX_INT_NCALLBACKS
 *   Maximum number of supported pin interrupt callbacks.
 * CONFIG_TCA64XX_INT_POLL
 *   Enables a poll for missed interrupts
 * CONFIG_TCA64XX_INT_POLLDELAY
 *   If CONFIG_TCA64XX_INT_POLL=y, then this is the delay in microseconds
 *   between polls for missed interrupts.
 */

#ifndef CONFIG_I2C
#  error "CONFIG_I2C is required by TCA64XX"
#endif

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#  ifndef CONFIG_TCA64XX_INT_NCALLBACKS
#    define CONFIG_TCA64XX_INT_NCALLBACKS 4
#  endif
#endif

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#  ifndef CONFIG_SCHED_WORKQUEUE
#    error Work queue support required.  CONFIG_SCHED_WORKQUEUE must be selected.
#  endif
#endif

#ifndef CONFIG_TCA64XX_INT_POLLDELAY
#  define CONFIG_TCA64XX_INT_POLLDELAY 500000
#endif

/* TCA64XX Definitions **********************************************************************/

/* I2C frequency */

#define TCA64XX_I2C_MAXFREQUENCY        400000       /* 400KHz */

/* TCA64XX Parts ****************************************************************************/

#define TCA6408_INPUT_REG               0x00
#define TCA6408_OUTPUT_REG              0x01
#define TCA6408_POLARITY_REG            0x02
#define TCA6408_CONFIG_REG              0x03

#define TCA6408_NR_GPIOS                8

#define TCA6416_INPUT0_REG              0x00
#define TCA6416_INPUT1_REG              0x01
#define TCA6416_OUTPUT0_REG             0x02
#define TCA6416_OUTPUT1_REG             0x03
#define TCA6416_POLARITY0_REG           0x04
#define TCA6416_POLARITY1_REG           0x05
#define TCA6416_CONFIG0_REG             0x06
#define TCA6416_CONFIG1_REG             0x07

#define TCA6416_NR_GPIOS                16

#define TCA6424_INPUT0_REG              0x00
#define TCA6424_INPUT1_REG              0x01
#define TCA6424_INPUT2_REG              0x02
#define TCA6424_OUTPUT0_REG             0x04
#define TCA6424_OUTPUT1_REG             0x05
#define TCA6424_OUTPUT2_REG             0x06
#define TCA6424_POLARITY0_REG           0x08
#define TCA6424_POLARITY1_REG           0x09
#define TCA6424_POLARITY2_REG           0x0A
#define TCA6424_CONFIG0_REG             0x0C
#define TCA6424_CONFIG1_REG             0x0D
#define TCA6424_CONFIG2_REG             0x0E

#define TCA6424_NR_GPIOS                24

#define TCA64XX_NR_GPIO_MAX             TCA6424_NR_GPIOS

/* 1us (datasheet: reset pulse duration (Tw) is 4ns */

#define TCA64XX_TW                      1

/* 1us (datasheet: time to reset (Treset) is 600ns */

#define TCA64XX_TRESET                  1

#define TCA64XX_IRQ_TYPE_EDGE_BOTH      0x00000000
#define TCA64XX_IRQ_TYPE_EDGE_RISING    0x00000001
#define TCA64XX_IRQ_TYPE_EDGE_FALLING   0x00000002
#define TCA64XX_IRQ_TYPE_LEVEL_HIGH     0x00000001
#define TCA64XX_IRQ_TYPE_LEVEL_LOW      0x00000002

#define TCA64XX_IRQ_TYPE_EDGE           0x00000000
#define TCA64XX_IRQ_TYPE_LEVEL          0x00000001

#define TCA64XX_POLLDELAY       (CONFIG_TCA64XX_INT_POLLDELAY / USEC_PER_TICK)

#define TCA64_LEVEL_SENSITIVE(d,p) \
  (((d)->trigger  & ((ioe_pinset_t)1 << (p))) == 0)
#define TCA64_LEVEL_HIGH(d,p) \
  (((d)->level[0] & ((ioe_pinset_t)1 << (p))) != 0)
#define TCA64_LEVEL_LOW(d,p) \
  (((d)->level[1] & ((ioe_pinset_t)1 << (p))) != 0)

#define TCA64_EDGE_SENSITIVE(d,p) \
  (((d)->trigger  & ((ioe_pinset_t)1 << (p))) != 0)
#define TCA64_EDGE_RISING(d,p) \
  (((d)->level[0] & ((ioe_pinset_t)1 << (p))) != 0)
#define TCA64_EDGE_FALLING(d,p) \
  (((d)->level[1] & ((ioe_pinset_t)1 << (p))) != 0)
#define TCA64_EDGE_BOTH(d,p) \
  (TCA64_LEVEL_RISING(d,p) && TCA64_LEVEL_FALLING(d,p))

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/* This structure represents the configuration of one part */

struct tca64_part_s
{
  uint8_t tp_id;       /* Part ID (see enum tca64xx_part_e) */
  uint8_t tp_ngpios;   /* Number of supported GPIOs */
  uint8_t tp_input;    /* Address of first input register */
  uint8_t tp_output;   /* Address of first output register */
  uint8_t tp_polarity; /* Address of first polarity register */
  uint8_t tp_config;   /* Address of first configuration register */
};

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
/* This type represents on registered pin interrupt callback */

struct tca64_callback_s
{
   ioe_pinset_t pinset;              /* Set of pin interrupts that will generate
                                      * the callback. */
   ioe_callback_t cbfunc;            /* The saved callback function pointer */
   FAR void *cbarg;                  /* Callback argument */
};
#endif

/* This structure represents the state of the TCA64XX driver */

struct tca64_dev_s
{
  struct ioexpander_dev_s dev;       /* Nested structure to allow casting as public gpio
                                      * expander. */
  FAR struct tca64_config_s *config; /* Board configuration data */
  FAR struct i2c_master_s *i2c;      /* Saved I2C driver instance */
  uint8_t part;                      /* TCA64xx part ID (see enum tca64xx_part_e) */
  sem_t exclsem;                     /* Mutual exclusion */

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#ifdef CONFIG_TCA64XX_INT_POLL
  WDOG_ID wdog;                      /* Timer used to poll for missed interrupts */
#endif

  ioe_pinset_t input;                /* Last input registeres */
  ioe_pinset_t intstat;              /* Pending interrupts */
  ioe_pinset_t trigger;              /* Bit encoded: 0=level 1=edge */
  ioe_pinset_t level[2];             /* Bit encoded: 01=high/rising, 10 low/falling, 11 both */
  struct work_s work;                /* Supports the interrupt handling "bottom half" */

  /* Saved callback information for each I/O expander client */

  struct tca64_callback_s cb[CONFIG_TCA64XX_INT_NCALLBACKS];
#endif
};

#endif /* CONFIG_IOEXPANDER && CONFIG_IOEXPANDER_TCA64XX */
#endif /* __DRIVERS_IOEXPANDER_TCA64XX_H */
