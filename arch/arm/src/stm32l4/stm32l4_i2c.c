/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_i2c.c
 * STM32L4 I2C driver - based on STM32F7 I2C Hardware Layer - Device Driver
 *
 * Original STM32L4 I2C driver:
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *   Copyright (C) 2011-2013, 2016-2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Author: John Wharington
 *   Author: Sebastien Lorquet
 *   Author: dev@ziggurat29.com
 *
 * STM32L4 I2C driver based on STM32F7 I2C driver:
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * With extensions and modifications for the F1, F2, and F4 by:
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            John Wharington
 *            David Sidrane <david_s5@nscdg.com>
 *
 * Major rewrite of ISR and supporting methods, including support
 * for NACK and RELOAD by:
 *
 *   Copyright (c) 2016 Doug Vetter.  All rights reserved.
 *   Author: Doug Vetter <oss@aileronlabs.com>
 *
 * Port from STM32F7 to STM32L4:
 *   Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

/* --------------------------------------------------------------------------
 *
 * STM32 L4 I2C Driver based on STM32 F7 I2C Driver:
 *
 * STM32 L4 and F7 peripheral differences:
 *  - I2C Peripheral on these families are identical (incl. errata) except
 *    for I2C_CR1 register bit 18:
 *      STM32F7: reserved
 *      STM32L4: WUPEN, Wakeup from Stop mode enable. Requires use of HSI
 *               clock for I2C (RCC_CCIPR_I2CxSEL=HSI).
 *
 * Supports:
 *  - Master operation:
 *      Standard-mode (up to 100 kHz)
 *      Fast-mode (up to 400 kHz)
 *      fI2CCLK clock source selection is based on RCC_CCIPR_I2CxSEL
 *      being set to PCLK and the calculations are based on PCLK frequency
 *      of 80 MHz
 *
 *  - Multiple instances (shared bus)
 *  - Interrupt based operation
 *  - RELOAD support
 *  - I2C_M_NOSTART support
 *
 * Test Environment:
 *  - STM32L451CEU6 based board with I2C slaves LIS2DH accelerometer and
 *    BQ24296 charger
 *
 * Supported on STM32F7 driver, untested on STM32L4:
 *  - Master operation:
 *      Fast-mode Plus (up to 1 MHz)
 *  - Test Environment:
 *      STM32F7676ZI on ST Nucleo-144 Board (ST Part STM32F429ZIT6)
 *
 * Unsupported, possible future work:
 *  - Wakeup from Stop mode
 *  - More effective error reporting to higher layers
 *  - Slave operation
 *  - Support of fI2CCLK frequencies other than 80 MHz and other clock
 *    sources
 *  - Polled operation (code present but untested)
 *  - SMBus support
 *  - Multi-master support
 *  - IPMI
 *
 * Operational Status:
 *
 *  All supported features have been tested and found to be operational.
 *
 *  Although the RELOAD capability has been tested as it was required to
 *  implement the I2C_M_NOSTART flag on F3 hardware, the associated
 *  logic to support the transfer messages with more than 255 byte
 *  payloads has not been  tested as the author lacked access to a real
 *  device supporting these types of transfers.
 *
 * Performance Benchmarks: TBD
 *
 *  Time to transfer two messages, each a byte in length, in addition to the
 *  START condition, in interrupt mode:
 *
 *  DEBUG enabled (development): TBDms
 *      Excessive delay here is caused by printing to the console and
 *      is of no concern.
 *
 *  DEBUG disabled (production): TBSus
 *      Between Messages: TBDus
 *      Between Bytes: TBDus
 *
 * Implementation:
 *
 *  - Device: structure as defined by the nuttx/i2c/i2c.h
 *
 *  - Instance: represents each individual access to the I2C driver, obtained
 *      by the i2c_init(); it extends the Device structure from the
 *      nuttx/i2c/i2c.h;
 *      Instance points to OPS, to common I2C Hardware private data and
 *      contains its own private data including frequency, address and mode
 *      of operation.
 *
 *  - Private: Private data of an I2C Hardware
 *
 * High Level Functional Desecription
 *
 * This driver works with I2C "messages" (struct i2c_msg_s), which carry a
 * buffer intended to transfer data to, or store data read from, the I2C bus.
 *
 * As the hardware can only transmit or receive one byte at a time the basic
 * job of the driver (and the ISR specifically) is to process each message in
 * the order they are stored in the message list, one byte at a time.  When
 * no messages are left the ISR exits and returns the result to the caller.
 *
 * The order of the list of I2C messages provided to the driver is important
 * and dependent upon the hardware in use.  A typical I2C transaction between
 * the F3 as an I2C Master and some other IC as a I2C Slave requires two
 * messages that communicate the:
 *
 *    1) Subaddress (register offset on the slave device)
 *    2) Data sent to or read from the device
 *
 * These messages will typically be one byte in length but may be up to 2^31
 * bytes in length.  Incidentally, the maximum length is limited only because
 * i2c_msg_s.length is a signed int for some odd reason.
 *
 * Interrupt mode relies on the following interrupt events:
 *
 *   TXIS  - Transmit interrupt
 *           (data transmitted to bus and acknowedged)
 *   NACKF - Not Acknowledge Received
 *           (data transmitted to bus and NOT acknowledged)
 *   RXNE  - Receive interrupt
 *           (data received from bus)
 *   TC    - Transfer Complete
 *           (All bytes in message transferred)
 *   TCR   - Transfer Complete (Reload)
 *           (Current batch of bytes in message transferred)
 *
 * The driver currently supports Single Master mode only.  Slave mode is not
 * supported.  Additionally, the driver runs in Software End Mode (AUTOEND
 * disabled) so the driver is responsible for telling the hardware what to
 * do at the end of a transfer.
 *
 * --------------------------------------------------------------------------
 *
 * Configuration:
 *
 *  To use this driver, enable the following configuration variable:
 *
 *    CONFIG_STM32L4_I2C
 *
 *  and one or more interfaces:
 *
 *    CONFIG_STM32L4_I2C1
 *    CONFIG_STM32L4_I2C2
 *    CONFIG_STM32L4_I2C3
 *    CONFIG_STM32L4_I2C4
 *
 *  To configure the ISR timeout using fixed values
 * (CONFIG_STM32L4_I2C_DYNTIMEO=n):
 *
 *    CONFIG_STM32L4_I2CTIMEOSEC   (Timeout in seconds)
 *    CONFIG_STM32L4_I2CTIMEOMS    (Timeout in milliseconds)
 *    CONFIG_STM32L4_I2CTIMEOTICKS (Timeout in ticks)
 *
 *  To configure the ISR timeout using dynamic values
 * (CONFIG_STM32L4_I2C_DYNTIMEO=y):
 *
 *    CONFIG_STM32L4_I2C_DYNTIMEO_USECPERBYTE
 *                    (Timeout in microseconds per byte)
 *    CONFIG_STM32L4_I2C_DYNTIMEO_STARTSTOP
 *                    (Timeout for start/stop inmilliseconds)
 *
 *  Debugging output enabled with:
 *
 *    CONFIG_DEBUG_FEATURES and CONFIG_DEBUG_I2C_{ERROR|WARN|INFO}
 *
 *  ISR Debugging output may be enabled with:
 *
 *    CONFIG_DEBUG_FEATURES and CONFIG_DEBUG_I2C_INFO
 *
 * --------------------------------------------------------------------------
 *
 * References (STM32F7):
 *
 *  RM0316:
 *     ST STM32F76xxx and STM32F77xxx Reference Manual
 *     Document ID: DocID028270 Revision 2, April 2016.
 *
 *  DATASHEET:
 *     ST STM32F777xx/STM32F778Ax/STM32F779x Datasheet
 *     Document ID: DocID028294, Revision 3, May 2016.
 *
 *  ERRATA:
 *     STM32F76xxx/STM32F77xxx Errata sheet Rev A device limitations
 *     Document ID: DocID028806, Revision 2, April 2016.
 *
 *  I2CSPEC:
 *     I2C Bus Specification and User Manual
 *     Document ID: UM10204, Revision 6, April 2014.
 *
 * References (STM32L4):
 *
 *  RM0394:
 *     ST STM32L43xxx STM32L44xxx STM32L45xxx STM32L46xxx Reference Manual
 *     Document ID: DocID027295, Revision 3, April 2017.
 *
 *  ERRATA:
 *     STM32L451xx Errata sheet device limitations
 *     Document ID: DocID030061, Revision 4, September 2017.
 *
 * --------------------------------------------------------------------------
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>
#include <nuttx/power/pm.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32l4_gpio.h"
#include "stm32l4_rcc.h"
#include "stm32l4_i2c.h"
#include "stm32l4_waste.h"

/* At least one I2C peripheral must be enabled */

#if defined(CONFIG_STM32L4_I2C1) || defined(CONFIG_STM32L4_I2C2) || \
    defined(CONFIG_STM32L4_I2C3) || defined(CONFIG_STM32L4_I2C4)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_I2C_POLLED may be set so that I2C interrupts will not be used.
 *  Instead, CPU-intensive polling will be used.
 */

/* Interrupt wait timeout in seconds and milliseconds */

#if !defined(CONFIG_STM32L4_I2CTIMEOSEC) && !defined(CONFIG_STM32L4_I2CTIMEOMS)
#  define CONFIG_STM32L4_I2CTIMEOSEC 0
#  define CONFIG_STM32L4_I2CTIMEOMS  500   /* Default is 500 milliseconds */
#  warning "Using Default 500 Ms Timeout"
#elif !defined(CONFIG_STM32L4_I2CTIMEOSEC)
#  define CONFIG_STM32L4_I2CTIMEOSEC 0     /* User provided milliseconds */
#elif !defined(CONFIG_STM32L4_I2CTIMEOMS)
#  define CONFIG_STM32L4_I2CTIMEOMS  0     /* User provided seconds */
#endif

/* Interrupt wait time timeout in system timer ticks */

#ifndef CONFIG_STM32L4_I2CTIMEOTICKS
#  define CONFIG_STM32L4_I2CTIMEOTICKS \
    (SEC2TICK(CONFIG_STM32L4_I2CTIMEOSEC) + MSEC2TICK(CONFIG_STM32L4_I2CTIMEOMS))
#endif

#ifndef CONFIG_STM32L4_I2C_DYNTIMEO_STARTSTOP
#  define CONFIG_STM32L4_I2C_DYNTIMEO_STARTSTOP TICK2USEC(CONFIG_STM32L4_I2CTIMEOTICKS)
#endif

/* Macros to convert a I2C pin to a GPIO output */

#define I2C_OUTPUT (GPIO_OUTPUT | GPIO_FLOAT | GPIO_OPENDRAIN | \
                    GPIO_SPEED_50MHz | GPIO_OUTPUT_SET)

#define MKI2C_OUTPUT(p) (((p) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | I2C_OUTPUT)

#define I2C_CR1_TXRX (I2C_CR1_RXIE | I2C_CR1_TXIE)
#define I2C_CR1_ALLINTS (I2C_CR1_TXRX | I2C_CR1_TCIE | I2C_CR1_ERRIE)

/* I2C event tracing
 *
 * To enable tracing statements which show the details of the state machine
 * enable the following configuration variable:
 *
 * CONFIG_I2C_TRACE
 *
 * Note: This facility uses syslog, which sends output to the console by
 * default.  No other debug configuration variables are required.
 */

#ifndef CONFIG_I2C_TRACE
#  define stm32l4_i2c_tracereset(p)
#  define stm32l4_i2c_tracenew(p,s)
#  define stm32l4_i2c_traceevent(p,e,a)
#  define stm32l4_i2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#  define CONFIG_I2C_NTRACE 32
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Interrupt state */

enum stm32l4_intstate_e
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_WAITING,       /* Waiting for completion of interrupt activity */
  INTSTATE_DONE,          /* Interrupt activity complete */
};

/* Trace events */

enum stm32l4_trace_e
{
  I2CEVENT_NONE = 0,
  I2CEVENT_STATE_ERROR,
  I2CEVENT_ISR_SHUTDOWN,
  I2CEVENT_ISR_CALL,
  I2CEVENT_ISR_EMPTY_CALL,
  I2CEVENT_MSG_HANDLING,
  I2CEVENT_POLL_NOT_READY,
  I2CEVENT_EMPTY_MSG,
  I2CEVENT_START,
  I2CEVENT_ADDRESS_ACKED,
  I2CEVENT_ADDRESS_NACKED,
  I2CEVENT_NACK,
  I2CEVENT_READ,
  I2CEVENT_READ_ERROR,
  I2CEVENT_WRITE_TO_DR,
  I2CEVENT_WRITE_STOP,
  I2CEVENT_WRITE_RESTART,
  I2CEVENT_WRITE_NO_RESTART,
  I2CEVENT_WRITE_ERROR,
  I2CEVENT_WRITE_FLAG_ERROR,
  I2CEVENT_TC_RESTART,
  I2CEVENT_TC_NO_RESTART
};

/* Trace data */

struct stm32l4_trace_s
{
  uint32_t status;               /* I2C 32-bit SR2|SR1 status */
  uint32_t count;                /* Interrupt count when status change */
  enum stm32l4_intstate_e event; /* Last event that occurred with this status */
  uint32_t parm;                 /* Parameter associated with the event */
  clock_t time;                  /* First of event or first status */
};

/* I2C Device hardware configuration */

struct stm32l4_i2c_config_s
{
  uint32_t base;              /* I2C base address */
  uint32_t clk_bit;           /* Clock enable bit */
  uint32_t reset_bit;         /* Reset bit */
  uint32_t scl_pin;           /* GPIO configuration for SCL as SCL */
  uint32_t sda_pin;           /* GPIO configuration for SDA as SDA */
#ifndef CONFIG_I2C_POLLED
  uint32_t ev_irq;            /* Event IRQ */
  uint32_t er_irq;            /* Error IRQ */
#endif
};

/* I2C Device Private Data */

struct stm32l4_i2c_priv_s
{
  /* Port configuration */

  const struct stm32l4_i2c_config_s *config;

  int refs;                    /* Reference count */
  mutex_t lock;                /* Mutual exclusion mutex */
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;               /* Interrupt wait semaphore */
#endif
  volatile uint8_t intstate;   /* Interrupt handshake (see enum stm32l4_intstate_e) */

  uint8_t msgc;                /* Message count */
  struct i2c_msg_s *msgv;      /* Message list */
  uint8_t *ptr;                /* Current message buffer */
  uint32_t frequency;          /* Current I2C frequency */
  int dcnt;                    /* Current message bytes remaining to transfer */
  uint16_t flags;              /* Current message flags */
  bool astart;                 /* START sent */

  /* I2C trace support */

#ifdef CONFIG_I2C_TRACE
  int tndx;                    /* Trace array index */
  clock_t start_time;          /* Time when the trace was started */

  /* The actual trace data */

  struct stm32l4_trace_s trace[CONFIG_I2C_NTRACE];
#endif

  uint32_t status;             /* End of transfer SR2|SR1 status */

#ifdef CONFIG_PM
  struct pm_callback_s pm_cb;  /* PM callbacks */
#endif
};

/* I2C Device, Instance */

struct stm32l4_i2c_inst_s
{
  const struct i2c_ops_s    *ops;  /* Standard I2C operations */
  struct stm32l4_i2c_priv_s *priv; /* Common driver private data structure */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline
uint16_t stm32l4_i2c_getreg(struct stm32l4_i2c_priv_s *priv,
                            uint8_t offset);
static inline
void stm32l4_i2c_putreg(struct stm32l4_i2c_priv_s *priv,
                        uint8_t offset, uint16_t value);
static inline
void stm32l4_i2c_putreg32(struct stm32l4_i2c_priv_s *priv,
                          uint8_t offset, uint32_t value);
static inline
void stm32l4_i2c_modifyreg32(struct stm32l4_i2c_priv_s *priv,
                             uint8_t offset, uint32_t clearbits,
                             uint32_t setbits);
#ifdef CONFIG_STM32L4_I2C_DYNTIMEO
static uint32_t stm32l4_i2c_toticks(int msgc, struct i2c_msg_s *msgs);
#endif /* CONFIG_STM32L4_I2C_DYNTIMEO */
static inline
int  stm32l4_i2c_sem_waitdone(struct stm32l4_i2c_priv_s *priv);
static inline
void stm32l4_i2c_sem_waitstop(struct stm32l4_i2c_priv_s *priv);
#ifdef CONFIG_I2C_TRACE
static void stm32l4_i2c_tracereset(struct stm32l4_i2c_priv_s *priv);
static void stm32l4_i2c_tracenew(struct stm32l4_i2c_priv_s *priv,
                                 uint32_t status);
static void
stm32l4_i2c_traceevent(struct stm32l4_i2c_priv_s *priv,
                       enum stm32l4_trace_e event, uint32_t parm);
static void stm32l4_i2c_tracedump(struct stm32l4_i2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */
static void stm32l4_i2c_setclock(struct stm32l4_i2c_priv_s *priv,
                                 uint32_t frequency);
static inline
void stm32l4_i2c_sendstart(struct stm32l4_i2c_priv_s *priv);
static inline void stm32l4_i2c_sendstop(struct stm32l4_i2c_priv_s *priv);
static inline
uint32_t stm32l4_i2c_getstatus(struct stm32l4_i2c_priv_s *priv);
static int stm32l4_i2c_isr_process(struct stm32l4_i2c_priv_s *priv);
#ifndef CONFIG_I2C_POLLED
static int stm32l4_i2c_isr(int irq, void *context, void *arg);
#endif
static int stm32l4_i2c_init(struct stm32l4_i2c_priv_s *priv);
static int stm32l4_i2c_deinit(struct stm32l4_i2c_priv_s *priv);

static int stm32l4_i2c_process(struct i2c_master_s *dev,
                               struct i2c_msg_s *msgs, int count);
static int stm32l4_i2c_transfer(struct i2c_master_s *dev,
                                struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int stm32l4_i2c_reset(struct i2c_master_s *dev);
#endif
#ifdef CONFIG_PM
static int stm32l4_i2c_pm_prepare(struct pm_callback_s *cb, int domain,
                                  enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_STM32L4_I2C1
static const struct stm32l4_i2c_config_s stm32l4_i2c1_config =
{
  .base       = STM32L4_I2C1_BASE,
  .clk_bit    = RCC_APB1ENR1_I2C1EN,
  .reset_bit  = RCC_APB1RSTR1_I2C1RST,
  .scl_pin    = GPIO_I2C1_SCL,
  .sda_pin    = GPIO_I2C1_SDA,
#ifndef CONFIG_I2C_POLLED
  .ev_irq     = STM32L4_IRQ_I2C1EV,
  .er_irq     = STM32L4_IRQ_I2C1ER
#endif
};

static struct stm32l4_i2c_priv_s stm32l4_i2c1_priv =
{
  .config     = &stm32l4_i2c1_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = SEM_INITIALIZER(0),
#endif
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .frequency  = 0,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0,
#ifdef CONFIG_PM
  .pm_cb.prepare = stm32l4_i2c_pm_prepare,
#endif
};
#endif

#ifdef CONFIG_STM32L4_I2C2
static const struct stm32l4_i2c_config_s stm32l4_i2c2_config =
{
  .base       = STM32L4_I2C2_BASE,
  .clk_bit    = RCC_APB1ENR1_I2C2EN,
  .reset_bit  = RCC_APB1RSTR1_I2C2RST,
  .scl_pin    = GPIO_I2C2_SCL,
  .sda_pin    = GPIO_I2C2_SDA,
#ifndef CONFIG_I2C_POLLED
  .ev_irq     = STM32L4_IRQ_I2C2EV,
  .er_irq     = STM32L4_IRQ_I2C2ER
#endif
};

static struct stm32l4_i2c_priv_s stm32l4_i2c2_priv =
{
  .config     = &stm32l4_i2c2_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = SEM_INITIALIZER(0),
#endif
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .frequency  = 0,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0,
#ifdef CONFIG_PM
  .pm_cb.prepare = stm32l4_i2c_pm_prepare,
#endif
};
#endif

#ifdef CONFIG_STM32L4_I2C3
static const struct stm32l4_i2c_config_s stm32l4_i2c3_config =
{
  .base       = STM32L4_I2C3_BASE,
  .clk_bit    = RCC_APB1ENR1_I2C3EN,
  .reset_bit  = RCC_APB1RSTR1_I2C3RST,
  .scl_pin    = GPIO_I2C3_SCL,
  .sda_pin    = GPIO_I2C3_SDA,
#ifndef CONFIG_I2C_POLLED
  .ev_irq     = STM32L4_IRQ_I2C3EV,
  .er_irq     = STM32L4_IRQ_I2C3ER
#endif
};

static struct stm32l4_i2c_priv_s stm32l4_i2c3_priv =
{
  .config     = &stm32l4_i2c3_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = SEM_INITIALIZER(0),
#endif
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .frequency  = 0,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0,
#ifdef CONFIG_PM
  .pm_cb.prepare = stm32l4_i2c_pm_prepare,
#endif
};
#endif

#ifdef CONFIG_STM32L4_I2C4
static const struct stm32l4_i2c_config_s stm32l4_i2c4_config =
{
  .base       = STM32L4_I2C4_BASE,
  .clk_bit    = RCC_APB1ENR2_I2C4EN,
  .reset_bit  = RCC_APB1RSTR2_I2C4RST,
  .scl_pin    = GPIO_I2C4_SCL,
  .sda_pin    = GPIO_I2C4_SDA,
#ifndef CONFIG_I2C_POLLED
  .ev_irq     = STM32L4_IRQ_I2C4EV,
  .er_irq     = STM32L4_IRQ_I2C4ER
#endif
};

static struct stm32l4_i2c_priv_s stm32l4_i2c4_priv =
{
  .config     = &stm32l4_i2c4_config,
  .refs       = 0,
  .lock       = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr    = SEM_INITIALIZER(0),
#endif
  .intstate   = INTSTATE_IDLE,
  .msgc       = 0,
  .msgv       = NULL,
  .ptr        = NULL,
  .frequency  = 0,
  .dcnt       = 0,
  .flags      = 0,
  .status     = 0,
#ifdef CONFIG_PM
  .pm_cb.prepare = stm32l4_i2c_pm_prepare,
#endif
};
#endif

/* Device Structures, Instantiation */

static const struct i2c_ops_s stm32l4_i2c_ops =
{
  .transfer = stm32l4_i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = stm32l4_i2c_reset
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_i2c_getreg
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ****************************************************************************/

static inline
uint16_t stm32l4_i2c_getreg(struct stm32l4_i2c_priv_s *priv,
                            uint8_t offset)
{
  return getreg16(priv->config->base + offset);
}

/****************************************************************************
 * Name: stm32l4_i2c_getreg32
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline
uint32_t stm32l4_i2c_getreg32(struct stm32l4_i2c_priv_s *priv,
                              uint8_t offset)
{
  return getreg32(priv->config->base + offset);
}

/****************************************************************************
 * Name: stm32l4_i2c_putreg
 *
 * Description:
 *  Put a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void stm32l4_i2c_putreg(struct stm32l4_i2c_priv_s *priv,
                                      uint8_t offset, uint16_t value)
{
  putreg16(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: stm32l4_i2c_putreg32
 *
 * Description:
 *  Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void stm32l4_i2c_putreg32(struct stm32l4_i2c_priv_s *priv,
                                        uint8_t offset, uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: stm32l4_i2c_modifyreg32
 *
 * Description:
 *   Modify a 32-bit register value by offset
 *
 ****************************************************************************/

static inline
void stm32l4_i2c_modifyreg32(struct stm32l4_i2c_priv_s *priv,
                             uint8_t offset, uint32_t clearbits,
                             uint32_t setbits)
{
  modifyreg32(priv->config->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: stm32l4_i2c_toticks
 *
 * Description:
 *   Return a micro-second delay based on the number of bytes left to be
 *   processed.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32L4_I2C_DYNTIMEO
static uint32_t stm32l4_i2c_toticks(int msgc, struct i2c_msg_s *msgs)
{
  size_t bytecount = 0;
  int i;

  /* Count the number of bytes left to process */

  for (i = 0; i < msgc; i++)
    {
      bytecount += msgs[i].length;
    }

  /* Then return a number of microseconds based on a user provided scaling
   * factor.
   */

  return USEC2TICK(CONFIG_STM32L4_I2C_DYNTIMEO_USECPERBYTE * bytecount);
}
#endif

/****************************************************************************
 * Name: stm32l4_i2c_enableinterrupts
 *
 * Description:
 *   Enable I2C interrupts
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline
void stm32l4_i2c_enableinterrupts(struct stm32l4_i2c_priv_s *priv)
{
    stm32l4_i2c_modifyreg32(priv, STM32L4_I2C_CR1_OFFSET, 0,
                            (I2C_CR1_TXRX | I2C_CR1_NACKIE));
}
#endif

/****************************************************************************
 * Name: stm32l4_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 * There are two versions of this function.  The first is included when using
 * interrupts while the second is used if polling (CONFIG_I2C_POLLED=y).
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline
int stm32l4_i2c_sem_waitdone(struct stm32l4_i2c_priv_s *priv)
{
  irqstate_t flags;
  int ret;

  flags = enter_critical_section();

  /* Enable I2C interrupts */

  /* The TXIE and RXIE interrupts are enabled initially in
   * stm32l4_i2c_process. The remainder of the interrupts, including
   * error-related, are enabled here.
   */

  stm32l4_i2c_modifyreg32(priv, STM32L4_I2C_CR1_OFFSET, 0,
                        (I2C_CR1_ALLINTS & ~I2C_CR1_TXRX));

  /* Signal the interrupt handler that we are waiting */

  priv->intstate = INTSTATE_WAITING;
  do
    {
      /* Wait until either the transfer is complete or the timeout expires */

#ifdef CONFIG_STM32L4_I2C_DYNTIMEO
      ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                       stm32l4_i2c_toticks(priv->msgc, priv->msgv));
#else
      ret = nxsem_tickwait_uninterruptible(&priv->sem_isr,
                                           CONFIG_STM32L4_I2CTIMEOTICKS);
#endif
      if (ret < 0)
        {
          /* Break out of the loop on irrecoverable errors.  This would
           * include timeouts and mystery errors reported by
           * nxsem_tickwait_uninterruptible.
           */

          break;
        }
    }

  /* Loop until the interrupt level transfer is complete. */

  while (priv->intstate != INTSTATE_DONE);

  /* Set the interrupt state back to IDLE */

  priv->intstate = INTSTATE_IDLE;

  /* Disable I2C interrupts */

  stm32l4_i2c_modifyreg32(priv, STM32L4_I2C_CR1_OFFSET, I2C_CR1_ALLINTS, 0);

  leave_critical_section(flags);
  return ret;
}
#else
static inline
int stm32l4_i2c_sem_waitdone(struct stm32l4_i2c_priv_s *priv)
{
  clock_t timeout;
  clock_t start;
  clock_t elapsed;
  int ret;

  /* Get the timeout value */

#ifdef CONFIG_STM32L4_I2C_DYNTIMEO
  timeout = stm32l4_i2c_toticks(priv->msgc, priv->msgv);
#else
  timeout = CONFIG_STM32L4_I2CTIMEOTICKS;
#endif

  /* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
   * are currently disabled but will be temporarily re-enabled below when
   * nxsem_tickwait_uninterruptible() sleeps.
   */

  priv->intstate = INTSTATE_WAITING;
  start = clock_systime_ticks();

  do
    {
      /* Calculate the elapsed time */

      elapsed = clock_systime_ticks() - start;

      /* Poll by simply calling the timer interrupt handler until it
       * reports that it is done.
       */

      stm32l4_i2c_isr_process(priv);
    }

  /* Loop until the transfer is complete. */

  while (priv->intstate != INTSTATE_DONE && elapsed < timeout);

  i2cinfo("intstate: %d elapsed: %ld threshold: %ld status: 0x%08x\n",
          priv->intstate, (long)elapsed, (long)timeout, priv->status);

  /* Set the interrupt state back to IDLE */

  ret = priv->intstate == INTSTATE_DONE ? OK : -ETIMEDOUT;
  priv->intstate = INTSTATE_IDLE;
  return ret;
}
#endif

/****************************************************************************
 * Name: stm32l4_i2c_set_7bit_address
 *
 * Description:
 *
 ****************************************************************************/

static inline void
stm32l4_i2c_set_7bit_address(struct stm32l4_i2c_priv_s *priv)
{
  stm32l4_i2c_modifyreg32(priv, STM32L4_I2C_CR2_OFFSET, I2C_CR2_SADD7_MASK,
                        ((priv->msgv->addr & 0x7f) << I2C_CR2_SADD7_SHIFT));
}

/****************************************************************************
 * Name: stm32l4_i2c_set_bytes_to_transfer
 *
 * Description:
 *
 ****************************************************************************/

static inline void
stm32l4_i2c_set_bytes_to_transfer(struct stm32l4_i2c_priv_s *priv,
                                  uint8_t n_bytes)
{
  stm32l4_i2c_modifyreg32(priv, STM32L4_I2C_CR2_OFFSET, I2C_CR2_NBYTES_MASK,
                          (n_bytes << I2C_CR2_NBYTES_SHIFT));
}

/****************************************************************************
 * Name: stm32l4_i2c_set_write_transfer_dir
 *
 * Description:
 *
 ****************************************************************************/

static inline void
stm32l4_i2c_set_write_transfer_dir(struct stm32l4_i2c_priv_s *priv)
{
  stm32l4_i2c_modifyreg32(priv, STM32L4_I2C_CR2_OFFSET, I2C_CR2_RD_WRN, 0);
}

/****************************************************************************
 * Name: stm32l4_i2c_set_read_transfer_dir
 *
 * Description:
 *
 ****************************************************************************/

static inline void
stm32l4_i2c_set_read_transfer_dir(struct stm32l4_i2c_priv_s *priv)
{
  stm32l4_i2c_modifyreg32(priv, STM32L4_I2C_CR2_OFFSET,
                          0, I2C_CR2_RD_WRN);
}

/****************************************************************************
 * Name: stm32l4_i2c_enable_reload
 *
 * Description:
 *
 ****************************************************************************/

static inline void
stm32l4_i2c_enable_reload(struct stm32l4_i2c_priv_s *priv)
{
  stm32l4_i2c_modifyreg32(priv, STM32L4_I2C_CR2_OFFSET,
                          0, I2C_CR2_RELOAD);
}

/****************************************************************************
 * Name: stm32l4_i2c_disable_reload
 *
 * Description:
 *
 ****************************************************************************/

static inline void
stm32l4_i2c_disable_reload(struct stm32l4_i2c_priv_s *priv)
{
  stm32l4_i2c_modifyreg32(priv, STM32L4_I2C_CR2_OFFSET,
                          I2C_CR2_RELOAD, 0);
}

/****************************************************************************
 * Name: stm32l4_i2c_sem_waitstop
 *
 * Description:
 *   Wait for a STOP to complete
 *
 ****************************************************************************/

static inline
void stm32l4_i2c_sem_waitstop(struct stm32l4_i2c_priv_s *priv)
{
  clock_t start;
  clock_t elapsed;
  clock_t timeout;
  uint32_t cr;
  uint32_t sr;

  /* Select a timeout */

#ifdef CONFIG_STM32L4_I2C_DYNTIMEO
  timeout = USEC2TICK(CONFIG_STM32L4_I2C_DYNTIMEO_STARTSTOP);
#else
  timeout = CONFIG_STM32L4_I2CTIMEOTICKS;
#endif

  /* Wait as stop might still be in progress */

  start = clock_systime_ticks();
  do
    {
      /* Calculate the elapsed time */

      elapsed = clock_systime_ticks() - start;

      /* Check for STOP condition */

      cr = stm32l4_i2c_getreg32(priv, STM32L4_I2C_CR2_OFFSET);
      if ((cr & I2C_CR2_STOP) == 0)
        {
          return;
        }

      /* Check for timeout error */

      sr = stm32l4_i2c_getreg(priv, STM32L4_I2C_ISR_OFFSET);
      if ((sr & I2C_INT_TIMEOUT) != 0)
        {
          return;
        }
    }

  /* Loop until the stop is complete or a timeout occurs. */

  while (elapsed < timeout);

  /* If we get here then a timeout occurred with the STOP condition
   * still pending.
   */

  i2cinfo("Timeout with CR: %04" PRIx32 " SR: %04" PRIx32 "\n", cr, sr);
}

/****************************************************************************
 * Name: stm32l4_i2c_trace*
 *
 * Description:
 *   I2C trace instrumentation
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void stm32l4_i2c_traceclear(struct stm32l4_i2c_priv_s *priv)
{
  struct stm32l4_trace_s *trace = &priv->trace[priv->tndx];

  trace->status = 0;              /* I2C 32-bit status */
  trace->count  = 0;              /* Interrupt count when status change */
  trace->event  = I2CEVENT_NONE;  /* Last event that occurred with this status */
  trace->parm   = 0;              /* Parameter associated with the event */
  trace->time   = 0;              /* Time of first status or event */
}

static void stm32l4_i2c_tracereset(struct stm32l4_i2c_priv_s *priv)
{
  /* Reset the trace info for a new data collection */

  priv->tndx       = 0;
  priv->start_time = clock_systime_ticks();
  stm32l4_i2c_traceclear(priv);
}

static void stm32l4_i2c_tracenew(struct stm32l4_i2c_priv_s *priv,
                                 uint32_t status)
{
  struct stm32l4_trace_s *trace = &priv->trace[priv->tndx];

  /* Is the current entry uninitialized?   Has the status changed? */

  if (trace->count == 0 || status != trace->status)
    {
      /* Yes.. Was it the status changed?  */

      if (trace->count != 0)
        {
          /* Yes.. bump up the trace index
           * (unless we are out of trace entries)
           */

          if (priv->tndx >= (CONFIG_I2C_NTRACE - 1))
            {
              i2cerr("ERROR: Trace table overflow\n");
              return;
            }

          priv->tndx++;
          trace = &priv->trace[priv->tndx];
        }

      /* Initialize the new trace entry */

      stm32l4_i2c_traceclear(priv);
      trace->status = status;
      trace->count  = 1;
      trace->time   = clock_systime_ticks();
    }
  else
    {
      /* Just increment the count of times that we have seen this status */

      trace->count++;
    }
}

static void stm32l4_i2c_traceevent(struct stm32l4_i2c_priv_s *priv,
                                   enum stm32l4_trace_e event, uint32_t parm)
{
  struct stm32l4_trace_s *trace;

  if (event != I2CEVENT_NONE)
    {
      trace = &priv->trace[priv->tndx];

      /* Initialize the new trace entry */

      trace->event  = event;
      trace->parm   = parm;

      /* Bump up the trace index (unless we are out of trace entries) */

      if (priv->tndx >= (CONFIG_I2C_NTRACE - 1))
        {
          i2cerr("ERROR: Trace table overflow\n");
          return;
        }

      priv->tndx++;
      stm32l4_i2c_traceclear(priv);
    }
}

static void stm32l4_i2c_tracedump(struct stm32l4_i2c_priv_s *priv)
{
  struct stm32l4_trace_s *trace;
  int i;

  syslog(LOG_DEBUG, "Elapsed time: %d\n",
         (int)(clock_systime_ticks() - priv->start_time));

  for (i = 0; i < priv->tndx; i++)
    {
      trace = &priv->trace[i];
      syslog(LOG_DEBUG,
             "%2d. STATUS: %08x COUNT: %3d EVENT: %2d PARM: %08x TIME: %d\n",
             i + 1, trace->status, trace->count,  trace->event, trace->parm,
             (int)(trace->time - priv->start_time));
    }
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: stm32l4_i2c_setclock
 *
 * Description:
 *
 *   Sets the I2C bus clock frequency by configuring the I2C_TIMINGR
 *   register.
 *
 *   This function supports bus clock frequencies of:
 *
 *      1000Khz (Fast Mode+)
 *      400Khz  (Fast Mode)
 *      100Khz  (Standard Mode)
 *      10Khz   (Standard Mode)
 *
 *   Attempts to set a different frequency will quietly provision the default
 *   of 10Khz.
 *
 *   The only differences between the various modes of operation (std, fast,
 *   fast+) are the bus clock speed and setup/hold times.  Setup/hold times
 *   are specified as a MINIMUM time for the given mode, and naturally std
 *   mode has the longest minimum times.  As a result, by provisioning
 *   setup/hold times for std mode they are also compatible with fast/fast+,
 *   though some performance degradation occurs in fast/fast+ as a result of
 *   the times being somewhat longer than strictly required.  The values
 *   remain as they are because reliability is favored over performance.
 *
 * Clock Selection:
 *
 *   The I2C peripheral clock can be provided by either PCLK1, SYSCLK or the
 *   HSI.
 *
 *    PCLK1 >------|\   I2CCLK
 *   SYSCLK >------| |--------->
 *      HSI >------|/
 *
 *   PCLK is the default and is expected to be 80Mhz.
 *
 *   SYSCLK can, in turn, be derived from the HSI, HSE, PPLCLK.
 *
 *      HSI >------|\
 *                 | |  SYSCLK
 *      PLL >------| |--------->
 *                 | |
 *      HSE >------|/
 *
 *
 * References:
 *
 *  App Note AN4235 and the associated software STSW-STM32126.
 *
 ****************************************************************************/

static void stm32l4_i2c_setclock(struct stm32l4_i2c_priv_s *priv,
                                 uint32_t frequency)
{
  int i2cclk_mhz;
  uint32_t pe;
  uint8_t presc = 0;
  uint8_t scl_delay = 0;
  uint8_t sda_delay = 0;
  uint8_t scl_h_period = 0;
  uint8_t scl_l_period = 0;

  if (frequency != priv->frequency)
    {
      /* I2C peripheral must be disabled to update clocking configuration */

      pe = (stm32l4_i2c_getreg32(priv,
                                 STM32L4_I2C_CR1_OFFSET) & I2C_CR1_PE);
      if (pe)
        {
          stm32l4_i2c_modifyreg32(priv, STM32L4_I2C_CR1_OFFSET,
                                  I2C_CR1_PE, 0);
        }

#if defined(STM32L4_I2C_USE_HSI16) || (STM32L4_PCLK1_FREQUENCY == 16000000)
      i2cclk_mhz = 16;
#elif STM32L4_PCLK1_FREQUENCY == 48000000
      i2cclk_mhz = 48;
#elif STM32L4_PCLK1_FREQUENCY == 80000000
      i2cclk_mhz = 80;
#elif STM32L4_PCLK1_FREQUENCY == 120000000
      i2cclk_mhz = 120;
#else
#   warning STM32_I2C_INIT: Peripheral clock is PCLK and the speed/timing calculations need to be redone.
#endif

      if (i2cclk_mhz == 80)
        {
          /* Default timing calculations from original STM32L4 driver: */

          /*  The Speed and timing calculation are based on the following
           *  fI2CCLK = PCLK and is 80 MHz
           *  Analog filter is on,
           *  Digital filter off
           *  Rise Time is 120 ns and fall is 10 ns
           *  Mode is FastMode
           */

          if (frequency == 100000)
            {
              /* 100 KHz values from I2C timing tool with clock 80 MHz */

              presc        = 0x01;    /* PRESC - (+1) prescale I2CCLK */
              scl_l_period = 0xe7;    /* SCLL - SCL low period in master mode */
              scl_h_period = 0x9b;    /* SCLH - SCL high period in master mode */
              sda_delay    = 0x00;    /* SDADEL - (+1) data hold time after SCL falling edge */
              scl_delay    = 0x0d;    /* SCLDEL - (+1) data setup time from SDA edge to SCL rising edge */
            }
          else if (frequency == 400000)
            {
              /* 400 KHz values from I2C timing tool for clock of 80 MHz */

              presc        = 0x01;    /* PRESC - (+1) prescale I2CCLK */
              scl_l_period = 0x43;    /* SCLL - SCL low period in master mode */
              scl_h_period = 0x13;    /* SCLH - SCL high period in master mode */
              sda_delay    = 0x00;    /* SDADEL - (+1) data hold time after SCL falling edge */
              scl_delay    = 0x07;    /* SCLDEL - (+1) data setup time from SDA edge to SCL rising edge */
            }
          else if (frequency == 1000000)
            {
              /* 1000 KHz values from I2C timing tool for clock of 80 MHz */

              presc        = 0x01;    /* PRESC - (+1) prescale I2CCLK */
              scl_l_period = 0x14;    /* SCLL - SCL low period in master mode */
              scl_h_period = 0x13;    /* SCLH - SCL high period in master mode */
              sda_delay    = 0x00;    /* SDADEL - (+1) data hold time after SCL falling edge */
              scl_delay    = 0x05;    /* SCLDEL - (+1) data setup time from SDA edge to SCL rising edge */
            }
          else
            {
              /* 10 KHz values from I2C timing tool with clock 80 MHz */

              presc        = 0x0b;    /* PRESC - (+1) prescale I2CCLK */
              scl_l_period = 0xff;    /* SCLL - SCL low period in master mode */
              scl_h_period = 0xba;    /* SCLH - SCL high period in master mode */
              sda_delay    = 0x00;    /* SDADEL - (+1) data hold time after SCL falling edge */
              scl_delay    = 0x01;    /* SCLDEL - (+1) data setup time from SDA edge to SCL rising edge */
            }
        }
      else if (i2cclk_mhz == 120)
        {
          /*  The Speed and timing calculation are based on the following
           *  fI2CCLK = PCLK and is 120 MHz
           *  Analog filter is on,
           *  Digital filter off
           *  Rise Time is 120 ns and fall is 25 ns
           *  Mode is FastMode
           */

          if (frequency == 100000)
            {
              /* 100 KHz values from I2C timing tool with clock 120 MHz */

              presc        = 2;
              scl_delay    = 14;
              sda_delay    = 0;
              scl_h_period = 157;
              scl_l_period = 230;
            }
          else if (frequency == 400000)
            {
              /* 400 KHz values from I2C timing tool for clock of 120 MHz */

              presc        = 2;
              scl_delay    = 8;
              sda_delay    = 0;
              scl_h_period = 21;
              scl_l_period = 66;
            }
          else if (frequency == 1000000)
            {
              /* 1000 KHz values from I2C timing tool for clock of 120 MHz */

              presc        = 2;
              scl_delay    = 6;
              sda_delay    = 0;
              scl_h_period = 7;
              scl_l_period = 20;
            }
          else
            {
              /* 10 KHz values not supported */

              DEBUGPANIC();
            }
        }
      else if (i2cclk_mhz == 16)
        {
          /*  The Speed and timing calculation are based on the following
           *  fI2CCLK = HSI and is 16Mhz
           *  Analog filter is on,
           *  Digital filter off
           *  Rise Time is 120 ns and fall is 10ns
           *  Mode is FastMode
           */

          if (frequency == 100000)
            {
              presc        = 0;
              scl_delay    = 5;
              sda_delay    = 0;
              scl_h_period = 61;
              scl_l_period = 89;
            }
          else if (frequency == 400000)
            {
              presc        = 0;
              scl_delay    = 3;
              sda_delay    = 0;
              scl_h_period = 6;
              scl_l_period = 24;
            }
          else if (frequency == 1000000)
            {
              presc        = 0;
              scl_delay    = 2;
              sda_delay    = 0;
              scl_h_period = 1;
              scl_l_period = 5;
            }
          else
            {
              presc        = 7;
              scl_delay    = 0;
              sda_delay    = 0;
              scl_h_period = 35;
              scl_l_period = 162;
            }
        }
      else if (i2cclk_mhz == 48)
        {
          if (frequency == 100000)
            {
              presc        = 2;
              scl_delay    = 10;
              sda_delay    = 0;
              scl_h_period = 62;
              scl_l_period = 85;
            }
          else if (frequency == 400000)
            {
              presc        = 1;
              scl_delay    = 8;
              sda_delay    = 0;
              scl_h_period = 12;
              scl_l_period = 33;
            }
          else if (frequency == 1000000)
            {
              presc        = 0;
              scl_delay    = 5;
              sda_delay    = 0;
              scl_h_period = 8;
              scl_l_period = 22;
            }
        }
      else
        {
          DEBUGPANIC();
        }

      uint32_t timingr =
        (presc << I2C_TIMINGR_PRESC_SHIFT) |
        (scl_delay << I2C_TIMINGR_SCLDEL_SHIFT) |
        (sda_delay << I2C_TIMINGR_SDADEL_SHIFT) |
        (scl_h_period << I2C_TIMINGR_SCLH_SHIFT) |
        (scl_l_period << I2C_TIMINGR_SCLL_SHIFT);

      stm32l4_i2c_putreg32(priv, STM32L4_I2C_TIMINGR_OFFSET, timingr);

      if (pe)
        {
          stm32l4_i2c_modifyreg32(priv, STM32L4_I2C_CR1_OFFSET,
                                  0, I2C_CR1_PE);
        }

      priv->frequency = frequency;
    }
}

/****************************************************************************
 * Name: stm32l4_i2c_sendstart
 *
 * Description:
 *   Send the START condition / force Master mode
 *
 *  A START condition in I2C consists of a single byte that contains both the
 *  7 bit slave address and a read/write bit (0 = WRITE, 1 = READ).  If the
 *  address is recognized by one of the slave devices that slave device will
 *  ACK the byte so that data transfers can begin.
 *
 *  A RESTART (or repeated START per the I2CSPEC) is simply a START condition
 *  issued in the middle of a transfer (i.e. after the initial START and
 *  before a STOP).  A RESTART sends a new address byte and R/W bit to the
 *  bus. A RESTART is optional in most cases but mandatory in the event the
 *  transfer direction is changed.
 *
 *  Most of the time reading data from an I2C slave requires a WRITE of the
 *  subaddress followed by a READ (and hence a RESTART in between).  Writing
 *  to an I2C slave typically requires only WRITE operations and hence no
 *  RESTARTs.
 *
 *  This function is therefore called both at the beginning of a transfer
 *  (START) and at appropriate times during a transfer (RESTART).
 *
 ****************************************************************************/

static inline
void stm32l4_i2c_sendstart(struct stm32l4_i2c_priv_s *priv)
{
  bool next_norestart = false;

  /* Set the private "current message" data used in protocol processing.
   *
   * ptr:   A pointer to the start of the current message buffer.  This is
   *        advanced after each byte in the current message is transferred.
   *
   * dcnt:  A running counter of the bytes in the current message waiting to
   *        be transferred.  This is decremented each time a byte is
   *        transferred. The hardware normally accepts a maximum of 255 bytes
   *        per transfer but can support more via the RELOAD mechanism.
   *        If dcnt initially exceeds 255, the RELOAD mechanism will be
   *        enabled automatically.
   *
   * flags: Used to characterize handling of the current message.
   *
   *  The default flags value is 0 which specifies:
   *
   *   - A transfer direction of WRITE (R/W bit = 0)
   *   - RESTARTs between all messages
   *
   *  The following flags can be used to override this behavior as follows:
   *
   *   - I2C_M_READ: Sets the transfer direction to READ (R/W bit = 1)
   *   - I2C_M_NOSTART: Prevents a RESTART from being issued prior to the
   *      transfer of the message (where allowed by the protocol).
   *
   */

  priv->ptr   = priv->msgv->buffer;
  priv->dcnt  = priv->msgv->length;
  priv->flags = priv->msgv->flags;

  if ((priv->flags & I2C_M_NOSTART) != 0)
    {
      /* Flag the first byte as an address byte */

      priv->astart = true;
    }

  /* Enabling RELOAD allows the transfer of:
   *
   *  - individual messages with a payload exceeding 255 bytes
   *  - multiple messages back to back without a RESTART in between
   *
   * so we enable it if either of those conditions exist and disable
   * it otherwise.
   */

  /* Check if there are multiple messages and the next is a continuation */

  if (priv->msgc > 1)
    {
      next_norestart = (((priv->msgv + 1)->flags & I2C_M_NOSTART) != 0);
    }

  if (next_norestart || priv->dcnt > 255)
    {
      i2cinfo("RELOAD enabled: dcnt = %i msgc = %i\n",
          priv->dcnt, priv->msgc);
      stm32l4_i2c_enable_reload(priv);
    }
  else
    {
      i2cinfo("RELOAD disable: dcnt = %i msgc = %i\n",
          priv->dcnt, priv->msgc);
      stm32l4_i2c_disable_reload(priv);
    }

  /* Set the number of bytes to transfer (I2C_CR2->NBYTES) to the number of
   * bytes in the current message or 255, whichever is lower so as to not
   * exceed the hardware maximum allowed.
   */

  if (priv->dcnt > 255)
    {
      stm32l4_i2c_set_bytes_to_transfer(priv, 255);
    }
  else
    {
      stm32l4_i2c_set_bytes_to_transfer(priv, priv->dcnt);
    }

  /* Set the (7 bit) address.
   * 10 bit addressing is not yet supported.
   */

  stm32l4_i2c_set_7bit_address(priv);

  /* The flag of the current message is used to determine the direction of
   * transfer required for the current message.
   */

  if (priv->flags & I2C_M_READ)
    {
      stm32l4_i2c_set_read_transfer_dir(priv);
    }
  else
    {
      stm32l4_i2c_set_write_transfer_dir(priv);
    }

  /* Set the I2C_CR2->START bit to 1 to instruct the hardware to send the
   * START condition using the address and transfer direction data entered.
   */

  i2cinfo("Sending START: dcnt=%i msgc=%i flags=0x%04x\n",
          priv->dcnt, priv->msgc, priv->flags);

  stm32l4_i2c_modifyreg32(priv, STM32L4_I2C_CR2_OFFSET, 0, I2C_CR2_START);
}

/****************************************************************************
 * Name: stm32l4_i2c_sendstop
 *
 * Description:
 *   Send the STOP conditions
 *
 *   A STOP condition can be requested by setting the STOP bit in the I2C_CR2
 *   register. Setting the STOP bit clears the TC flag and the STOP condition
 *   is sent on the bus.
 *
 ****************************************************************************/

static inline
void stm32l4_i2c_sendstop(struct stm32l4_i2c_priv_s *priv)
{
  i2cinfo("Sending STOP\n");
  stm32l4_i2c_traceevent(priv, I2CEVENT_WRITE_STOP, 0);

  stm32l4_i2c_modifyreg32(priv, STM32L4_I2C_CR2_OFFSET,
                          0, I2C_CR2_STOP);
}

/****************************************************************************
 * Name: stm32l4_i2c_getstatus
 *
 * Description:
 *   Get 32-bit status (SR1 and SR2 combined)
 *
 ****************************************************************************/

static inline
uint32_t stm32l4_i2c_getstatus(struct stm32l4_i2c_priv_s *priv)
{
  return getreg32(priv->config->base + STM32L4_I2C_ISR_OFFSET);
}

/****************************************************************************
 * Name: stm32l4_i2c_clearinterrupts
 *
 * Description:
 *  Clear all interrupts
 *
 ****************************************************************************/

static inline
void stm32l4_i2c_clearinterrupts(struct stm32l4_i2c_priv_s *priv)
{
  stm32l4_i2c_modifyreg32(priv, STM32L4_I2C_ICR_OFFSET,
                          0, I2C_ICR_CLEARMASK);
}

/****************************************************************************
 * Name: stm32l4_i2c_isr_process
 *
 * Description:
 *  Common interrupt service routine (ISR) that handles I2C protocol logic.
 *  This is instantiated for each configured I2C interface
 * (I2C1, I2C2, I2C3).
 *
 *  This ISR is activated and deactivated by:
 *
 *   stm32l4_i2c_process
 *    and
 *   stm32l4_i2c_waitdone
 *
 * Input Parameters:
 *   priv - The private struct of the I2C driver.
 *
 ****************************************************************************/

static int stm32l4_i2c_isr_process(struct stm32l4_i2c_priv_s *priv)
{
  uint32_t status;

  /* Get state of the I2C controller */

  status = stm32l4_i2c_getreg32(priv, STM32L4_I2C_ISR_OFFSET);

  i2cinfo("ENTER: status = 0x%08" PRIx32 "\n", status);

  /* Update private version of the state */

  priv->status = status;

  /* If this is a new transmission set up the trace table accordingly */

  stm32l4_i2c_tracenew(priv, status);
  stm32l4_i2c_traceevent(priv, I2CEVENT_ISR_CALL, 0);

  /* ------------------- Start of I2C protocol handling ------------------ */

  /* I2C protocol logic follows. It's organized in an if else chain such that
   * only one mode of operation is executed every time the ISR is called.
   *
   * If you need to add additional states to support new features be sure
   * they continue the chain (i.e. begin with "else if") and are placed
   * before the empty call / error states at the end of the chain.
   */

  /* NACK Handling
   *
   * This branch is only triggered when the NACK (Not Acknowledge Received)
   * interrupt occurs.  This interrupt will only fire when the
   * I2C_CR1->NACKIE bit is 1.
   *
   * I2C_ISR->NACKF is set by hardware when a NACK is received after a byte
   * is transmitted and the slave fails to acknowledge it.  This is the
   * opposite of, and mutually exclusive to, the I2C_ISR->TXIS event.
   *
   * In response to the NACK the hardware automatically triggers generation
   * of a STOP condition, terminating the transfer.  The only valid response
   * to this state is to exit the ISR and report the failure.
   *
   * To differentiate an "address NACK" from a NACK that might occur during
   * the transfer of other bytes the "priv->astart" parameter is
   * used.  This flag is set to TRUE in sendstart() and set to FALSE when
   * the first TXIS event is received, which would be after the first byte
   * (the address) is transmitted successfully (acknowledged).
   */

  if (status & I2C_INT_NACK)
    {
      if (priv->astart == true)
        {
          /* NACK received on first (address) byte: address is invalid */

          i2cinfo("NACK: Address invalid: dcnt=%i "
                  "msgc=%i status=0x%08" PRIx32 "\n",
                  priv->dcnt, priv->msgc, status);
          stm32l4_i2c_traceevent(priv, I2CEVENT_ADDRESS_NACKED,
                                 priv->msgv->addr);
        }
      else
        {
          /* NACK received on regular byte */

          i2cinfo("NACK: NACK received: dcnt=%i "
                  "msgc=%i status=0x%08" PRIx32 "\n",
                  priv->dcnt, priv->msgc, status);
          stm32l4_i2c_traceevent(priv, I2CEVENT_ADDRESS_NACKED,
                                 priv->msgv->addr);
        }

      /* Set flags to terminate message transmission:
       *
       *   set message length to -1 to indicate last byte of message sent
       *   set message count to 0 to indicate no more messages to send
       *
       * As we fall through the logic in the ISR the message handling block
       * will be triggered by these flags and signal the ISR to terminate.
       */

      priv->dcnt = -1;
      priv->msgc = 0;
    }

  /* Transmit Interrupt Status (TXIS) Handler
   *
   * This branch is only triggered when the TXIS interrupt occurs.  This
   * interrupt will only fire when the I2C_CR1->TXIE bit is 1.
   *
   * This indicates the transmit data register I2C_TXDR has been emptied
   * following the successful transmission of a byte and slave
   * acknowledgement. In this state the I2C_TXDR register is ready to accept
   * another byte for transmission.  The TXIS bit will be cleared
   * automatically when the next byte is written to I2C_TXDR.
   *
   * The number of TXIS events during the transfer corresponds to NBYTES.
   *
   * The TXIS flag is not set when a NACK is received.
   *
   * When RELOAD is disabled (RELOAD=0) and NBYTES data have been
   * transferred:
   *
   *   - In Automatic End Mode (AUTOEND=1), a STOP is automatically sent.
   *
   *     Note:  Automatic End Mode is not currently supported.
   *
   *   - In Software End Mode (AUTOEND=0), the TC event occurs and the SCL
   *     line is stretched low in order to allow software actions (STOP,
   *     RESTART).
   *
   * When RELOAD is enabled (RELOAD=1) and NBYTES bytes have been transferred
   * a TCR event occurs instead and that handler simply updates NBYTES which
   * causes TXIS events to continue.  The process repeats until all bytes in
   * the message have been transferred.
   */

  else if ((priv->flags & (I2C_M_READ)) == 0 &&
           (status & (I2C_ISR_TXIS)) != 0)
    {
      /* TXIS interrupt occurred, address valid, ready to transmit */

      stm32l4_i2c_traceevent(priv, I2CEVENT_WRITE, 0);
      i2cinfo("TXIS: ENTER dcnt = %i msgc = %i status 0x%08" PRIx32 "\n",
              priv->dcnt, priv->msgc, status);

      /* The first event after the address byte is sent will be either TXIS
       * or NACKF so it's safe to set the astart flag to false on
       * the first TXIS event to indicate that it is no longer necessary to
       * check for address validity.
       */

      if (priv->astart == true)
        {
          i2cinfo("TXIS: Address Valid\n");
          stm32l4_i2c_traceevent(priv, I2CEVENT_ADDRESS_ACKED,
                                 priv->msgv->addr);
          priv->astart = false;
        }

      /* If one or more bytes in the current message are ready to transmit */

      if (priv->dcnt > 0)
        {
          /* Prepare to transmit the current byte */

          stm32l4_i2c_traceevent(priv, I2CEVENT_WRITE_TO_DR, priv->dcnt);
          i2cinfo("TXIS: Write Data 0x%02x\n", *priv->ptr);

          /* Decrement byte counter */

          priv->dcnt--;

          /* If we are about to transmit the last byte in the current
           * message
           */

          if (priv->dcnt == 0)
            {
              /* If this is also the last message to send, disable RELOAD so
               * TC fires next and issues STOP condition.  If we don't do
               * this TCR will fire next, and since there are no bytes to
               * send we can't write NBYTES to clear TCR so it will fire
               * forever.
               */

              if (priv->msgc == 1)
                {
                  stm32l4_i2c_disable_reload(priv);
                }
            }

          /* Transmit current byte */

          stm32l4_i2c_putreg(priv, STM32L4_I2C_TXDR_OFFSET, *priv->ptr);

          /* Advance to next byte */

          priv->ptr++;
        }
      else
        {
          /* Unsupported state */

          i2cerr("ERROR: TXIS Unsupported state detected, dcnt=%i, "
                 "status 0x%08" PRIx32 "\n",
                 priv->dcnt, status);
          stm32l4_i2c_traceevent(priv, I2CEVENT_WRITE_ERROR, 0);
        }

      i2cinfo("TXIS: EXIT  dcnt = %i msgc = %i status 0x%08" PRIx32 "\n",
              priv->dcnt, priv->msgc, status);
    }

  /* Receive Buffer Not Empty (RXNE) State Handler
   *
   * This branch is only triggered when the RXNE interrupt occurs.  This
   * interrupt will only fire when the I2C_CR1->RXIE bit is 1.
   *
   * This indicates data has been received from the bus and is waiting to
   * be read from the I2C_RXDR register.  When I2C_RXDR is read this bit
   * is automatically cleared and then an ACK or NACK is sent depending on
   * whether we have more bytes to receive.
   *
   * When RELOAD is disabled and bytes remain to be transferred an
   * acknowledge is automatically sent on the bus and the RXNE events
   * continue until the last byte is received.
   *
   * When RELOAD is disabled (RELOAD=0) and BYTES have been transferred:
   *
   *   - In Automatic End Mode (AUTOEND=1), a NACK and a STOP are
   *     automatically sent after the last received byte.
   *
   *     Note:  Automatic End Mode is not currently supported.
   *
   *   - In Software End Mode (AUTOEND=0), a NACK is automatically sent after
   *     the last received byte, the TC event occurs and the SCL line is
   *     stretched low in order to allow software actions (STOP, RESTART).
   *
   * When RELOAD is enabled (RELOAD=1) and NBYTES bytes have been transferred
   * a TCR event occurs and that handler simply updates NBYTES which causes
   * RXNE events to continue until all bytes have been transferred.
   */

  else if ((priv->flags & (I2C_M_READ)) != 0 && (status & I2C_ISR_RXNE) != 0)
    {
      /* When read flag is set and the receive buffer is not empty
       * (RXNE is set) then the driver can read from the data register.
       */

      stm32l4_i2c_traceevent(priv, I2CEVENT_READ, 0);
      i2cinfo("RXNE: ENTER dcnt = %i msgc = %i status 0x%08" PRIx32 "\n",
              priv->dcnt, priv->msgc, status);

      /* If more bytes in the current message */

      if (priv->dcnt > 0)
        {
          stm32l4_i2c_traceevent(priv, I2CEVENT_RCVBYTE, priv->dcnt);

          /* No interrupts or context switches may occur in the following
           * sequence.  Otherwise, additional bytes may be received.
           */

#ifdef CONFIG_I2C_POLLED
          irqstate_t state = enter_critical_section();
#endif
          /* Receive a byte */

          *priv->ptr = stm32l4_i2c_getreg(priv, STM32L4_I2C_RXDR_OFFSET);

          i2cinfo("RXNE: Read Data 0x%02x\n", *priv->ptr);

          /* Advance buffer to the next byte in the message */

          priv->ptr++;

          /* Signal byte received */

          priv->dcnt--;

#ifdef CONFIG_I2C_POLLED
          leave_critical_section(state);
#endif
        }
      else
        {
          /* Unsupported state */

          stm32l4_i2c_traceevent(priv, I2CEVENT_READ_ERROR, 0);
          status = stm32l4_i2c_getreg(priv, STM32L4_I2C_ISR_OFFSET);
          i2cerr("ERROR: RXNE Unsupported state detected, dcnt=%i, "
                 "status 0x%08" PRIx32 "\n",
                 priv->dcnt, status);

          /* Set signals that will terminate ISR and wake waiting thread */

          priv->dcnt = -1;
          priv->msgc = 0;
        }

      i2cinfo("RXNE: EXIT  dcnt = %i msgc = %i status 0x%08" PRIx32 "\n",
              priv->dcnt, priv->msgc, status);
    }

  /* Transfer Complete (TC) State Handler
   *
   * This branch is only triggered when the TC interrupt occurs.  This
   * interrupt will only fire when:
   *
   * I2C_CR1->TCIE = 1 (Transfer Complete Interrupts Enabled)
   * I2C_CR2->RELOAD = 0 (Reload Mode Disabled)
   * I2C_CR2->AUTOEND = 0 (Autoend Mode Disabled, i.e. Software End Mode)
   *
   * This event indicates that the number of bytes initially defined
   * in NBYTES, meaning, the number of bytes in the current message
   * (priv->dcnt) has been successfully transmitted or received.
   *
   * When the TC interrupt occurs we have two choices to clear it and move
   * on, regardless of the transfer direction:
   *
   *    - if more messages follow, perform a repeated START if required
   *      and then fall through to transmit or receive the next message.
   *
   *    - if no messages follow, perform a STOP and set flags needed to
   *      exit the ISR.
   *
   * The fact that the hardware must either RESTART or STOP when a TC
   * event occurs explains why, when messages must be sent back to back
   * (i.e. without a restart by specifying the I2C_M_NOSTART flag),
   * RELOAD mode must be enabled and TCR event(s) must be generated
   * instead.  See the TCR handler for more.
   */

  else if ((status & I2C_ISR_TC) != 0)
    {
      i2cinfo("TC: ENTER dcnt = %i msgc = %i status 0x%08" PRIx32 "\n",
              priv->dcnt, priv->msgc, status);

      /* Prior message has been sent successfully. Or there could have
       * been an error that set msgc to 0; So test for that case as
       * we do not want to decrement msgc less then zero nor move msgv
       * past the last message.
       */

      if (priv->msgc > 0)
        {
          priv->msgc--;
        }

      /* Are there additional messages remain to be transmitted / received? */

      if (priv->msgc > 0)
        {
          i2cinfo("TC: RESTART: dcnt=%i, msgc=%i\n",
          priv->dcnt, priv->msgc);
          stm32l4_i2c_traceevent(priv, I2CEVENT_TC_NO_RESTART, priv->msgc);

          /* Issue a START condition.
           *
           * Note that the first thing sendstart does is update the
           * private structure "current message" data (ptr, dcnt, flags)
           * so they all reflect the next message in the list so we
           * update msgv before we get there.
           */

          /* Advance to the next message in the list */

          priv->msgv++;

          stm32l4_i2c_sendstart(priv);
        }
      else
        {
          /* Issue a STOP conditions.
           *
           * No additional messages to transmit / receive, so the
           * transfer is indeed complete.  Nothing else to do but
           * issue a STOP and exit.
           */

          i2cinfo("TC: STOP: dcnt=%i msgc=%i\n",
          priv->dcnt, priv->msgc);
          stm32l4_i2c_traceevent(priv, I2CEVENT_STOP, priv->dcnt);

          stm32l4_i2c_sendstop(priv);

          /* Set signals that will terminate ISR and wake waiting thread */

          priv->dcnt = -1;
          priv->msgc = 0;
        }

      i2cinfo("TC: EXIT dcnt = %i msgc = %i status 0x%08" PRIx32 "\n",
              priv->dcnt, priv->msgc, status);
    }

  /* Transfer Complete (Reload) State Handler
   *
   * This branch is only triggered when the TCR interrupt occurs.  This
   * interrupt will only fire when:
   *
   *  I2C_CR1->TCIE = 1 (Transfer Complete Interrupts Enabled)
   *  I2C_CR2->RELOAD = 1 (Reload Mode Active)
   *  I2C_CR2->AUTOEND = 0 (Autoend Mode Disabled, i.e. Software End Mode)
   *
   * This is similar to the TC event except that TCR assumes that additional
   * bytes are available to transfer.  So despite what its name might imply
   * the transfer really isn't complete.
   *
   * There are two reasons RELOAD would be enabled:
   *
   * 1) We're trying to send a message with a payload greater than 255 bytes.
   * 2) We're trying to send messages back to back, regardless of their
   *    payload size, to avoid a RESTART (i.e. I2C_M_NOSTART flag is set).
   *
   * These conditions may be true simultaneously, as would be the case if
   * we're sending multiple messages with payloads > 255 bytes.   So we only
   * advance to the next message if we arrive here and dcnt is 0, meaning,
   * we're finished with the last message and ready to move to the next.
   *
   * This logic supports the transfer of bytes limited only by the size of
   * the i2c_msg_s length variable.  The SCL line will be stretched low
   * until NBYTES is written with a non-zero value, allowing the transfer
   * to continue.
   *
   * TODO: RESTARTs are required by the I2CSPEC if the next message transfer
   * direction changes.  Right now the NORESTART flag overrides this
   * behavior. May have to introduce logic to issue sendstart, assuming it's
   * legal with the hardware in the TCR state.
   */

  else if ((status & I2C_ISR_TCR) != 0)
    {
      i2cinfo("TCR: ENTER dcnt = %i msgc = %i status 0x%08" PRIx32 "\n",
              priv->dcnt, priv->msgc, status);

      /* If no more bytes in the current message to transfer */

      if (priv->dcnt == 0)
        {
          /* Prior message has been sent successfully */

          priv->msgc--;

          /* Advance to the next message in the list */

          priv->msgv++;

          /* Update current message data */

          priv->ptr    = priv->msgv->buffer;
          priv->dcnt   = priv->msgv->length;
          priv->flags  = priv->msgv->flags;

          /* if this is the last message, disable reload so the
           * TC event fires next time.
           */

          if (priv->msgc == 0)
            {
              i2cinfo("TCR: DISABLE RELOAD: dcnt = %i msgc = %i\n",
              priv->dcnt, priv->msgc);

              stm32l4_i2c_disable_reload(priv);
            }

          /* Update NBYTES with length of current message */

          i2cinfo("TCR: NEXT MSG dcnt = %i msgc = %i\n",
                  priv->dcnt, priv->msgc);

          stm32l4_i2c_set_bytes_to_transfer(priv, priv->dcnt);
        }
      else
        {
          /* More bytes in the current (greater than 255 byte payload
           * length) message, so set NBYTES according to the bytes
           * remaining in the message, up to a maximum each cycle of 255.
           */

          if (priv->dcnt > 255)
            {
              i2cinfo(
                "TCR: ENABLE RELOAD: NBYTES = 255 dcnt = %i msgc = %i\n",
                priv->dcnt, priv->msgc);

              /* More than 255 bytes to transfer so the RELOAD bit is
               * set in order to generate a TCR event rather than a TC
               * event when 255 bytes are successfully transferred.
               * This forces us to return here to update NBYTES and
               * continue until NBYTES is set to less than 255 bytes,
               * at which point RELOAD will be disabled and a TC
               * event will (eventually) follow to officially terminate
               * the transfer.
               */

              stm32l4_i2c_enable_reload(priv);

              stm32l4_i2c_set_bytes_to_transfer(priv, 255);
            }
          else
            {
              /* Less than 255 bytes left to transfer, which means we'll
               * complete the transfer of all bytes in the current message
               * the next time around.
               *
               * This means we need to disable the RELOAD functionality so
               * we receive a TC event next time which will allow us to
               * either RESTART and continue sending the contents of the
               * next message or send a STOP condition and exit the ISR.
               */

              i2cinfo("TCR: DISABLE RELOAD: NBYTES = dcnt = %i msgc = %i\n",
                      priv->dcnt, priv->msgc);

              stm32l4_i2c_disable_reload(priv);

              stm32l4_i2c_set_bytes_to_transfer(priv, priv->dcnt);
            }

          i2cinfo("TCR: EXIT dcnt = %i msgc = %i status 0x%08" PRIx32 "\n",
                  priv->dcnt, priv->msgc, status);
        }
    }

  /* Empty call handler
   *
   * Case to handle an empty call to the ISR where it has nothing to
   * do and should exit immediately.
   */

  else if (priv->dcnt == -1 && priv->msgc == 0)
    {
      status = stm32l4_i2c_getreg(priv, STM32L4_I2C_ISR_OFFSET);
      i2cwarn("WARNING: EMPTY CALL: Stopping ISR: status 0x%08" PRIx32 "\n",
              status);
      stm32l4_i2c_traceevent(priv, I2CEVENT_ISR_EMPTY_CALL, 0);
    }

  /* Error handler
   *
   * We get to this branch only if we can't handle the current state.
   *
   * This should not happen in interrupt based operation.
   *
   * This will happen during polled operation when the device is not
   * in one of the supported states when polled.
   */

  else
    {
#ifdef CONFIG_I2C_POLLED
      stm32l4_i2c_traceevent(priv, I2CEVENT_POLL_DEV_NOT_RDY, 0);
#else
      /* Read rest of the state */

      status = stm32l4_i2c_getreg(priv, STM32L4_I2C_ISR_OFFSET);

      i2cerr("ERROR: Invalid state detected, status 0x%08" PRIx32 "\n",
             status);

      /* set condition to terminate ISR and wake waiting thread */

      priv->dcnt = -1;
      priv->msgc = 0;
      stm32l4_i2c_traceevent(priv, I2CEVENT_STATE_ERROR, 0);
#endif
    }

  /* ------------------- End of I2C protocol handling ------------------ */

  /* Message Handling
   *
   * Transmission of the whole message chain has been completed. We have to
   * terminate the ISR and wake up stm32l4_i2c_process() that is waiting for
   * the ISR cycle to handle the sending/receiving of the messages.
   */

  if (priv->dcnt == -1 && priv->msgc == 0)
    {
      i2cinfo("MSG: Shutting down I2C ISR\n");

      stm32l4_i2c_traceevent(priv, I2CEVENT_ISR_SHUTDOWN, 0);

      /* clear pointer to message content to reflect we are done
       * with the current transaction.
       */

      priv->msgv = NULL;

#ifdef CONFIG_I2C_POLLED
      priv->intstate = INTSTATE_DONE;
#else

      status = stm32l4_i2c_getreg32(priv, STM32L4_I2C_ISR_OFFSET);

      /* Update private state to capture NACK which is used in combination
       * with the astart flag to report the type of NACK received (address
       * vs data) to the upper layers once we exit the ISR.
       *
       * Note: We do this prior to clearing interrupts because the NACKF
       * flag will naturally be cleared by that process.
       */

      priv->status = status;

      /* Clear all interrupts */

      stm32l4_i2c_modifyreg32(priv, STM32L4_I2C_ICR_OFFSET,
                              0, I2C_ICR_CLEARMASK);

      /* If a thread is waiting then inform it transfer is complete */

      if (priv->intstate == INTSTATE_WAITING)
        {
          nxsem_post(&priv->sem_isr);
          priv->intstate = INTSTATE_DONE;
        }
#endif
    }

  status = stm32l4_i2c_getreg32(priv, STM32L4_I2C_ISR_OFFSET);
  i2cinfo("EXIT: status = 0x%08" PRIx32 "\n", status);

  return OK;
}

/****************************************************************************
 * Name: stm32l4_i2c_isr
 *
 * Description:
 *   Common I2C interrupt service routine
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int stm32l4_i2c_isr(int irq, void *context, void *arg)
{
  struct stm32l4_i2c_priv_s *priv = (struct stm32l4_i2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);
  return stm32l4_i2c_isr_process(priv);
}
#endif

/****************************************************************************
 * Name: stm32l4_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ****************************************************************************/

static int stm32l4_i2c_init(struct stm32l4_i2c_priv_s *priv)
{
  /* Power-up and configure GPIOs */

  /* Enable power and reset the peripheral */

#ifdef CONFIG_STM32L4_I2C4
  if (priv->config->base == STM32L4_I2C4_BASE)
    {
      modifyreg32(STM32L4_RCC_APB1ENR2, 0, priv->config->clk_bit);
      modifyreg32(STM32L4_RCC_APB1RSTR2, 0, priv->config->reset_bit);
      modifyreg32(STM32L4_RCC_APB1RSTR2, priv->config->reset_bit, 0);
    }
  else
#endif
    {
      modifyreg32(STM32L4_RCC_APB1ENR1, 0, priv->config->clk_bit);
      modifyreg32(STM32L4_RCC_APB1RSTR1, 0, priv->config->reset_bit);
      modifyreg32(STM32L4_RCC_APB1RSTR1, priv->config->reset_bit, 0);
    }

  /* Configure pins */

  if (stm32l4_configgpio(priv->config->scl_pin) < 0)
    {
      return ERROR;
    }

  if (stm32l4_configgpio(priv->config->sda_pin) < 0)
    {
      stm32l4_unconfiggpio(priv->config->scl_pin);
      return ERROR;
    }

#ifndef CONFIG_I2C_POLLED
  /* Attach error and event interrupts to the ISRs */

  irq_attach(priv->config->ev_irq, stm32l4_i2c_isr, priv);
  irq_attach(priv->config->er_irq, stm32l4_i2c_isr, priv);
  up_enable_irq(priv->config->ev_irq);
  up_enable_irq(priv->config->er_irq);
#endif

  /* TODO:
   * - Provide means to set peripheral clock source via RCC_CCIPR_I2CxSEL
   * - Make clock source Kconfigurable (currently PCLK = 80 MHz)
   */

  /* Force a frequency update */

  priv->frequency = 0;
  stm32l4_i2c_setclock(priv, 100000);

  /* Enable I2C peripheral */

  stm32l4_i2c_modifyreg32(priv, STM32L4_I2C_CR1_OFFSET, 0, I2C_CR1_PE);

  return OK;
}

/****************************************************************************
 * Name: stm32l4_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ****************************************************************************/

static int stm32l4_i2c_deinit(struct stm32l4_i2c_priv_s *priv)
{
  /* Disable I2C */

  stm32l4_i2c_putreg32(priv, STM32L4_I2C_CR1_OFFSET, 0);

  /* Unconfigure GPIO pins */

  stm32l4_unconfiggpio(priv->config->scl_pin);
  stm32l4_unconfiggpio(priv->config->sda_pin);

#ifndef CONFIG_I2C_POLLED

  /* Disable and detach interrupts */

  up_disable_irq(priv->config->ev_irq);
  up_disable_irq(priv->config->er_irq);
  irq_detach(priv->config->ev_irq);
  irq_detach(priv->config->er_irq);
#endif

  /* Disable clocking */

#ifdef CONFIG_STM32L4_I2C4
  if (priv->config->base == STM32L4_I2C4_BASE)
    {
      modifyreg32(STM32L4_RCC_APB1ENR2, priv->config->clk_bit, 0);
    }
  else
#endif
    {
      modifyreg32(STM32L4_RCC_APB1ENR1, priv->config->clk_bit, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32l4_i2c_process
 *
 * Description:
 *   Common I2C transfer logic
 *
 *   Initiates a master mode transaction on the I2C bus to transfer the
 *   provided messages to and from the slave devices.
 *
 ****************************************************************************/

static int stm32l4_i2c_process(struct i2c_master_s *dev,
                               struct i2c_msg_s *msgs, int count)
{
  struct stm32l4_i2c_inst_s *inst = (struct stm32l4_i2c_inst_s *)dev;
  struct stm32l4_i2c_priv_s *priv = inst->priv;
  uint32_t    status = 0;
  uint32_t    cr1;
  uint32_t    cr2;
  int         errval = 0;
  int         waitrc = 0;

  DEBUGASSERT(count > 0);

  /* Wait for any STOP in progress */

  stm32l4_i2c_sem_waitstop(priv);

  /* Clear any pending error interrupts */

  stm32l4_i2c_clearinterrupts(priv);

  /* Old transfers are done */

  priv->msgv = msgs;
  priv->msgc = count;

  /* Reset I2C trace logic */

  stm32l4_i2c_tracereset(priv);

  /* Set I2C clock frequency (on change it toggles I2C_CR1_PE !) */

  stm32l4_i2c_setclock(priv, msgs->frequency);

  /* Trigger start condition, then the process moves into the ISR.  I2C
   * interrupts will be enabled within stm32l4_i2c_waitdone().
   */

  priv->status = 0;

#ifndef CONFIG_I2C_POLLED
  /* Enable transmit and receive interrupts here so when we send the start
   * condition below the ISR will fire if the data was sent and some
   * response from the slave received.  All other interrupts relevant to
   * our needs are enabled in stm32l4_i2c_sem_waitdone() below.
   */

  stm32l4_i2c_enableinterrupts(priv);
#endif

  /* Trigger START condition generation, which also sends the slave address
   * with read/write flag and the data in the first message
   */

  stm32l4_i2c_sendstart(priv);

  /* Wait for the ISR to tell us that the transfer is complete by attempting
   * to grab the semaphore that is initially locked by the ISR.  If the ISR
   * does not release the lock so we can obtain it here prior to the end of
   * the timeout period waitdone returns error and we report a timeout.
   */

  waitrc = stm32l4_i2c_sem_waitdone(priv);

  cr1 = stm32l4_i2c_getreg32(priv, STM32L4_I2C_CR1_OFFSET);
  cr2 = stm32l4_i2c_getreg32(priv, STM32L4_I2C_CR2_OFFSET);
#if !defined(CONFIG_DEBUG_I2C)
  UNUSED(cr1);
  UNUSED(cr2);
#endif

  /* Status after a normal / good exit is usually 0x00000001, meaning the TXE
   * bit is set.  That occurs as a result of the I2C_TXDR register being
   * empty, and it naturally will be after the last byte is transmitted.
   * This bit is cleared when we attempt communications again and re-enable
   * the peripheral.  The priv->status field can hold additional information
   * like a NACK, so we reset the status field to include that information.
   */

  status = stm32l4_i2c_getstatus(priv);

  /* The priv->status field can hold additional information like a NACK
   * event so we include that information.
   */

  status = priv->status & 0xffffffff;

  if (waitrc < 0)
    {
      /* Connection timed out */

      errval = ETIMEDOUT;
      i2cerr("ERROR: Waitdone timed out CR1: 0x%08" PRIx32
             " CR2: 0x%08" PRIx32 " status: 0x%08" PRIx32 "\n",
             cr1, cr2, status);
    }
  else
    {
      i2cinfo("Waitdone success: CR1: 0x%08" PRIx32 " CR2: 0x%08" PRIx32
              " status: 0x%08" PRIx32 "\n",
              cr1, cr2, status);
    }

  UNUSED(cr1);
  UNUSED(cr2);

  i2cinfo("priv->status: 0x%08" PRIx32 "\n", priv->status);

  /* Check for error status conditions */

  if ((status & (I2C_INT_BERR |
                 I2C_INT_ARLO |
                 I2C_INT_OVR |
                 I2C_INT_PECERR |
                 I2C_INT_TIMEOUT |
                 I2C_INT_NACK)) != 0)

    {
      /* one or more errors in the mask are present */

      if (status & I2C_INT_BERR)
        {
          /* Bus Error, ignore it because of errata (revision A,Z) */

          i2cerr("ERROR: I2C Bus Error\n");

          /* errval = EIO; */
        }
      else if (status & I2C_INT_ARLO)
        {
          /* Arbitration Lost (master mode) */

          i2cerr("ERROR: I2C Arbitration Lost\n");
          errval = EAGAIN;
        }

      else if (status & I2C_INT_OVR)
        {
          /* Overrun/Underrun */

          i2cerr("ERROR: I2C Overrun/Underrun\n");
          errval = EIO;
        }
      else if (status & I2C_INT_PECERR)
        {
      /* PEC Error in reception (SMBus Only) */

          i2cerr("ERROR: I2C PEC Error\n");
          errval = EPROTO;
        }
      else if (status & I2C_INT_TIMEOUT)
        {
          /* Timeout or Tlow Error (SMBus Only) */

          i2cerr("ERROR: I2C Timeout / Tlow Error\n");
          errval = ETIME;
        }
      else if (status & I2C_INT_NACK)
        {
          /* NACK Received, flag as "communication error on send" */

          if (priv->astart == TRUE)
            {
              i2cwarn("WARNING: I2C Address NACK\n");
              errval = EADDRNOTAVAIL;
            }
          else
            {
              i2cwarn("WARNING: I2C Data NACK\n");
              errval = ECOMM;
            }
        }
      else
        {
          /* Unrecognized error */

          i2cerr("ERROR: I2C Unrecognized Error");
          errval = EINTR;
        }
    }

  /* This is not an error, but should not happen.  The BUSY signal can be
   * present if devices on the bus are in an odd state and need to be reset.
   * NOTE:
   * We will only see this busy indication if stm32l4_i2c_sem_waitdone()
   * fails above;  Otherwise it is cleared.
   */

  else if ((status & I2C_ISR_BUSY) != 0)
    {
      /* I2C Bus Busy
       *
       * This is a status condition rather than an error.
       *
       * We will only see this busy indication if stm32l4_i2c_sem_waitdone()
       * fails above;  Otherwise it is cleared by the hardware when the ISR
       * wraps up the transfer with a STOP condition.
       */

      clock_t start = clock_systime_ticks();
      clock_t timeout = USEC2TICK(USEC_PER_SEC / priv->frequency) + 1;

      status = stm32l4_i2c_getstatus(priv);

      while (status & I2C_ISR_BUSY)
        {
          if ((clock_systime_ticks() - start) > timeout)
            {
              i2cerr("ERROR: I2C Bus busy");
              errval = EBUSY;
              break;
            }

          status = stm32l4_i2c_getstatus(priv);
        }
    }

  /* Dump the trace result */

  stm32l4_i2c_tracedump(priv);
  nxmutex_unlock(&priv->lock);

  return -errval;
}

/****************************************************************************
 * Name: stm32l4_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ****************************************************************************/

static int stm32l4_i2c_transfer(struct i2c_master_s *dev,
                                struct i2c_msg_s *msgs,
                                int count)
{
  int ret;

  /* Ensure that address or flags don't change meanwhile */

  ret = nxmutex_lock(&((struct stm32l4_i2c_inst_s *)dev)->priv->lock);
  if (ret >= 0)
    {
      ret = stm32l4_i2c_process(dev, msgs, count);
    }

  return ret;
}

/****************************************************************************
 * Name: stm32l4_i2c_reset
 *
 * Description:
 *   Reset an I2C bus
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int stm32l4_i2c_reset(struct i2c_master_s *dev)
{
  struct stm32l4_i2c_priv_s *priv;
  unsigned int clock_count;
  unsigned int stretch_count;
  uint32_t scl_gpio;
  uint32_t sda_gpio;
  uint32_t frequency;
  int ret;

  DEBUGASSERT(dev);

  /* Get I2C private structure */

  priv = ((struct stm32l4_i2c_inst_s *)dev)->priv;

  /* Our caller must own a ref */

  DEBUGASSERT(priv->refs > 0);

  /* Lock out other clients */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EIO;

  /* Save the current frequency */

  frequency = priv->frequency;

  /* De-init the port */

  stm32l4_i2c_deinit(priv);

  /* Use GPIO configuration to un-wedge the bus */

  scl_gpio = MKI2C_OUTPUT(priv->config->scl_pin);
  sda_gpio = MKI2C_OUTPUT(priv->config->sda_pin);

  /* Let SDA go high */

  stm32l4_gpiowrite(sda_gpio, 1);

  /* Clock the bus until any slaves currently driving it let it go. */

  clock_count = 0;
  while (!stm32l4_gpioread(sda_gpio))
    {
      /* Give up if we have tried too hard */

      if (clock_count++ > 10)
        {
          goto out;
        }

      /* Sniff to make sure that clock stretching has finished.
       *
       * If the bus never relaxes, the reset has failed.
       */

      stretch_count = 0;
      while (!stm32l4_gpioread(scl_gpio))
        {
          /* Give up if we have tried too hard */

          if (stretch_count++ > 10)
            {
              goto out;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      stm32l4_gpiowrite(scl_gpio, 0);
      up_udelay(10);

      /* Drive SCL high again */

      stm32l4_gpiowrite(scl_gpio, 1);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave
   * state machines.
   */

  stm32l4_gpiowrite(sda_gpio, 0);
  up_udelay(10);
  stm32l4_gpiowrite(scl_gpio, 0);
  up_udelay(10);
  stm32l4_gpiowrite(scl_gpio, 1);
  up_udelay(10);
  stm32l4_gpiowrite(sda_gpio, 1);
  up_udelay(10);

  /* Revert the GPIO configuration. */

  stm32l4_unconfiggpio(sda_gpio);
  stm32l4_unconfiggpio(scl_gpio);

  /* Re-init the port */

  stm32l4_i2c_init(priv);

  /* Restore the frequency */

  stm32l4_i2c_setclock(priv, frequency);
  ret = OK;

out:

  /* Release the port for re-use by other clients */

  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Name: stm32l4_i2c_pm_prepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a
 *   warning that the system is about to enter into a new power state.  The
 *   driver should begin whatever operations that may be required to enter
 *   power state.  The driver may abort the state change mode by returning
 *   a non-zero value from the callback function.
 *
 * Input Parameters:
 *   cb      - Returned to the driver.  The driver version of the callback
 *             structure may include additional, driver-specific state
 *             data at the end of the structure.
 *   domain  - Identifies the activity domain of the state change
 *   pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   0 (OK) means the event was successfully processed and that the driver
 *   is prepared for the PM state change.  Non-zero means that the driver
 *   is not prepared to perform the tasks needed achieve this power setting
 *   and will cause the state change to be aborted.  NOTE:  The prepare
 *   method will also be recalled when reverting from lower back to higher
 *   power consumption modes (say because another driver refused a lower
 *   power state change).  Drivers are not permitted to return non-zero
 *   values when reverting back to higher power consumption modes!
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int stm32l4_i2c_pm_prepare(struct pm_callback_s *cb, int domain,
                                  enum pm_state_e pmstate)
{
  struct stm32l4_i2c_priv_s *priv =
                           (struct stm32l4_i2c_priv_s *)((char *)cb -
                            offsetof(struct stm32l4_i2c_priv_s, pm_cb));

  /* Logic to prepare for a reduced power state goes here. */

  switch (pmstate)
    {
    case PM_NORMAL:
    case PM_IDLE:
      break;

    case PM_STANDBY:
    case PM_SLEEP:

      /* Check if exclusive lock for I2C bus is held. */

      if (nxmutex_is_locked(&priv->lock))
        {
          /* Exclusive lock is held, do not allow entry to deeper PM
           * states.
           */

          return -EBUSY;
        }

      break;

    default:

      /* Should not get here */

      break;
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ****************************************************************************/

struct i2c_master_s *stm32l4_i2cbus_initialize(int port)
{
  struct stm32l4_i2c_priv_s *priv = NULL;  /* private data of device with multiple instances */
  struct stm32l4_i2c_inst_s *inst = NULL;  /* device, single instance */

  /* Get I2C private structure */

  switch (port)
    {
#ifdef CONFIG_STM32L4_I2C1
      case 1:
        priv = (struct stm32l4_i2c_priv_s *)&stm32l4_i2c1_priv;
        break;
#endif
#ifdef CONFIG_STM32L4_I2C2
      case 2:
        priv = (struct stm32l4_i2c_priv_s *)&stm32l4_i2c2_priv;
        break;
#endif
#ifdef CONFIG_STM32L4_I2C3
      case 3:
        priv = (struct stm32l4_i2c_priv_s *)&stm32l4_i2c3_priv;
        break;
#endif
#ifdef CONFIG_STM32L4_I2C4
      case 4:
        priv = (struct stm32l4_i2c_priv_s *)&stm32l4_i2c4_priv;
        break;
#endif
      default:
        return NULL;
    }

  /* Allocate instance */

  if (!(inst = kmm_malloc(sizeof(struct stm32l4_i2c_inst_s))))
    {
      return NULL;
    }

  /* Initialize instance */

  inst->ops  = &stm32l4_i2c_ops;
  inst->priv = priv;

  /* Init private data for the first time, increment refs count,
   * power-up hardware and configure GPIOs.
   */

  nxmutex_lock(&priv->lock);
  if (priv->refs++ == 0)
    {
      stm32l4_i2c_init(priv);

#ifdef CONFIG_PM
      /* Register to receive power management callbacks */

      DEBUGVERIFY(pm_register(&priv->pm_cb));
#endif
    }

  nxmutex_unlock(&priv->lock);
  return (struct i2c_master_s *)inst;
}

/****************************************************************************
 * Name: stm32l4_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ****************************************************************************/

int stm32l4_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct stm32l4_i2c_priv_s *priv;

  DEBUGASSERT(dev);
  priv = ((struct stm32l4_i2c_inst_s *)dev)->priv;

  /* Decrement refs and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  nxmutex_lock(&priv->lock);
  if (--priv->refs)
    {
      nxmutex_unlock(&priv->lock);
      kmm_free(dev);
      return OK;
    }

#ifdef CONFIG_PM
  /* Unregister power management callbacks */

  pm_unregister(&priv->pm_cb);
#endif

  /* Disable power and other HW resource (GPIO's) */

  stm32l4_i2c_deinit(priv);
  nxmutex_unlock(&priv->lock);

  kmm_free(dev);
  return OK;
}

#endif /* CONFIG_STM32L4_I2C1 || CONFIG_STM32L4_I2C2 || CONFIG_STM32L4_I2C3 || CONFIG_STM32L4_I2C4 */
