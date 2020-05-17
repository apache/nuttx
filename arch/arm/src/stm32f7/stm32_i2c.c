/************************************************************************************
 * arch/arm/src/stm32/stm32f7_i2c.c
 * STM32 I2C Hardware Layer - Device Driver
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
 *            Bob Feretich <bob.feretich@rafresearch.com>
 *
 * Major rewrite of ISR and supporting methods, including support
 * for NACK and RELOAD by:
 *
 *   Copyright (c) 2016 Doug Vetter.  All rights reserved.
 *   Author: Doug Vetter <oss@aileronlabs.com>
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
 ************************************************************************************/

/* ------------------------------------------------------------------------------
 *
 * STM32 F7 I2C Driver
 *
 * Supports:
 *  - Master operation:
 *      Standard-mode (up to 100 kHz)
 *      Fast-mode (up to 400 kHz)
 *      Fast-mode Plus (up to 1 MHz)
 *      fI2CCLK clock source selection is based on STM32_RCC_DCKCFGR2_I2CxSRC
 *      being set to HSI and the calculations are based on STM32_HSI_FREQUENCY
 *      of 16mHz
 *
 *  - Multiple instances (shared bus)
 *  - Interrupt based operation
 *  - RELOAD support
 *
 * Unsupported, possible future work:
 *  - More effective error reporting to higher layers
 *  - Slave operation
 *  - Support of fI2CCLK frequencies other than 16Mhz
 *  - Polled operation (code present but untested)
 *  - SMBus support
 *  - Multi-master support
 *  - IPMI
 *
 * Test Environment:
 *
 *  - STM32F7676ZI on ST Nucleo-144 Board (ST Part STM32F429ZIT6)
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
 *  - Instance: represents each individual access to the I2C driver, obtained by
 *      the i2c_init(); it extends the Device structure from the nuttx/i2c/i2c.h;
 *      Instance points to OPS, to common I2C Hardware private data and contains
 *      its own private data including frequency, address and mode of operation.
 *
 *  - Private: Private data of an I2C Hardware
 *
 * High Level Functional Description
 *
 * This driver works with I2C "messages" (struct i2c_msg_s), which carry a buffer
 * intended to transfer data to, or store data read from, the I2C bus.
 *
 * As the hardware can only transmit or receive one byte at a time the basic job
 * of the driver (and the ISR specifically) is to process each message in the
 * order they are stored in the message list, one byte at a time.  When
 * no messages are left the ISR exits and returns the result to the caller.
 *
 * The order of the list of I2C messages provided to the driver is important and
 * dependent upon the hardware in use.  A typical I2C transaction between the F3
 * as an I2C Master and some other IC as a I2C Slave requires two messages that
 * communicate the:
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
 *           (data transmitted to bus and acknowledged)
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
 * ------------------------------------------------------------------------------
 *
 * Configuration:
 *
 *  To use this driver, enable the following configuration variable:
 *
 *  One of:
 *
 *    CONFIG_STM32F7_STM32F72XX
 *    CONFIG_STM32F7_STM32F73XX
 *    CONFIG_STM32F7_STM32F74XX
 *    CONFIG_STM32F7_STM32F75XX
 *    CONFIG_STM32F7_STM32F76XX
 *    CONFIG_STM32F7_STM32F77XX
 *
 *
 *  and one or more interfaces:
 *
 *    CONFIG_STM32F7_I2C1
 *    CONFIG_STM32F7_I2C2
 *    CONFIG_STM32F7_I2C3
 *    CONFIG_STM32F7_I2C4
 *
 *  To configure the ISR timeout using fixed values (CONFIG_STM32F7_I2C_DYNTIMEO=n):
 *
 *    CONFIG_STM32F7_I2CTIMEOSEC   (Timeout in seconds)
 *    CONFIG_STM32F7_I2CTIMEOMS    (Timeout in milliseconds)
 *    CONFIG_STM32F7_I2CTIMEOTICKS (Timeout in ticks)
 *
 *  To configure the ISR timeout using dynamic values
 *  (CONFIG_STM32F7_I2C_DYNTIMEO=y):
 *
 *    CONFIG_STM32F7_I2C_DYNTIMEO_USECPERBYTE  (Timeout in microseconds per byte)
 *    CONFIG_STM32F7_I2C_DYNTIMEO_STARTSTOP    (Timeout for start/stop in
 *                                              milliseconds)
 *
 *  Debugging output enabled with:
 *
 *    CONFIG_DEBUG_FEATURES and CONFIG_DEBUG_I2C_{ERROR|WARN|INFO}
 *
 *  ISR Debugging output may be enabled with:
 *
 *    CONFIG_DEBUG_FEATURES and CONFIG_DEBUG_I2C_INFO
 *
 * ------------------------------------------------------------------------------
 *
 * References:
 *
 *  RM0431:
 *     ST STM32F72xxx and STM32F73xxx Reference Manual
 *     Document ID: DocID029480 Revision 1, Jan 2017.
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
 * ------------------------------------------------------------------------------
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/power/pm.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "arm_arch.h"

#include "stm32_rcc.h"
#include "stm32_i2c.h"
#include "stm32_gpio.h"
#include "hardware/stm32_pinmap.h"

/* At least one I2C peripheral must be enabled */

#if defined(CONFIG_STM32F7_I2C1) || defined(CONFIG_STM32F7_I2C2) || \
    defined(CONFIG_STM32F7_I2C3) || defined(CONFIG_STM32F7_I2C4)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#undef INVALID_CLOCK_SOURCE

#ifdef CONFIG_STM32F7_I2C1
#  if STM32_RCC_DCKCFGR2_I2C1SRC != RCC_DCKCFGR2_I2C1SEL_HSI
#    warning "Clock Source STM32_RCC_DCKCFGR2_I2C1SRC must be HSI"
#    define INVALID_CLOCK_SOURCE
#  endif
#endif
#ifdef CONFIG_STM32F7_I2C2
#  if STM32_RCC_DCKCFGR2_I2C2SRC != RCC_DCKCFGR2_I2C2SEL_HSI
#    warning "Clock Source STM32_RCC_DCKCFGR2_I2C2SRC must be HSI"
#    define INVALID_CLOCK_SOURCE
#  endif
#endif
#ifdef CONFIG_STM32F7_I2C3
#  if STM32_RCC_DCKCFGR2_I2C3SRC != RCC_DCKCFGR2_I2C3SEL_HSI
#    warning "Clock Source STM32_RCC_DCKCFGR2_I2C3SRC must be HSI"
#    define INVALID_CLOCK_SOURCE
#  endif
#endif
#ifdef CONFIG_STM32F7_I2C4
#  if STM32_RCC_DCKCFGR2_I2C4SRC != RCC_DCKCFGR2_I2C4SEL_HSI
#    warning "Clock Source STM32_RCC_DCKCFGR2_I2C4SRC must be HSI"
#    define INVALID_CLOCK_SOURCE
#  endif
#endif

/* CONFIG_I2C_POLLED may be set so that I2C interrupts will not be used.  Instead,
 * CPU-intensive polling will be used.
 */

/* Interrupt wait timeout in seconds and milliseconds */

#if !defined(CONFIG_STM32F7_I2CTIMEOSEC) && !defined(CONFIG_STM32F7_I2CTIMEOMS)
#  define CONFIG_STM32F7_I2CTIMEOSEC 0
#  define CONFIG_STM32F7_I2CTIMEOMS  500   /* Default is 500 milliseconds */
#  warning "Using Default 500 Ms Timeout"
#elif !defined(CONFIG_STM32F7_I2CTIMEOSEC)
#  define CONFIG_STM32F7_I2CTIMEOSEC 0     /* User provided milliseconds */
#elif !defined(CONFIG_STM32F7_I2CTIMEOMS)
#  define CONFIG_STM32F7_I2CTIMEOMS  0     /* User provided seconds */
#endif

/* Interrupt wait time timeout in system timer ticks */

#ifndef CONFIG_STM32F7_I2CTIMEOTICKS
#  define CONFIG_STM32F7_I2CTIMEOTICKS \
    (SEC2TICK(CONFIG_STM32F7_I2CTIMEOSEC) + MSEC2TICK(CONFIG_STM32F7_I2CTIMEOMS))
#endif

#ifndef CONFIG_STM32F7_I2C_DYNTIMEO_STARTSTOP
#  define CONFIG_STM32F7_I2C_DYNTIMEO_STARTSTOP TICK2USEC(CONFIG_STM32F7_I2CTIMEOTICKS)
#endif

/* Macros to convert a I2C pin to a GPIO output */

#define I2C_OUTPUT (GPIO_OUTPUT | GPIO_FLOAT | GPIO_OPENDRAIN |\
                      GPIO_SPEED_50MHz | GPIO_OUTPUT_SET)

#define MKI2C_OUTPUT(p) (((p) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | I2C_OUTPUT)

#define I2C_CR1_TXRX      (I2C_CR1_RXIE | I2C_CR1_TXIE)
#define I2C_CR1_ALLINTS   (I2C_CR1_TXRX | I2C_CR1_TCIE | I2C_CR1_ERRIE)

/* Unused bit in I2c_ISR used to communicate a bad state has occurred in
 * the isr processing
 */

#define I2C_INT_BAD_STATE 0x8000000

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
#  define stm32_i2c_tracereset(p)
#  define stm32_i2c_tracenew(p,s)
#  define stm32_i2c_traceevent(p,e,a)
#  define stm32_i2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#  define CONFIG_I2C_NTRACE 32
#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* Interrupt state */

enum stm32_intstate_e
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_WAITING,       /* Waiting for completion of interrupt activity */
  INTSTATE_DONE,          /* Interrupt activity complete */
};

/* Trace events */

enum stm32_trace_e
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

struct stm32_trace_s
{
  uint32_t status;             /* I2C 32-bit SR2|SR1 status */
  uint32_t count;              /* Interrupt count when status change */
  enum stm32_intstate_e event; /* Last event that occurred with this status */
  uint32_t parm;               /* Parameter associated with the event */
  clock_t time;                /* First of event or first status */
};

/* I2C Device hardware configuration */

struct stm32_i2c_config_s
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

struct stm32_i2c_priv_s
{
  /* Port configuration */

  const struct stm32_i2c_config_s *config;

  int refs;                    /* Reference count */
  sem_t sem_excl;              /* Mutual exclusion semaphore */
#ifndef CONFIG_I2C_POLLED
  sem_t sem_isr;               /* Interrupt wait semaphore */
#endif
  volatile uint8_t intstate;   /* Interrupt handshake (see enum stm32_intstate_e) */

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

  struct stm32_trace_s trace[CONFIG_I2C_NTRACE];
#endif

  uint32_t status;             /* End of transfer SR2|SR1 status */

#ifdef CONFIG_PM
  struct pm_callback_s pm_cb;  /* PM callbacks */
#endif
};

/* I2C Device, Instance */

struct stm32_i2c_inst_s
{
  const struct i2c_ops_s  *ops;  /* Standard I2C operations */
  struct stm32_i2c_priv_s *priv; /* Common driver private data structure */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

static inline uint16_t stm32_i2c_getreg(FAR struct stm32_i2c_priv_s *priv,
                                        uint8_t offset);
static inline void stm32_i2c_putreg(FAR struct stm32_i2c_priv_s *priv,
                                    uint8_t offset, uint16_t value);
static inline void stm32_i2c_putreg32(FAR struct stm32_i2c_priv_s *priv,
                                      uint8_t offset, uint32_t value);
static inline void stm32_i2c_modifyreg32(FAR struct stm32_i2c_priv_s *priv,
                                         uint8_t offset, uint32_t clearbits,
                                         uint32_t setbits);
#ifdef CONFIG_STM32F7_I2C_DYNTIMEO
static useconds_t stm32_i2c_tousecs(int msgc, FAR struct i2c_msg_s *msgs);
#endif /* CONFIG_STM32F7_I2C_DYNTIMEO */
static inline int  stm32_i2c_sem_waitdone(FAR struct stm32_i2c_priv_s *priv);
static inline void stm32_i2c_sem_waitstop(FAR struct stm32_i2c_priv_s *priv);
static inline void stm32_i2c_sem_post(FAR struct i2c_master_s *dev);
static inline void stm32_i2c_sem_init(FAR struct i2c_master_s *dev);
static inline void stm32_i2c_sem_destroy(FAR struct i2c_master_s *dev);
#ifdef CONFIG_I2C_TRACE
static void stm32_i2c_tracereset(FAR struct stm32_i2c_priv_s *priv);
static void stm32_i2c_tracenew(FAR struct stm32_i2c_priv_s *priv, uint32_t status);
static void stm32_i2c_traceevent(FAR struct stm32_i2c_priv_s *priv,
                               enum stm32_trace_e event, uint32_t parm);
static void stm32_i2c_tracedump(FAR struct stm32_i2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */
static void stm32_i2c_setclock(FAR struct stm32_i2c_priv_s *priv,
                               uint32_t frequency);
static inline void stm32_i2c_sendstart(FAR struct stm32_i2c_priv_s *priv);
static inline void stm32_i2c_sendstop(FAR struct stm32_i2c_priv_s *priv);
static inline uint32_t stm32_i2c_getstatus(FAR struct stm32_i2c_priv_s *priv);
static int stm32_i2c_isr_process(struct stm32_i2c_priv_s * priv);
#ifndef CONFIG_I2C_POLLED
static int stm32_i2c_isr(int irq, void *context, FAR void *arg);
#endif
static int stm32_i2c_init(FAR struct stm32_i2c_priv_s *priv);
static int stm32_i2c_deinit(FAR struct stm32_i2c_priv_s *priv);

static int stm32_i2c_process(FAR struct i2c_master_s *dev,
                             FAR struct i2c_msg_s *msgs, int count);
static int stm32_i2c_transfer(FAR struct i2c_master_s *dev,
                              FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int stm32_i2c_reset(FAR struct i2c_master_s * dev);
#endif
#ifdef CONFIG_PM
static int stm32_i2c_pm_prepare(FAR struct pm_callback_s *cb, int domain,
                                enum pm_state_e pmstate);
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifdef CONFIG_STM32F7_I2C1
static const struct stm32_i2c_config_s stm32_i2c1_config =
{
  .base          = STM32_I2C1_BASE,
  .clk_bit       = RCC_APB1ENR_I2C1EN,
  .reset_bit     = RCC_APB1RSTR_I2C1RST,
  .scl_pin       = GPIO_I2C1_SCL,
  .sda_pin       = GPIO_I2C1_SDA,
#ifndef CONFIG_I2C_POLLED
  .ev_irq        = STM32_IRQ_I2C1EV,
  .er_irq        = STM32_IRQ_I2C1ER
#endif
};

static struct stm32_i2c_priv_s stm32_i2c1_priv =
{
  .config        = &stm32_i2c1_config,
  .refs          = 0,
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .frequency     = 0,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0,
#ifdef CONFIG_PM
  .pm_cb.prepare = stm32_i2c_pm_prepare,
#endif
};
#endif

#ifdef CONFIG_STM32F7_I2C2
static const struct stm32_i2c_config_s stm32_i2c2_config =
{
  .base          = STM32_I2C2_BASE,
  .clk_bit       = RCC_APB1ENR_I2C2EN,
  .reset_bit     = RCC_APB1RSTR_I2C2RST,
  .scl_pin       = GPIO_I2C2_SCL,
  .sda_pin       = GPIO_I2C2_SDA,
#ifndef CONFIG_I2C_POLLED
  .ev_irq        = STM32_IRQ_I2C2EV,
  .er_irq        = STM32_IRQ_I2C2ER
#endif
};

static struct stm32_i2c_priv_s stm32_i2c2_priv =
{
  .config        = &stm32_i2c2_config,
  .refs          = 0,
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .frequency     = 0,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0,
#ifdef CONFIG_PM
  .pm_cb.prepare = stm32_i2c_pm_prepare,
#endif
};
#endif

#ifdef CONFIG_STM32F7_I2C3
static const struct stm32_i2c_config_s stm32_i2c3_config =
{
  .base          = STM32_I2C3_BASE,
  .clk_bit       = RCC_APB1ENR_I2C3EN,
  .reset_bit     = RCC_APB1RSTR_I2C3RST,
  .scl_pin       = GPIO_I2C3_SCL,
  .sda_pin       = GPIO_I2C3_SDA,
#ifndef CONFIG_I2C_POLLED
  .ev_irq        = STM32_IRQ_I2C3EV,
  .er_irq        = STM32_IRQ_I2C3ER
#endif
};

static struct stm32_i2c_priv_s stm32_i2c3_priv =
{
  .config        = &stm32_i2c3_config,
  .refs          = 0,
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .frequency     = 0,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0,
#ifdef CONFIG_PM
  .pm_cb.prepare = stm32_i2c_pm_prepare,
#endif
};
#endif

#ifdef CONFIG_STM32F7_I2C4
static const struct stm32_i2c_config_s stm32_i2c4_config =
{
  .base          = STM32_I2C4_BASE,
  .clk_bit       = RCC_APB1ENR_I2C4EN,
  .reset_bit     = RCC_APB1RSTR_I2C4RST,
  .scl_pin       = GPIO_I2C4_SCL,
  .sda_pin       = GPIO_I2C4_SDA,
#ifndef CONFIG_I2C_POLLED
  .ev_irq        = STM32_IRQ_I2C4EV,
  .er_irq        = STM32_IRQ_I2C4ER
#endif
};

static struct stm32_i2c_priv_s stm32_i2c4_priv =
{
  .config        = &stm32_i2c4_config,
  .refs          = 0,
  .intstate      = INTSTATE_IDLE,
  .msgc          = 0,
  .msgv          = NULL,
  .ptr           = NULL,
  .frequency     = 0,
  .dcnt          = 0,
  .flags         = 0,
  .status        = 0,
#ifdef CONFIG_PM
  .pm_cb.prepare = stm32_i2c_pm_prepare,
#endif
};
#endif

/* Device Structures, Instantiation */

static const struct i2c_ops_s stm32_i2c_ops =
{
  .transfer      = stm32_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset         = stm32_i2c_reset,
#endif
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_i2c_getreg
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ************************************************************************************/

static inline uint16_t stm32_i2c_getreg(FAR struct stm32_i2c_priv_s *priv,
                                        uint8_t offset)
{
  return getreg16(priv->config->base + offset);
}

/************************************************************************************
 * Name: stm32_i2c_getreg32
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ************************************************************************************/

static inline uint32_t stm32_i2c_getreg32(FAR struct stm32_i2c_priv_s *priv,
                                          uint8_t offset)
{
  return getreg32(priv->config->base + offset);
}

/************************************************************************************
 * Name: stm32_i2c_putreg
 *
 * Description:
 *  Put a 16-bit register value by offset
 *
 ************************************************************************************/

static inline void stm32_i2c_putreg(FAR struct stm32_i2c_priv_s *priv,
                                    uint8_t offset, uint16_t value)
{
  putreg16(value, priv->config->base + offset);
}

/************************************************************************************
 * Name: stm32_i2c_putreg32
 *
 * Description:
 *  Put a 32-bit register value by offset
 *
 ************************************************************************************/

static inline void stm32_i2c_putreg32(FAR struct stm32_i2c_priv_s *priv,
                                      uint8_t offset, uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

/************************************************************************************
 * Name: stm32_i2c_modifyreg32
 *
 * Description:
 *   Modify a 32-bit register value by offset
 *
 ************************************************************************************/

static inline void stm32_i2c_modifyreg32(FAR struct stm32_i2c_priv_s *priv,
                                         uint8_t offset, uint32_t clearbits,
                                         uint32_t setbits)
{
  modifyreg32(priv->config->base + offset, clearbits, setbits);
}

/************************************************************************************
 * Name: stm32_i2c_tousecs
 *
 * Description:
 *   Return a micro-second delay based on the number of bytes left to be processed.
 *
 ************************************************************************************/

#ifdef CONFIG_STM32F7_I2C_DYNTIMEO
static useconds_t stm32_i2c_tousecs(int msgc, FAR struct i2c_msg_s *msgs)
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

  return (useconds_t)(CONFIG_STM32F7_I2C_DYNTIMEO_USECPERBYTE * bytecount);
}
#endif

/************************************************************************************
 * Name: stm32_i2c_enableinterrupts
 *
 * Description:
 *   Enable I2C interrupts
 *
 ************************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline void stm32_i2c_enableinterrupts(struct stm32_i2c_priv_s *priv)
{
    stm32_i2c_modifyreg32(priv, STM32_I2C_CR1_OFFSET, 0,
                          (I2C_CR1_TXRX | I2C_CR1_NACKIE));
}
#endif

/************************************************************************************
 * Name: stm32_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 * There are two versions of this function.  The first is included when using
 * interrupts while the second is used if polling (CONFIG_I2C_POLLED=y).
 *
 ************************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline int stm32_i2c_sem_waitdone(FAR struct stm32_i2c_priv_s *priv)
{
  struct timespec abstime;
  irqstate_t flags;
  int ret;

  flags = enter_critical_section();

  /* Enable I2C interrupts */

  /* The TXIE and RXIE interrupts are enabled initially in stm32_i2c_process.
   * The remainder of the interrupts, including error-related, are enabled here.
   */

  stm32_i2c_modifyreg32(priv, STM32_I2C_CR1_OFFSET, 0,
                        (I2C_CR1_ALLINTS & ~I2C_CR1_TXRX));

  /* Signal the interrupt handler that we are waiting */

  priv->intstate = INTSTATE_WAITING;
  do
    {
      /* Get the current time */

      clock_gettime(CLOCK_REALTIME, &abstime);

      /* Calculate a time in the future */

#if CONFIG_STM32F7_I2CTIMEOSEC > 0
      abstime.tv_sec += CONFIG_STM32F7_I2CTIMEOSEC;
#endif

      /* Add a value proportional to the number of bytes in the transfer */

#ifdef CONFIG_STM32F7_I2C_DYNTIMEO
      abstime.tv_nsec += 1000 * stm32_i2c_tousecs(priv->msgc, priv->msgv);
      if (abstime.tv_nsec >= 1000 * 1000 * 1000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000 * 1000 * 1000;
        }

#elif CONFIG_STM32F7_I2CTIMEOMS > 0
      abstime.tv_nsec += CONFIG_STM32F7_I2CTIMEOMS * 1000 * 1000;
      if (abstime.tv_nsec >= 1000 * 1000 * 1000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000 * 1000 * 1000;
        }
#endif

      /* Wait until either the transfer is complete or the timeout expires */

      ret = nxsem_timedwait_uninterruptible(&priv->sem_isr, &abstime);
      if (ret < 0)
        {
          /* Break out of the loop on irrecoverable errors.  This would
           * include timeouts and mystery errors reported by nxsem_timedwait.
           */

          break;
        }
    }

  /* Loop until the interrupt level transfer is complete. */

  while (priv->intstate != INTSTATE_DONE);

  /* Set the interrupt state back to IDLE */

  priv->intstate = INTSTATE_IDLE;

  /* Disable I2C interrupts */

  stm32_i2c_modifyreg32(priv, STM32_I2C_CR1_OFFSET, I2C_CR1_ALLINTS, 0);

  leave_critical_section(flags);
  return ret;
}
#else
static inline int stm32_i2c_sem_waitdone(FAR struct stm32_i2c_priv_s *priv)
{
  clock_t timeout;
  clock_t start;
  clock_t elapsed;
  int ret;

  /* Get the timeout value */

#ifdef CONFIG_STM32F7_I2C_DYNTIMEO
  timeout = USEC2TICK(stm32_i2c_tousecs(priv->msgc, priv->msgv));
#else
  timeout = CONFIG_STM32F7_I2CTIMEOTICKS;
#endif

  /* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
   * are currently disabled but will be temporarily re-enabled below when
   * nxsem_timedwait() sleeps.
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

      stm32_i2c_isr_process(priv);
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

/************************************************************************************
 * Name: stm32_i2c_set_7bit_address
 *
 * Description:
 *
 ************************************************************************************/

static inline void
stm32_i2c_set_7bit_address(FAR struct stm32_i2c_priv_s *priv)
{
  stm32_i2c_modifyreg32(priv, STM32_I2C_CR2_OFFSET, I2C_CR2_SADD7_MASK,
                        ((priv->msgv->addr & 0x7f) << I2C_CR2_SADD7_SHIFT));
}

/************************************************************************************
 * Name: stm32_i2c_set_bytes_to_transfer
 *
 * Description:
 *
 ************************************************************************************/

static inline void
stm32_i2c_set_bytes_to_transfer(FAR struct stm32_i2c_priv_s *priv,
                               uint8_t n_bytes)
{
  stm32_i2c_modifyreg32(priv, STM32_I2C_CR2_OFFSET, I2C_CR2_NBYTES_MASK,
                        (n_bytes << I2C_CR2_NBYTES_SHIFT));
}

/************************************************************************************
 * Name: stm32_i2c_set_write_transfer_dir
 *
 * Description:
 *
 ************************************************************************************/

static inline void
stm32_i2c_set_write_transfer_dir(FAR struct stm32_i2c_priv_s *priv)
{
  stm32_i2c_modifyreg32(priv, STM32_I2C_CR2_OFFSET, I2C_CR2_RD_WRN, 0);
}

/************************************************************************************
 * Name: stm32_i2c_set_read_transfer_dir
 *
 * Description:
 *
 ************************************************************************************/

static inline void
stm32_i2c_set_read_transfer_dir(FAR struct stm32_i2c_priv_s *priv)
{
  stm32_i2c_modifyreg32(priv, STM32_I2C_CR2_OFFSET, 0, I2C_CR2_RD_WRN);
}

/************************************************************************************
 * Name: stm32_i2c_enable_reload
 *
 * Description:
 *
 ************************************************************************************/

static inline void
stm32_i2c_enable_reload(FAR struct stm32_i2c_priv_s *priv)
{
  stm32_i2c_modifyreg32(priv, STM32_I2C_CR2_OFFSET, 0, I2C_CR2_RELOAD);
}

/************************************************************************************
 * Name: stm32_i2c_disable_reload
 *
 * Description:
 *
 ************************************************************************************/

static inline void
stm32_i2c_disable_reload(FAR struct stm32_i2c_priv_s *priv)
{
  stm32_i2c_modifyreg32(priv, STM32_I2C_CR2_OFFSET, I2C_CR2_RELOAD, 0);
}

/************************************************************************************
 * Name: stm32_i2c_sem_waitstop
 *
 * Description:
 *   Wait for a STOP to complete
 *
 ************************************************************************************/

static inline void stm32_i2c_sem_waitstop(FAR struct stm32_i2c_priv_s *priv)
{
  clock_t start;
  clock_t elapsed;
  clock_t timeout;
  uint32_t cr;
  uint32_t sr;

  /* Select a timeout */

#ifdef CONFIG_STM32F7_I2C_DYNTIMEO
  timeout = USEC2TICK(CONFIG_STM32F7_I2C_DYNTIMEO_STARTSTOP);
#else
  timeout = CONFIG_STM32F7_I2CTIMEOTICKS;
#endif

  /* Wait as stop might still be in progress */

  start = clock_systime_ticks();
  do
    {
      /* Calculate the elapsed time */

      elapsed = clock_systime_ticks() - start;

      /* Check for STOP condition */

      cr = stm32_i2c_getreg32(priv, STM32_I2C_CR2_OFFSET);
      if ((cr & I2C_CR2_STOP) == 0)
        {
          return;
        }

      /* Check for timeout error */

      sr = stm32_i2c_getreg(priv, STM32_I2C_ISR_OFFSET);
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

  i2cinfo("Timeout with CR: %04x SR: %04x\n", cr, sr);
}

/************************************************************************************
 * Name: stm32_i2c_sem_post
 *
 * Description:
 *   Release the mutual exclusion semaphore
 *
 ************************************************************************************/

static inline void stm32_i2c_sem_post(FAR struct i2c_master_s *dev)
{
  nxsem_post(&((struct stm32_i2c_inst_s *)dev)->priv->sem_excl);
}

/************************************************************************************
 * Name: stm32_i2c_sem_init
 *
 * Description:
 *   Initialize semaphores
 *
 ************************************************************************************/

static inline void stm32_i2c_sem_init(FAR struct i2c_master_s *dev)
{
  nxsem_init(&((struct stm32_i2c_inst_s *)dev)->priv->sem_excl, 0, 1);

#ifndef CONFIG_I2C_POLLED
  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&((struct stm32_i2c_inst_s *)dev)->priv->sem_isr, 0, 0);
  nxsem_set_protocol(&((struct stm32_i2c_inst_s *)dev)->priv->sem_isr, SEM_PRIO_NONE);
#endif
}

/************************************************************************************
 * Name: stm32_i2c_sem_destroy
 *
 * Description:
 *   Destroy semaphores.
 *
 ************************************************************************************/

static inline void stm32_i2c_sem_destroy(FAR struct i2c_master_s *dev)
{
  nxsem_destroy(&((struct stm32_i2c_inst_s *)dev)->priv->sem_excl);
#ifndef CONFIG_I2C_POLLED
  nxsem_destroy(&((struct stm32_i2c_inst_s *)dev)->priv->sem_isr);
#endif
}

/************************************************************************************
 * Name: stm32_i2c_trace*
 *
 * Description:
 *   I2C trace instrumentation
 *
 ************************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void stm32_i2c_traceclear(FAR struct stm32_i2c_priv_s *priv)
{
  struct stm32_trace_s *trace = &priv->trace[priv->tndx];

  trace->status = 0;              /* I2C 32-bit status */
  trace->count  = 0;              /* Interrupt count when status change */
  trace->event  = I2CEVENT_NONE;  /* Last event that occurred with this status */
  trace->parm   = 0;              /* Parameter associated with the event */
  trace->time   = 0;              /* Time of first status or event */
}

static void stm32_i2c_tracereset(FAR struct stm32_i2c_priv_s *priv)
{
  /* Reset the trace info for a new data collection */

  priv->tndx       = 0;
  priv->start_time = clock_systime_ticks();
  stm32_i2c_traceclear(priv);
}

static void stm32_i2c_tracenew(FAR struct stm32_i2c_priv_s *priv,
                               uint32_t status)
{
  struct stm32_trace_s *trace = &priv->trace[priv->tndx];

  /* Is the current entry uninitialized?   Has the status changed? */

  if (trace->count == 0 || status != trace->status)
    {
      /* Yes.. Was it the status changed?  */

      if (trace->count != 0)
        {
          /* Yes.. bump up the trace index (unless we are out of trace entries) */

          if (priv->tndx >= (CONFIG_I2C_NTRACE - 1))
            {
              i2cerr("ERROR: Trace table overflow\n");
              return;
            }

          priv->tndx++;
          trace = &priv->trace[priv->tndx];
        }

      /* Initialize the new trace entry */

      stm32_i2c_traceclear(priv);
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

static void stm32_i2c_traceevent(FAR struct stm32_i2c_priv_s *priv,
                                enum stm32_trace_e event, uint32_t parm)
{
  struct stm32_trace_s *trace;

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
      stm32_i2c_traceclear(priv);
    }
}

static void stm32_i2c_tracedump(FAR struct stm32_i2c_priv_s *priv)
{
  struct stm32_trace_s *trace;
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

/************************************************************************************
 * Name: stm32_i2c_setclock
 *
 * Description:
 *
 *   Sets the I2C bus clock frequency by configuring the I2C_TIMINGR register.
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
 *   fast+) are the bus clock speed and setup/hold times.  Setup/hold times are
 *   specified as a MINIMUM time for the given mode, and naturally std mode
 *   has the longest minimum times.  As a result, by provisioning setup/hold
 *   times for std mode they are also compatible with fast/fast+, though some
 *   performance degradation occurs in fast/fast+ as a result of the times
 *   being somewhat longer than strictly required.  The values remain as they
 *   are because reliability is favored over performance.
 *
 * Clock Selection:
 *
 *   The I2C peripheral clock can be provided by either PCLK1, SYSCLK or the HSI.
 *
 *    PCLK1 >------|\   I2CCLK
 *   SYSCLK >------| |--------->
 *      HSI >------|/
 *
 *   HSI is the default and is always 16Mhz.
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
 ************************************************************************************/

static void stm32_i2c_setclock(FAR struct stm32_i2c_priv_s *priv, uint32_t frequency)
{
  uint8_t presc;
  uint8_t scl_delay;
  uint8_t sda_delay;
  uint8_t scl_h_period;
  uint8_t scl_l_period;

  /* I2C peripheral must be disabled to update clocking configuration.
   * This will SW reset the device.
   */

  stm32_i2c_modifyreg32(priv, STM32_I2C_CR1_OFFSET, I2C_CR1_PE, 0);

  if (frequency != priv->frequency)
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

      uint32_t timingr =
        (presc << I2C_TIMINGR_PRESC_SHIFT) |
        (scl_delay << I2C_TIMINGR_SCLDEL_SHIFT) |
        (sda_delay << I2C_TIMINGR_SDADEL_SHIFT) |
        (scl_h_period << I2C_TIMINGR_SCLH_SHIFT) |
        (scl_l_period << I2C_TIMINGR_SCLL_SHIFT);

      stm32_i2c_putreg32(priv, STM32_I2C_TIMINGR_OFFSET, timingr);
      priv->frequency = frequency;
    }

  /* Enable I2C peripheral */

  stm32_i2c_modifyreg32(priv, STM32_I2C_CR1_OFFSET, 0, I2C_CR1_PE);
}

/************************************************************************************
 * Name: stm32_i2c_sendstart
 *
 * Description:
 *   Send the START condition / force Master mode
 *
 *   A START condition in I2C consists of a single byte that contains both the
 *   7 bit slave address and a read/write bit (0 = WRITE, 1 = READ).  If the
 *   address is recognized by one of the slave devices that slave device will
 *   ACK the byte so that data transfers can begin.
 *
 *   A RESTART (or repeated START per the I2CSPEC) is simply a START condition
 *   issued in the middle of a transfer (i.e. after the initial START and before
 *   a STOP).  A RESTART sends a new address byte and R/W bit to the bus. A
 *   RESTART is optional in most cases but mandatory in the event the transfer
 *   direction is changed.
 *
 *   Most of the time reading data from an I2C slave requires a WRITE of the
 *   subaddress followed by a READ (and hence a RESTART in between).  Writing
 *   to an I2C slave typically requires only WRITE operations and hence no
 *   RESTARTs.
 *
 *   This function is therefore called both at the beginning of a transfer
 *   (START) and at appropriate times during a transfer (RESTART).
 *
 ************************************************************************************/

static inline void stm32_i2c_sendstart(FAR struct stm32_i2c_priv_s *priv)
{
  bool next_norestart = false;

  /* Set the private "current message" data used in protocol processing.
   *
   * ptr:   A pointer to the start of the current message buffer.  This is
   *        advanced after each byte in the current message is transferred.
   *
   * dcnt:  A running counter of the bytes in the current message waiting to be
   *        transferred.  This is decremented each time a byte is transferred.
   *        The hardware normally accepts a maximum of 255 bytes per transfer
   *        but can support more via the RELOAD mechanism.  If dcnt initially
   *        exceeds 255, the RELOAD mechanism will be enabled automatically.
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

  if ((priv->flags & I2C_M_NOSTART) == 0)
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
      stm32_i2c_enable_reload(priv);
    }
  else
    {
      i2cinfo("RELOAD disable: dcnt = %i msgc = %i\n",
          priv->dcnt, priv->msgc);
      stm32_i2c_disable_reload(priv);
    }

  /* Set the number of bytes to transfer (I2C_CR2->NBYTES) to the number of
   * bytes in the current message or 255, whichever is lower so as to not
   * exceed the hardware maximum allowed.
   */

  if (priv->dcnt > 255)
    {
      stm32_i2c_set_bytes_to_transfer(priv, 255);
    }
  else
    {
      stm32_i2c_set_bytes_to_transfer(priv, priv->dcnt);
    }

  /* Set the (7 bit) address.
   * 10 bit addressing is not yet supported.
   */

  stm32_i2c_set_7bit_address(priv);

  /* The flag of the current message is used to determine the direction of
   * transfer required for the current message.
   */

  if (priv->flags & I2C_M_READ)
    {
      stm32_i2c_set_read_transfer_dir(priv);
    }
  else
    {
      stm32_i2c_set_write_transfer_dir(priv);
    }

  /* Set the I2C_CR2->START bit to 1 to instruct the hardware to send the
   * START condition using the address and transfer direction data entered.
   */

  i2cinfo("Sending START: dcnt=%i msgc=%i flags=0x%04x\n",
          priv->dcnt, priv->msgc, priv->flags);

  stm32_i2c_modifyreg32(priv, STM32_I2C_CR2_OFFSET, 0, I2C_CR2_START);
}

/************************************************************************************
 * Name: stm32_i2c_sendstop
 *
 * Description:
 *   Send the STOP conditions
 *
 *   A STOP condition can be requested by setting the STOP bit in the I2C_CR2
 *   register. Setting the STOP bit clears the TC flag and the STOP condition is
 *   sent on the bus.
 *
 ************************************************************************************/

static inline void stm32_i2c_sendstop(FAR struct stm32_i2c_priv_s *priv)
{
  i2cinfo("Sending STOP\n");
  stm32_i2c_traceevent(priv, I2CEVENT_WRITE_STOP, 0);

  stm32_i2c_modifyreg32(priv, STM32_I2C_CR2_OFFSET, 0, I2C_CR2_STOP);
}

/************************************************************************************
 * Name: stm32_i2c_getstatus
 *
 * Description:
 *   Get 32-bit status (SR1 and SR2 combined)
 *
 ************************************************************************************/

static inline uint32_t stm32_i2c_getstatus(FAR struct stm32_i2c_priv_s *priv)
{
  return getreg32(priv->config->base + STM32_I2C_ISR_OFFSET);
}

/************************************************************************************
 * Name: stm32_i2c_clearinterrupts
 *
 * Description:
 *  Clear all interrupts
 *
 ************************************************************************************/

static inline void stm32_i2c_clearinterrupts(struct stm32_i2c_priv_s *priv)
{
  stm32_i2c_modifyreg32(priv, STM32_I2C_ICR_OFFSET, 0, I2C_ICR_CLEARMASK);
}

/************************************************************************************
 * Name: stm32_i2c_isr_process
 *
 * Description:
 *  Common interrupt service routine (ISR) that handles I2C protocol logic.
 *  This is instantiated for each configured I2C interface (I2C1, I2C2, I2C3).
 *
 *  This ISR is activated and deactivated by:
 *
 *   stm32_i2c_process
 *    and
 *   stm32_i2c_waitdone
 *
 * Input Parameters:
 *   priv - The private struct of the I2C driver.
 *
 ************************************************************************************/

static int stm32_i2c_isr_process(struct stm32_i2c_priv_s *priv)
{
  uint32_t status;

  /* Get state of the I2C controller */

  status = stm32_i2c_getreg32(priv, STM32_I2C_ISR_OFFSET);

  i2cinfo("ENTER: status = 0x%08x\n", status);

  /* Update private version of the state assuming a good state */

  priv->status = status & ~I2C_INT_BAD_STATE;

  /* If this is a new transmission set up the trace table accordingly */

  stm32_i2c_tracenew(priv, status);
  stm32_i2c_traceevent(priv, I2CEVENT_ISR_CALL, 0);

  /* --------------------- Start of I2C protocol handling -------------------- */

  /* I2C protocol logic follows. It's organized in an if else chain such that
   * only one mode of operation is executed every time the ISR is called.
   *
   * If you need to add additional states to support new features be sure they
   * continue the chain (i.e. begin with "else if") and are placed before the
   * empty call / error states at the end of the chain.
   */

  /* NACK Handling
   *
   * This branch is only triggered when the NACK (Not Acknowledge Received)
   * interrupt occurs.  This interrupt will only fire when the I2C_CR1->NACKIE
   * bit is 1.
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

          i2cinfo("NACK: Address invalid: dcnt=%i msgc=%i status=0x%08x\n",
          priv->dcnt, priv->msgc, status);
          stm32_i2c_traceevent(priv, I2CEVENT_ADDRESS_NACKED, priv->msgv->addr);
        }
      else
        {
          /* NACK received on regular byte */

          i2cinfo("NACK: NACK received: dcnt=%i msgc=%i status=0x%08x\n",
          priv->dcnt, priv->msgc, status);
          stm32_i2c_traceevent(priv, I2CEVENT_ADDRESS_NACKED, priv->msgv->addr);
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
   * following the successful transmission of a byte and slave acknowledgment.
   * In this state the I2C_TXDR register is ready to accept another byte for
   * transmission.  The TXIS bit will be cleared automatically when the next
   * byte is written to I2C_TXDR.
   *
   * The number of TXIS events during the transfer corresponds to NBYTES.
   *
   * The TXIS flag is not set when a NACK is received.
   *
   * When RELOAD is disabled (RELOAD=0) and NBYTES data have been transferred:
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

  else if ((priv->flags & (I2C_M_READ)) == 0 && (status & (I2C_ISR_TXIS)) != 0)
    {
      /* TXIS interrupt occurred, address valid, ready to transmit */

      stm32_i2c_traceevent(priv, I2CEVENT_WRITE, 0);
      i2cinfo("TXIS: ENTER dcnt = %i msgc = %i status 0x%08x\n",
              priv->dcnt, priv->msgc, status);

      /* The first event after the address byte is sent will be either TXIS
       * or NACKF so it's safe to set the astart flag to false on
       * the first TXIS event to indicate that it is no longer necessary to
       * check for address validity.
       */

      if (priv->astart == true)
        {
          i2cinfo("TXIS: Address Valid\n");
          stm32_i2c_traceevent(priv, I2CEVENT_ADDRESS_ACKED, priv->msgv->addr);
          priv->astart = false;
        }

      /* If one or more bytes in the current message are ready to transmit */

      if (priv->dcnt > 0)
        {
          /* Prepare to transmit the current byte */

          stm32_i2c_traceevent(priv, I2CEVENT_WRITE_TO_DR, priv->dcnt);
          i2cinfo("TXIS: Write Data 0x%02x\n", *priv->ptr);

          /* Decrement byte counter */

          priv->dcnt--;

          /* If we are about to transmit the last byte in the current message */

          if (priv->dcnt == 0)
            {
              /* If this is also the last message to send, disable RELOAD so
               * TC fires next and issues STOP condition.  If we don't do this
               * TCR will fire next, and since there are no bytes to send we
               * can't write NBYTES to clear TCR so it will fire forever.
               */

              if (priv->msgc == 1)
                {
                  stm32_i2c_disable_reload(priv);
                }
            }

          /* Transmit current byte */

          stm32_i2c_putreg(priv, STM32_I2C_TXDR_OFFSET, *priv->ptr);

          /* Advance to next byte */

          priv->ptr++;
        }
      else
        {
          /* Unsupported state */

          i2cerr("ERROR: TXIS Unsupported state detected, dcnt=%i, status 0x%08x\n",
          priv->dcnt, status);
          stm32_i2c_traceevent(priv, I2CEVENT_WRITE_ERROR, 0);

          /* Indicate the bad state, so that on termination HW will be reset */

          priv->status |= I2C_INT_BAD_STATE;
        }

      i2cinfo("TXIS: EXIT  dcnt = %i msgc = %i status 0x%08x\n",
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
   * When RELOAD is disabled and bytes remain to be transferred an acknowledge
   * is automatically sent on the bus and the RXNE events continue until the
   * last byte is received.
   *
   * When RELOAD is disabled (RELOAD=0) and BYTES have been transferred:
   *
   *   - In Automatic End Mode (AUTOEND=1), a NACK and a STOP are automatically
   *     sent after the last received byte.
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

      stm32_i2c_traceevent(priv, I2CEVENT_READ, 0);
      i2cinfo("RXNE: ENTER dcnt = %i msgc = %i status 0x%08x\n",
              priv->dcnt, priv->msgc, status);

      /* If more bytes in the current message */

      if (priv->dcnt > 0)
        {
          stm32_i2c_traceevent(priv, I2CEVENT_RCVBYTE, priv->dcnt);

          /* No interrupts or context switches may occur in the following
           * sequence.  Otherwise, additional bytes may be received.
           */

#ifdef CONFIG_I2C_POLLED
          irqstate_t state = enter_critical_section();
#endif
          /* Receive a byte */

          *priv->ptr = stm32_i2c_getreg(priv, STM32_I2C_RXDR_OFFSET);

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

          stm32_i2c_traceevent(priv, I2CEVENT_READ_ERROR, 0);
          status = stm32_i2c_getreg(priv, STM32_I2C_ISR_OFFSET);
          i2cerr("ERROR: RXNE Unsupported state detected, dcnt=%i, status 0x%08x\n",
                 priv->dcnt, status);

          /* Set signals that will terminate ISR and wake waiting thread */

          priv->status |= I2C_INT_BAD_STATE;
          priv->dcnt = -1;
          priv->msgc = 0;
        }

      i2cinfo("RXNE: EXIT  dcnt = %i msgc = %i status 0x%08x\n",
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
   * in NBYTES, meaning, the number of bytes in the current message (priv->dcnt)
   * has been successfully transmitted or received.
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
      i2cinfo("TC: ENTER dcnt = %i msgc = %i status 0x%08x\n",
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
          stm32_i2c_traceevent(priv, I2CEVENT_TC_NO_RESTART, priv->msgc);

          /* Issue a START condition.
           *
           * Note that the first thing sendstart does is update the
           * private structure "current message" data (ptr, dcnt, flags)
           * so they all reflect the next message in the list so we
           * update msgv before we get there.
           */

          /* Advance to the next message in the list */

          priv->msgv++;

          stm32_i2c_sendstart(priv);
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
          stm32_i2c_traceevent(priv, I2CEVENT_STOP, priv->dcnt);

          stm32_i2c_sendstop(priv);

          /* Set signals that will terminate ISR and wake waiting thread */

          priv->dcnt = -1;
          priv->msgc = 0;
        }

      i2cinfo("TC: EXIT dcnt = %i msgc = %i status 0x%08x\n",
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
   *  1) We're trying to send a message with a payload greater than 255 bytes.
   *  2) We're trying to send messages back to back, regardless of their
   *     payload size, to avoid a RESTART (i.e. I2C_M_NOSTART flag is set).
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
   * direction changes.  Right now the NORESTART flag overrides this behavior.
   * May have to introduce logic to issue sendstart, assuming it's legal
   * with the hardware in the TCR state.
   */

  else if ((status & I2C_ISR_TCR) != 0)
    {
      i2cinfo("TCR: ENTER dcnt = %i msgc = %i status 0x%08x\n",
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

              stm32_i2c_disable_reload(priv);
            }

          /* Update NBYTES with length of current message */

          i2cinfo("TCR: NEXT MSG dcnt = %i msgc = %i\n",
                  priv->dcnt, priv->msgc);

          stm32_i2c_set_bytes_to_transfer(priv, priv->dcnt);
        }
      else
        {
          /* More bytes in the current (greater than 255 byte payload
           * length) message, so set NBYTES according to the bytes
           * remaining in the message, up to a maximum each cycle of 255.
           */

          if (priv->dcnt > 255)
            {
              i2cinfo("TCR: ENABLE RELOAD: NBYTES = 255 dcnt = %i msgc = %i\n",
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

              stm32_i2c_enable_reload(priv);

              stm32_i2c_set_bytes_to_transfer(priv, 255);
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

              stm32_i2c_disable_reload(priv);

              stm32_i2c_set_bytes_to_transfer(priv, priv->dcnt);
            }

          i2cinfo("TCR: EXIT dcnt = %i msgc = %i status 0x%08x\n",
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
      status = stm32_i2c_getreg(priv, STM32_I2C_ISR_OFFSET);
      i2cwarn("WARNING: EMPTY CALL: Stopping ISR: status 0x%08x\n", status);
      stm32_i2c_traceevent(priv, I2CEVENT_ISR_EMPTY_CALL, 0);
    }

  /* Error handler
   *
   * We get to this branch only if we can't handle the current state.
   *
   * This can happen in interrupt based operation on ARLO & BUSY.
   *
   * This will happen during polled operation when the device is not
   * in one of the supported states when polled.
   */

  else
    {
#ifdef CONFIG_I2C_POLLED
      stm32_i2c_traceevent(priv, I2CEVENT_POLL_DEV_NOT_RDY, 0);
#else
      /* Read rest of the state */

      status = stm32_i2c_getreg(priv, STM32_I2C_ISR_OFFSET);

      i2cerr("ERROR: Invalid state detected, status 0x%08x\n", status);

      /* set condition to terminate ISR and wake waiting thread */

      priv->status |= I2C_INT_BAD_STATE;
      priv->dcnt = -1;
      priv->msgc = 0;
      stm32_i2c_traceevent(priv, I2CEVENT_STATE_ERROR, 0);
#endif
    }

  /* --------------------- End of I2C protocol handling -------------------- */

  /* Message Handling
   *
   * Transmission of the whole message chain has been completed. We have to
   * terminate the ISR and wake up stm32_i2c_process() that is waiting for
   * the ISR cycle to handle the sending/receiving of the messages.
   */

  if (priv->dcnt == -1 && priv->msgc == 0)
    {
      i2cinfo("MSG: Shutting down I2C ISR\n");

      stm32_i2c_traceevent(priv, I2CEVENT_ISR_SHUTDOWN, 0);

      /* clear pointer to message content to reflect we are done
       * with the current transaction.
       */

      priv->msgv = NULL;

#ifdef CONFIG_I2C_POLLED
      priv->intstate = INTSTATE_DONE;
#else

      /* We will update private state to capture NACK which is used in
       * combination with the astart flag to report the type of NACK received
       * (address vs data) to the upper layers once we exit the ISR.
       *
       * Note: status is captured  prior to clearing interrupts because
       * the NACKF flag will naturally be cleared by that process.
       */

      status = stm32_i2c_getreg32(priv, STM32_I2C_ISR_OFFSET);

      /* Clear all interrupts */

      stm32_i2c_modifyreg32(priv, STM32_I2C_ICR_OFFSET, 0, I2C_ICR_CLEARMASK);

      /* Was a bad state detected in the processing? */

      if (priv->status & I2C_INT_BAD_STATE)
        {
          /* SW reset device  */

          stm32_i2c_modifyreg32(priv, STM32_I2C_CR1_OFFSET, I2C_CR1_PE, 0);
        }

      /* Update private status from above sans I2C_INT_BAD_STATE */

      priv->status = status;

      /* If a thread is waiting then inform it transfer is complete */

      if (priv->intstate == INTSTATE_WAITING)
        {
          nxsem_post(&priv->sem_isr);
          priv->intstate = INTSTATE_DONE;
        }
#endif
    }

  status = stm32_i2c_getreg32(priv, STM32_I2C_ISR_OFFSET);
  i2cinfo("EXIT: status = 0x%08x\n", status);

  return OK;
}

/************************************************************************************
 * Name: stm32_i2c_isr
 *
 * Description:
 *   Common I2C interrupt service routine
 *
 ************************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int stm32_i2c_isr(int irq, void *context, FAR void *arg)
{
  struct stm32_i2c_priv_s *priv = (struct stm32_i2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);
  return stm32_i2c_isr_process(priv);
}
#endif

/************************************************************************************
 * Name: stm32_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ************************************************************************************/

static int stm32_i2c_init(FAR struct stm32_i2c_priv_s *priv)
{
  /* Power-up and configure GPIOs */

  /* Enable power and reset the peripheral */

  modifyreg32(STM32_RCC_APB1ENR, 0, priv->config->clk_bit);
  modifyreg32(STM32_RCC_APB1RSTR, 0, priv->config->reset_bit);
  modifyreg32(STM32_RCC_APB1RSTR, priv->config->reset_bit, 0);

  /* Configure pins */

  if (stm32_configgpio(priv->config->scl_pin) < 0)
    {
      return ERROR;
    }

  if (stm32_configgpio(priv->config->sda_pin) < 0)
    {
      stm32_unconfiggpio(priv->config->scl_pin);
      return ERROR;
    }

#ifndef CONFIG_I2C_POLLED
  /* Attach error and event interrupts to the ISRs */

  irq_attach(priv->config->ev_irq, stm32_i2c_isr, priv);
  irq_attach(priv->config->er_irq, stm32_i2c_isr, priv);
  up_enable_irq(priv->config->ev_irq);
  up_enable_irq(priv->config->er_irq);
#endif

  /* TODO:
   * - Provide means to set peripheral clock source via RCC_CFGR3_I2CxSW
   * - Set to HSI by default, make Kconfig option
   */

  /* Force a frequency update */

  priv->frequency = 0;
  stm32_i2c_setclock(priv, 100000);

  return OK;
}

/************************************************************************************
 * Name: stm32_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ************************************************************************************/

static int stm32_i2c_deinit(FAR struct stm32_i2c_priv_s *priv)
{
  /* Disable I2C */

  stm32_i2c_putreg32(priv, STM32_I2C_CR1_OFFSET, 0);

  /* Unconfigure GPIO pins */

  stm32_unconfiggpio(priv->config->scl_pin);
  stm32_unconfiggpio(priv->config->sda_pin);

#ifndef CONFIG_I2C_POLLED

  /* Disable and detach interrupts */

  up_disable_irq(priv->config->ev_irq);
  up_disable_irq(priv->config->er_irq);
  irq_detach(priv->config->ev_irq);
  irq_detach(priv->config->er_irq);
#endif

  /* Disable clocking */

  modifyreg32(STM32_RCC_APB1ENR, priv->config->clk_bit, 0);

  return OK;
}

/************************************************************************************
 * Name: stm32_i2c_process
 *
 * Description:
 *   Common I2C transfer logic
 *
 *   Initiates a master mode transaction on the I2C bus to transfer the provided
 *   messages to and from the slave devices.
 *
 ************************************************************************************/

static int stm32_i2c_process(FAR struct i2c_master_s *dev,
                             FAR struct i2c_msg_s *msgs, int count)
{
  struct stm32_i2c_inst_s     *inst = (struct stm32_i2c_inst_s *)dev;
  FAR struct stm32_i2c_priv_s *priv = inst->priv;
  uint32_t    status = 0;
  uint32_t    cr1;
  uint32_t    cr2;
  int         errval = 0;
  int         waitrc = 0;

  DEBUGASSERT(count > 0);

  /* Wait for any STOP in progress */

  stm32_i2c_sem_waitstop(priv);

  /* Clear any pending error interrupts */

  stm32_i2c_clearinterrupts(priv);

  /* Old transfers are done */

  priv->msgv = msgs;
  priv->msgc = count;

  /* Reset I2C trace logic */

  stm32_i2c_tracereset(priv);

  /* Set I2C clock frequency toggles I2C_CR1_PE performing a SW reset! */

  stm32_i2c_setclock(priv, msgs->frequency);

  /* Trigger start condition, then the process moves into the ISR.  I2C
   * interrupts will be enabled within stm32_i2c_waitdone().
   */

  priv->status = 0;

#ifndef CONFIG_I2C_POLLED
  /* Enable transmit and receive interrupts here so when we send the start
   * condition below the ISR will fire if the data was sent and some
   * response from the slave received.  All other interrupts relevant to
   * our needs are enabled in stm32_i2c_sem_waitdone() below.
   */

  stm32_i2c_enableinterrupts(priv);
#endif

  /* Trigger START condition generation, which also sends the slave address
   * with read/write flag and the data in the first message
   */

  stm32_i2c_sendstart(priv);

  /* Wait for the ISR to tell us that the transfer is complete by attempting
   * to grab the semaphore that is initially locked by the ISR.  If the ISR
   * does not release the lock so we can obtain it here prior to the end of
   * the timeout period waitdone returns error and we report a timeout.
   */

  waitrc = stm32_i2c_sem_waitdone(priv);

  cr1 = stm32_i2c_getreg32(priv, STM32_I2C_CR1_OFFSET);
  cr2 = stm32_i2c_getreg32(priv, STM32_I2C_CR2_OFFSET);
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

  status = stm32_i2c_getstatus(priv);

  /* The priv->status field can hold additional information like a NACK
   * event so we include that information.
   */

  status = priv->status & 0xffffffff;

  if (waitrc < 0)
    {
      /* Connection timed out */

      errval = ETIMEDOUT;
      i2cerr("ERROR: Waitdone timed out CR1: 0x%08x CR2: 0x%08x status: 0x%08x\n",
             cr1, cr2, status);
    }
  else
    {
      i2cinfo("Waitdone success: CR1: 0x%08x CR2: 0x%08x status: 0x%08x\n",
             cr1, cr2, status);
    }

  UNUSED(cr1);
  UNUSED(cr2);

  i2cinfo("priv->status: 0x%08x\n", priv->status);

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
   * NOTE:  We will only see this busy indication if stm32_i2c_sem_waitdone()
   * fails above;  Otherwise it is cleared.
   */

  else if ((status & I2C_ISR_BUSY) != 0)
    {
      /* I2C Bus Busy
       *
       * This is a status condition rather than an error.
       *
       * We will only see this busy indication if stm32_i2c_sem_waitdone()
       * fails above;  Otherwise it is cleared by the hardware when the ISR
       * wraps up the transfer with a STOP condition.
       */

      clock_t start = clock_systime_ticks();
      clock_t timeout = USEC2TICK(USEC_PER_SEC / priv->frequency) + 1;

      status = stm32_i2c_getstatus(priv);

      while (status & I2C_ISR_BUSY)
        {
          if ((clock_systime_ticks() - start) > timeout)
            {
              i2cerr("ERROR: I2C Bus busy");
              errval = EBUSY;
              break;
            }

          status = stm32_i2c_getstatus(priv);
        }
    }

  /* Dump the trace result */

  stm32_i2c_tracedump(priv);
  stm32_i2c_sem_post(dev);

  return -errval;
}

/************************************************************************************
 * Name: stm32_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ************************************************************************************/

static int stm32_i2c_transfer(FAR struct i2c_master_s *dev,
                              FAR struct i2c_msg_s *msgs, int count)
{
  int ret;

  /* Ensure that address or flags don't change meanwhile */

  ret = nxsem_wait(&((struct stm32_i2c_inst_s *)dev)->priv->sem_excl);
  if (ret >= 0)
    {
      ret = stm32_i2c_process(dev, msgs, count);
    }

  return ret;
}

/************************************************************************************
 * Name: stm32_i2c_reset
 *
 * Description:
 *   Reset an I2C bus
 *
 ************************************************************************************/

#ifdef CONFIG_I2C_RESET
static int stm32_i2c_reset(FAR struct i2c_master_s * dev)
{
  struct stm32_i2c_priv_s * priv;
  unsigned int clock_count;
  unsigned int stretch_count;
  uint32_t scl_gpio;
  uint32_t sda_gpio;
  uint32_t frequency;
  int ret;

  DEBUGASSERT(dev);

  /* Get I2C private structure */

  priv = ((struct stm32_i2c_inst_s *)dev)->priv;

  /* Our caller must own a ref */

  DEBUGASSERT(priv->refs > 0);

  /* Lock out other clients */

  ret = nxsem_wait_uninterruptible(&priv->sem_excl);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EIO;

  /* Save the current frequency */

  frequency = priv->frequency;

  /* De-init the port */

  stm32_i2c_deinit(priv);

  /* Use GPIO configuration to un-wedge the bus */

  scl_gpio = MKI2C_OUTPUT(priv->config->scl_pin);
  sda_gpio = MKI2C_OUTPUT(priv->config->sda_pin);

  stm32_configgpio(sda_gpio);
  stm32_configgpio(scl_gpio);

  /* Let SDA go high */

  stm32_gpiowrite(sda_gpio, 1);

  /* Clock the bus until any slaves currently driving it let it go. */

  clock_count = 0;
  while (!stm32_gpioread(sda_gpio))
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
      while (!stm32_gpioread(scl_gpio))
        {
          /* Give up if we have tried too hard */

          if (stretch_count++ > 10)
            {
              goto out;
            }

          up_udelay(10);
        }

      /* Drive SCL low */

      stm32_gpiowrite(scl_gpio, 0);
      up_udelay(10);

      /* Drive SCL high again */

      stm32_gpiowrite(scl_gpio, 1);
      up_udelay(10);
    }

  /* Generate a start followed by a stop to reset slave
   * state machines.
   */

  stm32_gpiowrite(sda_gpio, 0);
  up_udelay(10);
  stm32_gpiowrite(scl_gpio, 0);
  up_udelay(10);
  stm32_gpiowrite(scl_gpio, 1);
  up_udelay(10);
  stm32_gpiowrite(sda_gpio, 1);
  up_udelay(10);

  /* Revert the GPIO configuration. */

  stm32_unconfiggpio(sda_gpio);
  stm32_unconfiggpio(scl_gpio);

  /* Re-init the port */

  stm32_i2c_init(priv);

  /* Restore the frequency */

  stm32_i2c_setclock(priv, frequency);
  ret = OK;

out:

  /* Release the port for re-use by other clients */

  stm32_i2c_sem_post(dev);
  return ret;
}
#endif /* CONFIG_I2C_RESET */

/************************************************************************************
 * Name: stm32_i2c_pm_prepare
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
 ************************************************************************************/

#ifdef CONFIG_PM
static int stm32_i2c_pm_prepare(FAR struct pm_callback_s *cb, int domain,
                                enum pm_state_e pmstate)
{
  struct stm32_i2c_priv_s *priv =
      (struct stm32_i2c_priv_s *)((char *)cb -
                                    offsetof(struct stm32_i2c_priv_s, pm_cb));
  int sval;

  /* Logic to prepare for a reduced power state goes here. */

  switch (pmstate)
    {
    case PM_NORMAL:
    case PM_IDLE:
      break;

    case PM_STANDBY:
    case PM_SLEEP:

      /* Check if exclusive lock for I2C bus is held. */

      if (nxsem_get_value(&priv->sem_excl, &sval) < 0)
        {
          DEBUGASSERT(false);
          return -EINVAL;
        }

      if (sval <= 0)
        {
          /* Exclusive lock is held, do not allow entry to deeper PM states. */

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

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ************************************************************************************/

FAR struct i2c_master_s *stm32_i2cbus_initialize(int port)
{
  struct stm32_i2c_priv_s * priv = NULL;  /* private data of device with multiple instances */
  struct stm32_i2c_inst_s * inst = NULL;  /* device, single instance */
  irqstate_t irqs;
#ifdef CONFIG_PM
  int ret;
#endif

#if STM32_HSI_FREQUENCY != 16000000 || defined(INVALID_CLOCK_SOURCE)
# warning STM32_I2C_INIT: Peripheral clock is HSI and it must be 16mHz or the speed/timing calculations need to be redone.
  return NULL;
#endif

  /* Get I2C private structure */

  switch (port)
    {
#ifdef CONFIG_STM32F7_I2C1
      case 1:
        priv = (struct stm32_i2c_priv_s *)&stm32_i2c1_priv;
        break;
#endif
#ifdef CONFIG_STM32F7_I2C2
      case 2:
        priv = (struct stm32_i2c_priv_s *)&stm32_i2c2_priv;
        break;
#endif
#ifdef CONFIG_STM32F7_I2C3
      case 3:
        priv = (struct stm32_i2c_priv_s *)&stm32_i2c3_priv;
        break;
#endif
#ifdef CONFIG_STM32F7_I2C4
      case 4:
        priv = (struct stm32_i2c_priv_s *)&stm32_i2c4_priv;
        break;
#endif
      default:
        return NULL;
    }

  /* Allocate instance */

  if (!(inst = kmm_malloc(sizeof(struct stm32_i2c_inst_s))))
    {
      return NULL;
    }

  /* Initialize instance */

  inst->ops       = &stm32_i2c_ops;
  inst->priv      = priv;

  /* Init private data for the first time, increment refs count,
   * power-up hardware and configure GPIOs.
   */

  irqs = enter_critical_section();

  if ((volatile int)priv->refs++ == 0)
    {
      stm32_i2c_sem_init((struct i2c_master_s *)inst);
      stm32_i2c_init(priv);

#ifdef CONFIG_PM
      /* Register to receive power management callbacks */

      ret = pm_register(&priv->pm_cb);
      DEBUGASSERT(ret == OK);
      UNUSED(ret);
#endif
    }

  leave_critical_section(irqs);
  return (struct i2c_master_s *)inst;
}

/************************************************************************************
 * Name: stm32_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ************************************************************************************/

int stm32_i2cbus_uninitialize(FAR struct i2c_master_s * dev)
{
  irqstate_t irqs;

  DEBUGASSERT(dev);

  /* Decrement refs and check for underflow */

  if (((struct stm32_i2c_inst_s *)dev)->priv->refs == 0)
    {
      return ERROR;
    }

  irqs = enter_critical_section();

  if (--((struct stm32_i2c_inst_s *)dev)->priv->refs)
    {
      leave_critical_section(irqs);
      kmm_free(dev);
      return OK;
    }

  leave_critical_section(irqs);

#ifdef CONFIG_PM
  /* Unregister power management callbacks */

  pm_unregister(&((struct stm32_i2c_inst_s *)dev)->priv->pm_cb);
#endif

  /* Disable power and other HW resource (GPIO's) */

  stm32_i2c_deinit(((struct stm32_i2c_inst_s *)dev)->priv);

  /* Release unused resources */

  stm32_i2c_sem_destroy((struct i2c_master_s *)dev);

  kmm_free(dev);
  return OK;
}

#endif /* CONFIG_STM32F7_I2C1 || CONFIG_STM32F7_I2C2 || \
        * CONFIG_STM32F7_I2C3 || CONFIG_STM32F7_I2C4 */
