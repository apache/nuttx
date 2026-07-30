/****************************************************************************
 * arch/arm/src/common/ameba/ameba_gpio.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* NuttX GPIO (ioexpander) lower half for the Realtek Ameba GPIO controller.
 *
 * The pad mux, pull resistors and GPIO port registers are all programmed
 * through the SDK fwlib GPIO API (GPIO_Init(), GPIO_WriteBit(), ...).
 *
 * Most of the API has no SDK source and resolves to the on-chip ROM symbol
 * table (GPIO_WriteBit, GPIO_ReadDataBit, GPIO_INTConfig, GPIO_Direction,
 * GPIO_INTMode, Pinmux_Config, PAD_PullCtrl, ...).  Only GPIO_INTStatusGet
 * and GPIO_INTStatusClearEdge are missing from ROM; they live in fwlib
 * ram_common/ameba_gpio.c, compiled into libameba_fwlib.a to provide them
 * (see AMEBA_FWLIB_SRCS).  That same source also defines GPIO_Init (which
 * IS in ROM), so the local object overrides the ROM PROVIDE(GPIO_Init) as a
 * harmless side effect -- same source, equivalent behaviour.  To keep the
 * vendor headers out of the NuttX include world, the handful of fwlib
 * symbols and the GPIO_InitTypeDef layout used here are declared locally
 * below rather than pulled in from <ameba_gpio.h>.
 *
 * Interrupt dispatch stays NuttX-native: the port's NVIC vector is owned by
 * NuttX (irq_attach), and the ISR reads/clears the controller status through
 * the fwlib GPIO_INTStatus* helpers before calling each pending pin's
 * registered callback.
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <nuttx/ioexpander/gpio.h>

#include "arm_internal.h"
#include "ameba_gpio.h"

/* Per-chip GPIO parameters (port count, per-port NVIC vectors, RCC gate
 * bits).  Resolved from the configured chip's directory on the include path
 * (arch/.../chip -> arch/arm/src/<chip>); see <ameba_gpio_chip.h> there.
 */

#include "ameba_gpio_chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pin encoding (SDK "PinName": port in bits[7:5], pin num in bits[4:0]). */

#define AMEBA_PIN_PORT(pin)     (((pin) >> 5) & 0x07)
#define AMEBA_PIN_NUM(pin)      ((pin) & 0x1f)

/* Mirror of the fwlib GPIO_InitTypeDef field values (ameba_gpio.h).
 * Declared locally so the SDK header need not leak into NuttX includes.
 */

#define AMEBA_GPIO_MODE_IN      0x0   /* GPIO_Mode_IN                       */
#define AMEBA_GPIO_MODE_OUT     0x1   /* GPIO_Mode_OUT                      */
#define AMEBA_GPIO_MODE_INT     0x2   /* GPIO_Mode_INT                      */

#define AMEBA_GPIO_PUPD_NONE    0x0   /* GPIO_PuPd_NOPULL                   */
#define AMEBA_GPIO_PUPD_DOWN    0x1   /* GPIO_PuPd_DOWN                     */
#define AMEBA_GPIO_PUPD_UP      0x2   /* GPIO_PuPd_UP                       */

#define AMEBA_GPIO_IT_LEVEL     0x0   /* GPIO_INT_Trigger_LEVEL             */
#define AMEBA_GPIO_IT_EDGE      0x1   /* GPIO_INT_Trigger_EDGE              */
#define AMEBA_GPIO_IT_BOTHEDGE  0x2   /* GPIO_INT_Trigger_BOTHEDGE          */

#define AMEBA_GPIO_POL_LOW      0x0   /* GPIO_INT_POLARITY_ACTIVE_LOW       */
#define AMEBA_GPIO_POL_HIGH     0x1   /* GPIO_INT_POLARITY_ACTIVE_HIGH      */

#define AMEBA_GPIO_DEBOUNCE_OFF 0x0   /* GPIO_INT_DEBOUNCE_DISABLE          */

/* Second argument to GPIO_INTConfig() / state passed to GPIO_WriteBit(). */

#define AMEBA_DISABLE           0x0
#define AMEBA_ENABLE            0x1

/* AMEBA_APBPERIPH_GPIO (the RCC gate bits), AMEBA_GPIO_NPORTS and the
 * AMEBA_GPIO_PORT_IRQS vector table come from <ameba_gpio_chip.h>.  The
 * fwlib GPIO_Init() leaves the peripheral clock to the caller, so the driver
 * gates AMEBA_APBPERIPH_GPIO on before use (RCC touches the LSYS block).
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Layout-compatible mirror of the fwlib GPIO_InitTypeDef (all u32, same
 * order); passed by address to GPIO_Init().
 */

struct ameba_gpio_init_s
{
  uint32_t mode;               /* GPIO_Mode        */
  uint32_t pupd;               /* GPIO_PuPd        */
  uint32_t ittrigger;          /* GPIO_ITTrigger   */
  uint32_t itpolarity;         /* GPIO_ITPolarity  */
  uint32_t itdebounce;         /* GPIO_ITDebounce  */
  uint32_t pin;                /* GPIO_Pin         */
};

struct ameba_gpio_dev_s
{
  struct gpio_dev_s gpio;      /* Lower-half GPIO device (must be first) */
  uint8_t pin;                 /* SDK pin encoding (port[7:5] | num[4:0]) */
  bool leveltrig;              /* Level-triggered interrupt (not edge) */
  pin_interrupt_t callback;    /* Interrupt callback (interrupt pins only) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SDK fwlib GPIO API used by this driver.  These are plain extern symbols;
 * the driver does not care where each comes from -- every one is resolved
 * at link time either by the on-chip ROM symbol table or by a fwlib object
 * compiled into libameba_fwlib.a.  Which source applies is per-chip:
 *
 * RCC_PeriphClockCmd, GPIO_Init, GPIO_WriteBit, GPIO_ReadDataBit,
 * GPIO_INTConfig and GPIO_INTMode are in ROM on every Ameba ARM chip.
 * GPIO_Init calls Pinmux_Config and GPIO_Direction internally.  On
 * RTL8721Dx its ram_common override also calls PAD_PullCtrl and (for
 * INT mode) GPIO_INTMode; on amebagreen2 the ROM version does neither,
 * so the driver calls both explicitly after GPIO_Init.
 *
 * GPIO_INTStatusGet and GPIO_INTStatusClearEdge are the gap.  On RTL8721Dx
 * they come from fwlib ram_common/ameba_gpio.c, compiled in via
 * AMEBA_FWLIB_SRCS (that object also carries GPIO_Init and harmlessly
 * overrides the ROM copy).  On amebalite/amebasmart they are in neither ROM
 * nor any SDK source, so a chip adding those parts MUST supply its own
 * definition -- a small object writing the port's GPIO_INT_STATUS /
 * GPIO_INT_EOI -- via its board build.  The shared driver needs no change;
 * this is the per-chip contract for these two symbols.
 */

extern void RCC_PeriphClockCmd(uint32_t periph, uint32_t clock,
                               uint8_t newstate);
extern void GPIO_Init(struct ameba_gpio_init_s *init);
extern void GPIO_INTMode(uint32_t pin, uint32_t newstate, uint32_t trigger,
                         uint32_t polarity, uint32_t debounce);
extern void GPIO_WriteBit(uint32_t pin, uint32_t state);
extern uint32_t GPIO_ReadDataBit(uint32_t pin);
extern void GPIO_INTConfig(uint32_t pin, uint32_t newstate);
extern void PAD_PullCtrl(uint32_t pin, uint32_t pupd);
extern uint32_t GPIO_INTStatusGet(uint32_t port);
extern void GPIO_INTStatusClearEdge(uint32_t port);

/* GPIO lower-half operations */

static int ameba_gpio_read(struct gpio_dev_s *dev, bool *value);
static int ameba_gpio_write(struct gpio_dev_s *dev, bool value);
static int ameba_gpio_attach(struct gpio_dev_s *dev,
                             pin_interrupt_t callback);
static int ameba_gpio_enable(struct gpio_dev_s *dev, bool enable);
static int ameba_gpio_setpintype(struct gpio_dev_s *dev,
                                 enum gpio_pintype_e pintype);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_operations_s g_ameba_gpio_ops =
{
  .go_read       = ameba_gpio_read,
  .go_write      = ameba_gpio_write,
  .go_attach     = ameba_gpio_attach,
  .go_enable     = ameba_gpio_enable,
  .go_setpintype = ameba_gpio_setpintype,
};

/* Driver-wide state, kept in one controller instance instead of
 * scattered file-scope globals.  A single NVIC vector per port drives
 * up to 32 pins, so the dispatch table, the attach flags and the NVIC
 * vectors are all per-port, and one spinlock guards the shared table.
 */

struct ameba_gpio_ctrl_s
{
  /* Per-pin device lookup for interrupt dispatch, indexed [port][pin-num];
   * set once an interrupt pin is registered, read by the port ISR.
   */

  struct ameba_gpio_dev_s *int_dev[AMEBA_GPIO_NPORTS][32];

  /* Set once a port's NVIC vector has been attached + enabled. */

  bool port_attached[AMEBA_GPIO_NPORTS];

  /* Per-port NVIC vector, from the chip's <ameba_gpio_chip.h>. */

  const int irq[AMEBA_GPIO_NPORTS];

  spinlock_t lock;
};

static struct ameba_gpio_ctrl_s g_ameba_gpio =
{
  .irq  = AMEBA_GPIO_PORT_IRQS,
  .lock = SP_UNLOCKED,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_gpio_configure
 *
 * Description:
 *   Program the pad mux, direction, pull and (for interrupt pins) trigger
 *   mode for the given pin type through the fwlib GPIO_Init().  Interrupt
 *   pins are left masked until the application enables them.
 *
 ****************************************************************************/

static void ameba_gpio_configure(struct ameba_gpio_dev_s *priv,
                                  enum gpio_pintype_e pintype)
{
  struct ameba_gpio_init_s init;

  init.pin        = priv->pin;
  init.pupd       = AMEBA_GPIO_PUPD_NONE;
  init.ittrigger  = AMEBA_GPIO_IT_EDGE;
  init.itpolarity = AMEBA_GPIO_POL_LOW;
  init.itdebounce = AMEBA_GPIO_DEBOUNCE_OFF;

  /* Gate on the GPIO peripheral clock (idempotent). */

  RCC_PeriphClockCmd(AMEBA_APBPERIPH_GPIO, AMEBA_APBPERIPH_GPIO,
                     AMEBA_ENABLE);

  switch (pintype)
    {
      case GPIO_OUTPUT_PIN:
      case GPIO_OUTPUT_PIN_OPENDRAIN:
        init.mode = AMEBA_GPIO_MODE_OUT;
        break;

      case GPIO_INPUT_PIN:
        init.mode = AMEBA_GPIO_MODE_IN;
        break;

      case GPIO_INPUT_PIN_PULLUP:
        init.mode = AMEBA_GPIO_MODE_IN;
        init.pupd = AMEBA_GPIO_PUPD_UP;
        break;

      case GPIO_INPUT_PIN_PULLDOWN:
        init.mode = AMEBA_GPIO_MODE_IN;
        init.pupd = AMEBA_GPIO_PUPD_DOWN;
        break;

      case GPIO_INTERRUPT_HIGH_PIN:
        init.mode       = AMEBA_GPIO_MODE_INT;
        init.ittrigger  = AMEBA_GPIO_IT_LEVEL;
        init.itpolarity = AMEBA_GPIO_POL_HIGH;
        init.pupd       = AMEBA_GPIO_PUPD_DOWN;
        break;

      case GPIO_INTERRUPT_LOW_PIN:
        init.mode       = AMEBA_GPIO_MODE_INT;
        init.ittrigger  = AMEBA_GPIO_IT_LEVEL;
        init.itpolarity = AMEBA_GPIO_POL_LOW;
        init.pupd       = AMEBA_GPIO_PUPD_UP;
        break;

      case GPIO_INTERRUPT_RISING_PIN:
        init.mode       = AMEBA_GPIO_MODE_INT;
        init.ittrigger  = AMEBA_GPIO_IT_EDGE;
        init.itpolarity = AMEBA_GPIO_POL_HIGH;
        init.pupd       = AMEBA_GPIO_PUPD_DOWN;
        break;

      case GPIO_INTERRUPT_BOTH_PIN:
        init.mode       = AMEBA_GPIO_MODE_INT;
        init.ittrigger  = AMEBA_GPIO_IT_BOTHEDGE;
        init.itpolarity = AMEBA_GPIO_POL_LOW;
        break;

      case GPIO_INTERRUPT_PIN:
      case GPIO_INTERRUPT_FALLING_PIN:
      default:
        init.mode       = AMEBA_GPIO_MODE_INT;
        init.ittrigger  = AMEBA_GPIO_IT_EDGE;
        init.itpolarity = AMEBA_GPIO_POL_LOW;
        init.pupd       = AMEBA_GPIO_PUPD_UP;
        break;
    }

  /* Remember whether this is a level-triggered interrupt.  The port ISR
   * masks such pins after dispatch: GPIO_INTStatusClearEdge() clears only
   * edge latches, so a still-active level would otherwise re-enter the ISR
   * forever (an interrupt storm).  Edge pins are left unmasked so a single
   * attach/enable keeps receiving edges as before.
   */

  priv->leveltrig = (init.mode == AMEBA_GPIO_MODE_INT &&
                     init.ittrigger == AMEBA_GPIO_IT_LEVEL);

  GPIO_Init(&init);

  /* On amebagreen2 (and similar), the ROM GPIO_Init does not call
   * PAD_PullCtrl for interrupt or input modes.  Call it explicitly here so
   * the pad pull always matches init.pupd.  On RTL8721Dx whose ram_common
   * GPIO_Init already calls it this is a harmless redundant write.
   */

  PAD_PullCtrl(init.pin, init.pupd);

  /* On chips whose ROM GPIO_Init does not call GPIO_INTMode internally
   * (confirmed for amebagreen2; harmless double-call on RTL8721Dx where
   * ram_common already does it), explicitly programme the trigger/polarity
   * into the GPIO interrupt controller now.  GPIO_INTMode with ENABLE
   * also unmasks the pin in the controller, so immediately follow with
   * GPIO_INTConfig(DISABLE) to keep it masked until the application
   * calls go_enable().  Clear stale edge latches first while the pin is
   * still unmasked; GPIO_INTStatusClearEdge operates on the unmasked
   * status so clearing after masking would miss this pin.  The whole
   * sequence runs under the port lock to prevent the ISR from dispatching
   * a stale edge between the clear and the mask.
   */

  if (init.mode == AMEBA_GPIO_MODE_INT)
    {
      irqstate_t flags = spin_lock_irqsave(&g_ameba_gpio.lock);

      GPIO_INTMode(priv->pin, AMEBA_ENABLE, init.ittrigger,
                   init.itpolarity, init.itdebounce);
      GPIO_INTStatusClearEdge(AMEBA_PIN_PORT(priv->pin));
      GPIO_INTConfig(priv->pin, AMEBA_DISABLE);

      spin_unlock_irqrestore(&g_ameba_gpio.lock, flags);
    }
}

/****************************************************************************
 * Name: ameba_gpio_interrupt
 *
 * Description:
 *   NuttX-native vector for a GPIO port.  Reads the (masked) interrupt
 *   status, clears the edge latches and dispatches to each pending pin's
 *   registered callback.
 *
 ****************************************************************************/

static int ameba_gpio_interrupt(int irq, void *context, void *arg)
{
  int port = (int)(uintptr_t)arg;
  uint32_t status = GPIO_INTStatusGet(port);

  /* Clear the port's edge latches.  GPIO_INTStatusClearEdge() re-reads the
   * status internally and takes no bitmask, so an edge that arrives between
   * the read above and this clear is acknowledged without being dispatched
   * this pass -- an inherent limit of the two-call fwlib interface.  It is
   * harmless for the board's single edge pin, and level-triggered sources
   * re-assert and fire again regardless.
   */

  GPIO_INTStatusClearEdge(port);

  while (status != 0)
    {
      int num = __builtin_ctz(status);
      struct ameba_gpio_dev_s *priv = g_ameba_gpio.int_dev[port][num];

      status &= ~((uint32_t)1 << num);

      if (priv != NULL && priv->callback != NULL)
        {
          /* A level-triggered pin keeps its interrupt status asserted for
           * as long as the level is active; GPIO_INTStatusClearEdge() above
           * cannot clear it, so mask the pin now to stop it re-entering the
           * ISR forever.  The application re-enables it (go_enable) once it
           * has serviced the event.  Edge pins are left unmasked.
           */

          if (priv->leveltrig)
            {
              GPIO_INTConfig(priv->pin, AMEBA_DISABLE);
            }

          priv->callback(&priv->gpio, num);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: ameba_gpio_read
 ****************************************************************************/

static int ameba_gpio_read(struct gpio_dev_s *dev, bool *value)
{
  struct ameba_gpio_dev_s *priv = (struct ameba_gpio_dev_s *)dev;

  DEBUGASSERT(priv != NULL && value != NULL);

  *value = (GPIO_ReadDataBit(priv->pin) != 0);
  return OK;
}

/****************************************************************************
 * Name: ameba_gpio_write
 ****************************************************************************/

static int ameba_gpio_write(struct gpio_dev_s *dev, bool value)
{
  struct ameba_gpio_dev_s *priv = (struct ameba_gpio_dev_s *)dev;

  DEBUGASSERT(priv != NULL);

  GPIO_WriteBit(priv->pin, value ? AMEBA_ENABLE : AMEBA_DISABLE);
  return OK;
}

/****************************************************************************
 * Name: ameba_gpio_attach
 ****************************************************************************/

static int ameba_gpio_attach(struct gpio_dev_s *dev,
                             pin_interrupt_t callback)
{
  struct ameba_gpio_dev_s *priv = (struct ameba_gpio_dev_s *)dev;

  DEBUGASSERT(priv != NULL);

  /* Mask the interrupt while (re)attaching, then record the callback. */

  GPIO_INTConfig(priv->pin, AMEBA_DISABLE);
  priv->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: ameba_gpio_enable
 ****************************************************************************/

static int ameba_gpio_enable(struct gpio_dev_s *dev, bool enable)
{
  struct ameba_gpio_dev_s *priv = (struct ameba_gpio_dev_s *)dev;

  DEBUGASSERT(priv != NULL);

  if (enable && priv->callback != NULL)
    {
      /* Unmask the pin, then drop any stale edge that unmasking just exposed
       * (the GPIO_Init arming transient, or an edge latched while masked) so
       * the first wait does not fire on a past interrupt.  Clearing runs
       * with the pin unmasked -- GPIO_INTStatusClearEdge() works through the
       * masked GPIO_INT_STATUS -- and the port lock keeps the ISR from
       * dispatching that stale edge between the two steps.  Level-triggered
       * pins do not latch, so a genuinely active level still fires at once.
       */

      irqstate_t flags = spin_lock_irqsave(&g_ameba_gpio.lock);

      GPIO_INTConfig(priv->pin, AMEBA_ENABLE);
      GPIO_INTStatusClearEdge(AMEBA_PIN_PORT(priv->pin));

      spin_unlock_irqrestore(&g_ameba_gpio.lock, flags);
    }
  else
    {
      GPIO_INTConfig(priv->pin, AMEBA_DISABLE);
    }

  return OK;
}

/****************************************************************************
 * Name: ameba_gpio_setpintype
 ****************************************************************************/

static int ameba_gpio_setpintype(struct gpio_dev_s *dev,
                                  enum gpio_pintype_e pintype)
{
  struct ameba_gpio_dev_s *priv = (struct ameba_gpio_dev_s *)dev;

  DEBUGASSERT(priv != NULL);

  if (pintype >= GPIO_NPINTYPES)
    {
      return -EINVAL;
    }

  ameba_gpio_configure(priv, pintype);
  dev->gp_pintype = pintype;
  return OK;
}

/****************************************************************************
 * Name: ameba_gpio_port_attach
 *
 * Description:
 *   Attach and enable the NuttX-native interrupt vector for the GPIO port
 *   that owns the given pin (once per port).
 *
 ****************************************************************************/

static void ameba_gpio_port_attach(uint8_t pin)
{
  irqstate_t flags;
  int port = AMEBA_PIN_PORT(pin);

  flags = spin_lock_irqsave(&g_ameba_gpio.lock);

  if (!g_ameba_gpio.port_attached[port])
    {
      int irq = g_ameba_gpio.irq[port];

      irq_attach(irq, ameba_gpio_interrupt, (void *)(uintptr_t)port);
      up_enable_irq(irq);
      g_ameba_gpio.port_attached[port] = true;
    }

  spin_unlock_irqrestore(&g_ameba_gpio.lock, flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_gpio_register
 *
 * Description:
 *   See ameba_gpio.h.
 *
 ****************************************************************************/

int ameba_gpio_register(int minor, uint8_t pin, enum gpio_pintype_e pintype)
{
  struct ameba_gpio_dev_s *priv;
  bool interrupt;
  int ret;

  if (pintype >= GPIO_NPINTYPES)
    {
      return -EINVAL;
    }

  interrupt = (pintype >= GPIO_INTERRUPT_PIN);

  priv = kmm_zalloc(sizeof(struct ameba_gpio_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->pin             = pin;
  priv->gpio.gp_pintype = pintype;
  priv->gpio.gp_ops     = &g_ameba_gpio_ops;

  /* Apply the pad/pin configuration for this pin type. */

  ameba_gpio_configure(priv, pintype);

  if (interrupt)
    {
      /* Publish the device for the port ISR and make sure the owning port's
       * NuttX vector is live.  The pin stays masked until the application
       * attaches a callback and enables it.
       */

      g_ameba_gpio.int_dev[AMEBA_PIN_PORT(pin)][AMEBA_PIN_NUM(pin)] = priv;
      ameba_gpio_port_attach(pin);
    }

  ret = gpio_pin_register(&priv->gpio, minor);
  if (ret < 0)
    {
      gpioerr("ERROR: gpio_pin_register(%d) failed: %d\n", minor, ret);

      if (interrupt)
        {
          GPIO_INTConfig(pin, AMEBA_DISABLE);
          g_ameba_gpio.int_dev[AMEBA_PIN_PORT(pin)][AMEBA_PIN_NUM(pin)] =
            NULL;
        }

      kmm_free(priv);
    }

  return ret;
}
