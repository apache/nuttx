/****************************************************************************
 * drivers/ioexpander/pl061.c
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

#include <errno.h>
#include <nuttx/config.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/pl061.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PL061_GETREG(d,o)   (*(FAR volatile uint8_t *)((d)->base + (o)))
#define PL061_PUTREG(d,v,o) (*((FAR volatile uint8_t *)((d)->base + (o))) = (v))
#define PL061_MODREG(d,v,m,o) \
  do \
    { \
      irqstate_t flags = spin_lock_irqsave(&d->lock); \
      PL061_PUTREG(d, ((PL061_GETREG(d,o) & ~(m)) | ((v) & (m))), o); \
      spin_unlock_irqrestore(&d->lock, flags); \
    } \
  while (0)

#define PL061_GPIODIR 0x400
#define PL061_GPIOIS  0x404
#define PL061_GPIOIBE 0x408
#define PL061_GPIOIEV 0x40C
#define PL061_GPIOIE  0x410
#define PL061_GPIORIS 0x414
#define PL061_GPIOMIS 0x418
#define PL061_GPIOIC  0x41C

#define PL061_GPIO_NR 8

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_PL061_INT_ENABLE
/* This type represents on registered pin interrupt callback */

struct pl061_callback_s
{
  ioe_pinset_t pinset;   /* Set of pin interrupts that the callback */
  ioe_callback_t cbfunc; /* The saved callback function pointer */
  FAR void *cbarg;
};
#endif

struct pl061_dev_s
{
  struct ioexpander_dev_s dev; /* Allow casting as public gpio expander */
  uintptr_t base;
  spinlock_t lock;

#ifdef CONFIG_PL061_INT_ENABLE
  /* Saved callback information for each I/O expander client */

  struct pl061_callback_s cb[PL061_GPIO_NR];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pl061_direction(FAR struct ioexpander_dev_s *dev,
                           uint8_t pin, int dir);
static int pl061_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                        int opt, FAR void *val);
static int pl061_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          bool value);
static int pl061_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                         FAR bool *value);
#ifdef CONFIG_PL061_INT_ENABLE
static FAR void *pl061_attach(FAR struct ioexpander_dev_s *dev,
                              ioe_pinset_t pinset, ioe_callback_t callback,
                              FAR void *arg);
static int pl061_detach(FAR struct ioexpander_dev_s *dev,
                        FAR void *handle);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ioexpander_ops_s g_pl061_ops =
{
  pl061_direction,
  pl061_option,
  pl061_writepin,
  pl061_readpin,
  NULL
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , NULL
  , NULL
  , NULL
#endif
#ifdef CONFIG_PL061_INT_ENABLE
  , pl061_attach
  , pl061_detach
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int pl061_direction(FAR struct ioexpander_dev_s *dev,
                           uint8_t pin, int dir)
{
  FAR struct pl061_dev_s *priv = (FAR struct pl061_dev_s *)dev;

  DEBUGASSERT(priv != NULL &&  pin < PL061_GPIO_NR);

  if (dir == IOEXPANDER_DIRECTION_OUT)
    {
      PL061_MODREG(priv, 1 << pin, 1 << pin, PL061_GPIODIR);
    }
  else if (dir == IOEXPANDER_DIRECTION_IN)
    {
      PL061_MODREG(priv, 0, 1 << pin, PL061_GPIODIR);
    }
  else
    {
      return -EINVAL;
    }

  return 0;
}

static int pl061_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                        int opt, FAR void *val)
{
  FAR struct pl061_dev_s *priv = (FAR struct pl061_dev_s *)dev;
  unsigned int ival = (unsigned int)((uintptr_t)val);

  if (opt == IOEXPANDER_OPTION_INTCFG)
    {
      /* Set the pin option */

      switch (ival)
        {
          case IOEXPANDER_VAL_HIGH:    /* Interrupt on high level */
            PL061_MODREG(priv, 1 << pin, 1 << pin, PL061_GPIOIS);
            PL061_MODREG(priv, 1 << pin, 1 << pin, PL061_GPIOIEV);
            PL061_MODREG(priv, 1 << pin, 1 << pin, PL061_GPIOIE);
            break;

          case IOEXPANDER_VAL_LOW:     /* Interrupt on low level */
            PL061_MODREG(priv, 1 << pin, 1 << pin, PL061_GPIOIS);
            PL061_MODREG(priv, 0, 1 << pin, PL061_GPIOIEV);
            PL061_MODREG(priv, 1 << pin, 1 << pin, PL061_GPIOIE);
            break;

          case IOEXPANDER_VAL_EDGE:  /* Interrupt on edge */
            PL061_MODREG(priv, 0, 1 << pin, PL061_GPIOIS);
            PL061_MODREG(priv, 1 << pin, 1 << pin, PL061_GPIOIBE);
            PL061_MODREG(priv, 1 << pin, 1 << pin, PL061_GPIOIE);
            break;

          case IOEXPANDER_VAL_RISING:  /* Interrupt on rising edge */
            PL061_MODREG(priv, 0, 1 << pin, PL061_GPIOIS);
            PL061_MODREG(priv, 0, 1 << pin, PL061_GPIOIBE);
            PL061_MODREG(priv, 1 << pin, 1 << pin, PL061_GPIOIEV);
            PL061_MODREG(priv, 1 << pin, 1 << pin, PL061_GPIOIE);
            break;

          case IOEXPANDER_VAL_FALLING: /* Interrupt on falling edge */
            PL061_MODREG(priv, 0, 1 << pin, PL061_GPIOIS);
            PL061_MODREG(priv, 0, 1 << pin, PL061_GPIOIBE);
            PL061_MODREG(priv, 0, 1 << pin, PL061_GPIOIEV);
            PL061_MODREG(priv, 1 << pin, 1 << pin, PL061_GPIOIE);
            break;

          case IOEXPANDER_VAL_DISABLE: /* Disable Interrupt */
            PL061_MODREG(priv, 0, 1 << pin, PL061_GPIOIE);
            break;

          default:
            return -EINVAL;
        }
    }
  else
    {
      return -EINVAL;
    }

  return 0;
}

static int pl061_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          bool value)
{
  FAR struct pl061_dev_s *priv = (FAR struct pl061_dev_s *)dev;

  PL061_PUTREG(priv, value << pin, 1 << (pin + 2));
  return 0;
}

static int pl061_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                         FAR bool *value)
{
  FAR struct pl061_dev_s *priv = (FAR struct pl061_dev_s *)dev;

  *value = (PL061_GETREG(priv, 1 << (pin + 2)) >> pin) & 1;
  return 0;
}

#ifdef CONFIG_PL061_INT_ENABLE
static FAR void *pl061_attach(FAR struct ioexpander_dev_s *dev,
                              ioe_pinset_t pinset, ioe_callback_t callback,
                              FAR void *arg)
{
  FAR struct pl061_dev_s *priv = (FAR struct pl061_dev_s *)dev;
  FAR void *handle = NULL;
  irqstate_t flags;
  int i;

  flags = spin_lock_irqsave(&priv->lock);
  for (i = 0; i < PL061_GPIO_NR; i++)
    {
      /* Is this entry available (i.e., no callback attached) */

      if (priv->cb[i].cbfunc == NULL)
        {
          /* Yes.. use this entry */

          priv->cb[i].pinset = pinset;
          priv->cb[i].cbfunc = callback;
          priv->cb[i].cbarg = arg;
          handle = &priv->cb[i];
          break;
        }
    }

  spin_unlock_irqrestore(&priv->lock, flags);
  return handle;
}

static int pl061_detach(FAR struct ioexpander_dev_s *dev,
                        FAR void *handle)
{
  FAR struct pl061_callback_s *cb = handle;

  cb->pinset = 0;
  cb->cbfunc = NULL;
  cb->cbarg  = NULL;
  return 0;
}

static int pl061_interrupt(int irq, void *context, void *arg)
{
  FAR struct pl061_dev_s *priv = arg;
  uint8_t pinset = PL061_GETREG(priv, PL061_GPIOMIS);
  int i;

  /* Perform pin interrupt callbacks */

  for (i = 0; i < PL061_GPIO_NR; i++)
    {
      /* Is this entry valid (i.e., callback attached)?  If so, did
       * any of the requested pin interrupts occur?
       */

      if (priv->cb[i].cbfunc != NULL)
        {
          /* Did any of the requested pin interrupts occur? */

          ioe_pinset_t match = pinset & priv->cb[i].pinset;

          if (match != 0)
            {
              /* Yes.. perform the callback */

              priv->cb[i].cbfunc(&priv->dev, match,
                                 priv->cb[i].cbarg);
            }
        }
    }

  PL061_PUTREG(priv, pinset, PL061_GPIOIC);

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct ioexpander_dev_s *pl061_ioe_initialize(uintptr_t base, int irq)
{
  FAR struct pl061_dev_s *priv = kmm_zalloc(sizeof(struct pl061_dev_s));

  priv->dev.ops = &g_pl061_ops;
  priv->base = base;

#ifdef CONFIG_PL061_INT_ENABLE
  /* Attach the I/O expander interrupt handler and enable interrupts */

  irq_attach(irq, pl061_interrupt, priv);
  up_enable_irq(irq);

#endif

  return &priv->dev;
}
