/****************************************************************************
 * drivers/misc/lwl_console.c
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/param.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/serial/serial.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Lightweight Link (LWL)
 * ======================
 *
 * Lightweight bidirectional communication between target and debug host
 * without any need for additional hardware.  It works with OpenOCD and any
 * other debugger that can read and write target memory while the CPU is
 * running.  See Documentation/guides/drivers/lwl.rst for the full
 * description.
 *
 * A 'downword' of 32 bits carries data from the host to the target and an
 * 'upword' of the same size carries data in the opposite direction, with a
 * ping-pong handshake on two 'sense' bits:
 *
 *   Bit    Downword (host -> target)    Upword (target -> host)
 *   31     A  link active (host)        A  link active (target)
 *   30     D  downsense (new data)      D  downsense ack
 *   29     U  upsense ack               U  upsense (new data)
 *   28-27  VV number of valid octets    VV number of valid octets
 *   26-24  XXX port (1 = console)       XXX port (1 = console)
 *   23-0   O2 O1 O0 data octets         O2 O1 O0 data octets
 *
 * Hosts that support it can instead collect the output from a ring buffer
 * in the control block, many bytes per debugger access: the target then
 * reports the A and D bits in the 'status' word and the upword is unused.
 *
 * The driver is a lower half of the generic serial driver, so the console
 * gets proper RX/TX buffering, termios processing, poll() and signals.
 * It never busy-waits for the host: the link is serviced either from a
 * "doorbell" interrupt that the host raises after every update or, when
 * the doorbell is not configured, from a low rate watchdog timer.
 */

/* Protocol bits */

#define LWL_ACTIVEBIT        (1u << 31)
#define LWL_GETACTIVE(x)     (((x) & LWL_ACTIVEBIT) != 0)

#define LWL_DNSENSEBIT       (1u << 30)
#define LWL_DNSENSE(x)       ((x) & LWL_DNSENSEBIT)
#define LWL_UPSENSEBIT       (1u << 29)
#define LWL_UPSENSE(x)       ((x) & LWL_UPSENSEBIT)
#define LWL_SENSEMASK        (LWL_DNSENSEBIT | LWL_UPSENSEBIT)

#define LWL_GETOCTVAL(x)     (((x) >> 27) & 3)
#define LWL_OCTVAL(x)        (((uint32_t)(x) & 3) << 27)
#define LWL_GETPORT(x)       (((x) >> 24) & 7)
#define LWL_PORT(x)          (((uint32_t)(x) & 7) << 24)
#define LWL_MAXOCTETS        3

#define LWL_PORT_CONSOLE     1

/* Host capabilities, written by the host before it opens the link */

#define LWL_CAP_RING         (1u << 0) /* The host reads the ring buffer */

/* Signatures that let the host locate the control block in target RAM.
 * LWL_EXT_SIG tells the host that the fields that follow the upword are
 * valid (legacy firmware does not have them).
 */

#define LWL_ID_SIG           0x7216a318
#define LWL_EXT_SIG          0x4c574c32 /* "LWL2" */

/* Doorbell: the host writes 'dbval' to 'dbaddr' after an update of the
 * link.  The board chooses a register that the debugger can write while
 * the CPU is running and whose write makes the doorbell IRQ pending, for
 * instance an NVIC Interrupt Set-Pending Register on Cortex-M.  The
 * interrupt handler writes 0 to it, which clears level-type software
 * interrupts (e.g. the RISC-V CLINT msip) and is ignored by set-pending
 * registers.
 */

#ifdef CONFIG_LWL_CONSOLE_DOORBELL
#  define LWL_DBIRQ          CONFIG_LWL_CONSOLE_DOORBELL_IRQ
#  define LWL_DBADDR         CONFIG_LWL_CONSOLE_DOORBELL_ADDR
#  define LWL_DBVAL          CONFIG_LWL_CONSOLE_DOORBELL_VALUE
#  if LWL_DBADDR == 0 || LWL_DBVAL == 0
#    error "LWL doorbell needs CONFIG_LWL_CONSOLE_DOORBELL_ADDR and _VALUE"
#  endif
#else
#  define LWL_DBADDR         0
#  define LWL_DBVAL          0
#endif

/* Without the doorbell the link is polled */

#ifndef CONFIG_LWL_CONSOLE_DOORBELL
#  define LWL_POLL           1
#  define LWL_POLLINTERVAL   CONFIG_LWL_CONSOLE_POLL_INTERVAL
#  define LWL_POLLTICKS      MAX(USEC2TICK(LWL_POLLINTERVAL), 1)
#endif

#define LWL_RINGSIZE         CONFIG_LWL_CONSOLE_RINGSIZE

#if LWL_RINGSIZE % 4 != 0
#  error "CONFIG_LWL_CONSOLE_RINGSIZE must be a multiple of 4"
#endif

/* The debugger accesses memory behind the data cache.  The fields written
 * by the host and the ones written by the target are kept in different
 * 64 byte groups, so cleaning a line never writes stale host data back.
 */

#define LWL_GROUP            64

#ifdef CONFIG_ARCH_DCACHE
#  define LWL_ALIGN          aligned_data(LWL_GROUP)
#else
#  define LWL_ALIGN
#endif

/* Number of consecutive busy polls after which an output that must not
 * block (up_putc() and writes from interrupt context) considers the host
 * gone and starts dropping data instead of hanging the system.
 */

#define LWL_STALL_SPINS      0x400000

/* Longest output line that is shown again when a host connects */

#define LWL_LINESIZE         80

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The control block shared with the host.  The host relies on the offsets
 * of the fields, so they never change.
 */

struct LWL_ALIGN lwl_entry_s
{
  /* 0x00: written by the host, static target data and the legacy upword */

  uint32_t          sig;      /* LWL_ID_SIG: location signature */
  volatile uint32_t downword; /* Host to target word */
  volatile uint32_t upword;   /* Target to host word, word protocol only */
  uint32_t          extsig;   /* LWL_EXT_SIG: the fields below are valid */
  uint32_t          dbaddr;   /* Doorbell address, 0 if there is none */
  uint32_t          dbval;    /* Value the host writes to dbaddr */
  uint32_t          ringsize; /* Size of ring[], 0 if there is none */
  uint32_t          reserved0[9];

  /* 0x40: written by the host */

  volatile uint32_t hostcaps; /* LWL_CAP_* supported by the host */
  volatile uint32_t uptail;   /* Next ring[] octet the host will read */
  uint32_t          reserved1[14];

  /* 0x80: written by the target */

  volatile uint32_t status;   /* A and D bits, ring protocol only */
  volatile uint32_t uphead;   /* Next ring[] octet the target will write */
  uint32_t          reserved2[14];

  /* 0xc0: output ring buffer, written by the target */

#if LWL_RINGSIZE > 0
  uint8_t           ring[LWL_RINGSIZE];
#endif
};

struct lwl_priv_s
{
#ifdef LWL_POLL
  struct wdog_s     wdog;     /* Polling timer */
#endif
  uint32_t          spins;    /* Consecutive busy polls of the uplink */
  uint8_t           rxoff;    /* Octets of the downword already consumed */
  uint8_t           linelen;  /* Length of line[], > LWL_LINESIZE if lost */
  uint8_t           replay;   /* Octets of line[] already replayed */
  bool              replayon; /* line[] is being sent to a new host */
  bool              active;   /* The link is open */
  bool              usering;  /* The host reads the ring buffer */
  bool              attached; /* The serial device is open */
  bool              rxint;    /* "RX interrupt" enabled by the upper half */
  bool              txint;    /* "TX interrupt" enabled by the upper half */
  bool              stalled;  /* The host stopped collecting uplink data */

  /* Last unterminated output line, e.g. the shell prompt */

  char              line[LWL_LINESIZE];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Serial lower half methods */

static int  lwl_setup(FAR struct uart_dev_s *dev);
static void lwl_shutdown(FAR struct uart_dev_s *dev);
static int  lwl_attach(FAR struct uart_dev_s *dev);
static void lwl_detach(FAR struct uart_dev_s *dev);
static int  lwl_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int  lwl_receive(FAR struct uart_dev_s *dev,
                        FAR unsigned int *status);
static void lwl_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool lwl_rxavailable(FAR struct uart_dev_s *dev);
static void lwl_send(FAR struct uart_dev_s *dev, int ch);
static void lwl_txint(FAR struct uart_dev_s *dev, bool enable);
static bool lwl_txready(FAR struct uart_dev_s *dev);
static bool lwl_txempty(FAR struct uart_dev_s *dev);
static ssize_t lwl_recvbuf(FAR struct uart_dev_s *dev,
                           FAR void *buf, size_t len);
static ssize_t lwl_sendbuf(FAR struct uart_dev_s *dev,
                           FAR const void *buf, size_t len);

#ifdef LWL_POLL
static void lwl_poll(wdparm_t arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lwl_entry_s g_lwl =
{
  .sig      = LWL_ID_SIG,
  .extsig   = LWL_EXT_SIG,
  .dbaddr   = LWL_DBADDR,
  .dbval    = LWL_DBVAL,
  .ringsize = LWL_RINGSIZE,
};

static const struct uart_ops_s g_lwl_ops =
{
  .setup       = lwl_setup,
  .shutdown    = lwl_shutdown,
  .attach      = lwl_attach,
  .detach      = lwl_detach,
  .ioctl       = lwl_ioctl,
  .receive     = lwl_receive,
  .rxint       = lwl_rxint,
  .rxavailable = lwl_rxavailable,
  .send        = lwl_send,
  .txint       = lwl_txint,
  .txready     = lwl_txready,
  .txempty     = lwl_txempty,
  .recvbuf     = lwl_recvbuf,
  .sendbuf     = lwl_sendbuf,
};

static char g_lwl_rxbuffer[CONFIG_LWL_CONSOLE_RXBUFSIZE];
static char g_lwl_txbuffer[CONFIG_LWL_CONSOLE_TXBUFSIZE];

static struct lwl_priv_s g_lwl_priv;

static struct uart_dev_s g_lwl_dev =
{
  .isconsole = true,
  .recv      =
  {
    .size    = CONFIG_LWL_CONSOLE_RXBUFSIZE,
    .buffer  = g_lwl_rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_LWL_CONSOLE_TXBUFSIZE,
    .buffer  = g_lwl_txbuffer,
  },
  .ops       = &g_lwl_ops,
  .priv      = &g_lwl_priv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lwl_clean
 *
 * Description:
 *   Make target writes to the control block visible to the host.
 *
 ****************************************************************************/

static inline void lwl_clean(FAR volatile void *start, size_t len)
{
  up_clean_dcache((uintptr_t)start, (uintptr_t)start + len);
}

/****************************************************************************
 * Name: lwl_getdown
 *
 * Description:
 *   Return the current downword, after making all the host writes visible.
 *
 ****************************************************************************/

static uint32_t lwl_getdown(void)
{
  up_invalidate_dcache((uintptr_t)&g_lwl, (uintptr_t)&g_lwl.status);
  return g_lwl.downword;
}

/****************************************************************************
 * Name: lwl_getack / lwl_putack
 *
 * Description:
 *   Access the word holding the target's A and D bits: the status word for
 *   hosts that read the ring buffer, the upword for the others.
 *
 ****************************************************************************/

static inline uint32_t lwl_getack(FAR struct lwl_priv_s *priv)
{
  return priv->usering ? g_lwl.status : g_lwl.upword;
}

static void lwl_putack(FAR struct lwl_priv_s *priv, uint32_t word)
{
  if (priv->usering)
    {
      g_lwl.status = word;
      lwl_clean(&g_lwl.status, sizeof(g_lwl.status));
    }
  else
    {
      g_lwl.upword = word;
      lwl_clean(&g_lwl.upword, sizeof(g_lwl.upword));
    }
}

/****************************************************************************
 * Name: lwl_pollstart
 *
 * Description:
 *   (Re)start the polling timer while the device is open, if the link is
 *   polled (no doorbell).
 *
 ****************************************************************************/

static void lwl_pollstart(FAR struct uart_dev_s *dev)
{
#ifdef LWL_POLL
  FAR struct lwl_priv_s *priv = dev->priv;

  if (priv->attached && !WDOG_ISACTIVE(&priv->wdog))
    {
      wd_start(&priv->wdog, LWL_POLLTICKS, lwl_poll, (wdparm_t)dev);
    }
#endif
}

/****************************************************************************
 * Name: lwl_link
 *
 * Description:
 *   Track the link state requested by the host.  When the host opens the
 *   link the target adopts the host's sense bits and empties the ring, so
 *   that a new host never sees stale data nor waits for an acknowledge.
 *
 * Returned Value:
 *   true if the link is active; the current downword is returned in *down.
 *
 ****************************************************************************/

static bool lwl_link(FAR struct uart_dev_s *dev, FAR uint32_t *down)
{
  FAR struct lwl_priv_s *priv = dev->priv;
  uint32_t dn = lwl_getdown();

  if (!LWL_GETACTIVE(dn))
    {
      if (priv->active)
        {
          /* The host closed the link */

          g_lwl.upword = 0;
          g_lwl.status = 0;
          lwl_clean(&g_lwl.upword, sizeof(g_lwl.upword));
          lwl_clean(&g_lwl.status, sizeof(g_lwl.status));

          priv->active   = false;
          priv->rxoff    = 0;
          priv->replayon = false;
        }

      return false;
    }

  if (!priv->active)
    {
      priv->active  = true;
      priv->usering = LWL_RINGSIZE > 0 &&
                      (g_lwl.hostcaps & LWL_CAP_RING) != 0;
      priv->rxoff   = 0;
      priv->spins   = 0;
      priv->stalled = false;

      if (priv->usering)
        {
          g_lwl.uphead = g_lwl.uptail % MAX(LWL_RINGSIZE, 1);
          lwl_clean(&g_lwl.uphead, sizeof(g_lwl.uphead));
          lwl_putack(priv, LWL_ACTIVEBIT | LWL_DNSENSE(dn));
        }
      else
        {
          lwl_putack(priv, LWL_ACTIVEBIT | (dn & LWL_SENSEMASK));
        }

      /* The new host has not seen the line in progress (typically the
       * shell prompt), which was collected by a previous host or before
       * the link went down: send it again.
       */

      priv->replay   = 0;
      priv->replayon = priv->linelen > 0 && priv->linelen <= LWL_LINESIZE;
    }

  *down = dn;
  return true;
}

/****************************************************************************
 * Name: lwl_txroom
 *
 * Description:
 *   Return how many octets can be sent to the host now.
 *
 ****************************************************************************/

static size_t lwl_txroom(FAR struct lwl_priv_s *priv, uint32_t down)
{
#if LWL_RINGSIZE > 0
  if (priv->usering)
    {
      uint32_t tail = g_lwl.uptail % LWL_RINGSIZE;

      return (tail + LWL_RINGSIZE - g_lwl.uphead - 1) % LWL_RINGSIZE;
    }
#endif

  return LWL_UPSENSE(down) == LWL_UPSENSE(g_lwl.upword) ? LWL_MAXOCTETS : 0;
}

/****************************************************************************
 * Name: lwl_put
 *
 * Description:
 *   Send octets to the host, there must be room for at least one.
 *
 * Returned Value:
 *   The number of octets sent.
 *
 ****************************************************************************/

static size_t lwl_put(FAR struct lwl_priv_s *priv, uint32_t down,
                      FAR const uint8_t *src, size_t len)
{
  uint32_t upword;
  size_t i;

#if LWL_RINGSIZE > 0
  if (priv->usering)
    {
      uint32_t head = g_lwl.uphead;

      /* Copy the data first, then publish the new head */

      len = MIN(len, MIN(lwl_txroom(priv, down), LWL_RINGSIZE - head));
      memcpy(&g_lwl.ring[head], src, len);
      lwl_clean(&g_lwl.ring[head], len);

      g_lwl.uphead = (head + len) % LWL_RINGSIZE;
      lwl_clean(&g_lwl.uphead, sizeof(g_lwl.uphead));
      return len;
    }
#endif

  len    = MIN(len, LWL_MAXOCTETS);
  upword = g_lwl.upword;
  upword = LWL_ACTIVEBIT | LWL_OCTVAL(len) | LWL_PORT(LWL_PORT_CONSOLE) |
           LWL_DNSENSE(upword) | (LWL_UPSENSE(upword) ^ LWL_UPSENSEBIT);

  for (i = 0; i < len; i++)
    {
      upword |= (uint32_t)src[i] << (8 * i);
    }

  g_lwl.upword = upword;
  lwl_clean(&g_lwl.upword, sizeof(g_lwl.upword));
  return len;
}

/****************************************************************************
 * Name: lwl_replay
 *
 * Description:
 *   Send the next octets of the line replayed to a new host.
 *
 ****************************************************************************/

static void lwl_replay(FAR struct lwl_priv_s *priv, uint32_t down)
{
  priv->replay  += lwl_put(priv, down,
                           (FAR const uint8_t *)&priv->line[priv->replay],
                           priv->linelen - priv->replay);
  priv->replayon = priv->replay < priv->linelen;
}

/****************************************************************************
 * Name: lwl_rxack
 *
 * Description:
 *   Acknowledge the current downword, the host may then send the next one.
 *
 ****************************************************************************/

static void lwl_rxack(FAR struct lwl_priv_s *priv, uint32_t down)
{
  priv->rxoff = 0;
  lwl_putack(priv, (lwl_getack(priv) & ~LWL_DNSENSEBIT) | LWL_DNSENSE(down));
}

/****************************************************************************
 * Name: lwl_rxpending
 *
 * Description:
 *   Return true if the downword holds data not acknowledged yet.
 *
 ****************************************************************************/

static inline bool lwl_rxpending(FAR struct lwl_priv_s *priv, uint32_t down)
{
  return LWL_DNSENSE(down) != LWL_DNSENSE(lwl_getack(priv));
}

/****************************************************************************
 * Name: lwl_service
 *
 * Description:
 *   Move data between the control block and the serial buffers.  Called
 *   from the doorbell interrupt, from the polling timer and whenever the
 *   upper half enables the RX or TX "interrupts".  The uart lock
 *   serializes it with the upper half and with up_putc(), and keeps the
 *   scheduler locked while uart_recvchars()/uart_xmitchars() wake up
 *   readers and writers.
 *
 ****************************************************************************/

static void lwl_service(FAR struct uart_dev_s *dev)
{
  FAR struct lwl_priv_s *priv = dev->priv;
  irqstate_t flags;
  uint32_t down;

  flags = uart_spinlock(dev, true);

  if (lwl_link(dev, &down))
    {
      if (priv->rxint)
        {
          uart_recvchars(dev);
        }

      down = lwl_getdown();
      if (priv->replayon && lwl_txroom(priv, down) > 0)
        {
          lwl_replay(priv, down);
        }

      if (!priv->replayon && priv->txint && lwl_txroom(priv, down) > 0)
        {
          uart_xmitchars(dev);
        }
    }
  else if (priv->txint)
    {
      FAR struct uart_buffer_s *xmit = &dev->xmit;
      int head = xmit->head + 1 < xmit->size ? xmit->head + 1 : 0;

      /* No host is attached.  Keep the most recent output, so that it is
       * shown when the host connects, but drop the oldest data when the
       * buffer is full instead of blocking the writer forever.
       */

      if (head == xmit->tail)
        {
          xmit->tail = (xmit->tail + MAX(xmit->size / 4, 1)) % xmit->size;
          uart_datasent(dev);
        }
    }

  uart_spinunlock(dev, true, flags);
}

/****************************************************************************
 * Name: lwl_poll
 *
 * Description:
 *   Polling timer expiration.
 *
 ****************************************************************************/

#ifdef LWL_POLL
static void lwl_poll(wdparm_t arg)
{
  FAR struct uart_dev_s *dev = (FAR struct uart_dev_s *)arg;

  lwl_service(dev);
  lwl_pollstart(dev);
}
#endif

/****************************************************************************
 * Name: lwl_interrupt
 *
 * Description:
 *   Doorbell interrupt, raised by the host after it updated the link.
 *
 ****************************************************************************/

#ifdef CONFIG_LWL_CONSOLE_DOORBELL
static int lwl_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct uart_dev_s *dev = arg;

  /* Acknowledge first, so that a new ring during the service is kept */

  *(FAR volatile uint32_t *)LWL_DBADDR = 0;

  lwl_service(dev);
  return OK;
}
#endif

/****************************************************************************
 * Name: lwl_setup
 ****************************************************************************/

static int lwl_setup(FAR struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: lwl_shutdown
 ****************************************************************************/

static void lwl_shutdown(FAR struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: lwl_attach
 ****************************************************************************/

static int lwl_attach(FAR struct uart_dev_s *dev)
{
  FAR struct lwl_priv_s *priv = dev->priv;
#ifdef CONFIG_LWL_CONSOLE_DOORBELL
  int ret;

  ret = irq_attach(LWL_DBIRQ, lwl_interrupt, dev);
  if (ret < 0)
    {
      return ret;
    }

  up_enable_irq(LWL_DBIRQ);
#endif

  priv->attached = true;
  lwl_pollstart(dev);
  return OK;
}

/****************************************************************************
 * Name: lwl_detach
 ****************************************************************************/

static void lwl_detach(FAR struct uart_dev_s *dev)
{
  FAR struct lwl_priv_s *priv = dev->priv;

#ifdef CONFIG_LWL_CONSOLE_DOORBELL
  up_disable_irq(LWL_DBIRQ);
  irq_detach(LWL_DBIRQ);
#endif

  priv->attached = false;
#ifdef LWL_POLL
  wd_cancel(&priv->wdog);
#endif
}

/****************************************************************************
 * Name: lwl_ioctl
 ****************************************************************************/

static int lwl_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: lwl_recvbuf
 *
 * Description:
 *   Copy the unread octets of the current downword, acknowledging it when
 *   all of them have been consumed.
 *
 ****************************************************************************/

static ssize_t lwl_recvbuf(FAR struct uart_dev_s *dev,
                           FAR void *buf, size_t len)
{
  FAR struct lwl_priv_s *priv = dev->priv;
  FAR uint8_t *dest = buf;
  uint32_t down;
  size_t nbytes = 0;

  if (lwl_link(dev, &down) && lwl_rxpending(priv, down))
    {
      while (nbytes < len && priv->rxoff < LWL_GETOCTVAL(down))
        {
          dest[nbytes++] = (uint8_t)(down >> (8 * priv->rxoff++));
        }

      if (priv->rxoff >= LWL_GETOCTVAL(down))
        {
          lwl_rxack(priv, down);
        }
    }

  return nbytes;
}

/****************************************************************************
 * Name: lwl_receive
 ****************************************************************************/

static int lwl_receive(FAR struct uart_dev_s *dev, FAR unsigned int *status)
{
  uint8_t ch = 0;

  *status = lwl_recvbuf(dev, &ch, 1) == 1 ? 0 : -EAGAIN;
  return ch;
}

/****************************************************************************
 * Name: lwl_rxint
 ****************************************************************************/

static void lwl_rxint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct lwl_priv_s *priv = dev->priv;

  priv->rxint = enable;
  if (enable)
    {
      /* Data may be waiting because the RX buffer was full */

      lwl_service(dev);
    }
}

/****************************************************************************
 * Name: lwl_rxavailable
 *
 * Description:
 *   Return true if there is downlink data that can be stored.  While the RX
 *   buffer is full the downword is left unacknowledged, which stops the
 *   host until the application reads, so no input is ever lost.
 *
 ****************************************************************************/

static bool lwl_rxavailable(FAR struct uart_dev_s *dev)
{
  FAR struct lwl_priv_s *priv = dev->priv;
  FAR struct uart_buffer_s *recv = &dev->recv;
  uint32_t down;
  int head;

  if (!lwl_link(dev, &down) || !lwl_rxpending(priv, down))
    {
      return false;
    }

  if (LWL_GETPORT(down) != LWL_PORT_CONSOLE ||
      priv->rxoff >= LWL_GETOCTVAL(down))
    {
      /* Nothing for us in this word, just let the host go on */

      lwl_rxack(priv, down);
      return false;
    }

  head = recv->head + 1 < recv->size ? recv->head + 1 : 0;
  return head != recv->tail;
}

/****************************************************************************
 * Name: lwl_sendbuf
 *
 * Description:
 *   Send octets to the host.  The data is dropped if there is no host to
 *   collect it.
 *
 ****************************************************************************/

static ssize_t lwl_sendbuf(FAR struct uart_dev_s *dev,
                           FAR const void *buf, size_t len)
{
  FAR struct lwl_priv_s *priv = dev->priv;
  FAR const uint8_t *src = buf;
  uint32_t down;
  size_t i;

  if (!lwl_link(dev, &down) || lwl_txroom(priv, down) == 0)
    {
      return len;
    }

  /* The line replayed to a new host goes first */

  if (priv->replayon)
    {
      lwl_replay(priv, down);
      return 0;
    }

  len = lwl_put(priv, down, src, len);

  /* Remember the line in progress, to replay it to the next host */

  for (i = 0; i < len; i++)
    {
      if (src[i] == '\n')
        {
          priv->linelen = 0;
        }
      else if (priv->linelen < LWL_LINESIZE)
        {
          priv->line[priv->linelen++] = src[i];
        }
      else
        {
          priv->linelen = LWL_LINESIZE + 1;
        }
    }

  return len;
}

/****************************************************************************
 * Name: lwl_send
 ****************************************************************************/

static void lwl_send(FAR struct uart_dev_s *dev, int ch)
{
  uint8_t c = ch;

  lwl_sendbuf(dev, &c, 1);
}

/****************************************************************************
 * Name: lwl_txint
 ****************************************************************************/

static void lwl_txint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct lwl_priv_s *priv = dev->priv;

  priv->txint = enable;
  if (enable)
    {
      lwl_service(dev);
    }
}

/****************************************************************************
 * Name: lwl_txready
 *
 * Description:
 *   Return true if lwl_sendbuf() can be called: there is room for the data,
 *   or there is no host and the data will be dropped.
 *
 ****************************************************************************/

static bool lwl_txready(FAR struct uart_dev_s *dev)
{
  FAR struct lwl_priv_s *priv = dev->priv;
  uint32_t down;

  if (!lwl_link(dev, &down) || lwl_txroom(priv, down) > 0)
    {
      priv->spins   = 0;
      priv->stalled = false;
      return true;
    }

  /* Writers that cannot sleep (up_putc() and writes from interrupt
   * context) spin here.  Don't let a vanished host hang them forever.
   */

  if (!priv->stalled && ++priv->spins >= LWL_STALL_SPINS)
    {
      priv->stalled = true;
    }

  return priv->stalled;
}

/****************************************************************************
 * Name: lwl_txempty
 ****************************************************************************/

static bool lwl_txempty(FAR struct uart_dev_s *dev)
{
  FAR struct lwl_priv_s *priv = dev->priv;
  uint32_t down;

  if (!lwl_link(dev, &down))
    {
      return true;
    }

#if LWL_RINGSIZE > 0
  if (priv->usering)
    {
      return lwl_txroom(priv, down) == LWL_RINGSIZE - 1;
    }
#endif

  return lwl_txroom(priv, down) > 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Low-level, unbuffered console output used by syslog and by the assert
 *   logic.  It waits for room in the link.
 *
 ****************************************************************************/

void up_putc(int ch)
{
  irqstate_t flags;
  uint8_t c = ch;

  flags = uart_spinlock(&g_lwl_dev, false);

  do
    {
      while (!lwl_txready(&g_lwl_dev))
        {
        }
    }
  while (lwl_sendbuf(&g_lwl_dev, &c, 1) == 0);

  uart_spinunlock(&g_lwl_dev, false, flags);
}

/****************************************************************************
 * Name: lwlconsole_init
 *
 * Description:
 *   Register the LWL console as /dev/console.
 *
 ****************************************************************************/

void lwlconsole_init(void)
{
  uart_register("/dev/console", &g_lwl_dev);
}
