/****************************************************************************
 * drivers/net/slip.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Reference: RFC 1055
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <wdog.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/net.h>

#include <net/uip/uip.h>
#include <net/uip/uip-arp.h>
#include <net/uip/uip-arch.h>

#if defined(CONFIG_NET) && defined(CONFIG_NET_SLIP)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if UIP_LLH_LEN > 0
#  error "CONFIG_NET_LLH_LEN must be set to zero"
#endif

#ifndef CONFIG_SLIP_STACKSIZE
#  define CONFIG_SLIP_STACKSIZE 1024
#endif

#ifndef CONFIG_SLIP_DEFPRIO
#  define CONFIG_SLIP_DEFPRIO 128
#endif

/*  SLIP special character codes *******************************************/

#define SLIP_END      0300    /* Indicates end of packet */
#define SLIP_ESC      0333    /* Indicates byte stuffing */
#define SLIP_ESC_END  0334    /* ESC ESC_END means SLIP_END data byte */
#define SLIP_ESC_ESC  0335    /* ESC ESC_ESC means ESC data byte */

/* General driver definitions **********************************************/

/* CONFIG_SLIP_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_SLIP_NINTERFACES
# define CONFIG_SLIP_NINTERFACES 1
#endif

/* TX poll deley = 1 seconds. CLK_TCK is the number of clock ticks per second */

#define SLIP_WDDELAY   (1*CLK_TCK)
#define SLIP_POLLHSEC  (1*2)

/* TX timeout = 1 minute */

#define SLIP_TXTIMEOUT (60*CLK_TCK)

/* Statistics helper */

#ifdef CONFIG_NET_STATISTICS
#  define SLIP_STAT(p,f) (p->stats.f)++
#else
#  define SLIP_STAT(p,f)
#endif

/* This is a helper pointer for accessing the contents of the Ethernet header */

#define BUF ((struct uip_eth_hdr *)priv->dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Driver statistics */

#ifdef CONFIG_NET_STATISTICS
struct slip_statistics_s
{
  uint32_t transmitted;     /* Number of packets transmitted */
                            /* Number of packets received is the sum of: */
  uint32_t rxip;            /* Number of IP packets received */
  uint32_t rxarp;           /* + Number of ARP packets received */
  uint32_t rxproto;         /* + Number of unsupported packets received */
};
#endif

/* The slip_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct slip_driver_s
{
  bool     bifup;           /* true:ifup false:ifdown */
  WDOG_ID  txpoll;          /* TX poll timer */
  int      fd;              /* File descriptor associated with stream */
  FILE    *stream;          /* The contained serial stream */
  sem_t    waitsem;         /* Only used at start-up */

  /* Driver statistics */

#ifdef CONFIG_NET_STATISTICS
  struct slip_statistics_s stats;
#endif

  /* This holds the information visible to uIP/NuttX */

  struct uip_driver_s dev;  /* Interface understood by uIP */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

 /* We really should get rid of CONFIG_SLIP_NINTERFACES and, instead,
  * malloc() new interface instances as needed.
  */

static struct slip_driver_s g_slip[CONFIG_SLIP_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void slip_semtake(FAR struct slip_driver_s *priv);

/* Common TX logic */

static inline void slip_write(FAR struct slip_driver_s *priv,
                              const uint8_t *buffer, int len);
static inline void slip_putc(FAR struct slip_driver_s *priv, int ch);
static int  slip_transmit(FAR struct slip_driver_s *priv);
static int  slip_uiptxpoll(struct uip_driver_s *dev);

/* Interrupt handling */

static inline int slip_getc(FAR struct slip_driver_s *priv);
static inline void slip_readpacket(FAR struct slip_driver_s *priv);
static void slip_receive(FAR struct slip_driver_s *priv);
static int slip_rxtask(int argc, char *argv[]);

/* Watchdog timer expirations */

static void slip_polltimer(int argc, uint32_t arg, ...);

/* NuttX callback functions */

static int slip_ifup(struct uip_driver_s *dev);
static int slip_ifdown(struct uip_driver_s *dev);
static int slip_txavail(struct uip_driver_s *dev);
#ifdef CONFIG_NET_IGMP
static int slip_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
static int slip_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: slip_semtake
 ****************************************************************************/

static void slip_semtake(FAR struct slip_driver_s *priv)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&priv->waitsem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

#define slip_semgive(p) sem_post(&(p)->waitsem);

/****************************************************************************
 * Function: slip_write
 *
 * Description:
 *   Just an inline wrapper around fwrite with error checking.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *   buffer - Buffer data to send
 *   len - Buffer length in bytes
 *
 ****************************************************************************/

static inline void slip_write(FAR struct slip_driver_s *priv,
                              const uint8_t *buffer, int len)
{
#if CONFIG_DEBUG
  size_t nbytes = fwrite(buffer, 1, len, priv->stream);
  DEBUGASSERT(nbytes == len);
#else
  (void)fwrite(buffer, 1, len, priv->stream);
#endif
}

/****************************************************************************
 * Function: slip_putc
 *
 * Description:
 *   Just an inline wrapper around putc with error checking.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *   ch - The character to send
 *
 ****************************************************************************/

static inline void slip_putc(FAR struct slip_driver_s *priv, int ch)
{
#if CONFIG_DEBUG
  int ret = putc(ch, priv->stream);
  DEBUGASSERT(ret == ch);
#else
  putc(ch, priv->stream);
#endif
}

/****************************************************************************
 * Function: slip_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int slip_transmit(FAR struct slip_driver_s *priv)
{
  uint8_t *src;
  uint8_t *start;
  uint8_t esc;
  int remaining;
  int len;

  /* Increment statistics */

  SLIP_STAT(priv, transmitted);

  /* Send an initial END character to flush out any data that may have
   * accumulated in the receiver due to line noise
   */

  slip_putc(priv, SLIP_END);

  /* For each byte in the packet, send the appropriate character sequence */

  src       = priv->dev.d_buf;
  remaining = priv->dev.d_len;
  start     = src;
  
  while (remaining-- > 0)
    {
      switch (*src)
        {
          /* If it's the same code as an END character, we send a special two
           * character code so as not to make the receiver think we sent an
           * END
           */

          case SLIP_END:
            esc = SLIP_ESC_END;
            goto escape;

          /* if it's the same code as an ESC character, we send a special two
           * character code so as not to make the receiver think we sent an
           * ESC
           */

          case SLIP_ESC:
            esc = SLIP_ESC_ESC;

          escape:
            {
              /* Flush any unsent data */

              if (len > 0)
                {
                  slip_write(priv, start, len);

                  /* Reset */

                  start = src + 1;
                  len   = 0;
                }

              /* Then send the escape sequence */

              slip_putc(priv, SLIP_ESC);
              slip_putc(priv, esc);
            }
          break;

          /* otherwise, just bump up the count */

          default:
            len++;
            break;
        }

      /* Point to the next character in the packet */

      src++;
    }

  /* We have looked at every charcter in the packet.  Now flush any unsent
   * data
   */

  if (len > 0)
    {
      slip_write(priv, start, len);
    }

  /* And send the END token */

  slip_putc(priv, SLIP_END);
  return OK;
}

/****************************************************************************
 * Function: slip_uiptxpoll
 *
 * Description:
 *   The transmitter is available, check if uIP has any outgoing packets ready
 *   to send.  This is a callback from uip_poll().  uip_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int slip_uiptxpoll(struct uip_driver_s *dev)
{
  FAR struct slip_driver_s *priv = (FAR struct slip_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      uip_arp_out(&priv->dev);
      slip_transmit(priv);
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: slip_getc
 *
 * Description:
 *   Get one byte from the serial input.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   The returned byte
 *
 ****************************************************************************/

static inline int slip_getc(FAR struct slip_driver_s *priv)
{
  int ret = getc(priv->stream);
  DEBUGASSERT(ret != EOF);
  return ret;
}

/****************************************************************************
 * Function: slip_readpacket
 *
 * Description:
 *   Read a packet from the serial input
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void slip_readpacket(FAR struct slip_driver_s *priv)
{
  uint8_t ch;
  int     nbytes = 0;

  /* Copy the data data from the hardware to priv->dev.d_buf until we
   * put together a whole packet. Make sure not to copy them into the
   * packet if we run out of room.
   */

  for (;;)
    {
      /* Get the next character in the stream */

      ch = slip_getc(priv);

      /* Handle bytestuffing if necessary */
 
      switch (ch)
        {
        /* If it's an END character then we're done with the packet.
         * (OR we are just starting a packet)
         */

        case SLIP_END:
          /* a minor optimization: if there is no data in the packet, ignore
           * it. This is meant to avoid bothering IP with all the empty
           * packets generated by the duplicate END characters which are in
           * turn sent to try to detect line noise.
           */

          if (nbytes > 0)
            {
              priv->dev.d_len = nbytes;
              return;
            }
          break;

        /* if it's the same code as an ESC character, wait and get another
         * character and then figure out what to store in the packet based
         * on that.
         */

        case SLIP_ESC:
          ch = slip_getc(priv);

          /* if "ch" is not one of these two, then we have a protocol
           * violation.  The best bet seems to be to leave the byte alone
           * and just stuff it into the packet
           */

          switch (ch)
            {
            case SLIP_ESC_END:
              ch = SLIP_END;
              break;
             case SLIP_ESC_ESC:
              ch = SLIP_ESC;
              break;
            default:
              ndbg("ERROR: Protocol violation: %02x\n", ch);
              break;
            }

          /* Here we fall into the default handler and let it store the
           * character for us
           */

        default:
          if (nbytes < priv->dev.d_len)
            {
              priv->dev.d_buf[nbytes++] = ch;
            }
          break;
        }
    }
}

/****************************************************************************
 * Function: slip_receive
 *
 * Description:
 *   Accept and handle an incoming packet.
 *
 * Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void slip_receive(FAR struct slip_driver_s *priv)
{
  /* Copy the data data from the hardware to priv->dev.d_buf until we put
   * together a whole packet.
   */

  slip_readpacket(priv);

   /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv6
  if (BUF->type == HTONS(UIP_ETHTYPE_IP6))
#else
  if (BUF->type == HTONS(UIP_ETHTYPE_IP))
#endif
    {
      SLIP_STAT(priv, rxip);
      uip_arp_ipin(&priv->dev);
      uip_input(&priv->dev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, the field  d_len will set to a value > 0.
       */

      if (priv->dev.d_len > 0)
        {
          uip_arp_out(&priv->dev);
          slip_transmit(priv);
        }
    }
  else if (BUF->type == htons(UIP_ETHTYPE_ARP))
    {
      SLIP_STAT(priv, rxarp);
      uip_arp_arpin(&priv->dev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, the field  d_len will set to a value > 0.
       */

      if (priv->dev.d_len > 0)
        {
          slip_transmit(priv);
        }
    }
  else
    {
      SLIP_STAT(priv, rxproto);
    }
}

/****************************************************************************
 * Function: slip_rxtask
 *
 * Description:
 *   Wait for incoming data.
 *
 * Parameters:
 *   argc
 *   argv
 *
 * Returned Value:
 *   (Does not return)
 *
 * Assumptions:
 *
 ****************************************************************************/

static int slip_rxtask(int argc, char *argv[])
{
  FAR struct slip_driver_s *priv;
  struct pollfd fds[1];
  unsigned int index = *(argv[1]) - '0';
  irqstate_t flags;
  int ret;

  ndbg("index: %d\n", index);
  DEBUGASSERT(index < CONFIG_SLIP_NINTERFACES);

  /* Get our private data structure instance */

  priv = &g_slip[index];
  slip_semgive(priv);

  /* Loop forever */

  for (;;)
    {
      /* Set up the poll */

      memset(fds, 0, sizeof(struct pollfd));
      fds[0].fd      = priv->fd;
      fds[0].events  = POLLIN;
      fds[0].revents = 0;

      /* Wait for incoming data (with no timeout) */

      ret = poll(fds, 1, -1);
      DEBUGASSERT(ret >= 0);
      if (ret > 0)
        {
          /* Call slip_receive() to handle incoming data.  Interrupts must
           * be disabled here to keept the POLL watchdoe from firing.
           */

          flags = irqsave();
          slip_receive(priv);
          irqrestore(flags);
        }
    }

  /* We won't get here */

  return OK;
}

/****************************************************************************
 * Function: slip_polltimer
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void slip_polltimer(int argc, uint32_t arg, ...)
{
  FAR struct slip_driver_s *priv = (FAR struct slip_driver_s *)arg;

  /* If so, update TCP timing states and poll uIP for new XMIT data. */

  (void)uip_timer(&priv->dev, slip_uiptxpoll, SLIP_POLLHSEC);

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->txpoll, SLIP_WDDELAY, slip_polltimer, 1, arg);
}

/****************************************************************************
 * Function: slip_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided 
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int slip_ifup(struct uip_driver_s *dev)
{
  FAR struct slip_driver_s *priv = (FAR struct slip_driver_s *)dev->d_private;

  ndbg("Bringing up: %d.%d.%d.%d\n",
       dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
       (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24 );

  /* Set and activate a timer process */

  (void)wd_start(priv->txpoll, SLIP_WDDELAY, slip_polltimer, 1, (uint32_t)priv);

  /* Mark the interface up */

  priv->bifup = true;
  return OK;
}

/****************************************************************************
 * Function: slip_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int slip_ifdown(struct uip_driver_s *dev)
{
  FAR struct slip_driver_s *priv = (FAR struct slip_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the interrupts */

  flags = irqsave();

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(priv->txpoll);

  /* Mark the device "down" */

  priv->bifup = false;
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: slip_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a 
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int slip_txavail(struct uip_driver_s *dev)
{
  FAR struct slip_driver_s *priv = (FAR struct slip_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable interrupts because this function may be called from interrupt
   * level processing.
   */

  flags = irqsave();

  /* Ignore the notification if the interface is not yet up */

  if (priv->bifup)
    {
      /* Poll uIP for new XMIT data */

      (void)uip_poll(&priv->dev, slip_uiptxpoll);
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Function: slip_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added 
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int slip_addmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct slip_driver_s *priv = (FAR struct slip_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Function: slip_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed 
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int slip_rmmac(struct uip_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct slip_driver_s *priv = (FAR struct slip_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: slip_initialize
 *
 * Description:
 *   Instantiate a SLIP network interface.
 *
 * Parameters:
 *   intf - In the case where there are multiple SLIP interfaces, this value
 *          identifies which is to be initialized.  The network name will be,
 *          for example, "/dev/slip5" for intf == 5
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int slip_initialize(int intf, const char *devname)
{
  struct slip_driver_s *priv;
  char buffer[IFNAMSIZ];
  pid_t pid;

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(inf < CONFIG_SLIP_NINTERFACES);
  priv = &g_slip[intf];

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct slip_driver_s));
  priv->dev.d_ifup    = slip_ifup;     /* I/F down callback */
  priv->dev.d_ifdown  = slip_ifdown;   /* I/F up (new IP address) callback */
  priv->dev.d_txavail = slip_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->dev.d_addmac  = slip_addmac;   /* Add multicast MAC address */
  priv->dev.d_rmmac   = slip_rmmac;    /* Remove multicast MAC address */
#endif
  priv->dev.d_private = priv;          /* Used to recover private state from dev */

  /* Open the device */

  priv->fd            = open(devname, O_RDWR, 0666);
  if (priv->fd < 0)
    {
      ndbg("ERROR: Failed to open %s: %d\n", devname, errno);
      return -errno;
    }

  priv->stream        = fdopen(priv->fd, "rw");
  if (priv->stream == NULL)
    {
      ndbg("ERROR: Failed to fdopen %s: %d\n", devname, errno);
      close(priv->fd);
      return -errno;
    }

  /* Initialize the wait semaphore */

  sem_init(&priv->waitsem, 0, 0);

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling slip_ifdown().
   */

  slip_ifdown(&priv->dev);

  /* Start the SLIP receiver task */

  snprintf(buffer, IFNAMSIZ, "%d", intf);

#ifndef CONFIG_CUSTOM_STACK
  pid = task_create("usbhost", CONFIG_SLIP_DEFPRIO, CONFIG_SLIP_STACKSIZE,
                    (main_t)slip_rxtask, (const char **)buffer);
#else
  pid = task_create("usbhost", CONFIG_SLIP_DEFPRIO,
                    (main_t)slip_rxtask, (const char **)buffer);
#endif
  if (pid < 0)
    {
      ndbg("ERROR: Failed to start receiver task\n");
      return -errno;
    }

  /* Wait and make sure that the receive task is started. */

  slip_semtake(priv);

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->txpoll       = wd_create();   /* Create periodic poll timer */

  /* Register the device with the OS so that socket IOCTLs can be performed */

#if CONFIG_NSOCKET_DESCRIPTORS > 0
  snprintf(buffer, IFNAMSIZ, "slip%d", intf);
#endif
  (void)netdev_register(&priv->dev);
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_SLIP */

