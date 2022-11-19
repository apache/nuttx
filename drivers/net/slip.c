/****************************************************************************
 * drivers/net/slip.c
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

/* Reference: RFC 1055 */

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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/signal.h>
#include <nuttx/kthread.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/slip.h>

#if defined(CONFIG_NET) && defined(CONFIG_NET_SLIP)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* NOTE:  Slip requires UART hardware handshake.  If hardware handshake is
 * not available with your UART, then you might try the 'slattach' option
 * -L which enable "3-wire operation."  That allows operation without the
 * hardware handshake (but with the possibility of data overrun).
 */

/* Configuration ************************************************************/

#ifndef CONFIG_NET_SLIP_STACKSIZE
#  define CONFIG_NET_SLIP_STACKSIZE 2048
#endif

#ifndef CONFIG_NET_SLIP_DEFPRIO
#  define CONFIG_NET_SLIP_DEFPRIO 128
#endif

/* The Linux slip module hard-codes its MTU size to 296 (40 bytes for the
 * IP+TCP headers plus 256 bytes of data).  So you might as well set
 * CONFIG_NET_SLIP_PKTSIZE to 296 as well.
 *
 * There may be an issue with this setting, however.  I see that Linux uses
 * a MTU of 296 and window of 256, but actually only sends 168 bytes of data:
 * 40 + 128.  I believe that is to allow for the 2x worst cast packet
 * expansion.  Ideally we would like to advertise the 256 MSS, but restrict
 * transfers to 128 bytes (possibly by modifying the tcp_mss() macro).
 */

#if CONFIG_NET_SLIP_PKTSIZE < 296
#  error "CONFIG_NET_SLIP_PKTSIZE >= 296 is required"
#endif

/* CONFIG_NET_SLIP_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_NET_SLIP_NINTERFACES
# define CONFIG_NET_SLIP_NINTERFACES 1
#endif

/*  SLIP special character codes ********************************************/

#define SLIP_END      0300    /* Indicates end of packet */
#define SLIP_ESC      0333    /* Indicates byte stuffing */
#define SLIP_ESC_END  0334    /* ESC ESC_END means SLIP_END data byte */
#define SLIP_ESC_ESC  0335    /* ESC ESC_ESC means ESC data byte */

/* General driver definitions ***********************************************/

/* TX poll delay = 1 second = 1000000 microseconds. */

#define SLIP_WDDELAY   (1*1000000)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The slip_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct slip_driver_s
{
  volatile bool bifup;      /* true:ifup false:ifdown */
  bool          txnodelay;  /* True: nxsig_usleep() not needed */
  struct file   file;       /* TTY file descriptor */
  uint16_t      rxlen;      /* The number of bytes in rxbuf */
  pid_t         rxpid;      /* Receiver thread ID */
  pid_t         txpid;      /* Transmitter thread ID */
  sem_t         waitsem;    /* Mutually exclusive access to the network */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev;  /* Interface understood by the network */
  uint8_t rxbuf[CONFIG_NET_SLIP_PKTSIZE + 2];
  uint8_t txbuf[CONFIG_NET_SLIP_PKTSIZE + 2];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* We really should get rid of CONFIG_NET_SLIP_NINTERFACES and, instead,
 * kmm_malloc() new interface instances as needed.
 */

static struct slip_driver_s g_slip[CONFIG_NET_SLIP_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static void slip_write(FAR struct slip_driver_s *priv,
                       FAR const uint8_t *buffer, int len);
static void slip_putc(FAR struct slip_driver_s *priv, int ch);
static void slip_transmit(FAR struct slip_driver_s *priv);
static int slip_txpoll(FAR struct net_driver_s *dev);
static int slip_txtask(int argc, FAR char *argv[]);

/* Packet receiver task */

static int slip_getc(FAR struct slip_driver_s *priv);
static inline void slip_receive(FAR struct slip_driver_s *priv);
static int slip_rxtask(int argc, FAR char *argv[]);

/* NuttX callback functions */

static int slip_ifup(FAR struct net_driver_s *dev);
static int slip_ifdown(FAR struct net_driver_s *dev);
static int slip_txavail(FAR struct net_driver_s *dev);
#ifdef CONFIG_NET_MCASTGROUP
static int slip_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
static int slip_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: slip_write
 *
 * Description:
 *   Just an inline wrapper around fwrite with error checking.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *   buffer - Buffer data to send
 *   len - Buffer length in bytes
 *
 ****************************************************************************/

static inline void slip_write(FAR struct slip_driver_s *priv,
                              FAR const uint8_t *buffer, int len)
{
  /* Handle the case where the write is awakened by a signal */

  while (file_write(&priv->file, buffer, len) < 0)
    {
    }
}

/****************************************************************************
 * Name: slip_putc
 *
 * Description:
 *   Just an inline wrapper around putc with error checking.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *   ch - The character to send
 *
 ****************************************************************************/

static inline void slip_putc(FAR struct slip_driver_s *priv, int ch)
{
  uint8_t buffer = (uint8_t)ch;
  slip_write(priv, &buffer, 1);
}

/****************************************************************************
 * Name: slip_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static void slip_transmit(FAR struct slip_driver_s *priv)
{
  uint8_t *src;
  uint8_t *start;
  uint8_t  esc;
  int      remaining;
  int      len;

  /* Increment statistics */

  ninfo("Sending packet size %d\n", priv->dev.d_len);
  NETDEV_TXPACKETS(&priv->dev);

  /* Send an initial END character to flush out any data that may have
   * accumulated in the receiver due to line noise
   */

  slip_putc(priv, SLIP_END);

  /* For each byte in the packet, send the appropriate character sequence */

  src       = priv->dev.d_buf;
  remaining = priv->dev.d_len;
  start     = src;
  len       = 0;

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

          /* If it's the same code as an ESC character, we send a special two
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
                }

              /* Reset */

              start = src + 1;
              len   = 0;

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

  /* We have looked at every character in the packet.  Now flush any unsent
   * data
   */

  if (len > 0)
    {
      slip_write(priv, start, len);
    }

  /* And send the END token */

  slip_putc(priv, SLIP_END);
  NETDEV_TXDONE(&priv->dev);
  priv->txnodelay = true;
}

/****************************************************************************
 * Name: slip_txpoll
 *
 * Description:
 *   Check if the network has any outgoing packets ready to send.  This is a
 *   callback from devif_poll().  devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete, or
 *   2. During normal periodic polling
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   The initiator of the poll holds the priv->waitsem;
 *
 ****************************************************************************/

static int slip_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct slip_driver_s *priv =
    (FAR struct slip_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      if (!devif_loopback(&priv->dev))
        {
          slip_transmit(priv);
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Name: slip_txtask
 *
 * Description:
 *   Polling and transmission is performed on tx thread.
 *
 * Input Parameters:
 *   arg  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int slip_txtask(int argc, FAR char *argv[])
{
  FAR struct slip_driver_s *priv;
  unsigned int index = *(argv[1]) - '0';
  int ret;

  nerr("index: %d\n", index);
  DEBUGASSERT(index < CONFIG_NET_SLIP_NINTERFACES);

  /* Get our private data structure instance and wake up the waiting
   * initialization logic.
   */

  priv = &g_slip[index];
  nxsem_post(&priv->waitsem);

  /* Loop forever */

  for (; ; )
    {
      /* Wait for the timeout to expire (or until we are signaled by  */

      ret = nxsem_wait_uninterruptible(&priv->waitsem);
      if (ret < 0)
        {
          DEBUGASSERT(ret == -ECANCELED);
          break;
        }

      if (!priv->txnodelay)
        {
          nxsem_post(&priv->waitsem);
          nxsig_usleep(SLIP_WDDELAY);
        }
      else
        {
          priv->txnodelay = false;
          nxsem_post(&priv->waitsem);
        }

      /* Is the interface up? */

      if (priv->bifup)
        {
          /* Poll the networking layer for new XMIT data. */

          net_lock();
          priv->dev.d_buf = priv->txbuf;

          /* perform the normal TX poll */

          devif_poll(&priv->dev, slip_txpoll);

          net_unlock();
        }
    }

  return OK;
}

/****************************************************************************
 * Name: slip_getc
 *
 * Description:
 *   Get one byte from the serial input.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   The returned byte
 *
 ****************************************************************************/

static inline int slip_getc(FAR struct slip_driver_s *priv)
{
  uint8_t ch;

  while (file_read(&priv->file, &ch, 1) < 0)
    {
    }

  return ch;
}

/****************************************************************************
 * Name: slip_receive
 *
 * Description:
 *   Read a packet from the serial input
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void slip_receive(FAR struct slip_driver_s *priv)
{
  uint8_t ch;

  /* Copy the data from the hardware to the RX buffer until we put
   * together a whole packet. Make sure not to copy them into the
   * packet if we run out of room.
   */

  ninfo("Receiving packet\n");
  for (; ; )
    {
      /* Get the next character in the stream. */

      ch = slip_getc(priv);

      /* Handle bytestuffing if necessary */

      switch (ch)
        {
        /* If it's an END character then we're done with the packet.
         * (OR we are just starting a packet)
         */

        case SLIP_END:
          {
            ninfo("END\n");

            /* A minor optimization: if there is no data in the packet,
             * ignore it. This is meant to avoid bothering IP with all the
             * empty packets generated by the duplicate END characters which
             * are in turn sent to try to detect line noise.
             */

            if (priv->rxlen > 0)
              {
                ninfo("Received packet size %d\n", priv->rxlen);
                return;
              }
          }
          break;

        /* if it's the same code as an ESC character, wait and get another
         * character and then figure out what to store in the packet based
         * on that.
         */

        case SLIP_ESC:
          {
            ninfo("ESC\n");
            ch = slip_getc(priv);

            /* if "ch" is not one of these two, then we have a protocol
             * violation.  The best bet seems to be to leave the byte alone
             * and just stuff it into the packet
             */

            switch (ch)
              {
              case SLIP_ESC_END:
                ninfo("ESC-END\n");
                ch = SLIP_END;
                break;

               case SLIP_ESC_ESC:
                ninfo("ESC-ESC\n");
                ch = SLIP_ESC;
                break;

              default:
                nerr("ERROR: Protocol violation: %02x\n", ch);
                break;
              }

            /* Here we fall into the default handler and let it store the
             * character for us
             */
          }

        default:
          {
            if (priv->rxlen < CONFIG_NET_SLIP_PKTSIZE + 2)
              {
                priv->rxbuf[priv->rxlen++] = ch;
              }
          }
          break;
        }
    }
}

/****************************************************************************
 * Name: slip_rxtask
 *
 * Description:
 *   Wait for incoming data.
 *
 * Input Parameters:
 *   argc
 *   argv
 *
 * Returned Value:
 *   (Does not return)
 *
 * Assumptions:
 *
 ****************************************************************************/

static int slip_rxtask(int argc, FAR char *argv[])
{
  FAR struct slip_driver_s *priv;
  FAR struct net_driver_s *dev;
  unsigned int index = *(argv[1]) - '0';
  int ch;

  nerr("index: %d\n", index);
  DEBUGASSERT(index < CONFIG_NET_SLIP_NINTERFACES);

  /* Get our private data structure instance and wake up the waiting
   * initialization logic.
   */

  priv = &g_slip[index];
  nxsem_post(&priv->waitsem);

  /* Loop forever */

  dev = &priv->dev;
  for (; ; )
    {
      /* Wait for the next character to be available on the input stream. */

      ninfo("Waiting...\n");
      ch = slip_getc(priv);

      /* Ignore any input that we receive before the interface is up. */

      if (!priv->bifup)
        {
          continue;
        }

      /* We have something...
       *
       * END characters may appear at packet boundaries BEFORE as well as
       * after the beginning of the packet.  This is normal and expected.
       */

      if (ch == SLIP_END)
        {
          priv->rxlen = 0;
        }

      /* Otherwise, we are in danger of being out-of-sync.  Apparently the
       * leading END character is optional.  Let's try to continue.
       */

      else
        {
          priv->rxbuf[0] = (uint8_t)ch;
          priv->rxlen    = 1;
        }

      /* Copy the data data from the hardware to priv->rxbuf until we put
       * together a whole packet.
       */

      slip_receive(priv);

      /* Handle the IP input.  Get exclusive access to the network. */

      net_lock();
      priv->dev.d_buf = priv->rxbuf;
      priv->dev.d_len = priv->rxlen;

      NETDEV_RXPACKETS(&priv->dev);

      /* All packets are assumed to be IP packets (we don't have a choice..
       * there is no Ethernet header containing the EtherType).  So pass the
       * received packet on for IP processing -- but only if it is big
       * enough to hold an IP header.
       */

#ifdef CONFIG_NET_IPv4
      if ((IPv4BUF->vhl & IP_VERSION_MASK) == IPv4_VERSION)
        {
          NETDEV_RXIPV4(&priv->dev);
          ipv4_input(&priv->dev);

          /* If the above function invocation resulted in data that should
           * be sent out on the network, the field  d_len will set to a
           * value > 0.  NOTE that we are transmitting using the RX buffer!
           */

          if (priv->dev.d_len > 0)
            {
              slip_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if ((IPv6BUF->vtc & IP_VERSION_MASK) == IPv6_VERSION)
        {
          NETDEV_RXIPV6(&priv->dev);
          ipv6_input(&priv->dev);

          /* If the above function invocation resulted in data that should
           * be sent out on the network, the field  d_len will set to a
           * value > 0.  NOTE that we are transmitting using the RX buffer!
           */

          if (priv->dev.d_len > 0)
            {
              slip_transmit(priv);
            }
        }
      else
#endif
        {
          NETDEV_RXERRORS(&priv->dev);
        }

      net_unlock();
    }

  /* We won't get here */

  return OK;
}

/****************************************************************************
 * Name: slip_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int slip_ifup(FAR struct net_driver_s *dev)
{
  FAR struct slip_driver_s *priv =
    (FAR struct slip_driver_s *)dev->d_private;

#ifdef CONFIG_NET_IPv4
  nerr("Bringing up: %d.%d.%d.%d\n",
        (int)(dev->d_ipaddr & 0xff), (int)((dev->d_ipaddr >> 8) & 0xff),
        (int)((dev->d_ipaddr >> 16) & 0xff), (int)(dev->d_ipaddr >> 24));
#endif
#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

  /* Mark the interface up */

  priv->bifup = true;
  netdev_carrier_on(dev);
  return OK;
}

/****************************************************************************
 * Name: slip_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int slip_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct slip_driver_s *priv =
    (FAR struct slip_driver_s *)dev->d_private;

  netdev_carrier_off(dev);

  /* Mark the device "down" */

  priv->bifup = false;
  return OK;
}

/****************************************************************************
 * Name: slip_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int slip_txavail(FAR struct net_driver_s *dev)
{
  FAR struct slip_driver_s *priv =
    (FAR struct slip_driver_s *)dev->d_private;

  /* Ignore the notification if the interface is not yet up */

  if (priv->bifup)
    {
      /* Wake up the TX polling thread */

      priv->txnodelay = true;
      nxsig_kill(priv->txpid, SIGALRM);
    }

  return OK;
}

/****************************************************************************
 * Name: slip_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int slip_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct slip_driver_s *priv =
    (FAR struct slip_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Name: slip_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware
 *   multicast address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int slip_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct slip_driver_s *priv =
    (FAR struct slip_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: slip_initialize
 *
 * Description:
 *   Instantiate a SLIP network interface.
 *
 * Input Parameters:
 *   intf    - In the case where there are multiple SLIP interfaces, this
 *             value identifies which is to be initialized. The number of
 *             possible SLIP interfaces is determined by
 *   devname - This is the path to the serial device that will support SLIP.
 *             For example, this might be "/dev/ttyS1"
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int slip_initialize(int intf, FAR const char *devname)
{
  FAR struct slip_driver_s *priv;
  char buffer[8];
  FAR char *argv[2];
  int ret;

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(intf < CONFIG_NET_SLIP_NINTERFACES);
  priv = &g_slip[intf];

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct slip_driver_s));
  priv->dev.d_ifup    = slip_ifup;     /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = slip_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = slip_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac  = slip_addmac;   /* Add multicast MAC address */
  priv->dev.d_rmmac   = slip_rmmac;    /* Remove multicast MAC address */
#endif
  priv->dev.d_private = priv;          /* Used to recover private state from dev */

  /* Open the device */

  ret = file_open(&priv->file, devname, O_RDWR, 0666);
  if (ret < 0)
    {
      nerr("ERROR: Failed to open %s: %d\n", devname, ret);
      return ret;
    }

  /* Initialize the wait semaphore */

  nxsem_init(&priv->waitsem, 0, 0);

  /* Start the SLIP receiver kernel thread */

  snprintf(buffer, 8, "%d", intf);
  argv[0] = buffer;
  argv[1] = NULL;

  ret = kthread_create("rxslip", CONFIG_NET_SLIP_DEFPRIO,
                       CONFIG_NET_SLIP_STACKSIZE, slip_rxtask, argv);
  if (ret < 0)
    {
      nerr("ERROR: Failed to start receiver task\n");
      return ret;
    }

  priv->rxpid = (pid_t)ret;

  /* Wait and make sure that the receive task is started. */

  nxsem_wait_uninterruptible(&priv->waitsem);

  /* Start the SLIP transmitter kernel thread */

  ret = kthread_create("txslip", CONFIG_NET_SLIP_DEFPRIO,
                       CONFIG_NET_SLIP_STACKSIZE, slip_txtask, argv);
  if (ret < 0)
    {
      nerr("ERROR: Failed to start receiver task\n");
      return ret;
    }

  priv->txpid = (pid_t)ret;

  /* Wait and make sure that the transmit task is started. */

  nxsem_wait_uninterruptible(&priv->waitsem);

  /* Bump the semaphore count so that it can now be used as a mutex */

  nxsem_post(&priv->waitsem);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->dev, NET_LL_SLIP);
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_SLIP */
