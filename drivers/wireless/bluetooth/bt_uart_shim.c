/****************************************************************************
 * drivers/wireless/bluetooth/bt_uart_shim.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Dave Marples <dave@marples.net>
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

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/semaphore.h>
#include <nuttx/serial/tioctl.h>
#include <nuttx/wireless/bluetooth/bt_uart.h>
#include <nuttx/wireless/bluetooth/bt_uart_shim.h>
#include <termios.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure is the variable state of the binding to the UART */

struct hciuart_state_s
{
  /* Registered Rx callback */

  btuart_rxcallback_t callback; /* Rx callback function */
  FAR void *arg;                /* Rx callback argument */

  int h;                        /* File handle to serial device */
  struct file f;                /* File structure, detached */

  sem_t dready;                 /* Semaphore used by the poll operation */
  bool enabled;                 /* Flag indicating that reception is enabled */

  int serialmontask;            /* The receive serial octets task handle */
  volatile struct pollfd p;     /* Polling structure for serial monitor task */
};

struct hciuart_config_s
{
  /* Setup the interface from the upper to the lower */

  struct btuart_lowerhalf_s lower;    /* Generic UART lower half */
  struct hciuart_state_s state;       /* Variable state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* UART Lower-Half Methods */

static void hciuart_rxattach(FAR const struct btuart_lowerhalf_s *lower,
                             btuart_rxcallback_t callback, FAR void *arg);
static void hciuart_rxenable(FAR const struct btuart_lowerhalf_s *lower,
                             bool enable);
static int hciuart_setbaud(FAR const struct btuart_lowerhalf_s *lower,
                           uint32_t baud);
static ssize_t hciuart_read(FAR const struct btuart_lowerhalf_s *lower,
                            FAR void *buffer, size_t buflen);
static ssize_t hciuart_write(FAR const struct btuart_lowerhalf_s *lower,
                             FAR const void *buffer, size_t buflen);
static ssize_t hciuart_rxdrain(FAR const struct btuart_lowerhalf_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure is the configuration of the HCI UART shim */

static struct btuart_lowerhalf_s g_lowerstatic =
{
  .rxattach = hciuart_rxattach,
  .rxenable = hciuart_rxenable,
  .setbaud = hciuart_setbaud,
  .read = hciuart_read,
  .write = hciuart_write,
  .rxdrain = hciuart_rxdrain
};

/* This is held global because its inconvenient to pass to the task */

static FAR struct hciuart_config_s *g_n;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hciuart_rxattach
 *
 * Description:
 *   Attach/detach the upper half Rx callback.
 *
 *   rxattach() allows the upper half logic to attach a callback function
 *   that will be used to inform the upper half that an Rx frame is
 *   available.  This callback will, most likely, be invoked in the
 *   context of an interrupt callback.  The receive() method should then
 *   be invoked in order to receive the obtain the Rx frame data.
 *
 ****************************************************************************/

static void
hciuart_rxattach(FAR const struct btuart_lowerhalf_s *lower,
                 btuart_rxcallback_t callback, FAR void *arg)
{
  struct hciuart_config_s *config = (struct hciuart_config_s *)lower;
  struct hciuart_state_s *state;
  irqstate_t flags;

  state = &config->state;

  /* If the callback is NULL, then we are detaching */

  flags = spin_lock_irqsave();
  if (callback == NULL)
    {
      /* Disable Rx callbacks and detach the Rx callback */

      state->callback = NULL;
      state->arg = NULL;
    }

  /* Otherwise, we are attaching */

  else
    {
      state->callback = NULL;
      state->arg = arg;
      state->callback = callback;
    }

  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name: hciuart_rxenable
 *
 * Description:
 *   Enable/disable RX callbacks from the HCI UART.
 *
 *   hciuart_rxenable() may be used to enable or disable callback events.
 *   This probably translates to enabling and disabled Rx interrupts at
 *   the UART.  NOTE:  Rx event notification should be done sparingly:
 *   Rx data overrun may occur when Rx events are disabled!
 *
 ****************************************************************************/

static void hciuart_rxenable(FAR const struct btuart_lowerhalf_s *lower,
                             bool enable)
{
  FAR struct hciuart_config_s *config = (FAR struct hciuart_config_s *)lower;
  FAR struct hciuart_state_s *s = &config->state;

  irqstate_t flags = spin_lock_irqsave();
  if (enable != s->enabled)
    {
      wlinfo(enable?"Enable\n":"Disable\n");
    }

  s->enabled = enable;

  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name: hciuart_setbaud
 *
 * Description:
 *   The HCI UART comes up with some initial BAUD rate.  Some support
 *   auto-BAUD detection, some support writing a configuration file to
 *   select the initial BAUD.  The simplest strategy, however, is simply
 *   to use the HCI UART's default initial BAUD to perform the basic
 *   bring up, then send a vendor-specific command to increase the HCI
 *   UARTs BAUD.  This method then may be used to adjust the lower half
 *   driver to the new HCI UART BAUD.
 *
 ****************************************************************************/

static int
hciuart_setbaud(FAR const struct btuart_lowerhalf_s *lower, uint32_t baud)
{
  FAR struct hciuart_config_s *config = (FAR struct hciuart_config_s *)lower;
  FAR struct hciuart_state_s *state = &config->state;
  int ret;

  struct termios tio;

#ifndef CONFIG_SERIAL_TERMIOS
#  error TERMIOS Support needed for hciuart_setbaud
#endif

  ret = file_ioctl(&state->f, TCGETS, (long unsigned int)&tio);
  if (ret)
    {
      wlerr("ERROR during TCGETS\n");
      return ret;
    }

  if (baud != 0)
    {
      cfsetspeed(&tio, baud);
    }

  /* To be a H4 interface, CTS/RTS are needed */

  tio.c_cflag |= CRTS_IFLOW | CCTS_OFLOW;

  ret = file_ioctl(&state->f, TCSETS, (long unsigned int)&tio);

  if (ret)
    {
      wlerr("ERROR during TCSETS, does UART support CTS/RTS?\n");
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: hciuart_read
 *
 * Description:
 *   Read UART data.
 *
 *   hciuart_read() after receipt of a callback notifying the upper half of
 *   the availability of Rx frame, the upper half may call the receive()
 *   method in order to obtain the buffered Rx frame data.
 *
 ****************************************************************************/

static ssize_t
hciuart_read(FAR const struct btuart_lowerhalf_s *lower,
             FAR void *buffer, size_t buflen)
{
  FAR struct hciuart_config_s *config = (FAR struct hciuart_config_s *)lower;
  FAR struct hciuart_state_s *state = &config->state;
  size_t ntotal;

  wlinfo("config %p buffer %p buflen %lu\n",
         config, buffer, (size_t) buflen);

  /* NOTE: This assumes that the caller has exclusive access to the Rx
   * buffer, i.e., one lower half instance can server only one upper half!
   */

  ntotal = file_read(&state->f, buffer, buflen);
  return ntotal;
}

/****************************************************************************
 * Name: hciuart_write
 *
 * Description:
 *   Write UART data.
 *
 *   hciuart_write() will add the outgoing frame to the Tx buffer and will
 *   return immediately.  This function may block only in the event that
 *   there is insufficient buffer space to hold the Tx frame data.  In that
 *   case the lower half will block until there is sufficient to buffer
 *   the entire outgoing packet.
 *
 ****************************************************************************/

static ssize_t
hciuart_write(FAR const struct btuart_lowerhalf_s *lower,
              FAR const void *buffer, size_t buflen)
{
  FAR const struct hciuart_config_s *config
    = (FAR const struct hciuart_config_s *)lower;
  FAR const struct hciuart_state_s *state = &config->state;

  wlinfo("config %p buffer %p buflen %lu\n",
         config, buffer, (size_t) buflen);

  buflen = file_write((struct file *)&state->f, buffer, buflen);

  return buflen;
}

/****************************************************************************
 * Name: hciuart_rxdrain
 *
 * Description:
 *   Flush/drain all buffered RX data
 *
 ****************************************************************************/

static ssize_t hciuart_rxdrain(FAR const struct btuart_lowerhalf_s *lower)
{
  FAR struct hciuart_config_s *config = (FAR struct hciuart_config_s *)lower;
  FAR struct hciuart_state_s *s = &config->state;

  file_ioctl(&s->f, TCDRN, 0);
  return 0;
}

/****************************************************************************
 * Name: hcicollecttask
 *
 * Description:
 *   Loop and alert when serial data arrive
 *
 ****************************************************************************/

static int hcicollecttask(int argc, FAR char **argv)
{
  FAR struct hciuart_state_s *s = &g_n->state;

  file_poll(&s->f, (struct pollfd *)&s->p, true);

  for (; ; )
    {
      /* Wait for data to arrive */

      int ret = nxsem_wait(s->p.sem);
      if (ret < 0)
        {
          wlwarn("Poll interrupted %d\n", ret);
          continue;
        }

      /* These flags can change dynamically as new events occur, so
       * snapshot.
       */

      irqstate_t flags = enter_critical_section();
      uint32_t tevents = s->p.revents;
      s->p.revents = 0;
      leave_critical_section(flags);

      wlinfo("Poll completed %d\n", tevents);

      /* Given the nature of file_poll, there are multiple reasons why
       * we might be here, so make sure we only consider the read.
       */

      if (tevents & POLLIN)
        {
          if (!s->enabled)
            {
              /* We aren't expected to be listening, so drop these data */

              wlwarn("Dropping data\n");
              hciuart_rxdrain(&g_n->lower);
            }
          else
            {
              if (s->callback != NULL)
                {
                  wlinfo("Activating callback\n");
                  s->callback(&g_n->lower, s->arg);
                }
              else
                {
                  wlwarn("Dropping data (no CB)\n");
                  hciuart_rxdrain(&g_n->lower);
                }
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_uart_shim_getdevice
 *
 * Description:
 *   Get a pointer to the device that will be used to communicate with the
 *   regular serial port on the HCI.
 *
 * Input Parameters:
 *   Entry in filesystem hierarchy for device
 *
 * Returned Value:
 *   Pointer to device interface
 *
 ****************************************************************************/

FAR void *bt_uart_shim_getdevice(FAR char *path)
{
  FAR struct hciuart_state_s *s;
  int ret;

  /* Get the memory for this shim instance */

  g_n = (FAR struct hciuart_config_s *)
    kmm_zalloc(sizeof(struct hciuart_config_s));

  if (!g_n)
    {
      return 0;
    }

  s = &g_n->state;

  ret = file_open(&s->f, path, O_RDWR | O_BINARY);
  if (ret < 0)
    {
      kmm_free(g_n);
      g_n = 0;
      return 0;
    }

  /* Hook the routines in */

  memcpy(&g_n->lower, &g_lowerstatic, sizeof(struct btuart_lowerhalf_s));

  /* Put materials into poll structure */

  nxsem_set_protocol(&s->dready, SEM_PRIO_NONE);

  s->p.fd = s->h;
  s->p.events = POLLIN;
  s->p.sem = &s->dready;

  s->enabled = true;

  s->serialmontask = kthread_create("BT HCI Rx",
                                    CONFIG_BLUETOOTH_TXCONN_PRIORITY,
                                    1024, hcicollecttask, NULL);

  return g_n;
}
