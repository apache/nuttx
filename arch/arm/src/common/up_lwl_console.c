/****************************************************************************
 * drivers/serial/lwlconsole.c
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

#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <syscall.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Lightweight Link (lwl)
 * ======================
 *
 * Lightweight bidirectional communication between target and debug host
 * without any need for additional hardware.
 *
 * Works with openOCD and other debuggers that are capable of reading and
 * writing memory while the target is running.
 *
 * Principle of operation is simple; An 'upword' of 32 bits communicates
 * from the target to the host, a 'downword' of the same size runs in the
 * opposite direction. These two words can be in any memory that is
 * read/write access for both the target and the debug host. A simple ping
 * pong handshake protocol over these words allows up/down link
 * communication.  On the upside no additional integration is needed. On
 * the downside it may be necessary to feed lwl with cycles to poll for
 * changes in the downword, depending on the use case.
 *
 * Bit configuration
 * -----------------
 *
 * Downword (Host to target);
 *
 * A D U VV XXX
 *
 * A   31    1 - Service Active (Set by host)
 * D   30    1 - Downsense (Toggled when there is data)
 * U   29    1 - Upsense ack (Toggled to acknowledge receipt of uplink data)
 * VV  28-27 2 - Valid Octets (Number of octets valid in the message)
 * XXX 26-24 3 - Port in use (Type of the message)
 * O2  23-16 8 - Octet 2
 * O1  15-08 8 - Octet 1
 * O0  07-00 8 - Octet 0
 *
 * Upword (Target to Host);
 *
 * A   31    1 - Service Active (Set by device)
 * D   30    1 - Downsense ack (Toggled to acknowledge receipt of downlink
 *               data)
 * U   29    1 - Upsense (Toggled when there is data)
 * VV  28-27 2 - Valid upword octets
 * XXX 26-24 3 - Port in use (Type of the message)
 * O2  23-16 8 - Octet 2
 * O1  15-08 8 - Octet 1
 * O0  07-00 8 - Octet 0
 *
 */

/* Protocol bits */

#define LWL_GETACTIVE(x) (((x) & (1 << 31)) != 0)
#define LWL_ACTIVE(x) (((x)&1) << 31)

#define LWL_DNSENSEBIT (1 << 30)
#define LWL_DNSENSE(x) ((x)&LWL_DNSENSEBIT)
#define LWL_UPSENSEBIT (1 << 29)
#define LWL_UPSENSE(x) ((x)&LWL_UPSENSEBIT)
#define LWL_SENSEMASK (3 << 29)

#define LWL_GETOCTVAL(x) (((x) >> 27) & 3)
#define LWL_OCTVAL(x) (((x)&3) << 27)
#define LWL_GETPORT(x) (((x) >> 24) & 7)
#define LWL_PORT(x) (((x)&7) << 24)

#define LWL_PORT_CONSOLE 1
#define ID_SIG 0x7216A318

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t lwlconsole_read(struct file *filep, char *buffer,
                               size_t buflen);
static ssize_t lwlconsole_write(struct file *filep, const char *buffer,
                                size_t buflen);
static int lwlconsole_ioctl(struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct
{
  uint32_t sig;               /* Location signature */
  volatile uint32_t downword; /* Host to Target word */
  uint32_t upword;            /* Target to Host word */
} g_d =
{
  .sig = ID_SIG
};

static const struct file_operations g_consoleops =
{
  NULL,                       /* open */
  NULL,                       /* close */
  lwlconsole_read,            /* read */
  lwlconsole_write,           /* write */
  NULL,                       /* seek */
  lwlconsole_ioctl,           /* ioctl */
  NULL                        /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
    ,
  NULL                        /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool linkactive(void)
{
  return (LWL_GETACTIVE(g_d.downword) != 0);
}

static bool writeword(uint32_t newupword)
{
  /* Check link is active */

  if (!linkactive())
    {
      return false;
    }

  /* Spin waiting for previous data to be collected */

  while (LWL_UPSENSE(g_d.downword) != LWL_UPSENSE(g_d.upword))
    {
    }

  /* Load new data, toggling UPSENSE bit to show it is new */

  g_d.upword = LWL_DNSENSE(g_d.upword) | newupword |
               (LWL_UPSENSE(g_d.upword) ? 0 : LWL_UPSENSEBIT);

  return true;
}

static bool write8bits(uint8_t port, uint8_t val)
{
  /* Prepare new word */

  uint32_t newupword = LWL_ACTIVE(true) | LWL_OCTVAL(1) |
                       LWL_PORT(port) | (val & 0xff);

  return writeword(newupword);
}

static bool write16bits(uint8_t port, uint32_t val)
{
  /* Prepare new word */

  uint32_t newupword = LWL_ACTIVE(true) | LWL_OCTVAL(2) |
                       LWL_PORT(port) | (val & 0xffff);

  return writeword(newupword);
}

static bool write24bits(uint8_t port, uint32_t val)
{
  /* Prepare new word */

  uint32_t newupword = LWL_ACTIVE(true) | LWL_OCTVAL(3) |
                       LWL_PORT(port) | (val & 0xffffff);

  return writeword(newupword);
}

static bool read8bits(uint8_t port, uint8_t * store)
{
  if (LWL_DNSENSE(g_d.downword) == LWL_DNSENSE(g_d.upword))
    {
      return false;
    }

  *store = g_d.downword & 255;

  /* Flip the bit to indicate the datum is read */

  g_d.upword = (g_d.upword & ~LWL_DNSENSEBIT) | LWL_DNSENSE(g_d.downword);

  return true;
}

/****************************************************************************
 * Name: lwlconsole_ioctl
 ****************************************************************************/

static int lwlconsole_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: lwlconsole_read
 ****************************************************************************/

static ssize_t lwlconsole_read(struct file *filep, char *buffer,
                               size_t buflen)
{
  if (buflen == 0 || !linkactive())
    {
      return 0;
    }

  while (!read8bits(LWL_PORT_CONSOLE, (uint8_t *) buffer))
    {
    }

  return 1;
}

/****************************************************************************
 * Name: lwlconsole_write
 ****************************************************************************/

static ssize_t lwlconsole_write(struct file *filep, const char *buffer,
                                size_t buflen)
{
  uint32_t oc = 0;

  while (buflen)
    {
      switch (buflen)
        {
          case 0:
            return oc;

          case 1:
            if (write8bits(LWL_PORT_CONSOLE, buffer[0]))
              {
                oc++;
                buffer++;
                buflen--;
              }
            break;

          case 2:
            if (write16bits(LWL_PORT_CONSOLE, buffer[0] | (buffer[1] << 8)))
              {
                oc += 2;
                buffer += 2;
                buflen -= 2;
              }
            break;

          default:
            if (write24bits(LWL_PORT_CONSOLE, buffer[0] |
                            (buffer[1] << 8) | (buffer[2] << 16)))
              {
                oc += 3;
                buffer += 3;
                buflen -= 3;
              }
            break;
        }
    }

  return oc;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lwlconsole_init
 ****************************************************************************/

void lwlconsole_init(void)
{
  g_d.upword = 0;
  register_driver("/dev/console", &g_consoleops, 0666, NULL);
}
