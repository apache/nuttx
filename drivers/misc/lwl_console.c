/****************************************************************************
 * drivers/misc/lwl_console.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/drivers/drivers.h>

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
#define LWL_ID_SIG 0x7216A318

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t lwlconsole_read(FAR struct file *filep, FAR char *buffer,
                               size_t buflen);
static ssize_t lwlconsole_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct lwl_entry_s
{
  uint32_t sig;               /* Location signature */
  volatile uint32_t downword; /* Host to Target word */
  uint32_t upword;            /* Target to Host word */
};

static struct lwl_entry_s g_d =
{
  LWL_ID_SIG
};

static const struct file_operations g_consoleops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  lwlconsole_read,      /* read */
  lwlconsole_write,     /* write */
  NULL,                 /* seek */
  NULL,                 /* ioctl */
  NULL                  /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                /* unlink */
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

static bool read8bits(uint8_t port, FAR uint8_t *store)
{
  if (!linkactive())
    {
      return false;
    }

  /* Spin waiting for a byte to be received */

  while (LWL_DNSENSE(g_d.downword) == LWL_DNSENSE(g_d.upword))
    {
    }

  *store = g_d.downword & 255;

  /* Flip the bit to indicate the datum is read */

  g_d.upword = (g_d.upword & ~LWL_DNSENSEBIT) | LWL_DNSENSE(g_d.downword);

  return true;
}

/****************************************************************************
 * Name: lwlconsole_read
 ****************************************************************************/

static ssize_t lwlconsole_read(FAR struct file *filep, FAR char *buffer,
                               size_t buflen)
{
  if (buflen == 0)
    {
      return 0;
    }

  if (!read8bits(LWL_PORT_CONSOLE, (FAR uint8_t *) buffer))
    {
      return 0;
    }

  return 1;
}

/****************************************************************************
 * Name: lwlconsole_write
 ****************************************************************************/

static ssize_t lwlconsole_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen)
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
  register_driver("/dev/console", &g_consoleops, 0666, NULL);
}
