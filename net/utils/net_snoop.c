/****************************************************************************
 * net/utils/net_snoop.c
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

#include <assert.h>
#include <endian.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>

#include <sys/param.h>

#include <nuttx/net/snoop.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SNOOP_VERSION_1 1
#define SNOOP_VERSION_2 2

/* microseconds since midnight, January 1st, 0 AD nominal Gregorian. */

#define SNOOP_EPOCH_USEC(tv) (((tv).tv_sec - 0x386d4380ll) * 1000000ll \
                              + (tv).tv_usec + 0x00e03ab44a676000ll)

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* The availability of tools to capture, display and interpret packets
 * traversing a network has proven extremely useful in debugging
 * networking problems.  The ability to capture packets and store them
 * for later analysis allows one to de-couple the tasks of collecting
 * information about a network problem and analysing that information.
 *
 * More info about snoop datalink type, please refer to
 * https://www.rfc-editor.org/rfc/rfc1761.txt and
 * https://fte.com/webhelpii/hsu/Content/Technical_Information/
 * BT_Snoop_File_Format.htm
 */

/* The snoop packet capture file is an array of octets structured as
 * follows:
 *
 *    +------------------------+
 *    |                        |
 *    |      File Header       |
 *    |                        |
 *    +------------------------+
 *    |                        |
 *    |     Packet Record      |
 *    ~        Number 1        ~
 *    |                        |
 *    +------------------------+
 *    .                        .
 *    .                        .
 *    .                        .
 *    +------------------------+
 *    |                        |
 *    |     Packet Record      |
 *    ~        Number N        ~
 *    |                        |
 *    +------------------------+
 */

/* snoop_file_header_s
 *
 * The structure of the File Header is as follows:
 *
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *    |                                                               |
 *    +                     Identification Pattern                    +
 *    |                                                               |
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *    |                       Version Number                          |
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *    |                         Datalink Type                         |
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */

begin_packed_struct struct snoop_file_header_s
{
  uint8_t magic[8];  /* Identification Pattern */
  uint32_t version;  /* Version Number */
  uint32_t datalink; /* Datalink Type */
} end_packed_struct;

/* snoop_packet_header_s
 *
 * The structure of the packet record is as follows:
 *
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *    |                        Original Length                        |
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *    |                        Included Length                        |
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *    |                      Packet Record Length                     |
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *    |                        Cumulative Drops                       |
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *    |                       Timestamp Seconds                       |
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *    |                     Timestamp Microseconds                    |
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *    |                                                               |
 *    .                                                               .
 *    .                          Packet Data                          .
 *    .                                                               .
 *    +                                               +- - - - - - - -+
 *    |                                               |     Pad       |
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */

begin_packed_struct struct snoop_packet_header_s
{
  uint32_t orig_len;    /* actual length of packet */
  uint32_t incl_len;    /* number of octets captured in file */
  union
  {
    uint32_t flags;     /* Packet Flags: 1 hci cmd , eg: btsnoop */
    uint32_t rec_len;   /* length of record */
  };
  uint32_t cum_drops;   /* cumulative number of dropped packets */
  union
  {
    uint64_t ts_usec;   /* timestamp microseconds, eg: btsnoop */
    struct
    {
      uint32_t ts_sec;  /* timestamp seconds */
      uint32_t ts_usec; /* timestamp microseconds */
    } ts;
  };
} end_packed_struct;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: snoop_fill_packet_header
 *
 * Description:
 *   This function fill snoop packet header info.
 *
 ****************************************************************************/

static void snoop_fill_packet_header(FAR struct snoop_s *snoop,
                                     uint32_t bytes, uint32_t drops,
                                     uint32_t flags, FAR struct
                                     snoop_packet_header_s *header)
{
  struct timeval tv;

  switch (snoop->datalink)
    {
      case SNOOP_DATALINK_HCI_UNENCAP:
      case SNOOP_DATALINK_HCI_UART:
      case SNOOP_DATALINK_HCI_BSCP:
      case SNOOP_DATALINK_HCI_SERIAL:
        gettimeofday(&tv, NULL);
        header->ts_usec = htobe64(SNOOP_EPOCH_USEC(tv));
        header->flags = htobe32(flags);
        break;

      case SNOOP_DATALINK_TYPE_TOKENBUS:
      case SNOOP_DATALINK_TYPE_TOKERING:
      case SNOOP_DATALINK_TYPE_METRONET:
      case SNOOP_DATALINK_TYPE_ETHERNET:
      case SNOOP_DATALINK_TYPE_HDLC:
      case SNOOP_DATALINK_TYPE_CHARSYNC:
      case SNOOP_DATALINK_TYPE_IBMC2C:
      case SNOOP_DATALINK_TYPE_FDDI:
      case SNOOP_DATALINK_TYPE_OTHER:
        gettimeofday(&tv, NULL);
        header->ts.ts_sec = htobe32(tv.tv_sec);
        header->ts.ts_usec = htobe32(tv.tv_usec);
        header->rec_len = htobe32(flags);
        break;

      default:
        DEBUGASSERT(false);
    }

  header->orig_len = htobe32(bytes);
  header->incl_len = htobe32(bytes);
  header->cum_drops = htobe32(drops);
}

/****************************************************************************
 * Name: snoop_flush
 *
 * Description:
 *   This function could flush snoop buf into file.
 *
 ****************************************************************************/

static int snoop_flush(FAR struct snoop_s *snoop)
{
  ssize_t ret;

  if (snoop->next == 0)
    {
      return 0;
    }

  do
    {
      ret = file_write(&snoop->filep, snoop->buf, snoop->next);
      if (ret < 0)
        {
          break;
        }

      snoop->next -= ret;
      memmove(snoop->buf, snoop->buf + ret, snoop->next);
    }
  while (snoop->next > 0);

#ifndef CONFIG_DISABLE_MOUNTPOINT
  if (snoop->autosync)
    {
      ret = file_fsync(&snoop->filep);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: snoop_flush_lock
 *
 * Description:
 *   Snoop flush atomic
 *
 ****************************************************************************/

static int snoop_flush_lock(FAR struct snoop_s *snoop)
{
  irqstate_t flags;
  int ret;

  flags = enter_critical_section();
  nxmutex_lock(&snoop->mutex);
  ret = snoop_flush(snoop);
  nxmutex_unlock(&snoop->mutex);
  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: snoop_flush_work
 *
 * Description:
 *   Do snoop flush work.
 *
 ****************************************************************************/

static void snoop_flush_work(FAR void *arg)
{
  snoop_flush_lock((FAR struct snoop_s *)arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: snoop_open
 *
 * Description:
 *   This function open snoop file by datalink.
 *
 * Input Parameters:
 *   snoop     The snoop driver struct
 *   filename  Snoop file name
 *   datalink  Snoop datalink type, such as SNOOP_DATALINK_TYPE_XX
 *   autosync  whether do file_sync when snoop_dump
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int snoop_open(FAR struct snoop_s *snoop, FAR const char *filename,
               uint32_t datalink, bool autosync)
{
  struct snoop_file_header_s header;
  int ret;

  if (!snoop)
    {
      return -EINVAL;
    }

  switch (datalink)
    {
      case SNOOP_DATALINK_TYPE_TOKENBUS:
      case SNOOP_DATALINK_TYPE_TOKERING:
      case SNOOP_DATALINK_TYPE_METRONET:
      case SNOOP_DATALINK_TYPE_ETHERNET:
      case SNOOP_DATALINK_TYPE_HDLC:
      case SNOOP_DATALINK_TYPE_CHARSYNC:
      case SNOOP_DATALINK_TYPE_IBMC2C:
      case SNOOP_DATALINK_TYPE_FDDI:
      case SNOOP_DATALINK_TYPE_OTHER:
        {
          static const uint8_t snoop_magic[] =
            {
              's', 'n', 'o', 'o', 'p', '\0', '\0', '\0'
            };

          memcpy(header.magic, snoop_magic, nitems(snoop_magic));
          header.version = htobe32(SNOOP_VERSION_2);
          break;
        };

      case SNOOP_DATALINK_HCI_UNENCAP:
      case SNOOP_DATALINK_HCI_UART:
      case SNOOP_DATALINK_HCI_BSCP:
      case SNOOP_DATALINK_HCI_SERIAL:
        {
          static const uint8_t btsnoop_magic[] =
            {
              'b', 't', 's', 'n', 'o', 'o', 'p', '\0'
            };

          memcpy(header.magic, btsnoop_magic, nitems(btsnoop_magic));
          header.version = htobe32(SNOOP_VERSION_1);
          break;
        }

      default:
        {
          return -EINVAL;
        }
    }

  ret = file_open(&snoop->filep, filename, O_RDWR | O_CREAT);
  if (ret < 0)
    {
      return ret;
    }

  snoop->datalink = datalink;
  snoop->autosync = autosync;
  snoop->next     = 0;

  header.datalink = htobe32(datalink);
  ret = file_write(&snoop->filep, &header, sizeof(header));
  if (ret != sizeof(header))
    {
      ret = ret < 0 ? ret : -EINVAL;
      goto error;
    }

  nxmutex_init(&snoop->mutex);
  return OK;

error:
  snoop_close(snoop);
  return ret;
}

/****************************************************************************
 * Name: snoop_dump
 *
 * Description:
 *   This function dump nbytes buf data into snoop file.
 *
 * Input Parameters:
 *   snoop     The snoop driver struct
 *   buf       Snoop buffer
 *   nbytes    Snoop buffer size
 *   drops     cumulative number of dropped packets
 *   flags     Packet Flags: 1 hci cmd , eg: btsnoop
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int snoop_dump(FAR struct snoop_s *snoop, FAR const void *buf,
               uint32_t nbytes, uint32_t drops, uint32_t flags)
{
  struct snoop_packet_header_s header;
  irqstate_t irqflags;
  int ret = 0;

  if (!snoop)
    {
      return -EINVAL;
    }

  snoop_fill_packet_header(snoop, nbytes, drops, flags, &header);

  irqflags = enter_critical_section();
  if (up_interrupt_context())
    {
      if (sizeof(snoop->buf) - snoop->next <
          nbytes + sizeof(struct snoop_packet_header_s))
        {
          ret = -ENOMEM;
          goto out_leave;
        }

      memcpy(snoop->buf + snoop->next, &header, sizeof(header));
      snoop->next += sizeof(header);
      memcpy(snoop->buf + snoop->next, buf, nbytes);
      snoop->next += nbytes;

      if (work_available(&snoop->work))
        {
          work_queue(HPWORK, &snoop->work, snoop_flush_work, snoop, 0);
        }

      goto out_leave;
    }
  else
    {
      nxmutex_lock(&snoop->mutex);
      ret = snoop_flush(snoop);
      if (ret < 0)
        {
          goto out_unlock;
        }

      ret = file_write(&snoop->filep, &header, sizeof(header));
      if (ret < 0)
        {
          goto out_unlock;
        }
      else if (ret != sizeof(header))
        {
          ret = -EINVAL;
          goto out_unlock;
        }

      ret = file_write(&snoop->filep, buf, nbytes);
      if (ret < 0)
        {
          goto out_unlock;
        }
      else if (ret != nbytes)
        {
          ret = -EINVAL;
          goto out_unlock;
        }
    }

out_unlock:
  nxmutex_unlock(&snoop->mutex);
out_leave:
  leave_critical_section(irqflags);
  return ret;
}

/****************************************************************************
 * Name: snoop_sync
 *
 * Description:
 *   This function sync snoop buffer.
 *
 * Input Parameters:
 *   snoop     The snoop driver struct
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int snoop_sync(FAR struct snoop_s *snoop)
{
  if (!snoop)
    {
      return -EINVAL;
    }

  return snoop_flush_lock(snoop);
}

/****************************************************************************
 * Name: snoop_close
 *
 * Description:
 *   This function close snoop file.
 *
 * Input Parameters:
 *   snoop     The snoop driver struct
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int snoop_close(FAR struct snoop_s *snoop)
{
  if (!snoop)
    {
      return -EINVAL;
    }

  nxmutex_destroy(&snoop->mutex);
  return file_close(&snoop->filep);
}
