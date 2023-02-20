/****************************************************************************
 * net/devif/devif_filesend.c
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

#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_MM_IOB

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devif_file_send
 *
 * Description:
 *   Called from socket logic in response to a xmit or poll request from the
 *   the network interface driver.
 *
 *   This is identical to calling devif_file_send() except that the data is
 *   in a available file handle.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

int devif_file_send(FAR struct net_driver_s *dev, FAR struct file *file,
                    unsigned int len, unsigned int offset,
                    unsigned int target_offset)
{
  FAR struct iob_s *iob;
  unsigned int copyin;
  unsigned int remain;
  int ret;

  if (dev == NULL)
    {
      ret = -ENODEV;
      goto errout;
    }

  if (len == 0)
    {
      ret = -EINVAL;
      goto errout;
    }

#ifndef CONFIG_NET_IPFRAG
  if (len > NETDEV_PKTSIZE(dev) - NET_LL_HDRLEN(dev) - target_offset)
    {
      ret = -EMSGSIZE;
      goto errout;
    }
#endif

  /* Append the send buffer after device buffer */

  if (len > iob_navail(false) * CONFIG_IOB_BUFSIZE ||
      netdev_iob_prepare(dev, false, 0) != OK)
    {
      ret = -ENOMEM;
      goto errout;
    }

  iob_update_pktlen(dev->d_iob, target_offset);

  ret = file_seek(file, offset, SEEK_SET);
  if (ret < 0)
    {
      goto errout;
    }

  iob = dev->d_iob;
  remain = len;

  while (remain > 0)
    {
      if (iob->io_len + iob->io_offset == CONFIG_IOB_BUFSIZE)
        {
          if (iob->io_flink == NULL)
            {
              iob->io_flink = iob_tryalloc(false);
              if (iob->io_flink == NULL)
                {
                  ret = -ENOMEM;
                  goto errout;
                }
            }

          iob = iob->io_flink;
        }

      copyin = CONFIG_IOB_BUFSIZE -
               (iob->io_len + iob->io_offset);
      if (copyin > remain)
        {
          copyin = remain;
        }

      if (copyin > 0)
        {
          ret = file_read(file, iob->io_data +
                          (iob->io_len + iob->io_offset),
                          copyin);
          if (ret < 0)
            {
              goto errout;
            }

          iob->io_len += ret;
          remain      -= ret;
        }
    }

  iob_update_pktlen(dev->d_iob, target_offset + len);

  dev->d_sndlen = len;
  return len;

errout:
  if (ret < 0)
    {
      if (dev != NULL)
        {
          netdev_iob_release(dev);
        }

      nerr("ERROR: devif_iob_send error: %d\n", ret);
    }

  return ret;
}

#endif /* CONFIG_MM_IOB */
