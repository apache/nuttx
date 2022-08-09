/****************************************************************************
 * arch/arm/src/rtl8720c/ameba_hci.c
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
#include <fcntl.h>
#include <nuttx/serial/serial.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netdev/netdev.h>
#include "amebaz_hci_board.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AMEBAZ_WIRELESS_NAME            "wlan0"
#define H4_CMD                          0x01
#define H4_EVT                          0x04
#define HCI_H4_HDR_SIZE                 1
#define HCI_CMD_HDR_SIZE                3
typedef struct
{
  struct file_operations  i_ops;
  struct file             filep;
} hci_dev_t;
struct bt_hci_evt_hdr
{
  uint8_t evt;
  uint8_t len;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static int hci_recv(struct file *filep, uint8_t *buf, size_t count)
{
  ssize_t ret;
  ssize_t nread = 0;
  while (count != nread)
    {
      ret = file_read(filep, buf + nread, count - nread);
      if (ret < 0)
        {
          if (ret == -EAGAIN)
            {
              continue;
            }
          else
            {
              return ret;
            }
        }

      nread += ret;
    }

  return nread;
}

static int hci_send(struct file *filep, uint8_t *buf, size_t count)
{
  ssize_t ret;
  ssize_t nwritten = 0;
  while (nwritten != count)
    {
      ret = file_write(filep, buf + nwritten, count - nwritten);
      if (ret < 0)
        {
          if (ret == -EAGAIN)
            {
              continue;
            }
          else
            {
              return ret;
            }
        }

      nwritten += ret;
    }

  return nwritten;
}

#ifdef HCI_START_IQK
static int hci_do_iqk(struct file *filep)
{
  /* OpCode: 0xfd4a, h4 data_len: Cmd(8), Event(7) */

  unsigned char command[8];
  struct t_data
  {
    uint8_t offset;
    uint16_t value;
  };

  struct t_data data[4] =
  {
    {0x00, 0x4000},
    {0x01, 0x0f88},
    {0x02, 0x3400},
    {0x3f, 0x0700}
  };

  if (0 == hci_check_iqk())
    {
      return 0;
    }

  for (unsigned char i = 0; i < 4; i++)
    {
      command[0] = H4_CMD;
      command[1] = 0x4a;
      command[2] = 0xfd;
      command[3] = 4;
      command[4] = data[i].offset;
      command[5] = (uint8_t)(data[i].value >> 0);
      command[6] = (uint8_t)(data[i].value >> 8);
      command[7] = 0;
      if (hci_send(filep, command, 8) != 8)
        {
          return -EIO;
        }

      hci_recv(filep, command, 1);
      if (command[0] != H4_EVT)
        {
          return -EIO;
        }

      hci_recv(filep, command + 1, sizeof(struct bt_hci_evt_hdr));
      hci_recv(filep, command + 3, command[2]);

      /* Check OpCode and Status */

      if (!(command[4] == 0x4a && command[5] == 0xfd) || command[6] != 0x00)
        {
          return -EIO;
        }
    }

  if (hci_start_iqk())
    {
      return -EIO;
    }

  return 0;
}

#endif
static int hci_check_local_ver(struct file *filep)
{
  /* OpCode: 0x1001, h4 buf_len: Cmd(1+3=4), Event(1+14=15) */

  unsigned char command[15];
  command[0] = H4_CMD;
  command[1] = 0x01;
  command[2] = 0x10;
  command[3] = 0;
  if (hci_send(filep, command, 4) != 4)
    {
      return -EIO;
    }

  hci_recv(filep, command, 1);
  if (command[0] != H4_EVT)
    {
      return -EIO;
    }

  hci_recv(filep, command + 1, sizeof(struct bt_hci_evt_hdr));
  hci_recv(filep, command + 3, command[2]);

  /* Check OpCode and Status */

  if (!(command[4] == 0x01 && command[5] == 0x10) || command[6] != 0x00)
    {
      return -EIO;
    }

  /* Only Check LMP Subversion */

  uint16_t lmp_sbuver =
    ((uint16_t)command[13]) | (((uint16_t)command[14]) << 8);
  if (BT_DEFAUT_LMP_SUBVER != lmp_sbuver)
    {
      return -EALREADY;
    }

  return 0;
}

static int hci_check_local_rom_ver(struct file *filep)
{
  /* OpCode: 0xfc6d, h4 buf_len: Cmd(1+3=4), Event(1+7=8) */

  unsigned char command[8];
  command[0] = H4_CMD;
  command[1] = 0x6d;
  command[2] = 0xfc;
  command[3] = 0;
  if (hci_send(filep, command, 4) != 4)
    {
      return -EIO;
    }

  hci_recv(filep, command, 1);
  if (command[0] != H4_EVT)
    {
      return -EIO;
    }

  hci_recv(filep, command + 1, sizeof(struct bt_hci_evt_hdr));
  hci_recv(filep, command + 3, command[2]);

  /* Check OpCode and Status */

  if (!(command[4] == 0x6d && command[5] == 0xfc) || command[6] != 0x00)
    {
      return -EIO;
    }

  /* Get Chip Id (Rom_Ver+1) and Find Patch */

  if (hci_find_fw_patch(command[7] + 1))
    {
      return -EIO;
    }

  return 0;
}

static int hci_update_baudrate(struct file *filep)
{
  /* OpCode: 0xfc17, h4 buf_len: Cmd(1+7=8), Event(1+6=7) */

  unsigned char command[8];
  struct termios toptions;
  uint32_t bt_baudrate;
  uint32_t uart_baudrate;
  command[0] = H4_CMD;
  command[1] = 0x17;
  command[2] = 0xfc;
  command[3] = sizeof(uint32_t);
  if (hci_get_baudrate(&bt_baudrate, &uart_baudrate))
    {
      return -EIO;
    }

  memcpy(&command[4], &bt_baudrate, sizeof(uint32_t));
  if (hci_send(filep, command, 8) != 8)
    {
      return -EIO;
    }

  hci_recv(filep, command, 1);
  if (command[0] != H4_EVT)
    {
      return -EIO;
    }

  hci_recv(filep, command + 1, sizeof(struct bt_hci_evt_hdr));
  hci_recv(filep, command + 3, command[2]);

  /* Check OpCode and Status */

  if (!(command[4] == 0x17 && command[5] == 0xfc) || command[6] != 0x00)
    {
      return -EIO;
    }

  file_ioctl(filep, TCGETS, (unsigned long)&toptions);
  cfsetispeed(&toptions, uart_baudrate);
  cfsetospeed(&toptions, uart_baudrate);
  return file_ioctl(filep, TCSETS, (unsigned long)&toptions);
}

static int hci_load_firmware(struct file *filep)
{
  int header_size = HCI_H4_HDR_SIZE + HCI_CMD_HDR_SIZE;
  uint8_t command[AMEBAZ_COMMAND_FRAGMENT_SIZE + header_size];
  struct net_driver_s *drv;
  int buffer_size;
  uint8_t *addr;
  int i;
  int ret;
  drv = netdev_findbyname(AMEBAZ_WIRELESS_NAME);
  if (drv == NULL)
    {
      return -EINVAL;
    }

  addr = drv->d_mac.ether.ether_addr_octet;
  if (hci_set_init_config_mac(addr, 1))
    {
      return -EIO;
    }

  while (hci_fetch_command(command) != AMEBAZ_COMMAND_DONE)
    {
      command[0] = H4_CMD;
      command[1] = 0x20;
      command[2] = 0xfc;
      buffer_size = header_size + command[3];
      usleep(10);
      ret = hci_send(filep, command, buffer_size);
      if (ret != buffer_size)
        {
          return ret;
        }

      hci_recv(filep, command, 1);
      if (H4_EVT != command[0])
        {
          return -EIO;
        }

      hci_recv(filep, command + 1, 2);
      hci_recv(filep, command + 3, command[2]);

      /* Check OpCode and Status */

      if (!(command[4] == 0x20 && command[5] == 0xfc) || command[6] != 0x00)
        {
          return -EIO;
        }
    }

  return OK;
}

static int hci_update_efuse_iqk(struct file *filep)
{
  /* OpCode: 0xfd91, h4 buf_len: Cmd(1+15=16), Event(1+6=7) */

  unsigned char command[16];
  command[0] = H4_CMD;
  command[1] = 0x91;
  command[2] = 0xfd;
  if (hci_get_efuse_iqk_data(command + 3))
    {
      return -EIO;
    }

  if (hci_send(filep, command, sizeof(command)) != sizeof(command))
    {
      return -EIO;
    }

  hci_recv(filep, command, 1);
  if (command[0] != H4_EVT)
    {
      return -EIO;
    }

  hci_recv(filep, command + 1, sizeof(struct bt_hci_evt_hdr));
  hci_recv(filep, command + 3, command[2]);
  return 0;
}

static int hci_open(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  hci_dev_t *dev = inode->i_private;
  int ret;
  ret = file_open(&dev->filep,
                  CONFIG_AMEBA_HCI_DEV_NAME, O_RDWR);
  if (ret < 0)
    {
      return ret;
    }

  if (hci_board_init())
    {
      goto bail;
    }

#ifdef HCI_START_IQK
  ret = hci_do_iqk(&dev->filep);
  if (ret < 0)
    {
      goto bail;
    }

#endif
  ret = hci_check_local_ver(&dev->filep);
  if (ret < 0)
    {
      if (-EALREADY == ret)
        {
          ret = 0;
        }

      goto bail;
    }

  ret = hci_check_local_rom_ver(&dev->filep);
  if (ret < 0)
    {
      goto bail;
    }

  ret = hci_update_baudrate(&dev->filep);
  if (ret < 0)
    {
      goto bail;
    }

  ret = hci_load_firmware(&dev->filep);
  if (ret < 0)
    {
      goto bail;
    }

  ret = hci_update_efuse_iqk(&dev->filep);
bail:
  ret = hci_board_init_done();
  if (ret < 0)
    {
      file_close(&dev->filep);
    }

  return ret;
}

static int hci_close(struct file *filep)
{
  struct inode *inode = filep->f_inode;
  hci_dev_t *dev = inode->i_private;

  /* FIXME: BT PowerOff */

  return file_close(&dev->filep);
}

static ssize_t hci_read(struct file *filep,
                        char *buffer, size_t buflen)
{
  struct inode *inode = filep->f_inode;
  hci_dev_t *dev = inode->i_private;
  return file_read(&dev->filep, buffer, buflen);
}

static ssize_t hci_write(struct file *filep, const char *buffer,
                         size_t buflen)
{
  struct inode *inode = filep->f_inode;
  hci_dev_t *dev = inode->i_private;
  return file_write(&dev->filep, buffer, buflen);
}

static int hci_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode *inode = filep->f_inode;
  hci_dev_t *dev = inode->i_private;
  return file_ioctl(&dev->filep, cmd, arg);
}

static int hci_poll(struct file *filep,
                    struct pollfd *fds, bool setup)
{
  struct inode *inode = filep->f_inode;
  hci_dev_t *dev = inode->i_private;
  return file_poll(&dev->filep, fds, setup);
}

static hci_dev_t g_hcidev =
{
  .i_ops =
    {
      .open   = hci_open,
      .close  = hci_close,
      .read   = hci_read,
      .write  = hci_write,
      .ioctl  = hci_ioctl,
      .poll   = hci_poll
    },
};

int amebaz_bt_hci_uart_register(const char *path)
{
  return register_driver(path, &g_hcidev.i_ops, 0666, &g_hcidev);
}

