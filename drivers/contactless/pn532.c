/****************************************************************************
 * drivers/contactless/pn532.c
 *
 *   Copyright(C) 2012, 2013, 2016 Offcode Ltd. All rights reserved.
 *   Authors: Janne Rosberg <janne@offcode.fi>
 *            Teemu Pirinen <teemu@offcode.fi>
 *            Juho Grundström <juho@offcode.fi>
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>

#include "pn532.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Bit order H/W feature must be enabled in order to support LSB first
 * operation.
 */

#if !defined(CONFIG_SPI_HWFEATURES) || !defined(CONFIG_SPI_BITORDER)
#  error CONFIG_SPI_HWFEATURES=y and CONFIG_SPI_BITORDER=y required by this driver
#endif

#ifndef CONFIG_ARCH_HAVE_SPI_BITORDER
#  warning This platform does not support SPI LSB-bit order
#endif

#ifdef CONFIG_CL_PN532_DEBUG_TX
#  define tracetx errdumpbuffer
#else
#    define tracetx(x...)
#endif

#ifdef CONFIG_CL_PN532_DEBUG_RX
#  define tracerx errdumpbuffer
#else
#    define tracerx(x...)
#endif

#define FRAME_SIZE(f) (sizeof(struct pn532_frame) + f->len + 2)
#define FRAME_POSTAMBLE(f) (f->data[f->len + 1])

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void pn532_configspi(FAR struct spi_dev_s *spi);
static void pn532_lock(FAR struct spi_dev_s *spi);
static void pn532_unlock(FAR struct spi_dev_s *spi);

/* Character driver methods */

static int     _open(FAR struct file *filep);
static int     _close(FAR struct file *filep);
static ssize_t _read(FAR struct file *filep,
                     FAR char *buffer, size_t buflen);
static ssize_t _write(FAR struct file *filep, FAR const char *buffer,
                      size_t buflen);
static int     _ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static uint8_t pn532_checksum(uint8_t value);
static uint8_t pn532_data_checksum(FAR uint8_t *data, int datalen);

int pn532_read(FAR struct pn532_dev_s *dev, FAR uint8_t *buff, uint8_t n);

#if 0 /* TODO */
/* IRQ Handling */

static int pn532_irqhandler(FAR int irq, FAR void *context, FAR void *dev);
static inline int pn532_attachirq(FAR struct pn532_dev_s *dev, xcpt_t isr);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_pn532fops =
{
  _open,
  _close,
  _read,
  _write,
  NULL,
  _ioctl,
  NULL
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL
#endif
};

static const uint8_t pn532ack[] =
{
  0x00, 0x00, 0xff, 0x00, 0xff, 0x00
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void pn532_lock(FAR struct spi_dev_s *spi)
{
  int ret;

  SPI_LOCK(spi, true);

  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);

  ret = SPI_HWFEATURES(spi, HWFEAT_LSBFIRST);
  if (ret < 0)
    {
      ctlserr("ERROR: SPI_HWFEATURES failed to set bit order: %d\n", ret);
    }

  SPI_SETFREQUENCY(spi, CONFIG_PN532_SPI_FREQ);
}

static void pn532_unlock(FAR struct spi_dev_s *spi)
{
  SPI_LOCK(spi, false);
}

static inline void pn532_configspi(FAR struct spi_dev_s *spi)
{
  int ret;

  /* Configure SPI for the PN532 module. */

  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);

  ret = SPI_HWFEATURES(spi, HWFEAT_LSBFIRST);
  if (ret < 0)
    {
      ctlserr("ERROR: SPI_HWFEATURES failed to set bit order: %d\n", ret);
    }

  SPI_SETFREQUENCY(spi, CONFIG_PN532_SPI_FREQ);
}

static inline void pn532_select(FAR struct pn532_dev_s *dev)
{
  if (dev->config->select)
    {
      dev->config->select(dev, true);
    }
  else
    {
      SPI_SELECT(dev->spi, SPIDEV_CONTACTLESS(0), true);
    }
}

static inline void pn532_deselect(FAR struct pn532_dev_s *dev)
{
  if (dev->config->select)
    {
      dev->config->select(dev, false);
    }
  else
    {
      SPI_SELECT(dev->spi, SPIDEV_CONTACTLESS(0), false);
    }
}

static void pn532_frame_init(FAR struct pn532_frame *frame, uint8_t cmd)
{
  frame->preamble   = PN532_PREAMBLE;
  frame->start_code = PN532_SOF;
  frame->tfi        = PN532_HOSTTOPN532;
  frame->data[0]    = cmd;
  frame->len        = 2;
}

static void pn532_frame_finish(FAR struct pn532_frame *frame)
{
  frame->lcs = pn532_checksum(frame->len);
  frame->data[frame->len - 1] = pn532_data_checksum(&frame->tfi, frame->len);
  frame->data[frame->len]   = PN532_POSTAMBLE;
}

static inline uint8_t pn532_checksum(uint8_t value)
{
  return ~value + 1;
}

static uint8_t pn532_data_checksum(FAR uint8_t *data, int datalen)
{
  uint8_t sum = 0x00;
  int i;

  for (i = 0; i < datalen; i++)
    {
      sum += data[i];
    }

  return pn532_checksum(sum);
}

bool pn532_rx_frame_is_valid(FAR struct pn532_frame *f, bool check_data)
{
  uint8_t chk;

  if (f->start_code != PN532_SOF)
    {
      ctlserr("ERROR: Frame startcode 0x%X != 0x%X\n",
               PN532_SOF, f->start_code);
      return false;
    }

  if (f->tfi != PN532_PN532TOHOST)
    {
      return false;
    }

  chk = pn532_checksum(f->len);
  if (chk != f->lcs)
    {
      ctlserr("ERROR: Frame data len checksum failed");
      return false;
    }

  if (check_data)
    {
      chk = pn532_data_checksum(&f->tfi, f->len);
      if (chk != f->data[f->len - 1])
        {
          ctlserr("ERROR: Frame data checksum failed: calc=0x%X != 0x%X",
                   chk, f->data[f->len - 1]);
          return false;
        }
    }

  return true;
}

static uint8_t pn532_status(FAR struct pn532_dev_s *dev)
{
  int rs;

  pn532_lock(dev->spi);
  pn532_select(dev);

  rs = SPI_SEND(dev->spi, PN532_SPI_STATREAD);
  rs = SPI_SEND(dev->spi, PN532_SPI_STATREAD);

  pn532_deselect(dev);
  pn532_unlock(dev->spi);

  return rs;
}

/****************************************************************************
 * Name: pn532_wait_rx_ready
 *
 * Description:
 *   Blocks until Data frame available from chip.
 *
 * Input Parameters:
 *   dev
 *   timeout
 *
 * Returned Value:
 *   0 for OK. -ETIMEDOUT if no data available
 *
 ****************************************************************************/

static int pn532_wait_rx_ready(FAR struct pn532_dev_s *dev, int timeout)
{
  int ret = OK;

#ifdef CONFIG_PN532_USE_IRQ_FLOW_CONTROL
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += 1;
  nxsem_timedwait(dev->sem_rx, &ts);
#endif

  /* TODO: Handle Exception bits 2, 3 */

  while (pn532_status(dev) != PN532_SPI_READY)
    {
      if (--timeout == 0x00)
        {
          ctlserr("ERROR: wait RX timeout!\n");
          return -ETIMEDOUT;
        }

      nxsig_usleep(1000);
    }

  dev->state = PN532_STATE_DATA_READY;
  return ret;
}

/****************************************************************************
 * Name: pn532_writecommand
 *
 * Description:
 *   Helper for debug/testing
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

#if 0
static void pn532_writecommand(FAR struct pn532_dev_s *dev, uint8_t cmd)
{
  char cmd_buffer[16];
  FAR struct pn532_frame *f = (FAR struct pn532_frame *)cmd_buffer;

  pn532_frame_init(f, cmd);
  pn532_frame_finish(f);

  pn532_lock(dev->spi);
  pn532_select(dev);
  nxsig_usleep(10000);

  SPI_SEND(dev->spi, PN532_SPI_DATAWRITE);
  SPI_SNDBLOCK(dev->spi, f, FRAME_SIZE(f));

  pn532_deselect(dev);
  pn532_unlock(dev->spi);

  tracetx("command sent", (uint8_t *) f, FRAME_SIZE(f));
}
#endif

/****************************************************************************
 * Name: pn532_read
 *
 * Description:
 *   RAW Read data from chip.
 *   NOTE: This WON'T wait if data is available!
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

int pn532_read(FAR struct pn532_dev_s *dev, FAR uint8_t *buff, uint8_t n)
{
  pn532_lock(dev->spi);
  pn532_select(dev);
  SPI_SEND(dev->spi, PN532_SPI_DATAREAD);
  SPI_RECVBLOCK(dev->spi, buff, n);
  pn532_deselect(dev);
  pn532_unlock(dev->spi);

  tracerx("read", buff, n);
  return n;
}

int pn532_read_more(FAR struct pn532_dev_s *dev,
                    FAR uint8_t *buff, uint8_t n)
{
  pn532_lock(dev->spi);
  pn532_select(dev);
  SPI_RECVBLOCK(dev->spi, buff, n);
  pn532_deselect(dev);
  pn532_unlock(dev->spi);

  tracerx("read_more", buff, n);
  return n;
}

/****************************************************************************
 * Name: pn532_read_ack
 *
 * Description:
 *   Read Ack response from device
 *
 * Input Parameters:
 *   dev
 *
 * Returned Value:
 *   0 = NOK, 1 = OK
 *
 ****************************************************************************/

int pn532_read_ack(FAR struct pn532_dev_s *dev)
{
  int res = 0;
  uint8_t ack[6];

  pn532_read(dev, (uint8_t *) &ack, 6);

  if (memcmp(&ack, &pn532ack, 6) == 0x00)
    {
      res = 1;
    }
  else
    {
      ctlsinfo("ACK NOK");
      res = 0;
    }

  return res;
}

/****************************************************************************
 * Name: pn532_write_frame
 *
 * Description:
 *   Write frame to chip.  Also waits and reads ACK frame from chip.
 *
 *   Construct frame with
 *      pn532_frame_init(), pn532_frame_finish()
 *
 * Input Parameters:
 *   dev - Device instance
 *   f   - Pointer to start frame
 *
 * Returned Value:
 *   0 for OK, negative for error
 *
 ****************************************************************************/

int pn532_write_frame(FAR struct pn532_dev_s *dev, FAR struct pn532_frame *f)
{
  int res = OK;

  pn532_lock(dev->spi);
  pn532_select(dev);
  nxsig_usleep(2000);

  SPI_SEND(dev->spi, PN532_SPI_DATAWRITE);
  SPI_SNDBLOCK(dev->spi, f, FRAME_SIZE(f));
  pn532_deselect(dev);
  pn532_unlock(dev->spi);
  tracetx("WriteFrame", (uint8_t *) f, FRAME_SIZE(f));

  /* Wait ACK frame */

  res = pn532_wait_rx_ready(dev, 30);
  if (res == OK)
    {
      if (!pn532_read_ack(dev))
        {
          ctlserr("ERROR: command FAILED\n");
          res = -EIO;
        }
    }

  return res;
}

int pn532_read_frame(FAR struct pn532_dev_s *dev, FAR struct pn532_frame *f,
                     int max_size)
{
  int res = -EIO;

  /* Wait for frame available */

  if ((res = pn532_wait_rx_ready(dev, 100)) == OK)
    {
      /* Read header */

      pn532_read(dev, (uint8_t *) f, sizeof(struct pn532_frame));
      if (pn532_rx_frame_is_valid(f, false))
        {
          if (max_size < f->len)
            {
              return -EINVAL;
            }

          pn532_read_more(dev, &f->data[0], f->len);

          /* TODO: optimize frame integrity check...
           * pn532_data_checksum(&f.tfi, f->len);
           * errdumpbuffer("RX Frame:", f, f->len+6);
           */

          if (pn532_rx_frame_is_valid(f, true))
            {
              res = OK;
            }
        }
    }

  return res;
}

bool pn532_set_config(FAR struct pn532_dev_s *dev, uint8_t flags)
{
  char cmd_buffer[2 + 7];
  FAR struct pn532_frame *f = (FAR struct pn532_frame *)cmd_buffer;

  pn532_frame_init(f, PN532_COMMAND_SETPARAMETERS);
  f->data[1] = flags;
  f->len += 1;
  pn532_frame_finish(f);

  uint8_t resp[9];
  bool res = false;

  if (pn532_write_frame(dev, f) == OK)
    {
      pn532_read(dev, (uint8_t *) &resp, 9);
      tracerx("set config response", resp, 9);
      res = true;
    }

  return res;
}

int pn532_sam_config(FAR struct pn532_dev_s *dev,
                     FAR struct pn_sam_settings_s *settings)
{
  char cmd_buffer[4 + 7];
  FAR struct pn532_frame *f = (FAR struct pn532_frame *) cmd_buffer;
  int res;

  pn532_frame_init(f, PN532_COMMAND_SAMCONFIGURATION);
  f->data[1] = PN532_SAM_NORMAL_MODE;
  f->data[2] = 0x14;    /* Timeout LSB=50ms 0x14*50ms = 1sec */
  f->data[3] = 0x01;    /* P-70, IRQ enabled */

  if (settings)
    {
      /*  TODO: !!! */
    }

  f->len += 3;
  pn532_frame_finish(f);

  res = -EIO;

  if (pn532_write_frame(dev, f) == OK)
    {
      if (pn532_read_frame(dev, f, 4) == OK)
        {
          tracerx("sam config response", (uint8_t *) f->data, 3);
          if (f->data[0] == PN532_COMMAND_SAMCONFIGURATION + 1)
            {
              res = OK;
            }
        }
    }

  return res;
}

int pn532_get_fw_version(FAR struct pn532_dev_s *dev,
                         FAR struct pn_firmware_version *fv)
{
  uint8_t cmd_buffer[4 + 8 + 1];
  FAR struct pn532_frame *f = (FAR struct pn532_frame *) cmd_buffer;
  struct pn_firmware_version *fw;
  int res = -EIO;

  pn532_frame_init(f, PN532_COMMAND_GETFIRMWAREVERSION);
  pn532_frame_finish(f);

  if (pn532_write_frame(dev, f) == OK)
    {
      if (pn532_read_frame(dev, f, 6) == OK)
        {
          if (f->data[0] == PN532_COMMAND_GETFIRMWAREVERSION + 1)
            {
              fw = (FAR struct pn_firmware_version *) &f->data[1];
              ctlsinfo("FW: %d.%d on IC:0x%X (Features: 0x%X)\n",
                        fw->ver, fw->rev, fw->ic, fw->support);
              if (fv)
                {
                  memcpy(fv, fw, sizeof(struct pn_firmware_version));
                }

              res = OK;
            }
        }
    }

  return res;
}

int pn532_write_gpio(FAR struct pn532_dev_s *dev, uint8_t p3, uint8_t p7)
{
  uint8_t cmd_buffer[3 + 7];
  FAR struct pn532_frame *f = (FAR struct pn532_frame *) cmd_buffer;
  int res = -EIO;

  pn532_frame_init(f, PN532_COMMAND_WRITEGPIO);
  f->data[1] = p3;
  f->data[2] = p7;
  f->len += 2;
  pn532_frame_finish(f);

  if (pn532_write_frame(dev, f))
    {
      pn532_read(dev, cmd_buffer, 10);
      tracetx("Resp:", cmd_buffer, 10);
      ctlsinfo("TFI=%x, data0=%X", f->tfi, f->data[0]);
      if ((f->tfi == PN532_PN532TOHOST) &&
          (f->data[0] == PN532_COMMAND_WRITEGPIO + 1))
        {
          res = OK;
        }
    }

  return res;
}

uint32_t pn532_write_passive_data(FAR struct pn532_dev_s *dev,
                                  uint8_t address, FAR uint8_t *data,
                                  uint8_t len)
{
  uint8_t cmd_buffer[8 + 7];
  FAR struct pn532_frame *f = (FAR struct pn532_frame *) cmd_buffer;
  uint8_t resp[20];
  uint32_t res = -EIO;

  pn532_frame_init(f, PN532_COMMAND_INDATAEXCHANGE);
  f->data[1] = 1;       /* max n cards at once */
  f->data[2] = 0xa2;    /* command WRITE */
  f->data[3] = address; /* ADDRESS, 0 = serial */
  memcpy(&f->data[4], data, len);
  f->len += 7;
  pn532_frame_finish(f);

  if (pn532_write_frame(dev, f) == OK)
    {
      if (dev->state == PN532_STATE_DATA_READY)
        {
          if (pn532_read_frame(dev, (FAR struct pn532_frame *) resp, 15)
              == OK)
            {
              dev->state = PN532_STATE_IDLE;
              f = (FAR struct pn532_frame *) resp;
              tracerx("passive target id resp:", (FAR uint8_t *)f,
                      f->len + 6);

              if (f->data[0] == PN532_COMMAND_INDATAEXCHANGE + 1)
                {
                  res = f->data[1];
                }
            }
        }
    }

  return res;
}

uint32_t pn532_read_passive_data(FAR struct pn532_dev_s *dev,
                                 uint8_t address, FAR uint8_t *data,
                                 uint8_t len)
{
  uint8_t cmd_buffer[4 + 7];
  FAR struct pn532_frame *f = (FAR struct pn532_frame *) cmd_buffer;
  uint8_t resp[30];
  uint32_t res = -1;

  pn532_frame_init(f, PN532_COMMAND_INDATAEXCHANGE);
  f->data[1] = 1;       /* max n cards at once */
  f->data[2] = 0x30;    /* command READ */
  f->data[3] = address; /* ADDRESS, 0 = serial */
  f->len += 3;
  pn532_frame_finish(f);

  if (pn532_write_frame(dev, f) == OK)
    {
      if (dev->state == PN532_STATE_DATA_READY)
        {
          if (pn532_read_frame(dev,
                               (FAR struct pn532_frame *)resp, 25) == OK)
            {
              dev->state = PN532_STATE_IDLE;
              f = (FAR struct pn532_frame *) resp;
              tracerx("passive target id resp:", (FAR uint8_t *)f,
                      f->len + 6);

              if (f->data[0] == PN532_COMMAND_INDATAEXCHANGE + 1)
                {
                  if (f->data[1] == 0 && data && len)
                    {
                      memcpy(data, &f->data[2], len);
                    }

                  res = f->data[1];
                }
            }
        }
    }

  return res;
}

uint32_t pn532_read_passive_target_id(FAR struct pn532_dev_s *dev,
                                      uint8_t baudrate)
{
  uint8_t cmd_buffer[4 + 7];
  FAR struct pn532_frame *f = (FAR struct pn532_frame *) cmd_buffer;
  uint8_t resp[20];
  uint32_t res = -EAGAIN;
  int i;

  if (dev->state == PN532_STATE_DATA_READY)
    {
      res = -EIO;
      if (pn532_read_frame(dev, (FAR struct pn532_frame *) resp, 15) == OK)
        {
          FAR struct pn_poll_response *r;

          dev->state = PN532_STATE_IDLE;
          f = (FAR struct pn532_frame *) resp;
          r = (FAR struct pn_poll_response *) &f->data[1];

          tracerx("passive target id resp:", (FAR uint8_t *)f, f->len + 6);

          if (f->data[0] == PN532_COMMAND_INLISTPASSIVETARGET + 1)
            {
              uint32_t cid = 0;

              if (r->nbtg == 1)
                {
                  FAR struct pn_target_type_a *t =
                    (FAR struct pn_target_type_a *)&r->target_data;

                  ctlsinfo("Found %d card(s)\n", r->nbtg);

                  /* now supports only type_a cards
                   * if (poll_mode == PN532_POLL_MOD_106KBPS_A)
                   */

                  ctlsinfo("sens:0x%x  sel:0x%x", t->sens_res, t->sel_res);
                  ctlsinfo("idlen:0x%x ", t->nfcid_len);

                  /* generate 32bit cid from id (could be longer)
                   * HACK: Using only top 4 bytes.
                   */

                  for (i = 0; i < 4 /* t->nfcid_len */; i++)
                    {
                      cid <<= 8;
                      cid |= t->nfcid_data[i];
                    }
                }

              res = cid;
            }
        }
    }

  return res;
}

static int pn532_read_passive_target(FAR struct pn532_dev_s *dev,
                                     uint8_t baudrate)
{
  uint8_t cmd_buffer[4 + 7];
  FAR struct pn532_frame *f = (FAR struct pn532_frame *) cmd_buffer;

  pn532_frame_init(f, PN532_COMMAND_INLISTPASSIVETARGET);
  f->data[1] = 1;
  f->data[2] = baudrate;
  f->len += 2;
  pn532_frame_finish(f);
  return pn532_write_frame(dev, f);
}

bool pn532_set_rf_config(struct pn532_dev_s * dev,
                         struct pn_rf_config_s * conf)
{
  bool res = false;
  uint8_t cmd_buffer[15 + 7];
  FAR struct pn532_frame *f = (FAR struct pn532_frame *) cmd_buffer;

  pn532_frame_init(f, PN532_COMMAND_RFCONFIGURATION);
  f->data[1] = conf->cfg_item;
  memcpy(&f->data[2], conf->config, conf->data_size);
  f->len += conf->data_size + 1;
  pn532_frame_finish(f);

  if (pn532_write_frame(dev, f) == OK)
    {
      pn532_read(dev, (uint8_t *) f, 10);
      tracerx("rf config response", (uint8_t *) f, 10);
      if (pn532_rx_frame_is_valid(f, true))
        {
          if (f->data[0] == PN532_COMMAND_RFCONFIGURATION + 1)
            {
              res = true;
            }
        }
    }

  return res;
}

/****************************************************************************
 * Name: pn532_attachirq
 *
 * Description:
 *   IRQ handling TODO:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

#if 0
static inline int (FAR struct pn532_dev_s *dev, xcpt_t isr)
{
  return dev->config->irqattach(dev, isr);
}

static int irq_handler(int irq, FAR void *context)
{
  ctlsinfo("*IRQ*\n");
  work_queue(HPWORK, &g_dev->irq_work, pn532_worker, dev, 0);

  return OK;
}
#endif

/****************************************************************************
 * Name: pn532_open
 *
 * Description:
 *   This function is called whenever the PN532 device is opened.
 *
 ****************************************************************************/

static int _open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct pn532_dev_s *dev;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = inode->i_private;

  pn532_configspi(dev->spi);

  dev->config->reset(1);
  nxsig_usleep(10000);

  pn532_sam_config(dev, NULL);
  pn532_get_fw_version(dev, NULL);

  dev->state = PN532_STATE_IDLE;
  return OK;
}

/****************************************************************************
 * Name: _close
 *
 * Description:
 *   This routine is called when the PN532 device is closed.
 *
 ****************************************************************************/

static int _close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct pn532_dev_s *dev;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = inode->i_private;

  dev->config->reset(0);
  dev->state = PN532_STATE_NOT_INIT;

#ifdef CONFIG_PM
  if (dev->pm_level >= PM_SLEEP)
    {
#if 0
      priv->config->reset(0);
#endif
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: _read
 *
 * Description:
 *   This routine is called when the device is read.
 *
 * Returned Value:
 *   TAG id as string to buffer or -EIO if no TAG found
 *
 ****************************************************************************/

static ssize_t _read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  FAR struct inode *inode;
  FAR struct pn532_dev_s *dev;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = inode->i_private;

  uint32_t id = pn532_read_passive_target_id(dev, PN532_MIFARE_ISO14443A);
  if (id != 0xffffffff)
    {
      if (buffer)
        {
          return snprintf(buffer, buflen, "0X%X", id);
        }
    }

  return -EIO;
}

/****************************************************************************
 * Name: pn532_write
 ****************************************************************************/

static ssize_t _write(FAR struct file *filep, FAR const char *buffer,
                      size_t buflen)
{
  FAR struct inode *inode;
  FAR struct pn532_dev_s *dev;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = inode->i_private;

  UNUSED(dev);

  return -ENOSYS;
}

/****************************************************************************
 * Name: pn532_ioctl
 ****************************************************************************/

static int _ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct pn532_dev_s *dev;
  int ret = OK;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = inode->i_private;

  switch (cmd)
    {
    case PN532IOC_READ_TAG_DATA:
      {
        FAR struct pn_mifare_tag_data_s *tag_data =
          (FAR struct pn_mifare_tag_data_s *) arg;

        if (tag_data != 0)
          {
            /* HACK: get rid of previous command */

            if (dev->state == PN532_STATE_CMD_SENT)
              {
                if (pn532_wait_rx_ready(dev, 1))
                  {
                    pn532_read_passive_target_id(dev, 0);
                  }
              }

            ret = pn532_read_passive_data(dev, tag_data->address,
                                          (uint8_t *) &tag_data->data,
                                          sizeof(tag_data->data));

            dev->state = PN532_STATE_IDLE;
          }
      }
      break;

    case PN532IOC_WRITE_TAG_DATA:
      {
        FAR struct pn_mifare_tag_data_s *tag_data =
          (FAR struct pn_mifare_tag_data_s *) arg;

        if (tag_data != 0)
          {
            /* HACK: get rid of previous command */

            if (dev->state == PN532_STATE_CMD_SENT)
              {
                if (pn532_wait_rx_ready(dev, 1))
                  {
                    pn532_read_passive_target_id(dev, 0);
                  }
              }

            ret = pn532_write_passive_data(dev, tag_data->address,
                                           (FAR uint8_t *) &tag_data->data,
                                           sizeof(tag_data->data));

            dev->state = PN532_STATE_IDLE;
          }
      }
      break;

    case PN532IOC_SET_SAM_CONF:
      pn532_sam_config(dev, (FAR struct pn_sam_settings_s *) arg);
      break;

    case PN532IOC_READ_PASSIVE:
      if (dev->state == PN532_STATE_CMD_SENT)
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          *ptr = pn532_read_passive_target_id(dev, 0);
        }
      else
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
          *ptr = -1;
        }
      break;

    case PN532IOC_SET_RF_CONF:
      pn532_set_rf_config(dev, (FAR struct pn_rf_config_s *) arg);
      break;

    case PN532IOC_SEND_CMD_READ_PASSIVE:
      ret = pn532_read_passive_target(dev, 0);
      if (ret == 0)
        {
          dev->state = PN532_STATE_CMD_SENT;
        }
      else
        {
          dev->state = PN532_STATE_IDLE;
        }
      break;

    case PN532IOC_GET_DATA_READY:
      if (pn532_wait_rx_ready(dev, 1))
        {
          ret = 0;
        }
      else
        {
          ret = 1;
        }
      break;

    case PN532IOC_GET_TAG_ID:
      {
        FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);
        *ptr = pn532_read_passive_target_id(dev, 0);
      }
      break;

    case PN532IOC_GET_STATE:
      ret = dev->state;
      break;

    default:
      ctlserr("ERROR: Unrecognized cmd: %d\n", cmd);
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pn532_register
 *
 * Description:
 *   Register the PN532 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register.
 *             E.g., "/dev/nfc0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             PN532.
 *   config  - chip config
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int pn532_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                   FAR struct pn532_config_s *config)
{
  FAR struct pn532_dev_s *dev;
  int ret;

  /* Initialize the PN532 device structure */

  dev = (FAR struct pn532_dev_s *)kmm_malloc(sizeof(struct pn532_dev_s));
  if (!dev)
    {
      ctlserr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  dev->spi = spi;
  dev->config = config;

#if defined CONFIG_PM
  dev->pm_level = PM_IDLE;
#endif

  /* pn532_attachirq(dev, pn532_irqhandler); */

  /* Register the character driver */

  ret = register_driver(devpath, &g_pn532fops, 0666, dev);
  if (ret < 0)
    {
      ctlserr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(dev);
    }

  return ret;
}
