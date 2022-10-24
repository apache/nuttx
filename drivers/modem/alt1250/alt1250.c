/****************************************************************************
 * drivers/modem/alt1250/alt1250.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <poll.h>
#include <errno.h>
#include <nuttx/wireless/lte/lte_ioctl.h>
#include <nuttx/modem/alt1250.h>
#include <assert.h>

#include "altcom_pkt.h"
#include "altcom_hdlr.h"
#include "altmdm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WRITE_OK 0
#define WRITE_NG 1

#define rel_evtbufinst(inst, dev) unlock_evtbufinst(inst, dev)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods. */

static int alt1250_open(FAR struct file *filep);
static int alt1250_close(FAR struct file *filep);
static ssize_t alt1250_read(FAR struct file *filep, FAR char *buffer,
                            size_t len);
static int alt1250_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int alt1250_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);

parse_handler_t alt1250_additional_parsehdlr(uint16_t, uint8_t);
compose_handler_t alt1250_additional_composehdlr(uint32_t, FAR uint8_t *,
                                                 size_t);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface. */

static const struct file_operations g_alt1250fops =
{
  alt1250_open,  /* open */
  alt1250_close, /* close */
  alt1250_read,  /* read */
  NULL,          /* write */
  NULL,          /* seek */
  alt1250_ioctl, /* ioctl */
  alt1250_poll,  /* poll */
};
static uint8_t g_recvbuff[ALTCOM_RX_PKT_SIZE_MAX];
static uint8_t g_sendbuff[ALTCOM_PKT_SIZE_MAX];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: add_list
 ****************************************************************************/

static void add_list(FAR struct alt_queue_s *head,
                     FAR struct alt_container_s *list)
{
  FAR struct alt_container_s *next;

  nxmutex_lock(&head->lock);

  while (list != NULL)
    {
      next = (FAR struct alt_container_s *)sq_next(&list->node);

      sq_next(&list->node) = NULL;
      sq_addlast(&list->node, &head->queue);

      list = next;
    }

  nxmutex_unlock(&head->lock);
}

/****************************************************************************
 * Name: remove_list_all
 ****************************************************************************/

static FAR struct alt_container_s *remove_list_all(
  FAR struct alt_queue_s *head)
{
  FAR struct alt_container_s *list;

  nxmutex_lock(&head->lock);

  list = (FAR struct alt_container_s *)sq_peek(&head->queue);
  sq_init(&head->queue);

  nxmutex_unlock(&head->lock);

  return list;
}

/****************************************************************************
 * Name: remove_list
 ****************************************************************************/

static FAR struct alt_container_s *remove_list(FAR struct alt_queue_s *head,
                                            uint16_t cmdid, uint16_t transid)
{
  FAR struct alt_container_s *list;

  nxmutex_lock(&head->lock);

  list = (FAR struct alt_container_s *)sq_peek(&head->queue);
  while (list != NULL)
    {
      if ((list->altcid == cmdid) && (list->alttid == transid))
        {
          sq_rem(&list->node, &head->queue);
          sq_next(&list->node) = NULL;
          break;
        }

      list = (FAR struct alt_container_s *)sq_next(&list->node);
    }

  nxmutex_unlock(&head->lock);

  return list;
}

/****************************************************************************
 * Name: set_senddisable
 ****************************************************************************/

static void set_senddisable(FAR struct alt1250_dev_s *dev, bool disable)
{
  nxmutex_lock(&dev->senddisablelock);

  dev->senddisable = disable;

  nxmutex_unlock(&dev->senddisablelock);
}

/****************************************************************************
 * Name: is_senddisable
 ****************************************************************************/

static bool is_senddisable(FAR struct alt1250_dev_s *dev)
{
  bool disable;

  nxmutex_lock(&dev->senddisablelock);

  disable = dev->senddisable;

  nxmutex_unlock(&dev->senddisablelock);

  return disable;
}

/****************************************************************************
 * Name: read_evtbitmap
 ****************************************************************************/

static ssize_t read_data(FAR struct alt1250_dev_s *dev,
                         FAR struct alt_readdata_s *rdata)
{
  int idx;

  nxmutex_lock(&dev->evtmaplock);

  /* change status to NOT WRITABLE */

  for (idx = 0; idx < (sizeof(uint64_t) * 8); idx++)
    {
      if ((dev->evtbitmap & (1ULL << idx)) != 0)
        {
          if (dev->evtbuff->ninst >= idx)
            {
              FAR alt_evtbuf_inst_t *inst = &dev->evtbuff->inst[idx];

              nxmutex_lock(&inst->stat_lock);

              inst->stat = ALTEVTBUF_ST_NOTWRITABLE;

              nxmutex_unlock(&inst->stat_lock);
            }
        }
    }

  rdata->evtbitmap = dev->evtbitmap;
  rdata->head = remove_list_all(&dev->replylist);

  if (dev->evtbitmap & ALT1250_EVTBIT_RESET)
    {
      /* Resume sending because daemon has been notified of the reset
       * reliably.
       */

      set_senddisable(dev, false);
    }

  dev->evtbitmap = 0ULL;

  nxmutex_unlock(&dev->evtmaplock);

  return sizeof(struct alt_readdata_s);
}

/****************************************************************************
 * Name: write_evtbitmap
 ****************************************************************************/

static void write_evtbitmap(FAR struct alt1250_dev_s *dev,
                            uint64_t bitmap)
{
  nxmutex_lock(&dev->evtmaplock);

  dev->evtbitmap |= bitmap;

  if ((dev->evtbitmap & ALT1250_EVTBIT_RESET) != 0)
    {
      dev->evtbitmap = ALT1250_EVTBIT_RESET;
    }

  m_info("write bitmap: 0x%llx\n", bitmap);

  nxmutex_unlock(&dev->evtmaplock);
}

/****************************************************************************
 * Name: write_evtbitmapwithlist
 ****************************************************************************/

static void write_evtbitmapwithlist(FAR struct alt1250_dev_s *dev,
  uint64_t bitmap, FAR struct alt_container_s *container)
{
  nxmutex_lock(&dev->evtmaplock);

  dev->evtbitmap |= bitmap;

  if (dev->evtbitmap & ALT1250_EVTBIT_RESET)
    {
      dev->evtbitmap = ALT1250_EVTBIT_RESET;
    }

  add_list(&dev->replylist, container);

  nxmutex_unlock(&dev->evtmaplock);
}

/****************************************************************************
 * Name: is_evtbitmap_avail
 ****************************************************************************/

static int is_evtbitmap_avail(FAR struct alt1250_dev_s *dev)
{
  int ret;

  nxmutex_lock(&dev->evtmaplock);

  /* 0 means it is not available, otherwise it is available. */

  ret = (0ULL != dev->evtbitmap);

  nxmutex_unlock(&dev->evtmaplock);

  return ret;
}

/****************************************************************************
 * Name: add_evtbuff
 ****************************************************************************/

static void add_evtbuff(FAR struct alt1250_dev_s *dev,
                        FAR struct alt_evtbuffer_s *buff)
{
  dev->evtbuff = buff;
}

/****************************************************************************
 * Name: write_evtbuff_byidx
 ****************************************************************************/

static int write_evtbuff_byidx(FAR struct alt1250_dev_s *dev,
  uint64_t idx, CODE void(*write_func)(FAR void *outp[], FAR void *inp),
  FAR void *inp)
{
  int ret = WRITE_NG;

  nxmutex_lock(&dev->evtmaplock);

  if (dev->evtbuff)
    {
      if (dev->evtbuff->ninst >= idx)
        {
          FAR alt_evtbuf_inst_t *inst = &dev->evtbuff->inst[idx];

          nxmutex_lock(&inst->stat_lock);
          if (inst->stat == ALTEVTBUF_ST_WRITABLE)
            {
              write_func(inst->outparam, inp);
              dev->evtbitmap |= (1ULL << idx);
              ret = WRITE_OK;
            }

          nxmutex_unlock(&inst->stat_lock);
        }
    }

  nxmutex_unlock(&dev->evtmaplock);

  return ret;
}

/****************************************************************************
 * Name: lock_evtbuffinst
 ****************************************************************************/

static void lock_evtbuffinst(FAR alt_evtbuf_inst_t *inst,
                             FAR struct alt1250_dev_s *dev)
{
  nxmutex_lock(&dev->evtmaplock);
  nxmutex_lock(&inst->stat_lock);
}

/****************************************************************************
 * Name: unlock_evtbufinst
 ****************************************************************************/

static void unlock_evtbufinst(FAR alt_evtbuf_inst_t *inst,
                              FAR struct alt1250_dev_s *dev)
{
  nxmutex_unlock(&inst->stat_lock);
  nxmutex_unlock(&dev->evtmaplock);
}

/****************************************************************************
 * Name: search_evtbufinst
 ****************************************************************************/

static FAR alt_evtbuf_inst_t *search_evtbufinst(uint16_t cid,
  FAR uint64_t *bitmap, FAR struct alt1250_dev_s *dev)
{
  FAR alt_evtbuf_inst_t *ret = NULL;
  unsigned int i;

  *bitmap = 0ULL;

  for (i = 0; i < dev->evtbuff->ninst; i++)
    {
      ret = &dev->evtbuff->inst[i];

      if (ret->altcid == cid)
        {
          *bitmap = 1ULL << i;
          return ret;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: cid_to_searchable
 ****************************************************************************/

static uint16_t cid_to_searchable(uint16_t cid, uint8_t altver)
{
  uint16_t cidv1;

  cid &= ~ALTCOM_CMDID_REPLY_BIT;
  if (altver == ALTCOM_VER4)
    {
      /* Change the command ID to Version 1
       * Even if it cannot be converted, try to search the table
       * using the original command ID.
       */

      cidv1 = convert_cid2v1(cid);
      if (cidv1 != APICMDID_UNKNOWN)
        {
          cid = cidv1;
        }
    }

  return cid;
}

/****************************************************************************
 * Name: get_bitmap
 ****************************************************************************/

static uint64_t get_bitmap(FAR struct alt1250_dev_s *dev, uint16_t cid,
                           uint8_t altver)
{
  uint64_t bitmap = 0ULL;

  cid = cid_to_searchable(cid, altver);

  search_evtbufinst(cid, &bitmap, dev);

  return bitmap;
}

/****************************************************************************
 * Name: get_evtbuffinst_withlock
 ****************************************************************************/

static FAR alt_evtbuf_inst_t *get_evtbuffinst_withlock(
  FAR struct alt1250_dev_s *dev, uint16_t cid, uint8_t altver,
  FAR uint64_t *bitmap)
{
  FAR alt_evtbuf_inst_t *inst = NULL;
  FAR alt_evtbuf_inst_t *ret = NULL;

  cid = cid_to_searchable(cid, altver);

  if (cid == APICMDID_SOCK_SELECT)
    {
      ret = &dev->select_inst;

      lock_evtbuffinst(ret, dev);

      ret->outparam = dev->select_container->outparam;
      ret->outparamlen = dev->select_container->outparamlen;

      search_evtbufinst(cid, bitmap, dev);
    }
  else
    {
      inst = search_evtbufinst(cid, bitmap, dev);
      if (inst)
        {
          lock_evtbuffinst(inst, dev);

          if (inst->stat == ALTEVTBUF_ST_WRITABLE)
            {
              ret = inst;
            }
          else
            {
              unlock_evtbufinst(inst, dev);
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: write_restart_param
 ****************************************************************************/

static void write_restart_param(FAR void *outp[], FAR void *buff)
{
  FAR int *out_reason = (FAR int *)outp[0];
  FAR int *in_reason = (FAR int *)buff;

  *out_reason = *in_reason;
}

/****************************************************************************
 * Name: pollnotify
 ****************************************************************************/

static void pollnotify(FAR struct alt1250_dev_s *dev)
{
  nxmutex_lock(&dev->pfdlock);

  if (dev->pfd != NULL)
    {
      /* If poll() waits, notify  */

      poll_notify(&dev->pfd, 1, POLLIN);
    }

  nxmutex_unlock(&dev->pfdlock);
}

/****************************************************************************
 * Name: get_composehdlr
 ****************************************************************************/

compose_handler_t get_composehdlr(uint32_t cmdid, FAR uint8_t *payload,
                                  size_t size)
{
  compose_handler_t ret;

  ret = alt1250_composehdlr(cmdid);

#ifdef CONFIG_MODEM_ALT1250_ADDITIONAL_FUNC
  if (ret == NULL)
    {
      ret = alt1250_additional_composehdlr(cmdid, payload, size);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: get_parsehdlr
 ****************************************************************************/

parse_handler_t get_parsehdlr(uint16_t altcid, uint8_t altver)
{
  parse_handler_t ret;

  ret = alt1250_parsehdlr(altcid, altver);

#ifdef CONFIG_MODEM_ALT1250_ADDITIONAL_FUNC
  if (ret == NULL)
    {
      ret = alt1250_additional_parsehdlr(altcid, altver);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: alt1250_power_control
 ****************************************************************************/

static int alt1250_power_control(FAR struct alt1250_dev_s *dev,
                                 FAR struct alt_power_s *req)
{
  int ret = OK;

  switch (req->cmdid)
    {
      case LTE_CMDID_POWERON:
        ret = altmdm_poweron();
        break;

      case LTE_CMDID_POWEROFF:
        ret = altmdm_poweroff();
        break;

      case LTE_CMDID_TAKEWLOCK:
        ret = altmdm_take_wlock();
        break;

      case LTE_CMDID_GIVEWLOCK:
        ret = altmdm_give_wlock();
        break;

      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: make_altcomcmd_and_send
 ****************************************************************************/

static int make_altcomcmd_and_send(FAR struct alt1250_dev_s *dev,
                                   FAR alt_container_t *req)
{
  int ret = OK;
  compose_handler_t handler;
  uint8_t altver;
  uint16_t cid;
  uint16_t tid;
  FAR uint8_t *payload;
  int remainlen;
  int pos;

  m_info("send request: command ID=0x%08lx\n", req->cmdid);

  payload = get_payload((FAR struct altcom_cmdhdr_s *)g_sendbuff);

  handler = get_composehdlr(req->cmdid & ~LTE_CMDOPT_ASYNC_BIT, payload,
                            ALTCOM_PAYLOAD_SIZE_MAX);
  if (handler)
    {
      altver = altmdm_get_protoversion();
      if ((altver == ALTCOM_VERX) || is_senddisable(dev))
        {
          ret = -ENETDOWN;
        }
      else
        {
          ret = handler(req->inparam, req->inparamlen, altver, payload,
                        ALTCOM_PAYLOAD_SIZE_MAX, &cid);

          ret = (ret > ALTCOM_PAYLOAD_SIZE_MAX) ? -ENOSPC : ret;

          if (ret >= 0)
            {
              tid = altcom_make_header(
                (FAR struct altcom_cmdhdr_s *)g_sendbuff,
                altver, cid, ret);

              req->altcid = cid | ALTCOM_CMDID_REPLY_BIT;
              req->alttid = tid;

              if (req->outparam != NULL)
                {
                  add_list(&dev->waitlist, req);
                }

              remainlen = get_pktlen(altver, (uint16_t)ret);
              pos = 0;

              /* If the modem sleeps during the split transmission,
               * the receive buffer of the modem will be cleared.
               * Therefore, split packets sent before sleep will be
               * discarded. To avoid this, do not enter sleep state
               * until all the packets have been sent.
               */

              altmdm_take_wlock();

              do
                {
                  ret = altmdm_write(&g_sendbuff[pos], remainlen);
                  if (ret < 0)
                    {
                      break;
                    }
                  else
                    {
                      m_info(
                        "write success: size=%d, cid=0x%04x tid=0x%04x\n",
                        ret, cid, tid);
                      remainlen -= ret;
                      pos += ret;
                    }
                }
              while (remainlen > 0);

              altmdm_give_wlock();

              if (ret < 0)
                {
                  m_err("altmdm_write() failed: %d\n", ret);
                  ret = -ENETDOWN;

                  /* If the container is not left in the waitlist,
                   * it has already been processed by the recvthread.
                   * ENETRESET is returned to the caller to indicate that
                   * the container has been processed.
                   */

                  if ((req->outparam != NULL) && (remove_list(&dev->waitlist,
                       req->altcid, req->alttid) == NULL))
                    {
                      ret = -ENETRESET;
                    }
                }
              else
                {
                  m_info("write success: size=%d, cid=0x%04x tid=0x%04x\n",
                         ret, cid, tid);
                  ret = OK;
                }
            }
          else
            {
              m_err("handler() failed: %d\n", ret);
            }
        }
    }
  else
    {
      ret = -ENOSYS;
    }

  return ret;
}

/****************************************************************************
 * Name: exchange_selectcontainer
 ****************************************************************************/

static int exchange_selectcontainer(FAR struct alt1250_dev_s *dev,
                                    FAR alt_container_t **container)
{
  FAR alt_container_t *newcontainer;

  if (container == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->select_inst.stat_lock);

  newcontainer = *container;
  *container = dev->select_container;
  dev->select_container = newcontainer;

  nxmutex_unlock(&dev->select_inst.stat_lock);

  return OK;
}

/****************************************************************************
 * Name: altcom_recvthread
 ****************************************************************************/

static void altcom_recvthread(FAR void *arg)
{
  int ret;
  FAR struct alt1250_dev_s *dev = (FAR struct alt1250_dev_s *)arg;
  bool is_running = true;
  FAR struct alt_container_s *head;
  FAR struct alt_container_s *container;
  uint16_t cid;
  uint16_t tid;
  uint8_t altver;
  parse_handler_t handler;
  uint64_t bitmap = 0ULL;
  int recvedlen = 0;

  m_info("recv thread start\n");

  altmdm_init(dev->spi, dev->lower);

  while (is_running)
    {
      ret = altmdm_read(g_recvbuff + recvedlen,
                        ALTCOM_RX_PKT_SIZE_MAX - recvedlen);

      /* Normal packet received */

      if (ret >= 0)
        {
          m_info("read packet %d bytes\n", ret);

          recvedlen += ret;

          ret = altcom_is_pkt_ok(g_recvbuff, recvedlen);
          if (ret > 0)
            {
              /* Cases in which fragmented packets are received.
               * Therefore, the receive process is performed again.
               */

              m_info("This is fragmented packet received. remain len: %d\n",
                     ret);
              continue;
            }

          if (ret < 0)
            {
              /* Forced reset of modem due to packet format error detected */

              m_err("[altcom] Forced modem reset due to parse failure\n");

              altmdm_reset();
            }
          else
            {
              bool is_discard = false;

              /* parse ALTCOM command ID and transaction ID from header */

              cid = parse_cid((FAR struct altcom_cmdhdr_s *)g_recvbuff);
              tid = parse_tid((FAR struct altcom_cmdhdr_s *)g_recvbuff);
              altver = get_altver(
                    (FAR struct altcom_cmdhdr_s *)g_recvbuff);

              m_info("receive cid:0x%04x tid:0x%04x\n", cid, tid);

              /* Is error indication packet?
               * This packet is a response to a command that is not supported
               * by the ALT1250. The header of the request packet is included
               * in the contents of the this packet.
               */

              if (is_errind(cid))
                {
                  /* Get ALTCOM command ID and transaction ID
                   * from error indication packet
                   */

                  cid = parse_cid4errind(
                    (FAR struct altcom_cmdhdr_s *)g_recvbuff);
                  tid = parse_tid4errind(
                    (FAR struct altcom_cmdhdr_s *)g_recvbuff);

                  m_info("receive errind cid:0x%04x tid:0x%04x\n", cid, tid);

                  container = remove_list(&dev->waitlist, cid, tid);
                  if (container != NULL)
                    {
                      /* It means that requested command not implemented
                       * by modem
                       */

                      container->result = -ENOSYS;
                    }
                  else
                    {
                      /* Discard the event packet */

                      is_discard = true;

                      m_warn("container is not found\n");
                    }
                }
              else
                {
                  container = remove_list(&dev->waitlist, cid, tid);

                  handler = get_parsehdlr(cid, altver);
                  if (handler)
                    {
                      FAR uint8_t *payload = get_payload(
                        (FAR struct altcom_cmdhdr_s *)g_recvbuff);

                      if (container)
                        {
                          m_info("handler and container is found\n");

                          bitmap = get_bitmap(dev, cid, altver);

                          /* Perform parse handler */

                          container->result = handler(dev, payload,
                            get_payload_len(
                              (FAR struct altcom_cmdhdr_s *)g_recvbuff),
                            altver, container->outparam,
                            container->outparamlen, &bitmap);
                        }
                      else
                        {
                          FAR alt_evtbuf_inst_t *inst;

                          m_warn("container is not found\n");

                          /* If the state of the instance is NotWritable,
                           * instanse will be returned as NULL.
                           */

                          inst = get_evtbuffinst_withlock(dev, cid, altver,
                                                          &bitmap);
                          if (inst)
                            {
                              /* Perform parse handler */

                              ret = handler(dev, payload, get_payload_len(
                                (FAR struct altcom_cmdhdr_s *)g_recvbuff),
                                altver, inst->outparam, inst->outparamlen,
                                &bitmap);

                              unlock_evtbufinst(inst, dev);

                              if (ret >= 0)
                                {
                                  write_evtbitmap(dev, bitmap);
                                }
                              else
                                {
                                  /* Discard the event packet */

                                  is_discard = true;
                                }
                            }
                          else
                            {
                              /* Discard the event packet */

                              is_discard = true;
                            }
                        }
                    }
                  else if (container)
                    {
                      container->result = -ENOSYS;
                      m_warn("handler is not found\n");
                    }
                  else
                    {
                      /* Discard the event packet */

                      is_discard = true;

                      m_warn("container and handler is not found\n");
                    }
                }

              if (container)
                {
                  if (container->cmdid & LTE_CMDOPT_ASYNC_BIT)
                    {
                      bitmap |= ALT1250_EVTBIT_REPLY;
                    }
                  else
                    {
                      bitmap = ALT1250_EVTBIT_REPLY;
                    }

                  write_evtbitmapwithlist(dev, bitmap, container);
                }

              if (is_discard)
                {
                  dev->discardcnt++;
                  m_err("discard event %lu\n", dev->discardcnt);
                }
              else
                {
                  pollnotify(dev);
                }
            }
        }
      else
        {
          switch (ret)
            {
              case ALTMDM_RETURN_RESET_PKT:
                {
                  m_info("recieve ALTMDM_RETURN_RESET_PKT\n");
                  set_senddisable(dev, true);
                }
                break;

              case ALTMDM_RETURN_RESET_V1:
              case ALTMDM_RETURN_RESET_V4:
                {
                  uint32_t reason = altmdm_get_reset_reason();

                  m_info("recieve ALTMDM_RETURN_RESET_V1/V4\n");

                  ret = write_evtbuff_byidx(dev, 0, write_restart_param,
                    (FAR void *)&reason);

                  /* If there is a waiting list,
                   * replace it with the replay list.
                   */

                  head = remove_list_all(&dev->waitlist);

                  write_evtbitmapwithlist(dev, ALT1250_EVTBIT_RESET, head);
                  pollnotify(dev);
                }
                break;

              case ALTMDM_RETURN_EXIT:
                {
                  m_info("recieve ALTMDM_RETURN_EXIT\n");
                  is_running = false;
                }
                break;

              default:
                DEBUGASSERT(0);
                break;
            }
        }

      recvedlen = 0;
    }

  m_info("recv thread end\n");

  pthread_exit(0);
}

/****************************************************************************
 * Name: alt1250_open
 ****************************************************************************/

static int alt1250_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct alt1250_dev_s *dev;
  int ret = OK;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  dev = (FAR struct alt1250_dev_s *)inode->i_private;
  DEBUGASSERT(dev);

  nxmutex_lock(&dev->refslock);

  if (dev->crefs > 0)
    {
      ret = -EPERM;
    }

  /* Increment the count of open references on the driver */

  dev->crefs++;

  nxmutex_unlock(&dev->refslock);

  if (ret == OK)
    {
      nxsem_init(&dev->waitlist.lock, 0, 1);
      nxsem_init(&dev->replylist.lock, 0, 1);
      nxsem_init(&dev->evtmaplock, 0, 1);
      nxsem_init(&dev->pfdlock, 0, 1);
      nxsem_init(&dev->senddisablelock, 0, 1);
      nxsem_init(&dev->select_inst.stat_lock, 0, 1);

      sq_init(&dev->waitlist.queue);
      sq_init(&dev->replylist.queue);

      dev->senddisable = true;

      ret = pthread_create(&dev->recvthread, NULL,
        (pthread_startroutine_t)altcom_recvthread,
        (pthread_addr_t)dev);
      if (ret < 0)
        {
          m_err("thread create failed: %d\n", errno);
          ret = -errno;

          nxsem_destroy(&dev->waitlist.lock);
          nxsem_destroy(&dev->replylist.lock);
          nxsem_destroy(&dev->evtmaplock);
          nxsem_destroy(&dev->pfdlock);
          nxsem_destroy(&dev->senddisablelock);
          nxsem_destroy(&dev->select_inst.stat_lock);

          nxmutex_lock(&dev->refslock);
          dev->crefs--;
          nxmutex_unlock(&dev->refslock);
        }
      else
        {
          pthread_setname_np(dev->recvthread, "altcom_recvthread");
        }
    }

  return ret;
}

/****************************************************************************
 * Name: alt1250_close
 ****************************************************************************/

static int alt1250_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct alt1250_dev_s *dev;
  int ret = OK;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  dev = (FAR struct alt1250_dev_s *)inode->i_private;
  DEBUGASSERT(dev);

  nxmutex_lock(&dev->refslock);

  if (dev->crefs == 0)
    {
      ret = -EPERM;
    }
  else
    {
      /* Decrement the count of open references on the driver */

      dev->crefs--;
    }

  nxmutex_unlock(&dev->refslock);

  if (ret == OK)
    {
      nxsem_destroy(&dev->waitlist.lock);
      nxsem_destroy(&dev->replylist.lock);
      nxsem_destroy(&dev->evtmaplock);
      nxsem_destroy(&dev->pfdlock);
      nxsem_destroy(&dev->senddisablelock);
      nxsem_destroy(&dev->select_inst.stat_lock);

      altmdm_fin();
      pthread_join(dev->recvthread, NULL);
    }

  return ret;
}

/****************************************************************************
 * Name: alt1250_read
 ****************************************************************************/

static ssize_t alt1250_read(FAR struct file *filep, FAR char *buffer,
                            size_t len)
{
  FAR struct inode *inode;
  FAR struct alt1250_dev_s *dev;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  dev = (FAR struct alt1250_dev_s *)inode->i_private;
  DEBUGASSERT(dev);

  if (len != sizeof(struct alt_readdata_s))
    {
      return -EINVAL;
    }

  return read_data(dev, (FAR struct alt_readdata_s *)buffer);
}

/****************************************************************************
 * Name: alt1250_ioctl
 ****************************************************************************/

static int alt1250_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct alt1250_dev_s *dev;
  int ret = OK;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  dev = (FAR struct alt1250_dev_s *)inode->i_private;
  DEBUGASSERT(dev);

  switch (cmd)
    {
      case ALT1250_IOC_POWER:
        {
          FAR struct alt_power_s *req = (FAR struct alt_power_s *)arg;

          /* Performs power control or power consumption control
           * of the modem.
           */

          ret = alt1250_power_control(dev, req);
        }
        break;

      case ALT1250_IOC_SEND:
        {
          FAR alt_container_t *req = (FAR alt_container_t *)arg;

          ret = make_altcomcmd_and_send(dev, req);
        }
        break;

      case ALT1250_IOC_SETEVTBUFF:
        {
          FAR struct alt_evtbuffer_s *buff =
            (FAR struct alt_evtbuffer_s *)arg;
          add_evtbuff(dev, buff);
        }
        break;

      case ALT1250_IOC_EXCHGCONTAINER:
        {
          FAR alt_container_t **container = (FAR alt_container_t **)arg;

          ret = exchange_selectcontainer(dev, container);
        }
        break;

      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: alt1250_poll
 ****************************************************************************/

static int alt1250_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode *inode;
  FAR struct alt1250_dev_s *dev;
  int ret = OK;

  /* Get our private data structure */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  dev = (FAR struct alt1250_dev_s *)inode->i_private;
  DEBUGASSERT(dev);

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      nxmutex_lock(&dev->pfdlock);

      if (is_evtbitmap_avail(dev))
        {
          poll_notify(&fds, 1, POLLIN);
        }
      else
        {
          dev->pfd = fds;
        }

      nxmutex_unlock(&dev->pfdlock);
    }
  else
    {
      nxmutex_lock(&dev->pfdlock);
      dev->pfd = NULL;
      nxmutex_unlock(&dev->pfdlock);
    }

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR void *alt1250_register(FAR const char *devpath,
                           FAR struct spi_dev_s *dev,
                           FAR const struct alt1250_lower_s *lower)
{
  FAR struct alt1250_dev_s *priv;
  int ret;

  priv = (FAR struct alt1250_dev_s *)
    kmm_malloc(sizeof(struct alt1250_dev_s));
  if (!priv)
    {
      m_err("Failed to allocate instance.\n");
      return NULL;
    }

  memset(priv, 0, sizeof(struct alt1250_dev_s));

  priv->spi = dev;
  priv->lower = lower;

  nxsem_init(&priv->refslock, 0, 1);

  ret = register_driver(devpath, &g_alt1250fops, 0666, priv);
  if (ret < 0)
    {
      m_err("Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return NULL;
    }

  return priv;
}

uint64_t get_event_lapibuffer(FAR struct alt1250_dev_s *dev,
                              uint32_t lapicmdid,
                              alt_evtbuf_inst_t **inst)
{
  FAR alt_evtbuf_inst_t *evtinst = NULL;
  unsigned int i;
  uint64_t ret = 0ULL;

  for (i = 0; i < dev->evtbuff->ninst; i++)
    {
      evtinst = &dev->evtbuff->inst[i];

      if (evtinst->cmdid == lapicmdid)
        {
          nxmutex_lock(&evtinst->stat_lock);

          if (evtinst->stat == ALTEVTBUF_ST_WRITABLE)
            {
              *inst = evtinst;
              ret = 1ULL << i;
            }

          nxmutex_unlock(&evtinst->stat_lock);
          break;
        }
    }

  return ret;
}
