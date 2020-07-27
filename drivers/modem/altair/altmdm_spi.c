/****************************************************************************
 * drivers/modem/altmdm/altmdm_spi.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>
#include <nuttx/modem/altmdm.h>

#include "altmdm_sys.h"
#include "altmdm_pm.h"
#include "altmdm_pm_state.h"

#if defined(CONFIG_MODEM_ALTMDM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EVENT_BIT(b)       (1<<(b))
#define EVENT_NONE         (0)
#define EVENT_TXREQ        EVENT_BIT(0)
#define EVENT_RXREQ        EVENT_BIT(1)
#define EVENT_CANCEL       EVENT_BIT(2)
#define EVENT_EXIT         EVENT_BIT(3)
#define EVENT_RXRDY        EVENT_BIT(4)
#define EVENT_RXBUFRDY     EVENT_BIT(5)
#define EVENT_TX_DONE      EVENT_BIT(6)
#define EVENT_RX_DONE      EVENT_BIT(7)
#define EVENT_SLEEPREQ     EVENT_BIT(8)
#define EVENT_SLEEP_DONE   EVENT_BIT(9)
#define EVENT_SV_TIMER_EXP EVENT_BIT(10)
#define EVENT_XFERRDY      EVENT_BIT(11)
#define EVENT_REQMASK      (EVENT_TXREQ | EVENT_RXREQ | EVENT_SLEEPREQ | \
                            EVENT_SV_TIMER_EXP)
#define EVENT_TRANS_WAIT   (EVENT_TXREQ | EVENT_RXREQ | EVENT_RXBUFRDY | \
                            EVENT_SLEEPREQ | EVENT_SV_TIMER_EXP | EVENT_EXIT)
#if defined(CONFIG_MODEM_ALTMDM_XFER_TASK_PRIORITY)
#  define XFER_TASK_PRI    (CONFIG_MODEM_ALTMDM_XFER_TASK_PRIORITY)
#else
#  define XFER_TASK_PRI    (170)
#endif

#define XFER_TASK_STKSIZE  (1536)
#define XFER_TASK_NAME     "altmdm_xfer_task"
#if defined(CONFIG_MODEM_ALTMDM_MAX_PACKET_SIZE)
#  if ((CONFIG_MODEM_ALTMDM_MAX_PACKET_SIZE & 0x3) != 0)
#    error MODEM_ALTMDM_MAX_PACKET_SIZE must be aligned 0x4 (4B)
#  endif
#  define MAX_PKT_SIZE     (CONFIG_MODEM_ALTMDM_MAX_PACKET_SIZE)
#else
#  define MAX_PKT_SIZE     (2064)
#endif
#define UNIT_SIZE          (4)
#define RX_ENABLE          (0)
#define RX_DISABLE         (1)
#define SLEEP_OK           (0)
#define SLEEP_NG           (-EBUSY)

/* Timeout is counted in units of millisecond. */

#define WAIT_RXREQ_TIMEOUT    (5)
#define WAIT_RXREQ_HEADER_TIMEOUT (5 * 1000)
#define WAIT_XFERRDY_TIMEOUT  (25 * 1000)
#if defined(CONFIG_MODEM_ALTMDM_SLEEP_TIMER_VAL)
#  if (CONFIG_MODEM_ALTMDM_SLEEP_TIMER_VAL < 20)
#    error MODEM_ALTMDM_SLEEP_TIMER_VAL too small
#  endif
#  define SV_TIMER_TIMOUT_VAL (CONFIG_MODEM_ALTMDM_SLEEP_TIMER_VAL)
#else
#  define SV_TIMER_TIMOUT_VAL (20)
#endif
#define WRITE_WAIT_TIMEOUT    (ALTMDM_SYS_FLAG_TMOFEVR)
#define SREQ_WAIT_TIMEOUT     (ALTMDM_SYS_FLAG_TMOFEVR)
#define YIELD_TASK_NOBUFF     (100 * 1000) /* microsecond */

/* Defines for transfer mode */

#define MODE_RXDATA             (0)   /* Data receive mode. */
#define MODE_TXDATA             (1)   /* Data send mode. */
#define MODE_TRXDATA            (2)   /* Data send and receive mode. */
#define MODE_RXDATANOBUFF       (3)   /* Data receive mode when there is no
                                       * receiving buffer.
                                       */
#define MODE_TRXDATANOBUFF      (4)   /* Data send and receive mode when
                                       * there is no receiving buffer.
                                       */
#define MODE_RXINVALID          (5)   /* Data receive mode when receiving
                                       * header is wrong.
                                       */
#define MODE_TRXHEADERFAILTXREQ (6)   /* Data send and receive mode when
                                       * header transfer fails. Send request
                                       * triggered.
                                       */
#define MODE_TRXHEADERFAILRXREQ (7)   /* Data receive mode when header
                                       * transfer fails. Receive request
                                       * triggered.
                                       */
#define MODE_RXRESET            (8)   /* Reset packet receive mode. */
#define MODE_TRXRESET           (9)   /* Data send and reset packet receive
                                       * mode.
                                       */

/* Defines for transfer result code */

#define TRANS_OK                 (1)  /* OK. */
#define TRANS_OK_RXDATANOBUFF    (-1) /* OK when MODE_RXDATANOBUFF mode. */
#define TRANS_OK_TRXDATANORXBUFF (-2) /* OK when MODE_TRXDATANOBUFF mode. */
#define TRANS_RXINVALID          (-3) /* NG when MODE_RXINVALID mode. */
#define TRANS_WAITRCVRTMO        (-4) /* NG when receiver ready timeout. */
#define TRANS_OK_RCVBUFFUL       (-5) /* OK when receiver is buffer full. */

#define RESET_BOOTSTAT_BOOTING   (0)  /* Boot stat: booting */
#define RESET_BOOTSTAT_UPDATING  (1)  /* Boot stat: updating */

#define STAT_INF_GET_BOOTSTAT    (1 << 1) /* Status info: Get bootstatus bit */
#define STAT_INF_RESET           (1 << 2) /* Status info: Reset bit */
#define STAT_INF_BUFFFULL        (1 << 3) /* Status info: Buffer full bit */

#define STAT_INF_IS_RESET(info)     ((info & STAT_INF_RESET)    >> 2)
#define STAT_INF_IS_BUFF_FULL(info) ((info & STAT_INF_BUFFFULL) >> 3)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct altmdm_dev_s *g_privdata = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: svtimer_handler
 *
 * Description:
 *   Timeout handler for supervisory timer.
 *
 ****************************************************************************/

static void svtimer_handler(int signo, FAR siginfo_t * info,
                            FAR void *ucontext)
{
  FAR struct altmdm_dev_s *priv =
    (FAR struct altmdm_dev_s *)(info->si_value.sival_ptr);
  struct altmdm_sys_flagstate_s flag_status;

  altmdm_sys_referflag(&priv->spidev.xfer_flag, &flag_status);
  if (!(flag_status.flag_pattern & EVENT_SV_TIMER_EXP))
    {
      altmdm_sys_setflag(&priv->spidev.xfer_flag, EVENT_SV_TIMER_EXP);
    }
}

/****************************************************************************
 * Name: init_svtimer
 *
 * Description:
 *   Initialize supervisory timer.
 *
 ****************************************************************************/

static int init_svtimer(FAR struct altmdm_dev_s *priv)
{
  priv->spidev.sleep_param.sv_timerid = NULL;

  return 0;
}

/****************************************************************************
 * Name: delete_svtimer
 *
 * Description:
 *   Delete supervisory timer.
 *
 ****************************************************************************/

static int delete_svtimer(FAR struct altmdm_dev_s *priv)
{
  altmdm_sys_stoptimer(priv->spidev.sleep_param.sv_timerid);
  priv->spidev.sleep_param.sv_timerid = NULL;

  return 0;
}

/****************************************************************************
 * Name: start_svtimer
 *
 * Description:
 *   Start supervisory timer.
 *
 ****************************************************************************/

static int start_svtimer(FAR struct altmdm_dev_s *priv)
{
  timer_t timerid;

  timerid = altmdm_sys_starttimer(SV_TIMER_TIMOUT_VAL, SV_TIMER_TIMOUT_VAL,
                                  svtimer_handler, 0, priv);
  if (timerid == NULL)
    {
      return -1;
    }

  priv->spidev.sleep_param.sv_timerid = timerid;

  return 0;
}

/****************************************************************************
 * Name: stop_svtimer
 *
 * Description:
 *   Stop supervisory timer.
 *
 ****************************************************************************/

static int stop_svtimer(FAR struct altmdm_dev_s *priv)
{
  altmdm_sys_stoptimer(priv->spidev.sleep_param.sv_timerid);
  priv->spidev.sleep_param.sv_timerid = NULL;

  return 0;
}

/****************************************************************************
 * Name: do_dmaxfer
 *
 * Description:
 *   Execute DMA transfer.
 *
 ****************************************************************************/

static int do_dmaxfer(FAR struct altmdm_dev_s *priv, FAR void *tx_buffer,
                      FAR void *rx_buffer, size_t dma_size)
{
  SPI_EXCHANGE(priv->spi, tx_buffer, rx_buffer, dma_size);

  return 0;
}

/****************************************************************************
 * Name: get_dmasize
 *
 * Description:
 *   Get the data size used for DMA transfer.
 *
 ****************************************************************************/

static int get_dmasize(FAR struct altmdm_dev_s *priv, int data_size)
{
  return data_size;
}

/****************************************************************************
 * Name: wait_receiverready
 *
 * Description:
 *   Wait until receiver is ready.
 *
 ****************************************************************************/

static int wait_receiverready(FAR struct altmdm_dev_s *priv,
                              uint32_t timeout_ms)
{
  int ret;
  uint32_t ptn;
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;

  ret = altmdm_sys_waitflag(&spidev->xfer_flag, EVENT_RXREQ,
                            ALTMDM_SYS_FLAG_WMODEOR, &ptn,
                            timeout_ms);
  if (ret != 0)
    {
      m_err("receiver ready timeout.\n");
      return TRANS_WAITRCVRTMO;
    }

  return 0;
}

/****************************************************************************
 * Name: wait_xferready
 *
 * Description:
 *   Wait until xfer is ready.
 *
 ****************************************************************************/

static int wait_xferready(FAR struct altmdm_dev_s *priv)
{
  int ret;
  uint32_t ptn;
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;

  ret = altmdm_sys_waitflag(&spidev->xferready_flag, EVENT_XFERRDY,
                            ALTMDM_SYS_FLAG_WMODEOR, &ptn,
                            WAIT_XFERRDY_TIMEOUT);
  if (ret != 0)
    {
      m_err("xfer ready timeout.\n");

      /* Make it ready to transfer.
       * It is assumed that modem does not implement reset packet.
       */

      if (!spidev->is_xferready)
        {
          spidev->is_xferready = true;
          m_info("ready to xfer\n");
        }

      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: notify_xferready
 *
 * Description:
 *   Notify xfer is ready.
 *
 ****************************************************************************/

static int notify_xferready(FAR struct altmdm_dev_s *priv)
{
  int ret;
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;

  ret = altmdm_sys_setflag(&spidev->xferready_flag, EVENT_XFERRDY);

  return ret;
}

/****************************************************************************
 * Name: init_rxbuffer
 *
 * Description:
 *   Initialize the receive buffer used for data transfer.
 *
 ****************************************************************************/

static void init_rxbuffer(FAR struct altmdm_dev_s *priv,
                          FAR struct altmdm_spi_rxbuff_s **rxbuff,
                          uint32_t unit_size)
{
  FAR struct altmdm_spi_rxbuff_s *l_buff = NULL;

  l_buff = (FAR struct altmdm_spi_rxbuff_s *)kmm_malloc
    (sizeof(struct altmdm_spi_rxbuff_s));
  if (l_buff == NULL)
    {
      m_err("cannot allocate memory for received buffer\n");
    }
  else
    {
      memset(l_buff, 0x00, sizeof(struct altmdm_spi_rxbuff_s));

      l_buff->buff_addr = (char *)kmm_malloc(unit_size);
      if (l_buff->buff_addr == NULL)
        {
          m_err("cannot allocate memory for received buffer\n");
          kmm_free(l_buff);
          l_buff = NULL;
        }
      else
        {
          l_buff->buff_size = unit_size;
        }
    }

  *rxbuff = l_buff;
}

/****************************************************************************
 * Name: uninit_rxbuffer
 *
 * Description:
 *   Uninitialize the receive buffer used for data transfer.
 *
 ****************************************************************************/

static void uninit_rxbuffer(FAR struct altmdm_dev_s *priv,
                            FAR struct altmdm_spi_rxbuff_s *rxbuff)
{
  if (rxbuff != NULL)
    {
      if (rxbuff->buff_addr != NULL)
        {
          kmm_free(rxbuff->buff_addr);
        }

      kmm_free(rxbuff);
    }
}

/****************************************************************************
 * Name: alloc_rxbuffer
 *
 * Description:
 *   Get the allocated receive buffer.
 *
 ****************************************************************************/

static void alloc_rxbuffer(FAR struct altmdm_dev_s *priv,
                           FAR struct altmdm_spi_rxbuff_s **rxbuff,
                           uint32_t size)
{
  if (priv->spidev.rxbuffinfo.free_buff == NULL)
    {
      m_err("cannot allocate free rx buffer\n");
    }
  else
    {
      *rxbuff = priv->spidev.rxbuffinfo.free_buff;
      priv->spidev.rxbuffinfo.free_buff = NULL;
    }
}

/****************************************************************************
 * Name: free_rxbuffer
 *
 * Description:
 *   Release allocated receive buffer.
 *
 ****************************************************************************/

static void free_rxbuffer(FAR struct altmdm_dev_s *priv,
                          FAR struct altmdm_spi_rxbuff_s *rxbuff)
{
  if (priv->spidev.rxbuffinfo.free_buff != NULL)
    {
      m_err("cannot free rx buffer\n");
    }
  else
    {
      priv->spidev.rxbuffinfo.free_buff = rxbuff;
    }
}

/****************************************************************************
 * Name: create_rxbufffifo
 *
 * Description:
 *   Create receiving FIFO. This fifo is used when copying to the buffer
 *   specified by the user.
 *
 ****************************************************************************/

static void create_rxbufffifo(FAR struct altmdm_dev_s *priv)
{
  FAR struct altmdm_spi_rxbufffifo_s *fifo = &priv->spidev.rxbuffinfo.fifo;

  init_rxbuffer(priv, &priv->spidev.rxbuffinfo.free_buff, MAX_PKT_SIZE);

  fifo->head = NULL;
  fifo->tail = NULL;

  altmdm_sys_initcsem(&fifo->csem);
}

/****************************************************************************
 * Name: destroy_rxbufffifo
 *
 * Description:
 *   Destroy receiving FIFO.
 *
 ****************************************************************************/

static void destroy_rxbufffifo(FAR struct altmdm_dev_s *priv)
{
  FAR struct altmdm_spi_rxbufffifo_s *fifo = &priv->spidev.rxbuffinfo.fifo;

  uninit_rxbuffer(priv, priv->spidev.rxbuffinfo.free_buff);

  altmdm_sys_deletecsem(&fifo->csem);
}

/****************************************************************************
 * Name: put_rxbufffifo
 *
 * Description:
 *   Put date into the receiving FIFO.
 *
 ****************************************************************************/

static void put_rxbufffifo(FAR struct altmdm_dev_s *priv,
                           FAR struct altmdm_spi_rxbuff_s *rxbuff)
{
  FAR struct altmdm_spi_rxbufffifo_s *fifo = &priv->spidev.rxbuffinfo.fifo;
  irqstate_t flags;

  flags = enter_critical_section();

  if (fifo->head == NULL)
    {
      fifo->head = rxbuff;
      fifo->tail = rxbuff;
      rxbuff->next = NULL;
    }
  else
    {
      fifo->tail->next = rxbuff;
      fifo->tail = rxbuff;
      rxbuff->next = NULL;
    }

  leave_critical_section(flags);

  altmdm_sys_postcsem(&fifo->csem);
}

/****************************************************************************
 * Name: get_rxbufffifo
 *
 * Description:
 *   Get date from the receiving FIFO. If the FIFO is empty,
 *   it is kept waiting until data is put in the FIFO.
 *
 ****************************************************************************/

static void get_rxbufffifo(FAR struct altmdm_dev_s *priv,
                           FAR struct altmdm_spi_rxbuff_s **rxbuff)
{
  FAR struct altmdm_spi_rxbufffifo_s *fifo = &priv->spidev.rxbuffinfo.fifo;
  irqstate_t flags;

  *rxbuff = NULL;

  altmdm_sys_waitcsem(&fifo->csem);

  flags = enter_critical_section();

  if (fifo->head != NULL)
    {
      *rxbuff = fifo->head;
      fifo->head = fifo->head->next;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: abort_get_rxbufffifo
 *
 * Description:
 *   If waiting for data to be put in the FIFO, then abort it.
 *
 ****************************************************************************/

static void abort_get_rxbufffifo(FAR struct altmdm_dev_s *priv)
{
  int val;
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;
  FAR struct altmdm_spi_rxbufffifo_s *fifo = &priv->spidev.rxbuffinfo.fifo;

  if (!altmdm_sys_getcsemvalue(&fifo->csem, &val))
    {
      if (val < 0)
        {
          spidev->rx_param.rxabort = true;
          altmdm_sys_postcsem(&fifo->csem);
        }
    }
}

/****************************************************************************
 * Name: clear_txheader
 *
 * Description:
 *   Clear the header for transmission.
 *
 ****************************************************************************/

static void clear_txheader(FAR struct altmdm_dev_s *priv)
{
  memset(&priv->spidev.tx_param.header, 0x00,
         sizeof(struct altmdm_spi_xferhdr_s));
}

/****************************************************************************
 * Name: clear_rxheader
 *
 * Description:
 *   Clear the header for reception.
 *
 ****************************************************************************/

static void clear_rxheader(FAR struct altmdm_dev_s *priv)
{
  memset(&priv->spidev.rx_param.header, 0x00,
         sizeof(struct altmdm_spi_xferhdr_s));
}

/****************************************************************************
 * Name: set_txheader_datasize
 *
 * Description:
 *   Set transmission data size in the header.
 *
 ****************************************************************************/

static void set_txheader_datasize(FAR struct altmdm_dev_s *priv,
                                  int total_size, int actual_size)
{
  FAR struct altmdm_spi_xferhdr_s *tx_header = &priv->spidev.tx_param.header;

  tx_header->header[0] =
    (tx_header->header[0] & 0xf0) + ((total_size >> 10) & 0x0000000f);
  tx_header->header[1] = (total_size >> 2) & 0x000000ff;
  tx_header->header[2] =
    ((total_size & 0x00000003) << 6) + ((actual_size >> 8) & 0x0000003f);
  tx_header->header[3] = (actual_size & 0x000000ff);
}

/****************************************************************************
 * Name: set_txheader_possibleofrx
 *
 * Description:
 *   Set flag in the header. The flag indicates that master can receive data.
 *
 ****************************************************************************/

static void set_txheader_possibleofrx(FAR struct altmdm_dev_s *priv,
                                      bool possible)
{
  FAR struct altmdm_spi_xferhdr_s *tx_header = &priv->spidev.tx_param.header;

  if (!possible)
    {
      tx_header->header[0] = 0x80 + (tx_header->header[0] & 0x7f);
    }
  else
    {
      tx_header->header[0] = 0x00 + (tx_header->header[0] & 0x7f);
    }
}

/****************************************************************************
 * Name: set_txheader_sleepreq
 *
 * Description:
 *   Set flag in the header. The flag indicates that sleep request.
 *
 ****************************************************************************/

static void set_txheader_sleepreq(FAR struct altmdm_dev_s *priv)
{
  FAR struct altmdm_spi_xferhdr_s *tx_header = &priv->spidev.tx_param.header;

  tx_header->header[0] = 0x10 + (tx_header->header[0] & 0xef);
}

/****************************************************************************
 * Name: show_txheader
 *
 * Description:
 *   Show header for transmission.
 *
 ****************************************************************************/

static void show_txheader(FAR struct altmdm_dev_s *priv)
{
#ifdef MODEM_ALTMDM_DEBUG
  FAR struct altmdm_spi_xferhdr_s *tx_header = &priv->spidev.tx_param.header;

  m_info("[TXHDR]=%02x,%02x,%02x,%02x\n",
         tx_header->header[0], tx_header->header[1],
         tx_header->header[2], tx_header->header[3]);
#endif
}

/****************************************************************************
 * Name: parse_rxheader
 *
 * Description:
 *   Parse header for receiving.
 *
 ****************************************************************************/

static void parse_rxheader(FAR struct altmdm_dev_s *priv,
                           FAR int *total_size, FAR int *actual_size,
                           FAR int *is_reset, FAR int *is_bufful)
{
  FAR struct altmdm_spi_xferhdr_s *rx_header = &priv->spidev.rx_param.header;

  m_info("[RXHDR]%02x,%02x,%02x,%02x\n",
         rx_header->header[0], rx_header->header[1],
         rx_header->header[2], rx_header->header[3]);

  *total_size = ((rx_header->header[0] & 0x0f) << 10) +
    ((rx_header->header[1] & 0xff) << 2) + (rx_header->header[2] >> 6);
  *actual_size = ((rx_header->header[2] & 0x3f) << 8) +
    (rx_header->header[3] & 0xff);

  priv->spidev.rx_param.status_info = (rx_header->header[0] & 0xf0) >> 4;

  *is_reset = STAT_INF_IS_RESET(priv->spidev.rx_param.status_info);
  *is_bufful = STAT_INF_IS_BUFF_FULL(priv->spidev.rx_param.status_info);

  m_info("t=%d a=%d r=%d b=%d\n",
         *total_size, *actual_size, *is_reset, *is_bufful);

  return;
}

/****************************************************************************
 * Name: verify_rxheader
 *
 * Description:
 *   Verify header for receiving.
 *
 ****************************************************************************/

static int verify_rxheader(FAR struct altmdm_dev_s *priv,
                           int total_size, int actual_size)
{
  int calc_total_size;

  if ((total_size == 0) || (actual_size == 0))
    {
      return -1;
    }

  if ((total_size > MAX_PKT_SIZE) || (actual_size > MAX_PKT_SIZE))
    {
      return -1;
    }

  if (total_size % UNIT_SIZE)
    {
      return -1;
    }

  if (total_size != actual_size)
    {
      calc_total_size = ((actual_size / UNIT_SIZE) + 1) * UNIT_SIZE;

      if (total_size != calc_total_size)
        {
          return -1;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: do_xferheader
 *
 * Description:
 *   Execute header transfer.
 *
 ****************************************************************************/

static int do_xferheader(FAR struct altmdm_dev_s *priv,
                         uint32_t is_rxreq, uint32_t is_txreq,
                         uint32_t is_sleepreq, uint32_t is_rcvrready)
{
  int ret;
  int dma_xfer_size;
  struct altmdm_spi_dev_s *spidev = &priv->spidev;
  bool possibleofrx = true;

  /* Make transfer header */

  clear_txheader(priv);

  if ((is_sleepreq) || (is_txreq))
    {
      DEBUGASSERT(priv->lower);
      priv->lower->master_request(true);

      if (is_sleepreq)
        {
          set_txheader_sleepreq(priv);
        }
      else
        {
          set_txheader_datasize(priv, spidev->tx_param.total_size,
                                spidev->tx_param.actual_size);
        }
    }

  if (!is_sleepreq)
    {
      alloc_rxbuffer(priv, &spidev->rx_param.rxbuff, MAX_PKT_SIZE);
      if (spidev->rx_param.rxbuff == NULL)
        {
          possibleofrx = false;
        }

      set_txheader_possibleofrx(priv, possibleofrx);
    }

  show_txheader(priv);

  /* Wait for Receiver Ready to receive. */

  if ((!is_rxreq) && (!is_rcvrready))
    {
      ret = wait_receiverready(priv, WAIT_RXREQ_HEADER_TIMEOUT);
      if (ret < 0)
        {
          goto trans_header_error;
        }
    }

  /* Get DMA transfer size */

  dma_xfer_size = get_dmasize(priv, sizeof(struct altmdm_spi_xferhdr_s));

  ret = do_dmaxfer(priv, (FAR void *)&spidev->tx_param.header,
                   (FAR void *)&spidev->rx_param.header, dma_xfer_size);
  if (ret < 0)
    {
      goto trans_header_error;
    }

  return ret;

trans_header_error:
  m_err("ERR:%04d Transfer Header Failed. ret = %d.\n", __LINE__, ret);

  /* Clear Header */

  clear_rxheader(priv);
  set_txheader_datasize(priv, 0, 0);

  return ret;
}

/****************************************************************************
 * Name: do_receivedata
 *
 * Description:
 *   Executes the receive only transfer mode.
 *
 ****************************************************************************/

static int do_receivedata(FAR struct altmdm_dev_s *priv)
{
  int ret;
  int dma_xfer_size;
  FAR void *rxbuff;
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;

  /* Wait for Receiver Ready to receive. */

  ret = wait_receiverready(priv, WAIT_RXREQ_TIMEOUT);
  if (ret >= 0)
    {
      /* Get DMA transfer size */

      dma_xfer_size = get_dmasize(priv, spidev->rx_param.total_size);

      rxbuff = spidev->rx_param.rxbuff->buff_addr;

      /* Do DMA transfer */

      ret = do_dmaxfer(priv, NULL, rxbuff, dma_xfer_size);
    }

  if (ret < 0)
    {
      m_err("Rcv Data Failed. ret = %d.\n", ret);
      clear_rxheader(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: do_senddata
 *
 * Description:
 *   Executes the transmission only transfer mode.
 *
 ****************************************************************************/

static int do_senddata(FAR struct altmdm_dev_s *priv)
{
  int ret;
  int dma_xfer_size;
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;

  /* Wait for Receiver Ready to receive. */

  ret = wait_receiverready(priv, WAIT_RXREQ_TIMEOUT);
  if (ret >= 0)
    {
      /* Get DMA transfer size */

      dma_xfer_size = get_dmasize(priv, spidev->tx_param.total_size);

      /* Do DMA transfer */

      ret = do_dmaxfer(priv, (FAR void *)spidev->tx_param.buff_addr,
                       NULL, dma_xfer_size);
    }

  if (ret < 0)
    {
      m_err("ERR:%04d Snd Data Failed. ret = %d.\n", __LINE__, ret);
      clear_txheader(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: do_trxdata
 *
 * Description:
 *   Executes the transmission and receive transfer mode.
 *
 ****************************************************************************/

static int do_trxdata(FAR struct altmdm_dev_s *priv)
{
  int ret;
  int xfer_size;
  int dma_xfer_size;
  FAR void *rxbuff;
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;

  /* Wait for Receiver Ready to receive. */

  ret = wait_receiverready(priv, WAIT_RXREQ_TIMEOUT);
  if (ret >= 0)
    {
      /* Choose the larger one. */

      if (spidev->tx_param.total_size < spidev->rx_param.total_size)
        {
          xfer_size = spidev->rx_param.total_size;
        }
      else
        {
          xfer_size = spidev->tx_param.total_size;
        }

      rxbuff = spidev->rx_param.rxbuff->buff_addr;

      /* Get DMA transfer size */

      dma_xfer_size = get_dmasize(priv, xfer_size);

      /* Do DMA transfer */

      ret = do_dmaxfer(priv, (FAR void *)spidev->tx_param.buff_addr, rxbuff,
                       dma_xfer_size);
    }

  if (ret < 0)
    {
      m_err("ERR:%04d Trx Data Failed. ret = %d.\n", __LINE__, ret);
      clear_txheader(priv);
      clear_rxheader(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: do_receivedata_nobuff
 *
 * Description:
 *   Executes the receive only transfer mode. When receiving buffer cannot
 *   be prepared.
 *
 ****************************************************************************/

static int do_receivedata_nobuff(FAR struct altmdm_dev_s *priv)
{
  int ret;
  int dma_xfer_size;
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;

  /* Wait for Receiver Ready to receive. */

  ret = wait_receiverready(priv, WAIT_RXREQ_TIMEOUT);
  if (ret >= 0)
    {
      /* Get DMA transfer size */

      dma_xfer_size = get_dmasize(priv, spidev->rx_param.total_size);

      /* Do DMA transfer */

      ret = do_dmaxfer(priv, NULL, NULL, dma_xfer_size);
    }

  if (ret < 0)
    {
      m_err("ERR:%04d Rcv Data Nobuff Failed. ret = %d.\n", __LINE__, ret);
      clear_rxheader(priv);
    }
  else
    {
      ret = TRANS_OK_RXDATANOBUFF;
    }

  return ret;
}

/****************************************************************************
 * Name: do_trxdata_norxbuff
 *
 * Description:
 *   Executes the transmission and receive transfer mode. When receiving
 *   buffer cannot be prepared.
 *
 ****************************************************************************/

static int do_trxdata_norxbuff(FAR struct altmdm_dev_s *priv)
{
  int ret;
  int xfer_size;
  int dma_xfer_size;
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;

  /* Wait for Receiver Ready to receive. */

  ret = wait_receiverready(priv, WAIT_RXREQ_TIMEOUT);
  if (ret >= 0)
    {
      /* Choose the larger one. */

      if (spidev->tx_param.total_size < spidev->rx_param.total_size)
        {
          xfer_size = spidev->rx_param.total_size;
        }
      else
        {
          xfer_size = spidev->tx_param.total_size;
        }

      /* Get DMA transfer size */

      dma_xfer_size = get_dmasize(priv, xfer_size);

      /* Do DMA transfer */

      ret = do_dmaxfer(priv, (FAR void *)spidev->tx_param.buff_addr,
                       NULL, dma_xfer_size);
    }

  if (ret < 0)
    {
      m_err("ERR:%04d Trx Data Norxbuff Failed. ret = %d.\n", __LINE__, ret);
      clear_txheader(priv);
      clear_rxheader(priv);
    }
  else
    {
      ret = TRANS_OK_TRXDATANORXBUFF;
    }

  return ret;
}

/****************************************************************************
 * Name: do_receivesleepdata
 *
 * Description:
 *   Executes the sleep data transfer mode.
 *
 ****************************************************************************/

static int do_receivesleepdata(FAR struct altmdm_dev_s *priv, FAR int *resp)
{
  int ret;
  int dma_xfer_size;
  char rxbuff[UNIT_SIZE];

  /* Wait for Receiver Ready to receive. */

  ret = wait_receiverready(priv, WAIT_RXREQ_TIMEOUT);
  if (ret >= 0)
    {
      /* Get DMA transfer size */

      dma_xfer_size = get_dmasize(priv, UNIT_SIZE);

      /* Do DMA transfer */

      ret = do_dmaxfer(priv, NULL, (FAR void *)rxbuff, dma_xfer_size);
    }

  if (ret < 0)
    {
      m_err("ERR:%04d Rcv Sleep Resp Data Failed. ret = %d.\n",
             __LINE__, ret);
    }
  else
    {
      m_info("[SRESP] 0x%02x,0x%02x,0x%02x,0x%02x\n",
             rxbuff[0], rxbuff[1],
             rxbuff[2], rxbuff[3]);

      if (!memcmp(rxbuff, "OK", 2))
        {
          *resp = SLEEP_OK;
        }
      else
        {
          *resp = SLEEP_NG;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: do_receivereset
 *
 * Description:
 *   Executes the reset packet receive only transfer mode.
 *
 ****************************************************************************/

static int do_receivereset(FAR struct altmdm_dev_s *priv)
{
  int ret;
  int dma_xfer_size;
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;
  char rxbuff[UNIT_SIZE];

  /* Wait for Receiver Ready to receive. */

  ret = wait_receiverready(priv, WAIT_RXREQ_TIMEOUT);
  if (ret >= 0)
    {
      /* Get DMA transfer size */

      dma_xfer_size = get_dmasize(priv, spidev->rx_param.total_size);

      /* Do DMA transfer */

      ret = do_dmaxfer(priv, NULL, (FAR void *)rxbuff, dma_xfer_size);
    }

  if (ret < 0)
    {
      m_err("ERR:%04d Rcv Reset Failed. ret = %d.\n", __LINE__, ret);
      clear_rxheader(priv);
    }
  else
    {
      if ((STAT_INF_GET_BOOTSTAT | STAT_INF_RESET) ==
          (spidev->rx_param.
           status_info & (STAT_INF_GET_BOOTSTAT | STAT_INF_RESET)))
        {
          switch (rxbuff[0])
            {
            case RESET_BOOTSTAT_BOOTING:
              altmdm_pm_set_bootstatus(priv,
                                       MODEM_PM_ERR_RESET_BOOTSTAT_BOOTING);
              break;

            case RESET_BOOTSTAT_UPDATING:
              altmdm_pm_set_bootstatus(priv,
                                       MODEM_PM_ERR_RESET_BOOTSTAT_UPDATING);
              break;

            default:
              m_err
                ("ERR:%04d Invalid payload of reset packet. " \
                 "%02x,%02x,%02x,%02x\n",
                 __LINE__, rxbuff[0], rxbuff[1], rxbuff[2], rxbuff[3]);
              break;
            }
        }
      else if (STAT_INF_RESET ==
               (spidev->rx_param.status_info & STAT_INF_RESET))
        {
          altmdm_pm_set_bootstatus(priv, MODEM_PM_ERR_RESET_BOOTSTAT_DONE);
        }
      else
        {
          m_err("ERR:%04d Unexpected status info. %04x\n", __LINE__,
                spidev->rx_param.status_info);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: do_trxreset
 *
 * Description:
 *   Executes the transmission and receive reset packet transfer mode.
 *
 ****************************************************************************/

static int do_trxreset(FAR struct altmdm_dev_s *priv)
{
  int ret;
  int xfer_size;
  int dma_xfer_size;
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;

  /* Wait for Receiver Ready to receive. */

  ret = wait_receiverready(priv, WAIT_RXREQ_TIMEOUT);
  if (ret >= 0)
    {
      /* If a conflict occurs with the reset packet,
       * the packet is transferred by the size specified
       * by the receiving side. Discard the data on the sending side.
       */

      xfer_size = spidev->rx_param.total_size;

      /* Get DMA transfer size */

      dma_xfer_size = get_dmasize(priv, xfer_size);

      /* Do DMA transfer */

      ret = do_dmaxfer(priv, NULL, NULL, dma_xfer_size);
    }

  if (ret < 0)
    {
      m_err("ERR:%04d Trx Reset Failed. ret = %d.\n", __LINE__, ret);
      clear_txheader(priv);
      clear_rxheader(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: do_xfersleep
 *
 * Description:
 *   Executes the sleep request.
 *
 ****************************************************************************/

static int do_xfersleep(FAR struct altmdm_dev_s *priv, uint32_t is_rcvrready)
{
  int ret;
  int resp = 0;
  int is_reset = 0;
  int total_size;
  int actual_size;
  int is_bufful;

  /* Transfer header for sleep request */

  ret = do_xferheader(priv, 0, 0, 1, is_rcvrready);
  if (ret >= 0)
    {
      parse_rxheader(priv, &total_size, &actual_size, &is_reset, &is_bufful);

      /* Transfer data for sleep request */

      ret = do_receivesleepdata(priv, &resp);
      if (ret >= 0)
        {
          ret = resp;
        }
    }

  if (ret < 0)
    {
      ret = SLEEP_NG;
    }

  DEBUGASSERT(priv && priv->lower);
  priv->lower->master_request(false);

  if (is_reset)
    {
      if (!priv->spidev.is_xferready)
        {
          priv->spidev.is_xferready = true;
          m_info("ready to xfer\n");
          notify_xferready(priv);
        }

      altmdm_pm_notify_reset(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: decide_xfermode
 *
 * Description:
 *   Decide transfer mode.
 *
 ****************************************************************************/

static uint32_t decide_xfermode(FAR struct altmdm_dev_s *priv,
                                uint32_t is_rxreq, uint32_t is_txreq,
                                int ret)
{
  int retval;
  int is_reset;
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;
  uint32_t mode = MODE_RXINVALID;

  if (ret < 0)
    {
      if (is_txreq)
        {
          mode = MODE_TRXHEADERFAILTXREQ;
        }
      else
        {
          mode = MODE_TRXHEADERFAILRXREQ;
        }

      if (spidev->rx_param.rxbuff != NULL)
        {
          free_rxbuffer(priv, spidev->rx_param.rxbuff);
          spidev->rx_param.rxbuff = NULL;
        }
    }
  else
    {
      parse_rxheader(priv, &spidev->rx_param.total_size,
                     &spidev->rx_param.actual_size, &is_reset,
                     &spidev->tx_param.is_bufful);

      if (is_rxreq)
        {
          retval = verify_rxheader(priv, spidev->rx_param.total_size,
                                   spidev->rx_param.actual_size);
          if (retval != 0)
            {
              m_info("RX header:total=0x%02x, actual=0x%02x.\n",
                     spidev->rx_param.total_size,
                     spidev->rx_param.actual_size);

              if (spidev->rx_param.rxbuff != NULL)
                {
                  free_rxbuffer(priv, spidev->rx_param.rxbuff);
                  spidev->rx_param.rxbuff = NULL;
                }

              if (!is_txreq)
                {
                  return mode;
                }

              is_rxreq = 0;

              m_info("RX Header invalid. But Send will be done.\n");
            }
        }
      else if ((spidev->rx_param.actual_size != 0) ||
               (spidev->rx_param.total_size != 0))
        {
          retval = verify_rxheader(priv, spidev->rx_param.total_size,
                                   spidev->rx_param.actual_size);
          if (retval == 0)
            {
              is_rxreq = 1;
            }
        }

      /* Diceide transfer mode here. */

      if (is_txreq)
        {
          if (is_rxreq)
            {
              if (is_reset)
                {
                  if (spidev->rx_param.rxbuff != NULL)
                    {
                      free_rxbuffer(priv, spidev->rx_param.rxbuff);
                      spidev->rx_param.rxbuff = NULL;
                    }

                  mode = MODE_TRXRESET;
                }
              else
                {
                  if (spidev->rx_param.rxbuff == NULL)
                    {
                      mode = MODE_TRXDATANOBUFF;
                    }
                  else
                    {
                      mode = MODE_TRXDATA;
                      spidev->rx_param.rxbuff->rx_size =
                        spidev->rx_param.actual_size;
                      m_info("received size = %d.\n",
                             spidev->rx_param.rxbuff->rx_size);
                    }
                }
            }
          else
            {
              if (spidev->rx_param.rxbuff != NULL)
                {
                  free_rxbuffer(priv, spidev->rx_param.rxbuff);
                  spidev->rx_param.rxbuff = NULL;
                }

              mode = MODE_TXDATA;
            }
        }
      else
        {
          if (is_reset)
            {
              if (spidev->rx_param.rxbuff != NULL)
                {
                  free_rxbuffer(priv, spidev->rx_param.rxbuff);
                  spidev->rx_param.rxbuff = NULL;
                }

              mode = MODE_RXRESET;
            }
          else
            {
              if (spidev->rx_param.rxbuff == NULL)
                {
                  mode = MODE_RXDATANOBUFF;
                }
              else
                {
                  mode = MODE_RXDATA;
                  spidev->rx_param.rxbuff->rx_size =
                    spidev->rx_param.actual_size;
                  m_info("received size = %d.\n",
                         spidev->rx_param.rxbuff->rx_size);
                }
            }
        }
    }

  clear_rxheader(priv);
  clear_txheader(priv);

  return mode;
}

/****************************************************************************
 * Name: done_xfer
 *
 * Description:
 *   Notify that transfer has completed.
 *
 ****************************************************************************/

static void done_xfer(FAR struct altmdm_dev_s *priv, uint32_t xfer_mode,
                      int ret)
{
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;

  switch (xfer_mode)
    {
    case MODE_RXDATA:
      if (ret < 0)
        {
          free_rxbuffer(priv, spidev->rx_param.rxbuff);
          spidev->rx_param.rxbuff = NULL;
        }
      else
        {
          put_rxbufffifo(priv, spidev->rx_param.rxbuff);
          spidev->rx_param.rxbuff = NULL;
        }
      break;

    case MODE_TXDATA:
      if (spidev->tx_param.is_bufful)
        {
          spidev->tx_param.is_bufful = 0;
          spidev->tx_param.result = (ret < 0) ? ret : TRANS_OK_RCVBUFFUL;
        }
      else
        {
          spidev->tx_param.result = (ret < 0) ? ret : TRANS_OK;
        }

      altmdm_sys_setflag(&spidev->tx_param.done_flag, EVENT_TX_DONE);
      break;

    case MODE_TRXDATA:
      if (spidev->tx_param.is_bufful)
        {
          spidev->tx_param.is_bufful = 0;
          spidev->tx_param.result = (ret < 0) ? ret : TRANS_OK_RCVBUFFUL;
        }
      else
        {
          spidev->tx_param.result = (ret < 0) ? ret : TRANS_OK;
        }

      altmdm_sys_setflag(&spidev->tx_param.done_flag, EVENT_TX_DONE);

      if (ret < 0)
        {
          free_rxbuffer(priv, spidev->rx_param.rxbuff);
          spidev->rx_param.rxbuff = NULL;
        }
      else
        {
          put_rxbufffifo(priv, spidev->rx_param.rxbuff);
          spidev->rx_param.rxbuff = NULL;
        }
      break;

    case MODE_TRXDATANOBUFF:
      if (spidev->tx_param.is_bufful)
        {
          spidev->tx_param.is_bufful = 0;
          spidev->tx_param.result = (ret < 0) ? ret : TRANS_OK_RCVBUFFUL;
        }
      else
        {
          spidev->tx_param.result = (ret < 0) ? ret : TRANS_OK;
        }

      altmdm_sys_setflag(&spidev->tx_param.done_flag, EVENT_TX_DONE);

      /* Yield the CPU so that the upper layer can execute
       * the receiving process.
       */

      nxsig_usleep(YIELD_TASK_NOBUFF);

      break;

    case MODE_TRXHEADERFAILTXREQ:
      spidev->tx_param.result = ret;
      altmdm_sys_setflag(&spidev->tx_param.done_flag, EVENT_TX_DONE);
      break;

    case MODE_RXRESET:
      if (!spidev->is_xferready)
        {
          spidev->is_xferready = true;
          m_info("ready to xfer\n");
          notify_xferready(priv);
        }

      altmdm_pm_notify_reset(priv);
      break;

    case MODE_TRXRESET:
      if (!spidev->is_xferready)
        {
          spidev->is_xferready = true;
          m_info("ready to xfer\n");
          notify_xferready(priv);
        }

      altmdm_pm_notify_reset(priv);
      if (spidev->tx_param.is_bufful)
        {
          spidev->tx_param.is_bufful = 0;
          spidev->tx_param.result = (ret < 0) ? ret : TRANS_OK_RCVBUFFUL;
        }
      else
        {
          spidev->tx_param.result = (ret < 0) ? ret : TRANS_OK;
        }

      altmdm_sys_setflag(&spidev->tx_param.done_flag, EVENT_TX_DONE);
      break;

    case MODE_RXDATANOBUFF:

      /* Yield the CPU so that the upper layer can execute
       * the receiving process.
       */

      nxsig_usleep(YIELD_TASK_NOBUFF);
      break;

    case MODE_TRXHEADERFAILRXREQ:
    case MODE_RXINVALID:
      break;

    default:
      break;
    }
}

/****************************************************************************
 * Name: done_sleep
 *
 * Description:
 *   Notify that sleep request has completed.
 *
 ****************************************************************************/

static void done_sleep(FAR struct altmdm_dev_s *priv, int ret)
{
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;

  spidev->sleep_param.result = ret;
  altmdm_sys_setflag(&spidev->sleep_param.done_flag, EVENT_SLEEP_DONE);
}

/****************************************************************************
 * Name: xfer_task_init
 *
 * Description:
 *   Initialize SPI transfer task.
 *
 ****************************************************************************/

static void xfer_task_init(FAR struct altmdm_dev_s *priv)
{
  sigset_t mask;

  sigfillset(&mask);
  sigprocmask(SIG_SETMASK, &mask, NULL);

  init_svtimer(priv);
}

/****************************************************************************
 * Name: xfer_task
 *
 * Description:
 *   ALTMDM SPI transfer task.
 *
 ****************************************************************************/

static int xfer_task(int argc, char *argv[])
{
  int ret;
  int sleep_result;
  int res_code;
  uint32_t is_txreq;
  uint32_t is_rxreq;
  uint32_t is_sleepreq;
  uint32_t is_timerexp;
  uint32_t do_sleep;
  uint32_t ptn;
  uint32_t xfer_mode;
  uint32_t modem_state;
  uint32_t is_rcvrready = 0;
  FAR struct altmdm_dev_s *priv = g_privdata;
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;

  xfer_task_init(priv);

  while (!spidev->is_not_run)
    {
      ret = altmdm_sys_waitflag(&spidev->xfer_flag, EVENT_TRANS_WAIT,
                                ALTMDM_SYS_FLAG_WMODEOR, &ptn,
                                ALTMDM_SYS_FLAG_TMOFEVR);
      if (ret != 0)
        {
          m_err("wait flag failed:%d.\n", ret);
          continue;
        }

      m_info("ptn:%x.\n", ptn);

      if (ptn & EVENT_REQMASK)
        {
          is_txreq = ptn & EVENT_TXREQ;
          is_rxreq = ptn & EVENT_RXREQ;
          is_sleepreq = ptn & EVENT_SLEEPREQ;
          is_timerexp = ptn & EVENT_SV_TIMER_EXP;
          do_sleep = 0;
          sleep_result = SLEEP_NG;

          /* Sleep transition event received. Check if it can sleep. */

          if (is_sleepreq || is_timerexp)
            {
              /* If data transfer required at the same time, cannot sleep. */

              if (!(is_txreq || is_rxreq))
                {
                  if (altmdm_pm_cansleep(priv))
                    {
                      do_sleep = 1;
                    }
                  else if (is_timerexp)
                    {
                      /* Case where modem spontaneously enters sleep state
                       * and timer is not stopped.
                       */

                      modem_state = altmdm_pm_getinternalstate();
                      if (modem_state == MODEM_PM_INTERNAL_STATE_SLEEP)
                        {
                          stop_svtimer(priv);
                        }
                    }
                }
            }

          if (do_sleep)
            {
              stop_svtimer(priv);

              /* Transfer sleep packet */

              sleep_result = do_xfersleep(priv, 0);
              if (sleep_result == SLEEP_OK)
                {
                  altmdm_pm_sleepmodem(priv);
                }
            }

          if (is_sleepreq)
            {
              /* Send sleep response */

              done_sleep(priv, sleep_result);
            }

          /* Receive data transfer request */

          if (is_txreq || is_rxreq)
            {
              stop_svtimer(priv);

              /* Wakeup modem before data transfer */

              if (is_rxreq)
                {
                  res_code = altmdm_pm_wakeup(priv, NULL);
                }
              else
                {
                  res_code = altmdm_pm_wakeup(priv, wait_receiverready);
                }

              if (res_code == MODEM_PM_WAKEUP_FAIL)
                {
                  done_xfer(priv, MODE_TXDATA, TRANS_WAITRCVRTMO);
                  start_svtimer(priv);
                  continue;
                }
              else if (res_code == MODEM_PM_WAKEUP_DONE)
                {
                  is_rcvrready = 1;
                }

              /* transfer header */

              ret = do_xferheader(priv, is_rxreq, is_txreq, 0, is_rcvrready);

              if (is_rcvrready)
                {
                  is_rcvrready = 0;
                }

              xfer_mode = decide_xfermode(priv, is_rxreq, is_txreq, ret);

              switch (xfer_mode)
                {
                case MODE_RXDATA:
                  ret = do_receivedata(priv);
                  break;

                case MODE_TXDATA:
                  ret = do_senddata(priv);
                  break;

                case MODE_TRXDATA:
                  ret = do_trxdata(priv);
                  break;

                case MODE_RXDATANOBUFF:
                  ret = do_receivedata_nobuff(priv);
                  break;

                case MODE_TRXDATANOBUFF:
                  ret = do_trxdata_norxbuff(priv);
                  break;

                case MODE_RXRESET:
                  ret = do_receivereset(priv);
                  break;

                case MODE_TRXRESET:
                  ret = do_trxreset(priv);
                  break;

                case MODE_RXINVALID:
                  ret = TRANS_RXINVALID;
                  break;

                case MODE_TRXHEADERFAILTXREQ:
                case MODE_TRXHEADERFAILRXREQ:
                  break;

                default:
                  m_err("ERR:%04d Unknown decision of transfer: %d.\n",
                        __LINE__, xfer_mode);
                  break;
                }

              if (is_txreq)
                {
                  DEBUGASSERT(priv->lower);
                  priv->lower->master_request(false);
                }

              m_info("m=%d\n", xfer_mode);
              done_xfer(priv, xfer_mode, ret);
              start_svtimer(priv);
            }
        }

      if (ptn & EVENT_EXIT)
        {
          spidev->is_not_run = true;
        }
    }

  delete_svtimer(priv);
  task_delete(0);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: altmdm_spi_gpioreadyisr
 *
 * Description:
 *   Interrupt handler for SLAVE_REQUEST GPIO line.
 *
 ****************************************************************************/

int altmdm_spi_gpioreadyisr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct altmdm_dev_s *priv = g_privdata;
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;

  altmdm_sys_setflag(&spidev->xfer_flag, EVENT_RXREQ);

  return 0;
}

/****************************************************************************
 * Name: altmdm_spi_init
 *
 * Description:
 *   Initialize ALTMDM driver.
 *
 ****************************************************************************/

int altmdm_spi_init(FAR struct altmdm_dev_s *priv)
{
  int ret = 0;

  g_privdata = priv;

  /* Initialize modem power management driver */

  altmdm_pm_init(priv);

  memset(&priv->spidev, 0, sizeof(struct altmdm_spi_dev_s));
  priv->spidev.is_not_run = false;
  priv->spidev.is_xferready = false;

  altmdm_sys_initlock(&priv->spidev.tx_param.lock);
  altmdm_sys_initlock(&priv->spidev.rx_param.lock);
  altmdm_sys_initlock(&priv->spidev.sleep_param.lock);

  altmdm_sys_initflag(&priv->spidev.xfer_flag);
  altmdm_sys_initflag(&priv->spidev.xferready_flag);
  altmdm_sys_initflag(&priv->spidev.tx_param.done_flag);
  altmdm_sys_initflag(&priv->spidev.sleep_param.done_flag);

  create_rxbufffifo(priv);

  DEBUGASSERT(priv->lower);

  /* SPI settings */

  SPI_LOCK(priv->spi, true);
  SPI_SETMODE(priv->spi, SPIDEV_MODE0);
  SPI_SETBITS(priv->spi, 8);
  SPI_SETFREQUENCY(priv->spi, priv->lower->spi_maxfreq());
  SPI_LOCK(priv->spi, false);

  priv->lower->sready_irqattach(true, altmdm_spi_gpioreadyisr);

  priv->spidev.task_id = task_create(XFER_TASK_NAME, XFER_TASK_PRI,
                                     XFER_TASK_STKSIZE, xfer_task, NULL);
  if (priv->spidev.task_id == ERROR)
    {
      m_err("Failed to create xfer task\n");
    }

  return ret;
}

/****************************************************************************
 * Name: altmdm_spi_uninit
 *
 * Description:
 *   Uninitialize ALTMDM driver.
 *
 ****************************************************************************/

int altmdm_spi_uninit(FAR struct altmdm_dev_s *priv)
{
  altmdm_sys_setflag(&priv->spidev.xfer_flag, EVENT_EXIT);

  /* check transfer task is deleted or not */

  while (1)
    {
      if (priv->spidev.is_not_run)
        {
          break;
        }

      nxsig_usleep(10);
    }

  altmdm_sys_deletelock(&priv->spidev.tx_param.lock);
  altmdm_sys_deletelock(&priv->spidev.rx_param.lock);
  altmdm_sys_deletelock(&priv->spidev.sleep_param.lock);

  altmdm_sys_deleteflag(&priv->spidev.xfer_flag);
  altmdm_sys_deleteflag(&priv->spidev.tx_param.done_flag);
  altmdm_sys_deleteflag(&priv->spidev.sleep_param.done_flag);

  if (priv->spidev.rx_param.rxbuff != NULL)
    {
      free_rxbuffer(priv, priv->spidev.rx_param.rxbuff);
      priv->spidev.rx_param.rxbuff = NULL;
    }

  destroy_rxbufffifo(priv);

  DEBUGASSERT(priv->lower);
  priv->lower->sready_irqattach(false, NULL);

  /* Uninitialize modem power management driver */

  altmdm_pm_uninit(priv);

  return 0;
}

/****************************************************************************
 * Name: altmdm_spi_enable
 *
 * Description:
 *   Enable ALTMDM SPI driver.
 *
 ****************************************************************************/

int altmdm_spi_enable(FAR struct altmdm_dev_s *priv)
{
  DEBUGASSERT(priv && priv->lower);
  priv->lower->sready_irqenable(true);

  return 0;
}

/****************************************************************************
 * Name: altmdm_spi_disable
 *
 * Description:
 *   Disable ALTMDM SPI driver.
 *
 ****************************************************************************/

int altmdm_spi_disable(FAR struct altmdm_dev_s *priv)
{
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;

  DEBUGASSERT(priv->lower);
  priv->lower->sready_irqenable(false);

  spidev->is_xferready = false;

  return 0;
}

/****************************************************************************
 * Name: altmdm_spi_read
 *
 * Description:
 *   ALTMDM SPI driver read method.
 *
 ****************************************************************************/

ssize_t altmdm_spi_read(FAR struct altmdm_dev_s * priv,
                        FAR const char *buffer, size_t readlen)
{
  FAR struct altmdm_spi_dev_s *spidev;
  FAR struct altmdm_spi_rxbuff_s *rbuff;
  ssize_t rsize = readlen;

  /* Check argument */

  if ((priv == NULL) || (buffer == NULL))
    {
      return -EINVAL;
    }

  if (!readlen || readlen > MAX_PKT_SIZE)
    {
      m_err("Invalid read length:%d.\n", readlen);
      return -EINVAL;
    }

  spidev = &priv->spidev;

  altmdm_sys_lock(&spidev->rx_param.lock);

  get_rxbufffifo(priv, &rbuff);
  if (spidev->rx_param.rxabort)
    {
      spidev->rx_param.rxabort = false;
      if (rbuff != NULL)
        {
          m_info("rx buffer discard because of abort.%d\n", __LINE__);
          free_rxbuffer(priv, rbuff);
        }

      rsize = -ECONNABORTED;
    }
  else
    {
      if (rbuff == NULL)
        {
          m_err("get rx buffer failed.\n");
          rsize = -EIO;
        }
      else if (rbuff->rx_size > readlen)
        {
          m_info("get rx buffer.%d\n", __LINE__);
          rsize = readlen;
          memcpy((void *)buffer, rbuff->buff_addr, rsize);
          free_rxbuffer(priv, rbuff);
        }
      else
        {
          m_info("get rx buffer.%d\n", __LINE__);
          rsize = rbuff->rx_size;
          memcpy((void *)buffer, rbuff->buff_addr, rsize);
          free_rxbuffer(priv, rbuff);
        }
    }

  altmdm_sys_unlock(&spidev->rx_param.lock);

  return rsize;
}

/****************************************************************************
 * Name: altmdm_spi_write
 *
 * Description:
 *   ALTMDM SPI driver write method.
 *
 ****************************************************************************/

ssize_t altmdm_spi_write(FAR struct altmdm_dev_s * priv,
                         FAR const char *buffer, size_t witelen)
{
  int ret;
  int remainder;
  uint32_t ptn;
  FAR struct altmdm_spi_dev_s *spidev;
  ssize_t wsize = witelen;

  /* Check argument */

  if ((priv == NULL) || (buffer == NULL))
    {
      return -EINVAL;
    }

  if (!witelen || witelen > MAX_PKT_SIZE)
    {
      m_err("Invalid write length:%d.\n", witelen);
      return -EINVAL;
    }

  spidev = &priv->spidev;

  if (!spidev->is_xferready)
    {
      wait_xferready(priv);
    }

  altmdm_sys_lock(&spidev->tx_param.lock);

again:
  spidev->tx_param.buff_addr = (void *)buffer;
  spidev->tx_param.actual_size = witelen;

  remainder = witelen % UNIT_SIZE;
  if (remainder == 0)
    {
      spidev->tx_param.total_size = spidev->tx_param.actual_size;
    }
  else
    {
      spidev->tx_param.total_size =
        ((spidev->tx_param.actual_size / UNIT_SIZE) + 1) * UNIT_SIZE;
    }

  spidev->tx_param.result = 0;

  altmdm_sys_setflag(&spidev->xfer_flag, EVENT_TXREQ);

  ret = altmdm_sys_waitflag(&spidev->tx_param.done_flag,
                            EVENT_TX_DONE,
                            ALTMDM_SYS_FLAG_WMODEOR,
                            &ptn,
                            WRITE_WAIT_TIMEOUT);
  if (ret != OK)
    {
      m_err("wait failed:%d\n", ret);
      wsize = -ETIME;
    }
  else
    {
      switch (spidev->tx_param.result)
        {
        case TRANS_OK:
        case TRANS_OK_TRXDATANORXBUFF:
          wsize = witelen;
          break;

        case TRANS_RXINVALID:
        case TRANS_WAITRCVRTMO:
          wsize = -EIO;
          break;

        case TRANS_OK_RCVBUFFUL:
          nxsig_usleep(100);
          goto again;
          break;

        default:
          m_err("Unexpected situation. tx result = %d.\n",
                spidev->tx_param.result);
          wsize = -EIO;
          break;
        }

      m_info("%s: tx result: %d.\n", __func__, spidev->tx_param.result);
      m_info("%s: write size: %d.\n", __func__, wsize);
    }

  altmdm_sys_unlock(&spidev->tx_param.lock);

  return wsize;
}

/****************************************************************************
 * Name: spicom_read_abort
 *
 * Description:
 *   Abort the read process.
 *
 ****************************************************************************/

int altmdm_spi_readabort(FAR struct altmdm_dev_s *priv)
{
  /* Check argument */

  if (priv == NULL)
    {
      return -EINVAL;
    }

  abort_get_rxbufffifo(priv);

  return OK;
}

/****************************************************************************
 * Name: altmdm_spi_sleepmodem
 *
 * Description:
 *   Make ALTMDM sleep.
 *
 ****************************************************************************/

int altmdm_spi_sleepmodem(FAR struct altmdm_dev_s *priv)
{
  int ret;
  bool sleep_requested;
  uint32_t ptn;
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;

  altmdm_sys_lock(&spidev->sleep_param.lock);
  sleep_requested = spidev->sleep_param.requested;
  if (!sleep_requested)
    {
      spidev->sleep_param.requested = true;
    }

  altmdm_sys_unlock(&spidev->sleep_param.lock);

  if (sleep_requested)
    {
      ret = -EBUSY;
    }
  else
    {
      spidev->sleep_param.result = 0;

      altmdm_sys_setflag(&spidev->xfer_flag, EVENT_SLEEPREQ);

      ret = altmdm_sys_waitflag(&spidev->sleep_param.done_flag,
                                EVENT_SLEEP_DONE, ALTMDM_SYS_FLAG_WMODEOR,
                                &ptn, SREQ_WAIT_TIMEOUT);
      if (ret != OK)
        {
          m_err("wait failed:%d\n", ret);
        }
      else
        {
          ret = spidev->sleep_param.result;
          m_info("%s: sleep result: %d.\n", __func__,
                 spidev->sleep_param.result);
        }

      spidev->sleep_param.requested = false;
    }

  return ret;
}

/****************************************************************************
 * Name: altmdm_spi_setreceiverready
 *
 * Description:
 *   Set receiver ready notification.
 *
 ****************************************************************************/

int altmdm_spi_setreceiverready(FAR struct altmdm_dev_s *priv)
{
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;

  altmdm_sys_setflag(&spidev->xfer_flag, EVENT_RXREQ);

  return 0;
}

/****************************************************************************
 * Name: altmdm_spi_isreceiverready
 *
 * Description:
 *   Check already notified or not by altmdm_spi_setreceiverready.
 *
 ****************************************************************************/

int altmdm_spi_isreceiverready(FAR struct altmdm_dev_s *priv)
{
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;
  struct altmdm_sys_flagstate_s flag_status;

  altmdm_sys_referflag(&spidev->xfer_flag, &flag_status);
  if (flag_status.flag_pattern & EVENT_RXREQ)
    {
      return 1;
    }

  return 0;
}

/****************************************************************************
 * Name: altmdm_spi_clearreceiverready
 *
 * Description:
 *   Clear receiver ready notification.
 *
 ****************************************************************************/

int altmdm_spi_clearreceiverready(FAR struct altmdm_dev_s *priv)
{
  FAR struct altmdm_spi_dev_s *spidev = &priv->spidev;

  altmdm_sys_clearflag(&spidev->xfer_flag, EVENT_RXREQ);

  return 0;
}

#endif /* CONFIG_MODEM_ALTMDM */
