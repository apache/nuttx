/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_usbhost.c
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
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/ohci.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbhost_devaddr.h>
#include <nuttx/usb/hid.h>

#include <nuttx/kmalloc.h>

#include <nuttx/irq.h>

#include <arch/board/board.h>

#include "renesas_internal.h"
#include "chip.h"
#include "rx65n_usbhost.h"

#include "arch/rx65n/iodefine.h"
#include "rx65n_cmtw.h"

#define INTSTS0_BIT_VALUES_TO_ACK       (0xd870u)
#define INTSTS1_BIT_VALUES_TO_ACK       (0xd870u)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* All I/O buffers must lie in AHB SRAM because of the OHCI DMA.
 * It might be okay if no I/O buffers are used *IF* the application
 * can guarantee that all end-user I/O buffers reside in AHB SRAM.
 */

#if defined(CONFIG_USBHOST) && defined(CONFIG_USBDEV)
#  error "Both USB Host & Device cannot be configured"
#endif

#ifndef CONFIG_RX65N_USBHOST_NPREALLOC
#  define CONFIG_RX65N_USBHOST_NPREALLOC 8
#endif

#ifndef CONFIG_DEBUG_USB_INFO
#  undef CONFIG_RX65N_USBHOST_REGDEBUG
#endif

/* USB Host Memory **********************************************************/

/* Periodic intervals 2, 4, 8, 16,and 32 supported */

#define MIN_PERINTERVAL 2
#define MAX_PERINTERVAL 32

/* Descriptors **************************************************************/

/* TD delay interrupt value */

#define TD_DELAY(n) (uint32_t)((n) << GTD_STATUS_DI_SHIFT)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure retains the state of the USB host controller */

struct rx65n_usbhost_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbhost_s
   * to struct rx65n_usbhost_s.
   */

  struct usbhost_driver_s drvr;

  /* This is the hub port description understood by class drivers */

  struct usbhost_roothubport_s rhport;

  /* Driver status */

  volatile bool    change;      /* Connection change */
  volatile bool    connected;   /* Connected to device */

  /* Thread is waiting for a port status change */

  volatile bool    pscwait;     /* Thread is waiting for a port status change */

#ifndef CONFIG_USBHOST_INT_DISABLE

  /* Minimum periodic IN EP polling interval: 2, 4, 6, 16, or 32 */

  uint8_t          ininterval;

  /* Minimum periodic IN EP polling interval: 2, 4, 6, 16, or 32 */

  uint8_t          outinterval;

#endif

  mutex_t          lock;         /* Support mutually exclusive access */

  /* Semaphore to wait Write-back Done Head event */

  sem_t            pscsem;
  struct work_s    rx65n_interrupt_bhalf; /* Supports interrupt bottom half */

#ifdef CONFIG_USBHOST_HUB

  /* Used to pass external hub port events */

  volatile struct usbhost_hubport_s *hport;
#endif
};

/* This structure describes one asynchronous transfer */

struct rx65n_usbhost_xfrinfo_s
{
  volatile bool wdhwait;      /* Thread is waiting for WDH interrupt */

  /* TD control status bits from last Write-back Done Head event */

  volatile uint8_t tdstatus;
  uint8_t *buffer;            /* Transfer buffer start */
  uint16_t buflen;            /* Buffer length */
  uint16_t xfrd;              /* Number of bytes transferred */
  volatile uint8_t tdxfercond;

  /* New variable. added
   * TD transfer condition : used for deciding
   * what to do in the interrupt/bottom
   * half context
   */

#ifdef CONFIG_USBHOST_ASYNCH
#if RX65N_USBHOST_IOBUFFERS > 0

  /* Remember the allocated DMA buffer address so that it can be freed when
   * the transfer completes.
   */

  uint8_t *alloc;             /* Allocated buffer */
#endif

  /* Retain the callback information for the asynchronous transfer
   * completion.
   */

  usbhost_asynch_t callback;  /* Transfer complete callback */
  void *arg;                  /* Argument that accompanies the callback */
#endif
};

/* The OCHI expects the size of an endpoint descriptor to be 16 bytes.
 * However, the size allocated for an endpoint descriptor is 32 bytes.
 * This extra 16-bytes is used by the OHCI host driver in
 * order to maintain additional endpoint-specific data.
 */

struct rx65n_usbhost_ed_s
{
  /* Hardware specific fields */

  struct ohci_ed_s hw;        /* 0-15 */

  /* Software specific fields */

  /* 16: Transfer type.  See SB_EP_ATTR_XFER_* in usb.h */

  uint8_t          xfrtype;

  /* 17: Periodic EP polling interval: 2, 4, 6, 16, or 32 */

  uint8_t          interval;

  /* 18: Semaphore used to wait for Writeback Done Head event */

  sem_t            wdhsem;

  /* Pointer to struct that manages asynchronous transfers on this pipe */

  /* 16: pipe number associated with with this Endpoint */

  uint8_t          pipenum;

  struct rx65n_usbhost_xfrinfo_s *xfrinfo;
};

struct rx65n_usbhost_gtd_s
{
  /* Hardware specific fields */

  struct ohci_gtd_s hw;

  /* Software specific fields */

  struct rx65n_usbhost_ed_s *ed;      /* Pointer to parent ED */
  uint8_t           pad[12];
};

/* The following is used to manage lists of free EDs, TDs, and TD buffers */

struct rx65n_usbhost_list_s
{
  struct rx65n_usbhost_list_s *flink;

  /* Link to next buffer in the list */

  /* Variable length buffer data follows */
};

struct rx65n_usbhost_ed_s  aligned_data(32);
struct rx65n_usbhost_gtd_s  aligned_data(32);

/* This must be aligned to a 256-byte boundary */

static struct ohci_hcca_s g_hcca        aligned_data(256);
static struct ohci_hcca_s *HCCA;

static struct rx65n_usbhost_gtd_s       *TDTAIL;

/* static struct rx65n_usbhost_ed_s      g_rx65n_ep0ed; */

static struct rx65n_usbhost_ed_s        *EDCTRL;

static struct rx65n_usbhost_gtd_s
        g_rx65n_tdlist[CONFIG_RX65N_USBHOST_NTDS];
static struct rx65n_usbhost_ed_s
        g_rx65n_edlist[CONFIG_RX65N_USBHOST_NEDS + 1];
static uint8_t nrdy_retries[CONFIG_RX65N_USBHOST_NEDS + 1];
#define RX65N_TD_BUF_SIZE (CONFIG_RX65N_USBHOST_TDBUFFERS * \
                      CONFIG_RX65N_USBHOST_TDBUFSIZE)
static uint8_t g_tdbuffer [RX65N_TD_BUF_SIZE];

static uint8_t g_kbdport;
static uint16_t g_usbidx;
static uint8_t g_hubkbd;
static uint8_t g_kbdpipe;
static uint8_t g_attached;
static uint8_t g_detached;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#define rx65n_usbhost_getreg(addr)     getreg16((volatile short *)addr)
#define rx65n_usbhost_putreg(val,addr) putreg16(val,(volatile short *)addr)
static void rx65n_usbhost_setbit(volatile short *regadd,
                                 uint16_t setbitval);
static void rx65n_usbhost_clearbit(volatile short *regadd,
                                   uint16_t clearbitval);

/* Semaphores ***************************************************************/

/* Byte stream access helper functions **************************************/

static inline uint16_t rx65n_usbhost_getle16(const uint8_t *val);

/* OHCI memory pool helper functions ****************************************/

static inline void rx65n_usbhost_edfree(struct rx65n_usbhost_ed_s *ed);
static struct rx65n_usbhost_gtd_s *rx65n_usbhost_tdalloc(uint8_t epnum);
static void rx65n_usbhost_tdfree(struct rx65n_usbhost_gtd_s *buffer);
static uint8_t *rx65n_usbhost_tballoc(void);
static void rx65n_usbhost_tbfree(uint8_t *buffer);
#if RX65N_USBHOST_IOBUFFERS > 0
static uint8_t *rx65n_usbhhost_allocio(void);
static void rx65n_usbhhost_freeio(uint8_t *buffer);
#endif
static struct rx65n_usbhost_xfrinfo_s *rx65n_usbhost_alloc_xfrinfo(void);
static void rx65n_usbhost_free_xfrinfo
        (struct rx65n_usbhost_xfrinfo_s *xfrinfo);

/* ED list helper functions *************************************************/

static inline int rx65n_usbhost_addctrled(struct rx65n_usbhost_s *priv,
                                          struct rx65n_usbhost_ed_s *ed);
static inline int rx65n_usbhost_remctrled(struct rx65n_usbhost_s *priv,
                                          struct rx65n_usbhost_ed_s *ed);

static inline int rx65n_usbhost_addbulked(struct rx65n_usbhost_s *priv,
                                          const struct usbhost_epdesc_s
                                          *epdesc, struct rx65n_usbhost_ed_s
                                          *ed);

static inline int rx65n_usbhost_rembulked(struct rx65n_usbhost_s *priv,
                                          struct rx65n_usbhost_ed_s *ed);

#if !defined(CONFIG_USBHOST_INT_DISABLE) || !defined(CONFIG_USBHOST_ISOC_DISABLE)
static unsigned int rx65n_usbhost_getinterval(uint8_t interval);
static void rx65n_usbhost_setinttab(uint32_t value, unsigned int interval,
                                    unsigned int offset);
#endif

static inline int rx65n_usbhost_addinted(struct rx65n_usbhost_s *priv,
                                const struct usbhost_epdesc_s *epdesc,
                                struct rx65n_usbhost_ed_s *ed);
static inline int rx65n_usbhost_reminted(struct rx65n_usbhost_s *priv,
                                struct rx65n_usbhost_ed_s *ed);

static inline int rx65n_usbhost_addisoced(struct rx65n_usbhost_s *priv,
                                  const struct usbhost_epdesc_s *epdesc,
                                  struct rx65n_usbhost_ed_s *ed);
static inline int rx65n_usbhost_remisoced(struct rx65n_usbhost_s *priv,
                                  struct rx65n_usbhost_ed_s *ed);

/* Descriptor helper functions **********************************************/

static int rx65n_usbhost_enqueuetd(struct rx65n_usbhost_s *priv,
                           struct rx65n_usbhost_ed_s *ed, uint32_t dirpid,
                           uint32_t toggle, volatile uint8_t *buffer,
                           size_t buflen);
static int rx65n_usbhost_ctrltd(struct rx65n_usbhost_s *priv,
                        struct rx65n_usbhost_ed_s *ed,
                        uint32_t dirpid, uint8_t *buffer, size_t buflen);

/* Interrupt handling *******************************************************/

static int rx65n_usbhost_usbinterrupt(int irq, void *context, void *arg);

/* USB host controller operations *******************************************/

static int rx65n_usbhost_wait(struct usbhost_connection_s *conn,
                              struct usbhost_hubport_s **hport);
static int rx65n_usbhost_rh_enumerate(struct usbhost_connection_s *conn,
                                      struct usbhost_hubport_s *hport);
static int rx65n_usbhost_enumerate(struct usbhost_connection_s *conn,
                                   struct usbhost_hubport_s *hport);

static int rx65n_usbhost_ep0configure(struct usbhost_driver_s *drvr,
                                      usbhost_ep_t ep0, uint8_t funcaddr,
                                      uint8_t speed, uint16_t maxpacketsize);
static int rx65n_usbhost_epalloc(struct usbhost_driver_s *drvr,
                                 const struct usbhost_epdesc_s *epdesc,
                                 usbhost_ep_t *ep);
static int rx65n_usbhost_epfree(struct usbhost_driver_s *drvr,
                                usbhost_ep_t ep);
static int rx65n_usbhost_alloc(struct usbhost_driver_s *drvr,
                               uint8_t **buffer, size_t *maxlen);
static int rx65n_usbhost_free(struct usbhost_driver_s *drvr,
                              uint8_t *buffer);
static int rx65n_usbhost_ioalloc(struct usbhost_driver_s *drvr,
                                 uint8_t **buffer, size_t buflen);
static int rx65n_usbhost_iofree(struct usbhost_driver_s *drvr,
                                uint8_t *buffer);
static int rx65n_usbhost_ctrlin(struct usbhost_driver_s *drvr,
                                usbhost_ep_t ep0,
                                const struct usb_ctrlreq_s *req,
                                uint8_t *buffer);
static int rx65n_usbhost_ctrlout(struct usbhost_driver_s *drvr,
                                 usbhost_ep_t ep0,
                                 const struct usb_ctrlreq_s *req,
                                 const uint8_t *buffer);
static int rx65n_usbhost_transfer_common(struct rx65n_usbhost_s *priv,
                                         struct rx65n_usbhost_ed_s *ed,
                                         uint8_t *buffer, size_t buflen);
#if RX65N_USBHOST_IOBUFFERS > 0
static int rx65n_usbhost_dma_alloc(struct rx65n_usbhost_s *priv,
                                   struct rx65n_usbhost_ed_s *ed,
                                   uint8_t *userbuffer,
                                   size_t buflen, uint8_t **alloc);
static void rx65n_usbhost_dma_free(struct rx65n_usbhost_s *priv,
                                   struct rx65n_usbhost_ed_s *ed,
                                   uint8_t *userbuffer,
                                   size_t buflen, uint8_t *alloc);
#endif
static ssize_t rx65n_usbhost_transfer(struct usbhost_driver_s *drvr,
                                      usbhost_ep_t ep,
                                      uint8_t *buffer, size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static void rx65n_usbhost_asynch_completion(struct rx65n_usbhost_s *priv,
                                            struct rx65n_usbhost_ed_s *ed);
static int rx65n_usbhost_asynch(struct usbhost_driver_s *drvr,
                                usbhost_ep_t ep, uint8_t *buffer,
                                size_t buflen, usbhost_asynch_t callback,
                                void *arg);
#endif
static int rx65n_usbhost_cancel(struct usbhost_driver_s *drvr,
                                usbhost_ep_t ep);
#ifdef CONFIG_USBHOST_HUB
static int rx65n_usbhost_connect(struct usbhost_driver_s *drvr,
                                 struct usbhost_hubport_s *hport,
                                 bool connected);
#endif
static void rx65n_usbhost_disconnect(struct usbhost_driver_s *drvr,
                                     struct usbhost_hubport_s *hport);

/* Initialization ***********************************************************/

static inline void rx65n_usbhost_ep0init(struct rx65n_usbhost_s *priv);

/* Prototype for USB Private Functions */

static void usb_cstd_set_nak(uint16_t pipe);
static void usb_cstd_clr_transaction_counter(uint16_t trnreg);
void usb_hstd_read_lnst(uint16_t *buf);
void usb_hstd_chk_sof(void);
void usb_hstd_attach(uint16_t result);
void usb_hstd_detach(void);
static void usb_cstd_chg_curpipe(uint16_t pipe, uint16_t fifosel,
                                 uint16_t isel);
static void usb_cstd_do_aclrm(uint16_t pipe);
static void usb_cstd_set_buf(uint16_t pipe);
static void usb_cstd_clr_stall(uint16_t pipe);
static void usb_cstd_pipe_init(uint16_t pipe);
void usb_hstd_bus_reset(void);
uint16_t usb_hstd_detach_process(void);
static uint16_t usb_cstd_get_pipe_dir(uint16_t pipe);
static void usb_cstd_clr_pipe_cnfg(uint16_t pipe_no);

static void *hw_usb_get_fifosel_adr(uint16_t pipemode);
static void *hw_usb_get_fifoctr_adr(uint16_t pipemode);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* In this driver implementation, support is provided for only a single a
 * single USB device.  All status information can be simply retained in a
 * single globalinstance.
 */

static struct rx65n_usbhost_s g_usbhost =
{
  .lock = NXMUTEX_INITIALIZER,
  .pscsem = SEM_INITIALIZER(0),
};

/* This is the connection/enumeration interface */

static struct usbhost_connection_s g_usbconn =
{
  .wait      = rx65n_usbhost_wait,
  .enumerate = rx65n_usbhost_enumerate,
};

/* This is a free list of EDs and TD buffers */

static struct rx65n_usbhost_list_s *g_edfree; /* List of unused EDs */
static struct rx65n_usbhost_list_s *g_tdfree; /* List of unused TDs */
static struct rx65n_usbhost_list_s *g_tbfree; /* List of unused TBs */
#if RX65N_USBHOST_IOBUFFERS > 0
static struct rx65n_usbhost_list_s *g_iofree; /* unused IO buffers */
#endif

/* Pool and free-list of transfer structures */

static struct rx65n_usbhost_list_s *g_xfrfree;
static struct rx65n_usbhost_xfrinfo_s g_xfrbuffers
       [CONFIG_RX65N_USBHOST_NPREALLOC];

/* Prototype for interrupt bottom half function */

static void rx65n_usbhost_bottomhalf(void *arg);

typedef struct usb_pipe_table
{
    uint16_t    use_flag;
    uint16_t    pipe_cfg;
    uint16_t    pipe_maxp;
    uint16_t    pipe_peri;
} usb_pipe_table_t;

usb_pipe_table_t g_usb_pipe_table[USB_MAX_PIPE_NUM + 1];

uint8_t kbd_report_data[8];
uint8_t kbd_interrupt_in_pipe = 0;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Prototypes to avoid errors */

static uint16_t usb_cstd_get_buf_size(uint16_t pipe);
static uint16_t usb_cstd_is_set_frdy(uint16_t pipe, uint16_t fifosel,
                                     uint16_t isel);
static uint16_t usb_cstd_get_maxpacket_size(uint16_t pipe);
uint8_t *usb_hstd_read_fifo(uint16_t count, uint16_t pipemode,
                            uint8_t *read_p);
static uint16_t usb_cstd_get_pid(uint16_t pipe);
uint8_t *usb_hstd_write_fifo(uint16_t count, uint16_t pipemode,
                             uint8_t *write_p);
static void usb_cstd_set_transaction_counter(uint16_t trnreg,
                                             uint16_t trncnt);
static void usb_cstd_nrdy_enable(uint16_t pipe);
void usb_hstd_buf_to_fifo(uint8_t *buffer, size_t buflen, uint16_t pipe,
                          uint16_t useport);
void usb_hstd_forced_termination(uint16_t pipe, uint16_t status);
void usb_hstd_nrdy_endprocess(uint16_t pipe);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void rx65n_usbhost_setbit(volatile short *regadd,
                                 uint16_t setbit_val)
{
  volatile uint16_t value_to_set;
  value_to_set = rx65n_usbhost_getreg(regadd);
  value_to_set = value_to_set | setbit_val;
  rx65n_usbhost_putreg(value_to_set, regadd);
}

static void rx65n_usbhost_clearbit(volatile short *regadd,
                                   uint16_t clearbit_val)
{
  volatile uint16_t value_to_clear;
  value_to_clear = rx65n_usbhost_getreg(regadd);
  value_to_clear = value_to_clear & (~clearbit_val);
  rx65n_usbhost_putreg(value_to_clear, regadd);
}

/****************************************************************************
 * Function Name   : usb_chattaring
 * Description     : Remove chattaring processing
 * Arguments       : uint16_t *syssts : SYSSTS register value
 * Return value    : LNST bit value
 ****************************************************************************/

uint16_t usb_chattaring(uint16_t *syssts)
{
  uint16_t lnst[4];

  /* WAIT_LOOP */

  while (1)
    {
      lnst[0] = (*syssts) & RX65N_USB_SYSSTS0_LNST;
      up_mdelay(1); /* 1ms wait */
      lnst[1] = (*syssts) & RX65N_USB_SYSSTS0_LNST;
      up_mdelay(1); /* 1ms wait */
      lnst[2] = (*syssts) & RX65N_USB_SYSSTS0_LNST;
      if ((lnst[0] == lnst[1]) && (lnst[0] == lnst[2]))
        {
          break;
        }
    }

  return lnst[0];
}

/****************************************************************************
 * Function Name   : hw_usb_hset_enb_ovrcre
 * Description     : Set specified port's OVRCRE-bit (Overcurrent Input
 *                 : Change Interrupt Status Enable) in the INTENB1 register.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_hset_enb_ovrcre(void)
{
  rx65n_usbhost_setbit(RX65N_USB_INTENB1, RX65N_USB_INTENB1_OVRCRE);
}

/****************************************************************************
 * Function Name   : hw_usb_hclear_enb_ovrcre
 * Description     : Clear the OVRCRE-bit of the specified port's INTENB1
 *                 : register, to prohibit VBUS interrupts.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_hclear_enb_ovrcre(void)
{
  rx65n_usbhost_clearbit(RX65N_USB_INTENB1, RX65N_USB_INTENB1_OVRCRE);
}

/****************************************************************************
 * Function Name   : hw_usb_hset_enb_bchge
 * Description     : The BCHGE-bit (USB Bus Change Interrupt Enable) is set
 *                 : in the specified port's INTENB1 register. This will
 *                 : cause a BCHG interrupt when a change of USB bus state
 *                 : has been detected.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_hset_enb_bchge(void)
{
  rx65n_usbhost_setbit(RX65N_USB_INTENB1, RX65N_USB_INTENB1_BCHGE);
}

/****************************************************************************
 * Function Name   : hw_usb_hclear_enb_bchge
 * Description     : The BCHGE-bit (USB Bus Change Interrupt Enable) is
 *                 : cleared in the specified port's INTENB1 register.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_hclear_enb_bchge(void)
{
  rx65n_usbhost_clearbit(RX65N_USB_INTENB1, RX65N_USB_INTENB1_BCHGE);
}

/****************************************************************************
 * Function Name   : hw_usb_hset_enb_dtche
 * Description     : Enable the specified port's DTCHE-interrupt
 *                 : "Disconnection Detection" by setting the DTCHE-bit.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_hset_enb_dtche(void)
{
  rx65n_usbhost_setbit(RX65N_USB_INTENB1, RX65N_USB_INTENB1_DTCHE);
}

/****************************************************************************
 * Function Name   : hw_usb_hclear_enb_dtche
 * Description     : Disable the specified port's DTCHE-interrupt
 *                 : "Disconnection Detection" by clearing the DTCHE-bit.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_hclear_enb_dtche(void)
{
  rx65n_usbhost_clearbit(RX65N_USB_INTENB1, RX65N_USB_INTENB1_DTCHE);
}

/****************************************************************************
 * Function Name   : hw_usb_hset_enb_attche
 * Description     : Enable the specified port's ATTCHE-interrupt "Connection
 *                 : Detection" by setting the ATTCHE-bit.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_hset_enb_attche(void)
{
  rx65n_usbhost_setbit(RX65N_USB_INTENB1, RX65N_USB_INTENB1_ATTCHE);
}

/****************************************************************************
 * Function Name   : hw_usb_hclear_enb_attche
 * Description     : Disable the specified port's ATTCHE-interrupt
 *                 : "Disconnection Detection" by clearing the ATTCHE-bit.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_hclear_enb_attche(void)
{
  rx65n_usbhost_clearbit(RX65N_USB_INTENB1, RX65N_USB_INTENB1_ATTCHE);
}

/****************************************************************************
 * Function Name   : hw_usb_hset_enb_signe
 * Description     : Enable the SIGNE-interrupt "Setup Transaction
 *                 : Error" by setting the SIGNE-bit.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_hset_enb_signe(void)
{
  rx65n_usbhost_setbit(RX65N_USB_INTENB1, RX65N_USB_INTENB1_SIGNE);
}

/****************************************************************************
 * Function Name   : hw_usb_hset_enb_sacke
 * Description     : Enable the SACKE-interrupt "Setup Transaction
 *                 : Normal Response" by setting the SACKE-bit.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_hset_enb_sacke(void)
{
  rx65n_usbhost_setbit(RX65N_USB_INTENB1, RX65N_USB_INTENB1_SACKE);
}

/****************************************************************************
 * Function Name   : hw_usb_hclear_sts_ovrcr
 * Description     : Clear the specified port's OVRCR-bit; "Over current
 *                 : Input Change Interrupt Status".
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_hclear_sts_ovrcr(void)
{
  rx65n_usbhost_putreg(((~RX65N_USB_INTSTS1_OVRCRE) &
                        INTSTS1_BIT_VALUES_TO_ACK), RX65N_USB_INTSTS1);
}

/****************************************************************************
 * Function Name   : hw_usb_hclear_sts_bchg
 * Description     : Clear the specified port's BCHG-bit; "USB Bus Change
 *                 : Interrupt Status".
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_hclear_sts_bchg(void)
{
  rx65n_usbhost_putreg(((~RX65N_USB_INTSTS1_BCHG) &
                        INTSTS1_BIT_VALUES_TO_ACK), RX65N_USB_INTSTS1);
}

/****************************************************************************
 * Function Name   : hw_usb_hclear_sts_dtch
 * Description     : Clear the specified port's DTCH-bit; "USB Disconnection
 *                 : Detection Interrupt Status".
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_hclear_sts_dtch(void)
{
  rx65n_usbhost_putreg(((~RX65N_USB_INTSTS1_DTCH) &
                        INTSTS1_BIT_VALUES_TO_ACK), RX65N_USB_INTSTS1);
}

/****************************************************************************
 * Function Name   : hw_usb_hclear_sts_attch
 * Description     : Clear the specified port's ATTCH-bit; "ATTCH Interrupt
 *                 : Status".
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_hclear_sts_attch(void)
{
  rx65n_usbhost_putreg(((~RX65N_USB_INTSTS1_ATTCH) &
                       INTSTS1_BIT_VALUES_TO_ACK), RX65N_USB_INTSTS1);
}

/****************************************************************************
 * Function Name   : hw_usb_hclear_sts_sign
 * Description     : Clear the SIGN-bit; "Setup Transaction Error
 *                 : Interrupt Status".
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_hclear_sts_sign(void)
{
  rx65n_usbhost_putreg(((~RX65N_USB_INTSTS1_SIGN) &
                       INTSTS1_BIT_VALUES_TO_ACK), RX65N_USB_INTSTS1);
}

/****************************************************************************
 * Function Name   : hw_usb_hclear_sts_sack
 * Description     : Clear the SACK-bit; "Setup Transaction Normal
 *                 : Response Interrupt Status".
 *                 : Interrupt Status".
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_hclear_sts_sack(void)
{
  rx65n_usbhost_putreg(((~RX65N_USB_INTSTS1_SACK) &
                       INTSTS1_BIT_VALUES_TO_ACK), RX65N_USB_INTSTS1);
}

/****************************************************************************
 * Function Name   : hw_usb_hwrite_dcpctr
 * Description     : Write the specified data value to the DCPCTR register.
 * Arguments       : uint16_t  data   : Setting value
 * Return value    : none
 ****************************************************************************/

void hw_usb_hwrite_dcpctr(uint16_t data)
{
  rx65n_usbhost_putreg(data, RX65N_USB_DCPCTR);
}

/****************************************************************************
 * Function Name   : hw_usb_hset_sureq
 * Description     : Set te SUREQ-bit in the DCPCTR register
 *                 : (Set SETUP packet send when HostController function
 *                                 : is selected)
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_hset_sureq(void)
{
  rx65n_usbhost_setbit(RX65N_USB_DCPCTR, RX65N_USB_DCPCTR_SUREQ);
}

/****************************************************************************
 * Function Name   : hw_usb_hread_devadd
 * Description     : Return the DEVADD register value for the specified USB
 *                 ; device address.
 * Arguments       : uint16_t  devsel : USB device address value
 * Return value    : DEVADDx content
 ****************************************************************************/

uint16_t hw_usb_hread_devadd(uint16_t devsel)
{
  volatile uint16_t *preg;
  uint16_t devadr;
  uint16_t return_value;

  devadr = devsel >> USB_DEVADDRBIT;

  if (devadr > USB_MAXDEVADDR)
    {
      uerr("ERROR: device address %d is more than max device \
            address.\n", devadd);
      return -ENODEV;
    }
  else
    {
      preg = (uint16_t *)((RX65N_USB_DEVADD0) + (devadr));
      return_value = ((*preg));
      return return_value;
    }
}

/****************************************************************************
 * Function Name   : hw_usb_hset_usbspd
 * Description     : Set the DEVADD register's USBSPD for the specified
 *                                 : device address.
 * Arguments       : uint16_t  devsel : USB device address value
 *                 : uint16_t  data   : The value to write.
 * Return value    : none
 ****************************************************************************/

void hw_usb_hset_usbspd(uint16_t devsel, uint8_t data)
{
  volatile uint16_t *preg;
  uint16_t devadr;

  devadr = devsel;

  preg = (uint16_t *)(RX65N_USB_DEVADD0 + devadr);

  (*preg) &= (~RX65N_USB_DEVSPD);

  (*preg) |= data;
}

/****************************************************************************
 * Function Name   : hw_usb_hmodule_init
 * Description     : USB module initialization for USB Host mode
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_hmodule_init(void)
{
  uint16_t sts;

  rx65n_usbhost_setbit(RX65N_USB_SYSCFG, RX65N_USB_SYSCFG_SCKE);
  up_mdelay(1);

  /* wait until SCKE bit is set */

  while (1)
    {
      uint16_t regval;
      regval = rx65n_usbhost_getreg(RX65N_USB_SYSCFG);
      if (regval & RX65N_USB_SYSCFG_SCKE)
      break;
    }

  putreg32(0x05, RX65N_USB_PHYSLEW);

  rx65n_usbhost_setbit(RX65N_USB_SYSCFG, RX65N_USB_SYSCFG_DCFM);

  rx65n_usbhost_setbit(RX65N_USB_SYSCFG, RX65N_USB_SYSCFG_DRPD);

  sts = usb_chattaring((uint16_t *)RX65N_USB_SYSSTS0);

  rx65n_usbhost_setbit(RX65N_USB_SYSCFG, RX65N_USB_SYSCFG_USBE);

  rx65n_usbhost_setbit(RX65N_USB_CFIFOSEL, RX65N_USB_CFIFOSEL_MBW_16);
  rx65n_usbhost_setbit(RX65N_USB_D0FIFOSEL, RX65N_USB_DFIFOSEL_MBW_16);
  rx65n_usbhost_setbit(RX65N_USB_D1FIFOSEL, RX65N_USB_DFIFOSEL_MBW_16);

  switch (sts)
  {
    case RX65N_USB_SYSSTS0_LNST_LS_JSTS:
    case RX65N_USB_SYSSTS0_LNST_FS_JSTS: /* USB device already connected */

    syslog(LOG_INFO, "USB Device already connected\n");
    rx65n_usbhost_setbit(RX65N_USB_DVSTCTR0, RX65N_USB_DVSTCTR0_USBRST);
    up_mdelay(20);                        /* Need to wait greater equal 10ms in USB spec */
    rx65n_usbhost_clearbit(RX65N_USB_DVSTCTR0,
                           RX65N_USB_DVSTCTR0_USBRST);

    /* WAIT_LOOP */

    while (((rx65n_usbhost_getreg(RX65N_USB_DVSTCTR0)) &
        RX65N_USB_DVSTCTR0_RHST) == 4)
      {
        /* Wait till the reset is completed */

        up_mdelay(1);
      }

    if (((rx65n_usbhost_getreg(RX65N_USB_DVSTCTR0)) &
          RX65N_USB_DVSTCTR0_RHST) ==
          RX65N_USB_DVSTCTR0_SPEED_LOW)
      {
        rx65n_usbhost_setbit(RX65N_USB_SOFCFG,
        RX65N_USB_SOFCFG_TRNENSEL);
      }

    rx65n_usbhost_setbit(RX65N_USB_DVSTCTR0,
    RX65N_USB_DVSTCTR0_UACT);
    break;

    case RX65N_USB_SYSSTS0_LNST_SE0:     /* USB device not connected */
    rx65n_usbhost_setbit(RX65N_USB_INTENB1, RX65N_USB_INTENB1_ATTCHE);
    break;

    default:
    break;
  }

  rx65n_usbhost_putreg(((~RX65N_USB_INTSTS1_OVRCRE) &
  INTSTS1_BIT_VALUES_TO_ACK), RX65N_USB_INTSTS1);
  rx65n_usbhost_setbit(RX65N_USB_INTENB0, (RX65N_USB_INTENB0_BEMPE |
  RX65N_USB_INTENB0_NRDYE | RX65N_USB_INTENB0_BRDYE));
  rx65n_usbhost_setbit(RX65N_USB_INTENB1, RX65N_USB_INTENB1_ATTCHE);
}

/****************************************************************************
 * Function Name   : hw_usb_read_syscfg
 * Description     : Returns the SYSCFG register value.
 * Arguments       : none
 * Return value    : SYSCFG content.
 ****************************************************************************/

uint16_t hw_usb_read_syscfg(void)
{
  return rx65n_usbhost_getreg(RX65N_USB_SYSCFG);
}

/****************************************************************************
 * Function Name   : hw_usb_set_usbe
 * Description     : Enable USB operation.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_set_usbe(void)
{
  rx65n_usbhost_setbit(RX65N_USB_SYSCFG, RX65N_USB_SYSCFG_USBE);
}

/****************************************************************************
 * Function Name   : hw_usb_clear_usbe
 * Description     : Enable USB operation.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_clear_usbe(void)
{
  rx65n_usbhost_clearbit(RX65N_USB_SYSCFG, RX65N_USB_SYSCFG_USBE);
}

/****************************************************************************
 * Function Name   : hw_usb_read_syssts
 * Description     : Returns the value of the specified port's SYSSTS
 *                 : register.
 * Arguments       : none
 * Return value    : SYSSTS0 content
 ****************************************************************************/

static uint16_t hw_usb_read_syssts(void)
{
  return rx65n_usbhost_getreg(RX65N_USB_SYSSTS0);
}

/****************************************************************************
 * Function Name   : hw_usb_read_dvstctr
 * Description     : Returns the specified port's DVSTCTR register content.
 * Arguments       : none
 * Return value    : DVSTCTR0 content
 ****************************************************************************/

static uint16_t hw_usb_read_dvstctr(void)
{
  return rx65n_usbhost_getreg(RX65N_USB_DVSTCTR0);
}

/****************************************************************************
 * Function Name   : hw_usb_rmw_dvstctr
 * Description     : Read-modify-write the specified port's DVSTCTR.
 * Arguments       : uint16_t  data  : The value to write.
 *                 : uint16_t  bitptn: Bit pattern to read-modify-write.
 * Return value    : none
 ****************************************************************************/

void hw_usb_rmw_dvstctr(uint16_t data, uint16_t bitptn)
{
  uint16_t buf;

  buf = rx65n_usbhost_getreg(RX65N_USB_DVSTCTR0);
  buf &= (~bitptn);
  buf |= (data & bitptn);

  rx65n_usbhost_putreg(buf, RX65N_USB_DVSTCTR0);
}

/****************************************************************************
 * Function Name   : hw_usb_clear_dvstctr
 * Description     : Clear the bit pattern specified in argument, of the
 *                                 : specified port's DVSTCTR register.
 * Arguments       : uint16_t  bitptn: Bit pattern to read-modify-write.
 * Return value    : none
 ****************************************************************************/

void hw_usb_clear_dvstctr(uint16_t bitptn)
{
  rx65n_usbhost_clearbit(RX65N_USB_DVSTCTR0, bitptn);
}

/****************************************************************************
 * Function Name   : hw_usb_set_vbout
 * Description     : Set specified port's VBOUT-bit in the DVSTCTR register.
 *                 : (To output a "High" to pin VBOUT.)
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_set_vbout(void)
{
  rx65n_usbhost_setbit(RX65N_USB_DVSTCTR0, RX65N_USB_DVSTCTR0_VBUSEN);
}

/****************************************************************************
 * Function Name   : hw_usb_clear_vbout
 * Description     : Clear specified port's VBOUT-bit in the DVSTCTR register
 *                 : (To output a "Low" to pin VBOUT.)
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_clear_vbout(void)
{
  rx65n_usbhost_clearbit(RX65N_USB_DVSTCTR0, RX65N_USB_DVSTCTR0_VBUSEN);
}

/****************************************************************************
 * Function Name   : hw_usb_read_fifo16
 * Description     : Data is read from the specified pipemode's FIFO register
 *                 : 16-bits  wide, corresponding to the specified PIPEMODE.
 * Arguments       : uint16_t  pipemode  : CUSE/D0DMA/D1DMA
 * Return value    : CFIFO/D0FIFO/D1FIFO content (16-bit)
 ****************************************************************************/

static uint16_t hw_usb_read_fifo16(uint16_t pipemode)
{
  uint16_t data = 0;

  switch (pipemode)
    {
      case RX65N_USB_USING_CFIFO:
      data = USB0.CFIFO.WORD;
      break;

      case RX65N_USB_USING_D0FIFO:
      data = rx65n_usbhost_getreg(RX65N_USB_D0FIFO);
      break;

      case RX65N_USB_USING_D1FIFO:
      data = rx65n_usbhost_getreg(RX65N_USB_D1FIFO);
      break;

      default:
      syslog(LOG_INFO, "Debug : %s(): Line : %d what is this \
             pipe mode?? %d\n", __func__, __LINE__, pipemode);
      break;
    }

  return data;
}

/****************************************************************************
 * Function Name   : hw_usb_write_fifo16
 * Description     : Data is written to the specified pipemode's FIFO
 *                 : register, 16-bits wide, corresponding to the specified
 *                 : PIPEMODE.
 * Arguments       : uint16_t  pipemode  : CUSE/D0DMA/D1DMA
 *                 : uint16_t  data      : Setting value
 * Return value    : none
 ****************************************************************************/

static void hw_usb_write_fifo16(uint16_t pipemode, uint16_t data)
{
  switch (pipemode)
    {
      case RX65N_USB_USING_CFIFO:
      data = rx65n_usbhost_putreg(data, RX65N_USB_CFIFO);
      break;

      case RX65N_USB_USING_D0FIFO:
      data = rx65n_usbhost_putreg(data, RX65N_USB_D0FIFO);
      break;

      case RX65N_USB_USING_D1FIFO:
      data = rx65n_usbhost_putreg(data, RX65N_USB_D1FIFO);
      break;

      default:
      syslog(LOG_INFO, "Debug : %s(): Line : %d what is this \
             pipe mode?? %d\n", __func__, __LINE__, pipemode);
      break;
    }
}

/****************************************************************************
 * Function Name   : hw_usb_write_fifo8
 * Description     : Data is written to the specified pipemode's FIFO
 *                 : register, 8-bits wide, corresponding to the specified
 *                 : PIPEMODE.
 * Arguments       : uint16_t  pipemode  : CUSE/D0DMA/D1DMA
 *                 : uint8_t  data       : Setting value
 * Return value    : none
 ****************************************************************************/

static void hw_usb_write_fifo8(uint16_t pipemode, uint8_t data)
{
  switch (pipemode)
  {
    case USB_CUSE:

    /* USB0_CFIFO8 = data; */

    putreg8(data, RX65N_USB_CFIFO);
    break;

    case USB_D0USE:

    /* USB0_D0FIFO8 = data; */

    putreg8(data, RX65N_USB_D0FIFO);
    break;

    case USB_D1USE:

    /* USB0_D1FIFO8 = data; */

    putreg8(data, RX65N_USB_D1FIFO);
    break;

    default:
    syslog(LOG_INFO, "Debug : %s(): Line : %d Debug hook...\n",
           __func__, __LINE__);
    break;
  }
}

/****************************************************************************
 * Function Name   : hw_usb_get_fifosel_adr
 * Description     : Returns the *address* of the FIFOSEL register
 *                 : corresponding to specified PIPEMODE.
 * Arguments       : uint16_t  pipemode  : CUSE/D0DMA/D1DMA
 * Return value    : none
 ****************************************************************************/

static void *hw_usb_get_fifosel_adr(uint16_t pipemode)
{
  void *p_reg = NULL;

  switch (pipemode)
    {
      case RX65N_USB_USING_CFIFO:
      p_reg = (void *)RX65N_USB_CFIFOSEL;
      break;

      case RX65N_USB_USING_D0FIFO:
      p_reg = (void *)RX65N_USB_D0FIFOSEL;
      break;

      case RX65N_USB_USING_D1FIFO:
      p_reg = (void *)RX65N_USB_D1FIFOSEL;
      break;

      default:
      syslog(LOG_INFO,
             "Debug : %s(): Line : %d what is this pipe mode?? %d\n",
             __func__, __LINE__, pipemode);
      break;
    }

  return p_reg;
}

/****************************************************************************
 * Function Name   : hw_usb_read_fifosel
 * Description     : Returns the value of the specified pipemode's FIFOSEL
 *                 : register.
 * Arguments       : uint16_t  pipemode  : CUSE/D0DMA/D1DMA
 * Return value    : FIFOSEL content
 ****************************************************************************/

static uint16_t hw_usb_read_fifosel(uint16_t pipemode)
{
  volatile uint16_t *p_reg;

  p_reg = (uint16_t *)hw_usb_get_fifosel_adr(pipemode);

  return *p_reg;
}

/****************************************************************************
 * Function Name   : hw_usb_rmw_fifosel
 * Description     : Data is written to the specified pipemode's FIFOSEL
 *                 : register (the FIFOSEL corresponding to the specified
 *                 : PIPEMODE), using read-modify-write.
 * Arguments       : uint16_t  pipemode  : CUSE/D0DMA/D1DMA
 *                 : uint16_t  data      : Setting value
 *                 : uint16_t bitptn   : bitptn: Bit pattern to
 *                 : read-modify-write.
 * Return value    : none
 ****************************************************************************/

static void hw_usb_rmw_fifosel(uint16_t pipemode, uint16_t data,
                               uint16_t bitptn)
{
  uint16_t buf;
  volatile uint16_t *p_reg;

  p_reg = (uint16_t *)hw_usb_get_fifosel_adr(pipemode);

  buf = *p_reg;
  buf &= (~bitptn);
  buf |= (data & bitptn);
  *p_reg = buf;
}

/****************************************************************************
 * Function Name   : hw_usb_set_mbw
 * Description     : Set MBW-bits (CFIFO Port Access Bit Width) of the
 *                 : FIFOSEL corresponding to the specified PIPEMODE, to
 *                 : select 8 or 16-bit wide FIFO port access.
 * Arguments       : uint16_t  pipemode   : CUSE/D0DMA/D1DMA
 *                 : uint16_t  data       : Setting value
 *                 : (data = 0x0400), 32 bit (data = 0x0800) access mode.
 * Return value    : none
 ****************************************************************************/

static void hw_usb_set_mbw(uint16_t pipemode, uint16_t data)
{
  volatile uint16_t *p_reg;

  p_reg = hw_usb_get_fifosel_adr(pipemode);
  (*p_reg) &= (~RX65N_USB_CFIFOSEL_MBW_16);

  if (0 != data)
    {
      (*p_reg) |= RX65N_USB_CFIFOSEL_MBW_16;
    }
}

/****************************************************************************
 * Function Name   : hw_usb_set_curpipe
 * Description     : Set pipe to the number given; in the FIFOSEL
 *                 : corresponding to specified PIPEMODE.
 * Arguments       : uint16_t  pipemode   : CUSE/D0DMA/D1DMA
 *                 : uint16_t  pipeno     : Pipe number.
 * Return value    : none
 ****************************************************************************/

static void hw_usb_set_curpipe(uint16_t pipemode, uint16_t pipeno)
{
  volatile uint16_t *p_reg;
  volatile uint16_t reg;

  p_reg = hw_usb_get_fifosel_adr(pipemode);
  reg = *p_reg;

#ifdef CONFIG_RX65N_USBHOST_DMA
  if ((USB_D0USE == pipemode) || (USB_D1USE == pipemode))
    {
      if (false == usb_check_use_usba_module(ptr))
        {
          reg &= (~USB_DREQE);
        }
    }
#endif /* NOT_USING_D0_D1_FIFO */

  reg &= (~RX65N_USB_CFIFOSEL_CURPIPE_MASK);
  *p_reg = reg;

  reg |= pipeno;

  *p_reg = reg;
}

/****************************************************************************
 * Function Name   : hw_usb_get_fifoctr_adr
 * Description     : Returns the *address* of the FIFOCTR register
 *                 : corresponding to specified PIPEMODE.
 *                 : (FIFO Port Control Register.)
 * Arguments       : uint16_t  pipemode   : CUSE/D0DMA/D1DMA
 * Return value    : none
 ****************************************************************************/

static void *hw_usb_get_fifoctr_adr(uint16_t pipemode)
{
  void *p_reg = NULL;
  switch (pipemode)
    {
      case RX65N_USB_USING_CFIFO:
      p_reg = (void *)RX65N_USB_CFIFOCTR;
      break;

      case RX65N_USB_USING_D0FIFO:
      p_reg = (void *)RX65N_USB_D0FIFOCTR;
      break;

      case RX65N_USB_USING_D1FIFO:
      p_reg = (void *)RX65N_USB_D0FIFOCTR;
      break;

      default:
      syslog(LOG_INFO, "Debug : %s(): Line : %d what is this \
             pipe mode?? %d\n",
             __func__, __LINE__, pipemode);
      break;
    }

  return p_reg;
}

/****************************************************************************
 * Function Name   : hw_usb_read_fifoctr
 * Description     : Returns the value of the FIFOCTR register corresponding
 *                 : to specified PIPEMODE.
 * Arguments       : uint16_t  pipemode   : CUSE/D0DMA/D1DMA
 * Return value    : FIFOCTR content
 ****************************************************************************/

static uint16_t hw_usb_read_fifoctr(uint16_t pipemode)
{
  volatile uint16_t *p_reg;

  p_reg = (uint16_t *)hw_usb_get_fifoctr_adr(pipemode);

  return *p_reg;
}

/****************************************************************************
 * Function Name   : hw_usb_set_bval
 * Description     : Set BVAL (Buffer Memory Valid Flag) to the number given;
 *                 : in the FIFOCTR corresponding to the specified PIPEMODE.
 * Arguments       : uint16_t  pipemode   : CUSE/D0DMA/D1DMA
 * Return value    : none
 ****************************************************************************/

static void hw_usb_set_bval(uint16_t pipemode)
{
  volatile uint16_t *p_reg;

  p_reg = (uint16_t *)hw_usb_get_fifoctr_adr(pipemode);

  (*p_reg) |= RX65N_USB_FIFOCTR_BVAL;
}

/****************************************************************************
 * Function Name   : hw_usb_set_bclr
 * Description     : Set BCLR (CPU Buffer Clear) to the number given; in the
 *                 : FIFOCTR corresponding to the specified PIPEMODE.
 * Arguments       : uint16_t  pipemode   : CUSE/D0DMA/D1DMA
 * Return value    : none
 ****************************************************************************/

static void hw_usb_set_bclr(uint16_t pipemode)
{
  volatile uint16_t *p_reg;

  p_reg = (uint16_t *)hw_usb_get_fifoctr_adr(pipemode);

  *p_reg = RX65N_USB_FIFOCTR_BCLR;
}

/****************************************************************************
 * Function Name   : hw_usb_set_intenb
 * Description     : Bit(s) to be set in INTENB register,
 *                 : enabling the respective USB interrupt(s).
 * Arguments       : uint16_t  data  : Bit pattern: Respective interrupts
 *                 :         with '1' will be enabled.
 * Return value    : none
 ****************************************************************************/

void hw_usb_set_intenb(uint16_t data)
{
  rx65n_usbhost_setbit(RX65N_USB_INTENB0, data);
}

/****************************************************************************
 * Function Name   : hw_usb_set_brdyenb
 * Description     : A bit is set in the specified pipe's BRDYENB, enabling
 *                 : the respective pipe BRDY interrupt(s).
 * Arguments       : uint16_t  pipeno : Pipe number.
 * Return value    : none
 ****************************************************************************/

static void hw_usb_set_brdyenb(uint16_t pipeno)
{
  rx65n_usbhost_setbit(RX65N_USB_BRDYENB, (1 << pipeno));
}

/****************************************************************************
 * Function Name   : hw_usb_clear_brdyenb
 * Description     : Clear the PIPExBRDYE-bit of the specified pipe to
 *                 : prohibit BRDY interrupts of that pipe.
 * Arguments       : uint16_t  pipeno : Pipe number.
 * Return value    : none
 ****************************************************************************/

static void hw_usb_clear_brdyenb(uint16_t pipeno)
{
  rx65n_usbhost_clearbit(RX65N_USB_BRDYENB, (1 << pipeno));
}

/****************************************************************************
 * Function Name   : hw_usb_set_nrdyenb
 * Description     : A bit is set in the specified pipe's NRDYENB, enabling
 *                 : the respective pipe NRDY interrupt(s).
 * Arguments       : uint16_t  pipeno : Pipe number
 * Return value    : none
 ****************************************************************************/

static void hw_usb_set_nrdyenb(uint16_t pipeno)
{
  rx65n_usbhost_setbit(RX65N_USB_NRDYENB, (1 << pipeno));
}

/****************************************************************************
 * Function Name   : hw_usb_clear_nrdyenb
 * Description     : Clear the PIPExNRDYE-bit of the specified pipe to
 *                 : prohibit NRDY interrupts of that pipe.
 * Arguments       : uint16_t  pipeno : Pipe number.
 * Return value    : none
 ****************************************************************************/

static void hw_usb_clear_nrdyenb(uint16_t pipeno)
{
  rx65n_usbhost_clearbit(RX65N_USB_NRDYENB, (1 << pipeno));
}

/****************************************************************************
 * Function Name   : hw_usb_set_bempenb
 * Description     : A bit is set in the specified pipe's BEMPENB enabling
 *                 : the respective pipe's BEMP interrupt(s).
 * Arguments       : uint16_t  pipeno  : Pipe number.
 * Return value    : none
 ****************************************************************************/

static void hw_usb_set_bempenb(uint16_t pipeno)
{
  rx65n_usbhost_setbit(RX65N_USB_BEMPENB, (1 << pipeno));
}

/****************************************************************************
 * Function Name   : hw_usb_clear_bempenb
 * Description     : Clear the PIPExBEMPE-bit of the specified pipe to
 *                 : prohibit BEMP interrupts of that pipe.
 * Arguments       : uint16_t  pipeno : Pipe number.
 * Return value    : none
 ****************************************************************************/

static void hw_usb_clear_bempenb(uint16_t pipeno)
{
  rx65n_usbhost_clearbit(RX65N_USB_BEMPENB, (1 << pipeno));
}

/****************************************************************************
 * Function Name   : hw_usb_clear_sts_sofr
 * Description     : Clear the SOFR-bit (Frame Number Refresh Interrupt
 *                 : Status) of the clear SOF interrupt status.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void hw_usb_clear_sts_sofr(void)
{
  rx65n_usbhost_clearbit(RX65N_USB_INTSTS0, RX65N_USB_INTSTS0_SOFR);
}

/****************************************************************************
 * Function Name   : hw_usb_clear_sts_brdy
 * Description     : Clear the PIPExBRDY status bit of the specified pipe to
 *                 : clear its BRDY interrupt status.
 * Arguments       : uint16_t  pipeno: Pipe number.
 * Return value    : none
 ****************************************************************************/

static void hw_usb_clear_sts_brdy(uint16_t pipeno)
{
  rx65n_usbhost_putreg((~(1 << pipeno)) & RX65N_USB_PIPE_ALL,
  RX65N_USB_BRDYSTS);
}

/****************************************************************************
 * Function Name   : hw_usb_clear_status_nrdy
 * Description     : Clear the PIPExNRDY status bit of the specified pipe to
 *                 : clear its NRDY interrupt status.
 * Arguments       : uint16_t  pipeno: Pipe number.
 * Return value    : none
 ****************************************************************************/

static void hw_usb_clear_status_nrdy(uint16_t pipeno)
{
  rx65n_usbhost_putreg((~(1 << pipeno)) & RX65N_USB_PIPE_ALL,
    RX65N_USB_NRDYSTS);
}

/****************************************************************************
 * Function Name   : hw_usb_clear_status_bemp
 * Description     : Clear the PIPExBEMP status bit of the specified pipe to
 *                 :  its BEMP interrupt status.
 * Arguments       : uint16_t  pipeno: Pipe number.
 * Return value    : none
 ****************************************************************************/

static void hw_usb_clear_status_bemp(uint16_t pipeno)
{
  rx65n_usbhost_putreg((~(1 << pipeno)) & RX65N_USB_PIPE_ALL,
    RX65N_USB_BEMPSTS);
}

/****************************************************************************
 * Function Name   : hw_usb_write_dcpcfg
 * Description     : Specified data is written to DCPCFG register.
 * Arguments       : uint16_t  data   : Setting value
 * Return value    : none
 ****************************************************************************/

static void hw_usb_write_dcpcfg(uint16_t data)
{
  rx65n_usbhost_putreg(data, RX65N_USB_DCPCFG);
}

/****************************************************************************
 * Function Name   : hw_usb_read_dcpmaxp
 * Description     : Returns DCPMAXP register content.
 * Arguments       : none
 * Return value    : DCPMAXP content
 ****************************************************************************/

static uint16_t hw_usb_read_dcpmaxp(void)
{
  return rx65n_usbhost_getreg(RX65N_USB_DCPMAXP);
}

/****************************************************************************
 * Function Name   : hw_usb_write_dcpmxps
 * Description     : Specified data is written to DCPMAXP register.
 * Arguments       : uint16_t  data   : Setting value
 * Return value    : none
 ****************************************************************************/

static void hw_usb_write_dcpmxps(uint16_t data)
{
  rx65n_usbhost_putreg(data, RX65N_USB_DCPMAXP);
}

/****************************************************************************
 * Function Name   : hw_usb_write_pipesel
 * Description     : Specified data is written to PIPESEL register.
 * Arguments       : uint16_t  data   : The value to write.
 * Return value    : none
 ****************************************************************************/

static void hw_usb_write_pipesel(uint16_t data)
{
  rx65n_usbhost_putreg(data, RX65N_USB_PIPESEL);
}

/****************************************************************************
 * Function Name   : hw_usb_read_pipecfg
 * Description     : Returns PIPECFG register content.
 * Arguments       : none
 * Return value    : PIPECFG content
 ****************************************************************************/

static uint16_t hw_usb_read_pipecfg(void)
{
  return rx65n_usbhost_getreg(RX65N_USB_PIPECFG);
}

/****************************************************************************
 * Function Name   : hw_usb_write_pipecfg
 * Description     : Specified data is written to PIPECFG register.
 * Arguments       : uint16_t  data   : Setting value
 * Return value    : none
 ****************************************************************************/

static void hw_usb_write_pipecfg(uint16_t data)
{
  rx65n_usbhost_putreg(data, RX65N_USB_PIPECFG);
}

/****************************************************************************
 * Function Name   : hw_usb_read_pipemaxp
 * Description     : Returns PIPEMAXP register content.
 * Arguments       : none
 * Return value    : PIPEMAXP content
 ****************************************************************************/

static uint16_t hw_usb_read_pipemaxp(void)
{
  return rx65n_usbhost_getreg(RX65N_USB_PIPEMAXP);
}

/****************************************************************************
 * Function Name   : hw_usb_write_pipemaxp
 * Description     : Specified data is written to PIPEMAXP register.
 * Arguments       : uint16_t  data   : Setting value
 * Return value    : none
 ****************************************************************************/

static void hw_usb_write_pipemaxp(uint16_t data)
{
  rx65n_usbhost_putreg(data, RX65N_USB_PIPEMAXP);
}

/****************************************************************************
 * Function Name   : hw_usb_write_pipeperi
 * Description     : Specified data is written to PIPEPERI register.
 * Arguments       : uint16_t  data   : Setting value
 * Return value    : none
 ****************************************************************************/

static void hw_usb_write_pipeperi(uint16_t data)
{
  rx65n_usbhost_putreg(data, RX65N_USB_PIPEPERI);
}

/****************************************************************************
 * Function Name   : hw_usb_read_pipectr
 * Description     : Returns DCPCTR or the specified pipe's PIPECTR register
 *                 : content. The Pipe Control Register returned is
 *                 : determined by the specified pipe number.
 * Arguments       : uint16_t  pipeno : Pipe number.
 * Return value    : PIPExCTR content
 ****************************************************************************/

static uint16_t hw_usb_read_pipectr(uint16_t pipeno)
{
  volatile uint16_t *p_reg;

  /* Control pipe */

  if (pipeno == 0)
    {
      p_reg = (uint16_t *)RX65N_USB_DCPCTR;
    }
  else
    {
      p_reg = (uint16_t *)(RX65N_USB_PIPE1CTR + (pipeno - 1));
    }

  return rx65n_usbhost_getreg(p_reg);
}

/****************************************************************************
 * Function Name   : hw_usb_write_pipectr
 * Description     : Specified data is written to the specified pipe's
 *                 : PIPEPERI register.
 * Arguments       : uint16_t  pipeno : Pipe number
 *                 : uint16_t  data   : Setting value
 * Return value    : none
 ****************************************************************************/

void hw_usb_write_pipectr(uint16_t pipeno, uint16_t data)
{
  volatile uint16_t *p_reg;

  if (USB_PIPE0 == pipeno)
    {
      p_reg = (uint16_t *)RX65N_USB_DCPCTR;
    }
  else
    {
      p_reg = (uint16_t *)RX65N_USB_PIPE1CTR + (pipeno - 1);
    }

  *p_reg = data;
}

/****************************************************************************
 * Function Name   : hw_usb_set_aclrm
 * Description     : The ACLRM-bit (Auto Buffer Clear Mode) is set in the
 *                 : specified pipe's control register.
 * Arguments       : uint16_t  pipeno : Pipe number
 * Return value    : none
 ****************************************************************************/

static void hw_usb_set_aclrm(uint16_t pipeno)
{
  volatile uint16_t *p_reg;
  if (USB_PIPE0 == pipeno)
    {
      p_reg = (uint16_t *)RX65N_USB_DCPCTR;
    }
  else
    {
      p_reg = (uint16_t *)RX65N_USB_PIPE1CTR + (pipeno - 1);
    }

  (*p_reg) |= RX65N_USB_PIPECTR_ACLRM;
}

/****************************************************************************
 * Function Name   : hw_usb_clear_aclrm
 * Description     : Clear the ACLRM bit in the specified pipe's control
 *                 : register to disable Auto Buffer Clear Mode.
 *                 : its BEMP interrupt status.
 * Arguments       : usb_utr_t *ptr   : Pointer to usb_utr_t structure.
 *                 : uint16_t  pipeno : Pipe number
 * Return value    : none
 ****************************************************************************/

static void hw_usb_clear_aclrm(uint16_t pipeno)
{
  volatile uint16_t *p_reg;
  if (USB_PIPE0 == pipeno)
    {
      p_reg = (uint16_t *)RX65N_USB_DCPCTR;
    }
  else
    {
      p_reg = (uint16_t *)RX65N_USB_PIPE1CTR + (pipeno - 1);
    }

  (*p_reg) &= (~RX65N_USB_PIPECTR_ACLRM);
}

/****************************************************************************
 * Function Name   : hw_usb_set_sqclr
 * Description     : The SQCLR-bit (Toggle Bit Clear) is set in the specified
 *                 : pipe's control register. Setting SQSET to 1 makes DATA0
 *                 : the expected data in the pipe's next transfer.
 * Arguments       : uint16_t  pipeno : Pipe number
 * Return value    : none
 ****************************************************************************/

static void hw_usb_set_sqclr(uint16_t pipeno)
{
  volatile uint16_t *p_reg;
  if (USB_PIPE0 == pipeno)
    {
      p_reg = (uint16_t *)RX65N_USB_DCPCTR;
    }
    else
    {
      p_reg = (uint16_t *)RX65N_USB_PIPE1CTR + (pipeno - 1);
    }

  (*p_reg) |= RX65N_USB_PIPECTR_SQCLR;
}

/****************************************************************************
 * Function Name   : hw_usb_set_sqset
 * Description     : The SQSET-bit (Toggle Bit Set) is set in the specified
 *                 : pipe's control register. Setting SQSET to 1 makes DATA1
 *                 : the expected data in the next transfer.
 * Arguments       : usb_utr_t *ptr   : Pointer to usb_utr_t structure.
 *                 : uint16_t  pipeno : Pipe number
 * Return value    : none
 ****************************************************************************/

void hw_usb_set_sqset(uint16_t pipeno)
{
  volatile uint16_t *p_reg;
  if (USB_PIPE0 == pipeno)
    {
      p_reg = (uint16_t *)RX65N_USB_DCPCTR;
    }
  else
    {
      p_reg = (uint16_t *)RX65N_USB_PIPE1CTR + (pipeno - 1);
    }

  (*p_reg) |= RX65N_USB_PIPECTR_SQSET;
}

/****************************************************************************
 * Function Name   : hw_usb_set_pid
 * Description     : Set the specified PID of the specified pipe's
 *                 : DCPCTR/PIPECTR register.
 * Arguments       : uint16_t  pipeno : Pipe number
 *                 : uint16_t  data   : NAK/BUF/STALL.
 * Return value    : none
 ****************************************************************************/

static void hw_usb_set_pid(uint16_t pipeno, uint16_t data)
{
  volatile uint16_t *p_reg;
  if (USB_PIPE0 == pipeno)
    {
      p_reg = (uint16_t *)RX65N_USB_DCPCTR;
    }
  else
    {
      p_reg = (uint16_t *)RX65N_USB_PIPE1CTR + (pipeno - 1);
    }

  (*p_reg) &= (~RX65N_USB_PIPECTR_PID_MASK);
  (*p_reg) |= data;
}

/****************************************************************************
 * Function Name   : hw_usb_clear_pid
 * Description     : Clear the specified PID-bits of the specified pipe's
 *                 : DCPCTR/PIPECTR register.
 * Arguments       : uint16_t  pipeno : Pipe number
 *                 : uint16_t  data  : NAK/BUF/STALL - to be cleared.
 * Return value    : none
 ****************************************************************************/

static void hw_usb_clear_pid(uint16_t pipeno, uint16_t data)
{
  volatile uint16_t *p_reg;
  if (USB_PIPE0 == pipeno)
    {
      p_reg = (uint16_t *)RX65N_USB_DCPCTR;
    }
  else
    {
      p_reg = (uint16_t *)RX65N_USB_PIPE1CTR + (pipeno - 1);
    }

  (*p_reg) &= (~RX65N_USB_PIPECTR_PID_MASK);
}

/****************************************************************************
 * Function Name   : hw_usb_set_trenb
 * Description     : The TRENB-bit (Transaction Counter Enable) is set in
 *                 : the specified pipe's control register, to enable the
 *                 : counter.
 * Arguments       : uint16_t  pipeno : Pipe number
 * Return value    : none
 ****************************************************************************/

static void hw_usb_set_trenb(uint16_t pipeno)
{
  volatile uint16_t *p_reg;

  p_reg = (uint16_t *)RX65N_USB_PIPE1TRE + ((pipeno - 1) * 2);

  (*p_reg) |= RX65N_USB_PIPETRE_TRENB;
}

/****************************************************************************
 * Function Name   : hw_usb_clear_trenb
 * Description     : The TRENB-bit (Transaction Counter Enable) is cleared in
 *                 : the specified pipe's control register, to disable the
 *                 : counter.
 * Arguments       : uint16_t  pipeno : Pipe number
 * Return value    : none
 ****************************************************************************/

static void hw_usb_clear_trenb(uint16_t pipeno)
{
  volatile uint16_t *p_reg;

  p_reg = (uint16_t *)RX65N_USB_PIPE1TRE + ((pipeno - 1) * 2);

  (*p_reg) &= (~RX65N_USB_PIPETRE_TRENB);
}

/****************************************************************************
 * Function Name   : hw_usb_set_trclr
 * Description     : The TRENB-bit (Transaction Counter Clear) is set in the
 *                 : specified pipe's control register to clear the current
 *                 : counter value.
 * Arguments       : uint16_t  pipeno : Pipe number
 * Return value    : none
 ****************************************************************************/

static void hw_usb_set_trclr(uint16_t pipeno)
{
  volatile uint16_t *p_reg;

  p_reg = (uint16_t *)RX65N_USB_PIPE1TRE + ((pipeno - 1) * 2);
  (*p_reg) |= RX65N_USB_PIPETRE_TRCLR;
}

/****************************************************************************
 * Function Name   : hw_usb_write_pipetrn
 * Description     : Specified data is written to the specified pipe's
 *                 : PIPETRN register.
 * Arguments       : uint16_t  pipeno : Pipe number
 *                 : uint16_t  data   : The value to write.
 * Return value    : none
 ****************************************************************************/

static void hw_usb_write_pipetrn(uint16_t pipeno, uint16_t data)
{
  volatile uint16_t *p_reg;

  p_reg = (uint16_t *)RX65N_USB_PIPE1TRN + ((pipeno - 1) * 2);

  *p_reg = data;
}

/****************************************************************************
 * Function Name   : usb_hstd_bchg_enable
 * Description     : Enable BCHG interrupt for the specified USB port.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void usb_hstd_bchg_enable(void)
{
  hw_usb_hclear_sts_bchg();
  hw_usb_hset_enb_bchge();
}

/****************************************************************************
 * Function Name   : usb_hstd_bchg_disable
 * Description     : Disable BCHG interrupt for specified USB port.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void usb_hstd_bchg_disable(void)
{
  hw_usb_hclear_sts_bchg();
  hw_usb_hclear_enb_bchge();
}

/****************************************************************************
 * Function Name   : usb_hstd_set_uact
 * Description     : Start sending SOF to the connected USB device.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void usb_hstd_set_uact(void)
{
  hw_usb_rmw_dvstctr(RX65N_USB_DVSTCTR0_UACT,
                    ((RX65N_USB_DVSTCTR0_USBRST |
                      RX65N_USB_DVSTCTR0_RESUME) |
                     RX65N_USB_DVSTCTR0_UACT));
}

/****************************************************************************
 * Function Name   : usb_hstd_attch_enable
 * Description     : Enable ATTCH (attach) interrupt of the specified USB
 *                 : port.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void usb_hstd_attch_enable(void)
{
  /* ATTCH status Clear */

  hw_usb_hclear_sts_attch();

  /* Attach enable */

  hw_usb_hset_enb_attche();
}

/****************************************************************************
 * Function Name   : usb_hstd_attch_disable
 * Description     : Disable ATTCH (attach) interrupt of the specified USB
 *                 : port.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void usb_hstd_attch_disable(void)
{
  /* ATTCH Clear */

  hw_usb_hclear_sts_attch();

  /* Attach disable */

  hw_usb_hclear_enb_attche();
}

/****************************************************************************
 * Function Name   : usb_hstd_dtch_enable
 * Description     : Enable DTCH (detach) interrupt of the specified USB
 *                 : port.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void usb_hstd_dtch_enable(void)
{
  /* DTCH Clear */

  hw_usb_hclear_sts_dtch();

  /* Detach enable */

  hw_usb_hset_enb_dtche();
}

/****************************************************************************
 * Function Name   : usb_hstd_dtch_disable
 * Description     : Disable DTCH (detach) interrupt of the specified USB
 *                 : port.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void usb_hstd_dtch_disable(void)
{
  /* DTCH Clear */

  hw_usb_hclear_sts_dtch();

  /* Detach disable */

  hw_usb_hclear_enb_dtche();
}

/****************************************************************************
 * Function Name   : usb_hstd_berne_enable
 * Description     : Enable BRDY/NRDY/BEMP interrupt.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void usb_hstd_berne_enable(void)
{
  /* Enable BEMP, NRDY, BRDY */

  hw_usb_set_intenb((RX65N_USB_INTENB0_BEMPE |
  RX65N_USB_INTENB0_NRDYE) |
  RX65N_USB_INTENB0_BRDYE);
}

/****************************************************************************
 * Function Name   : usb_hstd_do_sqtgl
 * Description     : Toggle setting of the toggle-bit for the specified pipe
 *                 : by argument.
 * Arguments       : uint16_t pipe    : Pipe number.
 *                 : uint16_t toggle  : Current toggle status.
 * Return value    : none
 ****************************************************************************/

void usb_hstd_do_sqtgl(uint16_t pipe, uint16_t toggle)
{
  if (USB_MAX_PIPE_NO < pipe)
    {
      return; /* Error */
    }

  /* Check toggle */

  if (RX65N_USB_DCPCTR_SQMON == (toggle & RX65N_USB_DCPCTR_SQMON))
    {
      /* Do pipe SQSET */

      hw_usb_set_sqset(pipe);
    }
  else
    {
      /* Do pipe SQCLR */

      hw_usb_set_sqclr(pipe);
    }
}

/****************************************************************************
 * Function Name   : usb_hstd_write_data_control_pipe
 * Description     : Request the USB FIFO to write data, and manage the size
 *                 : of written data.
 * Arguments       : uint8_t      buf_add     : Buffer address
 *                 : size_t       buf_size    : Size of the data
 * Return value    : uint16_t end_flag
 ****************************************************************************/

uint16_t usb_hstd_write_data_control_pipe(uint8_t * buf_add,
                                          size_t buf_size)
{
  uint16_t size;
  uint16_t count = 0;
  uint16_t buffer;
  uint16_t mxps;
  uint16_t end_flag;

  buffer = usb_cstd_is_set_frdy(USB_PIPE0,
                                RX65N_USB_USING_CFIFO,
                                RX65N_USB_CFIFOSEL_ISEL);

  /* Check error */

  if (USB_FIFOERROR == buffer)
    {
      /* FIFO access error */

      return USB_FIFOERROR;
    }

  /* Data buffer size */

  size = buf_size;

  /* Max Packet Size */

  mxps = usb_cstd_get_maxpacket_size(USB_PIPE0);

  /* Data size check */

  if (buf_size <= (uint32_t)size)
    {
      count = buf_size;

      /* Data count check */

      if (0 == count)
        {
          end_flag = USB_WRITESHRT;
        }
      else if (0 != (count % mxps))
        {
          /* Short Packet is end of write */

          end_flag = USB_WRITESHRT;
        }
      else
        {
          end_flag = USB_WRITING;
        }
    }
  else
    {
      /* Write continues */

      end_flag = USB_WRITING;
      count = mxps;
    }

  buf_add = usb_hstd_write_fifo(count, USB_CUSE, buf_add);
  g_rx65n_edlist[USB_PIPE0].xfrinfo->xfrd += count;

  /* Check data count to remain */

  /* Check if this affects any of the standard requrest commands... */

  if (buf_size <= (uint32_t) size)
    {
      /* Clear data count */

      buffer = hw_usb_read_fifoctr(USB_CUSE); /* Read CFIFOCTR */

      /* Check BVAL */

      if (0u == (buffer & USB_BVAL))
        {
          /* Short Packet */

          hw_usb_set_bval(USB_CUSE);
        }
    }

  return end_flag;
}

/****************************************************************************
 * Function Name   : usb_hstd_write_data
 * Description     : Switch PIPE, request the USB FIFO to write data, and
 *                 : manage the size of written data.
 * Arguments       : uint16_t     pipe        : Pipe no.
 *                 : uint16_t     pipemode    : CUSE/D0DMA/D1DMA
 * Return value    : uint16_t end_flag
 ****************************************************************************/

uint16_t usb_hstd_write_data(uint8_t *buf_add, size_t buf_size,
uint16_t pipe, uint16_t pipemode)
{
  uint16_t size;
  uint16_t count = 0;
  uint16_t buffer;
  uint16_t mxps;
  uint16_t end_flag;

  if (USB_MAX_PIPE_NO < pipe)
    {
      return USB_WRITESHRT; /* Error */
    }

  /* Changes FIFO port by the pipe. */

      buffer = usb_cstd_is_set_frdy(pipe, pipemode, USB_FALSE);

  /* Check error */

  if (USB_FIFOERROR == buffer)
    {
      /* FIFO access error */

      return USB_FIFOERROR;
    }

  /* Data buffer size */

  size = usb_cstd_get_buf_size(pipe);

  /* Max Packet Size */

  mxps = usb_cstd_get_maxpacket_size(pipe);

  /* Data size check */

  if (buf_size <= (uint32_t)size)
    {
      count = buf_size;

      /* Data count check */

      if (0 == count)
        {
          end_flag = USB_WRITESHRT;
        }
      else if (0 != (count % mxps))
        {
          /* Short Packet is end of write */

          end_flag = USB_WRITESHRT;
        }
      else
        {
            {
              /* Write continues */

              end_flag = USB_WRITEEND;
            }
        }
    }
  else
    {
      /* Write continues */

      end_flag = USB_WRITING;
      count = mxps;
    }

  buf_add = usb_hstd_write_fifo(count, pipemode, buf_add);
  g_rx65n_edlist[pipe].xfrinfo->xfrd += count;
  g_rx65n_edlist[pipe].xfrinfo->buffer += count;

  /* Check data count to remain */

  if (buf_size <= (uint32_t) size)
    {
      /* Clear data count */

      buffer = hw_usb_read_fifoctr(pipemode); /* Read CFIFOCTR */

      /* Check BVAL */

      if (0u == (buffer & USB_BVAL))
        {
          /* Short Packet */

          hw_usb_set_bval(pipemode);
        }
    }

  return end_flag;
}

/****************************************************************************
 * Function Name   : usb_hstd_receive_start
 * Description     : Start data reception using CPU/DMA transfer to USB
 *                 : Host/USB device.
 * Arguments       : uint16_t     pipe    : Pipe no.
 * Return value    : none
 ****************************************************************************/

void usb_hstd_receive_start(uint8_t *buffer, size_t buflen, uint8_t pipe)
{
  uint32_t length;
  uint16_t mxps;
  uint16_t useport;

  if (USB_MAX_PIPE_NO < pipe)
    {
      return; /* Error */
    }

  /* Select NAK */

  usb_cstd_set_nak(pipe);

  /* Ignore count clear */

  hw_usb_clear_status_bemp(pipe); /* BEMP Status Clear */
  hw_usb_clear_sts_brdy(pipe);    /* BRDY Status Clear */
  hw_usb_clear_status_nrdy(pipe); /* NRDY Status Clear */
  nrdy_retries[pipe] = 0;         /* Initialize theh NRDY retries to 0 */
  useport = USB_CUSE;

  /* Changes the FIFO port by the pipe. */

  usb_cstd_chg_curpipe(pipe, useport, USB_FALSE);
  mxps = usb_cstd_get_maxpacket_size(pipe); /* Max Packet Size */

  length = buflen;

  if (0u != length)
    {
      /* Data length check */

      if ((0u == (length % mxps))
        {
          /* Set Transaction counter */

          usb_cstd_set_transaction_counter(pipe, length / mxps);
        }
      else
        {
          /* Set Transaction counter */

          usb_cstd_set_transaction_counter(pipe, length / mxps + 1u));
        }
    }

  usb_cstd_set_buf(pipe); /* Set BUF */

  /* Enable Ready Interrupt */

  hw_usb_set_brdyenb(pipe);

  /* Enable Not Ready Interrupt */

  usb_cstd_nrdy_enable(pipe);
}

/****************************************************************************
 * Function Name   : usb_hstd_send_start
 * Description     : Start data transmission using CPUtransfer to USB host/
 *                 : /device.
 * Arguments       : uint8_t pipe  : Pipe no.
 * Return value    : none
 ****************************************************************************/

void usb_hstd_send_start(uint8_t *buffer, size_t buflen, uint8_t pipe)
{
  uint16_t useport;

  if (USB_MAX_PIPE_NO < pipe)
    {
      return; /* Error */
    }

  usb_cstd_set_nak(pipe);            /* Select NAK */
  hw_usb_clear_status_bemp(pipe);    /* BEMP Status Clear */
  hw_usb_clear_sts_brdy(pipe);       /* BRDY Status Clear */
  hw_usb_clear_status_nrdy(pipe);    /* NRDY Status Clear */
  nrdy_retries[pipe] = 0;            /* Initialize theh NRDY retries to 0 */

  /* Pipe number to FIFO port select */

  useport = USB_CUSE;

  /* Changes the FIFO port by the pipe. */

  usb_cstd_chg_curpipe(pipe, useport, USB_FALSE);

  /* Buffer to FIFO data write */

  usb_hstd_buf_to_fifo(buffer, buflen, pipe, useport);
  usb_cstd_set_buf(pipe); /* Set BUF */
}

/****************************************************************************
 * Function Name   : usb_hstd_get_pipe_no
 * Description     : Get PIPE No.
 * Arguments       : uint8_t  type   : Transfer Type.(USB_EP_BULK/USB_EP_INT)
 *                 : uint8_t  dir    : (USB_PIPE_DIR_IN/USB_PIPE_DIR_OUT)
 * Return value    : Pipe no (USB_PIPE1->USB_PIPE9:OK, USB_NULL:Error)
 ****************************************************************************/

uint8_t usb_hstd_get_pipe_no(uint8_t type, uint8_t dir)
{
  uint8_t     pipe_no = USB_NULL;
  uint16_t    pipe;

  if (USB_EP_BULK == type)
    {
      /* BULK PIPE Loop */

      /* WAIT_LOOP */

      if (dir == USB_EP_IN)
        {
          pipe = USB_BULK_PIPE_START;
        }
      else
        {
          pipe = USB_BULK_PIPE_START + 1;
        }

      for (; pipe < (USB_BULK_PIPE_END + 1); pipe = pipe + 2)
        {
          if (USB_FALSE == g_usb_pipe_table[pipe].use_flag)
            {
              /* Check Free pipe */

              pipe_no = pipe; /* Set Free pipe */
              break;
            }
        }
    }

  if (USB_EP_INT == type)
    {
      /* Interrupt PIPE Loop */

      /* WAIT_LOOP */

      for (pipe = USB_INT_PIPE_START; pipe < (USB_INT_PIPE_END +1); pipe++)
        {
          if (USB_FALSE == g_usb_pipe_table[pipe].use_flag)
            {
              /* Check Free pipe */

              pipe_no = pipe; /* Set Free pipe */
              break;
            }
        }
    }

  return pipe_no;
}

/****************************************************************************
 * Function Name   : usb_hstd_read_data_control_pipe
 * Description     : Request to read data from USB FIFO, and manage the size
 *                 : of the data read.
 * Arguments       : None
 * Return value    : USB_READING / USB_READEND / USB_READSHRT / USB_READOVER
 ****************************************************************************/

uint16_t usb_hstd_read_data_control_pipe(void)
{
  uint16_t count;
  uint16_t buffer;
  uint16_t mxps;
  uint16_t dtln;
  uint16_t end_flag;

  /* Changes FIFO port by the pipe. */

  buffer = usb_cstd_is_set_frdy(USB_PIPE0, USB_CUSE, USB_FALSE);

  if (USB_FIFOERROR == buffer)
    {
      /* FIFO access error */

      return USB_FIFOERROR;
      syslog(LOG_INFO, "FIFO ERROR");
    }

  dtln = buffer & RX65N_USB_FIFOCTR_DTLN;

  /* Max Packet Size */

  mxps = usb_cstd_get_maxpacket_size(USB_PIPE0);

  /* now calculate the count */

  count = g_rx65n_tdlist[USB_PIPE0].ed->xfrinfo->buflen -
  g_rx65n_tdlist[USB_PIPE0].ed->xfrinfo->xfrd;

  if (count < dtln)
    {
      /* Buffer Over ? */

      end_flag = USB_READOVER;
      usb_cstd_set_nak(USB_PIPE0); /* Set NAK */
      count = g_rx65n_tdlist[USB_PIPE0].ed->xfrinfo->xfrd;
      g_rx65n_tdlist[USB_PIPE0].ed->xfrinfo->xfrd = dtln;
    }

  else if (count == dtln)
    {
      /* Just Receive Size */

      count = dtln;
      end_flag = USB_READEND;
      usb_cstd_set_nak(USB_PIPE0); /* Set NAK */
    }
  else
    {
      /* Continus Receive data */

      count = dtln;

      end_flag = USB_READING;
      if ((0 == count) || (0 != (count % mxps)))
        {
          /* Null Packet receive */

          end_flag = USB_READSHRT;
          usb_cstd_set_nak(USB_PIPE0); /* Select NAK */
        }
    }

  if (0 == dtln)
    {
      /* 0 length packet */

      /* Clear BVAL */

      hw_usb_set_bclr(USB_CUSE);
    }
  else
    {
      g_rx65n_tdlist[USB_PIPE0].ed->xfrinfo->buffer = usb_hstd_read_fifo(
          count, USB_CUSE, g_rx65n_tdlist[USB_PIPE0].ed->xfrinfo->buffer);
    }

  g_rx65n_tdlist[USB_PIPE0].ed->xfrinfo->xfrd += count;

  return end_flag;
}

/****************************************************************************
 * Function Name   : usb_hstd_read_data
 * Description     : Request to read data from USB FIFO, and manage the size
 *                 : of the data read.
 * Arguments       : uint16_t     pipe     : Pipe no.
 *                 : uint16_t     pipemode : Pipe mode (CFIFO/D0FIFO/D1FIFO)
 * Return value    : USB_READING / USB_READEND / USB_READSHRT / USB_READOVER
 ****************************************************************************/

uint16_t usb_hstd_read_data(uint16_t pipe, uint16_t pipemode)
{
  uint16_t count;
  uint16_t buffer;
  uint16_t mxps;
  uint16_t dtln;
  uint16_t end_flag = 0;

  if (USB_MAX_PIPE_NO < pipe)
    {
      return USB_ERROR; /* Error */
    }

  if (pipe != 0) /* Data transfer for non CTRL pipe */
    {
      buffer = usb_cstd_is_set_frdy(pipe, pipemode, USB_FALSE);

      if (USB_FIFOERROR == buffer)
        {
          return USB_FIFOERROR;
        }

      dtln = buffer & RX65N_USB_FIFOCTR_DTLN;
      mxps = usb_cstd_get_maxpacket_size(pipe);

      /* now calculate the count */

  if (pipe == g_kbdpipe)
    {
      /* For HID KBD read the received data */

      count = dtln;
    }
  else
    {
      count = g_rx65n_edlist[pipe].xfrinfo->buflen -
      g_rx65n_edlist[pipe].xfrinfo->xfrd;
    }

  if (count < dtln)
    {
      /* Buffer Over ? */

      count = dtln;
      end_flag = USB_READOVER;
    }

  else if (count == dtln)
    {
      /* Just Receive Size */

      count = dtln;
      end_flag = USB_READEND;
      usb_cstd_set_nak(pipe);
    }
  else
    {
      /* Continus Receive data */

      count = dtln;

      end_flag = USB_READING;
      if ((0 == count) || (0 != (count % mxps)))
        {
          /* Null Packet receive */

          end_flag = USB_READSHRT;
        }
    }

  usb_hstd_read_fifo(count,
                     pipemode, g_rx65n_edlist[pipe].xfrinfo->buffer);

  g_rx65n_edlist[pipe].xfrinfo->xfrd += count;

  if (pipe == g_kbdpipe)
    {
      /* For HID KBD read pointer should not be incremented */
    }
  else
    {
      g_rx65n_edlist[pipe].xfrinfo->buffer += count;
    }
    }

  return end_flag;
}

/****************************************************************************
 * Function Name   : usb_hstd_data_end
 * Description     : Set USB registers as appropriate after data
 *                 : transmission/reception, and call the callback function
 *                 : as transmission/reception is complete.
 * Arguments       : uint16_t     pipe    : Pipe no.
 *                 : uint16_t     status  : Transfer status type.
 * Return value    : none
 ****************************************************************************/

void usb_hstd_data_end(uint16_t pipe, uint16_t status)
{
  if (USB_MAX_PIPE_NO < pipe)
    {
        return; /* Error */
    }

  /* PID = NAK */

  /* Set NAK */

  usb_cstd_set_nak(pipe);

  /* Disable Interrupt */

  /* Disable Ready Interrupt */

  hw_usb_clear_brdyenb(pipe);

  /* Disable Not Ready Interrupt */

  hw_usb_clear_nrdyenb(pipe);

  /* Disable Empty Interrupt */

  hw_usb_clear_bempenb(pipe);

  /* Disable Transaction count */

  usb_cstd_clr_transaction_counter(pipe);
}

/****************************************************************************
 * Function Name   : usb_hstd_buf_to_fifo
 * Description     : Set USB registers as required to write from data buffer
 *                 : to USBFIFO, to have USB FIFO to write data to bus.
 * Arguments       : uint16_t pipe     : Pipe no.
 *                 : uint16_t useport  : Port no.
 * Return value    : none
 ****************************************************************************/

void usb_hstd_buf_to_fifo(uint8_t *buffer, size_t buflen, uint16_t pipe,
                          uint16_t useport)
{
  uint16_t end_flag;

  if (USB_MAX_PIPE_NO < pipe)
    {
      return; /* Error */
    }

  /* Disable Ready and empty Interrupt */

  hw_usb_clear_brdyenb(pipe);
  hw_usb_clear_bempenb(pipe);

  end_flag = usb_hstd_write_data(buffer, buflen, pipe, useport);

  switch (end_flag)
    {
      case USB_WRITING:

      /* Enable Ready Interrupt */

      hw_usb_set_brdyenb(pipe);

      /* Enable Not Ready Interrupt */

      usb_cstd_nrdy_enable(pipe);

      break;

      case USB_WRITEEND:

      /* End of data write */

      /* continue */

      case USB_WRITESHRT:

      /* End of data write */

      /* Enable Empty Interrupt */

      hw_usb_set_bempenb(pipe);

      /* Enable Not Ready Interrupt */

      usb_cstd_nrdy_enable(pipe);

      break;

      case USB_FIFOERROR:

      /* FIFO access error */

      syslog(LOG_INFO, "### FIFO access error\n");
      usb_hstd_forced_termination(pipe, USB_DATA_ERR);
      break;

      default:
      usb_hstd_forced_termination(pipe, USB_DATA_ERR);
      break;
    }
}

/****************************************************************************
 * Function Name   : usb_hstd_brdy_pipe_process
 * Description     : Search for the PIPE No. that BRDY interrupt occurred,
 *                 : and request data transmission/reception from the PIPE
 * Arguments       : uint16_t     bitsts  : BRDYSTS Register & BRDYENB
 *                 : Register
 * Return value    : none
 ****************************************************************************/

void usb_hstd_brdy_pipe_process(uint16_t bitsts)
{
  uint16_t i;
  uint16_t end_flag;
  size_t data_len;
  uint8_t *data_address;
  uint16_t buffer;
#ifdef CONFIG_USBHOST_ASYNCH
  struct rx65n_usbhost_s *priv = &g_usbhost;
#endif

  for (i = USB_MIN_PIPE_NO; i <= USB_MAX_PIPE_NO; i++)
    {
      if (0 != (bitsts & USB_BITSET(i)))
        {
          /* Clear the Interrupt status bit - clear for both BEMP and BRDY */

          hw_usb_clear_sts_brdy(i);
          hw_usb_clear_status_bemp(i);
          if (RX65N_USB_PIPECFG_DIR == usb_cstd_get_pipe_dir(i))
            {
              /* Buffer to FIFO data write */

              data_len = g_rx65n_edlist[i].xfrinfo->buflen -
                         g_rx65n_edlist[i].xfrinfo->xfrd;
              buffer = usb_cstd_get_pid(i);

              /* MAX packet size error ? */

              if (RX65N_USB_DCPCTR_PIDSTALL == (buffer &
                  RX65N_USB_DCPCTR_PIDSTALL))
                {
                  syslog(LOG_INFO, "### STALL Pipe %d\n", i);
                  usb_hstd_forced_termination(i, USB_DATA_STALL);
                }
              else
                {
                  if (data_len == 0)
                    {
                      syslog(LOG_INFO, "BRDY can NOT be with 0 len\
                             data for pipe\n", i);
                    }

                  /* If still data is present - let the data
                   * transfer coninue
                   *
                   */

                  else
                    {
                      data_address = g_rx65n_edlist[i].xfrinfo->buffer;
                      usb_hstd_buf_to_fifo(data_address, data_len, i,
                                           USB_CUSE);
                      usb_cstd_set_buf(i); /* Set BUF */
                    }
                }
            }
          else
            {
              /* FIFO to Buffer data read */

              end_flag = usb_hstd_read_data(i, USB_CUSE);

              if (end_flag != USB_READING)
                {
#ifdef CONFIG_USBHOST_ASYNCH
      if ((i == g_kbdpipe) &&
         (NULL == g_rx65n_edlist[i].xfrinfo->callback))
#else
      if (i == g_kbdpipe)
#endif
                    {
                      usb_cstd_clr_stall(i);
                    }

                  else
                    {
                      usb_hstd_data_end(i, USB_DATA_OK);
                      g_rx65n_edlist[i].xfrinfo->tdstatus =
                      TD_CC_NOERROR;

#ifdef CONFIG_USBHOST_ASYNCH
                      if ((g_rx65n_edlist[i].xfrinfo != 0)
                          && (g_rx65n_edlist[i].xfrinfo->callback))
                        {
                          hw_usb_write_dcpmxps(USB_DEFPACKET + USB_DEVICE_1);
                          rx65n_usbhost_asynch_completion(priv,
                          &g_rx65n_edlist[i]);
                        }

#endif
                    }

                  nxsem_post(&g_rx65n_edlist[i].wdhsem);
                }
            }
        }
    }
}

/****************************************************************************
 * Function Name   : usb_hstd_nrdy_pipe_process
 * Description     : Search for PIPE No. that occurred NRDY interrupt, and
 *                 : execute the process for PIPE when NRDY interrupt
 *                 : occurred
 * Arguments       : uint16_t  bitsts  : NRDYSTS Register & NRDYENB Register
 * Return value    : none
 ****************************************************************************/

void usb_hstd_nrdy_pipe_process(uint16_t bitsts)
{
  uint16_t i;

  /* WAIT_LOOP */

  for (i = USB_MIN_PIPE_NO; i <= USB_MAX_PIPE_NO; i++)
    {
      if (0 != (bitsts & USB_BITSET(i)))
        {
          hw_usb_clear_status_nrdy(i);
          usb_hstd_nrdy_endprocess(i);
        }
    }
}

/****************************************************************************
 * Function Name   : usb_hstd_bemp_pipe_process
 * Description     : Search for PIPE No. that BEMP interrupt occurred, and
 *                                 : complete data transmission for the PIPE
 * Arguments       : uint16_t bitsts  : BEMPSTS Register & BEMPENB Register
 * Return value    : none
 ****************************************************************************/

void usb_hstd_bemp_pipe_process(uint16_t bitsts)
{
  uint16_t buffer;
  uint16_t i;
  size_t data_len;

  /* WAIT_LOOP */

  for (i = USB_MIN_PIPE_NO; i <= USB_PIPE5; i++)
    {
      if (0 != (bitsts & USB_BITSET(i)))
        {
          /* Clear the Interrupt status bit */

          hw_usb_clear_status_bemp(i);
          hw_usb_clear_sts_brdy(i);

         data_len = g_rx65n_edlist[i].xfrinfo->buflen -
                     g_rx65n_edlist[i].xfrinfo->xfrd;
          buffer = usb_cstd_get_pid(i);

          /* MAX packet size error ? */

          if (RX65N_USB_DCPCTR_PIDSTALL == (buffer &
              RX65N_USB_DCPCTR_PIDSTALL))
            {
              syslog(LOG_INFO, "### STALL Pipe %d\n", i);
              usb_hstd_forced_termination(i, USB_DATA_STALL);
            }
          else
            {
              if (data_len == 0)
                {
                  usb_hstd_data_end(i, USB_DATA_OK);
                  g_rx65n_edlist[i].xfrinfo->tdstatus = TD_CC_NOERROR;

                  /* Release the semaphore for this pipe */

                 nxsem_post(&g_rx65n_edlist[i].wdhsem);
                }

              else
                {
                  syslog(LOG_INFO,
                         "BEMP can NOT be with %d len data for pipe\n",
                         data_len, i);
                }
            }
        }
    }
}

/****************************************************************************
 * Function Name   : usb_hstd_get_pipe_peri_value
 * Description     : Get value to be set in PIPEPERI
 * Arguments       : uint8_t   binterval : bInterval for ENDPOINT Descriptor
 * Return value    : Value for set PIPEPERI
 ****************************************************************************/

uint16_t usb_hstd_get_pipe_peri_value(uint8_t binterval)
{
  uint16_t    pipe_peri = USB_NULL;
  uint16_t    work1;
  uint16_t    work2;

  /* Set interval counter */

  if (0 != binterval)
    {
      /* FS/LS interrupt */

      /* The time of the FS/LS interrupt forwarding of the interval is
       * specified by the unit of ms. It is necessary to calculate to
       * specify USB-IP by the n-th power of two. NAK rate of the control
       * and the bulk transfer doesn't correspond.
           */

      work1 = binterval;
      work2 = 0;
      for (; work1 != 0; work2++ )
        {
          work1 >>= 1;
        }

      if (0 != work2)
        {
          /* Interval time */

          pipe_peri |= (work2 - 1);
        }
    }

  return pipe_peri;
}

/****************************************************************************
 * Function Name   : usb_hstd_chk_attach
 * Description     : Checks whether USB Device is attached or not and return
 *                 : USB speed of USB Device
 * Arguments       : none
 * Return value    : uint16_t   : connection status
 *                 :            : (USB_ATTACHF/USB_ATTACHL/USB_DETACH/USB_OK)
 * Note            : Please change for your SYSTEM
 ****************************************************************************/

uint16_t usb_hstd_chk_attach(void)
{
  uint16_t buf[3];
  usb_hstd_read_lnst(buf);

  if (0 == (buf[1] & RX65N_USB_DVSTCTR0_RHST))
    {
      if (RX65N_USB_SYSSTS0_LNST_FS_JSTS == (buf[0] & 3))
        {
          /* High/Full speed device */

          return USB_ATTACHF;
        }
      else if (RX65N_USB_SYSSTS0_LNST_LS_JSTS == (buf[0] & 3))
        {
          /* Low speed device */

          return USB_ATTACHL;
        }
      else if (RX65N_USB_SYSSTS0_LNST_SE0 == (buf[0] & 3))
        {
          syslog(LOG_INFO, "Debug: Detach device\n");
        }
      else
        {
          syslog(LOG_INFO, "Attach unknown speed device\n");
        }
    }
  else
    {
      syslog(LOG_INFO, "Already device attached\n");
      return 0;
    }

  return USB_DETACH;
}

/****************************************************************************
 * Function Name   : usb_hstd_chk_clk
 * Description     : Checks SOF sending setting when USB Device is detached
 *                 : or suspended , BCHG interrupt enable setting and
 *: clock stop processing
 * Arguments       : uint16_t event    : device state
 * Return value    : none
 ****************************************************************************/

void usb_hstd_chk_clk(uint16_t event)
{
  if ((USB_DETACH == event) || (USB_SUSPEND == event))
    {
      usb_hstd_chk_sof();

      /* Enable port BCHG interrupt */

      usb_hstd_bchg_enable();
    }
}

/****************************************************************************
 * Function Name   : usb_hstd_detach_process
 * Description     : Handles the require processing when USB device is
 *                 : detached (Data transfer forcibly termination processing
 *: to the connected USB Device, the clock supply stop setting and
 *: the USB interrupt disable setting etc)
 * Arguments       : none
 * Return value    : Device speed
 ****************************************************************************/

uint16_t usb_hstd_detach_process(void)
{
  uint16_t connect_inf;
  uint16_t i;

  /* ATTCH interrupt disable */

  usb_hstd_attch_disable();

  /* DTCH  interrupt disable */

  usb_hstd_dtch_disable();
  usb_hstd_bchg_disable();

  /* Check if control pipe in use */

  if (RX65N_USB_PIPECTR_PIDBUF == usb_cstd_get_pid(USB_PIPE0))
    {
      /* End of data transfer (IN/OUT) */

      usb_hstd_forced_termination(USB_PIPE0, USB_DATA_STOP);
    }

  usb_cstd_clr_pipe_cnfg(USB_PIPE0);

  /* WAIT_LOOP */

  for (i = USB_MIN_PIPE_NO; i <= USB_MAX_PIPE_NO; i++)
    {
      /* Not control transfer */

      /* Is this pipe used */

      if (g_usb_pipe_table[i].use_flag == USB_TRUE)
        {
          /* PID=BUF ? */

          if (RX65N_USB_PIPECTR_PIDBUF == usb_cstd_get_pid(i))
            {
              /* End of data transfer (IN/OUT) */

              usb_hstd_forced_termination(i, USB_DATA_STOP);
            }

          usb_cstd_clr_pipe_cnfg(i);
        }
    }

  /* Decide USB Line state (ATTACH) */

  connect_inf = usb_hstd_chk_attach();
  switch (connect_inf)
    {
      case USB_ATTACHL:
      usb_hstd_attach(connect_inf);
      break;

      case USB_ATTACHF:
      usb_hstd_attach(connect_inf);
      break;

      case USB_DETACH:

      /* USB detach */

      usb_hstd_detach();

      /* Check clock */

      usb_hstd_chk_clk(USB_DETACH);
      break;

      default:

      /* USB detach */

      usb_hstd_detach();

      /* Check clock */

      usb_hstd_chk_clk(USB_DETACH);
      break;
    }

  return connect_inf;
}

/****************************************************************************
 * Function Name   : usb_hstd_read_lnst
 * Description     : Reads LNST register two times, checks whether these
 *                 : values are equal and returns the value of DVSTCTR
 *                 : register that correspond to the port specified by 2nd
 *                 : argument.
 * Arguments       : uint16_t *buf : Pointer to the buffer to store DVSTCTR
 *                                 : register
 * Return value    : none
 * Note            : Please change for your SYSTEM
 ****************************************************************************/

void usb_hstd_read_lnst(uint16_t *buf)
{
  /* WAIT_LOOP */

  if (g_attached)
    {
      do
        {
          buf[0] = hw_usb_read_syssts();

          /* 30ms wait */

          up_mdelay(30);
          buf[1] = hw_usb_read_syssts();
          if ((buf[0] & RX65N_USB_SYSSTS0_LNST) ==
          (buf[1] & RX65N_USB_SYSSTS0_LNST))
            {
              /* 20ms wait */

              up_mdelay(20);
              buf[1] = hw_usb_read_syssts();
            }
        }
      while (((buf[0] & RX65N_USB_SYSSTS0_LNST) !=
            (buf[1] & RX65N_USB_SYSSTS0_LNST)) || (((buf[0] & 3) == 0)));
  buf[1] = hw_usb_read_dvstctr();
    }

  if (g_detached)
    {
      do
        {
          buf[0] = hw_usb_read_syssts();

          /* 30ms wait */

          up_mdelay(30);
          buf[1] = hw_usb_read_syssts();
          if ((buf[0] & RX65N_USB_SYSSTS0_LNST) ==
             (buf[1] & RX65N_USB_SYSSTS0_LNST))
            {
              /* 20ms wait */

              up_mdelay(20);
              buf[1] = hw_usb_read_syssts();
            }
        }
      while (((buf[0] & RX65N_USB_SYSSTS0_LNST) !=
            (buf[1] & RX65N_USB_SYSSTS0_LNST)));
  buf[1] = hw_usb_read_dvstctr();
    }
}

/****************************************************************************
 * Function Name   : usb_hstd_attach_process
 * Description     : Interrupt disable setting when USB Device is attached
 *                 : handles the required interrupt disable setting etc when
 *                 : USB device is attached.
 * Arguments       : none
 * Return value    : Speed of the device
 * Note            : Please change for your SYSTEM
 ****************************************************************************/

uint16_t usb_hstd_attach_process(void)
{
  uint16_t connect_inf;

  /* ATTCH interrupt disable */

  usb_hstd_attch_disable();

  /* DTCH  interrupt disable */

  usb_hstd_dtch_disable();
  usb_hstd_bchg_disable();

  /* Decide USB Line state (ATTACH) */

  connect_inf = usb_hstd_chk_attach();
  switch (connect_inf)
    {
      case USB_ATTACHL:
      usb_hstd_attach(connect_inf);
      break;

      case USB_ATTACHF:
      usb_hstd_attach(connect_inf);
      break;

      case USB_DETACH:

      /* USB detach */

      usb_hstd_detach();

      /* Check clock */

      usb_hstd_chk_clk(USB_DETACH);
      break;

      default:
      usb_hstd_attach(USB_ATTACHF);
      break;
    }

  return connect_inf;
}

/****************************************************************************
 * Function Name   : usb_hstd_chk_sof
 * Description     : Checks whether SOF is sended or not
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void usb_hstd_chk_sof(void)
{
  up_mdelay(1);
}

/****************************************************************************
 * Function Name   : usb_hstd_bus_reset
 * Description     : Setting USB register when BUS Reset
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void usb_hstd_bus_reset(void)
{
  uint16_t buf;
  uint16_t i;

  /* USBRST=1, UACT=0 */

  hw_usb_rmw_dvstctr(RX65N_USB_DVSTCTR0_USBRST,
  (RX65N_USB_DVSTCTR0_USBRST | RX65N_USB_DVSTCTR0_UACT));

  /* Wait 50ms */

  up_mdelay(50); /* usb_cpu_delay_xms(50); */

  /* USBRST=0, RESUME=0, UACT=1 */

  usb_hstd_set_uact();

  /* Wait 10ms or more (USB reset recovery) */

  up_mdelay(20); /* usb_cpu_delay_xms(20); */

  /* WAIT_LOOP */

  /* If DVSTCTR0 last 3 bits i.e. RHST bit fields are 1xx.
   * Then the USB reset is still in progress
   */

  for (i = 0, buf = 0x04; (i < 3) && (0x04 == buf); ++i)
    {
      /* DeviceStateControlRegister - ResetHandshakeStatusCheck */

      buf = hw_usb_read_dvstctr() & RX65N_USB_DVSTCTR0_RHST;
      if (0x04 == buf)
        {
          /* Wait */

          up_mdelay(10);
        }
    }

  /* 30ms wait */

  up_mdelay(30);
}

/****************************************************************************
 * Function Name   : usb_hstd_write_fifo
 * Description     : Write specified amount of data to specified USB FIFO.
 * Arguments       : uint16_t count      : Write size
 *                 : uint16_t pipemode   : The mode of CPU/DMA(D0)/DMA(D1).
 *                 : uint16_t *write_p   : Address of buffer of data to write
 * Return value    : The incremented address of last argument (write_p).
 ****************************************************************************/

uint8_t *usb_hstd_write_fifo(uint16_t count, uint16_t pipemode,
  uint8_t *write_p)
{
  uint16_t even;
    {
      /* WAIT_LOOP */

      for (even = count >> 1; 0 != even; --even)
        {
          /* 16bit access */

          hw_usb_write_fifo16(pipemode, *((uint16_t *)write_p));

          /* Renewal write pointer */

          write_p += sizeof(uint16_t);
        }

      if ((count & 0x0001u) != 0u)
        {
          /* count == odd */

          /* Change FIFO access width */

          hw_usb_set_mbw(pipemode, USB_MBW_8);

          /* FIFO write */

          hw_usb_write_fifo8(pipemode, *write_p);

          /* Return FIFO access width */

          hw_usb_set_mbw(pipemode, USB_MBW_16);

          /* Renewal write pointer */

          write_p++;
        }
    }

  return write_p;
}

/****************************************************************************
 * Function Name   : usb_hstd_read_fifo
 * Description     : Read specified buffer size from the USB FIFO.
 * Arguments       : uint16_t count   :Read size
 *                 : uint16_t pipemode:The mode of CPU/DMA(D0)/DMA(D1).
 *                 : uint16_t *read_p :Address of buffer to store the read
 *                 : data
 * Return value    : Pointer to a buffer that contains the data to be read
 *                 : next.
 ****************************************************************************/

uint8_t *usb_hstd_read_fifo(uint16_t count, uint16_t pipemode,
  uint8_t *read_p)
{
  uint16_t even;
  uint32_t odd_byte_data_temp;

  /* WAIT_LOOP */

  for (even = count >> 1; 0 != even; --even)
    {
      /* 16bit FIFO access */

      *(uint16_t *)read_p = hw_usb_read_fifo16(pipemode);

      /* Renewal read pointer */

      read_p += sizeof(uint16_t);
    }

  if ((count & 0x0001) != 0)
    {
      /* 16bit FIFO access */

      odd_byte_data_temp = hw_usb_read_fifo16(pipemode);

      /* Condition compilation by the difference of the little endian */

      *read_p = (uint8_t) (odd_byte_data_temp & 0x00ff);

      /* Renewal read pointer */

      read_p += sizeof(uint8_t);
    }

  return read_p;
}

/****************************************************************************
 * Function Name   : usb_hstd_forced_termination
 * Description     : Terminate data transmission and reception.
 * Arguments       : uint16_t     pipe    : Pipe Number
 *                 : uint16_t     status  : Transfer status type
 * Return value    : none
 * Note            : In the case of timeout status, it does not call
 *                 : back.
 ****************************************************************************/

void usb_hstd_forced_termination(uint16_t pipe, uint16_t status)
{
  uint16_t buffer;

  /* PID = NAK */

  /* Set NAK */

  usb_cstd_set_nak(pipe);

  /* Disable Ready Interrupt */

  hw_usb_clear_brdyenb(pipe);

  /* Disable Not Ready Interrupt */

  hw_usb_clear_nrdyenb(pipe);

  /* Disable Empty Interrupt */

  hw_usb_clear_bempenb(pipe);

  usb_cstd_clr_transaction_counter(pipe);

  /* Clear CFIFO-port */

  buffer = hw_usb_read_fifosel(USB_CUSE);
  if ((buffer & USB_CURPIPE) == pipe)
    {
      /* Changes the FIFO port by the pipe. */

      usb_cstd_chg_curpipe(USB_PIPE0, USB_CUSE, USB_FALSE);
    }

#ifdef DMA_DTC_NOT_ENABLED
#if ((USB_CFG_DTC == USB_CFG_ENABLE) || (USB_CFG_DMA == USB_CFG_ENABLE))

  /* Clear D0FIFO-port */

  buffer = hw_usb_read_fifosel(ptr, USB_D0USE);
  if ((buffer & USB_CURPIPE) == pipe)
    {
      /* Changes the FIFO port by the pipe. */

      usb_cstd_chg_curpipe(ptr, USB_PIPE0, USB_D0USE, USB_FALSE);
    }

  /* Clear D1FIFO-port */

  buffer = hw_usb_read_fifosel(ptr, USB_D1USE);
  if ((buffer & USB_CURPIPE) == pipe)
    {
      /* Changes the FIFO port by the pipe. */

      usb_cstd_chg_curpipe(ptr, USB_PIPE0, USB_D1USE, USB_FALSE);
    }

#endif /* ((USB_CFG_DTC==USB_CFG_ENABLE)||(USB_CFG_DMA==USB_CFG_ENABLE)) */
#endif /* DMA_DTC_NOT_ENABLED */

  /* Do Aclr */

  usb_cstd_do_aclrm(pipe);
}

/****************************************************************************
 * Function Name   : usb_hstd_nrdy_endprocess
 * Description     : NRDY interrupt processing. (Forced termination of
 *                 : data transmission and reception of specified pipe.)
 * Arguments       : uint16_t     pipe        : Pipe No
 * Return value    : none
 * Note            : none
 ****************************************************************************/

void usb_hstd_nrdy_endprocess(uint16_t pipe)
{
  uint16_t buffer;

  /* Host Function */

  hw_usb_write_pipesel(pipe);      /* Select the pipe */
  buffer = usb_cstd_get_pid(pipe); /* Read the PID of selected pipe */
  usb_cstd_set_nak(pipe);          /* Set PIPE to NAK. Stop sending tokens. */

  /* STALL ? */

  if (RX65N_USB_DCPCTR_PIDSTALL == (buffer & RX65N_USB_DCPCTR_PIDSTALL))
    {
      usb_hstd_forced_termination(pipe, USB_DATA_STALL);
    }
  else
    {
      /* Wait for About 60ns */

      buffer = hw_usb_read_syssts();

      if (nrdy_retries[pipe] >= 1)
        {
          /* Data Device Ignore X 3 call back */

          /* End of data transfer */

          usb_hstd_forced_termination(pipe, USB_DATA_TMO);

          g_rx65n_edlist[pipe].xfrinfo->tdstatus = TD_CC_DEVNOTRESPONDING;

          /* Release the semaphore for this pipe */

          nxsem_post(&g_rx65n_edlist[pipe].wdhsem);
        }
      else
        {
          nrdy_retries[pipe]++; /* Increment and note error count */

          /* 5ms wait */

          up_mdelay(1);

          /* PIPEx Data Retry */

          usb_hstd_data_end(pipe, USB_DATA_TMO);
          hw_usb_set_bclr(USB_CUSE); /* Clear Buffer on CPU side */
          g_rx65n_edlist[pipe].xfrinfo->tdstatus = TD_CC_DEVNOTRESPONDING;

          /* 5ms wait */

          usb_cstd_pipe_init(pipe);

          /* Release the semaphore for this pipe */

          nxsem_post(&g_rx65n_edlist[pipe].wdhsem);
        }
    }
}

/****************************************************************************
 * Function Name   : usb_hstd_bus_int_disable
 * Description     : Disable USB Bus Interrupts OVRCR, ATTCH, DTCH, and BCHG.
 * Arguments       : none
 * Return          : none
 ****************************************************************************/

void usb_hstd_bus_int_disable(void)
{
  /* ATTCH interrupt disable */

  usb_hstd_attch_disable();

  /* DTCH     interrupt disable */

  usb_hstd_dtch_disable();

  /* BCHG     interrupt disable */

  usb_hstd_bchg_disable();
}

/****************************************************************************
 * Function Name   : usb_hstd_attach
 * Description     : Set USB registers as required when USB device is
 *                 : attached, and notify MGR (manager) task that attach
 *                 : event occurred.
 * Arguments       : uint16_t     result  : Result.
 * Return value    : none
 ****************************************************************************/

void usb_hstd_attach(uint16_t result)
{
  /* DTCH  interrupt enable */

  usb_hstd_dtch_enable();

  /* Interrupt Enable */

  usb_hstd_berne_enable();

  usb_hstd_bus_reset();
}

/****************************************************************************
 * Function Name   : usb_hstd_detach
 * Description     : Set USB register as required when USB device is detached
 *                 : and notify MGR (manager) task that detach occurred.
 * Arguments       : none
 * Return value    : none
 ****************************************************************************/

void usb_hstd_detach(void)
{
  /* DVSTCTR clear */

  hw_usb_clear_dvstctr(RX65N_USB_DVSTCTR0_RWUPE |
                       RX65N_USB_DVSTCTR0_USBRST |
                       RX65N_USB_DVSTCTR0_RESUME |
                       RX65N_USB_DVSTCTR0_UACT);

  /* ATTCH interrupt enable */

  usb_hstd_attch_enable();
}

/****************************************************************************
 * Function Name   : usb_cstd_get_buf_size
 * Description     : Return buffer size, or max packet size, of specified
 *                 : pipe.
 * Arguments       : uint16_t pipe     : Pipe number.
 * Return value    : uint16_t          : FIFO buffer size or max packet size.
 ****************************************************************************/

static uint16_t usb_cstd_get_buf_size(uint16_t pipe)
{
  uint16_t size = 0;
  uint16_t buffer;

  if (USB_PIPE0 == pipe)
    {
      /* Not continuation transmit */

      buffer = hw_usb_read_dcpmaxp();

      /* Max Packet Size */

      size = buffer & RX65N_USB_DCPMAXP_MXPS_MASK;
    }
  else
    {
      /* Pipe select */

      hw_usb_write_pipesel(pipe);

      /* Read the maximum packet size of selected pipe */

      buffer = hw_usb_read_pipemaxp();

      /* Max Packet Size */

      size = buffer & RX65N_USB_PIPEMAXP_MXPSMASK;
    }

  return size;
}

/****************************************************************************
 * Function Name   : usb_cstd_pipe_init
 * Description     : Initialization of registers associated with specified
 *                 : pipe.
 * Arguments       : uint16_t pipe     : Pipe Number
 * Return value    : none
 ****************************************************************************/

static void usb_cstd_pipe_init(uint16_t pipe)
{
  /* Interrupt Disable */

  /* Ready         Int Disable */

  hw_usb_clear_brdyenb(pipe);

  /* NotReady      Int Disable */

  hw_usb_clear_nrdyenb(pipe);

  /* Empty/SizeErr Int Disable */

  hw_usb_clear_bempenb(pipe);

  /* PID=NAK & clear STALL */

  usb_cstd_clr_stall(pipe);

  /* PIPE Configuration */

  hw_usb_write_pipesel(pipe);

  rx65n_usbhost_putreg(pipe, RX65N_USB_PIPESEL);
  rx65n_usbhost_putreg(g_usb_pipe_table[pipe].pipe_cfg,
  RX65N_USB_PIPECFG);
  rx65n_usbhost_putreg(g_usb_pipe_table[pipe].pipe_maxp,
  RX65N_USB_PIPEMAXP);
  rx65n_usbhost_putreg(g_usb_pipe_table[pipe].pipe_peri,
  RX65N_USB_PIPEPERI);

  /* FIFO buffer DATA-PID initialized */

  hw_usb_write_pipesel(USB_PIPE0);

  /* SQCLR */

  hw_usb_set_sqclr(pipe);

  /* ACLRM */

  usb_cstd_do_aclrm(pipe);

  /* Interrupt status clear */

  /* Ready         Int Clear */

  hw_usb_clear_sts_brdy(pipe);

  /* NotReady      Int Clear */

  hw_usb_clear_status_nrdy(pipe);

  /* Empty/SizeErr Int Clear */

  hw_usb_clear_status_bemp(pipe);
}

/****************************************************************************
 * Function Name   : usb_cstd_clr_pipe_cnfg
 * Description     : Clear specified pipe configuration register.
 * Arguments       : uint16_t pipe_no  : pipe number
 * Return value    : none
 ****************************************************************************/

static void usb_cstd_clr_pipe_cnfg(uint16_t pipe_no)
{
  /* PID=NAK & clear STALL */

  usb_cstd_clr_stall(pipe_no);

  /* Interrupt disable */

  /* Ready         Int Disable */

  hw_usb_clear_brdyenb(pipe_no);

  /* NotReady      Int Disable */

  hw_usb_clear_nrdyenb(pipe_no);

  /* Empty/SizeErr Int Disable */

  hw_usb_clear_bempenb(pipe_no);

  /* PIPE Configuration */

  usb_cstd_chg_curpipe(USB_PIPE0, USB_CUSE, USB_FALSE);
  hw_usb_write_pipesel(pipe_no);
  hw_usb_write_pipecfg(0);

  hw_usb_write_pipemaxp(0);
  hw_usb_write_pipeperi(0);
  hw_usb_write_pipesel(0);

  /* FIFO buffer DATA-PID initialized */

  /* SQCLR */

  hw_usb_set_sqclr(pipe_no);

  /* ACLRM */

  usb_cstd_do_aclrm(pipe_no);

  usb_cstd_clr_transaction_counter(pipe_no);

  /* Interrupt status clear */

  /* Ready         Int Clear */

  hw_usb_clear_sts_brdy(pipe_no);

  /* NotReady      Int Clear */

  hw_usb_clear_status_nrdy(pipe_no);

  /* Empty/SizeErr Int Clear */

  hw_usb_clear_status_bemp(pipe_no);
}

/****************************************************************************
 * Function Name   : usb_cstd_set_nak
 * Description     : Set up to NAK the specified pipe.
 * Arguments       : uint16_t pipe     : Pipe Number
 * Return value    : none
 ****************************************************************************/

static void usb_cstd_set_nak(uint16_t pipe)
{
  uint16_t buf;
  uint16_t n;

  /* Set NAK */

  hw_usb_clear_pid(pipe, USB_PID_BUF);

  /* The state of PBUSY continues while transmitting the packet
   * when it is a detach. 1ms comes off when leaving because the
   * packet duration might not exceed 1ms.
   */

  /* Whether it is PBUSY release or 1ms passage can be judged. */

  /* WAIT_LOOP */

  for (n = 0; n < 0xffffu; ++n)
    {
      /* PIPE control reg read */

      buf = hw_usb_read_pipectr(pipe);
      if (0 == (buf & RX65N_USB_DCPCTR_PID_MASK))
        {
          n = 0xfffeu;
        }
    }
}

/****************************************************************************
 * Function Name   : usb_cstd_is_set_frdy
 * Description     : Changes the specified FIFO port by the specified pipe.
 * Arguments       : uint16_t pipe     : Pipe Number
 *                 : uint16_t fifosel  : FIFO select
 *                 : uint16_t isel     : ISEL bit status
 * Return value    : FRDY status
 ****************************************************************************/

static uint16_t usb_cstd_is_set_frdy(uint16_t pipe, uint16_t fifosel,
                                     uint16_t isel)
{
  uint16_t buffer;
  uint16_t i;

  /* Changes the FIFO port by the pipe. */

  usb_cstd_chg_curpipe(pipe, fifosel, isel);

  /* WAIT_LOOP */

  for (i = 0; i < 4; i++)
    {
      buffer = hw_usb_read_fifoctr(fifosel);

      if (RX65N_USB_FIFOCTR_FRDY == (buffer &
          RX65N_USB_FIFOCTR_FRDY))
        {
          return buffer;
        }

      buffer = hw_usb_read_syscfg();
      buffer = hw_usb_read_syssts();
      nxsig_usleep(1);
    }

  return RX65N_USB_FIFO_ERROR;
}

/****************************************************************************
 * Function Name   : usb_cstd_chg_curpipe
 * Description     : Switch FIFO and pipe number.
 * Arguments  : uint16_t pipe     : Pipe number.
 *            : uint16_t fifosel  : FIFO selected (CPU, D0, D1..)
 *            : uint16_t isel : CFIFO Port Access Direction.
 *            : (Pipe1 to 9:Set to 0)
 * Return value    : none
 ****************************************************************************/

void usb_cstd_chg_curpipe(uint16_t pipe, uint16_t fifosel, uint16_t isel)
{
  uint16_t buffer;

  /* Select FIFO */

  switch (fifosel)
    {
      /* CFIFO use */

      case RX65N_USB_USING_CFIFO:

      /* ISEL=1, CURPIPE=0 */

      hw_usb_rmw_fifosel(USB_CUSE, ((USB_RCNT | isel) | pipe),
                        ((USB_RCNT | USB_ISEL) | USB_CURPIPE));

      /* WAIT_LOOP */

      do
        {
          buffer = hw_usb_read_fifosel(USB_CUSE);
        }

      while ((buffer & (USB_ISEL | USB_CURPIPE)) != (isel | pipe));
      break;

#ifdef DTC_DMA_ENABLED
#if ((USB_CFG_DTC == USB_CFG_ENABLE) || (USB_CFG_DMA == USB_CFG_ENABLE))

      /* D0FIFO use */

      case USB_D0USE:

      /* D1FIFO use */

      case USB_D1USE:

      /* DxFIFO pipe select */

      hw_usb_set_curpipe(fifosel, pipe);

      /* WAIT_LOOP */

      do
        {
          buffer = hw_usb_read_fifosel(fifosel);
        }
      while ((buffer & USB_CURPIPE) != pipe);
      break;

#endif /* ((USB_CFG_DTC==USB_CFG_ENABLE||(USB_CFG_DMA==USB_CFG_ENABLE))*/
#endif /* DTC_DMA_ENABLED */

      default:
      break;
    }
}

/****************************************************************************
 * Function Name   : usb_cstd_set_transaction_counter
 * Description     : Set specified Pipe Transaction Counter Register.
 * Arguments       : uint16_t trnreg   : Pipe number
 *                 : uint16_t trncnt   : Transaction counter
 * Return value    : none
 ****************************************************************************/

static void usb_cstd_set_transaction_counter(uint16_t trnreg,
                                             uint16_t trncnt)
{
  hw_usb_set_trclr(trnreg);
  hw_usb_write_pipetrn(trnreg, trncnt);
  hw_usb_set_trenb(trnreg);
}

/****************************************************************************
 * Function Name   : usb_cstd_clr_transaction_counter
 * Description     : Clear specified Pipe Transaction Counter Register.
 * Arguments       : uint16_t trnreg   : Pipe Number
 * Return value    : none
 ****************************************************************************/

static void usb_cstd_clr_transaction_counter(uint16_t trnreg)
{
  hw_usb_clear_trenb(trnreg);
  hw_usb_set_trclr(trnreg);
}

/****************************************************************************
 * Function Name   : usb_cstd_nrdy_enable
 * Description     : Enable NRDY interrupt of the specified pipe.
 * Arguments       : uint16_t pipe  : Pipe number.
 * Return value    : none
 ****************************************************************************/

static void usb_cstd_nrdy_enable(uint16_t pipe)
{
  if (USB_MAX_PIPE_NO < pipe)
    {
      return; /* Error */
    }

  /* Enable NRDY */

  hw_usb_set_nrdyenb(pipe);
}

/****************************************************************************
 * Function Name   : usb_cstd_get_pid
 * Description     : Fetch specified pipe's PID.
 * Arguments       : uint16_t pipe  : Pipe number.
 * Return value    : uint16_t PID-bit status
 ****************************************************************************/

static uint16_t usb_cstd_get_pid(uint16_t pipe)
{
  uint16_t buf;

  if (USB_MAX_PIPE_NO < pipe)
    {
      return USB_NULL; /* Error */
    }

  /* PIPE control reg read */

  buf = hw_usb_read_pipectr(pipe);
  return buf & RX65N_USB_DCPCTR_PID_MASK;
}

/****************************************************************************
 * Function Name   : usb_cstd_get_maxpacket_size
 * Description     : Fetch MaxPacketSize of the specified pipe.
 * Arguments       : uint16_t pipe  : Pipe number.
 * Return value    : uint16_t MaxPacketSize
 ****************************************************************************/

static uint16_t usb_cstd_get_maxpacket_size(uint16_t pipe)
{
  uint16_t size;
  uint16_t buffer;

  if (USB_MAX_PIPE_NO < pipe)
    {
      return USB_NULL; /* Error */
    }

  if (USB_PIPE0 == pipe)
    {
      buffer = hw_usb_read_dcpmaxp();
    }
  else
    {
      /* Pipe select */

      hw_usb_write_pipesel(pipe);
      buffer = hw_usb_read_pipemaxp();
    }

  /* Max Packet Size */

  size = (buffer & RX65N_USB_DCPMAXP_MXPS_MASK);

  return size;
}

/****************************************************************************
 * Function Name   : usb_cstd_get_pipe_dir
 * Description     : Get PIPE DIR
 * Arguments       : uint16_t pipe  : Pipe number.
 * Return value    : uint16_t pipe direction.
 ****************************************************************************/

static uint16_t usb_cstd_get_pipe_dir(uint16_t pipe)
{
  uint16_t buffer;

  if (USB_MAX_PIPE_NO < pipe)
    {
      return USB_NULL; /* Error */
    }

  /* Pipe select */

  hw_usb_write_pipesel(pipe);

  /* Read Pipe direction */

  buffer = hw_usb_read_pipecfg();
  return buffer & RX65N_USB_PIPECFG_DIR;
}

/****************************************************************************
 * Function Name   : usb_cstd_do_aclrm
 * Description     : Set the ACLRM-bit (Auto Buffer Clear Mode) of the
 *                 : specified pipe.
 * Arguments       : uint16_t pipe  : Pipe number.
 * Return value    : none
 ****************************************************************************/

static void usb_cstd_do_aclrm(uint16_t pipe)
{
  if (USB_MAX_PIPE_NO < pipe)
    {
      return; /* Error */
    }

  /* Control ACLRM */

  hw_usb_set_aclrm(pipe);
  hw_usb_clear_aclrm(pipe);
}

/****************************************************************************
 * Function Name   : usb_cstd_set_buf
 * Description     : Set PID (packet ID) of the specified pipe to BUF.
 * Arguments       : uint16_t pipe  : Pipe number.
 * Return value    : none
 ****************************************************************************/

static void usb_cstd_set_buf(uint16_t pipe)
{
  if (USB_MAX_PIPE_NO < pipe)
    {
      return; /* Error */
    }

  /* PIPE control reg set */

  hw_usb_set_pid(pipe, RX65N_USB_DCPCTR_PIDBUF);
}

/****************************************************************************
 * Function Name   : usb_cstd_clr_stall
 * Description     : Set up to NAK the specified pipe, and clear the
 *                 : STALL-bit set to the PID of the specified pipe.
 * Arguments       : usb_utr_t *ptr : Pointer to usb_utr_t structure.
 *                 : uint16_t pipe  : Pipe number.
 * Return value    : none
 * Note            : PID is set to NAK.
 ****************************************************************************/

static void usb_cstd_clr_stall(uint16_t pipe)
{
  if (USB_MAX_PIPE_NO < pipe)
    {
      return; /* Error */
    }

  /* Set NAK */

  usb_cstd_set_nak(pipe);

  /* Clear STALL */

  hw_usb_clear_pid(pipe, RX65N_USB_DCPCTR_PIDSTALL);
}

/****************************************************************************
 * Function Name   : usb_hstd_ctrl_write_start
 * Description     : Start data stage of Control Write transfer.
 * Arguments       : uint32_t  Bsize   : Data Size
 *                 : uint8_t   *Table  : Data Table Address
 * Return          : uint16_t          : USB_WRITEEND / USB_WRITING
 *                 :                   : USB_WRITESHRT / USB_FIFOERROR
 ****************************************************************************/

uint16_t usb_hstd_ctrl_write_start(uint8_t *buf_add, size_t buf_size)
{
  uint16_t end_flag;

  /* PID=NAK & clear STALL */

  usb_cstd_clr_stall(USB_PIPE0);

  /* DCP Configuration Register  (0x5c) */

  hw_usb_write_dcpcfg((USB_CNTMDFIELD | USB_DIRFIELD));

  hw_usb_set_sqset(USB_PIPE0); /* SQSET=1, PID=NAK */

  hw_usb_clear_status_bemp(USB_PIPE0);

  end_flag = usb_hstd_write_data_control_pipe(buf_add, buf_size);

  switch (end_flag)
    {
      /* End of data write */

      case USB_WRITESHRT:

      /* Next stage is Control write status stage */

      /* Enable Empty Interrupt */

      hw_usb_set_bempenb(USB_PIPE0);

      /* Enable Not Ready Interrupt */

      usb_cstd_nrdy_enable(USB_PIPE0);

      /* Set BUF */

      usb_cstd_set_buf(USB_PIPE0);
      break;

      /* End of data write (not null) */

      case USB_WRITEEND:

      /* continue */

      /* Continue of data write */

      case USB_WRITING:

      /* Enable Empty Interrupt */

      hw_usb_set_bempenb(USB_PIPE0);

      /* Enable Not Ready Interrupt */

      usb_cstd_nrdy_enable(USB_PIPE0);

      /* Set BUF */

      usb_cstd_set_buf(USB_PIPE0);
      break;

      /* FIFO access error */

      case USB_FIFOERROR:
      break;

      default:
      break;
    }

  /* End or Err or Continue */

  return end_flag;
}

/****************************************************************************
 * Function Name   : usb_hstd_ctrl_read_start
 * Description     : Start data stage of Control Read transfer.
 * Arguments       : none
 * Return          : none
 ****************************************************************************/

void usb_hstd_ctrl_read_start(void)
{
  /* PID=NAK & clear STALL */

  usb_cstd_clr_stall(USB_PIPE0);

  /* DCP Configuration Register  (0x5c) */

  hw_usb_write_dcpcfg(RX65N_USB_PIPECFG_SHTNAK);

  /* SQSET=1, PID=NAK */

  hw_usb_hwrite_dcpctr(RX65N_USB_DCPCTR_SQSET);

  /* Interrupt enable */

  /* Enable Ready Interrupt */

  hw_usb_set_brdyenb(USB_PIPE0);

  /* Enable Not Ready Interrupt */

  usb_cstd_nrdy_enable(USB_PIPE0);
  usb_cstd_set_buf(USB_PIPE0); /* Set BUF */
}

/****************************************************************************
 * Function Name   : usb_host_read_pipe_start
 * Description     : Start data stage of data Read transfer.
 * Arguments       : uint32_t Bsize   : Data Size
 *                 : uint8_t *Table   : Data Table Address
 * Return          : none
 ****************************************************************************/

void usb_host_read_pipe_start(uint16_t pipe)
{
  uint16_t config_reg;
  uint16_t ctrl_reg;

  /* PID=NAK & clear STALL */

  usb_cstd_clr_stall(pipe);

  config_reg = hw_usb_read_pipecfg();

  /* Pipe disabled at the end of transfer i.e. it is set to NAK */

  config_reg = config_reg | RX65N_USB_PIPECFG_SHTNAK;

  /* Set the direction as read */

  config_reg = config_reg & ~(RX65N_USB_PIPECFG_DIR);
  hw_usb_write_pipecfg(config_reg);

  ctrl_reg = hw_usb_read_pipectr (pipe);
  ctrl_reg = ctrl_reg | RX65N_USB_PIPECTR_SQSET;
  hw_usb_write_pipectr(pipe, ctrl_reg);

  /* usb_hstd_do_sqtgl((uint16_t) USB_PIPE0, 0); */

  /* Interrupt enable */

  /* Enable Ready Interrupt */

  hw_usb_set_brdyenb(pipe);

  /* Enable Not Ready Interrupt */

  usb_cstd_nrdy_enable(pipe);

  /* Set BUF */

  usb_cstd_set_buf(pipe);
}

/****************************************************************************
 * Function Name   : usb_hstd_status_start
 * Description     : Start status stage of Control Command.
 * Arguments       : none
 ****************************************************************************/

void usb_hstd_status_start(void)
{
  /* Interrupt Disable */

  /* BEMP0 Disable */

  hw_usb_clear_bempenb(USB_PIPE0);

  /* BRDY0 Disable */

  hw_usb_clear_brdyenb(USB_PIPE0);
}

/****************************************************************************
 * Function Name   : usb_hstd_ctrl_end
 * Description     : Call the user registered callback function that notifies
 *                 : completion of a control transfer.
 * Arguments       : uint16_t  status : Transfer status
 * Return          : none
 ****************************************************************************/

void usb_hstd_ctrl_end(uint16_t status)
{
  /* Interrupt Disable */

  hw_usb_clear_bempenb(USB_PIPE0); /* BEMP0 Disable */
  hw_usb_clear_brdyenb(USB_PIPE0); /* BRDY0 Disable */
  hw_usb_clear_nrdyenb(USB_PIPE0); /* NRDY0 Disable */

  usb_cstd_clr_stall(USB_PIPE0); /* PID=NAK & clear STALL */
  hw_usb_set_mbw(USB_CUSE, USB0_CFIFO_MBW);

  /* SUREQ=1, SQCLR=1, PID=NAK */

  hw_usb_hwrite_dcpctr((USB_SUREQCLR | USB_SQCLR);

  /* CFIFO buffer clear */

  usb_cstd_chg_curpipe(USB_PIPE0, USB_CUSE, USB_FALSE);
  hw_usb_set_bclr(USB_CUSE); /* Clear BVAL */
  usb_cstd_chg_curpipe((USB_PIPE0, USB_CUSE, USB_ISEL);
  hw_usb_set_bclr(USB_CUSE); /* Clear BVAL */
}

/****************************************************************************
 * Function Name   : usb_hstd_brdy_pipe
 * Description     : BRDY Interrupt
 * Arguments       : uint16_t bitsts  : BRDYSTS Reg & BRDYENB Reg
 * Return value    : none
 ****************************************************************************/

void usb_hstd_brdy_pipe(void)
{
  uint16_t bitsts;

  bitsts = rx65n_usbhost_getreg(RX65N_USB_BRDYSTS);

  /* When operating by the host function, usb_hstd_brdy_pipe() is executed
   * without fail because only one BRDY message is issued even when the
   * demand of PIPE0 and PIPEx has been generated at the same time.
   */

  if (USB_BRDY0 == (bitsts & USB_BRDY0))
    {
      hw_usb_clear_sts_brdy(USB_PIPE0);

      /* Branch  by the Control transfer stage management */

      if (EDCTRL->xfrinfo->tdxfercond == USB_STATUSRD)
        {
          /* This interrupt occurred due to setup packet status stage of
           * writing NULL packet.
           */

          /* Call this to end the setup packet */

          /* Control Read/Write End */

          usb_hstd_ctrl_end(USB_CTRL_END);

          nxsem_post(&EDCTRL->wdhsem);
          return; /* Nothing else to do here... as of now... */
        }

      switch (EDCTRL->xfrinfo->tdxfercond)
        {
          /* Data stage of Control read transfer */

          case USB_DATARD:
          EDCTRL->xfrinfo->tdxfercond = usb_hstd_read_data_control_pipe ();

          switch (EDCTRL->xfrinfo->tdxfercond)
            {
              /* End of data read */

              case USB_READEND:

              /* continue */

              /* End of data read */

              case USB_READSHRT:

              usb_hstd_status_start();
              break;

              case USB_READING: /* Continue of data read */

              /* Still data is there - so set the condition for
               * reading in next interrupt...
               */

              EDCTRL->xfrinfo->tdxfercond = USB_DATARD;
              break;

              case USB_READOVER: /* FIFO access error */

              /* Control Read/Write End */

              usb_hstd_ctrl_end(USB_DATA_OVR);
              break;

              case USB_FIFOERROR: /* FIFO access error */

              /* Control Read/Write End */

              usb_hstd_ctrl_end(USB_DATA_ERR);
              syslog(LOG_INFO, "ERROR");
              break;

              default:
              break;
            }
          break;

          /* Data stage of Control read transfer */

          case USB_DATARDCNT:

            switch (usb_hstd_read_data_control_pipe())
              {
                case USB_READEND: /* End of data read */

                /* Control Read/Write End */

                usb_hstd_ctrl_end(USB_CTRL_READING);
                break;

                case USB_READSHRT: /* End of data read */

                /* Control Read/Write Status */

                usb_hstd_status_start();
                break;

                case USB_READING: /* Continue of data read */

                /* Still data is there - so set the condition for
                 * reading in next interrupt...
                 */

                EDCTRL->xfrinfo->tdxfercond = USB_DATARDCNT;
                break;

                case USB_READOVER: /* FIFO access error */

                /* Control Read/Write End */

                usb_hstd_ctrl_end(USB_DATA_OVR);
                break;

                case USB_FIFOERROR: /* FIFO access error */

                /* Control Read/Write End */

                usb_hstd_ctrl_end(USB_DATA_ERR);
                syslog(LOG_INFO, "ERROR");
                break;

                default:
                break;
            }
          break;

          /* Status stage Control write (NoData control) transfer */

          case USB_STATUSWR:

          /* Control Read/Write End */

          usb_hstd_ctrl_end(USB_CTRL_END);
          break;

          default:
          break;
        }

      if ((EDCTRL->xfrinfo->tdxfercond == USB_READEND) ||
          (EDCTRL->xfrinfo->tdxfercond == USB_READSHRT) ||
          (EDCTRL->xfrinfo->tdxfercond == USB_READOVER))
        {
          nxsem_post(&EDCTRL->wdhsem);
        }

      hw_usb_clear_sts_brdy(USB_PIPE0); /* This was missing? */
    }

  /* BRDY interrupt */

  usb_hstd_brdy_pipe_process(bitsts);
}

/****************************************************************************
 * Function Name   : usb_hstd_nrdy_pipe
 * Description     : NRDY Interrupt
 * Arguments       : usb_utr_t *ptr   : Pointer to usb_utr_t structure.
 *                 : uint16_t bitsts  : NRDYSTS Reg & NRDYENB Reg
 * Return value    : none
 ****************************************************************************/

void usb_hstd_nrdy_pipe(void)
{
  uint16_t buffer;
  uint16_t bitsts;

  bitsts = rx65n_usbhost_getreg(RX65N_USB_NRDYSTS); /* ptr->status; */

  /* When operating by the host function, usb_hstd_nrdy_pipe()
   * is executed without fail because
   * only one NRDY message is issued even when the demand of
   * PIPE0 and PIPEx has been generated at the same time.
   */

  if (USB_NRDY0 == (bitsts & USB_NRDY0))
    {
      hw_usb_clear_status_nrdy(USB_PIPE0);

      /* Get Pipe PID from pipe number */

      buffer = usb_cstd_get_pid(USB_PIPE0);

      /* STALL ? */

      if (RX65N_USB_DCPCTR_PIDSTALL == (buffer &
          RX65N_USB_DCPCTR_PIDSTALL))
        {
          /* PIPE0 STALL call back */

          usb_hstd_ctrl_end(USB_DATA_STALL);
        }
      else
        {
          /* Control Data Stage Device Ignore X 3 call back */

          usb_hstd_ctrl_end(USB_DATA_ERR);
        }
    }

  /* Nrdy Pipe interrupt */

  usb_hstd_nrdy_pipe_process(bitsts);
}

/****************************************************************************
 * Function Name   : usb_hstd_bemp_pipe
 * Description     : BEMP interrupt
 * Arguments       : uint16_t bitsts  : BEMPSTS Reg & BEMPENB Reg
 * Return value    : none
 ****************************************************************************/

void usb_hstd_bemp_pipe(void)
{
  uint16_t buffer;
  uint16_t bitsts;

  bitsts = rx65n_usbhost_getreg(RX65N_USB_BEMPSTS); /* ptr->status; */

  /* When operating by the host function, usb_hstd_bemp_pipe()
   * is executed without fail because
   * only one BEMP message is issued even when the demand of PIPE0
   * and PIPEx has been generated at the same time.
   */

  if (USB_BEMP0 == (bitsts & USB_BEMP0))
    {
      hw_usb_clear_status_bemp(USB_PIPE0);
      if (EDCTRL->xfrinfo->tdxfercond == USB_STATUSWR)
        {
          /* This interrupt occurred due to setup packet status
           * stage of writing NULL packet.
           */

          /* BEMP0 Disable */

          hw_usb_clear_bempenb(USB_PIPE0);

          /* BRDY0 Disable */

          hw_usb_clear_brdyenb(USB_PIPE0);

          /* Call this to end the setup packet */

          /* Control Read/Write End */

          usb_hstd_ctrl_end(USB_CTRL_END);

          nxsem_post(&EDCTRL->wdhsem);
          return; /* As of now, Nothing else to do here... */
        }

      /* Get Pipe PID from pipe number */

      buffer = usb_cstd_get_pid(USB_PIPE0);

      /* MAX packet size error ? */

      if (RX65N_USB_DCPCTR_PIDSTALL == (buffer & RX65N_USB_DCPCTR_PIDSTALL))
        {
          /* PIPE0 STALL call back */

          usb_hstd_ctrl_end(USB_DATA_STALL);
        }
      else
        {
          /* Branch  by the Control transfer stage management */

          switch (bitsts)
            {
              /* Continuas of data stage (Control write) */

              case USB_DATAWR:

              /* We should not get into this... */

              /* Buffer to CFIFO data write */

              switch (usb_hstd_write_data_control_pipe(0, 0))
                {
                  /* End of data write */

                  case USB_WRITESHRT:

                  hw_usb_set_bempenb(USB_PIPE0);

                  /* Enable Not Ready Interrupt */

                  usb_cstd_nrdy_enable(USB_PIPE0);
                  break;

                  /* End of data write (not null) */

                  case USB_WRITEEND:

                  /* continue */

                  /* Continue of data write */

                  case USB_WRITING:

                  /* Enable Empty Interrupt */

                  hw_usb_set_bempenb(USB_PIPE0);

                  /* Enable Not Ready Interrupt */

                  usb_cstd_nrdy_enable(USB_PIPE0);
                  break;

                  /* FIFO access error */

                  case USB_FIFOERROR:

                  /* Control Read/Write End */

                  usb_hstd_ctrl_end(USB_DATA_ERR);
                  break;

                 default:
                 break;
                }
              break;

             /* Next stage to Control write data */

              case USB_DATAWRCNT:

              /* Buffer to CFIFO data write */

              /* We should not get here... */

              switch (usb_hstd_write_data_control_pipe(0, 0))
                {
                  /* End of data write */

                  case USB_WRITESHRT:

                  hw_usb_set_bempenb(USB_PIPE0);

                  /* Enable Not Ready Interrupt */

                  usb_cstd_nrdy_enable(USB_PIPE0);
                  break;

                  /* End of data write (not null) */

                  case USB_WRITEEND:

                  /* Control Read/Write End */

                  usb_hstd_ctrl_end(USB_CTRL_WRITING);
                  break;

                  /* Continue of data write */

                  case USB_WRITING:

                  /* Enable Empty Interrupt */

                  hw_usb_set_bempenb(USB_PIPE0);

                  /* Enable Not Ready Interrupt */

                  usb_cstd_nrdy_enable(USB_PIPE0);
                  break;

                  /* FIFO access error */

                  case USB_FIFOERROR:

                  /* Control Read/Write End */

                  usb_hstd_ctrl_end(USB_DATA_ERR);
                  break;

                  default:
                  break;
                }
              break;

              case USB_STATUSWR: /* End of data stage (Control write) */
              usb_hstd_status_start();
              break;

              /* Status stage of Control read transfer */

              case USB_STATUSRD:

              /* Control Read/Write End */

              usb_hstd_ctrl_end(USB_CTRL_END);
              break;

              default:
              break;
            }
        }
    }

  usb_hstd_bemp_pipe_process(bitsts); /* BEMP interrupt */
}

/****************************************************************************
 * Name: rx65n_usbhost_printreg
 *
 * Description:
 *   Print the contents of an RX65N register operation
 *
 ****************************************************************************/

#ifdef CONFIG_RX65N_USBHOST_REGDEBUG
static void rx65n_usbhost_printreg(uint32_t addr, uint32_t val, bool iswrite)
{
  uinfo("%08x%s%08x\n", addr, iswrite ? "<-" : "->", val);
}
#endif

/****************************************************************************
 * Name: rx65n_usbhost_checkreg
 *
 * Description:
 *   Get the contents of an RX65N register
 *
 ****************************************************************************/

#ifdef CONFIG_RX65N_USBHOST_REGDEBUG
static void rx65n_usbhost_checkreg(uint32_t addr, uint32_t val, bool iswrite)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval = 0;
  static uint32_t count = 0;
  static bool     prevwrite = false;

  /* Is this the same value that we read from/wrote to the same register
   * last time? Are we polling the register?  If so, suppress the output.
   */

  if (addr == prevaddr && val == preval && prevwrite == iswrite)
    {
      /* Yes.. Just increment the count */

      count++;
    }
  else
    {
      /* No this is a new address or value or operation. Were there any
       * duplicate accesses before this one?
       */

      if (count > 0)
        {
          /* Yes.. Just one? */

          if (count == 1)
            {
              /* Yes.. Just one */

              rx65n_usbhost_printreg(prevaddr, preval, prevwrite);
            }
          else
            {
              /* No.. More than one. */

              uinfo("[repeats %d more times]\n", count);
            }
        }

      /* Save the new address, value, count, and operation for next time */

      prevaddr  = addr;
      preval    = val;
      count     = 0;
      prevwrite = iswrite;

      /* Show the new regisgter access */

      rx65n_usbhost_printreg(addr, val, iswrite);
    }
}
#endif

/****************************************************************************
 * Name: rx65n_usbhost_getreg
 *
 * Description:
 *   Get the contents of an RX65N register
 *
 ****************************************************************************/

#ifdef CONFIG_RX65N_USBHOST_REGDEBUG
static uint16_t rx65n_usbhost_getreg(uint32_t addr)
{
  /* Read the value from the register */

  uint16_t val = getreg16(addr);

  /* Check if we need to print this value */

  rx65n_usbhost_checkreg(addr, val, false);
  return val;
}
#endif

/****************************************************************************
 * Name: rx65n_usbhost_putreg
 *
 * Description:
 *   Set the contents of an RX65N register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_RX65N_USBHOST_REGDEBUG
static void rx65n_usbhost_putreg(uint16_t val, uint32_t addr)
{
  /* Check if we need to print this value */

  rx65n_usbhost_checkreg(addr, val, true);

  /* Write the value */

  putreg16(val, addr);
}
#endif

/****************************************************************************
 * Name: rx65n_usbhost_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 ****************************************************************************/

static inline uint16_t rx65n_usbhost_getle16(const uint8_t *val)
{
  return val[1] << 8 | val[0];
}

/****************************************************************************
 * Name: rx65n_usbhost_edfree
 *
 * Description:
 *   Return an endpoint descriptor to the free list
 *
 ****************************************************************************/

static inline void rx65n_usbhost_edfree(struct rx65n_usbhost_ed_s *ed)
{
  struct rx65n_usbhost_list_s *entry = (struct rx65n_usbhost_list_s *)ed;

  /* Put the ED back into the free list */

  entry->flink = g_edfree;
  g_edfree     = entry;
}

/****************************************************************************
 * Name: rx65n_usbhost_tdalloc
 *
 * Description:
 *   Allocate an transfer descriptor from the free list
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *   - Protected from concurrent access to the TD pool by the interrupt
 *     handler
 *   - Protection from re-entrance must be assured by the caller
 *
 ****************************************************************************/

static struct rx65n_usbhost_gtd_s *rx65n_usbhost_tdalloc(uint8_t ep_num)
{
  /* Currently each TD would associate with one EP. So the ep_numb is
   * passed to tdalloc function and it would return the TD with this,
   * there is no need to free this
   */

  memset(&(g_rx65n_tdlist[ep_num]), 0, sizeof(struct rx65n_usbhost_gtd_s));
  return &(g_rx65n_tdlist[ep_num]);
}

/****************************************************************************
 * Name: rx65n_tdfree
 *
 * Description:
 *   Return an transfer descriptor to the free list
 *
 * Assumptions:
 * Only called from the WDH interrupt handler (and during initialization).
 * Interrupts are disabled in any case.
 *
 ****************************************************************************/

static void rx65n_usbhost_tdfree(struct rx65n_usbhost_gtd_s *td)
{
  struct rx65n_usbhost_list_s *tdfree = (struct rx65n_usbhost_list_s *)td;

  /* This should not happen but just to be safe, don't free the common, pre-
   * allocated tail TD.
   */

  if (tdfree != NULL && td != TDTAIL)
    {
      tdfree->flink = g_tdfree;
      g_tdfree      = tdfree;
    }
}

/****************************************************************************
 * Name: rx65n_usbhost_tballoc
 *
 * Description:
 *   Allocate an request/descriptor transfer buffer from the free list
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *   - Protection from re-entrance must be assured by the caller
 *
 ****************************************************************************/

static uint8_t *rx65n_usbhost_tballoc(void)
{
  uint8_t *ret = (uint8_t *)g_tbfree;

  if (ret)
    {
      g_tbfree = ((struct rx65n_usbhost_list_s *)ret)->flink;
    }

  return ret;
}

/****************************************************************************
 * Name: rx65n_usbhost_tbfree
 *
 * Description:
 *   Return an request/descriptor transfer buffer to the free list
 *
 ****************************************************************************/

static void rx65n_usbhost_tbfree(uint8_t *buffer)
{
  struct rx65n_usbhost_list_s *tbfree =
  (struct rx65n_usbhost_list_s *)buffer;

  if (tbfree)
    {
      tbfree->flink = g_tbfree;
      g_tbfree      = tbfree;
    }
}

/****************************************************************************
 * Name: rx65n_usbhhost_allocio
 *
 * Description:
 *   Allocate an IO buffer from the free list
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *   - Protection from re-entrance must be assured by the caller
 *
 ****************************************************************************/

#if RX65N_USBHOST_IOBUFFERS > 0
static uint8_t *rx65n_usbhhost_allocio(void)
{
  uint8_t *ret;
  irqstate_t flags;

  flags = enter_critical_section();
  ret = (uint8_t *)g_iofree;
  if (ret)
    {
      g_iofree = ((struct rx65n_usbhost_list_s *)ret)->flink;
    }

  leave_critical_section(flags);
  return ret;
}
#endif

/****************************************************************************
 * Name: rx65n_usbhhost_freeio
 *
 * Description:
 *   Return an TD buffer to the free list
 *
 ****************************************************************************/

#if RX65N_USBHOST_IOBUFFERS > 0
static void rx65n_usbhhost_freeio(uint8_t *buffer)
{
  struct rx65n_usbhost_list_s *iofree;
  irqstate_t flags;

  /* Could be called from the interrupt level */

  flags         = enter_critical_section();
  iofree        = (struct rx65n_usbhost_list_s *)buffer;
  iofree->flink = g_iofree;
  g_iofree      = iofree;
  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: rx65n_usbhost_alloc_xfrinfo
 *
 * Description:
 *   Allocate an asynchronous data structure from the free list
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *   - Protection from re-entrance must be assured by the caller
 *
 ****************************************************************************/

static struct rx65n_usbhost_xfrinfo_s *rx65n_usbhost_alloc_xfrinfo(void)
{
  struct rx65n_usbhost_xfrinfo_s *ret;
  irqstate_t flags;

  flags = enter_critical_section();
  ret = (struct rx65n_usbhost_xfrinfo_s *)g_xfrfree;

  if (ret)
    {
      g_xfrfree = ((struct rx65n_usbhost_list_s *)ret)->flink;
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: rx65n_usbhhost_freeio
 *
 * Description:
 *   Return an TD buffer to the free list
 *
 ****************************************************************************/

static void rx65n_usbhost_free_xfrinfo(struct
                rx65n_usbhost_xfrinfo_s *xfrinfo)
{
  struct rx65n_usbhost_list_s *node;
  irqstate_t flags;

  /* Could be called from the interrupt level */

  flags        = enter_critical_section();
  node         = (struct rx65n_usbhost_list_s *)xfrinfo;
  node->flink  = g_xfrfree;
  g_xfrfree    = node;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rx65n_usbhost_addctrled
 *
 * Description:
 *   Helper function to add an ED to the control list.
 *
 ****************************************************************************/

static inline int rx65n_usbhost_addctrled(struct rx65n_usbhost_s *priv,
                                          struct rx65n_usbhost_ed_s *ed)
{
  /* Ctrl ED is always used and statically assigned */

  return OK;
}

/****************************************************************************
 * Name: rx65n_usbhost_remctrled
 *
 * Description:
 *   Helper function remove an ED from the control list.
 *
 ****************************************************************************/

static inline int rx65n_usbhost_remctrled(struct rx65n_usbhost_s *priv,
                                          struct rx65n_usbhost_ed_s *ed)
{
  /* Ctrl ED is always used and statically assigned */

  /* The semaphore for the USB HID keyboard is released */

  if (g_kbdport == g_usbidx)
    {
      nxsem_post(&g_rx65n_edlist[g_kbdpipe].wdhsem);
    }

  return OK;
}

/****************************************************************************
 * Name: rx65n_usbhost_addbulked
 *
 * Description:
 *   Helper function to add an ED to the bulk list.
 *
 ****************************************************************************/

static inline int rx65n_usbhost_addbulked(struct rx65n_usbhost_s *priv,
                                   const struct usbhost_epdesc_s *epdesc,
                                   struct rx65n_usbhost_ed_s *ed)
{
#ifndef CONFIG_USBHOST_BULK_DISABLE
  irqstate_t flags;
  uint8_t    pipe_no;
  uint16_t   pipe_cfg;
  uint16_t   pipe_maxp;

  /* Pipe Cycle Control Register is not needed for bulk pipe */

  /* uint16_t    pipe_peri; */

  /* Disable bulk list processing while we modify the list */

  flags   = enter_critical_section();
  pipe_no = ed->pipenum;

  /* To begin with start config as Bulk pipe */

  pipe_cfg = 0x4000;

  /* Update the direction */

  /* Direction is in */

  if (epdesc->in)
    {
      pipe_cfg = pipe_cfg & ~ (RX65N_USB_PIPECFG_DIR);
      pipe_cfg = pipe_cfg | RX65N_USB_PIPECFG_SHTNAK;
    }
  else
    {
      pipe_cfg = pipe_cfg | (RX65N_USB_PIPECFG_DIR);
    }

  /* Finally update the address */

  pipe_cfg = pipe_cfg | ((epdesc->addr) & RX65N_USB_PIPECFG_EPNUM_MASK);

  /* Update pipe_maxp : Begin with initial value from device address */

  pipe_maxp = rx65n_usbhost_getreg(RX65N_USB_DCPMAXP);
  pipe_maxp &= ~(RX65N_USB_PIPEMAXP_MXPSMASK);
  pipe_maxp = pipe_maxp | (epdesc->mxpacketsize);

  /* Now all the values are ready to be written */

  g_usb_pipe_table[pipe_no].pipe_cfg = pipe_cfg;
  g_usb_pipe_table[pipe_no].pipe_maxp = pipe_maxp;
  g_usb_pipe_table[pipe_no].pipe_peri = 0;

  /* Now update these values in the requried pipe */

  usb_cstd_pipe_init(pipe_no);
  leave_critical_section(flags);

  return OK;

#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: rx65n_usbhost_rembulked
 *
 * Description:
 *   Helper function remove an ED from the bulk list.
 *
 ****************************************************************************/

static inline int rx65n_usbhost_rembulked(struct rx65n_usbhost_s *priv,
                                          struct rx65n_usbhost_ed_s *ed)
{
/* This function requires implementation
 * for OHCI specific interrupt disabling
 * As RX65N is not OHCI compliant, this function
 * need not be implemented
 */

  return OK;
}

/****************************************************************************
 * Name: rx65n_usbhost_getinterval
 *
 * Description:
 *   Convert the endpoint polling interval into a HCCA table increment
 *
 ****************************************************************************/

#if !defined(CONFIG_USBHOST_INT_DISABLE) || \
    !defined(CONFIG_USBHOST_ISOC_DISABLE)
static unsigned int rx65n_usbhost_getinterval(uint8_t interval)
{
  /* The bInterval field of the endpoint descriptor contains the polling
   * interval for interrupt and isochronous endpoints. For other types of
   * endpoint, this value should be ignored. bInterval is provided in units
   * of 1MS frames.
   */

  if (interval < 3)
    {
      return 2;
    }
  else if (interval < 7)
    {
      return 4;
    }
  else if (interval < 15)
    {
      return 8;
    }
  else if (interval < 31)
    {
      return 16;
    }
  else
    {
      return 32;
    }
}
#endif

/****************************************************************************
 * Name: rx65n_usbhost_setinttab
 *
 * Description:
 *      Set the interrupt table to the selected value using the provided
 *  interval and offset.
 *
 ****************************************************************************/

#if !defined(CONFIG_USBHOST_INT_DISABLE) || \
!defined(CONFIG_USBHOST_ISOC_DISABLE)
static void rx65n_usbhost_setinttab(uint32_t value, unsigned int interval,
  unsigned int offset)
{
  unsigned int i;

  for (i = offset; i < HCCA_INTTBL_WSIZE; i += interval)
    {
      HCCA->inttbl[i] = value;
    }
}
#endif

/****************************************************************************
 * Name: rx65n_usbhost_addinted
 *
 * Description:
 *   Helper function to add an ED to the HCCA interrupt table.
 *
 *   To avoid reshuffling the table so much and to keep life simple in
 *    general, the following rules are applied:
 *
 *     1. IN EDs get the even entries, OUT EDs get the odd entries.
 *     2. Add IN/OUT EDs are scheduled together at the minimum interval of
 *        all IN/OUT EDs.
 *
 *   This has the following consequences:
 *
 *     1. The minimum support polling rate is 2MS, and
 *     2. Some devices may get polled at a much higher rate than they request
 *
 ****************************************************************************/

static inline int rx65n_usbhost_addinted(struct rx65n_usbhost_s *priv,
                                 const struct usbhost_epdesc_s *epdesc,
                                 struct rx65n_usbhost_ed_s *ed)
{
#ifndef CONFIG_USBHOST_INT_DISABLE
  unsigned int interval;
  unsigned int offset;
  uint32_t head;
  uint8_t     pipe_no;
  uint16_t    pipe_cfg;
  uint16_t    pipe_maxp;
  uint16_t    pipe_peri;

  /* Get the quantized interval value associated with this ED and save it
   * in the ED.
   */

  interval     = rx65n_usbhost_getinterval(epdesc->interval);

  ed->interval = interval;
  uinfo("interval: %d->%d\n", epdesc->interval, interval);

  /* Get the offset associated with the ED direction. IN EDs get the even
   * entries, OUT EDs get the odd entries.
   *
   * Get the new, minimum interval. Add IN/OUT EDs are scheduled together
   * at the minimum interval of all IN/OUT EDs.
   */

  if (epdesc->in)
    {
      offset = 0;
      if (priv->ininterval > interval)
        {
          priv->ininterval = interval;
        }
      else
        {
          interval = priv->ininterval;
        }
    }

  else
    {
      offset = 1;
      if (priv->outinterval > interval)
        {
          priv->outinterval = interval;
        }
      else
        {
          interval = priv->outinterval;
        }
    }

  uinfo("min interval: %d offset: %d\n", interval, offset);

  /* Get value for Interval Error Detection Interval  */

  pipe_no = ed->pipenum;

  if (epdesc->mxpacketsize != 1)
    {
      g_kbdpipe = pipe_no;
      kbd_interrupt_in_pipe = pipe_no;
    }
  else
    {
      g_kbdpipe = 0;
    }

  /* To begin with start config as INT pipe */

  pipe_cfg = 0x8000;

  /* Update the direction */

  /* Direction is in */

  if (epdesc->in)
    {
       pipe_cfg = pipe_cfg & ~ (RX65N_USB_PIPECFG_DIR);
    }
  else
    {
      pipe_cfg = pipe_cfg | ~ (RX65N_USB_PIPECFG_DIR);
    }

  /* Finally update the address */

  pipe_cfg = pipe_cfg | ((epdesc->addr) & RX65N_USB_PIPECFG_EPNUM_MASK);

  /* Update pipe_maxp : Begin with initial value from endpoint address */

  pipe_maxp = rx65n_usbhost_getreg(RX65N_USB_DCPMAXP);
  pipe_maxp &= ~(RX65N_USB_PIPEMAXP_MXPSMASK);
  pipe_maxp = pipe_maxp | (epdesc->mxpacketsize);

  /* Now get the Pipe Peri values */

  pipe_peri = usb_hstd_get_pipe_peri_value(epdesc->interval);

  /* Now all the values are ready to be written */

  g_usb_pipe_table[pipe_no].pipe_cfg = pipe_cfg;
  g_usb_pipe_table[pipe_no].pipe_maxp = pipe_maxp;
  g_usb_pipe_table[pipe_no].pipe_peri = pipe_peri;

  /* Now update these values in the requried pipe */

  usb_cstd_pipe_init(pipe_no);

  struct rx65n_usbhost_xfrinfo_s *xfrinfo;
  DEBUGASSERT(ed->xfrinfo == NULL);

  xfrinfo = rx65n_usbhost_alloc_xfrinfo();
  if (xfrinfo == NULL)
    {
      rx65n_usbhost_free_xfrinfo(xfrinfo);
      uerr("ERROR: rx65n_usbhost_alloc_xfrinfo failed\n");
      return -ENOMEM;
    }

  /* Initialize the transfer structure */

  memset(xfrinfo, 0, sizeof(struct rx65n_usbhost_xfrinfo_s));
  xfrinfo->buffer = &kbd_report_data[0];
  xfrinfo->buflen = epdesc->mxpacketsize;
  xfrinfo->tdxfercond = USB_DATARD;

  ed->xfrinfo = xfrinfo;

  /* Start the reading the interrupt pipe for KBD */

  /* Now start the interrupt pipe */

  usb_host_read_pipe_start(pipe_no);

  /* Get the head of the first of the duplicated entries.  The first offset
   * entry is always guaranteed to contain the common ED list head.
   */

  head = HCCA->inttbl[offset];

  /* Clear all current entries in the interrupt table for this direction */

  rx65n_usbhost_setinttab(0, 2, offset);

  /* Add the new ED before the old head of the periodic ED list and set the
   * new ED as the head ED in all of the appropriate entries of the HCCA
   * interrupt table.
   */

  ed->hw.nexted = head;
  rx65n_usbhost_setinttab((uint32_t)ed, interval, offset);
  uinfo("head: %08x next: %08x\n", ed, head);

  return OK;
#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: rx65n_usbhost_reminted
 *
 * Description:
 *   Helper function to remove an ED from the HCCA interrupt table.
 *
 *   To avoid reshuffling the table so much and to keep life simple in
 *    general, the following rules are applied:
 *
 *     1. IN EDs get the even entries, OUT EDs get the odd entries.
 *     2. Add IN/OUT EDs are scheduled together at the minimum interval of
 *        all IN/OUT EDs.
 *
 *   This has the following consequences:
 *
 *     1. The minimum support polling rate is 2MS, and
 *     2. Some devices may get polled at a much higher rate than they
 *        request.
 *
 ****************************************************************************/

static inline int rx65n_usbhost_reminted(struct rx65n_usbhost_s *priv,
                                         struct rx65n_usbhost_ed_s *ed)
{
  /* This function is to disable OHCI specific interrupts
   * As, RX65N is not OHCI compliant, this function does not require
   * any implementation
   */

  return OK;
}

/****************************************************************************
 * Name: rx65n_usbhost_addisoced
 *
 * Description:
 *   Helper functions to add an ED to the periodic table.
 *
 ****************************************************************************/

static inline int rx65n_usbhost_addisoced(struct rx65n_usbhost_s *priv,
                                  const struct usbhost_epdesc_s *epdesc,
                                  struct rx65n_usbhost_ed_s *ed)
{
#ifndef CONFIG_USBHOST_ISOC_DISABLE
  printf("Isochronous endpoints not yet supported\n");
#endif
  return -ENOSYS;
}

/****************************************************************************
 * Name: rx65n_usbhost_remisoced
 *
 * Description:
 *   Helper functions to remove an ED from the periodic table.
 *
 ****************************************************************************/

static inline int rx65n_usbhost_remisoced(struct rx65n_usbhost_s *priv,
                                          struct rx65n_usbhost_ed_s *ed)
{
#ifndef CONFIG_USBHOST_ISOC_DISABLE
  printf("Isochronous endpoints not yet supported\n");
#endif
  return -ENOSYS;
}

/****************************************************************************
 * Name: rx65n_usbhost_enqueuetd
 *
 * Description:
 *   Enqueue a transfer descriptor.  Notice that this function only supports
 *   queue on TD per ED.
 *
 ****************************************************************************/

static int rx65n_usbhost_enqueuetd(struct rx65n_usbhost_s *priv,
                                   struct rx65n_usbhost_ed_s *ed,
                                   uint32_t dirpid, uint32_t toggle,
                                   volatile uint8_t *buffer, size_t buflen)
{
  struct rx65n_usbhost_gtd_s *td;
  int ret = -ENOMEM;

  /* Allocate a TD from the free list */

  /* Currently each TD would associate with one EP. So the epnumb
   * is passed to tdalloc function and it would return the TD with
   * this, there is no need to free this - there is no need
   */

  td = rx65n_usbhost_tdalloc(ed->pipenum);

  if (td != NULL)
    {
      /* Initialize allocated TD and link it before the common tail TD. */

      td->hw.ctrl = (GTD_STATUS_R | dirpid | TD_DELAY(0) |
        toggle | GTD_STATUS_CC_MASK);
      TDTAIL->hw.ctrl     = 0;
      td->hw.cbp          = (uint32_t)buffer;
      TDTAIL->hw.cbp      = 0;
      td->hw.nexttd       = (uint32_t)TDTAIL;
      TDTAIL->hw.nexttd   = 0;
      td->hw.be           = (uint32_t)(buffer + (buflen - 1));
      TDTAIL->hw.be       = 0;

      /* Configure driver-only fields in the extended TD structure */

      td->ed              = ed;

      /* Link the td to the head of the ED's TD list */

      ed->hw.headp        = (uint32_t)td | ((ed->hw.headp) & ED_HEADP_C);
      ed->hw.tailp        = (uint32_t)TDTAIL;
      ret                 = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: rx65n_usbhost_wdhwait
 *
 * Description:
 *   Set the request for the Writeback Done Head event well BEFORE enabling
 *   the transfer (as soon as we are absolutely committed to the to avoid
 *   transfer). We do this to minimize race conditions.  This logic would
 *   have to be  expanded if we want to have more than one packet in flight
 *   at a time!
 *
 ****************************************************************************/

static int rx65n_usbhost_wdhwait(struct rx65n_usbhost_s *priv,
                                 struct rx65n_usbhost_ed_s *ed)
{
  struct rx65n_usbhost_xfrinfo_s *xfrinfo;
  irqstate_t flags = enter_critical_section();
  int        ret   = -ENODEV;

  DEBUGASSERT(ed && ed->xfrinfo);
  xfrinfo = ed->xfrinfo;

  /* Is the device still connected? */

  if (priv->connected)
    {
      /* Yes.. then set wdhwait to indicate that we expect to be informed
       * when either (1) the device is disconnected, or (2) the transfer
           *  completed.
       */

      xfrinfo->wdhwait = true;
      ret = OK;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: rx65n_usbhost_ctrltd
 *
 * Description:
 *   Process a IN or OUT request on the control endpoint.  This function
 *   will enqueue the request and wait for it to complete.  Only one
 *   transfer may be queued; Neither these methods nor the transfer() method
 *   can be called again until the control transfer functions returns.
 *
 *   These are blocking methods; these functions will not return until the
 *   control transfer has completed.
 *
 ****************************************************************************/

static int rx65n_usbhost_ctrltd(struct rx65n_usbhost_s *priv,
                                struct rx65n_usbhost_ed_s *ed,
                                uint32_t dirpid, uint8_t *buffer,
                                size_t buflen)
{
  struct rx65n_usbhost_xfrinfo_s *xfrinfo;
  uint32_t toggle;
  int ret;
  uint16_t sdata;

  /* Allocate a structure to retain the information needed when the
   * transfer completes.
   */

  DEBUGASSERT(ed->xfrinfo == NULL);

  xfrinfo = rx65n_usbhost_alloc_xfrinfo();
  if (xfrinfo == NULL)
    {
      uerr("ERROR: rx65n_usbhost_alloc_xfrinfo failed\n");
      return -ENOMEM;
    }

  /* Initialize the transfer structure */

  memset(xfrinfo, 0, sizeof(struct rx65n_usbhost_xfrinfo_s));
  xfrinfo->buffer = buffer;
  xfrinfo->buflen = buflen;

  /* copy the transfer condition - any way we are copying this
   * address to ed just 2 lines after this...
   */

  xfrinfo->tdxfercond = ed->xfrinfo->tdxfercond;

  ed->xfrinfo = xfrinfo;

  /* Set the request for the Write-back Done Head event well BEFORE
   * enabling the transfer.
   */

  if (priv->connected)
    {
      ret = rx65n_usbhost_wdhwait(priv, ed);
      if (ret < 0)
        {
          syslog(LOG_INFO, "ERROR: Device disconnected\n");
          goto errout_with_xfrinfo;
        }
    }

  /* Configure the toggle field in the TD */

  if (dirpid == GTD_STATUS_DP_SETUP)
    {
      toggle = GTD_STATUS_T_DATA0;
    }
  else
    {
      toggle = GTD_STATUS_T_DATA1;
    }

  /* Then enqueue the transfer */

  xfrinfo->tdstatus = TD_CC_NOERROR;
  ret = rx65n_usbhost_enqueuetd(priv, ed, dirpid, toggle, buffer, buflen);

  if (ret == OK)
    {
      /* Set ControlListFilled.  This bit is used to indicate whether there
       * are TDs on the Control list.
       */

      if (dirpid == GTD_STATUS_DP_SETUP)
        {
          nxmutex_lock(&priv->lock);

          /* Set DATA0 bit of DCPCTR */

          rx65n_usbhost_setbit(RX65N_USB_DCPCTR, RX65N_USB_DCPCTR_SQCLR);

          sdata = *(ed->xfrinfo->buffer) | (*(ed->xfrinfo->buffer + 1) << 8);

          /* Request type and Request */

          rx65n_usbhost_putreg(sdata, RX65N_USB_USBREQ);
          sdata = *(ed->xfrinfo->buffer + 2) |
                  (*(ed->xfrinfo->buffer + 3) << 8);
          rx65n_usbhost_putreg(sdata, RX65N_USB_USBVAL); /* wValue */
          sdata = *(ed->xfrinfo->buffer + 4) |
                  (*(ed->xfrinfo->buffer + 5) << 8);
         rx65n_usbhost_putreg(sdata, RX65N_USB_USBINDX); /* wIndex */
          sdata = *(ed->xfrinfo->buffer + 6) |
                  (*(ed->xfrinfo->buffer + 7) << 8);
         rx65n_usbhost_putreg(sdata, RX65N_USB_USBLENG); /* wLen */

          rx65n_usbhost_setbit(RX65N_USB_DCPCTR, RX65N_USB_DCPCTR_SQSET);

          hw_usb_hclear_sts_sign();
          hw_usb_hclear_sts_sack();

          hw_usb_hset_enb_signe();
          hw_usb_hset_enb_sacke();
          hw_usb_hset_sureq();

          /* At this point every thing is done w.r.t hardware to send the
           * setup packet... Now release the exclusive access mutex and
           * wait for wdhsem
           */

          nxmutex_unlock(&priv->lock);

          /* Wait for the Writeback Done Head interrupt */

          if (priv->connected)
            {
             nxsem_wait_uninterruptible(&ed->wdhsem);
            }

          /* Disable setup packet status response */

          rx65n_usbhost_clearbit(RX65N_USB_INTENB1,
          RX65N_USB_INTENB1_SACKE | RX65N_USB_INTENB1_SIGNE);
        }

      else if (dirpid == GTD_STATUS_DP_IN)
        {
          nxmutex_lock(&priv->lock);

          /* BEMP0 Disable */

          hw_usb_clear_bempenb(USB_PIPE0);

          /* BRDY0 Disable */

          hw_usb_clear_brdyenb(USB_PIPE0);

          usb_hstd_ctrl_read_start();

          /* At this point every thing is done w.r.t hardware to setup
           * to receive the setup data... Now wait for interrupt
           */

          nxmutex_unlock(&priv->lock);

          if (priv->connected)
            {
             nxsem_wait_uninterruptible(&ed->wdhsem);
            }
        }

      else if (dirpid == GTD_STATUS_DP_OUT)
        {
          nxmutex_lock(&priv->lock);

          /* process setup packet status phase */

          usb_hstd_ctrl_write_start(buffer, buflen);

          nxmutex_unlock(&priv->lock);

          if (priv->connected)
            {
               nxsem_wait_uninterruptible(&ed->wdhsem);
            }

          /* Disable Empty Interrupt */

          hw_usb_clear_bempenb(USB_PIPE0);

          /* Disable Not Ready Interrupt */

          hw_usb_clear_nrdyenb(USB_PIPE0);
        }

      /* Check the TD completion status bits */

      if (xfrinfo->tdstatus == TD_CC_NOERROR)
        {
          ret = OK;
        }
      else
        {
          uerr("ERROR: Bad TD completion status: %d\n", xfrinfo->tdstatus);
          ret = xfrinfo->tdstatus == TD_CC_STALL ? -EPERM : -EIO;
        }
    }
  else
    {
    }

  /* Make sure that there is no outstanding request on this endpoint */

errout_with_xfrinfo:
  rx65n_usbhost_free_xfrinfo(xfrinfo);
  ed->xfrinfo = NULL;
  return ret;
}

/****************************************************************************
 * Name: rx65n_usbhost_usbinterrupt
 *
 * Description:
 *   USB interrupt handler
 *
 ****************************************************************************/

static int rx65n_usbhost_usbinterrupt(int irq, void *context, void *arg)
{
  uint16_t intenb0;
  uint16_t intenb1;
  uint16_t intsts0;
  uint16_t intsts1;
  uint16_t ack_interrupt;

  /* Read Interrupt Status and mask out interrupts that are not enabled. */

  intenb0 = rx65n_usbhost_getreg(RX65N_USB_INTENB0);
  intenb1 = rx65n_usbhost_getreg(RX65N_USB_INTENB1);
  intsts0 = rx65n_usbhost_getreg(RX65N_USB_INTSTS0);
  intsts1 = rx65n_usbhost_getreg(RX65N_USB_INTSTS1);

  if ((((intsts1 & intenb1) & RX65N_USB_INTSTS1_SACK)) ==
        RX65N_USB_INTSTS1_SACK)
    {
      hw_usb_hclear_sts_sack();

      /* Disable setup packet status response */

      rx65n_usbhost_clearbit(RX65N_USB_INTENB1, RX65N_USB_INTENB1_SACKE |
       RX65N_USB_INTENB1_SIGNE);

      /* release the EP0 ep->wdhsem semaphore */

      DEBUGASSERT(work_available(&g_usbhost.rx65n_interrupt_bhalf));

      DEBUGVERIFY(work_queue(HPWORK, &g_usbhost.rx65n_interrupt_bhalf,
       rx65n_usbhost_bottomhalf,
      (void *)USB_PROCESS_SACK_INT, 0));
    }

  /* Is it ERROR for Setup packet... */

  else if ((((intsts1 & intenb1) & RX65N_USB_INTSTS1_SIGN)) ==
    RX65N_USB_INTSTS1_SIGN)
    {
      hw_usb_hclear_sts_sign();

      /* Disable setup packet status response */

      rx65n_usbhost_clearbit(RX65N_USB_INTENB1, RX65N_USB_INTENB1_SACKE |
      RX65N_USB_INTENB1_SIGNE);
      DEBUGASSERT(work_available(&g_usbhost.rx65n_interrupt_bhalf));

      DEBUGVERIFY(work_queue(HPWORK, &g_usbhost.rx65n_interrupt_bhalf,
      rx65n_usbhost_bottomhalf,
      (void *)USB_PROCESS_SIGN_INT, 0));
    }

  /* Check for EOFERR interrupt... Not using it though */

  else if ((((intsts1 & intenb1) & RX65N_USB_INTSTS1_EOFERR)) ==
    RX65N_USB_INTSTS1_EOFERR)
    {
      rx65n_usbhost_putreg(((~RX65N_USB_INTSTS1_EOFERR) &
      INTSTS1_BIT_VALUES_TO_ACK), RX65N_USB_INTSTS1);
    }

  /* Check for over current condition... */

  else if ((((intsts1 & intenb1) & RX65N_USB_INTSTS1_OVRCRE)) ==
    RX65N_USB_INTSTS1_OVRCRE)
    {
      ack_interrupt = INTSTS1_BIT_VALUES_TO_ACK &
      (~(RX65N_USB_INTSTS1_OVRCRE));

      /* Acknowledge the OVRCR interrupt */

      rx65n_usbhost_putreg(ack_interrupt, RX65N_USB_INTSTS1);
    }

  /* Check for attach condition...  */

  else if ((((intsts1 & intenb1) & RX65N_USB_INTSTS1_ATTCH)) ==
    RX65N_USB_INTSTS1_ATTCH)
    {
      hw_usb_hclear_sts_attch();

      usb_hstd_bus_int_disable();
      DEBUGASSERT(work_available(&g_usbhost.rx65n_interrupt_bhalf));

      DEBUGVERIFY(work_queue(HPWORK, &g_usbhost.rx65n_interrupt_bhalf,
      rx65n_usbhost_bottomhalf,
      (void *)USB_PROCESS_ATTACHED_INT, 0));
    }

  /* Check for detach condition... */

  else if ((((intsts1 & intenb1) & RX65N_USB_INTSTS1_DTCH)) ==
    RX65N_USB_INTSTS1_DTCH)
    {
      hw_usb_hclear_sts_dtch();

      usb_hstd_bus_int_disable();
      DEBUGASSERT(work_available(&g_usbhost.rx65n_interrupt_bhalf));

      DEBUGVERIFY(work_queue(HPWORK, &g_usbhost.rx65n_interrupt_bhalf,
      rx65n_usbhost_bottomhalf,
      (void *)USB_PROCESS_DETACHED_INT, 0));
    }

  /* Check for BCHG interrupt... Not using it though */

  else if ((((intsts1 & intenb1) & RX65N_USB_INTSTS1_BCHG)) ==
     RX65N_USB_INTSTS1_BCHG)
    {
      hw_usb_hclear_sts_bchg();
      hw_usb_hclear_enb_bchge();
    }

  /* Check for BRDY interrupt... */

  else if ((((intsts0 & intenb0) & RX65N_USB_INTSTS0_BRDY)) ==
    RX65N_USB_INTSTS0_BRDY)
    {
      /* Schedule the workque for processing */

      DEBUGASSERT(work_available(&g_usbhost.rx65n_interrupt_bhalf));

      DEBUGVERIFY(work_queue(HPWORK, &g_usbhost.rx65n_interrupt_bhalf,
      rx65n_usbhost_bottomhalf,
      (void *)USB_PROCESS_BRDY_INT, 0));
    }

  /* Check for BEMP interrupt... */

  else if ((((intsts0 & intenb0) & RX65N_USB_INTSTS0_BEMP)) ==
    RX65N_USB_INTSTS0_BEMP)
    {
      /* Schedule the workque for processing */

      DEBUGASSERT(work_available(&g_usbhost.rx65n_interrupt_bhalf));

      DEBUGVERIFY(work_queue(HPWORK, &g_usbhost.rx65n_interrupt_bhalf,
      rx65n_usbhost_bottomhalf,
      (void *)USB_PROCESS_BEMP_INT, 0));
    }

  /* Check for NRDY interrupt... */

  else if ((((intsts0 & intenb0) & RX65N_USB_INTSTS0_NRDY)) ==
     RX65N_USB_INTSTS0_NRDY)
    {
      /* Schedule the workque for processing */

      DEBUGASSERT(work_available(&g_usbhost.rx65n_interrupt_bhalf));

      DEBUGVERIFY(work_queue(HPWORK, &g_usbhost.rx65n_interrupt_bhalf,
      rx65n_usbhost_bottomhalf,
      (void *)USB_PROCESS_NRDY_INT, 0));
    }

  /* Check for SOF interrupt... Not using though */

  else if ((((intsts0 & intenb0) & RX65N_USB_INTSTS0_SOFR)) ==
    RX65N_USB_INTSTS0_SOFR)
    {
       hw_usb_clear_sts_sofr();
    }

  /* Check for VBINT interrupt... Not using though */

  else if ((((intsts0 & intenb0) & RX65N_USB_INTSTS0_VBINT)) ==
    RX65N_USB_INTSTS0_VBINT)
    {
      rx65n_usbhost_clearbit(RX65N_USB_INTSTS0, RX65N_USB_INTSTS0_VBINT);
    }

  /* If none of the interrupt - what happens? */

  else
    {
      syslog(LOG_INFO, "Unhandled interrupt. INTENB0 = 0x%x, \
             INTENB1 = 0x%x, INTSTS0 = 0x%x and INTSTS1 = 0x%x\n",
             intenb0, intenb1, intsts0, intsts1);
    }

  return OK;
}

/****************************************************************************
 * Name: rx65n_usbhost_bottomhalf
 *
 * Description:
 * OHCI interrupt bottom half.  This function runs on the high priority
 * worker thread and was xcheduled when the last interrupt occurred.
 * Possibly this acts as host_thread functionality which is present in
 * FIT driver
 ****************************************************************************/

static void rx65n_usbhost_bottomhalf(void *arg)
{
  struct rx65n_usbhost_s *priv = &g_usbhost;
  uint32_t bottom_half_processing = (uint32_t)arg;
  uint16_t device_speed;

  /* connected_times variable is used to check the number of times */

  /* connected  and disconnected */

  static uint32_t connected_times = 0;

  /* We need to have exclusive access to the OHCI data structures.
   * Waiting here is not a good thing to do on the worker thread, but there
   * is no real option (other than to reschedule and delay).
   */

  nxmutex_lock(&g_usbhost.lock);

  if (bottom_half_processing == USB_PROCESS_SACK_INT)
    {
      EDCTRL->xfrinfo->tdstatus = TD_CC_NOERROR;
      hw_usb_hclear_sts_sack();
      nxsem_post(&EDCTRL->wdhsem);
    }

  else if (bottom_half_processing == USB_PROCESS_SIGN_INT)
    {
      EDCTRL->xfrinfo->tdstatus = TD_CC_PIDCHECKFAILURE;
      hw_usb_hclear_sts_sign();
      nxsem_post(&EDCTRL->wdhsem);
    }

  else if (bottom_half_processing == USB_PROCESS_ATTACHED_INT)
    {
      g_attached = true;
      device_speed = usb_hstd_attach_process();
      if (!priv->connected)
        {
          /* Yes.. connected. */

          connected_times++;
          syslog(LOG_INFO, "NuttX: USB Device Connected. %d\n",
                 connected_times);
          priv->connected = true;
          priv->change    = true;

          /* Update the speed of the connected device */

          if (device_speed == USB_ATTACHL)
            {
              g_usbhost.rhport.hport.speed = USB_SPEED_LOW;
            }
          else
            {
              g_usbhost.rhport.hport.speed = USB_SPEED_FULL;
            }

          /* Notify any waiters */

          if (priv->pscwait)
            {
              priv->pscwait = false;
              nxsem_post(&priv->pscsem);
            }
        }
      else
        {
          syslog(LOG_INFO, "WARNING: Spurious status change (connected)\n");
        }

      g_attached = false;
    }

  else if (bottom_half_processing == USB_PROCESS_DETACHED_INT)
    {
      g_detached = true;
      device_speed = usb_hstd_detach_process();

      if (priv->connected)
        {
          /* Yes.. disconnect the device */

          syslog(LOG_INFO, "NuttX: USB Device Disconnected. %d\n",
                 connected_times);
          priv->connected = false;
          priv->change    = true;

          /* As detach calls several free functions, make sure access to */

          /* hardware is available */

          nxmutex_unlock(&g_usbhost.lock);

          /* Are we bound to a class instance? */

          if (g_kbdpipe)
            {
              nxsem_post(&g_rx65n_edlist[g_kbdpipe].wdhsem);
            }

          g_kbdpipe = 0;
          if (priv->rhport.hport.devclass)
            {
              /* Yes.. Disconnect the class */

              CLASS_DISCONNECTED(priv->rhport.hport.devclass);
              priv->rhport.hport.devclass = NULL;
            }

          /* Notify any waiters for the Root Hub Status change event */

          if (priv->pscwait)
            {
              nxsem_post(&priv->pscsem);
              priv->pscwait = false;
            }

          return;
        }
      else
        {
           syslog(LOG_INFO, "WARNING: Spurious status change \
                  (disconnected)\n");
        }

      g_detached = false;
    }

  else if (bottom_half_processing == USB_PROCESS_BRDY_INT)
    {
      usb_hstd_brdy_pipe();
    }

  else if (bottom_half_processing == USB_PROCESS_BEMP_INT)
    {
      usb_hstd_bemp_pipe();
    }

  else if (bottom_half_processing == USB_PROCESS_NRDY_INT)
    {
      usb_hstd_nrdy_pipe();
    }

  /* If the bottom half is not any of the above then log the
   * reason for bottom half being called
   */

  else
    {
      nxsig_usleep(100);
      uwarn("WARNING: un known bottomhalf. Value is %d\n",
         bottom_half_processing);
      syslog(LOG_INFO, "WARNING: un known bottomhalf. Value is %d\n",
      bottom_half_processing);
    }

  nxmutex_unlock(&g_usbhost.lock);
}

/****************************************************************************
 * USB Host Controller Operations
 ****************************************************************************/

/****************************************************************************
 * Name: rx65n_usbhost_wait
 *
 * Description:
 *   Wait for a device to be connected or disconnected to/from a hub port.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter from
 *      the call to the USB driver initialization logic.
 *   hport - The location to return the hub port descriptor that detected
 *      the connection related event.
 *
 * Returned Value:
 *   Zero (OK) is returned on success when a device is connected or
 *   disconnected. This function will not return until either (1) a device
 *   is connected or disconnect to/from any hub port or until (2) some
 *   failure occurs.  On a failure, a negated errno value is returned
 *   indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int rx65n_usbhost_wait(struct usbhost_connection_s *conn,
                              struct usbhost_hubport_s **hport)
{
  struct rx65n_usbhost_s *priv = (struct rx65n_usbhost_s *)&g_usbhost;
  struct usbhost_hubport_s *connport;
  irqstate_t flags;
  int ret;

  flags = enter_critical_section();
  for (; ; )
    {
      /* Is there a change in the connection state of the single root hub
       * port?
       */

      if (priv->change)
        {
          connport = &priv->rhport.hport;
          priv->change = false;

          /* Yes.. check for false alarms */

          if (priv->connected != connport->connected)
            {
              /* Not a false alarm.. Remember the new state */

              connport->connected = priv->connected;

              /* And return the root hub port */

              *hport = connport;
              leave_critical_section(flags);

              uinfo("RHport Connected: %s\n",
                    connport->connected ? "YES" : "NO");

              return OK;
            }
        }

#ifdef CONFIG_USBHOST_HUB
      /* Is a device connected to an external hub? */

      if (priv->hport)
        {
          /* Yes.. return the external hub port */

          connport = (struct usbhost_hubport_s *)priv->hport;
          priv->hport = NULL;

          *hport = connport;
          leave_critical_section(flags);

          uinfo("Hub port Connected: %s\n",
                connport->connected ? "YES" : "NO");
          return OK;
        }

#endif
      /* Wait for the next connection event */

      priv->pscwait = true;
      ret = nxsem_wait_uninterruptible(&priv->pscsem);
      if (ret < 0)
        {
          return ret;
        }
    }
}

/****************************************************************************
 * Name: rx65n_usbhost_enumerate
 *
 * Description:
 *   Enumerate the connected device.  As part of this enumeration process,
 *   the driver will (1) get the device's configuration descriptor, (2)
 *   extract the class ID info from the configuration descriptor, (3) call
 *   usbhost_findclass() to find the class that supports this device, (4)
 *   call the create() method on the struct usbhost_registry_s interface
 *   to get a class instance, and finally (5) call the connect() method
 *   of the struct usbhost_class_s interface.  After that, the class is in
 *   charge of the sequence of operations.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter from
 *      the call to the USB driver initialization logic.
 *   hport - The descriptor of the hub port that has the newly connected
 *      device.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int rx65n_usbhost_rh_enumerate(struct usbhost_connection_s *conn,
                                      struct usbhost_hubport_s *hport)
{
  struct rx65n_usbhost_s *priv = (struct rx65n_usbhost_s *)&g_usbhost;
  DEBUGASSERT(conn != NULL && hport != NULL && hport->port == 0);

  /* Are we connected to a device?  The caller should have called the wait()
   * method first to be assured that a device is connected.
   */

  while (!priv->connected)
    {
      /* No, return an error */

      uwarn("WARNING: Not connected\n");

      return -ENODEV;
    }

  /* USB 2.0 spec says at least 50ms delay before port reset */

  nxsig_usleep(100 * 1000);

  /* Put RH port 1 in reset.
   * Currently supporting only single downstream port)
   */

  nxsig_usleep(200 * 1000);
  return OK;
}

/****************************************************************************
 * Name: rx65n_usbhost_enumerate
 *
 * Description:
 *   Enumerate the connected device.  As part of this enumeration process,
 *   the driver will (1) get the device's configuration descriptor, (2)
 *   extract the class ID info from the configuration descriptor, (3) call
 *   usbhost_findclass() to find the class that supports this device, (4)
 *   call the create() method on the struct usbhost_registry_s interface
 *   to get a class instance, and finally (5) call the connect() method
 *   of the struct usbhost_class_s interface.  After that, the class is in
 *   charge of the sequence of operations.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter from
 *      the call to the USB driver initialization logic.
 *   hport - The descriptor of the hub port that has the newly connected
 *      device.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int rx65n_usbhost_enumerate(struct usbhost_connection_s *conn,
                                   struct usbhost_hubport_s *hport)
{
  int ret;

  DEBUGASSERT(hport);

  /* If this is a connection on the root hub, then we need to go to
   * little more effort to get the device speed.  If it is a connection
   * on an external hub, then we already have that information.
   */

#ifdef CONFIG_USBHOST_HUB
  if (ROOTHUB(hport))
#endif
    {
      ret = rx65n_usbhost_rh_enumerate(conn, hport);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Then let the common usbhost_enumerate do the real enumeration. */

  uinfo("Enumerate the device\n");

  ret = usbhost_enumerate(hport, &hport->devclass);
  if (ret < 0)
    {
      uerr("ERROR: Enumeration failed: %d\n", ret);
      syslog(LOG_INFO, "ERROR: Enumeration failed: %d\n", ret);
    }

  else
    {
      syslog(LOG_INFO, "Root Hub Port device enumerated");
    }

  return ret;
}

/****************************************************************************
 * Name: rx65n_usbhost_ep0configure
 *
 * Description:
 *   Configure endpoint 0.  This method is normally used internally by the
 *   enumerate() method but is made available at the interface to support
 *   an external implementation of the enumeration logic.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from
 *      the call to the class create() method.
 *   ep0 - The (opaque) EP0 endpoint instance
 *   funcaddr - The USB address of the function containing the endpoint
 *     that EP0 controls
 *   speed - The speed of the port USB_SPEED_LOW, _FULL, or _HIGH
 *   mps (maxpacketsize) - The maximum number of bytes that can be sent
 *    to or received from the endpoint in a single data packet
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno
 *   value is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int rx65n_usbhost_ep0configure(struct usbhost_driver_s *drvr,
    usbhost_ep_t ep0,
    uint8_t funcaddr, uint8_t speed,
    uint16_t maxpacketsize)
{
  struct rx65n_usbhost_s *priv = (struct rx65n_usbhost_s *)drvr;
  uint16_t current_dcpmaxp;

  DEBUGASSERT(drvr != NULL && ep0 != NULL &&
    funcaddr < 128 && maxpacketsize < 2048);

  /* We must have exclusive access to EP0 and the control list */

  nxmutex_lock(&priv->lock);
  usb_cstd_set_nak(USB_PIPE0);

  /* Make sure, all the DEVADDn registers are set to default state */

  /* if (funcaddr == 0) */

  hw_usb_hset_usbspd(funcaddr, (speed << 6));

  /* else
   * hw_usb_hset_usbspd (funcaddr, (1 << 6));
   * debug_ptr = RX65N_USB_DEVADD0+funcaddr;
   */

  current_dcpmaxp = hw_usb_read_dcpmaxp();
  current_dcpmaxp = current_dcpmaxp & (~RX65N_USB_DCPMAXP_MXPS_MASK);
  current_dcpmaxp |= maxpacketsize;
  current_dcpmaxp = current_dcpmaxp & (~RX65N_USB_DCPMAXP_DEVADDR_MASK);
  current_dcpmaxp |= funcaddr <<  RX65N_USB_DCPMAXP_DEVADDR_SHIFT;
  hw_usb_write_dcpmxps(current_dcpmaxp);

  hw_usb_set_curpipe(USB_CUSE, USB_PIPE0);
  hw_usb_set_bclr(USB_CUSE);

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: rx65n_usbhost_epalloc
 *
 * Description:
 *   Allocate and configure one endpoint.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *      call to the class create() method.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno
 *   value is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int rx65n_usbhost_epalloc(struct usbhost_driver_s *drvr,
                                 const struct usbhost_epdesc_s *epdesc,
                                 usbhost_ep_t *ep)
{
  struct rx65n_usbhost_s *priv = (struct rx65n_usbhost_s *)drvr;
  struct usbhost_hubport_s *hport;
  struct rx65n_usbhost_ed_s *ed;
  int ret  = -ENOMEM;
  uint8_t pipe_num;
  uint8_t pipe_type;
  uint8_t pipe_dir;

  /* Sanity check.  NOTE that this method should only be called if a
   * device is connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(priv && epdesc && ep && priv->connected);

  /* We must have exclusive access to the ED pool, the bulk list, the
   * periodic list and the interrupt table.
   */

  nxmutex_lock(&priv->lock);

  /* Take the ED descriptor from the list of ED Array - based on pipe num
   * Also note it down as part of ED structurie itself
   * for futer use - if needed
   * Take the next ED from the beginning of the free list
   */

  /* DEBUGASSERT(pipe_no == USB_NULL); */

  if (epdesc->in)
    {
       pipe_dir = USB_EP_IN;
    }
  else
    {
      pipe_dir = USB_EP_OUT;
    }

  if ((epdesc->xfrtype) == USB_EP_ATTR_XFER_CONTROL)
    {
      pipe_type = USB_EP_CTRL;
    }
  else if ((epdesc->xfrtype) == USB_EP_ATTR_XFER_BULK)
    {
      pipe_type = USB_EP_BULK;
    }
  else if ((epdesc->xfrtype) == USB_EP_ATTR_XFER_INT)
    {
      pipe_type = USB_EP_INT;
    }
  else
    {
      pipe_type = USB_EP_ISO;
    }

  pipe_num = usb_hstd_get_pipe_no(pipe_type, pipe_dir);
  DEBUGASSERT(pipe_no == USB_NULL);
  g_usb_pipe_table[pipe_num].use_flag = USB_TRUE;

  ed =  &g_rx65n_edlist[pipe_num];
  if (ed)
    {
      /* Remove the ED from the freelist */

      /* Configure the endpoint descriptor. */

      memset((void *)ed, 0, sizeof(struct rx65n_usbhost_ed_s));

      hport = epdesc->hport;
      ed->hw.ctrl = (uint32_t)(hport->funcaddr) << ED_CONTROL_FA_SHIFT |
                    (uint32_t)(epdesc->addr)    << ED_CONTROL_EN_SHIFT |
                    (uint32_t)(epdesc->mxpacketsize) << ED_CONTROL_MPS_SHIFT;

      /* Note down the pipe number for reference */

      ed->pipenum = pipe_num;

      /* Get the direction of the endpoint.  For control endpoints, the
       * direction is in the TD.
       */

      if (epdesc->xfrtype == USB_EP_ATTR_XFER_CONTROL)
        {
          ed->hw.ctrl |= ED_CONTROL_D_TD1;
        }
      else if (epdesc->in)
        {
          ed->hw.ctrl |= ED_CONTROL_D_IN;
        }
      else
        {
          ed->hw.ctrl |= ED_CONTROL_D_OUT;
        }

      /* Check for a low-speed device */

      if (hport->speed == USB_SPEED_LOW)
        {
          ed->hw.ctrl |= ED_CONTROL_S;
        }

      /* Set the transfer type */

      ed->xfrtype = epdesc->xfrtype;

      /* Special Case isochronous transfer types */

      uinfo("EP%d CTRL:%08x\n", epdesc->addr, ed->hw.ctrl);
      nxsem_init(&ed->wdhsem, 0, 0);

      /* Link the common tail TD to the ED's TD list */

      ed->hw.headp = (uint32_t)TDTAIL;
      ed->hw.tailp = (uint32_t)TDTAIL;

      /* Now add the endpoint descriptor to the appropriate list */

      switch (ed->xfrtype)
        {
        case USB_EP_ATTR_XFER_CONTROL:
        ret = rx65n_usbhost_addctrled(priv, ed);
        break;

        case USB_EP_ATTR_XFER_BULK:
        ret = rx65n_usbhost_addbulked(priv, epdesc, ed);
        break;

        case USB_EP_ATTR_XFER_INT:
        ret = rx65n_usbhost_addinted(priv, epdesc, ed);
        break;

        case USB_EP_ATTR_XFER_ISOC:
        ret = rx65n_usbhost_addisoced(priv, epdesc, ed);
        break;

        default:
        ret = -EINVAL;
        break;
        }

      /* Was the ED successfully added? */

      if (ret < 0)
        {
          /* No.. destroy it and report the error */

          uerr("ERROR: Failed to queue ED for transfer type: %d\n",
            ed->xfrtype);
          nxsem_destroy(&ed->wdhsem);
          rx65n_usbhost_edfree(ed);
        }
      else
        {
          /* Yes.. return an opaque reference to the ED */

          *ep = (usbhost_ep_t)ed;
        }
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: rx65n_usbhost_epfree
 *
 * Description:
 *   Free and endpoint previously allocated by DRVR_EPALLOC.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *      call to the class create() method.
 *   ep - The endpint to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno
 *   value is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int rx65n_usbhost_epfree(struct usbhost_driver_s *drvr,
                                usbhost_ep_t ep)
{
  struct rx65n_usbhost_s *priv = (struct rx65n_usbhost_s *)drvr;
  struct rx65n_usbhost_ed_s *ed   = (struct rx65n_usbhost_ed_s *)ep;
  int ret;

  /* There should not be any pending, real TDs linked to this ED */

  DEBUGASSERT(ed && (ed->hw.headp & ED_HEADP_ADDR_MASK) == TDTAIL_ADDR);

  /* We must have exclusive access to the ED pool, the bulk list,
   * the periodic list and the interrupt table.
   */

  nxmutex_lock(&priv->lock);

  /* Remove the ED to the correct list depending on the trasfer type */

  switch (ed->xfrtype)
    {
    case USB_EP_ATTR_XFER_CONTROL:
    ret = rx65n_usbhost_remctrled(priv, ed);
    break;

    case USB_EP_ATTR_XFER_BULK:
    ret = rx65n_usbhost_rembulked(priv, ed);
    break;

    case USB_EP_ATTR_XFER_INT:
    ret = rx65n_usbhost_reminted(priv, ed);
    break;

    case USB_EP_ATTR_XFER_ISOC:
    ret = rx65n_usbhost_remisoced(priv, ed);
    break;

    default:
    ret = -EINVAL;
    break;
    }

  /* Put the ED back into the free list */

  rx65n_usbhost_edfree(ed);
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: rx65n_usbhost_alloc
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor
 *   data can be accessed more efficiently.  This method provides a mechanism
 *   to allocate the request/descriptor memory.  If the underlying hardware
 *   does not support such "special" memory, this functions may
 *   simply map to kmm_malloc.
 *
 *   This interface was optimized under a particular assumption.  It was
 *   assumed that the driver maintains a pool of small, pre-allocated buffers
 *   for descriptor traffic.  NOTE that size is not an input, but an output:
 *   The size of the pre-allocated buffer is returned.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *      call to the class create() method.
 *   buffer - The address of a memory location provided by the caller in
 *     which to return the allocated buffer memory address.
 *   maxlen - The address of a memory location provided by the caller in
 *     which to return the maximum size of the allocated buffer memory.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value i
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int rx65n_usbhost_alloc(struct usbhost_driver_s *drvr,
                               uint8_t **buffer, size_t *maxlen)
{
  struct rx65n_usbhost_s *priv = (struct rx65n_usbhost_s *)drvr;

  DEBUGASSERT(priv && buffer && maxlen);
  int ret = -ENOMEM;

  /* We must have exclusive access to the transfer buffer pool */

  nxmutex_lock(&priv->lock);

  *buffer = rx65n_usbhost_tballoc();
  if (*buffer)
    {
      *maxlen = CONFIG_RX65N_USBHOST_TDBUFSIZE;
      ret = OK;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: rx65n_usbhost_free
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor
 *   data can be accessed more efficiently.  This method provides a
 *   mechanism to free that request/descriptor memory.  If the underlying
 *   hardware does not support such "special" memory, this functions may
 *   simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *      call to the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno
 *   value is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int rx65n_usbhost_free(struct usbhost_driver_s *drvr, uint8_t *buffer)
{
  struct rx65n_usbhost_s *priv = (struct rx65n_usbhost_s *)drvr;
  DEBUGASSERT(buffer);

  /* We must have exclusive access to the transfer buffer pool */

  nxmutex_lock(&priv->lock);
  rx65n_usbhost_tbfree(buffer);
  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: rx65n_usbhost_ioalloc
 *
 * Description:
 *   Some hardware supports special memory in which larger IO buffers can
 *   be accessed more efficiently.  This method provides a mechanism to
 *   allocate the request/descriptor memory.  If the underlying hardware
 *   does not support such "special" memory, this functions may simply map
 *   to kmm_malloc.
 *
 *   This interface differs from DRVR_ALLOC in that the buffers are
 *                      variable-sized.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *      call to the class create() method.
 *   buffer - The address of a memory location provided by the caller in
 *     which to return the allocated buffer memory address.
 *   buflen - The size of the buffer required.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno
 *   returned indicating the nature of the failure
 *   value is
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int rx65n_usbhost_ioalloc(struct usbhost_driver_s *drvr,
                                 uint8_t **buffer, size_t buflen)
{
  uint8_t *alloc;

  DEBUGASSERT(drvr && buffer && buflen > 0);

  /* There is no special memory requirement */

  alloc = (uint8_t *)kmm_malloc(buflen);
  if (!alloc)
    {
      return -ENOMEM;
    }

  /* Return the allocated buffer */

  *buffer = alloc;
  return OK;
}

/****************************************************************************
 * Name: rx65n_usbhost_iofree
 *
 * Description:
 *   Some hardware supports special memory in which IO data can  be
 *   accessed more efficiently.  This method provides a mechanism to free
 *   that IO buffer memory. If the underlying hardware does not support
 *   such "special" memory,  this functions may simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *      call to the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *    is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int rx65n_usbhost_iofree(struct usbhost_driver_s *drvr,
                                uint8_t *buffer)
{
  DEBUGASSERT(drvr && buffer);

#if RX65N_USBHOST_IOBUFFERS > 0
  rx65n_usbhhost_freeio(buffer);
  return OK;
#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: rx65n_usbhost_ctrlin and rx65n_usbhost_ctrlout
 *
 * Description:
 * Description:
 *   Process a IN or OUT request on the control endpoint.  These methods
 *   will enqueue the request and wait for it to complete.  Only one
 *   transfer may be queued; Neither these methods nor the transfer() method
 *   can be called again until the control transfer functions returns.
 *
 *   These are blocking methods; these functions will not return until the
 *   control transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *      call to the class create() method.
 *   ep0 - The control endpoint to send/receive the control request.
 *   req - Describes the request to be sent.  This request must lie in
 *      memory created by DRVR_ALLOC.
 *   buffer - A buffer used for sending the request and for returning any
 *     responses.  This buffer must be large enough to hold the length value
 *     in the request description. buffer must have been allocated using
 *      DRVR_ALLOC.
 *
 *   NOTE: On an IN transaction, req and buffer may refer to the same
 *   allocated memory.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated
 *   errno value is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int rx65n_usbhost_ctrlin(struct usbhost_driver_s *drvr,
                                usbhost_ep_t ep0,
                                const struct usb_ctrlreq_s *req,
                                uint8_t *buffer)
{
  struct rx65n_usbhost_s *priv = (struct rx65n_usbhost_s *)drvr;
  struct rx65n_usbhost_ed_s *ed = (struct rx65n_usbhost_ed_s *)ep0;
  uint16_t len;
  int ret;
  uint8_t req_type;
  uint8_t req_req;

  uint8_t *local_buf;
  local_buf = buffer;

  DEBUGASSERT(priv != NULL && ed != NULL && req != NULL);

  uinfo("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], req->len[1], req->len[0]);

  /* We must have exclusive access to EP0 and the control list */

  len = rx65n_usbhost_getle16(req->len);
  req_type = req->type;
  req_req = req->req;

  if (req_type == (USB_REQ_DIR_IN | USB_REQ_TYPE_CLASS |
    USB_REQ_RECIPIENT_INTERFACE) &&
    (req_req == USBHID_REQUEST_GETREPORT))
    {
      /* No need to check for this class request */
    }
  else
    {
      ed->xfrinfo->tdxfercond = USB_SETUPRD;
      ret = rx65n_usbhost_ctrltd(priv, ed, GTD_STATUS_DP_SETUP,
      (uint8_t *)req, USB_SIZEOF_CTRLREQ);

      if (ret == OK)
        {
          if (len)
            {
              ed->xfrinfo->tdxfercond = USB_DATARD;
              ret = rx65n_usbhost_ctrltd(priv, ed, GTD_STATUS_DP_IN,
              buffer, len);
              g_usbidx = USB0.USBINDX;
            }

      if (ret == OK)
        {
          ed->xfrinfo->tdxfercond = USB_STATUSWR;
          ret = rx65n_usbhost_ctrltd(priv, ed,
                GTD_STATUS_DP_OUT, NULL, 0);
        }
    }
    }

  /* If this is Get Report request */

  if (req_type == (USB_REQ_DIR_IN | USB_REQ_TYPE_CLASS |
                USB_REQ_RECIPIENT_INTERFACE) &&
                (req_req == USBHID_REQUEST_GETREPORT))
    {
      nxsem_wait_uninterruptible(
        &g_rx65n_edlist[kbd_interrupt_in_pipe].wdhsem);

          *(local_buf + 0) = kbd_report_data [0];
          *(local_buf + 1) = kbd_report_data [1];
          *(local_buf + 2) = kbd_report_data [2];
          *(local_buf + 3) = kbd_report_data [3];
          *(local_buf + 4) = kbd_report_data [4];
          *(local_buf + 5) = kbd_report_data [5];
          *(local_buf + 6) = kbd_report_data [6];
          *(local_buf + 7) = kbd_report_data [7];
          ret = OK;

          /* Reset the transferred count so that for the next interrupt */

          /* for read does the calculation correctly */

          g_rx65n_edlist[kbd_interrupt_in_pipe].xfrinfo->xfrd = 0;
          usb_cstd_set_buf(kbd_interrupt_in_pipe); /* Set BUF */
    }

  return ret;
}

static int rx65n_usbhost_ctrlout(struct usbhost_driver_s *drvr,
  usbhost_ep_t ep0, const struct usb_ctrlreq_s *req,
                        const uint8_t *buffer)
{
  struct rx65n_usbhost_s *priv = (struct rx65n_usbhost_s *)drvr;
  struct rx65n_usbhost_ed_s *ed = (struct rx65n_usbhost_ed_s *)ep0;
  uint16_t len;
  int ret;
  static int dev_addressed_state = 0;

  /* Assumption : This control out is called first time for
   * set address command. Just resetting the bus after the
   * set address command
   */

  if (dev_addressed_state == 0)
    {
      /* Not sure, if the reset is needed or not - at least the FIT code
       * does not have reset...
       * so removing it...
       * usb_hstd_bus_reset();
       */

      dev_addressed_state = 0xff;
    }

  DEBUGASSERT(priv != NULL && ed != NULL && req != NULL);

  uinfo("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
  req->type, req->req, req->value[1], req->value[0],
  req->index[1], req->index[0], req->len[1], req->len[0]);

  /* We must have exclusive access to EP0 and the control list */

  len = rx65n_usbhost_getle16(req->len);
  ed->xfrinfo->tdxfercond = USB_SETUPWR;
  ret = rx65n_usbhost_ctrltd(priv, ed, GTD_STATUS_DP_SETUP,
  (uint8_t *)req, USB_SIZEOF_CTRLREQ);

  if (ret == OK)
    {
      if (len)
        {
          ed->xfrinfo->tdxfercond = USB_DATAWR;
          ret = rx65n_usbhost_ctrltd(priv, ed, GTD_STATUS_DP_OUT,
          (uint8_t *)buffer, len);
        }

      if (ret == OK)
        {
          ed->xfrinfo->tdxfercond = USB_STATUSRD;
          ret = rx65n_usbhost_ctrltd(priv, ed, GTD_STATUS_DP_IN, NULL, 0);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: rx65n_usbhost_transfer_common
 *
 * Description:
 *   Initiate a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request and return immediately
 *
 * Input Parameters:
 *   priv - Internal driver state structure.
 *   ed - The IN or OUT endpoint descriptor for the device endpoint on
 *      which to perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or
 *     received (IN endpoint). buffer must have been allocated using
 *     DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno
 *   value is returned indicating the nature of the failure.
 *
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int rx65n_usbhost_transfer_common(struct rx65n_usbhost_s *priv,
                                         struct rx65n_usbhost_ed_s *ed,
                                         uint8_t *buffer, size_t buflen)
{
  struct rx65n_usbhost_xfrinfo_s *xfrinfo;
  uint32_t dirpid;
  bool in;
  int ret;

  xfrinfo = ed->xfrinfo;
  in      = (ed->hw.ctrl & ED_CONTROL_D_MASK) == ED_CONTROL_D_IN;

  uinfo("EP%u %s toggle:%u maxpacket:%u buflen:%lu\n",
        (ed->hw.ctrl  & ED_CONTROL_EN_MASK) >> ED_CONTROL_EN_SHIFT,
        in ? "IN" : "OUT",
        (ed->hw.headp & ED_HEADP_C) != 0 ? 1 : 0,
        (ed->hw.ctrl  & ED_CONTROL_MPS_MASK) >> ED_CONTROL_MPS_SHIFT,
        (unsigned long)buflen);

  /* Get the direction of the endpoint */

  if (in)
    {
      dirpid = GTD_STATUS_DP_IN;
    }
  else
    {
      dirpid = GTD_STATUS_DP_OUT;
    }

  /* Then enqueue the transfer */

  xfrinfo->tdstatus = TD_CC_NOERROR;

  ret = rx65n_usbhost_enqueuetd(priv, ed, dirpid, GTD_STATUS_T_TOGGLE,
    buffer, buflen);

  if (ed->pipenum == USB_PIPE6 && in == 1)
    {
      usb_hstd_receive_start(buffer, buflen, ed->pipenum);
    }

  if (ret == OK)
    {
      /* BulkListFilled. This bit is used to indicate whether there are any
       * TDs on the Bulk list.
       */

      if (ed->xfrtype == USB_EP_ATTR_XFER_BULK)
        {
          if (in)
            {
              usb_hstd_receive_start(buffer, buflen, ed->pipenum);
            }
          else
            {
              usb_hstd_send_start(buffer, buflen, ed->pipenum);
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: rx65n_usbhost_dma_alloc
 *
 * Description:
 *   Allocate DMA memory to perform a transfer, copying user data as
 *   necessary
 * Input Parameters:
 *   priv - Internal driver state structure.
 *   ed - The IN or OUT endpoint descriptor for the device endpoint on which
 *      to perform the transfer.
 *   userbuffer - The user buffer containing the data to be sent (OUT
 *      endpoint) or received (IN endpoint).
 *   buflen - The length of the data to be sent or received.
 *   alloc - The location to return the allocated DMA buffer.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

#if RX65N_USBHOST_IOBUFFERS > 0
static int rx65n_usbhost_dma_alloc(struct rx65n_usbhost_s *priv,
                                   struct rx65n_usbhost_ed_s *ed,
                                   uint8_t *userbuffer, size_t buflen,
                                   uint8_t **alloc)
{
  syslog(LOG_INFO, "Debug : %s(): Line : %d\n", __func__, __LINE__);

  /* This need to be impemented if DMA is used */

  return OK;
}

/****************************************************************************
 * Name: rx65n_usbhost_dma_free
 *
 * Description:
 *   Free allocated DMA memory.
 *
 * Input Parameters:
 *   priv - Internal driver state structure.
 *   ed - The IN or OUT endpoint descriptor for the device endpoint
 *      on which to perform the transfer.
 *   userbuffer - The user buffer containing the data to be sent
 *      (OUT endpoint) or received (IN endpoint).
 *   buflen - The length of the data to be sent or received.
 *   alloc - The allocated DMA buffer to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno
 *   value is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static void rx65n_usbhost_dma_free(struct rx65n_usbhost_s *priv,
                                   struct rx65n_usbhost_ed_s *ed,
                                   uint8_t *userbuffer, size_t buflen,
                                   uint8_t *newbuffer)
{
  syslog(LOG_INFO, "Debug : %s(): Line : %d\n", __func__, __LINE__);
}
#endif

/****************************************************************************
 * Name: rx65n_usbhost_transfer
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request, blocking until the transfer completes.
 *   Only one transfer may be  queued; Neither this method nor the ctrlin or
 *   ctrlout methods can be called again until this function returns.
 *
 *   This is a blocking method; this functions will not return until the
 *   transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *      call to the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on
 *      which to perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or
 *     received (IN endpoint).  buffer must have been allocated using
 *     DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *
 * Returned Value:
 *   On success, a non-negative value is returned that indicates the number
 *   of bytes successfully transferred.  On a failure, a negated errno
 *   value is returned that indicates the nature of the failure:
 *
 *     EAGAIN - If devices NAKs the transfer (or NYET or other error where
 *              it may be appropriate to restart the entire transaction).
 *     EPERM  - If the endpoint stalls
 *     EIO    - On a TX or data toggle error
 *     EPIPE  - Overrun errors
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static ssize_t rx65n_usbhost_transfer(struct usbhost_driver_s *drvr,
                                      usbhost_ep_t ep,
                                      uint8_t *buffer, size_t buflen)
{
  struct rx65n_usbhost_s *priv = (struct rx65n_usbhost_s *)drvr;
  struct rx65n_usbhost_ed_s *ed = (struct rx65n_usbhost_ed_s *)ep;
  struct rx65n_usbhost_xfrinfo_s *xfrinfo;
#if RX65N_USBHOST_IOBUFFERS > 0
  uint8_t *alloc = NULL;
  uint8_t *userbuffer = NULL;
#endif
  ssize_t nbytes;
  int ret;
  uint8_t transfer_retry_count = 0;

  DEBUGASSERT(priv && ed && buffer && buflen > 0);

  if (nrdy_retries[ed->pipenum] != 0)
    {
      /* nRdy has occured alreday - just return with -ve value,
       * so that file close is also completes with this error
       *
       */

      return -EBUSY;
    }

  do
    {
      /* We must have exclusive access to the endpoint, the TD pool, the I/O
       * buffer pool, the bulk and interrupt lists, and the HCCA interrupt
       * table.
       *
       */

      nxmutex_lock(&priv->lock);

      /* Allocate a structure to retain the information needed when the
       * transfer completes.
       *
       */

      DEBUGASSERT(ed->xfrinfo == NULL);

      xfrinfo = rx65n_usbhost_alloc_xfrinfo();
      if (xfrinfo == NULL)
        {
          uerr("ERROR: rx65n_usbhost_alloc_xfrinfo failed\n");
          nbytes = -ENOMEM;
          nxmutex_unlock(&priv->lock);
          goto errout_with_lock;
        }

      /* Newly added condition */

      if (xfrinfo < 0)
        {
          uerr("ERROR: rx65n_usbhost_alloc_xfrinfo failed\n");
          nbytes = -ENOMEM;
          goto errout_with_xfrinfo;
        }

      /* Initialize the transfer structure */

      memset(xfrinfo, 0, sizeof(struct rx65n_usbhost_xfrinfo_s));
      xfrinfo->buffer = buffer;
      xfrinfo->buflen = buflen;

      /* To begin with transferred bytes = 0 */

      xfrinfo->xfrd = 0;

      ed->xfrinfo = xfrinfo;

#if RX65N_USBHOST_IOBUFFERS > 0
      /* Allocate an IO buffer if the user buffer does not lie in AHB SRAM */

      ret = rx65n_usbhost_dma_alloc(priv, ed, buffer, buflen, &alloc);
      if (ret < 0)
        {
          uerr("ERROR: rx65n_usbhost_dma_alloc failed: %d\n", ret);
          nbytes = (ssize_t)ret;
          goto errout_with_xfrinfo;
        }

      /* If a buffer was allocated, then use it instead of the
       * callers buffer
       *
       */

      if (alloc)
        {
          userbuffer = buffer;
          buffer  = alloc;
        }
#endif

      /* Set the request for the Writeback Done Head event well
       * BEFORE enabling
       * the transfer.
       */

      ret = rx65n_usbhost_wdhwait(priv, ed);
      if (ret < 0)
        {
          uerr("ERROR: Device disconnected\n");
          nbytes = (ssize_t)ret;
          goto errout_with_buffers;
        }

      /* Set up the transfer */

      ret = rx65n_usbhost_transfer_common(priv, ed, buffer, buflen);
      nxmutex_unlock(&priv->lock);
      if (ret < 0)
        {
          uerr("ERROR: rx65n_usbhost_transfer_common failed: %d\n", ret);
          nbytes = (ssize_t)ret;
          goto errout_with_wdhwait;
        }

      /* Wait for the Write-back Done Head interrupt */

      if (priv->connected)
        {
          nxsem_wait_uninterruptible(&ed->wdhsem);
        }

      /* Update the buffer pointer for next buffer operation */

      /* Check the TD completion status bits */

      if (xfrinfo->tdstatus == TD_CC_NOERROR)
        {
          /* Return the number of bytes successfully transferred */

          nbytes = xfrinfo->xfrd;
          DEBUGASSERT(nbytes >= 0 && nbytes <= buflen);
        }
  else
    {
      /* Map the bad completion status to something that a class driver
       * might understand.
       */

      uerr("ERROR: Bad TD completion status: %d\n", xfrinfo->tdstatus);

      switch (xfrinfo->tdstatus)
        {
        case TD_CC_STALL:
          nbytes = -EPERM;
          break;

        /* This case is newly added for NRDY issue */

        case TD_CC_DEVNOTRESPONDING:
          xfrinfo->wdhwait = false;
          transfer_retry_count++;
          nbytes = -EBUSY;
          break;

        case TD_CC_USER:
          nbytes = -ESHUTDOWN;
          break;

        default:
          nbytes = -EIO;
          break;
        }
     }

errout_with_wdhwait:

  /* Make sure that there is no outstanding request on this endpoint */

  xfrinfo->wdhwait = false;

errout_with_buffers:
#if RX65N_USBHOST_IOBUFFERS > 0

  /* Free any temporary IO buffers */

  rx65n_usbhost_dma_free(priv, ed, userbuffer, buflen, alloc);
#endif

errout_with_xfrinfo:

  /* Make sure that there is no outstanding request on this endpoint */

  rx65n_usbhost_free_xfrinfo(xfrinfo);
  ed->xfrinfo = NULL;
  }
  while (0);

errout_with_lock:

  /* nxmutex_unlock(&priv->lock); */

  return nbytes;
}

/****************************************************************************
 * Name: rx65n_usbhost_asynch_completion
 *
 * Description:
 *   This function is called at the interrupt level when an asynchronous
 *   transfer completes.  It performs the pending callback.
 *
 * Input Parameters:
 *   priv - Internal driver state structure.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which
 *      the transfer was performed.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Called from the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static void rx65n_usbhost_asynch_completion(struct rx65n_usbhost_s *priv,
                                            struct rx65n_usbhost_ed_s *ed)
{
  struct rx65n_usbhost_xfrinfo_s *xfrinfo;
  usbhost_asynch_t callback;
  void *arg;
  ssize_t nbytes;

  DEBUGASSERT(ed != NULL && ed->xfrinfo != NULL);
  xfrinfo = ed->xfrinfo;

  DEBUGASSERT(xfrinfo->wdhwait == false &&  xfrinfo->callback != NULL &&
              xfrinfo->buffer != NULL && xfrinfo->buflen > 0);

  /* Check the TD completion status bits */

  if (xfrinfo->tdstatus == TD_CC_NOERROR)
    {
      /* Provide the number of bytes successfully transferred */

      nbytes = xfrinfo->xfrd;
    }
  else
    {
      /* Map the bad completion status to something that a class driver
       * might understand.
       */

      uerr("ERROR: Bad TD completion status: %d\n", xfrinfo->tdstatus);

      switch (xfrinfo->tdstatus)
        {
        case TD_CC_STALL:
        nbytes = -EPERM;
        break;

        case TD_CC_USER:
        nbytes = -ESHUTDOWN;
        break;

        default:
        nbytes = -EIO;
        break;
        }
     }

#if RX65N_USBHOST_IOBUFFERS > 0

  /* Free any temporary IO buffers */

  rx65n_usbhost_dma_free(priv, ed, xfrinfo->buffer, xfrinfo->buflen,
  xfrinfo->alloc);
#endif

  /* Extract the callback information before freeing the buffer */

  callback = xfrinfo->callback;
  arg = xfrinfo->arg;

  /* Make sure that there is no outstanding request on this endpoint */

  rx65n_usbhost_free_xfrinfo(xfrinfo);
  ed->xfrinfo  = NULL;

  /* Then perform the callback */

  callback(arg, nbytes);
}
#endif

/****************************************************************************
 * Name: rx65n_usbhost_asynch
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request and return immediately.  When the transfer
 *   completes, the callback will be invoked with the provided transfer.
 *   This method is useful for receiving interrupt transfers which may come
 *   infrequently.
 *
 *   Only one transfer may be queued; Neither this method nor the ctrlin or
 *   ctrlout methods can be called again until the transfer completes.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *      call to the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which
 *      to perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or
 *     received (IN endpoint).  buffer must have been allocated using
 *         DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *   callback - This function will be called when the transfer completes.
 *   arg - The arbitrary parameter that will be passed to the callback
 *     function when the transfer completes.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int rx65n_usbhost_asynch(struct usbhost_driver_s *drvr,
                                usbhost_ep_t ep,
                                uint8_t *buffer, size_t buflen,
                                usbhost_asynch_t callback, void *arg)
{
  struct rx65n_usbhost_s *priv = (struct rx65n_usbhost_s *)drvr;
  struct rx65n_usbhost_ed_s *ed = (struct rx65n_usbhost_ed_s *)ep;
  struct rx65n_usbhost_xfrinfo_s *xfrinfo;
  int ret;

  DEBUGASSERT(priv && ed && ed->xfrinfo == NULL &&
  buffer && buflen > 0 && callback);

  /* We must have exclusive access to the endpoint, the TD pool, the I/O
   * buffer pool, the bulk and interrupt lists, and the HCCA interrupt table.
   */

  nxmutex_lock(&priv->lock);

  /* Allocate a structure to retain the information needed when the
   * asynchronous transfer completes.
   */

  DEBUGASSERT(ed->xfrinfo == NULL);

  xfrinfo = rx65n_usbhost_alloc_xfrinfo();
  if (xfrinfo == NULL)
    {
      uerr("ERROR: rx65n_usbhost_alloc_xfrinfo failed\n");
      ret = -ENOMEM;
      goto errout_with_lock;
    }

  /* Initialize the transfer structure */

  memset(xfrinfo, 0, sizeof(struct rx65n_usbhost_xfrinfo_s));
  xfrinfo->buffer   = buffer;
  xfrinfo->buflen   = buflen;
  xfrinfo->callback = callback;
  xfrinfo->arg      = arg;

  ed->xfrinfo       = xfrinfo;

#if RX65N_USBHOST_IOBUFFERS > 0
  /* Allocate an IO buffer if the user buffer does not lie in AHB SRAM */

  ret = rx65n_usbhost_dma_alloc(priv, ed, buffer, buflen, &xfrinfo->alloc);
  if (ret < 0)
    {
      uerr("ERROR: rx65n_usbhost_dma_alloc failed: %d\n", ret);
      goto errout_with_lock;
    }

  /* If a buffer was allocated, then use it instead of the callers buffer */

  if (xfrinfo->alloc)
    {
      buffer  = xfrinfo->alloc;
    }
#endif

  /* Set up the transfer */

  ret = rx65n_usbhost_transfer_common(priv, ed, buffer, buflen);
  if (ret < 0)
    {
      uerr("ERROR: rx65n_usbhost_transfer_common failed: %d\n", ret);
      goto errout_with_asynch;
    }

  /* And return now.  The callback will be invoked when the transfer
   * completes.
   */

  /* Enable Ready Interrupt */

  nxmutex_unlock(&priv->lock);
  return OK;

errout_with_asynch:
#if RX65N_USBHOST_IOBUFFERS > 0

  /* Free any temporary IO buffers */

  rx65n_usbhost_dma_free(priv, ed, buffer, buflen, xfrinfo->alloc);
#endif

  /* Free the transfer structure */

  rx65n_usbhost_free_xfrinfo(xfrinfo);
  ed->xfrinfo = NULL;

errout_with_lock:
  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif /* CONFIG_USBHOST_ASYNCH */

/****************************************************************************
 * Name: rx65n_usbhost_cancel
 *
 * Description:
 *   Cancel a pending transfer on an endpoint.  Cancelled synchronous or
 *   asynchronous transfer will complete normally with the error -ESHUTDOWN.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *      call to the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on
 *      which an asynchronous transfer should be transferred.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 ****************************************************************************/

static int rx65n_usbhost_cancel(struct usbhost_driver_s *drvr,
                                usbhost_ep_t ep)
{
#ifdef CONFIG_USBHOST_ASYNCH
  struct rx65n_usbhost_s *priv = (struct rx65n_usbhost_s *)drvr;
#endif
  struct rx65n_usbhost_xfrinfo_s *xfrinfo;
  irqstate_t flags;

  /* These first steps must be atomic as possible */

  flags = enter_critical_section();

  /* It is possible there there is no transfer to be in progress */

  xfrinfo = g_rx65n_edlist[USB_PIPE6].xfrinfo;
  if (xfrinfo)
    {
      /* It might be possible for no transfer to be in progress (callback ==
       * NULL and wdhwait == false)
       */

#ifdef CONFIG_USBHOST_ASYNCH
      if (xfrinfo->callback || xfrinfo->wdhwait)
#else
      if (xfrinfo->wdhwait)
#endif
        {
          /* Control endpoints should not come through this path and
           * isochronous endpoints are not yet implemented.  So we only have
           * to distinguish bulk and interrupt endpoints.
           */

          if (g_rx65n_edlist[USB_PIPE6].xfrtype == USB_EP_ATTR_XFER_BULK)
            {
              g_rx65n_edlist[USB_PIPE6].hw.headp = (uint32_t)TDTAIL;
              g_rx65n_edlist[USB_PIPE6].xfrinfo  = NULL;
            }
          else
            {
              /* Remove the TDs attached to the ED, keeping the Ed in the
               * list.
               */

              g_rx65n_edlist[USB_PIPE6].hw.headp = (uint32_t)TDTAIL;
            }

          xfrinfo->tdstatus = TD_CC_USER;

          /* If there is a thread waiting for the transfer to complete, then
           * wake up the thread.
           */

          if (xfrinfo->wdhwait)
            {
#ifdef CONFIG_USBHOST_ASYNCH
              /* Yes.. there should not also be a callback scheduled */

              DEBUGASSERT(xfrinfo->callback == NULL);
#endif

              /* Wake up the waiting thread */

              nxsem_post(&g_rx65n_edlist[USB_PIPE6].wdhsem);

              /* And free the transfer structure */

              rx65n_usbhost_free_xfrinfo(xfrinfo);
              g_rx65n_edlist[USB_PIPE6].xfrinfo = NULL;
            }
#ifdef CONFIG_USBHOST_ASYNCH
          else
            {
              /* Otherwise, perform the callback and free the transfer
               * structure.
               */

              rx65n_usbhost_asynch_completion(priv,
              &g_rx65n_edlist[USB_PIPE6]);
              usb_cstd_clr_pipe_cnfg(USB_PIPE6);
              g_usb_pipe_table[USB_PIPE6].use_flag = USB_FALSE;
            }
#endif
        }
      else
        {
          /* Just free the transfer structure */

          rx65n_usbhost_free_xfrinfo(xfrinfo);
          g_rx65n_edlist[USB_PIPE6].xfrinfo = NULL;
        }
    }

  /* Determine the return value */

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: rx65n_usbhost_connect
 *
 * Description:
 *   New connections may be detected by an attached hub.  This method is the
 *   mechanism that is used by the hub class to introduce a new connection
 *   and port description to the system.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *      call to the class create() method.
 *   hport - The descriptor of the hub port that detected the connection
 *      related event
 *   connected - True: device connected; false: device disconnected
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno
 *   value is returned indicating the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_HUB
static int rx65n_usbhost_connect(struct usbhost_driver_s *drvr,
                                 struct usbhost_hubport_s *hport,
                                 bool connected)
{
  struct rx65n_usbhost_s *priv = (struct rx65n_usbhost_s *)drvr;
  DEBUGASSERT(priv != NULL && hport != NULL);

  int ret;

  /* Set the connected/disconnected flag */

  hport->connected = connected;
  uinfo("Hub port %d connected: %s\n", hport->port,
    connected ? "YES" : "NO");

  /* Report the connection event */

  priv->hport = hport;
  ret = usbhost_enumerate(hport, &hport->devclass);
  if (ret < 0)
    {
      syslog(LOG_INFO, "Enumeration failed with %d", ret);
    }

  hw_usb_write_dcpmxps(USB_DEFPACKET + USB_DEVICE_1);

  if (hport->speed == USB_SPEED_LOW)
    {
      switch (hport->port)
        {
          case HUB_PORT1:
          case HUB_PORT2:
          case HUB_PORT3:
          case HUB_PORT4:
          g_kbdport = hport->port;
          g_hubkbd = true;
          break;

          default:
          syslog(LOG_INFO, "Undefined Port");
          break;
        }
    }

  if (ret >= 0)
    {
      syslog(LOG_INFO, "Hub Port device enumerated\n");
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: rx65n_usbhost_disconnect
 *
 * Description:
 *   Called by the class when an error occurs and driver has been
 *       disconnected. The USB host driver should discard the handle to the
 *   class instance (it is stale) and not attempt any further interaction
 *   with the class driver instance (until a new instance is received
 *   from the create() method).
 *   The driver should not called the class' disconnected() method.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *      call to the class create() method.
 *   hport - The port from which the device is being disconnected.
 *      Might be a port on a hub.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static void rx65n_usbhost_disconnect(struct usbhost_driver_s *drvr,
                                     struct usbhost_hubport_s *hport)
{
  int i;
  struct rx65n_usbhost_s *priv = &g_usbhost;
  uint16_t pipe;

  struct rx65n_usbhost_xfrinfo_s *xfrinfo;
  uint8_t *buffer;
  DEBUGASSERT(hport != NULL);

  if (hport->port)
    {
      if (hport->speed == USB_SPEED_LOW)
        {
          g_usb_pipe_table[kbd_interrupt_in_pipe].use_flag = USB_FALSE;

          for (i = 0, xfrinfo = g_xfrbuffers;
               i < CONFIG_RX65N_USBHOST_NPREALLOC;
               i++, xfrinfo++)
            {
              /* Put the transfer structure in a free list */
#ifdef CONFIG_USBHOST_ASYNCH
              if (!(xfrinfo->callback))
#endif
                {
                  rx65n_usbhost_free_xfrinfo(xfrinfo);
                  g_rx65n_edlist[kbd_interrupt_in_pipe].xfrinfo = NULL;
                }
            }

          g_kbdpipe = 0;
          g_hubkbd = false;
          syslog(LOG_INFO, "KBD Device Disconnected from Hub\n");
        }

      if (hport->speed == USB_SPEED_FULL)
        {
          for (pipe = USB_BULK_PIPE_START ; pipe <= USB_BULK_PIPE_END;
               pipe++)
            {
              if (g_usb_pipe_table[pipe].use_flag == USB_TRUE)
                {
                  g_usb_pipe_table[pipe].use_flag = USB_FALSE;
                }
            }

          syslog(LOG_INFO, "MSC Device Disconnected from Hub\n");
        }
    }

  else if (!hport->port)
    {
      if (g_usbhost.rhport.hport.speed == USB_SPEED_LOW)
        {
          g_usb_pipe_table[kbd_interrupt_in_pipe].use_flag = USB_FALSE;
          for (i = 0, xfrinfo = g_xfrbuffers;
               i < CONFIG_RX65N_USBHOST_NPREALLOC;
               i++, xfrinfo++)
            {
              /* Put the transfer structure in a free list */

               rx65n_usbhost_free_xfrinfo(xfrinfo);
               g_rx65n_edlist[kbd_interrupt_in_pipe].xfrinfo = NULL;
            }
        }

      if (g_usbhost.rhport.hport.speed == USB_SPEED_FULL)
        {
          if (g_hubkbd)
            {
              /* If Keyboard device is connected, and the USB Hub disconnect
               * task is invoked, then make the Hub task sleep,
               * so that in this time the keyboard task completes
               * its disconnection event.
               *
               */

              nxsig_usleep(100000);
            }

          for (i = 0; i < CONFIG_RX65N_USBHOST_NEDS; i++)
            {
              /* Put the ED in a free list */

              rx65n_usbhost_edfree(&g_rx65n_edlist[i]);
            }

          /* Initialize user-configurable TDs */

          for (i = 0; i < CONFIG_RX65N_USBHOST_NTDS; i++)
            {
              /* Put the TD in a free list */

              rx65n_usbhost_tdfree(&g_rx65n_tdlist[i]);
            }

          /* Initialize user-configurable request/descriptor transfer
           * buffers
           */

          buffer = g_tdbuffer;

          for (i = 0; i < CONFIG_RX65N_USBHOST_TDBUFFERS; i++)
            {
              /* Put the TD buffer in a free list */

              rx65n_usbhost_tbfree(buffer);
              buffer += CONFIG_RX65N_USBHOST_TDBUFSIZE;
            }

          /* Initialize transfer structures */

          for (i = 0, xfrinfo = g_xfrbuffers;
               i < CONFIG_RX65N_USBHOST_NPREALLOC;
               i++, xfrinfo++)
            {
              /* Put the transfer structure in a free list */

              rx65n_usbhost_free_xfrinfo(xfrinfo);
            }

          /* Wait 50MS then perform hardware reset */

          up_mdelay(50);

          /* Set up the root hub port EP0 */

          rx65n_usbhost_ep0init(priv);

          /* To begin with make all pipes are available */

          for (i = USB_MIN_PIPE_NUM; i < (USB_MAX_PIPE_NUM +1); i++)
            {
              g_usb_pipe_table[i].use_flag = USB_FALSE;
            }
        }
    }

  hport->devclass = NULL;
}

/****************************************************************************
 * Initialization
 ****************************************************************************/

/****************************************************************************
 * Name: rx65n_usbhost_ep0init
 *
 * Description:
 *   Initialize ED for EP0, add it to the control ED list, and enable
 *   control transfers.
 *
 * Input Parameters:
 *   priv - private driver state instance.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void rx65n_usbhost_ep0init(struct rx65n_usbhost_s *priv)
{
  /* Initialize the common tail TD. */

  memset(TDTAIL, 0, sizeof(struct rx65n_usbhost_gtd_s));
  TDTAIL->ed = EDCTRL;

  /* Link the common tail TD to the ED's TD list */

  memset(EDCTRL, 0, sizeof(struct rx65n_usbhost_ed_s));
  EDCTRL->hw.headp = (uint32_t)TDTAIL;
  EDCTRL->hw.tailp = (uint32_t)TDTAIL;
  EDCTRL->xfrtype  = USB_EP_ATTR_XFER_CONTROL;

  /* Set the head of the control list to the NULL (for now). */

  /* Then add EP0 to the empty Control List */

  rx65n_usbhost_addctrled(priv, EDCTRL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rx65n_usbhost_initialize
 *
 * Description:
 *   Initialize USB host device controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than USB host controller, then
 *     this identifies which controller is being initialized.  Normally, this
 *     is just zero.
 *
 * Returned Value:
 *   And instance of the USB host interface.  The controlling task should
 *   use this interface to (1) call the wait() method to wait for a device
 *   to be connected, and (2) call the enumerate() method to bind the
 *   device to a class driver.
 *
 * Assumptions:
 * - This function should called in the initialization sequence in order
 *   to initialize the USB device functionality.
 * - Class drivers should be initialized prior to calling this function.
 *   Otherwise, there is a race condition if the device is already .
 *   connected
 ****************************************************************************/

struct usbhost_connection_s *rx65n_usbhost_initialize(int controller)
{
  struct rx65n_usbhost_s *priv = &g_usbhost;
  struct usbhost_driver_s *drvr;
  struct usbhost_hubport_s *hport;
  struct rx65n_usbhost_xfrinfo_s *xfrinfo;
  uint32_t reg32;
  uint8_t *buffer;
  irqstate_t flags;
  int i;

  DEBUGASSERT(controller == 0);
  DEBUGASSERT(sizeof(struct rx65n_ed_s)  <= RX65N_ED_SIZE);
  DEBUGASSERT(sizeof(struct rx65n_usbhost_gtd_s) <= RX65N_TD_SIZE);

  /* Initialize all the TDs, EDs (including EDCTRL i.e.
   * Control Endpoint) and HCCA to 0
   */

  /* Set HCCA base address */

  HCCA = &g_hcca;

  /* Set TD TAIL address */

  TDTAIL = &g_rx65n_tdlist[0];

  /* Set ED Tail address as well... */

  /* EDCTRL = &g_rx65n_ep0ed; */

  /* Set first element as CTRL pipe - as there is always
   * only one CTRL pipe
   */

  EDCTRL = &g_rx65n_edlist[0];

  memset((void *)TDTAIL, 0,
                ((sizeof(struct ohci_gtd_s)) * (CONFIG_RX65N_USBHOST_NTDS)));
  memset((void *)(&g_rx65n_edlist[0]), 0,
                ((sizeof(struct rx65n_usbhost_ed_s)) *
                (CONFIG_RX65N_USBHOST_NEDS)));
  memset((void *)EDCTRL, 0, sizeof(struct rx65n_usbhost_ed_s));
  memset((void *)HCCA,   0, sizeof(struct ohci_hcca_s));

  /* Initialize the state data structure */

  /* Initialize the device operations */

  drvr                 = &priv->drvr;
  drvr->ep0configure   = rx65n_usbhost_ep0configure;
  drvr->epalloc        = rx65n_usbhost_epalloc;
  drvr->epfree         = rx65n_usbhost_epfree;
  drvr->alloc          = rx65n_usbhost_alloc;
  drvr->free           = rx65n_usbhost_free;
  drvr->ioalloc        = rx65n_usbhost_ioalloc;
  drvr->iofree         = rx65n_usbhost_iofree;
  drvr->ctrlin         = rx65n_usbhost_ctrlin;
  drvr->ctrlout        = rx65n_usbhost_ctrlout;
  drvr->transfer       = rx65n_usbhost_transfer;
#ifdef CONFIG_USBHOST_ASYNCH
  drvr->asynch         = rx65n_usbhost_asynch;
#endif
  drvr->cancel         = rx65n_usbhost_cancel;
#ifdef CONFIG_USBHOST_HUB
  drvr->connect        = rx65n_usbhost_connect;
#endif
  drvr->disconnect     = rx65n_usbhost_disconnect;

  /* Initialize the public port representation */

  hport                = &priv->rhport.hport;
  hport->drvr          = drvr;
#ifdef CONFIG_USBHOST_HUB
  hport->parent        = NULL;
#endif
  hport->ep0           = EDCTRL;
  hport->speed         = USB_SPEED_FULL;
  hport->funcaddr      = 0;

  /* Initialize function address generation logic */

  usbhost_devaddr_initialize(&priv->rhport);

#ifndef CONFIG_USBHOST_INT_DISABLE
  priv->ininterval  = MAX_PERINTERVAL;
  priv->outinterval = MAX_PERINTERVAL;
#endif

  /* Enable write to System registers */

  putreg16(RX65N_PRCR_VALUE, RX65N_PRCR_ADDR);

  /* Start CMT module */

  reg32 = getreg32(RX65N_MSTPCRB_ADDR);

  /* Clear bit 19 - so that USB module is released from stop state */

  reg32 &= (~RX65N_MSTPCRB_START_STOP_USB);
  putreg32(reg32, RX65N_MSTPCRB_ADDR);

  reg32 = getreg32(RX65N_MSTPCRB_ADDR);

  /* Enable power by setting PCUSB in the PCONP register.
   * Disable interrupts because this register may be shared with other
   * drivers.
   */

  flags = enter_critical_section();
  putreg32(0, RX65N_USB_DPUSR0R); /* FIT code writes 0 to this DPUSR0R */

  hw_usb_hmodule_init();
  rx65n_usbhost_setbit(RX65N_USB_SOFCFG, RX65N_USB_SOFCFG_TRNENSEL);

  hw_usb_set_vbout();
  up_mdelay(100);

  leave_critical_section(flags);
  nxsem_init(&EDCTRL->wdhsem, 0, 0);

  /* Initialize user-configurable EDs */

  for (i = 0; i < CONFIG_RX65N_USBHOST_NEDS; i++)
    {
      /* Put the ED in a free list */

      rx65n_usbhost_edfree(&g_rx65n_edlist[i]);
    }

  /* Initialize user-configurable TDs */

  for (i = 0; i < CONFIG_RX65N_USBHOST_NTDS; i++)
    {
      /* Put the TD in a free list */

      rx65n_usbhost_tdfree(&g_rx65n_tdlist[i]);
    }

  /* Initialize user-configurable request/descriptor transfer
   * buffers
   */

  buffer = g_tdbuffer;

  for (i = 0; i < CONFIG_RX65N_USBHOST_TDBUFFERS; i++)
    {
      /* Put the TD buffer in a free list */

      rx65n_usbhost_tbfree(buffer);
      buffer += CONFIG_RX65N_USBHOST_TDBUFSIZE;
    }

#if RX65N_USBHOST_IOBUFFERS > 0
  /* Initialize user-configurable IO buffers */

  buffer = (uint8_t *)RX65N_USBHOST_IOFREE_BASE;
  for (i = 0; i < RX65N_USBHOST_IOBUFFERS; i++)
    {
      /* Put the IO buffer in a free list */

      rx65n_usbhhost_freeio(buffer);
      buffer += CONFIG_RX65N_USBHOST_IOBUFSIZE;
    }
#endif

  /* Initialize transfer structures */

  for (i = 0, xfrinfo = g_xfrbuffers;
       i < CONFIG_RX65N_USBHOST_NPREALLOC;
       i++, xfrinfo++)
    {
      /* Put the transfer structure in a free list */

      rx65n_usbhost_free_xfrinfo(xfrinfo);
    }

  /* Wait 50MS then perform hardware reset */

  up_mdelay(50);

  /* Set up the root hub port EP0 */

  rx65n_usbhost_ep0init(priv);

  /* To begin with make all pipes are available */

  for (i = USB_MIN_PIPE_NUM; i < (USB_MAX_PIPE_NUM +1); i++)
    {
       g_usb_pipe_table[i].use_flag = USB_FALSE;
    }

  /* Attach USB host controller interrupt handler */

  ICU.SLIBR185.BYTE = 0x3eu;
  IPR(PERIB, INTB185) = _0F_CMTW_PRIORITY_LEVEL15;

  if (irq_attach(RX65N_INTB185_IRQ, rx65n_usbhost_usbinterrupt,
  NULL) != 0)
    {
      syslog(LOG_INFO, "ERROR: Failed to attach IRQ\n");
      return NULL;
    }

  IEN(PERIB, INTB185) = 1U;

  syslog(LOG_INFO, "Debug:USB host Initialized, Device connected:%s\n",
         priv->connected ? "YES" : "NO");

  return &g_usbconn;
}
