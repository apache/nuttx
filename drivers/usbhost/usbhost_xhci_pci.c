/****************************************************************************
 * drivers/usbhost/usbhost_xhci_pci.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <sys/endian.h>

#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/wqueue.h>
#include <nuttx/addrenv.h>
#include <nuttx/spinlock.h>

#include <nuttx/pci/pci.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbhost_devaddr.h>
#include <nuttx/usb/usbhost_trace.h>

#include "usbhost_xhci.h"
#include "usbhost_xhci_trace.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pre-requisites */

#if CONFIG_USBHOST_XHCI_MAX_DEVS > XHCI_MAX_DEVS
#  error Invalid value for CONFIG_USBHOST_XHCI_MAX_DEVS
#endif

/* USB HUB support is not yet implemented */

#ifdef CONFIG_USBHOST_HUB
#  error XHCI USB HUB support is not yet implemented
#endif

/* Some constants for this implementation */

#define XHCI_MAX_ERST            (1)
#define XHCI_CMD_MAX             (16)
#define XHCI_EVENT_MAX           (232)
#define XHCI_TD_MAX              (8)
#define XHCI_BUFSIZE             (512)

/* Port numbers macros */

#define HPNDX(hp)                ((hp)->port)
#define HPORT(hp)                (HPNDX(hp) + 1)
#define RHPNDX(rh)               ((rh)->hport.hport.port)
#define RHPORT(rh)               (RHPNDX(rh) + 1)

/* Other helper macros */

#define XHCI_XCONN_FROM_CONN(c)  ((FAR struct usbhost_conn_xhci_s *)c)
#define XHCI_PRIV_FROM_CONN(c)   (XHCI_XCONN_FROM_CONN(c)->priv)
#define XHCI_RHPORT_FROM_DRVR(d) ((FAR struct xhci_rhport_s *)d)
#define XHCI_PRIV_FROM_RHPORT(r) (r->priv)
#define XHCI_PRIV_FROM_DRVR(d)   (XHCI_PRIV_FROM_RHPORT(XHCI_RHPORT_FROM_DRVR(d)))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* USB device state
 *
 * Reference:
 *   - 4.5.3: Slot States
 */

enum xhci_slot_e
{
  XHCI_SLOT_DISABLED,
  XHCI_SLOT_ENABLED,
  XHCI_SLOT_DEFAULT,
  XHCI_SLOT_ADDRESSED,
  XHCI_SLOT_CONFIGURED,
};

/* Ring state */

struct xhci_ring_s
{
  FAR struct xhci_trb_s *ring;  /* Ring */
  size_t                 i;     /* Ring pointer */
  size_t                 len;   /* Ring length */
  bool                   ccs;   /* Consumer Cycle State */
};

/* EP info */

struct xhci_epinfo_s
{
  uint8_t            epno:7;       /* Endpoint number */
  uint8_t            dirin:1;      /* 1:IN endpoint 0:OUT endpoint */
  uint8_t            toggle:1;     /* Next data toggle */
#ifndef CONFIG_USBHOST_INT_DISABLE
  uint8_t            interval;     /* Polling interval */
#endif
  uint8_t            devaddr;      /* Device address returned from xHCI */
  uint8_t            status;       /* Retained token status bits (for debug purposes) */
  bool               iocwait;      /* TRUE: Thread is waiting for transfer completion */
  uint8_t            xfrtype:2;    /* See USB_EP_ATTR_XFER_* definitions in usb.h */
  int                result;       /* The result of the transfer */
  size_t             xfrd;         /* On completion, will hold the number of bytes transferred */
  size_t             buflen;       /* Buffer length used for transfer */
  sem_t              iocsem;       /* Semaphore used to wait for transfer completion */
#ifdef CONFIG_USBHOST_ASYNCH
  usbhost_asynch_t   callback;     /* Transfer complete callback */
  FAR void          *arg;          /* Argument that accompanies the callback */
#endif
  struct xhci_ring_s td;           /* TD ring for this endpoint */
  uint8_t            slot;         /* Slot where this EP resides */

  /* These fields are used in the split-transaction protocol. */

  uint8_t           hubaddr;      /* USB device address of the high-speed hub below
                                   * which a full/low-speed device is attached.
                                   */
  uint8_t           hubport;      /* The port on the above high-speed hub. */
};

/* This structure retains the state of one root hub port */

struct xhci_rhport_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbhost_s
   * to struct xhci_rhport_s.
   */

  struct usbhost_driver_s       drvr;

  /* Root hub port status */

  bool                          connected;  /* Connected to device */
  int8_t                        slot;       /* Slot ID associated with this port */
  struct xhci_epinfo_s          ep0;        /* EP0 endpoint info */
  struct usbhost_roothubport_s  hport;      /* This is the hub port description understood
                                             * by class drivers
                                             */
  FAR struct usbhost_xhci_s    *priv;       /* Reference to xHCI instance */
  FAR struct xhci_dev_s        *dev;        /* Device reference */
};

/* USB Devices xhci data */

struct xhci_dev_s
{
  uint8_t                          state;   /* Slot stat */
  uint8_t                          slot;    /* Slot ID associated with this device */
  FAR struct xhci_dev_ctx_s       *ctx;     /* Output Device Context. Managed by xHC */
  FAR struct xhci_input_dev_ctx_s *input;   /* Input Device Context. Input to xHC */
  FAR struct xhci_rhport_s        *rhport;  /* Root Hub Port associated with this device */

  /* Reference to allocated endpoints */

  FAR struct xhci_epinfo_s *epinfo[XHCI_MAX_ENDPOINTS];
};

/* This structure contains the internal, private state of the xhci driver */

struct usbhost_xhci_s
{
#ifdef CONFIG_USBHOST_HUB
  FAR struct usbhost_hubport_s *hport;      /* Used to pass external hub port events */
#endif
  struct usbhost_devaddr_s      devgen;     /* Address generation data */
  bool                          pscwait;    /* TRUE: Thread is waiting for port status change event */
  sem_t                         pscsem;     /* Semaphore to wait for port status change events */
  mutex_t                       lock;       /* Support mutually exclusive access */
  spinlock_t                    spinlock;

  /* xHCI parameters */

  uint8_t                       no_ports;   /* Number of USB Ports */
  uint8_t                       no_slots;   /* Maximum number of Device Slots (one per USB device) */
  uint8_t                       no_scratch; /* Number of scratch buffers */
  uint8_t                       no_erst;    /* Event Ring Segment Table size */

  /* xHCI data */

  FAR struct xhci_rhport_s     *rhport;     /* Root hub ports */
  FAR struct xhci_dev_s        *devs;       /* USB device xHC data. One entry per
                                             * one supported USB device.
                                             */

  /* Allocated buffers for controller */

  FAR uint64_t                 *pg_ctx;     /* Device Context (no_slots + 1 elements).
                                             * Slot 0 reserved for Scratchpad Buffer Array
                                             */
  FAR uint64_t                 *pg_sb;      /* Scratchpad Buffer Array (no_scratch elements) */
  FAR struct xhci_event_ring_s *pg_erst;    /* Event Ring Segment Table */

  /* Event ring handling */

  struct xhci_ring_s            evnt;       /* Event ring handler */

  /* Command ring handling */

  sem_t                         cmdsem;     /* Command done semaphore */
  struct xhci_trb_s             cmdres;     /* Command result */
  struct xhci_ring_s            cmd;        /* Command ring handler */

  /* PCI data */

  FAR struct pci_device_s      *pcidev;     /* PCI device reference */
  int                           irq;        /* IRQ number used by the device */
  uint32_t                      pending;    /* IRQ pending status */
  struct work_s                 work;       /* IRQ work */
  struct work_s                 pscwork;    /* Port status change work */
  uint64_t                      base;       /* xHCI base address */
  uint64_t                      capa_base;  /* Capability base */
  uint64_t                      oper_base;  /* Operational base */
  uint64_t                      runt_base;  /* Runtime base */
  uint64_t                      door_base;  /* Doorbell base */
};

/* xHCI connection monitoring */

struct usbhost_conn_xhci_s
{
  struct usbhost_connection_s  conn;  /* Connection monitoring */
  FAR struct usbhost_xhci_s   *priv;  /* Reference to xHCI instance */
  int                          pid;   /* Waiter thread PID */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers ******************************************************************/

static uint32_t xhci_capa_getreg(FAR struct usbhost_xhci_s *priv,
                                 unsigned int offset);

static uint8_t xhci_capa_getreg_1b(FAR struct usbhost_xhci_s *priv,
                                   unsigned int offset);
static void xhci_capa_putreg_1b(FAR struct usbhost_xhci_s *priv,
                                unsigned int offset,
                                uint8_t value);

static uint32_t xhci_oper_getreg(FAR struct usbhost_xhci_s *priv,
                                 unsigned int offset);
static void xhci_oper_putreg(FAR struct usbhost_xhci_s *priv,
                             unsigned int offset,
                             uint32_t value);

static void xhci_oper_putreg_8b(FAR struct usbhost_xhci_s *priv,
                                unsigned int offset,
                                uint64_t value);

static uint32_t xhci_runt_getreg(FAR struct usbhost_xhci_s *priv,
                                 unsigned int offset);
static void xhci_runt_putreg(FAR struct usbhost_xhci_s *priv,
                             unsigned int offset,
                             uint32_t value);

static void xhci_runt_putreg_8b(FAR struct usbhost_xhci_s *priv,
                                unsigned int offset,
                                uint64_t value);

static void xhci_door_putreg(FAR struct usbhost_xhci_s *priv,
                             unsigned int offset,
                             uint32_t value);

/* Byte stream access helper functions **************************************/

static inline uint16_t xhci_getle16(FAR const uint8_t *val);

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_USB_INFO
static void xhci_dump_capa_reg(FAR struct usbhost_xhci_s *priv,
                               FAR const char *msg, unsigned int offset);
static void xhci_dump_oper_reg(FAR struct usbhost_xhci_s *priv,
                               FAR const char *msg, unsigned int offset);
static void xhci_dump_runt_reg(FAR struct usbhost_xhci_s *priv,
                               FAR const char *msg, unsigned int offset);
static void xhci_dump_mem(FAR struct usbhost_xhci_s *priv,
                          FAR const char *msg);
#endif

/* Ring management **********************************************************/

static int xhci_ring_init(FAR struct xhci_ring_s *ring, size_t len);
static void xhci_ring_deinit(FAR struct xhci_ring_s *ring);
static void xhci_ring_reset(FAR struct xhci_ring_s *ring, bool swap_ccs);
static void xhci_add_trb(FAR struct usbhost_xhci_s *priv,
                         FAR struct xhci_ring_s *ring,
                         FAR struct xhci_trb_s *trb,
                         int len);

/* xHCI operations **********************************************************/

static int xhci_bios_wait(FAR struct usbhost_xhci_s *priv);
static int xhci_ctrl_start(FAR struct usbhost_xhci_s *priv);
static int xhci_ctrl_halt(FAR struct usbhost_xhci_s *priv);
static int xhci_ctrl_reset(FAR struct usbhost_xhci_s *priv);

/* Port management **********************************************************/

static void xhci_probe_ports(FAR struct usbhost_xhci_s *priv);
static int xhci_port_enable(FAR struct usbhost_xhci_s *priv,
                            FAR struct usbhost_hubport_s *hport);

/* Slot management **********************************************************/

static void xhci_dcbaa_set(FAR struct usbhost_xhci_s *priv, uint8_t index,
                           uintptr_t ctx);
static void xhci_ep_configure(FAR struct usbhost_xhci_s *priv,
                              FAR struct xhci_ep_ctx_s *ctx,
                              uint8_t type, uint16_t maxpkt,
                              uint8_t maxburst, uint64_t tr_dp,
                              uint8_t mult, uint8_t interval);
static int xhci_address_set(FAR struct usbhost_xhci_s *priv,
                            FAR struct xhci_rhport_s *rhport, bool setaddr);
static int xhci_slot_init(FAR struct usbhost_xhci_s *priv,
                          FAR struct xhci_dev_s *dev);
static int xhci_device_init(FAR struct usbhost_xhci_s *priv,
                            FAR struct xhci_rhport_s *rhport);
static int xhci_device_deinit(FAR struct usbhost_xhci_s *priv,
                              FAR struct xhci_rhport_s *rhport);
static inline uint8_t xhci_epno_get(FAR struct xhci_epinfo_s *epinfo);
static void xhci_context_ctrl(FAR struct usbhost_xhci_s *priv,
                              FAR struct xhci_dev_s *dev,
                              uint32_t drop, uint32_t add);

/* Command handling *********************************************************/

static int xhci_command(FAR struct usbhost_xhci_s *priv,
                        FAR struct xhci_trb_s *trb, uint16_t timeout_ms);
static int xhci_cmd_sloten(FAR struct usbhost_xhci_s *priv,
                           FAR uint8_t *slot);
static int xhci_cmd_slotdis(FAR struct usbhost_xhci_s *priv, uint8_t slot);
static int xhci_cmd_setaddr(FAR struct usbhost_xhci_s *priv, uint8_t slot,
                            uint64_t ctx, bool bsr);
static int xhci_cmd_cfgep(FAR struct usbhost_xhci_s *priv, uint8_t slot,
                          uint64_t ctx, bool deconfig);
static int xhci_cmd_stopep(FAR struct usbhost_xhci_s *priv, uint8_t slot,
                           uint8_t ep, bool suspend);
static int xhci_cmd_evalctx(FAR struct usbhost_xhci_s *priv, uint8_t slot,
                            uint64_t ctx);

/* Transfer handling ********************************************************/

static void xhci_ep_doorbell(FAR struct usbhost_xhci_s *priv,
                             FAR struct xhci_epinfo_s *epinfo);
static int xhci_ioc_setup(FAR struct xhci_rhport_s *rhport,
                          FAR struct xhci_epinfo_s *epinfo,
                          size_t buflen);
static int xhci_ioc_wait(FAR struct xhci_epinfo_s *epinfo);
#ifdef CONFIG_USBHOST_ASYNCH
static inline int xhci_ioc_async_setup(FAR struct xhci_rhport_s *rhport,
                                       FAR struct xhci_epinfo_s *epinfo,
                                       usbhost_asynch_t callback,
                                       FAR void *arg);
static void xhci_asynch_completion(FAR struct xhci_epinfo_s *epinfo);
#endif
static int xhci_control_setup(FAR struct xhci_rhport_s *rhport,
                              FAR struct xhci_epinfo_s *epinfo,
                              FAR const struct usb_ctrlreq_s *req,
                              FAR uint8_t *buffer, size_t buflen);
static int xhci_normal_setup(FAR struct xhci_rhport_s *rhport,
                             FAR struct xhci_epinfo_s *epinfo,
                             FAR uint8_t *buffer, size_t buflen);
#ifndef CONFIG_USBHOST_ISOC_DISABLE
static int xhci_isoc_setup(FAR struct xhci_rhport_s *rhport,
                           FAR struct xhci_epinfo_s *epinfo,
                           FAR uint8_t *buffer, size_t buflen);
#endif
static ssize_t xhci_transfer_wait(FAR struct usbhost_xhci_s *priv,
                                  FAR struct xhci_epinfo_s *epinfo);

/* Interrupt handling *******************************************************/

static void xhci_portsc_work(FAR void *arg);
static void xhci_transfer_complete(FAR struct usbhost_xhci_s *priv,
                                   FAR struct xhci_trb_s *evt);
static void xhci_event_complete(FAR struct usbhost_xhci_s *priv,
                                FAR struct xhci_trb_s *evt);
static int xhci_events_poll(FAR struct usbhost_xhci_s *priv);
static void xhci_interrupt_work(FAR void *arg);
static int xhci_interrupt(int irq, FAR void *context, FAR void *arg);

/* USB host controller operations *******************************************/

static int xhci_wait(FAR struct usbhost_connection_s *conn,
                     FAR struct usbhost_hubport_s **hport);
static int xhci_rh_enumerate(FAR struct usbhost_connection_s *conn,
                            FAR struct usbhost_hubport_s *hport);
static int xhci_enumerate(FAR struct usbhost_connection_s *conn,
                          FAR struct usbhost_hubport_s *hport);

static int xhci_ep0configure(FAR struct usbhost_driver_s *drvr,
                             usbhost_ep_t ep0, uint8_t funcaddr,
                             uint8_t speed, uint16_t maxpacketsize);
static int xhci_epalloc(FAR struct usbhost_driver_s *drvr,
                        FAR const struct usbhost_epdesc_s *epdesc,
                        FAR usbhost_ep_t *ep);
static int xhci_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep);
static int xhci_alloc(FAR struct usbhost_driver_s *drvr,
                      FAR uint8_t **buffer, FAR size_t *maxlen);
static int xhci_free(FAR struct usbhost_driver_s *drvr,
                     FAR uint8_t *buffer);
static int xhci_ioalloc(FAR struct usbhost_driver_s *drvr,
                        FAR uint8_t **buffer, size_t buflen);
static int xhci_iofree(FAR struct usbhost_driver_s *drvr,
                       FAR uint8_t *buffer);

static int xhci_ctrl_xfer(FAR struct usbhost_driver_s *drvr,
                          usbhost_ep_t ep0,
                          FAR const struct usb_ctrlreq_s *req,
                          FAR uint8_t *buffer);
static int xhci_ctrlin(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                       FAR const struct usb_ctrlreq_s *req,
                       FAR uint8_t *buffer);
static int xhci_ctrlout(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                        FAR const struct usb_ctrlreq_s *req,
                        FAR const uint8_t *buffer);
static ssize_t xhci_transfer(FAR struct usbhost_driver_s *drvr,
                             FAR usbhost_ep_t ep, uint8_t *buffer,
                             size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static int xhci_asynch(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                       FAR uint8_t *buffer, size_t buflen,
                       usbhost_asynch_t callback, void *arg);
#endif
static int xhci_cancel(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep);
#ifdef CONFIG_USBHOST_HUB
static int xhci_connect(FAR struct usbhost_driver_s *drvr,
                        FAR struct usbhost_hubport_s *hport,
                        bool connected);
#endif

static void xhci_disconnect(FAR struct usbhost_driver_s *drvr,
                            FAR struct usbhost_hubport_s *hport);

/* Initialization ***********************************************************/

static int xhci_hw_getparams(FAR struct usbhost_xhci_s *priv);
static int xhci_irq_initialize(FAR struct usbhost_xhci_s *priv);
static int xhci_mem_alloc(FAR struct usbhost_xhci_s *priv);
static int xhci_mem_free(FAR struct usbhost_xhci_s *priv);
static int xhci_hw_initialize(FAR struct usbhost_xhci_s *priv);
static int xhci_sw_initialize(FAR struct usbhost_xhci_s *priv);
static int pci_xhci_probe(FAR struct pci_device_s *dev);
static void pci_xhci_remove(FAR struct pci_device_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* PCI device table */

static const struct pci_device_id_s g_pci_xhci_id_table[] =
{
  /* QEMU xHCI */

  {
    PCI_DEVICE(0x1b36, 0x000d),
    .driver_data = 0
  },

  /* Intel Alder Lake USB 3.2 xHCI controller */

  {
    PCI_DEVICE(0x8086, 0x51ed),
    .driver_data = 0
  },

  /* Intel Alder Lake-S USB 3.2 Gen 2x2 xHCI controller */

  {
    PCI_DEVICE(0x8086, 0x7ae0),
    .driver_data = 0
  },
  { }
};

/* PCI driver */

static struct pci_driver_s g_pci_xhci_drv =
{
  .id_table = g_pci_xhci_id_table,
  .probe    = pci_xhci_probe,
  .remove   = pci_xhci_remove,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xhci_capa_getreg
 *
 * Description:
 *   Get register (USB Legacy Support Capability)
 *
 ****************************************************************************/

static uint32_t xhci_capa_getreg(FAR struct usbhost_xhci_s *priv,
                                 unsigned int offset)
{
  uintptr_t addr = priv->capa_base + offset;
  return *((FAR volatile uint32_t *)addr);
}

/****************************************************************************
 * Name: xhci_capa_getreg_1b
 *
 * Description:
 *   Get 1B register (USB Legacy Support Capability)
 *
 ****************************************************************************/

static uint8_t xhci_capa_getreg_1b(FAR struct usbhost_xhci_s *priv,
                                   unsigned int offset)
{
  uintptr_t addr = priv->capa_base + offset;
  return *((FAR volatile uint8_t *)addr);
}

/****************************************************************************
 * Name: xhci_capa_putreg_1b
 *
 * Description:
 *   Put 1B register (USB Legacy Support Capability)
 *
 ****************************************************************************/

static void xhci_capa_putreg_1b(FAR struct usbhost_xhci_s *priv,
                                unsigned int offset,
                                uint8_t value)
{
  uintptr_t addr = priv->capa_base + offset;
  *((FAR volatile uint8_t *)addr) = value;
}

/****************************************************************************
 * Name: xhci_oper_getreg
 *
 * Description:
 *   Get register (Host Controller Operational Registers)
 *
 ****************************************************************************/

static uint32_t xhci_oper_getreg(FAR struct usbhost_xhci_s *priv,
                                 unsigned int offset)
{
  uintptr_t addr = priv->oper_base + offset;
  return *((FAR volatile uint32_t *)addr);
}

/****************************************************************************
 * Name: xhci_oper_putreg
 *
 * Description:
 *   Put register (Host Controller Operational Registers)
 *
 ****************************************************************************/

static void xhci_oper_putreg(FAR struct usbhost_xhci_s *priv,
                             unsigned int offset,
                             uint32_t value)
{
  uintptr_t addr = priv->oper_base + offset;
  *((FAR volatile uint32_t *)addr) = value;
}

/****************************************************************************
 * Name: xhci_oper_putreg_8b
 *
 * Description:
 *   Put register (Host Controller Operational Registers)
 *
 ****************************************************************************/

static void xhci_oper_putreg_8b(FAR struct usbhost_xhci_s *priv,
                                unsigned int offset,
                                uint64_t value)
{
  uintptr_t addr = priv->oper_base + offset;
  *((FAR volatile uint64_t *)addr) = value;
}

/****************************************************************************
 * Name: xhci_runt_getreg
 *
 * Description:
 *   Get register (Host Controller Runtime Registers)
 *
 ****************************************************************************/

static uint32_t xhci_runt_getreg(FAR struct usbhost_xhci_s *priv,
                                 unsigned int offset)
{
  uintptr_t addr = priv->runt_base + offset;
  return *((FAR volatile uint32_t *)addr);
}

/****************************************************************************
 * Name: xhci_runt_putreg
 *
 * Description:
 *   Put register (Host Controller Runtime Registers)
 *
 ****************************************************************************/

static void xhci_runt_putreg(FAR struct usbhost_xhci_s *priv,
                             unsigned int offset,
                             uint32_t value)
{
  uintptr_t addr = priv->runt_base + offset;
  *((FAR volatile uint32_t *)addr) = value;
}

/****************************************************************************
 * Name: xhci_runt_putreg_8b
 *
 * Description:
 *   Put register (Host Controller Runtime Registers)
 *
 ****************************************************************************/

static void xhci_runt_putreg_8b(FAR struct usbhost_xhci_s *priv,
                                unsigned int offset,
                                uint64_t value)
{
  uintptr_t addr = priv->runt_base + offset;
  *((FAR volatile uint64_t *)addr) = value;
}

/****************************************************************************
 * Name: xhci_door_putreg
 *
 * Description:
 *   Put register (Doorbell Registers)
 *
 ****************************************************************************/

static void xhci_door_putreg(FAR struct usbhost_xhci_s *priv,
                             unsigned int offset,
                             uint32_t value)
{
  uintptr_t addr = priv->door_base + offset;
  *((FAR volatile uint32_t *)addr) = value;
}

#ifdef CONFIG_DEBUG_USB_INFO
/****************************************************************************
 * Name: xhci_dump_capa_reg
 ****************************************************************************/

static void xhci_dump_capa_reg(FAR struct usbhost_xhci_s *priv,
                               FAR const char *msg, unsigned int offset)
{
  pciinfo("\t%s:\t\t0x%" PRIx32 "\n", msg, xhci_capa_getreg(priv, offset));
}

/****************************************************************************
 * Name: xhci_dump_oper_reg
 ****************************************************************************/

static void xhci_dump_oper_reg(FAR struct usbhost_xhci_s *priv,
                               FAR const char *msg, unsigned int offset)
{
  pciinfo("\t%s:\t\t0x%" PRIx32 "\n", msg, xhci_oper_getreg(priv, offset));
}

/****************************************************************************
 * Name: xhci_dump_runt_reg
 ****************************************************************************/

static void xhci_dump_runt_reg(FAR struct usbhost_xhci_s *priv,
                               FAR const char *msg, unsigned int offset)
{
  pciinfo("\t%s:\t\t0x%" PRIx32 "\n", msg, xhci_runt_getreg(priv, offset));
}

/****************************************************************************
 * Name: xhci_dump_mem
 ****************************************************************************/

static void xhci_dump_mem(FAR struct usbhost_xhci_s *priv,
                          FAR const char *msg)
{
  int i;

  pciinfo("Dump xHCI registers: %s\n", msg);

  pciinfo("=== Host Controller Capability Registers ===\n");
  xhci_dump_capa_reg(priv, "CAPLENGTH   ", XHCI_CAPLENGTH);
  xhci_dump_capa_reg(priv, "HCIVERSION  ", XHCI_HCIVERSION);
  xhci_dump_capa_reg(priv, "HCSPARAMS1  ", XHCI_HCSPARAMS1);
  xhci_dump_capa_reg(priv, "HCSPARAMS2  ", XHCI_HCSPARAMS2);
  xhci_dump_capa_reg(priv, "HCSPARAMS3  ", XHCI_HCSPARAMS3);
  xhci_dump_capa_reg(priv, "HCCPARAMS1  ", XHCI_HCCPARAMS1);
  xhci_dump_capa_reg(priv, "DBOFF       ", XHCI_DBOFF);
  xhci_dump_capa_reg(priv, "RTSOFF      ", XHCI_RTSOFF);
  xhci_dump_capa_reg(priv, "HCCPARAMS2  ", XHCI_HCCPARAMS2);

  pciinfo("=== Host Controller Operational Registers ===\n");
  xhci_dump_oper_reg(priv, "USBCMD      ", XHCI_USBCMD);
  xhci_dump_oper_reg(priv, "USBSTS      ", XHCI_USBSTS);
  xhci_dump_oper_reg(priv, "PAGESIZE    ", XHCI_PAGESIZE);
  xhci_dump_oper_reg(priv, "DNCTRL      ", XHCI_DNCTRL);
  xhci_dump_oper_reg(priv, "CRCR        ", XHCI_CRCR);
  xhci_dump_oper_reg(priv, "DCBAAP      ", XHCI_DCBAAP);
  xhci_dump_oper_reg(priv, "CONFIG      ", XHCI_CONFIG);

  for (i = 0; i < priv->no_ports; i++)
    {
      pciinfo("port %d --------------------------------\n", i);
      xhci_dump_oper_reg(priv, "PORTSC      ", XHCI_PORTSC(i));
      xhci_dump_oper_reg(priv, "PORTPMSC    ", XHCI_PORTPMSC(i));
      xhci_dump_oper_reg(priv, "PORTLI      ", XHCI_PORTLI(i));
    }

  /* Only one interrupter used */

  pciinfo("=== Host Controller Runtime Registers ===\n");
  xhci_dump_runt_reg(priv, "MFINDEX     ", XHCI_MFINDEX);
  xhci_dump_runt_reg(priv, "IMAN(0)     ", XHCI_IMAN(0));
  xhci_dump_runt_reg(priv, "IMOD(0)     ", XHCI_IMOD(0));
  xhci_dump_runt_reg(priv, "ERSTSZ(0)   ", XHCI_ERSTSZ(0));
  xhci_dump_runt_reg(priv, "ERSTBA(0)   ", XHCI_ERSTBA(0));
  xhci_dump_runt_reg(priv, "ERDP(0)     ", XHCI_ERDP(0));
}
#endif

/****************************************************************************
 * Name: xhci_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 ****************************************************************************/

static inline uint16_t xhci_getle16(FAR const uint8_t *val)
{
#ifdef CONFIG_ENDIAN_BIG
  return (uint16_t)val[0] << 8 | (uint16_t)val[1];
#else
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
#endif
}

/****************************************************************************
 * Name: xhci_ring_init
 *
 * Description:
 *   Initialize xHCI ring handler.
 *
 *   If ring buffer is already initialized, this function reset ring
 *   to a initial state.
 *
 * Returned Value:
 *   OK on success.
 *
 ****************************************************************************/

static int xhci_ring_init(FAR struct xhci_ring_s *ring, size_t len)
{
  FAR struct xhci_trb_s *trb;

  if (!ring->ring)
    {
      /* Allocate ring data */

      ring->ring = kmm_memalign(XHCI_BUF_ALIGN,
                                sizeof(struct xhci_trb_s) * len);
      if (!ring->ring)
        {
          return -ENOMEM;
        }

      /* Store length */

      ring->len = len;
    }

  /* Reset data in ring */

  memset(ring->ring, 0, ring->len * sizeof(struct xhci_trb_s));

  /* Fill Link TRB */

  trb     = &ring->ring[ring->len - 1];
  trb->d0 = htole64(up_addrenv_va_to_pa(&ring->ring[0]));
  trb->d1 = 0;
  trb->d2 = 0;

  up_flush_dcache((uintptr_t)trb, (uintptr_t)(trb + 1));

  /* Reset state */

  ring->i   = 0;
  ring->ccs = true;

  return OK;
}

/****************************************************************************
 * Name: xhci_ring_deinit
 *
 * Description:
 *   Initialize xHCI ring handler.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void xhci_ring_deinit(FAR struct xhci_ring_s *ring)
{
  /* Free ring memory */

  kmm_free(ring->ring);
}

/****************************************************************************
 * Name: xhci_ring_reset
 *
 * Description:
 *   Reset xHCI ring handler.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void xhci_ring_reset(FAR struct xhci_ring_s *ring, bool swap_ccs)
{
  /* Reset pointer */

  ring->i = 0;

  /* Swap CCS if requestede */

  if (swap_ccs)
    {
      ring->ccs = !ring->ccs;
    }
  else
    {
      ring->ccs  = true;
    }
}

/****************************************************************************
 * Name: xhci_add_trb
 *
 * Description:
 *   Reset TRB to a ring.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void xhci_add_trb(FAR struct usbhost_xhci_s *priv,
                         FAR struct xhci_ring_s *ring,
                         FAR struct xhci_trb_s *trb,
                         int len)
{
  uint32_t d2;
  int      i;

  for (i = 0; i < len; i++)
    {
      d2 = trb[i].d2;

      if (ring->ccs)
        {
          d2 |= XHCI_TRB_D2_C;
        }
      else
        {
          d2 &= ~XHCI_TRB_D2_C;
        }

      /* Make sure the cycle bit has the correct value */

      DEBUGASSERT((ring->ring[ring->i].d2 & XHCI_TRB_D2_C) != ring->ccs);

      /* Write TRB */

      ring->ring[ring->i].d0 = htole64(trb[i].d0);
      ring->ring[ring->i].d1 = htole32(trb[i].d1);
      ring->ring[ring->i].d2 = htole32(d2);

      /* Next TD */

      ring->i++;

      /* Handle end of the command ring */

      if (ring->i >= ring->len - 1)
        {
          /* Make sure the cycle bit has the correct value */

          DEBUGASSERT((ring->ring[0].d2 & XHCI_TRB_D2_C) == ring->ccs);

          if (ring->ccs)
            {
              d2 = XHCI_TRB_D2_C | XHCI_TRB_D2_TC |
                   XHCI_TRB_D2_TYPE_SET(XHCI_TRB_TYPE_LINK);
            }
          else
            {
              d2 = XHCI_TRB_D2_TC |
                   XHCI_TRB_D2_TYPE_SET(XHCI_TRB_TYPE_LINK);
            }

          /* Other parameters are already correct for this TRB */

          ring->ring[ring->i].d2 = htole32(d2);

          /* Update CCS */

          xhci_ring_reset(ring, true);
        }
    }

  /* Flush ring */

  up_flush_dcache((uintptr_t)ring->ring,
                  (uintptr_t)(ring->ring + ring->len));
}

/****************************************************************************
 * Name: xhci_bios_wait
 *
 * Description:
 *   Wait for BIOS to give up the controller lock
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int xhci_bios_wait(FAR struct usbhost_xhci_s *priv)
{
  uint32_t cstart;
  uint32_t ecp;
  uint32_t eec;
  uint8_t  sem;
  uint8_t  timeout;
  int      ret = OK;

  /* Get Extended Capability Pointer */

  cstart = XHCI_HCCPARAMS1_XECP(xhci_capa_getreg(priv, XHCI_HCCPARAMS1));

  /* Find USBLEGSUP - if present, we have to acquire for BIOS semaphore */

  eec = -1;
  ecp = (cstart << 2);

  while (1)
    {
      if (ecp == 0 || XHCI_USBLEGSUP_NEXT(eec) == 0)
        {
          break;
        }

      eec = xhci_capa_getreg(priv, ecp);

      if (XHCI_USBLEGSUP_ID(eec) == XHCI_ID_USBLEGSUP)
        {
          /* We have to wait for semaphore */

          ret = -EAGAIN;

          /* Get BIOS semaphore */

          sem = xhci_capa_getreg_1b(priv, ecp + XHCI_USBLEGSUP_BIOS_SEM);
          if (sem == 0)
            {
              ret = OK;
              break;
            }

          /* Get semaphore request */

          xhci_capa_putreg_1b(priv, ecp + XHCI_USBLEGSUP_OS_SEM, 1);

          /* Wait for semaphore released from BIOS */

          for (timeout = 0; timeout < 100; timeout++)
            {
              sem = xhci_capa_getreg_1b(priv, ecp + XHCI_USBLEGSUP_BIOS_SEM);
              if (sem == 0)
                {
                  ret = OK;
                  break;
                }

              up_mdelay(100);
            }
        }

      /* Next cap */

      ecp += (XHCI_USBLEGSUP_NEXT(eec) << 2);
    }

  return ret;
}

/****************************************************************************
 * Name: xhci_ctrl_start
 *
 * Description:
 *   Start controller.
 *
 *   According to "4.2 Host Controller Initialization".
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int xhci_ctrl_start(FAR struct usbhost_xhci_s *priv)
{
  FAR struct xhci_event_ring_s *evnt;
  uint32_t                      regval;
  int                           ret;
  int                           i;

  pciinfo("Start controller\n");

  /* Reset controller before writing any Operational or Runtime registers */

  ret = xhci_ctrl_reset(priv);
  if (ret < 0)
    {
      usbhost_trace1(XHCI_TRACE1_RESET_FAILED, 0);
      return ret;
    }

  /* TODO: clear interrupts and disable all device notifications */

  /* Max Device Slots Enabled */

  xhci_oper_putreg(priv, XHCI_CONFIG, priv->no_slots);

  /* Slot 0 in Device Context is reserved for Scratchpad Buffer Array */

  priv->pg_ctx[0] = htole64(up_addrenv_va_to_pa(priv->pg_sb));

  /* Device Context Base Address Array Pointer */

  xhci_oper_putreg_8b(priv, XHCI_DCBAAP, up_addrenv_va_to_pa(priv->pg_ctx));

  /* Event Ring Segment Table Size */

  xhci_runt_putreg(priv, XHCI_ERSTSZ(0), priv->no_erst);

  /* Initialize command ring state and event ring state */

  ret = xhci_ring_init(&priv->cmd, XHCI_CMD_MAX);
  if (ret < 0)
    {
      pcierr("cmd ring init failed\n");
      return ret;
    }

  ret = xhci_ring_init(&priv->evnt, XHCI_EVENT_MAX);
  if (ret < 0)
    {
      pcierr("event ring init failed\n");
      return ret;
    }

  /* Configure Event Ring */

  evnt = (struct xhci_event_ring_s *)priv->pg_erst;
  evnt->base = htole64(up_addrenv_va_to_pa(priv->evnt.ring));
  evnt->size = XHCI_EVENT_MAX;
  evnt->res  = 0;

  /* Flush all memory before write to ERDP so xhci sees correct data */

  up_flush_dcache_all();

  xhci_runt_putreg_8b(priv, XHCI_ERDP(0),
                      up_addrenv_va_to_pa(priv->evnt.ring));

  /* Write ERSTBA with ERST(0).BaseAddress.
   *
   * This must be done after ERST[0] initialization and after write to
   * ERSTSZ. When the ERSTBA register is written, the Event Ring State
   * Machine is set to the Start state.
   *
   * For details look at "4.9.4 Event Ring Management"
   */

  xhci_runt_putreg_8b(priv, XHCI_ERSTBA(0),
                      up_addrenv_va_to_pa(priv->pg_erst));

  /* Last item in the command ring points to the beginning of the ring */

  priv->cmd.ring[XHCI_CMD_MAX - 1].d0 = htole64(
    up_addrenv_va_to_pa(priv->cmd.ring));

  /* Configure the Command Ring */

  xhci_oper_putreg_8b(priv, XHCI_CRCR,
                      up_addrenv_va_to_pa(priv->cmd.ring) | XHCI_CRCR_RCS);

  /* Enable interrupts */

  regval = xhci_runt_getreg(priv, XHCI_IMAN(0));
  regval |= XHCI_IMAN_IE;
  xhci_runt_putreg(priv, XHCI_IMAN(0), regval);

  /* Flush all memory once again */

  up_flush_dcache_all();

  /* Turn the host controller ON, enable interrupts and system errors */

  xhci_oper_putreg(priv, XHCI_USBCMD,
                         XHCI_USBCMD_RS |
                         XHCI_USBCMD_INTE |
                         XHCI_USBCMD_HSEE);

  /* Wait for controller started */

  ret = -EAGAIN;
  for (i = 0; i < 10; i++)
    {
      up_mdelay(100);

      if (!(xhci_oper_getreg(priv, XHCI_USBSTS) & XHCI_USBSTS_HCH))
        {
          ret = OK;
          break;
        }
    }

  /* Check for timeout */

  if (ret != OK)
    {
      pcierr("Can't start controller!");
      return ret;
    }

  /* Poll all pending events */

  xhci_events_poll(priv);

  return OK;
}

/****************************************************************************
 * Name: xhci_ctrl_halt
 *
 * Description:
 *   Halt controller.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int xhci_ctrl_halt(FAR struct usbhost_xhci_s *priv)
{
  int ret = -EAGAIN;
  int i;

  /* Halt controller */

  xhci_oper_putreg(priv, XHCI_USBCMD, 0);

  /* Wait for controller halted */

  for (i = 0; i < 10; i++)
    {
      up_mdelay(100);

      if (xhci_oper_getreg(priv, XHCI_USBSTS) & XHCI_USBSTS_HCH)
        {
          ret = OK;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: xhci_ctrl_reset
 *
 * Description:
 *   Reset controller.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int xhci_ctrl_reset(FAR struct usbhost_xhci_s *priv)
{
  int ret = -EAGAIN;
  int i;

  /* Halt controller */

  xhci_oper_putreg(priv, XHCI_USBCMD, XHCI_USBCMD_HCRST);

  /* Wait for controller halted */

  for (i = 0; i < 10; i++)
    {
      up_mdelay(100);

      if (!(xhci_oper_getreg(priv, XHCI_USBSTS) & XHCI_USBSTS_CNR))
        {
          ret = OK;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: xhci_probe_ports
 *
 * Description:
 *   Initial ports probe.
 *
 ****************************************************************************/

static void xhci_probe_ports(FAR struct usbhost_xhci_s *priv)
{
  uint32_t portsc;
  int      i;

  for (i = 0; i < priv->no_ports; i++)
    {
      portsc = xhci_oper_getreg(priv, XHCI_PORTSC(i));
      priv->rhport[i].connected = ((portsc & XHCI_PORTSC_CCS) != 0);

      /* Clear status change */

      xhci_oper_putreg(priv, XHCI_PORTSC(i), portsc);
    }
}

/****************************************************************************
 * Name: xhci_port_enable
 *
 * Description:
 *   Set port to the Enable state.
 *
 ****************************************************************************/

static int xhci_port_enable(FAR struct usbhost_xhci_s *priv,
                            FAR struct usbhost_hubport_s *hport)
{
  uint32_t retries;
  uint32_t regval;
  uint8_t  speed;
  int      rhpndx;

  DEBUGASSERT(hport != NULL);
  rhpndx = hport->port;

  regval = xhci_oper_getreg(priv, XHCI_PORTSC(rhpndx));

  /* A USB3 protocol port attempts to automatically advance to the
   * Enabled state for port as part of the attach process.
   */

  if (!(regval & XHCI_PORTSC_PED))
    {
      /* Reset the port */

      regval = xhci_oper_getreg(priv, XHCI_PORTSC(rhpndx));
      regval |= XHCI_PORTSC_PR;
      xhci_oper_putreg(priv, XHCI_PORTSC(rhpndx), regval);

      /* REVISIT: we get Port Status Change Event here */

      /* Wait for Enabled state for port */

      retries = 10;
      while (!(xhci_oper_getreg(priv, XHCI_PORTSC(rhpndx))
               & XHCI_PORTSC_PED) && retries > 0)
        {
          retries--;
          up_mdelay(100);
        }

      if (retries == 0)
        {
          return -ETIMEDOUT;
        }
    }

  /* Get port status */

  regval = xhci_oper_getreg(priv, XHCI_PORTSC(rhpndx));

  /* Get port speed */

  speed = XHCI_PORTSC_PS(regval);
  switch (speed)
    {
      case XHCI_PORTSC_PS_FULL:
        {
          hport->speed = USB_SPEED_FULL;
          break;
        }

      case XHCI_PORTSC_PS_LOW:
        {
          hport->speed = USB_SPEED_LOW;
          break;
        }

      case XHCI_PORTSC_PS_HIGH:
        {
          hport->speed = USB_SPEED_HIGH;
          break;
        }

      case XHCI_PORTSC_PS_SUPPER11:
        {
          hport->speed = USB_SPEED_SUPER;
          break;
        }

      case XHCI_PORTSC_PS_SUPPER21:
      case XHCI_PORTSC_PS_SUPPER12:
      case XHCI_PORTSC_PS_SUPPER22:
        {
          hport->speed = USB_SPEED_SUPER_PLUS;
          break;
        }

      default:
        {
          pcierr("speed = 0x%x\n", speed);
          hport->speed = USB_SPEED_UNKNOWN;
          return -EINVAL;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: xhci_dcbaa_set
 *
 * Description:
 *   Set entry in the Device Context Base Address Array, which should point
 *   to the Output Device Context data structure.
 *
 ****************************************************************************/

static void xhci_dcbaa_set(FAR struct usbhost_xhci_s *priv, uint8_t index,
                           uintptr_t ctx)
{
  /* NOTE: context must be physical address! */

  priv->pg_ctx[index] = htole64(ctx);

  /* Flush context */

  up_flush_dcache((uintptr_t)priv->pg_ctx,
                  (uintptr_t)(priv->pg_ctx + priv->no_slots + 1));
}

/****************************************************************************
 * Name: xhci_ep_configure
 *
 * Description:
 *   Configure endpoint context.
 *
 * Reference:
 *   - 4.8.2 Endpoint Context Initialization
 *   - 6.2.3 Endpoint Context
 *
 ****************************************************************************/

static void xhci_ep_configure(FAR struct usbhost_xhci_s *priv,
                              FAR struct xhci_ep_ctx_s *ctx,
                              uint8_t type, uint16_t maxpkt,
                              uint8_t maxburst, uint64_t tr_dp,
                              uint8_t mult, uint8_t interval)
{
  uint32_t ctx0 = 0;
  uint32_t ctx1 = 0;
  uint64_t ctx2 = 0;

  /* Set type */

  ctx1 |= XHCI_EP_CTX1_EPTYPE(type);

  /* Set max packet size */

  ctx1 |= XHCI_EP_CTX1_MAXPKT(maxpkt);

  /* Set max burst size */

  ctx1 |= XHCI_EP_CTX1_MAXBRST(maxburst);

  /* Set TR Dequeue Pointer.
   * NOTE: must be physical address aligned to 16-byte.
   */

  DEBUGASSERT(tr_dp != 0 && tr_dp % 16 == 0);
  ctx2 = tr_dp;

  /* Set DCS.
   * Should be set to 1 only if no stream (USB3.0 specific).
   */

  ctx2 |= XHCI_EP_CTX2_DCS;

  /* Set interval */

  ctx0 |= XHCI_EP_CTX0_INTERVAL(interval);

  /* Set max primary streams.
   * Set to zero for now (USB3.0 specific)
   */

  ctx0 |= XHCI_EP_CTX0_MAXPSTR(0);

  /* Set mult */

  ctx0 |= XHCI_EP_CTX0_MULT(mult);

  /* Set error count to 3 if this is not ISOCH endpoint */

  if (type != XHCI_EPTYPE_ISO_OUT && type != XHCI_EPTYPE_ISO_IN)
    {
      ctx1 |= XHCI_EP_CTX1_CERR(3);
    }

  /* Write context */

  ctx->ctx0 = htole32(ctx0);
  ctx->ctx1 = htole32(ctx1);
  ctx->ctx2 = htole64(ctx2);

  /* Flush context */

  up_flush_dcache((uintptr_t)ctx,
                  (uintptr_t)ctx + sizeof(struct xhci_ep_ctx_s));
}

/****************************************************************************
 * Name: xhci_address_set
 *
 * Description:
 *   Set address request.
 *
 *   If setaddr is true, then xHC issue a SET_ADDRESS request.
 *
 ****************************************************************************/

static int xhci_address_set(FAR struct usbhost_xhci_s *priv,
                            FAR struct xhci_rhport_s *rhport, bool setaddr)
{
  FAR struct xhci_dev_s *dev;
  uint64_t ctx;

  dev = rhport->dev;
  ctx = up_addrenv_va_to_pa(dev->input);

  return xhci_cmd_setaddr(priv, rhport->slot, ctx, !setaddr);
}

/****************************************************************************
 * Name: xhci_slot_init
 *
 * Description:
 *   Initialize Device Slot data.
 *
 * Assumption:
 *   1. All slot resources already allocated.
 *   2. Port is in Enabled state.
 *
 * Reference:
 *   - 4.3.3. Device Slot Initialization
 *
 ****************************************************************************/

static int xhci_slot_init(FAR struct usbhost_xhci_s *priv,
                          FAR struct xhci_dev_s *dev)
{
  uint32_t regval;
  uint16_t maxpkt;
  uintptr_t drdp;

  /* Step 1. The Input Context data structure already allocated.
   * Initialize all fields to 0.
   */

  memset(dev->input, 0, sizeof(struct xhci_input_dev_ctx_s));

  /* Step 2. Initialize the Input Control Context by setting the A0 and
   * A1 flags to 1 (Slot flag and EP0 flag).
   */

  regval = XHCI_IN_CTX1_A(XHCI_SLOT_FLAG) |
           XHCI_IN_CTX1_A(XHCI_EP0_FLAG);
  xhci_context_ctrl(priv, dev, 0, regval);

  /* Step 3. Initialize the Input Slot Context */

  regval = XHCI_ST_CTX0_CTXENT_SET(1);

#ifdef CONFIG_USBHOST_HUB
  /* TODO:
   *   1. Activate the transaction translator if required
   *   2. Configure hub bit in slot context if hub
   *   3. configure route string
   */

#  warning missing logic
#endif

  dev->input->slot.ctx[0] = htole32(regval);

  /* Configure Root Hub Port Number (starts from 1) */

  regval = XHCI_ST_CTX1_RHPN_SET(RHPNDX(dev->rhport) + 1);

  /* TODO: configure number of ports */

  regval |= XHCI_ST_CTX1_PORTS_SET(0);
  dev->input->slot.ctx[1] = htole32(regval);

  /* Step 4. the Transfer Ring for the Default Control Endpoint is already
   * allocated.
   */

  drdp = up_addrenv_va_to_pa(dev->rhport->ep0.td.ring);

  /* Step 5. Initialize the Input default control Endpoint 0 Context */

  DEBUGASSERT(dev->rhport != NULL);
  if (dev->rhport->hport.hport.speed == USB_SPEED_HIGH)
    {
      /* For high-speed, we must use 64 bytes */

      maxpkt = 64;
    }
  else
    {
      /* Eight will work for both low- and full-speed */

      maxpkt = 8;
    }

  DEBUGASSERT(drdp != 0);
  xhci_ep_configure(priv,
                    &dev->input->ep[0],
                    XHCI_EPTYPE_CTRL, maxpkt,
                    0, drdp,
                    0, 0);

  /* Step 6. The output Device Context data structure already allocated.
   * Initialize all fields to 0.
   */

  memset(dev->ctx, 0, sizeof(struct xhci_dev_ctx_s));

  /* Flush Device input context */

  up_flush_dcache((uintptr_t)dev->input,
                  (uintptr_t)dev->input +
                  sizeof(struct xhci_input_dev_ctx_s));

  /* Step 7. Load the appropriate (Device Slot ID) entry in the Device
   * Context Base Address Array with a pointer to the Output Device
   * Context data structure
   */

  xhci_dcbaa_set(priv, dev->slot, up_addrenv_va_to_pa(dev->ctx));

  return OK;
}

/****************************************************************************
 * Name: xhci_device_init
 *
 * Description:
 *   Initialize Device.
 *
 * Assumption:
 *   1. All device resources already allocated.
 *   2. Port is in Enabled state.
 *
 * Reference:
 *   - 4.3: USB Device Initialization
 *
 ****************************************************************************/

static int xhci_device_init(FAR struct usbhost_xhci_s *priv,
                            FAR struct xhci_rhport_s *rhport)
{
  FAR struct xhci_dev_s *dev;
  uint8_t                slot;
  int                    ret;

  /* We enter this function after steps 1-3 from "USB Device Initialization"
   * are done.
   */

  /* Step 4: Get Device Slot.
   *
   * Reference:
   *   - 4.3.2: Device Slot Assignment
   */

  ret = xhci_cmd_sloten(priv, &slot);
  if (ret < 0 || slot > priv->no_slots)
    {
      /* Something goes wrong ! */

      usbhost_vtrace1(XHCI_TRACE1_SLOTEN_FAILED, ret);
      return ret;
    }

  /* Slot ID is an index to the identify Device data */

  rhport->dev = &priv->devs[slot - 1];
  rhport->slot = slot;
  dev = rhport->dev;

  /* Slot has been allocated to software and is now in Enabled state */

  dev->state = XHCI_SLOT_ENABLED;

  /* Step 5: Initialize the data structures associated with the slot.
   * All data structured are already allocated.
   */

  ret = xhci_ring_init(&rhport->ep0.td, XHCI_TD_MAX);
  if (ret < 0)
    {
      pcierr("ep0 ring init failed\n");
      return ret;
    }

  rhport->ep0.slot = slot;
  dev->rhport      = rhport;
  dev->slot        = slot;
  dev->epinfo[0]   = &rhport->ep0;

  ret = xhci_slot_init(priv, dev);
  if (ret < 0)
    {
      return ret;
    }

  /* Step 6: Assign and address to the device and enable its Default
   * Control Endpoint.
   *
   * NOTE: we don't send SET_ADDRESS request here.
   *       This is done in xhci_ctrlin() and controlled by NuttX USB Host
   *       stack.
   */

  ret = xhci_address_set(priv, rhport, false);
  if (ret < 0)
    {
      pcierr("failed to set address %d\n", ret);
      return ret;
    }

  /* Steps 7-12 don't belong here! */

  return OK;
}

/****************************************************************************
 * Name: xhci_device_deinit
 *
 * Description:
 *   Free Device.
 *
 * Reference:
 *   - 4.3: USB Device Initialization
 *
 ****************************************************************************/

static int xhci_device_deinit(FAR struct usbhost_xhci_s *priv,
                              FAR struct xhci_rhport_s *rhport)
{
  uint8_t slot = rhport->slot;
  int     ret;

  /* Disable Slot */

  ret = xhci_cmd_slotdis(priv, slot);
  if (ret < 0)
    {
      pcierr("xhci_cmd_slotdis failed %d\n", ret);
    }

  /* Clear DCBAA entry for this slot */

  xhci_dcbaa_set(priv, slot, 0);

  /* Clean up device data, but don't touch allocated memory! */

  rhport->dev->state = XHCI_SLOT_DISABLED;

  memset(rhport->dev->ctx, 0, sizeof(struct xhci_dev_ctx_s));
  memset(rhport->dev->input, 0, sizeof(struct xhci_input_dev_ctx_s));

  /* Remove reference to a device slot */

  rhport->dev = NULL;

  return OK;
}

/****************************************************************************
 * Name: xhci_epno_get
 *
 * Description:
 *   Get EP index for a given endpoint.
 *
 *   Returns Device Context Index (DCI).
 *
 ****************************************************************************/

static inline uint8_t xhci_epno_get(FAR struct xhci_epinfo_s *epinfo)
{
  DEBUGASSERT(epinfo);

  if (epinfo->epno == 0)
    {
      return 1;
    }

  if (epinfo->dirin)
    {
      return epinfo->epno * 2  + 1;
    }
  else
    {
      return epinfo->epno * 2;
    }
}

/****************************************************************************
 * Name: xhci_context_ctrl
 *
 * Description:
 *   Configure Input Control Context, which defines which Device Context
 *   data structures are affected by a command.
 *
 * Assumption:
 *   Input Context must be flushed by caller.
 *
 ****************************************************************************/

static void xhci_context_ctrl(FAR struct usbhost_xhci_s *priv,
                              FAR struct xhci_dev_s *dev,
                              uint32_t drop, uint32_t add)
{
  int i;

  dev->input->input.ctx[0] = htole32(drop);
  dev->input->input.ctx[1] = htole32(add);

  /* Update Context Entries in Slot */

  for (i = 31; i != 1; i--)
    {
      if (add & (1 << i))
        {
          break;
        }
    }

  dev->input->slot.ctx[0] &= ~XHCI_ST_CTX0_CTXENT_MASK;
  dev->input->slot.ctx[0] |= XHCI_ST_CTX0_CTXENT_SET(i);
}

/****************************************************************************
 * Name: xhci_command
 *
 * Description:
 *   Issue a xHCI command.
 *
 * NOTE:
 *  trb data in host specific byte order. This function converts it
 *  to a correct order
 *
 ****************************************************************************/

static int xhci_command(FAR struct usbhost_xhci_s *priv,
                        FAR struct xhci_trb_s *trb, uint16_t timeout_ms)
{
  int ret;

  /* Lock bus */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Add command to ring */

  xhci_add_trb(priv, &priv->cmd, trb, 1);

  /* Ringing the Host Controller Doorbell */

  xhci_door_putreg(priv, XHCI_DOORBEL(0), 0);

  /* Wait for Command Completion Event */

  ret = nxsem_tickwait_uninterruptible(&priv->cmdsem,
                                       MSEC2TICK(timeout_ms));
  if (ret < 0)
    {
      /* Check for missed interrupts */

      xhci_events_poll(priv);
    }

  /* Return command results */

  trb->d0 = priv->cmdres.d0;
  trb->d1 = priv->cmdres.d1;
  trb->d2 = priv->cmdres.d2;

  if (XHCI_TRB_D1_CC_GET(trb->d1) != XHCI_TRB_CC_SUCCESS)
    {
      pcierr("event CC = %d\n", XHCI_TRB_D1_CC_GET(trb->d1));
      ret = -EIO;
    }

  /* Clean response */

  priv->cmdres.d0 = 0;
  priv->cmdres.d1 = 0;
  priv->cmdres.d2 = 0;

  /* Unlock bus */

  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: xhci_cmd_sloten
 *
 * Description:
 *   Enable Slot Command.
 *
 ****************************************************************************/

static int xhci_cmd_sloten(FAR struct usbhost_xhci_s *priv,
                           FAR uint8_t *slot)
{
  struct xhci_trb_s trb;
  int               ret;

  /* Host specific byte order. Conversion done by xhci_command() */

  trb.d0 = 0;
  trb.d1 = 0;
  trb.d2 = XHCI_TRB_D2_TYPE_SET(XHCI_TRB_TYPE_EN_SLOT);

  ret = xhci_command(priv, &trb, 1000);
  if (ret < 0)
    {
      *slot = 0;
      return ret;
    }

  /* Return available slot */

  *slot = XHCI_TRB_D2_SLOTID_GET(le32toh(trb.d2));

  return OK;
}

/****************************************************************************
 * Name: xhci_cmd_slotdis
 *
 * Description:
 *   Disable Slot Command.
 *
 ****************************************************************************/

static int xhci_cmd_slotdis(FAR struct usbhost_xhci_s *priv, uint8_t slot)
{
  struct xhci_trb_s trb;

  /* Host specific byte order. Conversion done by xhci_command() */

  trb.d0 = 0;
  trb.d1 = 0;
  trb.d2 = XHCI_TRB_D2_TYPE_SET(XHCI_TRB_TYPE_DIS_SLOT) |
           XHCI_TRB_D2_SLOTID_SET(slot);

  return xhci_command(priv, &trb, 100);
}

/****************************************************************************
 * Name: xhci_cmd_setaddr
 *
 * Description:
 *   Address Device Command.
 *
 ****************************************************************************/

static int xhci_cmd_setaddr(FAR struct usbhost_xhci_s *priv, uint8_t slot,
                            uint64_t ctx, bool bsr)
{
  struct xhci_trb_s trb;

  /* Host specific byte order. Conversion done by xhci_command() */

  trb.d0 = htole64(ctx);
  trb.d1 = 0;
  trb.d2 = XHCI_TRB_D2_TYPE_SET(XHCI_TRB_TYPE_ADDR_DEV) |
           XHCI_TRB_D2_SLOTID_SET(slot);

  if (bsr)
    {
      trb.d2 |= XHCI_TRB_D2_BSR;
    }

  return xhci_command(priv, &trb, 100);
}

/****************************************************************************
 * Name: xhci_cmd_cfgep
 *
 * Description:
 *   Configure EP Command.
 *
 ****************************************************************************/

static int xhci_cmd_cfgep(FAR struct usbhost_xhci_s *priv, uint8_t slot,
                          uint64_t ctx, bool deconfig)
{
  struct xhci_trb_s trb;

  /* Host specific byte order. Conversion done by xhci_command() */

  trb.d0 = htole64(ctx);
  trb.d1 = 0;
  trb.d2 = XHCI_TRB_D2_TYPE_SET(XHCI_TRB_TYPE_CFG_EP) |
           XHCI_TRB_D2_SLOTID_SET(slot);

  if (deconfig)
    {
      trb.d2 |= XHCI_TRB_D2_DC;
    }

  return xhci_command(priv, &trb, 100);
}

/****************************************************************************
 * Name: xhci_cmd_stopep
 *
 * Description:
 *   Stop endpoint
 *
 ****************************************************************************/

static int xhci_cmd_stopep(FAR struct usbhost_xhci_s *priv, uint8_t slot,
                           uint8_t ep, bool suspend)
{
  struct xhci_trb_s trb;

  /* Host specific byte order. Conversion done by xhci_command() */

  trb.d0 = 0;
  trb.d1 = 0;
  trb.d2 = XHCI_TRB_D2_TYPE_SET(XHCI_TRB_TYPE_STOP_EP) |
           XHCI_TRB_D2_SLOTID_SET(slot) | XHCI_TRB_D2_EP_SET(ep);

  if (suspend)
    {
      trb.d2 |= XHCI_TRB_D2_SP;
    }

  return xhci_command(priv, &trb, 100);
}

/****************************************************************************
 * Name: xhci_cmd_evalctx
 *
 * Description:
 *   Evaluate Context Command
 *
 ****************************************************************************/

static int xhci_cmd_evalctx(FAR struct usbhost_xhci_s *priv, uint8_t slot,
                            uint64_t ctx)
{
  struct xhci_trb_s trb;

  /* Host specific byte order. Conversion done by xhci_command() */

  trb.d0 = htole64(ctx);
  trb.d1 = 0;
  trb.d2 = XHCI_TRB_D2_TYPE_SET(XHCI_TRB_TYPE_EVAL_CTX) |
           XHCI_TRB_D2_SLOTID_SET(slot);

  return xhci_command(priv, &trb, 100);
}

/****************************************************************************
 * Name: xhci_ep_doorbell
 *
 * Description:
 *   Ring doorbell associated with a given endpoint.
 *
 ****************************************************************************/

static void xhci_ep_doorbell(FAR struct usbhost_xhci_s *priv,
                             FAR struct xhci_epinfo_s *epinfo)
{
  uint8_t  target = xhci_epno_get(epinfo);
  uint32_t regval = 0;

  /* Doorbel tareget is EP number */

  regval |= XHCI_DOORBEL_TARGET(target);

  /* Streams not supported yet (USB3.0 specific) */

  regval |= XHCI_DOORBEL_TASK(0);

  /* Ring doorbell */

  xhci_door_putreg(priv, XHCI_DOORBEL(epinfo->slot), regval);
}

/****************************************************************************
 * Name: xhci_ioc_setup
 *
 * Description:
 *   Set the request for the IOC event well BEFORE enabling the transfer (as
 *   soon as we are absolutely committed to the transfer).  We do
 *   this to minimize race conditions.  This logic would have to be expanded
 *   if we want to have more than one packet in flight at a time!
 *
 * Assumption:
 *   The caller holds the XHCI lock
 *
 ****************************************************************************/

static int xhci_ioc_setup(FAR struct xhci_rhport_s *rhport,
                          FAR struct xhci_epinfo_s *epinfo,
                          size_t buflen)
{
  FAR struct usbhost_xhci_s *priv = XHCI_PRIV_FROM_RHPORT(rhport);
  irqstate_t                 flags;
  int                        ret  = -ENODEV;

  DEBUGASSERT(rhport && epinfo && !epinfo->iocwait);
#ifdef CONFIG_USBHOST_ASYNCH
  DEBUGASSERT(epinfo->callback == NULL);
#endif

  /* Is the device still connected? */

  flags = spin_lock_irqsave(&priv->spinlock);
  if (rhport->connected)
    {
      /* Then set iocwait to indicate that we expect to be informed when
       * either (1) the device is disconnected, or (2) the transfer
       * completed.
       */

      epinfo->iocwait  = true;   /* We want to be awakened by IOC interrupt */
      epinfo->status   = 0;      /* No status yet */
      epinfo->xfrd     = 0;      /* Nothing transferred yet */
      epinfo->buflen   = buflen; /* Buffer length */
      epinfo->result   = -EBUSY; /* Transfer in progress */
#ifdef CONFIG_USBHOST_ASYNCH
      epinfo->callback = NULL;   /* No asynchronous callback */
      epinfo->arg      = NULL;
#endif
      ret              = OK;     /* We are good to go */
    }

  spin_unlock_irqrestore(&priv->spinlock, flags);
  return ret;
}

/****************************************************************************
 * Name: xhci_ioc_wait
 *
 * Description:
 *   Wait for the IOC event.
 *
 * Assumption:
 *   The caller does *NOT* hold the xHCI lock.  That would cause a deadlock
 *   when the bottom-half, worker thread needs to take the semaphore.
 *
 ****************************************************************************/

static int xhci_ioc_wait(FAR struct xhci_epinfo_s *epinfo)
{
  int ret = OK;

  /* Wait for the IOC event.  Loop to handle any false alarm semaphore
   * counts.  Return an error if the task is canceled.
   */

  while (epinfo->iocwait)
    {
      ret = nxsem_wait_uninterruptible(&epinfo->iocsem);
      if (ret < 0)
        {
          break;
        }
    }

  return ret < 0 ? ret : epinfo->result;
}

/****************************************************************************
 * Name: xhci_control_setup
 *
 * Description:
 *   Process a IN or OUT request control ep.
 *   This function will enqueue the request and wait for it to
 *   complete.  Bulk data transfers differ in that req == NULL and there are
 *   not SETUP or STATUS phases.
 *
 *   This is a blocking function; it will not return until the control
 *   transfer has completed.
 *
 * Assumption:
 *   The caller holds the xHCI lock.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is return on
 *   any failure.
 *
 ****************************************************************************/

static int xhci_control_setup(FAR struct xhci_rhport_s *rhport,
                              FAR struct xhci_epinfo_s *epinfo,
                              FAR const struct usb_ctrlreq_s *req,
                              FAR uint8_t *buffer, size_t buflen)
{
  FAR struct usbhost_xhci_s *priv = XHCI_PRIV_FROM_RHPORT(rhport);
  struct xhci_trb_s          trb[3];
  uint8_t                    trt;
  int                        i    = 0;

  /* Prepare Setup Stage TRB */

  trb[i].d0 = *((FAR uint64_t *)req);
  trb[i].d1 = XHCI_TRB_D1_TXLEN_SET(8);
  trb[i].d2 = XHCI_TRB_D2_TYPE_SET(XHCI_TRB_TYPE_SETUP_STAGE) |
              XHCI_TRB_D2_IDT;

  /* Reference:
   *   - Table 4-7: USB SETUP Data to Data Stage TRB and Status Stage
   *                TRB mapping
   */

  if (req->type & USB_REQ_DIR_IN)
    {
      trt = XHCI_TRB_D2_TRT_INDATA;
    }
  else if (req->type & USB_REQ_DIR_OUT)
    {
      trt = XHCI_TRB_D2_TRT_OUTDATA;
    }
  else
    {
      trt = XHCI_TRB_D2_TRT_NODATA;
    }

  trb[i].d2 |= XHCI_TRB_D2_TRT_SET(trt);

  /* Next TRB */

  i++;

  /* Prepare Data Stage TRB */

  if (buffer)
    {
      trb[i].d0 = up_addrenv_va_to_pa(buffer);
      trb[i].d1 = XHCI_TRB_D1_TXLEN_SET(buflen);
      trb[i].d2 = XHCI_TRB_D2_TYPE_SET(XHCI_TRB_TYPE_DATA_STAGE);

      if (req->type & USB_REQ_DIR_IN)
        {
          trb[i].d2 |= XHCI_TRB_D2_DIR;
        }

      /* Next TRB */

      i++;
    }

  /* Prepare Status Stage TRB */

  trb[i].d0 = 0;
  trb[i].d1 = XHCI_TRB_D1_IRQ_SET(0) | XHCI_TRB_D1_TXLEN_SET(0);
  trb[i].d2 = XHCI_TRB_D2_IOC |
              XHCI_TRB_D2_TYPE_SET(XHCI_TRB_TYPE_STAT_STAGE);

  if (!(req->type & USB_REQ_DIR_IN))
    {
      trb[i].d2 |= XHCI_TRB_D2_DIR;
    }

  /* Next TRB */

  i++;

  /* Add TRBs to ring */

  xhci_add_trb(priv, &epinfo->td, trb, i);

  /* Trigger transfer */

  xhci_ep_doorbell(priv, epinfo);

  return OK;
}

/****************************************************************************
 * Name: xhci_normal_setup
 *
 * Description:
 *   Process a IN or OUT request on bulk or interrupt endpoint.
 *   This function will enqueue the request and wait for it to complete.
 *
 *   This is a blocking function; it will not return until the control
 *   transfer has completed.
 *
 * Assumption:
 *   The caller holds the xHCI lock.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int xhci_normal_setup(FAR struct xhci_rhport_s *rhport,
                             FAR struct xhci_epinfo_s *epinfo,
                             FAR uint8_t *buffer, size_t buflen)
{
  FAR struct usbhost_xhci_s *priv = XHCI_PRIV_FROM_RHPORT(rhport);
  struct xhci_trb_s          trb;

  /* Prepare TRB */

  trb.d0 = up_addrenv_va_to_pa(buffer);
  trb.d1 = XHCI_TRB_D1_IRQ_SET(0) | XHCI_TRB_D1_TXLEN_SET(buflen);
  trb.d2 = XHCI_TRB_D2_IOC | XHCI_TRB_D2_TYPE_SET(XHCI_TRB_TYPE_NORMAL);

  /* Add TRBs to ring */

  xhci_add_trb(priv, &epinfo->td, &trb, 1);

  /* Trigger transfer */

  xhci_ep_doorbell(priv, epinfo);

  return OK;
}

#ifndef CONFIG_USBHOST_ISOC_DISABLE
/****************************************************************************
 * Name: xhci_isoc_setup
 *
 * Description:
 *   Process a request on isoch endpoint.
 *
 * Assumption:
 *   The caller holds the xHCI lock.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int xhci_isoc_setup(FAR struct xhci_rhport_s *rhport,
                           FAR struct xhci_epinfo_s *epinfo,
                           FAR uint8_t *buffer, size_t buflen)
{
  FAR struct usbhost_xhci_s *priv = XHCI_PRIV_FROM_RHPORT(rhport);
  struct xhci_trb_s          trb;

  /* Prepare TRB */

  trb.d0 = up_addrenv_va_to_pa(buffer);
  trb.d1 = XHCI_TRB_D1_IRQ_SET(0) | XHCI_TRB_D1_TXLEN_SET(buflen);
  trb.d2 = XHCI_TRB_D2_IOC | XHCI_TRB_D2_TYPE_SET(XHCI_TRB_TYPE_ISOCH);

  /* Start Isoch ASAP */

  trb.d2 |= XHCI_TRB_D2_SIA;

  /* Add TRBs to ring */

  xhci_add_trb(priv, &epinfo->td, &trb, 1);

  /* Trigger transfer */

  xhci_ep_doorbell(priv, epinfo);

  return OK;
}
#endif

/****************************************************************************
 * Name: xhci_transfer_wait
 *
 * Description:
 *   Wait for an IN or OUT transfer to complete.
 *
 * Assumption:
 *   The caller holds the xHCI lock.  The caller must be aware that the xHCI
 *   lock will released while waiting for the transfer to complete, but will
 *   be re-acquired when before returning.  The state of xHCI resources could
 *   be very different upon return.
 *
 * Returned Value:
 *   On success, this function returns the number of bytes actually
 *   transferred.  For control transfers, this size includes the size of the
 *   control request plus the size of the data (which could be short); for
 *   bulk transfers, this will be the number of data bytes transfers (which
 *   could be short).
 *
 ****************************************************************************/

static ssize_t xhci_transfer_wait(FAR struct usbhost_xhci_s *priv,
                                  FAR struct xhci_epinfo_s *epinfo)
{
  int ret;

  /* Wait for the IOC completion event */

  ret = xhci_ioc_wait(epinfo);

  /* Did xhci_ioc_wait() or nxmutex_lock report an error? */

  if (ret < 0)
    {
      usbhost_trace1(XHCI_TRACE1_TRANSFER_FAILED, -ret);
      epinfo->iocwait = false;
      return (ssize_t)ret;
    }

  /* Transfer completed successfully.  Return the number of bytes
   * transferred.
   */

  return epinfo->xfrd;
}

#ifdef CONFIG_USBHOST_ASYNCH
/****************************************************************************
 * Name: xhci_ioc_async_setup
 *
 * Description:
 *   Setup to receive an asynchronous notification when a transfer completes.
 *
 * Input Parameters:
 *   epinfo - The IN or OUT endpoint descriptor for the device endpoint on
 *      which the transfer will be performed.
 *   callback - The function to be called when the transfer completes
 *   arg - An arbitrary argument that will be provided with the callback.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Called from the interrupt level
 *
 ****************************************************************************/

static inline int xhci_ioc_async_setup(FAR struct xhci_rhport_s *rhport,
                                       FAR struct xhci_epinfo_s *epinfo,
                                       usbhost_asynch_t callback,
                                       FAR void *arg)
{
  FAR struct usbhost_xhci_s *priv = XHCI_PRIV_FROM_RHPORT(rhport);
  irqstate_t                 flags;
  int                        ret  = -ENODEV;

  DEBUGASSERT(rhport && epinfo && !epinfo->iocwait &&
              epinfo->callback == NULL);

  /* Is the device still connected? */

  flags = spin_lock_irqsave(&priv->spinlock);
  if (rhport->connected)
    {
      /* Then save callback information to be used when either (1) the
       * device is disconnected, or (2) the transfer completes.
       */

      epinfo->iocwait  = false;    /* No synchronous wakeup */
      epinfo->status   = 0;        /* No status yet */
      epinfo->xfrd     = 0;        /* Nothing transferred yet */
      epinfo->result   = -EBUSY;   /* Transfer in progress */
      epinfo->callback = callback; /* Asynchronous callback */
      epinfo->arg      = arg;      /* Argument that accompanies the callback */
      ret              = OK;       /* We are good to go */
    }

  spin_unlock_irqrestore(&priv->spinlock, flags);
  return ret;
}

/****************************************************************************
 * Name: xhci_asynch_completion
 *
 * Description:
 *   This function is called at the interrupt level when an asynchronous
 *   transfer completes.  It performs the pending callback.
 *
 * Input Parameters:
 *   epinfo - The IN or OUT endpoint descriptor for the device endpoint on
 *      which the transfer was performed.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Called from the interrupt level
 *
 ****************************************************************************/

static void xhci_asynch_completion(FAR struct xhci_epinfo_s *epinfo)
{
  usbhost_asynch_t callback;
  ssize_t nbytes;
  FAR void *arg;
  int result;

  DEBUGASSERT(epinfo != NULL && epinfo->iocwait == false &&
              epinfo->callback != NULL);

  /* Extract and reset the callback info */

  callback         = epinfo->callback;
  arg              = epinfo->arg;
  result           = epinfo->result;
  nbytes           = epinfo->xfrd;

  epinfo->callback = NULL;
  epinfo->arg      = NULL;
  epinfo->result   = OK;
  epinfo->iocwait  = false;

  /* Then perform the callback.  Provide the number of bytes successfully
   * transferred or the negated errno value in the event of a failure.
   */

  if (result < 0)
    {
      nbytes = (ssize_t)result;
    }

  callback(arg, nbytes);
}
#endif

/****************************************************************************
 * Name: xhci_portsc_work
 *
 * Description:
 *   Handle Port Change work.
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *   - Never called directly from xhci_events_poll() otherwise it gets stuck
 *     on CLASS_DISCONNECTED()
 *
 ****************************************************************************/

static void xhci_portsc_work(FAR void *arg)
{
  FAR struct usbhost_xhci_s    *priv = arg;
  FAR struct usbhost_hubport_s *hport;
  FAR struct xhci_rhport_s     *rhport;
  uint32_t                      portsc;
  int                           rhpndx;

  /* REVISIT: should this logic be protected? We can't use spinlock
   * here because we get stack in CLASS_DISCONNECTED().
   */

  /* Handle root hub status change on each root port */

  for (rhpndx = 0; rhpndx < priv->no_ports; rhpndx++)
    {
      rhport = &priv->rhport[rhpndx];
      portsc = xhci_oper_getreg(priv, XHCI_PORTSC(rhpndx));

      usbhost_vtrace2(XHCI_VTRACE2_PORTSC, rhpndx + 1, portsc);

      /* Handle port connection status change (CSC) events */

      if ((portsc & XHCI_PORTSC_CSC) != 0)
        {
          usbhost_vtrace1(XHCI_VTRACE1_PORTSC_CSC, portsc);

          /* Check current connect status */

          if ((portsc & XHCI_PORTSC_CCS) != 0)
            {
              /* Connected ... Did we just become connected? */

              if (!rhport->connected)
                {
                  /* Yes.. connected. */

                  rhport->connected = true;

                  usbhost_vtrace2(XHCI_VTRACE2_PORTSC_CONNECTED,
                                  rhpndx + 1, priv->pscwait);

                  /* Notify any waiters */

                  if (priv->pscwait)
                    {
                      nxsem_post(&priv->pscsem);
                      priv->pscwait = false;
                    }
                }
              else
                {
                  usbhost_vtrace1(XHCI_VTRACE1_PORTSC_CONNALREADY, portsc);
                }
            }
          else
            {
              /* Disconnected... Did we just become disconnected? */

              if (rhport->connected)
                {
                  /* Yes.. disconnect the device */

                  usbhost_vtrace2(XHCI_VTRACE2_PORTSC_DISCONND,
                                  rhpndx + 1, priv->pscwait);

                  rhport->connected = false;

                  /* Are we bound to a class instance? */

                  hport = &rhport->hport.hport;
                  if (hport->devclass)
                    {
                      /* Yes.. Disconnect the class. */

                      CLASS_DISCONNECTED(hport->devclass);
                      hport->devclass = NULL;
                    }

                  /* Notify any waiters for the Root Hub Status change
                   * event.
                   */

                  if (priv->pscwait)
                    {
                      nxsem_post(&priv->pscsem);
                      priv->pscwait = false;
                    }
                }
              else
                {
                  usbhost_vtrace1(XHCI_VTRACE1_PORTSC_DISCALREADY, portsc);
                }
            }
        }

      /* Clear pending bit but don't touch PED ! */

      portsc &= ~XHCI_PORTSC_PED;
      xhci_oper_putreg(priv, XHCI_PORTSC(rhpndx), portsc);
    }
}

/****************************************************************************
 * Name: xhci_transfer_complete
 *
 * Description:
 *   Handle transfer complete event
 *
 ****************************************************************************/

static void xhci_transfer_complete(FAR struct usbhost_xhci_s *priv,
                                   FAR struct xhci_trb_s *evt)
{
  FAR struct xhci_epinfo_s *epinfo;
  uint32_t                  tl   = XHCI_TRB_D1_TXLEN_GET(evt->d1);
  uint8_t                   slot = XHCI_TRB_D2_SLOTID_GET(evt->d2);
  uint8_t                   ep   = XHCI_TRB_D2_EP_GET(evt->d2);
  uint8_t                   ret  = XHCI_TRB_D1_CC_GET(evt->d1);
  irqstate_t                flags;

  /* Get EP associated with this transfer */

  epinfo = priv->devs[slot - 1].epinfo[ep - 1];
  DEBUGASSERT(epinfo != NULL);

  flags = spin_lock_irqsave(&priv->spinlock);

  /* Get transferred length */

  if (epinfo->buflen > 0)
    {
      epinfo->xfrd = epinfo->buflen - tl;
    }

  /* Check transfer status */

  if (ret == XHCI_TRB_CC_SUCCESS)
    {
      /* Report success */

      epinfo->status = 0;
      epinfo->result = OK;
    }

  else if (ret == XHCI_TRB_CC_STALL)
    {
      /* Report STALL condition */

      epinfo->status = 0;
      epinfo->result = -EPERM;
    }

  else if (ret == XHCI_TRB_CC_SHORT_PKT)
    {
      /* Report success */

      epinfo->status = 0;
      epinfo->result = OK;
    }

  else
    {
      /* Report error */

      pcierr("transfer CC = %d\n", ret);
      epinfo->status = ret;
      epinfo->result = -EIO;
    }

  /* Is there a thread waiting for this transfer to complete? */

  if (epinfo->iocwait)
    {
      /* Yes... wake it up */

      epinfo->iocwait = 0;
      nxsem_post(&epinfo->iocsem);
    }

#ifdef CONFIG_USBHOST_ASYNCH
  /* No.. Is there a pending asynchronous transfer? */

  else if (epinfo->callback != NULL)
    {
      /* Yes.. perform the callback */

      xhci_asynch_completion(epinfo);
    }
#endif

  spin_unlock_irqrestore(&priv->spinlock, flags);
}

/****************************************************************************
 * Name: xhci_envet_complete
 *
 * Description:
 *   Handle event complete event
 *
 ****************************************************************************/

static void xhci_event_complete(FAR struct usbhost_xhci_s *priv,
                                FAR struct xhci_trb_s *evt)
{
  irqstate_t flags;

  /* REVISIT: we assume for now that only one command is pending */

  /* Store command result */

  flags = spin_lock_irqsave(&priv->spinlock);
  priv->cmdres.d0 = evt->d0;
  priv->cmdres.d1 = evt->d1;
  priv->cmdres.d2 = evt->d2;
  spin_unlock_irqrestore(&priv->spinlock, flags);

  /* Signal that command is done */

  nxsem_post(&priv->cmdsem);
}

/****************************************************************************
 * Name: xhci_events_poll
 *
 * Description:
 *   Poll all pending events
 *
 ****************************************************************************/

static int xhci_events_poll(FAR struct usbhost_xhci_s *priv)
{
  FAR struct xhci_trb_s *evt;
  uintptr_t              addr;
  uint8_t                type;
  uint32_t               d2;

  /* Invalidate event ring */

  up_invalidate_dcache((uintptr_t)priv->evnt.ring,
                       (uintptr_t)(priv->evnt.ring + XHCI_EVENT_MAX));

  /* Handle all pending events */

  while (1)
    {
      evt = &priv->evnt.ring[priv->evnt.i];

      /* Update address */

      addr = (uintptr_t)evt;

      d2 = le32toh(evt->d2);
      if ((d2 & XHCI_TRB_D2_C) != priv->evnt.ccs)
        {
          break;
        }

      type = XHCI_TRB_D2_TYPE_GET(d2);

      switch (type)
        {
          /* Transfer Event */

          case XHCI_TRB_EVT_TRANSFER:
            {
              xhci_transfer_complete(priv, evt);

              break;
            }

          /* Command Completion Event */

          case XHCI_TRB_EVT_CMD_COMP:
            {
              xhci_event_complete(priv, evt);

              break;
            }

          /* Port Status Change Event */

          case XHCI_TRB_EVT_PSTAT_CHANGE:
            {
              /* We have to handle Port Status Change in a separate work
               * queue, otherwise we'll get stuck when handling disconnect
               * request.
               */

              if (work_available(&priv->pscwork))
                {
                  work_queue(LPWORK, &priv->pscwork, xhci_portsc_work,
                             (FAR void *)priv, 0);
                }

              break;
            }

          default:
            {
              pciinfo("ignored event %d\n", type);
              break;
            }
        }

      /* Next event */

      priv->evnt.i++;

      /* Handle ring wrap */

      if (priv->evnt.i >= XHCI_EVENT_MAX)
        {
          xhci_ring_reset(&priv->evnt, true);
        }
    }

  /* Clear ERDP busy bit and update dequeue pointer */

  addr = up_addrenv_va_to_pa((FAR void *)addr);
  addr |= XHCI_ERDP_EHB;
  xhci_runt_putreg_8b(priv, XHCI_ERDP(0), addr);

  return OK;
}

/****************************************************************************
 * Name: xhci_interupt_work
 *
 * Description:
 *   Handle xHCI interrupts
 *
 ****************************************************************************/

static void xhci_interrupt_work(FAR void *arg)
{
  FAR struct usbhost_xhci_s *priv = arg;
  uint32_t                   iman;

  xhci_events_poll(priv);

  /* Port Change Detect */

  if (priv->pending & XHCI_USBSTS_PCD)
    {
      /* Handled as event in xhci_events_poll() */

      pciinfo("Port Change Detect\n");
    }

  /* Host Controller Halted */

  if (priv->pending & XHCI_USBSTS_HCH)
    {
      pciinfo("Host Controller Halted\n");
    }

  /* Host System Error */

  if (priv->pending & XHCI_USBSTS_HSE)
    {
      pciinfo("Host System Error\n");
    }

  /* Host Controller Error */

  if (priv->pending & XHCI_USBSTS_HCE)
    {
      pciinfo("Host Controller Error\n");
    }

  /* ACK interrupts */

  xhci_oper_putreg(priv, XHCI_USBSTS, priv->pending);

  /* Clear interrupter pending bit */

  iman = xhci_runt_getreg(priv, XHCI_IMAN(0));
  if (iman & XHCI_IMAN_IP)
    {
      xhci_runt_putreg(priv, XHCI_IMAN(0), iman);
    }

  /* Clear pending bits */

  priv->pending = 0;
}

/****************************************************************************
 * Name: xhci_interupt
 *
 * Description:
 *   Interrupt handler for xHCI
 *
 ****************************************************************************/

static int xhci_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct usbhost_xhci_s *priv = arg;

  /* Get pending interrupts */

  priv->pending = xhci_oper_getreg(priv, XHCI_USBSTS);

  /* Handle interrupts in worker */

  if (work_available(&priv->work))
    {
      work_queue(HPWORK, &priv->work, xhci_interrupt_work, arg, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: xhci_wait
 *
 * Description:
 *   Wait for a device to be connected or disconnected to/from a hub port.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter from
 *     the call to the USB driver initialization logic.
 *   hport - The location to return the hub port descriptor that detected
 *     the connection related event.
 *
 * Returned Value:
 *   Zero (OK) is returned on success when a device is connected or
 *   disconnected. This function will not return until either (1) a device is
 *   connected or disconnect to/from any hub port or until (2) some failure
 *   occurs.  On a failure, a negated errno value is returned indicating the
 *   nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int xhci_wait(FAR struct usbhost_connection_s *conn,
                     FAR struct usbhost_hubport_s **hport)
{
  FAR struct usbhost_xhci_s    *priv = XHCI_PRIV_FROM_CONN(conn);
  FAR struct xhci_rhport_s     *rhport;
  FAR struct usbhost_hubport_s *connport;
  irqstate_t                    flags;
  int                           rhpndx;
  int                           ret;

  /* Loop until the connection state changes on one of the root hub ports or
   * until an error occurs.
   */

  while (true)
    {
      flags = spin_lock_irqsave(&priv->spinlock);

      /* Check for a change in the connection state on any root hub port */

      for (rhpndx = 0; rhpndx < priv->no_ports; rhpndx++)
        {
          /* Has the connection state changed on the RH port? */

          rhport   = &priv->rhport[rhpndx];
          connport = &rhport->hport.hport;
          if (rhport->connected != connport->connected)
            {
              /* Yes.. Return the RH port to inform the caller which
               * port has the connection change.
               */

              connport->connected = rhport->connected;
              *hport = connport;
              spin_unlock_irqrestore(&priv->spinlock, flags);

              usbhost_vtrace2(XHCI_VTRACE2_MONWAKEUP,
                              rhpndx + 1, rhport->connected);
              return OK;
            }
        }

#ifdef CONFIG_USBHOST_HUB
      /* Is a device connected to an external hub? */

      if (priv->hport)
        {
          /* Yes.. return the external hub port */

          connport = priv->hport;
          priv->hport = NULL;

          *hport = (FAR struct usbhost_hubport_s *)connport;
          spin_unlock_irqrestore(&priv->spinlock, flags);

          usbhost_vtrace2(XHCI_VTRACE2_MONWAKEUP,
                          HPORT(connport), connport->connected);
          return OK;
        }
#endif

      /* No changes on any port. Wait for a connection/disconnection event
       * and check again
       */

      priv->pscwait = true;

      spin_unlock_irqrestore(&priv->spinlock, flags);

      ret = nxsem_wait_uninterruptible(&priv->pscsem);
      if (ret < 0)
        {
          return ret;
        }
    }
}

/****************************************************************************
 * Name: xhci_rh_enumerate/xhci_enumerate
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
 *   conn  - The USB host connection instance obtained as a parameter from
 *           the call to the USB driver initialization logic.
 *   hport - The descriptor of the hub port that has the newly connected
 *           device.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int xhci_rh_enumerate(FAR struct usbhost_connection_s *conn,
                             FAR struct usbhost_hubport_s *hport)
{
  FAR struct usbhost_xhci_s *priv = XHCI_PRIV_FROM_CONN(conn);
  FAR struct xhci_rhport_s  *rhport;
  int                        rhpndx;
  int                        ret;

  DEBUGASSERT(hport != NULL);
  rhpndx = hport->port;

  DEBUGASSERT(rhpndx >= 0 && rhpndx < priv->no_ports);
  rhport = &priv->rhport[rhpndx];

  /* Are we connected to a device?  The caller should have called the wait()
   * method first to be assured that a device is connected.
   */

  if (!rhport->connected)
    {
      /* No, return an error */

      pcierr("not connected\n");
      return -ENODEV;
    }

  /* Enable port */

  ret = xhci_port_enable(priv, hport);
  if (ret < 0)
    {
      pcierr("Failed to enable port %d\n", ret);
      return ret;
    }

  /* Initialize device data */

  ret = xhci_device_init(priv, rhport);
  if (ret < 0)
    {
      pcierr("Failed to initialize device %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: xhci_enumerate
 *
 * Description:
 *   See description above.
 *
 ****************************************************************************/

static int xhci_enumerate(FAR struct usbhost_connection_s *conn,
                          FAR struct usbhost_hubport_s *hport)
{
  int ret;

  /* If this is a connection on the root hub, then we need to go to
   * little more effort to get the device speed.  If it is a connection
   * on an external hub, then we already have that information.
   */

  DEBUGASSERT(hport);
#ifdef CONFIG_USBHOST_HUB
  if (ROOTHUB(hport))
#endif
    {
      ret = xhci_rh_enumerate(conn, hport);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Then let the common usbhost_enumerate do the real enumeration. */

  ret = usbhost_enumerate(hport, &hport->devclass);
  if (ret < 0)
    {
      /* Failed to enumerate */

      /* If this is a root hub port, then marking the hub port not connected
       * will cause xhci_wait() to return and we will try the connection
       * again.
       */

      hport->connected = false;
    }

  return ret;
}

/****************************************************************************
 * Name: xhci_ep0configure
 *
 * Description:
 *   Configure endpoint 0.  This method is normally used internally by the
 *   enumerate() method but is made available at the interface to support
 *   an external implementation of the enumeration logic.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   funcaddr - The USB address of the function containing the endpoint that
 *     EP0 controls.  A funcaddr of zero will be received if no address is
 *     yet assigned to the device.
 *   speed - The speed of the port USB_SPEED_LOW, _FULL, or _HIGH
 *   maxpacketsize - The maximum number of bytes that can be sent to or
 *    received from the endpoint in a single data packet
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int xhci_ep0configure(FAR struct usbhost_driver_s *drvr,
                             usbhost_ep_t ep0, uint8_t funcaddr,
                             uint8_t speed, uint16_t maxpacketsize)
{
  FAR struct xhci_rhport_s  *rhport = (FAR struct xhci_rhport_s *)drvr;
  FAR struct xhci_epinfo_s  *epinfo = (FAR struct xhci_epinfo_s *)ep0;
  FAR struct usbhost_xhci_s *priv   = XHCI_PRIV_FROM_DRVR(drvr);
  uint64_t                   ctx;
  int                        ret;

  DEBUGASSERT(drvr != NULL && epinfo != NULL && maxpacketsize < 2048);

  ret = nxmutex_lock(&priv->lock);
  if (ret >= 0)
    {
      /* Update max packet size */

      rhport->dev->input->ep[0].ctx1 &= ~XHCI_EP_CTX1_MAXPKT_MASK;
      rhport->dev->input->ep[0].ctx1 |= XHCI_EP_CTX1_MAXPKT(maxpacketsize);

      /* Add Slot Context and EP0 Context */

      xhci_context_ctrl(priv, rhport->dev, 0,
                        XHCI_IN_CTX1_A(XHCI_SLOT_FLAG) |
                        XHCI_IN_CTX1_A(XHCI_EP0_FLAG));

      /* Flush Device input context */

      up_flush_dcache((uintptr_t)rhport->dev->input,
                      (uintptr_t)rhport->dev->input +
                      sizeof(struct xhci_input_dev_ctx_s));

      /* Free mutex before command execution */

      nxmutex_unlock(&priv->lock);

      ctx = up_addrenv_va_to_pa(rhport->dev->input);
      ret = xhci_cmd_evalctx(priv, epinfo->slot, ctx);
    }

  return ret;
}

/****************************************************************************
 * Name: xhci_epalloc
 *
 * Description:
 *   Allocate and configure one endpoint.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int xhci_epalloc(FAR struct usbhost_driver_s *drvr,
                        FAR const struct usbhost_epdesc_s *epdesc,
                        FAR usbhost_ep_t *ep)
{
  FAR struct usbhost_xhci_s    *priv   = XHCI_PRIV_FROM_DRVR(drvr);
  FAR struct xhci_rhport_s     *rhport = (FAR struct xhci_rhport_s *)drvr;
  FAR struct usbhost_hubport_s *hport;
  FAR struct xhci_epinfo_s     *epinfo;
  FAR struct xhci_dev_s        *dev;
  uint32_t                      mask;
  uint8_t                       eptype;
  uint8_t                       idx;
  int                           ret;

  /* Sanity check.  NOTE that this method should only be called if a device
   * is connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(drvr != 0 && epdesc != NULL && epdesc->hport != NULL
              && ep != NULL);
  hport = epdesc->hport;

  /* Terse output only if we are tracing */

#ifdef CONFIG_USBHOST_TRACE
  usbhost_vtrace2(XHCI_VTRACE2_EPALLOC, epdesc->addr, epdesc->xfrtype);
#else
  pciinfo("EP%d DIR=%s FA=%08x TYPE=%d Interval=%d MaxPacket=%d\n",
          epdesc->addr, epdesc->in ? "IN" : "OUT", hport->funcaddr,
          epdesc->xfrtype, epdesc->interval, epdesc->mxpacketsize);
#endif

  /* Allocate a endpoint information structure */

  epinfo = kmm_zalloc(sizeof(struct xhci_epinfo_s));
  if (!epinfo)
    {
      return -ENOMEM;
    }

  /* Initialize the endpoint container (which is really just another form of
   * 'struct usbhost_epdesc_s', packed differently and with additional
   * information.  A cleaner design might just embed struct usbhost_epdesc_s
   * inside of struct xhci_epinfo_s and just memcpy here.
   */

  epinfo->dirin = epdesc->in;
  epinfo->epno  = epdesc->addr;

#ifndef CONFIG_USBHOST_INT_DISABLE
  epinfo->interval  = epdesc->interval;
#endif
  epinfo->xfrtype   = epdesc->xfrtype;
  nxsem_init(&epinfo->iocsem, 0, 0);

  /* xhci_epno_get() returns Device Context Index (DCI) */

  idx              = xhci_epno_get(epinfo);
  mask             = XHCI_IN_CTX1_A(XHCI_EP_FLAG(idx));
  dev              = rhport->dev;
  dev->epinfo[idx - 1] = epinfo;

  /* TD rings already allocated but not connected yet. */

  ret = xhci_ring_init(&epinfo->td, XHCI_TD_MAX);
  if (ret < 0)
    {
      pcierr("ep ring init failed\n");
      return ret;
    }

  /* Store slot ID for later */

  epinfo->slot = rhport->slot;

#ifdef CONFIG_USBHOST_HUB
  if (hport->speed != USB_SPEED_HIGH)
    {
      /* A high speed hub exists between this device and the root hub
       * otherwise we would not get here.
       */

      FAR struct usbhost_hubport_s *parent = hport->parent;

      for (; parent->speed != USB_SPEED_HIGH; parent = hport->parent)
        {
          hport = parent;
        }

      if (parent->speed == USB_SPEED_HIGH)
        {
          epinfo->hubport = HPORT(hport);
          epinfo->hubaddr = hport->parent->funcaddr;
        }
      else
        {
          return -EINVAL;
        }
    }
#endif

  /* Get EP type */

  switch (epinfo->xfrtype)
    {
      case USB_EP_ATTR_XFER_BULK:
        {
          eptype = epinfo->dirin ? XHCI_EPTYPE_BULK_IN :
                   XHCI_EPTYPE_BULK_OUT;
          break;
        }

#ifndef CONFIG_USBHOST_INT_DISABLE
      case USB_EP_ATTR_XFER_INT:
#endif
        {
          eptype = epinfo->dirin ? XHCI_EPTYPE_INTR_IN :
                   XHCI_EPTYPE_INTR_OUT;
          break;
        }

#ifndef CONFIG_USBHOST_ISOC_DISABLE
      case USB_EP_ATTR_XFER_ISOC:
        {
          eptype = epinfo->dirin ? XHCI_EPTYPE_ISO_IN :
                   XHCI_EPTYPE_ISO_OUT;
          break;
        }
#endif

      default:
        {
          return -ENOSYS;
        }
    }

  /* REVISIT: do we need disable EP here? */

  /* Initialize EP context.
   * Max Burst Size set for 0 for now (USB3.0 specific)
   */

  xhci_ep_configure(priv, &dev->input->ep[idx - 1],
                    eptype, epdesc->mxpacketsize, 0,
                    up_addrenv_va_to_pa(epinfo->td.ring),
                    0, epinfo->interval);

  /* Evaluate the slot context */

  xhci_context_ctrl(priv, dev, 0, mask | XHCI_IN_CTX1_A(XHCI_SLOT_FLAG));

  up_flush_dcache((uintptr_t)dev->input,
                  (uintptr_t)dev->input +
                  sizeof(struct xhci_input_dev_ctx_s));

  /* Configure EP */

  ret = xhci_cmd_cfgep(priv, epinfo->slot,
                       up_addrenv_va_to_pa(dev->input), false);
  if (ret < 0)
    {
      pcierr("failed to configure EP %d\n", ret);
      return ret;
    }

  /* Success.. return an opaque reference to the endpoint information
   * structure instance
   */

  *ep = (usbhost_ep_t)epinfo;
  return OK;
}

/****************************************************************************
 * Name: xhci_epfree
 *
 * Description:
 *   Free an endpoint previously allocated by DRVR_EPALLOC.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *           call to the class create() method.
 *   ep   - The endpoint to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int xhci_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  FAR struct xhci_epinfo_s *epinfo = (FAR struct xhci_epinfo_s *)ep;

  /* There should not be any pending, transfers */

  DEBUGASSERT(drvr && epinfo && epinfo->iocwait == 0);

  /* Free ring */

  xhci_ring_deinit(&epinfo->td);

  /* Free the container */

  kmm_free(epinfo);
  return OK;
}

/****************************************************************************
 * Name: xhci_alloc
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor
 *   data can be accessed more efficiently.  This method provides a
 *   mechanism to allocate the request/descriptor memory.  If the underlying
 *   hardware does not support such "special" memory, this functions may
 *   simply map to kmm_malloc().
 *
 *   This interface was optimized under a particular assumption.  It was
 *   assumed that the driver maintains a pool of small, pre-allocated buffers
 *   for descriptor traffic.  NOTE that size is not an input, but an output:
 *   The size of the pre-allocated buffer is returned.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   buffer - The address of a memory location provided by the caller in
 *     which to return the allocated buffer memory address.
 *   maxlen - The address of a memory location provided by the caller in
 *     which to return the maximum size of the allocated buffer memory.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int xhci_alloc(FAR struct usbhost_driver_s *drvr,
                      FAR uint8_t **buffer, FAR size_t *maxlen)
{
  int ret = -ENOMEM;

  DEBUGASSERT(drvr && buffer && maxlen);

  /* Allocated buffer must not cross page boundaries */

  *buffer = (FAR uint8_t *)kmm_memalign((XHCI_PAGE_SIZE / 2) , XHCI_BUFSIZE);
  if (*buffer)
    {
      *maxlen = XHCI_BUFSIZE;
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: xhci_free
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor
 *   data can be accessed more efficiently.  This method provides a
 *   mechanism to free that request/descriptor memory.  If the underlying
 *   hardware does not support such "special" memory, this functions may
 *   simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr   - The USB host driver instance obtained as a parameter from the
 *            call to the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int xhci_free(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer)
{
  DEBUGASSERT(drvr && buffer);

  /* No special action is require to free the transfer/descriptor buffer
   * memory
   */

  kmm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: xhci_ioalloc
 *
 * Description:
 *   Some hardware supports special memory in which larger IO buffers can
 *   be accessed more efficiently.  This method provides a mechanism to
 *   allocate the request/descriptor memory.  If the underlying hardware
 *   does not support such "special" memory, this functions may simply map
 *   to kumm_malloc.
 *
 *   This interface differs from DRVR_ALLOC in that the buffers are variable-
 *   sized.
 *
 * Input Parameters:
 *   drvr   - The USB host driver instance obtained as a parameter from the
 *            call to the class create() method.
 *   buffer - The address of a memory location provided by the caller in
 *            which to return the allocated buffer memory address.
 *   buflen - The size of the buffer required.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int xhci_ioalloc(FAR struct usbhost_driver_s *drvr,
                        FAR uint8_t **buffer, size_t buflen)
{
  int ret = -ENOMEM;

  DEBUGASSERT(drvr && buffer && buflen > 0);

  /* Large transfers are not supported now */

  if (buflen > XHCI_PAGE_SIZE)
    {
      return -ENOMEM;
    }

  /* Allocated buffer must not cross page boundaries */

  *buffer = (FAR uint8_t *)kmm_memalign((XHCI_PAGE_SIZE / 2) , buflen);
  if (*buffer)
    {
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: xhci_iofree
 *
 * Description:
 *   Some hardware supports special memory in which IO data can  be accessed
 *   more efficiently.  This method provides a mechanism to free that IO
 *   buffer memory.  If the underlying hardware does not support such
 *   "special" memory, this functions may simply map to kumm_free().
 *
 * Input Parameters:
 *   drvr   - The USB host driver instance obtained as a parameter from the
 *            call to the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int xhci_iofree(FAR struct usbhost_driver_s *drvr,
                       FAR uint8_t *buffer)
{
  DEBUGASSERT(drvr && buffer);

  /* No special action is require to free the transfer/descriptor buffer
   * memory
   */

  kmm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: xhci_ctrl_xfer
 *
 * Description:
 *   Process a IN or OUT request on the control endpoint.  These methods
 *   will enqueue the request and wait for it to complete.  Only one
 *   transfer may be queued; Neither these methods nor the transfer() method
 *   can be called again until the control transfer function returns.
 *
 *   These are blocking methods; these functions will not return until the
 *   control transfer has completed.
 *
 * Input Parameters:
 *   drvr   - The USB host driver instance obtained as a parameter from the
 *            call to the class create() method.
 *   ep0    - The control endpoint to send/receive the control request.
 *   req    - Describes the request to be sent.  This request must lie in
 *            memory created by DRVR_ALLOC.
 *   buffer - A buffer used for sending the request and for returning any
 *            responses.  This buffer must be large enough to hold the
 *            length value in the request description. buffer must have been
 *            allocated using DRVR_ALLOC.
 *
 *   NOTE: On an IN transaction, req and buffer may refer to the xHCI
 *   allocated memory.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int xhci_ctrl_xfer(FAR struct usbhost_driver_s *drvr,
                          usbhost_ep_t ep0,
                          FAR const struct usb_ctrlreq_s *req,
                          FAR uint8_t *buffer)
{
  FAR struct usbhost_xhci_s *priv    = XHCI_PRIV_FROM_DRVR(drvr);
  FAR struct xhci_rhport_s  *rhport  = (FAR struct xhci_rhport_s *)drvr;
  FAR struct xhci_epinfo_s  *ep0info = (FAR struct xhci_epinfo_s *)ep0;
  uint16_t                   len;
  ssize_t                    nbytes;
  int                        ret;

  DEBUGASSERT(rhport != NULL && ep0info != NULL && req != NULL);

  len = xhci_getle16(req->len);

  /* Terse output only if we are tracing */

#ifdef CONFIG_USBHOST_TRACE
  usbhost_vtrace2(XHCI_VTRACE2_CTRLINOUT, RHPORT(rhport), req->req);
#endif

  /* Special case for SET_ADDRESS request */

  if (req->req == USB_REQ_SETADDRESS)
    {
      /* Reset EP0 ring to its initial state, so when xHCI update EP0
       * context, TD dequeue pointer would be valid. It may be already
       * off, because the USB Host stack has already sent some messages
       * on control EP.
       */

      xhci_ring_init(&rhport->dev->rhport->ep0.td, 0);

      /* Issue SET_ADDRESS request */

      ret = xhci_address_set(priv, rhport, true);
      if (ret == OK)
        {
          /* Store USB Device Address assigned by xHCI */

          ep0info->devaddr =
            XHCI_ST_CTX3_ADDR_GET(rhport->dev->ctx->slot.ctx[3]);
          rhport->dev->input->slot.ctx[3] = rhport->dev->ctx->slot.ctx[3];
        }

      return OK;
    }

  /* We must have exclusive access to the XHCI hardware and data
   * structures.
   */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Set the request for the IOC event well BEFORE initiating the transfer. */

  ret = xhci_ioc_setup(rhport, ep0info, 0);
  if (ret != OK)
    {
      goto errout_with_lock;
    }

  /* Now initiate the transfer */

  ret = xhci_control_setup(rhport, ep0info, req, buffer, len);
  if (ret < 0)
    {
      pcierr("ERROR: xhci_control_setup failed: %d\n", ret);
      goto errout_with_iocwait;
    }

  nxmutex_unlock(&priv->lock);

  /* And wait for the transfer to complete */

  nbytes = xhci_transfer_wait(priv, ep0info);
  return nbytes >= 0 ? OK : (int)nbytes;

errout_with_iocwait:
  ep0info->iocwait = false;
errout_with_lock:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: xhci_ctrlin
 *
 * Description:
 *   Process IN request on the control endpoint. For details, see
 *   description for xhci_ctrl_xfer().
 *
 ****************************************************************************/

static int xhci_ctrlin(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                       FAR const struct usb_ctrlreq_s *req,
                       FAR uint8_t *buffer)
{
  /* xhci_ctrl_xfer() can handle both directions */

  return xhci_ctrl_xfer(drvr, ep0, req, buffer);
}

/****************************************************************************
 * Name: xhci_ctrlout
 *
 * Description:
 *   Process OUT request on the control endpoint. For details, see
 *   description for xhci_ctrl_xfer().
 *
 ****************************************************************************/

static int xhci_ctrlout(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                        FAR const struct usb_ctrlreq_s *req,
                        FAR const uint8_t *buffer)
{
  /* xhci_ctrl_xfer() can handle both directions. We just need to work around
   * the differences in the function signatures.
   */

  return xhci_ctrl_xfer(drvr, ep0, req, (FAR uint8_t *)buffer);
}

/****************************************************************************
 * Name: xhci_transfer
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
 *   drvr   - The USB host driver instance obtained as a parameter from the
 *            call to the class create() method.
 *   ep     - The IN or OUT endpoint descriptor for the device endpoint on
 *            which to perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or
 *            received (IN endpoint).  buffer must have been allocated using
 *            DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *
 * Returned Value:
 *   On success, a non-negative value is returned that indicates the number
 *   of bytes successfully transferred.  On a failure, a negated errno value
 *   is returned that indicates the nature of the failure:
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

static ssize_t xhci_transfer(FAR struct usbhost_driver_s *drvr,
                             usbhost_ep_t ep, FAR uint8_t *buffer,
                             size_t buflen)
{
  FAR struct usbhost_xhci_s *priv   = XHCI_PRIV_FROM_DRVR(drvr);
  FAR struct xhci_rhport_s  *rhport = (FAR struct xhci_rhport_s *)drvr;
  FAR struct xhci_epinfo_s  *epinfo = (FAR struct xhci_epinfo_s *)ep;
  ssize_t                    nbytes;
  int                        ret;

  DEBUGASSERT(priv && rhport && epinfo && buffer && buflen > 0);

  /* We must have exclusive access to the xHCI hardware and data
   * structures.
   */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  /* Set the request for the IOC event well BEFORE initiating the transfer. */

  ret = xhci_ioc_setup(rhport, epinfo, buflen);
  if (ret != OK)
    {
      goto errout_with_lock;
    }

  /* Initiate the transfer */

  switch (epinfo->xfrtype)
    {
      case USB_EP_ATTR_XFER_BULK:
#ifndef CONFIG_USBHOST_INT_DISABLE
      case USB_EP_ATTR_XFER_INT:
#endif
        {
          ret = xhci_normal_setup(rhport, epinfo, buffer, buflen);
          break;
        }

#ifndef CONFIG_USBHOST_ISOC_DISABLE
      case USB_EP_ATTR_XFER_ISOC:
        {
          ret = xhci_isoc_setup(rhport, epinfo, buffer, buflen);
          break;
        }
#endif

      case USB_EP_ATTR_XFER_CONTROL:
      default:
        {
          usbhost_trace1(XHCI_TRACE1_BADXFRTYPE, epinfo->xfrtype);
          ret = -ENOSYS;
          break;
        }
    }

  /* Check for errors in the setup of the transfer */

  if (ret < 0)
    {
      uerr("ERROR: Transfer setup failed: %d\n", ret);
      goto errout_with_iocwait;
    }

  nxmutex_unlock(&priv->lock);

  /* Then wait for the transfer to complete */

  nbytes = xhci_transfer_wait(priv, epinfo);
  return nbytes;

errout_with_iocwait:
  epinfo->iocwait = false;
errout_with_lock:
  nxmutex_unlock(&priv->lock);
  return (ssize_t)ret;
}

/****************************************************************************
 * Name: xhci_asynch
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
 *   drvr     - The USB host driver instance obtained as a parameter from
 *              the call to the class create() method.
 *   ep       - The IN or OUT endpoint descriptor for the device endpoint on
 *              which to perform the transfer.
 *   buffer   - A buffer containing the data to be sent (OUT endpoint) or
 *              received (IN endpoint).  buffer must have been allocated
 *              using DRVR_ALLOC
 *   buflen   - The length of the data to be sent or received.
 *   callback - This function will be called when the transfer completes.
 *   arg      - The arbitrary parameter that will be passed to the callback
 *              function when the transfer completes.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int xhci_asynch(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                       FAR uint8_t *buffer, size_t buflen,
                       usbhost_asynch_t callback, FAR void *arg)
{
  FAR struct usbhost_xhci_s *priv   = XHCI_PRIV_FROM_DRVR(drvr);
  FAR struct xhci_rhport_s  *rhport = (FAR struct xhci_rhport_s *)drvr;
  FAR struct xhci_epinfo_s  *epinfo = (FAR struct xhci_epinfo_s *)ep;
  int                        ret;

  DEBUGASSERT(priv && rhport && epinfo && buffer && buflen > 0);

  /* We must have exclusive access to the xHCI hardware and data
   * structures.
   */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Set the request for the callback well BEFORE initiating the transfer. */

  ret = xhci_ioc_async_setup(rhport, epinfo, callback, arg);
  if (ret != OK)
    {
      goto errout_with_lock;
    }

  /* Initiate the transfer */

  switch (epinfo->xfrtype)
    {
      case USB_EP_ATTR_XFER_BULK:
#ifndef CONFIG_USBHOST_INT_DISABLE
      case USB_EP_ATTR_XFER_INT:
#endif
        {
          ret = xhci_normal_setup(rhport, epinfo, buffer, buflen);
          break;
        }

#ifndef CONFIG_USBHOST_ISOC_DISABLE
      case USB_EP_ATTR_XFER_ISOC:
        {
          ret = xhci_isoc_setup(rhport, epinfo, buffer, buflen);
          break;
        }
#endif

      case USB_EP_ATTR_XFER_CONTROL:
      default:
        {
          usbhost_trace1(XHCI_TRACE1_BADXFRTYPE, epinfo->xfrtype);
          ret = -ENOSYS;
          break;
        }
    }

  /* Check for errors in the setup of the transfer */

  if (ret < 0)
    {
      goto errout_with_callback;
    }

  /* The transfer is in progress */

  nxmutex_unlock(&priv->lock);
  return OK;

errout_with_callback:
  epinfo->callback = NULL;
  epinfo->arg      = NULL;
errout_with_lock:
  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif /* CONFIG_USBHOST_ASYNCH */

/****************************************************************************
 * Name: xhci_cancel
 *
 * Description:
 *   Cancel a pending transfer on an endpoint.  Cancelled synchronous or
 *   asynchronous transfer will complete normally with the error -ESHUTDOWN.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which
 *     an asynchronous transfer should be transferred.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 ****************************************************************************/

static int xhci_cancel(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  FAR struct xhci_epinfo_s  *epinfo = (FAR struct xhci_epinfo_s *)ep;
  FAR struct usbhost_xhci_s *priv   = XHCI_PRIV_FROM_DRVR(drvr);
#ifdef CONFIG_USBHOST_ASYNCH
  usbhost_asynch_t           callback;
  FAR void                  *arg;
#endif
  irqstate_t                 flags;
  bool                       iocwait;

  DEBUGASSERT(epinfo);

  /* Sample and reset all transfer termination information.  This will
   * prevent any callbacks from occurring while we performing the
   * cancellation.  The transfer may still be in progress, however, so this
   * does not eliminate other DMA-related race conditions.
   */

  flags = spin_lock_irqsave(&priv->spinlock);
#ifdef CONFIG_USBHOST_ASYNCH
  callback         = epinfo->callback;
  arg              = epinfo->arg;
#endif
  iocwait          = epinfo->iocwait;

#ifdef CONFIG_USBHOST_ASYNCH
  epinfo->callback = NULL;
  epinfo->arg      = NULL;
#endif
  epinfo->iocwait  = false;
  spin_unlock_irqrestore(&priv->spinlock, flags);

  /* Bail if there is no transfer in progress for this endpoint */

#ifdef CONFIG_USBHOST_ASYNCH
  if (callback == NULL && !iocwait)
#else
  if (!iocwait)
#endif
    {
      return OK;
    }

  /* Stop endpoint */

  xhci_cmd_stopep(priv, epinfo->slot, xhci_epno_get(epinfo), false);

  /* REVISIT: what if we interrupted the execution of a TD? page 139 */

  epinfo->result = -ESHUTDOWN;

  if (iocwait)
    {
      /* Yes... wake it up */

      nxsem_post(&epinfo->iocsem);
    }

#ifdef CONFIG_USBHOST_ASYNCH
  /* No.. Is there a pending asynchronous transfer? */

  else
    {
      /* Yes.. perform the callback */

      DEBUGASSERT(callback != NULL);
      callback(arg, -ESHUTDOWN);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: xhci_connect
 *
 * Description:
 *   New connections may be detected by an attached hub.  This method is the
 *   mechanism that is used by the hub class to introduce a new connection
 *   and port description to the system.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   hport - The descriptor of the hub port that detected the connection
 *      related event
 *   connected - True: device connected; false: device disconnected
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_HUB
static int xhci_connect(FAR struct usbhost_driver_s *drvr,
                        FAR struct usbhost_hubport_s *hport,
                        bool connected)
{
#error missing logic
}
#endif

/****************************************************************************
 * Name: xhci_disconnect
 *
 * Description:
 *   Called by the class when an error occurs and device has been
 *   disconnected.  The USB host driver should discard the handle to the
 *   class instance (it is stale) and not attempt any further interaction
 *   with the class driver instance (until a new instance is received from
 *   the create() method).  The driver should not call the class
 *   disconnected() method.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   hport - The port from which the device is being disconnected.  Might be
 *      a port on a hub.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static void xhci_disconnect(FAR struct usbhost_driver_s *drvr,
                            FAR struct usbhost_hubport_s *hport)
{
  FAR struct usbhost_xhci_s *priv   = XHCI_PRIV_FROM_DRVR(drvr);
  FAR struct xhci_rhport_s  *rhport = (FAR struct xhci_rhport_s *)drvr;

  DEBUGASSERT(hport != NULL);
  hport->devclass = NULL;

  /* Deinit device slot */

  if (rhport->dev)
    {
      xhci_device_deinit(priv, rhport);
    }
}

/****************************************************************************
 * Name: xhci_hw_getparams
 *
 * Description:
 *   Get hardware description of a connected xHCI device.
 *
 ****************************************************************************/

static int xhci_hw_getparams(FAR struct usbhost_xhci_s *priv)
{
  uint32_t regval;

  /* Get data form Host Controller Capability 1 Parameters */

  regval = xhci_capa_getreg(priv, XHCI_HCCPARAMS1);
  if (regval & XHCI_HCCPARAMS1_CSZ)
    {
      pcierr("Only 32 byte Context data structures supported!\n");
      return -EIO;
    }

  /* Get data from Structural Parameters 1 register */

  regval = xhci_capa_getreg(priv, XHCI_HCSPARAMS1);
  priv->no_slots = XHCI_HCSPARAMS1_MAXSLOTS(regval);
  priv->no_ports = XHCI_HCSPARAMS1_MAXPORTS(regval);

  /* Limit number of slots to number of devices */

  if (priv->no_slots > CONFIG_USBHOST_XHCI_MAX_DEVS)
    {
      priv->no_slots = CONFIG_USBHOST_XHCI_MAX_DEVS;
    }

  pciinfo("no slots = %d, no ports = %d\n",
          priv->no_slots, priv->no_ports);

  /* Check if valid */

  if (priv->no_slots == 0 || priv->no_ports == 0)
    {
      return -EINVAL;
    }

  /* Get data from Structural Parameters 2 register */

  regval = xhci_capa_getreg(priv, XHCI_HCSPARAMS2);
  priv->no_scratch = XHCI_HCSPARAMS2_MAXSPB(regval);

  pciinfo("no scratch = %d\n", priv->no_scratch);

  priv->no_erst = 1 << XHCI_HCSPARAMS2_ERST(regval);

  pciinfo("no_erst = %d\n", priv->no_erst);

  /* Limit event ring segment table to 1 */

  if (priv->no_erst > XHCI_MAX_ERST)
    {
      priv->no_erst = XHCI_MAX_ERST;
    }

  pciinfo("no erst = %d\n", priv->no_erst);

  return OK;
}

/****************************************************************************
 * Name: xhci_irq_initialize
 *
 * Description:
 *   Initialize xHCI interrupts - require MSI-X support.
 *
 ****************************************************************************/

static int xhci_irq_initialize(FAR struct usbhost_xhci_s *priv)
{
  int ret;

  /* Allocate MSI */

  ret = pci_alloc_irq(priv->pcidev, &priv->irq, 1);
  if (ret != 1)
    {
      pcierr("Failed to allocate MSI %d\n", ret);
      return ret;
    }

  /* Attach IRQ */

  irq_attach(priv->irq, xhci_interrupt, priv);

  /* Connect MSI-X */

  ret = pci_connect_irq(priv->pcidev, &priv->irq, 1);
  if (ret != OK)
    {
      pcierr("Failed to connect MSI %d\n", ret);
      pci_release_irq(priv->pcidev, &priv->irq, 1);

      return -ENOTSUP;
    }

  up_enable_irq(priv->irq);

  return OK;
}

/****************************************************************************
 * Name: xhci_mem_alloc
 *
 * Description:
 *   Allocated memory for a new detected xHCI device.
 *
 ****************************************************************************/

static int xhci_mem_alloc(FAR struct usbhost_xhci_s *priv)
{
  size_t tmp;
  int    i;

  /* Allocate Scratchpad Buffer Array */

  tmp = priv->no_scratch * sizeof(uint64_t);
  priv->pg_sb = kmm_memalign(XHCI_BUF_ALIGN, tmp);
  if (!priv->pg_sb)
    {
      pcierr("pg_sb malloc failed\n");
      return -ENOMEM;
    }

  memset(priv->pg_sb, 0, tmp);

  for (i = 0; i < priv->no_scratch; i++)
    {
      /* Alloc page for each entry in array */

      priv->pg_sb[i] = up_addrenv_va_to_pa(
        kmm_memalign(XHCI_PAGE_SIZE, XHCI_PAGE_SIZE));
      if (!priv->pg_sb[i])
        {
          pcierr("pg_sb[i] malloc failed\n");
          return -ENOMEM;
        }

      /* Reset page */

      memset((FAR void *)(up_addrenv_pa_to_va(priv->pg_sb[i])),
             0, XHCI_PAGE_SIZE);
    }

  /* Allocate Device Context Array which shall be:
   *   size = MaxSlotsEn + 1 entries
   */

  tmp = sizeof(uint64_t) * (priv->no_slots + 1);
  priv->pg_ctx = kmm_memalign(XHCI_BUF_ALIGN, tmp);
  if (!priv->pg_ctx)
    {
      pcierr("pg_ctx malloc failed\n");
      return -ENOMEM;
    }

  /* Reset context */

  memset(priv->pg_ctx, 0, tmp);

  /* Allocate Event Table */

  tmp = sizeof(struct xhci_event_ring_s) * priv->no_erst;
  priv->pg_erst = kmm_memalign(XHCI_BUF_ALIGN, tmp);
  if (!priv->pg_erst)
    {
      pcierr("priv->pg_erst malloc failed\n");
      return -ENOMEM;
    }

  memset(priv->pg_erst, 0, tmp);

  /* Allocate root hub ports */

  priv->rhport = kmm_zalloc(priv->no_ports * sizeof(struct xhci_rhport_s));
  if (!priv->rhport)
    {
      pcierr("rhport zalloc failed!\n");
      return -ENOMEM;
    }

  /* Allocate xHC devices array */

  priv->devs = kmm_zalloc(priv->no_slots * sizeof(struct xhci_dev_s));
  if (!priv->devs)
    {
      pcierr("devs zalloc failed!\n");
      return -ENOMEM;
    }

  /* Allocate xHC devices resources */

  for (i = 0; i < priv->no_slots; i++)
    {
      /* Allocate Device Context */

      priv->devs[i].ctx = kmm_zalloc(sizeof(struct xhci_dev_ctx_s));
      if (!priv->devs[i].ctx)
        {
          pcierr("dev ctx zalloc failed!\n");
          return -ENOMEM;
        }

      /* Allocate Input Context. The Input Context shall be physically
       * contiguous within a page
       */

      priv->devs[i].input = kmm_memalign((XHCI_PAGE_SIZE / 2),
                            sizeof(struct xhci_input_dev_ctx_s));
      if (!priv->devs[i].input)
        {
          pcierr("dev input zalloc failed!\n");
          return -ENOMEM;
        }

      /* No endpoint for device yet */

      tmp = sizeof(uintptr_t) * XHCI_MAX_ENDPOINTS;
      memset(priv->devs[i].epinfo, 0, tmp);
    }

  return OK;
}

/****************************************************************************
 * Name: xhci_mem_free
 *
 * Description:
 *   Free allocated memory for a xHCI device.
 *
 ****************************************************************************/

static int xhci_mem_free(FAR struct usbhost_xhci_s *priv)
{
  int i;

  /* Free scratch buffers */

  for (i = 0; i < priv->no_scratch; i++)
    {
      kmm_free((FAR void *)priv->pg_sb[i]);
    }

  kmm_free(priv->pg_sb);

  /* Free devices */

  for (i = 0; i < priv->no_slots; i++)
    {
      kmm_free(priv->devs[i].ctx);
      kmm_free(priv->devs[i].input);
    }

  kmm_free(priv->devs);
  kmm_free(priv->pg_ctx);
  kmm_free(priv->pg_erst);
  kmm_free(priv->rhport);

  /* Free command ring and event ring */

  xhci_ring_deinit(&priv->cmd);
  xhci_ring_deinit(&priv->evnt);

  return OK;
}

/****************************************************************************
 * Name: xhci_hw_initialize
 *
 * Description:
 *   One-time setup of the host controller hardware for normal operations.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int xhci_hw_initialize(FAR struct usbhost_xhci_s *priv)
{
  int ret;

  /* Synchronize with BIOS */

  ret = xhci_bios_wait(priv);
  if (ret < 0)
    {
      pcierr("Failed to get xhci controller!\n");
      goto errout;
    }

  /* Get structural parameters */

  ret = xhci_hw_getparams(priv);
  if (ret < 0)
    {
      goto errout;
    }

  /* Allocate all required memory */

  ret = xhci_mem_alloc(priv);
  if (ret < 0)
    {
      goto errout;
    }

  /* Configure interrupts */

  ret = xhci_irq_initialize(priv);
  if (ret < 0)
    {
      goto errout;
    }

  /* Halt controller */

  ret = xhci_ctrl_halt(priv);
  if (ret < 0)
    {
      goto errout;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: xhci_sw_initialize
 *
 * Description:
 *   One-time setup of the host driver state structure.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline int xhci_sw_initialize(FAR struct usbhost_xhci_s *priv)
{
  FAR struct xhci_rhport_s     *rhport;
  FAR struct usbhost_hubport_s *hport;
  int                           i;

  /* Initialize sync objects */

  nxmutex_init(&priv->lock);
  nxsem_init(&priv->pscsem, 0, 0);
  nxsem_init(&priv->cmdsem, 0, 0);

  /* Initialize function address generation logic
   * REVISIT: xHCI hardware is responsible for device address, but NuttX USB
   *          Host stack require this to be initialized.
   */

  usbhost_devaddr_initialize(&priv->devgen);

  /* Initialize devices */

  for (i = 0; i < priv->no_slots; i++)
    {
      /* Slot disabled by defaulte */

      priv->devs[i].state = XHCI_SLOT_DISABLED;
    }

  /* Initialize the root hub port structures */

  for (i = 0; i < priv->no_ports; i++)
    {
      rhport = &priv->rhport[i];

      /* No device slot yet */

      rhport->dev = NULL;

      /* Connect xhci instance */

      rhport->priv = priv;

      /* Initialize the device operations */

      rhport->drvr.ep0configure   = xhci_ep0configure;
      rhport->drvr.epalloc        = xhci_epalloc;
      rhport->drvr.epfree         = xhci_epfree;
      rhport->drvr.alloc          = xhci_alloc;
      rhport->drvr.free           = xhci_free;
      rhport->drvr.ioalloc        = xhci_ioalloc;
      rhport->drvr.iofree         = xhci_iofree;
      rhport->drvr.ctrlin         = xhci_ctrlin;
      rhport->drvr.ctrlout        = xhci_ctrlout;
      rhport->drvr.transfer       = xhci_transfer;
#ifdef CONFIG_USBHOST_ASYNCH
      rhport->drvr.asynch         = xhci_asynch;
#endif
      rhport->drvr.cancel         = xhci_cancel;
#ifdef CONFIG_USBHOST_HUB
      rhport->drvr.connect        = xhci_connect;
#endif
      rhport->drvr.disconnect     = xhci_disconnect;
      rhport->hport.pdevgen       = &priv->devgen;

      /* Initialize EP0 */

      rhport->ep0.xfrtype         = USB_EP_ATTR_XFER_CONTROL;
      rhport->ep0.epno            = 0;
      rhport->ep0.devaddr         = 0;
      nxsem_init(&rhport->ep0.iocsem, 0, 0);

      /* Initialize the public port representation */

      hport                       = &rhport->hport.hport;
      hport->drvr                 = &rhport->drvr;
#ifdef CONFIG_USBHOST_HUB
      hport->parent               = NULL;
#endif
      hport->ep0                  = &rhport->ep0;
      hport->port                 = i;
      hport->speed                = USB_SPEED_FULL;
    }

  return OK;
}

/****************************************************************************
 * Name: pci_xhci_probe
 *
 * Description:
 *   Initialize PCI device.
 *
 ****************************************************************************/

static int pci_xhci_probe(FAR struct pci_device_s *dev)
{
  FAR struct usbhost_conn_xhci_s *conn = NULL;
  FAR struct usbhost_xhci_s      *priv = NULL;
  int                             ret  = -ENOMEM;

  /* Init PCI bus */

  pci_set_master(dev);
  pciinfo("Enabled bus mastering\n");
  pci_enable_device(dev);
  pciinfo("Enabled memory resources\n");

  /* Allocate connection structure */

  conn = kmm_zalloc(sizeof(struct usbhost_conn_xhci_s));
  if (!conn)
    {
      pcierr("zalloc failed!\n");
      goto errout;
    }

  /* Allocate the driver structure */

  priv = kmm_zalloc(sizeof(struct usbhost_xhci_s));
  if (!priv)
    {
      pcierr("zalloc failed!\n");
      goto errout;
    }

  /* Initialize connection data */

  conn->conn.wait      = xhci_wait;
  conn->conn.enumerate = xhci_enumerate;
  conn->priv           = priv;

  /* Connect PCI handler */

  priv->pcidev = dev;
  dev->priv    = conn;

  /* Get base address - BAR 0 */

  priv->base = (uintptr_t)pci_map_bar(dev, 0);
  if (!priv->base)
    {
      pcierr("Not found BAR 0!\n");
      ret = -EIO;
      goto errout;
    }

  /* Get register address */

  priv->capa_base = priv->base;
  priv->oper_base = priv->base + xhci_capa_getreg_1b(priv, XHCI_CAPLENGTH);
  priv->runt_base = priv->base + xhci_capa_getreg(priv, XHCI_RTSOFF);
  priv->door_base = priv->base + xhci_capa_getreg(priv, XHCI_DBOFF);

  usbhost_vtrace1(XHCI_VTRACE1_INITIALIZING, 0);

  /* Initialize HW */

  ret = xhci_hw_initialize(priv);
  if (ret < 0)
    {
      pcierr("failed to initialize HW!\n");
      goto errout;
    }

  /* Initialize SW */

  ret = xhci_sw_initialize(priv);
  if (ret < 0)
    {
      pcierr("failed to initialize SW!\n");
      goto errout;
    }

  /* Start controller */

  ret = xhci_ctrl_start(priv);
  if (ret < 0)
    {
      usbhost_trace1(XHCI_TRACE1_START_FAILED, 0);
      goto errout;
    }

#ifdef CONFIG_DEBUG_USB_INFO
  /* Dump xhci registers */

  xhci_dump_mem(priv, "after init");
#endif

  /* If there is a USB device in the slot at power up, then we will not
   * get the status change interrupt to signal us that the device is
   * connected.  We need to set the initial connected state accordingly.
   */

  xhci_probe_ports(priv);

  usbhost_vtrace1(XHCI_VTRACE1_INITIALIZING, 0);

  /* Initialize waiter */

  ret = usbhost_waiter_initialize(&conn->conn);
  if (ret < 0)
    {
      pcierr("failed to initialize waiter!\n");
      goto errout;
    }

  /* Store waiter PID */

  conn->pid = ret;

  return OK;

errout:

  /* Free allocated xhci buffers */

  xhci_mem_free(priv);

  /* Free allocated data */

  kmm_free(conn);
  kmm_free(priv);

  return ret;
}

/****************************************************************************
 * Name: pci_xhci_remove
 *
 * Description:
 *   Remove PCI device.
 *
 ****************************************************************************/

static void pci_xhci_remove(FAR struct pci_device_s *dev)
{
  FAR struct usbhost_conn_xhci_s *conn = dev->priv;
  FAR struct usbhost_xhci_s      *priv = conn->priv;

  /* Free xhci interrupts */

  irq_detach(priv->irq);
  pci_release_irq(dev, &priv->irq, 1);

  /* Disable PCI devicve */

  pci_clear_master(dev);
  pci_disable_device(dev);

  /* Delete waiter thread */

  kthread_delete(conn->pid);

  /* Free xhci buffers */

  xhci_mem_free(priv);

  /* Free driver data */

  kmm_free(conn);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pci_xhci_init
 *
 * Description:
 *   Initialize the USB host xHCI as PCI device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

int pci_xhci_init(void)
{
  return pci_register_driver(&g_pci_xhci_drv);
}
