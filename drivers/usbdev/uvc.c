/****************************************************************************
 * drivers/usbdev/uvc.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * USB Video Class (UVC) 1.1 Gadget Driver for NuttX
 * Bulk transport, uncompressed YUY2, QVGA (320x240)
 *
 * Architecture: modeled after drivers/usbdev/adb.c (usbdev_fs pattern)
 * but with custom class-specific setup handling for UVC PROBE/COMMIT.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/usb/uvc.h>
#ifdef CONFIG_USBDEV_COMPOSITE
#  include <nuttx/usb/composite.h>
#endif
#include <nuttx/fs/fs.h>
#include <nuttx/wqueue.h>
#include <poll.h>
#include <nuttx/mutex.h>
#include <nuttx/lib/lib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define USBUVC_CHARDEV_PATH       "/dev/uvc0"

#ifdef CONFIG_USBDEV_SELFPOWERED
#  define USBUVC_SELFPOWERED      USB_CONFIG_ATTR_SELFPOWER
#else
#  define USBUVC_SELFPOWERED      (0)
#endif

#ifdef CONFIG_USBDEV_REMOTEWAKEUP
#  define USBUVC_REMOTEWAKEUP     USB_CONFIG_ATTR_WAKEUP
#else
#  define USBUVC_REMOTEWAKEUP     (0)
#endif

#define USBUVC_MXDESCLEN          (64)
#define USBUVC_MAXSTRLEN          (USBUVC_MXDESCLEN - 2)

#define USBUVC_VERSIONNO          (0x0100)
#define USBUVC_STR_LANGUAGE       (0x0409)

/* String descriptor IDs — standalone mode uses absolute IDs;
 * composite mode uses relative IDs offset by devinfo.strbase.
 */

#ifndef CONFIG_USBUVC_COMPOSITE
#  define USBUVC_MANUFACTURERSTRID  (1)
#  define USBUVC_PRODUCTSTRID       (2)
#  define USBUVC_SERIALSTRID        (3)
#  define USBUVC_CONFIGSTRID        (4)
#  define USBUVC_VCIFSTRID          (5)
#  define USBUVC_VSIFSTRID          (6)
#  define USBUVC_NSTDSTRIDS         (6)  /* Total standalone string IDs */
#endif

/* In composite mode, we only contribute 2 interface strings.
 * The composite framework assigns strbase; our strings are
 * strbase+1 (VC) and strbase+2 (VS).
 */

#define USBUVC_VCIFSTRID_REL      (1)   /* Relative: VC interface string */
#define USBUVC_VSIFSTRID_REL      (2)   /* Relative: VS interface string */

/* Video parameters — defaults when no runtime params supplied */

#define USBUVC_DEF_WIDTH          320
#define USBUVC_DEF_HEIGHT         240
#define USBUVC_DEF_FPS            5
#define USBUVC_BPP                2       /* YUY2 = 2 bytes/pixel */

/* Number of poll waiters */

#ifndef CONFIG_USBUVC_NPOLLWAITERS
#  define CONFIG_USBUVC_NPOLLWAITERS 2
#endif

/* Number of write requests */

#ifndef CONFIG_USBUVC_NWRREQS
#  define CONFIG_USBUVC_NWRREQS   4
#endif

/* Bulk IN EP max packet size (Full-Speed) */

#ifndef CONFIG_USBUVC_EPBULKIN_FSSIZE
#  define CONFIG_USBUVC_EPBULKIN_FSSIZE 64
#endif

#ifndef CONFIG_USBUVC_EP0MAXPACKET
#  define CONFIG_USBUVC_EP0MAXPACKET    64
#endif

/* UVC descriptor unit/terminal IDs */

#define USBUVC_IT_ID              1   /* Input Terminal (Camera) */
#define USBUVC_OT_ID              2   /* Output Terminal (USB Streaming) */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Container for write requests — flink must be first for sq_entry_t cast */

struct uvc_wrreq_s
{
  FAR struct uvc_wrreq_s  *flink;    /* Singly linked list link */
  FAR struct usbdev_req_s *req;      /* The contained USB request */
};

struct uvc_dev_s
{
  FAR struct usbdev_s     *usbdev;
  FAR struct usbdev_ep_s  *epbulkin;
  FAR struct usbdev_req_s *ctrlreq;
  FAR struct usbdev_ep_s  *ep0;

  struct uvc_streaming_control_s probe;
  struct uvc_streaming_control_s commit;
  struct usbdev_devinfo_s  devinfo;

  /* Runtime video parameters from application */

  uint16_t                 width;
  uint16_t                 height;
  uint8_t                  fps;
  uint32_t                 frame_size;
  uint32_t                 frame_interval; /* in 100ns units */
  uint32_t                 bitrate;

  mutex_t                  lock;
  sem_t                    wrsem;     /* Signaled when wrreq available */
  bool                     streaming;
  FAR struct pollfd        *fds[CONFIG_USBUVC_NPOLLWAITERS];
  uint8_t                  fid;       /* Frame ID toggle bit */
  uint8_t                  config;

  /* Write request pool */

  struct sq_queue_s        wrfree;    /* Available write request containers */
  struct uvc_wrreq_s       wrreqs[CONFIG_USBUVC_NWRREQS];
  int                      nwralloc;
};

/* Forward declarations */

/* EP0 submit helper — in composite mode, route through composite layer */

#ifdef CONFIG_USBUVC_COMPOSITE
#  define UVC_EP0SUBMIT(driver, dev, ctrlreq, ctrl) \
     composite_ep0submit(driver, dev, ctrlreq, ctrl)
#else
#  define UVC_EP0SUBMIT(driver, dev, ctrlreq, ctrl) \
     EP_SUBMIT(dev->ep0, ctrlreq)
#endif

static int  uvc_bind(FAR struct usbdevclass_driver_s *driver,
                     FAR struct usbdev_s *dev);
static void uvc_unbind(FAR struct usbdevclass_driver_s *driver,
                       FAR struct usbdev_s *dev);
static int  uvc_setup(FAR struct usbdevclass_driver_s *driver,
                      FAR struct usbdev_s *dev,
                      FAR const struct usb_ctrlreq_s *ctrl,
                      FAR uint8_t *dataout, size_t outlen);
static void uvc_disconnect(FAR struct usbdevclass_driver_s *driver,
                           FAR struct usbdev_s *dev);
static void uvc_streaming_stop(FAR struct uvc_dev_s *priv);

static const struct usbdevclass_driverops_s g_uvc_driverops =
{
  .bind       = uvc_bind,
  .unbind     = uvc_unbind,
  .setup      = uvc_setup,
  .disconnect = uvc_disconnect,
};

/* File operations for /dev/uvc0 */

static ssize_t uvc_write(FAR struct file *filep,
                         FAR const char *buffer, size_t buflen);
static int     uvc_open(FAR struct file *filep);
static int     uvc_close(FAR struct file *filep);
static int     uvc_poll(FAR struct file *filep,
                        FAR struct pollfd *fds, bool setup);

static const struct file_operations g_uvc_fops =
{
  .open  = uvc_open,
  .close = uvc_close,
  .write = uvc_write,
  .poll  = uvc_poll,
};

/****************************************************************************
 * Private Data — USB Descriptors
 ****************************************************************************/

#ifndef CONFIG_USBUVC_COMPOSITE

/* Device Descriptor — standalone mode only */

static const struct usb_devdesc_s g_uvc_devdesc =
{
  .len          = USB_SIZEOF_DEVDESC,
  .type         = USB_DESC_TYPE_DEVICE,
  .usb          =
  {
    LSBYTE(0x0200), MSBYTE(0x0200)
  },
  .classid      = USB_CLASS_MISC,  /* 0xEF - IAD device */
  .subclass     = 0x02,            /* Common Class */
  .protocol     = 0x01,            /* IAD */
  .mxpacketsize = CONFIG_USBUVC_EP0MAXPACKET,
  .vendor       =
  {
    LSBYTE(0x1d6b), MSBYTE(0x1d6b)  /* Linux Foundation */
  },
  .product      =
  {
    LSBYTE(0x0102), MSBYTE(0x0102)  /* Webcam gadget */
  },
  .device       =
  {
    LSBYTE(USBUVC_VERSIONNO),
    MSBYTE(USBUVC_VERSIONNO)
  },
  .imfgr        = USBUVC_MANUFACTURERSTRID,
  .iproduct     = USBUVC_PRODUCTSTRID,
  .serno        = USBUVC_SERIALSTRID,
  .nconfigs     = USBUVC_NCONFIGS,
};

static const struct usb_cfgdesc_s g_uvc_cfgdesc =
{
  .len       = USB_SIZEOF_CFGDESC,
  .type      = USB_DESC_TYPE_CONFIG,
  .ninterfaces = USBUVC_NINTERFACES,
  .cfgvalue  = 1,
  .icfg      = USBUVC_CONFIGSTRID,
  .attr      = USB_CONFIG_ATTR_ONE | USBUVC_SELFPOWERED |
               USBUVC_REMOTEWAKEUP,
  .mxpower   = (CONFIG_USBDEV_MAXPOWER + 1) / 2,
};

#endif /* !CONFIG_USBUVC_COMPOSITE */

/* Interface Association Descriptor (IAD) for UVC
 * Note: bFirstInterface and iFunction are patched at runtime.
 */

static const uint8_t g_uvc_iad[] =
{
  0x08,                               /* bLength */
  USB_DESC_TYPE_INTERFACEASSOCIATION, /* bDescriptorType */
  0x00,                               /* bFirstInterface: patched */
  0x02,                               /* bInterfaceCount (VC + VS) */
  USB_CLASS_VIDEO,                    /* bFunctionClass */
  UVC_SC_VIDEO_INTERFACE_COLLECTION,  /* bFunctionSubClass */
  UVC_PC_PROTOCOL_UNDEFINED,          /* bFunctionProtocol */
  0x00,                               /* iFunction: patched */
};

/* VideoControl Interface Descriptor
 * Note: ifno and iif are patched at runtime.
 */

static const struct usb_ifdesc_s g_uvc_vc_ifdesc =
{
  .len      = USB_SIZEOF_IFDESC,
  .type     = USB_DESC_TYPE_INTERFACE,
  .ifno     = 0,                      /* patched: ifnobase */
  .alt      = 0,
  .neps     = 0,                      /* No interrupt EP for now */
  .classid  = USB_CLASS_VIDEO,
  .subclass = UVC_SC_VIDEOCONTROL,
  .protocol = UVC_PC_PROTOCOL_UNDEFINED,
  .iif      = 0,                      /* patched: strbase + VCIFSTRID_REL */
};

/* VC Header Descriptor (12 + 1*n bytes, n=1 VS interface)
 * Note: baInterfaceNr is patched at runtime to ifnobase + 1.
 */

static const uint8_t g_uvc_vc_header[] =
{
  0x0d,                               /* bLength (13) */
  UVC_CS_INTERFACE,                   /* bDescriptorType */
  UVC_VC_HEADER,                      /* bDescriptorSubtype */
  LSBYTE(0x0110),                     /* bcdUVC: 1.10 */
  MSBYTE(0x0110),
  LSBYTE(0x0042),                     /* wTotalLength: will be patched */
  MSBYTE(0x0042),
  (0x005b8d80 >>  0) & 0xff,          /* dwClockFrequency: 6MHz */
  (0x005b8d80 >>  8) & 0xff,
  (0x005b8d80 >> 16) & 0xff,
  (0x005b8d80 >> 24) & 0xff,
  0x01,                               /* bInCollection: 1 VS interface */
  0x00,                               /* baInterfaceNr(1): patched */
};

/* Input Terminal Descriptor (Camera) — 17 bytes */

static const uint8_t g_uvc_input_terminal[] =
{
  0x11,                               /* bLength (17) */
  UVC_CS_INTERFACE,                   /* bDescriptorType */
  UVC_VC_INPUT_TERMINAL,              /* bDescriptorSubtype */
  USBUVC_IT_ID,                       /* bTerminalID */
  LSBYTE(UVC_ITT_CAMERA),             /* wTerminalType */
  MSBYTE(UVC_ITT_CAMERA),
  0x00,                               /* bAssocTerminal */
  0x00,                               /* iTerminal */
  0x00, 0x00,                         /* wObjectiveFocalLengthMin */
  0x00, 0x00,                         /* wObjectiveFocalLengthMax */
  0x00, 0x00,                         /* wOcularFocalLength */
  0x02,                               /* bControlSize */
  0x00, 0x00,                         /* bmControls (none) */
};

/* Output Terminal Descriptor — 9 bytes */

static const uint8_t g_uvc_output_terminal[] =
{
  0x09,                               /* bLength */
  UVC_CS_INTERFACE,                   /* bDescriptorType */
  UVC_VC_OUTPUT_TERMINAL,             /* bDescriptorSubtype */
  USBUVC_OT_ID,                       /* bTerminalID */
  LSBYTE(UVC_TT_STREAMING),           /* wTerminalType */
  MSBYTE(UVC_TT_STREAMING),
  0x00,                               /* bAssocTerminal */
  USBUVC_IT_ID,                       /* bSourceID */
  0x00,                               /* iTerminal */
};

/* VideoStreaming Interface Descriptor
 * Note: ifno and iif are patched at runtime.
 */

static const struct usb_ifdesc_s g_uvc_vs_ifdesc =
{
  .len      = USB_SIZEOF_IFDESC,
  .type     = USB_DESC_TYPE_INTERFACE,
  .ifno     = 0,                      /* patched: ifnobase + 1 */
  .alt      = 0,
  .neps     = 1,                      /* 1 Bulk IN EP */
  .classid  = USB_CLASS_VIDEO,
  .subclass = UVC_SC_VIDEOSTREAMING,
  .protocol = UVC_PC_PROTOCOL_UNDEFINED,
  .iif      = 0,                      /* patched: strbase + VSIFSTRID_REL */
};

/* VS Input Header — 14 bytes (1 format, bControlSize=1) */

static const uint8_t g_uvc_vs_input_header[] =
{
  0x0e,                               /* bLength (14) */
  UVC_CS_INTERFACE,                   /* bDescriptorType */
  UVC_VS_INPUT_HEADER,                /* bDescriptorSubtype */
  0x01,                               /* bNumFormats: 1 */
  0x00, 0x00,                         /* wTotalLength: patched later */
  0x00,                               /* bEndpointAddress: patched later */
  0x00,                               /* bmInfo: no dynamic format change */
  USBUVC_OT_ID,                       /* bTerminalLink */
  0x00,                               /* bStillCaptureMethod: none */
  0x00,                               /* bTriggerSupport */
  0x00,                               /* bTriggerUsage */
  0x01,                               /* bControlSize: 1 byte */
  0x00,                               /* bmaControls(1): no controls */
};

/* VS Uncompressed Format Descriptor — 27 bytes */

static const uint8_t g_uvc_vs_format_uncomp[] =
{
  0x1b,                               /* bLength (27) */
  UVC_CS_INTERFACE,                   /* bDescriptorType */
  UVC_VS_FORMAT_UNCOMPRESSED,         /* bDescriptorSubtype */
  0x01,                               /* bFormatIndex: 1 */
  0x01,                               /* bNumFrameDescriptors: 1 */

  /* guidFormat: YUY2 */

  'Y',  'U',  'Y',  '2',  0x00, 0x00, 0x10, 0x00,
  0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71,

  0x10,                               /* bBitsPerPixel: 16 */
  0x01,                               /* bDefaultFrameIndex: 1 */
  0x00,                               /* bAspectRatioX */
  0x00,                               /* bAspectRatioY */
  0x00,                               /* bmInterlaceFlags */
  0x00,                               /* bCopyProtect */
};

/* VS Uncompressed Frame Descriptor length - 30 bytes (1 discrete interval) */

#define USBUVC_VS_FRAME_DESC_LEN  30

/* VS Color Matching Descriptor — 6 bytes */

static const uint8_t g_uvc_vs_color[] =
{
  0x06,                               /* bLength */
  UVC_CS_INTERFACE,                   /* bDescriptorType */
  UVC_VS_COLOR_FORMAT,                /* bDescriptorSubtype */
  0x01,                               /* bColorPrimaries: BT.709 */
  0x01,                               /* bTransferCharacteristics: BT.709 */
  0x04,                               /* bMatrixCoefficients: SMPTE 170M */
};

/* Bulk IN Endpoint info */

static const struct usbdev_epinfo_s g_uvc_epbulkin =
{
  .desc =
    {
      .len       = USB_SIZEOF_EPDESC,
      .type      = USB_DESC_TYPE_ENDPOINT,
      .addr      = USB_DIR_IN,
      .attr      = USB_EP_ATTR_XFER_BULK |
                   USB_EP_ATTR_NO_SYNC   |
                   USB_EP_ATTR_USAGE_DATA,
      .interval  = 0,
    },
  .reqnum        = CONFIG_USBUVC_NWRREQS,
  .fssize        = CONFIG_USBUVC_EPBULKIN_FSSIZE,
};

/* Endpoint info array for composite framework */

#ifdef CONFIG_USBDEV_COMPOSITE
static FAR const struct usbdev_epinfo_s *g_uvc_epinfos[USBUVC_NUM_EPS] =
{
  &g_uvc_epbulkin,
};
#endif

/* Global device instance (singleton) */

static struct uvc_dev_s *g_uvc_dev;

/****************************************************************************
 * Private Functions — Helpers
 ****************************************************************************/

static void uvc_default_streaming_ctrl(
    FAR struct uvc_dev_s *priv,
    FAR struct uvc_streaming_control_s *ctrl)
{
  memset(ctrl, 0, sizeof(*ctrl));
  ctrl->bmhint                   = 1;
  ctrl->bformatindex             = 1;
  ctrl->bframeindex              = 1;
  ctrl->dwframeinterval          = priv->frame_interval;
  ctrl->dwmaxvideoframesize      = priv->frame_size;
  ctrl->dwmaxpayloadtransfersize = priv->frame_size +
                                    UVC_PAYLOAD_HEADER_LEN;
  ctrl->dwclockfrequency         = 6000000;  /* 6 MHz */
  ctrl->bmframinginfo            = 0x03;     /* FID + EOF required */
  ctrl->bpreferedversion         = 1;
  ctrl->bminversion              = 1;
  ctrl->bmaxversion              = 1;
}

static void uvc_ep0incomplete(FAR struct usbdev_ep_s *ep,
                              FAR struct usbdev_req_s *req)
{
  if (req->result || req->xfrd != req->len)
    {
      uerr("EP0 complete: result=%d xfrd=%d len=%d\n",
           req->result, req->xfrd, req->len);
    }
}

static void uvc_wrreq_callback(FAR struct usbdev_ep_s *ep,
                                FAR struct usbdev_req_s *req)
{
  FAR struct uvc_dev_s *priv = g_uvc_dev;

  if (priv)
    {
      FAR struct uvc_wrreq_s *wrcontainer;
      irqstate_t flags;

      if (req->result != OK)
        {
          uerr("UVC bulk IN xfer failed: %d\n", req->result);
        }

      wrcontainer = (FAR struct uvc_wrreq_s *)req->priv;
      flags = enter_critical_section();
      sq_addlast((FAR sq_entry_t *)wrcontainer, &priv->wrfree);
      leave_critical_section(flags);

      nxsem_post(&priv->wrsem);
    }
}

/****************************************************************************
 * Private Functions — Configuration Descriptor Builder
 ****************************************************************************/

static int16_t uvc_mkcfgdesc(FAR uint8_t *buf,
                              FAR struct usbdev_devinfo_s *devinfo,
                              uint8_t speed, uint8_t type)
{
  FAR struct uvc_dev_s *priv = g_uvc_dev;
  FAR uint8_t *p = buf;
  uint16_t vs_total;
  uint16_t vc_total;
  int16_t totallen;
  uint8_t vc_ifno;
  uint8_t vs_ifno;
  uint8_t vc_strid;
  uint8_t vs_strid;

  UNUSED(speed);
  UNUSED(type);

  /* Compute interface numbers and string IDs from devinfo */

  vc_ifno  = devinfo->ifnobase;
  vs_ifno  = devinfo->ifnobase + 1;
  vc_strid = devinfo->strbase + USBUVC_VCIFSTRID_REL;
  vs_strid = devinfo->strbase + USBUVC_VSIFSTRID_REL;

  /* Calculate total descriptor size (without config header for composite) */

  totallen = (int16_t)(sizeof(g_uvc_iad) +
             sizeof(g_uvc_vc_ifdesc) +
             sizeof(g_uvc_vc_header) +
             sizeof(g_uvc_input_terminal) +
             sizeof(g_uvc_output_terminal) +
             sizeof(g_uvc_vs_ifdesc) +
             sizeof(g_uvc_vs_input_header) +
             sizeof(g_uvc_vs_format_uncomp) +
             USBUVC_VS_FRAME_DESC_LEN +
             sizeof(g_uvc_vs_color) +
             USB_SIZEOF_EPDESC);

#ifndef CONFIG_USBUVC_COMPOSITE
  totallen += sizeof(g_uvc_cfgdesc);
#endif

  if (!buf)
    {
      return totallen;
    }

#ifndef CONFIG_USBUVC_COMPOSITE
  /* Configuration descriptor header — standalone only */

  {
    FAR struct usb_cfgdesc_s *dest = (FAR struct usb_cfgdesc_s *)p;
    memcpy(p, &g_uvc_cfgdesc, sizeof(g_uvc_cfgdesc));
    dest->type        = type;
    dest->totallen[0] = LSBYTE(totallen);
    dest->totallen[1] = MSBYTE(totallen);
  }

  p += sizeof(g_uvc_cfgdesc);
#endif

  /* IAD — patch bFirstInterface and iFunction */

  memcpy(p, g_uvc_iad, sizeof(g_uvc_iad));
  p[2] = vc_ifno;                     /* bFirstInterface */
  p[7] = vc_strid;                    /* iFunction */
  p += sizeof(g_uvc_iad);

  /* VC Interface — patch ifno and iif */

  memcpy(p, &g_uvc_vc_ifdesc, sizeof(g_uvc_vc_ifdesc));
  ((FAR struct usb_ifdesc_s *)p)->ifno = vc_ifno;
  ((FAR struct usb_ifdesc_s *)p)->iif  = vc_strid;
  p += sizeof(g_uvc_vc_ifdesc);

  /* VC Header — patch wTotalLength and baInterfaceNr */

  vc_total = sizeof(g_uvc_vc_header) +
             sizeof(g_uvc_input_terminal) +
             sizeof(g_uvc_output_terminal);

  memcpy(p, g_uvc_vc_header, sizeof(g_uvc_vc_header));
  p[5]  = LSBYTE(vc_total);
  p[6]  = MSBYTE(vc_total);
  p[12] = vs_ifno;                    /* baInterfaceNr(1) */
  p += sizeof(g_uvc_vc_header);

  /* Input Terminal */

  memcpy(p, g_uvc_input_terminal, sizeof(g_uvc_input_terminal));
  p += sizeof(g_uvc_input_terminal);

  /* Output Terminal */

  memcpy(p, g_uvc_output_terminal, sizeof(g_uvc_output_terminal));
  p += sizeof(g_uvc_output_terminal);

  /* VS Interface — patch ifno and iif */

  memcpy(p, &g_uvc_vs_ifdesc, sizeof(g_uvc_vs_ifdesc));
  ((FAR struct usb_ifdesc_s *)p)->ifno = vs_ifno;
  ((FAR struct usb_ifdesc_s *)p)->iif  = vs_strid;
  p += sizeof(g_uvc_vs_ifdesc);

  /* VS Input Header — patch wTotalLength and bEndpointAddress */

  vs_total = sizeof(g_uvc_vs_input_header) +
             sizeof(g_uvc_vs_format_uncomp) +
             USBUVC_VS_FRAME_DESC_LEN +
             sizeof(g_uvc_vs_color);

  memcpy(p, g_uvc_vs_input_header, sizeof(g_uvc_vs_input_header));
  p[4] = LSBYTE(vs_total);
  p[5] = MSBYTE(vs_total);
  p[6] = USB_EPIN(devinfo->epno[USBUVC_EP_BULKIN_IDX]);
  p += sizeof(g_uvc_vs_input_header);

  /* VS Format */

  memcpy(p, g_uvc_vs_format_uncomp, sizeof(g_uvc_vs_format_uncomp));
  p += sizeof(g_uvc_vs_format_uncomp);

  /* VS Frame - built at runtime from video params */

  /* bLength (30) */

  p[0]  = USBUVC_VS_FRAME_DESC_LEN;

  /* bDescriptorType */

  p[1]  = UVC_CS_INTERFACE;

  /* bDescriptorSubtype */

  p[2]  = UVC_VS_FRAME_UNCOMPRESSED;

  /* bFrameIndex: 1 */

  p[3]  = 0x01;

  /* bmCapabilities */

  p[4]  = 0x00;

  /* wWidth */

  p[5]  = LSBYTE(priv->width);
  p[6]  = MSBYTE(priv->width);

  /* Height (2 bytes LE) */

  p[7]  = LSBYTE(priv->height);
  p[8]  = MSBYTE(priv->height);

  /* dwMinBitRate */

  p[9]  = (priv->bitrate >>  0) & 0xff;
  p[10] = (priv->bitrate >>  8) & 0xff;
  p[11] = (priv->bitrate >> 16) & 0xff;
  p[12] = (priv->bitrate >> 24) & 0xff;

  /* dwMaxBitRate */

  p[13] = (priv->bitrate >>  0) & 0xff;
  p[14] = (priv->bitrate >>  8) & 0xff;
  p[15] = (priv->bitrate >> 16) & 0xff;
  p[16] = (priv->bitrate >> 24) & 0xff;

  /* dwMaxVideoFrameBufferSize */

  p[17] = (priv->frame_size >>  0) & 0xff;
  p[18] = (priv->frame_size >>  8) & 0xff;
  p[19] = (priv->frame_size >> 16) & 0xff;
  p[20] = (priv->frame_size >> 24) & 0xff;

  /* dwDefaultFrameInterval */

  p[21] = (priv->frame_interval >>  0) & 0xff;
  p[22] = (priv->frame_interval >>  8) & 0xff;
  p[23] = (priv->frame_interval >> 16) & 0xff;
  p[24] = (priv->frame_interval >> 24) & 0xff;

  /* bFrameIntervalType: 1 discrete */

  p[25] = 0x01;

  /* dwFrameInterval(1) */

  p[26] = (priv->frame_interval >>  0) & 0xff;
  p[27] = (priv->frame_interval >>  8) & 0xff;
  p[28] = (priv->frame_interval >> 16) & 0xff;
  p[29] = (priv->frame_interval >> 24) & 0xff;
  p += USBUVC_VS_FRAME_DESC_LEN;

  /* VS Color Matching */

  memcpy(p, g_uvc_vs_color, sizeof(g_uvc_vs_color));
  p += sizeof(g_uvc_vs_color);

  /* Bulk IN EP descriptor */

  usbdev_copy_epdesc((FAR struct usb_epdesc_s *)p,
                     devinfo->epno[USBUVC_EP_BULKIN_IDX],
                     speed, &g_uvc_epbulkin);
  p += USB_SIZEOF_EPDESC;

  return (int16_t)(p - buf);
}

/****************************************************************************
 * Private Functions — String Descriptor
 ****************************************************************************/

static int uvc_mkstrdesc(uint8_t id, FAR struct usb_strdesc_s *strdesc)
{
  FAR uint8_t *data = (FAR uint8_t *)(strdesc + 1);
  FAR const char *str;
  int len;
  int ndata;
  int i;

  switch (id)
    {
#ifndef CONFIG_USBUVC_COMPOSITE
      case 0:
        {
          /* Descriptor 0 is the language id */

          strdesc->len  = 4;
          strdesc->type = USB_DESC_TYPE_STRING;
          data[0] = LSBYTE(USBUVC_STR_LANGUAGE);
          data[1] = MSBYTE(USBUVC_STR_LANGUAGE);
          return 4;
        }

      case USBUVC_MANUFACTURERSTRID:
        str = "NuttX";
        break;
      case USBUVC_PRODUCTSTRID:
        str = "NuttX UVC Camera";
        break;
      case USBUVC_SERIALSTRID:
        str = "0001";
        break;
      case USBUVC_CONFIGSTRID:
        str = "UVC Config";
        break;
#endif /* !CONFIG_USBUVC_COMPOSITE */

      /* Interface strings — used in both standalone and composite.
       * In standalone: absolute IDs (USBUVC_VCIFSTRID/VSIFSTRID).
       * In composite: relative IDs (USBUVC_VCIFSTRID_REL/VSIFSTRID_REL),
       *               the composite framework adds strbase.
       */

#ifdef CONFIG_USBUVC_COMPOSITE
      case USBUVC_VCIFSTRID_REL:
#else
      case USBUVC_VCIFSTRID:
#endif
        str = "Video Control";
        break;

#ifdef CONFIG_USBUVC_COMPOSITE
      case USBUVC_VSIFSTRID_REL:
#else
      case USBUVC_VSIFSTRID:
#endif
        str = "Video Streaming";
        break;
      default:
        return -EINVAL;
    }

  len = strlen(str);
  if (len > (USBUVC_MAXSTRLEN / 2))
    {
      len = (USBUVC_MAXSTRLEN / 2);
    }

  for (i = 0, ndata = 0; i < len; i++, ndata += 2)
    {
      data[ndata]     = str[i];
      data[ndata + 1] = 0;
    }

  strdesc->len  = ndata + 2;
  strdesc->type = USB_DESC_TYPE_STRING;
  return strdesc->len;
}

/****************************************************************************
 * Private Functions — UVC Class-Specific Setup (EP0)
 ****************************************************************************/

static int uvc_class_setup(FAR struct uvc_dev_s *priv,
                           FAR struct usbdev_s *dev,
                           FAR const struct usb_ctrlreq_s *ctrl,
                           FAR struct usbdev_req_s *ctrlreq)
{
  uint8_t req  = ctrl->req;
  uint8_t cs   = ctrl->value[1];  /* Control selector */
  uint16_t len = GETUINT16(ctrl->len);
  int ret = -EOPNOTSUPP;

  uinfo("UVC class setup: req=0x%02x cs=0x%02x len=%d\n", req, cs, len);

  if (cs == UVC_VS_PROBE_CONTROL || cs == UVC_VS_COMMIT_CONTROL)
    {
      FAR struct uvc_streaming_control_s *target;

      target = (cs == UVC_VS_PROBE_CONTROL) ?
               &priv->probe : &priv->commit;

      switch (req)
        {
          case UVC_SET_CUR:
            /* Host sends probe/commit data — we accept it.
             * The data arrives in dataout phase, handled below.
             * For now just return 0 to ACK the setup.
             */

            if (len >= UVC_PROBE_COMMIT_SIZE)
              {
                len = UVC_PROBE_COMMIT_SIZE;
              }

            ret = 0;
            break;

          case UVC_GET_CUR:
          case UVC_GET_MIN:
          case UVC_GET_MAX:
          case UVC_GET_DEF:
            if (len > UVC_PROBE_COMMIT_SIZE)
              {
                len = UVC_PROBE_COMMIT_SIZE;
              }

            memcpy(ctrlreq->buf, target, len);
            ctrlreq->len = len;
            ret = len;
            break;

          default:
            break;
        }
    }

  return ret;
}

/****************************************************************************
 * Private Functions — USB Class Driver Operations
 ****************************************************************************/

static int uvc_bind(FAR struct usbdevclass_driver_s *driver,
                    FAR struct usbdev_s *dev)
{
  FAR struct uvc_dev_s *priv = g_uvc_dev;
  FAR struct usbdev_req_s *req;
  int i;

  uinfo("UVC bind\n");

  priv->usbdev = dev;
  priv->ep0    = dev->ep0;

  /* Allocate control request */

  priv->ctrlreq = usbdev_allocreq(dev->ep0, 256);
  if (!priv->ctrlreq)
    {
      return -ENOMEM;
    }

  priv->ctrlreq->callback = uvc_ep0incomplete;

  /* Allocate Bulk IN endpoint — use devinfo.epno[] set by
   * standalone init or composite framework.
   */

  priv->epbulkin = DEV_ALLOCEP(dev,
                               USB_EPIN(priv->devinfo.epno[
                                        USBUVC_EP_BULKIN_IDX]),
                               true,
                               USB_EP_ATTR_XFER_BULK);
  if (!priv->epbulkin)
    {
      uerr("Failed to allocate bulk IN EP\n");
      return -ENODEV;
    }

  priv->epbulkin->priv = priv;
  priv->devinfo.epno[USBUVC_EP_BULKIN_IDX] =
    USB_EPNO(priv->epbulkin->eplog);

  /* Allocate write requests */

  sq_init(&priv->wrfree);
  for (i = 0; i < CONFIG_USBUVC_NWRREQS; i++)
    {
      req = usbdev_allocreq(priv->epbulkin,
                              CONFIG_USBUVC_EPBULKIN_FSSIZE);
      if (!req)
        {
          break;
        }

      req->callback = uvc_wrreq_callback;
      req->priv     = &priv->wrreqs[i];
      priv->wrreqs[i].req = req;
      sq_addlast((FAR sq_entry_t *)&priv->wrreqs[i], &priv->wrfree);
      priv->nwralloc++;
    }

  uvc_default_streaming_ctrl(priv, &priv->probe);
  uvc_default_streaming_ctrl(priv, &priv->commit);

  DEV_CONNECT(dev);
  return OK;
}

static void uvc_unbind(FAR struct usbdevclass_driver_s *driver,
                       FAR struct usbdev_s *dev)
{
  FAR struct uvc_dev_s *priv = g_uvc_dev;
  FAR struct uvc_wrreq_s *wrcontainer;

  uinfo("UVC unbind\n");

  /* Free write requests */

  while ((wrcontainer = (FAR struct uvc_wrreq_s *)
                         sq_remfirst(&priv->wrfree)))
    {
      usbdev_freereq(priv->epbulkin, wrcontainer->req);
      wrcontainer->req = NULL;
    }

  if (priv->epbulkin)
    {
      DEV_FREEEP(dev, priv->epbulkin);
      priv->epbulkin = NULL;
    }

  if (priv->ctrlreq)
    {
      usbdev_freereq(dev->ep0, priv->ctrlreq);
      priv->ctrlreq = NULL;
    }

  DEV_DISCONNECT(dev);
}

static int uvc_setup(FAR struct usbdevclass_driver_s *driver,
                     FAR struct usbdev_s *dev,
                     FAR const struct usb_ctrlreq_s *ctrl,
                     FAR uint8_t *dataout, size_t outlen)
{
  FAR struct uvc_dev_s *priv = g_uvc_dev;
  FAR struct usbdev_req_s *ctrlreq = priv->ctrlreq;
  uint16_t value  = GETUINT16(ctrl->value);
  uint16_t index  = GETUINT16(ctrl->index);
  uint16_t len    = GETUINT16(ctrl->len);
  int ret = -EOPNOTSUPP;

  UNUSED(index);

  uinfo("UVC setup: type=0x%02x req=0x%02x value=0x%04x len=%d\n",
        ctrl->type, ctrl->req, value, len);

  /* Handle class-specific requests */

  if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS)
    {
      /* SET_CUR with data phase — copy incoming data */

      if (ctrl->req == UVC_SET_CUR && dataout && outlen > 0)
        {
          uint8_t cs = ctrl->value[1];
          if (cs == UVC_VS_PROBE_CONTROL)
            {
              memcpy(&priv->probe, dataout,
                     outlen < UVC_PROBE_COMMIT_SIZE ?
                     outlen : UVC_PROBE_COMMIT_SIZE);

              /* Clamp to our only supported format */

              priv->probe.bformatindex = 1;
              priv->probe.bframeindex  = 1;
              priv->probe.dwframeinterval = priv->frame_interval;
              priv->probe.dwmaxvideoframesize = priv->frame_size;
              priv->probe.dwmaxpayloadtransfersize =
                priv->frame_size + UVC_PAYLOAD_HEADER_LEN;

              /* Send ZLP status stage for OUT control transfer */

              ctrlreq->len = 0;
              ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;
              UVC_EP0SUBMIT(driver, dev, ctrlreq, ctrl);
              return 0;
            }
          else if (cs == UVC_VS_COMMIT_CONTROL)
            {
              memcpy(&priv->commit, dataout,
                     outlen < UVC_PROBE_COMMIT_SIZE ?
                     outlen : UVC_PROBE_COMMIT_SIZE);

              /* Reset wrsem to 0 before starting new stream.
               * Stale counts from previous EP_CANCEL callbacks
               * would cause NULL wrcontainer dereferences.
               */

              while (nxsem_trywait(&priv->wrsem) == OK);

              priv->streaming = true;
              poll_notify(priv->fds, CONFIG_USBUVC_NPOLLWAITERS,
                          POLLOUT);
              uinfo("UVC streaming committed\n");

              /* Send ZLP status stage for OUT control transfer */

              ctrlreq->len = 0;
              ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;
              UVC_EP0SUBMIT(driver, dev, ctrlreq, ctrl);
              return 0;
            }
        }

      ret = uvc_class_setup(priv, dev, ctrl, ctrlreq);
      if (ret > 0)
        {
          ctrlreq->len = ret;
          ret = UVC_EP0SUBMIT(driver, dev, ctrlreq, ctrl);
          if (ret < 0)
            {
              uerr("EP_SUBMIT failed: %d\n", ret);
            }

          return ret;
        }

      /* ret == 0 means SET_CUR accepted, waiting for data phase.
       * ret < 0 means unsupported, will fall through.
       */

      if (ret == 0)
        {
          return ret;
        }
    }

  /* Handle standard requests — descriptor requests are standalone only;
   * SET_CONFIGURATION / SET_INTERFACE are needed in both modes.
   */

  if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD)
    {
      switch (ctrl->req)
        {
#ifndef CONFIG_USBUVC_COMPOSITE
          case USB_REQ_GETDESCRIPTOR:
            {
              switch (ctrl->value[1])
                {
                  case USB_DESC_TYPE_DEVICE:
                    {
                      ret = usbdev_copy_devdesc(ctrlreq->buf,
                                                &g_uvc_devdesc,
                                                dev->speed);
                    }
                    break;

                  case USB_DESC_TYPE_CONFIG:
                    {
                      ret = uvc_mkcfgdesc(ctrlreq->buf,
                                          &priv->devinfo,
                                          dev->speed,
                                          ctrl->value[1]);
                    }
                    break;

                  case USB_DESC_TYPE_STRING:
                    {
                      ret = uvc_mkstrdesc(ctrl->value[0],
                                (FAR struct usb_strdesc_s *)ctrlreq->buf);
                    }
                    break;

                  default:
                    break;
                }
            }
            break;
#endif /* !CONFIG_USBUVC_COMPOSITE */

          case USB_REQ_SETCONFIGURATION:
            if (value == 1)
              {
                /* Configure the bulk IN endpoint */

                struct usb_epdesc_s epdesc;
                epdesc.len      = USB_SIZEOF_EPDESC;
                epdesc.type     = USB_DESC_TYPE_ENDPOINT;
                epdesc.addr     = USB_EPIN(
                                    USB_EPNO(priv->epbulkin->eplog));
                epdesc.attr     = USB_EP_ATTR_XFER_BULK;
                epdesc.mxpacketsize[0] =
                  LSBYTE(CONFIG_USBUVC_EPBULKIN_FSSIZE);
                epdesc.mxpacketsize[1] =
                  MSBYTE(CONFIG_USBUVC_EPBULKIN_FSSIZE);
                epdesc.interval = 0;

                EP_CONFIGURE(priv->epbulkin, &epdesc, true);
                priv->config = value;
                ret = 0;
              }
            else if (value == 0)
              {
                uvc_streaming_stop(priv);
                EP_DISABLE(priv->epbulkin);
                priv->config = 0;
                ret = 0;
              }
            break;

          case USB_REQ_SETINTERFACE:

            /* alt=0 means host is stopping the stream */

            if ((value & 0xff) == 0 && priv->streaming)
              {
                uvc_streaming_stop(priv);
              }

            ret = 0;
            break;

          case USB_REQ_GETINTERFACE:
            ctrlreq->buf[0] = 0;
            ctrlreq->len    = 1;
            ret = UVC_EP0SUBMIT(driver, dev, ctrlreq, ctrl);
            return ret;

          default:
            break;
        }
    }

  /* Respond to the setup command if data was returned.  On an error
   * return value (ret < 0), the USB driver will stall EP0.
   */

  if (ret >= 0)
    {
      ctrlreq->len   = len < ret ? len : ret;
      ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;
      ret = UVC_EP0SUBMIT(driver, dev, ctrlreq, ctrl);
      if (ret < 0)
        {
          uerr("EP_SUBMIT failed: %d\n", ret);
        }
    }

  return ret;
}

static void uvc_streaming_stop(FAR struct uvc_dev_s *priv)
{
  /* Called from interrupt context (disconnect/setup) or from
   * uvc_write timeout — no mutex!
   * Cancel all pending bulk IN requests so callbacks fire immediately.
   * EP_CANCEL callbacks will post wrsem for each cancelled request,
   * which unblocks uvc_write().
   *
   * Guard against double-cancel: uvc_write timeout may call this,
   * then the USB disconnect ISR calls it again.  Calling EP_CANCEL
   * on an already-cancelled endpoint can corrupt the request list.
   */

  if (!priv->streaming)
    {
      return;
    }

  priv->streaming = false;
  poll_notify(priv->fds, CONFIG_USBUVC_NPOLLWAITERS, POLLHUP);

  if (priv->epbulkin)
    {
      EP_CANCEL(priv->epbulkin, NULL);
    }
}

static void uvc_disconnect(FAR struct usbdevclass_driver_s *driver,
                           FAR struct usbdev_s *dev)
{
  FAR struct uvc_dev_s *priv = g_uvc_dev;

  uinfo("UVC disconnect\n");

  /* Called from USB interrupt context — do NOT use mutex */

  priv->config = 0;
  uvc_streaming_stop(priv);
}

/****************************************************************************
 * Private Functions — File Operations (/dev/uvc0)
 ****************************************************************************/

static int uvc_open(FAR struct file *filep)
{
  return OK;
}

static int uvc_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: uvc_poll
 *
 * Description:
 *   Poll for POLLOUT — reports writable when host is streaming.
 *
 ****************************************************************************/

static int uvc_poll(FAR struct file *filep, FAR struct pollfd *fds,
                    bool setup)
{
  FAR struct uvc_dev_s *priv = g_uvc_dev;
  irqstate_t flags;
  int ret = OK;
  int i;

  if (!setup)
    {
      /* Teardown — clear the slot */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;
      if (slot)
        {
          *slot = NULL;
        }

      fds->priv = NULL;
      return OK;
    }

  /* Setup — find an available slot */

  nxmutex_lock(&priv->lock);
  flags = enter_critical_section();

  for (i = 0; i < CONFIG_USBUVC_NPOLLWAITERS; i++)
    {
      if (!priv->fds[i])
        {
          priv->fds[i] = fds;
          fds->priv     = &priv->fds[i];
          break;
        }
    }

  if (i >= CONFIG_USBUVC_NPOLLWAITERS)
    {
      ret = -EBUSY;
    }
  else if (priv->config && priv->streaming)
    {
      poll_notify(&priv->fds[i], 1, POLLOUT);
    }

  leave_critical_section(flags);
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: uvc_write
 *
 * Description:
 *   Write a complete video frame to the UVC bulk endpoint.
 *   The driver prepends UVC Payload Headers and splits the frame
 *   into max-packet-sized USB transfers.
 *
 *   Expected input: raw YUY2 frame data
 *
 ****************************************************************************/

static ssize_t uvc_write(FAR struct file *filep,
                         FAR const char *buffer, size_t buflen)
{
  FAR struct uvc_dev_s *priv = g_uvc_dev;
  FAR struct uvc_wrreq_s *wrcontainer;
  FAR struct usbdev_req_s *req;
  irqstate_t flags;
  size_t remaining = buflen;
  size_t offset = 0;
  size_t payload_max;
  size_t chunk;
  int ret;

  nxmutex_lock(&priv->lock);

  if (!priv->epbulkin || !priv->config || !priv->streaming)
    {
      nxmutex_unlock(&priv->lock);
      return -EAGAIN;
    }

  /* Zero-length write is used to probe readiness */

  if (buflen == 0)
    {
      nxmutex_unlock(&priv->lock);
      return 0;
    }

  payload_max = CONFIG_USBUVC_EPBULKIN_FSSIZE - UVC_PAYLOAD_HEADER_LEN;

  while (remaining > 0)
    {
      /* Get a write request from the pool */

      flags = enter_critical_section();
      wrcontainer = (FAR struct uvc_wrreq_s *)sq_remfirst(&priv->wrfree);
      leave_critical_section(flags);

      if (!wrcontainer)
        {
          /* No request available — wait with timeout.
           * If host stops reading (e.g. cheese closed), bulk IN
           * requests never complete.  Timeout and treat as host-gone.
           *
           * NOTE: When the host truly disconnects or closes the
           * video device, the USB disconnect ISR or a CLEAR_FEATURE
           * control transfer will call uvc_streaming_stop()
           * immediately — the timeout is only a safety net.
           *
           * It must be long enough to cover the worst case: host
           * sends COMMIT, then spends many seconds on REQBUFS /
           * MMAP / QBUF / STREAMON before reading bulk data.
           * After an unclean close (e.g. v4l2-ctl killed by
           * SIGTERM), the host UVC driver reset can add 5-10s.
           * 30 seconds is generous enough to never false-trigger.
           */

          nxmutex_unlock(&priv->lock);
          ret = nxsem_tickwait(&priv->wrsem, 30 * TICK_PER_SEC);
          nxmutex_lock(&priv->lock);

          if (ret == -ETIMEDOUT)
            {
              uwarn("UVC write timeout — host not reading\n");
              uvc_streaming_stop(priv);
              nxmutex_unlock(&priv->lock);
              return -EAGAIN;
            }

          /* Re-check state after re-acquiring lock.
           * streaming_stop() cancels EP requests which unblocks us.
           */

          if (!priv->streaming || !priv->config || !priv->epbulkin)
            {
              nxmutex_unlock(&priv->lock);
              return -EAGAIN;
            }

          /* wrcontainer might still be NULL if wrsem had stale count */

          continue;
        }

      req = wrcontainer->req;

      /* Build UVC Payload Header */

      chunk = remaining > payload_max ? payload_max : remaining;

      req->buf[0] = UVC_PAYLOAD_HEADER_LEN;  /* bHeaderLength */
      req->buf[1] = priv->fid & UVC_STREAM_FID;

      if (remaining <= payload_max)
        {
          /* Last packet of this frame */

          req->buf[1] |= UVC_STREAM_EOF;
        }

      /* Copy video data after header */

      memcpy(req->buf + UVC_PAYLOAD_HEADER_LEN, buffer + offset, chunk);
      req->len = chunk + UVC_PAYLOAD_HEADER_LEN;

      ret = EP_SUBMIT(priv->epbulkin, req);
      if (ret < 0)
        {
          /* Return request to pool */

          flags = enter_critical_section();
          sq_addlast((FAR sq_entry_t *)wrcontainer, &priv->wrfree);
          leave_critical_section(flags);

          if (ret == -ESHUTDOWN)
            {
              uwarn("UVC EP shutdown — stopping stream\n");
              priv->streaming = false;
              poll_notify(priv->fds, CONFIG_USBUVC_NPOLLWAITERS,
                          POLLHUP);
              nxmutex_unlock(&priv->lock);
              return -EAGAIN;
            }

          uerr("EP_SUBMIT failed: %d\n", ret);
          nxmutex_unlock(&priv->lock);
          return ret;
        }

      offset    += chunk;
      remaining -= chunk;
    }

  /* Toggle FID for next frame */

  priv->fid ^= 1;

  nxmutex_unlock(&priv->lock);
  return buflen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbdev_uvc_classobject
 *
 * Description:
 *   Create a UVC class driver instance.  Used by both standalone init
 *   and composite framework.
 *
 ****************************************************************************/

int usbdev_uvc_classobject(int minor,
                           FAR struct usbdev_devinfo_s *devinfo,
                           FAR const struct uvc_params_s *params,
                           FAR struct usbdevclass_driver_s **classdev)
{
  FAR struct uvc_dev_s *priv;
  FAR struct usbdevclass_driver_s *drvr;
  int ret;

  /* Allocate device struct + class driver struct */

  priv = kmm_zalloc(sizeof(struct uvc_dev_s) +
                     sizeof(struct usbdevclass_driver_s));
  if (!priv)
    {
      return -ENOMEM;
    }

  /* Store runtime video parameters */

  if (params)
    {
      priv->width  = params->width;
      priv->height = params->height;
      priv->fps    = params->fps;
    }
  else
    {
      priv->width  = USBUVC_DEF_WIDTH;
      priv->height = USBUVC_DEF_HEIGHT;
      priv->fps    = USBUVC_DEF_FPS;
    }

  priv->frame_size     = (uint32_t)priv->width * priv->height * USBUVC_BPP;
  priv->frame_interval = 10000000 / priv->fps;  /* 100ns units */
  priv->bitrate        = priv->frame_size * 8 * priv->fps;

  /* Save devinfo — interface/string/endpoint bases from caller */

  memcpy(&priv->devinfo, devinfo, sizeof(struct usbdev_devinfo_s));

  g_uvc_dev = priv;
  nxmutex_init(&priv->lock);
  nxsem_init(&priv->wrsem, 0, 0);

  /* Set up the class driver structure right after priv */

  drvr = (FAR struct usbdevclass_driver_s *)(priv + 1);
  drvr->ops   = &g_uvc_driverops;
  drvr->speed = USB_SPEED_FULL;

  /* Register the character device */

  ret = register_driver(USBUVC_CHARDEV_PATH, &g_uvc_fops, 0666, priv);
  if (ret < 0)
    {
      uerr("register_driver failed: %d\n", ret);
      kmm_free(priv);
      g_uvc_dev = NULL;
      return ret;
    }

  uinfo("UVC gadget initialized: %s\n", USBUVC_CHARDEV_PATH);
  *classdev = drvr;
  return OK;
}

/****************************************************************************
 * Name: usbdev_uvc_classuninitialize
 *
 * Description:
 *   Uninitialize a UVC class driver instance.
 *
 ****************************************************************************/

void usbdev_uvc_classuninitialize(
    FAR struct usbdevclass_driver_s *classdev)
{
  FAR struct uvc_dev_s *priv;

  if (!classdev)
    {
      return;
    }

  /* priv is right before classdev in the allocation */

  priv = (FAR struct uvc_dev_s *)classdev - 1;

  unregister_driver(USBUVC_CHARDEV_PATH);
  nxmutex_destroy(&priv->lock);
  nxsem_destroy(&priv->wrsem);
  kmm_free(priv);

  if (g_uvc_dev == priv)
    {
      g_uvc_dev = NULL;
    }
}

/****************************************************************************
 * Name: usbdev_uvc_initialize
 *
 * Description:
 *   Standalone mode initialization — creates the class driver and
 *   registers it directly with the USB device controller.
 *
 ****************************************************************************/

#ifndef CONFIG_USBUVC_COMPOSITE
FAR void *usbdev_uvc_initialize(FAR const struct uvc_params_s *params)
{
  FAR struct usbdevclass_driver_s *classdev = NULL;
  struct usbdev_devinfo_s devinfo;
  int ret;

  memset(&devinfo, 0, sizeof(devinfo));
  devinfo.ninterfaces = USBUVC_NINTERFACES;
  devinfo.nstrings    = USBUVC_NSTDSTRIDS;
  devinfo.nendpoints  = USBUVC_NUM_EPS;
  devinfo.epno[USBUVC_EP_BULKIN_IDX] = CONFIG_USBUVC_EPBULKIN;

  ret = usbdev_uvc_classobject(0, &devinfo, params, &classdev);
  if (ret < 0)
    {
      return NULL;
    }

  ret = usbdev_register(classdev);
  if (ret < 0)
    {
      uerr("usbdev_register failed: %d\n", ret);
      usbdev_uvc_classuninitialize(classdev);
      return NULL;
    }

  /* Return priv as handle (priv is right before classdev) */

  return (FAR void *)((FAR struct uvc_dev_s *)classdev - 1);
}

/****************************************************************************
 * Name: usbdev_uvc_uninitialize
 ****************************************************************************/

void usbdev_uvc_uninitialize(FAR void *handle)
{
  FAR struct uvc_dev_s *priv = handle;
  FAR struct usbdevclass_driver_s *drvr;

  if (!priv)
    {
      return;
    }

  drvr = (FAR struct usbdevclass_driver_s *)(priv + 1);
  usbdev_unregister(drvr);
  usbdev_uvc_classuninitialize(drvr);
}
#endif /* !CONFIG_USBUVC_COMPOSITE */

/****************************************************************************
 * Name: usbdev_uvc_get_composite_devdesc
 *
 * Description:
 *   Fill in a composite_devdesc_s for the UVC gadget.
 *   Board code must set classobject/uninitialize, ifnobase, strbase,
 *   and epno[] before passing to composite_initialize().
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_COMPOSITE
void usbdev_uvc_get_composite_devdesc(
    FAR struct composite_devdesc_s *dev)
{
  memset(dev, 0, sizeof(struct composite_devdesc_s));

  dev->mkconfdesc          = uvc_mkcfgdesc;
  dev->mkstrdesc           = uvc_mkstrdesc;

  /* classobject/uninitialize are left NULL — board code must wrap
   * usbdev_uvc_classobject() to pass uvc_params_s.
   */

  dev->nconfigs            = USBUVC_NCONFIGS;
  dev->configid            = 1;
  dev->cfgdescsize         = uvc_mkcfgdesc(NULL, NULL,
                                           USB_SPEED_UNKNOWN, 0);
  dev->devinfo.ninterfaces = USBUVC_NINTERFACES;
  dev->devinfo.nstrings    = USBUVC_NSTRIDS;
  dev->devinfo.nendpoints  = USBUVC_NUM_EPS;
  dev->devinfo.epinfos     = g_uvc_epinfos;
  dev->devinfo.name        = USBUVC_CHARDEV_PATH;
}
#endif
