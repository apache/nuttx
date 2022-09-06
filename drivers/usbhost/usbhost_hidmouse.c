/****************************************************************************
 * drivers/usbhost/usbhost_hidmouse.c
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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <poll.h>
#include <signal.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <fixedmath.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/fs/fs.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/hid.h>
#include <nuttx/usb/usbhost_devaddr.h>

#ifdef CONFIG_HIDMOUSE_TSCIF
#  include <nuttx/input/touchscreen.h>
#else
#  include <nuttx/input/mouse.h>
#endif

/* Don't compile if prerequisites are not met */

#if defined(CONFIG_USBHOST) && !defined(CONFIG_USBHOST_INT_DISABLE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Worker thread is needed, unfortunately, to handle some cornercase failure
 * conditions.  This is kind of wasteful and begs for a re-design.
 */

#ifndef CONFIG_SCHED_WORKQUEUE
#  warning "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

/* Provide some default values for other configuration settings */

#ifndef CONFIG_HIDMOUSE_XMAX
#  define CONFIG_HIDMOUSE_XMAX 320
#endif

#define HIDMOUSE_XMAX_B16 (CONFIG_HIDMOUSE_XMAX << 16)

#ifndef CONFIG_HIDMOUSE_YMAX
#  define CONFIG_HIDMOUSE_YMAX 240
#endif

#define HIDMOUSE_YMAX_B16 (CONFIG_HIDMOUSE_YMAX << 16)

#ifndef CONFIG_HIDMOUSE_XSCALE
#  define CONFIG_HIDMOUSE_XSCALE 0x00010000
#endif

#ifndef CONFIG_HIDMOUSE_YSCALE
#  define CONFIG_HIDMOUSE_YSCALE 0x00010000
#endif

#ifndef CONFIG_HIDMOUSE_XTHRESH
#  define CONFIG_HIDMOUSE_XTHRESH 12
#endif

#define HIDMOUSE_XTHRESH_B16 (CONFIG_HIDMOUSE_XTHRESH << 16)

#ifndef CONFIG_HIDMOUSE_YTHRESH
#  define CONFIG_HIDMOUSE_YTHRESH 12
#endif

#define HIDMOUSE_YTHRESH_B16 (CONFIG_HIDMOUSE_YTHRESH << 16)

#ifdef CONFIG_HIDMOUSE_TSCIF
#  undef CONFIG_INPUT_MOUSE_WHEEL
#endif

#ifdef CONFIG_INPUT_MOUSE_WHEEL

#  ifndef CONFIG_HIDMOUSE_WMAX
#    define CONFIG_HIDMOUSE_WMAX 100
#  endif

#  define HIDMOUSE_WMAX_B16 (CONFIG_HIDMOUSE_WMAX << 16)

#  ifndef CONFIG_HIDMOUSE_WSCALE
#    define CONFIG_HIDMOUSE_WSCALE 0x00010000
#  endif

#  ifndef CONFIG_HIDMOUSE_WTHRESH
#    define CONFIG_HIDMOUSE_WTHRESH 1
#  endif

#  define HIDMOUSE_WTHRESH_B16 (CONFIG_HIDMOUSE_WTHRESH << 16)

#endif /* CONFIG_INPUT_MOUSE_WHEEL */

#ifndef CONFIG_HIDMOUSE_DEFPRIO
#  define CONFIG_HIDMOUSE_DEFPRIO 50
#endif

#ifndef CONFIG_HIDMOUSE_STACKSIZE
#  define CONFIG_HIDMOUSE_STACKSIZE 1024
#endif

#ifndef CONFIG_HIDMOUSE_NPOLLWAITERS
#  define CONFIG_HIDMOUSE_NPOLLWAITERS 2
#endif

/* This is a value for the threshold that guarantees a big difference on the
 * first left button (but can't overflow).
 */

#define INVALID_POSITION_B16 (0x7fffffff)

/* Driver support ***********************************************************/

/* This format is used to construct the /dev/mouse[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#ifdef CONFIG_HIDMOUSE_TSCIF
#  define DEV_FORMAT        "/dev/input%d"
#  define DEV_NAMELEN       14
#else
#  define DEV_FORMAT        "/dev/mouse%d"
#  define DEV_NAMELEN       14
#endif

/* Used in usbhost_cfgdesc() */

#define USBHOST_IFFOUND     0x01 /* Required I/F descriptor found */
#define USBHOST_EPINFOUND   0x02 /* Required interrupt IN EP descriptor found */
#define USBHOST_ALLFOUND    (USBHOST_IFFOUND|USBHOST_EPINFOUND)

#define USBHOST_MAX_CREFS   0x7fff

/* Debug ********************************************************************/

/* Both CONFIG_DEBUG_INPUT and CONFIG_DEBUG_USB could apply to this file.
 * We assume here that CONFIG_DEBUG_INPUT might be enabled separately, but
 * CONFIG_DEBUG_USB implies both.
 */

#ifndef CONFIG_DEBUG_INPUT
#  undef  ierr
#  define ierr    uerr
#  undef  iinfo
#  define iinfo   uinfo
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_HIDMOUSE_TSCIF
/* This describes the state of one event */

enum mouse_button_e
{
  BUTTON_NONE = 0,                      /* No button pressed */
  BUTTON_PRESSED,                       /* Left buffer pressed */
  BUTTON_MOVE,                          /* Same button press, possibly different position */
  BUTTON_RELEASED                       /* Left button released */
};

/* This structure describes the results of one mouse sample.  Since this
 * emulates a touchscreen, only the mouse left button is used.
 */

struct mouse_sample_s
{
  uint8_t  id;                          /* Sampled touch point ID */
  uint8_t  event;                       /* Button state (see enum mouse_button_e) */
  bool     valid;                       /* True: x,y contain valid, sampled data */
  uint16_t x;                           /* Measured X position */
  uint16_t y;                           /* Measured Y position */
};

#else
/* This structure summarizes the mouse report */

struct mouse_sample_s
{
  uint8_t  buttons;                     /* Button state (see MOUSE_BUTTON_* definitions) */
  uint16_t x;                           /* Accumulated X position */
  uint16_t y;                           /* Accumulated Y position */
#ifdef CONFIG_INPUT_MOUSE_WHEEL
  uint16_t wheel;                       /* Reported wheel position */
#endif
};
#endif

/* This structure contains the internal, private state of the USB host
 * mouse storage class.
 */

struct usbhost_state_s
{
  /* This is the externally visible portion of the state */

  struct usbhost_class_s  usbclass;

  /* This is an instance of the USB host driver
   * bound to this class instance
   */

  struct usbhost_driver_s *drvr;

  /* The remainder of the fields are provide o the mouse class driver */

  volatile bool           disconnected; /* TRUE: Device has been disconnected */
  volatile bool           polling;      /* TRUE: Poll thread is running */
  volatile bool           open;         /* TRUE: The mouse device is open */
  volatile bool           valid;        /* TRUE: New sample data is available */
  uint8_t                 devno;        /* Minor number in the /dev/mouse[n] device */
  uint8_t                 nwaiters;     /* Number of threads waiting for mouse data */
#ifdef CONFIG_HIDMOUSE_TSCIF
  uint8_t                 id;           /* Current "touch" point ID */
#else
  uint8_t                 buttons;      /* Current state of the mouse buttons */
#endif
  int16_t                 crefs;        /* Reference count on the driver instance */
  mutex_t                 lock;         /* Used to maintain mutual exclusive access */
  sem_t                   waitsem;      /* Used to wait for mouse data */
  FAR uint8_t            *tbuffer;      /* The allocated transfer buffer */
  b16_t                   xaccum;       /* Current integrated X position */
  b16_t                   yaccum;       /* Current integrated Y position */
  b16_t                   xlast;        /* Last reported X position */
  b16_t                   ylast;        /* Last reported Y position */
#ifdef CONFIG_INPUT_MOUSE_WHEEL
  b16_t                   waccum;       /* Current integrated while position */
  b16_t                   wlast;        /* Last reported wheel position */
#endif
  size_t                  tbuflen;      /* Size of the allocated transfer buffer */
  pid_t                   pollpid;      /* PID of the poll task */
  struct work_s           work;         /* For cornercase error handling by the worker thread */
  struct mouse_sample_s   sample;       /* Last sampled mouse data */
  usbhost_ep_t            epin;         /* Interrupt IN endpoint */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

  struct pollfd *fds[CONFIG_HIDMOUSE_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Memory allocation services */

static inline FAR struct usbhost_state_s *usbhost_allocclass(void);
static inline void usbhost_freeclass(FAR struct usbhost_state_s *usbclass);

/* Device name management */

static int usbhost_allocdevno(FAR struct usbhost_state_s *priv);
static void usbhost_freedevno(FAR struct usbhost_state_s *priv);
static inline void usbhost_mkdevname(FAR struct usbhost_state_s *priv,
                                     FAR char *devname);

/* Mouse polling thread */

static void usbhost_destroy(FAR void *arg);
static void usbhost_notify(FAR struct usbhost_state_s *priv);
static void usbhost_position(FAR struct usbhost_state_s *priv,
                             FAR struct usbhid_mousereport_s *rpt);
#ifdef CONFIG_HIDMOUSE_TSCIF
static bool usbhost_touchscreen(FAR struct usbhost_state_s *priv,
                                FAR struct usbhid_mousereport_s *rpt);
#endif
static bool usbhost_threshold(FAR struct usbhost_state_s *priv);
static int usbhost_mouse_poll(int argc, char *argv[]);
static int usbhost_sample(FAR struct usbhost_state_s *priv,
                          FAR struct mouse_sample_s *sample);
static int usbhost_waitsample(FAR struct usbhost_state_s *priv,
                              FAR struct mouse_sample_s *sample);

/* Helpers for usbhost_connect() */

static inline int usbhost_cfgdesc(FAR struct usbhost_state_s *priv,
                                  FAR const uint8_t *configdesc,
                                  int desclen);
static inline int usbhost_devinit(FAR struct usbhost_state_s *priv);

/* (Little Endian) Data helpers */

static inline uint16_t usbhost_getle16(const uint8_t *val);

/* Transfer descriptor memory management */

static inline int usbhost_tdalloc(FAR struct usbhost_state_s *priv);
static inline int usbhost_tdfree(FAR struct usbhost_state_s *priv);

/* struct usbhost_registry_s methods */

static FAR struct usbhost_class_s *
usbhost_create(FAR struct usbhost_hubport_s *hport,
               FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int usbhost_connect(FAR struct usbhost_class_s *usbclass,
                           FAR const uint8_t *configdesc, int desclen);
static int usbhost_disconnected(FAR struct usbhost_class_s *usbclass);

/* Driver methods.  We export the mouse as a standard character driver */

static int usbhost_open(FAR struct file *filep);
static int usbhost_close(FAR struct file *filep);
static ssize_t usbhost_read(FAR struct file *filep,
                            FAR char *buffer, size_t len);
static ssize_t usbhost_write(FAR struct file *filep,
                             FAR const char *buffer, size_t len);
static int usbhost_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure provides the registry entry ID information that will  be
 * used to associate the USB host mouse class driver to a connected USB
 * device.
 */

static const struct usbhost_id_s g_hidmouse_id =
{
  USB_CLASS_HID,           /* base */
  USBHID_SUBCLASS_BOOTIF,  /* subclass */
  USBHID_PROTOCOL_MOUSE,   /* proto */
  0,                       /* vid */
  0                        /* pid */
};

/* This is the USB host storage class's registry entry */

static struct usbhost_registry_s g_hidmouse =
{
  NULL,                    /* flink */
  usbhost_create,          /* create */
  1,                       /* nids */
  &g_hidmouse_id           /* id[] */
};

static const struct file_operations g_hidmouse_fops =
{
  usbhost_open,            /* open */
  usbhost_close,           /* close */
  usbhost_read,            /* read */
  usbhost_write,           /* write */
  NULL,                    /* seek */
  NULL,                    /* ioctl */
  usbhost_poll             /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                   /* unlink */
#endif
};

/* This is a bitmap that is used to allocate device names /dev/mouse0-31. */

static uint32_t g_devinuse;

/* The following are used to managed the class creation operation */

static mutex_t g_lock = NXMUTEX_INITIALIZER;
static sem_t g_syncsem = SEM_INITIALIZER(0);
static FAR struct usbhost_state_s *g_priv;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_allocclass
 *
 * Description:
 *   This is really part of the logic that implements the create() method
 *   of struct usbhost_registry_s.  This function allocates memory for one
 *   new class instance.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s.  NULL is returned on failure; this function will
 *   will fail only if there are insufficient resources to create another
 *   USB host class instance.
 *
 ****************************************************************************/

static inline FAR struct usbhost_state_s *usbhost_allocclass(void)
{
  FAR struct usbhost_state_s *priv;

  DEBUGASSERT(!up_interrupt_context());

  priv = (FAR struct usbhost_state_s *)
    kmm_malloc(sizeof(struct usbhost_state_s));

  uinfo("Allocated: %p\n", priv);
  return priv;
}

/****************************************************************************
 * Name: usbhost_freeclass
 *
 * Description:
 *   Free a class instance previously allocated by usbhost_allocclass().
 *
 * Input Parameters:
 *   usbclass - A reference to the class instance to be freed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void usbhost_freeclass(FAR struct usbhost_state_s *usbclass)
{
  DEBUGASSERT(usbclass != NULL);

  /* Free the class instance. */

  uinfo("Freeing: %p\n", usbclass);
  kmm_free(usbclass);
}

/****************************************************************************
 * Name: Device name management
 *
 * Description:
 *   Some tiny functions to coordinate management of device names.
 *
 ****************************************************************************/

static int usbhost_allocdevno(FAR struct usbhost_state_s *priv)
{
  irqstate_t flags;
  int devno;

  flags = enter_critical_section();
  for (devno = 0; devno < 26; devno++)
    {
      uint32_t bitno = 1 << devno;
      if ((g_devinuse & bitno) == 0)
        {
          g_devinuse |= bitno;
          priv->devno = devno;
          leave_critical_section(flags);
          return OK;
        }
    }

  leave_critical_section(flags);
  return -EMFILE;
}

static void usbhost_freedevno(FAR struct usbhost_state_s *priv)
{
  int devno = priv->devno;

  if (devno >= 0 && devno < 26)
    {
      irqstate_t flags = enter_critical_section();
      g_devinuse &= ~(1 << devno);
      leave_critical_section(flags);
    }
}

static inline void usbhost_mkdevname(FAR struct usbhost_state_s *priv,
                                     FAR char *devname)
{
  snprintf(devname, DEV_NAMELEN, DEV_FORMAT, priv->devno);
}

/****************************************************************************
 * Name: usbhost_destroy
 *
 * Description:
 *   The USB device has been disconnected and the reference count on the USB
 *   host class instance has gone to 1.. Time to destroy the USB host class
 *   instance.
 *
 * Input Parameters:
 *   arg - A reference to the class instance to be destroyed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_destroy(FAR void *arg)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)arg;
  FAR struct usbhost_hubport_s *hport;
  char devname[DEV_NAMELEN];

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL);
  uinfo("crefs: %d\n", priv->crefs);

  hport = priv->usbclass.hport;

  /* Unregister the driver */

  uinfo("Unregister driver\n");
  usbhost_mkdevname(priv, devname);
  unregister_driver(devname);

  /* Release the device name used by this connection */

  usbhost_freedevno(priv);

  /* Free the interrupt endpoints */

  if (priv->epin)
    {
      DRVR_EPFREE(hport->drvr, priv->epin);
    }

  /* Free any transfer buffers */

  usbhost_tdfree(priv);

  /* Destroy the mutex & semaphores */

  nxmutex_destroy(&priv->lock);
  nxsem_destroy(&priv->waitsem);

  /* Disconnect the USB host device */

  DRVR_DISCONNECT(hport->drvr, hport);

  /* Free the function address assigned to this device */

  usbhost_devaddr_destroy(hport, hport->funcaddr);
  hport->funcaddr = 0;

  /* And free the class instance. */

  usbhost_freeclass(priv);
}

/****************************************************************************
 * Name: usbhost_notify
 *
 * Description:
 *   Wake any threads waiting for mouse data
 *
 * Input Parameters:
 *   priv - A reference to the mouse state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_notify(FAR struct usbhost_state_s *priv)
{
  /* If there are threads waiting for read data, then signal one of them
   * that the read data is available.
   */

  if (priv->nwaiters > 0)
    {
      nxsem_post(&priv->waitsem);
    }

  /* If there are threads waiting on poll() for mouse data to become
   * available, then wake them up now.  NOTE: we wake up all waiting
   * threads because we do not know that they are going to do.  If they
   * all try to read the data, then some make end up blocking after all.
   */

  poll_notify(priv->fds, CONFIG_HIDMOUSE_NPOLLWAITERS, POLLIN);
}

/****************************************************************************
 * Name: usbhost_position
 *
 * Description:
 *   Integrate the current mouse displacement to get the updated mouse
 *   position.
 *
 * Input Parameters:
 *   priv  - A reference to the mouse state structure.
 *   rpt   - The new mouse report data.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void usbhost_position(FAR struct usbhost_state_s *priv,
                             FAR struct usbhid_mousereport_s *rpt)
{
  int32_t disp;
  b16_t pos;

  /* The following logic performs an constant integration of the mouse X/Y
   * displacement data in order to keep the X/Y positional data current.
   */

  /* Sign extend the mouse X position.  We do this manually because some
   * architectures do not support signed character types and some compilers
   * may be configured to treat all characters as unsigned.
   */

#ifdef CONFIG_HIDMOUSE_SWAPXY
  disp = rpt->ydisp;
  if ((rpt->ydisp & 0x80) != 0)
    {
      disp |= 0xffffff00;
    }
#else
  disp = rpt->xdisp;
  if ((rpt->xdisp & 0x80) != 0)
    {
      disp |= 0xffffff00;
    }
#endif

  /* Scale the X displacement and determine the new X position */

  pos = priv->xaccum + CONFIG_HIDMOUSE_XSCALE * disp;

  /* Make sure that the scaled X position does not become negative or exceed
   * the maximum.
   */

  if (pos > HIDMOUSE_XMAX_B16)
    {
      pos = HIDMOUSE_XMAX_B16;
    }
  else if (pos < 0)
    {
      pos = 0;
    }

  /* Save the updated X position */

  priv->xaccum = pos;

  /* Do the same for the Y position */

#ifdef CONFIG_HIDMOUSE_SWAPXY
  disp = rpt->xdisp;
  if ((rpt->xdisp & 0x80) != 0)
    {
      disp |= 0xffffff00;
    }
#else
  disp = rpt->ydisp;
  if ((rpt->ydisp & 0x80) != 0)
    {
      disp |= 0xffffff00;
    }
#endif

  pos = priv->yaccum + CONFIG_HIDMOUSE_YSCALE * disp;

  if (pos > HIDMOUSE_YMAX_B16)
    {
      pos = HIDMOUSE_YMAX_B16;
    }
  else if (pos < 0)
    {
      pos = 0;
    }

  priv->yaccum = pos;

#ifdef CONFIG_INPUT_MOUSE_WHEEL
  /* Do the same for the wheel position */

  disp = rpt->wdisp;
  if ((rpt->wdisp & 0x80) != 0)
    {
      disp |= 0xffffff00;
    }

  pos = priv->waccum + CONFIG_HIDMOUSE_WSCALE * disp;

  if (pos > HIDMOUSE_WMAX_B16)
    {
      pos = HIDMOUSE_WMAX_B16;
    }
  else if (pos < 0)
    {
      pos = 0;
    }

  priv->waccum = pos;
#endif
}

/****************************************************************************
 * Name: usbhost_touchscreen
 *
 * Description:
 *   Execute the (emulated) touchscreen press/drag/release state machine.
 *
 * Input Parameters:
 *   priv  - A reference to the mouse state structure.
 *   rpt   - The new mouse report data.
 *
 * Returned Value:
 *   False if the mouse data should not be reported.
 *
 ****************************************************************************/

#ifdef CONFIG_HIDMOUSE_TSCIF
static bool usbhost_touchscreen(FAR struct usbhost_state_s *priv,
                                FAR struct usbhid_mousereport_s *rpt)
{
  /* Check if the left button is pressed */

  if ((rpt->buttons & USBHID_MOUSEIN_BUTTON1) == 0)
    {
      /* The left button is not pressed.. reset thresholding variables. */

      priv->xlast = INVALID_POSITION_B16;
      priv->ylast = INVALID_POSITION_B16;

      /* Ignore the report if the button was not pressed last time
       * (BUTTON_NONE == button released and already reported;
       * BUTTON_RELEASED == button released, but not yet reported)
       */

      if (priv->sample.event == BUTTON_NONE ||
          priv->sample.event == BUTTON_RELEASED)
        {
          return false;
        }

      /* The left button has just been released.  NOTE: We know from a
       * previous test, that this is a button release condition. This will
       * be changed to BUTTON_NONE after the button release has been
       * reported.
       */

      priv->sample.event = BUTTON_RELEASED;
    }

  /* It is a left button press event.  If the last button release event has
   * not been processed yet, then we have to ignore the button press event
   * (or else it will look like a drag event)
   */

  else if (priv->sample.event == BUTTON_RELEASED)
    {
      /* If we have not yet processed the button release event, then we
       * cannot handle this button press event. We will have to discard the
       * data and wait for the next sample.
       */

      return false;
    }

  /* Handle left-button down events */

  else
    {
      /* If this is the first left button press report, then report that
       * event.  If event == BUTTON_PRESSED, it will be  set to set to
       * BUTTON_MOVE after the button press is first sampled.
       */

      if (priv->sample.event != BUTTON_MOVE)
        {
          /* First button press */

          priv->sample.event = BUTTON_PRESSED;
        }

      /* Otherwise, perform a thresholding operation so that the results
       * will be more stable.  If the difference from the last sample is
       * small, then ignore the event.
       */

      else if (!usbhost_xythreshold(priv))
        {
          return false;
        }
    }

  /* We get here:
   *
   * (1) When the left button is just release,
   * (2) When the left button is first pressed, or
   * (3) When the left button is held and some significant 'dragging'
   *     has occurred.
   */

  return true;
}
#endif

/****************************************************************************
 * Name: usbhost_threshold
 *
 * Description:
 *   Check if the current mouse position differs from the previous mouse
 *   position by a threshold amount.
 *
 * Input Parameters:
 *   priv - A reference to the mouse state structure.
 *
 * Returned Value:
 *   True if the mouse position is significantly different from the last
 *   reported mouse position.
 *
 ****************************************************************************/

static bool usbhost_threshold(FAR struct usbhost_state_s *priv)
{
#if CONFIG_HIDMOUSE_XTHRESH > 0 && CONFIG_HIDMOUSE_YTHRESH > 0
  b16_t pos;
  b16_t diff;

  /* Get the difference in the X position from the last report */

  pos = priv->xaccum;
  if (pos > priv->xlast)
    {
      diff = pos - priv->xlast;
    }
  else
    {
      diff = priv->xlast - pos;
    }

  /* Check if the X difference exceeds the report threshold */

  if (diff >= HIDMOUSE_XTHRESH_B16)
    {
      return true;
    }

  /* Little or no change in the X direction, check the Y direction.  */

  pos = priv->yaccum;
  if (pos > priv->ylast)
    {
      diff = pos - priv->ylast;
    }
  else
    {
      diff = priv->ylast - pos;
    }

  if (diff >= HIDMOUSE_YTHRESH_B16)
    {
      return true;
    }

#ifdef CONFIG_INPUT_MOUSE_WHEEL
  /* Get the difference in the wheel position from the last report */

  pos = priv->waccum;
  if (pos > priv->wlast)
    {
      diff = pos - priv->wlast;
    }
  else
    {
      diff = priv->wlast - pos;
    }

  /* Check if the X difference exceeds the report threshold */

  if (diff >= HIDMOUSE_WTHRESH_B16)
    {
      return true;
    }
#endif

  /* Little or no change in either direction... don't report anything. */

  return false;
#else
  /* No thresholding */

  return true;
#endif
}

/****************************************************************************
 * Name: usbhost_mouse_poll
 *
 * Description:
 *   Periodically check for new mouse data.
 *
 * Input Parameters:
 *   arg - A reference to the class instance to be destroyed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int usbhost_mouse_poll(int argc, char *argv[])
{
  FAR struct usbhost_state_s *priv;
  FAR struct usbhost_hubport_s *hport;
  FAR struct usbhid_mousereport_s *rpt;
  irqstate_t flags;
#ifndef CONFIG_HIDMOUSE_TSCIF
  uint8_t buttons;
#endif
#if defined(CONFIG_DEBUG_USB) && defined(CONFIG_DEBUG_INFO)
  unsigned int npolls = 0;
#endif
  unsigned int nerrors = 0;
  ssize_t nbytes;
  int ret = OK;

  uinfo("Started\n");

  /* Synchronize with the start-up logic.  Get the private instance, re-start
   * the start-up logic, and wait a bit to make sure that all of the class
   * creation logic has a chance to run to completion.
   *
   * NOTE: that the reference count is *not* incremented here.  When the
   * driver structure was created, it was created with a reference count of
   * one.  This thread is responsible for that count.  The count will be
   * decremented when this thread exits.
   */

  priv = g_priv;
  DEBUGASSERT(priv != NULL  && priv->usbclass.hport != NULL);
  hport = priv->usbclass.hport;

  priv->polling = true;
  nxsem_post(&g_syncsem);
  nxsig_sleep(1);

  /* Loop here until the device is disconnected */

  uinfo("Entering poll loop\n");

  while (!priv->disconnected)
    {
      /* Read the next mouse report.  We will stall here until the mouse
       * sends data.
       */

      nbytes = DRVR_TRANSFER(hport->drvr, priv->epin,
                             priv->tbuffer, priv->tbuflen);

      /* Check for errors -- Bail if an excessive number of consecutive
       * errors are encountered.
       */

      if (nbytes < 0)
        {
          /* If DRVR_TRANSFER() returns EAGAIN, that simply means that
           * the devices was not ready and has NAK'ed the transfer.  That
           * should not be treated as an error (unless it persists for a
           * long time).
           */

          uerr("ERROR: DRVR_TRANSFER returned: %d/%u\n",
               (int)nbytes, nerrors);

          if (nbytes != -EAGAIN)
            {
              if (++nerrors > 200)
                {
                  uerr("  Too many errors... aborting: %d\n", nerrors);
                  ret = (int)nbytes;
                  break;
                }
            }
        }

      /* The report was received correctly. */

      else
        {
          /* Success, reset the error counter */

          nerrors = 0;

          /* Ignore the mouse data if no task has opened the driver. */

          if (priv->open)
            {
              /* Get exclusive access to the mouse state data */

              ret = nxmutex_lock(&priv->lock);
              if (ret < 0)
                {
                  /* Break out and disconnect if the thread is canceled. */

                  break;
                }

              /* Get the HID mouse report */

              rpt = (FAR struct usbhid_mousereport_s *)priv->tbuffer;

              /* Get the updated mouse position */

              usbhost_position(priv, rpt);

#ifdef CONFIG_HIDMOUSE_TSCIF
              /* Execute the touchscreen state machine */

              if (usbhost_touchscreen(priv, rpt))
#else
              /* Check if any buttons have changed.  If so, then report the
               * new mouse data.
               *
               * If not, then perform a thresholding operation so that the
               * results will be more stable.  If the difference from the
               * last sample is small, then ignore the event.
               */

              buttons = rpt->buttons & USBHID_MOUSEIN_BUTTON_MASK;
              if (buttons != priv->buttons || usbhost_threshold(priv))
#endif
                {
                  /* We get here when either there is a meaningful button
                   * change and/or a significant movement of the mouse.  We
                   * are going to report the mouse event.
                   *
                   * Snap to the new x/y position for subsequent
                   * thresholding
                   */

                  priv->xlast = priv->xaccum;
                  priv->ylast = priv->yaccum;
#ifdef CONFIG_INPUT_MOUSE_WHEEL
                  priv->wlast = priv->waccum;
#endif
                  /* Update the sample X/Y positions */

                  priv->sample.x = b16toi(priv->xaccum);
                  priv->sample.y = b16toi(priv->yaccum);
#ifdef CONFIG_INPUT_MOUSE_WHEEL
                  priv->sample.wheel = b16toi(priv->waccum);
#endif

#ifdef CONFIG_HIDMOUSE_TSCIF
                  /* The X/Y positional data is now valid */

                  priv->sample.valid = true;

                  /* Indicate the availability of new sample data
                   * for this ID
                   */

                  priv->sample.id = priv->id;
#else
                  /* Report and remember the new button state */

                  priv->sample.buttons = buttons;
                  priv->buttons = buttons;
#endif
                  priv->valid = true;

                  /* Notify any waiters that new HIDMOUSE data is available */

                  usbhost_notify(priv);
                }

              /* Release our lock on the state structure */

              nxmutex_unlock(&priv->lock);
            }
        }

      /* If USB debug is on, then provide some periodic indication that
       * polling is still happening.
       */

#if defined(CONFIG_DEBUG_USB) && defined(CONFIG_DEBUG_INFO)
      npolls++;
      if ((npolls & 31) == 0)
        {
          uinfo("Still polling: %d\n", npolls);
        }
#endif
    }

  /* We get here when the driver is removed, when too many errors have
   * been encountered, or the parent thread has been canceled.
   *
   * Make sure that we have exclusive access to the private data structure.
   * There may now be other tasks with the character driver open and actively
   * trying to interact with the class driver.
   */

  nxmutex_lock(&priv->lock);

  /* Indicate that we are no longer running and decrement the reference
   * count held by this thread.  If there are no other users of the class,
   * we can destroy it now.  Otherwise, we have to wait until the all
   * of the file descriptors are closed.
   */

  uinfo("Mouse removed, polling halted\n");

  flags = enter_critical_section();
  priv->polling = false;

  /* Decrement the reference count held by this thread. */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  /* There are two possibilities:
   * 1) The reference count is greater than zero.  This means that there
   *    are still open references to the mouse driver.  In this case
   *    we need to wait until usbhost_close() is called and all of the
   *    open driver references are decremented.  Then usbhost_destroy() can
   *    be called from usbhost_close().
   * 2) The reference count is now zero.  This means that there are no
   *    further open references and we can call usbhost_destroy() now.
   */

  if (priv->crefs < 1)
    {
      /* Unregister the driver and destroy the instance (while we hold
       * the semaphore!)
       */

      usbhost_destroy(priv);
    }
  else
    {
      /* No, we will destroy the driver instance when it is final open
       * reference is closed
       */

      nxmutex_unlock(&priv->lock);
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: usbhost_sample
 *
 * Description:
 *   Check if mouse data is available
 *
 ****************************************************************************/

static int usbhost_sample(FAR struct usbhost_state_s *priv,
                          FAR struct mouse_sample_s *sample)
{
  irqstate_t flags;
  int ret = -EAGAIN;

  /* Interrupts me be disabled when this is called to (1) prevent posting
   * of semaphores from interrupt handlers, and (2) to prevent sampled data
   * from changing until it has been reported.
   */

  flags = enter_critical_section();

  /* Is there new mouse data available? */

  if (priv->valid)
    {
      /* Return a copy of the sampled data. */

      memcpy(sample, &priv->sample, sizeof(struct mouse_sample_s));

#ifdef CONFIG_HIDMOUSE_TSCIF
      /* Now manage state transitions */

      if (sample->event == BUTTON_RELEASED)
        {
          /* Next.. No button press.  Increment the ID so that next event ID
           * will be unique.  X/Y positions are no longer valid.
           */

          priv->sample.event = BUTTON_NONE;
          priv->id++;
          priv->sample.valid = false;
        }
      else if (sample->event == BUTTON_PRESSED)
        {
          /* First report -- next report will be a movement */

          priv->sample.event = BUTTON_MOVE;
        }

#endif

      /* The sample has been reported and is no longer valid */

      priv->valid = false;
      ret = OK;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: usbhost_waitsample
 *
 * Description:
 *   Wait for the next valid mouse sample
 *
 * Input Parameters:
 *    priv   - HID mouse state instance
 *    sample - The location to regurn the sample data
 *
 ****************************************************************************/

static int usbhost_waitsample(FAR struct usbhost_state_s *priv,
                              FAR struct mouse_sample_s *sample)
{
  irqstate_t flags;
  int ret;

  /* Interrupts me be disabled when this is called to (1) prevent posting
   * of semaphores from interrupt handlers, and (2) to prevent sampled data
   * from changing until it has been reported.
   *
   * In addition, we will also disable pre-emption to prevent other threads
   * from getting control while we muck with the semaphores.
   */

  sched_lock();
  flags = enter_critical_section();

  /* Now release the semaphore that manages mutually exclusive access to
   * the device structure.  This may cause other tasks to become ready to
   * run, but they cannot run yet because pre-emption is disabled.
   */

  nxmutex_unlock(&priv->lock);

  /* Try to get the a sample... if we cannot, then wait on the semaphore
   * that is posted when new sample data is available.
   */

  while (usbhost_sample(priv, sample) < 0)
    {
      /* Wait for a change in the HIDMOUSE state */

      iinfo("Waiting..\n");
      priv->nwaiters++;
      ret = nxsem_wait(&priv->waitsem);
      priv->nwaiters--;

      if (ret < 0)
        {
          ierr("ERROR: nxsem_wait: %d\n", ret);
          goto errout;
        }

      /* Did the mouse become disconnected while we were waiting */

      if (priv->disconnected)
        {
          ret = -ENODEV;
          goto errout;
        }
    }

  iinfo("Sampled\n");

  /* Re-acquire the semaphore that manages mutually exclusive access to
   * the device structure.  We may have to wait here.  But we have our
   * sample. Interrupts and pre-emption will be re-enabled while we wait.
   */

  ret = nxmutex_lock(&priv->lock);

errout:
  /* Then re-enable interrupts.  We might get interrupt here and there
   * could be a new sample.  But no new threads will run because we still
   * have pre-emption disabled.
   */

  leave_critical_section(flags);

  /* Restore pre-emption.  We might get suspended here but that is okay
   * because we already have our sample.  Note:  this means that if there
   * were two threads reading from the HIDMOUSE for some reason, the data
   * might be read out of order.
   */

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: usbhost_cfgdesc
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   priv - The USB host class instance.
 *   configdesc - A pointer to a uint8_t buffer container the configuration
 *     descriptor.
 *   desclen - The length in bytes of the configuration descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static inline int usbhost_cfgdesc(FAR struct usbhost_state_s *priv,
                                  FAR const uint8_t *configdesc, int desclen)
{
  FAR struct usbhost_hubport_s *hport;
  FAR struct usb_cfgdesc_s *cfgdesc;
  FAR struct usb_desc_s *desc;
  FAR struct usbhost_epdesc_s epindesc;
  int remaining;
  uint8_t found = 0;
  bool done = false;
  int ret;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL &&
              configdesc != NULL && desclen >= sizeof(struct usb_cfgdesc_s));
  hport = priv->usbclass.hport;

  /* Keep the compiler from complaining about uninitialized variables */

  memset(&epindesc, 0, sizeof(struct usbhost_epdesc_s));

  /* Verify that we were passed a configuration descriptor */

  cfgdesc = (FAR struct usb_cfgdesc_s *)configdesc;
  if (cfgdesc->type != USB_DESC_TYPE_CONFIG)
    {
      return -EINVAL;
    }

  /* Get the total length of the configuration descriptor (little endian).
   * It might be a good check to get the number of interfaces here too.
   */

  remaining = (int)usbhost_getle16(cfgdesc->totallen);

  /* Skip to the next entry descriptor */

  configdesc += cfgdesc->len;
  remaining  -= cfgdesc->len;

  /* Loop where there are more descriptors to examine */

  while (remaining >= sizeof(struct usb_desc_s) && !done)
    {
      /* What is the next descriptor? */

      desc = (FAR struct usb_desc_s *)configdesc;
      switch (desc->type)
        {
        /* Interface descriptor. We really should get the number of endpoints
         * from this descriptor too.
         */

        case USB_DESC_TYPE_INTERFACE:
          {
            uinfo("Interface descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_IFDESC);

            /* Did we already find what we needed from
             * a preceding interface?
             */

            if ((found & USBHOST_ALLFOUND) == USBHOST_ALLFOUND)
              {
                /* Yes.. then break out of the loop and use the preceding
                 * interface.
                 */

                done = true;
              }
            else
              {
                /* Otherwise, discard any endpoints previously found */

                found = USBHOST_IFFOUND;
              }
          }
          break;

        /* HID descriptor */

        case USBHID_DESCTYPE_HID:
            uinfo("HID descriptor\n");
            break;

        /* Endpoint descriptor.  We expect one or two interrupt endpoints,
         * a required IN endpoint and an optional OUT endpoint.
         */

        case USB_DESC_TYPE_ENDPOINT:
          {
            FAR struct usb_epdesc_s *epdesc =
              (FAR struct usb_epdesc_s *)configdesc;

            uinfo("Endpoint descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_EPDESC);

            /* Check for an interrupt endpoint. */

            if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) ==
                USB_EP_ATTR_XFER_INT)
              {
                /* Yes.. it is a interrupt endpoint.  IN or OUT? */

                if (USB_ISEPIN(epdesc->addr))
                  {
                    /* It is an interrupt IN endpoint.  There should be only
                     * one interrupt IN endpoint.
                     */

                    if ((found & USBHOST_EPINFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know what
                         * to do with this.
                         */

                        return -EINVAL;
                      }

                    found |= USBHOST_EPINFOUND;

                    /* Save the interrupt IN endpoint information */

                    epindesc.hport        = hport;
                    epindesc.addr         = epdesc->addr &
                                            USB_EP_ADDR_NUMBER_MASK;
                    epindesc.in           = true;
                    epindesc.xfrtype      = USB_EP_ATTR_XFER_INT;
                    epindesc.interval     = epdesc->interval;
                    epindesc.mxpacketsize =
                      usbhost_getle16(epdesc->mxpacketsize);

                    uinfo("Interrupt IN EP addr:%d mxpacketsize:%d\n",
                          epindesc.addr, epindesc.mxpacketsize);
                  }
              }
          }
          break;

        /* Other descriptors are just ignored for now */

        default:
          uinfo("Other descriptor: %d\n", desc->type);
          break;
        }

      /* What we found everything that we are going to find? */

      if (found == USBHOST_ALLFOUND)
        {
          /* Yes.. then break out of the loop
           * and use the preceding interface
           */

          done = true;
        }

      /* Increment the address of the next descriptor */

      configdesc += desc->len;
      remaining  -= desc->len;
    }

  /* Sanity checking... did we find all of things that we need? */

  if ((found & USBHOST_ALLFOUND) != USBHOST_ALLFOUND)
    {
      uerr("ERROR: Found IF:%s EPIN:%s\n",
           (found & USBHOST_IFFOUND) != 0  ? "YES" : "NO",
           (found & USBHOST_EPINFOUND) != 0 ? "YES" : "NO");
      return -EINVAL;
    }

  /* We are good... Allocate the interrupt IN endpoint. */

  ret = DRVR_EPALLOC(hport->drvr, &epindesc, &priv->epin);
  if (ret < 0)
    {
      uerr("ERROR: Failed to allocate interrupt IN endpoint\n");
      return ret;
    }

  uinfo("Endpoint allocated\n");
  return OK;
}

/****************************************************************************
 * Name: usbhost_devinit
 *
 * Description:
 *   The USB device has been successfully connected.  This completes the
 *   initialization operations.  It is first called after the
 *   configuration descriptor has been received.
 *
 *   This function is called from the connect() method.  This function always
 *   executes on the thread of the caller of connect().
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline int usbhost_devinit(FAR struct usbhost_state_s *priv)
{
  char devname[DEV_NAMELEN];
  int ret;

  /* Set aside a transfer buffer for exclusive
   * use by the mouse class driver
   */

  ret = usbhost_tdalloc(priv);
  if (ret < 0)
    {
      uerr("ERROR: Failed to allocate transfer buffer\n");
      return ret;
    }

  /* Increment the reference count.  This will prevent usbhost_destroy() from
   * being called asynchronously if the device is removed.
   */

  priv->crefs++;
  DEBUGASSERT(priv->crefs == 2);

  /* Start a worker task to poll the USB device.  It would be nice to use
   * the NuttX worker thread to do this, but this task needs to wait for
   * events and activities on the worker thread should not involve
   * significant waiting.  Having a dedicated thread is more efficient in
   * this sense, but requires more memory resources, primarily for the
   * dedicated stack (CONFIG_HIDMOUSE_STACKSIZE).
   */

  uinfo("Start poll task\n");

  /* The inputs to a task started by kthread_create() are very awkward for
   * this purpose.  They are really designed for command line tasks
   * (argc/argv).  So the following is kludge pass binary data when the
   * mouse poll task is started.
   *
   * First, make sure we have exclusive access to g_priv (what is the
   * likelihood of this being used?  About zero, but we protect it anyway).
   */

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      usbhost_tdfree(priv);
      goto errout;
    }

  g_priv = priv;

  ret = kthread_create("mouse", CONFIG_HIDMOUSE_DEFPRIO,
                       CONFIG_HIDMOUSE_STACKSIZE, usbhost_mouse_poll, NULL);
  if (ret < 0)
    {
      /* Failed to started the poll thread...
       * probably due to memory resources
       */

      nxmutex_unlock(&g_lock);
      goto errout;
    }

  priv->pollpid = (pid_t)ret;

  /* Now wait for the poll task to get properly initialized */

  ret = nxsem_wait_uninterruptible(&g_syncsem);
  nxmutex_unlock(&g_lock);

  if (ret < 0)
    {
      goto errout;
    }

  /* Register the driver */

  uinfo("Register driver\n");
  usbhost_mkdevname(priv, devname);
  ret = register_driver(devname, &g_hidmouse_fops, 0666, priv);

  /* We now have to be concerned about asynchronous modification of crefs
   * because the driver has been registered.
   */

errout:
  nxmutex_lock(&priv->lock);
  priv->crefs--;
  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: usbhost_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   val - A pointer to the first byte of the little endian value.
 *
 * Returned Value:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

static inline uint16_t usbhost_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: usbhost_tdalloc
 *
 * Description:
 *   Allocate transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Value:
 *   On success, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int usbhost_tdalloc(FAR struct usbhost_state_s *priv)
{
  FAR struct usbhost_hubport_s *hport;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL &&
              priv->tbuffer == NULL);
  hport = priv->usbclass.hport;

  return DRVR_ALLOC(hport->drvr, &priv->tbuffer, &priv->tbuflen);
}

/****************************************************************************
 * Name: usbhost_tdfree
 *
 * Description:
 *   Free transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Value:
 *   On success, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int usbhost_tdfree(FAR struct usbhost_state_s *priv)
{
  FAR struct usbhost_hubport_s *hport;
  int result = OK;
  DEBUGASSERT(priv);

  if (priv->tbuffer)
    {
      DEBUGASSERT(priv->usbclass.hport);
      hport         = priv->usbclass.hport;
      result        = DRVR_FREE(hport->drvr, priv->tbuffer);
      priv->tbuffer = NULL;
      priv->tbuflen = 0;
    }

  return result;
}

/****************************************************************************
 * struct usbhost_registry_s methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_create
 *
 * Description:
 *   This function implements the create() method of struct
 *   usbhost_registry_s.  The create() method is a callback into the class
 *   implementation.  It is used to (1) create a new instance of the USB
 *   host class state and to (2) bind a USB host driver "session" to the
 *   class instance.  Use of this create() method will support environments
 *   where there may be multiple USB ports and multiple USB devices
 *   simultaneously connected.
 *
 * Input Parameters:
 *   hport - The hub port that manages the new class instance.
 *   id - In the case where the device supports multiple base classes,
 *     subclasses, or protocols, this specifies which to configure for.
 *
 * Returned Value:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s that can be used by the USB host driver to communicate
 *   with the USB host class.  NULL is returned on failure; this function
 *   will fail only if the hport input parameter is NULL or if there are
 *   insufficient resources to create another USB host class instance.
 *
 ****************************************************************************/

static FAR struct usbhost_class_s *
usbhost_create(FAR struct usbhost_hubport_s *hport,
               FAR const struct usbhost_id_s *id)
{
  FAR struct usbhost_state_s *priv;

  /* Allocate a USB host class instance */

  priv = usbhost_allocclass();
  if (priv)
    {
      /* Initialize the allocated storage class instance */

      memset(priv, 0, sizeof(struct usbhost_state_s));

      /* Assign a device number to this class instance */

      if (usbhost_allocdevno(priv) == OK)
        {
          /* Initialize class method function pointers */

          priv->usbclass.hport        = hport;
          priv->usbclass.connect      = usbhost_connect;
          priv->usbclass.disconnected = usbhost_disconnected;

          /* The initial reference count is 1... One reference is held by
           * the driver's usbhost_mouse_poll() task.
           */

          priv->crefs = 1;

          /* Initialize mutex & semaphores */

          nxmutex_init(&priv->lock);
          nxsem_init(&priv->waitsem, 0, 0);

          /* Return the instance of the USB mouse class driver */

          return &priv->usbclass;
        }
    }

  /* An error occurred. Free the allocation and return NULL on all failures */

  if (priv)
    {
      usbhost_freeclass(priv);
    }

  return NULL;
}

/****************************************************************************
 * struct usbhost_class_s methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_connect
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   usbclass - The USB host class entry previously obtained from a call to
 *     create().
 *   configdesc - A pointer to a uint8_t buffer container the configuration
 *     descriptor.
 *   desclen - The length in bytes of the configuration descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 *   NOTE that the class instance remains valid upon return with a failure.
 *   It is the responsibility of the higher level enumeration logic to call
 *   CLASS_DISCONNECTED to free up the class driver resources.
 *
 * Assumptions:
 *   - This function will *not* be called from an interrupt handler.
 *   - If this function returns an error, the USB host controller driver
 *     must call to DISCONNECTED method to recover from the error
 *
 ****************************************************************************/

static int usbhost_connect(FAR struct usbhost_class_s *usbclass,
                           FAR const uint8_t *configdesc, int desclen)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)usbclass;
  int ret;

  DEBUGASSERT(priv != NULL && configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

  /* Parse the configuration descriptor to get the endpoints */

  ret = usbhost_cfgdesc(priv, configdesc, desclen);
  if (ret < 0)
    {
      uerr("ERROR: usbhost_cfgdesc() failed: %d\n", ret);
    }
  else
    {
      /* Now configure the device and register the NuttX driver */

      ret = usbhost_devinit(priv);
      if (ret < 0)
        {
          uerr("ERROR: usbhost_devinit() failed: %d\n", ret);
        }
    }

  /* ERROR handling:  Do nothing. If we return and error during connection,
   * the driver is required to call the DISCONNECT method.  Possibilities:
   *
   * - Failure occurred before the mouse poll task was started successfully.
   *   In this case, the disconnection will have to be handled on the worker
   *   task.
   * - Failure occurred after the mouse poll task was started successfully.
   *   In this case, the disconnection can be performed on the mouse poll
   *   thread.
   */

  return ret;
}

/****************************************************************************
 * Name: usbhost_disconnected
 *
 * Description:
 *   This function implements the disconnected() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to inform the class that the USB device has
 *   been disconnected.
 *
 * Input Parameters:
 *   usbclass - The USB host class entry previously obtained from a call to
 *     create().
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

static int usbhost_disconnected(FAR struct usbhost_class_s *usbclass)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)usbclass;
  int i;

  DEBUGASSERT(priv != NULL);

  /* Set an indication to any users of the mouse device that the device
   * is no longer available.
   */

  priv->disconnected = true;
  uinfo("Disconnected\n");

  /* Are there a thread(s) waiting for mouse data that will never come? */

  for (i = 0; i < priv->nwaiters; i++)
    {
      /* Yes.. wake them up */

      nxsem_post(&priv->waitsem);
    }

  /* Possibilities:
   *
   * - Failure occurred before the mouse poll task was started successfully.
   *   In this case, the disconnection will have to be handled on the worker
   *   task.
   * - Failure occurred after the mouse poll task was started successfully.
   *   In this case, the disconnection can be performed on the mouse poll
   *   thread.
   */

  if (priv->polling)
    {
      /* The polling task is still alive. Signal the mouse polling task.
       * When that task wakes up, it will decrement the reference count and,
       * perhaps, destroy the class instance.  Then it will exit.
       */

      nxsig_kill(priv->pollpid, SIGALRM);
    }
  else
    {
      /* In the case where the failure occurs before the polling task was
       * started.  Now what?  We are probably executing from an interrupt
       * handler here.  We will use the worker thread.  This is kind of
       * wasteful and begs for a re-design.
       */

      DEBUGASSERT(priv->work.worker == NULL);
      work_queue(HPWORK, &priv->work, usbhost_destroy, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Character driver methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int usbhost_open(FAR struct file *filep)
{
  FAR struct inode           *inode;
  FAR struct usbhost_state_s *priv;
  irqstate_t flags;
  int ret;

  uinfo("Entry\n");
  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv && priv->crefs > 0 && priv->crefs < USBHOST_MAX_CREFS);
  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if the mouse device is still connected.  We need to disable
   * interrupts momentarily to assure that there are no asynchronous
   * disconnect events.
   */

  flags = enter_critical_section();
  if (priv->disconnected)
    {
      /* No... the driver is no longer bound to the class.  That means that
       * the USB storage device is no longer connected.  Refuse any further
       * attempts to open the driver.
       */

      ret = -ENODEV;
    }
  else
    {
      /* Was the driver previously open?  We need to perform special
       * initialization on the first time that the driver is opened.
       */

      if (!priv->open)
        {
          /* Set the thresholding values so that the first button press
           * will be reported.
           */

          priv->xlast = INVALID_POSITION_B16;
          priv->ylast = INVALID_POSITION_B16;
#ifdef CONFIG_INPUT_MOUSE_WHEEL
          priv->wlast = INVALID_POSITION_B16;
#endif
          /* Set the reported position to the center of the range */

          priv->xaccum = (HIDMOUSE_XMAX_B16 >> 1);
          priv->yaccum = (HIDMOUSE_YMAX_B16 >> 1);
        }

      /* Otherwise, just increment the reference count on the driver */

      priv->crefs++;
      priv->open = true;
      ret        = OK;
    }

  leave_critical_section(flags);

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: usbhost_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int usbhost_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct usbhost_state_s *priv;
  irqstate_t flags;
  int ret;

  uinfo("Entry\n");
  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Decrement the reference count on the driver */

  DEBUGASSERT(priv->crefs >= 1);
  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* We need to disable interrupts momentarily to assure that there are no
   * asynchronous poll or disconnect events.
   */

  flags = enter_critical_section();
  priv->crefs--;

  /* Check if the USB mouse device is still connected.  If the device is
   * no longer connected, then unregister the driver and free the driver
   * class instance.
   */

  if (priv->disconnected)
    {
      /* If the reference count is one or less then there are two
       * possibilities:
       *
       * 1) It might be zero meaning that the polling thread has already
       *    exited and decremented its count.
       * 2) If might be one meaning either that (a) the polling thread is
       *    still running and still holds a count, or (b) the polling thread
       *    has exited, but there is still an outstanding open reference.
       */

      if (priv->crefs == 0 || (priv->crefs == 1 && priv->polling))
        {
          /* Yes.. In either case, then the driver is no longer open */

          priv->open = false;

          /* Check if the USB keyboard device is still connected. */

          if (priv->crefs == 0)
            {
              /* The polling thread is no longer running */

              DEBUGASSERT(!priv->polling);

              /* If the device is no longer connected, unregister the driver
               * and free the driver class instance.
               */

              usbhost_destroy(priv);

              /* Skip giving the semaphore... it is no longer valid */

              leave_critical_section(flags);
              return OK;
            }
          else /* if (priv->crefs == 1) */
            {
              /* The polling thread is still running.  Signal it so that it
               * will wake up and call usbhost_destroy().  The particular
               * signal that we use does not matter in this case.
               */

              nxsig_kill(priv->pollpid, SIGALRM);
            }
        }
    }

  nxmutex_unlock(&priv->lock);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: usbhost_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t usbhost_read(FAR struct file *filep, FAR char *buffer,
                            size_t len)
{
  FAR struct inode           *inode;
  FAR struct usbhost_state_s *priv;
#ifdef CONFIG_HIDMOUSE_TSCIF
  FAR struct touch_sample_s  *report;
#else
  FAR struct mouse_report_s  *report;
#endif
  struct mouse_sample_s       sample;
  int                         ret;

  uinfo("Entry\n");
  DEBUGASSERT(filep && filep->f_inode && buffer);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv && priv->crefs > 0 && priv->crefs < USBHOST_MAX_CREFS);
  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if the mouse is still connected.  We need to disable interrupts
   * momentarily to assure that there are no asynchronous disconnect events.
   */

  if (priv->disconnected)
    {
      /* No... the driver is no longer bound to the class.  That means that
       * the USB mouse is no longer connected.  Refuse any further attempts
       * to access the driver.
       */

      ret = -ENODEV;
      goto errout;
    }

  /* Try to read sample data. */

  ret = usbhost_sample(priv, &sample);
  if (ret < 0)
    {
      /* Sample data is not available now.  We would ave to wait to get
       * receive sample data.  If the user has specified the O_NONBLOCK
       * option, then just return an error.
       */

      if (filep->f_oflags & O_NONBLOCK)
        {
          /* Yes.. then return a failure */

          ret = -EAGAIN;
          goto errout;
        }

      /* Wait for sample data */

      ret = usbhost_waitsample(priv, &sample);
      if (ret < 0)
        {
          /* We might have been awakened by a signal */

          ierr("ERROR: usbhost_waitsample: %d\n", ret);
          goto errout;
        }
    }

  /* We now have sampled HIDMOUSE data that we can report to the caller.  */

#ifdef CONFIG_HIDMOUSE_TSCIF
  report = (FAR struct touch_sample_s *)buffer;
  memset(report, 0, SIZEOF_TOUCH_SAMPLE_S(1));

  report->npoints            = 1;
  report->point[0].id        = sample.id;
  report->point[0].x         = sample.x;
  report->point[0].y         = sample.y;

  /* Report the appropriate flags */

  if (sample.event == BUTTON_RELEASED)
    {
      /* But was released.  Is the positional data valid?  This is
       * important to know because the release will be sent to the
       * window based on its last positional data.
       */

      if (sample.valid)
        {
          report->point[0].flags  = TOUCH_UP | TOUCH_ID_VALID |
                                    TOUCH_POS_VALID;
        }
      else
        {
          report->point[0].flags  = TOUCH_UP | TOUCH_ID_VALID;
        }
    }
  else if (sample.event == BUTTON_PRESSED)
    {
      /* First event */

      report->point[0].flags  = TOUCH_DOWN | TOUCH_ID_VALID |
                                TOUCH_POS_VALID;
    }
  else /* if (sample->event == BUTTON_MOVE) */
    {
      /* Movement of the same event */

      report->point[0].flags  = TOUCH_MOVE | TOUCH_ID_VALID |
                                TOUCH_POS_VALID;
    }

  iinfo("  id:      %d\n", report->point[0].id);
  iinfo("  flags:   %02x\n", report->point[0].flags);
  iinfo("  x:       %d\n", report->point[0].x);
  iinfo("  y:       %d\n", report->point[0].y);

  ret = SIZEOF_TOUCH_SAMPLE_S(1);
#else
  report = (FAR struct mouse_report_s *)buffer;
  memset(report, 0, sizeof(struct mouse_report_s));

  report->buttons = sample.buttons;
  report->x       = sample.x;
  report->y       = sample.y;
#ifdef CONFIG_INPUT_MOUSE_WHEEL
  report->wheel   = sample.wheel;
#endif

  ret = sizeof(struct mouse_report_s);
#endif

errout:
  nxmutex_unlock(&priv->lock);
  iinfo("Returning: %d\n", ret);
  return (ssize_t)ret;
}

/****************************************************************************
 * Name: usbhost_write
 *
 * Description:
 *   Standard character driver write method.
 *
 ****************************************************************************/

static ssize_t usbhost_write(FAR struct file *filep, FAR const char *buffer,
                             size_t len)
{
  /* We won't try to write to the mouse */

  return -ENOSYS;
}

/****************************************************************************
 * Name: usbhost_poll
 *
 * Description:
 *   Standard character driver poll method.
 *
 ****************************************************************************/

static int usbhost_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode           *inode;
  FAR struct usbhost_state_s *priv;
  int                         ret;
  int                         i;

  uinfo("Entry\n");
  DEBUGASSERT(filep && filep->f_inode && fds);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv);
  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if the mouse is still connected.  We need to disable interrupts
   * momentarily to assure that there are no asynchronous disconnect events.
   */

  if (priv->disconnected)
    {
      /* No... the driver is no longer bound to the class.  That means that
       * the USB mouse is no longer connected.  Refuse any further attempts
       * to access the driver.
       */

      ret = -ENODEV;
    }
  else if (setup)
    {
      /* This is a request to set up the poll.  Find an available slot for
       * the poll structure reference
       */

      for (i = 0; i < CONFIG_HIDMOUSE_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_HIDMOUSE_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? Notify
       * the POLLIN event if there is buffered mouse data.
       */

      if (priv->valid)
        {
          poll_notify(priv->fds, CONFIG_HIDMOUSE_NPOLLWAITERS, POLLIN);
        }
    }
  else
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot);

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
    }

errout:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_mouse_init
 *
 * Description:
 *   Initialize the USB storage HID mouse class driver.  This function
 *   should be called be platform-specific code in order to initialize and
 *   register support for the USB host HID mouse class device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

int usbhost_mouse_init(void)
{
  /* Advertise our availability to support (certain) mouse devices */

  return usbhost_registerclass(&g_hidmouse);
}

#endif /* CONFIG_USBHOST)&& !CONFIG_USBHOST_INT_DISABLE */
