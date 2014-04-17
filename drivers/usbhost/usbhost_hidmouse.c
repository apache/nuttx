/****************************************************************************
 * drivers/usbhost/usbhost_hidmouse.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <poll.h>
#include <semaphore.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <fixedmath.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/wqueue.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/hid.h>

#ifdef CONFIG_HIDMOUSE_TSCIF
#  include <nuttx/input/touchscreen.h>
#else
#  include <nuttx/input/mouse.h>
#endif

/* Don't compile if prerequisites are not met */

#if defined(CONFIG_USBHOST) && !defined(CONFIG_USBHOST_INT_DISABLE) && CONFIG_NFILE_DESCRIPTORS > 0

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

/* Signals must not be disabled as they are needed by usleep.  Need to have
 * CONFIG_DISABLE_SIGNALS=n
 */

#ifdef CONFIG_DISABLE_SIGNALS
#  warning "Signal support is required (CONFIG_DISABLE_SIGNALS)"
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
#  undef CONFIG_MOUSE_WHEEL
#endif

#ifdef CONFIG_MOUSE_WHEEL

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

#endif /* CONFIG_MOUSE_WHEEL */

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
#  define DEV_NAMELEN       13
#else
#  define DEV_FORMAT        "/dev/mouse%d"
#  define DEV_NAMELEN       13
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
#  undef  idbg
#  define idbg    udbg
#  undef  illdbg
#  define illdbg  ulldbg
#  undef  ivdbg
#  define ivdbg   uvdbg
#  undef  illvdbg
#  define illvdbg ullvdbg
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
#ifdef CONFIG_MOUSE_WHEEL
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

  struct usbhost_class_s  class;

  /* This is an instance of the USB host driver bound to this class instance */

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
  sem_t                   exclsem;      /* Used to maintain mutual exclusive access */
  sem_t                   waitsem;      /* Used to wait for mouse data */
  FAR uint8_t            *tbuffer;      /* The allocated transfer buffer */
  b16_t                   xaccum;       /* Current integrated X position */
  b16_t                   yaccum;       /* Current integrated Y position */
  b16_t                   xlast;        /* Last reported X position */
  b16_t                   ylast;        /* Last reported Y position */
#ifdef CONFIG_MOUSE_WHEEL
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

#ifndef CONFIG_DISABLE_POLL
  struct pollfd *fds[CONFIG_HIDMOUSE_NPOLLWAITERS];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Semaphores */

static void usbhost_takesem(sem_t *sem);
#define usbhost_givesem(s) sem_post(s);

/* Polling support */

#ifndef CONFIG_DISABLE_POLL
static void usbhost_pollnotify(FAR struct usbhost_state_s *dev);
#else
#  define usbhost_pollnotify(dev)
#endif

/* Memory allocation services */

static inline FAR struct usbhost_state_s *usbhost_allocclass(void);
static inline void usbhost_freeclass(FAR struct usbhost_state_s *class);

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
                                  int desclen, uint8_t funcaddr);
static inline int usbhost_devinit(FAR struct usbhost_state_s *priv);

/* (Little Endian) Data helpers */

static inline uint16_t usbhost_getle16(const uint8_t *val);
static inline void usbhost_putle16(uint8_t *dest, uint16_t val);
static inline uint32_t usbhost_getle32(const uint8_t *val);
#if 0 /* Not used */
static void usbhost_putle32(uint8_t *dest, uint32_t val);
#endif

/* Transfer descriptor memory management */

static inline int usbhost_tdalloc(FAR struct usbhost_state_s *priv);
static inline int usbhost_tdfree(FAR struct usbhost_state_s *priv);

/* struct usbhost_registry_s methods */

static struct usbhost_class_s *usbhost_create(FAR struct usbhost_driver_s *drvr,
                                              FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int usbhost_connect(FAR struct usbhost_class_s *class,
                           FAR const uint8_t *configdesc, int desclen,
                           uint8_t funcaddr);
static int usbhost_disconnected(FAR struct usbhost_class_s *class);

/* Driver methods.  We export the mouse as a standard character driver */

static int usbhost_open(FAR struct file *filep);
static int usbhost_close(FAR struct file *filep);
static ssize_t usbhost_read(FAR struct file *filep,
                            FAR char *buffer, size_t len);
static ssize_t usbhost_write(FAR struct file *filep,
                             FAR const char *buffer, size_t len);
#ifndef CONFIG_DISABLE_POLL
static int usbhost_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure provides the registry entry ID information that will  be
 * used to associate the USB host mouse class driver to a connected USB
 * device.
 */

static const const struct usbhost_id_s g_hidmouse_id =
{
  USB_CLASS_HID,           /* base     */
  USBHID_SUBCLASS_BOOTIF,  /* subclass */
  USBHID_PROTOCOL_MOUSE,   /* proto    */
  0,                       /* vid      */
  0                        /* pid      */
};

/* This is the USB host storage class's registry entry */

static struct usbhost_registry_s g_hidmouse =
{
  NULL,                    /* flink     */
  usbhost_create,          /* create    */
  1,                       /* nids      */
  &g_hidmouse_id           /* id[]      */
};

static const struct file_operations g_hidmouse_fops =
{
  usbhost_open,            /* open      */
  usbhost_close,           /* close     */
  usbhost_read,            /* read      */
  usbhost_write,           /* write     */
  0,                       /* seek      */
  0                        /* ioctl     */
#ifndef CONFIG_DISABLE_POLL
  , usbhost_poll           /* poll      */
#endif
};

/* This is a bitmap that is used to allocate device names /dev/mouse0-31. */

static uint32_t                g_devinuse;

/* The following are used to managed the class creation operation */

static sem_t                   g_exclsem; /* For mutually exclusive thread creation */
static sem_t                   g_syncsem; /* Thread data passing interlock */
static struct usbhost_state_s *g_priv;    /* Data passed to thread */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_takesem
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 ****************************************************************************/

static void usbhost_takesem(sem_t *sem)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: usbhost_pollnotify
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static void usbhost_pollnotify(FAR struct usbhost_state_s *priv)
{
  int i;

  for (i = 0; i < CONFIG_HIDMOUSE_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= (fds->events & POLLIN);
          if (fds->revents != 0)
            {
              uvdbg("Report events: %02x\n", fds->revents);
              sem_post(fds->sem);
            }
        }
    }
}
#endif

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
 * Returned Values:
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
  priv = (FAR struct usbhost_state_s *)kmalloc(sizeof(struct usbhost_state_s));
  uvdbg("Allocated: %p\n", priv);;
  return priv;
}

/****************************************************************************
 * Name: usbhost_freeclass
 *
 * Description:
 *   Free a class instance previously allocated by usbhost_allocclass().
 *
 * Input Parameters:
 *   class - A reference to the class instance to be freed.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline void usbhost_freeclass(FAR struct usbhost_state_s *class)
{
  DEBUGASSERT(class != NULL);

  /* Free the class instance. */

  uvdbg("Freeing: %p\n", class);;
  kfree(class);
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

  flags = irqsave();
  for (devno = 0; devno < 26; devno++)
    {
      uint32_t bitno = 1 << devno;
      if ((g_devinuse & bitno) == 0)
        {
          g_devinuse |= bitno;
          priv->devno = devno;
          irqrestore(flags);
          return OK;
        }
    }

  irqrestore(flags);
  return -EMFILE;
}

static void usbhost_freedevno(FAR struct usbhost_state_s *priv)
{
  int devno = priv->devno;

  if (devno >= 0 && devno < 26)
    {
      irqstate_t flags = irqsave();
      g_devinuse &= ~(1 << devno);
      irqrestore(flags);
    }
}

static inline void usbhost_mkdevname(FAR struct usbhost_state_s *priv,
                                     FAR char *devname)
{
  (void)snprintf(devname, DEV_NAMELEN, DEV_FORMAT, priv->devno);
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
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_destroy(FAR void *arg)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)arg;
  char devname[DEV_NAMELEN];

  DEBUGASSERT(priv != NULL);
  uvdbg("crefs: %d\n", priv->crefs);

  /* Unregister the driver */

  uvdbg("Unregister driver\n");
  usbhost_mkdevname(priv, devname);
  (void)unregister_driver(devname);

  /* Release the device name used by this connection */

  usbhost_freedevno(priv);

  /* Free the interrupt endpoints */

  if (priv->epin)
    {
      DRVR_EPFREE(priv->drvr, priv->epin);
    }

  /* Free any transfer buffers */

  usbhost_tdfree(priv);

  /* Destroy the semaphores */

  sem_destroy(&priv->exclsem);
  sem_destroy(&priv->waitsem);

  /* Disconnect the USB host device */

  DRVR_DISCONNECT(priv->drvr);

  /* And free the class instance.  Hmmm.. this may execute on the worker
   * thread and the work structure is part of what is getting freed.  That
   * should be okay because once the work contained is removed from the
   * queue, it should not longer be accessed by the worker thread.
   */

  usbhost_freeclass(priv);
}

/****************************************************************************
 * Name: usbhost_notify
 *
 * Description:
 *   Wake any threads waiting for mouse data
 *
 * Input Paramters:
 *   priv - A reference to the mouse state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_notify(FAR struct usbhost_state_s *priv)
{
#ifndef CONFIG_DISABLE_POLL
  int i;
#endif

  /* If there are threads waiting for read data, then signal one of them
   * that the read data is available.
   */

  if (priv->nwaiters > 0)
    {
      sem_post(&priv->waitsem);
    }

  /* If there are threads waiting on poll() for mouse data to become available,
   * then wake them up now.  NOTE: we wake up all waiting threads because we
   * do not know that they are going to do.  If they all try to read the data,
   * then some make end up blocking after all.
   */

#ifndef CONFIG_DISABLE_POLL
  for (i = 0; i < CONFIG_HIDMOUSE_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          ivdbg("Report events: %02x\n", fds->revents);
          sem_post(fds->sem);
        }
    }
#endif
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

#ifdef CONFIG_MOUSE_WHEEL
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

#ifdef CONFIG_MOUSE_WHEEL
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
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static int usbhost_mouse_poll(int argc, char *argv[])
{
  FAR struct usbhost_state_s *priv;
  FAR struct usbhid_mousereport_s *rpt;
#ifndef CONFIG_HIDMOUSE_TSCIF
  uint8_t buttons;
#endif
#if defined(CONFIG_DEBUG_USB) && defined(CONFIG_DEBUG_VERBOSE)
  unsigned int npolls = 0;
#endif
  unsigned int nerrors = 0;
  int ret;

  uvdbg("Started\n");

  /* Synchronize with the start-up logic.  Get the private instance, re-start
   * the start-up logic, and wait a bit to make sure that all of the class
   * creation logic has a chance to run to completion.
   *
   * NOTE: that the reference count is incremented here.  Therefore, we know
   * that the driver data structure will remain stable while this thread is
   * running.
   */

  priv = g_priv;
  DEBUGASSERT(priv != NULL);

  priv->polling = true;
  priv->crefs++;
  usbhost_givesem(&g_syncsem);
  sleep(1);

  /* Loop here until the device is disconnected */

  uvdbg("Entering poll loop\n");

  while (!priv->disconnected)
    {
      /* Read the next mouse report.  We will stall here until the mouse
       * sends data.
       */

      ret = DRVR_TRANSFER(priv->drvr, priv->epin,
                          priv->tbuffer, priv->tbuflen);

      /* Check for errors -- Bail if an excessive number of errors
       * are encountered.
       */

      if (ret != OK)
        {
          /* If DRVR_TRANSFER() returns EAGAIN, that simply means that
           * the devices was not ready and has NAK'ed the transfer.  That
           * should not be treated as an error (unless it persists for a
           * long time).
           */

          udbg("ERROR: DRVR_TRANSFER returned: %d/%d\n", ret, nerrors);
          if (ret != -EAGAIN)
            {
              if (++nerrors > 200)
                {
                  udbg("Too many errors... aborting: %d\n", nerrors);
                  break;
                }
            }
        }

      /* The report was received correctly.  But ignore the mouse data if no
       * task has opened the driver.
       */

      else if (priv->open)
        {
          /* Get exclusive access to the mouse state data */

          usbhost_takesem(&priv->exclsem);

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
              /* We get here when either there is a meaning button change
               * and/or a significant movement of the mouse.  We are going
               * to report the mouse event.
               *
               * Snap to the new x/y position for subsequent thresholding
               */

              priv->xlast = priv->xaccum;
              priv->ylast = priv->yaccum;
#ifdef CONFIG_MOUSE_WHEEL
              priv->wlast = priv->waccum;
#endif
              /* Update the sample X/Y positions */

              priv->sample.x = b16toi(priv->xaccum);
              priv->sample.y = b16toi(priv->yaccum);
#ifdef CONFIG_MOUSE_WHEEL
              priv->sample.wheel = b16toi(priv->waccum);
#endif

#ifdef CONFIG_HIDMOUSE_TSCIF
              /* The X/Y positional data is now valid */

              priv->sample.valid = true;

              /* Indicate the availability of new sample data for this ID */

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
        }

      /* Release our lock on the state structure */

      usbhost_givesem(&priv->exclsem);

      /* If USB debug is on, then provide some periodic indication that
       * polling is still happening.
       */

#if defined(CONFIG_DEBUG_USB) && defined(CONFIG_DEBUG_VERBOSE)
      npolls++;
      if ((npolls & 31) == 0)
        {
          udbg("Still polling: %d\n", npolls);
        }
#endif
    }

  /* We get here when the driver is removed.. or when too many errors have
   * been encountered.
   *
   * Make sure that we have exclusive access to the private data structure.
   * There may now be other tasks with the character driver open and actively
   * trying to interact with the class driver.
   */

  usbhost_takesem(&priv->exclsem);

  /* Indicate that we are no longer running and decrement the reference
   * count help by this thread.  If there are no other users of the class,
   * we can destroy it now.  Otherwise, we have to wait until the all
   * of the file descriptors are closed.
   */

  udbg("Mouse removed, polling halted\n");
  priv->polling = false;
  if (--priv->crefs < 2)
    {
      /* Destroy the instance (while we hold the semaphore!) */

      usbhost_destroy(priv);
    }
  else
    {
      /* No, we will destroy the driver instance when it is finally closed */

      usbhost_givesem(&priv->exclsem);
    }

  return 0;
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

  flags = irqsave();

  /* Is there new mouse data available? */

  if (priv->valid)
    {
      /* Return a copy of the sampled data. */

      memcpy(sample, &priv->sample, sizeof(struct mouse_sample_s ));

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

  irqrestore(flags);
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
  flags = irqsave();

  /* Now release the semaphore that manages mutually exclusive access to
   * the device structure.  This may cause other tasks to become ready to
   * run, but they cannot run yet because pre-emption is disabled.
   */

  sem_post(&priv->exclsem);

  /* Try to get the a sample... if we cannot, then wait on the semaphore
   * that is posted when new sample data is available.
   */

  while (usbhost_sample(priv, sample) < 0)
    {
      /* Wait for a change in the HIDMOUSE state */

      ivdbg("Waiting..\n");
      priv->nwaiters++;
      ret = sem_wait(&priv->waitsem);
      priv->nwaiters--;

      if (ret < 0)
        {
          /* If we are awakened by a signal, then we need to return
           * the failure now.
           */

          idbg("sem_wait: %d\n", errno);
          DEBUGASSERT(errno == EINTR);
          ret = -EINTR;
          goto errout;
        }

      /* Did the mouse become disconnected while we were waiting */

      if (priv->disconnected)
        {
          ret = -ENODEV;
          goto errout;
        }
    }

  ivdbg("Sampled\n");

   /* Re-acquire the semaphore that manages mutually exclusive access to
   * the device structure.  We may have to wait here.  But we have our sample.
   * Interrupts and pre-emption will be re-enabled while we wait.
   */

  ret = sem_wait(&priv->exclsem);

errout:
  /* Then re-enable interrupts.  We might get interrupt here and there
   * could be a new sample.  But no new threads will run because we still
   * have pre-emption disabled.
   */

  irqrestore(flags);

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
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static inline int usbhost_cfgdesc(FAR struct usbhost_state_s *priv,
                                  FAR const uint8_t *configdesc, int desclen,
                                  uint8_t funcaddr)
{
  FAR struct usb_cfgdesc_s *cfgdesc;
  FAR struct usb_desc_s *desc;
  FAR struct usbhost_epdesc_s epindesc;
  int remaining;
  uint8_t found = 0;
  bool done = false;
  int ret;

  DEBUGASSERT(priv != NULL &&
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

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
            uvdbg("Interface descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_IFDESC);

            /* Did we already find what we needed from a preceding interface? */

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
            uvdbg("HID descriptor\n");
            break;

        /* Endpoint descriptor.  We expect one or two interrupt endpoints,
         * a required IN endpoint and an optional OUT endpoint.
         */

        case USB_DESC_TYPE_ENDPOINT:
          {
            FAR struct usb_epdesc_s *epdesc = (FAR struct usb_epdesc_s *)configdesc;

            uvdbg("Endpoint descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_EPDESC);

            /* Check for an interrupt endpoint. */

            if ((epdesc->attr & USB_EP_ATTR_XFERTYPE_MASK) == USB_EP_ATTR_XFER_INT)
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

                    epindesc.addr         = epdesc->addr & USB_EP_ADDR_NUMBER_MASK;
                    epindesc.in           = true;
                    epindesc.funcaddr     = funcaddr;
                    epindesc.xfrtype      = USB_EP_ATTR_XFER_INT;
                    epindesc.interval     = epdesc->interval;
                    epindesc.mxpacketsize = usbhost_getle16(epdesc->mxpacketsize);

                    uvdbg("Interrupt IN EP addr:%d mxpacketsize:%d\n",
                          epindesc.addr, epindesc.mxpacketsize);
                  }
              }
          }
          break;

        /* Other descriptors are just ignored for now */

        default:
          uvdbg("Other descriptor: %d\n", desc->type);
          break;
        }

      /* What we found everything that we are going to find? */

      if (found == USBHOST_ALLFOUND)
        {
          /* Yes.. then break out of the loop and use the preceding interface */

          done = true;
        }

      /* Increment the address of the next descriptor */

      configdesc += desc->len;
      remaining  -= desc->len;
    }

  /* Sanity checking... did we find all of things that we need? */

  if ((found & USBHOST_ALLFOUND) != USBHOST_ALLFOUND)
    {
      ulldbg("ERROR: Found IF:%s EPIN:%s\n",
             (found & USBHOST_IFFOUND) != 0  ? "YES" : "NO",
             (found & USBHOST_EPINFOUND) != 0 ? "YES" : "NO");
      return -EINVAL;
    }

  /* We are good... Allocate the interrupt IN endpoint. */

  ret = DRVR_EPALLOC(priv->drvr, &epindesc, &priv->epin);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate interrupt IN endpoint\n");
      return ret;
    }

  ullvdbg("Endpoint allocated\n");
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
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline int usbhost_devinit(FAR struct usbhost_state_s *priv)
{
  char devname[DEV_NAMELEN];
  int ret;

  /* Set aside a transfer buffer for exclusive use by the mouse class driver */

  ret = usbhost_tdalloc(priv);
  if (ret != OK)
    {
      udbg("ERROR: Failed to allocate transfer buffer\n");
      return ret;
    }

  /* Increment the reference count.  This will prevent usbhost_destroy() from
   * being called asynchronously if the device is removed.
   */

  priv->crefs++;
  DEBUGASSERT(priv->crefs == 2);

  /* Start a worker task to poll the USB device.  It would be nice to used the
   * the NuttX worker thread to do this, but this task needs to wait for events
   * and activities on the worker thread should not involve significant waiting.
   * Having a dedicated thread is more efficient in this sense, but requires more
   * memory resources, primarily for the dedicated stack (CONFIG_HIDMOUSE_STACKSIZE).
   */

  uvdbg("user_start: Start poll task\n");

  /* The inputs to a task started by task_create() are very awkward for this
   * purpose.  They are really designed for command line tasks (argc/argv). So
   * the following is kludge pass binary data when the mouse poll task
   * is started.
   *
   * First, make sure we have exclusive access to g_priv (what is the likelihood
   * of this being used?  About zero, but we protect it anyway).
   */

  usbhost_takesem(&g_exclsem);
  g_priv = priv;

#ifndef CONFIG_CUSTOM_STACK
  priv->pollpid = task_create("mouse", CONFIG_HIDMOUSE_DEFPRIO,
                              CONFIG_HIDMOUSE_STACKSIZE,
                              (main_t)usbhost_mouse_poll, (FAR char * const *)NULL);
#else
  priv->pollpid = task_create("mouse", CONFIG_HIDMOUSE_DEFPRIO,
                              (main_t)usbhost_mouse_poll, (FAR char * const *)NULL);
#endif
  if (priv->pollpid == ERROR)
    {
      /* Failed to started the poll thread... probably due to memory resources */

      usbhost_givesem(&g_exclsem);
      ret = -ENOMEM;
      goto errout;
    }

  /* Now wait for the poll task to get properly initialized */

  usbhost_takesem(&g_syncsem);
  usbhost_givesem(&g_exclsem);

  /* Register the driver */

  uvdbg("Register driver\n");
  usbhost_mkdevname(priv, devname);
  ret = register_driver(devname, &g_hidmouse_fops, 0666, priv);

  /* We now have to be concerned about asynchronous modification of crefs
   * because the driver has been registered.
   */

errout:
  usbhost_takesem(&priv->exclsem);
  priv->crefs--;
  usbhost_givesem(&priv->exclsem);
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
 * Returned Values:
 *   A uint16_t representing the whole 16-bit integer value
 *
 ****************************************************************************/

static inline uint16_t usbhost_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: usbhost_putle16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 16-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void usbhost_putle16(uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}

/****************************************************************************
 * Name: usbhost_getle32
 *
 * Description:
 *   Get a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the big endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline uint32_t usbhost_getle32(const uint8_t *val)
{
 /* Little endian means LS halfword first in byte stream */

  return (uint32_t)usbhost_getle16(&val[2]) << 16 | (uint32_t)usbhost_getle16(val);
}

/****************************************************************************
 * Name: usbhost_putle32
 *
 * Description:
 *   Put a (possibly unaligned) 32-bit little endian value.
 *
 * Input Parameters:
 *   dest - A pointer to the first byte to save the little endian value.
 *   val - The 32-bit value to be saved.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

#if 0 /* Not used */
static void usbhost_putle32(uint8_t *dest, uint32_t val)
{
  /* Little endian means LS halfword first in byte stream */

  usbhost_putle16(dest, (uint16_t)(val & 0xffff));
  usbhost_putle16(dest+2, (uint16_t)(val >> 16));
}
#endif

/****************************************************************************
 * Name: usbhost_tdalloc
 *
 * Description:
 *   Allocate transfer buffer memory.
 *
 * Input Parameters:
 *   priv - A reference to the class instance.
 *
 * Returned Values:
 *   On success, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int usbhost_tdalloc(FAR struct usbhost_state_s *priv)
{
  DEBUGASSERT(priv && priv->tbuffer == NULL);
  return DRVR_ALLOC(priv->drvr, &priv->tbuffer, &priv->tbuflen);
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
 * Returned Values:
 *   On success, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int usbhost_tdfree(FAR struct usbhost_state_s *priv)
{
  int result = OK;
  DEBUGASSERT(priv);

  if (priv->tbuffer)
    {
      DEBUGASSERT(priv->drvr);
      result         = DRVR_FREE(priv->drvr, priv->tbuffer);
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
 *   This function implements the create() method of struct usbhost_registry_s.
 *   The create() method is a callback into the class implementation.  It is
 *   used to (1) create a new instance of the USB host class state and to (2)
 *   bind a USB host driver "session" to the class instance.  Use of this
 *   create() method will support environments where there may be multiple
 *   USB ports and multiple USB devices simultaneously connected.
 *
 * Input Parameters:
 *   drvr - An instance of struct usbhost_driver_s that the class
 *     implementation will "bind" to its state structure and will
 *     subsequently use to communicate with the USB host driver.
 *   id - In the case where the device supports multiple base classes,
 *     subclasses, or protocols, this specifies which to configure for.
 *
 * Returned Values:
 *   On success, this function will return a non-NULL instance of struct
 *   usbhost_class_s that can be used by the USB host driver to communicate
 *   with the USB host class.  NULL is returned on failure; this function
 *   will fail only if the drvr input parameter is NULL or if there are
 *   insufficient resources to create another USB host class instance.
 *
 ****************************************************************************/

static FAR struct usbhost_class_s *
  usbhost_create(FAR struct usbhost_driver_s *drvr,
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

          priv->class.connect      = usbhost_connect;
          priv->class.disconnected = usbhost_disconnected;

          /* The initial reference count is 1... One reference is held by
           * the driver.
           */

          priv->crefs              = 1;

          /* Initialize semaphores */

          sem_init(&priv->exclsem, 0, 1);
          sem_init(&priv->waitsem, 0, 0);

          /* Bind the driver to the storage class instance */

          priv->drvr               = drvr;

          /* Return the instance of the USB mouse class driver */

          return &priv->class;
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
 *   class - The USB host class entry previously obtained from a call to create().
 *   configdesc - A pointer to a uint8_t buffer container the configuration
 *     descriptor.
 *   desclen - The length in bytes of the configuration descriptor.
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 *   NOTE that the class instance remains valid upon return with a failure.  It is
 *   the responsibility of the higher level enumeration logic to call
 *   CLASS_DISCONNECTED to free up the class driver resources.
 *
 * Assumptions:
 *   - This function will *not* be called from an interrupt handler.
 *   - If this function returns an error, the USB host controller driver
 *     must call to DISCONNECTED method to recover from the error
 *
 ****************************************************************************/

static int usbhost_connect(FAR struct usbhost_class_s *class,
                           FAR const uint8_t *configdesc, int desclen,
                           uint8_t funcaddr)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)class;
  int ret;

  DEBUGASSERT(priv != NULL &&
              configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

  /* Parse the configuration descriptor to get the endpoints */

  ret = usbhost_cfgdesc(priv, configdesc, desclen, funcaddr);
  if (ret != OK)
    {
      udbg("usbhost_cfgdesc() failed: %d\n", ret);
    }
  else
    {
      /* Now configure the device and register the NuttX driver */

      ret = usbhost_devinit(priv);
      if (ret != OK)
        {
          udbg("usbhost_devinit() failed: %d\n", ret);
        }
    }

  /* ERROR handling:  Do nothing. If we return and error during connection,
   * the driver is required to call the DISCONNECT method.  Possibilities:
   *
   * - Failure occurred before the mouse poll task was started successfully.
   *   In this case, the disconnection will have to be handled on the worker
   *   task.
   * - Failure occurred after the mouse poll task was started successfully.  In
   *   this case, the disconnection can be performed on the mouse poll thread.
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
 *   class - The USB host class entry previously obtained from a call to
 *     create().
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

static int usbhost_disconnected(struct usbhost_class_s *class)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)class;
  int i;

  DEBUGASSERT(priv != NULL);

  /* Set an indication to any users of the mouse device that the device
   * is no longer available.
   */

  priv->disconnected = true;
  ullvdbg("Disconnected\n");

  /* Are there a thread(s) waiting for mouse data that will never come? */

  for (i = 0; i < priv->nwaiters; i++)
    {
      /* Yes.. wake them up */

      usbhost_givesem(&priv->waitsem);
    }

  /* Possibilities:
   *
   * - Failure occurred before the mouse poll task was started successfully.
   *   In this case, the disconnection will have to be handled on the worker
   *   task.
   * - Failure occurred after the mouse poll task was started successfully.  In
   *   this case, the disconnection can be performed on the mouse poll thread.
   */

  if (priv->polling)
    {
      /* The polling task is still alive. Signal the mouse polling task.
       * When that task wakes up, it will decrement the reference count and,
       * perhaps, destroy the class instance.  Then it will exit.
       */

      (void)kill(priv->pollpid, SIGALRM);
    }
  else
    {
      /* In the case where the failure occurs before the polling task was
       * started.  Now what?  We are probably executing from an interrupt
       * handler here.  We will use the worker thread.  This is kind of
       * wasteful and begs for a re-design.
       */

      DEBUGASSERT(priv->work.worker == NULL);
      (void)work_queue(HPWORK, &priv->work, usbhost_destroy, priv, 0);
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

  uvdbg("Entry\n");
  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv && priv->crefs > 0 && priv->crefs < USBHOST_MAX_CREFS);
  usbhost_takesem(&priv->exclsem);

  /* Check if the mouse device is still connected.  We need to disable
   * interrupts momentarily to assure that there are no asynchronous disconnect
   * events.
   */

  flags = irqsave();
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
#ifdef CONFIG_MOUSE_WHEEL
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

  irqrestore(flags);

  usbhost_givesem(&priv->exclsem);
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
  FAR struct inode           *inode;
  FAR struct usbhost_state_s *priv;

  uvdbg("Entry\n");
  DEBUGASSERT(filep && filep->f_inode);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Decrement the reference count on the driver */

  DEBUGASSERT(priv->crefs > 1);
  usbhost_takesem(&priv->exclsem);
  priv->crefs--;

  /* Is this the last reference (other than the one held by the USB host
   * controller driver)
   */

  if (priv->crefs <= 1)
    {
      irqstate_t flags;

      /* Yes.. then the driver is no longer open */

      priv->open = false;

      /* We need to disable interrupts momentarily to assure that there are
       * no asynchronous disconnect events.
       */

      flags = irqsave();

      /* Check if the USB mouse device is still connected.  If the device is
       * no longer connected, then unregister the driver and free the driver
       * class instance.
       */

      if (priv->disconnected)
        {
          /* Destroy the class instance (we can't use priv after this; we can't
           * 'give' the semaphore)
           */

          usbhost_destroy(priv);
          irqrestore(flags);
          return OK;
        }

      irqrestore(flags);
    }

  usbhost_givesem(&priv->exclsem);
  return OK;
}

/****************************************************************************
 * Name: usbhost_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t usbhost_read(FAR struct file *filep, FAR char *buffer, size_t len)
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

  uvdbg("Entry\n");
  DEBUGASSERT(filep && filep->f_inode && buffer);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv && priv->crefs > 0 && priv->crefs < USBHOST_MAX_CREFS);
  usbhost_takesem(&priv->exclsem);

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

          idbg("usbhost_waitsample: %d\n", ret);
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
          report->point[0].flags  = TOUCH_UP | TOUCH_ID_VALID | TOUCH_POS_VALID;
        }
      else
        {
          report->point[0].flags  = TOUCH_UP | TOUCH_ID_VALID;
        }
    }
  else if (sample.event == BUTTON_PRESSED)
    {
      /* First event */

      report->point[0].flags  = TOUCH_DOWN | TOUCH_ID_VALID | TOUCH_POS_VALID;
    }
  else /* if (sample->event == BUTTON_MOVE) */
    {
      /* Movement of the same event */

      report->point[0].flags  = TOUCH_MOVE | TOUCH_ID_VALID | TOUCH_POS_VALID;
    }

  ivdbg("  id:      %d\n", report->point[0].id);
  ivdbg("  flags:   %02x\n", report->point[0].flags);
  ivdbg("  x:       %d\n", report->point[0].x);
  ivdbg("  y:       %d\n", report->point[0].y);

  ret = SIZEOF_TOUCH_SAMPLE_S(1);
#else
  report = (FAR struct mouse_report_s *)buffer;
  memset(report, 0, sizeof(struct mouse_report_s));

  report->buttons = sample.buttons;
  report->x       = sample.x;
  report->y       = sample.y;
#ifdef CONFIG_MOUSE_WHEEL
  report->wheel   = sample.wheel;
#endif

  ret = sizeof(struct mouse_report_s);
#endif

errout:
  usbhost_givesem(&priv->exclsem);
  ivdbg("Returning: %d\n", ret);
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

#ifndef CONFIG_DISABLE_POLL
static int usbhost_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode           *inode;
  FAR struct usbhost_state_s *priv;
  int                         ret = OK;
  int                         i;

  uvdbg("Entry\n");
  DEBUGASSERT(filep && filep->f_inode && fds);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv);
  usbhost_takesem(&priv->exclsem);

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
          usbhost_pollnotify(priv);
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
  sem_post(&priv->exclsem);
  return ret;
}
#endif

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
 * Returned Values:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

int usbhost_mouse_init(void)
{
  /* Perform any one-time initialization of the class implementation */

  sem_init(&g_exclsem, 0, 1);
  sem_init(&g_syncsem, 0, 0);

  /* Advertise our availability to support (certain) mouse devices */

  return usbhost_registerclass(&g_hidmouse);
}

#endif /* CONFIG_USBHOST)&& !CONFIG_USBHOST_INT_DISABLE && CONFIG_NFILE_DESCRIPTORS */
