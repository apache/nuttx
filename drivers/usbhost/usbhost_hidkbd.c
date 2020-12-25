/****************************************************************************
 * drivers/usbhost/usbhost_hidkbd.c
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
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>
#include <signal.h>
#include <time.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/hid.h>
#include <nuttx/usb/usbhost_devaddr.h>

#ifdef CONFIG_HIDKBD_ENCODED
#  include <nuttx/streams.h>
#  include <nuttx/input/kbd_codec.h>
#endif

/* Don't compile if prerequisites are not met */

#if defined(CONFIG_USBHOST) && !defined(CONFIG_USBHOST_INT_DISABLE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* This determines how often the USB keyboard will be polled in units of
 * of microseconds.  The default is 100MS.
 */

#ifndef CONFIG_HIDKBD_POLLUSEC
#  define CONFIG_HIDKBD_POLLUSEC (100*1000)
#endif

/* Worker thread is needed, unfortunately, to handle some cornercase failure
 * conditions.  This is kind of wasteful and begs for a re-design.
 */

#ifndef CONFIG_SCHED_WORKQUEUE
#  warning "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

/* Provide some default values for other configuration settings */

#ifndef CONFIG_HIDKBD_DEFPRIO
#  define CONFIG_HIDKBD_DEFPRIO 50
#endif

#ifndef CONFIG_HIDKBD_STACKSIZE
#  define CONFIG_HIDKBD_STACKSIZE 1024
#endif

#ifndef CONFIG_HIDKBD_BUFSIZE
#  define CONFIG_HIDKBD_BUFSIZE 64
#endif

#ifndef CONFIG_HIDKBD_NPOLLWAITERS
#  define CONFIG_HIDKBD_NPOLLWAITERS 2
#endif

/* The default is to support scancode mapping for the standard 104 key
 * keyboard.  Setting CONFIG_HIDKBD_RAWSCANCODES will disable all scancode
 * mapping; Setting CONFIG_HIDKBD_ALLSCANCODES will enable mapping of all
 * scancodes;
 */

#ifndef CONFIG_HIDKBD_RAWSCANCODES
#  ifdef CONFIG_HIDKBD_ALLSCANCODES
#    define USBHID_NUMSCANCODES (USBHID_KBDUSE_MAX+1)
#  else
#    define USBHID_NUMSCANCODES 104
#  endif
#endif

/* We can't support encoding of special characters of unless the Keyboard
 * CODEC is enabled.
 */

#ifndef CONFIG_LIB_KBDCODEC
#  undef CONFIG_HIDKBD_ENCODED
#endif

/* If we are using raw scancodes, then we cannot support encoding of
 * special characters either.
 */

#ifdef CONFIG_HIDKBD_RAWSCANCODES
#  undef CONFIG_HIDKBD_ENCODED
#endif

/* Driver support ***********************************************************/

/* This format is used to construct the /dev/kbd[n] device driver path.  It
 * defined here so that it will be used consistently in all places.
 */

#define DEV_FORMAT          "/dev/kbd%c"
#define DEV_NAMELEN         11

/* Used in usbhost_cfgdesc() */

#define USBHOST_IFFOUND     0x01 /* Required I/F descriptor found */
#define USBHOST_EPINFOUND   0x02 /* Required interrupt IN EP descriptor found */
#define USBHOST_EPOUTFOUND  0x04 /* Optional interrupt OUT EP descriptor found */
#define USBHOST_RQDFOUND    (USBHOST_IFFOUND|USBHOST_EPINFOUND)
#define USBHOST_ALLFOUND    (USBHOST_RQDFOUND|USBHOST_EPOUTFOUND)

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

/* This structure contains the internal, private state of the USB host
 * keyboard storage class.
 */

struct usbhost_state_s
{
  /* This is the externally visible portion of the state */

  struct usbhost_class_s  usbclass;

  /* The remainder of the fields are provide o the keyboard class driver */

  char                    devchar;      /* Character identifying the /dev/kbd[n] device */
  volatile bool           disconnected; /* TRUE: Device has been disconnected */
  volatile bool           polling;      /* TRUE: Poll thread is running */
  volatile bool           open;         /* TRUE: The keyboard device is open */
  volatile bool           waiting;      /* TRUE: waiting for keyboard data */
  uint8_t                 ifno;         /* Interface number */
  int16_t                 crefs;        /* Reference count on the driver instance */
  sem_t                   exclsem;      /* Used to maintain mutual exclusive access */
  sem_t                   waitsem;      /* Used to wait for keyboard data */
  FAR uint8_t            *tbuffer;      /* The allocated transfer buffer */
  size_t                  tbuflen;      /* Size of the allocated transfer buffer */
  pid_t                   pollpid;      /* PID of the poll task */
  struct work_s           work;         /* For cornercase error handling by the worker thread */

  /* Endpoints:
   * EP0 (Control):
   * - Receiving and responding to requests for USB control and class data.
   * - IN data when polled by the HID class driver (Get_Report)
   * - OUT data from the host.
   * EP Interrupt IN:
   * - Receiving asynchronous (unrequested) IN data from the device.
   * EP Interrupt OUT (optional):
   * - Transmitting low latency OUT data to the device.
   * - If not present, EP0 used.
   */

  usbhost_ep_t            epin;         /* Interrupt IN endpoint */
  usbhost_ep_t            epout;        /* Optional interrupt OUT endpoint */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

  struct pollfd *fds[CONFIG_HIDKBD_NPOLLWAITERS];

  /* Buffer used to collect and buffer incoming keyboard characters */

  volatile uint16_t       headndx;      /* Buffer head index */
  volatile uint16_t       tailndx;      /* Buffer tail index */
  uint8_t                 kbdbuffer[CONFIG_HIDKBD_BUFSIZE];
};

/* This type is used for encoding special characters */

#ifdef CONFIG_HIDKBD_ENCODED
struct usbhost_outstream_s
{
  struct lib_outstream_s stream;
  FAR struct usbhost_state_s *priv;
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Semaphores */

static int  usbhost_takesem(FAR sem_t *sem);
static void usbhost_forcetake(FAR sem_t *sem);
#define usbhost_givesem(s) nxsem_post(s);

/* Polling support */

static void usbhost_pollnotify(FAR struct usbhost_state_s *dev);

/* Memory allocation services */

static inline FAR struct usbhost_state_s *usbhost_allocclass(void);
static inline void usbhost_freeclass(FAR struct usbhost_state_s *usbclass);

/* Device name management */

static int  usbhost_allocdevno(FAR struct usbhost_state_s *priv);
static void usbhost_freedevno(FAR struct usbhost_state_s *priv);
static inline void usbhost_mkdevname(FAR struct usbhost_state_s *priv,
              FAR char *devname);

/* Keyboard polling thread */

static void usbhost_destroy(FAR void *arg);
static void usbhost_putbuffer(FAR struct usbhost_state_s *priv,
              uint8_t keycode);
#ifdef CONFIG_HIDKBD_ENCODED
static void usbhost_putstream(FAR struct lib_outstream_s *this, int ch);
#endif
static inline uint8_t usbhost_mapscancode(uint8_t scancode,
              uint8_t modifier);
#ifdef CONFIG_HIDKBD_ENCODED
static inline void usbhost_encodescancode(FAR struct usbhost_state_s *priv,
              uint8_t scancode, uint8_t modifier);
#endif
static int  usbhost_kbdpoll(int argc, char *argv[]);

/* Helpers for usbhost_connect() */

static inline int usbhost_cfgdesc(FAR struct usbhost_state_s *priv,
              FAR const uint8_t *configdesc, int desclen);
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

static struct usbhost_class_s *usbhost_create(
              FAR struct usbhost_hubport_s *hport,
              FAR const struct usbhost_id_s *id);

/* struct usbhost_class_s methods */

static int  usbhost_connect(FAR struct usbhost_class_s *usbclass,
                            FAR const uint8_t *configdesc, int desclen);
static int  usbhost_disconnected(FAR struct usbhost_class_s *usbclass);

/* Driver methods.  We export the keyboard as a standard character driver */

static int  usbhost_open(FAR struct file *filep);
static int  usbhost_close(FAR struct file *filep);
static ssize_t usbhost_read(FAR struct file *filep,
              FAR char *buffer, size_t len);
static ssize_t usbhost_write(FAR struct file *filep,
              FAR const char *buffer, size_t len);
static int  usbhost_poll(FAR struct file *filep, FAR struct pollfd *fds,
              bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure provides the registry entry ID information that will  be
 * used to associate the USB host keyboard class driver to a connected USB
 * device.
 */

static const struct usbhost_id_s g_hidkbd_id =
{
  USB_CLASS_HID,            /* base     */
  USBHID_SUBCLASS_BOOTIF,   /* subclass */
  USBHID_PROTOCOL_KEYBOARD, /* proto    */
  0,                        /* vid      */
  0                         /* pid      */
};

/* This is the USB host storage class's registry entry */

static struct usbhost_registry_s g_hidkbd =
{
  NULL,                     /* flink     */
  usbhost_create,           /* create    */
  1,                        /* nids      */
  &g_hidkbd_id              /* id[]      */
};

static const struct file_operations g_hidkbd_fops =
{
  usbhost_open,             /* open      */
  usbhost_close,            /* close     */
  usbhost_read,             /* read      */
  usbhost_write,            /* write     */
  NULL,                     /* seek      */
  NULL,                     /* ioctl     */
  usbhost_poll              /* poll      */
};

/* This is a bitmap that is used to allocate device names /dev/kbda-z. */

static uint32_t g_devinuse;

/* The following are used to managed the class creation operation */

static sem_t                   g_exclsem; /* For mutually exclusive thread creation */
static sem_t                   g_syncsem; /* Thread data passing interlock */
static struct usbhost_state_s *g_priv;    /* Data passed to thread */

/* The following tables map keyboard scan codes to printable ASIC
 * characters.  There is no support here for function keys or cursor
 * controls.
 */

#ifndef CONFIG_HIDKBD_RAWSCANCODES
#ifdef CONFIG_HIDKBD_ENCODED

/* The first and last scancode values with encode-able values */

#define FIRST_ENCODING   USBHID_KBDUSE_ENTER          /* 0x28 Keyboard Return (ENTER) */
#ifndef CONFIG_HIDKBD_ALLSCANCODES
#  define LAST_ENCODING  USBHID_KBDUSE_POWER          /* 0x66 Keyboard Power */
#else
#  define LAST_ENCODING  USBHID_KBDUSE_KPDHEXADECIMAL /* 0xdd Keypad Hexadecimal */
#endif

#define USBHID_NUMENCODINGS (LAST_ENCODING - FIRST_ENCODING + 1)

static const uint8_t encoding[USBHID_NUMENCODINGS] =
{
  /* 0x28-0x2f: Enter,escape,del,back-tab,space,_,+,{ */

  KEYCODE_ENTER,        0,
  KEYCODE_FWDDEL,       KEYCODE_BACKDEL,
  0,                    0,
  0,                    0,

  /* 0x30-0x37: },|,Non-US tilde,:,",grave tilde,<,> */

  0,                    0,
  0,                    0,
  0,                    0,
  0,                    0,

  /* 0x38-0x3f: /,CapsLock,F1,F2,F3,F4,F5,F6 */

  0,                    KEYCODE_CAPSLOCK,
  KEYCODE_F1,           KEYCODE_F2,
  KEYCODE_F3,           KEYCODE_F4,
  KEYCODE_F5,           KEYCODE_F6,

  /* 0x40-0x47: F7,F8,F9,F10,F11,F12,PrtScn,ScrollLock */

  KEYCODE_F7,           KEYCODE_F8,
  KEYCODE_F9,           KEYCODE_F10,
  KEYCODE_F11,          KEYCODE_F12,
  KEYCODE_PRTSCRN,      KEYCODE_SCROLLLOCK,

  /* 0x48-0x4f: Pause,Insert,Home,PageUp,
   * DeleteForward,End,PageDown,RightArrow
   */

  KEYCODE_PAUSE,        KEYCODE_INSERT,
  KEYCODE_HOME,         KEYCODE_PAGEUP,
  KEYCODE_FWDDEL,       KEYCODE_END,
  KEYCODE_PAGEDOWN,     KEYCODE_RIGHT,

  /* 0x50-0x57: LeftArrow,DownArrow,UpArrow,Num Lock,/,*,-,+ */

  KEYCODE_LEFT,         KEYCODE_DOWN,
  KEYCODE_UP,           KEYCODE_NUMLOCK,
  0,                    0,
  0,                    0,

  /* 0x58-0x5f: Enter,1-7 */

  KEYCODE_ENTER,        0,
  0,                    0,
  0,                    0,
  0,                    0,

  /* 0x60-0x66: 8-9,0,.,Non-US \,Application,Power */

  0,                    0,
  0,                    0,
  0,                    0,
  KEYCODE_POWER,

#ifdef CONFIG_HIDKBD_ALLSCANCODES

  0, /* 0x67 = */

  /* 0x68-0x6f: F13,F14,F15,F16,F17,F18,F19,F20 */

  KEYCODE_F13,          KEYCODE_F14,
  KEYCODE_F15,          KEYCODE_F16,
  KEYCODE_F17,          KEYCODE_F18,
  KEYCODE_F19,          KEYCODE_F20,

  /* 0x70-0x77: F21,F22,F23,F24,Execute,Help,Menu,Select */

  KEYCODE_F21,          KEYCODE_F22,
  KEYCODE_F23,          KEYCODE_F24,
  KEYCODE_EXECUTE,      KEYCODE_HELP,
  KEYCODE_MENU,         KEYCODE_SELECT,

  /* 0x78-0x7f: Stop,Again,Undo,Cut,Copy,Paste,Find,Mute */

  KEYCODE_STOP,         KEYCODE_AGAIN,
  KEYCODE_UNDO,         KEYCODE_CUT,
  KEYCODE_COPY,         KEYCODE_PASTE,
  KEYCODE_FIND,         KEYCODE_MUTE,

  /* 0x80-0x87: VolUp,VolDown,LCapsLock,lNumLock,LScrollLock,,,
   * =,International1
   */

  KEYCODE_VOLUP,        KEYCODE_VOLDOWN,
  KEYCODE_LCAPSLOCK,    KEYCODE_LNUMLOCK,
  KEYCODE_LSCROLLLOCK,  0,
  0,                    0,

  /* 0x88-0x8f: International 2-9 */

  0,                    0,
  0,                    0,
  0,                    0,
  0,                    0,

  /* 0x90-0x97: LAN 1-8 */

  KEYCODE_LANG1,        KEYCODE_LANG2,
  KEYCODE_LANG3,        KEYCODE_LANG4,
  KEYCODE_LANG5,        KEYCODE_LANG6,
  KEYCODE_LANG7,        KEYCODE_LANG8,

  /* 0x98-0x9f: LAN 9,Erase,SysReq,Cancel,Clear,Prior,Return,Separator */

  0,                    0,
  KEYCODE_SYSREQ,       KEYCODE_CANCEL,
  KEYCODE_CLEAR,        0,
  KEYCODE_ENTER,        0,

  /* 0xa0-0xa7: Out,Oper,Clear,CrSel,Excel,(reserved) */

  0,                    0,
  0,                    0,
  0,                    0,
  0,                    0,

  /* 0xa8-0xaf: (reserved) */

  0,                    0,
  0,                    0,
  0,                    0,
  0,                    0,

  /* 0xb0-0xb7: 00,000,ThouSeparator,DecSeparator,CurrencyUnit,SubUnit,(,) */

  0,                    0,
  0,                    0,
  0,                    0,
  0,                    0,

  /* 0xb8-0xbf: {,},tab,backspace,A-D */

  0,                    0,
  0,                    KEYCODE_BACKDEL,
  0,                    0,
  0,                    0,

  /* 0xc0-0xc7: E-F,XOR,^,%,<,>,& */

  0,                    0,
  0,                    0,
  0,                    0,
  0,                    0,

  /* 0xc8-0xcf: &&,|,||,:,#, ,@,! */

  0,                    0,
  0,                    0,
  0,                    0,
  0,                    0,

  /* 0xd0-0xd7: Memory Store,Recall,Clear,Add,Subtract,Muliply,Divide,+/- */

  KEYCODE_MEMSTORE,     KEYCODE_MEMRECALL,
  KEYCODE_MEMCLEAR,     KEYCODE_MEMADD,
  KEYCODE_MEMSUB,       KEYCODE_MEMMUL,
  KEYCODE_MEMDIV,       KEYCODE_NEGATE,

  /* 0xd8-0xdd: Clear,ClearEntry,Binary,Octal,Decimal,Hexadecimal */

  KEYCODE_CLEAR,        KEYCODE_CLEARENTRY,
  KEYCODE_BINARY,       KEYCODE_OCTAL,
  KEYCODE_DECIMAL,      KEYCODE_HEXADECIMAL
#endif
};

#endif

static const uint8_t ucmap[USBHID_NUMSCANCODES] =
{
  0,    0,      0,      0,       'A',  'B',  'C',    'D',  /* 0x00-0x07: Reserved, errors, A-D */
  'E',  'F',    'G',    'H',     'I',  'J',  'K',    'L',  /* 0x08-0x0f: E-L */
  'M',  'N',    'O',    'P',     'Q',  'R',  'S',    'T',  /* 0x10-0x17: M-T */
  'U',  'V',    'W',    'X',     'Y',  'Z',  '!',    '@',  /* 0x18-0x1f: U-Z,!,@  */
  '#',  '$',    '%',    '^',     '&',  '*',  '(',    ')',  /* 0x20-0x27: #,$,%,^,&,*,(,) */
  '\n', '\033', '\177', 0,       ' ',  '_',  '+',    '{',  /* 0x28-0x2f: Enter,escape,del,back-tab,space,_,+,{ */
  '}',  '|',    0,      ':',     '"',  '~',  '<',    '>',  /* 0x30-0x37: },|,Non-US tilde,:,",grave tilde,<,> */
  '?',  0,       0,      0,      0,    0,    0,      0,    /* 0x38-0x3f: /,CapsLock,F1,F2,F3,F4,F5,F6 */
  0,    0,       0,      0,      0,    0,    0,      0,    /* 0x40-0x47: F7,F8,F9,F10,F11,F12,PrtScn,ScrollLock */
  0,    0,       0,      0,      0,    0,    0,      0,    /* 0x48-0x4f: Pause,Insert,Home,PageUp,DeleteForward,End,PageDown,RightArrow */
  0,    0,       0,      0,      '/',  '*',  '-',    '+',  /* 0x50-0x57: LeftArrow,DownArrow,UpArrow,Num Lock,/,*,-,+ */
  '\n', '1',     '2',    '3',    '4',  '5',  '6',    '7',  /* 0x58-0x5f: Enter,1-7 */
  '8',  '9',     '0',    '.',    0,    0,    0,      '=',  /* 0x60-0x67: 8-9,0,.,Non-US \,Application,Power,= */
#ifdef CONFIG_HIDKBD_ALLSCANCODES
  0,    0,       0,      0,      0,    0,    0,      0,    /* 0x68-0x6f: F13,F14,F15,F16,F17,F18,F19,F20 */
  0,    0,       0,      0,      0,    0,    0,      0,    /* 0x70-0x77: F21,F22,F23,F24,Execute,Help,Menu,Select */
  0,    0,       0,      0,      0,    0,    0,      0,    /* 0x78-0x7f: Stop,Again,Undo,Cut,Copy,Paste,Find,Mute */
  0,    0,       0,      0,      0,    ',',  0,      0,    /* 0x80-0x87: VolUp,VolDown,LCapsLock,lNumLock,LScrollLock,,,=,International1 */
  0,    0,       0,      0,      0,    0,    0,      0,    /* 0x88-0x8f: International 2-9 */
  0,    0,       0,      0,      0,    0,    0,      0,    /* 0x90-0x97: LAN 1-8 */
  0,    0,       0,      0,      0,    0,    '\n',   0,    /* 0x98-0x9f: LAN 9,Erase,SysReq,Cancel,Clear,Prior,Return,Separator */
  0,    0,       0,      0,      0,    0,    0,      0,    /* 0xa0-0xa7: Out,Oper,Clear,CrSel,Excel,(reserved) */
  0,    0,       0,      0,      0,    0,    0,      0,    /* 0xa8-0xaf: (reserved) */
  0,    0,       0,      0,      0,    0,    '(',    ')',  /* 0xb0-0xb7: 00,000,ThouSeparator,DecSeparator,CurrencyUnit,SubUnit,(,) */
  '{',  '}',    '\t',    \177,   'A',  'B',  'C',    'D',  /* 0xb8-0xbf: {,},tab,backspace,A-D */
  'F',  'F',     0,      '^',    '%',  '<', '>',     '&',  /* 0xc0-0xc7: E-F,XOR,^,%,<,>,& */
  0,    '|',     0,      ':',    '%',  ' ', '@',     '!',  /* 0xc8-0xcf: &&,|,||,:,#, ,@,! */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0xd0-0xd7: Memory Store,Recall,Clear,Add,Subtract,Muliply,Divide,+/- */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0xd8-0xdf: Clear,ClearEntry,Binary,Octal,Decimal,Hexadecimal */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0xe0-0xe7: Left Ctrl,Shift,Alt,GUI, Right Ctrl,Shift,Alt,GUI */
#endif
};

static const uint8_t lcmap[USBHID_NUMSCANCODES] =
{
  0,    0,       0,      0,      'a',  'b', 'c',     'd',  /* 0x00-0x07: Reserved, errors, a-d */
  'e',  'f',     'g',    'h',    'i',  'j', 'k',     'l',  /* 0x08-0x0f: e-l */
  'm',  'n',     'o',    'p',    'q',  'r', 's',     't',  /* 0x10-0x17: m-t */
  'u',  'v',     'w',    'x',    'y',  'z', '1',     '2',  /* 0x18-0x1f: u-z,1-2  */
  '3',  '4',     '5',    '6',    '7',  '8', '9',     '0',  /* 0x20-0x27: 3-9,0 */
  '\n', '\033',  '\177', '\t',   ' ',  '-', '=',     '[',  /* 0x28-0x2f: Enter,escape,del,tab,space,-,=,[ */
  ']',  '\\',    '\234', ';',    '\'', '`', ',',     '.',  /* 0x30-0x37: ],\,Non-US pound,;,',grave accent,,,. */
  '/',  0,       0,      0,      0,    0,   0,       0,    /* 0x38-0x3f: /,CapsLock,F1,F2,F3,F4,F5,F6 */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0x40-0x47: F7,F8,F9,F10,F11,F12,PrtScn,ScrollLock */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0x48-0x4f: Pause,Insert,Home,PageUp,DeleteForward,End,PageDown,RightArrow */
  0,    0,       0,      0,      '/',  '*', '-',     '+',  /* 0x50-0x57: LeftArrow,DownArrow,UpArrow,Num Lock,/,*,-,+ */
  '\n', '1',     '2',    '3',    '4',  '5', '6',     '7',  /* 0x58-0x5f: Enter,1-7 */
  '8',  '9',     '0',    '.',    0,    0,   0,       '=',  /* 0x60-0x67: 8-9,0,.,Non-US \,Application,Power,= */
#ifdef CONFIG_HIDKBD_ALLSCANCODES
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0x68-0x6f: F13,F14,F15,F16,F17,F18,F19,F20 */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0x70-0x77: F21,F22,F23,F24,Execute,Help,Menu,Select */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0x78-0x7f: Stop,Again,Undo,Cut,Copy,Paste,Find,Mute */
  0,    0,       0,      0,      0,    ',', 0,       0,    /* 0x80-0x87: VolUp,VolDown,LCapsLock,lNumLock,LScrollLock,,,=,International1 */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0x88-0x8f: International 2-9 */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0x90-0x97: LAN 1-8 */
  0,    0,       0,      0,      0,    0,   '\n',    0,    /* 0x98-0x9f: LAN 9,Erase,SysReq,Cancel,Clear,Prior,Return,Separator */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0xa0-0xa7: Out,Oper,Clear,CrSel,Excel,(reserved) */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0xa8-0xaf: (reserved) */
  0,    0,       0,      0,      0,    0,   '(',     ')',  /* 0xb0-0xb7: 00,000,ThouSeparator,DecSeparator,CurrencyUnit,SubUnit,(,) */
  '{',  '}',    '\t',    '\177', 'A',  'B', 'C',     'D',  /* 0xb8-0xbf: {,},tab,backspace,A-D */
  'F',  'F',     0,      '^',    '%',  '<', '>',     '&',  /* 0xc0-0xc7: E-F,XOR,^,%,<,>,& */
  0,    '|',     0,      ':',    '%',  ' ', '@',     '!',  /* 0xc8-0xcf: &&,|,||,:,#, ,@,! */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0xd0-0xd7: Memory Store,Recall,Clear,Add,Subtract,Muliply,Divide,+/- */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0xd8-0xdf: Clear,ClearEntry,Binary,Octal,Decimal,Hexadecimal */
  0,    0,       0,      0,      0,    0,   0,       0,    /* 0xe0-0xe7: Left Ctrl,Shift,Alt,GUI, Right Ctrl,Shift,Alt,GUI */
#endif
};
#endif /* CONFIG_HIDKBD_RAWSCANCODES */

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

static int usbhost_takesem(FAR sem_t *sem)
{
  return nxsem_wait_uninterruptible(sem);
}

/****************************************************************************
 * Name: usbhost_forcetake
 *
 * Description:
 *   This is just another wrapper but this one continues even if the thread
 *   is canceled.  This must be done in certain conditions where were must
 *   continue in order to clean-up resources.
 *
 ****************************************************************************/

static void usbhost_forcetake(FAR sem_t *sem)
{
  int ret;

  do
    {
      ret = nxsem_wait_uninterruptible(sem);

      /* The only expected error would -ECANCELED meaning that the
       * parent thread has been canceled.  We have to continue and
       * terminate the poll in this case.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (ret < 0);
}

/****************************************************************************
 * Name: usbhost_pollnotify
 ****************************************************************************/

static void usbhost_pollnotify(FAR struct usbhost_state_s *priv)
{
  int i;

  for (i = 0; i < CONFIG_HIDKBD_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= (fds->events & POLLIN);
          if (fds->revents != 0)
            {
              uinfo("Report events: %02x\n", fds->revents);
              nxsem_post(fds->sem);
            }
        }
    }
}

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
          priv->devchar = 'a' + devno;
          leave_critical_section(flags);
          return OK;
        }
    }

  leave_critical_section(flags);
  return -EMFILE;
}

static void usbhost_freedevno(FAR struct usbhost_state_s *priv)
{
  int devno = 'a' - priv->devchar;

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
  snprintf(devname, DEV_NAMELEN, DEV_FORMAT, priv->devchar);
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
  hport = priv->usbclass.hport;

  uinfo("crefs: %d\n", priv->crefs);

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

  if (priv->epout)
    {
      DRVR_EPFREE(hport->drvr, priv->epout);
    }

  /* Free any transfer buffers */

  usbhost_tdfree(priv);

  /* Destroy the semaphores */

  nxsem_destroy(&priv->exclsem);
  nxsem_destroy(&priv->waitsem);

  /* Disconnect the USB host device */

  DRVR_DISCONNECT(hport->drvr, hport);

  /* Free the function address assigned to this device */

  usbhost_devaddr_destroy(hport, hport->funcaddr);
  hport->funcaddr = 0;

  /* And free the class instance.  */

  usbhost_freeclass(priv);
}

/****************************************************************************
 * Name: usbhost_putbuffer
 *
 * Description:
 *   Add one character to the user buffer.
 *
 * Input Parameters:
 *   priv - Driver internal state
 *   keycode - The value to add to the user buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usbhost_putbuffer(FAR struct usbhost_state_s *priv,
                              uint8_t keycode)
{
  register unsigned int head;
  register unsigned int tail;

  /* Copy the next keyboard character into the user buffer. */

  head = priv->headndx;
  priv->kbdbuffer[head] = keycode;

  /* Increment the head index */

  if (++head >= CONFIG_HIDKBD_BUFSIZE)
    {
      head = 0;
    }

  /* If the buffer is full, then increment the tail index to make space.  Is
   * it better to lose old keystrokes or new?
   */

  tail = priv->tailndx;
  if (tail == head)
    {
      if (++tail >= CONFIG_HIDKBD_BUFSIZE)
        {
          tail = 0;
        }

      /* Save the updated tail index */

      priv->tailndx = tail;
    }

  /* Save the updated head index */

  priv->headndx = head;
}

/****************************************************************************
 * Name: usbhost_putstream
 *
 * Description:
 *   A wrapper for usbhost_putc that is compatible with the lib_outstream_s
 *   putc methods.
 *
 * Input Parameters:
 *   stream - The struct lib_outstream_s reference
 *   ch - The character to add to the user buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_HIDKBD_ENCODED
static void usbhost_putstream(FAR struct lib_outstream_s *stream, int ch)
{
  FAR struct usbhost_outstream_s *privstream =
    (FAR struct usbhost_outstream_s *)stream;

  DEBUGASSERT(privstream && privstream->priv);
  usbhost_putbuffer(privstream->priv, (uint8_t)ch);
  stream->nput++;
}
#endif

/****************************************************************************
 * Name: usbhost_mapscancode
 *
 * Description:
 *   Map a keyboard scancode to a printable ASCII character.  There is no
 *   support here for function keys or cursor controls in this version of
 *   the driver.
 *
 * Input Parameters:
 *   scancode - Scan code to be mapped.
 *   modifier - Ctrl,Alt,Shift,GUI modifier bits
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline uint8_t usbhost_mapscancode(uint8_t scancode, uint8_t modifier)
{
#ifndef CONFIG_HIDKBD_RAWSCANCODES
  /* Range check */

  if (scancode >= USBHID_NUMSCANCODES)
    {
      return 0;
    }

  /* Is either shift key pressed? */

  if ((modifier & (USBHID_MODIFER_LSHIFT | USBHID_MODIFER_RSHIFT)) != 0)
    {
      return ucmap[scancode];
    }
  else
    {
      return lcmap[scancode];
    }
#else
  return scancode;
#endif
}

/****************************************************************************
 * Name: usbhost_encodescancode
 *
 * Description:
 *  Check if the key has a special function encoding and, if it does, add
 *  the encoded value to the user buffer.
 *
 * Input Parameters:
 *   priv - Driver internal state
 *   scancode - Scan code to be mapped.
 *   modifier - Ctrl, Alt, Shift, GUI modifier bits
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_HIDKBD_ENCODED
static inline void usbhost_encodescancode(FAR struct usbhost_state_s *priv,
                                          uint8_t scancode, uint8_t modifier)
{
  uint8_t encoded;

  /* Check if the raw scancode is in a valid range */

  if (scancode >= FIRST_ENCODING && scancode <= LAST_ENCODING)
    {
      /* Yes the value is within range */

      encoded = encoding[scancode - FIRST_ENCODING];
      iinfo("  scancode: %02x modifier: %02x encoded: %d\n",
            scancode, modifier, encoded);

      if (encoded)
        {
          struct usbhost_outstream_s usbstream;

          /* And it does correspond to a special function key */

          usbstream.stream.put  = usbhost_putstream;
          usbstream.stream.nput = 0;
          usbstream.priv        = priv;

          /* Add the special function value to the user buffer */

          kbd_specpress((enum kbd_keycode_e)encoded,
                        (FAR struct lib_outstream_s *)&usbstream);
        }
    }
}
#endif

/****************************************************************************
 * Name: usbhost_kbdpoll
 *
 * Description:
 *   Periodically check for new keyboard data.
 *
 * Input Parameters:
 *   arg - A reference to the class instance to be destroyed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int usbhost_kbdpoll(int argc, char *argv[])
{
  FAR struct usbhost_state_s *priv;
  FAR struct usbhost_hubport_s *hport;
  FAR struct usb_ctrlreq_s *ctrlreq;
  irqstate_t flags;
#ifndef CONFIG_HIDKBD_NODEBOUNCE
  uint8_t lastkey[6] =
  {
    0, 0, 0, 0, 0, 0
  };
#endif

#if defined(CONFIG_DEBUG_USB) && defined(CONFIG_DEBUG_INFO)
  unsigned int npolls = 0;
#endif
  unsigned int nerrors = 0;
  useconds_t delay;
  bool empty = true;
  bool newstate;
  int ret;

  uinfo("Started\n");

  /* Synchronize with the start-up logic.  Get the private instance, re-start
   * the start-up logic, and wait a bit to make sure that all of the class
   * creation logic has a chance to run to completion.
   *
   * NOTE: that the reference count is *not* incremented here.  When the
   * driver structure was created, it was created with a reference count of
   * one.  This thread is responsible for that count.  The count will be
   * decrement when this thread exits.
   */

  priv = g_priv;
  DEBUGASSERT(priv != NULL && priv->usbclass.hport);
  hport = priv->usbclass.hport;

  priv->polling = true;
  usbhost_givesem(&g_syncsem);
  nxsig_sleep(1);

  /* Loop here until the device is disconnected */

  uinfo("Entering poll loop\n");

  while (!priv->disconnected)
    {
      /* Make sure that we have exclusive access to the private data
       * structure. There may now be other tasks with the character driver
       * open and actively trying to interact with the class driver.
       */

      ret = usbhost_takesem(&priv->exclsem);
      if (ret < 0)
        {
          return ret;
        }

      /* Format the HID report request:
       *
       *   bmRequestType 10100001
       *   bRequest      GET_REPORT (0x01)
       *   wValue        Report Type and Report Index
       *   wIndex        Interface Number
       *   wLength       Descriptor Length
       *   Data          Descriptor Data
       */

      ctrlreq       = (struct usb_ctrlreq_s *)priv->tbuffer;
      ctrlreq->type = USB_REQ_DIR_IN | USB_REQ_TYPE_CLASS |
                      USB_REQ_RECIPIENT_INTERFACE;
      ctrlreq->req  = USBHID_REQUEST_GETREPORT;

      usbhost_putle16(ctrlreq->value, (USBHID_REPORTTYPE_INPUT << 8));
      usbhost_putle16(ctrlreq->index, priv->ifno);
      usbhost_putle16(ctrlreq->len,   sizeof(struct usbhid_kbdreport_s));

      /* Send HID report request */

      ret = DRVR_CTRLIN(hport->drvr, hport->ep0, ctrlreq, priv->tbuffer);
      usbhost_givesem(&priv->exclsem);

      /* Check for errors -- Bail if an excessive number of consecutive
       * errors are encountered.
       */

      if (ret < 0)
        {
          nerrors++;
          uerr("ERROR: GETREPORT/INPUT, DRVR_CTRLIN returned: %d/%d\n",
               ret, nerrors);

          if (nerrors > 200)
            {
              uerr("  Too many errors... aborting: %d\n", nerrors);
              break;
            }
        }

      /* The report was received correctly.  But ignore the keystrokes if no
       * task has opened the driver.
       */

      else if (priv->open)
        {
          struct usbhid_kbdreport_s *rpt =
            (struct usbhid_kbdreport_s *)priv->tbuffer;
          uint8_t keycode;
          int i;

          /* Success, reset the error counter */

          nerrors = 0;

          /* Add the newly received keystrokes to our internal buffer */

          ret = usbhost_takesem(&priv->exclsem);
          if (ret < 0)
            {
              return ret;
            }

          for (i = 0; i < 6; i++)
            {
              /* Is this key pressed?  But not pressed last time?
               * HID spec: "The order of keycodes in array fields has no
               * significance. Order determination is done by the host
               * software comparing the contents of the previous report to
               * the current report. If two or more keys are reported in
               * one report, their order is indeterminate. Keyboards may
               * buffer events that would have otherwise resulted in
               * multiple event in a single report.
               *
               * "'Repeat Rate' and 'Delay Before First Repeat' are
               * implemented by the host and not in the keyboard (this
               * means the BIOS in legacy mode). The host may use the
               * device report rate and the number of reports to determine
               * how long a key is being held down. Alternatively, the host
               * may use its own clock or the idle request for the timing
               * of these features."
               */

              if (rpt->key[i] != USBHID_KBDUSE_NONE
#ifndef CONFIG_HIDKBD_NODEBOUNCE
                 && rpt->key[i] != lastkey[0]
                 && rpt->key[i] != lastkey[1]
                 && rpt->key[i] != lastkey[2]
                 && rpt->key[i] != lastkey[3]
                 && rpt->key[i] != lastkey[4]
                 && rpt->key[i] != lastkey[5]
#endif
                 )
                {
                  /* Yes.. Add it to the buffer. */

                  /* Map the keyboard scancode to a printable ASCII
                   * character.  There is no support here for function keys
                   * or cursor controls in this version of the driver.
                   */

                  keycode = usbhost_mapscancode(rpt->key[i], rpt->modifier);
                  iinfo("Key %d: %02x keycode:%c modifier: %02x\n",
                         i, rpt->key[i], keycode ? keycode : ' ',
                         rpt->modifier);

                  /* Zero at this point means that the key does not map to a
                   * printable character.
                   */

                  if (keycode != 0)
                    {
                      /* Handle control characters.  Zero after this means
                       * a valid, NUL character.
                       */

                      if ((rpt->modifier & (USBHID_MODIFER_LCTRL |
                                            USBHID_MODIFER_RCTRL)) != 0)
                        {
                          keycode &= 0x1f;
                        }

                      /* Copy the next keyboard character into the user
                       * buffer.
                       */

                      usbhost_putbuffer(priv, keycode);
                    }

                  /* The zero might, however, map to a special keyboard
                   * action (such as a cursor movement or function key).
                   * Attempt to encode the special key.
                   */

#ifdef CONFIG_HIDKBD_ENCODED
                  else
                    {
                      usbhost_encodescancode(priv, rpt->key[i],
                                             rpt->modifier);
                    }
#endif
                }

              /* Save the scancode (or lack thereof) for key debouncing on
               * next keyboard report.
               */

#ifndef CONFIG_HIDKBD_NODEBOUNCE
              lastkey[i] = rpt->key[i];
#endif
            }

          /* Is there data available? */

          newstate = (priv->headndx == priv->tailndx);
          if (!newstate)
            {
              /* Did we just transition from no data available to data
               * available?  If so, wake up any threads waiting for the
               * POLLIN event.
               */

              if (empty)
                {
                  usbhost_pollnotify(priv);
                }

              /* Yes.. Is there a thread waiting for keyboard data now? */

              if (priv->waiting)
                {
                  /* Yes.. wake it up */

                  usbhost_givesem(&priv->waitsem);
                  priv->waiting = false;
                }
            }

          empty = newstate;
          usbhost_givesem(&priv->exclsem);
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

      /* Wait for the required amount (or until a signal is received).  We
       * will wake up when either the delay elapses or we are signalled that
       * the device has been disconnected.
       *
       * If we are getting errors, then sleep longer.  In the event that
       * the keyboard is connected via a hub, there may be a significant
       * amount of time after the keyboard is removed before we are stopped.
       */

      if (nerrors > 1)
        {
          delay = nerrors * CONFIG_HIDKBD_POLLUSEC;
        }
      else
        {
          delay = CONFIG_HIDKBD_POLLUSEC;
        }

      nxsig_usleep(delay);
    }

  /* We get here when the driver is removed.. or when too many errors have
   * been encountered.
   *
   * Make sure that we have exclusive access to the private data structure.
   * There may now be other tasks with the character driver open and actively
   * trying to interact with the class driver.
   */

  usbhost_forcetake(&priv->exclsem);

  /* Indicate that we are no longer running and decrement the reference
   * count held by this thread.  If there are no other users of the class,
   * we can destroy it now.  Otherwise, we have to wait until the all
   * of the file descriptors are closed.
   */

  uinfo("Keyboard removed, polling halted\n");

  flags = enter_critical_section();
  priv->polling = false;

  /* Decrement the reference count held by this thread. */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  /* There are two possibilities:
   * 1) The reference count is greater than zero.  This means that there
   *    are still open references to the keyboard driver.  In this case
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

      usbhost_givesem(&priv->exclsem);
    }

  leave_critical_section(flags);
  return 0;
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
  FAR struct usbhost_epdesc_s epoutdesc;
  int remaining;
  uint8_t found = 0;
  bool done = false;
  int ret;

  DEBUGASSERT(priv != NULL && priv->usbclass.hport != NULL &&
              configdesc != NULL && desclen >= sizeof(struct usb_cfgdesc_s));
  hport = priv->usbclass.hport;

  /* Keep the compiler from complaining about uninitialized variables */

  memset(&epindesc, 0, sizeof(struct usbhost_epdesc_s));
  memset(&epoutdesc, 0, sizeof(struct usbhost_epdesc_s));

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
            FAR struct usb_ifdesc_s *ifdesc =
              (FAR struct usb_ifdesc_s *)configdesc;

            uinfo("Interface descriptor\n");
            DEBUGASSERT(remaining >= USB_SIZEOF_IFDESC);

            /* Did we already find what we needed
             * from a preceding interface?
             */

            if ((found & USBHOST_RQDFOUND) == USBHOST_RQDFOUND)
              {
                /* Yes.. then break out of the loop and use the preceding
                 * interface.
                 */

                done       = true;
              }
            else
              {
                /* Otherwise, save the interface number and discard any
                 * endpoints previously found
                 */

                priv->ifno = ifdesc->ifno;
                found      = USBHOST_IFFOUND;
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

                if (USB_ISEPOUT(epdesc->addr))
                  {
                    /* It is an interrupt OUT endpoint.  There not be more
                     * than one interrupt OUT endpoint.
                     */

                    if ((found & USBHOST_EPOUTFOUND) != 0)
                      {
                        /* Oops.. more than one endpoint.  We don't know what
                         * to do with this.
                         */

                        return -EINVAL;
                      }

                    found |= USBHOST_EPOUTFOUND;

                    /* Save the interrupt OUT endpoint information */

                    epoutdesc.hport        = hport;
                    epoutdesc.addr         = epdesc->addr &
                                             USB_EP_ADDR_NUMBER_MASK;
                    epoutdesc.in           = false;
                    epoutdesc.xfrtype      = USB_EP_ATTR_XFER_INT;
                    epoutdesc.interval     = epdesc->interval;
                    epoutdesc.mxpacketsize =
                      usbhost_getle16(epdesc->mxpacketsize);

                    uinfo("Interrupt OUT EP addr:%d mxpacketsize:%d\n",
                          epoutdesc.addr, epoutdesc.mxpacketsize);
                  }
                else
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
                    epindesc.in           = 1;
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

  if ((found & USBHOST_RQDFOUND) != USBHOST_RQDFOUND)
    {
      uerr("ERROR: Found IF:%s EPIN:%s\n",
           (found & USBHOST_IFFOUND) != 0  ? "YES" : "NO",
           (found & USBHOST_EPINFOUND) != 0 ? "YES" : "NO");
      return -EINVAL;
    }

  /* We are good... Allocate the endpoints.  First, the required interrupt
   * IN endpoint.
   */

  ret = DRVR_EPALLOC(hport->drvr, &epindesc, &priv->epin);
  if (ret < 0)
    {
      uerr("ERROR: Failed to allocate interrupt IN endpoint\n");
      return ret;
    }

  /* Then the optional interrupt OUT endpoint */

  uinfo("Found EPOOUT:%s\n",
       (found & USBHOST_EPOUTFOUND) != 0 ? "YES" : "NO");

  if ((found & USBHOST_EPOUTFOUND) != 0)
    {
      ret = DRVR_EPALLOC(hport->drvr, &epoutdesc, &priv->epout);
      if (ret < 0)
        {
          uerr("ERROR: Failed to allocate interrupt OUT endpoint\n");
          DRVR_EPFREE(hport->drvr, priv->epin);
          return ret;
        }
    }

  uinfo("Endpoints allocated\n");
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
   * use by the keyboard class driver
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
   * dedicated stack (CONFIG_HIDKBD_STACKSIZE).
   */

  uinfo("Start poll task\n");

  /* The inputs to a task started by kthread_create() are very awkard for
   * this purpose.  They are really designed for command line tasks
   * (argc/argv).  So the following is kludge pass binary data when the
   * keyboard poll task is started.
   *
   * First, make sure we have exclusive access to g_priv (what is the
   * likelihood of this being used?  About zero, but we protect it anyway).
   */

  ret = usbhost_takesem(&g_exclsem);
  if (ret < 0)
    {
      usbhost_tdfree(priv);
      goto errout;
    }

  g_priv = priv;

  priv->pollpid = kthread_create("kbdpoll", CONFIG_HIDKBD_DEFPRIO,
                                 CONFIG_HIDKBD_STACKSIZE,
                                 (main_t)usbhost_kbdpoll,
                                 (FAR char * const *)NULL);
  if (priv->pollpid < 0)
    {
      /* Failed to started the poll thread...
       * probably due to memory resources
       */

      usbhost_givesem(&g_exclsem);
      ret = (int)priv->pollpid;
      goto errout;
    }

  /* Now wait for the poll task to get properly initialized */

  ret = usbhost_takesem(&g_syncsem);
  usbhost_givesem(&g_exclsem);

  if (ret < 0)
    {
      goto errout;
    }

  /* Register the driver */

  uinfo("Register driver\n");
  usbhost_mkdevname(priv, devname);
  ret = register_driver(devname, &g_hidkbd_fops, 0666, priv);

  /* We now have to be concerned about asynchronous modification of crefs
   * because the driver has been registered.
   */

errout:
  usbhost_forcetake(&priv->exclsem);
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
 * Returned Value:
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
 * Returned Value:
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
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline uint32_t usbhost_getle32(const uint8_t *val)
{
  /* Little endian means LS halfword first in byte stream */

  return (uint32_t)usbhost_getle16(&val[2]) << 16 |
         (uint32_t)usbhost_getle16(val);
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
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if 0 /* Not used */
static void usbhost_putle32(uint8_t *dest, uint32_t val)
{
  /* Little endian means LS halfword first in byte stream */

  usbhost_putle16(dest, (uint16_t)(val & 0xffff));
  usbhost_putle16(dest + 2, (uint16_t)(val >> 16));
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
 * Returned Value:
 *   On success, zero (OK) is returned.  On failure, an negated errno value
 *   is returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static int usbhost_tdalloc(FAR struct usbhost_state_s *priv)
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

static int usbhost_tdfree(FAR struct usbhost_state_s *priv)
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
           * the driver's usbhost_kbdpoll() task.
           */

          priv->crefs = 1;

          /* Initialize semaphores */

          nxsem_init(&priv->exclsem, 0, 1);
          nxsem_init(&priv->waitsem, 0, 0);

          /* The waitsem semaphore is used for signaling and, hence, should
           * not have priority inheritance enabled.
           */

          nxsem_set_protocol(&priv->waitsem, SEM_PRIO_NONE);

          /* Return the instance of the USB keyboard class driver */

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
 * Name: usbhost_connect
 *
 * Description:
 *   This function implements the connect() method of struct
 *   usbhost_class_s.  This method is a callback into the class
 *   implementation.  It is used to provide the device's configuration
 *   descriptor to the class so that the class may initialize properly
 *
 * Input Parameters:
 *   usbclass   - The USB host class entry previously obtained from a call
 *                to create().
 *   configdesc - A pointer to a uint8_t buffer container the configuration
 *                descriptor.
 *   desclen    - The length in bytes of the configuration descriptor.
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

  DEBUGASSERT(priv != NULL &&
              configdesc != NULL &&
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
   * - Failure occurred before the kbdpoll task was started successfully.
   *   In this case, the disconnection will have to be handled on the worker
   *   task.
   * - Failure occurred after the kbdpoll task was started successfully.  In
   *   this case, the disconnection can be performed on the kbdpoll thread.
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

static int usbhost_disconnected(struct usbhost_class_s *usbclass)
{
  FAR struct usbhost_state_s *priv = (FAR struct usbhost_state_s *)usbclass;

  DEBUGASSERT(priv != NULL);

  /* Set an indication to any users of the keyboard device that the device
   * is no longer available.
   */

  priv->disconnected = true;
  uinfo("Disconnected\n");

  /* Is there a thread waiting for keyboard data that will never come? */

  if (priv->waiting)
    {
      /* Yes.. wake it up */

      usbhost_givesem(&priv->waitsem);
      priv->waiting = false;
    }

  /* Possibilities:
   *
   * - Failure occurred before the kbdpoll task was started successfully.
   *   In this case, the disconnection will have to be handled on the worker
   *   task.
   * - Failure occurred after the kbdpoll task was started successfully.  In
   *   this case, the disconnection can be performed on the kbdpoll thread.
   */

  if (priv->polling)
    {
      /* The polling task is still alive. Signal the keyboard polling task.
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
  ret = usbhost_takesem(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if the keyboard device is still connected.  We need to disable
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
      /* Otherwise, just increment the reference count on the driver */

      priv->crefs++;
      priv->open = true;
      ret        = OK;
    }

  leave_critical_section(flags);

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
  ret = usbhost_takesem(&priv->exclsem);
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

          priv->open    = false;
          priv->headndx = 0;
          priv->tailndx = 0;

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

  usbhost_givesem(&priv->exclsem);
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
  size_t                      nbytes;
  unsigned int                tail;
  int                         ret;

  uinfo("Entry\n");
  DEBUGASSERT(filep && filep->f_inode && buffer);
  inode = filep->f_inode;
  priv  = inode->i_private;

  /* Make sure that we have exclusive access to the private data structure */

  DEBUGASSERT(priv && priv->crefs > 0 && priv->crefs < USBHOST_MAX_CREFS);
  ret = usbhost_takesem(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if the keyboard is still connected.  We need to disable interrupts
   * momentarily to assure that there are no asynchronous disconnect events.
   */

  if (priv->disconnected)
    {
      /* No... the driver is no longer bound to the class.  That means that
       * the USB keyboard is no longer connected.  Refuse any further
       * attempts to access the driver.
       */

      ret = -ENODEV;
    }
  else
    {
      /* Is there keyboard data now? */

      while (priv->tailndx == priv->headndx)
        {
          /* No.. were we open non-blocking? */

          if (filep->f_oflags & O_NONBLOCK)
            {
              /* Yes.. then return a failure */

              ret = -EAGAIN;
              goto errout;
            }

          /* Wait for data to be available */

          uinfo("Waiting...\n");

          priv->waiting = true;
          usbhost_givesem(&priv->exclsem);
          ret = usbhost_takesem(&priv->waitsem);
          if (ret < 0)
            {
              return ret;
            }

          ret = usbhost_takesem(&priv->exclsem);
          if (ret < 0)
            {
              return ret;
            }

          /* Did the keyboard become disconnected while we were waiting */

          if (priv->disconnected)
            {
              ret = -ENODEV;
              goto errout;
            }
        }

      /* Read data from our internal buffer of received characters */

      for (tail  = priv->tailndx, nbytes = 0;
           tail != priv->headndx && nbytes < len;
           nbytes++)
        {
          /* Copy the next keyboard character into the user buffer */

          *buffer++ = priv->kbdbuffer[tail];

          /* Handle wrap-around of the tail index */

          if (++tail >= CONFIG_HIDKBD_BUFSIZE)
            {
              tail = 0;
            }
        }

      ret = nbytes;

      /* Update the tail index (perhaps marking the buffer empty) */

      priv->tailndx = tail;
    }

errout:
  usbhost_givesem(&priv->exclsem);
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
  /* We won't try to write to the keyboard */

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
  ret = usbhost_takesem(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if the keyboard is still connected.  We need to disable interrupts
   * momentarily to assure that there are no asynchronous disconnect events.
   */

  if (priv->disconnected)
    {
      /* No... the driver is no longer bound to the class.  That means that
       * the USB keyboard is no longer connected.  Refuse any further
       * attempts to access the driver.
       */

      ret = -ENODEV;
    }
  else if (setup)
    {
      /* This is a request to set up the poll.  Find an available slot for
       * the poll structure reference
       */

      for (i = 0; i < CONFIG_HIDKBD_NPOLLWAITERS; i++)
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

      if (i >= CONFIG_HIDKBD_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? Notify
       * the POLLIN event if there is buffered keyboard data.
       */

      if (priv->headndx != priv->tailndx)
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

      *slot                = NULL;
      fds->priv            = NULL;
    }

errout:
  nxsem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_kbdinit
 *
 * Description:
 *   Initialize the USB storage HID keyboard class driver.  This function
 *   should be called be platform-specific code in order to initialize and
 *   register support for the USB host HID keyboard class device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success this function will return zero (OK);  A negated errno value
 *   will be returned on failure.
 *
 ****************************************************************************/

int usbhost_kbdinit(void)
{
  /* Perform any one-time initialization of the class implementation */

  nxsem_init(&g_exclsem, 0, 1);
  nxsem_init(&g_syncsem, 0, 0);

  /* The g_syncsem semaphore is used for signaling and, hence, should not
   * have priority inheritance enabled.
   */

  nxsem_set_protocol(&g_syncsem, SEM_PRIO_NONE);

  /* Advertise our availability to support (certain) devices */

  return usbhost_registerclass(&g_hidkbd);
}

#endif /* CONFIG_USBHOST)&& !CONFIG_USBHOST_INT_DISABLE */
