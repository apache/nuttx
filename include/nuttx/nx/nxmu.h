/****************************************************************************
 * include/nuttx/nx/nxmu.h
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

#ifndef __INCLUDE_NUTTX_NX_NXMU_H
#define __INCLUDE_NUTTX_NX_NXMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <mqueue.h>

#include <nuttx/semaphore.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxcursor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_NX_MXSERVERMSGS
#  define CONFIG_NX_MXSERVERMSGS 32 /* Number of pending messages in server MQ */
#endif

#ifndef CONFIG_NX_MXCLIENTMSGS
#  define CONFIG_NX_MXCLIENTMSGS 16 /* Number of pending messages in each client MQ */
#endif

/* Used to create unique client MQ name */

#define NX_CLIENT_MQNAMEFMT  "nxc%d"
#define NX_CLIENT_MXNAMELEN  (12)

#define NX_MXSVRMSGLEN       (64) /* Maximum size of a client->server command */
#define NX_MXEVENTLEN        (64) /* Maximum size of an event */
#define NX_MXCLIMSGLEN       (64) /* Maximum size of a server->client message */

/* Message priorities -- they must all be at the same priority to assure
 * FIFO execution.
 */

#define NX_CLIMSG_PRIO 42
#define NX_SVRMSG_PRIO 42

/* Handy macros */

#define nxmu_semgive(sem)    _SEM_POST(sem) /* To match nxmu_semtake() */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Client/Connection structures *********************************************/

/* Client state */

enum nx_clistate_e
{
  NX_CLISTATE_NOTCONNECTED = 0,   /* Waiting for server to acknowledge connection */
  NX_CLISTATE_CONNECTED,          /* Connection established (normal state) */
  NX_CLISTATE_DISCONNECT_PENDING, /* Waiting for server to acknowledge disconnect */
};

/* This structure represents a connection between the client and the server */

struct nxmu_conn_s
{
  /* This number uniquely identifies the client */

  int cid;                /* Client ID (CID) */
  uint8_t state;          /* See enum nx_clistate_e */

  /* These are only usable on the client side of the connection */

  mqd_t crdmq;            /* MQ to read from the server (may be non-blocking) */
  mqd_t cwrmq;            /* MQ to write to the server (blocking) */

  /* These are only usable on the server side of the connection */

  mqd_t swrmq;            /* MQ to write to the client */
};

/* Message IDs **************************************************************/

enum nxmsg_e
{
  /* Server-to-Client Messages **********************************************/

  NX_CLIMSG_CONNECTED = 1,    /* The server has completed the connection and is ready */
  NX_CLIMSG_DISCONNECTED,     /* The server has disconnected */
  NX_CLIMSG_REDRAW,           /* Re-draw the specified window */
  NX_CLIMSG_NEWPOSITION,      /* New window size/position */
  NX_CLIMSG_MOUSEIN,          /* New mouse positional data available for window */
  NX_CLIMSG_KBDIN,            /* New keypad input available for window */
  NX_CLIMSG_EVENT,            /* Server->client event */

  /* Client-to-Server Messages **********************************************/

  NX_SVRMSG_CONNECT,          /* Establish connection with new NX server client */
  NX_SVRMSG_DISCONNECT,       /* Tear down connection with terminating client */
  NX_SVRMSG_OPENWINDOW,       /* Create a new window */
  NX_SVRMSG_CLOSEWINDOW,      /* Close an existing window */
  NX_SVRMSG_BLOCKED,          /* The window is blocked */
  NX_SVRMSG_SYNCH,            /* Window syncrhonization request */
  NX_SVRMSG_CURSOR_ENABLE,    /* Enable/disablel cursor presentation */
  NX_SVRMSG_CURSOR_IMAGE,     /* Set cursor image */
  NX_SVRMSG_CURSOR_SETPOS,    /* Set cursor position */
  NX_SVRMSG_REQUESTBKGD,      /* Open the background window */
  NX_SVRMSG_RELEASEBKGD,      /* Release the background window */
  NX_SVRMSG_SETPOSITION,      /* Window position has changed */
  NX_SVRMSG_SETSIZE,          /* Window size has changed */
  NX_SVRMSG_GETPOSITION,      /* Get the current window position and size */
  NX_SVRMSG_RAISE,            /* Move the window to the top */
  NX_SVRMSG_LOWER,            /* Move the window to the bottom */
  NX_SVRMSG_MODAL,            /* Select/de-slect window modal state */
  NX_SVRMSG_SETVISIBILITY,    /* Show or hide a window */
  NX_SVRMSG_SETPIXEL,         /* Set a single pixel in the window with a color */
  NX_SVRMSG_FILL,             /* Fill a rectangle in the window with a color */
  NX_SVRMSG_GETRECTANGLE,     /* Get a rectangular region in the window */
  NX_SVRMSG_FILLTRAP,         /* Fill a trapezoidal region in the window with a color */
  NX_SVRMSG_MOVE,             /* Move a rectangular region within the window */
  NX_SVRMSG_BITMAP,           /* Copy a rectangular bitmap into the window */
  NX_SVRMSG_SETBGCOLOR,       /* Set the color of the background */
  NX_SVRMSG_MOUSEIN,          /* New mouse report from mouse client */
  NX_SVRMSG_KBDIN,            /* New keyboard report from keyboard client */
  NX_SVRMSG_REDRAWREQ         /* Request re-drawing of rectangular region */
};

/* Server-to-Client Message Structures **************************************/

/* The generic message structure.  All messages begin with this form. */

struct nxclimsg_s
{
  uint32_t msgid;                  /* Any of nxclimsg_e */
};

/* The server is now connected */

struct nxclimsg_connected_s
{
  uint32_t msgid;                  /* NX_CLIMSG_REDRAW_CONNECTED */
};

/* The server is now disconnected */

struct nxclimsg_disconnected_s
{
  uint32_t msgid;                  /* NX_CLIMSG_REDRAW_DISCONNECTED */
};

/* This message is received when a requested window has been opened.  If wnd
 * is NULL then errorcode is the errno value that provides the explanation of
 * the error.
 */

struct nxclimsg_redraw_s
{
  uint32_t msgid;                /* NX_CLIMSG_REDRAW */
  FAR struct nxbe_window_s *wnd; /* The handle to the window to redraw in */
  FAR struct nxgl_rect_s rect;   /* The rectangle to be redrawn */
  bool more;                     /* true: more redraw messages follow */
};

/* This message informs the client of the new size
 * or position of the window
 */

struct nxclimsg_newposition_s
{
  uint32_t msgid;                /* NX_CLIMSG_NEWPOSITION */
  FAR struct nxbe_window_s *wnd; /* The window whose position/size has changed */
  FAR struct nxgl_size_s size;   /* The current window size */
  FAR struct nxgl_point_s pos;   /* The current window position */
  FAR struct nxgl_rect_s bounds; /* Size of screen */
};

/* This message reports a new mouse event to a particular window */

#ifdef CONFIG_NX_XYINPUT
struct nxclimsg_mousein_s
{
  uint32_t msgid;                /* NX_SVRMSG_MOUSEIN */
  FAR struct nxbe_window_s *wnd; /* The handle of window receiving mouse input */
  struct nxgl_point_s pos;       /* Mouse X/Y position */
  uint8_t buttons;               /* Mouse button set */
};
#endif

/* This message reports a new keypad event to a particular window */

#ifdef CONFIG_NX_KBD
struct nxclimsg_kbdin_s
{
  uint32_t msgid;                /* NX_CLIMSG_KBDIN */
  FAR struct nxbe_window_s *wnd; /* The handle of window receiving keypad input */
  uint8_t nch;                   /* Number of characters received */
  uint8_t ch[1];                 /* Array of received characters */
};
#endif

/* This message provides server event notifications to the client. */

struct nxclimsg_event_s
{
  uint32_t msgid;                /* NX_CLIMSG_BLOCKED */
  FAR struct nxbe_window_s *wnd; /* The window that is blocked */
  FAR void *arg;                 /* User argument */
  enum nx_event_e event;         /* Server event */
};

/* Client-to-Server Message Structures **************************************/

/* The generic message structure.
 * All server messages begin with this form.
 * Also messages that have no additional data fields use this structure.
 * This includes: NX_SVRMSG_CONNECT and NX_SVRMSG_DISCONNECT.
 */

struct nxsvrmsg_s                 /* Generic server message */
{
  uint32_t msgid;                 /* One of enum nxsvrmsg_e */
  FAR struct nxmu_conn_s *conn;   /* The specific connection sending the message */
};

/* This message requests the server to create a new window */

struct nxsvrmsg_openwindow_s
{
  uint32_t msgid;                 /* NX_SVRMSG_OPENWINDOW */
  FAR struct nxbe_window_s *wnd;  /* The pre-allocated window structure */
};

/* This message informs the server that client wishes to close a window */

struct nxsvrmsg_closewindow_s
{
  uint32_t msgid;                  /* NX_SVRMSG_CLOSEWINDOW */
  FAR struct nxbe_window_s *wnd;   /* The window to be closed */
};

/* This message is just a marker that is queued and forwarded by the server
 * (NX_CLIMSG_BLOCKED).  Messages to the window were blocked just after this
 * message was sent.  Receipt of this message indicates both that the window
 * blocked and that there are no further queued messages for the window.
 */

struct nxsvrmsg_blocked_s
{
  uint32_t msgid;                /* NX_SVRMSG_BLOCKED */
  FAR struct nxbe_window_s *wnd; /* The window that is blocked */
  FAR void *arg;                 /* User argument */
};

/* Synchronization request.  This is essentially an 'echo':  The NX server
 * will receive the synchronization request and simply respond with a
 * synchronized event.
 */

struct nxsvrmsg_synch_s
{
  uint32_t msgid;                /* NX_SVRMSG_SYNCH */
  FAR struct nxbe_window_s *wnd; /* The window that requires synch'ing */
  FAR void *arg;                 /* User argument */
};

#if defined(CONFIG_NX_SWCURSOR) || defined(CONFIG_NX_HWCURSOR)
/* Enable/disable cursor */

struct nxsvrmsg_curenable_s
{
  uint32_t msgid;                /* NX_SVRMSG_CURSOR_ENABLE */
  bool enable;                   /* True: show the cursor, false: hide the cursor */
};

#if defined(CONFIG_NX_HWCURSORIMAGE) || defined(CONFIG_NX_SWCURSOR)
/* Set cursor image. */

struct nxsvrmsg_curimage_s
{
  uint32_t msgid;                    /* NX_SVRMSG_CURSOR_IMAGE */
  FAR struct nx_cursorimage_s image  /* Describes the cursor image */
};
#endif

/* Set cursor position. */

struct nxsvrmsg_curpos_s
{
  uint32_t msgid;                  /* NX_SVRMSG_CURSOR_SETPOS */
  FAR struct nxgl_point_s pos;     /* The new cursor position */
};
#endif

/* This message requests the server to create a new window */

struct nxsvrmsg_requestbkgd_s
{
  uint32_t msgid;                  /* NX_SVRMSG_REQUESTBKGD */
  FAR struct nxmu_conn_s *conn;    /* The specific connection sending the message */

  FAR const struct nx_callback_s *cb; /* Event handling callbacks */
  FAR void *arg;                      /* Client argument used with callbacks */
};

/* This message informs the server that client wishes to close a window */

struct nxsvrmsg_releasebkgd_s
{
  uint32_t msgid;                  /* NX_SVRMSG_RELEASEBKGD */
};

/* This message informs the server that the size
 * or position of the window has changed
 */

struct nxsvrmsg_setposition_s
{
  uint32_t msgid;                  /* NX_SVRMSG_SETPOSITION */
  FAR struct nxbe_window_s *wnd;   /* The window whose position/size has changed */
  FAR struct nxgl_point_s pos;     /* The new window position */
};

/* This message informs the server that the size
 * or position of the window has changed
 */

struct nxsvrmsg_setsize_s
{
  uint32_t msgid;                  /* NX_SVRMSG_SETSIZE */
  FAR struct nxbe_window_s *wnd;   /* The window whose position/size has changed */
  FAR struct nxgl_size_s  size;    /* The new window size */
};

/* This message informs the server that the size
 * or position of the window has changed
 */

struct nxsvrmsg_getposition_s
{
  uint32_t msgid;                  /* NX_SVRMSG_FETPOSITION */
  FAR struct nxbe_window_s *wnd;   /* The window whose position/size has changed */
};

/* This message informs the server to raise this window to
 * the top of the display
 */

struct nxsvrmsg_raise_s
{
  uint32_t msgid;                  /* NX_SVRMSG_RAISE */
  FAR struct nxbe_window_s *wnd;   /* The window to be raised */
};

/* This message informs the server to lower this window to
 * the bottom of the display
 */

struct nxsvrmsg_lower_s
{
  uint32_t msgid;                  /* NX_SVRMSG_LOWER */
  FAR struct nxbe_window_s *wnd;   /* The window to be lowered */
};

/* This message either (1) raises a window to the top of the display and
 * selects the modal state, or (2) de-selects the modal state.
 */

struct nxsvrmsg_modal_s
{
  uint32_t msgid;                  /* NX_SVRMSG_MODAL */
  FAR struct nxbe_window_s *wnd;   /* The window to be modified */
  bool modal;                      /* True: enter modal state; False: leave modal state */
};

/* This message either (1) hides a visible window, or (2) makes a hidden
 * window visible.
 */

struct nxsvrmsg_setvisibility_s
{
  uint32_t msgid;                  /* NX_SVRMSG_SETVISIBILITY */
  FAR struct nxbe_window_s *wnd;   /* The window to be modified */
  bool hide;                       /* True: Hide window; False: show window */
};

/* Set a single pixel in the window with a color */

struct nxsvrmsg_setpixel_s
{
  uint32_t msgid;                  /* NX_SVRMSG_SETPIXEL */
  FAR struct nxbe_window_s *wnd;   /* The window to fill  */
  struct nxgl_point_s pos;         /* The position of the pixel in the window */

  nxgl_mxpixel_t color[CONFIG_NX_NPLANES]; /* Color to use in the fill */
};

/* Fill a rectangle in the window with a color */

struct nxsvrmsg_fill_s
{
  uint32_t msgid;                  /* NX_SVRMSG_FILL */
  FAR struct nxbe_window_s *wnd;   /* The window to fill  */
  struct nxgl_rect_s rect;         /* The rectangle in the window to fill */

  nxgl_mxpixel_t color[CONFIG_NX_NPLANES]; /* Color to use in the fill */
};

/* Get a rectangular region from the window */

struct nxsvrmsg_getrectangle_s
{
  uint32_t msgid;                  /* NX_SVRMSG_GETRECTANGLE */
  FAR struct nxbe_window_s *wnd;   /* The window to get from  */
  struct nxgl_rect_s rect;         /* The rectangle in the window to get from */
  unsigned int plane;              /* The plane number to read */
  FAR uint8_t *dest;               /* Memory location in which to store the graphics data */
  unsigned int deststride;         /* Width of the destination memory in bytes */
  sem_t *sem_done;                 /* Semaphore to report when command is done. */
};

/* Fill a trapezoidal region in the window with a color */

struct nxsvrmsg_filltrapezoid_s
{
  uint32_t msgid;                  /* NX_SVRMSG_FILLTRAP */
  FAR struct nxbe_window_s *wnd;   /* The window to fill  */
  FAR struct nxgl_rect_s clip;     /* The clipping window */
  struct nxgl_trapezoid_s trap;    /* The trapezoidal region in the window to fill */

  nxgl_mxpixel_t color[CONFIG_NX_NPLANES]; /* Color to use in the fill */
};

/* Move a rectangular region within the window */

struct nxsvrmsg_move_s
{
  uint32_t msgid;                  /* NX_SVRMSG_MOVE */
  FAR struct nxbe_window_s *wnd;   /* The window within which the move is done  */
  struct nxgl_rect_s rect;         /* Describes the rectangular region to move */
  struct nxgl_point_s offset;      /* The offset to move the region */
};

/* Copy a rectangular bitmap into the window */

struct nxsvrmsg_bitmap_s
{
  uint32_t msgid;                 /* NX_SVRMSG_BITMAP */
  FAR struct nxbe_window_s *wnd;  /* The window with will receive the bitmap image  */
  struct nxgl_rect_s dest;        /* Destination location of the bitmap in the window */

  FAR const void *src[CONFIG_NX_NPLANES]; /* The start of the source image. */

  struct nxgl_point_s origin;     /* Offset into the source image data */
  unsigned int stride;            /* The width of the full source image in pixels. */
  sem_t *sem_done;                /* Semaphore to report when command is done. */
};

/* Set the color of the background */

struct nxsvrmsg_setbgcolor_s
{
  uint32_t msgid;                          /* NX_SVRMSG_SETBGCOLOR */
  nxgl_mxpixel_t color[CONFIG_NX_NPLANES]; /* Color to use in the background */
};

/* This message reports a new mouse event from a hardware controller
 * attached to the server as a regular client (this message may have
 * even been sent from an interrupt handler).
 */

#ifdef CONFIG_NX_XYINPUT
struct nxsvrmsg_mousein_s
{
  uint32_t msgid;                  /* NX_SVRMSG_MOUSEIN */
  struct nxgl_point_s pt;          /* Mouse X/Y position */
  uint8_t buttons;                 /* Mouse button set */
};
#endif

/* This message reports a new keyboard event from a hardware controller
 * attached to some kind of keypad (this message may have even been sent
 * from an interrupt handler).
 */

#ifdef CONFIG_NX_KBD
struct nxsvrmsg_kbdin_s
{
  uint32_t msgid;                  /* NX_SVRMSG_KBDIN */
  uint8_t nch ;                    /* Number of characters received */
  uint8_t ch[1];                   /* Array of received characters */
};
#endif

/* Request re-drawing of rectangular region */

struct nxsvrmsg_redrawreq_s
{
  uint32_t msgid;                  /* NX_SVRMSG_REDRAWREQ */
  FAR struct nxbe_window_s *wnd;   /* The window to be redrawn  */
  struct nxgl_rect_s rect;         /* Describes the rectangular region to be redrawn */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmu_start
 *
 * Description:
 *   nxmu_start() provides a wrapper function to simplify and standardize
 *   the starting of the NX server.
 *
 *   nxmu_start() can be called (indirectly) from applications via the
 *   boardctl() interface with the BOARDIOC_NX_START command.
 *
 * Input Parameters:
 *   display - Display number served by this NXMU instance.
 *   plane   - Plane number to use for display info
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  This indicates that the NX server
 *   has been successfully started, is running, and waiting to accept
 *   connections from NX clients.
 *
 *   A negated errno value is returned on failure.  The errno value indicates
 *   the nature of the failure.
 *
 ****************************************************************************/

int nxmu_start(int display, int plane);

/****************************************************************************
 * Name: nxmu_semtake
 *
 * Description:
 *   Take the semaphore, handling EINTR wakeups.  See the nxmu_semgive macro.
 *
 * Input Parameters:
 *   sem - the semaphore to be taken.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxmu_semtake(sem_t *sem);

/****************************************************************************
 * Name: nxmu_sendserver
 *
 * Description:
 *  Send a message to the server at NX_SVRMSG_PRIO priority
 *
 * Input Parameters:
 *   conn   - A pointer to the server connection structure
 *   msg    - A pointer to the message to send
 *   msglen - The length of the message in bytes.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxmu_sendserver(FAR struct nxmu_conn_s *conn,
                    FAR const void *msg, size_t msglen);

/****************************************************************************
 * Name: nxmu_sendwindow
 *
 * Description:
 *  Send a message to the server destined for a specific window at
 *  NX_SVRMSG_PRIO priority
 *
 * Input Parameters:
 *   wnd    - A pointer to the back-end window structure
 *   msg    - A pointer to the message to send
 *   msglen - The length of the message in bytes.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int nxmu_sendwindow(FAR struct nxbe_window_s *wnd, FAR const void *msg,
                    size_t msglen);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_NX_NXMU_H */
