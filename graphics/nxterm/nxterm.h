/****************************************************************************
 * nuttx/graphics/nxterm/nxterm.h
 *
 *   Copyright (C) 2012, 2014, 2017 Gregory Nutt. All rights reserved.
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

#ifndef __GRAPHICS_NXTERM_NXTERM_H
#define __GRAPHICS_NXTERM_NXTERM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxtk.h>
#include <nuttx/nx/nxfonts.h>
#include <nuttx/nx/nxterm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* NxTerm Definitions ****************************************************/

/* Bitmap flags */

#define BMFLAGS_NOGLYPH    (1 << 0) /* No glyph available, use space */
#define BM_ISSPACE(bm)     (((bm)->flags & BMFLAGS_NOGLYPH) != 0)

/* Device path formats */

#define NX_DEVNAME_FORMAT  "/dev/nxterm%d"
#define NX_DEVNAME_SIZE    16

/* Semaphore protection */

#define NO_HOLDER          (pid_t)-1

/* VT100 escape sequence processing */

#define VT100_MAX_SEQUENCE 3

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Identifies the state of the VT100 escape sequence processing */

enum nxterm_vt100state_e
{
  VT100_NOT_CONSUMED = 0, /* Character is not part of a VT100 escape sequence */
  VT100_CONSUMED,         /* Character was consumed as part of the VT100 escape processing */
  VT100_PROCESSED,        /* The full VT100 escape sequence was processed */
  VT100_ABORT             /* Invalid/unsupported character in buffered escape sequence */
};

/* Describes on set of console window callbacks */

struct nxterm_state_s;
struct nxterm_operations_s
{
  int (*fill)(FAR struct nxterm_state_s *priv,
              FAR const struct nxgl_rect_s *rect,
              nxgl_mxpixel_t wcolor[CONFIG_NX_NPLANES]);
#ifndef CONFIG_NX_WRITEONLY
  int (*move)(FAR struct nxterm_state_s *priv,
              FAR const struct nxgl_rect_s *rect,
              FAR const struct nxgl_point_s *offset);
#endif
  int (*bitmap)(FAR struct nxterm_state_s *priv,
                FAR const struct nxgl_rect_s *dest,
                FAR const void *src[CONFIG_NX_NPLANES],
                FAR const struct nxgl_point_s *origin,
                unsigned int stride);
};

/* Describes on character on the display */

struct nxterm_bitmap_s
{
  uint8_t code;                        /* Character code */
  uint8_t flags;                       /* See BMFLAGS_* */
  struct nxgl_point_s pos;             /* Character position */
};

/* Describes the state of one NX console driver*/

struct nxterm_state_s
{
  FAR const struct nxterm_operations_s *ops; /* Window operations */
  FAR void *handle;                          /* The window handle */
  FAR struct nxterm_window_s wndo;           /* Describes the window and font */
  sem_t exclsem;                             /* Forces mutually exclusive access */
#ifdef CONFIG_DEBUG_GRAPHICS
  pid_t holder;                              /* Deadlock avoidance */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  bool unlinked;                             /* True:  Driver has been unlinked */
  uint8_t orefs;                             /* Open reference count */
#endif
  uint8_t minor;                             /* Device minor number */

  /* Text output support */

  uint8_t fheight;                           /* Max height of a font in pixels */
  uint8_t fwidth;                            /* Max width of a font in pixels */
  uint8_t spwidth;                           /* The width of a space */

  uint16_t maxchars;                         /* Size of the bm[] array */
  uint16_t nchars;                           /* Number of chars in the bm[] array */

  struct nxgl_point_s fpos;                  /* Next display position */

  /* VT100 escape sequence processing */

  char seq[VT100_MAX_SEQUENCE];              /* Buffered characters */
  uint8_t nseq;                              /* Number of buffered characters */

  /* Font cache data storage */

  FCACHE fcache;                             /* Font cache handle */
  struct nxterm_bitmap_s cursor;
  struct nxterm_bitmap_s bm[CONFIG_NXTERM_MXCHARS];

  /* Keyboard input support */

#ifdef CONFIG_NXTERM_NXKBDIN
  sem_t waitsem;                             /* Supports waiting for input data */
  uint8_t nwaiters;                          /* Number of threads waiting for data */
  uint8_t head;                              /* rxbuffer head/input index */
  uint8_t tail;                              /* rxbuffer tail/output index */

  uint8_t rxbuffer[CONFIG_NXTERM_KBDBUFSIZE];

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

  struct pollfd *fds[CONFIG_NXTERM_NPOLLWAITERS];
#endif /* CONFIG_NXTERM_NXKBDIN */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This is the common NX driver file operations */

extern const struct file_operations g_nxterm_drvrops;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Semaphore helpers */

#ifdef CONFIG_DEBUG_GRAPHICS
int nxterm_semwait(FAR struct nxterm_state_s *priv);
int nxterm_sempost(FAR struct nxterm_state_s *priv);
#else
#  define nxterm_semwait(p) nxsem_wait(&p->exclsem)
#  define nxterm_sempost(p) nxsem_post(&p->exclsem)
#endif

/* Common device registration/un-registration */

FAR struct nxterm_state_s *nxterm_register(NXTERM handle,
    FAR struct nxterm_window_s *wndo, FAR const struct nxterm_operations_s *ops,
    int minor);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
void nxterm_unregister(FAR struct nxterm_state_s *priv);
#endif

/* Driver methods */

#ifdef CONFIG_NXTERM_NXKBDIN
ssize_t nxterm_read(FAR struct file *filep, FAR char *buffer, size_t len);
int nxterm_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup);
#endif

/* IOCTL handlers */

void nxterm_redraw(NXTERM handle, FAR const struct nxgl_rect_s *rect,
                   bool more);
#ifdef CONFIG_NXTERM_NXKBDIN
void nxterm_kbdin(NXTERM handle, FAR const uint8_t *buffer, uint8_t buflen);
#endif
int nxterm_resize(NXTERM handle, FAR const struct nxgl_size_s *size);

/* VT100 Terminal emulation */

enum nxterm_vt100state_e nxterm_vt100(FAR struct nxterm_state_s *priv, char ch);

/* Generic text display helpers */

void nxterm_home(FAR struct nxterm_state_s *priv);
void nxterm_clear(FAR struct nxterm_state_s *priv);
void nxterm_newline(FAR struct nxterm_state_s *priv);
FAR const struct nxterm_bitmap_s *nxterm_addchar(FAR struct nxterm_state_s *priv,
    uint8_t ch);
int nxterm_hidechar(FAR struct nxterm_state_s *priv,
    FAR const struct nxterm_bitmap_s *bm);
int nxterm_backspace(FAR struct nxterm_state_s *priv);
void nxterm_fillchar(FAR struct nxterm_state_s *priv,
    FAR const struct nxgl_rect_s *rect, FAR const struct nxterm_bitmap_s *bm);

void nxterm_putc(FAR struct nxterm_state_s *priv, uint8_t ch);
void nxterm_showcursor(FAR struct nxterm_state_s *priv);
void nxterm_hidecursor(FAR struct nxterm_state_s *priv);

/* Scrolling support */

void nxterm_scroll(FAR struct nxterm_state_s *priv, int scrollheight);

#endif /* __GRAPHICS_NXTERM_NXTERM_H */
