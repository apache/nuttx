/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-m5-cardputer/src/esp32s3_kbd.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/kmalloc.h>
#include <nuttx/input/keyboard.h>

#include <arch/board/board.h>

#include "espressif/esp_gpio.h"
#include "esp32s3-m5-cardputer.h"

#ifdef CONFIG_ESP32S3_M5_CARDPUTER_KEYBOARD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The 56-key keyboard is an 8x7 matrix.  A 74HC138 3-to-8 demultiplexer
 * drives one of eight rows low at a time (selected by SEL0..SEL2); the seven
 * column inputs are read back (a pressed key pulls its column low).
 *
 * Electrically this gives an 8x7 grid.  Following the M5Cardputer wiring,
 * each scanned position (demux index i in 0..7, column j in 0..6) maps to
 * the physical 4x14 key layout as:
 *
 *   physical row    y = i % 4
 *   physical column x = j + (i / 4) * 7
 */

#define KBD_NSEL        3
#define KBD_NCOL        7
#define KBD_NROW_PHYS   4
#define KBD_NCOL_PHYS   14

/* Scan/debounce period */

#define KBD_POLL_TICKS  MSEC2TICK(20)

/* Modifier key positions in the physical layout (row, col) */

#define KBD_SHIFT_Y     2
#define KBD_SHIFT_X     1
#define KBD_CTRL_Y      3
#define KBD_CTRL_X      0
#define KBD_FN_Y        2
#define KBD_FN_X        0

/* Codes emitted for the Fn + navigation cluster (cursor keys).  They are
 * placed above the printable ASCII range so applications reading /dev/kbd0
 * can tell them apart from regular characters.
 */

#define KBD_CODE_UP     0x80
#define KBD_CODE_DOWN   0x81
#define KBD_CODE_LEFT   0x82
#define KBD_CODE_RIGHT  0x83

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32s3_kbd_s
{
  struct keyboard_lowerhalf_s lower;       /* Keyboard upper-half binding */
  struct work_s work;                      /* Scanning work item */

  /* Debounced key state and the code emitted on each key press */

  bool pressed[KBD_NROW_PHYS][KBD_NCOL_PHYS];
  uint8_t code[KBD_NROW_PHYS][KBD_NCOL_PHYS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_sel_pins[KBD_NSEL] =
{
  CARDPUTER_GPIO_KB_A0, CARDPUTER_GPIO_KB_A1, CARDPUTER_GPIO_KB_A2
};

static const uint8_t g_col_pins[KBD_NCOL] =
{
  CARDPUTER_GPIO_KB_C0, CARDPUTER_GPIO_KB_C1, CARDPUTER_GPIO_KB_C2,
  CARDPUTER_GPIO_KB_C3, CARDPUTER_GPIO_KB_C4, CARDPUTER_GPIO_KB_C5,
  CARDPUTER_GPIO_KB_C6
};

/* Physical 4x14 layout.  '\0' marks a modifier or unused position (these are
 * handled by their fixed positions, not emitted as characters).  This table
 * follows the M5Cardputer key layout and should be verified against the
 * hardware if the printed legends do not match.
 */

static const char g_normal[KBD_NROW_PHYS][KBD_NCOL_PHYS] =
{
  {'`',  '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '-', '=', '\b'},
  {'\t', 'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p', '[', ']', '\\'},
  {'\0', '\0', 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ';', '\'', '\r'},
  {'\0', '\0', '\0', 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '/', ' '}
};

static const char g_shift[KBD_NROW_PHYS][KBD_NCOL_PHYS] =
{
  {'~',  '!', '@', '#', '$', '%', '^', '&', '*', '(', ')', '_', '+', '\b'},
  {'\t', 'Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P', '{', '}', '|'},
  {'\0', '\0', 'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', ':', '"', '\r'},
  {'\0', '\0', '\0', 'Z', 'X', 'C', 'V', 'B', 'N', 'M', '<', '>', '?', ' '}
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_kbd_select
 *
 * Description:
 *   Drive the 74HC138 select lines with the given row index (0..7).
 *
 ****************************************************************************/

static void esp32s3_kbd_select(uint8_t row)
{
  int i;

  for (i = 0; i < KBD_NSEL; i++)
    {
      esp_gpiowrite(g_sel_pins[i], (row >> i) & 1);
    }
}

/****************************************************************************
 * Name: esp32s3_kbd_resolve
 *
 * Description:
 *   Resolve a physical key position into a character, applying the SHIFT,
 *   CTRL and FN modifiers.  Returns 0 for modifier/unused positions.
 *
 ****************************************************************************/

static uint8_t esp32s3_kbd_resolve(int y, int x, bool shift, bool ctrl,
                                   bool fn)
{
  char ch;

  /* FN layer: the ; . , / cluster becomes the cursor (arrow) keys, reported
   * as out-of-band codes so applications can act on them (e.g. scrolling).
   */

  if (fn)
    {
      if (y == 2 && x == 11)
        {
          return KBD_CODE_UP;                        /* Fn + ';' */
        }

      if (y == 3 && x == 11)
        {
          return KBD_CODE_DOWN;                      /* Fn + '.' */
        }

      if (y == 3 && x == 10)
        {
          return KBD_CODE_LEFT;                      /* Fn + ',' */
        }

      if (y == 3 && x == 12)
        {
          return KBD_CODE_RIGHT;                     /* Fn + '/' */
        }
    }

  ch = shift ? g_shift[y][x] : g_normal[y][x];

  if (ch == '\0')
    {
      return 0;
    }

  /* CTRL turns a letter into its control code (e.g. CTRL-C -> 0x03) */

  if (ctrl && ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z')))
    {
      ch = (char)(ch & 0x1f);
    }

  return (uint8_t)ch;
}

/****************************************************************************
 * Name: esp32s3_kbd_worker
 *
 * Description:
 *   Periodically scan the matrix and report key transitions.
 *
 ****************************************************************************/

static void esp32s3_kbd_worker(FAR void *arg)
{
  FAR struct esp32s3_kbd_s *priv = arg;
  bool scan[KBD_NROW_PHYS][KBD_NCOL_PHYS];
  bool shift;
  bool ctrl;
  bool fn;
  int i;
  int j;
  int y;
  int x;

  memset(scan, 0, sizeof(scan));

  /* Read the full matrix into the physical layout */

  for (i = 0; i < (1 << KBD_NSEL); i++)
    {
      esp32s3_kbd_select((uint8_t)i);
      up_udelay(5);

      for (j = 0; j < KBD_NCOL; j++)
        {
          if (!esp_gpioread(g_col_pins[j]))
            {
              /* Map the (demux index i, column j) to the physical 4x14 grid
               * exactly as the M5Cardputer firmware does:
               *   y = 3 - (i % 4)
               *   x = (i > 3) ? (2 * j) : (2 * j + 1)
               */

              y = (KBD_NROW_PHYS - 1) - (i % KBD_NROW_PHYS);
              x = (i >= KBD_NROW_PHYS) ? (2 * j) : (2 * j + 1);
              scan[y][x] = true;
            }
        }
    }

  /* Latch the modifier state for this scan */

  shift = scan[KBD_SHIFT_Y][KBD_SHIFT_X];
  ctrl  = scan[KBD_CTRL_Y][KBD_CTRL_X];
  fn    = scan[KBD_FN_Y][KBD_FN_X];

  /* Emit press/release transitions */

  for (y = 0; y < KBD_NROW_PHYS; y++)
    {
      for (x = 0; x < KBD_NCOL_PHYS; x++)
        {
          if (scan[y][x] == priv->pressed[y][x])
            {
              continue;
            }

          priv->pressed[y][x] = scan[y][x];

          if (scan[y][x])
            {
              uint8_t code = esp32s3_kbd_resolve(y, x, shift, ctrl, fn);
#ifdef CONFIG_ESP32S3_M5_CARDPUTER_KBD_DEBUG
              syslog(LOG_INFO, "kbd: press y=%d x=%d code=0x%02x '%c'\n",
                     y, x, code, (code >= 0x20 && code < 0x7f) ? code : '.');
#endif
              if (code != 0)
                {
                  priv->code[y][x] = code;
                  keyboard_event(&priv->lower, code, KEYBOARD_PRESS);
                }
            }
          else if (priv->code[y][x] != 0)
            {
              keyboard_event(&priv->lower, priv->code[y][x],
                             KEYBOARD_RELEASE);
              priv->code[y][x] = 0;
            }
        }
    }

  work_queue(LPWORK, &priv->work, esp32s3_kbd_worker, priv, KBD_POLL_TICKS);
}

/****************************************************************************
 * Name: esp32s3_kbd_open / esp32s3_kbd_close
 ****************************************************************************/

static int esp32s3_kbd_open(FAR struct keyboard_lowerhalf_s *lower)
{
  return OK;
}

static int esp32s3_kbd_close(FAR struct keyboard_lowerhalf_s *lower)
{
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_kbd_initialize
 *
 * Description:
 *   Initialize and register the Cardputer matrix keyboard at devpath
 *   (e.g. "/dev/kbd0").
 *
 ****************************************************************************/

int esp32s3_kbd_initialize(FAR const char *devpath)
{
  FAR struct esp32s3_kbd_s *priv;
  int ret;
  int i;

  priv = kmm_zalloc(sizeof(struct esp32s3_kbd_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->lower.open  = esp32s3_kbd_open;
  priv->lower.close = esp32s3_kbd_close;
  priv->lower.write = NULL;

  /* Configure the demux select lines as outputs and the column inputs with
   * pull-ups (a pressed key pulls the selected column low).
   */

  for (i = 0; i < KBD_NSEL; i++)
    {
      esp_configgpio(g_sel_pins[i], OUTPUT);
      esp_gpiowrite(g_sel_pins[i], 1);
    }

  for (i = 0; i < KBD_NCOL; i++)
    {
      esp_configgpio(g_col_pins[i], INPUT_PULLUP);
    }

  ret = keyboard_register(&priv->lower, devpath,
                          CONFIG_ESP32S3_M5_CARDPUTER_KBD_BUFNUM);
  if (ret < 0)
    {
      ierr("ERROR: keyboard_register(%s) failed: %d\n", devpath, ret);
      kmm_free(priv);
      return ret;
    }

  /* Start the periodic scan */

  ret = work_queue(LPWORK, &priv->work, esp32s3_kbd_worker, priv,
                   KBD_POLL_TICKS);
  if (ret < 0)
    {
      ierr("ERROR: failed to start keyboard scan: %d\n", ret);
      keyboard_unregister(&priv->lower, devpath);
      kmm_free(priv);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_ESP32S3_M5_CARDPUTER_KEYBOARD */
