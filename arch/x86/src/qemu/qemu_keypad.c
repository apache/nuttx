/****************************************************************************
 * arch/x86/src/qemu_keypad.c
 *
 *   Copyright (C) 2013 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/fs/fs.h>

#include <stdint.h>
#include <semaphore.h>
#include <errno.h>
#include <unistd.h>
#include <sched.h>

#include <arch/io.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef enum
{
  /* The keyboard syms have been cleverly chosen to map to ASCII */

  KEY_UNKNOWN        = 0,
  KEY_NOKEY          = 0,
  KEY_FIRST          = 0,
  KEY_BACKSPACE      = 8,
  KEY_TAB            = 9,
  KEY_CLEAR          = 12,
  KEY_RETURN         = 13,
  KEY_PAUSE          = 19,
  KEY_ESCAPE         = 27,

  KEY_SPACE          = 32,
  KEY_EXCLAIM        = 33,
  KEY_QUOTEDBL       = 34,
  KEY_HASH           = 35,
  KEY_DOLLAR         = 36,
  KEY_AMPERSAND      = 38,
  KEY_QUOTE          = 39,
  KEY_LEFTPAREN      = 40,
  KEY_RIGHTPAREN     = 41,
  KEY_ASTERISK       = 42,
  KEY_PLUS           = 43,
  KEY_COMMA          = 44,
  KEY_MINUS          = 45,
  KEY_PERIOD         = 46,
  KEY_SLASH          = 47,
  KEY_0              = 48,
  KEY_1              = 49,
  KEY_2              = 50,
  KEY_3              = 51,
  KEY_4              = 52,
  KEY_5              = 53,
  KEY_6              = 54,
  KEY_7              = 55,
  KEY_8              = 56,
  KEY_9              = 57,

  KEY_COLON          = 58,
  KEY_SEMICOLON      = 59,
  KEY_LESS           = 60,
  KEY_EQUALS         = 61,
  KEY_GREATER        = 62,
  KEY_QUESTION       = 63,
  KEY_AT             = 64,

  KEY_LEFTBRACKET    = 91,
  KEY_BACKSLASH      = 92,
  KEY_RIGHTBRACKET   = 93,
  KEY_CARET          = 94,
  KEY_UNDERSCORE     = 95,
  KEY_BACKQUOTE      = 96,
  KEY_a              = 97,
  KEY_b              = 98,
  KEY_c              = 99,
  KEY_d              = 100,
  KEY_e              = 101,
  KEY_f              = 102,
  KEY_g              = 103,
  KEY_h              = 104,
  KEY_i              = 105,
  KEY_j              = 106,
  KEY_k              = 107,
  KEY_l              = 108,
  KEY_m              = 109,
  KEY_n              = 110,
  KEY_o              = 111,
  KEY_p              = 112,
  KEY_q              = 113,
  KEY_r              = 114,
  KEY_s              = 115,
  KEY_t              = 116,
  KEY_u              = 117,
  KEY_v              = 118,
  KEY_w              = 119,
  KEY_x              = 120,
  KEY_y              = 121,
  KEY_z              = 122,
  KEY_DELETE         = 127,
  KEY_DEBUG          = 138,     /* 0x80+0x0a */

  /* Arrows + Home/End pad */

  KEY_S100           = 173,
  KEY_UP             = 173,
  KEY_DOWN           = 174,
  KEY_RIGHT          = 175,
  KEY_LEFT           = 176,
  KEY_INSERT         = 177,
  KEY_HOME           = 178,
  KEY_END            = 179,
  KEY_PAGEUP         = 180,
  KEY_PAGEDOWN       = 181,

  /* Function keys */

  KEY_F1             = 182,
  KEY_F2             = 183,
  KEY_F3             = 184,
  KEY_F4             = 185,
  KEY_F5             = 186,
  KEY_F6             = 187,
  KEY_F7             = 188,
  KEY_F8             = 189,
  KEY_F9             = 190,
  KEY_F10            = 191,
  KEY_F11            = 192,
  KEY_F12            = 193,
  KEY_F13            = 194,
  KEY_F14            = 195,
  KEY_F15            = 196,

  /* Key state modifier keys */

  KEY_NUMLOCK        = 200,
  KEY_CAPSLOCK       = 201,
  KEY_SCROLLOCK      = 202,
  KEY_RSHIFT         = 203,
  KEY_LSHIFT         = 204,
  KEY_RCTRL          = 205,
  KEY_LCTRL          = 206,
  KEY_RALT           = 207,
  KEY_LALT           = 208,
  KEY_RMETA          = 209,
  KEY_LMETA          = 210,
  KEY_LSUPER         = 211,                /* Left "Windows" key */
  KEY_RSUPER         = 212,                /* Right "Windows" key */
  KEY_MODE           = 213,                /* "Alt Gr" key */
  KEY_COMPOSE        = 214,                /* Multi-key compose key */

  /* Miscellaneous function keys */

  KEY_HELP           = 215,
  KEY_PRINT          = 216,
  KEY_SYSREQ         = 217,
  KEY_BREAK          = 218,
  KEY_QUIT           = 250,

  /* Add any other keys here */

  KEY_LAST
} MDKKey;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int keypad_open(struct file *filep);
static int keypad_close(struct file *filep);
static ssize_t keypad_read(struct file *filep, FAR char *buf, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const unsigned char g_kdbus[128] =
{
  0,  KEY_QUIT, '1', '2', '3', '4', '5', '6', '7', '8',  /* 9 */
  '9', '0', '-', '=', '\b',                              /* Backspace */
  '\t',                                                  /* Tab */
  'q', 'w', 'e', 'r',                                    /* 19 */
  't', 'y', 'u', 'i', 'o', 'p', '[', ']', '\n',          /* Enter key */
  0,                                                     /* 29   - Control */
  'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ';',      /* 39 */
  '\'', '`',   0,                                        /* Left shift */
  '\\', 'z', 'x', 'c', 'v', 'b', 'n',                    /* 49 */
  'm', ',', '.', '/',   0,                               /* Right shift */
  '*',
  0,                                                     /* Alt */
  ' ',                                                   /* Space bar */
  0,                                                     /* Caps lock */
  KEY_F1,                                                /* 59 - F1 key ... > */
  KEY_F2, KEY_F3, KEY_F4, KEY_F5, KEY_F6, KEY_F7, KEY_F8, KEY_F9,
  KEY_F10,                                               /* < ... F10 */
  0,                                                     /* 69 - Num lock*/
  0,                                                     /* Scroll Lock */
  KEY_HOME,                                              /* Home key */
  KEY_UP,                                                /* Up Arrow */
  KEY_PAGEUP,                                            /* Page Up */
  '-',
  KEY_LEFT,                                              /* Left Arrow */
  0,
  KEY_RIGHT,                                             /* Right Arrow */
  '+',
  KEY_END,                                               /* 79 - End key*/
  KEY_DOWN,                                              /* Down Arrow */
  KEY_PAGEDOWN,                                          /* Page Down */
  KEY_INSERT,                                            /* Insert Key */
  KEY_DELETE,                                            /* Delete Key */
  0, 0, 0,
  0,                                                     /* F11 Key */
  0,                                                     /* F12 Key */
  0,                                                     /* All other keys are undefined */
};

static const struct file_operations g_keypadops =
{
  keypad_open,                  /* open */
  keypad_close,                 /* close */
  keypad_read,                  /* read */
  0,                            /* write */
  0,                            /* seek */
  0,                            /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int keypad_open(struct file *filep)
{
  return OK;
}

static int keypad_close(struct file *filep)
{
  return OK;
}

static ssize_t keypad_read(struct file *filep, FAR char *buf, size_t buflen)
{
  uint_fast8_t keycode = 0;
  uint_fast8_t scancode = 0;
  uint_fast8_t keyflags;

  if (buf == NULL || buflen < 1)
    {
      /* Well... nothing to do */

      return -EINVAL;
    }

  keyflags = inb(0x64);
  if (keyflags & 0x01)
    {
      scancode = inb(0x60);
      if ((scancode & 0x80) == 0)
        {
          keycode = g_kdbus[scancode];
          buf[0] = keycode;
          return 1;
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name:  qemu_keypad
 *
 * Description:
 *   Registers the QEMU keypad driver
 *
 ****************************************************************************/

void qemu_keypad(void)
{
  (void)register_driver("/dev/keypad", &g_keypadops, 0444, NULL);
}
