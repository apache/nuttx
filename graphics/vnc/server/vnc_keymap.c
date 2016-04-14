/****************************************************************************
 * graphics/vnc/vnc_keymap.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <errno.h>

#include <nuttx/ascii.h>

#define XK_MISCELLANY 1 /* Select X11 character set */
#define XK_LATIN1     1
#define XK_XKB_KEYS   1

#include <nuttx/input/x11_keysymdef.h>
#include <nuttx/input/kbd_codec.h>

#include "vnc_server.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FIRST_PRTCHAR ASCII_SPACE
#define LAST_PRTCHAR  ASCII_TILDE
#define NPRTCHARS     (ASCII_TILDE + ASCII_SPACE - 1)

/****************************************************************************
 * Private types
 ****************************************************************************/

enum vnc_modifier_e
{
  MOD_SHIFT = 0,   /* Left or right shift key */
  MOD_CONTROL,     /* Left or right control key */
  MOD_ALT,         /* Alt key */
  MOD_CAPSLOCK,    /* Caps lock */
  MOD_SHIFTLOCK,   /* Shift lock */
#ifdef CONFIG_VNCSERVER_KBDENCODE
  MOD_SCROLLLOCK,  /* Scroll lock */
  MOD_NUMLOCK,     /* Num lock */
#endif
  NMODIFIERS
};

struct vnc_keymap_s
{
  uint16_t nxcode;
  uint16_t x11code;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Special key modifiers */

static const struct vnc_keymap_s g_modifiers[] =
{
  {MOD_SHIFT,      XK_Shift_L},
  {MOD_SHIFT,      XK_Shift_R},
  {MOD_CONTROL,    XK_Control_L},
  {MOD_CONTROL,    XK_Control_R},
  {MOD_ALT,        XK_Alt_L},
  {MOD_ALT,        XK_Alt_R},
  {MOD_CAPSLOCK,   XK_Caps_Lock},
  {MOD_SHIFTLOCK,  XK_Shift_Lock},
#ifdef CONFIG_VNCSERVER_KBDENCODE
  {MOD_SCROLLLOCK, XK_Scroll_Lock},
  {MOD_NUMLOCK,    XK_Num_Lock},
#endif
};

#define G_MODIFIERS_NELEM (sizeof(g_modifiers) / sizeof(struct vnc_keymap_s))

/* Map special mappings for X11 codes to ASCII characters */

static const struct vnc_keymap_s g_asciimap[] =
{
  /* Control characters */

#ifdef CONFIG_VNCSERVER_KBDENCODE
  {ASCII_BS,         XK_BackSpace},
#endif
  {ASCII_TAB,        XK_Tab},
  {ASCII_LF,         XK_Linefeed},
  {ASCII_CR,         XK_Return},
  {ASCII_ESC,        XK_Escape},
#ifdef CONFIG_VNCSERVER_KBDENCODE
  {ASCII_DEL,        XK_Delete},
#endif

  /* Alternative encodings */

  {'`',              XK_dead_grave},
  {'´',              XK_dead_acute},
  {ASCII_TILDE,      XK_dead_tilde},
  {ASCII_CARET,      XK_dead_circumflex},

  /* Keypad aliases */

  {ASCII_0,          XK_KP_0},
  {ASCII_1,          XK_KP_1},
  {ASCII_2,          XK_KP_2},
  {ASCII_3,          XK_KP_3},
  {ASCII_4,          XK_KP_4},
  {ASCII_5,          XK_KP_5},
  {ASCII_6,          XK_KP_6},
  {ASCII_7,          XK_KP_7},
  {ASCII_8,          XK_KP_8},
  {ASCII_9,          XK_KP_9},

  {ASCII_ASTERISK,   XK_KP_Multiply},
  {ASCII_PLUS,       XK_KP_Add},
  {ASCII_COMMA,      XK_KP_Separator},
  {ASCII_HYPHEN,     XK_KP_Subtract},
  {ASCII_PERIOD,     XK_KP_Decimal},
  {ASCII_DIVIDE,     XK_KP_Divide},

  {ASCII_SPACE,      XK_KP_Space},
  {ASCII_TAB,        XK_KP_Tab},
  {ASCII_CR,         XK_KP_Enter}
#ifdef CONFIG_VNCSERVER_KBDENCODE
  , {ASCII_DEL,      XK_KP_Delete}
#endif
};

#define G_ASCIIMAP_NELEM (sizeof(g_asciimap) / sizeof(struct vnc_keymap_s))

#ifdef CONFIG_VNCSERVER_KBDENCODE
static const struct vnc_keymap_s g_cursor[] =
{
  {KEYCODE_BACKDEL,  XK_BackSpace},
  {KEYCODE_FWDDEL,   XK_Delete},
  {KEYCODE_FWDDEL,   XK_KP_Delete},
  {KEYCODE_HOME,     XK_Home},
  {KEYCODE_HOME,     XK_KP_Home},
  {KEYCODE_END,      XK_End},
  {KEYCODE_END,      XK_KP_End},
  {KEYCODE_LEFT,     XK_Left},
  {KEYCODE_LEFT,     XK_KP_Left},
  {KEYCODE_RIGHT,    XK_Right},
  {KEYCODE_RIGHT,    XK_KP_Right},
  {KEYCODE_UP,       XK_Up},
  {KEYCODE_UP,       XK_KP_Up},
  {KEYCODE_DOWN,     XK_Down},
  {KEYCODE_DOWN,     XK_KP_Down},
  {KEYCODE_PAGEUP,   XK_Page_Up},
  {KEYCODE_PAGEUP,   XK_KP_Prior},
  {KEYCODE_PAGEUP,   XK_KP_Page_Up},
  {KEYCODE_PAGEDOWN, XK_Page_Down},
  {KEYCODE_PAGEDOWN, XK_KP_Next},
  {KEYCODE_PAGEDOWN, XK_KP_Page_Down},
  {KEYCODE_INSERT,   XK_Insert},
  {KEYCODE_INSERT,   XK_KP_Insert},

  {KEYCODE_SELECT,   XK_Select},
  {KEYCODE_EXECUTE,  XK_Execute},
  {KEYCODE_HELP,     XK_Help},
  {KEYCODE_MENU,     XK_Alt_L},
  {KEYCODE_MENU,     XK_Alt_R},
  {KEYCODE_PAUSE,    XK_Pause},
  {KEYCODE_PRTSCRN,  XK_Print},
  {KEYCODE_CLEAR,    XK_Clear},
  {MOD_SCROLLLOCK,   XK_Scroll_Lock},
  {MOD_NUMLOCK,      XK_Num_Lock},

  {KEYCODE_F1,       XK_KP_F1},
  {KEYCODE_F1,       XK_F1},
  {KEYCODE_F2,       XK_KP_F2},
  {KEYCODE_F2,       XK_F2},
  {KEYCODE_F3,       XK_KP_F3},
  {KEYCODE_F3,       XK_F3},
  {KEYCODE_F4,       XK_KP_F4},
  {KEYCODE_F4,       XK_F4},
  {KEYCODE_F5,       XK_F5},
  {KEYCODE_F6,       XK_F6},
  {KEYCODE_F7,       XK_F7},
  {KEYCODE_F8,       XK_F8},
  {KEYCODE_F9,       XK_F9},
  {KEYCODE_F10,      XK_F10},
  {KEYCODE_F11,      XK_F11},
  {KEYCODE_F12,      XK_F12},
  {KEYCODE_F13,      XK_F13},
  {KEYCODE_F14,      XK_F14},
  {KEYCODE_F15,      XK_F15},
  {KEYCODE_F16,      XK_F16},
  {KEYCODE_F17,      XK_F17},
  {KEYCODE_F18,      XK_F18},
  {KEYCODE_F19,      XK_F19},
  {KEYCODE_F20,      XK_F20},
  {KEYCODE_F21,      XK_F21},
  {KEYCODE_F22,      XK_F22},
  {KEYCODE_F23,      XK_F23},
  {KEYCODE_F24,      XK_F24},
};
#endif

/* Changes the case of a character.  Based on US keyboard layout */

static const uint8_t g_caseswap[NPRTCHARS] =
{
  ASCII_SPACE,     ASCII_1,          ASCII_RSQUOTE,    ASCII_3,          /*   ! " # */
  ASCII_4,         ASCII_5,          ASCII_7,          ASCII_QUOTE,      /* $ % & ' */
  ASCII_9,         ASCII_0,          ASCII_8,          ASCII_EQUAL,      /* ( ) * + */
  ASCII_LT,        ASCII_UNDERSCORE, ASCII_GT,         ASCII_QUESTION,   /* , - . / */
  ASCII_RPAREN,    ASCII_EXCLAM,     ASCII_AT,         ASCII_NUMBER,     /* 0 1 2 3 */
  ASCII_DOLLAR,    ASCII_PERCENT,    ASCII_CIRCUMFLEX, ASCII_AMPERSAND,  /* 4 5 6 7 */
  ASCII_ASTERISK,  ASCII_LPAREN,     ASCII_SEMICOLON,  ASCII_COLON,      /* 8 9 : ; */
  ASCII_COMMA,     ASCII_PLUS,       ASCII_PERIOD,     ASCII_SLASH,      /* < = > ? */
  ASCII_2,         ASCII_a,          ASCII_b,          ASCII_c,          /* @ A B C */
  ASCII_d,         ASCII_e,          ASCII_f,          ASCII_g,          /* D E F G */
  ASCII_h,         ASCII_i,          ASCII_j,          ASCII_k,          /* H I J K */
  ASCII_l,         ASCII_m,          ASCII_n,          ASCII_o,          /* L M N O */
  ASCII_p,         ASCII_q,          ASCII_r,          ASCII_s,          /* P Q R S */
  ASCII_t,         ASCII_u,          ASCII_v,          ASCII_v,          /* T U V W */
  ASCII_x,         ASCII_y,          ASCII_z,          ASCII_LBRACE,     /* X Y Z [ */
  ASCII_VERTBAR,   ASCII_RBRACE,     ASCII_6,          ASCII_HYPHEN,     /* \ ] ^ _ */
  ASCII_TILDE,     ASCII_A,          ASCII_B,          ASCII_C,          /* ' a b c */
  ASCII_D,         ASCII_E,          ASCII_F,          ASCII_G,          /* c e f g */
  ASCII_H,         ASCII_I,          ASCII_J,          ASCII_K,          /* h i j k */
  ASCII_L,         ASCII_M,          ASCII_N,          ASCII_O,          /* l m n o */
  ASCII_P,         ASCII_Q,          ASCII_R,          ASCII_S,          /* p q r s */
  ASCII_T,         ASCII_U,          ASCII_V,          ASCII_W,          /* t u v w */
  ASCII_X,         ASCII_Y,          ASCII_Z,          ASCII_LBRACKET,   /* x y z { */
  ASCII_BACKSLASH, ASCII_RBRACKET,   ASCII_RSQUOTE,                      /* | } ~   */
};

/* State of each modifier */

static bool g_modstate[NMODIFIERS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_kbd_lookup
 *
 * Description:
 *   Attempt to map the X11 keycode by searching in a lookup table.
 *
 ****************************************************************************/

static int vnc_kbd_lookup(FAR const struct vnc_keymap_s *table,
                          unsigned int nelem, uint32_t keysym)
{
  int i;

  /* First just try to map the virtual keycode using our lookup-table */

  for (i = 0; i < nelem; i++)
    {
      if (table[i].x11code == keysym)
        {
          /* We have a match */

          return (int)table[i].nxcode;
        }
    }

  /* No match */

  return -EINVAL;
}

/****************************************************************************
 * Name: vnc_kbd_ascii
 *
 * Description:
 *   Attempt to map the X11 keycode into the corresponding ASCII code.
 *
 ****************************************************************************/

static int vnc_kbd_ascii(uint32_t keysym)
{
  /* ISO/IEC 8859-1 Latin1 matches C ASCII in this range: */

#ifdef CONFIG_VNCSERVER_KBDENCODE
  if (keysym >= ASCII_SPACE && keysym < ASCII_DEL)
#else
  if (keysym >= ASCII_SPACE && keysym <= ASCII_DEL)
#endif
    {
      return (int)keysym;
    }

  /* Perform a lookup to handler some special cases */

  return vnc_kbd_lookup(g_asciimap, G_ASCIIMAP_NELEM, keysym);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vnc_key_map
 *
 * Description:
 *   Map the receive X11 keysym into something understood by NuttX and route
 *   that through NX to the appropriate window.
 *
 * Input Parameters:
 *   session - An instance of the session structure allocated by
 *     vnc_create_session().
 *   keysym  - The X11 keysym value (see include/nuttx/inputx11_keysymdef)
 *   keydown - True: Key pressed; False: Key released
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void vnc_key_map(FAR struct vnc_session_s *session, uint32_t keysym,
                 bool keydown)
{
  int16_t keych;
  uint8_t ch;
  int ret;

  /* Check for modifier keys */

  keych = vnc_kbd_lookup(g_modifiers, G_MODIFIERS_NELEM, keysym);
  if (keych >= 0)
    {
      g_modstate[keych] = keydown;
      return;
    }

  /* Try to convert the keycode to an ASCII value */

  keych = vnc_kbd_ascii((char)(keysym & 255));
  if (keych >= 0)
    {
      /* It is a simple ASCII-mappable LATIN1 character.  Now we need
       * to apply any modifiers.
       */

      if (g_modstate[MOD_CONTROL])
        {
          /* Make into a control character */

          keych &= 0x1f;
        }

      /* Other modifiers apply only to printable characters */

      else if (keych >= FIRST_PRTCHAR && keych <= LAST_PRTCHAR)
        {
          /* If Shift Lock is selected, then the case of all characters
           * should be reversed (unless the Shift key is also pressed)
           */

          if (g_modstate[MOD_SHIFTLOCK])
            {
              if (g_modstate[MOD_SHIFT])
                {
                  /* Swap case */

                  keych = g_caseswap[keych];
                }
            }

          /* If Caps Lock is selected, then the case of printable
           * characters should be reversed (unless the Shift key is also
           * pressed)
           */

          else if (g_modstate[MOD_CAPSLOCK])
            {
              if (g_modstate[MOD_SHIFT])
                {
                  /* Swap case */

                  keych = g_caseswap[keych];
                }
            }

          /* If only the Shift Key is pressed, then the case of all
           * characters should be reversed.
           */

          else if (g_modstate[MOD_SHIFT])
            {
              keych = g_caseswap[keych];
            }
        }

#ifdef CONFIG_VNCSERVER_KBDENCODE
      /* Encode the normal character */
#warning Missing logic

      /* Inject the normal character sequence into NX */
#warning Missing logic
#else
      /* Inject the key press into NX */

      ch = (uint8_t)keych;
      ret = nx_kbdin(session->handle, 1, &ch);
      if (ret < 0)
        {
          gdbg("ERROR: nx_kbdin() failed: %d\n", ret)
        }
#endif
    }

  /* Not mappable to an ASCII LATIN1 character */

#ifdef CONFIG_VNCSERVER_KBDENCODE
  else
    {
      /* Lookup cursor movement/screen control keysyms */

      keych = vnc_kbd_lookup(g_modifiers, G_MODIFIERS_NELEM, keysym);
      if (keych >= 0)
        {
          /* Inject the special character sequence into NX */
#warning Missing logic
        }
    }
#endif
}
