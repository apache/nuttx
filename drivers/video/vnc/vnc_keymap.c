/****************************************************************************
 * drivers/video/vnc/vnc_keymap.c
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

#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/ascii.h>
#include <nuttx/video/vnc.h>
#include <nuttx/input/x11_keysym.h>
#include <nuttx/input/kbd_codec.h>

#include "vnc_server.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FIRST_PRTCHAR   ASCII_SPACE
#define LAST_PRTCHAR    ASCII_TILDE
#define NPRTCHARS       (ASCII_TILDE + ASCII_SPACE - 1)

#define ISPRINTABLE(c)  ((c) >= FIRST_PRTCHAR && (c) <= LAST_PRTCHAR)
#define ISLOWERCASE(c)  ((c) >= ASCII_a && (c) <= ASCII_z)
#define ISUPPERCASE(c)  ((c) >= ASCII_A && (c) <= ASCII_Z)
#define ISALPHABETIC(c) (ISLOWERCASE(c) || ISUPPERCASE(c))

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
  {'\xb4',           XK_dead_acute},
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
  {ASCII_CR,         XK_KP_Enter},
#ifdef CONFIG_VNCSERVER_KBDENCODE
  {ASCII_DEL,      XK_KP_Delete},
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
 * Name: vnc_kbd_encode
 *
 * Description:
 *   Encode one escape sequence command into the proivded buffer.
 *
 * Input Parameters:
 *   buffer     - The location to write the sequence
 *   keycode    - The command to be added to the output stream.
 *   terminator - Escape sequence terminating character.
 *
 * Returned Value:
 *   Number of bytes written
 *
 ****************************************************************************/

#ifdef CONFIG_VNCSERVER_KBDENCODE
static inline int vnc_kbd_encode(FAR uint8_t *buffer, uint8_t keycode,
                                 uint8_t terminator)
{
  *buffer++ = ASCII_ESC;
  *buffer++ = ASCII_LBRACKET;
  *buffer++ = keycode;
  *buffer   = terminator;
  return 4;
}
#endif

/****************************************************************************
 * Name: vnc_kbd_press
 *
 * Description:
 *   Indicates a normal key press event.  Put one byte of normal keyboard
 *   data into the user provided buffer.
 *
 * Input Parameters:
 *   buffer - The location to write the sequence
 *   ch     - The character to be added to the output stream.
 *
 * Returned Value:
 *   Number of bytes written
 *
 ****************************************************************************/

#ifdef CONFIG_VNCSERVER_KBDENCODE
static inline void vnc_kbd_press(FAR uint8_t *buffer, uint8_t ch)
{
  *buffer = ch;
  return 1;
}
#endif

/****************************************************************************
 * Name: vnc_kbd_release
 *
 * Description:
 *   Encode the release of a normal key.
 *
 * Input Parameters:
 *   buffer  - The location to write the sequence
 *   ch - The character associated with the key that was releared.
 *
 * Returned Value:
 *   Number of bytes written
 *
 ****************************************************************************/

#ifdef CONFIG_VNCSERVER_KBDENCODE
static inline void vnc_kbd_release(FAR uint8_t *buffer, uint8_t ch)
{
  return vnc_kbd_encode(buffer, ch, ('a' + KBD_RELEASE));
}
#endif

/****************************************************************************
 * Name: vnc_kbd_specpress
 *
 * Description:
 *   Denotes a special key press event.  Put one special keyboard command
 *   into the user provided buffer.
 *
 * Input Parameters:
 *   buffer  - The location to write the sequence
 *   keycode - The command to be added to the output stream.
 *
 * Returned Value:
 *   Number of bytes written
 *
 ****************************************************************************/

#ifdef CONFIG_VNCSERVER_KBDENCODE
static inline void vnc_kbd_specpress(FAR uint8_t *buffer, uint8_t keycode)
{
  return vnc_kbd_encode(buffer, keycode, stream, ('a' + KBD_SPECPRESS));
}
#endif

/****************************************************************************
 * Name: vnc_kbd_specrel
 *
 * Description:
 *   Denotes a special key release event.  Put one special keyboard
 *   command into the user provided buffer.
 *
 * Input Parameters:
 *   buffer  - The location to write the sequence
 *   keycode - The command to be added to the output stream.
 *
 * Returned Value:
 *   Number of bytes written
 *
 ****************************************************************************/

#ifdef CONFIG_VNCSERVER_KBDENCODE
static inline void vnc_kbd_specrel(FAR uint8_t *buffer, uint8_t keycode)
{
  return vnc_kbd_encode(buffer, keycode, stream, ('a' + KBD_SPECREL));
}
#endif

/****************************************************************************
 * Name: vnc_kbd_lookup
 *
 * Description:
 *   Attempt to map the X11 keycode by searching in a lookup table.
 *
 ****************************************************************************/

static int vnc_kbd_lookup(FAR const struct vnc_keymap_s *table,
                          unsigned int nelem, uint16_t keysym)
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

static int vnc_kbd_ascii(uint16_t keysym)
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

void vnc_key_map(FAR struct vnc_session_s *session, uint16_t keysym,
                 bool keydown)
{
#ifdef CONFIG_VNCSERVER_KBDENCODE
  uint8_t buffer[4]
  int nch;
#else
  uint8_t buffer;
#endif
  int16_t keych;

  /* Check for modifier keys */

  keych = vnc_kbd_lookup(g_modifiers, G_MODIFIERS_NELEM, keysym);
  if (keych >= 0)
    {
      g_modstate[keych] = keydown;
      return;
    }

#ifndef CONFIG_VNCSERVER_KBDENCODE
  /* If we are not encoding key presses, then we have to ignore key release
   * events.
   */

  if (!keydown)
    {
      return;
    }
#endif

  /* If no external keyboard input handler has been provided,
   * then we have to drop the keyboard input.
   */

  if (session->kbdout == NULL)
    {
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

      else if (ISPRINTABLE(keych))
        {
          /* If Shift Lock is selected, then the case of all printable
           * characters should be reversed (unless the Shift key is also
           * pressed)
           */

          if (g_modstate[MOD_SHIFTLOCK])
            {
              if (g_modstate[MOD_SHIFT])
                {
                  /* Swap case */

                  keych = g_caseswap[keych];
                }
            }

          /* If Caps Lock is selected, then the case of alphabetic
           * characters should be reversed (unless the Shift key is also
           * pressed)
           */

          else if (g_modstate[MOD_CAPSLOCK] && ISALPHABETIC(keych))
            {
              if (g_modstate[MOD_SHIFT])
                {
                  /* Swap case */

                  keych = g_caseswap[keych];
                }
            }

          /* If (1) only the Shift Key is pressed or (2) the Shift key is
           * pressed with Caps Lock, but the character is not alphabetic,
           * then the case of all printable characters should be reversed.
           */

          else if (g_modstate[MOD_SHIFT])
            {
              keych = g_caseswap[keych];
            }
        }

#ifdef CONFIG_VNCSERVER_KBDENCODE
      /* Encode the normal character */

      if (keydown)
        {
          nch = vnc_kbd_press(buffer, keych);
        }
      else
        {
          nch = vnc_kbd_release(buffer, keych);
        }

      /* Inject the normal character sequence into NX */

      session->kbdout(session->arg, nch, buffer);
#else
      /* Inject the single key press into NX */

      buffer = (uint8_t)keych;
      session->kbdout(session->arg, 1, &buffer);
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
          /* Encode the speical character */

          if (keydown)
            {
              nch = vnc_kbd_specpress(buffer, keych);
            }
          else
            {
              nch = vnc_kbd_specrel(buffer, keych);
            }

          /* Inject the special character sequence into NX */

          session->kbdout(session->arg, nch, buffer);
        }
    }
#endif
}
