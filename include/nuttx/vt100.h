/****************************************************************************
 * include/nuttx/vt100.h
 * VT100 Escape Sequences
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

#ifndef __INCLUDE_NUTTX_VT100_H
#define __INCLUDE_NUTTX_VT100_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/ascii.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VT100_SETNL          {ASCII_ESC, '[', '2', '0', 'h'}  /* Set new line mode */
#define VT100_SETAPPL        {ASCII_ESC, '[', '?', '1', 'h'}  /* Set cursor key to application */
#define VT100_SETCOL         {ASCII_ESC, '[', '?', '3', 'h'}  /* Set number of columns to 132 */
#define VT100_SETSMOOTH      {ASCII_ESC, '[', '?', '4', 'h'}  /* Set smooth scrolling */
#define VT100_SETREVSCRN     {ASCII_ESC, '[', '?', '5', 'h'}  /* Set reverse video on screen */
#define VT100_SETORGREL      {ASCII_ESC, '[', '?', '6', 'h'}  /* Set origin to relative */
#define VT100_SETWRAP        {ASCII_ESC, '[', '?', '7', 'h'}  /* Set auto-wrap mode */
#define VT100_SETREP         {ASCII_ESC, '[', '?', '8', 'h'}  /* Set auto-repeat mode */
#define VT100_SETINTER       {ASCII_ESC, '[', '?', '9', 'h'}  /* Set interlacing mode */

#define VT100_SETLF          {ASCII_ESC, '[', '2', '0', 'l'}  /* Set line feed mode */
#define VT100_SETCURSOR      {ASCII_ESC, '[', '?', '1', 'l'}  /* Set cursor key to cursor */
#define VT100_SETVT52        {ASCII_ESC, '[', '?', '2', 'l'}  /* Set VT52 (versus ANSI) */
#define VT100_RESETCOL       {ASCII_ESC, '[', '?', '3', 'l'}  /* Set number of columns to 80 */
#define VT100_SETJUMP        {ASCII_ESC, '[', '?', '4', 'l'}  /* Set jump scrolling */
#define VT100_SETNORMSCRN    {ASCII_ESC, '[', '?', '5', 'l'}  /* Set normal video on screen */
#define VT100_SETORGABS      {ASCII_ESC, '[', '?', '6', 'l'}  /* Set origin to absolute */
#define VT100_RESETWRAP      {ASCII_ESC, '[', '?', '7', 'l'}  /* Reset auto-wrap mode */
#define VT100_RESETREP       {ASCII_ESC, '[', '?', '8', 'l'}  /* Reset auto-repeat mode */
#define VT100_RESETINTER     {ASCII_ESC, '[', '?', '9', 'l'}  /* Reset interlacing mode */

#define VT100_ALTKEYPAD      {ASCII_ESC, '='}                 /* Set alternate keypad mode */
#define VT100_NUMKEYPAD      {ASCII_ESC, '>'}                 /* Set numeric keypad mode */

#define VT100_SETUKG0        {ASCII_ESC, '(', 'A'}            /* Set United Kingdom G0 character set */
#define VT100_SETUKG1        {ASCII_ESC, ')', 'A'}            /* Set United Kingdom G1 character set */
#define VT100_SETUSG0        {ASCII_ESC, '(', 'B'}            /* Set United States G0 character set */
#define VT100_SETUSG1        {ASCII_ESC, ')', 'B'}            /* Set United States G1 character set */
#define VT100_SETSPECG0      {ASCII_ESC, '(', '0'}            /* Set G0 special chars. & line set */
#define VT100_SETSPECG1      {ASCII_ESC, ')', '0'}            /* Set G1 special chars. & line set */
#define VT100_SETALTG0       {ASCII_ESC, '(', '1'}            /* Set G0 alternate character ROM */
#define VT100_SETALTG1       {ASCII_ESC, ')', '1'}            /* Set G1 alternate character ROM */
#define VT100_SETALTSPECG0   {ASCII_ESC, '(', '2'}            /* Set G0 alt char ROM and spec. graphics */
#define VT100_SETALTSPECG1   {ASCII_ESC, ')', '2'}            /* Set G1 alt char ROM and spec. graphics */

#define VT100_SETSS2         {ASCII_ESC, 'N'}                 /* Set single shift 2 */
#define VT100_SETSS3         {ASCII_ESC, 'O'}                 /* Set single shift 3 */

#define VT100_MODESOFF       {ASCII_ESC, '[', 'm'}            /* Turn off character attributes */
#define VT100_MODESOFF2      {ASCII_ESC, '[', '0', 'm'}       /* Turn off character attributes */
#define VT100_BOLD           {ASCII_ESC, '[', '1', 'm'}       /* Turn bold mode on */
#define VT100_LOWINT         {ASCII_ESC, '[', '2', 'm'}       /* Turn low intensity mode on */
#define VT100_FORE_BLACK     {ASCII_ESC, '[', '3', '0', 'm'}  /* Set foreground to color #0 - black */
#define VT100_FORE_RED       {ASCII_ESC, '[', '3', '1', 'm'}  /* Set foreground to color #1 - red */
#define VT100_FORE_GREEN     {ASCII_ESC, '[', '3', '2', 'm'}  /* Set foreground to color #2 - green */
#define VT100_FORE_YELLOW    {ASCII_ESC, '[', '3', '3', 'm'}  /* Set foreground to color #3 - yellow */
#define VT100_FORE_BLUE      {ASCII_ESC, '[', '3', '4', 'm'}  /* Set foreground to color #4 - blue */
#define VT100_FORE_MAGENTA   {ASCII_ESC, '[', '3', '5', 'm'}  /* Set foreground to color #5 - magenta */
#define VT100_FORE_CYAN      {ASCII_ESC, '[', '3', '6', 'm'}  /* Set foreground to color #6 - cyan */
#define VT100_FORE_WHITE     {ASCII_ESC, '[', '3', '7', 'm'}  /* Set foreground to color #7 - white */
#define VT100_FORE_DEFAULT   {ASCII_ESC, '[', '3', '9', 'm'}  /* Set foreground to color #9 - default */
#define VT100_UNDERLINE      {ASCII_ESC, '[', '4', 'm'}       /* Turn underline mode on */
#define VT100_BACK_BLACK     {ASCII_ESC, '[', '4', '0', 'm'}  /* Set background to color #0 - black */
#define VT100_BACK_RED       {ASCII_ESC, '[', '4', '1', 'm'}  /* Set background to color #1 - red */
#define VT100_BACK_GREEN     {ASCII_ESC, '[', '4', '2', 'm'}  /* Set background to color #2 - green */
#define VT100_BACK_YELLOW    {ASCII_ESC, '[', '4', '3', 'm'}  /* Set background to color #3 - yellow */
#define VT100_BACK_BLUE      {ASCII_ESC, '[', '4', '4', 'm'}  /* Set background to color #4 - blue */
#define VT100_BACK_MAGENTA   {ASCII_ESC, '[', '4', '5', 'm'}  /* Set background to color #5 - magenta */
#define VT100_BACK_CYAN      {ASCII_ESC, '[', '4', '6', 'm'}  /* Set background to color #6 - cyan */
#define VT100_BACK_WHITE     {ASCII_ESC, '[', '4', '7', 'm'}  /* Set background to color #7 - white */
#define VT100_BACK_DEFAULT   {ASCII_ESC, '[', '4', '9', 'm'}  /* Set background to color #9 - default */
#define VT100_BLINK          {ASCII_ESC, '[', '5', 'm'}       /* Turn blinking mode on */
#define VT100_REVERSE        {ASCII_ESC, '[', '7', 'm'}       /* Turn reverse video on */
#define VT100_INVISIBLE      {ASCII_ESC, '[', '8', 'm'}       /* Turn invisible text mode on */
#define VT100_BOLDOFF        {ASCII_ESC, '[', '2', '2', 'm'}  /* Turn bold off */
#define VT100_UNDERLINEOFF   {ASCII_ESC, '[', '2', '4', 'm'}  /* Turn underline off */
#define VT100_BLINKOFF       {ASCII_ESC, '[', '2', '5', 'm'}  /* Turn blink off */
#define VT100_REVERSEOFF     {ASCII_ESC, '[', '2', '7', 'm'}  /* Turn reverse video off */

#define VT100_SETWIN(t,b)    {ASCII_ESC, '[', (t), ';', (b), 'r'} /* Set top and bottom line#s of a window */

#define VT100_CURSOROFF      {ASCII_ESC, '[', '?', '2', '5', 'l'} /* Cursor OFF */
#define VT100_CURSORON       {ASCII_ESC, '[', '?', '2', '5', 'h'} /* Cursor ON */
#define VT100_CURSOROFF2     {ASCII_ESC, '[', '?', '5', '0', 'l'} /* Cursor OFF */
#define VT100_CURSORON2      {ASCII_ESC, '[', '?', '5', '0', 'h'} /* Cursor ON */

#define VT100_CURSORUP(n)    {ASCII_ESC, '[', (n), 'A'}       /* Move cursor up n lines */
#define VT100_CURSORDN(n)    {ASCII_ESC, '[', (n), 'B'}       /* Move cursor down n lines */
#define VT100_CURSORRT(n)    {ASCII_ESC, '[', (n), 'C'}       /* Move cursor right n lines */
#define VT100_CURSORLF(n)    {ASCII_ESC, '[', (n), 'D'}       /* Move cursor left n lines */
#define VT100_CURSORHOME     {ASCII_ESC, '[', 'H'}            /* Move cursor to upper left corner */
#define VT100_CURSORHOME_    {ASCII_ESC, '[', ';', 'H'}       /* Move cursor to upper left corner */

#define VT100_CURSORPOS(v,h) {ASCII_ESC, '[', (v), ';', (h), 'H'} /* Move cursor to screen location v,h */

#define VT100_HVHOME         {ASCII_ESC, '[', 'f'}            /* Move cursor to upper left corner */
#define VT100_HVHOME_        {ASCII_ESC, '[', ';', 'f'}       /* Move cursor to upper left corner */

#define VT100_HVPOS(v,h)     {ASCII_ESC, '[', (v), ';', (h), 'f'} /* Move cursor to screen location v,h */

#define VT100_INDEX          {ASCII_ESC, 'D'}                 /* Move/scroll window up one line */
#define VT100_REVINDEX       {ASCII_ESC, 'M'}                 /* Move/scroll window down one line */
#define VT100_NEXTLINE       {ASCII_ESC, 'E'}                 /* Move to next line */
#define VT100_SAVECURSOR     {ASCII_ESC, '7'}                 /* Save cursor position and attributes */
#define VT100_RESTORECURSOR  {ASCII_ESC, '8'}                 /* Restore cursor position and attribute */

#define VT100_TABSET         {ASCII_ESC, 'H'}                 /* Set a tab at the current column */
#define VT100_TABCLR         {ASCII_ESC, '[', 'g'}            /* Clear a tab at the current column */
#define VT100_TABCLR_        {ASCII_ESC, '[', '0', 'g'}       /* Clear a tab at the current column */
#define VT100_TABCLRALL      {ASCII_ESC, '[', '3', 'g'}       /* Clear all tabs */

#define VT100_DHTOP          {ASCII_ESC, '#', '3'}            /* Double-height letters, top half */
#define VT100_DHBOT          {ASCII_ESC, '#', '4'}            /* Double-height letters, bottom hal */
#define VT100_SWSH           {ASCII_ESC, '#', '5'}            /* Single width, single height letters */
#define VT100_DWSH           {ASCII_ESC, '#', '6'}            /* Double width, single height letters */

#define VT100_CLEAREOL       {ASCII_ESC, '[', 'K'}            /* Clear line from cursor right */
#define VT100_CLEAREOL_      {ASCII_ESC, '[', '0', 'K'}       /* Clear line from cursor right */
#define VT100_CLEARBOL       {ASCII_ESC, '[', '1', 'K'}       /* Clear line from cursor left */
#define VT100_CLEARLINE      {ASCII_ESC, '[', '2', 'K'}       /* Clear entire line */

#define VT100_CLEAREOS       {ASCII_ESC, '[', 'J'}            /* Clear screen from cursor down */
#define VT100_CLEAREOS_      {ASCII_ESC, '[', '0', 'J'}       /* Clear screen from cursor down */
#define VT100_CLEARBOS       {ASCII_ESC, '[', '1', 'J'}       /* Clear screen from cursor up */
#define VT100_CLEARSCREEN    {ASCII_ESC, '[', '2', 'J'}       /* Clear entire screen */

#define VT100_DEVSTAT        {ASCII_ESC, '[', '5', 'n'}       /* Device status report */
#define VT100_TERMOK         {ASCII_ESC, '[', '0', 'n'}       /* Response: terminal is OK */
#define VT100_TERMNOK        {ASCII_ESC, '[', '3', 'n'}       /* Response: terminal is not OK */

#define VT100_GETCURSOR      {ASCII_ESC, '[', '6', 'n'}       /* Get cursor position */

#define VT100_CURSORPOSAT    {ASCII_ESC, '[', (v), ';', (h), 'R'}  /* Response: cursor is at v,h */

#define VT100_IDENT          {ASCII_ESC, '[', 'c'}            /* Identify what terminal type */
#define VT100_IDENT_         {ASCII_ESC, '[', '0', 'c'}       /* Identify what terminal type */

#define VT100_GETTYPE        {ASCII_ESC, '[', '?', '1', ';', (n), '0', 'c'} /* Response: terminal type code n */

#define VT100_RESET RIS      {ASCII_ESC, 'c'}                 /*  Reset terminal to initial state */

#define VT100_ALIGN          {ASCII_ESC, '#', '8'}            /* Screen alignment display */

#define VT100_TESTPU         {ASCII_ESC, '[', '2', ';', '1', 'y'} /* Confidence power up test */
#define VT100_TESTLB         {ASCII_ESC, '[', '2', ';', '2', 'y'} /* Confidence loopback test */
#define VT100_TESTPUREP      {ASCII_ESC, '[', '2', ';', '9', 'y'} /* Repeat power up test */

#define VT100_TESTLBREP      {ASCII_ESC, '[', '2', ';', '1', '0', 'y'} /* Repeat loopback test */

#define VT100_LEDSOFF        {ASCII_ESC, '[', '0', 'q'}       /* Turn off all four leds */
#define VT100_LED1           {ASCII_ESC, '[', '1', 'q'}       /* Turn on LED #1 */
#define VT100_LED2           {ASCII_ESC, '[', '2', 'q'}       /* Turn on LED #2 */
#define VT100_LED3           {ASCII_ESC, '[', '3', 'q'}       /* Turn on LED #3 */
#define VT100_LED4           {ASCII_ESC, '[', '4', 'q'}       /* Turn on LED #4 */

/* All codes below are for use in VT52 compatibility mode. */

#define VT52_SETANSI         {ASCII_ESC, '<'}                 /* Enter/exit ANSI mode */

#define VT52_ALTKEYPAD       {ASCII_ESC, '='}                 /* Enter alternate keypad mode */
#define VT52_NUMKEYPAD       {ASCII_ESC, '>'}                 /* Exit alternate keypad mode */

#define VT52_SETGR           {ASCII_ESC, 'F'}                 /* Use special graphics character set */
#define VT52_RESETGR         {ASCII_ESC, 'G'}                 /* Use normal US/UK character set */

#define VT52_CURSORUP        {ASCII_ESC, 'A'}                 /* Move cursor up one line */
#define VT52_CURSORDN        {ASCII_ESC, 'B'}                 /* Move cursor down one line */
#define VT52_CURSORRT        {ASCII_ESC, 'C'}                 /* Move cursor right one char */
#define VT52_CURSORLF        {ASCII_ESC, 'D'}                 /* Move cursor left one char */
#define VT52_CURSORHOME      {ASCII_ESC, 'H'}                 /* Move cursor to upper left corner */
#define VT52_CURSORPOS(v,h)  {ASCII_ESC, (v), (h)}            /* Move cursor to v,h location */
#define VT52_REVINDEX        {ASCII_ESC, 'I'}                 /* Generate a reverse line-feed */

#define VT52_CLEAREOL        {ASCII_ESC, 'K'}                 /* Erase to end of current line */
#define VT52_CLEAREOS        {ASCII_ESC, 'J'}                 /* Erase to end of screen */

#define VT52_IDENT           {ASCII_ESC, 'Z'}                 /* Identify what the terminal is */
#define VT52_IDENTRESP       {ASCII_ESC, '/', 'Z'}            /* Correct response to ident */

/* Format strings for VT100 sequences that require numeric arguments */

#define VT100_BLACK          0                                /* Color #0 - black */
#define VT100_RED            1                                /* Color #1 - red */
#define VT100_GREEN          2                                /* Color #2 - green */
#define VT100_YELLOW         3                                /* Color #3 - yellow */
#define VT100_BLUE           4                                /* Color #4 - blue */
#define VT100_MAGENTA        5                                /* Color #5 - magenta */
#define VT100_CYAN           6                                /* Color #6 - cyan */
#define VT100_WHITE          7                                /* Color #7 - white */
#define VT100_DEFAULT        9                                /* Color #9 - default */
#define VT100_FMT_FORE_COLOR "\033[3%dm"                      /* Set foreground to color #n, n=0-8,9*/
#define VT100_FMT_BACK_COLOR "\033[4%dm"                      /* Color #n, n=0-8,9*/

#define VT100_FMT_SETWIN     "\033[%d;%dr"                    /* Set top and bottom line#s of a window */
#define VT100_FMT_CURSORUP   "\033[%dA"                       /* Move cursor up n lines */
#define VT100_FMT_CURSORDN   "\033[%dB"                       /* Move cursor down n lines */
#define VT100_FMT_CURSORRT   "\033[%dC"                       /* Move cursor right n lines */
#define VT100_FMT_CURSORLF   "\033[%dD"                       /* Move cursor left n lines */
#define VT100_FMT_CURSORPOS  "\033[%d;%dH"                    /* Move cursor to screen location v,h */
#define VT100_FMT_HVPOS      "\033[%d;%df"                    /* Move cursor to screen location v,h */
#define VT52_FMT_CURSORPOS   "\033%d%d"                       /* Move cursor to v,h location */

/* VT100 Special Key Codes
 *
 * These are sent from the terminal back to the computer when the particular
 * key is pressed.  Note that the numeric keypad keys send different codes
 * in numeric mode than in alternate mode.
 */

/* Function Keys */

#define VT100_PF1            {ASCII_ESC, 'O', 'P'}
#define VT100_PF2            {ASCII_ESC, 'O', 'Q'}
#define VT100_PF3            {ASCII_ESC, 'O', 'R'}
#define VT100_PF4            {ASCII_ESC, 'O', 'S'}

/* Arrow keys */

#define VT100_UP_RESET       {ASCII_ESC, 'A'}
#define VT100_UP_SET         {ASCII_ESC, 'O', 'A'}
#define VT100_DOWN_RESET     {ASCII_ESC, 'B'}
#define VT100_DOWN_SET       {ASCII_ESC, 'O', 'B'}
#define VT100_RIGHT_RESET    {ASCII_ESC, 'C'}
#define VT100_RIGHT_SET      {ASCII_ESC, 'O', 'C'}
#define VT100_LEFT_RESET     {ASCII_ESC, 'D'}
#define VT100_LEFT_SET       {ASCII_ESC, 'O', 'D'}

/* Numeric Keypad Keys */

#define VT100_NUMERIC_0      {'0'}
#define VT100_ALT_0          {ASCII_ESC, 'O', 'p'}
#define VT100_NUMERIC_1      {'1'}
#define VT100_ALT_1          {ASCII_ESC, 'O', 'q'}
#define VT100_NUMERIC_2      {'2'}
#define VT100_ALT_2          {ASCII_ESC, 'O', 'r'}
#define VT100_NUMERIC_3      {'3'}
#define VT100_ALT_3          {ASCII_ESC, 'O', 's'}
#define VT100_NUMERIC_4      {'4'}
#define VT100_ALT_4          {ASCII_ESC, 'O', 't'}
#define VT100_NUMERIC_5      {'5'}
#define VT100_ALT_5          {ASCII_ESC, 'O', 'u'}
#define VT100_NUMERIC_6      {'6'}
#define VT100_ALT_6          {ASCII_ESC, 'O', 'v'}
#define VT100_NUMERIC_7      {'7'}
#define VT100_ALT_7          {ASCII_ESC, 'O', 'w'}
#define VT100_NUMERIC_8      {'8'}
#define VT100_ALT_8          {ASCII_ESC, 'O', 'x'}
#define VT100_NUMERIC_9      {'9',
#define VT100_ALT_9          {ASCII_ESC, 'O', 'y'}
#define VT100_NUMERIC_MINUS  {'-'}
#define VT100_ALT_MINUS      {ASCII_ESC, 'O', 'm'}
#define VT100_NUMERIC_COMMA  {','}
#define VT100_ALT_COMMA      {ASCII_ESC, 'O', 'l'}
#define VT100_NUMERIC_PERIOD {'.'}
#define VT100_ALT_PERIOD     {ASCII_ESC, 'O', 'n'}
#define VT100_NUMERIC_ENTER  {ASCII_CR}
#define VT100_ALT_ENTER      {ASCII_ESC, 'O', 'M'}

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_NUTTX_VT100_H */
