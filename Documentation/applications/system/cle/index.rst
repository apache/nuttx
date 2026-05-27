======================================
``cle`` EMACS-like Command Line Editor
======================================

Overview
========

The ``cle`` library provides an EMACS-like command line editor for NuttX
applications.  It is similar to GNU ``readline`` but much smaller, making it
suitable for resource-constrained embedded systems.  ``cle`` is not a
standalone command; it is a library that applications call to get interactive
line editing with history and key bindings.

The editor assumes a VT100-compatible terminal and a fixed-width character
set (such as Courier).  It does not support word-oriented operations or
keypad cursor control.

Configuration
=============

Enable the editor with ``CONFIG_SYSTEM_CLE``.  The following options are
available when ``CONFIG_SYSTEM_CLE`` is enabled:

``CONFIG_SYSTEM_COLOR_CLE``
  Add simple color highlighting to the prompt, command text, and output.
  The colors are hardcoded: yellow for the prompt, cyan for command text,
  and green for output.

``CONFIG_SYSTEM_CLE_CMD_HISTORY``
  Enable Unix-style command history using the up and down arrow keys.
  The history is stored in an in-memory circular buffer.

  In FLAT and PROTECTED builds the history is shared by all threads; in
  KERNEL builds each process has its own history buffer.

``CONFIG_SYSTEM_CLE_CMD_HISTORY_LINELEN``
  Maximum length of one command line in the history buffer.  Default: 64
  (when ``CONFIG_DEFAULT_SMALL`` is set) or 80.

``CONFIG_SYSTEM_CLE_CMD_HISTORY_LEN``
  Number of history records to keep.  Default: 4 (when
  ``CONFIG_DEFAULT_SMALL`` is set) or 16.

``CONFIG_SYSTEM_CLE_DEBUGLEVEL``
  Debug output level.  0 = off, 1 = errors on console, 2 = full debug
  information.  Debug output is sent via ``syslog()``.

Usage
=====

``cle`` is a library, not a standalone command.  Applications call one of
the following functions:

.. code-block:: c

   #include <system/cle.h>

   int cle_fd(FAR char *line, FAR const char *prompt, uint16_t linelen,
              int infd, int outfd);

   #ifdef CONFIG_FILE_STREAM
   int cle(FAR char *line, FAR const char *prompt, uint16_t linelen,
           FAR FILE *instream, FAR FILE *outstream);
   #endif

Parameters
----------

``line``
  Buffer to store the edited line.

``prompt``
  Prompt string displayed before the cursor.

``linelen``
  Maximum length of the line buffer.

``infd`` / ``instream``
  Input file descriptor (``cle_fd``) or FILE stream (``cle``).

``outfd`` / ``outstream``
  Output file descriptor (``cle_fd``) or FILE stream (``cle``).

Return Value
------------

Both functions return the number of characters in the edited line, or a
negative error code on failure.

Key Bindings
============

``cle`` uses EMACS-style control key bindings:

``Ctrl-A``
  Move cursor to the start of the current line.

``Ctrl-B``
  Move cursor left one character.

``Ctrl-D``
  Delete the character at the cursor position.

``Ctrl-E``
  Move cursor to the end of the current line.

``Ctrl-F``
  Move cursor right one character.

``Ctrl-H`` (Backspace)
  Delete the character to the left of the cursor.

``Ctrl-K``
  Delete from the cursor to the end of the line.

``Ctrl-L``
  Clear the screen and redraw the current line.

``Ctrl-N``
  Move down one line (when command history is enabled).

``Ctrl-P``
  Move up one line (when command history is enabled).

``Ctrl-U``
  Delete the entire line.

``\\`` (backslash)
  Quote the next character; the quoted character is inserted literally.

Examples
========

Use ``cle`` in an application to get interactive line editing:

.. code-block:: c

   #include <system/cle.h>
   #include <stdio.h>

   int main(void)
   {
     char line[128];
     int len;

     printf("Enter a command: ");
     len = cle(line, "> ", sizeof(line), stdin, stdout);
     if (len > 0)
       {
         printf("You typed: %s\n", line);
       }

     return 0;
   }

Notes
=====

- ``cle`` requires a VT100-compatible terminal.  It uses VT100 escape
  sequences for cursor movement, line editing, and screen clearing.

- The editor uses a fixed-width character set assumption.  Variable-width
  fonts may cause display issues.

- When ``CONFIG_SYSTEM_CLE_CMD_HISTORY`` is enabled, the history buffer
  is shared across all threads in FLAT and PROTECTED builds.  In KERNEL
  builds, each process has its own history buffer.

- Memory usage is approximately 1.5–2 KB.

- ``cle`` does not support word-oriented operations (move by word, delete
  word, etc.) or keypad cursor control.