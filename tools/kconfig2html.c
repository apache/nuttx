/****************************************************************************
 * tools/kconfig2html.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#define _GNU_SOURCE 1
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <ctype.h>
#include <libgen.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef USE_JQUERY

#define LINE_SIZE        1024
#define SCRATCH_SIZE     2048
#define MAX_DEPENDENCIES 128
#define MAX_LEVELS       100
#define MAX_SELECT       64
#define MAX_DEFAULTS     196
#define TAB_SIZE         4
#define VAR_SIZE         80
#define HTML_VAR_SIZE    (2*VAR_SIZE + 64)

#define BODYFILE_NAME    ".k2h-body.dat"
#define APNDXFILE_NAME   ".k2h-apndx.dat"

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum token_type_e
{
  TOKEN_NONE = 0,
  TOKEN_NOTRESERVED,
  TOKEN_COMMENT,
  TOKEN_CONFIG,
  TOKEN_MENUCONFIG,
  TOKEN_BOOL,
  TOKEN_TRISTATE,
  TOKEN_INT,
  TOKEN_HEX,
  TOKEN_STRING,
  TOKEN_DEFAULT,
  TOKEN_RANGE,
  TOKEN_SELECT,
  TOKEN_DEPENDS,
  TOKEN_ON,
  TOKEN_OPTION,
  TOKEN_HELP,
  TOKEN_MAINMENU,
  TOKEN_MENU,
  TOKEN_ENDMENU,
  TOKEN_CHOICE,
  TOKEN_ENDCHOICE,
  TOKEN_PROMPT,
  TOKEN_IF,
  TOKEN_ENDIF,
  TOKEN_SOURCE
};

enum config_type_e
{
  VALUE_NONE = 0,
  VALUE_INT,
  VALUE_HEX,
  VALUE_BOOL,
  VALUE_TRISTATE,
  VALUE_STRING
};

enum error_e
{
  ERROR_UNRECOGNIZED_OPTION = 1,
  ERROR_MISSING_OPTION_ARGUMENT,
  ERROR_UNEXPECTED_OPTION,
  ERROR_TOO_MANY_ARGUMENTS,
  ERROR_OUTFILE_OPEN_FAILURE,
  ERROR_BODYFILE_OPEN_FAILURE,
  ERROR_APNDXFILE_OPEN_FAILURE,
  ERROR_KCONFIG_OPEN_FAILURE,
  ERROR_APPENDFILE_OPEN_FAILURE,
  ERROR_MENU_LEVEL_UNDERRUN,
  ERROR_TOO_MANY_DEFAULTS,
  ERROR_MISSING_DEFAULT_VALUE,
  ERROR_GARBAGE_AFTER_DEFAULT,
  ERROR_DEFAULT_UNDERFLOW,
  ERROR_TOO_MANY_SELECT,
  ERROR_TOO_MANY_DEPENDENCIES,
  ERROR_DEPENDENCIES_UNDERFLOW,
  ERRROR_MISSING_ON_AFTER_DEPENDS,
  ERRROR_ON_AFTER_DEPENDS,
  ERROR_NESTING_TOO_DEEP,
  ERROR_NESTING_UNDERFLOW
};

typedef void (*output_t)(const char *fmt, ...);

struct reserved_s
{
  enum token_type_e ttype;
  const char *tname;
};

struct default_item_s
{
  char *d_default;
  char *d_dependency;
};

struct default_s
{
  int d_nitems;
  struct default_item_s d_item[MAX_DEFAULTS];
};

struct select_s
{
  int s_nvar;
  char *s_varname[MAX_SELECT];
};

struct config_s
{
  enum config_type_e c_type;
  char *c_name;
  char *c_desc;
  char *c_lower;
  char *c_upper;
  struct default_s c_default;
  struct select_s c_select;
  int c_ndependencies;
};

struct choice_s
{
  char *c_prompt;
  struct default_s c_default;
  int c_ndependencies;
};

struct menu_s
{
  char *m_name;
  int m_ndependencies;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_line[LINE_SIZE+1];
static char g_scratch[SCRATCH_SIZE+1];
static FILE *g_outfile;
static FILE *g_bodyfile;
static FILE *g_apndxfile;
static char *g_lnptr;
static bool g_debug;
static bool g_preread;
static const char *g_kconfigroot;
static const char *g_appsdir;
static int g_paranum[MAX_LEVELS];
static int g_level;
static char *g_dependencies[MAX_DEPENDENCIES];
static int g_ndependencies;
static int g_inchoice;
static int g_menu_number;
static int g_choice_number;
static int g_toggle_number;

static const char g_delimiters[] = " ,";

static struct reserved_s g_reserved[] =
{
  {TOKEN_COMMENT,    "comment"},
  {TOKEN_CONFIG,     "config"},
  {TOKEN_MENUCONFIG, "menuconfig"},
  {TOKEN_BOOL,       "bool"},
  {TOKEN_TRISTATE,   "tristate"},
  {TOKEN_INT,        "int"},
  {TOKEN_HEX,        "hex"},
  {TOKEN_STRING,     "string"},
  {TOKEN_DEFAULT,    "default"},
  {TOKEN_RANGE,      "range"},
  {TOKEN_SELECT,     "select"},
  {TOKEN_DEPENDS,    "depends"},
  {TOKEN_ON,         "on"},
  {TOKEN_OPTION,     "option"},
  {TOKEN_HELP,       "help"},
  {TOKEN_HELP,       "---help---"},
  {TOKEN_MAINMENU,   "mainmenu"},
  {TOKEN_MENU,       "menu"},
  {TOKEN_ENDMENU,    "endmenu"},
  {TOKEN_CHOICE,     "choice"},
  {TOKEN_ENDCHOICE,  "endchoice"},
  {TOKEN_PROMPT,     "prompt"},
  {TOKEN_SOURCE,     "source"},
  {TOKEN_IF,         "if"},
  {TOKEN_ENDIF,      "endif"},
  {TOKEN_NOTRESERVED, NULL}       /* Terminates list */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: debug
 *
 * Description:
 *   Debug output (conditional)
 *
 ****************************************************************************/

static void debug(const char *fmt, ...)
{
  va_list ap;

  if (g_debug)
    {
      va_start(ap, fmt);
      vfprintf(stderr, fmt, ap);
      va_end(ap);
    }
}

/****************************************************************************
 * Name: error
 *
 * Description:
 *   Error output (unconditional)
 *
 ****************************************************************************/

static void error(const char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  vfprintf(stderr, fmt, ap);
  va_end(ap);
}

/****************************************************************************
 * Name: output
 *
 * Description:
 *   Output to the final HTML file
 *
 ****************************************************************************/

static void output(const char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  vfprintf(g_outfile, fmt, ap);
  va_end(ap);
}

/****************************************************************************
 * Name: body
 *
 * Description:
 *   HTML body output to a temporary file.
 *
 ****************************************************************************/

static void body(const char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  vfprintf(g_bodyfile, fmt, ap);
  va_end(ap);
}

/****************************************************************************
 * Name: appendix
 *
 * Description:
 *   Output to a appendix file.
 *
 ****************************************************************************/

static void appendix(const char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  vfprintf(g_apndxfile, fmt, ap);
  va_end(ap);
}

/****************************************************************************
 * Name: append_file
 *
 * Description:
 *   Append the specified file to the output file and remove it.
 *
 ****************************************************************************/

static void append_file(const char *filename)
{
  FILE *stream;
  int ch;

  /* Open the file for reading */

  stream = fopen(filename, "r");
  if (!stream)
    {
      error("open %s failed: %s\n", filename, strerror(errno));
      exit(ERROR_APPENDFILE_OPEN_FAILURE);
    }

  /* Copy the file to the output */

  while ((ch = getc(stream)) != EOF)
    {
      putc(ch, g_outfile);
    }

  /* Close and remove the file */

  fclose(stream);
  unlink(filename);
}

/****************************************************************************
 * Name: show_usage
 *
 * Description:
 *   Show usage of this program and exit with the specified error code
 *
 ****************************************************************************/

static void show_usage(const char *progname, int exitcode)
{
  error("USAGE: %s [-d] [-a <apps directory>] {-o <out file>] [<Kconfig root>]\n", progname);
  error("       %s [-h]\n\n", progname);
  error("Where:\n\n");
  error("\t-a : Select relative path to the apps/ directory. Theis path is relative\n");
  error("\t     to the <Kconfig directory>.  Default: ../apps\n");
  error("\t-o : Send output to <out file>.  Default: Output goes to stdout\n");
  error("\t-d : Enable debug output\n");
  error("\t-h : Prints this message and exits\n");
  error("\t<Kconfig root> is the directory containing the root Kconfig file.\n");
  error("\t     Default <Kconfig directory>: .\n");
  exit(exitcode);
}

/****************************************************************************
 * Name: skip_space
 *
 * Description:
 *   Skip over any spaces
 *
 ****************************************************************************/

static char *skip_space(char *ptr)
{
  while (*ptr && isspace((int)*ptr)) ptr++;
  return ptr;
}

/****************************************************************************
 * Name: dequote
 *
 * Description:
 *   Remove quotation marks from a string.
 *
 ****************************************************************************/

static char *dequote(char *ptr)
{
  int len;

  /* Check if there is a trailing quote */

  len = strlen(ptr);
  if (ptr[len-1] == '"')
    {
      /* Yes... replace it with a terminator */

      ptr[len-1] = '\0';
      len--;
    }

  /* Is there a leading quote? */

  if (ptr[0] == '"')
    {
      /* Yes.. skip over the leading quote */

      ptr++;
      len--;
    }

  /* Handle the case where nothing is left after dequoting */

  if (len <= 0)
    {
      ptr = NULL;
    }

  return ptr;
}

/****************************************************************************
 * Name: htmlize_character
 *
 * Description:
 *   Transfer and HTML-ize a character. Convert characters:
 *
 *     "   &quot;   quotation mark
 *     '   &apos;   apostrophe
 *     &   &amp;     ampersand
 *     <   &lt;     less-than
 *     >   &gt;     greater-than
 *
 ****************************************************************************/

static int htmlize_character(char *dest, char ch)
{
  const char *str;

  /* Transfer the character from into the destination buffer, perform the
   * conversion only if the character is one of the special characters.
   */

  str = NULL;

  switch (ch)
    {
      case '"':
        str = "&quot;";
        break;

      case '\'':
        str = "&apos;";
        break;

      case '&':
        str = "&amp;";
        break;

      case '<':
        str = "&lt;";
        break;

      case '>':
        str = "&gt;";
        break;

      default:
        *dest++ = ch;
        *dest   = '\0';
        return 1;
    }

  /* Transfer a string */

  *dest = '\0';
  strcat(dest, str);
  return strlen(str);
}

/****************************************************************************
 * Name: htmlize_text
 *
 * Description:
 *   HTML-ize a free-text string.  This function performs the conversions of
 *   in htmlize_character() for a text string.
 *
 ****************************************************************************/

static char *htmlize_text(const char *src)
{
  char *dest = g_scratch;

  /* We may get here with the source pointer equal to NULL (or a pointer to
   * a NUL string).  Return the
   * disfavor.
   */

  if (!src || !*src)
    {
      return NULL;
    }

  /* Transfer each character from the source string into the scratch buffer */

  for (; *src; src++)
    {
      /* Expand characters as necessary.  NOTE:  There is no check if the
       * HTML-expanded text overflows the g_scratch[] buffer.  If you see
       * errors, be suspicious.
       */

      dest += htmlize_character(dest, *src);
    }

  return g_scratch;
}

/****************************************************************************
 * Name: htmlize_expression
 *
 * Description:
 *   HTML-ize an expression of configuration variables.  This function
 *   performs the same conversions as in htmlize_character(), but also
 *   expands and adds hyper links for configuration variables.
 *
 ****************************************************************************/

static char *htmlize_expression(const char *src)
{
  char varname[VAR_SIZE+1];
  char htmlvar[HTML_VAR_SIZE+1];
  char *dest = g_scratch;
  char ch = '\0';
  char lastc;

  /* We may get here with the source pointer equal to NULL.  Return the
   * disfavor.
   */

  if (!src)
    {
      return NULL;
    }

  /* Transfer each character from the source string into the scratch buffer */

  dest  = g_scratch;
  *dest = '\0';

  while (*src)
    {
      /* Remember the last character and advance to the next character */

      lastc = ch;
      ch    = *src;

      /* Skip control characters and out-of-range 7-bit ASCII characters */

      if (*src < 0x20 || *src > 0x7e)
        {
          src++;
          continue;
        }

      /* Output no more than one consecutive space character.  This depends
       * on the fact that kconfig_line has replaces all of the forms of
       * whitespace with a space character.
       */

      if (*src == ' ')
        {
          if (lastc != ' ')
            {
              *dest++ = *src;
              *dest   = '\0';
            }

          src++;
          continue;
        }

      /* Concatenate variable name strings.  There strings probably begin
       * with a uppercase letter, but here all alphanumeric values (plus '_'_
       * are concatenated.
       */

      if (isalnum(((int)*src)) || *src == '_')
        {
          int namlen = 0;

          do
            {
              /* Don't overflow the tiny variable name buffer */

              if (namlen >= VAR_SIZE)
                {
                  error("Configuration variable name too long\n");
                  break;
                }

              /* Add the next character to the name */

              varname[namlen] = *src++;
              namlen++;
              varname[namlen] = '\0';
            }
          while (isalnum(((int)*src)) || *src == '_');

          /* HTML-ize the name into our bigger, local scratch buffer */

          snprintf(htmlvar, HTML_VAR_SIZE, "<a href=\"#CONFIG_%s\"><code>CONFIG_%s</code></a>",
                   varname, varname);

          /* Then transfer the string into the scratch buffer */

          strcat(dest, htmlvar);
          dest += strlen(htmlvar);
        }

      /* All that remains are space and the punctuation characters */

      else
        {
          /* Expand characters as necessary */

          dest += htmlize_character(dest, *src);
          src++;
        }
    }

  return g_scratch;
}

/****************************************************************************
 * Name: read_line
 *
 * Description:
 *   Read a new line from the Kconfig file into the g_line[] buffer, using
 *   the g_scratch buffer if necessary to concatenate lines that end with a
 *   line continuation character (backslash).
 *
 ****************************************************************************/

static char *read_line(FILE *stream)
{
  char *ptr;
  int len;

  g_lnptr = NULL;

  /* Read the next line */

  g_line[LINE_SIZE] = '\0';
  if (!fgets(g_line, LINE_SIZE, stream))
    {
      return NULL;
    }

  /* Loop to handle continuation lines */

  for (;;)
    {
      /* How long is the line so far? */

      len = strlen(g_line);

      /* Remove any newline character at the end of the buffer */

      if (g_line[len-1] == '\n')
        {
          len--;
          g_line[len] = '\0';
        }

      /* Does this continue on the next line?  Note that this check
       * could erroneoulsy combine two lines if a comment line ends with
       * a line continuation... Don't do that!
       */

      if (g_line[len-1] != '\\')
        {
          /* No.. return now */

          g_lnptr = g_line;
          return g_line;
        }

      /* Yes.. Replace the backslash with a space delimiter */

      g_line[len-1] = ' ';

      /* Read the next line into the scratch buffer */

      g_scratch[SCRATCH_SIZE] = '\0';
      if (!fgets(g_scratch, SCRATCH_SIZE, stream))
        {
          return NULL;
        }

      /* Skip any leading whitespace and copy the rest of the next line
       * into the line buffer.  Note that the leading white space is
       * replaced with a single character to serve as a delimiter.
       */

      ptr = skip_space(g_scratch);
      strncpy(&g_line[len], ptr, LINE_SIZE - len);
    }
}

/****************************************************************************
 * Name: kconfig_line
 *
 * Description:
 *   Read a new line, skipping over leading white space and ignore lines
 *   that contain only comments.
 *
 ****************************************************************************/

static char *kconfig_line(FILE *stream)
{
  char *ptr;

  for (;;)
    {
      /* Read the next line from the Kconfig file */
      /* Is there already valid data in the line buffer?  This can happen while parsing
       * help text and we read one line too far.
       */

      if (!g_preread)
        {
          /* Read the next line */

          if (!read_line(stream))
            {
              return NULL;
            }
        }

      g_preread = false;

      /* Replace all whitespace characters with spaces to simplify parsing */

      for (ptr = g_line; *ptr; ptr++)
        {
          if (isspace(((int)*ptr)))
            {
              *ptr = ' ';
            }
        }

      /* Skip any leading whitespace.  Ignore empty lines and lines that
       * contain only comments.
       */

      ptr = skip_space(g_line);
      if (*ptr && *ptr != '#' && *ptr != '\n')
        {
          g_lnptr = ptr;
          return ptr;
        }
    }
}

/****************************************************************************
 * Name: tokenize
 *
 * Description:
 *   Check if this string corresponds to a string in the reserved word table.
 *
 ****************************************************************************/

static enum token_type_e tokenize(const char *token)
{
  struct reserved_s *ptr;

  for (ptr = g_reserved; ptr->tname; ptr++)
    {
      if (strcmp(token, ptr->tname) == 0)
        {
          break;
        }
    }

  return ptr->ttype;
}

/****************************************************************************
 * Name: findchar
 *
 * Description:
 *   Find a character in a string.  This differs from strchr() because it
 *   skips over quoted characters.  For example, if you are searching for
 *   '"', encountering '"' will terminate the search, but "\"" will not.
 *
 ****************************************************************************/

static char *findchar(char *ptr, char ch)
{
  bool escaped = false;

  /* Search for the leading quotation marked */

  for (; *ptr; ptr++)
    {
      if (escaped)
        {
          /* Skip over this character and reset the escaped flag */

          escaped = false;
        }
      else if (*ptr == '\\')
        {
          /* Set the escaped flag to skip over the next character */

          escaped = true;
        }
      else if (*ptr == ch)
        {
          /* We have found the (unescaped) character we are looking for */

          return ptr;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: get_token
 *
 * Description:
 *   Get the next delimited token from the line buffer.
 *
 ****************************************************************************/

static char *get_token(void)
{
  char *pbegin;
  char *pend = NULL;

  /* The position to begin/resume parsing is in g_lnptr. */

  if (g_lnptr && *g_lnptr)
    {
      pbegin = g_lnptr;
    }
  else
    {
      return NULL;
    }

  /* Find the beginning of the next token */

  for (;
       *pbegin && strchr(g_delimiters, *pbegin) != NULL;
       pbegin++);

  /* If we are at the end of the string with nothing
   * but delimiters found, then return NULL.
   */

  if (!*pbegin)
    {
      g_lnptr = pbegin;
      return NULL;
    }

  /* Get if the token is a quoted string */

  if (*pbegin == '"')
    {
      /*  Search for the trailing quotation mark */

      pend = findchar(pbegin + 1, '"');

      /* Did we find the trailing quotation mark */

      if (pend)
        {
          /* Yes.. skip over it */

          pend++;
        }
    }
  else
    {
      /* Find the end of the token */

      for (pend = pbegin + 1;
           *pend && strchr(g_delimiters, *pend) == NULL;
           pend++);
    }

  /* pend either points to the end of the string or to
   * the first delimiter after the string.
   */

  if (*pend)
    {
      /* Turn the delimiter into a null terminator */

      *pend++ = '\0';
    }

  /* Save the pointer where we left off and return the
   * beginning of the token.
   */

  g_lnptr = pend;
  return pbegin;
}

/****************************************************************************
 * Name: get_html_string
 *
 * Description:
 *   Extract a quoted string from the line buffer, dequote it, and make it
 *   HTML ready.
 *
 ****************************************************************************/

static char *get_html_string(void)
{
  char *pbegin;
  char *pend;
  int len;

  /* Search for the leading quotation mark in the line buffer */

  pbegin = strchr(g_lnptr, '"');
  if (pbegin)
    {
      /* We found the leading quote. Skip over the leading quote */

      pbegin++;
    }
  else
    {
      /* The string is unquoted.  The beginning of the string is here,
       * skipping over any leading whitespace.
       */

      pbegin = skip_space(g_lnptr);
    }

  /* Search for the trailing quotation mark.  If there is none, then
   * the string goes to the end of the line.
   */

  pend = findchar(pbegin, '"');
  if (pend)
    {
      /* Replace the final quote with a NUL.  g_lnptr is set to
       * the next valid character after the terminating quote.
       */

      *pend   = '\0';
      g_lnptr = pend + 1;
    }
  else
    {
      /* Get the length of the string.  Return NULL if all that is
       * left on the line is a NUL string.
       */

      len = strlen(pbegin);
      if (len < 1)
        {
          return NULL;
        }

      /* Use the rest of the line.  g_lnptr is set to point at the
       * terminating NUL.
       */

      pend = pbegin + len;
      g_lnptr = pend;
    }

  return htmlize_text(pbegin);
}

/****************************************************************************
 * Name: push_dependency
 *
 * Description:
 *   Add the new dependency to the current list of dependencies.
 *
 ****************************************************************************/

static void push_dependency(const char *dependency)
{
  int ndx = g_ndependencies;

  if (ndx >= MAX_DEPENDENCIES)
    {
      error("Too many dependencies, aborting\n");
      exit(ERROR_TOO_MANY_DEPENDENCIES);
    }

  g_dependencies[ndx] = strdup(dependency);
  g_ndependencies = ndx + 1;
}

/****************************************************************************
 * Name: pop_dependency
 *
 * Description:
 *   Remove the last, pushed dependency
 *
 ****************************************************************************/

static void pop_dependency(void)
{
  int ndx = g_ndependencies - 1;
  if (ndx < 0)
    {
      error("Dependency underflow, aborting\n");
      exit(ERROR_DEPENDENCIES_UNDERFLOW);
    }

  if (g_dependencies[ndx])
    {
      free(g_dependencies[ndx]);
      g_dependencies[ndx] = NULL;
    }

  g_ndependencies = ndx;
}

/****************************************************************************
 * Name: incr_level
 *
 * Description:
 *   Increment the paragraph numbering level
 *
 ****************************************************************************/

static void incr_level(void)
{
  int ndx = g_level;

  if (ndx >= MAX_LEVELS)
    {
      error("Nesting level is too deep, aborting\n");
      exit(ERROR_NESTING_TOO_DEEP);
    }

  g_paranum[ndx] = 1;
  g_level = ndx + 1;
}

/****************************************************************************
 * Name: decr_level
 *
 * Description:
 *   Decrease the paragraph numbering level.
 *
 ****************************************************************************/

static void decr_level(void)
{
  int ndx = g_level;

  g_paranum[ndx] = '\0';
  ndx--;

  if (ndx < 0)
    {
      error("Nesting level underflow, aborting\n");
      exit(ERROR_NESTING_UNDERFLOW);
    }

  g_level = ndx;
}

/****************************************************************************
 * Name: incr_paranum
 *
 * Description:
 *   Increment the paragraph number at this level
 *
 ****************************************************************************/

static void incr_paranum(void)
{
  int ndx = g_level - 1;

  if (ndx < 0)
    {
      error("Nesting level underflow, aborting\n");
      exit(ERROR_NESTING_UNDERFLOW);
    }

  g_paranum[ndx]++;
}

/****************************************************************************
 * Name: get_paranum
 *
 * Description:
 *   Return a string for this paragraph (uses g_scratch[]).
 *
 ****************************************************************************/

static const char *get_paranum(void)
{
  char buffer[16];
  int i;

  g_scratch[0] = '\0';
  for (i = 0; i < g_level; i++)
    {
      if (i > 0)
        {
          strcat(g_scratch, ".");
        }

      snprintf(buffer, 16, "%d", g_paranum[i]);
      strcat(g_scratch, buffer);
    }

  return g_scratch;
}

/****************************************************************************
 * Name: type2str
 *
 * Description:
 *   Return a string given a member of the configuration variable type
 *   enumeration.
 *
 ****************************************************************************/

static const char *type2str(enum config_type_e valtype)
{
  switch (valtype)
    {
      case VALUE_BOOL:
        return "Boolean";

      case VALUE_TRISTATE:
        return "Tristate";

      case VALUE_INT:
        return "Integer";

      case VALUE_HEX:
        return "Hexadecimal";

      case VALUE_STRING:
        return "String";

      default:
        break;
    }

  return "Unknown";
}

/****************************************************************************
 * Name: process_help
 *
 * Description:
 *   Read and generate HTML for the help text that is expected after the
 *   configuration configuration variable description.
 *
 ****************************************************************************/

static inline void process_help(FILE *stream, output_t outfunc)
{
  char *ptr;
  int help_indent = 0;
  int indent;
  bool blank;
  bool done;
  bool newpara;
  bool preformatted;

  /* Read each comment line */

  newpara = true;
  preformatted = false;

  for (;;)
   {
      /* Read the next line of comment text */

      if (!read_line(stream))
        {
          break;
        }

      /* What is the indentation level? The first help line sets the
       * indentation level.  The first line encounter with lower
       * indentation terminates the help.
       */

      ptr     = g_line;
      indent  = 0;
      blank   = false;
      done    = false;

      while (!done)
        {
          int ch = (int)*ptr;
          switch (ch)
            {
              case ' ':
                indent++;
                ptr++;
                break;

              case '\t':
                indent += TAB_SIZE;
                ptr++;
                break;

              case '#':
#if 0
                blank = true;
#endif
                done = true;
                break;

              case '\n':
              case '\0':
                blank = true;
                done = true;
                break;

              default:
                done = true;
                break;
            }
        }

      /* Blank lines are a special case */

      if (blank)
        {
          /* Avoid putting an empty paragraph at the end of the help */

          if (preformatted)
            {
              outfunc("</pre></ul>\n");
              preformatted = false;
            }

          if (!newpara)
            {
              outfunc("</p>\n");
              newpara = true;
            }

          continue;
        }

      /* Check the indentation level */

      if (indent == 0)
        {
          g_preread = true;
          break;
        }
      else if (!help_indent)
        {
          help_indent = indent;
        }
      else if (indent < help_indent)
        {
          g_preread = true;
          break;
        }

      /* Add the next line of help text */

      if (newpara)
        {
          outfunc("<p>\n");
          newpara = false;
        }

      /* Lines that are indented at greater levels are assumed to be
       * pre-formatted text.  This is not part of the Kconfig language but
       * rather simply a NuttX Kconfig convention.
       */

      if (indent > help_indent)
        {
          if (!preformatted)
            {
              outfunc("  <ul><pre>\n");
              newpara = false;
              preformatted = true;
            }

          outfunc("%s\n", htmlize_text(ptr));
        }
      else
        {
          if (preformatted)
            {
              outfunc("</pre></ul>\n");
              preformatted = false;
            }

          outfunc("  %s", htmlize_text(ptr));
        }
    }

  if (!newpara)
    {
      outfunc("\n</p>\n");
    }

  if (preformatted)
    {
      outfunc("</pre></ul>\n");
    }
}

/****************************************************************************
 * Name: process_default
 *
 * Description:
 *   Read and parse the Kconfig default statement.
 *
 ****************************************************************************/

static void process_default(FILE *stream, struct default_s *defp)
{
  enum token_type_e tokid;
  char *token;
  int ndx;

  /* Check if we have space for another default value */

  ndx = defp->d_nitems;
  if (ndx >= MAX_DEFAULTS)
    {
      error("Too many default values\n");
      exit(ERROR_TOO_MANY_DEFAULTS);
    }

  /* Get the next token which will be the value of the default */

  token = get_token();
  if (!token)
    {
      error("Missing default value\n");
      exit(ERROR_MISSING_DEFAULT_VALUE);
    }

  defp->d_item[ndx].d_default = strdup(token);
  defp->d_item[ndx].d_dependency = NULL;

  /* Check if the default value is followed by "depends on" */

  token = get_token();
  if (token)
    {
      /* Yes.. something follows the default value. */

      tokid = tokenize(token);
      if (tokid != TOKEN_IF)
        {
          error("Unrecognized garbage after default value\n");
          exit(ERROR_GARBAGE_AFTER_DEFAULT);
        }

      /* The rest of the line is the dependency */

      defp->d_item[ndx].d_dependency = strdup(g_lnptr);
   }

  /* Update the number of defaults we have encountered in this block */

  defp->d_nitems++;
}

/****************************************************************************
 * Name: print_default
 *
 * Description:
 *   Output and the list of defaults to the HTML body file.
 *
 ****************************************************************************/

static void print_default(struct default_s *defp, output_t outfunc)
{
  struct default_item_s *item;
  int i;

  /* Check if there are any default value */

  if (defp->d_nitems > 0)
    {
      /* Yes, output the defaults differently if there is only one */

      if (defp->d_nitems == 1)
        {
          /* Output the Default */

          item = &defp->d_item[0];
          outfunc("  <li>\n");
          outfunc("    <i>Default</i>: %s\n", item->d_default);

          /* Output the dependency */

          if (item->d_dependency)
            {
              outfunc("    <p>\n");
              outfunc("      <i>Dependency:</i>\n");
              outfunc("      %s\n", htmlize_expression(item->d_dependency));
              outfunc("    </p>\n");
            }

          outfunc("  </li>\n");
        }
      else
        {
          /* Output a sub-list of defaults. */

          outfunc("  <li>\n");
          outfunc("    <i>Default Values</i>:\n");
          outfunc("    <ul>\n");

          for (i = 0; i < defp->d_nitems; i++)
            {
              /* Output the Default */

              item = &defp->d_item[i];
              outfunc("      <li>\n");
              outfunc("        <i>Default</i>: %s\n", item->d_default);

              /* Output the dependency */

              if (item->d_dependency)
                {
                  outfunc("        <p>\n");
                  outfunc("          <i>Dependency:</i>\n");
                  outfunc("          %s\n", htmlize_expression(item->d_dependency));
                  outfunc("        </p>\n");
                }
            }

          outfunc("    </ul>\n");
          outfunc("  </li>\n");
        }
    }
}

/****************************************************************************
 * Name: free_default
 *
 * Description:
 *   Output and the list of defaults to the HTML body file.
 *
 ****************************************************************************/

static void free_default(struct default_s *defp)
{
  struct default_item_s *item;
  int i;

  /* Free strings for each default */

  for (i = 0; i < defp->d_nitems; i++)
    {
      /* Free the default value string */

      item = &defp->d_item[i];
      free(item->d_default);

      /* Free any dependency on the default */

      if (item->d_dependency)
        {
          free(item->d_dependency);
        }
    }
}

/****************************************************************************
 * Name: process_dependson
 *
 * Description:
 *   Parse a "depends on" dependency and add the new dependency to the
 *   stack of dependencies.
 *
 ****************************************************************************/

static void process_dependson(void)
{
  char *value = get_token();
  if (strcmp(value, "on") != 0)
    {
      error("Expected \"on\" after \"depends\"\n");
      exit(ERRROR_ON_AFTER_DEPENDS);
    }

    push_dependency(htmlize_expression(g_lnptr));
}

/****************************************************************************
 * Name: print_dependencies
 *
 * Description:
 *   Output the current stack of dependencies
 *
 ****************************************************************************/

static void print_dependencies(output_t outfunc)
{
  int i;

  if (g_ndependencies > 0)
    {
      outfunc("  <li><i>Dependencies</i>: %s", g_dependencies[0]);

      for (i = 1; i < g_ndependencies; i++)
        {
          outfunc(", %s\n", g_dependencies[i]);
        }

      outfunc("</li>\n");
    }
}

/****************************************************************************
 * Name: free_dependencies
 *
 * Description:
 *   Pop dependencies from the stack.
 *
 ****************************************************************************/

static void free_dependencies(int ndependencies)
{
  int i;

  for (i = 0; i < ndependencies; i++)
    {
      pop_dependency();
    }
}

/****************************************************************************
 * Name: process_config
 *
 * Description:
 *   Process one configuration variable paragraph
 *
 ****************************************************************************/

static inline char *process_config(FILE *stream, const char *varname,
                                   const char *kconfigdir,
                                   const char *kconfigname)
{
  enum token_type_e tokid;
  struct config_s config;
  output_t outfunc;
  bool help;
  bool hidden;
  const char *paranum;
  char *token;
  char *ptr;
  int i;

  /* Get the configuration information */

  memset(&config, 0, sizeof(struct config_s));
  config.c_name = strdup(varname);

  /* Process each line in the configuration */

  help = false;
  token = NULL;

  while ((ptr = kconfig_line(stream)) != NULL)
    {
      /* Process the first token on the Kconfig file line */

      token = get_token();
      if (token != NULL)
        {
          tokid = tokenize(token);
          switch (tokid)
            {
              case TOKEN_BOOL:
              case TOKEN_TRISTATE:
                {
                  /* Save the type of the configuration variable */

                  config.c_type = tokid == TOKEN_BOOL ? VALUE_BOOL : VALUE_TRISTATE;

                  /* Get the description following the type */

                  ptr = get_html_string();
                  if (ptr)
                    {
                      config.c_desc = strdup(ptr);
                    }

                  /* Indicate that the line has been consumed */

                  token = NULL;
                }
                break;

              case TOKEN_INT:
                {
                  /* Save the type of the configuration variable */

                  config.c_type = VALUE_INT;

                  /* Get the description following the type */

                  ptr = get_html_string();
                  if (ptr)
                    {
                      config.c_desc = strdup(ptr);
                    }

                  /* Indicate that the line has been consumed */

                  token = NULL;
                }
                break;

              case TOKEN_HEX:
                {
                  /* Save the type of the configuration variable */

                  config.c_type = VALUE_HEX;

                  /* Get the description following the type */

                  ptr = get_html_string();
                  if (ptr)
                    {
                      config.c_desc = strdup(ptr);
                    }

                  /* Indicate that the line has been consumed */

                  token = NULL;
                }
                break;

              case TOKEN_STRING:
                {
                  /* Save the type of the configuration variable */

                  config.c_type = VALUE_STRING;

                  /* Get the description following the type */

                  ptr = get_html_string();
                  if (ptr)
                    {
                      config.c_desc = strdup(ptr);
                    }

                  /* Indicate that the line has been consumed */

                  token = NULL;
                }
                break;

              case TOKEN_DEFAULT:
                {
                  process_default(stream, &config.c_default);
                  token = NULL;
                }
                break;

              case TOKEN_RANGE:
                {
                  char *value = get_token();
                  if (value)
                    {
                      config.c_lower = strdup(value);

                      value = get_token();
                      if (value)
                        {
                          config.c_upper = strdup(value);
                        }
                    }

                  token = NULL;
                }
                break;

              case TOKEN_SELECT:
                {
                  char *value;
                  int ndx;

                  ndx = config.c_select.s_nvar;
                  if (ndx >= MAX_SELECT)
                    {
                      error("Too many 'select' lines\n");
                      exit(ERROR_TOO_MANY_SELECT);
                    }

                  value = get_token();
                  config.c_select.s_varname[ndx] = strdup(value);
                  config.c_select.s_nvar = ndx + 1;
                  token = NULL;
                }
                break;

              case TOKEN_DEPENDS:
                {
                  process_dependson();
                  config.c_ndependencies++;
                  token = NULL;
                }
                break;

              case TOKEN_OPTION:
                {
                  token = NULL; /* Ignored */
                }
                break;

              case TOKEN_HELP:
                {
                  help  = true;
                  token = NULL;
                }
                break;

              default:
                {
                  debug("CONFIG_%s: Terminating token: %s\n",
                        config.c_name, token);
                }
                break;
            }

          /* Break out on the help token (or the first unhandled token) */

          if (help || token != NULL)
            {
              break;
            }
        }
    }

  /* Is this an internal configuration variable with no description?
   * If so, send the output to the appendix file.
   */

  hidden  = (config.c_desc == NULL);
  outfunc = hidden ? appendix : body;
  hidden |= g_inchoice;

  /* Print the configuration variable name and the short description */

  outfunc("<h3><a name=\"CONFIG_%s\">", config.c_name);

  /* If we are not in a choice block, than give the variable a paragraph
   * number and put it in the table of contents.
   */

  if (!hidden)
    {
      paranum = get_paranum();
      output("<li><a href=\"#CONFIG_%s\">%s <code>CONFIG_%s</code>",
             config.c_name, paranum, config.c_name);
      outfunc("%s ", paranum);
      incr_paranum();
    }

  outfunc("<code>CONFIG_%s</code>", config.c_name);

  /* Output the short description in the paragraph title (if we have one) */

  if (config.c_desc)
    {
      if (!hidden)
        {
          output(": %s", config.c_desc);
        }

      outfunc(": %s", config.c_desc);
    }

  outfunc("</a></h3>\n");

  if (!hidden)
    {
      output("</a></li>\n");
    }

  /* Configuration description is indented */

  outfunc("<ul>\n");

  /* Print the type of the configuration variable */

  if (config.c_type != VALUE_NONE)
    {
      outfunc("  <li><i>Type</i>: %s</li>\n", type2str(config.c_type));
    }

  /* Print the default values of the configuration variable */

  print_default(&config.c_default, outfunc);

  /* Print the range of values of the configuration variable */

  if (config.c_lower || config.c_upper)
    {
      outfunc("  <li><i>Range</i>:\n");
      if (config.c_lower)
        {
          outfunc(" %s", config.c_lower);
        }

      outfunc(" -", config.c_lower);

      if (config.c_upper)
        {
          outfunc(" %s", config.c_upper);
        }

      outfunc("</li>\n");
    }

  /* Print the default value of the configuration variable auto-selected by this setting */

  if (config.c_select.s_nvar > 0)
    {
      outfunc("  <li><i>Selects</i>: <a href=\"#CONFIG_%s\"><code>CONFIG_%s</code></a>",
              config.c_select.s_varname[0], config.c_select.s_varname[0]);

      for (i = 1; i < config.c_select.s_nvar; i++)
        {
          outfunc(", <a href=\"#CONFIG_%s\"><code>CONFIG_%s</code></a>",
                  config.c_select.s_varname[i], config.c_select.s_varname[i]);
        }

      outfunc("</li>\n");
    }

  /* Print the list of dependencies (if any) */

  print_dependencies(outfunc);

  /* Show the configuration file. */

  outfunc("  <li><i>Kconfig file</i>: <code>%s/%s</code>\n",
          kconfigdir, kconfigname);

  /* Print any help text */

  if (help)
    {
      process_help(stream, outfunc);
      token = NULL;
    }

  /* End of configuration description */

  outfunc("</ul>\n");

  /* Free allocated memory */

  free_dependencies(config.c_ndependencies);
  free_default(&config.c_default);

  if (config.c_name)
    {
      free(config.c_name);
    }

  if (config.c_desc)
    {
      free(config.c_desc);
    }

  if (config.c_lower)
    {
      free(config.c_lower);
    }

  if (config.c_upper)
    {
      free(config.c_upper);
    }

  if (config.c_select.s_nvar > 0)
    {
      for (i = 0; i < config.c_select.s_nvar; i++)
        {
          free(config.c_select.s_varname[i]);
        }
    }

  return token;
}

/****************************************************************************
 * Name: process_choice
 *
 * Description:
 *   Process a choice paragraph
 *
 ****************************************************************************/

static char *parse_kconfigfile(FILE *stream, const char *kconfigdir,
                               const char *kconfigfile); /* Forward reference */

static inline char *process_choice(FILE *stream, const char *kconfigdir,
                                   const char *kconfigname)
{
  enum token_type_e tokid;
  struct choice_s choice;
  const char *paranum;
  char *token = NULL;
  char *ptr;
  bool help = false;

  /* Get the choice information */

  memset(&choice, 0, sizeof(struct choice_s));

  /* Process each line in the choice */

  while ((ptr = kconfig_line(stream)) != NULL)
    {
      /* Process the first token on the Kconfig file line */

      token = get_token();
      if (token != NULL)
        {
          tokid = tokenize(token);
          switch (tokid)
            {
              case TOKEN_PROMPT:
                {
                  /* Get the prompt string */

                  ptr = get_html_string();
                  if (ptr)
                    {
                      choice.c_prompt = strdup(ptr);
                    }

                  /* Indicate that the line has been consumed */

                  token = NULL;
                }
                break;

              case TOKEN_DEFAULT:
                {
                  process_default(stream, &choice.c_default);
                  token = NULL;
                }
                break;

              case TOKEN_DEPENDS:
                {
                  process_dependson();
                  choice.c_ndependencies++;
                  token = NULL;
                }
                break;

              case TOKEN_HELP:
                {
                  help  = true;
                  token = NULL;
                }
                break;

              default:
                {
                  debug("Choice: Terminating token: %s\n", token);
                }
                break;
            }

          /* Break out on the help token (or the first unhandled token) */

          if (help || token != NULL)
            {
              break;
            }
        }
    }

  paranum = get_paranum();
  output("<li><a href=\"#choice_%d\">%s Choice", g_choice_number, paranum);
  body("\n<h3><a name=\"choice_%d\">%s Choice", g_choice_number, paranum);

  if (choice.c_prompt)
    {
      output(": %s", choice.c_prompt);
      body(": %s", choice.c_prompt);
    }

  output("</a></li>\n");
  body("</a></h3>\n");
  g_choice_number++;

  /* Print the default values of the configuration variable */

  body("<ul>\n");
  print_default(&choice.c_default, body);

  /* Print the list of dependencies (if any) */

  print_dependencies(body);

  /* Show the configuration file.
   * REVISIT: Shows wrong file name if the name of the Kconfig file is not
   * Kconfig.
   */

   body("  <li><i>Kconfig file</i>: <code>%s/%s</code>\n</li>",
        kconfigdir, kconfigname);

   /* Print any help text */

  if (help)
    {
      process_help(stream, body);
      token = NULL;
    }

   body("</ul>\n");

   /* Then show the choice options */

   body("<p><b>Choice Options:</b></p>");
   body("<ul>\n");

  /* Free allocated memory */

  free_dependencies(choice.c_ndependencies);
  free_default(&choice.c_default);

  if (choice.c_prompt)
    {
      free(choice.c_prompt);
    }

  /* Increment the paragraph level */

  incr_level();

  debug("process_choice: TOKEN_CHOICE\n");
  debug("  kconfigdir:  %s\n", kconfigdir);
  debug("  kconfigname: %s\n", kconfigname);
  debug("  level:       %d\n", g_level);

  /* Then return in choice mode */

  g_inchoice++;
  return token;
}

/****************************************************************************
 * Name: process_menu
 *
 * Description:
 *   Process a menu paragraph
 *
 ****************************************************************************/

static inline char *process_menu(FILE *stream, const char *kconfigdir,
                                 const char *kconfigname)
{
  enum token_type_e tokid;
  struct menu_s menu;
  const char *paranum;
  char *menuname;
  char *token = NULL;

  /* Get the menu information */

  memset(&menu, 0, sizeof(struct menu_s));

  /* Get the menu name */

  menuname = get_html_string();
  menu.m_name = strdup(menuname);

  /* Process each line in the choice */

  while (kconfig_line(stream) != NULL)
    {
      /* Process the first token on the Kconfig file line */

      token = get_token();
      if (token != NULL)
        {
          tokid = tokenize(token);
          switch (tokid)
            {
              case TOKEN_DEPENDS:
                {
                  process_dependson();
                  menu.m_ndependencies++;
                  token = NULL;
                }
                break;

              default:
                {
                  debug("Menu: Terminating token: %s\n", token);
                }
                break;
            }

          /* Break out on the first unhandled token */

          if (token != NULL)
            {
              break;
            }
        }
    }

  /* Output menu information */

  paranum = get_paranum();
  if (menu.m_name)
    {
      output("<li><a name=\"menu_%d_toc\"><a href=\"#menu_%d\">%s Menu: %s</a></a></li>\n",
             g_menu_number, g_menu_number, paranum, menu.m_name);
      body("\n<h1><a name=\"menu_%d\">%s Menu: %s</a></h1>\n",
           g_menu_number, paranum, menu.m_name);
    }
  else
    {
      output("<li><a name=\"menu_%d_toc\"><a href=\"#menu_%d\">%s Menu</a></a></li>\n",
             g_menu_number, g_menu_number, paranum);
      body("\n<h1><a name=\"menu_%d\">%s Menu</a></h1>\n",
             g_menu_number, paranum);
    }

  /* Output logic to toggle the contents below the menu in the table of
   * contents.
   */

#ifdef USE_JQUERY
  output("<a id=\"link_%d\" href=\"#menu_%d_toc\" onclick=\"toggle('toggle_%d', 'link_%d')\">Expand</a>\n",
         g_menu_number, g_toggle_number, g_toggle_number);
#else
  output("<a href=\"#menu_%d_toc\" onclick=\"toggle('toggle_%d', this)\">Expand</a>\n",
         g_menu_number, g_toggle_number);
#endif
  output("<ul id=\"toggle_%d\"  style=\"display:none\">\n",
         g_toggle_number);

  g_menu_number++;
  g_toggle_number++;

  /* Print the list of dependencies (if any) */

  body("<ul>\n");
  print_dependencies(body);

  /* Show the configuration file */

  body("  <li><i>Kconfig file</i>: <code>%s/%s</code>\n",
       kconfigdir, kconfigname);
  body("</ul>\n");

  /* Free any allocated memory */

  free_dependencies(menu.m_ndependencies);

  if (menu.m_name)
    {
      free(menu.m_name);
    }

  /* Increment the paragraph level */

  incr_level();

  debug("process_menu: TOKEN_MENU\n");
  debug("  kconfigdir:  %s\n", kconfigdir);
  debug("  kconfigname: %s\n", kconfigname);
  debug("  level:       %d\n", g_level);

  /* Return the terminating token */

  return token;
}

/****************************************************************************
 * Name: parse_kconfigfile
 *
 * Description:
 *   Parse a Kconfig file.
 *
 ****************************************************************************/

static void process_kconfigfile(const char *kconfigdir, const char *kconfigname); /* Forward reference */
static char *parse_kconfigfile(FILE *stream, const char *kconfigdir,
                               const char *kconfigname)
{
  enum token_type_e tokid;
  char *token = NULL;

  /* Process each line in the Kconfig file */

  while (kconfig_line(stream) != NULL)
    {
      /* Process the first token on the Kconfig file line */

      token = get_token();
      while (token != NULL)
        {
          tokid = tokenize(token);

          switch (tokid)
            {
              case TOKEN_SOURCE:
                {
                  /* Get the relative path from the Kconfig file line */

                  char *source = get_token();

                  /* Remove optional quoting */

                  source = dequote(source);
                  if (source)
                    {
                      char *configname = basename(source);
                      char *subdir = dirname(source);
                      char *dirpath;

                      /* Check for an absolute path */

                      if (source[0] == '/')
                        {
                          dirpath = strdup(subdir);
                        }
                      else
                        {
                          /* Check if the directory path contains $APPSDIR */

                          char *appsdir = strstr(subdir, "$APPSDIR");
                          if (appsdir)
                            {
                              char *tmp = appsdir + strlen("$APPSDIR");

                              *appsdir = '\0';
                              asprintf(&dirpath, "%s/%s%s%s",
                                       g_kconfigroot, subdir, g_appsdir, tmp);
                            }
                          else
                            {
                              asprintf(&dirpath, "%s/%s", g_kconfigroot, subdir);
                            }
                        }

                      configname = strdup(configname);

                      debug("parse_kconfigfile: Recursing for TOKEN_SOURCE\n");
                      debug("  source:      %s\n", source);
                      debug("  subdir:      %s\n", subdir);
                      debug("  dirpath:     %s\n", dirpath);
                      debug("  configname:  %s\n", configname);

                      /* Then recurse */

                      process_kconfigfile(dirpath, configname);
                      token = NULL;
                      free(dirpath);
                      free(configname);
                    }

                  /* Set the token string to NULL to indicate that we need to read the next line */

                  token = NULL;
                }
                break;

              case TOKEN_CONFIG:
              case TOKEN_MENUCONFIG:
                {
                  char *varname = get_token();
                  token = process_config(stream, varname, kconfigdir,
                                         kconfigname);
                }
                break;

              case TOKEN_COMMENT:
              case TOKEN_MAINMENU:
                {
                  token = NULL; /* ignored */
                }
                break;

              case TOKEN_MENU:
                {
                  token = process_menu(stream, kconfigdir, kconfigname);
                }
                break;

              case TOKEN_CHOICE:
                {
                  token = process_choice(stream, kconfigdir, kconfigname);
                }
                break;

              case TOKEN_ENDCHOICE:
                {
                  /* Reduce body indentation level */

                  body("</ul>\n");
                  g_inchoice--;

                  /* Decrement the paragraph level */

                  decr_level();
                  incr_paranum();
                  token = NULL;
                }
                break;

              case TOKEN_ENDMENU:
                {
                  /* Reduce table of contents indentation level.  NOTE that
                   * this also terminates the toggle block that began with the
                   * matching <ul>
                   */

                  output("</ul>\n");

                  /* Decrement the paragraph level */

                  decr_level();
                  incr_paranum();
                  token = NULL;
                }
                break;

              case TOKEN_IF:
                {
                  char *dependency = get_token();
                  push_dependency(htmlize_expression(dependency));
                  token = NULL;
                }
                break;

              case TOKEN_ENDIF:
                {
                  pop_dependency();
                  token = NULL;
                }
                break;

              default:
                {
                  /* Set token to NULL to skip to the next line. */

                  error("File %s/%s Unhandled token: %s\n",
                        kconfigdir, kconfigname, token);
                  token = NULL;
                }
                break;
            }
        }
    }

  return token;
}

/****************************************************************************
 * Name: process_kconfigfile
 *
 * Description:
 *   Open and parse a Kconfig file
 *
 ****************************************************************************/

static void process_kconfigfile(const char *kconfigdir,
                                const char *kconfigname)
{
  FILE *stream;
  char *kconfigpath;

  /* Create the full path to the Kconfig file */

  asprintf(&kconfigpath, "%s/%s", kconfigdir, kconfigname);
  debug("process_kconfigfile: Entry\n");
  debug("  kconfigdir:  %s\n", kconfigdir);
  debug("  kconfigpath: %s\n", kconfigpath);
  debug("  level:       %d\n", g_level);

  /* Open the Kconfig file */

  stream = fopen(kconfigpath, "r");
  if (!stream)
    {
      error("open %s failed: %s\n", kconfigpath, strerror(errno));
      exit(ERROR_KCONFIG_OPEN_FAILURE);
    }

  /* Process each line in the Kconfig file */

  parse_kconfigfile(stream, kconfigdir, kconfigname);

  /* Close the Kconfig file and release the memory holding the full path */

  fclose(stream);
  free(kconfigpath);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main
 *
 * Description:
 *   Program entry point.
 *
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  char *outfile;
  const char *paranum;
  time_t now;
  struct tm *ptm;
  int ch;

  /* Parse command line options */

  g_debug       = false;
  g_kconfigroot = ".";
  g_appsdir     = "../apps";
  g_outfile     = stdout;
  outfile       = NULL;

  while ((ch = getopt(argc, argv, ":dhia:o:")) > 0)
    {
      switch (ch)
        {
          case 'a' :
            g_appsdir = optarg;
            break;

          case 'o' :
            outfile = optarg;
            break;

          case 'h' :
            show_usage(argv[0], 0);

          case 'd' :
            g_debug = true;
            break;

          case '?' :
            error("Unrecognized option: %c\n", optopt);
            show_usage(argv[0], ERROR_UNRECOGNIZED_OPTION);

          case ':' :
            error("Missing option argument, option: %c\n", optopt);
            show_usage(argv[0], ERROR_MISSING_OPTION_ARGUMENT);

          default:
            error("Unexpected option: %c\n", ch);
            show_usage(argv[0], ERROR_UNEXPECTED_OPTION);
        }
    }

  if (optind < argc)
    {
      g_kconfigroot = argv[optind];
      optind++;
    }

  debug("Using <Kconfig directory>: %s\n", g_kconfigroot);
  debug("Using <apps directory>:    %s\n", g_appsdir);
  debug("Using <out file>:          %s\n", outfile ? outfile : "stdout");

  if (optind < argc)
    {
       error("Unexpected garbage at the end of the line\n");
       show_usage(argv[0], ERROR_TOO_MANY_ARGUMENTS);
    }

  /* Open the output file (if any).  The output file will hold the
   * Table of Contents as the HTML document is generated.
   */

  if (outfile)
    {
      g_outfile = fopen(outfile, "w");
      if (!g_outfile)
        {
          error("open %s failed: %s\n", outfile, strerror(errno));
          exit(ERROR_OUTFILE_OPEN_FAILURE);
        }
    }

  /* Open the temporary file that holds the HTML body.  The HTML
   * body will be appended after the Table of contents.
   */

  g_bodyfile = fopen(BODYFILE_NAME, "w");
  if (!g_bodyfile)
    {
      error("open %s failed: %s\n", BODYFILE_NAME, strerror(errno));
      exit(ERROR_BODYFILE_OPEN_FAILURE);
    }

  /* Open the temporary file that holds the appendix.  The appendix
   * will be appended after the HTML body.
   */

  g_apndxfile = fopen(APNDXFILE_NAME, "w");
  if (!g_apndxfile)
    {
      error("open %s failed: %s\n", APNDXFILE_NAME, strerror(errno));
      exit(ERROR_APNDXFILE_OPEN_FAILURE);
    }

  /* Get the current date string in the scratch buffer */

  now = time(NULL);
  ptm = localtime(&now);
  strftime(g_scratch, SCRATCH_SIZE, "%B %d, %Y", ptm);

  /* Output header boilerplater */

  output("<html>\n");
  output("<head>\n");
  output("<title>NuttX Configuration Options</title>\n");
  output("</head>\n");

  output("<body background=\"backgd.gif\">\n");
  output("<hr><hr>\n");
  output("<table width =\"100%%\">\n");
  output("<tr align=\"center\" bgcolor=\"#e4e4e4\">\n");
  output("<td>\n");
  output("<h1><big><font color=\"#3c34ec\"><i>NuttX Configuration Variables</i></font></big></h1>\n");
  output("<p>Last Updated: %s</p>\n", g_scratch);
  output("</td>\n");
  output("</tr>\n");
  output("</table>\n");

#ifdef USE_JQUERY
  output("<script src=\"http://code.jquery.com/jquery-1.9.1.js\"></script>\n");
  output("<script type=\"text/javascript\">\n");
  output("function toggle(list_id, link_id) {\n");
  output("  var list = $('#' + list_id);\n");
  output("  var link = $('#' + link_id);\n");
  output("  if (list.is(\":visible\")) {\n");
  output("    list.hide();\n");
  output("    link.text('Expand');\n");
  output("  } else {\n");
  output("    list.show();\n");
  output("    link.text('Collapse');\n");
  output("  }\n");
  output("}\n");
  output("</script>\n");
#else
  output("<script type=\"text/javascript\">\n");
  output("function toggle(id, link) {\n");
  output("  var e = document.getElementById(id);\n");
  output("  if (e.style.display == '') {\n");
  output("    e.style.display = 'none';\n");
  output("    link.innerHTML = 'Expand';\n");
  output("  } else {\n");
  output("    e.style.display = '';\n");
  output("    link.innerHTML = 'Collapse';\n");
  output("  }\n");
  output("}\n");
  output("</script>\n");
#endif

  output("<hr><hr>\n");
  output("<table width =\"100%%\">\n");
  output("  <tr bgcolor=\"#e4e4e4\">\n");
  output("    <td>\n");
  output("      <h1>Table of Contents</h1>\n");
  output("    </td>\n");
  output("  </tr>\n");
  output("</table>\n");
  output("<ul>\n");

  incr_level();
  paranum = get_paranum();
  output("<li><a href=\"#menu_%d\">%s Menu: Main</a></li>\n",
         g_menu_number, paranum);

  body("<table width =\"100%%\">\n");
  body("  <tr bgcolor=\"#e4e4e4\">\n");
  body("  <td>\n");
  body("    <a name=\"menu_%d\"><h1>%s Menu: Main</h1></a>\n",
       g_menu_number, paranum);
  body("  </td>\n");
  body("  </tr>\n");
  body("</table>\n");

  g_menu_number++;

  /* Increment the paragraph level again:  Everything is included within the main menu. */

  incr_level();

  /* Tell the reader that this is an auto-generated file */

  body("<p>\n");
  body("  <b>Overview</b>.\n");
  body("  The NuttX RTOS is highly configurable.\n");
  body("  The NuttX configuration files are maintained using the <a href=\"https://bitbucket.org/nuttx/tools/src/master/kconfig-frontends\">kconfig-frontends</a> tool.\n");
  body("  That configuration tool uses <code>Kconfig</code> files that can be found through the NuttX source tree.\n");
  body("  Each <code>Kconfig</code> files contains declarations of configuration variables.\n");
  body("  Each configuration variable provides one configuration option for the NuttX RTOS.\n");
  body("  This configurable options are described in this document.\n");
  body("</p>\n");
  body("<p>\n");
  body("  <b>Main Menu</b>.\n");
  body("  The normal way to start the NuttX configuration is to enter this command line from the NuttX build directory: <code>make menuconfig</code>.\n");
  body("  Note that NuttX must first be configured <i>before</i> this command so that the configuration file (<code>.config</code>) is present in the top-level build directory.\n");
  body("  The main menu is the name give to the opening menu display after this command is executed.\n");
  body("</p>\n");
  body("<p>\n");
  body("  <b>Maintenance Note</b>.\n");
  body("  This documentation was auto-generated using the <a href=\"https://bitbucket.org/nuttx/nuttx/src/master/tools/kconfig2html.c\">kconfig2html</a> tool\n");
  body("  That tool analyzes the NuttX <code>Kconfig</code> files and generates this HTML document.\n");
  body("  This HTML document file should not be edited manually.\n");
  body("  In order to make changes to this document, you should instead modify the <code>Kconfig</code> file(s) that were used to generated this document and then execute the <code>kconfig2html</code> again to regenerate the HTML document file.\n");
  body("</p>\n");

  /* Process the Kconfig files through recursive descent */

  process_kconfigfile(g_kconfigroot, "Kconfig");

  /* Terminate the table of contents */

  output("<li><a href=\"#appendixa\">Appendix A: Hidden Configuration Variables</a></li>\n");
  output("</ul>\n");

  /* Close the HMTL body file and copy it to the output file */

  fclose(g_bodyfile);
  append_file(BODYFILE_NAME);

  /* Output introductory information for the appendix */

  output("<table width =\"100%%\">\n");
  output("  <tr bgcolor=\"#e4e4e4\">\n");
  output("  <td>\n");
  output("    <a name=\"appendixa\"><h1>Appendix A: Hidden Configuration Variables</h1></a>\n");
  output("  </td>\n");
  output("  </tr>\n");
  output("</table>\n");

  output("<p>\n");
  output("  This appendix holds internal configurations variables that are not visible to the user.\n");
  output("  These settings are presented out-of-context because they cannot be directly controlled by the user.\n");
  output("  Many of these settings are selected automatically and indirectly when other, visible configuration variables are selected.\n");
  output("  One purpose of these hidden configuration variables is to control menuing in the kconfig-frontends configuration tool.\n");
  output("  Many configuration variables with a form like <code>CONFIG_ARCH_HAVE_</code><i>feature</i>, for example, are used only to indicate that the selected architecture supports <i>feature</i> and so addition selection associated with <i>feature</i> will become accessible to the user.\n");
  output("</p>\n");
  output("<ul>\n");

  /* Close the appendix file and copy it to the output file */

  fclose(g_apndxfile);
  append_file(APNDXFILE_NAME);

  /* Output trailer boilerplater */

  output("</ul>\n");
  output("</body>\n");
  output("</html>\n");

  /* Close the output file (if any) and the temporary file*/

  if (outfile)
    {
      fclose(g_outfile);
    }

  return 0;
}
