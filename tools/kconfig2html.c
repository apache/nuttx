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

#define LINE_SIZE        1024
#define SCRATCH_SIZE     1024
#define MAX_DEPENDENCIES 100
#define MAX_LEVELS       100
#define MAX_SELECT       16
#define MAX_DEFAULTS     80
#define TAB_SIZE         4
#define VAR_SIZE         80
#define HTML_VAR_SIZE    (2*VAR_SIZE + 64)

#define TMPFILE_NAME     "kconfig2html-tmp.dat"

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
  VALUE_STRING
};

enum error_e
{
  ERROR_UNRECOGNIZED_OPTION = 1,
  ERROR_MISSING_OPTION_ARGUMENT,
  ERROR_UNEXPECTED_OPTION,
  ERROR_TOO_MANY_ARGUMENTS,
  ERROR_OUTFILE_OPEN_FAILURE,
  ERROR_TMPFILE_OPEN_FAILURE,
  ERROR_KCONFIG_OPEN_FAILURE,
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

struct reserved_s
{
  enum token_type_e ttype;
  const char *tname;
};

struct default_item_s
{
  char *ddefault;
  char *ddependency;
};

struct default_s
{
  int dnitems;
  struct default_item_s ditem[MAX_DEFAULTS];
};

struct select_s
{
  int snvar;
  char *svarname[MAX_SELECT];
};

struct config_s
{
  enum config_type_e ctype;
  char *cname;
  char *cdesc;
  char *clower;
  char *cupper;
  struct default_s cdefault;
  struct select_s cselect;
  int cndependencies;
};

struct choice_s
{
  char *cprompt;
  struct default_s cdefault;
  int cndependencies;
};

struct menu_s
{
  char *mname;
  int mndependencies;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_line[LINE_SIZE+1];
static char g_scratch[SCRATCH_SIZE+1];
static FILE *g_outfile;
static FILE *g_tmpfile;
static char *g_lnptr;
static bool g_debug;
static bool g_internal;
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

static const char g_delimiters[] = " ,";

static struct reserved_s g_reserved[] =
{
  {TOKEN_COMMENT,    "comment"},
  {TOKEN_CONFIG,     "config"},
  {TOKEN_MENUCONFIG, "menuconfig"},
  {TOKEN_BOOL,       "bool"},
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
      (void)vfprintf(stderr, fmt, ap);
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
  (void)vfprintf(stderr, fmt, ap);
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
  (void)vfprintf(g_outfile, fmt, ap);
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
  (void)vfprintf(g_tmpfile, fmt, ap);
  va_end(ap);
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
  error("USAGE: %s [-d] [-i] [-a <apps directory>] {-o <out file>] [<Kconfig root>]\n", progname);
  error("       %s [-h]\n\n", progname);
  error("Where:\n\n");
  error("\t-a : Select relative path to the apps/ directory. Theis path is relative\n");
  error("\t     to the <Kconfig directory>.  Default: ../apps\n");
  error("\t-o : Send output to <out file>.  Default: Output goes to stdout\n");
  error("\t-i : Show hidden, internal configuration variables\n");
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

  /* Check if there is a traiing quote */

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
   * conversion only the the character is one of the special characters.
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
 *   HTML-ize a free-text string.  This function preforms the conversions of
 *   in htmlize_character() for a text string.
 *
 ****************************************************************************/

static char *htmlize_text(const char *src)
{
  char *dest = g_scratch;

  /* We may get here with the source pointer equal to NULL.  Return the
   * disfavor.
   */

  if (!src)
    {
      return NULL;
    }

  /* Transfer each character from the source string into the scratch buffer */

  for (; *src; src++)
    {
      /* Expand characters as necessary */

      dest += htmlize_character(dest, *src);
    }

  return g_scratch;
}

/****************************************************************************
 * Name: htmlize_expression
 *
 * Description:
 *   HTML-ize an expression of configuration variables.  This function
 *   preforms the same conversions as in htmlize_character(), but also
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

  for(;;)
    {
      /* How long is the line so far? */

      len = strlen(g_line);

      /* Remove any newline character at the end of the buffer */

      if (g_line[len-1] == '\n')
        {
          len--;
          g_line[len] = '\0';
        }

      /* Does this continue on the next line?  Note taht this check
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

  /* Search for the leading quotation mark in the line buffer */

  pbegin = strchr(g_lnptr, '"');
  if (pbegin)
    {
      /* Skip over the quote */

      pbegin++;

      /*  Search for the trailing quotation mark */

      pend = findchar(pbegin, '"');
      if (pend)
        {
          /* Replace the final quote with a NUL */

          *pend = '\0';
        }
    }

  g_lnptr = pend + 1;
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

static inline void process_help(FILE *stream)
{
  char *ptr;
  int help_indent = 0;
  int indent;
  bool blank;
  bool done;
  bool newpara;

  /* Read each comment line */

  newpara = true;
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

          if (!newpara)
            {
              body("</p>\n");
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
          body("</p>\n");
          newpara = false;
        }

      body("  %s", htmlize_text(ptr));
    }

  if (!newpara)
    {
      body("</p>\n");
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

  ndx = defp->dnitems;
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

  defp->ditem[ndx].ddefault = strdup(token);
  defp->ditem[ndx].ddependency = NULL;

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

      defp->ditem[ndx].ddependency = strdup(g_lnptr);
   }

  /* Update the number of defaults we have encountered in this block */

  defp->dnitems++;
}

/****************************************************************************
 * Name: print_default
 *
 * Description:
 *   Output and the list of defaults to the the HTML body file.
 *
 ****************************************************************************/

static void print_default(struct default_s *defp)
{
  struct default_item_s *item;
  int i;

  /* Check if there are any default value */

  if (defp->dnitems > 0)
    {
      /* Yes, output the defaults differently if there is only one */

      if (defp->dnitems == 1)
        {
          /* Output the Default */

          item = &defp->ditem[0];
          body("  <li>\n");
          body("    <i>Default</i>: %s\n", item->ddefault);

          /* Output the dependency */

          if (item->ddependency)
            {
              body("    <p>\n");
              body("      <i>Dependency:</i>\n");
              body("      %s\n", htmlize_expression(item->ddependency));
              body("    </p>\n");
            }

          body("  </li>\n");
        }
      else
        {
          /* Output a sub-list of defaults. */

          body("  <li>\n");
          body("    <i>Default Values</i>:\n");
          body("    <ul>\n");

          for (i = 0; i < defp->dnitems; i++)
            {
              /* Output the Default */

              item = &defp->ditem[i];
              body("      <li>\n");
              body("        <i>Default</i>: %s\n", item->ddefault);

              /* Output the dependency */

              if (item->ddependency)
                {
                  body("        <p>\n");
                  body("          <i>Dependency:</i>\n");
                  body("          %s\n", htmlize_expression(item->ddependency));
                  body("        </p>\n");
                }
            }

          body("    </ul>\n");
          body("  </li>\n");
        }
    }
}

/****************************************************************************
 * Name: free_default
 *
 * Description:
 *   Output and the list of defaults to the the HTML body file.
 *
 ****************************************************************************/

static void free_default(struct default_s *defp)
{
  struct default_item_s *item;
  int i;

  /* Free strings for each default */

  for (i = 0; i < defp->dnitems; i++)
    {
      /* Free the default value string */

      item = &defp->ditem[i];
      free(item->ddefault);

      /* Free any dependency on the default */

      if (item->ddependency)
        {
          free(item->ddependency);
        }
    }
}

/****************************************************************************
 * Name: process_config
 *
 * Description:
 *   Process one configuration variable paragraph
 *
 ****************************************************************************/

static inline char *process_config(FILE *stream, const char *configname,
                                   const char *kconfigdir)
{
  enum token_type_e tokid;
  struct config_s config;
  bool help;
  const char *paranum;
  char *token;
  char *ptr;
  int i;

  /* Get the configuration information */

  memset(&config, 0, sizeof(struct config_s));
  config.cname = strdup(configname);

  /* Process each line in the configuration */

  help = false;

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
                {
                  /* Save the type of the configuration variable */

                  config.ctype = VALUE_BOOL;

                  /* Get the description following the type */

                  ptr = get_html_string();
                  if (ptr)
                    {
                      config.cdesc = strdup(ptr);
                    }

                  /* Indicate that the line has been consumed */

                  token = NULL;
                }
                break;

              case TOKEN_INT:
                {
                  /* Save the type of the configuration variable */

                  config.ctype = VALUE_INT;

                  /* Get the description following the type */

                  ptr = get_html_string();
                  if (ptr)
                    {
                      config.cdesc = strdup(ptr);
                    }

                  /* Indicate that the line has been consumed */

                  token = NULL;
                }
                break;

              case TOKEN_HEX:
                {
                  /* Save the type of the configuration variable */

                  config.ctype = VALUE_HEX;

                  /* Get the description following the type */

                  ptr = get_html_string();
                  if (ptr)
                    {
                      config.cdesc = strdup(ptr);
                    }

                  /* Indicate that the line has been consumed */

                  token = NULL;
                }
                break;

              case TOKEN_STRING:
                {
                  /* Save the type of the configuration variable */

                  config.ctype = VALUE_STRING;

                  /* Get the description following the type */

                  ptr = get_html_string();
                  if (ptr)
                    {
                      config.cdesc = strdup(ptr);
                    }

                  /* Indicate that the line has been consumed */

                  token = NULL;
                }
                break;

              case TOKEN_DEFAULT:
                {
                  process_default(stream, &config.cdefault);
                  token = NULL;
                }
                break;

              case TOKEN_RANGE:
                {
                  char *value = get_token();
                  if (value)
                    {
                      config.clower = strdup(value);

                      value = get_token();
                      if (value)
                        {
                          config.cupper = strdup(value);
                        }
                    }

                  token = NULL;
                }
                break;

              case TOKEN_SELECT:
                {
                  char *value;
                  int ndx;

                  ndx = config.cselect.snvar;
                  if (ndx >= MAX_SELECT)
                    {
                      error("Too many 'select' lines\n");
                      exit(ERROR_TOO_MANY_SELECT);
                    }

                  value = get_token();
                  config.cselect.svarname[ndx] = strdup(value);
                  config.cselect.snvar = ndx + 1;
                  token = NULL;
                }
                break;

              case TOKEN_DEPENDS:
                {
                  char *value = get_token();
                  if (strcmp(value, "on") != 0)
                    {
                      error("Expected \"on\" after \"depends\"\n");
                      exit(ERRROR_ON_AFTER_DEPENDS);
                    }

                  push_dependency(htmlize_expression(g_lnptr));
                  config.cndependencies++;
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
                        config.cname, token);
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

  /* Is this an internal configuration varaible with no description?
   * Were we asked to show these internal variables?  If not then
   * don't output anything.
   */

  if (config.cdesc || g_internal)
    {
      /* Print the configuration variable name and the short description */

      body("<h3><a name=\"CONFIG_%s\">", config.cname);

      /* If we are not in a choice block, than give the variable a paragraph
       * number and put it in the table of contents.
       */

      if (!g_inchoice)
        {
          paranum = get_paranum();
          output("<li><a href=\"#CONFIG_%s\">%s <code>CONFIG_%s</code>",
                 config.cname, paranum, config.cname);
          body("%s ", paranum);
          incr_paranum();
        }

      body("<code>CONFIG_%s</code>", config.cname);

      if (config.cdesc)
        {
          if (!g_inchoice)
            {
              output(": %s", config.cdesc);
            }

          body(": %s", config.cdesc);
        }

      body("</a></h3>\n");
      if (!g_inchoice)
        {
          output("</a></li>\n");
        }

      /* Configuration description is indented */

      body("<ul>\n");

      /* Print the type of the configuration variable */

      if (config.ctype != VALUE_NONE)
        {
          body("  <li><i>Type</i>: %s</li>\n", type2str(config.ctype));
        }

      /* Print the default values of the configuration variable */

      print_default(&config.cdefault);

      /* Print the range of values of the configuration variable */

      if (config.clower || config.cupper)
        {
          body("  <li><i>Range</i>:\n");
          if (config.clower)
            {
              body(" %s", config.clower);
            }

          body(" -", config.clower);

          if (config.cupper)
            {
              body(" %s", config.cupper);
            }

          body("</li>\n");
        }

      /* Print the default value of the configuration variable auto-selected by this setting */

      if (config.cselect.snvar > 0)
        {
          body("  <li><i>Selects</i>: <a href=\"#CONFIG_%s\">CONFIG_%s</a>",
               config.cselect.svarname[0], config.cselect.svarname[0]);

          for (i = 1; i < config.cselect.snvar; i++)
            {
              body(", <a href=\"#CONFIG_%s\">CONFIG_%s</a>",
                   config.cselect.svarname[i], config.cselect.svarname[i]);
            }

          body("</li>\n");
        }

      /* Print the list of dependencies (if any) */

      if (g_ndependencies > 0)
        {
          body("  <li><i>Dependencies</i>: %s", g_dependencies[0]);

          for (i = 1; i < g_ndependencies; i++)
            {
              body(", %s\n", g_dependencies[i]);
            }

          body("</li>\n");
        }

      /* Show the configuration file */

      body("  <li><i>Kconfig file</i>: <code>%s/Kconfig</code>\n", kconfigdir);

      /* Print any help text */

      if (help)
        {
          process_help(stream);
          token = NULL;
        }
      else if (!config.cdesc)
        {
          body("<p>This is a hidden, internal configuration variable that cannot be explicitly set by the user.</p>\n");
        }

      /* End of configuration description */

      body("</ul>\n");
  }

  /* Remove any dependencies that apply only to this configuration */

  for (i = 0; i < config.cndependencies; i++)
    {
      pop_dependency();
    }

  /* Free allocated memory */

  free_default(&config.cdefault);

  if (config.cname)
    {
      free(config.cname);
    }

  if (config.cdesc)
    {
      free(config.cdesc);
    }

  if (config.clower)
    {
      free(config.clower);
    }

  if (config.cupper)
    {
      free(config.cupper);
    }

  if (config.cselect.snvar > 0)
    {
      for (i = 0; i < config.cselect.snvar; i++)
        {
          free(config.cselect.svarname[i]);
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

static char *parse_kconfigfile(FILE *stream, const char *kconfigdir); /* Forward reference */
static inline char *process_choice(FILE *stream, const char *kconfigdir)
{
  enum token_type_e tokid;
  struct choice_s choice;
  const char *paranum;
  char *token = NULL;
  char *ptr;
  bool help;
  int i;

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
                      choice.cprompt = strdup(ptr);
                    }

                  /* Indicate that the line has been consumed */

                  token = NULL;
                }
                break;

              case TOKEN_DEFAULT:
                {
                  process_default(stream, &choice.cdefault);
                  token = NULL;
                }
                break;

              case TOKEN_DEPENDS:
                {
                  char *value = get_token();
                  if (strcmp(value, "on") != 0)
                    {
                      error("Expected \"on\" after \"depends\"\n");
                      exit(ERRROR_ON_AFTER_DEPENDS);
                    }

                  push_dependency(htmlize_expression(g_lnptr));
                  choice.cndependencies++;
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

  if (choice.cprompt)
    {
      output(": %s", choice.cprompt);
      body(": %s", choice.cprompt);
    }

  output("</a></li>\n");
  body("</a></h3>\n");
  g_choice_number++;

  /* Print the default values of the configuration variable */

  body("<ul>\n");
  print_default(&choice.cdefault);

  /* Print the list of dependencies (if any) */

  if (g_ndependencies > 0)
    {
      body("  <li><i>Dependencies</i>: %s", g_dependencies[0]);
      for (i = 1; i < g_ndependencies; i++)
        {
          body(", %s\n", g_dependencies[i]);
        }
      body("</li>\n");
    }

   /* Show the configuration file */

   body("  <li><i>Kconfig file</i>: <code>%s/Kconfig</code>\n</li>", kconfigdir);

   /* Print any help text */

  if (help)
    {
      process_help(stream);
      token = NULL;
    }

   body("</ul>\n");

   /* Then show the choice options */

   body("<p><b>Choice Options:</b></p>", kconfigdir);
   body("<ul>\n");

  /* Remove any dependencies that apply only to this configuration */

  for (i = 0; i < choice.cndependencies; i++)
    {
      pop_dependency();
    }

  /* Free allocated memory */

  free_default(&choice.cdefault);

  if (choice.cprompt)
    {
      free(choice.cprompt);
    }

  /* Increment the nesting level */

   incr_level();

   debug("process_choice: TOKEN_CHOICE\n");
   debug("  kconfigdir:  %s\n", kconfigdir);
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

static inline char *process_menu(FILE *stream, const char *kconfigdir)
{
  enum token_type_e tokid;
  struct menu_s menu;
  const char *paranum;
  char *menuname;
  char *token = NULL;
  char *ptr;
  int i;

  /* Get the menu information */

  memset(&menu, 0, sizeof(struct menu_s));

  /* Get the menu name */

  menuname = get_html_string();
  menu.mname = strdup(menuname);

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
              case TOKEN_DEPENDS:
                {
                  char *value = get_token();
                  if (strcmp(value, "on") != 0)
                    {
                      error("Expected \"on\" after \"depends\"\n");
                      exit(ERRROR_ON_AFTER_DEPENDS);
                    }

                  push_dependency(htmlize_expression(g_lnptr));
                  menu.mndependencies++;
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
  if (menu.mname)
    {
      output("<li><a href=\"#menu_%d\">%s Menu: %s</a></li>\n",
             g_menu_number, paranum, menu.mname);
      output("<ul>\n");
             body("\n<h1><a name=\"menu_%d\">%s Menu: %s</a></h1>\n",
             g_menu_number, paranum, menu.mname);
    }
  else
    {
      output("<li><a href=\"#menu_%d\">%s Menu</a></li>\n",
             g_menu_number, paranum);
      body("\n<h1><a name=\"menu_%d\">%s Menu</a></h1>\n",
             g_menu_number, paranum);
    }

  g_menu_number++;

  /* Print the list of dependencies (if any) */

  body("<ul>\n");
  if (g_ndependencies > 0)
    {
      body("  <li><i>Dependencies</i>: %s", g_dependencies[0]);

      for (i = 1; i < g_ndependencies; i++)
        {
          body(", %s\n", g_dependencies[i]);
        }

      body("</li>\n");
    }

  /* Show the configuration file */

  body("  <li><i>Kconfig file</i>: <code>%s/Kconfig</code>\n", kconfigdir);
  body("</ul>\n");

  /* Remove any dependencies that apply only to this configuration */

  for (i = 0; i < menu.mndependencies; i++)
    {
      pop_dependency();
    }

  /* Free any allocated memory */

  if (menu.mname)
    {
      free(menu.mname);
    }

  /* Increment the nesting level */

  incr_level();

  debug("process_menu: TOKEN_MENU\n");
  debug("  kconfigdir:  %s\n", kconfigdir);
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

static void process_kconfigfile(const char *kconfigdir); /* Forward reference */
static char *parse_kconfigfile(FILE *stream, const char *kconfigdir)
{
  enum token_type_e tokid;
  char *token = NULL;
  char *ptr;

  /* Process each line in the Kconfig file */

  while ((ptr = kconfig_line(stream)) != NULL)
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

                  char *relpath = get_token();

                  /* Remove optional quoting */

                  relpath = dequote(relpath);
                  if (relpath)
                    {
                      char *subdir = dirname(relpath);
                      char *dirpath;

                      /* Check if the directory path contains $APPSDIR */

                      char *appsdir = strstr(subdir, "$APPSDIR");
                      if (appsdir)
                        {
                          char *tmp = appsdir + strlen("$APPSDIR");

                          *appsdir = '\0';
                          asprintf(&dirpath, "%s/%s%s%s", g_kconfigroot, subdir, g_appsdir, tmp);
                        }
                      else
                        {
                          asprintf(&dirpath, "%s/%s", g_kconfigroot, subdir);
                        }

                      debug("parse_kconfigfile: Recursing for TOKEN_SOURCE\n");
                      debug("  relpath:     %s\n", relpath);
                      debug("  subdir:      %s\n", subdir);
                      debug("  dirpath:     %s\n", dirpath);

                      /* Then recurse */

                      process_kconfigfile(dirpath);
                      token = NULL;
                      free(dirpath);
                    }

                  /* Set the token string to NULL to indicate that we need to read the next line */

                  token = NULL;
                }
                break;

              case TOKEN_CONFIG:
              case TOKEN_MENUCONFIG:
                {
                  char *configname = get_token();
                  token = process_config(stream, configname, kconfigdir);
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
                  token = process_menu(stream, kconfigdir);
                }
                break;

              case TOKEN_CHOICE:
                {
                  token = process_choice(stream, kconfigdir);
                }
                break;

              case TOKEN_ENDCHOICE:
                {
                  /* Reduce body indentation level */

                  body("</ul>\n");
                  g_inchoice--;

                  /* Decrement the nesting level */

                  decr_level();
                  incr_paranum();
                  token = NULL;
                }
                break;

              case TOKEN_ENDMENU:
                {
                  /* Reduce table of contents indentation level */

                  output("</ul>\n");

                  /* Decrement the nesting level */

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
                  /* Set token to NULL to skip to the next line */

                  error("Unhandled token: %s\n", token);
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

static void process_kconfigfile(const char *kconfigdir)
{
  FILE *stream;
  char *kconfigpath;

  /* Create the full path to the Kconfig file */

  asprintf(&kconfigpath, "%s/Kconfig", kconfigdir);
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

  parse_kconfigfile(stream, kconfigdir);

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
  g_internal    = false;
  g_kconfigroot = ".";
  g_appsdir     = "../apps";
  g_outfile     = stdout;
  outfile       = NULL;

  while ((ch = getopt(argc, argv, ":dha:o:")) > 0)
    {
      switch (ch)
        {
          case 'a' :
            g_appsdir = optarg;
            break;

          case 'o' :
            outfile = optarg;
            break;

          case 'i' :
            g_internal = true;
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

           break;
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

  /* Open the output file (if any) */

  if (outfile)
    {
      g_outfile = fopen(outfile, "w");
      if (!g_outfile)
        {
          error("open %s failed: %s\n", outfile, strerror(errno));
          exit(ERROR_OUTFILE_OPEN_FAILURE);
        }
    }

  /* Open the temporary file */

  g_tmpfile = fopen(TMPFILE_NAME, "w");
  if (!g_tmpfile)
    {
      error("open %s failed: %s\n", TMPFILE_NAME, strerror(errno));
      exit(ERROR_TMPFILE_OPEN_FAILURE);
    }

  /* Get the current date string in the scratch buffer */

  now = time(NULL);
  ptm = localtime(&now);
  (void)strftime(g_scratch, SCRATCH_SIZE, "%B %d, %Y", ptm);

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
  output("<center><h1>Table of contents</h1></center>\n");
  output("<ul>\n");

  incr_level();
  paranum = get_paranum();
  output("<li><a href=\"#menu_%d\">%s Menu: Main</a></li>\n",
         g_menu_number, paranum);
  body("<h1><a name=\"menu_%d\">%s Menu: Main</a></h1>\n",
       g_menu_number, paranum);
  g_menu_number++;

  /* Tell the reader that this is an auto-generated file */

  body("<p>\n");
  body("  <b>Overview</b>.\n");
  body("  The NuttX RTOS is highly configurable.\n");
  body("  The NuttX configuration files are maintained using the <a href=\"http://ymorin.is-a-geek.org/projects/kconfig-frontends\">kconfig-frontends</a> tool.\n");
  body("  That configuration tool uses <code>Kconfig</code> files that can be found through the NuttX source tree.\n");
  body("  Each <code>Kconfig</code> files contains declarations of configuration variables.\n");
  body("  Each configuration variable provides one configuration option for the NuttX RTOS.\n");
  body("  This configurable options are descrived in this document.\n");
  body("</p>\n");
  body("<p>\n");
  body("  <b>Mainenance Note</b>.\n");
  body("  This documenation was auto-generated using the <a href=\"http://sourceforge.net/p/nuttx/git/ci/master/tree/nuttx/tools/kconfig2html.c\">kconfig2html</a> tool\n");
  body("  That tools analyzes the NuttX <code>Kconfig</code> and generates this HTML document.\n");
  body("  This HTML document file should not be editted manually.\n");
  body("  In order to make changes to this document, you should instead modify the <code>Kconfig</code> file(s) that were used to generated this document and then execute the <code>kconfig2html</code> again to regenerate the HTML document file.\n");
  body("</p>\n");

  /* Process the Kconfig files through recursive descent */

  process_kconfigfile(g_kconfigroot);

  /* Terminate the table of contents */

  output("</ul>\n");

  /* Close the temporary file and copy it to the output file */

  fclose(g_tmpfile);
  g_tmpfile = fopen(TMPFILE_NAME, "r");
  if (!g_tmpfile)
    {
      error("open %s failed: %s\n", TMPFILE_NAME, strerror(errno));
      exit(ERROR_TMPFILE_OPEN_FAILURE);
    }

   while ((ch = getc(g_tmpfile)) != EOF)
     {
       (void)putc(ch, g_outfile);
     }

  /* Close and remove the temporary file again */

  fclose(g_tmpfile);
  unlink(TMPFILE_NAME);

  /* Output trailer boilerplater */

  output("</body>\n");
  output("</html>\n");

  /* Close the output file (if any) and the temporary file*/

  if (outfile)
    {
      fclose(g_outfile);
    }

  return 0;
}
