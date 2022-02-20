.. include:: /substitutions.rst
.. _coding-standard:

=================
C Coding Standard
=================

NuttX follows a specific coding style which needs to be followed at all times
a contribution to be accepted. Please read this document before working on
new code so that you can follow the style from the start. To check your code
for conformance to the coding style, you should use the `nxstyle <#nxstyle>`_
tool included under ``tools/`` in the main NuttX repository.

*******************
General Conventions
*******************

File Organization
=================

**File Extensions** Use the ``.h`` extension for C header files
and ``.c`` for C source files.

**File header**. Every C, C++, make file, or script begins with a file header.
That file header is enclosed with a *block comment* (see below). Within the
block comment, the following must appear:

  -  The relative path to the file from the top-level directory.
  -  An optional, one-line description of the file contents.
  -  A blank line
  -  A copyright notice indented two additional spaces
  -  A line identifying the author and contact information with the
     same indentation as the copyright notice.
  -  A blank line
  -  NuttX standard Apache 2.0 licensing information as provided in
     the `appendix <#appndxa>`__.

**Sample File Headers**. Sample file headers are provided in an
`Appendix <#appndxa>`__ to this document. No new software may be
included in the NuttX source tree that does not have licensing
information included in the file. No new software may be included
in the NuttX source tree that does not have a Apache 2.0 license
or license (or, in the case of 3rd party file, a compatible
license such as the BSD or MIT licenses). If the file does not
follow Apache 2.0 licensing, then the appropriate license
information should be provided in the header rather than the
Apache 2.0 licensing information and a NOTE should be included in
the top-level ``LICENSE`` and/or ``NOTICE`` file(s), as appropriate,
to indicate any variations from Apache 2.0 licensing.

**Grouping**. All like components in a C source or header file are
grouped together. Definitions do not appear arbitrarily through
the file, rather, like definitions are grouped together and
preceded by a *block comment* identifying the grouping.

**Block Comments**. Each grouping in the file is separated with a
*block comment*. The block comment consists of:

-  A line that consists of the opening C comment (``/*``) followed
   by a series of asterisks extending to the length of the line
   (usually to column 78).
-  The name of the grouping, starting at column 4. An asterisk
   preceives the name of the grouping in column 1.
-  A line that consists of the closing C comment (``*/``) at the
   end of the line (usually column 78) preceded by a series of
   asterisks extending to column 1.

**Examples of Block Comments**. See `Appendix A <#appndxa>`__ for
examples of block comments.

**Order of Groupings**. The following groupings should appear in
all C source files in the following order:

  #. Included Files
  #. Pre-processor Definitions
  #. Private Types (definitions)
  #. Private Function Prototypes (declarations)
  #. Private Data (definitions)
  #. Public Data (definitions)
  #. Private Functions (definitions)
  #. Public Functions (definitions)

The following groupings should appear in all C header files in the
following order:

  #. Included Files
  #. Pre-processor Definitions
  #. Public Types (definitions)
  #. Public Data (declarations)
  #. Inline Functions (definitions)
  #. Public Function Prototypes (declarations)

**Large vs. Small Files**. In larger files, block comments should
be included for all groupings, even if they are empty; the empty
grouping provides important information in itself. Smaller files
may omit some of the block comments; it is awkard if the block
comments are larger than the file content!

**Header File Idempotence**. C header file must protect against
multiple inclusion through the use of macros that "guard" against
multiple definitions if the header file is included multiple
times.

-  Each header file must contain the following pre-processor
   conditional logic near the beginning of the header file:
   Between the file header and the "Included Files" block comment.
   For example::

    #ifndef __INCLUDE_NUTTX_ARCH_H
    #define __INCLUDE_NUTTX_ARCH_H

   Notice that the definitions within of the header do not follow
   the usually rules: The presence of the conditional test at the
   top of the file does not cause the remaining definitions within
   the file to be indented.

-  Then conditional compilation is closed at the fine line of the
   header file with::

    #endif /* __INCLUDE_NUTTX_ARCH_H */

**Forming Guard Names**. Then pre-processor macro name used in the
guard is formed from the full, relative path to the header for
from the top-level, controlled directory. That path is preceded by
``__`` and ``_`` replaces each character that would otherwise be
invalid in a macro name. So, for example, ``__INCLUDE_NUTTX_ARCH_H``
corresponds to the header file ``include/nuttx/arch.h``

**Deoxygen Information**. NuttX does not use Deoxygen for
documentation and no file should contain Doxygen tags or Doxygen
style comments.

**Sample File Formats**. C source and header file templates are
provided in an `Appendix <#appndxa>`__ to this document.

Lines
=====

**Line Endings**. Files should be formatted with the newline
character (``\n``) as the line ending (Unix-style line endings)
and specifically *not* the carriage return, newline sequence
(``\r\n``) used with Windows-style line endings. There should be
no extra whitespace at the end of the line. In addition, all text
files should end in a single newline (``\n``). This avoids the
*"No newline at end of file"* warning generated by certain tools.

**Line Width**. Text should not extend past column 78 in the
typical C source or header file. Sometimes the nature of the
content of a file may require that the lines exceed this limit.
This often occurs in header files with naturally long definitions.
If the line width must extend 78 lines, then some wider line width
may be used in the file provided that it is used consistently.

**Line Wrapping**.

.. error:: This is incorrect

  .. code-block:: c

    struct some_long_struct_name_s
    {
      struct some_long_struct_name_s *flink;  /* The forward link to the next instance of struct some_long_struct_name_s in a singly linked list */
      int short_name1;   /* Short comment 1 */
      int short_name2;   /* This is a very long comment describing subtle aspects of the short_name2 field */
    };

    struct some_medium_name_s *ptr = (struct some_medium_name_s *)malloc(sizeof(some_medium_name_s);

    struct some_long_struct_name_s *ptr = (struct some_long_struct_name_s *)malloc(sizeof(some_long_struct_name_s);

    ret = some_function_with_many parameters(long_parameter_name_1, long_parameter_name_2, long_parameter_name_3, long_parameter_name_4, long_parameter_name_5, long_parameter_name_6, long_parameter_name_7, long_parameter_name_8);

    ret = some_function_with_many parameters(long_parameter_name_1,
      long_parameter_name_2,
      long_parameter_name_3
      long_parameter_name_4,
      long_parameter_name_5,
      long_parameter_name_6,
      long_parameter_name_7,
      long_parameter_name_8);

.. hint:: This is correct

  .. code-block:: c

    struct some_long_struct_name_s
    {
      /* The forward link to the next instance of struct
       * some_long_struct_name_s in a singly linked list.
       */

      struct some_long_struct_name_s *flink;
      int short_name1;   /* Short comment 1. */
      int short_name2;   /* This is a very long comment describing subtle
                          * aspects of the short_name2 field. */
    };

    FAR struct some_medium_name_s *ptr = (FAR struct some_medium_name_s *)
      malloc(sizeof(some_medium_name_s);

    FAR struct some_medium_name_s *ptr =
      (FAR struct some_medium_name_s *)malloc(sizeof(some_medium_name_s);

    FAR struct some_long_struct_name_s *ptr =
      (FAR struct some_long_struct_name_s *)
        malloc(sizeof(some_long_struct_name_s);

    ret = some_function_with_many parameters(long_parameter_name_1,
                                             long_parameter_name_2,
                                             long_parameter_name_3,
                                             long_parameter_name_4,
                                             long_parameter_name_5,
                                             long_parameter_name_6,
                                             long_parameter_name_7,
                                             long_parameter_name_8);

**NOTE**: See the discussion of `pointers <#farnear>`__ for
information about the ``FAR`` qualifier used above.

**Double Spacing**. A single blank line may be use to separate
logical groupings as the designer feels fit. Single blank lines
are also required in certain contexts as defined in this standard.
Additional blanks lines (two or more) are prohibited.

**Columnar Organization**. Similar things should be aligned on the
same column unless doing so would cause the line width to be
exceeded.

.. note:: This is acceptable

  .. code-block:: c

    dog = cat;
    monkey = oxen;
    aardvark = macaque;

.. hint:: This is preferred

  .. code-block:: c

    dog      = cat;
    monkey   = oxen;
    aardvark = macaque;

**Block Comments** The final asterisk (``*``) should occur at
column 78 (or the line width of files with longer lines). Note
that the final comment delimiter of the block comment is an
exception an lies at column 79.

Comments
========

**General**. Within a comment, the text must be standard English
conforming to standard English rules of grammar and spelling (US
English spelling). Of course, this is not the place to summarize
all English grammar, but as examples of common grammatic issues in
comments:

-  All sentences should begin with an upper-case character and end
   with either '.', '?', or '!'.
-  Sentence fragments and phrases are generally treated the same
   as sentences.
-  The punctuation '.' and ':' is followed by two spaces; the
   punctuation ',' and ';' is followed by a single space.
-  Text following '.' or ':' begins with an upper-case character;
   text following ',' or ';' begins with a lower-case character.

**Line Spacing** A single blank line should precede and follow
each comment. The only exceptions are:

For the file header block comment that begins on line one; there
is no preceding blank line in that case.
For conditional compilation. Conditional compilation should
include the conditional logic *and* all comments associated with
the conditional logic. In this case, the blank line appears
*before* the conditional, not after it. No blank lines precede any
comments following the conditional.
With braces. No blank line separates the line containing the
opening left brace from a comment. No blank line follows a comment
that may be the final line preceding a closing right brace.
With Labels. No blank line separates the line containing the label
from a comment.

.. error:: This is incorrect

  .. code-block:: c

      /* set a equal to b */
      a = b;
      /* set b equal to c */
      b = c;

      /* Do the impossible */

    #ifdef CONFIG_THE_IMPOSSIBLE
      the_impossible();
    #endif

      if (a == b)
        {

          /* Only a comment */

        }

      here:

      /* This is the place */

.. tip:: This is correct

  .. code-block:: c

      /* Set a equal to b. */

      a = b;

      /* Set b equal to c. */

      b = c;

    #ifdef CONFIG_THE_IMPOSSIBLE
      /* Do the impossible */

      the_impossible();
    #endif

      if (a == b)
        {
          /* Only a comment */
        }

      here:
        /* This is the place */

**Indentation** Comments should, typically, be placed before the
code section to which they apply. The comment indentation should
be the same as the follow indentation rules as the following code
(if applicable).

**Short, Single line comments**. Short comments must lie on a
single line. The comment delimiters must lie on the same line.

.. error:: This is incorrect

  .. code-block:: c

    /*
     * This is a single line comment
     */

.. tip:: This is correct

  .. code-block:: c

    /* This is a single line comment. */

**Multi-line comments**. If the comment is too long to fit on a
single, it must be broken into a multi-line comment. The comment
must be begin on the first line of the multi-line comment with the
opening comment delimiter (``/*``). The following lines of the
multi-line comment must be with an asterisk (``*``) aligned in the
same column as the asterisk in the preceding line. The closing
comment delimiter must lie on a separate line with the asterisk
(``*``) aligned in the same column as the asterisk in the
preceding line.

.. error:: This is incorrect

  .. code-block:: c

    /*
       This is the first line of a multi-line comment.
       This is the second line of a multi-line comment.
       This is the third line of a multi-line comment. */

    /* This is the first line of another multi-line comment.  */
    /* This is the second line of another multi-line comment. */
    /* This is the third line of another multi-line comment.  */

.. tip:: This is correct

  .. code-block:: c

    /* This is the first line of a multi-line comment.
     * This is the second line of a multi-line comment.
     * This is the third line of a multi-line comment.
     */

**Comments to the Right of Statements**. Comments to the right of
statements in C source files are discouraged. If such comments are
used, they should be (1) very short so that they do not exceed the
line width (typically 78 characters), (2) aligned so that the
comment begins in the same column on each line.

.. error:: This is incorrect

  .. code-block:: c

    dog = cat; /* Make the dog be a cat */
    monkey = oxen; /* Make the monkey be an oxen */
    aardvark = macaque; /* Make the aardvark be a macaque */

.. note:: This is acceptable

  .. code-block:: c

    dog      = cat;     /* Make the dog be a cat. */
    monkey   = oxen;    /* Make the monkey be an oxen. */
    aardvark = macaque; /* Make the aardvark be a macaque. */

.. tip:: This is preferred

  .. code-block:: c

    /* Make the dog be a cat. */

    dog      = cat;

    /* Make the monkey be an oxen. */

    monkey   = oxen;

    /* Make the aardvark be a macaque. */

    aardvark = macaque;

**Comments to the Right of Data Definitions**. Comments to the
right of a declaration with an enumeration or structure, on the
other hand, are encouraged, provided that the comments are short
and do not exceed the maximum line width (usually 78 characters).
Columnar alignment of comments is very desirable (but often cannot
be achieved without violating the line width).

.. error:: This is incorrect

  .. code-block:: c

    struct animals_s
    {
      int dog; /* This is a dog */
      int cat; /* This is a cat */
      double monkey; /* This is a monkey */
      double oxen; /* This is an oxen */
      bool aardvark; /* This is an aardvark */
      bool macaque; /* This is a macaque */
    };

.. note:: This is acceptable

  .. code-block:: c

    struct animals_s
    {
      int dog;       /* This is a dog. */
      int cat;       /* This is a cat. */
      double monkey; /* This is a monkey. */
      double oxen;   /* This is an oxen. */
      bool aardvark; /* This is an aardvark. */
      bool macaque;  /* This is a macaque. */
    };

.. tip:: This is preferred

  .. code-block:: c

    struct animals_s
    {
      int    dog;      /* This is a dog. */
      int    cat;      /* This is a cat. */
      double monkey;   /* This is a monkey. */
      double oxen;     /* This is an oxen. */
      bool   aardvark; /* This is an aardvark. */
      bool   macaque;  /* This is a macaque. */
    };

**Long Comments on the Right**. Comments on the right of
statements or data definitions must be short and fit on the same
line without exceeding the maximum line length. If a longer
comment is needed, then it should appear above the statement of
definition rather than to the right of the definition.

**Breaking Long Comments to the Right of Statements** Breaking
long comments to the right of statements is acceptable as well,
but not encouraged. In this case the comment must be begin on the
first line of the multi-line, right-hand comment with the opening
comment delimiter (/*). The following lines of the multi-line,
right hand comment must be with an asterisk (*) aligned in the
same column as the asterisk in the preceding line. The closing
comment delimiter must lie on the *same* line with the asterisk.

.. error:: This is incorrect

  .. code-block:: c

    dog = cat; /* This assignment will convert what was at one time a lowly dog into a ferocious feline. */

.. note:: This is acceptable

  .. code-block:: c

    dog = cat;       /* This assignment will convert what was at one time a
                      * lowly dog into a ferocious feline. */

.. tip:: This is preferred

  .. code-block:: c

    /* This assignment will convert what was at one time a lowly dog into a
     * ferocious feline.
     */

    dog = cat;

**Note** that if the comment is continued on multiple lines, the
comment alignment and multi-line comment rules still apply with
one exception: The closing ``*/`` appears on the same line as the
final text of the comment. This exception to the rule is enforced
to keep the statements and definitions from becoming to spread
out.

**Block comments**. Block comments are only used to delimit
groupings with the overall `file
organization <#fileorganization>`__ and should not be used unless
the usage is consistent with delimiting logical groupings in the
program.

**C Style Comments**. C99/C11/C++ style comments (beginning with
``//``) should not be used with NuttX. NuttX generally follows C89
and all code outside of architecture specific directories must be
compatible with C89.

.. error:: This is incorrect

  .. code-block:: c

    // This is a structure of animals
    struct animals_s
    {
      int    dog;      // This is a dog
      int    cat;      // This is a cat
      double monkey;   // This is a monkey
      double oxen;     // This is an oxen
      bool   aardvark; // This is an aardvark
      bool   macaque;  // This is a macaque
    };

.. tip:: This is correct

  .. code-block:: c

    /* This is a structure of animals. */

    struct animals_s
    {
      int    dog;      /* This is a dog. */
      int    cat;      /* This is a cat. */
      double monkey;   /* This is a monkey. */
      double oxen;     /* This is an oxen. */
      bool   aardvark; /* This is an aardvark. */
      bool   macaque;  /* This is a macaque. */
    };

**"Commenting Out" Large Code Blocks**. Do not use C or C++ comments to
disable compilation of large blocks of code. Instead, use ``#if 0`` to
do that. Make sure there is a comment before the ``#if 0`` to explain
why the code is not compiled.

.. error:: This is incorrect

  .. code-block:: c

    void some_function(void)
    {
      ... compiled code ...

      /*
      ... disabled code ..
       */

      ... compiled code ...
    }

    void some_function(void)
    {
      ... compiled code ...

      //
      // ... disabled code ..
      //

      ... compiled code ...
    }

.. tip:: This is correct

  .. code-block:: c

    void some_function(void)
    {
      ... compiled code ...

      /* The following code is disabled because it is no longer needed. */

    #if 0
      ... disabled code ..
    #endif

      ... compiled code ...
    }

Braces
======

In general, the use of braces in the NuttX coding standard is similar to
the use of braces in the `GNU Coding
standards <https://www.gnu.org/prep/standards/standards.pdf>`__ with a
few subtle differences.

**Coding Standard:**

-  **Always on Separate Lines**. Braces always appear on a separate line
   containing nothing else other than white space.
-  **Never Comments on Braces**. Do not put comments on the same line as
   braces.
-  **Compound Statements**. Within this document, an opening left brace
   followed by a sequence of statements, and ending with a closing right
   brace is referred to as a *compound statement*.
-  **Nested Compound Statements**. In the case where there are nested
   compound statements that end with several consecutive right braces,
   each closing right brace must lie on a separate line and must be
   indented to match the corresponding opening brace.
-  **Final brace followed by a single blank line**. The *final* right
   brace must be followed by a blank line as per standard rules. There
   are two exceptions to this rule:

   #. In the case where there are nested several consecutive right
      braces, no blank lines should be inserted except for after the
      *final* right brace.
   #. No blank should be used to separate the final, closing right brace
      when it is followed by a ``break;`` statement.

-  **Special Indentation Rules**. Special `indentation
   rules <#indentation>`__ apply to braces.

.. error:: This is incorrect

  .. code-block:: c

    while (true)
      {
        if (valid)
          {
          ...
          } /* if valid */
        else
          {
          ...
          } /* not valid */
      } /* end forever */
    if (a < b) {
      if (a < 0) {
          c = -a;
      } else {
          c = a;
      }
    } else {
      if (b < 0) {
          c = -b;
      } else {
          c = b;
      }
    }

.. tip:: This is correct

  .. code-block:: c

    while (true)
      {
        if (valid)
          {
          ...
          }
        else
          {
          ...
          }
      }

    if (a < b)
      {
        if (a < 0)
          {
            c = -a;
          }
        else
          {
            c = a;
          }
      }
    else
      {
        if (b < 0)
          {
            c = -b;
          }
        else
          {
            c = b;
          }
      }

**Exception to Indentation Rule for Braces**. The exception is braces
that following structure, enumeration, union, and function declarations.
There is no additional indentation for those braces; those braces align
with the beginning of the definition

.. error:: This is incorrect

  .. code-block:: c

    enum kinds_of_dogs_e
      {
      ...
      };

    struct dogs_s {
      ...
      union {
      ...
      } u;
      ...
    };

    struct cats_s
      {
      ...
        union
         {
         ...
         } u;
      ...
      };

    int animals(int animal)
      {
      ...
      }

.. tip:: This is correct

  .. code-block:: c

    enum kinds_of_dogs_e
    {
      ...
    };

    struct dogs_s
    {
      ...
      union
      {
      ...
      } u;
      ...
    };

    struct cats_s
    {
      ...
      union
      {
      ...
      } u;
      ...
    };

    int animals(int animal)
    {
      ...
    }

Indentation
===========

In general, the indentation in the NuttX coding standard is similar to
the indentation requirements of the `GNU Coding
standards <https://www.gnu.org/prep/standards/standards.pdf>`__ with a
few subtle differences.

**Indentation Unit**. Indentation is in units of two spaces; Each
indentation level is twos spaces further to the right than the preceding
indentation levels. TAB characters may not be used for indentation.

.. error:: This is incorrect

  .. code-block:: c

    if (x == y) {
	    dosomething(x);
    }

      if (x == y) {
          dosomething(x);
      }

.. tip:: This is correct

  .. code-block:: c

    if (x == y)
      {
        dosomething(x);
      }

**Use of TAB Characters**. The use of TAB characters for indentation is
prohibited in C source and header files. TAB characters are, however,
used in make files, assembly language source files, Kconfig files and
some script files. When TAB characters are used in these files, spaces
may not be used for indentation. The correct TAB setting is 4 spaces
(not 8) in these cases.

**Alignment of Braces**. Note that since braces must be on a separate
line (see above), this indentation by two spaces has an interesting
property:

-  All C statements (and case selectors) lie on lines that are multiples
   of 4 spaces (beginning with an indentation of two): 2, 6, 10, ...
   (4*n + 2) (for indentation level n = 0, 1, ...)

-  Braces lie on a separate line also indented by multiple of 4 spaces:
   4, 8, 12, ... 4*n (for indentation level n = 1, 2, ...)

Thus, all code at the indentation level should align on the same column.
Similarly, opening and closing braces at the same indentation level
should also align on the same (but different) column.

**Indentation of Pre-Processor Lines**. C Pre-processor commands
following any conditional computation are also indented following
basically the indentation same rules, differing in that the ``#`` always
remains in column 1.

When C pre-processor statements are indented, they should be should be
indented by 2 spaces per level-of-indentation following the ``#``. C
pre-processor statements should be indented when they are enclosed
within C pre-processor conditional logic (``#if``..\ ``#endif``). The
level of indentation increases with each level of such nested
conditional logic.

C pre-processor statements should always be indented in this way in the
``Pre-processor Definitions`` `section <#cfilestructure>`__ of each
file. C pre-processor statements may be indented in the
``Public/Private Data`` and ``Public/Private Functions`` sections of the
file. However, often the indentation of C pre-processor statements
conflicts with the indentation of the C code and makes the code more
difficult to read. In such cases, indentation of C pre-processor
statements should be omitted in those sections (only).

.. error:: This is incorrect

  .. code-block:: c

    #ifdef CONFIG_ABC
    #define ABC_THING1 1
    #define ABC_THING2 2
    #define ABC_THING3 3
    #endif

    #ifdef CONFIG_ABC
      #define ABC_THING1 1
      #define ABC_THING2 2
      #define ABC_THING3 3
    #endif

.. tip:: This is correct

  .. code-block:: c

    #ifdef CONFIG_ABC
    #  define ABC_THING1 1
    #  define ABC_THING2 2
    #  define ABC_THING3 3
    #endif

    #ifdef CONFIG_ABC
    #  define ABC_THING1 1
    #  define ABC_THING2 2
    #  define ABC_THING3 3
    #endif

**Exception**. Each header file includes `idempotence
definitions <#idempotence>`__ at the beginning of the header file. This
conditional compilation does *not* cause any change to the indentation.

.. error:: This is incorrect

  .. code-block:: c

    #ifndef __INCLUDE_SOMEHEADER_H
    #  define __INCLUDE_SOMEHEADER_H
    ...
    #  define THING1 1
    #  define THING2 2
    #  define THING3 3
    ...
    #endif /* __INCLUDE_SOMEHEADER_H */

.. tip:: This is correct

  .. code-block:: c

    #ifndef __INCLUDE_SOMEHEADER_H
    #define __INCLUDE_SOMEHEADER_H
    ...
    #define THING1 1
    #define THING2 2
    #define THING3 3
    ...
    #endif /* __INCLUDE_SOMEHEADER_H */

Parentheses
===========

**Coding Standard:**

-  **Space after key words**. Do not put a left parenthesis (``(``)
   immediately after any C keywords (``for``, ``switch``, ``while``,
   ``do``, ``return``, etc.). Put a space before the left parenthesis in
   these cases.
-  **Otherwise, no space before left parentheses**. Otherwise, there
   should be no space before the left parentheses
-  **No space between function name and argument list**. There should be
   no space between a function name and an argument list.
-  **Never space before the right parentheses**. There should never be
   space before a right parenthesis ( ``)`` ).
-  **No parentheses around returned values**. Returned values should
   never be enclosed in parentheses unless the parentheses are required
   to force the correct order of operations in a computed return value.

.. error:: This is incorrect

  .. code-block:: c

    int do_foobar ( void )
    {
      int ret = 0;
      int i;

      for( i = 0; ( ( i < 5 ) || ( ret < 10 ) ); i++ )
        {
          ret = foobar ( i );
        }

      return ( ret );
    }

.. tip:: This is correct

  .. code-block:: c

    int do_foobar(void)
    {
      int ret = 0;
      int i;

      for (i = 0; i < 5 || ret < 10; i++)
        {
          ret = foobar(i);
        }

      return ret;
    }

**NOTE:** Many people do not trust their understanding of the precedence
of operators and so use lots of parentheses in expressions to force the
order of evaluation even though the parentheses may have no effect. This
will certainly avoid errors due to an unexpected order of evaluation,
but can also make the code ugly and overly complex (as in the above
example). In general, NuttX does not use unnecessary parentheses to
force order of operations. There is no particular policy in this regard.
However, you are are advised to check your C Programming Language book
if necessary and avoid unnecessary parenthesis when possible.

*************************
Data and Type Definitions
*************************

One Definition/Declaration Per Line
===================================

.. error:: This is incorrect

  .. code-block:: c

    extern long time, money;
    char **ach, *bch;
    int i, j, k;

.. tip:: This is correct

  .. code-block:: c

    extern long time;
    extern long money;
    FAR char **ach;
    FAR char *bch;
    int i;
    int j;
    int k;

**NOTE**: See the discussion of `pointers <#farnear>`__ for information
about the ``FAR`` qualifier used above.

Global Variables
================

**Global vs. Local vs. Public vs. Private** By a *global* variable it is
meant any variable defined outside of a function. The distinction is
between this kind of *global* and function *local* definition and refers
to the scope a symbol *within a file*. A related concept for all
*global* names defined within a file is the scope of the name across
different files. If the global symbol is pre-pended with the ``static``
storage class then the scope of the global symbol is within the file
only. This is a somewhat different concept and within NuttX you will
find these distinguished as *private* vs. *public* global symbols.
However, within this standard, the term *global variable* will refer to
any variable that has more than local scope.

**Coding Standard:**

-  **Short global variable names**. Names should be terse, but generally
   descriptive of what the variable is for. Try to say something with
   the variable name, but don't try to say too much. For example, the
   variable name of ``g_filelen`` is preferable to something like
   ``g_lengthoffile``.
-  **Global variable prefix**. All global variables begin with the
   prefix ``g_`` to indicate the scope of variable.
-  **Module name prefix** If a global variable belongs in a *module*
   with a name of, say ``xyz``, then that module should be included as
   part of the prefix like: ``g_xyz_``.
-  **Lowercase**, Use all lower case letters.
-  **Minimal use of** ``_``. Preferably there are no ``_``
   separators within the name. Long variable names might require some
   delimitation using ``_``. Long variable names, however, are
   discouraged.
-  **Use structures**. If possible, wrap all global variables within a
   structure to minimize the liklihood of name collisions.
-  **Avoid global variables when possible**. Use of global variables, in
   general, is discourage unless there is no other reasonable solution.

.. error:: This is incorrect

  .. code-block:: c

    extern int someint;
    static int anotherint;
    uint32_t dwA32BitInt;
    uint32_t gAGlobalVariable;

.. note:: This is acceptable

  .. code-block:: c

    extern int g_someint;
    static int g_anotherint;
    uint32_t g_a32bitint;
    uint32_t g_aglobal;

.. tip:: This is preferred

  .. code-block:: c

    struct my_variables_s
    {
      uint32_t a32bitint;
      uint32_t aglobal;
    };

    extern int g_someint;
    static int g_anotherint;
    struct my_variables_s g_myvariables;

Parameters and Local Variables
==============================

**Coding Standard:**

-  **Common naming standard**. Naming for function parameters and local
   variables is the same.
-  **Short variable names**. Names should be terse, but generally
   descriptive of what the variable is for. Try to say something with
   the variable name, but don't try to say too much. For example, the
   variable name of ``len`` is preferable to something like
   ``lengthofiobuffer``.
-  **No special ornamentation**. There is no special ornamentation of
   the name to indication that the variable is a local variable. The
   prefix ``p`` or ``pp`` may be used on names of pointers (or pointer
   to pointers) if it helps to distinguish the variable from some other
   local variable with a similar name. Even this convention is
   discouraged when not necessary.
-  **Lowercase** Use all lower case letters.
-  **Minimal use of single character variable names**. Short variable
   names are preferred. However, single character variable names should
   be avoided. Exceptions to this include ``i``, ``j``, and ``k`` which
   are reserved only for use as loop indices (part of our Fortran
   legacy).
-  **Minimal use of** ``_``. Preferably there are no ``_``
   separators within the name. Long variable names might require some
   delimitation using ``_``. Long variable names, however, are
   discouraged.

.. error:: This is incorrect

  .. code-block:: c

    uint32_t somefunction(int a, uint32_t dwBValue)
    {
      uint32_t this_is_a_long_variable_name = 1;
      int i;

      for (i = 0; i &lt; a; i++)
        {
          this_is_a_long_variable_name *= dwBValue--;
        }

      return this_is_a_long_variable_name;
    }

.. tip:: This is correct

  .. code-block:: c

    uint32_t somefunction(int limit, uint32_t value)
    {
      uint32_t ret = 1;
      int i;

      for (i = 0; i &lt; limit; i++)
        {
          ret *= value--;
        }

      return ret;
    }

**NOTE:** You will see the local variable named ``ret`` is frequently
used in the code base for the name of a local variable whose value will
be returned or to received the returned value from a called function.

Type Definitions
================

**Coding Standard:**

-  **Short type names**. Type names should be terse, but generally
   descriptive of what the type is for. Try to say something with the
   type name, but don't try to say too much. For example, the type name
   of ``fhandle_t`` is preferable to something like
   ``openfilehandle_t``.
-  **Type name suffix**. All ``typedef``'ed names end with the suffix
   ``_t``.
-  **Module name prefix** If a type belongs in a *module* with a name
   of, say ``xyz``, then that module should be included as a prefix to
   the type name like: ``xyz_``.
-  **Lowercase**. Use all lower case letters.
-  **Minimal use of** ``_``. Preferably there are few ``_``
   separators within the type name. Long type names might require some
   delimitation using ``_``. Long type names, however, are
   discouraged.

.. error:: This is incorrect

  .. code-block:: c

    typedef void *myhandle;
    typedef int myInteger;

.. tip:: This is correct

  .. code-block:: c

    typedef FAR void *myhandle_t;
    typedef int myinteger_t;

**NOTE**: See the discussion of `pointers <#farnear>`__ for information
about the ``FAR`` qualifier used above.

Structures
==========

**Structure Naming**

-  **No un-named structures**. All structures must be named, even if
   they are part of a type definition. That is, a structure name must
   follow the reserved word ``struct`` in all structure definitions.
   There are two exceptions to this rule:

   #. First for structures that are defined within another union or
      structure (discouraged). In those cases, the structure name should
      always be omitted.
   #. Second for structures as the type of a local variable. In this
      case, again, the structure name should always be omitted.

-  **Structured defined with structures discouraged**. Fields within a
   structure may be another structure that is defined only with the
   scope of the containing structure. This practice is acceptable, but
   discouraged.
-  **No un-named structure fields**. Structures may contain other
   structures as fields. In this case, the structure field must be
   named. C11 permits such un-named structure fields within a structure.
   NuttX generally follows C89 and all code outside of architecture
   specific directories must be compatible with C89.
-  **No structure definitions within Type Definition**. The practice of
   defining a structure within a type definition is discouraged. It is
   preferred that the structure definition and the type definition be
   separate definitions. In general, the NuttX coding style discourages
   any ``typdef``-ing of structures; normally the full structure name is
   used as types throughout the code. The reason for this is that is
   structure pointers may be forward referenced in header files without
   having to include the file the provides the type definition. This
   greatly reduces header file coupling.
-  **Short structure names**. Structure names should be terse, but
   generally descriptive of what the structure contains. Try to say
   something with the structure name, but don't try to say too much. For
   example, the structure name of ``xyz_info_s`` is preferable to
   something like ``xyz_datainputstatusinformation_s``.
-  **Structure name suffix**. All structure names end with the suffix
   ``_s``.
-  **Module name prefix** If a structure belongs to a *module* with a
   name of, say ``xyz``, then that module should be included as a prefix
   to the structure name like: ``xyz_``.
-  **Lowercase**. Use all lower case letters.
-  **Minimal use of** ``_``. Preferably there are few ``_``
   separators within the structure name. Long variable names might
   require some delimitation using ``'_'``. Long variable names,
   however, are discouraged.

**Structure Field Naming**

-  **Common variable naming**. Structure field naming is generally the
   same as for local variables.
-  **One definition per line**. The `one definition per
   line <#onedatperline>`__ rule applies to structure fields, including
   bit field definitions.
-  **Each field should be commented**. Each structure field should be
   commented. Commenting should follow the `standard
   conventions <#comments>`__.
-  **Optional structure field prefix**. It make be helpful to add a
   two-letter prefix to each field name so that is is clear which
   structure the field belongs to. Although a good practice, that
   convention has not been used consistently in NuttX.
-  **Lowercase**. Use all lower case letters.
-  **Minimal use of** ``_``. Preferably there are few ``_``
   separators within the field name. Long variable names might require
   some delimitation using ``'_'``. Long variable names, however, are
   discouraged.

**Other Applicable Coding Standards**. See sections related to `line
formatting <#lines>`__, `use of braces <#braces>`__,
`indentation <#indentation>`__, and `comments <#comments>`__.

**Size Optimizations**. When declaring fields in structures, order the
declarations in such a way as to minimize memory waste due of data
alignment. This essentially means that that fields should be organized
by data size, not by functionality: Put all pointers togeter, all
``uint8_t``'s together, all ``uint32_t``'s together. Data types withi
well known like ``uint8_t`` and ``uint32_t`` should also be place in
either ascending or descending size order.

.. error:: This is incorrect

  .. code-block:: c

    typedef struct       /* Un-named structure */
    {
      ...
      int val1, val2, val3; /* Values 1-3 */
      ...
    } xzy_info_t;

    struct xyz_information
    {
      ...
      uint8_t bita : 1,  /* Bit A */
              bitb : 1,  /* Bit B */
              bitc : 1;  /* Bit C */
      ...
    };

    struct abc_s
    {
      ...
      struct
      {
        int a;           /* Value A */
        int b;           /* Value B */
        int c;           /* Value C */
      };                 /* Un-named structure field */
      ...
    };

.. tip:: This is correct

  .. code-block:: c

    struct xyz_info_s
    {
      ...
      int val1;          /* Value 1 */
      int val2;          /* Value 2 */
      int val3;          /* Value 3 */
      ...
    };

.. warning:: This is discouraged

  .. code-block:: c

    typedef struct xyz_info_s xzy_info_t;

The use of typedef'ed structures is acceptable but discouraged.

.. tip:: This is correct

  .. code-block:: c

    struct xyz_info_s
    {
      ...
      uint8_t bita : 1,  /* Bit A */
      uint8_t bitb : 1,  /* Bit B */
      uint8_t bitc : 1,  /* Bit C */
      ...
    };

.. warning:: This is discouraged

  .. code-block:: c

    struct abc_s
    {
      ...
      struct
      {
        int a;           /* Value A */
        int b;           /* Value B */
        int c;           /* Value C */
      } abc;
      ...
    };

The use of structures defined within other structures is acceptable provided that they define named fields.
The general practice of defining a structure within the scope of another structure, however, is still but discouraged in any case.
The following is preferred:

.. tip:: This is preferred

  .. code-block:: c

    struct abc_s
    {
      ...
      int a;             /* Value A */
      int b;             /* Value B */
      int c;             /* Value C */
      ...
    };

Unions
======

**Union and Field Names**. Naming of unions and fields within unions
follow the same naming rules as for `structures and structure
fields <#structures>`__. The only difference is that the suffix ``_u``
is used to identify unions.

**Other Applicable Coding Standards**. See sections related to `line
formatting <#lines>`__, `use of braces <#braces>`__,
`indentation <#indentation>`__, and `comments <#comments>`__.

.. note:: This is acceptable

  .. code-block:: c

    union xyz_union_u  /* All unions must be named */
    {
      uint8_t  b[4];   /* Byte values. */
      uint16_t h[2];   /* Half word values. */
      uint32_t w;      /* Word Value. */
    };

    typedef union xyz_union_u xzy_union_t;

The use of typedef'ed unions is acceptable but discouraged.

.. tip:: This is preferred

  .. code-block:: c

    struct xyz_info_s
    {
      ...
      union
      {
        uint8_t  b[4]; /* Byte values. */
        uint16_t h[2]; /* Half word values. */
        uint32_t w;    /* Word Value. */
      } u;             /* All union fields must be named */
      ...
    };

**NOTE:** Note that the union fields within structures are often named
``u``. This is another exception to the prohibition against using single
character variable and field names. The short field name ``u`` clearly
identifies a union field and prevents the full name of the union value
from being excessively long.

Enumerations
============

**Enumeration Naming**. Naming of enumerations follow the same naming
rules as for `structure <#structures>`__ and `union <#unions%22>`__
naming. The only difference is that the suffix ``_e`` is used to
identify an enumeration.

**Enumeration Value Naming**. Enumeration values, however, following a
naming convention more similar to `macros <#macros>`__.

-  **Uppercase**. Enumeration values are always in upper case.
-  **Use of** ``_`` **encouraged**. Unlike other naming, use of the
   underscore character ``_`` to break up enumeration names is
   encouraged.
-  **Prefix**. Each value in the enumeration should begin with an
   upper-case prefix that identifies the value as a member of the
   enumeration. This prefix should, ideally, derive from the name of the
   enumeration.
-  **No dangling commas**. There should be no dangling comma on the
   final value of the enumeration. The most commonly used tool chain are
   tolerant of such dangling commas, but others will not.

**Other Applicable Coding Standards**. See sections related to `line
formatting <#lines>`__, `use of braces <#braces>`__,
`indentation <#indentation>`__, and `comments <#comments>`__.

.. tip:: This is correct

  .. code-block:: c

    enum xyz_state_e
    {
      XYZ_STATE_UNINITIALIZED = 0, /* Uninitialized state. */
      XYZ_STATE_WAITING,           /* Waiting for input state. */
      XYZ_STATE_BUSY,              /* Busy processing input state. */
      XYZ_STATE_ERROR,             /* Halted due to an error. */
      XYZ_STATE_TERMINATING,       /* Terminating stated. */
      XYZ_STATE_TERMINATED         /* Terminating stated. */
    };

C Pre-processor Macros
======================

**Coding Standard:**

**Macro Naming**. Macro naming following a naming convention similar to
the naming of `enumeration values <#enumerations>`__.

-  **Uppercase**. Macro names are always in upper case.
-  **Lowercase Exceptions**. There are a few lower case values in NuttX
   macro names. Such as a lower-case ``p`` for a period or decimal point
   (such as ``VOLTAGE_3p3V``). I have also used lower-case ``v`` for a
   version number (such as ``CONFIG_NET_IPv6``). However, these are
   exceptions to the rule rather than illustrating a rule.
-  **Macros that could be functions**. Lower-case macro names are also
   acceptable if the macro is a substitute for a function name that
   might be used in some other context. In that case, normal function
   naming applies.
-  **Use of** ``_`` **encouraged**. Unlike other naming, use of the
   underscore character ``_`` to break up macro names is encouraged.
-  **Prefix**. Each related macro value should begin with an upper-case
   prefix that identifies the value as part of a set of values (and also
   to minimize the likelihood of naming collisions).
-  **Single space after** ``#define``. A single space character should
   separate the ``#define`` from the macro name. Tabs are never used.
-  **Normal commenting rules**. Normal commenting rules apply.
-  **Line continuations**. Macro definitions may be continued on the
   next line by terminating the line with the ``\`` character just
   before the newline character. There should be a single space before
   the ``\`` character. Aligned ``\`` characters on multiple line
   continuations are discouraged because they are a maintenance problem.
-  **Parentheses around macro argument expansions**. Macros may have
   argument lists. In the macros expansion, each argument should be
   enclosed in parentheses.
-  **Real statements**. If a macro functions as a statement, then the
   macro expansion should be wrapped in ``do { ... } while (0)`` to
   assume that the macros is, indeed, a statement.
-  **Magic numbers are prohibited in code**. Any numeric value is not
   intuitively obvious, must be properly named and provided as either a
   pre-processor macro or an enumeration value.
-  **Side effects**. Be careful of side effects.
-  **Indentation**. See the `Indentation of Pre-Processor
   Lines <#indentation>`__ requirements above.

**Other Applicable Coding Standards**. See sections related to `line
formatting <#lines>`__, `indentation <#indentation>`__, and
`comments <#comments>`__.

.. error:: This is incorrect

  .. code-block:: c

    #define max(a,b) a > b ? a : b

    #define ADD(x,y) x + y

    #ifdef HAVE_SOMEFUNCTION
    int somefunction(struct somestruct_s* psomething);
    #else
    #define SOMEFUNCTION() (0)
    #endif

    #	define	IS_A_CAT(c)		((c) == A_CAT)

    #define LONG_MACRO(a,b)                                  \
      {                                                      \
        int value;                                           \
        value = b-1;                                         \
        a = b*value;                                         \
      }

    #define DO_ASSIGN(a,b) a = b

.. tip:: This is correct

  .. code-block:: c

    #define MAX(a,b) (((a) > (b)) ? (a) : (b))

    #define ADD(x,y) ((x) + (y))

    #ifdef HAVE_SOMEFUNCTION
    int somefunction(struct somestruct_s* psomething);
    #else
    #  define somefunction(p) (0)
    #endif

    # define IS_A_CAT(c)  ((c) == A_CAT)

    #define LONG_MACRO(a,b) \
      { \
        int value; \
        value = (b)-1; \
        (a) = (b)*value; \
      }

    #define DO_ASSIGN(a,b) do { (a) = (b); } while (0)

.. _farnear:

Pointer Variables
=================

**Pointer Naming**. Pointers following same naming conventions as for
other variable types. A pointer (or pointer-to-a-pointer) variable may
be prefaced with ``p`` (or ``pp``) with no intervening underscore
character ``_`` in order to identify that variable is a pointer. That
convention is not encouraged, however, and is only appropriate if there
is some reason to be concerned that there might otherwise be confusion
with another variable that differs only in not being a pointer.

**White Space**. The asterisk used in the declaration of a pointer
variable or to dereference a pointer variable should be placed
immediately before the variable name with no intervening spaces. A space
should precede the asterisk in a cast to a pointer type.

.. error:: This is incorrect

  .. code-block:: c

    int somefunction(struct somestruct_s* psomething);

    ptr = (struct somestruct_s*)value;

.. tip:: This is correct

  .. code-block:: c

    int somefunction(FAR struct somestruct_s *something);

    ptr = (FAR struct somestruct_s *)value;

.. c:macro:: FAR

``FAR``, ``NEAR``, ``DSEG`` and ``CODE`` pointers. Some architectures
require a qualifier on pointers to identify the address space into which
the pointer refers. The macros ``FAR``, ``NEAR``, ``DSEG`` and ``CODE``
are defined in ``include/nuttx/compiler.h`` to provide meaning for this
qualifiers when needed. For portability, the general rule is that
pointers to data that may lie in the stack, heap, ``.bss``, or ``.data``
should be prefaced by the qualifier ``FAR``; pointers to functions
probably lie in a code address space and should have the qualifier
``CODE``. The typical effect of these macros on architectures where they
have meaning to determine the size of the pointer (size in the sense of
the width of the pointer value in bits).

Initializers
============

**Applicable Coding Standards**. See the section related to
`parentheses <#parentheses>`__.

**C89 Compatibility**. All common NuttX code must conform to ANSII C89
requirements. Newer C standards permit more flexible initialization with
named initializers and array initializers. However, these are not
backward compatible with C89 and cannot be used in common code. Newer
C99 features may be included in architecture-specific sub-directories
where there is no possibility of the use of such older toolchains. C11
is included in NuttX, but has not been verified and, hence, it not
encourage anywhere.

*********
Functions
*********

Function Headers
================

**Coding Standard:**

-  **Function headers**. Each function is preceded by a function header.
   The function header is a *block comment* that provides information
   about the function. The block comment consists of the following:

   -  The block comment begins with a line that consists of the opening
      C comment in column 1 (``/*``) followed by a series of asterisks
      extending to the length of the line (usually to column 78).
   -  The block comment ends with a line that consists of series of
      asterisks beginning at column 2 and extending to the near the end
      line (usually to column 77) followed by the closing C comment in
      (usually at column 78 for a total length of 79 characters).
   -  Information about the function is included in lines between the
      first and final lines. Each of these begin with a space in column
      1, an sterisk (``*``) in column 2, and a space in column 3.

-  **Function header preceded by one blank line**. Exactly one blank
   line precedes each function header.
-  **Function header followed by one blank line**. Exactly one blank
   line is placed after function header and before the function
   definition.
-  **Function header sections**. Within the function header, the
   following data sections must be provided:

   -  ``* Name:`` followed by the name of the function on the same
      line.
   -  ``* Description:`` followed by a description of the function
      beginning on the second line. Each line of the function
      description is indented by two additional spaces.
   -  ``* Input Parameters:`` followed by a description of the of
      each input parameter beginning on the second line. Each input
      parameter begins on a separator line indented by two additional
      spaces. The description needs to include (1) the name of the input
      parameters, and (2) a short description of the input parameter.
   -  ``* Returned Value:`` followed by a description of the of
      returned value(s) beginning on the second line. The description of
      the returned value should identify all error values returned by
      the function.
   -  ``* Assumptions/Limitations:`` followed by a any additional
      information that is needed to use the function correctly. This
      section is optional and may be omitted with there is no such
      special information required for use of the function.

   Each of these data sections is separated by a single line like ``*``.

**Function header template**. Refer to `Appendix A <#cfilestructure>`__
for the template for a function header.

Function Naming
===============

**Coding Standard:**

-  **Short function names**. Function names should be terse, but
   generally descriptive of what the function is for. Try to say
   something with the function name, but don't try to say too much. For
   example, the variable name of ``xyz_putvalue`` is preferable to
   something like ``xyz_savethenewvalueinthebuffer``.
-  **Lowercase**. Use all lower case letters.
-  **Module prefix**. All functions in the same *module*, or
   *sub-system*, or within the same file should have a name beginning
   with a common prefix separated from the remainder of the function
   name with the underscore, ``'_'``, character. For example, for a
   module called *xyz*, all of the functions should begin with ``xyz_``.
-  **Extended prefix**. Other larger functional grouping should have
   another level in the naming convention. For example, if module *xyz*
   contains a set of functions that manage a set of I/O buffers (IOB),
   then those functions all should get naming beginning with a common
   prefix like ``xyz_iob_``.
-  **Use of** ``_`` **discouraged**. Further use of the ``'_'`` separators
   is discouraged in function naming. Long function names might require
   some additional elimitation using ``'_'``. Long function names,
   however, are also discouraged.
-  **Verbs and Objects**. The remainder of the function name should be
   either in the form of *verb-object* or *object-verb*. It does not
   matter which as long as the usage is consistent within the *module*.
   Common verbs include *get* and *set* (or *put*) for operations that
   retrieve or store data, respectively. The verb *is* is reserved for
   functions that perform some test and return a boolean value to
   indicate the result of the test. In this case, the *object* should
   indicate what is testing and the return value of ``true`` should be
   consistent with result of the test being true.

Parameter Lists
===============

**Coding Standards**. See general rules for `parameter
naming <#localvariable>`__. See also the sections related to the use of
`parentheses <#parentheses>`__.

**Use of** ``const`` **parameters**. Use of the ``const`` storage class is
encouraged. This is appropriate to indicate that the function will not
modify the object.

Function Body
=============

**Coding Standard:**

-  **Single compound statement**. The function body consists of a single
   compound statement.
-  **Braces in column 1** The opening and close braces of the compound
   statement must be placed in column one.
-  **First definition or statement in column 3**. The first data
   definitions or statements in the function body are idented by two
   spaces. Standards for statements are covered in the `following
   paragraph <#statements>`__
-  **Local variables first**. Because NuttX conforms to the older C89
   standard, all variables that have scope over the compound statement
   must be defined at the beginning of the compound statement prior to
   any executable statements. Local variable definitions intermixed
   within the following sequence of executable statements are forbidden.
   A single blank line must follow the local variable definitions
   separating the local variable definitions from the following
   executable statements. **NOTE** that a function body consists of a
   compound statement, but typically so does the statement following
   ``if``, ``else``, ``for``, ``while``, ``do``. Local variable
   definitions are also acceptable at the beginning of these compound
   statements as with any other.
-  **Long functions are discouraged**. As a rule of thumb, the length of
   a function should be limited so that it would fit on a single page
   (if you were to print the source code).
-  **Return Statement**. The argument of the ``return`` statement should
   *not* be enclosed in parentheses. A reasonable exception is the case
   where the returned value argument is a complex expression and where
   the parentheses improve the readability of the code. Such complex
   expressions might be Boolean expressions or expressions containing
   conditions. Simple arithmetic computations would not be considered
   *complex* expressions.
-  **Space after the function body**. A one (and only one) blank line
   must follow the closing right brace of the function body.

**Other Applicable Coding Standards**. See sections related to `General
Conventions <#general>`__, `Parameters and Local
Variables <#localvariable>`__, and `Statements <#statements>`__.

.. error:: This is incorrect

  .. code-block:: c

    int myfunction(int a, int b)
      {
        int c, d;
        c = a
        d = b;

        int e = c + d;

        for (int i = 0; i &lt; a; i++)
          {
            for (int j = 0; j &lt; b; j++)
              {
                e += j * d;
              }
          }

        return (e / a);
      }

.. tip:: This is correct

  .. code-block:: c

    int myfunction(int a, int b)
    {
      int c;
      int d;
      int e;
      int i;

      c = a
      d = b;
      e = c + d;

      for (i = 0; i &lt; a; i++)
        {
          int j;

          for (j = 0; j &lt; b; j++)
            {
              e += j * d;
            }
        }

      return e / a;
    }

Returned Values
===============

**OS Internal Functions**. In general, OS internal functions return a
type ``int`` to indicate success or failure conditions. Non-negative
values indicate success. The return value of zero is the typical success
return value, but other successful return can be represented with other
positive values. Errors are always reported with negative values. These
negative values must be a well-defined ``errno`` as defined in the file
``nuttx/include/errno.h``.

**Application/OS Interface**. All but a few OS interfaces conform to
documented standards that have precedence over the coding standards of
this document.

**Checking Return Values**. Callers of internal OS functions should
always check return values for an error. At a minimum, a debug statement
should indicate that an error has occurred. Ignored return values are
always suspicious. All calls to ``malloc`` or ``realloc``, in
particular, must be checked for failures to allocate memory to avoid use
of NULL pointers.

**********
Statements
**********

One Statement Per Line
======================

**Coding Standard:**

-  **One statement per line**. There should never be more than one
   statement on a line.
-  **No more than one assignment per statement**. Related to this, there
   should never be multiple assignments in the same statement.
-  **Statements should never be on the same line as any keyword**.
   Statements should never be on the same line as case selectors or any
   C keyword.

**Other Applicable Coding Standards**. See the section related to the
use of `braces <#braces>`__.

.. error:: This is incorrect

  .. code-block:: c

    if (var1 &lt; var2) var1 = var2;

    case 5: var1 = var2; break;

    var1 = 5; var2 = 6; var3 = 7;

    var1 = var2 = var3 = 0;

.. tip:: This is correct

  .. code-block:: c

    if (var1 &lt; var2)
      {
        var1 = var2;
      }

    case 5:
      {
        var1 = var2;
      }
      break;

    var1 = 5;
    var2 = 6;
    var3 = 7;

    var1 = 0;
    var2 = 0;
    var3 = 0;

Casts
=====

**Coding Standard:**

-  **No space in cast**. There should be no space between a cast and the
   value being cast.

.. error:: This is incorrect

  .. code-block:: c

    struct something_s *x = (struct something_s*) y;

.. tip:: This is correct

  .. code-block:: c

    struct something_s *x = (struct something_s *)y;

Operators
=========

**Spaces before and after binary operators**. All binary operators
(operators that come between two values), such as ``+``, ``-``, ``=``,
``!=``, ``==``, ``>``, etc. should have a space before and after the
operator, for readability. As examples:

.. error:: This is incorrect

  .. code-block:: c

    for=bar;
    if(a==b)
    for(i=0;i<5;i++)

.. tip:: This is correct

  .. code-block:: c

    for = bar;
    if (a == b)
    for (i = 0; i < 5; i++)

**No space separating unary operators**. Unary operators (operators that
operate on only one value), such as ``++``, should *not* have a space
between the operator and the variable or number they are operating on.

.. error:: This is incorrect

  .. code-block:: c

    x ++;

.. tip:: This is correct

  .. code-block:: c

    x++;

**Forbidden Multicharacter Forms**. Many operators are expressed as a
character in combination with ``=`` such as ``+=``, ``>=``, ``>>=``,
etc. Some compilers will accept the ``=`` at the beginning or the end of
the sequence. This standard, however, requires that the ``=`` always
appear last in order to avoid amiguities that may arise if the ``=``
were to appear first. For example, ``a =++ b;`` could also be
interpreted as ``a =+ +b;`` or ``a = ++b`` all of which are very
different.

``if then else`` Statement
==========================

**Coding Standard:**

-  ``if`` **separated from** ``<condition>``. The ``if`` keyword and the
   ``<condition>`` must appear on the same line. The ``if`` keyword and
   the ``<condition>`` must be separated by a single space.
-  **Indentation and parentheses**. ``if <condition>`` follows the
   standard indentation and parentheses rules.
-  **Alignment**. The ``if`` in the ``if <condition>`` line and the
   ``else`` must be aligned at the same column.
-  **Statement(s) always enclosed in braces**. Statement(s) following
   the ``if <condition>`` and ``else`` keywords must always be enclosed
   in braces. Braces must follow the ``if <condition>`` and ``else``
   lines even in the cases where (a) there is no contained statement or
   (b) there is only a single statement!
-  **Braces and indentation**. The placement of braces and statements
   must follow the standard rules for `braces and
   indentation <#braces>`__.
-  **Final brace followed by a single blank line**. The *final* right
   brace of the ``if``-``else`` must be followed by a blank line in most
   cases (the exception given below). This may be the final brace of the
   ``if`` compound statement if the ``else`` keyword is not present. Or
   it may be the the final brace of the ``else`` compound statement if
   present. A blank line never follows the right brace closing the
   ``if`` compound statement if the ``else`` keyword is present. Use of
   braces must follow all other standard rules for `braces and
   spacing <#braces>`__.
-  **Exception**. That blank line must also be omitted for certain cases
   where the ``if <condition>``-``else`` statement is nested within
   another compound statement; there should be no blank lines between
   consecutive right braces as discussed in the standard rules for use
   of `braces <#braces>`__.

**Other Applicable Coding Standards**. See sections related to `use of
braces <#braces>`__ and `indentation <#indentation>`__.

.. error:: This is incorrect

  .. code-block:: c

    if(var1 < var2) var1 = var2;

    if(var > 0)
      var--;
    else
      var = 0;

    if (var1 > 0) {
      var1--;
    } else {
      var1 = 0;
    }
    var2 = var1;

.. tip:: This is correct

  .. code-block:: c

    if (var1 < var2)
      {
        var1 = var2;
      }

    if (var > 0)
      {
        var--;
      }
    else
      {
        var = 0;
      }

    if (var1 > 0)
      {
        var1--;
      }
    else
      {
        var1 = 0;
      }

    var2 = var1;

**Ternary operator** (``<condition> ? <then> : <else>``):

-  **Only if the expression is short**. Use of this form is only
   appropriate if the entire sequence is short and fits neatly on the
   line.
-  **Multiple lines forbidden**. This form is forbidden if it extends to
   more than one line.
-  **Use of parentheses**. The condition and the entire sequence are
   often enclosed in parentheses. These are, however, not required if
   the expressions evaluate properly without them.

**Other Applicable Coding Standards**. See sections related to
`parentheses <#parentheses>`__.

.. tip:: This is correct

  .. code-block:: c

    int arg1 = arg2 > arg3 ? arg2 : arg3;
    int arg1 = ((arg2 > arg3) ? arg2 : arg3);

``switch`` Statement
====================

**Definitions:**

-  **Case logic**. By *case logic* it is mean the ``case`` or
   ``default`` and all of the lines of code following the ``case`` or
   ``default`` up to the next ``case``, ``default``, or the right brace
   indicating the end of the switch statement.

**Coding Standard:**

-  ``switch`` **separated from** ``<value>``. The ``switch`` keyword and
   the switch ``<value>`` must appear on the same line. The ``if``
   keyword and the ``<value>`` must be separated by a single space.
-  **Falling through**. Falling through a case statement into the next
   case statement is be permitted as long as a comment is included.
-  ``default`` **case**. The ``default`` case should always be present
   and trigger an error if it is reached when it should not be.
-  **Case logic in braces**. It is preferable that all *case logic*
   (except for the ``break``) be enclosed in braces. If you need to
   instantiate local variables in case logic, then that logic must be
   surrounded with braces.
-  ``break`` **outside of braces**. ``break`` statements are normally
   indented by two spaces. When used conditionally with *case logic*,
   the placement of the break statement follows normal indentation
   rules.
-  **Case logic followed by a single blank line**. A single blank line
   must separate the *case logic* and any following ``case`` or
   ``default``. The should, however, be no blank lines between the *case
   logic* and the closing right brace.
-  **Switch followed by a single blank line**. The final right brace
   that closes the ``switch <value>`` statement must be followed by a
   single blank line.
-  **Exception**. That blank line must be omitted for certain cases
   where the ``switch <value>`` statement is nested within another
   compound statement; there should be no blank lines between
   consecutive right braces as discussed in the standard rules for use
   of `braces <#braces>`__.

**Other Applicable Coding Standards**. See sections related to `use of
braces <#braces>`__, `indentation <#indentation>`__, and
`comments <#comments>`__.

.. tip:: This is correct

  .. code-block:: c

    switch (...)
      {
        case 1:  /* Example of a comment following a case selector. */
        ...

        /* Example of a comment preceding a case selector. */

        case 2:
          {
            /* Example of comment following the case selector. */

            int value;
            ...
          }
          break;

        default:
          break;
      }

``while`` Statement
===================

**Coding Standard:**

-  ``while`` **separated from** ``<condition>``. The ``while`` keyword
   and the ``<condition>`` must appear on the same line. The ``while``
   keyword and the ``<condition>`` must be separated by a single space.
-  **Keywords on separate lines**. ``while <condition>`` must lie on a
   separate line with nothing else present on the line.
-  **Indentation and parentheses**. ``while <condition>`` follows the
   standard indentation and parentheses rules.
-  **Statements enclosed in braces** Statement(s) following the
   ``while <condition>`` must always be enclosed in braces, even if only
   a single statement follows.
-  **No braces on null statements**. No braces are required if no
   statements follow the ``while <condition>``. The single semicolon
   (null statement) is sufficient;
-  **Braces and indentation**. The placement of braces and statements
   must follow the standard rules for braces and indentation.
-  **Followed by a single blank line**. The final right brace that
   closes the ``while <condition>`` statement must be followed by a
   single blank line.
-  **Exception**. That blank line must be omitted for certain cases
   where the ``while <condition>`` statement is nested within another
   compound statement; there should be no blank lines between
   consecutive right braces as discussed in the standard rules for use
   of `braces <#braces>`__.

**Other Applicable Coding Standards**. See sections related to `use of
braces <#braces>`__, `indentation <#indentation>`__, and
`comments <#comments>`__.

.. error:: This is incorrect

  .. code-block:: c

    while( notready() )
      {
      }
    ready = true;

    while (*ptr != '\0') ptr++;

.. tip:: This is correct

  .. code-block:: c

    while (notready());

    ready = true;

    while (*ptr != '\0')
      {
        ptr++;
      }

``do while`` Statement
======================

**Coding Standard:**

-  **Keywords on separate lines**. ``do`` and ``while <condition>`` must
   lie on separate lines with nothing else present on the line.
-  **Indentation and parentheses**. ``do .. while <condition>`` follows
   the standard indentation and parentheses rules.
-  **Statements enclosed in braces** Statement(s) following the ``do``
   must always be enclosed in braces, even if only a single statement
   (or no statement) follows.
-  **Braces and indentation**. The placement of braces and statements
   must follow the standard rules for braces and indentation.
-  ``while`` **separated from** ``<condition>``. The ``while`` keyword
   and the ``<condition>`` must appear on the same line. The ``while``
   keyword and the ``<condition>`` must be separated by a single space.
-  **Followed by a single blank line**. The concluding
   ``while <condition>`` must be followed by a single blank line.

**Other Applicable Coding Standards**. See sections related to `use of
braces <#braces>`__, `indentation <#indentation>`__, and
`comments <#comments>`__.

.. error:: This is incorrect

  .. code-block:: c

    do {
      ready = !notready();
    } while (!ready);
    senddata();

    do ptr++; while (*ptr != '\0');

.. error:: This is incorrect

  .. code-block:: c

    do
      {
        ready = !notready();
      }
    while (!ready);

    senddata();

    do
      {
        ptr++;
      }
    while (*ptr != '\0');

Use of ``goto``
===============

**Coding Standard:**

-  **Limited Usage of** ``goto``. All use of the ``goto`` statement is
   prohibited except for one usage: for handling error conditions in
   complex, nested logic. A simple ``goto`` in those conditions can
   greatly improve the readability and complexity of the code.
-  **Label Naming**. Labels must all lower case. The underscore
   character ``_`` is permitted to break up long labels.
-  **Error Exit Labels**. The error exit label is normally called
   ``errout``. Multiple error labels are often to required to *unwind*
   to recover resources committed in logic prior to the error to
   otherwise *undo* preceding operations. Naming for these other labels
   would be some like ``errout_with_allocation``,
   ``errout_with_openfile``, etc.
-  **Label Positioning**. Labels are never indented. Labels must always
   begin in column 1.

.. tip:: This is correct

  .. code-block:: c

       FAR struct some_struct_s *ptr;
       int fd;
       int ret;
       ...

       if (arg == NULL)
         {
           ret = -EINVAL;
           goto errout;
         }
       ...
       ptr = (FAR struct some_struct_s *)malloc(sizeof(struct some_struct_s));
       if (!ptr)
         {
           ret = -ENOMEM;
           goto errout;
         }
       ...
       fd = open(filename, O_RDONLY);
       if (fd < 0)
         {
           errcode = -errno;
           DEBUGASSERT(errcode > 0);
           goto errotout_with_alloc;
         }
       ...
       ret = readfile(fd);
       if (ret < 0)
         {
           goto errout_with_openfile;
         }
       ...
    errout_with_openfile:
      close(fd);

    errout_with_alloc:
      free(ptr);

    error:
      return ret;

**NOTE**: See the discussion of `pointers <#farnear>`__ for information
about the ``FAR`` qualifier used above.

***
C++
***

There is no existing document that provides a complete coding standard
for NuttX C++ files. This section is included here to provide some
minimal guidance in C++ code development. In most details like
indentation, spacing, and file organization, it is identical to the C
coding standard. But there are significant differences in the acceptable
standard beyond that. The primary differences are as follows:

C++ style comments are not only permissible but are required (other than
for the following exception). This includes the block comments of in the
*Source File Structure* described in an `Appendix <#appndxa>`__ to this
standard.

Deoxygen tags are acceptable. As are C style comments when needed to
provide DOxygen tags.

There is currently no requirement to conform any specific C++ version.
However, for portability reasons, conformance to older, pre-C++11
standards is encouraged where reasonable.

C++ file name extensions: The extension ``.cxx`` is used for C++ source
files; the extension ``.hxx`` is used for C++ header files.

All naming must use *CamelCase*. Use of the underbar character, '_' is
discouraged. This includes variables, classes, structures, ..., etc.:
All user-nameable C++ elements. Pre-processor definitions are still
required to be all upper case.

Local variable, method names, and function names must all begin with a
lower case letter. As examples, ``myLocalVariable`` would be a compliant
name for a local variable; ``myMethod`` would be a compliant name for a
method;

Namespaces, global variable, class, structure, template, and enumeration
names begin with a capital letter identifying what is being named:

 *Namespace Names*
   Namespaces begin with an upper case character but no particular
   character is specified. As an example, ``MyNamespace`` is fully
   compliant.
 *Global Variable Names*
   Global variables and singletons begin with an upper case '**G**'. For
   example, ``GMyGlobalVariable``. The prefix ``g_`` is never used.
 *Implementation Class Names*
   Classes that implement methods begin with an upper case '**C**'. For
   example, ``CMyClass``. A fully qualified method of ``CMyClass`` could
   be ``MyNamespace::CMyClass::myMethod``
 *Pure Virtual Base Class Names*
   Such base classes begin with an upper case '**I**'. For example,
   ``IMyInterface``.
 *Template Class Names*
   Template classes begin with an upper case '**T**'. For example,
   ``TMyTemplate``.
 *``typedef``'d Type Names*
   Currently all such types also begin with an upper case '**T**'. That
   probably needs some resolution to distinguish for template names. The
   suffix ``_t`` is never used.
 *Structure Names*
   Structures begin with an upper case '**S**'. For example,
   ``SMyStructure``. The suffix ``_s`` is never used.
 *Enumerations Names*
   Enumerations begin with an upper case '**E**'. For example,
   ``EMyEnumeration``. The suffix ``_e`` is never used.

.. _appndxa:

********
Appendix
********

.. _cfilestructure:

C Source File Structure
=======================

.. code-block:: c

   /****************************************************************************
    * <Relative path to the file>
    * <Optional one line file description>
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

*All header files are included here.*

.. code-block:: c

   /****************************************************************************
    * Pre-processor Definitions
    ****************************************************************************/

*All C pre-processor macros are defined here.*

.. code-block:: c

   /****************************************************************************
    * Private Types
    ****************************************************************************/

*Any types, enumerations, structures or unions used by the file are
defined here.*

.. code-block:: c

   /****************************************************************************
    * Private Function Prototypes
    ****************************************************************************/

*Prototypes of all static functions in the file are provided here.*

.. code-block:: c

   /****************************************************************************
    * Private Data
    ****************************************************************************/

*All static data definitions appear here.*

.. code-block:: c

   /****************************************************************************
    * Public Data
    ****************************************************************************/

*All data definitions with global scope appear here.*

.. code-block:: c

   /****************************************************************************
    * Private Functions
    ****************************************************************************/

   /****************************************************************************
    * Name: <Static function name>
    *
    * Description:
    *   Description of the operation of the static function.
    *
    * Input Parameters:
    *   A list of input parameters, one-per-line, appears here along with a
    *   description of each input parameter.
    *
    * Returned Value:
    *   Description of the value returned by this function (if any),
    *   including an enumeration of all possible error values.
    *
    * Assumptions/Limitations:
    *   Anything else that one might need to know to use this function.
    *
    ****************************************************************************/

*All static functions in the file are defined in this grouping. Each is
preceded by a function header similar to the above.*

.. code-block:: c

   /****************************************************************************
    * Public Functions
    ****************************************************************************/

   /****************************************************************************
    * Name: <Global function name>
    *
    * Description:
    *   Description of the operation of the function.
    *
    * Input Parameters:
    *   A list of input parameters, one-per-line, appears here along with a
    *   description of each input parameter.
    *
    * Returned Value:
    *   Description of the value returned by this function (if any),
    *   including an enumeration of all possible error values.
    *
    * Assumptions/Limitations:
    *   Anything else that one might need to know to use this function.
    *
    ****************************************************************************/

*All global functions in the file are defined here.*

C Header File Structure
=======================

.. code-block:: c

  /****************************************************************************
  * <Relative path to the file>
  * <Optional one line file description>
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

*Header file* `idempotence <#idempotence>`__ *definitions go here*

.. code-block:: c

  /****************************************************************************
  * Included Files
  ****************************************************************************/

*All header files are included here.*

.. code-block:: c

  /****************************************************************************
  * Pre-processor Definitions
  ****************************************************************************/

*All C pre-processor macros are defined here.*

.. code-block:: c

  /****************************************************************************
  * Public Types
  ****************************************************************************/

  #ifndef __ASSEMBLY__

*Any types, enumerations, structures or unions are defined here.*

.. code-block:: c

  /****************************************************************************
  * Public Data
  ****************************************************************************/

  #ifdef __cplusplus
  #define EXTERN extern "C"
  extern "C"
  {
  #else
  #define EXTERN extern
  #endif

*All data declarations with global scope appear here, preceded by the
definition* ``EXTERN``.

.. code-block:: c

 /****************************************************************************
  * Inline Functions
  ****************************************************************************/

 /****************************************************************************
  * Name: <Inline function name>
  *
  * Description:
  *   Description of the operation of the inline function.
  *
  * Input Parameters:
  *   A list of input parameters, one-per-line, appears here along with a
  *   description of each input parameter.
  *
  * Returned Value:
  *   Description of the value returned by this function (if any),
  *   including an enumeration of all possible error values.
  *
  * Assumptions/Limitations:
  *   Anything else that one might need to know to use this function.
  *
  ****************************************************************************/

*Any static inline functions may be defined in this grouping. Each is
preceded by a function header similar to the above.*

.. code-block:: c

  /****************************************************************************
  * Public Function Prototypes
  ****************************************************************************/

  /****************************************************************************
  * Name: <Global function name>
  *
  * Description:
  *   Description of the operation of the function.
  *
  * Input Parameters:
  *   A list of input parameters, one-per-line, appears here along with a
  *   description of each input parameter.
  *
  * Returned Value:
  *   Description of the value returned by this function (if any),
  *   including an enumeration of all possible error values.
  *
  * Assumptions/Limitations:
  *   Anything else that one might need to know to use this function.
  *
  ****************************************************************************/

*All global functions in the file are prototyped here. The keyword*
``extern`` *or the definition* ``EXTERN`` *are never used with function
prototypes.*

.. code-block:: c

   #undef EXTERN
   #ifdef __cplusplus
   }
   #endif

   #endif /* __INCLUDE_ASSERT_H */

Ending with the header `idempotence <#idempotence>`__ ``#endif``.
