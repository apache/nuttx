=======================
Integrating with Newlib
=======================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/Integrating+with+Newlib

Built-In C Library
==================

NuttX has its own, tiny, built-in C library. Along with that C library are 
all of the headers files with the definitions specific to that built-in C 
library. The definitions in those header files are not compatible with the 
definitions that you will find in the header files that come with any other 
C library and trying to mix these with header files from other C libraries 
is bound to cause you problems.

When GCC is built, it is built against a C library. The NuttX `buildroot` 
tools are, of course, built against the built-in NuttX C-library and 
would seem to be the obvious choice of tools to use. But there are 
many reasons to use other tool chains. As examples, the NuttX `buildroot` 
tools have some limitations in C++ support. Another example, is that you 
might want to use the higher-performance math library that is included 
in some other implementation of the C library.

There are many C libraries available: `glibc` and `uClibc` are commonly 
used with Linux tools. These should be avoided. Most embedded toolchains 
will be built against `newlib`. So if you are not using the NuttX buildroot 
toolchain, you will most likely be using a toolchain that has `newlib` built 
into it. Because of this, you may see issues if you include `newlib` header 
files into your NuttX code.

Header File Issues
==================

math.h
------

Nuttx includes a built-in math library that can be selected with 
``CONFIG_LIBM=y``. There are reasons to use an external math library, 
however: The NuttX math library is written in C and will not be as 
performant as a custom math library tuned for your processor 
architecture. There some addition issues with the NuttX math 
libraries as documented in the top-level TODO list.

Many people choose to use the `newlib` math library. If you include 
``math.h`` without selecting ``CONFIG_LIBM=y``, you will probably get the 
`newlib` math library and you will certainly see a compilation error 
involving the definition of the type ``wint_t``.

There have been many work arounds described in the NuttX forum. 
Here are a few:

* Copy the newlib ``math.h`` to ``nuttx/include/math.h`` and remove 
  the reference to ``wint_t``.

* Add the following to ``nuttx/libc/stdio/lib_libvsprintf.c``. I 
  especially dislike this solution because it involves modication 
  to a NuttX header file that cannot be accepted upstream.

.. code-block:: c

    /* Include floating point functions */
 
    #ifdef CONFIG_LIBC_FLOATINGPOINT
    #  include "wchar.h"
    #  include "stdio/lib_libdtoa.c"
    #endif

* Provide your own version of ``math.h`` (for GCC only) containing the 
  following. And add the path to this ``math.h`` to your ``CFLAGS`` include 
  path arguments. The path can specified by adding ``-system`` or ``-I`` to the 
  CFLAGS. The path to this ``math.h`` must be defined last so that it has 
  precedence. This will include the NuttX ``wint_t`` definition, then continue 
  to include the default version of ``math.h``.

.. code-block:: c

    #ifndef _MYMATH_H
    #define _MYMATH_H
    
    #include "wchar.h"
    #include_next <math.h>
    
    #endif /* _MYMATH_H */

* The PX4 team uses these patches to `cwhar <https://github.com/PX4/Firmware/blob/nuttx_v3/nuttx-patches/c%2B%2B11.patch>`_ 
  and `math.h <https://github.com/PX4/Firmware/blob/nuttx_v3/nuttx-patches/math.h.patch>`_ 
  to solve the issue. But note the comments in that code:

.. code-block:: c 

    /* N.B. The following definitions are enabled at this time to allow the PX4
   * development to continue until there is a SAFE  solution to foreign
   * (non-nuttx) header file inclusion. There is a potential of a binary
   * incompatibility and runtime errors, memory overwrites or corruption
   * VVVVVVVVVVVVVVVVVVVVVVVVVV Begin Warning VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
   */

* Some people have suggested adding the type definition of ``wint_t`` to 
  ``nuttx/include/sys/types.h`` merely because that header file will then 
  be included into the newlib ``math.h``. This inclusion, of course, also 
  `very dangerous` since the types in the NuttX ``sys/types.h`` header file 
  may not agree with the types in the pre-compiled newlib math library. 
  This solution is not recommended, in any case. The type ``wint_t`` is 
  already correctly defined in ``nuttx/include/sys/wchar.h`` which is the 
  one and only correct location per 
  `OpenGroup.org <http://pubs.opengroup.org/onlinepubs/007908775/xsh/wchar.h.html>`_. 
  It is a mystery to me why the newlib ``math.h`` header file uses ``wint_t`` 
  without including ``wchar.h``. If it did, then there would then this 
  compilation issue would not exist 
  (there could still be subtle binary compatibility issues).

* The ideal solution would be to integrate a third-party, optimized, 
  ARM math library into NuttX, building it using only NuttX header 
  files. That would guarantee no binary incompatibility and would be 
  a very useful contribution to NuttX.

Changes to the ``nuttx/arch/<architecture>/src/Makefile`` may also be required so 
that the linker can find and include the math library (similar to the existed 
logic in the ``Makefile`` to find ``libgcc.a``).

Update: This issue may have finally been resolved with this commit:

.. code-block::

    commit 894ca622e6a408e5fa858a3fee46fb16f32cf86c
    Author: Xiang Xiao \<xiaoxiang@xiaomi.com\>
    Date:   Mon Aug 27 06:26:37 2018 -0600
    
    include/sys/types:  Move wint_t and wctype_t from wchar.h to
    types.h.  This change is compatible as before since wchar.h
    include types.h indirectly.  This fixes a compilation error with
    newlib's math.h:  'unknown type name wint_t'

cmath
-----

This error has been reported:

.. code-block:: bash

    /nuttx/include/cxx/cmath:124:11: error: '::log2l' has not been declared...

Apparently the function logic ``log2l()`` is defined in the NuttX ``math.h`` and 
added to the ``std::`` namespace in ``cmath``. But, apparently, the 
newlib ``math.h`` does not prototype ``lib2l()``.

If you plan to use the newlib ``math.h`` and the NuttX ``cmath``, then 
you probably have to modify ``cmath`` as well.

alloca.h
--------

If your imported application includes ``alloca.h``, then you will run into the 
same kinds of issues. Nuttx does not provide this header file an so you will 
probably end up including the newlib version of ``alloca.h`` which has similar 
disastrous results.

One solution for GCC (only) would be to provide you own ``alloca.h`` containing:

.. code-block:: C

    #ifndef _ALLOCA_H
    #define _ALLOCA_H
    
    #define alloca __builtin_alloca
    
    #endif /* _ALLOCA_H */

And add the path to this ``alloca.h`` to your ``CFLAG`` include path arguments.
The path can specified by adding ``-system`` or ``-I`` to the CFLAGS. The path 
to this ``alloca.h`` must be defined last so that it has precedence.

However, if you include newlib header files that include other ``alloc.h`` 
you may still end up including the newlib ``alloca.h``. So another solution 
might be to include your own ``math.h``, for example, which contains 
something like like:

.. code-block:: C

    #ifndef _MYMATH_H
    #define _MYMATH_H
    #ifndef _ALLOCA_H
    #define _ALLOCA_H

    #define alloca __builtin_alloca

    #endif /* _ALLOCA_H */

    #include_next <math.h>

    #endif /* _MYMATH_H */

This will provide the ``alloca()`` definition, then continue to include 
the default version of ``math.h``. This works because the idempotence 
pre-processor variable ``_ALLOC_H`` matches the same idempotence variable 
used in the newlib ``alloca.h``. Thus, any sneak inclusion of ``alloca.h`` 
with have not effect.

C++ Issues
==========

Most of the C++ issues that have not so much to do with header 
files as with C++ name mangling and strict typing.

new Operator
------------

The prototype for the C++ new operator is:

.. code-block:: C

    void *operator new(size_t nbytes)

However, ``size_t``, in turn, defined to be either ``unsigned long`` or 
``unsigned int`` in the toolchain. This differs with different 
versions of GCC toolchains and has nothing to do with header 
file inclusion. NuttX supports a configuration option to work 
around this, change new to either:

.. code-block:: C

    #ifdef CONFIG_CXX_NEWLONG
    void *operator new(unsigned long nbytes)
    #else
    void *operator new(unsigned int nbytes)
    #endif

This C++ name mangling issue has been around for years and varies from 
GCC compiler to GCC compiler, apparently due to some `newlib` 
configuration difference.

uint32_t
--------

Similarly, you may find that the definition of ``uint32_t`` in NuttX may 
be incompatible with your toolchain's libraries. You may, 
perhaps, see errors like:

.. code-block:: bash

    error: redeclaration of 'typedef long unsigned int std::uint_least32_t'

The definition in the ARM header file at ``nuttx/arch/arm/include/types.h`` is:

.. code-block:: C

    typedef signed int _int32_t;
    typedef unsigned int _uint32_t;

On ARM platforms, ``unsigned long`` and ``unsigned int`` are both 32-bit 
integers so it does not matter which definition you use. But it does matter 
if you are concerned about the C++ name mangling associated with the use of 
size_t by your compiler. If you see errors such as the above, then 
you can replace these type definition to avoid C++ name mangling 
incompatibilities like:

.. code-block:: C

    typedef signed long _int32_t;
    typedef unsigned long _uint32_t;

But this now may result in additional problems, now there may be 
incompatibilities between definition of ``size_t`` uses in NuttX and the 
definition of ``size_t`` used in the libraries.

size_t
------

``size_t`` should be an integer type wide enough to hold the size of the 
largest memory object. So ``size_t`` really depends on the size of the 
underlying pointer type. For CPUs with 16-bit addressing, for example, 
the width of ``size_t`` should be 16-bits; for CPUs with 32-bit addressing, 
the width should be 32-bits.

``uint32_t``, of course, should always be 32-bits in width.

Using newlib header files, you also encounter incompatibilities between 
the definitions of some types, the types ``uint32_t`` and ``size_t`` is often 
the sources of problems. For example:

.. code-block:: bash

    error: redeclaration of 'typedef unsigned int std::size_t'

The root cause of this issue is that the community cannot decide on 
the correct definition of ``size_t``. NuttX uses this `flexible` 
definition of ``size_t``:

.. code-block:: c

    typedef uint32_t size_t;

It is `flexible` in the sense that ``uint32_t`` is determined by architecture 
specific header files, `not` the RTOS itself. That definition will be either 
``unsigned long`` or ``unsigned int``. So the ``size_t`` type compatibility 
can differ with different compilers and also with different architectures 
(NOTE that since ``size_t`` should be wide enough to hold the size of the 
largest addressable object. ``uint32_t`` only works for 32-bit addressable 
machines. Perhaps, ``size_t`` should really be defined as type ``uintptr_t``?).

This can be fixed by changing the definition of ``uint32_t`` as described 
above. But that could introduce ``uint32_t`` name mangling incompatibilities. 
In that case, you have no option but to decouple the definition of ``size_t`` 
from ``uint32_t`` by changing the definition in ``nuttx/include/sys/types.h`` 
to:

.. code-block:: c

    typedef unsigned int size_t;

or 

.. code-block:: c

    typedef unsigned long size_t;

whichever resolves the incompatibility.