========
Fortify
========

Overview
--------

A common error in C programs is invoking functions that might exceed memory bounds,  
causing crashes or undefined behavior. Examples include incorrect usage of functions like  
``memcpy`` and ``memset``. `FORTIFY_SOURCE` is a mechanism designed to help developers quickly  
detect and mitigate boundary-related issues caused by improper use of library functions.

Support
-------

`FORTIFY_SOURCE` is implemented as a software check by the compiler and is supported across all architectures.  
It works by adding additional validation checks to standard library function calls.

Usage
-----

To enable `FORTIFY_SOURCE`, configure the kernel with the following option:

``CONFIG_FORTIFY_SOURCE=level``

Where `level` can be set as:

1. **Compile-time Checks**:  
   Detects issues during compilation by analyzing source code.

2. **Stack Variable Checks**:  
   Extends level 1 by checking stack variables at runtime.

3. **Heap Memory Checks**:  
   Builds on level 2 by adding checks for memory allocated with ``malloc``.  
   (Requires GCC version 12 or later.)

FORTIFY_SOURCE Overview
=======================

`FORTIFY_SOURCE` detects potential security vulnerabilities by statically analyzing source code at compile time.  
It replaces standard library function calls with safer versions that include additional boundary checks.  
These safer versions validate the operation's boundaries and the input's validity before performing certain operations.

GCC Built-in Functions
-----------------------

The GCC compiler internally implements two key functions for `FORTIFY_SOURCE`:  

- ``__builtin_object_size``: Determines the size of a statically allocated object.  
- ``__builtin_dynamic_object_size``: Determines the size of dynamically allocated objects (e.g., via ``malloc``).  

Starting with GCC 12, these functions support retrieving the size of variables allocated with ``malloc``.

By passing a variable or buffer as an argument to these functions, the compiler can compute the corresponding size.  
Using this size, it is possible to check for potential out-of-bounds behavior in runtime operations.

Example: memcpy Implementation in NuttX
----------------------------------------

The following example demonstrates how `FORTIFY_SOURCE` can be used to enhance security in a ``memcpy``  
implementation in NuttX:

.. code-block:: c

   fortify_function(memcpy) 
   FAR void *memcpy(FAR void *dest,
                    FAR const void *src,
                    size_t n)
   {
     fortify_assert(n <= fortify_size(dest, 0) && n <= fortify_size(src, 0));
     return __real_memcpy(dest, src, n);
   }

In this implementation, the ``fortify_assert`` macro ensures that the size of the source and destination buffers  
is sufficient to handle the requested memory operation. If the assertion fails, it indicates a potential buffer  
overflow, helping developers quickly identify and address such vulnerabilities.
