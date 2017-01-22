/****************************************************************************
 * libc/dllfcn/lib_dlopen.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <dllfcn.h>

#include <nuttx/module.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dlopen
 *
 * Description:
 *   dlopen() makes an executable object file specified by file available to
 *   the calling program.  The class of files eligible for this operation and
 *   the manner of their construction are specified by the implementation,
 *   though typically such files are executable objects such as shared
 *   libraries, relocatable files or programs. Note that some implementations
 *   permit the construction of dependencies between such objects that are
 *   embedded within files. In such cases, a dlopen() operation will load
 *   such dependencies in addition to the object referenced by file.
 *   Implementations may also impose specific constraints on the construction
 *   of programs that can employ dlopen() and its related services.
 *
 *   If a file is specified in multiple dlopen() invocations, mode is
 *   interpreted at each invocation. Note, however, that once RTLD_NOW has
 *   been specified all relocations will have been completed rendering
 *   further RTLD_NOW operations redundant and any further RTLD_LAZY
 *   operations irrelevant.  Similarly note that once RTLD_GLOBAL has been
 *   specified the object will maintain the RTLD_GLOBAL status regardless
 *   of any previous or future specification of RTLD_LOCAL, so long as the
 *   object remains in the address space (see dlclose()).
 *
 *   Symbols introduced into a program through calls to dlopen() may be
 *   used in relocation activities. Symbols so introduced may duplicate
 *   symbols already defined by the program or previous dlopen()
 *   operations. To resolve the ambiguities such a situation might
 *   present, the resolution of a symbol reference to symbol definition is
 *   based on a symbol resolution order. Two such resolution orders are
 *   defined: load or dependency ordering. Load order establishes an
 *   ordering among symbol definitions, such that the definition first
 *   loaded (including definitions from the image file and any dependent
 *   objects loaded with it) has priority over objects added later (via
 *   dlopen()). Load ordering is used in relocation processing. Dependency
 *   ordering uses a breadth-first order starting with a given object,
 *   then all of its dependencies, then any dependents of those, iterating
 *   until all dependencies are satisfied. With the exception of the global
 *   symbol object obtained via a dlopen() operation on a file of 0,
 *   dependency ordering is used by the dlsym() function. Load ordering is
 *   used in dlsym() operations upon the global symbol object.
 *
 *   When an object is first made accessible via dlopen() it and its
 *   dependent objects are added in dependency order. Once all the objects
 *   are added, relocations are performed using load order. Note that if an
 *   object or its dependencies had been previously loaded, the load and
 *   dependency orders may yield different resolutions.
 *
 *   The symbols introduced by dlopen() operations, and available through
 *   dlsym() are at a minimum those which are exported as symbols of global
 *   scope by the object. Typically such symbols will be those that were
 *   specified in (for example) C source code as having extern linkage. The
 *   precise manner in which an implementation constructs the set of
 *   exported symbols for a dlopen() object is specified by that
 *   implementation.
 *
 * Input Parameters:
 *   file - Used to construct a pathname to the object file. If file
 *          contains a slash character, the file argument is used as the
 *          pathname for the file. Otherwise, file is used in an
 *          implementation-dependent manner to yield a pathname.
 *
 *          If the value of file is 0, dlopen() provides a handle on a
 *          global symbol object. This object provides access to the symbols
 *          from an ordered set of objects consisting of the original
 *          program image file, together with any objects loaded at program
 *          startup as specified by that process image file (for example,
 *          shared libraries), and the set of objects loaded using a
 *          dlopen() operation together with the RTLD_GLOBAL flag. As the
 *          latter set of objects can change during execution, the set
 *          identified by handle can also change dynamically.
 *
 *          Only a single copy of an object file is brought into the address
 *          space, even if dlopen() is invoked multiple times in reference
 *          to the file, and even if different pathnames are used to
 *          reference the file.
 *  mode  - Describes how dlopen() will operate upon file with respect to
 *          the processing of relocations and the scope of visibility of the
 *          symbols provided within file. When an object is brought into the
 *          address space of a process, it may contain references to symbols
 *          whose addresses are not known until the object is loaded. These
 *          references must be relocated before the symbols can be accessed.
 *          The mode parameter governs when these relocations take place.
 *          See definitions above for values of the mode parameter:.
 *
 * Returned Value:
 *   A successful dlopen() returns a handle which the caller may use on
 *   subsequent calls to dlsym() and dlclose(). The value of this handle
 *   should not be interpreted in any way by the caller.
 *
 *   If file cannot be found, cannot be opened for reading, is not of an
 *   appropriate object format for processing by dlopen(), or if an error
 *   occurs during the process of loading file or relocating its symbolic
 *   references, dlopen() will return NULL. More detailed diagnostic
 *   information will be available through dlerror().
 *
 * Reference: OpenGroup.org
 *
 ****************************************************************************/

FAR void *dlopen(FAR const char *file, int mode)
{
#if defined(CONFIG_BUILD_FLAT)
  /* In the FLAT build, a shared library is essentially the same as a kernel
   * module.
   *
   * REVIST:  Missing functionality:
   * - No automatic binding of symbols
   * - No dependencies
   * - mode is ignored.
   */

  return insmod(file, file);

#elif defined(CONFIG_BUILD_PROTECTED)
  /* The PROTECTED build is equivalent to the FLAT build EXCEPT that there
   * must be two copies of the the module logic:  One residing in kernel
   * space and using the kernel symbol table and one residing in user space
   * using the user space symbol table.
   *
   * The brute force way to accomplish this is by just copying the kernel
   * module code into libc/module.
   */

#warning Missing logic
  return NULL;

#else /* if defined(CONFIG_BUILD_KERNEL) */
  /* The KERNEL build is considerably more complex:  In order to be shared,
   * the .text portion of the module must be (1) build for PIC/PID operation
   * and (2) must like in a shared memory region accessible from all
   * processes.  The .data/.bss portion of the module must be allocated in
   * the user space of each process, but must lie at the same virtual address
   * so that it can be referenced from the one copy of the text in the shared
   * memory region.
   */

#warning Missing logic
  return NULL;
#endif
}
