/****************************************************************************
 * include/dlfcn.h
 *
 *   Copyright (C) 2017, 2019 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_DLFCN_H
#define __INCLUDE_DLFCN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The dlfcn.h header defines at least the following macros for use in the
 * construction of a dlopen() mode argument:
 *
 *   RTLD_LAZY   - Relocations are performed at an implementation-dependent
 *                 time, ranging from the time of the dlopen() call until
 *                 the first reference to a given symbol occurs. Specifying
 *                 RTLD_LAZY should improve performance on implementations
 *                 supporting dynamic symbol binding as a process may not
 *                 reference all of the functions in any given object. And,
 *                 for systems supporting dynamic symbol resolution for
 *                 normal process execution, this behaviour mimics the
 *                 normal handling of process execution.
 *   RTLD_NOW    - All necessary relocations are performed when the object
 *                 is first loaded. This may waste some processing if
 *                 relocations are performed for functions that are never
 *                 referenced. This behaviour may be useful for
 *                 applications that need to know as soon as an object is
 *                 loaded that all symbols referenced during execution will
 *                 be available.
 *
 * Any object loaded by dlopen() that requires relocations against global
 * symbols can reference the symbols in the original process image file,
 * any objects loaded at program startup, from the object itself as well as
 * any other object included in the same dlopen() invocation, and any
 * objects that were loaded in any dlopen() invocation and which specified
 * the RTLD_GLOBAL flag. To determine the scope of visibility for the
 * symbols loaded with a dlopen() invocation, the mode parameter should be
 * bitwise or'ed with one of the following values:
 *
 *   RTLD_GLOBAL - The object's symbols are made available for the
 *                 relocation processing of any other object. In addition,
 *                 symbol lookup using dlopen(0, mode) and an associated
 *                 dlsym() allows objects loaded with this mode to be
 *                 searched.
 *   RTLD_LOCAL  - All symbols are not made available for relocation
 *                 processing by other modules.
 *
 * Reference: OpenGroup.org
 */

#define RTLD_LAZY   (0 << 0)
#define RTLD_NOW    (1 << 0)
#define RTLD_GLOBAL (1 << 1)
#define RTLD_LOCAL  (1 << 2)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: dlsymtab
 *
 * Description:
 *   dlsymtab() is a non-standard shared library interface.  It selects the
 *   symbol table to use when binding a shared library to the base firmware
 *   which may be in FLASH memory.
 *
 * Input Parameters:
 *   symtab   - The new symbol table.
 *   nsymbols - The number of symbols in the symbol table.
 *
 * Returned Value:
 *   Always returns OK.
 *
 ****************************************************************************/

struct symtab_s;
int dlsymtab(FAR const struct symtab_s *symtab, int nsymbols);

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
 * ****************************************************************************/

FAR void *dlopen(FAR const char *file, int mode);

/****************************************************************************
 * Name: dlsym
 *
 * Description:
 *   dlsym() allows a process to obtain the address of a symbol defined
 *   within an object made accessible through a dlopen() call. handle is the
 *   value returned from a call to dlopen() (and which has not since been
 *   released via a call to dlclose()), name is the symbol's name as a
 *   character string.
 *
 *   dlsym() will search for the named symbol in all objects loaded
 *   automatically as a result of loading the object referenced by handle
 *   (see dlopen()). Load ordering is used in dlsym() operations upon the
 *   global symbol object. The symbol resolution algorithm used will be
 *   dependency order as described in dlopen().
 *
 * Input Parameters:
 *   handle - The opaque, non-NULL value returned by a previous successful
 *            call to dlopen().
 *   name   - A pointer to the symbol name string.
 *
 * Returned Value:
 *   If handle does not refer to a valid object opened by dlopen(), or if
 *   the named symbol cannot be found within any of the objects associated
 *   with handle, dlsym() will return NULL. More detailed diagnostic
 *   information will be available through dlerror().
 *
 * Reference: OpenGroup.org
 *
 ****************************************************************************/

FAR void *dlsym(FAR void *handle, FAR const char *name);

/****************************************************************************
 * Name: dlclose
 *
 * Description:
 *   dlclose() is used to inform the system that the object referenced by a
 *   handle returned from a previous dlopen() invocation is no longer needed
 *   by the application.
 *
 *   The use of dlclose() reflects a statement of intent on the part of the
 *   process, but does not create any requirement upon the implementation,
 *   such as removal of the code or symbols referenced by handle. Once an
 *   object has been closed using dlclose() an application should assume
 *   that its symbols are no longer available to dlsym(). All objects loaded
 *   automatically as a result of invoking dlopen() on the referenced object
 *   are also closed.
 *
 *   Although a dlclose() operation is not required to remove structures
 *   from an address space, neither is an implementation prohibited from
 *   doing so. The only restriction on such a removal is that no object will
 *   be removed to which references have been relocated, until or unless all
 *   such references are removed. For instance, an object that had been
 *   loaded with a dlopen() operation specifying the RTLD_GLOBAL flag might
 *   provide a target for dynamic relocations performed in the processing of
 *   other objects - in such environments, an application may assume that no
 *   relocation, once made, will be undone or remade unless the object
 *   requiring the relocation has itself been removed.
 *
 * Input Parameters:
 *   handle - The opaque, non-NULL value returned by a previous successful
 *            call to dlopen().
 *
 * Returned Value:
 *   If the referenced object was successfully closed, dlclose() returns 0.
 *   If the object could not be closed, or if handle does not refer to an
 *   open object, dlclose() returns a non-zero value. More detailed
 *   diagnostic information will be available through dlerror().
 *
 * Reference: OpenGroup.org
 *  ****************************************************************************/

int dlclose(FAR void *handle);

/****************************************************************************
 * Name: dlerror
 *
 * Description:
 *   dlerror() returns a null-terminated character string (with no trailing
 *   newline) that describes the last error that occurred during dynamic
 *   linking processing. If no dynamic linking errors have occurred since
 *   the last invocation of dlerror(), dlerror() returns NULL. Thus,
 *   invoking dlerror() a second time, immediately following a prior
 *   invocation, will result in NULL being returned.
 *
 * Input Parameters:
 *   If successful, dlerror() returns a null-terminated character string.
 *   Otherwise, NULL is returned.
 *
 * Returned Value:
 *
 * Reference: OpenGroup.org
 *
 ****************************************************************************/

FAR char *dlerror(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_DLFCN_H */
