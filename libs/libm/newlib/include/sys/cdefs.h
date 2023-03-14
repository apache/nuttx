/****************************************************************************
 * libs/libm/newlib/include/sys/cdefs.h
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

#ifndef __LIBS_LIBM_NEWLIB_INCLUDE_SYS_CDEFS_H
#define __LIBS_LIBM_NEWLIB_INCLUDE_SYS_CDEFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/features.h>
#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define __PMT(args)         args
#define __DOTS              , ...
#define __THROW

#ifdef __GNUC__
# define __ASMNAME(cname)   __XSTRING(__USER_LABEL_PREFIX__) cname
#endif

#define __ptr_t             void *
#define __long_double_t     long double

#define __attribute_malloc__
#define __attribute_pure__
#define __attribute_format_strfmon__(a, b)
#define __flexarr           [0]

#ifndef __BOUNDED_POINTERS__
# define __bounded          /* nothing */
# define __unbounded        /* nothing */
# define __ptrvalue         /* nothing */
#endif

#ifndef __has_attribute
#define __has_attribute(x)  0
#endif
#ifndef __has_extension
#define __has_extension     __has_feature
#endif
#ifndef __has_feature
#define __has_feature(x)    0
#endif
#ifndef __has_include
#define __has_include(x)    0
#endif
#ifndef __has_builtin
#define __has_builtin(x)    0
#endif

#if defined(__cplusplus)
#define __BEGIN_DECLS   extern "C" {
#define __END_DECLS     }
#else
#define __BEGIN_DECLS
#define __END_DECLS
#endif

#if defined(__GNUC__)

#if __GNUC__ >= 3
#define __GNUCLIKE_ASM                      3
#define __GNUCLIKE_MATH_BUILTIN_CONSTANTS
#else
#define __GNUCLIKE_ASM                      2
#endif
#define __GNUCLIKE___TYPEOF                 1
#define __GNUCLIKE___SECTION                1

#define __GNUCLIKE_CTOR_SECTION_HANDLING    1

#define __GNUCLIKE_BUILTIN_CONSTANT_P       1

#if (__GNUC_MINOR__ > 95 || __GNUC__ >= 3)
#define __GNUCLIKE_BUILTIN_VARARGS          1
#define __GNUCLIKE_BUILTIN_STDARG           1
#define __GNUCLIKE_BUILTIN_VAALIST          1
#endif

#define __GNUC_VA_LIST_COMPATIBILITY        1

#define __compiler_membar()  __asm __volatile(" " : : : "memory")

#define __GNUCLIKE_BUILTIN_NEXT_ARG         1
#define __GNUCLIKE_MATH_BUILTIN_RELOPS

#define __GNUCLIKE_BUILTIN_MEMCPY           1

#define __CC_SUPPORTS_INLINE                1
#define __CC_SUPPORTS___INLINE              1
#define __CC_SUPPORTS___INLINE__            1

#define __CC_SUPPORTS___FUNC__              1
#define __CC_SUPPORTS_WARNING               1

#define __CC_SUPPORTS_VARADIC_XXX           1 /* see varargs.h */

#define __CC_SUPPORTS_DYNAMIC_ARRAY_INIT    1

#endif /* __GNUC__ */

#if defined(__STDC__) || defined(__cplusplus)
#define __P(protos)         protos /* full-blown ANSI C */
#define __CONCAT1(x, y)     x ## y
#define __CONCAT(x, y)      __CONCAT1(x, y)
#define __STRING(x)         #x          /* stringify without expanding x */
#define __XSTRING(x)        __STRING(x) /* expand x, then stringify */

#define __const     const               /* define reserved names to standard
                                         * */
#define __signed    signed
#define __volatile  volatile
#if defined(__cplusplus)
#define __inline    inline      /* convert to C++ keyword */
#else
#if !(defined(__CC_SUPPORTS___INLINE))
#define __inline    /* delete GCC keyword */
#endif /* ! __CC_SUPPORTS___INLINE */
#endif /* !__cplusplus */

#else                       /* !(__STDC__ || __cplusplus) */
#define __P(protos)     ()  /* traditional C preprocessor */
#define __CONCAT(x, y)  x##y
#define __STRING(x)     "x"

#if !defined(__CC_SUPPORTS___INLINE)
#define __const     /* delete pseudo-ANSI C keywords */
#define __inline
#define __signed
#define __volatile

#ifndef NO_ANSI_KEYWORDS
#define const       /* delete ANSI C keywords */
#define inline
#define signed
#define volatile
#endif  /* !NO_ANSI_KEYWORDS */
#endif  /* !__CC_SUPPORTS___INLINE */
#endif  /* !(__STDC__ || __cplusplus) */

#define __weak_symbol   __attribute__((__weak__))
#if !__GNUC_PREREQ__(2, 5)
#define __dead2
#define __pure2
#define __unused
#endif
#if __GNUC__ == 2 && __GNUC_MINOR__ >= 5 && __GNUC_MINOR__ < 7
#define __dead2         __attribute__((__noreturn__))
#define __pure2         __attribute__((__const__))
#define __unused

/* XXX Find out what to do for __packed, __aligned and __section */
#endif
#if __GNUC_PREREQ__(2, 7)
#define __dead2         __attribute__((__noreturn__))
#define __pure2         __attribute__((__const__))
#define __unused        __attribute__((__unused__))
#define __used          __attribute__((__used__))
#define __packed        __attribute__((__packed__))
#define __aligned(x)            __attribute__((__aligned__(x)))
#define __section(x)            __attribute__((__section__(x)))
#endif
#if __GNUC_PREREQ__(4, 3) || __has_attribute(__alloc_size__)
#define __alloc_size(x)         __attribute__((__alloc_size__(x)))
#define __alloc_size2(n, x)     __attribute__((__alloc_size__(n, x)))
#else
#define __alloc_size(x)
#define __alloc_size2(n, x)
#endif
#if __GNUC_PREREQ__(4, 9) || __has_attribute(__alloc_align__)
#define __alloc_align(x)        __attribute__((__alloc_align__(x)))
#else
#define __alloc_align(x)
#endif

#if !__GNUC_PREREQ__(2, 95)
#define __alignof(x)            __offsetof(struct { char __a;x __b; }, __b)
#endif

/* Keywords added in C11.
 */

#if !defined(__STDC_VERSION__) || __STDC_VERSION__ < 201112L

#if !__has_extension(c_alignas)
#if (defined(__cplusplus) && __cplusplus >= 201103L) || \
  __has_extension(cxx_alignas)
#define _Alignas(x)     alignas(x)
#else

/* XXX: Only emulates _Alignas(constant-expression);
 *      not _Alignas(type-name).
 */

#define _Alignas(x)     __aligned(x)
#endif
#endif

#if defined(__cplusplus) && __cplusplus >= 201103L
#define _Alignof(x)     alignof(x)
#else
#define _Alignof(x)     __alignof(x)
#endif

#if !defined(__cplusplus) && !__has_extension(c_atomic) && \
  !__has_extension(cxx_atomic) && !__GNUC_PREREQ__(4, 7)

/* No native support for _Atomic(). Place object in structure to prevent
 * most forms of direct non-atomic access.
 */

#define _Atomic(T)      struct { T volatile __val; }
#endif

#if defined(__cplusplus) && __cplusplus >= 201103L
#define _Noreturn   [[noreturn]]
#else
#define _Noreturn   __dead2
#endif

#if !__has_extension(c_static_assert)
#if (defined(__cplusplus) && __cplusplus >= 201103L) || \
  __has_extension(cxx_static_assert)
#define _Static_assert(x, y)    static_assert(x, y)
#elif __GNUC_PREREQ__(4, 6) && !defined(__cplusplus)

/* Nothing, gcc 4.6 and higher has _Static_assert built-in */
#elif defined(__COUNTER__)
#define _Static_assert(x, y)    __Static_assert(x, __COUNTER__)
#define __Static_assert(x, y)   ___Static_assert(x, y)
#define ___Static_assert(x, y)  typedef char __assert_ ## y [(x) ? 1 : -1] \
    __unused
#else
#define _Static_assert(x, y)    struct __hack
#endif
#endif

#if !__has_extension(c_thread_local)

/* XXX: Some compilers (Clang 3.3, GCC 4.7) falsely announce C++11 mode
 * without actually supporting the thread_local keyword. Don't check for
 * the presence of C++11 when defining _Thread_local.
 */
#if /* (defined(__cplusplus) && __cplusplus >= 201103L) || */ \
  __has_extension(cxx_thread_local)
#define _Thread_local thread_local
#else
#define _Thread_local   __thread
#endif
#endif

#endif /* __STDC_VERSION__ || __STDC_VERSION__ < 201112L */

/* Emulation of C11 _Generic().  Unlike the previously defined C11
 * keywords, it is not possible to implement this using exactly the same
 * syntax.  Therefore implement something similar under the name
 * __generic().  Unlike _Generic(), this macro can only distinguish
 * between a single type, so it requires nested invocations to
 * distinguish multiple cases.
 */

#if (defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L) || \
  __has_extension(c_generic_selections)
#define __generic(expr, t, yes, no) \
  _Generic(expr, t:                 \
           yes, default: no)
#elif __GNUC_PREREQ__(3, 1) && !defined(__cplusplus)
#define __generic(expr, t, yes, no)                                  \
  __builtin_choose_expr(__builtin_types_compatible_p(__typeof(expr), \
                                                     t), yes, no)
#endif

/* C99 Static array indices in function parameter declarations.  Syntax such
 *as:
 * void bar(int myArray[static 10]);
 * is allowed in C99 but not in C++.  Define __min_size appropriately so
 * headers using it can be compiled in either language.  Use like this:
 * void bar(int myArray[__min_size(10)]);
 */
#if !defined(__cplusplus) &&                       \
  (defined(__clang__) || __GNUC_PREREQ__(4, 6)) && \
  (!defined(__STDC_VERSION__) || (__STDC_VERSION__ >= 199901))
#define __min_size(x)   static (x)
#else
#define __min_size(x)   (x)
#endif

#if __GNUC_PREREQ__(2, 96)
#define __malloc_like       __attribute__((__malloc__))
#define __pure              __attribute__((__pure__))
#else
#define __malloc_like
#define __pure
#endif

#if __GNUC_PREREQ__(3, 1)
#define __always_inline     __inline__ __attribute__((__always_inline__))
#else
#define __always_inline
#endif

#if __GNUC_PREREQ__(3, 1)
#define __noinline          __attribute__ ((__noinline__))
#else
#define __noinline
#endif

#if __GNUC_PREREQ__(3, 3)
#define __nonnull(x)  __attribute__((__nonnull__ x))
#define __nonnull_all       __attribute__((__nonnull__))
#else
#define __nonnull(x)
#define __nonnull_all
#endif

#if __GNUC_PREREQ__(3, 4)
#define __fastcall          __attribute__((__fastcall__))
#define __result_use_check  __attribute__((__warn_unused_result__))
#else
#define __fastcall
#define __result_use_check
#endif

#if __GNUC_PREREQ__(4, 1)
#define __returns_twice     __attribute__((__returns_twice__))
#else
#define __returns_twice
#endif

#if __GNUC_PREREQ__(4, 6) || __has_builtin(__builtin_unreachable)
#define __unreachable()     __builtin_unreachable()
#else
#define __unreachable()     ((void)0)
#endif

/* XXX: should use `#if __STDC_VERSION__ < 199901'. */
#if !__GNUC_PREREQ__(2, 7)
#define __func__    NULL
#endif

/* GCC 2.95 provides `__restrict' as an extension to C90 to support the
 * C99-specific `restrict' type qualifier.  We happen to use `__restrict' as
 * a way to define the `restrict' type qualifier without disturbing older
 * software that is unaware of C99 keywords.
 */
#if !(__GNUC__ == 2 && __GNUC_MINOR__ == 95)
#if !defined(__STDC_VERSION__) || __STDC_VERSION__ < 199901
#define __restrict
#else
#define __restrict  restrict
#endif
#endif

/* GNU C version 2.96 adds explicit branch prediction so that
 * the CPU back-end can hint the processor and also so that
 * code blocks can be reordered such that the predicted path
 * sees a more linear flow, thus improving cache behavior, etc.
 *
 * The following two macros provide us with a way to utilize this
 * compiler feature.  Use __predict_true() if you expect the expression
 * to evaluate to true, and __predict_false() if you expect the
 * expression to evaluate to false.
 *
 * A few notes about usage:
 *
 *  * Generally, __predict_false() error condition checks (unless
 *    you have some _strong_ reason to do otherwise, in which case
 *    document it), and/or __predict_true() `no-error' condition
 *    checks, assuming you want to optimize for the no-error case.
 *
 *  * Other than that, if you don't know the likelihood of a test
 *    succeeding from empirical or other `hard' evidence, don't
 *    make predictions.
 *
 *  * These are meant to be used in places that are run `a lot'.
 *    It is wasteful to make predictions in code that is run
 *    seldomly (e.g. at subsystem initialization time) as the
 *    basic block reordering that this affects can often generate
 *    larger code.
 */
#if __GNUC_PREREQ__(2, 96)
#define __predict_true(exp)     __builtin_expect((exp), 1)
#define __predict_false(exp)    __builtin_expect((exp), 0)
#else
#define __predict_true(exp)     (exp)
#define __predict_false(exp)    (exp)
#endif

#if __GNUC_PREREQ__(4, 0)
#define __null_sentinel     __attribute__((__sentinel__))
#define __exported          __attribute__((__visibility__("default")))

/* Only default visibility is supported on PE/COFF targets. */

#ifndef __CYGWIN__
#define __hidden            __attribute__((__visibility__("hidden")))
#else
#define __hidden
#endif
#else
#define __null_sentinel
#define __exported
#define __hidden
#endif

#define __offsetof(type, field)     offsetof(type, field)
#define __rangeof(type, start, end) \
  (__offsetof(type, end) - __offsetof(type, start))

/* Given the pointer x to the member m of the struct s, return
 * a pointer to the containing structure.  When using GCC, we first
 * assign pointer x to a local variable, to check that its type is
 * compatible with member m.
 */

#if __GNUC_PREREQ__(3, 1)
#define __containerof(x, s, m)      ({                               \
    const volatile __typeof(((s *)0)->m) * __x = (x);                \
    __DEQUALIFY(s *, (const volatile char *)__x - __offsetof(s, m)); \
  })
#else
#define __containerof(x, s, m) \
  __DEQUALIFY(s *, (const volatile char *)(x) - __offsetof(s, m))
#endif

/* Compiler-dependent macros to declare that functions take printf-like
 * or scanf-like arguments.  They are null except for versions of gcc
 * that are known to support the features properly (old versions of gcc-2
 * didn't permit keeping the keywords out of the application namespace).
 */
#if !__GNUC_PREREQ__(2, 7)
#define __printflike(fmtarg, firstvararg)
#define __scanflike(fmtarg, firstvararg)
#define __format_arg(fmtarg)
#define __strfmonlike(fmtarg, firstvararg)
#define __strftimelike(fmtarg, firstvararg)
#else
#define __printflike(fmtarg, firstvararg) \
  __attribute__((__format__(__printf__, fmtarg, firstvararg)))
#define __scanflike(fmtarg, firstvararg) \
  __attribute__((__format__(__scanf__, fmtarg, firstvararg)))
#define __format_arg(fmtarg)  __attribute__((__format_arg__(fmtarg)))
#define __strfmonlike(fmtarg, firstvararg) \
  __attribute__((__format__(__strfmon__, fmtarg, firstvararg)))
#define __strftimelike(fmtarg, firstvararg) \
  __attribute__((__format__(__strftime__, fmtarg, firstvararg)))
#endif

/* Compiler-dependent macros that rely on FreeBSD-specific extensions. */
#if defined(__FreeBSD_cc_version) && __FreeBSD_cc_version >= 300001 && \
  defined(__GNUC__)
#define __printf0like(fmtarg, firstvararg) \
  __attribute__((__format__(__printf0__, fmtarg, firstvararg)))
#else
#define __printf0like(fmtarg, firstvararg)
#endif

#if defined(__GNUC__)
#define __strong_reference(sym, aliassym) \
  extern __typeof (sym) aliassym __attribute__ ((__alias__(#sym)))
#ifdef __ELF__
#ifdef __STDC__
#define __weak_reference(sym, alias) \
  __asm__ (".weak " #alias);         \
  __asm__ (".equ "  #alias ", " #sym)
#define __warn_references(sym, msg)        \
  __asm__ (".section .gnu.warning." #sym); \
  __asm__ (".asciz \"" msg "\"");          \
  __asm__ (".previous")
#define __sym_compat(sym, impl, verid) \
  __asm__ (".symver " #impl ", " #sym "@" #verid)
#define __sym_default(sym, impl, verid) \
  __asm__ (".symver " #impl ", " #sym "@@" #verid)
#else
#define __weak_reference(sym, alias) \
  __asm__ (".weak alias");           \
  __asm__ (".equ alias, sym")
#define __warn_references(sym, msg)      \
  __asm__ (".section .gnu.warning.sym"); \
  __asm__ (".asciz \"msg\"");            \
  __asm__ (".previous")
#define __sym_compat(sym, impl, verid) \
  __asm__ (".symver impl, sym@verid")
#define __sym_default(impl, sym, verid) \
  __asm__ (".symver impl, sym@@verid")
#endif  /* __STDC__ */
#else   /* !__ELF__ */
#ifdef __STDC__
#define __weak_reference(sym, alias)           \
  __asm__ (".stabs \"_" #alias "\",11,0,0,0"); \
  __asm__ (".stabs \"_" #sym "\",1,0,0,0")
#define __warn_references(sym, msg)        \
  __asm__ (".stabs \"" msg "\",30,0,0,0"); \
  __asm__ (".stabs \"_" #sym "\",1,0,0,0")
#else
#define __weak_reference(sym, alias)          \
  __asm__ (".stabs \"_/**/alias\",11,0,0,0"); \
  __asm__ (".stabs \"_/**/sym\",1,0,0,0")
#define __warn_references(sym, msg) \
  __asm__ (".stabs msg,30,0,0,0");  \
  __asm__ (".stabs \"_/**/sym\",1,0,0,0")
#endif  /* __STDC__ */
#endif  /* __ELF__ */
#endif  /* __GNUC__ */

#ifndef __FBSDID
#define __FBSDID(s)             struct __hack
#endif

#ifndef __RCSID
#define __RCSID(s)              struct __hack
#endif

#ifndef __RCSID_SOURCE
#define __RCSID_SOURCE(s)       struct __hack
#endif

#ifndef __SCCSID
#define __SCCSID(s)             struct __hack
#endif

#ifndef __COPYRIGHT
#define __COPYRIGHT(s)          struct __hack
#endif

#ifndef __DECONST
#define __DECONST(type, var)    ((type)(__uintptr_t)(const void *)(var))
#endif

#ifndef __DEVOLATILE
#define __DEVOLATILE(type, \
                     var)       ((type)(__uintptr_t)(volatile void *)(var))
#endif

#ifndef __DEQUALIFY
#define __DEQUALIFY(type,                                                    \
                    var)        ((type)(__uintptr_t)(const volatile void *)( \
                                   var))
#endif

/* Nullability qualifiers: currently only supported by Clang.
 */
#if !(defined(__clang__) && __has_feature(nullability))
#define _Nonnull
#define _Nullable
#define _Null_unspecified
#define __NULLABILITY_PRAGMA_PUSH
#define __NULLABILITY_PRAGMA_POP
#else
#define __NULLABILITY_PRAGMA_PUSH   _Pragma("clang diagnostic push") \
  _Pragma("clang diagnostic ignored \"-Wnullability-completeness\"")
#define __NULLABILITY_PRAGMA_POP    _Pragma("clang diagnostic pop")
#endif

/* Type Safety Checking
 *
 * Clang provides additional attributes to enable checking type safety
 * properties that cannot be enforced by the C type system.
 */

#if __has_attribute(__argument_with_type_tag__) && \
  __has_attribute(__type_tag_for_datatype__)
#define __arg_type_tag(arg_kind, arg_idx, type_tag_idx) \
  __attribute__((__argument_with_type_tag__(arg_kind, arg_idx, type_tag_idx)))
#define __datatype_type_tag(kind, type) \
  __attribute__((__type_tag_for_datatype__(kind, type)))
#else
#define __arg_type_tag(arg_kind, arg_idx, type_tag_idx)
#define __datatype_type_tag(kind, type)
#endif

/* Lock annotations.
 *
 * Clang provides support for doing basic thread-safety tests at
 * compile-time, by marking which locks will/should be held when
 * entering/leaving a functions.
 *
 * Furthermore, it is also possible to annotate variables and structure
 * members to enforce that they are only accessed when certain locks are
 * held.
 */

#if __has_extension(c_thread_safety_attributes)
#define __lock_annotate(x)  __attribute__((x))
#else
#define __lock_annotate(x)
#endif

/* Structure implements a lock. */

/* FIXME: Use __lockable__, etc. to avoid colliding with user namespace
 * macros,
 * once clang is fixed: https://bugs.llvm.org/show_bug.cgi?id=34319
 */

#define __lockable __lock_annotate(lockable)

/* Function acquires an exclusive or shared lock. */
#define __locks_exclusive(...) \
  __lock_annotate(exclusive_lock_function(__VA_ARGS__))
#define __locks_shared(...) \
  __lock_annotate(shared_lock_function(__VA_ARGS__))

/* Function attempts to acquire an exclusive or shared lock. */
#define __trylocks_exclusive(...) \
  __lock_annotate(exclusive_trylock_function(__VA_ARGS__))
#define __trylocks_shared(...) \
  __lock_annotate(shared_trylock_function(__VA_ARGS__))

/* Function releases a lock. */
#define __unlocks(...)  __lock_annotate(unlock_function(__VA_ARGS__))

/* Function asserts that an exclusive or shared lock is held. */
#define __asserts_exclusive(...) \
  __lock_annotate(assert_exclusive_lock(__VA_ARGS__))
#define __asserts_shared(...) \
  __lock_annotate(assert_shared_lock(__VA_ARGS__))

/* Function requires that an exclusive or shared lock is or is not held. */
#define __requires_exclusive(...) \
  __lock_annotate(exclusive_locks_required(__VA_ARGS__))
#define __requires_shared(...) \
  __lock_annotate(shared_locks_required(__VA_ARGS__))
#define __requires_unlocked(...) \
  __lock_annotate(locks_excluded(__VA_ARGS__))

/* Function should not be analyzed. */
#define __no_lock_analysis      __lock_annotate(no_thread_safety_analysis)

/* Function or variable should not be sanitized, e.g., by AddressSanitizer.
 * GCC has the nosanitize attribute, but as a function attribute only, and
 * warns on use as a variable attribute.
 */
#if __has_attribute(no_sanitize) && defined(__clang__)
#ifdef _KERNEL
#define __nosanitizeaddress     __attribute__((no_sanitize("kernel-address")))
#define __nosanitizememory      __attribute__((no_sanitize("kernel-memory")))
#else
#define __nosanitizeaddress     __attribute__((no_sanitize("address")))
#define __nosanitizememory      __attribute__((no_sanitize("memory")))
#endif
#define __nosanitizethread      __attribute__((no_sanitize("thread")))
#else
#define __nosanitizeaddress
#define __nosanitizememory
#define __nosanitizethread
#endif

/* Guard variables and structure members by lock. */
#define __guarded_by(x)     __lock_annotate(guarded_by(x))
#define __pt_guarded_by(x)  __lock_annotate(pt_guarded_by(x))

/* Alignment builtins for better type checking and improved code generation.
 * Provide fallback versions for other compilers (GCC/Clang < 10):
 */
#if !__has_builtin(__builtin_is_aligned)
#define __builtin_is_aligned(x, align) \
  (((__uintptr_t)x & ((align) - 1)) == 0)
#endif
#if !__has_builtin(__builtin_align_up)
#define __builtin_align_up(x, align) \
  ((__typeof__(x))(((__uintptr_t)(x) + ((align) - 1)) & (~((align) - 1))))
#endif
#if !__has_builtin(__builtin_align_down)
#define __builtin_align_down(x, align) \
  ((__typeof__(x))((x) & (~((align) - 1))))
#endif

#define __align_up(x, y)    __builtin_align_up(x, y)
#define __align_down(x, y)  __builtin_align_down(x, y)
#define __is_aligned(x, y)  __builtin_is_aligned(x, y)

#endif /* __LIBS_LIBM_NEWLIB_INCLUDE_SYS_CDEFS_H */
