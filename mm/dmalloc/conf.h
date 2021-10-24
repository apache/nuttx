/* conf.h.  Generated from conf.h.in by configure.  */
/*
 * Automatic configuration flags
 *
 * Copyright 2020 by Gray Watson
 *
 * This file is part of the dmalloc package.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose and without fee is hereby granted, provided
 * that the above copyright notice and this permission notice appear
 * in all copies, and that the name of Gray Watson not be used in
 * advertising or publicity pertaining to distribution of the document
 * or software without specific, written prior permission.
 * 
 * Gray Watson makes no representations about the suitability of the
 * software described herein for any purpose.  It is provided "as is"
 * without express or implied warranty.
 *
 * The author may be contacted via http://dmalloc.com/
 */

#ifndef __CONF_H__
#define __CONF_H__

#include <nuttx/config.h>

/* please see settings.h for manual configuration options */

/*
 * NOTE: The following settings should not need to be tuned by hand.
 */

/*
 * Set to 1 if the mprotect function was found and the PROT_NONE,
 * PROT_READ, and PROT_WRITE defines were found in sys/mman.h.  This
 * is so that we can restrict access to certain blocks of memory.
 */
#define PROTECT_ALLOWED 1

/*
 * (char *)sbrk(const int incr) is the main heap-memory allocation
 * routine that most systems employ.  This extends the program's data
 * space by INCR number of bytes.
 *
 * NOTE: If configure generates a 0 for this and HAVE_MMAP on your
 * system, you should see the INTERNAL_MEMORY_SPACE setting in the
 * settings.h file which is created from the settings.dist file.
 */
#define HAVE_SBRK 1
void *sbrk(intptr_t incr);

/*
 * (void *)mmap(...) is another heap-memory allocation routine that
 * systems employ.  On newer systems it is often preferable over sbrk.
 * It allocates a block of memory in the virtual-memory system.  The
 * USE_MMAP define is set if the standard mmap call works.
 *
 * NOTE: If configure generates a 0 for this and HAVE_SBRK on your
 * system, you should see the INTERNAL_MEMORY_SPACE setting in the
 * settings.h file which is created from the settings.dist file.
 */
#define HAVE_MMAP 1
#define USE_MMAP 0
#define HAVE_MUNMAP 1

/*
 * This is the basic block size in bits.  If possible, the configure
 * script will set this to be the value returned by the getpagesize()
 * function.  If not then some sort of best guess will be necessary.
 * 15 (meaning basic block size of 32k) will probably be good.
 *
 * NOTE: some sbrk functions round to the correct page-size.  No
 * problems aside from a possible small increase in the administration
 * overhead should happen if this value is too high.
 */
#ifdef CONFIG_MM_PGSIZE
#define BASIC_BLOCK CONFIG_MM_PGSIZE
#else
#define BASIC_BLOCK 12
#endif

/*
 * The alignment value of all allocations in number of bytes for
 * loading admin information before an allocation.  If possible, the
 * configure script will set this to be the value returned by
 * sizeof(long) which in most systems is the register width.
 *
 * NOTE: the value will never be auto-configured to be less than 8
 * because some system (like sparc for instance) report the sizeof(long)
 * == 4 while the register size is 8 bytes.  Certain memory needs to be of
 * the same base as the register size (stack frames, code, etc.).  Any
 * ideas how I can determine the register size in a better (and portable)
 * fashion?
 *
 * NOTE: larger the number the more memory may be wasted by certain
 * debugging settings like fence-post checking.
 */
#define ALLOCATION_ALIGNMENT 8

/*
 * When doing pointer arithmatic and other long value manipulation,
 * what type should we use.  Hopefully we get a 64-bit value here.
 */
#define PNT_ARITH_TYPE unsigned long

/*
 * This checks to see if the abort routine does extensive cleaning up
 * before halting a program.  If so then it may call malloc functions
 * making the library go recursive.  If abort is set to not okay then
 * you should tune the KILL_PROCESS and SIGNAL_INCLUDE options in
 * settings.h if you want the library to be able to dump core.
 */
#define ABORT_OKAY 1

/*
 * This checks to see if we can include signal.h and get SIGHUP,
 * SIGINT, and SIGTERM for the catch-signals token.  With this token,
 * you can have the library do an automatic shutdown if we see the
 * above signals.
 */
#define SIGNAL_OKAY 0
#define RETSIGTYPE void

/*
 * This checks to see if we can include return.h and use the assembly
 * macros there to call the callers address for logging.  If you do
 * not want this behavior, then set the USE_RETURN_MACROS to 0 in the
 * settings.h file.
 */
#define RETURN_MACROS_WORK 1

/*
 * Why can't the compiler folks agree about this.  I really hate Unix
 * sometimes for its blatant disregard for anything approaching a
 * standard.
 */
#define IDENT_WORKS 1

/*
 * Which pthread include file to use.
 */
#define HAVE_PTHREAD_H 1
#define HAVE_PTHREADS_H 0

/*
 * What pthread functions do we have?
 */
#define HAVE_PTHREAD_MUTEX_INIT 1
#define HAVE_PTHREAD_MUTEX_LOCK 1
#define HAVE_PTHREAD_MUTEX_UNLOCK 1

/*
 * What is the pthread mutex type?  Usually (always?) it is
 * pthread_mutex_t.
 */
#define THREAD_MUTEX_T sem_t

/*
 * On some systems, you initialize mutex variables with NULL.  Others
 * require various stupid non-portable incantations.  The OSF 3.2 guys
 * should be ashamed of themselves.  This only is used if the
 * LOCK_THREADS setting is enabled in the settings.h.
 */
#define THREAD_LOCK_INIT_VAL 1

/*
 * Under the Cygwin environment, when malloc calls getenv, it core
 * dumps.  This is because Cygwin, as far as I know, is loading the
 * shared libraries for the various system functions and goes
 * recursive with a call to getenv.  Ugh.
 *
 * So we have to delay the getenv call.  This sets when we can do the
 * getenv call so the environmental processing is delayed.
 */
#define GETENV_SAFE 1

/*
 * See whether support exists for the constructor attribute which
 * allows the library to run code before main() is called.  I know
 * that later versions of gcc have support for this and maybe other
 * compilers do as well.
 */
#define CONSTRUCTOR_WORKS 0

/*
 * See whether support exists for the destructor attribute which
 * allows the library to run code after main() is exited.  I know
 * that later versions of gcc have support for this and maybe other
 * compilers do as well.
 */
#define DESTRUCTOR_WORKS 0

/*
 * See if we have the GetEnvironmentVariableA Cygwin function.  This
 * is used as a getenv replacement.
 */
#define HAVE_GETENVIRONMENTVARIABLEA 0

/*
 * LIBRARY DEFINES:
 */

/*
 * Whether the compiler and OS has standard C headers.
 */
#define STDC_HEADERS 1

/*
 * Some systems have functions which can register routines to be
 * called by exit(3) (or when the program returns from main).  This
 * functionality allows the dmalloc_shutdown() routine to be called
 * automatically upon program completion so that the library can log
 * statistics.  Use the AUTO_SHUTDOWN define above to disable this.
 * Please send me mail if this functionality exists on your system but
 * in another name.
 *
 * NOTE: If neither is available, take a look at atexit.c in the
 * contrib directory which may provide this useful functionality for
 * your system.
 */
#define HAVE_ATEXIT CONFIG_SCHED_ATEXIT
#define HAVE_ON_EXIT CONFIG_SCHED_ONEXIT

/* Is the DMALLOC_SIZE type unsigned? */
#define DMALLOC_SIZE_UNSIGNED 1

/*
 * The dmalloc library provides its own versions of the following
 * functions, or knows how to work around their absence.
 */
/* bells and whistles */
#define HAVE_FORK 0
#define HAVE_GETHOSTNAME 1
#define HAVE_GETPID 1
#define HAVE_GETUID 1
#define HAVE_TIME 1
#define HAVE_CTIME 1

#define HAVE_VPRINTF 1
#define HAVE_SNPRINTF 1
#define HAVE_VSNPRINTF 1

#define HAVE_RECALLOC 0
#define HAVE_MEMALIGN 1
#define HAVE_VALLOC 1

#ifdef CONFIG_SCHED_BACKTRACE
#define HAVE_BACKTRACE 1
#else
#define HAVE_BACKTRACE 0
#endif

/* various functions for arg checking and/or internal use */

#define HAVE_ATOI 1
#define HAVE_ATOL 1
#define HAVE_BCMP 1
#define HAVE_BCOPY 1
#define HAVE_BZERO 1
#define HAVE_INDEX 1
#define HAVE_MEMCCPY 1
#define HAVE_MEMCHR 1
#define HAVE_MEMCMP 1
#define HAVE_MEMCPY 1
#define HAVE_MEMMOVE 1
#define HAVE_MEMSET 1
#define HAVE_RINDEX 1
#define HAVE_STRCASECMP 1
#define HAVE_STRCAT 1
#define HAVE_STRCHR 1
#define HAVE_STRCMP 1
#define HAVE_STRCPY 1
#define HAVE_STRCSPN 1
#define HAVE_STRDUP 1
#define HAVE_STRLEN 1
#define HAVE_STRNLEN 1
#define HAVE_STRNCASECMP 1
#define HAVE_STRNCAT 1
#define HAVE_STRNCMP 1
#define HAVE_STRNCPY 1
#define HAVE_STRNDUP 1
#define HAVE_STRPBRK 1
#define HAVE_STRRCHR 1
#define HAVE_STRSEP 1
#define HAVE_STRSPN 1
#define HAVE_STRSTR 1
#define HAVE_STRTOK 1

/* manual settings */
#include "settings.h"

#endif /* ! __CONF_H__ */
