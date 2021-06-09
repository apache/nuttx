/*
 * Defines for the dmalloc library
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

#ifndef __DMALLOC_H__
#define __DMALLOC_H__

/* this is dmalloc.h.2 */
/* produced by configure, inserted into dmalloc.h */

/* const is available */
/* strdup is not a macro */
#undef DMALLOC_STRDUP_MACRO
/* strndup is not a macro */
#undef DMALLOC_STRNDUP_MACRO

/*
 * the definition of DMALLOC_SIZE
 *
 * NOTE: some architectures have malloc, realloc, etc.
 * using unsigned instead of unsigned long.  You may
 * have to edit this by hand to fix any compilation
 * warnings or errors.
 */
#include <sys/types.h>
#define DMALLOC_SIZE size_t

/*
 * We use stdarg.h for the dmalloc_message and
 * dmalloc_vmessage functions.
 */
#include <stdarg.h>
#define DMALLOC_STDARG 1

/* dmalloc version defines */
#define DMALLOC_VERSION_MAJOR 5 /* X.0.0 */
#define DMALLOC_VERSION_MINOR 6 /* 0.X.0 */
#define DMALLOC_VERSION_PATCH 5 /* 0.0.X */

/* NOTE: start of dmalloc.h.3 */

/* this defines what type the standard void memory-pointer is */
#if (defined(__STDC__) && __STDC__ == 1) || defined(__cplusplus) || defined(STDC_HEADERS) || defined(_ISO_STDLIB_ISO_H)
#define DMALLOC_PNT		void *
#define DMALLOC_FREE_RET	void
#else
#define DMALLOC_PNT		char *
#define DMALLOC_FREE_RET	int
#define DMALLOC_FREE_RET_INT
#endif

/*
 * Malloc function return codes
 */
#define CALLOC_ERROR		0L		/* error from calloc */
#define MALLOC_ERROR		0L		/* error from malloc */
#define REALLOC_ERROR		0L		/* error from realloc */

/* NOTE: this if for non- __STDC__ systems only */
#define FREE_ERROR		0		/* error from free */
#define FREE_NOERROR		1		/* no error from free */

#define DMALLOC_ERROR		0		/* function failed */
#define DMALLOC_NOERROR		1		/* function succeeded */

#define DMALLOC_VERIFY_ERROR	0		/* function failed */
#define DMALLOC_VERIFY_NOERROR	1		/* function succeeded */
#define MALLOC_VERIFY_ERROR	DMALLOC_VERIFY_ERROR
#define MALLOC_VERIFY_NOERROR	DMALLOC_VERIFY_NOERROR

/*
 * Dmalloc function IDs for the dmalloc_track_t callback function.
 */
#define DMALLOC_FUNC_MALLOC	10	/* malloc function called */
#define DMALLOC_FUNC_CALLOC	11	/* calloc function called */
#define DMALLOC_FUNC_REALLOC	12	/* realloc function called */
#define DMALLOC_FUNC_RECALLOC	13	/* recalloc called */
#define DMALLOC_FUNC_MEMALIGN	14	/* memalign function called */
#define DMALLOC_FUNC_VALLOC	15	/* valloc function called */
#define DMALLOC_FUNC_STRDUP	16	/* strdup function called */
#define DMALLOC_FUNC_FREE	17	/* free function called */
#define DMALLOC_FUNC_CFREE	18	/* cfree function called */

#define DMALLOC_FUNC_NEW	20	/* new function called */
#define DMALLOC_FUNC_NEW_ARRAY	21	/* new[] function called */
#define DMALLOC_FUNC_DELETE	22	/* delete function called */
#define DMALLOC_FUNC_DELETE_ARRAY 23	/* delete[] function called */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Tracking function that can be registered by calling dmalloc_track(...).  This callback function will
 * be called each time an allocation occurs.
 */
typedef void  (*dmalloc_track_t)(const char *file, const unsigned int line,
				 const int func_id,
				 const DMALLOC_SIZE byte_size,
				 const DMALLOC_SIZE alignment,
				 const DMALLOC_PNT old_addr,
				 const DMALLOC_PNT new_addr);


/* internal dmalloc error number for reference purposes only */
extern
int		dmalloc_errno;

/* logfile for dumping dmalloc info, DMALLOC_LOGFILE env var overrides this */
extern
char		*dmalloc_logpath;

/*
 * void dmalloc_shutdown
 *
 * Shutdown the dmalloc library and provide statistics if necessary.
 */
extern
void	dmalloc_shutdown(void);

#if FINI_DMALLOC
/*
 * void __fini_dmalloc
 *
 * Automatic function to close dmalloc supported by some operating
 * systems.  Pretty cool OS/compiler hack.  By default it is not
 * necessary because we use atexit() and on_exit() to register the
 * close functions which are more portable.
 */
extern
void	__fini_dmalloc(void);
#endif /* if FINI_DMALLOC */

/*
 * DMALLOC_PNT dmalloc_malloc
 *
 * Allocate and return a memory block of a certain size.
 *
 * Returns a valid pointer on success or NULL on failure.
 *
 * ARGUMENTS:
 *
 * file -> File-name or return-address of the caller.
 *
 * line -> Line-number of the caller.
 *
 * size -> Number of bytes requested.
 *
 * func_id -> Function-id to identify the type of call.  See
 * dmalloc.h.
 *
 * alignment -> To align the new block to a certain number of bytes,
 * set this to a value greater than 0.
 *
 * xalloc_b -> If set to 1 then print an error and exit if we run out
 * of memory.
 */
extern
DMALLOC_PNT	dmalloc_malloc(const char *file, const int line,
			       const DMALLOC_SIZE size, const int func_id,
			       const DMALLOC_SIZE alignment,
			       const int xalloc_b);

/*
 * DMALLOC_PNT dmalloc_realloc
 *
 * Resizes and old pointer to a new number of bytes.
 *
 * Returns a valid pointer on success or NULL on failure.
 *
 * ARGUMENTS:
 *
 * file -> File-name or return-address of the caller.
 *
 * line -> Line-number of the caller.
 *
 * old_pnt -> Pointer to an existing memory chunk that we are
 * resizing.  If this is NULL then it basically does a malloc.
 *
 * new_size -> New number of bytes requested for the old pointer.
 *
 * func_id -> Function-id to identify the type of call.  See
 * dmalloc.h.
 *
 * xalloc_b -> If set to 1 then print an error and exit if we run out
 * of memory.
 */
extern
DMALLOC_PNT	dmalloc_realloc(const char *file, const int line,
				DMALLOC_PNT old_pnt, DMALLOC_SIZE new_size,
				const int func_id, const int xalloc_b);

/*
 * int dmalloc_free
 *
 * Release a pointer back into the heap.
 *
 * Returns FREE_NOERROR on success or FREE_ERROR on failure.
 *
 * Note: many operating systems define free to return (void) so this
 * return value may be filtered.  Dumb.
 *
 * ARGUMENTS:
 *
 * file -> File-name or return-address of the caller.
 *
 * line -> Line-number of the caller.
 *
 * pnt -> Existing pointer we are freeing.
 *
 * func_id -> Function-id to identify the type of call.  See
 * dmalloc.h.
 */
extern
int	dmalloc_free(const char *file, const int line, DMALLOC_PNT pnt,
		     const int func_id);

/*
 * DMALLOC_PNT dmalloc_strndup
 *
 * Allocate and return an allocated block of memory holding a copy of
 * a string of a certain number of characters.
 *
 * Returns a valid pointer on success or NULL on failure.
 *
 * ARGUMENTS:
 *
 * file -> File-name or return-address of the caller.
 *
 * line -> Line-number of the caller.
 *
 * string -> String we are duplicating.
 *
 * max_len -> Max length of the string we are duplicating.  Set to -1 for none.
 *
 * xalloc_b -> If set to 1 then print an error and exit if we run out
 * of memory.
 */
extern
char	*dmalloc_strndup(const char *file, const int line,
			 const char *string, const int max_len,
			 const int xalloc_b);

/*
 * DMALLOC_PNT malloc
 *
 * Overloading the malloc(3) function.  Allocate and return a memory
 * block of a certain size.
 *
 * Returns a valid pointer on success or NULL on failure.
 *
 * ARGUMENTS:
 *
 * size -> Number of bytes requested.
 */
extern
DMALLOC_PNT	malloc(DMALLOC_SIZE size);

/*
 * DMALLOC_PNT calloc
 *
 * Overloading the calloc(3) function.  Returns a block of zeroed memory.
 *
 * Returns a valid pointer on success or NULL on failure.
 *
 * ARGUMENTS:
 *
 * num_elements -> Number of elements being allocated.
 *
 * size -> The number of bytes in each element.
 */
extern
DMALLOC_PNT	calloc(DMALLOC_SIZE num_elements, DMALLOC_SIZE size);

/*
 * DMALLOC_PNT realloc
 *
 * Overload of realloc(3).  Resizes and old pointer to a new number of bytes.
 *
 * Returns a valid pointer on success or NULL on failure.
 *
 * ARGUMENTS:
 *
 * old_pnt -> Pointer to an existing memory chunk that we are
 * resizing.  If this is NULL then it basically does a malloc.
 *
 * new_size -> New number of bytes requested for the old pointer.
 */
extern
DMALLOC_PNT	realloc(DMALLOC_PNT old_pnt, DMALLOC_SIZE new_size);

/*
 * DMALLOC_PNT recalloc
 *
 * Overload of recalloc(3) which exists on some systems.  Resizes and
 * old pointer to a new number of bytes.  If we are expanding, then
 * any new bytes will be zeroed.
 *
 * Returns a valid pointer on success or NULL on failure.
 *
 * ARGUMENTS:
 *
 * old_pnt -> Pointer to an existing memory chunk that we are resizing.
 *
 * new_size -> New number of bytes requested for the old pointer.
 */
extern
DMALLOC_PNT	recalloc(DMALLOC_PNT old_pnt, DMALLOC_SIZE new_size);

/*
 * DMALLOC_PNT memalign
 *
 * Overloading the memalign(3) function.  Allocate and return a memory
 * block of a certain size which have been aligned to a certain
 * alignment.
 *
 * Returns a valid pointer on success or NULL on failure.
 *
 * ARGUMENTS:
 *
 * alignment -> Value to which the allocation must be aligned.  This
 * should probably be a multiple of 2 with a maximum value equivalent
 * to the block-size which is often 1k or 4k.
 *
 * size -> Number of bytes requested.
 */
extern
DMALLOC_PNT	memalign(DMALLOC_SIZE alignment, DMALLOC_SIZE size);

/*
 * DMALLOC_PNT valloc
 *
 * Overloading the valloc(3) function.  Allocate and return a memory
 * block of a certain size which have been aligned to page boundaries
 * which are often 1k or 4k.
 *
 * Returns a valid pointer on success or NULL on failure.
 *
 * ARGUMENTS:
 *
 * size -> Number of bytes requested.
 */
extern
DMALLOC_PNT	valloc(DMALLOC_SIZE size);

#ifndef DMALLOC_STRDUP_MACRO
/*
 * DMALLOC_PNT strdup
 *
 * Overload of strdup(3).  Allocate and return an allocated block of
 * memory holding a copy of a string.
 *
 * Returns a valid pointer on success or NULL on failure.
 *
 * ARGUMENTS:
 *
 * string -> String we are duplicating.
 */
extern
char	*strdup(const char *string);
#endif /* ifndef DMALLOC_STRDUP_MACRO */

#ifndef DMALLOC_STRNDUP_MACRO
/*
 * DMALLOC_PNT strndup
 *
 * Overload of strndup(3).  Allocate and return an allocated block of
 * memory holding a copy of a string with a maximum length.
 *
 * Returns a valid pointer on success or NULL on failure.
 *
 * ARGUMENTS:
 *
 * string -> String we are duplicating.
 *
 * max_len -> Max length of the string to duplicate.
 */
extern
char	*strndup(const char *string, const DMALLOC_SIZE max_len);
#endif /* ifndef DMALLOC_STRNDUP_MACRO */

/*
 * DMALLOC_FREE_RET free
 *
 * Release a pointer back into the heap.
 *
 * Returns FREE_ERROR, FREE_NOERROR or void depending on whether STDC
 * is defined by your compiler.
 *
 * ARGUMENTS:
 *
 * pnt -> Existing pointer we are freeing.
 */
extern
DMALLOC_FREE_RET	free(DMALLOC_PNT pnt);

/*
 * DMALLOC_FREE_RET cfree
 *
 * Same as free.
 *
 * Returns FREE_ERROR, FREE_NOERROR or void depending on whether STDC
 * is defined by your compiler.
 *
 * ARGUMENTS:
 *
 * pnt -> Existing pointer we are freeing.
 */
extern
DMALLOC_FREE_RET	cfree(DMALLOC_PNT pnt);

/*
 * int dmalloc_verify
 *
 * Verify a pointer which has previously been allocated by the
 * library or check the entire heap.
 *
 * Returns MALLOC_VERIFY_NOERROR on success or MALLOC_VERIFY_ERROR on failure.
 *
 * ARGUMENTS:
 *
 * pnt -> Pointer we are verifying.  If 0L then check the entire heap.
 */
extern
int	dmalloc_verify(const DMALLOC_PNT pnt);

/*
 * int malloc_verify
 *
 * Verify a pointer which has previously been allocated by the
 * library.  Same as dmalloc_verify.
 *
 * Returns MALLOC_VERIFY_NOERROR on success or MALLOC_VERIFY_ERROR on failure.
 *
 * ARGUMENTS:
 *
 * pnt -> Pointer we are verifying.  If 0L then check the entire heap.
 */
extern
int	malloc_verify(const DMALLOC_PNT pnt);

/*
 * int dmalloc_verify_pnt
 *
 * This function is mainly used by the arg_check.c functions to verify
 * specific pointers.  This can be used by users to provide more fine
 * grained tests on pointers.
 *
 * Returns MALLOC_VERIFY_NOERROR on success or MALLOC_VERIFY_ERROR on failure.
 *
 * ARGUMENTS:
 *
 * file -> File-name or return-address of the caller.  You can use
 * __FILE__ for this argument or 0L for none.
 *
 * line -> Line-number of the caller.  You can use __LINE__ for this
 * argument or 0 for none.
 *
 * func -> Function string which is checking the pointer.  0L if none.
 *
 * pnt -> Pointer we are checking.
 *
 * exact_b -> Set to 1 if this pointer was definitely handed back from
 * a memory allocation.  If set to 0 then this pointer can be inside
 * another allocation or outside the heap altogether.
 *
 * strlen_b -> Set to 1 to make sure that this pointer can handle
 * strlen(pnt) + 1 bytes up to the maximum specified by min_size.  If
 * this is 1 and min_size > 0 then it is in effect a strnlen.
 *
 * min_size -> Make sure that pointer can hold at least that many
 * bytes if inside of the heap.  If 0 then don't check the size.
 */
extern
int	dmalloc_verify_pnt_strsize(const char *file, const int line,
				   const char *func, const void *pnt,
				   const int exact_b, const int strlen_b,
				   const int min_size);

/*
 * int dmalloc_verify_pnt
 *
 * This function is mainly used by the arg_check.c functions to verify
 * specific pointers.  This can be used by users to provide more fine
 * grained tests on pointers.
 *
 * Returns MALLOC_VERIFY_NOERROR on success or MALLOC_VERIFY_ERROR on failure.
 *
 * ARGUMENTS:
 *
 * file -> File-name or return-address of the caller.  You can use
 * __FILE__ for this argument or 0L for none.
 *
 * line -> Line-number of the caller.  You can use __LINE__ for this
 * argument or 0 for none.
 *
 * func -> Function string which is checking the pointer.  0L if none.
 *
 * pnt -> Pointer we are checking.
 *
 * exact_b -> Set to 1 if this pointer was definitely handed back from
 * a memory allocation.  If set to 0 then this pointer can be inside
 * another allocation or outside the heap altogether.
 *
 * min_size -> Make sure that pointer can hold at least that many
 * bytes if inside of the heap.  If -1 then make sure it can handle
 * strlen(pnt) + 1 bytes (+1 for the \0).  If 0 then don't check the
 * size.  If you need strnlen functionality with a maximum on the
 * strlen, see dmalloc_verify_pnt_strsize.
 */
extern
int	dmalloc_verify_pnt(const char *file, const int line, const char *func,
			   const void *pnt, const int exact_b,
			   const int min_size);

/*
 * unsigned int dmalloc_debug
 *
 * Set the global debug functionality flags.  You can also use
 * dmalloc_debug_setup.
 *
 * Note: you cannot add or remove certain flags such as signal
 * handlers since they are setup at initialization time only.
 *
 * Returns the old debug flag value.
 *
 * ARGUMENTS:
 *
 * flags -> Flag value to set.  Pass in 0 to disable all debugging.
 */
extern
unsigned int	dmalloc_debug(const unsigned int flags);

/*
 * char *dmalloc_debug_current
 *
 * Returns the current debug functionality flags.  This allows you to
 * save a dmalloc library state to be restored later.
 */
extern
unsigned int	dmalloc_debug_current(void);

/*
 * char *dmalloc_debug_current_env
 *
 * Returns the current debug environment.  This allows you to save a
 * dmalloc library state to be restored later with a call to
 * dmalloc_debug_setup().
 *
 * Returns the current debug environment.
 *
 * ARGUMENTS:
 *
 * env_buf -> Buffer to use for getting the environment.
 *
 * env_buf_size -> Size of the buffer.
 */
extern
char	*dmalloc_debug_current_env(char *env_buf, const int env_buf_size);

/*
 * void dmalloc_debug_setup
 *
 * Set the global debugging functionality as an option string.
 * Normally this would be pased in in the DMALLOC_OPTIONS
 * environmental variable.  This is here to override the env or for
 * circumstances where modifying the environment is not possible or
 * does not apply such as servers or cgi-bin programs.
 *
 * ARGUMENTS:
 *
 * options_str -> Options string to set the library flags.
 */
extern
void	dmalloc_debug_setup(const char *options_str);

/*
 * int dmalloc_examine
 *
 * Examine a pointer and pass back information on its allocation size
 * as well as the file and line-number where it was allocated.  If the
 * file and line number is not available, then it will pass back the
 * allocation location's return-address if available.
 *
 * Returns DMALLOC_NOERROR on success or DMALLOC_ERROR on failure.
 *
 * ARGUMENTS:
 *
 * pnt -> Pointer we are checking.
 *
 * user_size_p <- Pointer to a DMALLOC_SIZE type variable which, if
 * not NULL, will be set to the size of bytes from the pointer.
 *
 * total_size_p <- Poiner to a DMALLOC_SIZE type variable which, if
 * not NULL, will be set to the total size given for this allocation
 * including administrative overhead.
 *
 * file_p <- Pointer to a character pointer which, if not NULL, will
 * be set to the file where the pointer was allocated.
 *
 * line_p <- Pointer to an unsigned integer which, if not NULL, will
 * be set to the line-number where the pointer was allocated.
 *
 * ret_attr_p <- Pointer to a void pointer, if not NULL, will be set
 * to the return-address where the pointer was allocated.
 *
 * used_mark_p <- Poiner to an unsigned integer which, if not NULL,
 * will be set to the mark of when the pointer was last "used".  This
 * could be when it was allocated, reallocated, or freed.
 *
 * seen_p <- Poiner to an unsigned long which, if not NULL, will be
 * set to the number of times that this pointer has been allocated,
 * realloced, or freed.  NOTE: LOG_PNT_SEEN_COUNT must be set to 1
 * otherwise no seen information is available and it will be set to 0.
 */
extern
int	dmalloc_examine(const DMALLOC_PNT pnt, DMALLOC_SIZE *user_size_p,
			DMALLOC_SIZE *total_size_p, char **file_p,
			unsigned int *line_p, DMALLOC_PNT *ret_attr_p,
			void **backtrace_p, unsigned int *backtrace_depth_p,
			unsigned long *used_mark_p, unsigned long *seen_p);

/*
 * void dmalloc_track
 *
 * Register an allocation tracking function which will be called each
 * time an allocation occurs.
 *
 * ARGUMENTS:
 *
 * track_func -> Function to register as the tracking function.  Set
 * to NULL to disable.
 */
extern
void	dmalloc_track(const dmalloc_track_t track_func);

/*
 * unsigned long dmalloc_mark
 *
 * Return to the caller the current "mark" which can be used later by
 * dmalloc_log_changed to log the changed pointers since this point.
 * Multiple marks can be saved and used.
 *
 * This is also the iteration number and can be logged at the front of
 * each memory transaction in the logfile with the LOG_ITERATION
 * define in settings.h and can be logged with each pointer with the
 * LOG_PNT_ITERATION define in settings.h.
 */
extern
unsigned long	dmalloc_mark(void);

/*
 * unsigned long dmalloc_memory_allocated
 *
 * Return the total number of bytes allocated by the program so far.
 */
extern
unsigned long	dmalloc_memory_allocated(void);

/*
 * unsigned int dmalloc_page_size
 *
 * Get the page-size being used by dmalloc.
 */
extern
unsigned int	dmalloc_page_size(void);

/*
 * unsigned long dmalloc_count_changed
 *
 * Count the changed memory bytes since a particular mark.
 *
 * Returns the number of bytes since mark.
 *
 * ARGUMENTS:
 *
 * mark -> Sets the point from which to count the changed memory.  You
 * can use dmalloc_mark to get the current mark value which can later
 * be passed in here.  Pass in 0 to report on the unfreed memory since
 * the program started.
 *
 * not_freed_b -> Set to 1 to count the new pointers that are non-freed.
 *
 * free_b -> Set to 1 to count the new pointers that are freed.
 */
extern
unsigned long	dmalloc_count_changed(const unsigned long mark,
				      const int not_freed_b, const int free_b);

/*
 * void dmalloc_log_status
 *
 * Dump dmalloc statistics to logfile.
 */
extern
void	dmalloc_log_stats(void);

/*
 * void dmalloc_log_unfreed
 *
 * Dump unfreed-memory info to logfile.
 */
extern
void	dmalloc_log_unfreed(void);

/*
 * void dmalloc_log_changed
 *
 * Dump the pointers that have changed since a point in time.
 *
 * ARGUMENTS:
 *
 * mark -> Sets the point to compare against.  You can use
 * dmalloc_mark to get the current mark value which can later be
 * passed in here.  Pass in 0 to log what has changed since the
 * program started.
 *
 * not_freed_b -> Set to 1 to log the new pointers that are non-freed.
 *
 * free_b -> Set to 1 to log the new pointers that are freed.
 *
 * details_b -> Set to 1 to dump the individual pointers that have
 * changed otherwise the summaries will be logged.
 */
extern
void	dmalloc_log_changed(const unsigned long mark, const int not_freed_b,
			    const int free_b, const int details_b);

/*
 * void dmalloc_vmessage
 *
 * Message writer with vprintf like arguments which adds a line to the
 * dmalloc logfile.
 *
 * ARGUMENTS:
 *
 * format -> Printf-style format statement.
 *
 * args -> Already converted pointer to a stdarg list.
 */
extern
void	dmalloc_vmessage(const char *format, va_list *args);

/*
 * void dmalloc_message
 *
 * Message writer with printf like arguments which adds a line to the
 * dmalloc logfile.
 *
 * ARGUMENTS:
 *
 * format -> Printf-style format statement.
 *
 * ... -> Variable argument list.
 */
extern
void	dmalloc_message(const char *format, ...)
#ifdef __GNUC__
  __attribute__ ((format (printf, 1, 2)))
#endif
;

/*
 * void dmalloc_get_stats
 *
 * Get a number of statistics about the current heap.
 *
 * ARGUMENTS:
 *
 * heap_low_p <- Pointer to pointer which, if not 0L, will be set to
 * the low address in the heap.
 *
 * heap_high_p <- Pointer to pointer which, if not 0L, will be set to
 * the high address in the heap.
 *
 * total_space_p <- Pointer to an unsigned long which, if not 0L, will
 * be set to the total space managed by the library including user
 * space, administrative space, and overhead.
 *
 * user_space_p <- Pointer to an unsigned long which, if not 0L, will
 * be set to the space given to the user process (allocated and free).
 *
 * current_allocated_p <- Pointer to an unsigned long which, if not
 * 0L, will be set to the current allocated space given to the user
 * process.
 *
 * current_pnt_np <- Pointer to an unsigned long which, if not 0L,
 * will be set to the current number of pointers allocated by the user
 * process.
 *
 * max_allocated_p <- Pointer to an unsigned long which, if not 0L,
 * will be set to the maximum allocated space given to the user
 * process.
 *
 * max_pnt_np <- Pointer to an unsigned long which, if not 0L, will be
 * set to the maximum number of pointers allocated by the user
 * process.
 *
 * max_one_p <- Pointer to an unsigned long which, if not 0L, will be
 * set to the maximum allocated with 1 call by the user process.
 */
extern
void	dmalloc_get_stats(DMALLOC_PNT *heap_low_p,
			  DMALLOC_PNT *heap_high_p,
			  unsigned long *total_space_p,
			  unsigned long *user_space_p,
			  unsigned long *current_allocated_p,
			  unsigned long *current_pnt_np,
			  unsigned long *max_allocated_p,
			  unsigned long *max_pnt_np,
			  unsigned long *max_one_p);

/*
 * const char *dmalloc_strerror
 *
 * Convert a dmalloc error code into its string equivalent.
 *
 * Returns String version of the error on success or "unknown error" on failure.
 *
 * ARGUMENTS:
 *
 * error_num -> Error number we are converting.
 */
extern
const char	*dmalloc_strerror(const int error_num);


#ifdef __cplusplus
}
#endif

/*
 * alloc macros to provide for memory FILE/LINE debugging information.
 */

#ifndef DMALLOC_DISABLE

#undef malloc
#define malloc(size) \
  dmalloc_malloc(__FILE__, __LINE__, (size), DMALLOC_FUNC_MALLOC, 0, 0)
#undef calloc
#define calloc(count, size) \
  dmalloc_malloc(__FILE__, __LINE__, (count)*(size), DMALLOC_FUNC_CALLOC, 0, 0)
#undef realloc
#define realloc(ptr, size) \
  dmalloc_realloc(__FILE__, __LINE__, (ptr), (size), DMALLOC_FUNC_REALLOC, 0)
#undef recalloc
#define recalloc(ptr, size) \
  dmalloc_realloc(__FILE__, __LINE__, (ptr), (size), DMALLOC_FUNC_RECALLOC, 0)
#undef memalign
#define memalign(alignment, size) \
  dmalloc_malloc(__FILE__, __LINE__, (size), DMALLOC_FUNC_MEMALIGN, \
		 (alignment), 0 /* no xalloc */)
#undef valloc
#define valloc(size) \
  dmalloc_malloc(__FILE__, __LINE__, (size), DMALLOC_FUNC_VALLOC, \
		0 /* special case */, 0 /* no xalloc */)
#ifndef DMALLOC_STRDUP_MACRO
#undef strdup
#define strdup(str) \
  dmalloc_strndup(__FILE__, __LINE__, (str), -1, 0)
#endif
#ifndef DMALLOC_STRNDUP_MACRO
#undef strndup
#define strndup(str, len) \
  dmalloc_strndup(__FILE__, __LINE__, (str), (len), 0)
#endif
#undef free
#define free(ptr) \
  dmalloc_free(__FILE__, __LINE__, (ptr), DMALLOC_FUNC_FREE)

#undef xmalloc
#define xmalloc(size) \
  dmalloc_malloc(__FILE__, __LINE__, (size), DMALLOC_FUNC_MALLOC, 0, 1)
#undef xcalloc
#define xcalloc(count, size) \
  dmalloc_malloc(__FILE__, __LINE__, (count)*(size), DMALLOC_FUNC_CALLOC, 0, 1)
#undef xrealloc
#define xrealloc(ptr, size) \
  dmalloc_realloc(__FILE__, __LINE__, (ptr), (size), DMALLOC_FUNC_REALLOC, 1)
#undef xrecalloc
#define xrecalloc(ptr, size) \
  dmalloc_realloc(__FILE__, __LINE__, (ptr), (size), DMALLOC_FUNC_RECALLOC, 1)
#undef xmemalign
#define xmemalign(alignment, size) \
  dmalloc_malloc(__FILE__, __LINE__, (size), DMALLOC_FUNC_MEMALIGN, \
		 (alignment), 1)
#undef xvalloc
#define xvalloc(size) \
  dmalloc_malloc(__FILE__, __LINE__, (size), DMALLOC_FUNC_VALLOC, 0, 1)
#undef xstrdup
#define xstrdup(str) \
  dmalloc_strndup(__FILE__, __LINE__, (str), -1, 1)
#undef xstrndup
#define xstrndup(str, len) \
  dmalloc_strndup(__FILE__, __LINE__, (str), (len), 1)
#undef xfree
#define xfree(ptr) \
  dmalloc_free(__FILE__, __LINE__, (ptr), DMALLOC_FUNC_FREE)

#ifdef DMALLOC_FUNC_CHECK

/*
 * do debugging on the following functions.  this may cause compilation or
 * other problems depending on your architecture.
 */
#undef atoi
#define atoi(str) \
  _dmalloc_atoi(__FILE__, __LINE__, (str))
#undef atol
#define atol(str) \
  _dmalloc_atol(__FILE__, __LINE__, (str))

#undef bcmp
#define bcmp(b1, b2, len) \
  _dmalloc_bcmp(__FILE__, __LINE__, (b1), (b2), (len))
#undef bcopy
#define bcopy(from, to, len) \
  _dmalloc_bcopy(__FILE__, __LINE__, (from), (to), (len))
#undef bzero
#define bzero(buf, len) \
  _dmalloc_bzero(__FILE__, __LINE__, (buf), (len))

#undef memcmp
#define memcmp(b1, b2, len) \
  _dmalloc_memcmp(__FILE__, __LINE__, (b1), (b2), (len))
#undef memcpy
#define memcpy(to, from, len) \
  _dmalloc_memcpy(__FILE__, __LINE__, (to), (from), (len))
#undef memmove
#define memmove(to, from, len) \
  _dmalloc_memmove(__FILE__, __LINE__, (to), (from), (len))
#undef memset
#define memset(buf, ch, len) \
  _dmalloc_memset(__FILE__, __LINE__, (buf), (ch), (len))

#undef index
#define index(str, ch) \
  _dmalloc_index(__FILE__, __LINE__, (str), (ch))
#undef rindex
#define rindex(str, ch) \
  _dmalloc_rindex(__FILE__, __LINE__, (str), (ch))

#undef strcat
#define strcat(to, from) \
  _dmalloc_strcat(__FILE__, __LINE__, (to), (from))
#undef strcmp
#define strcmp(s1, s2) \
  _dmalloc_strcmp(__FILE__, __LINE__, (s1), (s2))
#undef strlen
#define strlen(str) \
  _dmalloc_strlen(__FILE__, __LINE__, (str))
#undef strtok
#define strtok(str, sep) \
  _dmalloc_strtok(__FILE__, __LINE__, (str), (sep))

#undef memccpy
#define memccpy(s1, s2, ch, len) \
  _dmalloc_memccpy(__FILE__, __LINE__, (s1), (s2),(ch),(len))
#undef memchr
#define memchr(s1, ch, len) \
  _dmalloc_memchr(__FILE__, __LINE__, (s1), (ch), (len))

#undef strchr
#define strchr(str, ch) \
  _dmalloc_strchr(__FILE__, __LINE__, (str), (ch))
#undef strrchr
#define strrchr(str, ch) \
  _dmalloc_strrchr(__FILE__, __LINE__, (str), (ch))

#undef strcpy
#define strcpy(to, from) \
  _dmalloc_strcpy(__FILE__, __LINE__, (to), (from))
#undef strncpy
#define strncpy(to, from, len) \
  _dmalloc_strncpy(__FILE__, __LINE__, (to), (from), (len))
#undef strcasecmp
#define strcasecmp(s1, s2) \
  _dmalloc_strcasecmp(__FILE__, __LINE__, (s1), (s2))
#undef strncasecmp
#define strncasecmp(s1, s2, len) \
  _dmalloc_strncasecmp(__FILE__, __LINE__, (s1), (s2), (len))
#undef strspn
#define strspn(str, list) \
  _dmalloc_strspn(__FILE__, __LINE__, (str), (list))
#undef strcspn
#define strcspn(str, list) \
  _dmalloc_strcspn(__FILE__, __LINE__, (str), (list))
#undef strncat
#define strncat(to, from, len) \
  _dmalloc_strncat(__FILE__, __LINE__, (to), (from), (len))
#undef strncmp
#define strncmp(s1, s2, len) \
  _dmalloc_strncmp(__FILE__, __LINE__, (s1), (s2), (len))
#undef strpbrk
#define strpbrk(str, list) \
  _dmalloc_strpbrk(__FILE__, __LINE__, (str), (list))
#undef strstr
#define strstr(str, pat) \
  _dmalloc_strstr(__FILE__, __LINE__, (str), (pat))

#endif /* DMALLOC_FUNC_CHECK */
#endif /* ! DMALLOC_DISABLE */

/*
 * feel free to add your favorite functions here and to arg_check.[ch]
 */

#ifdef __cplusplus
extern "C" {
#endif


/*
 * Dummy function for checking atoi's arguments.
 */
extern
int	_dmalloc_atoi(const char *file, const int line, const char *str);

/*
 * Dummy function for checking atol's arguments.
 */
extern
long	_dmalloc_atol(const char *file, const int line, const char *str);

/*
 * Dummy function for checking bcmp's arguments.
 */
extern
int	_dmalloc_bcmp(const char *file, const int line,
		      const void *b1, const void *b2, const DMALLOC_SIZE len);

/*
 * Dummy function for checking bcopy's arguments.
 */
extern
void	_dmalloc_bcopy(const char *file, const int line,
		       const void *from, void *to, const DMALLOC_SIZE len);

/*
 * Dummy function for checking bzero's arguments.
 */
extern
void	_dmalloc_bzero(const char *file, const int line,
		       void *buf, const DMALLOC_SIZE len);

/*
 * Dummy function for checking index's arguments.
 */
extern
char	*_dmalloc_index(const char *file, const int line,
			const char *str, const char ch);

/*
 * Dummy function for checking memccpy's arguments.
 */
extern
void	*_dmalloc_memccpy(const char *file, const int line,
			  void *dest, const void *src, const int ch,
			  const DMALLOC_SIZE len);

/*
 * Dummy function for checking memchr's arguments.
 */
extern
void	*_dmalloc_memchr(const char *file, const int line,
			 const void *s1, const int ch, const DMALLOC_SIZE len);

/*
 * Dummy function for checking memcmp's arguments.
 */
extern
int	_dmalloc_memcmp(const char *file, const int line,
			const void *b1, const void *b2, const DMALLOC_SIZE len);

/*
 * Dummy function for checking memcpy's arguments.
 */
extern
void	*_dmalloc_memcpy(const char *file, const int line,
			 void *to, const void *from, const DMALLOC_SIZE len);

/*
 * Dummy function for checking memmove's arguments.
 */
extern
void	*_dmalloc_memmove(const char *file, const int line,
			  void *to, const void *from, const DMALLOC_SIZE len);

/*
 * Dummy function for checking memset's arguments.
 */
extern
void	*_dmalloc_memset(const char *file, const int line, void *buf,
			 const int ch, const DMALLOC_SIZE len);

/*
 * Dummy function for checking rindex's arguments.
 */
extern
char	*_dmalloc_rindex(const char *file, const int line,
			 const char *str, const char ch);

/*
 * Dummy function for checking strcasecmp's arguments.
 */
extern
int	_dmalloc_strcasecmp(const char *file, const int line,
			    const char *s1, const char *s2);

/*
 * Dummy function for checking strcat's arguments.
 */
extern
char	*_dmalloc_strcat(const char *file, const int line,
			 char *to, const char *from);

/*
 * Dummy function for checking strchr's arguments.
 */
extern
char	*_dmalloc_strchr(const char *file, const int line,
			 const char *str, const int ch);

/*
 * Dummy function for checking strcmp's arguments.
 */
extern
int	_dmalloc_strcmp(const char *file, const int line,
			const char *s1, const char *s2);

/*
 * Dummy function for checking strcpy's arguments.
 */
extern
char	*_dmalloc_strcpy(const char *file, const int line,
			 char *to, const char *from);

/*
 * Dummy function for checking strcspn's arguments.
 */
extern
int	_dmalloc_strcspn(const char *file, const int line,
			 const char *str, const char *list);

/*
 * Dummy function for checking strlen's arguments.
 */
extern
DMALLOC_SIZE	_dmalloc_strlen(const char *file, const int line,
				const char *str);

/*
 * Dummy function for checking strncasecmp's arguments.
 */
extern
int	_dmalloc_strncasecmp(const char *file, const int line,
			     const char *s1, const char *s2,
			     const DMALLOC_SIZE len);

/*
 * Dummy function for checking strncat's arguments.
 */
extern
char	*_dmalloc_strncat(const char *file, const int line,
			  char *to, const char *from, const DMALLOC_SIZE len);

/*
 * Dummy function for checking strncmp's arguments.
 */
extern
int	_dmalloc_strncmp(const char *file, const int line,
			 const char *s1, const char *s2,
			 const DMALLOC_SIZE len);

/*
 * Dummy function for checking strncpy's arguments.
 */
extern
char	*_dmalloc_strncpy(const char *file, const int line,
			  char *to, const char *from, const DMALLOC_SIZE len);

/*
 * Dummy function for checking strpbrk's arguments.
 */
extern
char	*_dmalloc_strpbrk(const char *file, const int line,
			  const char *str, const char *list);

/*
 * Dummy function for checking strrchr's arguments.
 */
extern
char	*_dmalloc_strrchr(const char *file, const int line,
			  const char *str, const int ch);

/*
 * Dummy function for checking strspn's arguments.
 */
extern
int	_dmalloc_strspn(const char *file, const int line,
			const char *str, const char *list);

/*
 * Dummy function for checking strstr's arguments.
 */
extern
char	*_dmalloc_strstr(const char *file, const int line,
			 const char *str, const char *pat);

/*
 * Dummy function for checking strtok's arguments.
 */
extern
char	*_dmalloc_strtok(const char *file, const int line,
			 char *str, const char *sep);


#ifdef __cplusplus
}
#endif

#endif /* ! __DMALLOC_H__ */
