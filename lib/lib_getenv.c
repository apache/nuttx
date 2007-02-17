/************************************************************
 * lib_getenv.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Compilation Switches
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <sys/types.h>
#include <stdlib.h>
#include <debug.h>

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Type Declarations
 ************************************************************/

/************************************************************
 * Private Function Prototypes
 ************************************************************/

/**********************************************************
 * Global Constant Data
 **********************************************************/

/************************************************************
 * Global Variables
 ************************************************************/

/**********************************************************
 * Private Constant Data
 **********************************************************/

#ifdef ENVIRONMENT_STRINGS
static const char environment[] = ENVIRONMENT_STRINGS;
#else
static const char environment[] = "";
#endif

/************************************************************
 * Private Variables
 ************************************************************/

/************************************************************
 * Function:  getenv
 *
 * Description:
 *
 *   getenv() searches the environment list for a string of
 *   the form name=value, and returns a pointer to the value
 *   in the current environment if such a string is present,
 *   otherwise a NULL pointer.  name can be either the desired
 *   name, null-terminated, or of the form name=value, in which
 *   case getenv() uses the portion to the left of the = as the
 *   search key.
 *
 *   getenv() is (by convention) declared in stdlib.h
 *
 ************************************************************/

char *getenv(const char *name)
{
  const char *penv = environment;
  int size = sizeof(environment);
  const char *pend = &environment[size-1];
  const char *ptmp;

  dbg("getenv(ge): name=\"%s\"\n", name);

  if (name) {

    /* Process each string in the environment. */
    while (penv < pend) {

      vdbg("(ge):\tCompare to=\"%s\"\n", penv);

      /* The logic below basically implements a version of
       * strcmp where the strings may be terminated with = signs. */
      ptmp = name;
      for (;;) {

	/* Are we at the end of the name-to-matching?  */
	if ((!*ptmp) || (*ptmp == '=')) {

	  /* Yes.. are we also at the end of the matching-name? */
	  if (*penv == '=') {

	    /* Yes.. return the pointer to the value. */
	    dbg("(ge):\tReturning \"%s\"\n", penv+1);
	    return ((char*)penv+1);

	  } /* end if */
	  else {

	    /* No.. Skip to the next name matching name candidate. */
	    while(*penv++);
	    break;
	    
	  } /* end else */
	} /* end if */

	/* NO.. are we at the end of the matching name candidate? */
	/* OR.. do the corresponding characters not match. */
	else if (*penv != *ptmp) {

	  /* Yes.. Skip to the next name matching name candidate. */
	  while(*penv++);
	  break;
	    
	} /* end else if */
	else {

	  /* No.. try the next characters. */
	  penv++; ptmp++;

	} /* end else */
      } /* end for */
    } /* end while */
  } /* end if */

  /* If we got here, then no matching string was found. */
		dbg("(ge):\tReturning NULL\n");
  return NULL;

} /* end getenv */
