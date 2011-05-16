/****************************************************************************
 * include/nuttx/event.h
 *
 *   Copyright(C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/** \file
 *  \author Uros Platise
 *  \brief Events
 * 
 * Event moudle offer real-time signaling and handling of real-time 
 * events at precision of system resolution clock_t.
 * 
 * Events are based on periodic timers, where each started event starts
 * counting at its relative time = 0, and overflows at given period. 
 * Overflow is signalized by posted event that is handled by the 
 * event_eventhandler() called by each thread or task working with 
 * Events. 
 * 
 * Events may be used inside one thread or task only or between tasks 
 * and threads. An event can also be bind to another event i.e.:
 *  - System Clock may issue a notification about Power Failure
 *  - Other threads may connect to this event and when handled, they may
 *    inquiry for pricise time when this event has happend, using the 
 *    event_gettime()
 * 
 * Or system is about to control a process, and instead of looping and 
 * inquiring for time, system timers offer simple and effective 
 * mechanism to control timed actions:
 *  - First event is about to wait 5 seconds, then it must assert some 
 *    switch, triggering another event after 2 seconds, to check the 
 *    state of the system.
 *  - After 2 seconds new event is triggerred, if it is time criticial
 *    it may check for the overrun time and handle further actions,
 *  - ...
 * 
 * Each event is denoted by a callback function event_callback_t, and 
 * an argument. Posted event with the same callback but different 
 * argument is treated as different event, while the same callback with
 * the same argument replaces previously started event.
 **/ 

#ifdef CONFIG_EVENT

#ifndef __INCLUDE_NUTTX_EVENT_H
#define __INCLUDE_NUTTX_EVENT_H

#include <nuttx/config.h>

#include <time.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/clock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/** Event Callback
 * 
 * \param arg an argument as posted by the event_post(),
 *  event_signal() or event_start() methods.
 * 
 * \return Optional time to increment to retrigger. This time is added
 *   to present actual time, so an event returning with constant value
 *   as return PERIOD would be periodic with long-term absolute precision.
 * 
 * \retval >0 Time to increment to second post of the event.
 * \retval 0  Acknowledge event.
 */
typedef clock_t (*event_callback_t)(FAR void *arg);

typedef struct event_s event_t;

/****************************************************************************
 * Public Functions
 ****************************************************************************/
 
/** Create new instance of Events (typically per thread) 
 * 
 * \param base Future extension to support multiple base timers. Pass NULL at
 *    the moment.
 * 
 * \return Valid structure or NULL on error with errno set.
 */
event_t * event_create(void * base);

/** Delete unused instance of Events. A call to this function also destroys
 *  all bindings. 
 * 
 * \param events Pointer to a structure as returned by event_create()
 * \return 0 On success or -1 on error.
 */
int event_delete(event_t *events);

/** Post an event, equal to period = 0 
 * 
 * In case of connected events, those are called prior calling
 * the given event. Since the event is called last on the list
 * it may serve as an acknowledgement mechanism.
 * 
 * \param events Pointer to a structure as returned by event_create()
 * \param event   Callback to be called
 * \param arg     Optional argument to be passed to the event.
 * \return 0 On success or -1 on error.
 */ 
int event_post(event_t *events, event_callback_t event, void *arg);

/** Trigger an event without calling it but may trigger connected events 
 *
 * \param events Pointer to a structure as returned by event_create()
 * \param event   Callback to be called
 * \param arg     Optional argument to be passed to the event.
 * \return 0 On success or -1 on error.
 */ 
int event_signal(event_t *events, event_callback_t event, void *arg);

/** Calls all connected events only immediately.
 *
 * \param events Pointer to a structure as returned by event_create()
 * \param event   Callback to be called
 * \param arg     Optional argument to be passed to the event.
 * \return 0 On success or -1 on error.
 */ 
int event_call(event_t *events, event_callback_t event, void *arg);

/** Are there any connections to given event
 * \param events Pointer to a structure as returned by event_create()
 * \param event   Callback to be called
 * \return 1 When empty otherwise 0
 */
int event_isempty(event_t *events, event_callback_t event);

/** Start timer with given period 
 * 
 * \param events Pointer to a structure as returned by event_create()
 * \param event   Callback to be called
 * \param arg     Optional argument to be passed to the event.
 * \param period  Time to first occurence.
 * \return 0 On success or -1 on error.
 */
int event_start(event_t *events, event_callback_t event, void *arg, clock_t period);

/** Stop timer, matches only those with equal event and arg.
 * 
 * \param events Pointer to a structure as returned by event_create()
 * \param event   Callback to be called
 * \param arg     Optional argument to be passed to the event.
 * \return 0 On success or -1 on error.
 */
int event_stop(event_t *events, event_callback_t event, void *arg);

/** Stop all timers related to the same event callback 
 * 
 * \param events Pointer to a structure as returned by event_create()
 * \param event   Callback to be called
 * \return 0 On success or -1 on error.
 */
int event_stopall(event_t *events, event_callback_t event);

/** Get present time of given timer 
 * 
 * \param events Pointer to a structure as returned by event_create()
 * \param event   Callback to be called
 * \param arg     Optional argument to be passed to the event.
 * \return 0 On success or -1 on error.
 */
clock_t event_gettime(event_t *events, event_callback_t event, void *arg);

/** Bind two events 
 * 
 * \param source_events Pointer to source event structure.
 * \param source_event   Source event 
 * \param dest_events   Pointer to destination event structure
 * \param dest_event     Destination event to be called, when source fires
 * \return 0 On success or -1 on error.
 */
int event_connect(event_t *source_events, event_callback_t source_event, 
                  event_t *dest_events, event_callback_t dest_event);
    
/** Unbind two events 
 * 
 * \param source_events Pointer to source event structure.
 * \param source_event   Source event 
 * \param dest_events   Pointer to destination event structure
 * \param dest_event     Destination event to be called, when source fires
 * \return 0 On success or -1 on error.
 */
int event_disconnect(event_t *source_events, event_callback_t source_event, 
                     event_t *dest_events, event_callback_t dest_event);

/** Unbind all events related to given destination
 * 
 * \param dest_events   Pointer to destination event structure
 * \param dest_event     Destination event to be called, when source fires
 * \return 0 On success or -1 on error.
 */
int event_disconnectall(event_t *dest_events, event_callback_t dest_event);

/** Handle callbacks 
 * 
 * \param events Pointer to a structure as returned by event_create()
 * \param timeout Time to wait for a callback to be served.
 * \return Remaining time.
 * */
clock_t event_handle(event_t *events, clock_t timeout);


#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_NUTTX_EVENT_H */
#endif /* CONFIG_EVENT */
