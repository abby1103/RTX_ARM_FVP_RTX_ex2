// allocate.h: Header file for the allocate.c file
// Copyright (C) 2005  Andrew Greenberg
// Distributed under the GNU GENERAL PUBLIC LICENSE (GPL) Version 2 (June 1991).
// See the "COPYING" file distributed with this software for more information.

#ifndef __THREADS_H
#define __THREADS_H

/*******************************************************************************
 * Definitions
 ******************************************************************************/

// NONE

/*******************************************************************************
 * Declarations
 ******************************************************************************/

// None

/*******************************************************************************
 * Externs
 ******************************************************************************/

extern osThreadId allocate_thread_id;
extern osThreadId display_thread_id;
extern osThreadId message_thread_id;
extern osThreadId measure_thread_id;

// Not really a thread, but an interrupt


#endif // __THREADS_H
