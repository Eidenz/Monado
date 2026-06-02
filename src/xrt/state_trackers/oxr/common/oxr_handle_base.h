// Copyright 2019, Collabora, Ltd.
// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Contains handle-related functions and defines only required in a few
 * locations.
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup oxr_main
 */

#pragma once

#include "../oxr_logger.h"

#include "oxr_handle_array.h"

#include <stdlib.h>


#ifdef __cplusplus
extern "C" {
#endif

// Forward declare
struct oxr_handle_base;

/*!
 * Function pointer type for a handle destruction function.
 *
 * @relates oxr_handle_base
 */
typedef XrResult (*oxr_handle_destroyer)(struct oxr_logger *log, struct oxr_handle_base *hb);

/*!
 * State of a handle base, to reduce likelihood of going "boom" on
 * out-of-order destruction or other unsavory behavior.
 *
 * @ingroup oxr_main
 */
enum oxr_handle_state
{
	/*! State during/before oxr_handle_init, or after failure */
	OXR_HANDLE_STATE_UNINITIALIZED = 0,

	/*! State after successful oxr_handle_init */
	OXR_HANDLE_STATE_LIVE,

	/*! State after successful oxr_handle_destroy */
	OXR_HANDLE_STATE_DESTROYED,
};

/*!
 * Used to hold diverse child handles and ensure orderly destruction.
 *
 * Each object referenced by an OpenXR handle should have one of these as its
 * first element, thus "extending" this class.
 */
struct oxr_handle_base
{
	//! Magic (per-handle-type) value for debugging.
	uint64_t debug;

	/*!
	 * Pointer to this object's parent handle holder, if any.
	 */
	struct oxr_handle_base *parent;

	/*!
	 * Array of children, if any.
	 */
	struct oxr_handle_array children;

	/*!
	 * Current handle state.
	 */
	enum oxr_handle_state state;

	/*!
	 * Destroy the object this handle refers to.
	 */
	oxr_handle_destroyer destroy;
};

/*
 *
 * oxr_handle_base.c
 *
 */

/*!
 * Initialize a handle holder, and if a parent is specified, update its child
 * list to include this handle.
 *
 * @protected @memberof oxr_handle_base
 */
XrResult
oxr_handle_init(struct oxr_logger *log,
                struct oxr_handle_base *hb,
                uint64_t debug,
                oxr_handle_destroyer destroy,
                struct oxr_handle_base *parent);

/*!
 * Recursively destroys a handle and all of its children, removing it from its parent if it has one.
 *
 * oxr_handle_destroy wraps this to provide some extra output and start `level`
 * at 0. `level`, which is reported in debug output, is the current depth of
 * recursion.
 *
 * @protected @memberof oxr_handle_base
 */
XrResult
oxr_handle_destroy_internal(struct oxr_logger *log, struct oxr_handle_base *hb, int level);

/*!
 * Allocate some memory for use as a handle, and initialize it as a handle.
 *
 * Mainly for internal use - use OXR_ALLOCATE_HANDLE instead which wraps this.
 *
 * @relates oxr_handle_base
 */
XrResult
oxr_handle_allocate_and_init(struct oxr_logger *log,
                             size_t size,
                             uint64_t debug,
                             oxr_handle_destroyer destroy,
                             struct oxr_handle_base *parent,
                             void **out);
/*!
 * Allocates memory for a handle and evaluates to an XrResult.
 *
 * @param LOG pointer to struct oxr_logger
 * @param OUT the pointer to handle struct type you already created.
 * @param DEBUG Magic per-type debugging constant
 * @param DESTROY Handle destructor function
 * @param PARENT a parent handle, if any
 *
 * Use when you want to do something other than immediately returning in case of
 * failure. If returning immediately is OK, see OXR_ALLOCATE_HANDLE_OR_RETURN().
 *
 * @relates oxr_handle_base
 */
#define OXR_ALLOCATE_HANDLE(LOG, OUT, DEBUG, DESTROY, PARENT)                                                          \
	oxr_handle_allocate_and_init(LOG, sizeof(*OUT), DEBUG, DESTROY, PARENT, (void **)&OUT)

/*!
 * Allocate memory for a handle, returning in case of failure.
 *
 * @param LOG pointer to struct oxr_logger
 * @param OUT the pointer to handle struct type you already created.
 * @param DEBUG Magic per-type debugging constant
 * @param DESTROY Handle destructor function
 * @param PARENT a parent handle, if any
 *
 * Will return an XrResult from the current function if something fails.
 * If that's not OK, see OXR_ALLOCATE_HANDLE().
 *
 * @relates oxr_handle_base
 */
#define OXR_ALLOCATE_HANDLE_OR_RETURN(LOG, OUT, DEBUG, DESTROY, PARENT)                                                \
	do {                                                                                                           \
		XrResult allocResult = OXR_ALLOCATE_HANDLE(LOG, OUT, DEBUG, DESTROY, PARENT);                          \
		if (allocResult != XR_SUCCESS) {                                                                       \
			return allocResult;                                                                            \
		}                                                                                                      \
	} while (0)

#ifdef __cplusplus
}
#endif
