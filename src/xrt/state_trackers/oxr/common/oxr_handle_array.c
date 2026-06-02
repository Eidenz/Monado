// Copyright 2019-2020, Collabora, Ltd.
// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  A dynamic array of handles.
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup oxr_main
 */

#include "util/u_misc.h"

#include "math/m_api.h"

#include "oxr_handle_base.h"

#include "oxr_handle_array.h"


XrResult
oxr_handle_array_destroy(struct oxr_logger *log, struct oxr_handle_array *array, int level)
{
	// Destroy child handles
	for (uint32_t i = 0; i < array->count; ++i) {
		struct oxr_handle_base *child = array->handles[i];

		if (child != NULL) {
			XrResult result = oxr_handle_destroy_internal(log, child, level + 1);
			if (result != XR_SUCCESS) {
				return result;
			}
		}
	}

	free(array->handles);
	array->handles = NULL;
	array->count = 0;
	array->capacity = 0;

	return XR_SUCCESS;
}

bool
oxr_handle_array_add(struct oxr_handle_array *array, struct oxr_handle_base *handle)
{
	// Count should never exceed capacity
	assert(array->count <= array->capacity);

	if (array->count == array->capacity) {
		uint32_t new_capacity = MAX(1, array->capacity * 2);

		U_ARRAY_REALLOC_OR_FREE(array->handles, struct oxr_handle_base *, new_capacity);

		if (array->handles == NULL) {
			return false;
		}

		array->capacity = new_capacity;
	}

	// When adding, we should always have capacity to do so
	assert(array->count < array->capacity);

	array->handles[array->count++] = handle;
	return true;
}

bool
oxr_handle_array_remove(struct oxr_handle_array *array, uint32_t index)
{
	if (index >= array->count) {
		assert(!"Attempted to remove handle at index greater than or equal to count");
		return false;
	}

	// Shift all later handles down by one
	for (uint32_t i = index; i < array->count - 1; ++i) {
		array->handles[i] = array->handles[i + 1];
	}
	array->handles[array->count - 1] = NULL;
	array->count--;

	return true;
}
