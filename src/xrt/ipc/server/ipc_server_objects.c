// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Tracking objects to IDs.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup ipc_server
 */

#include "xrt/xrt_device.h"
#include "xrt/xrt_tracking.h"

#include "shared/ipc_protocol.h"
#include "server/ipc_server.h"
#include "server/ipc_server_objects.h"

#include <assert.h>
#include <string.h>


/*
 *
 * Device functions.
 *
 */

xrt_result_t
ipc_server_objects_get_xdev_and_validate(volatile struct ipc_client_state *ics,
                                         uint32_t id,
                                         struct xrt_device **out_xdev)
{
	if (id >= XRT_SYSTEM_MAX_DEVICES) {
		IPC_ERROR(ics->server, "Invalid device ID %u (>= XRT_SYSTEM_MAX_DEVICES)", id);
		return XRT_ERROR_IPC_FAILURE;
	}

	struct xrt_device *xdev = ics->objects.xdevs[id];
	if (xdev == NULL) {
		IPC_ERROR(ics->server, "Device ID %u not found (NULL)", id);
		return XRT_ERROR_IPC_FAILURE;
	}

	*out_xdev = xdev;

	return XRT_SUCCESS;
}

xrt_result_t
ipc_server_objects_get_xdev_id_or_add(volatile struct ipc_client_state *ics, struct xrt_device *xdev, uint32_t *out_id)
{
	assert(out_id != NULL);
	assert(xdev != NULL);

	// Check if device already exists and return its ID.
	uint32_t index = 0;
	for (; index < XRT_SYSTEM_MAX_DEVICES; index++) {
		// Found a free slot.
		if (ics->objects.xdevs[index] == NULL) {
			break;
		}
		// Already tracked.
		if (ics->objects.xdevs[index] == xdev) {
			*out_id = index;
			return XRT_SUCCESS;
		}
	}

	if (index >= XRT_SYSTEM_MAX_DEVICES) {
		IPC_ERROR(ics->server, "Failed to find available slot for device: '%s'", xdev->str);
		return XRT_ERROR_IPC_FAILURE;
	}

	// Check that we can also get the tracking origin allocated.
	uint32_t tracking_origin_id = UINT32_MAX;
	xrt_result_t xret = ipc_server_objects_get_xtrack_id_or_add(ics, xdev->tracking_origin, &tracking_origin_id);
	IPC_CHK_AND_RET(ics->server, xret, "ipc_server_objects_get_xtrack_id_or_add");

	ics->objects.xdevs[index] = xdev;

	*out_id = index;

	return XRT_SUCCESS;
}


/*
 *
 * Tracking origin functions.
 *
 */

xrt_result_t
ipc_server_objects_get_xtrack_and_validate(volatile struct ipc_client_state *ics,
                                           uint32_t id,
                                           struct xrt_tracking_origin **out_xtrack)
{
	if (id >= XRT_SYSTEM_MAX_DEVICES) {
		IPC_ERROR(ics->server, "Invalid tracking origin ID %u (>= XRT_SYSTEM_MAX_DEVICES)", id);
		return XRT_ERROR_IPC_FAILURE;
	}

	struct xrt_tracking_origin *xtrack = ics->objects.xtracks[id];
	if (xtrack == NULL) {
		IPC_ERROR(ics->server, "Tracking origin ID %u not found (NULL)", id);
		return XRT_ERROR_IPC_FAILURE;
	}

	*out_xtrack = xtrack;

	return XRT_SUCCESS;
}

xrt_result_t
ipc_server_objects_get_xtrack_id_or_add(volatile struct ipc_client_state *ics,
                                        struct xrt_tracking_origin *xtrack,
                                        uint32_t *out_id)
{
	assert(out_id != NULL);

	// Find the next available slot in xtracks array and assign an ID, or if we find the xtrack return it.
	for (uint32_t index = 0; index < XRT_SYSTEM_MAX_DEVICES; index++) {
		if (ics->objects.xtracks[index] == NULL) {
			ics->objects.xtracks[index] = xtrack;
			*out_id = index;
			return XRT_SUCCESS;
		}
		if (ics->objects.xtracks[index] == xtrack) {
			*out_id = index;
			return XRT_SUCCESS;
		}
	}

	// No available slot or xtrack found
	IPC_ERROR(ics->server, "Failed to find available slot for tracking origin: '%s'", xtrack->name);

	return XRT_ERROR_IPC_FAILURE;
}
