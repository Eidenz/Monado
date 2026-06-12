// Copyright 2020-2023, Collabora, Ltd.
// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  IPC client @ref xrt_system_devices implementation struct.
 * @ingroup ipc_client
 */

#pragma once

#include "os/os_threading.h"

#include "b_system_devices.h"
#include "ipc_client_tracking_origin.h"


#ifdef __cplusplus
extern "C" {
#endif

struct ipc_connection;

/*!
 * Client side implementation of the system devices struct.
 */
struct ipc_client_system_devices
{
	//! @public Base
	struct b_system_devices base;

	//! Connection to service.
	struct ipc_connection *ipc_c;

	//! Tracking origin manager for on-demand fetching
	struct ipc_client_tracking_origin_manager tracking_origin_manager;

	struct xrt_reference feature_use[XRT_DEVICE_FEATURE_MAX_ENUM];

	//! Protects device list refreshes (hotplug).
	struct os_mutex refresh_mutex;
};

/*!
 * Fetch the device list from the server and create client proxies for any
 * device we don't have yet. Devices are placed at the index matching their
 * server-side ID so that role indices resolve to the right device; existing
 * proxies are never moved or destroyed (the device list is append-only).
 *
 * Called at connect to build the initial list, and from get_roles whenever
 * the server reports more devices than we know about (hotplug).
 */
xrt_result_t
ipc_client_system_devices_refresh(struct ipc_client_system_devices *icsd);

#ifdef __cplusplus
}
#endif
