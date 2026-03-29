// Copyright 2023, Collabora, Ltd.
// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  IPC Client system devices.
 * @author Korcan Hussein <korcan.hussein@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup ipc_client
 */

#include "util/u_system_helpers.h"
#include "util/u_var.h"

#include "b_system_devices.h"

#include "ipc_client.h"
#include "ipc_client_generated.h"
#include "ipc_client_system_devices.h"


/*
 *
 * Helpers
 *
 */

static inline struct ipc_client_system_devices *
ipc_system_devices(struct xrt_system_devices *xsysd)
{
	return (struct ipc_client_system_devices *)xsysd;
}


/*
 *
 * Member functions.
 *
 */

static xrt_result_t
ipc_client_system_devices_get_roles(struct xrt_system_devices *xsysd, struct xrt_system_roles *out_roles)
{
	struct ipc_client_system_devices *usysd = ipc_system_devices(xsysd);

	return ipc_call_system_devices_get_roles(usysd->ipc_c, out_roles);
}

static xrt_result_t
ipc_client_system_devices_feature_inc(struct xrt_system_devices *xsysd, enum xrt_device_feature_type type)
{
	struct ipc_client_system_devices *usysd = ipc_system_devices(xsysd);
	xrt_result_t xret;

	assert(type < XRT_DEVICE_FEATURE_MAX_ENUM);

	// If it wasn't zero nothing to do.
	if (!xrt_reference_inc_and_was_zero(&usysd->feature_use[type])) {
		return XRT_SUCCESS;
	}

	xret = ipc_call_system_devices_begin_feature(usysd->ipc_c, type);
	IPC_CHK_ALWAYS_RET(usysd->ipc_c, xret, "ipc_call_system_devices_begin_feature");
}

static xrt_result_t
ipc_client_system_devices_feature_dec(struct xrt_system_devices *xsysd, enum xrt_device_feature_type type)
{
	struct ipc_client_system_devices *usysd = ipc_system_devices(xsysd);
	xrt_result_t xret;

	assert(type < XRT_DEVICE_FEATURE_MAX_ENUM);

	// If it is not zero we are done.
	if (!xrt_reference_dec_and_is_zero(&usysd->feature_use[type])) {
		return XRT_SUCCESS;
	}

	xret = ipc_call_system_devices_end_feature(usysd->ipc_c, type);
	IPC_CHK_ALWAYS_RET(usysd->ipc_c, xret, "ipc_call_system_devices_end_feature");
}


static void
ipc_client_system_devices_destroy(struct xrt_system_devices *xsysd)
{
	struct ipc_client_system_devices *usysd = ipc_system_devices(xsysd);

	// Finalize the tracking origin manager (which cleans up all cached tracking origins)
	ipc_client_tracking_origin_manager_fini(&usysd->tracking_origin_manager);

	b_system_devices_close(&usysd->base.base);

	free(usysd);
}


/*
 *
 * 'Exported' functions.
 *
 */

xrt_result_t
ipc_client_system_devices_create(struct ipc_connection *ipc_c, struct ipc_client_system_devices **out_icsd)
{
	struct ipc_client_system_devices *icsd = U_TYPED_CALLOC(struct ipc_client_system_devices);
	icsd->base.base.get_roles = ipc_client_system_devices_get_roles;
	icsd->base.base.destroy = ipc_client_system_devices_destroy;
	icsd->base.base.feature_inc = ipc_client_system_devices_feature_inc;
	icsd->base.base.feature_dec = ipc_client_system_devices_feature_dec;
	icsd->ipc_c = ipc_c;

	// Initialize tracking origin manager
	xrt_result_t xret = ipc_client_tracking_origin_manager_init(&icsd->tracking_origin_manager, ipc_c);
	IPC_CHK_WITH_GOTO(ipc_c, xret, "ipc_client_tracking_origin_manager_init", err_free);

	*out_icsd = icsd;

	return XRT_SUCCESS;

err_free:
	free(icsd);

	return xret;
}
