// Copyright 2025-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Tracking objects to IDs.
 * @author Jakob Bornecrantz <tbornecrantz@nvidia.com>
 * @ingroup ipc_server
 */

#pragma once

#include "xrt/xrt_results.h"

struct xrt_space;
struct xrt_hand_tracker;
struct ipc_client_state;


/*
 *
 * Device functions.
 *
 */

/*!
 * Get a device by ID, must only be called from the per client
 * thread as this function accesses the client state's memory.
 *
 * @param ics The client state instance.
 * @param id The device ID.
 * @param out_xdev Will be filled with the device object on success.
 * @return XRT_SUCCESS on success, some other result on failure.
 *
 * @ingroup ipc_server
 */
xrt_result_t
ipc_server_objects_get_xdev_and_validate(volatile struct ipc_client_state *ics,
                                         uint32_t id,
                                         struct xrt_device **out_xdev);

/*!
 * Get a device ID for a given device object, must only be called from the per
 * client thread as this function accesses the client state's memory.
 *
 * @param ics The client state instance.
 * @param xdev The device object.
 * @param out_id Will be filled with the device ID on success.
 * @return XRT_SUCCESS on success, some other result on failure.
 *
 * @ingroup ipc_server
 */
xrt_result_t
ipc_server_objects_get_xdev_id_or_add(volatile struct ipc_client_state *ics, struct xrt_device *xdev, uint32_t *out_id);


/*
 *
 * Tracking origin functions.
 *
 */

/*!
 * Get a tracking origin by ID, must only be called from the per client
 * thread as this function accesses the client state's memory.
 *
 * @param ics The client state instance.
 * @param id The tracking origin ID.
 * @param out_xtrack Will be filled with the tracking origin object on success.
 * @return XRT_SUCCESS on success, some other result on failure.
 *
 * @ingroup ipc_server
 */
xrt_result_t
ipc_server_objects_get_xtrack_and_validate(volatile struct ipc_client_state *ics,
                                           uint32_t id,
                                           struct xrt_tracking_origin **out_xtrack);

/*!
 * Get a tracking origin ID for a given tracking origin object, must only be
 * called from the per client thread as this function accesses the client
 * state's memory.
 *
 * @param ics The client state instance.
 * @param xtrack The tracking origin object.
 * @param out_id Will be filled with the tracking origin ID on success.
 * @return XRT_SUCCESS on success, some other result on failure.
 *
 * @ingroup ipc_server
 */
xrt_result_t
ipc_server_objects_get_xtrack_id_or_add(volatile struct ipc_client_state *ics,
                                        struct xrt_tracking_origin *xtrack,
                                        uint32_t *out_id);


/*
 *
 * Space functions.
 *
 */

/*!
 * Get a space by ID, must only be called from the per client thread.
 *
 * @ingroup ipc_server
 */
xrt_result_t
ipc_server_objects_get_xspc_and_validate(volatile struct ipc_client_state *ics,
                                         uint32_t id,
                                         struct xrt_space **out_xspc);

/*!
 * Track a space and return its ID. Grabs and stores a reference if newly
 * tracked.
 *
 * @ingroup ipc_server
 */
xrt_result_t
ipc_server_objects_get_xspc_id_or_add(volatile struct ipc_client_state *ics, struct xrt_space *xspc, uint32_t *out_id);

/*!
 * Destroy a tracked space.
 *
 * @ingroup ipc_server
 */
xrt_result_t
ipc_server_objects_destroy_xspc(volatile struct ipc_client_state *ics, uint32_t id);


/*
 *
 * Hand tracker functions.
 *
 */

/*!
 * Get a hand tracker by ID, must only be called from the per client thread.
 *
 * @ingroup ipc_server
 */
xrt_result_t
ipc_server_objects_get_xht_and_validate(volatile struct ipc_client_state *ics,
                                        uint32_t id,
                                        struct xrt_hand_tracker **out_xht);

/*!
 * Track a newly created hand tracker and return its ID. Takes ownership of
 * @p xht until explicit destroy or client disconnect.
 *
 * @ingroup ipc_server
 */
xrt_result_t
ipc_server_objects_get_xht_id_or_add(volatile struct ipc_client_state *ics,
                                     struct xrt_hand_tracker *xht,
                                     uint32_t *out_id);

/*!
 * Destroy a tracked hand tracker.
 *
 * @ingroup ipc_server
 */
xrt_result_t
ipc_server_objects_destroy_xht(volatile struct ipc_client_state *ics, uint32_t id);
