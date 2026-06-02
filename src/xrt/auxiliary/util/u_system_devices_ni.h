// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Not implemented function helpers for system devices.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup aux_util
 */

#pragma once

#include "xrt/xrt_compiler.h"
#include "xrt/xrt_system.h"


#ifdef __cplusplus
extern "C" {
#endif


/*
 *
 * Not implemented function helpers.
 *
 */

/*!
 * Not implemented function for @ref xrt_system_devices::create_hand_tracker.
 *
 * @ingroup aux_util
 */
xrt_result_t
u_system_devices_ni_create_hand_tracker(struct xrt_system_devices *xsysd,
                                        const struct xrt_hand_tracker_create_info *info,
                                        struct xrt_hand_tracker **out_xht);

/*!
 * Not implemented function for @ref xrt_system_devices::feature_inc
 *
 * @ingroup aux_util
 */
xrt_result_t
u_system_devices_ni_feature_inc(struct xrt_system_devices *xsysd, enum xrt_device_feature_type type);

/*!
 * Not implemented function for @ref xrt_system_devices::feature_dec
 *
 * @ingroup aux_util
 */
xrt_result_t
u_system_devices_ni_feature_dec(struct xrt_system_devices *xsysd, enum xrt_device_feature_type type);

#ifdef __cplusplus
}
#endif
