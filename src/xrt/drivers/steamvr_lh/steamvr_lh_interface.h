// Copyright 2023, Shawn Wallace
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief SteamVR driver device interface.
 * @author Shawn Wallace <yungwallace@live.com>
 * @ingroup drv_steamvr_lh
 */

#include <xrt/xrt_results.h>

#include <stdbool.h>

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @defgroup drv_steamvr_lh Wrapper for the SteamVR Lighthouse driver.
 * @ingroup drv
 *
 * @brief Wrapper driver around the SteamVR Lighthouse driver.
 */

/*!
 * @dir drivers/steamvr_lh
 *
 * @brief @ref drv_steamvr_lh files.
 */

/*!
 * Creates the steamvr system devices.
 *
 * @ingroup drv_steamvr_lh
 */
enum xrt_result
steamvr_lh_create_devices(struct xrt_prober *xp, struct xrt_system_devices **out_xsysd);

/*!
 * Callback invoked when a device is hotplugged after
 * @ref steamvr_lh_create_devices returned, right before it is appended to
 * @ref xrt_system_devices::static_xdevs.
 *
 * @ingroup drv_steamvr_lh
 */
typedef void (*steamvr_lh_device_added_callback)(struct xrt_device *xdev, void *userdata);

/*!
 * Register a callback for hotplugged devices, used by the builder to add new
 * devices to the space overseer. Must be called before any client connects,
 * right after @ref steamvr_lh_create_devices.
 *
 * @ingroup drv_steamvr_lh
 */
void
steamvr_lh_set_device_added_callback(struct xrt_system_devices *xsysd,
                                     steamvr_lh_device_added_callback callback,
                                     void *userdata);

/*!
 * Whether @p xdev is a steamvr_lh device whose physical hardware is currently
 * powered on. Returns false for devices not owned by this driver.
 *
 * @ingroup drv_steamvr_lh
 */
bool
steamvr_lh_device_is_connected(struct xrt_device *xdev);


#ifdef __cplusplus
}
#endif
