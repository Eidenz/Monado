// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  UDCAP driver interface (consumed by the SteamVR builder).
 * @ingroup drv_udcap
 */

#pragma once

#include "xrt/xrt_compiler.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

struct xrt_device;

/*!
 * @defgroup drv_udcap UDCAP VR glove driver
 * @ingroup drv
 *
 * @brief Hand tracking + controller inputs for UDCAP (Udexreal) VR gloves,
 * sourced from the udcap-server bridge over shared memory. Finger pose is
 * computed by the gloves; the hand's 6DoF pose comes from a Lighthouse tracker
 * (e.g. a Vive Tracker) attached to it via a tracking override.
 */

/*!
 * Create UDCAP hand devices and attach each to a matching tracker from @p devs.
 *
 * Tracker<->hand pairing is taken from the env vars UDCAP_TRACKER_LEFT and
 * UDCAP_TRACKER_RIGHT (tracker serials); if unset, the first two trackers found
 * are used (left then right) with a warning.
 *
 * @param devs        Candidate devices to search for trackers (e.g. xsysd static xdevs).
 * @param dev_count   Number of entries in @p devs.
 * @param out_left    Set to the left-hand device (tracking-override wrapped) if created.
 * @param out_right   Set to the right-hand device if created.
 */
void
udcap_create_devices(struct xrt_device *const *devs,
                     size_t dev_count,
                     struct xrt_device **out_left,
                     struct xrt_device **out_right);

#ifdef __cplusplus
}
#endif
