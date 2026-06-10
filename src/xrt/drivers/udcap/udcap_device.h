// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  UDCAP hand-tracking device (reads the udcap-server shared memory).
 * @ingroup drv_udcap
 */

#pragma once

#include "xrt/xrt_device.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Create one UDCAP hand-tracking device for the given hand. Returns NULL if the
 * shared-memory segment published by udcap-server is not available.
 */
struct xrt_device *
udcap_device_create(enum xrt_hand hand);

/*!
 * Attach the device to a Lighthouse tracker that provides its 6DoF pose. The
 * device applies the per-hand offset (read live from shm) on top of the tracker
 * pose. Borrowed pointer; the tracker is not owned/destroyed by the device.
 */
void
udcap_device_set_tracker(struct xrt_device *xdev, struct xrt_device *tracker);

#ifdef __cplusplus
}
#endif
