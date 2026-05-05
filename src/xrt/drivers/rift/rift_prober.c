// Copyright 2025, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Prober for finding an Oculus Rift device based on USB VID/PID.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_rift
 */

#include "xrt/xrt_prober.h"
#include "xrt/xrt_config.h"

#include "util/u_misc.h"
#include "util/u_logging.h"
#include "util/u_debug.h"

#include "rift_internal.h"


#ifdef XRT_BUILD_DRIVER_OHMD
#define DEFAULT_ENABLE false
#else
#define DEFAULT_ENABLE true
#endif

DEBUG_GET_ONCE_BOOL_OPTION(rift_prober_enable, "RIFT_PROBER_ENABLE", DEFAULT_ENABLE)

bool
rift_is_oculus(struct xrt_prober *xp, struct xrt_prober_device *dev)
{
	unsigned char manufacturer[128] = {0};
	int result = xrt_prober_get_string_descriptor(xp, dev, XRT_PROBER_STRING_MANUFACTURER, manufacturer,
	                                              sizeof(manufacturer));
	if (result < 0) {
		return false;
	}

	// Some non-oculus devices (VR-Tek HMDs) reuse the same USB IDs as the oculus headsets, so we should check the
	// manufacturer
	if (strncmp((const char *)manufacturer, "Oculus VR, Inc.", sizeof(manufacturer)) != 0) {
		return false;
	}

	return true;
}

int
rift_found(struct xrt_prober *xp,
           struct xrt_prober_device **devices,
           size_t device_count,
           size_t index,
           cJSON *attached_data,
           struct xrt_device **out_xdevs)
{
	struct xrt_prober_device *dev = devices[index];

	// don't do anything if rift probing is not enabled
	if (!debug_get_bool_option_rift_prober_enable()) {
		return -1;
	}

	unsigned char serial_number[21] = {0};
	int result = xrt_prober_get_string_descriptor(xp, dev, XRT_PROBER_STRING_SERIAL_NUMBER, serial_number,
	                                              sizeof(serial_number));
	if (result < 0) {
		return -1;
	}

	if (!rift_is_oculus(xp, dev)) {
		return -1;
	}

	enum rift_variant variant;
	switch (dev->product_id) {
	case OCULUS_DK2_PID: variant = RIFT_VARIANT_DK2; break;
	case OCULUS_CV1_PID: variant = RIFT_VARIANT_CV1; break;
	default: return -1; break;
	}

	U_LOG_I("%s - Found at least the tracker of some Rift -- opening\n", __func__);
	struct os_hid_device *hmd_dev = NULL;
	result = xrt_prober_open_hid_interface(xp, dev, 0, &hmd_dev);
	if (result != 0) {
		return -1;
	}

	struct os_hid_device *radio_dev = NULL;
	if (variant == RIFT_VARIANT_CV1) {
		result = xrt_prober_open_hid_interface(xp, dev, 1, &radio_dev);
		if (result != 0) {
			return -1;
		}
	}

	struct rift_hmd *hmd = NULL;
	int created_devices = rift_devices_create( //
	    hmd_dev,                               //
	    radio_dev,                             //
	    variant,                               //
	    (char *)serial_number,                 //
	    NULL,                                  //
	    &hmd,                                  //
	    out_xdevs);                            //
	if (created_devices < 0) {
		return -1;
	}

	return created_devices;
}
