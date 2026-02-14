// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Rift prober code.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup xrt_iface
 */

#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_prober.h"
#include "xrt/xrt_frameserver.h"

#include "tracking/t_constellation.h"

#include "constellation/t_rift_blobwatch.h"

#include "util/u_builders.h"
#include "util/u_debug.h"
#include "util/u_misc.h"
#include "util/u_logging.h"
#include "util/u_system_helpers.h"
#include "util/u_trace_marker.h"
#include "util/u_var.h"
#include "util/u_sink.h"

#include "rift/rift_interface.h"

#ifdef XRT_BUILD_DRIVER_RIFT_SENSOR
#include "rift_sensor/rift_sensor_interface.h"
#endif


// Require a pixel of this brightness to be included in a blob at all, to help filter out general noise.
#define RIFT_SENSOR_BLOB_REQUIRED_THRESHOLD 0x40
// On CV1, the camera tends to detect very little noise, so we can be much more lenient with what pixels are considered
// for being a blob.
#define RIFT_SENSOR_PIXEL_THRESHOLD_CV1 0x24
// On DK2, the camera sees much more noise, so we need to be much stricter about what pixels get included as a blob.
#define RIFT_SENSOR_PIXEL_THRESHOLD_DK2 0x7f

/*
 *
 * Internal structures
 *
 */

struct rift_builder
{
	struct u_builder base;

	enum u_logging_level log_level;

	struct rift_hmd *hmd;

#ifdef XRT_BUILD_DRIVER_RIFT_SENSOR
	struct rift_sensor_context *sensor_context;
	struct rift_sensor **sensors;
	size_t num_sensors;

	struct t_blobwatch **blobwatches;
	struct u_sink_debug *blobwatch_debug_sinks;
#endif
};

static struct rift_builder *
rift_builder(struct xrt_builder *xb)
{
	return (struct rift_builder *)xb;
}

/*
 *
 * Misc stuff.
 *
 */

#ifdef XRT_BUILD_DRIVER_OHMD
#define DEFAULT_ENABLE false
#else
#define DEFAULT_ENABLE true
#endif

DEBUG_GET_ONCE_LOG_OPTION(rift_log, "RIFT_LOG", U_LOGGING_WARN)
DEBUG_GET_ONCE_BOOL_OPTION(rift_prober_enable, "RIFT_PROBER_ENABLE", DEFAULT_ENABLE)

#undef DEFAULT_ENABLE

#define RIFT_ERROR(p, ...) U_LOG_IFL_E(p->log_level, __VA_ARGS__)
#define RIFT_WARN(p, ...) U_LOG_IFL_W(p->log_level, __VA_ARGS__)
#define RIFT_DEBUG(p, ...) U_LOG_IFL_D(p->log_level, __VA_ARGS__)

static const char *driver_list[] = {
    "rift",
};


/*
 *
 * Member functions.
 *
 */

static xrt_result_t
rift_estimate_system(struct xrt_builder *xb,
                     cJSON *config,
                     struct xrt_prober *xp,
                     struct xrt_builder_estimate *estimate)
{
	struct rift_builder *rb = rift_builder(xb);

	struct xrt_prober_device **xpdevs = NULL;
	size_t xpdev_count = 0;
	xrt_result_t xret = XRT_SUCCESS;

	U_ZERO(estimate);

	if (!debug_get_bool_option_rift_prober_enable()) {
		return XRT_SUCCESS;
	}

	xret = xrt_prober_lock_list(xp, &xpdevs, &xpdev_count);
	if (xret != XRT_SUCCESS) {
		return xret;
	}

	struct xrt_prober_device *dev =
	    u_builder_find_prober_device(xpdevs, xpdev_count, OCULUS_VR_VID, OCULUS_CV1_PID, XRT_BUS_TYPE_USB);
	if (dev != NULL && rift_is_oculus(xp, dev)) {
		estimate->certain.head = true;

		// We *might* have controllers
		estimate->maybe.left = true;
		estimate->maybe.right = true;

		// We *might* have a tracker and a remote
		estimate->maybe.extra_device_count = 2;
	}

	dev = u_builder_find_prober_device(xpdevs, xpdev_count, OCULUS_VR_VID, OCULUS_DK2_PID, XRT_BUS_TYPE_USB);
	if (dev != NULL && rift_is_oculus(xp, dev)) {
		estimate->certain.head = true;
	}

	RIFT_DEBUG(rb, "Rift builder estimate: head %d, left %d, right %d, extra %d", estimate->certain.head,
	           estimate->maybe.left, estimate->maybe.right, estimate->maybe.extra_device_count);

	xret = xrt_prober_unlock_list(xp, &xpdevs);
	assert(xret == XRT_SUCCESS);

	return XRT_SUCCESS;
}

static xrt_result_t
rift_open_system_impl(struct xrt_builder *xb,
                      cJSON *config,
                      struct xrt_prober *xp,
                      struct xrt_tracking_origin *origin,
                      struct xrt_system_devices *xsysd,
                      struct xrt_frame_context *xfctx,
                      struct u_builder_roles_helper *ubrh)
{
	struct rift_builder *rb = rift_builder(xb);

	struct xrt_prober_device **xpdevs = NULL;
	size_t xpdev_count = 0;
	xrt_result_t xret = XRT_SUCCESS;
	int ret;
	(void)ret; // Avoid unused variable warning when sensors are disabled.

	DRV_TRACE_MARKER();

	xret = xrt_prober_lock_list(xp, &xpdevs, &xpdev_count);
	if (xret != XRT_SUCCESS) {
		goto unlock_and_fail;
	}

	enum rift_variant variant = RIFT_VARIANT_CV1;

	struct xrt_prober_device *head_xpdev =
	    u_builder_find_prober_device(xpdevs, xpdev_count, OCULUS_VR_VID, OCULUS_CV1_PID, XRT_BUS_TYPE_USB);

	// If there's no CV1, search for a DK2
	if (head_xpdev == NULL) {
		head_xpdev =
		    u_builder_find_prober_device(xpdevs, xpdev_count, OCULUS_VR_VID, OCULUS_DK2_PID, XRT_BUS_TYPE_USB);

		if (head_xpdev != NULL) {
			variant = RIFT_VARIANT_DK2;
		}
	}

	if (head_xpdev != NULL && rift_is_oculus(xp, head_xpdev)) {
		unsigned char serial_number[21] = {0};
		int result = xrt_prober_get_string_descriptor(xp, head_xpdev, XRT_PROBER_STRING_SERIAL_NUMBER,
		                                              serial_number, sizeof(serial_number));
		if (result < 0) {
			return -1;
		}

		struct os_hid_device *hmd_hid_dev = NULL;
		result = xrt_prober_open_hid_interface(xp, head_xpdev, 0, &hmd_hid_dev);
		if (result != 0) {
			return -1;
		}

		struct os_hid_device *radio_hid_dev = NULL;
		if (variant == RIFT_VARIANT_CV1) {
			result = xrt_prober_open_hid_interface(xp, head_xpdev, 1, &radio_hid_dev);
			if (result != 0) {
				return -1;
			}
		}

		struct xrt_device *xdevs[XRT_SYSTEM_MAX_DEVICES] = {0};
		int created_devices =
		    rift_devices_create(hmd_hid_dev, radio_hid_dev, variant, (char *)serial_number, &rb->hmd, xdevs);
		if (rb->hmd == NULL) {
			RIFT_ERROR(rb, "Rift HMD device creation failed");
			goto unlock_and_fail;
		}

		if (created_devices < 0) {
			RIFT_ERROR(rb, "Rift HMD device creation failed with code %d", created_devices);
			goto unlock_and_fail;
		}

		// Just clamp instead of overflowing the buffer
		if (created_devices + xsysd->xdev_count > XRT_SYSTEM_MAX_DEVICES) {
			created_devices = XRT_SYSTEM_MAX_DEVICES - xsysd->xdev_count;
		}

		memcpy(xsysd->xdevs + xsysd->xdev_count, xdevs, sizeof(struct xrt_device *) * created_devices);
		xsysd->xdev_count += created_devices;

		for (int i = 0; i < created_devices; i++) {
			struct xrt_device *xdev = xdevs[i];
			switch (xdev->device_type) {
			case XRT_DEVICE_TYPE_HMD: ubrh->head = xdev; break;
			case XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER: ubrh->left = xdev; break;
			case XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER: ubrh->right = xdev; break;
			case XRT_DEVICE_TYPE_GAMEPAD: ubrh->gamepad = xdev; break;
			default: break;
			}
		}
	}

	xret = xrt_prober_unlock_list(xp, &xpdevs);
	if (xret != XRT_SUCCESS) {
		goto fail;
	}

#ifdef XRT_BUILD_DRIVER_RIFT_SENSOR
	ret = rift_sensor_context_create(&rb->sensor_context, xfctx);
	if (ret != 0) {
		RIFT_WARN(rb, "Rift sensor context creation failed with code %d", ret);
	}

	uint8_t radio_id[5] = {0};
	if (rb->hmd) {
		rift_get_radio_id(rb->hmd, radio_id);
	}

	if (rb->sensor_context) {
		ret = rift_sensor_context_start(rb->sensor_context);
		if (ret != 0) {
			RIFT_WARN(rb, "Rift sensor context start failed with code %d", ret);
		}

		ssize_t num_sensors = rift_sensor_context_get_sensors(rb->sensor_context, &rb->sensors);
		if (num_sensors < 0) {
			RIFT_WARN(rb, "Rift sensor context get sensors failed with code %d", (int)num_sensors);
		}

		rb->blobwatches = U_TYPED_ARRAY_CALLOC(struct t_blobwatch *, num_sensors);
		rb->blobwatch_debug_sinks = U_TYPED_ARRAY_CALLOC(struct u_sink_debug, num_sensors);

		for (ssize_t i = 0; i < num_sensors; i++) {
			struct rift_sensor *sensor = rb->sensors[i];

			enum rift_variant sensor_variant = rift_sensor_get_variant(sensor);
			if (sensor_variant != variant) {
				continue;
			}

			struct u_sink_debug *debug_sink = &rb->blobwatch_debug_sinks[rb->num_sensors];
			struct t_blobwatch **blobwatch = &rb->blobwatches[rb->num_sensors];

			u_sink_debug_init(debug_sink);

			struct t_blob_sink *blob_sink;
			u_sink_blob_visualizer_create(xfctx, NULL, debug_sink, RIFT_SENSOR_WIDTH, RIFT_SENSOR_HEIGHT,
			                              &blob_sink);

			struct xrt_frame_sink *frame_sink;
			struct t_rift_blobwatch_params params = {
			    .pixel_threshold = variant == RIFT_VARIANT_CV1 ? RIFT_SENSOR_PIXEL_THRESHOLD_CV1
			                                                   : RIFT_SENSOR_PIXEL_THRESHOLD_DK2,
			    .blob_required_threshold = RIFT_SENSOR_BLOB_REQUIRED_THRESHOLD,
			    .max_match_dist = 50.0f,
			};
			ret = t_rift_blobwatch_create(&params, xfctx, blob_sink, &frame_sink, blobwatch);
			if (ret != 0) {
				RIFT_WARN(rb, "Failed to create Rift blobwatch for sensor %zd with code %d", i, ret);
				continue;
			}

			u_sink_create_format_converter(xfctx, XRT_FORMAT_L8, frame_sink, &frame_sink);

			if (!u_sink_simple_queue_create(xfctx, frame_sink, &frame_sink)) {
				RIFT_WARN(rb, "Failed to create Rift blobwatch queue for sensor %zd", i);
				continue;
			}

			struct xrt_fs *fs = rift_sensor_get_frame_server(sensor);
			if (!xrt_fs_stream_start(fs, frame_sink, XRT_FS_CAPTURE_TYPE_TRACKING, 0)) {
				RIFT_WARN(rb, "Failed to start Rift sensor frame server stream for sensor %zd", i);
				continue;
			}

			u_var_add_sink_debug(rb, debug_sink, "Sensor Blobwatch");
			RIFT_DEBUG(rb, "Rift sensor %zd initialized and streaming at index %zd", i, rb->num_sensors);

			rb->num_sensors++;

			// @note radio_id may be null on DK2, but DK2 doesn't use the radio ID so this is fine.
			ret = rift_sensor_enable_exposure_sync(rb->sensor_context, sensor, radio_id);
			if (ret != 0) {
				RIFT_WARN(rb, "Rift sensor context exposure sync enable failed with code %d", ret);
			}
		}
	}
#endif

	return XRT_SUCCESS;

unlock_and_fail:
	xret = xrt_prober_unlock_list(xp, &xpdevs);
	if (xret != XRT_SUCCESS) {
		return xret;
	}

	/* Fallthrough */
fail:
	return XRT_ERROR_DEVICE_CREATION_FAILED;
}

static void
rift_destroy(struct xrt_builder *xb)
{
	struct rift_builder *rb = rift_builder(xb);

#ifdef XRT_BUILD_DRIVER_RIFT_SENSOR
	if (rb->sensors) {
		free(rb->sensors);
	}

	if (rb->blobwatch_debug_sinks) {
		for (size_t i = 0; i < rb->num_sensors; i++) {
			u_sink_debug_destroy(&rb->blobwatch_debug_sinks[i]);
		}

		free(rb->blobwatch_debug_sinks);
		rb->blobwatch_debug_sinks = NULL;
	}

	if (rb->blobwatches) {
		// @note The blobwatches are freed when their frame nodes are destroyed, so we don't need to destroy
		// them here, just free the array.

		free(rb->blobwatches);
		rb->blobwatches = NULL;
	}
#endif

	u_var_remove_root(rb);

	free(xb);
}


/*
 *
 * 'Exported' functions.
 *
 */

struct xrt_builder *
rift_builder_create(void)
{
	struct rift_builder *rb = U_TYPED_CALLOC(struct rift_builder);

	rb->log_level = debug_get_log_option_rift_log();

	// xrt_builder fields.
	rb->base.base.estimate_system = rift_estimate_system;
	rb->base.base.open_system = u_builder_open_system_static_roles;
	rb->base.base.destroy = rift_destroy;
	rb->base.base.identifier = "rift";
	rb->base.base.name = "Oculus Rift";
	rb->base.base.driver_identifiers = driver_list;
	rb->base.base.driver_identifier_count = ARRAY_SIZE(driver_list);

	// u_builder fields.
	rb->base.open_system_static_roles = rift_open_system_impl;

	u_var_add_root(rb, "Rift Builder", false);
	u_var_add_log_level(rb, &rb->log_level, "Log Level");

	return &rb->base.base;
}
