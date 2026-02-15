// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Header defining the tracking system integration in Monado.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup xrt_iface
 */

#pragma once

#include "xrt/xrt_defines.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef int8_t t_constellation_device_id_t;
typedef int8_t t_constellation_led_id_it;

#define XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME 250

#define XRT_CONSTELLATION_INVALID_DEVICE_ID -1
#define XRT_CONSTELLATION_INVALID_LED_ID -1

/*!
 * A blob is a 2d position in a camera sensor's view that is being tracked. Generally used to represent found LEDs in a
 * camera's sensor.
 *
 * Blobs are given in pixel coordinates, with the origin at the top left of the image, and x going right and y going
 * down. The units are in pixels, but may be subpixel accurate. The tracking system is expected to handle the
 * undistortion of the blob positions.
 */
struct t_blob
{
	/*!
	 * The ID of a blob, which may be used to track it across frames. The meaning of the ID is up to the tracking
	 * system, but it attempts to be consistent across frames for the same blob.
	 */
	uint32_t blob_id;

	/*!
	 * The device ID this blob is associated with, if any. XRT_CONSTELLATION_INVALID_DEVICE_ID for unmatched.
	 * The tracker is expected to fill this in.
	 */
	t_constellation_device_id_t matched_device_id;

	/*!
	 * The LED ID this blob is associated with, if any. XRT_CONSTELLATION_INVALID_LED_ID for unmatched.
	 * The tracker is expected to fill this in.
	 */
	t_constellation_led_id_it matched_device_led_id;

	//! Centre of blob
	struct xrt_vec2 center;

	//! Estimated motion vector of blob, in pixels per second. Only valid if the tracking system provides it.
	struct xrt_vec2 motion_vector;

	//! The bounding box of the blob in pixel coordinates.
	struct xrt_rect bounding_box;

	//! The size of the blob, in pixels. May be {0,0}, and may be subpixel accurate.
	struct xrt_vec2 size;
};

struct t_blob_observation
{
	struct t_blobwatch *source;

	//! Internal ID for this observation, may be set by the blobwatch implementation if it needs to know this.
	uint64_t id;

	int64_t timestamp_ns;
	struct t_blob *blobs;
	uint32_t num_blobs;
};

/*!
 * @interface t_blob_sink
 *
 * A generic interface to allow a tracking system to receive "snapshots" of seen @ref t_blob in a frame.
 */
struct t_blob_sink
{
	/*!
	 * Push a set of blobs into the sink. The tracking system will typically call this once per frame for each
	 * camera view.
	 *
	 * @param[in] tbs The sink to push the blobs into.
	 * @param[in] observation The blob observation to push into the sink.
	 */
	void (*push_blobs)(struct t_blob_sink *tbs, struct t_blob_observation *observation);

	/*!
	 * Destroy this blob sink.
	 */
	void (*destroy)(struct t_blob_sink *tbs);
};

struct t_blobwatch
{
	/*!
	 * Notify the blobwatch that the blobs in the given observation with the correct ID set are associated with the
	 * given device. The blobwatch can use this information to track which blobs are associated with which devices
	 * across frames, and to provide this information to the tracker across frames to save it from doing that work
	 * again.
	 *
	 * @param[in] tbw The blobwatch to mark the blobs for.
	 * @param[in] tbo The observation containing the blobs to mark. The blobwatch will look at the blob IDs and the
	 * matched_device_id field to determine which blobs internally to mark with the given device ID.
	 * @param[in] device_id The device ID to mark
	 */
	void (*mark_blob_device)(struct t_blobwatch *tbw,
	                         const struct t_blob_observation *tbo,
	                         t_constellation_device_id_t device_id);

	/*!
	 * Destroy this blobwatch.
	 */
	void (*destroy)(struct t_blobwatch *tbw);
};


//! @public @memberof t_blob_sink
XRT_NONNULL_ALL static inline void
t_blob_sink_push_blobs(struct t_blob_sink *tbs, struct t_blob_observation *tbo)
{
	tbs->push_blobs(tbs, tbo);
}

//! @public @memberof t_blob_sink
XRT_NONNULL_ALL static inline void
t_blob_sink_destroy(struct t_blob_sink **tbs_ptr)
{
	struct t_blob_sink *tbs = *tbs_ptr;

	if (tbs == NULL) {
		return;
	}

	tbs->destroy(tbs);
	*tbs_ptr = NULL;
}

//! @public @memberof t_blobwatch
XRT_NONNULL_ALL static inline void
t_blobwatch_mark_blob_device(struct t_blobwatch *tbw,
                             const struct t_blob_observation *tbo,
                             t_constellation_device_id_t device_id)
{
	tbw->mark_blob_device(tbw, tbo, device_id);
}

//! @public @memberof t_blobwatch
XRT_NONNULL_ALL static inline void
t_blobwatch_destroy(struct t_blobwatch **tbw_ptr)
{
	struct t_blobwatch *tbw = *tbw_ptr;

	if (tbw == NULL) {
		return;
	}

	tbw->destroy(tbw);
	*tbw_ptr = NULL;
}

#ifdef __cplusplus
}
#endif
