// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Internal ures for the constellation tracker.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#pragma once

#include "xrt/xrt_frame.h"
#include "xrt/xrt_tracking.h"

#include "util/u_debug.h"
#include "util/u_logging.h"
#include "util/u_var.h"
#include "util/u_threading.h"
#include "util/u_weak_ptr.hpp"

#include "tracking/t_constellation.h"

#include "math/m_api.h"

#include <vector>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <optional>
#include <stdexcept>
#include <array>

#include "correspondence_search.h"
#include "led_search_model.h"
#include "pose_optimize.h"
#include "pose_metrics.h"
#include "t_constellation_tracker.h"


DEBUG_GET_ONCE_LOG_OPTION(constellation_tracker_log, "CONSTELLATION_TRACKER_LOG", U_LOGGING_WARN)

#define CT_TRACE(ct, ...) U_LOG_IFL_T(ct->log_level, __VA_ARGS__)
#define CT_DEBUG(ct, ...) U_LOG_IFL_D(ct->log_level, __VA_ARGS__)
#define CT_INFO(ct, ...) U_LOG_IFL_I(ct->log_level, __VA_ARGS__)
#define CT_WARN(ct, ...) U_LOG_IFL_W(ct->log_level, __VA_ARGS__)
#define CT_ERROR(ct, ...) U_LOG_IFL_E(ct->log_level, __VA_ARGS__)

#define MIN_ROT_ERROR DEG_TO_RAD(30)
#define MIN_POS_ERROR 0.10

/*
 *
 * Forward declares for C callback functions
 *
 */

extern "C" void
constellation_tracker_camera_push_blobs(t_blob_sink *tbs, t_blob_observation *tbo);

extern "C" void
constellation_tracker_camera_destroy(t_blob_sink *tbs);

extern "C" void *
constellation_tracker_camera_slow_thread(void *ptr);

extern "C" void *
constellation_tracker_camera_fast_thread(void *ptr);

extern "C" void
constellation_tracker_node_break_apart(xrt_frame_node *node);

extern "C" void
constellation_tracker_node_destroy(xrt_frame_node *node);


namespace xrt::tracking::constellation {

namespace os = xrt::auxiliary::os;

// Forward-declares
struct CameraMosaic;
struct Device;
struct Camera;
struct ConstellationTracker;

struct DeviceState
{
	//! The ID of the device.
	t_constellation_device_id_t device_id{XRT_CONSTELLATION_INVALID_DEVICE_ID};

	//! The "predicted" pose, which is the pose the device expects itself to be at at the time of the blobservation.
	std::optional<xrt_pose> Txr_world_device_prior{std::nullopt};

	//! Whether the device needs to have the slow processing thread run over it
	bool needs_slow_processing{false};
};

struct CameraSample
{
public: // Fields
	t_blobwatch *source{nullptr};
	uint64_t id{};
	int64_t timestamp_ns{};
	t_blob blobs[XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME]{};
	uint32_t blob_count{};

	//! The camera's position in the constellation tracker's tracking origin
	std::optional<xrt_pose> Txr_world_cam{std::nullopt};

	std::array<DeviceState, XRT_CONSTELLATION_MAX_DEVICES> device_states{};
	uint32_t device_count{};

public: // Methods
	std::optional<DeviceState *>
	GetDeviceState(t_constellation_device_id_t device_id);

	CameraSample(t_blob_observation &blobservation, Camera *camera);

	t_blob_observation
	ToBlobObservation() const &
	{
		t_blob_observation obs = {
		    .source = this->source,
		    .id = this->id,
		    .timestamp_ns = this->timestamp_ns,
		    .blobs = const_cast<t_blob *>(this->blobs),
		    .num_blobs = this->blob_count,
		};

		return obs;
	}

	t_blob_observation
	ToBlobObservation() && = delete;
	t_blob_observation
	ToBlobObservation() const && = delete;
};

struct Camera
{
public: // Fields
	t_blob_sink base = {
	    .push_blobs = constellation_tracker_camera_push_blobs,
	    .destroy = constellation_tracker_camera_destroy,
	};

	//! The owner tracker, so we can retrieve it from the blob sink callback
	ConstellationTracker *tracker;
	std::weak_ptr<CameraMosaic> mosaic;

	t_camera_calibration calibration;

	camera_model model;

	//! Does "slow" processing for this camera when fast recovery paths fail.
	os_thread_helper slow_processing_thread;
	struct
	{
		std::optional<CameraSample> sample;

		correspondence_search *cs;
	} slow_processing_thread_data;

	/*!
	 * Does "fast" processing for this camera, trying to recover a pose quickly. It's valid for this to happen at
	 * the same time as a slow process.
	 */
	os_thread_helper fast_processing_thread;
	struct
	{
		std::optional<CameraSample> sample;

		correspondence_search *cs;
	} fast_processing_thread_data;

	//! Locks all processing data
	mutable os::Mutex processing_lock;
	//! All data protected by the processing lock
	struct
	{
		xrt_pose Txr_origin_cam;
		bool has_concrete_pose;
	} locked_data;

public: // Methods
	static Camera *
	Get(t_blob_sink *tbs)
	{
		return container_of(tbs, Camera, base);
	}

	Camera(ConstellationTracker *tracker,
	       std::weak_ptr<CameraMosaic> mosaic,
	       const t_constellation_tracker_camera &camera_params,
	       enum u_logging_level *log_level_ptr);

	~Camera();

	// Delete all copy/move ctors, since the pointers for `base` need to be stable
	Camera(const Camera &) = delete;
	Camera(Camera &&) = delete;
	Camera &
	operator=(const Camera &) = delete;
	Camera &
	operator=(Camera &&) = delete;

	std::optional<xrt_pose>
	GetWorldPose(timepoint_ns when_ns);

	void
	DeferSampleToSlowThread(CameraSample &sample);

	//! Fast matching based on prior pose
	bool
	TryDevicePose(std::unique_ptr<Device> &device,
	              CameraSample &sample,
	              xrt_pose &Tcv_cam_world,
	              xrt_pose &Tcv_world_device_prior,
	              xrt_pose &Tcv_world_device_candidate);

	bool
	TryDeviceBlobRecovery(std::unique_ptr<Device> &device,
	                      CameraSample &sample,
	                      xrt_pose &Tcv_cam_world,
	                      xrt_pose &Tcv_world_device_prior);

	void
	SlowSampleProcess(CameraSample &sample);

	//! Returns whether a slow search is needed
	bool
	FastSampleProcess(CameraSample &sample);

	void
	PushPose(CameraSample &sample,
	         std::unique_ptr<Device> &device,
	         pose_metrics &score,
	         xrt_pose &Tcv_cam_device,
	         bool optimize);
};

struct CameraMosaic
{
public: // Fields
	std::vector<std::unique_ptr<Camera>> cameras;

	t_constellation_tracker_tracking_source *tracking_origin;

public: // Methods
	CameraMosaic(ConstellationTracker *tracker, const t_constellation_tracker_camera_mosaic &mosaic_params);

	~CameraMosaic() = default;

	std::optional<xrt_pose>
	GetTrackingOriginPose(timepoint_ns when_ns);
};

struct Device
{
public: // Fields
	t_constellation_tracker_device_params params;
	t_constellation_tracker_device *device;

	t_constellation_device_id_t id;

	// @todo remove when clang-format is updated in CI
	// clang-format off
	t_constellation_search_model *search_model{nullptr};
	xrt_vec3 prior_pos_error{MIN_POS_ERROR, MIN_POS_ERROR, MIN_POS_ERROR};
	xrt_vec3 prior_rot_error{MIN_POS_ERROR, MIN_POS_ERROR, MIN_POS_ERROR};
	float gravity_error_rad{MIN_ROT_ERROR}; /* Gravity vector uncertainty in radians 0..M_PI */

	mutable os::Mutex data_lock;
	struct
	{
		std::optional<xrt_pose> Txr_world_device_last_known;
	} locked_data;
	// clang-format on

public: // Methods
	Device(t_constellation_tracker_device_params *params,
	       t_constellation_tracker_device *device,
	       t_constellation_device_id_t id);

	~Device();
};

struct ConstellationTracker
{
public: // Fields
	xrt_frame_node node = {
	    .next = nullptr,
	    .break_apart = constellation_tracker_node_break_apart,
	    .destroy = constellation_tracker_node_destroy,
	};
	xrt_tracking_origin tracking_origin = {
	    .name = "Constellation Tracker",
	    .type = XRT_TRACKING_TYPE_CONSTELLATION,
	    .initial_offset = XRT_POSE_IDENTITY,
	};

	//! Whether the constellation tracker is running
	bool running = true;

	enum u_logging_level log_level = U_LOGGING_WARN;

	t_constellation_tracker_params params;

	std::vector<std::shared_ptr<CameraMosaic>> mosaics;

	std::shared_mutex device_lock;
	std::vector<std::unique_ptr<Device>> devices;
	t_constellation_device_id_t next_device_id{0};

public: // Methods
	static ConstellationTracker *
	Get(xrt_frame_node *node)
	{
		return container_of(node, ConstellationTracker, node);
	}

	ConstellationTracker(t_constellation_tracker_params *params);

	~ConstellationTracker();

	// Delete all copy/move ctors, since the pointers for `node` need to be stable
	ConstellationTracker(const ConstellationTracker &) = delete;
	ConstellationTracker(ConstellationTracker &&) = delete;
	ConstellationTracker &
	operator=(const ConstellationTracker &) = delete;
	ConstellationTracker &
	operator=(ConstellationTracker &&) = delete;

	t_constellation_device_id_t
	AddDevice(t_constellation_tracker_device_params *params, t_constellation_tracker_device *device);

	void
	RemoveDevice(t_constellation_device_id_t device_id);

	void
	MarkMatchingBlobs(CameraSample &sample,
	                  t_constellation_tracker_led_model &led_model,
	                  t_constellation_device_id_t device_id,
	                  pose_metrics_blob_match_info &blob_match_info);
};

}; // namespace xrt::tracking::constellation
