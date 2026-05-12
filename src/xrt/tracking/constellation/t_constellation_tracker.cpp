// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Implementation of the main logic for the constellation tracker.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#include "t_constellation_tracker_internal.hpp"


namespace xrt::tracking::constellation {

/*
 *
 * Helper functions
 *
 */

static void
get_pose_gravity_vector(xrt_pose &pose, xrt_vec3 &gravity)
{
	// Extract the gravity vector from the pose's orientation
	gravity.x = 0.0f;
	gravity.y = 1.0f;
	gravity.z = 0.0f;
	math_quat_rotate_vec3(&pose.orientation, &gravity, &gravity);
}

/*
 *
 * CameraSample implementations
 *
 */

std::optional<DeviceState *>
CameraSample::GetDeviceState(t_constellation_device_id_t device_id)
{
	for (uint32_t i = 0; i < this->device_count; i++) {
		if (this->device_states[i].device_id == device_id) {
			return &this->device_states[i];
		}
	}

	return std::nullopt;
}

CameraSample::CameraSample(t_blob_observation &blobservation, Camera *camera)
{
	// Copy the blob observation into this sample, since we need the data to be safe.
	this->source = blobservation.source;
	this->id = blobservation.id;
	this->timestamp_ns = blobservation.timestamp_ns;
	memcpy(blobs, blobservation.blobs, sizeof(t_blob) * blobservation.num_blobs);
	this->blob_count = blobservation.num_blobs;

	// Get the camera pose
	this->Txr_world_cam = camera->GetWorldPose(blobservation.timestamp_ns);

	this->device_states = {};
	this->device_count = 0;
}

/*
 *
 * Camera implementations
 *
 */

Camera::Camera(ConstellationTracker *tracker,
               std::weak_ptr<CameraMosaic> mosaic,
               const t_constellation_tracker_camera &camera_params,
               enum u_logging_level *log_level_ptr)
    : tracker(tracker), mosaic(mosaic), calibration(camera_params.calibration)
{
	this->tracker = tracker;
	this->mosaic = mosaic;
	this->calibration = camera_params.calibration;
	this->model = {
	    .width = calibration.image_size_pixels.w,
	    .height = calibration.image_size_pixels.h,
	    .calib = {},
	};
	t_camera_model_params_from_t_camera_calibration(&this->calibration, &this->model.calib);

	this->locked_data = {
	    .Txr_origin_cam = camera_params.pose_in_origin,
	    .has_concrete_pose = camera_params.has_concrete_pose,
	};

	this->slow_processing_thread_data.cs = correspondence_search_new(log_level_ptr, &this->model);
	this->fast_processing_thread_data.cs = correspondence_search_new(log_level_ptr, &this->model);

	if (os_thread_helper_init(&this->slow_processing_thread) < 0) {
		throw std::runtime_error("Slow processing thread failed to init");
	}
	if (os_thread_helper_init(&this->fast_processing_thread) < 0) {
		throw std::runtime_error("Fast processing thread failed to init");
	}

	if (os_thread_helper_start(&this->slow_processing_thread, constellation_tracker_camera_slow_thread, this) < 0) {
		throw std::runtime_error("Starting slow processing thread failed");
	}
	if (os_thread_helper_start(&this->fast_processing_thread, constellation_tracker_camera_fast_thread, this) < 0) {
		throw std::runtime_error("Starting fast processing thread failed");
	}
}

Camera::~Camera()
{
	if (this->slow_processing_thread.initialized) {
		os_thread_helper_destroy(&this->slow_processing_thread);
	}

	if (this->fast_processing_thread.initialized) {
		os_thread_helper_destroy(&this->fast_processing_thread);
	}

	if (this->slow_processing_thread_data.cs) {
		correspondence_search_free(this->slow_processing_thread_data.cs);
		this->slow_processing_thread_data.cs = nullptr;
	}

	if (this->fast_processing_thread_data.cs) {
		correspondence_search_free(this->fast_processing_thread_data.cs);
		this->fast_processing_thread_data.cs = nullptr;
	}
}

std::optional<xrt_pose>
Camera::GetWorldPose(timepoint_ns when_ns)
{
	std::shared_ptr<CameraMosaic> mosaic = this->mosaic.lock();
	U_ASSERT_WEAK_PTR_RET(
	    mosaic, "Camera's mosaic was destroyed while we still had a pointer to it, this should never happen",
	    std::nullopt);

	auto Txr_world_origin = mosaic->GetTrackingOriginPose(when_ns);
	if (!Txr_world_origin.has_value()) {
		return std::nullopt;
	}

	std::unique_lock<os::Mutex> lock(this->processing_lock);

	if (!this->locked_data.has_concrete_pose) {
		return std::nullopt;
	}

	xrt_pose Txr_world_cam;
	math_pose_transform(&Txr_world_origin.value(), &this->locked_data.Txr_origin_cam, &Txr_world_cam);
	return Txr_world_cam;
}

void
Camera::DeferSampleToSlowThread(CameraSample &sample)
{
	os_thread_helper_lock(&this->slow_processing_thread);
	this->slow_processing_thread_data.sample = sample;
	os_thread_helper_signal_locked(&this->slow_processing_thread);
	os_thread_helper_unlock(&this->slow_processing_thread);
}

bool
Camera::TryDevicePose(std::unique_ptr<Device> &device,
                      CameraSample &sample,
                      xrt_pose &Tcv_cam_world,
                      xrt_pose &Tcv_world_device_prior,
                      xrt_pose &Tcv_world_device_candidate)
{
	xrt_pose Tcv_cam_device_prior;
	math_pose_transform(&Tcv_cam_world, &Tcv_world_device_prior, &Tcv_cam_device_prior);

	xrt_pose Tcv_cam_device_candidate;
	math_pose_transform(&Tcv_cam_world, &Tcv_world_device_candidate, &Tcv_cam_device_candidate);

	pose_metrics score;
	pose_metrics_evaluate_pose_with_prior(&score, &Tcv_cam_device_candidate, false, &Tcv_cam_device_prior,
	                                      &device->prior_pos_error, &device->prior_rot_error, sample.blobs,
	                                      sample.blob_count, &device->params.led_model, device->id, &this->model,
	                                      NULL);

	if (POSE_HAS_FLAGS(&score, POSE_MATCH_GOOD | POSE_MATCH_LED_IDS)) {
		this->PushPose(sample, device, score, Tcv_cam_device_candidate, true);
		return true;
	}

	return false;
}

bool
Camera::TryDeviceBlobRecovery(std::unique_ptr<Device> &device,
                              CameraSample &sample,
                              xrt_pose &Tcv_cam_world,
                              xrt_pose &Tcv_world_device_prior)
{
	auto tracker = this->tracker;

	const uint32_t needed_blobs = 4;
	uint32_t num_blobs = 0;
	for (uint32_t index = 0; index < sample.blob_count; index++) {
		t_blob &b = sample.blobs[index];
		if (b.matched_device_id == device->id) {
			num_blobs++;

			if (num_blobs >= needed_blobs) {
				break;
			}
		}
	}
	if (num_blobs < needed_blobs) {
		return false;
	}

	xrt_pose Tcv_cam_device_prior;
	math_pose_transform(&Tcv_cam_world, &Tcv_world_device_prior, &Tcv_cam_device_prior);

	// RANSAC-PnP with the matched blobs
	xrt_pose Tcv_cam_device = Tcv_cam_device_prior;
	if (!ransac_pnp_pose(tracker->log_level, &Tcv_cam_device, sample.blobs, sample.blob_count,
	                     &device->params.led_model, device->id, &this->model, NULL, NULL)) {
		CT_DEBUG(tracker, "Camera %p RANSAC-PnP blob recovery for device %d from %u blobs failed", (void *)this,
		         device->id, sample.blob_count);
		return false;
	}

	// Evaluate the pose
	pose_metrics score;
	pose_metrics_evaluate_pose_with_prior(
	    &score, &Tcv_cam_device, true, &Tcv_cam_device_prior, &device->prior_pos_error, &device->prior_rot_error,
	    sample.blobs, sample.blob_count, &device->params.led_model, device->id, &this->model, NULL);

	if (POSE_HAS_FLAGS(&score, POSE_MATCH_GOOD)) {
		CT_DEBUG(tracker, "Camera %p RANSAC-PnP recovered pose for device %d from %u blobs", (void *)this,
		         device->id, sample.blob_count);
		this->PushPose(sample, device, score, Tcv_cam_device, false);
		return true;
	}

	return false;
}

void
Camera::SlowSampleProcess(CameraSample &sample)
{
	ConstellationTracker *tracker = this->tracker;

	auto &data = this->slow_processing_thread_data;

	CT_TRACE(tracker, "Starting slow processing for camera %p with %u blobs", (void *)this, sample.blob_count);

	correspondence_search_set_blobs(data.cs, sample.blobs, sample.blob_count);

	std::shared_ptr<CameraMosaic> mosaic = this->mosaic.lock();
	U_ASSERT_WEAK_PTR_RET(mosaic,
	                      "Camera mosaic was destroyed while processing a sample, this should never happen since "
	                      "the mosaic owns the camera");

	std::shared_lock lock(tracker->device_lock);

	auto Txr_world_cam = sample.Txr_world_cam;

	for (std::unique_ptr<Device> &device : tracker->devices) {
		auto search_model = device->search_model;

		correspondence_search_flags search_flags =
		    (correspondence_search_flags)(CS_FLAG_STOP_FOR_STRONG_MATCH | CS_FLAG_DEEP_SEARCH);

		xrt_pose Tcv_cam_device = XRT_POSE_IDENTITY;
		auto maybe_device_state = sample.GetDeviceState(device->id);
		if (maybe_device_state.has_value()) {
			auto device_state = *maybe_device_state;
			if (!device_state->needs_slow_processing) {
				continue; // we already did a fast process for this device and it succeeded, no need to
				          // do a slow one
			}

			if (device_state->Txr_world_device_prior.has_value() && Txr_world_cam.has_value()) {
				xrt_pose Txr_cam_world;
				math_pose_invert(&Txr_world_cam.value(), &Txr_cam_world);

				xrt_pose Txr_cam_device;
				math_pose_transform(&Txr_cam_world, &device_state->Txr_world_device_prior.value(),
				                    &Txr_cam_device);

				math_pose_convert_opencv(&Txr_cam_device, &Tcv_cam_device);

				search_flags = (correspondence_search_flags)(search_flags | CS_FLAG_HAVE_POSE_PRIOR);
			}
		}

		xrt_vec3 camera_gravity_vector = {0.0, 1.0, 0.0};
		if ((search_flags & CS_FLAG_HAVE_POSE_PRIOR) != 0) {
			std::unique_lock<os::Mutex> lock(this->processing_lock);

			// If we have a pose for the camera and we have a prior pose
			// (required by correspondence for search gravity matching)
			if (Txr_world_cam.has_value()) {
				// Acquire the camera's gravity vector under the processing lock
				get_pose_gravity_vector(Txr_world_cam.value(), camera_gravity_vector);

				// Add in to check gravity
				search_flags = (correspondence_search_flags)(search_flags | CS_FLAG_MATCH_GRAVITY);
			}
		}

		pose_metrics score;
		pose_metrics_blob_match_info blob_match_info;

		bool found_pose = correspondence_search_find_one_pose( //
		    data.cs,                                           //
		    search_model,                                      //
		    search_flags,                                      //
		    &Tcv_cam_device,                                   //
		    &device->prior_pos_error,                          //
		    &device->prior_rot_error,                          //
		    &camera_gravity_vector,                            //
		    device->gravity_error_rad,                         //
		    &score);                                           //
		if (found_pose) {
			pose_metrics_match_pose_to_blobs(&Tcv_cam_device, sample.blobs, sample.blob_count,
			                                 &device->params.led_model, device->id, &this->model,
			                                 &blob_match_info);
			tracker->MarkMatchingBlobs(sample, device->params.led_model, device->id, blob_match_info);

			auto tbo = sample.ToBlobObservation();
			t_blobwatch_mark_blob_device(sample.source, &tbo, device->id);

			this->PushPose(sample, device, score, Tcv_cam_device, true);

			// We found a pose for this device in this sample
			if (maybe_device_state.has_value()) {
				maybe_device_state.value()->needs_slow_processing = false;
			}
		}
	}
}

bool
Camera::FastSampleProcess(CameraSample &sample)
{
	ConstellationTracker *tracker = this->tracker;

	CT_TRACE(tracker, "Starting fast processing for camera %p with %u blobs", (void *)this, sample.blob_count);

	std::shared_ptr<CameraMosaic> mosaic = this->mosaic.lock();
	U_ASSERT_WEAK_PTR_RET(mosaic,
	                      "Camera mosaic was destroyed while processing a sample, this should never happen since "
	                      "the mosaic owns the camera",
	                      false);

	auto Txr_world_cam = sample.Txr_world_cam;
	if (!Txr_world_cam.has_value()) {
		CT_TRACE(tracker, "Camera %p has no world pose, cannot do fast processing", (void *)this);
		return false;
	}

	xrt_pose Tcv_world_cam;
	math_pose_convert_opencv(&Txr_world_cam.value(), &Tcv_world_cam);

	xrt_pose Tcv_cam_world;
	math_pose_invert(&Tcv_world_cam, &Tcv_cam_world);

	xrt_vec3 gravity_vector;
	get_pose_gravity_vector(Txr_world_cam.value(), gravity_vector);

	bool need_full_search = false;
	std::shared_lock lock(tracker->device_lock);
	for (std::unique_ptr<Device> &device : tracker->devices) {
		xrt_space_relation device_predicted_relation = XRT_SPACE_RELATION_ZERO; //< AKA "the prior"
		t_constellation_tracker_tracking_source_get_tracked_pose(
		    device->params.tracking_source, sample.timestamp_ns, &device_predicted_relation);

		// Whether the prior pose is actually valid
		bool prior_valid =
		    (device_predicted_relation.relation_flags &
		     (XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT)) ==
		    (XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT);

		auto &device_state = sample.device_states[sample.device_count++] = {
		    .device_id = device->id,
		    .Txr_world_device_prior =
		        prior_valid ? std::optional<xrt_pose>(device_predicted_relation.pose) : std::nullopt,
		    .needs_slow_processing = false,
		};

		xrt_pose Tcv_world_device_predicted; //< AKA "the prior"
		math_pose_convert_opencv(&device_predicted_relation.pose, &Tcv_world_device_predicted);

		// if we have a valid prior pose, try to use it for fast matching
		if (prior_valid && this->TryDevicePose(device, sample, Tcv_cam_world, Tcv_world_device_predicted,
		                                       Tcv_world_device_predicted)) {
			CT_DEBUG(tracker, "Fast processing for device %d succeeded", device->id);

			continue; // try the next device, we found a pose!
		}

		// Try to get a last known pose
		bool has_last_known = false;
		xrt_pose Tcv_world_device_last_known;
		{
			std::unique_lock<os::Mutex> lock(device->data_lock);

			if (device->locked_data.Txr_world_device_last_known.has_value()) {
				math_pose_convert_opencv(&device->locked_data.Txr_world_device_last_known.value(),
				                         &Tcv_world_device_last_known);
				has_last_known = true;
			}
		}

		if (has_last_known && this->TryDevicePose(device, sample, Tcv_cam_world, Tcv_world_device_predicted,
		                                          Tcv_world_device_last_known)) {
			CT_DEBUG(tracker, "Fast processing for device %d succeeded with last known pose", device->id);
			continue; // try the next device, we found a pose!
		}

		if (this->TryDeviceBlobRecovery(device, sample, Tcv_cam_world, Tcv_world_device_predicted)) {
			CT_DEBUG(tracker, "Fast processing for device %d succeeded with blob recovery", device->id);
			continue; // try the next device, we found a pose!
		}

		device_state.needs_slow_processing = true;

		need_full_search = true;
	}

	return need_full_search;
}

void
Camera::PushPose(CameraSample &camera_sample,
                 std::unique_ptr<Device> &device,
                 pose_metrics &score,
                 xrt_pose &Tcv_cam_device,
                 bool optimize)
{
	ConstellationTracker *tracker = this->tracker;

	// Try to optimize the pose
	if (optimize) {
		int num_leds_out;
		int num_inliers;
		if (!ransac_pnp_pose(tracker->log_level, &Tcv_cam_device, camera_sample.blobs, camera_sample.blob_count,
		                     &device->params.led_model, device->id, &this->model, &num_leds_out,
		                     &num_inliers)) {
			CT_DEBUG(tracker,
			         "Camera %d (group %d) RANSAC-PnP refinement for device %d from %u "
			         "blobs failed",
			         0, 0, device->id, camera_sample.blob_count);
		} else {
			CT_DEBUG(tracker,
			         "Camera %d (group %d) RANSAC-PnP refinement for device %d from %u "
			         "blobs had "
			         "%d LEDs with %d inliers. Produced pose %f,%f,%f,%f pos %f,%f,%f",
			         0, 0, device->id, camera_sample.blob_count, num_leds_out, num_inliers,
			         Tcv_cam_device.orientation.x, Tcv_cam_device.orientation.y,
			         Tcv_cam_device.orientation.z, Tcv_cam_device.orientation.w, Tcv_cam_device.position.x,
			         Tcv_cam_device.position.y, Tcv_cam_device.position.z);
		}
	}

	// Move to OpenXR space
	xrt_pose Txr_cam_device;
	math_pose_convert_opencv(&Tcv_cam_device, &Txr_cam_device);

	CT_DEBUG(tracker, "Pose: orient %f %f %f %f pos %f %f %f", Txr_cam_device.orientation.x,
	         Txr_cam_device.orientation.y, Txr_cam_device.orientation.z, Txr_cam_device.orientation.w,
	         Txr_cam_device.position.x, Txr_cam_device.position.y, Txr_cam_device.position.z);

	std::shared_ptr<CameraMosaic> mosaic = this->mosaic.lock();
	U_ASSERT_WEAK_PTR_RET(mosaic,
	                      "Camera mosaic was destroyed while processing a sample, this should never happen since "
	                      "the mosaic owns the camera");

	auto Txr_world_cam = camera_sample.Txr_world_cam;
	if (!Txr_world_cam.has_value()) {
		// Can't do anything if we can't locate the camera in the world.
		return;
	}

	xrt_pose Txr_world_device;
	math_pose_transform(&Txr_world_cam.value(), &Txr_cam_device, &Txr_world_device);

	// Push the sample to the device
	t_constellation_tracker_sample sample = {
	    .timestamp_ns = camera_sample.timestamp_ns,
	    .pose = Txr_world_device,
	};
	t_constellation_tracker_device_push_sample(device->device, &sample);

	{
		std::unique_lock<os::Mutex> lock(device->data_lock);

		device->locked_data.Txr_world_device_last_known = Txr_world_device;
	}

	CT_DEBUG(tracker, "Found pose for device %d", device->id);
}

/*
 *
 * CameraMosaic implementations
 *
 */

CameraMosaic::CameraMosaic(ConstellationTracker *tracker, const t_constellation_tracker_camera_mosaic &mosaic_params)
{
	this->tracking_origin = mosaic_params.tracking_origin;

	// NOTE: the stability of this vector is important since we're passing to C callbacks and APIs!
	this->cameras.reserve(mosaic_params.num_cameras);
}

std::optional<xrt_pose>
CameraMosaic::GetTrackingOriginPose(timepoint_ns when_ns)
{
	if (this->tracking_origin) {
		xrt_space_relation relation;
		t_constellation_tracker_tracking_source_get_tracked_pose(this->tracking_origin, when_ns, &relation);

		// If the tracking source has a valid position, grab it
		if ((relation.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0 &&
		    (relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0) {
			return relation.pose;
		} else {
			return std::nullopt;
		}
	}

	return std::optional<xrt_pose>(XRT_POSE_IDENTITY);
}

/*
 *
 * Device implementations
 *
 */

Device::Device(t_constellation_tracker_device_params *params,
               t_constellation_tracker_device *device,
               t_constellation_device_id_t id)
    : params(*params), device(device), id(id), data_lock(), locked_data({.Txr_world_device_last_known = std::nullopt})
{
	// Copy the LED model leds into safe memory, since we want to mutate it into OpenCV space
	this->params.led_model.leds = new t_constellation_tracker_led[this->params.led_model.led_count];
	memcpy(this->params.led_model.leds, params->led_model.leds,
	       sizeof(t_constellation_tracker_led) * this->params.led_model.led_count);

	// flip all LEDs from OpenXR -> OpenCV coordinate space, since the tracker works in OpenCV space
	for (size_t i = 0; i < this->params.led_model.led_count; i++) {
		t_constellation_tracker_led &dst = this->params.led_model.leds[i];
		t_constellation_tracker_led &src = params->led_model.leds[i];

		dst = src;
		dst.position.y = -dst.position.y;
		dst.position.z = -dst.position.z;
		dst.normal.y = -dst.normal.y;
		dst.normal.z = -dst.normal.z;
	}

	this->search_model = t_constellation_search_model_new(this->id, &this->params.led_model);
}

Device::~Device()
{
	if (this->search_model) {
		t_constellation_search_model_free(this->search_model);
		this->search_model = nullptr;
	}

	if (this->params.led_model.leds) {
		delete[] this->params.led_model.leds;
		this->params.led_model.leds = nullptr;
	}
}

/*
 *
 * ConstellationTracker implementations
 *
 */

ConstellationTracker::ConstellationTracker(t_constellation_tracker_params *params)
{
	this->log_level = debug_get_log_option_constellation_tracker_log();

	this->mosaics.reserve(params->num_mosaics);

	// Fill in our internal data structures based on the provided params
	for (size_t i = 0; i < params->num_mosaics; i++) {
		const t_constellation_tracker_camera_mosaic &mosaic_params = params->mosaics[i];

		std::shared_ptr<CameraMosaic> mosaic = std::make_shared<CameraMosaic>(this, mosaic_params);

		// Assert pointer stability!
		assert(mosaic->cameras.capacity() >= mosaic_params.num_cameras);

		for (size_t i = 0; i < mosaic_params.num_cameras; i++) {
			const t_constellation_tracker_camera &camera_params = mosaic_params.cameras[i];

			// This can't be in the constructor since the `shared_ptr` of the `mosaic` isn't formed
			// yet, but the camera needs to own a weak_ptr to it's mosaic.
			mosaic->cameras.push_back(
			    std::make_unique<Camera>(this, mosaic, camera_params, &this->log_level));
		}

		this->mosaics.push_back(mosaic);
	}

	// Fill in the blob sinks for each camera
	for (size_t i = 0; i < this->mosaics.size(); i++) {
		std::shared_ptr<CameraMosaic> &mosaic = this->mosaics[i];

		for (size_t j = 0; j < mosaic->cameras.size(); j++) {
			Camera *camera = mosaic->cameras[j].get();

			params->mosaics[i].cameras[j].blob_sink = &camera->base;
		}
	}

	this->params = *params;

	CT_DEBUG(this, "Created constellation tracker with %zu mosaics", this->mosaics.size());
}

ConstellationTracker::~ConstellationTracker()
{
	CT_DEBUG(this, "Destroying constellation tracker");
}

t_constellation_device_id_t
ConstellationTracker::AddDevice(t_constellation_tracker_device_params *params, t_constellation_tracker_device *device)
{
	std::unique_lock lock(this->device_lock);

	if (this->devices.size() >= XRT_CONSTELLATION_MAX_DEVICES) {
		throw std::runtime_error("Maximum number of devices already added to constellation tracker");
	}

	t_constellation_device_id_t id = this->next_device_id++;

	this->devices.push_back(std::make_unique<Device>(params, device, id));

	CT_DEBUG(this, "Added device with ID %d to constellation tracker", id);

	return id;
}

void
ConstellationTracker::RemoveDevice(t_constellation_device_id_t device_id)
{
	std::unique_lock lock(this->device_lock);

	size_t index = 0;
	for (auto &device : this->devices) {
		if (device->id == device_id) {
			break;
		}
		index++;
	}

	if (index == this->devices.size()) {
		throw std::invalid_argument("The device ID is not present in the device list.");
	}

	// Remove the device
	this->devices.erase(this->devices.begin() + index);
}

void
ConstellationTracker::MarkMatchingBlobs(CameraSample &sample,
                                        t_constellation_tracker_led_model &led_model,
                                        t_constellation_device_id_t device_id,
                                        pose_metrics_blob_match_info &blob_match_info)
{
	// First clear existing blob labels for this device
	for (uint32_t i = 0; i < sample.blob_count; i++) {
		t_blob &b = sample.blobs[i];
		t_constellation_device_id_t led_object_id = b.matched_device_id;

		// Skip blobs which already have an ID not belonging to this device
		if (led_object_id != device_id) {
			continue;
		}

		if (b.matched_device_led_id != XRT_CONSTELLATION_INVALID_LED_ID) {
			// @todo is this needed?
			// b.prev_led_id = b.led_id;
		}

		b.matched_device_led_id = XRT_CONSTELLATION_INVALID_LED_ID;
		b.matched_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
	}

	// Iterate the visible LEDs and mark matching blobs with this device ID and LED ID
	for (int i = 0; i < blob_match_info.num_visible_leds; i++) {
		pose_metrics_visible_led_info &led_info = blob_match_info.visible_leds[i];
		t_constellation_tracker_led *led = led_info.led;

		if (led_info.matched_blob != NULL) {
			t_blob *b = led_info.matched_blob;

			b->matched_device_led_id = led->id;
			b->matched_device_id = device_id;

			CT_DEBUG(this, "Marking LED %d/%d at %f,%f angle %f now %d (was %d)", device_id, led->id,
			         b->center.x, b->center.y, RAD_TO_DEG(acosf(led_info.facing_dot)),
			         b->matched_device_led_id, /* b->prev_led_id */ -1);
		} else {
			CT_DEBUG(this, "No blob for device %d LED %d @ %f,%f size %f px angle %f", device_id, led->id,
			         led_info.pos_px.x, led_info.pos_px.y, 2 * led_info.led_radius_px,
			         RAD_TO_DEG(acosf(led_info.facing_dot)));
		}
	}
}

}; // namespace xrt::tracking::constellation

using namespace xrt::tracking::constellation;

void *
constellation_tracker_camera_slow_thread(void *ptr)
{
	Camera *camera = (Camera *)ptr;

	os_thread_helper_lock(&camera->slow_processing_thread);
	while (os_thread_helper_is_running_locked(&camera->slow_processing_thread)) {
		os_thread_helper_wait_locked(&camera->slow_processing_thread);

		std::optional<CameraSample> maybe_sample = camera->slow_processing_thread_data.sample;
		camera->slow_processing_thread_data.sample.reset();

		os_thread_helper_unlock(&camera->slow_processing_thread);

		if (auto sample = maybe_sample) {
			camera->SlowSampleProcess(*sample);
		}

		os_thread_helper_lock(&camera->slow_processing_thread);
	}
	os_thread_helper_unlock(&camera->slow_processing_thread);

	return NULL;
}

void *
constellation_tracker_camera_fast_thread(void *ptr)
{
	Camera *camera = (Camera *)ptr;

	os_thread_helper_lock(&camera->fast_processing_thread);
	while (os_thread_helper_is_running_locked(&camera->fast_processing_thread)) {
		os_thread_helper_wait_locked(&camera->fast_processing_thread);

		std::optional<CameraSample> maybe_sample = camera->fast_processing_thread_data.sample;
		camera->fast_processing_thread_data.sample.reset();

		os_thread_helper_unlock(&camera->fast_processing_thread);

		if (auto sample = maybe_sample) {
			if (camera->FastSampleProcess(*sample)) {
				CT_TRACE(camera->tracker,
				         "Fast processing for camera %p failed, deferring to slow thread",
				         (void *)camera);
				camera->DeferSampleToSlowThread(*sample);
			}
		}

		os_thread_helper_lock(&camera->fast_processing_thread);
	}
	os_thread_helper_unlock(&camera->fast_processing_thread);

	return NULL;
}

void
constellation_tracker_camera_push_blobs(t_blob_sink *tbs, t_blob_observation *tbo)
{
	Camera *camera = Camera::Get(tbs);
	ConstellationTracker *tracker = camera->tracker;

	CT_TRACE(tracker, "Received blob observation with %u blobs", tbo->num_blobs);

	if (tbo->num_blobs == 0) {
		CT_TRACE(tracker, "No blobs in observation, skipping processing");
		return;
	}

	// Send to the fast thread
	os_thread_helper_lock(&camera->fast_processing_thread);
	camera->fast_processing_thread_data.sample = CameraSample(*tbo, camera);
	os_thread_helper_signal_locked(&camera->fast_processing_thread);
	os_thread_helper_unlock(&camera->fast_processing_thread);
}

void
constellation_tracker_camera_destroy(t_blob_sink *tbs)
{
	// do nothing, the constellation tracker will clean up the blob sinks when it is destroyed.
}

void
constellation_tracker_node_break_apart(xrt_frame_node *node)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(node);

	tracker->running = false;

	// Stop all the threads
	for (auto &mosaic : tracker->mosaics) {
		for (auto &camera : mosaic->cameras) {
			if (camera->slow_processing_thread.initialized) {
				os_thread_helper_stop_and_wait(&camera->slow_processing_thread);
			}

			if (camera->fast_processing_thread.initialized) {
				os_thread_helper_stop_and_wait(&camera->fast_processing_thread);
			}
		}
	}
}

void
constellation_tracker_node_destroy(xrt_frame_node *node)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(node);

	u_var_remove_root(tracker);

	delete tracker;
}

int
t_constellation_tracker_create(xrt_frame_context *xfctx,
                               t_constellation_tracker_params *params,
                               t_constellation_tracker **out_tracker)
{
	try {
		ConstellationTracker *tracker = new ConstellationTracker(params);

		// Add us to the frame context
		xrt_frame_context_add(xfctx, &tracker->node);

		*out_tracker = (t_constellation_tracker *)tracker;

		// Setup debug variable tracking
		u_var_add_root(tracker, "Constellation Tracker", true);
		u_var_add_log_level(tracker, &tracker->log_level, "Log Level");
	} catch (const std::exception &e) {
		U_LOG_E("Failed to create constellation tracker: %s", e.what());
		return -1;
	}

	return 0;
}

int
t_constellation_tracker_add_device(t_constellation_tracker *raw_tracker,
                                   t_constellation_tracker_device_params *params,
                                   t_constellation_tracker_device *device,
                                   t_constellation_device_id_t *out_device_id)
{
	ConstellationTracker *tracker = (ConstellationTracker *)raw_tracker;

	try {
		t_constellation_device_id_t device_id = tracker->AddDevice(params, device);
		*out_device_id = device_id;
	} catch (const std::exception &e) {
		CT_ERROR(tracker, "Failed to add device to constellation tracker: %s", e.what());
		return -1;
	}

	return 0;
}

int
t_constellation_tracker_remove_device(t_constellation_tracker *raw_tracker, t_constellation_device_id_t device)
{
	ConstellationTracker *tracker = (ConstellationTracker *)raw_tracker;

	try {
		tracker->RemoveDevice(device);
	} catch (const std::exception &e) {
		CT_ERROR(tracker, "Failed to add device to constellation tracker: %s", e.what());
		return -1;
	}

	return 0;
}

xrt_tracking_origin *
t_constellation_tracker_get_tracking_origin(t_constellation_tracker *raw_tracker)
{
	ConstellationTracker *tracker = (ConstellationTracker *)raw_tracker;

	return &tracker->tracking_origin;
}
