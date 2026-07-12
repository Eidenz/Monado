// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Implementation of the data recorder logic for the constellation tracker.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#include "xrt/xrt_byte_order.h"

#include "t_constellation_tracker_dataset.hpp"

#include <string>


namespace xrt::tracking::constellation {

#define CT_DATA_MAGIC_VALUE 0x066A4C50

enum PacketType
{
	PACKET_TYPE_CAMERA_SAMPLE = 1,
	PACKET_TYPE_DEVICE_INFO = 2,
};

namespace {

	void
	throw_if_stream_failed(std::fstream &file, const char *operation)
	{
		if (!file) {
			throw std::runtime_error(operation);
		}
	}

} // namespace

/*
 *
 * DataSerializer implementations
 *
 */

DataSerializer::DataSerializer(std::string file, bool write)
{
	this->file = std::fstream(file, std::ios::binary | (write ? std::ios::out : std::ios::in));
	if (!this->file.is_open()) {
		throw std::runtime_error("Failed to open file for data serialization.");
	}
}

DataSerializer::~DataSerializer()
{
	if (this->file.is_open()) {
		this->file.close();
	}
}

// uint8_t

void
DataSerializer::Write(uint8_t value)
{
	this->file.write(reinterpret_cast<const char *>(&value), sizeof(value));
	throw_if_stream_failed(this->file, "Failed to write dataset file.");
}

void
DataSerializer::Read(uint8_t &value)
{
	this->file.read(reinterpret_cast<char *>(&value), sizeof(value));
	throw_if_stream_failed(this->file, "Failed to read dataset file.");
}

// uint32_t

void
DataSerializer::Write(uint32_t value)
{
	__le32 le_value = __cpu_to_le32(value);
	this->file.write(reinterpret_cast<const char *>(&le_value), sizeof(le_value));
	throw_if_stream_failed(this->file, "Failed to write dataset file.");
}

void
DataSerializer::Read(uint32_t &value)
{
	__le32 le_value;
	this->file.read(reinterpret_cast<char *>(&le_value), sizeof(le_value));
	throw_if_stream_failed(this->file, "Failed to read dataset file.");
	value = __le32_to_cpu(le_value);
}

// float

void
DataSerializer::Write(float value)
{
	__lef32 le_value = __cpu_to_lef32(value);
	this->file.write(reinterpret_cast<const char *>(&le_value), sizeof(le_value));
	throw_if_stream_failed(this->file, "Failed to write dataset file.");
}

void
DataSerializer::Read(float &value)
{
	__lef32 le_value;
	this->file.read(reinterpret_cast<char *>(&le_value), sizeof(le_value));
	throw_if_stream_failed(this->file, "Failed to read dataset file.");
	value = __lef32_to_cpu(le_value);
}

// uint64_t

void
DataSerializer::Write(uint64_t value)
{
	__le64 le_value = __cpu_to_le64(value);
	this->file.write(reinterpret_cast<const char *>(&le_value), sizeof(le_value));
	throw_if_stream_failed(this->file, "Failed to write dataset file.");
}

void
DataSerializer::Read(uint64_t &value)
{
	__le64 le_value;
	this->file.read(reinterpret_cast<char *>(&le_value), sizeof(le_value));
	throw_if_stream_failed(this->file, "Failed to read dataset file.");
	value = __le64_to_cpu(le_value);
}

// double

void
DataSerializer::Write(double value)
{
	__lef64 le_value = __cpu_to_lef64(value);
	this->file.write(reinterpret_cast<const char *>(&le_value), sizeof(le_value));
	throw_if_stream_failed(this->file, "Failed to write dataset file.");
}

void
DataSerializer::Read(double &value)
{
	__lef64 le_value;
	this->file.read(reinterpret_cast<char *>(&le_value), sizeof(le_value));
	throw_if_stream_failed(this->file, "Failed to read dataset file.");
	value = __lef64_to_cpu(le_value);
}

// xrt_pose

void
DataSerializer::Write(const xrt_pose &value)
{
	this->Write(value.position.x);
	this->Write(value.position.y);
	this->Write(value.position.z);
	this->Write(value.orientation.x);
	this->Write(value.orientation.y);
	this->Write(value.orientation.z);
	this->Write(value.orientation.w);
}

void
DataSerializer::Read(xrt_pose &value)
{
	this->Read(value.position.x);
	this->Read(value.position.y);
	this->Read(value.position.z);
	this->Read(value.orientation.x);
	this->Read(value.orientation.y);
	this->Read(value.orientation.z);
	this->Read(value.orientation.w);
}

// FoundDevicePose

void
DataSerializer::Write(const FoundDevicePose &value)
{
	this->Write(value.Tcv_cam_device);
}

void
DataSerializer::Read(FoundDevicePose &value)
{
	this->Read(value.Tcv_cam_device);
}

// DeviceState

void
DataSerializer::Write(const DeviceState &value)
{
	this->Write(static_cast<uint8_t>(value.device_id));

	this->Write(value.Txr_world_device_prior);
	this->Write(value.found_pose);
}

void
DataSerializer::Read(DeviceState &value)
{
	uint8_t device_id;
	this->Read(device_id);
	value.device_id = static_cast<t_constellation_device_id_t>(device_id);

	this->Read(value.Txr_world_device_prior);
	this->Read(value.found_pose);
}

// t_blob

void
DataSerializer::Write(const t_blob &value)
{
	this->Write(value.blob_id);
	this->Write(static_cast<uint8_t>(value.matched_device_id));
	this->Write(static_cast<uint8_t>(value.matched_device_led_id));
	this->Write(value.center.x);
	this->Write(value.center.y);
	this->Write(value.motion_vector.x);
	this->Write(value.motion_vector.y);
	this->Write(static_cast<uint32_t>(value.bounding_box.offset.w));
	this->Write(static_cast<uint32_t>(value.bounding_box.offset.h));
	this->Write(static_cast<uint32_t>(value.bounding_box.extent.w));
	this->Write(static_cast<uint32_t>(value.bounding_box.extent.h));
	this->Write(value.size.x);
	this->Write(value.size.y);
	this->Write(value.brightness);
}

void
DataSerializer::Read(t_blob &value)
{
	this->Read(value.blob_id);

	uint8_t matched_device_id;
	this->Read(matched_device_id);
	value.matched_device_id = static_cast<t_constellation_device_id_t>(matched_device_id);

	uint8_t matched_device_led_id;
	this->Read(matched_device_led_id);
	value.matched_device_led_id = static_cast<t_constellation_device_id_t>(matched_device_led_id);

	this->Read(value.center.x);
	this->Read(value.center.y);
	this->Read(value.motion_vector.x);
	this->Read(value.motion_vector.y);

	uint32_t bounding_box_offset_w;
	this->Read(bounding_box_offset_w);
	value.bounding_box.offset.w = static_cast<int>(bounding_box_offset_w);

	uint32_t bounding_box_offset_h;
	this->Read(bounding_box_offset_h);
	value.bounding_box.offset.h = static_cast<int>(bounding_box_offset_h);

	uint32_t bounding_box_extent_w;
	this->Read(bounding_box_extent_w);
	value.bounding_box.extent.w = static_cast<int>(bounding_box_extent_w);

	uint32_t bounding_box_extent_h;
	this->Read(bounding_box_extent_h);
	value.bounding_box.extent.h = static_cast<int>(bounding_box_extent_h);

	this->Read(value.size.x);
	this->Read(value.size.y);
	this->Read(value.brightness);
}

// CameraSample

void
DataSerializer::Write(const CameraSample &value)
{
	this->Write(static_cast<uint64_t>(value.id));
	this->Write(static_cast<uint64_t>(value.timestamp_ns));

	this->Write(static_cast<uint32_t>(value.blob_count));
	for (size_t i = 0; i < value.blob_count; i++) {
		this->Write(value.blobs[i]);
	}

	this->Write(value.Txr_world_cam);

	this->Write(value.device_count);
	for (size_t i = 0; i < value.device_count; i++) {
		this->Write(value.device_states[i]);
	}

	this->Write(value.mosaic_index);
	this->Write(value.camera_index);
}

void
DataSerializer::Read(CameraSample &value)
{
	this->Read(value.id);

	uint64_t timestamp_ns;
	this->Read(timestamp_ns);
	value.timestamp_ns = timestamp_ns;

	this->Read(value.blob_count);
	if (value.blob_count > ARRAY_SIZE(value.blobs)) {
		throw std::runtime_error("Dataset sample blob_count exceeds CameraSample storage.");
	}

	for (size_t i = 0; i < value.blob_count; i++) {
		this->Read(value.blobs[i]);
	}

	this->Read(value.Txr_world_cam);

	uint32_t device_count;
	this->Read(device_count);
	value.device_count = device_count;

	if (device_count > value.device_states.max_size()) {
		throw std::runtime_error("Dataset sample device_count exceeds CameraSample storage.");
	}

	for (size_t i = 0; i < value.device_count; i++) {
		this->Read(value.device_states[i]);
	}

	this->Read(value.mosaic_index);
	this->Read(value.camera_index);
}

// t_camera_calibration

void
DataSerializer::Write(const t_camera_calibration &value)
{
	this->Write(static_cast<uint32_t>(value.image_size_pixels.w));
	this->Write(static_cast<uint32_t>(value.image_size_pixels.h));

	// @todo remove when clang-format is updated in CI
	// clang-format off
	double *intrinsics_as_array = reinterpret_cast<double *>(const_cast<double (*)[3][3]>(&value.intrinsics));
	// clang-format on

	for (size_t i = 0; i < 9; i++) {
		this->Write(intrinsics_as_array[i]);
	}

	for (size_t i = 0; i < ARRAY_SIZE(value.distortion_parameters_as_array); i++) {
		this->Write(value.distortion_parameters_as_array[i]);
	}

	this->Write(static_cast<uint8_t>(value.distortion_model));
}

void
DataSerializer::Read(t_camera_calibration &value)
{
	uint32_t image_size_w;
	this->Read(image_size_w);
	value.image_size_pixels.w = static_cast<int>(image_size_w);

	uint32_t image_size_h;
	this->Read(image_size_h);
	value.image_size_pixels.h = static_cast<int>(image_size_h);

	// @todo remove when clang-format is updated in CI
	// clang-format off
	double *intrinsics_as_array = reinterpret_cast<double *>(const_cast<double (*)[3][3]>(&value.intrinsics));
	// clang-format on

	for (size_t i = 0; i < 9; i++) {
		this->Read(intrinsics_as_array[i]);
	}

	for (size_t i = 0; i < ARRAY_SIZE(value.distortion_parameters_as_array); i++) {
		this->Read(value.distortion_parameters_as_array[i]);
	}

	uint8_t distortion_model;
	this->Read(distortion_model);
	value.distortion_model = static_cast<t_camera_distortion_model>(distortion_model);
}

// t_constellation_tracker_led

void
DataSerializer::Write(t_constellation_tracker_led &value)
{
	this->Write(value.position.x);
	this->Write(value.position.y);
	this->Write(value.position.z);

	this->Write(value.normal.x);
	this->Write(value.normal.y);
	this->Write(value.normal.z);

	this->Write(value.radius_m);
	this->Write(value.visibility_angle);

	this->Write(static_cast<uint8_t>(value.id));
}

void
DataSerializer::Read(t_constellation_tracker_led &value)
{
	this->Read(value.position.x);
	this->Read(value.position.y);
	this->Read(value.position.z);

	this->Read(value.normal.x);
	this->Read(value.normal.y);
	this->Read(value.normal.z);

	this->Read(value.radius_m);
	this->Read(value.visibility_angle);

	uint8_t id;
	this->Read(id);
	value.id = static_cast<t_constellation_led_id_it>(id);
}

// t_constellation_tracker_led_model

void
DataSerializer::Write(const t_constellation_tracker_led_model &value)
{
	this->Write(static_cast<uint32_t>(value.led_count));
	for (size_t i = 0; i < value.led_count; i++) {
		this->Write(value.leds[i]);
	}
}

void
DataSerializer::Read(std::vector<t_constellation_tracker_led> &led_storage, t_constellation_tracker_led_model &value)
{
	uint32_t led_count;
	this->Read(led_count);
	value.led_count = led_count;

	led_storage.resize(led_count);
	value.leds = led_storage.data();

	for (size_t i = 0; i < value.led_count; i++) {
		this->Read(value.leds[i]);
	}
}

// Flush

void
DataSerializer::Flush()
{
	this->file.flush();
}

/*
 *
 * DataRecorder implementations
 *
 */

DataRecorder::DataRecorder(ConstellationTracker *tracker, std::string out_file) : serializer(out_file, true)
{
	// Magic value to identify the file so we can prevent loading invalid files. Written in BE to preserve the
	// match with the source code.
	this->serializer.Write(static_cast<uint32_t>(__be32_to_cpu(CT_DATA_MAGIC_VALUE)));

	this->serializer.Write(static_cast<uint32_t>(tracker->mosaics.size()));
	for (auto &mosaic : tracker->mosaics) {
		this->serializer.Write(static_cast<uint32_t>(mosaic->cameras.size()));
		for (auto &camera : mosaic->cameras) {
			this->serializer.Write(camera->calibration);
		}
	}

	this->serializer.Flush();
}

void
DataRecorder::RecordSample(const CameraSample &sample)
{
	this->serializer.Write(static_cast<uint8_t>(PACKET_TYPE_CAMERA_SAMPLE));
	this->serializer.Write(sample);
	this->serializer.Flush();
}

void
DataRecorder::RecordDeviceInfo(const Device &device)
{
	this->serializer.Write(static_cast<uint8_t>(PACKET_TYPE_DEVICE_INFO));
	this->serializer.Write(static_cast<uint8_t>(device.id));
	this->serializer.Write(device.params.led_model);
	this->serializer.Flush();
}

/*
 *
 * DatasetReader implementations
 *
 */

DatasetReader::DatasetReader(std::string filename) : serializer(filename, false)
{
	uint32_t magic_value;
	this->serializer.Read(magic_value);
	if (magic_value != static_cast<uint32_t>(__be32_to_cpu(CT_DATA_MAGIC_VALUE))) {
		throw std::runtime_error(
		    "Invalid magic value in dataset file, not a valid constellation tracker dataset.");
	}

	uint32_t mosaic_count;
	this->serializer.Read(mosaic_count);

	for (size_t i = 0; i < mosaic_count; i++) {
		DatasetMosaic mosaic;

		uint32_t camera_count;
		this->serializer.Read(camera_count);

		for (size_t j = 0; j < camera_count; j++) {
			t_camera_calibration calibration;
			this->serializer.Read(calibration);
			mosaic.camera_calibrations.push_back(calibration);
		}

		this->mosaics.push_back(mosaic);
	}

	while (true) {
		try {
			uint8_t packet_type;
			this->serializer.Read(packet_type);
			switch (packet_type) {
			case PACKET_TYPE_CAMERA_SAMPLE: {
				CameraSample sample;
				this->serializer.Read(sample);
				this->samples.push_back(sample);
				break;
			}
			case PACKET_TYPE_DEVICE_INFO: {
				DatasetDevice &device = this->devices.emplace_back();

				uint8_t device_id;
				this->serializer.Read(device_id);
				device.id = static_cast<t_constellation_device_id_t>(device_id);

				this->serializer.Read(device.leds, device.led_model);

				break;
			}
			default: {
				throw std::runtime_error("Invalid packet type in dataset file. Got ID " +
				                         std::to_string(packet_type));
			}
			}
		} catch (const std::exception &e) {
			break;
		}
	}
}

}; // namespace xrt::tracking::constellation
