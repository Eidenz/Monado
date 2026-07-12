// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Implementation of the data recorder logic for the constellation tracker.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#pragma once

#include "t_constellation_tracker_internal.hpp"


namespace xrt::tracking::constellation {

struct DataSerializer
{
private: // Fields
	std::fstream file;

public: // Methods
	DataSerializer(std::string file, bool write);

	~DataSerializer();

	void
	Write(uint8_t value);
	void
	Read(uint8_t &value);

	void
	Write(uint32_t value);
	void
	Read(uint32_t &value);

	void
	Write(float value);
	void
	Read(float &value);

	void
	Write(uint64_t value);
	void
	Read(uint64_t &value);

	void
	Write(double value);
	void
	Read(double &value);

	void
	Write(const xrt_pose &value);
	void
	Read(xrt_pose &value);

	void
	Write(const FoundDevicePose &value);
	void
	Read(FoundDevicePose &value);

	void
	Write(const DeviceState &value);
	void
	Read(DeviceState &value);

	void
	Write(const t_blob &value);
	void
	Read(t_blob &value);

	void
	Write(const CameraSample &value);
	void
	Read(CameraSample &value);

	void
	Write(const t_camera_calibration &value);
	void
	Read(t_camera_calibration &value);

	void
	Write(t_constellation_tracker_led &value);
	void
	Read(t_constellation_tracker_led &value);

	void
	Write(const t_constellation_tracker_led_model &value);
	void
	Read(std::vector<t_constellation_tracker_led> &led_storage, t_constellation_tracker_led_model &value);

	template <typename T>
	void
	Write(const std::optional<T> &opt_value)
	{
		this->Write(static_cast<uint8_t>(opt_value.has_value() ? 1 : 0));
		if (opt_value.has_value()) {
			this->Write(opt_value.value());
		}
	}

	template <typename T>
	void
	Read(std::optional<T> &opt_value)
	{
		uint8_t has_value;
		this->Read(has_value);
		if (has_value) {
			T value;
			this->Read(value);
			opt_value = value;
		} else {
			opt_value = std::nullopt;
		}
	}

	void
	Flush();
};

struct DataRecorder
{
private: // Fields
	DataSerializer serializer;

public: // Methods
	DataRecorder(ConstellationTracker *tracker, std::string out_file);

	~DataRecorder() = default;

	void
	RecordSample(const CameraSample &sample);

	void
	RecordDeviceInfo(const Device &device);
};

struct DatasetMosaic
{
	std::vector<t_camera_calibration> camera_calibrations;
};

struct DatasetDevice
{
	t_constellation_device_id_t id;

	std::vector<t_constellation_tracker_led> leds;
	t_constellation_tracker_led_model led_model;
};

struct DatasetReader
{
private: // Fields
	DataSerializer serializer;

public: // Fields
	std::vector<DatasetMosaic> mosaics;
	std::vector<DatasetDevice> devices;

	std::vector<CameraSample> samples;

public: // Methods
	DatasetReader(std::string filename);
};

}; // namespace xrt::tracking::constellation
