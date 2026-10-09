// Copyright 2023, Shawn Wallace
// Copyright 2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief SteamVR driver context implementation and entrypoint.
 * @author Shawn Wallace <yungwallace@live.com>
 * @ingroup drv_steamvr_lh
 */

#include "xrt/xrt_config_arch.h"
#include "xrt/xrt_system.h"

#include "interfaces/context.hpp"

#include "util/u_var.h"
#include "util/u_device.h"
#include "util/u_screenshot.h"

#include "vive/vive_bindings.h"

#include "math/m_api.h"

#include "openvr_driver.h"

#include "vdf_parser.hpp"

#include "steamvr_lh_interface.h"
#include "device.hpp"

#include <cstring>
#include <dlfcn.h>
#include <memory>
#include <cmath>
#include <unordered_map>
#include <string_view>
#include <filesystem>
#include <istream>
#include <algorithm>

namespace {

// based on the logic at
// <https://github.com/ValveSoftware/openvr/blob/ae46a8dd0172580648c8922658a100439115d3eb/src/vrcore/pathtools_public.h#L143-L153>.
#if defined(XRT_OS_WINDOWS)
#error "steamvr_lh does not yet support windows!"
#define OVR_PLAT_EXT ".dll"

#if defined(XRT_ARCH_X86_64)
#define OVR_PLAT_SUBDIR "win64"
#elif defined(XRT_ARCH_X86)
#define OVR_PLAT_SUBDIR "win32"
#else
#error "steamvr_lh: Unknown Windows Architecture"
#endif

#elif defined(XRT_OS_OSX)
#define OVR_PLAT_EXT ".dylib"
// contains all architectures in a universal binary
#define OVR_PLAT_SUBDIR "osx32"

#elif defined(XRT_OS_LINUX)
#define OVR_PLAT_EXT ".so"

#if defined(XRT_ARCH_X86)
#define OVR_PLAT_SUBDIR "linux32"
#elif defined(XRT_OS_ANDROID) && defined(XRT_ARCH_ARM)
#define OVR_PLAT_SUBDIR "androidarm32"
#elif defined(XRT_OS_ANDROID) && defined(XRT_ARCH_ARM64)
#define OVR_PLAT_SUBDIR "androidarm64"
#elif defined(XRT_ARCH_ARM64)
#define OVR_PLAT_SUBDIR "linuxarm64"
#elif defined(XRT_ARCH_X86_64)
#define OVR_PLAT_SUBDIR "linux64"
#else
#error "steamvr_lh: Unknown Linux Architecture"
#endif

#else
#error "steamvr_lh: Unknown OS"
#endif

DEBUG_GET_ONCE_LOG_OPTION(lh_log, "LIGHTHOUSE_LOG", U_LOGGING_INFO)
DEBUG_GET_ONCE_BOOL_OPTION(lh_load_slimevr, "LH_LOAD_SLIMEVR", false)
DEBUG_GET_ONCE_NUM_OPTION(lh_discover_wait_ms, "LH_DISCOVER_WAIT_MS", 3000)
DEBUG_GET_ONCE_FLOAT_OPTION(lh_stick_deadzone, "LH_STICK_DEADZONE", 0)

static constexpr size_t MAX_CONTROLLERS = 16;


/*
 * The lighthouse driver is a process singleton (one driver_lighthouse.so, with
 * the working directory moved into its config folder), so one context serves
 * the C entry points at the end of this file. The devices own it; this only
 * observes.
 */
std::weak_ptr<Context> g_context;

// ~/.steam/root is a symlink to where the Steam root is
const std::string STEAM_INSTALL_DIR = std::string(getenv("HOME")) + "/.steam/root";
constexpr auto STEAMVR_APPID = "250820";

// Parse libraryfolder.vdf to find where SteamVR is installed
std::string
find_steamvr_install()
{
	using namespace tyti;
	u_logging_level level = debug_get_log_option_lh_log();
	std::ifstream file(STEAM_INSTALL_DIR + "/steamapps/libraryfolders.vdf");
	if (!file.is_open()) {
		U_LOG_IFL_E(level, "Failed to open libraryfolders.vdf");
		return std::string();
	}

	vdf::basic_object<char> root;
	try {
		root = vdf::read(file);
	} catch (std::exception &ex) {
		U_LOG_IFL_E(level, "Failed to read libraryfolders.vdf: %s", ex.what());
		return std::string();
	}

	assert(root.name == "libraryfolders");
	for (auto &[_, child] : root.childs) {
		U_LOG_D("Found library folder %s", child->attribs["path"].c_str());
		std::shared_ptr<vdf::object> apps = child->childs["apps"];
		for (auto &[appid, _] : apps->attribs) {
			if (appid == STEAMVR_APPID) {
				std::string path = child->attribs["path"] + "/steamapps/common/SteamVR";
				if (std::filesystem::exists(path)) {
					return path;
				}
			}
		}
	}
	return std::string();
}

} // namespace

#define CTX_ERR(...) U_LOG_IFL_E(log_level, __VA_ARGS__)
#define CTX_WARN(...) U_LOG_IFL_W(log_level, __VA_ARGS__)
#define CTX_INFO(...) U_LOG_IFL_I(log_level, __VA_ARGS__)
#define CTX_TRACE(...) U_LOG_IFL_T(log_level, __VA_ARGS__)
#define CTX_DEBUG(...) U_LOG_IFL_D(log_level, __VA_ARGS__)

/**
 * Since only the devices will live after our get_devices function is called, we make our Context
 * a shared ptr that is owned by the devices that exist, so that it is also cleaned up by the
 * devices that exist when they are all destroyed.
 */
std::shared_ptr<Context>
Context::create(const std::string &steam_install,
                const std::string &steamvr_install,
                std::vector<vr::IServerTrackedDeviceProvider *> providers)
{
	// xrt_tracking_origin initialization
	std::shared_ptr<Context> c =
	    std::make_shared<Context>(steam_install, steamvr_install, debug_get_log_option_lh_log());
	c->providers = std::move(providers);
	std::strncpy(c->name, "SteamVR Lighthouse Tracking", XRT_TRACKING_NAME_LEN);
	c->type = XRT_TRACKING_TYPE_LIGHTHOUSE;
	c->initial_offset = XRT_POSE_IDENTITY;
	for (vr::IServerTrackedDeviceProvider *const &driver : c->providers) {
		vr::EVRInitError err = driver->Init(c.get());
		if (err != vr::VRInitError_None) {
			U_LOG_IFL_E(c->log_level, "OpenVR driver initialization failed: error %u", err);
			return nullptr;
		}
	}
	c->frame_thread = std::thread([ctx = c.get()] {
		while (ctx->frame_thread_run.load()) {
			using namespace std::chrono_literals;
			// SteamVR calls `RunFrame()` approximately every 10.1ms
			const std::chrono::time_point<std::chrono::steady_clock> next =
			    std::chrono::steady_clock::now() + 10ms;
			// Create/activate any queued devices on this frame thread, so
			// Activate() callbacks don't race with the Update* callbacks
			// that RunFrame() triggers below (see process_pending_additions).
			ctx->process_pending_additions();
			for (vr::IServerTrackedDeviceProvider *const &provider : ctx->providers)
				provider->RunFrame();
			// Pick up a room setup written while we run (Monadeck, SteamVR).
			Device::poll_chaperone();
			ctx->frame_thread_event.try_acquire_until(next);
		}
	});
	return c;
}

Context::Context(const std::string &steam_install, const std::string &steamvr_install, u_logging_level level)
    : settings(steam_install, steamvr_install, this), resources(level, steamvr_install), log_level(level),
      frame_thread_run(true), frame_thread()
{}

Context::~Context()
{
	this->frame_thread_run.store(false);
	this->frame_thread_event.release();
	if (this->frame_thread.joinable())
		this->frame_thread.join();
	for (vr::IServerTrackedDeviceProvider *const &provider : providers)
		provider->Cleanup();
}

/***** IVRDriverContext methods *****/

void *
Context::GetGenericInterface(const char *pchInterfaceVersion, vr::EVRInitError *peError)
{
	CTX_DEBUG("Requested interface %s", pchInterfaceVersion);
#define MATCH_INTERFACE(version, interface)                                                                            \
	if (std::strcmp(pchInterfaceVersion, version) == 0) {                                                          \
		return interface;                                                                                      \
	}
#define MATCH_INTERFACE_THIS(interface) MATCH_INTERFACE(interface##_Version, static_cast<interface *>(this))

	// Known interfaces
	MATCH_INTERFACE_THIS(vr::IVRServerDriverHost);
	MATCH_INTERFACE_THIS(vr::IVRDriverInput);
	// This interface is not in a public header yet, but just passing IVRDriverInput_003 seems to work.
	MATCH_INTERFACE("IVRDriverInput_004", static_cast<vr::IVRDriverInput *>(this));
	MATCH_INTERFACE("IVRDriverInput_005", static_cast<vr::IVRDriverInput *>(this));
	MATCH_INTERFACE_THIS(vr::IVRProperties);
	MATCH_INTERFACE_THIS(vr::IVRDriverLog);
	MATCH_INTERFACE(vr::IVRSettings_Version, &settings);
	MATCH_INTERFACE(vr::IVRResources_Version, &resources);
	MATCH_INTERFACE(vr::IVRIOBuffer_Version, &iobuf);
	MATCH_INTERFACE(vr::IVRDriverManager_Version, &man);
	MATCH_INTERFACE(vr::IVRBlockQueue_Version, &blockqueue);
	MATCH_INTERFACE(vr::IVRPaths_Version, &paths);
	// This version of the interface is not in a public header.
	// Luckily it seems to be compatible with the previous version.
	MATCH_INTERFACE("IVRPaths_002", &paths);

	// Internal interfaces
	MATCH_INTERFACE("IVRServer_XXX", &server);
	MATCH_INTERFACE("IVRServerInternal_XXX", &server);
	return nullptr;
}

vr::DriverHandle_t
Context::GetDriverHandle()
{
	return 1;
}


/***** IVRServerDriverHost methods *****/

bool
Context::setup_hmd(const char *serial, vr::ITrackedDeviceServerDriver *driver)
{
	auto *hmd = new HmdDevice(DeviceBuilder{this->shared_from_this(), driver, serial, STEAM_INSTALL_DIR});

	{
		// Publish the device before activating it: the driver writes its properties and creates its
		// input components from within Activate, and those callbacks have to be able to find it.
		std::lock_guard lk(this->devices_mut);
		this->hmd = hmd;
	}

#define VERIFY(expr, msg)                                                                                              \
	if (!(expr)) {                                                                                                 \
		CTX_ERR("Activating HMD failed: %s", msg);                                                             \
		std::lock_guard lk(this->devices_mut);                                                                 \
		if (this->hmd == hmd) {                                                                                \
			this->hmd = nullptr;                                                                           \
		}                                                                                                      \
		delete hmd;                                                                                            \
		return false;                                                                                          \
	}
	// Never call into the driver with `devices_mut` held: it takes its own locks here, and calls back into
	// us (WritePropertyBatch, TrackedDevicePoseUpdated, ...) from its own threads while holding them.
	vr::EVRInitError err = driver->Activate(0);
	VERIFY(err == vr::VRInitError_None, std::to_string(err).c_str());

	auto *display = static_cast<vr::IVRDisplayComponent *>(driver->GetComponent(vr::IVRDisplayComponent_Version));
	VERIFY(display, "IVRDisplayComponent is null");
#undef VERIFY

	auto hmd_parts = std::make_unique<HmdDevice::Parts>();
	hmd_parts->base.view_count = 2;
	for (size_t idx = 0; idx < 2; ++idx) {
		vr::EVREye eye = (idx == 0) ? vr::Eye_Left : vr::Eye_Right;
		xrt_view &view = hmd_parts->base.views[idx];

		display->GetEyeOutputViewport(eye, &view.viewport.x_pixels, &view.viewport.y_pixels,
		                              &view.viewport.w_pixels, &view.viewport.h_pixels);

		view.display.w_pixels = view.viewport.w_pixels;
		view.display.h_pixels = view.viewport.h_pixels;
		view.rot = u_device_rotation_ident;
	}

	hmd_parts->base.screens[0].w_pixels =
	    hmd_parts->base.views[0].display.w_pixels + hmd_parts->base.views[1].display.w_pixels;
	hmd_parts->base.screens[0].h_pixels = hmd_parts->base.views[0].display.h_pixels;
	// nominal frame interval will be set when lighthouse gives us the display frequency
	// see HmdDevice::handle_property_write

	hmd_parts->base.blend_modes[0] = XRT_BLEND_MODE_OPAQUE;
	hmd_parts->base.blend_mode_count = 1;

	auto &distortion = hmd_parts->base.distortion;
	distortion.models = XRT_DISTORTION_MODEL_COMPUTE;
	distortion.preferred = XRT_DISTORTION_MODEL_COMPUTE;
	for (size_t idx = 0; idx < 2; ++idx) {
		xrt_fov &fov = distortion.fov[idx];
		float tan_left, tan_right, tan_top, tan_bottom;
		display->GetProjectionRaw((vr::EVREye)idx, &tan_left, &tan_right, &tan_top, &tan_bottom);
		fov.angle_left = atanf(tan_left);
		fov.angle_right = atanf(tan_right);
		fov.angle_up = atanf(tan_bottom);
		fov.angle_down = atanf(tan_top);
	}

	u_var_add_root(hmd, "SteamVR HMD Device", true);
	u_var_add_f32(hmd, &hmd->ipd, "IPD");

	u_var_add_f32(hmd, &distortion.fov[0].angle_up, "View 0 FovAngleUp");
	u_var_add_f32(hmd, &distortion.fov[0].angle_down, "View 0 FovAngleDown");
	u_var_add_f32(hmd, &distortion.fov[0].angle_left, "View 0 FovAngleLeft");
	u_var_add_f32(hmd, &distortion.fov[0].angle_right, "View 0 FovAngleRight");

	u_var_add_f32(hmd, &distortion.fov[1].angle_up, "View 1 FovAngleUp");
	u_var_add_f32(hmd, &distortion.fov[1].angle_down, "View 1 FovAngleDown");
	u_var_add_f32(hmd, &distortion.fov[1].angle_left, "View 1 FovAngleLeft");
	u_var_add_f32(hmd, &distortion.fov[1].angle_right, "View 1 FovAngleRight");

	hmd_parts->display = display;
	hmd->set_hmd_parts(std::move(hmd_parts));

	return true;
}

bool
Context::setup_controller(const char *serial, vr::ITrackedDeviceServerDriver *driver)
{
	// Defer creation to the frame thread: TrackedDeviceAdded may be called
	// from the lighthouse driver's background threads, and activating there
	// would have the Activate() callbacks race the Update* callbacks that
	// RunFrame() triggers (see process_pending_additions).
	{
		std::lock_guard lk(pending_addition_mut);
		pending_additions.push_back({serial, driver});
	}

	// Handed to the frame thread: discovery stays open until it has activated the device.
	{
		std::lock_guard lk(this->devices_mut);
		this->devices_in_setup++;
	}

	return true;
}

void
Context::process_pending_additions()
{
	std::vector<PendingAddition> to_add;
	{
		std::lock_guard lk(pending_addition_mut);
		to_add.swap(pending_additions);
	}

	for (const PendingAddition &pa : to_add) {
		activate_pending(pa);

		{
			std::lock_guard lk(this->devices_mut);
			this->devices_in_setup--;
		}
		this->discover_cv.notify_all();
	}
}

void
Context::activate_pending(const PendingAddition &pa)
{
	size_t device_idx = 0;
	ControllerDevice *device = nullptr;
	bool readded = false;

	{
		std::lock_guard lk(this->devices_mut);

		// Re-add of a known device (e.g. dongle reconnect): reuse its slot.
		for (; device_idx < MAX_CONTROLLERS; ++device_idx) {
			if (controller[device_idx] &&
			    std::strcmp(controller[device_idx]->serial, pa.serial.c_str()) == 0) {
				break;
			}
		}

		if (device_idx != MAX_CONTROLLERS) {
			CTX_INFO("Device %s re-added, reactivating at slot %zu", pa.serial.c_str(), device_idx);
			device = controller[device_idx];
			device->set_driver(pa.driver);
			readded = true;
		} else {
			// Find the first available slot for a new controller
			for (device_idx = 0; device_idx < MAX_CONTROLLERS; ++device_idx) {
				if (!controller[device_idx])
					break;
			}

			// Check if we've exceeded the maximum number of controllers
			if (device_idx == MAX_CONTROLLERS) {
				CTX_WARN("Attempted to activate more than %zu controllers - this is unsupported",
				         MAX_CONTROLLERS);
				return;
			}

			// Create the new controller and claim its slot before activating it: the driver writes
			// its properties and creates its input components from within Activate, and those
			// callbacks have to be able to find it.
			device =
			    new ControllerDevice(device_idx + 1, DeviceBuilder{this->shared_from_this(), pa.driver,
			                                                       pa.serial.c_str(), STEAM_INSTALL_DIR});
			controller[device_idx] = device;
		}
	}

	// Never call into the driver with `devices_mut` held: it takes its own locks here, and calls back into
	// us (WritePropertyBatch, TrackedDevicePoseUpdated, ...) from its own threads while holding them.
	vr::EVRInitError err = pa.driver->Activate(device_idx + 1);

	if (readded) {
		if (err != vr::VRInitError_None) {
			CTX_ERR("Reactivating controller %s failed: error %u", pa.serial.c_str(), err);
			return;
		}
		device->connected = true;
		return;
	}

	if (err != vr::VRInitError_None) {
		// Published all the same, like a device that fails during boot: it is
		// torn down with the system, and a later re-add revives it in place.
		// Until then it reports disconnected and never gets a role.
		CTX_ERR("Activating controller %s failed: error %u", pa.serial.c_str(), err);
		device->connected = false;
	}

	enum xrt_device_name name = device->name;
	switch (name) {
	case XRT_DEVICE_VIVE_WAND:
		device->binding_profiles = vive_binding_profiles_wand;
		device->binding_profile_count = vive_binding_profiles_wand_count;
		break;
	case XRT_DEVICE_INDEX_CONTROLLER:
		device->binding_profiles = vive_binding_profiles_index;
		device->binding_profile_count = vive_binding_profiles_index_count;
		break;
	case XRT_DEVICE_FLIPVR:
		device->binding_profiles = vive_binding_profiles_flipvr;
		device->binding_profile_count = vive_binding_profiles_flipvr_count;
		break;
	default: break;
	}

	// Until steamvr_lh_create_devices has assembled the system it picks the
	// device up from controller[]; after that this is a hotplug and we publish
	// it ourselves.
	std::lock_guard lk(this->devices_mut);
	if (xsysd != nullptr) {
		CTX_INFO("Hotplugged device %s at slot %zu", pa.serial.c_str(), device_idx);
		append_to_xsysd_locked(device);
	}
}

void
Context::wait_for_discover()
{
	std::unique_lock lk(this->devices_mut);

	this->discover_end_time =
	    std::chrono::steady_clock::now() + std::chrono::milliseconds(debug_get_num_option_lh_discover_wait_ms());

	while (true) {
		if (std::chrono::steady_clock::now() < this->discover_end_time) {
			this->discover_cv.wait_until(lk, this->discover_end_time);
			continue;
		}

		// The window is not over while a device we already know about is still being activated.
		if (this->devices_in_setup == 0)
			break;

		this->discover_cv.wait(lk, [this] { return this->devices_in_setup == 0; });
	}
}

void
Context::append_to_xsysd_locked(struct xrt_device *xdev)
{
	uint32_t count = xsysd->static_xdev_count;
	for (uint32_t i = 0; i < count; i++) {
		if (xsysd->static_xdevs[i] == xdev) {
			return; // Already published by steamvr_lh_create_devices.
		}
	}
	if (count >= XRT_SYSTEM_MAX_DEVICES) {
		CTX_WARN("Out of device slots, not exposing %s", xdev->str);
		return;
	}

	// Let the owner add the device to the space overseer first, so poses
	// can be located as soon as the device is visible.
	if (device_added_cb != nullptr) {
		device_added_cb(xdev, device_added_ud);
	}

	// Fill the slot, then publish it: IPC per-client threads read the
	// count before the slots, so they never see an empty published slot.
	xsysd->static_xdevs[count] = xdev;
	__atomic_store_n(&xsysd->static_xdev_count, count + 1, __ATOMIC_RELEASE);
}

void
Context::extend_discover()
{
	// Only ever push the deadline out - a device showing up must not cut `LH_DISCOVER_WAIT_MS` short.
	this->discover_end_time =
	    std::max(this->discover_end_time, std::chrono::steady_clock::now() + std::chrono::milliseconds(2000));
	this->discover_cv.notify_all();
}

// NOLINTBEGIN(bugprone-easily-swappable-parameters)
bool
Context::TrackedDeviceAdded(const char *pchDeviceSerialNumber,
                            vr::ETrackedDeviceClass eDeviceClass,
                            vr::ITrackedDeviceServerDriver *pDriver)
{
	{
		std::lock_guard lk(this->devices_mut);

		if (this->in_setup) {
			// Boot: keep the discovery window open a little longer for every
			// device that shows up, so late controllers still make the cut.
			this->extend_discover();
		}
		// Unlike upstream, devices appearing after setup are welcome: the frame
		// thread creates them and publishes them to xrt_system_devices itself
		// (see process_pending_additions).

		// The setup below runs without `devices_mut` held, and publishes the device before the driver has
		// finished activating it - keep discovery from ending in the middle of that.
		this->devices_in_setup++;
	}

	bool added = false;

	CTX_INFO("New device added: %s", pchDeviceSerialNumber);
	switch (eDeviceClass) {
	case vr::TrackedDeviceClass_HMD: {
		CTX_INFO("Found lighthouse HMD: %s", pchDeviceSerialNumber);
		added = setup_hmd(pchDeviceSerialNumber, pDriver);
		break;
	}
	case vr::TrackedDeviceClass_Controller: {
		CTX_INFO("Found lighthouse controller: %s", pchDeviceSerialNumber);
		added = setup_controller(pchDeviceSerialNumber, pDriver);
		break;
	}
	case vr::TrackedDeviceClass_TrackingReference: {
		CTX_INFO("Found lighthouse base station: %s", pchDeviceSerialNumber);
		break;
	}
	case vr::TrackedDeviceClass_GenericTracker: {
		CTX_INFO("Found lighthouse tracker: %s", pchDeviceSerialNumber);
		added = setup_controller(pchDeviceSerialNumber, pDriver);
		break;
	}
	default: {
		CTX_WARN("Attempted to add unsupported device class: %u", eDeviceClass);
		break;
	}
	}

	{
		std::lock_guard lk(this->devices_mut);
		this->devices_in_setup--;
	}
	this->discover_cv.notify_all();

	return added;
}

void
Context::TrackedDevicePoseUpdated(uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize)
{
	assert(sizeof(newPose) == unPoseStructSize);

	// Check for valid device index, allowing for the HMD plus up to MAX_CONTROLLERS controllers
	if (unWhichDevice > MAX_CONTROLLERS)
		return;

	Device *dev = nullptr;

	std::lock_guard lk(this->devices_mut);

	// If unWhichDevice is 0, it refers to the HMD; otherwise, it refers to one of the controllers
	if (unWhichDevice == 0) {
		dev = static_cast<Device *>(this->hmd);
	} else {
		// unWhichDevice - 1 will give the index into the controller array
		dev = static_cast<Device *>(this->controller[unWhichDevice - 1]);
	}

	// The slot is empty: Monado destroyed the device, or the frame thread has
	// not created a hotplugged one yet (the driver's threads can send poses
	// for it before that).
	if (dev == nullptr) {
		return;
	}

	dev->update_pose(newPose);
}

void
Context::VsyncEvent(double vsyncTimeOffsetSeconds)
{}

void
Context::add_event_locked(vr::VREvent_t event)
{
	this->events.emplace_back(std::chrono::steady_clock::now(), event);
	this->frame_thread_event.try_acquire();
	this->frame_thread_event.release();
}

void
Context::VendorSpecificEvent(uint32_t unWhichDevice,
                             vr::EVREventType eventType,
                             const vr::VREvent_Data_t &eventData,
                             double eventTimeOffset)
{
	std::lock_guard lk(event_queue_mut);
	this->add_event_locked({
	    .eventType = static_cast<uint32_t>(eventType),
	    .trackedDeviceIndex = unWhichDevice,
	    .eventAgeSeconds = {},
	    .data = eventData,
	});
}

bool
Context::IsExiting()
{
	return false;
}

size_t
Context::add_haptic_event(vr::VREvent_HapticVibration_t event, const size_t old_event_handle)
{
	vr::VREvent_t e;
	e.eventType = vr::EVREventType::VREvent_Input_HapticVibration;
	e.trackedDeviceIndex = event.containerHandle - 1;
	vr::VREvent_Data_t d;
	d.hapticVibration = event;
	e.data = d;

	std::unique_lock lk(event_queue_mut);
	while (events.size() >= 120) { // avoid unbounded allocation if misbehaving apps send too many events at once
		event_popped.wait(lk);
	}
	const size_t old_event_index = old_event_handle - events_tail;
	if (old_event_index < size_t(events.size())) {
		vr::VREvent_t *const old_event = &events[old_event_index].inner;
		if (old_event->eventType == e.eventType &&
		    old_event->data.hapticVibration.containerHandle == e.data.hapticVibration.containerHandle &&
		    old_event->data.hapticVibration.componentHandle == e.data.hapticVibration.componentHandle) {
			old_event->eventType = vr::EVREventType::VREvent_None;
		}
	}
	this->add_event_locked(e);
	return events_tail + events.size() - 1;
}

bool
Context::PollNextEvent(vr::VREvent_t *pEvent, uint32_t uncbVREvent)
{
	std::lock_guard lk(event_queue_mut);
	while (!events.empty()) {
		assert(sizeof(vr::VREvent_t) == uncbVREvent);
		Event e = events.front();
		events.pop_front();
		++events_tail;
		if (e.inner.eventType == vr::EVREventType::VREvent_None) {
			continue;
		}
		event_popped.notify_all();
		*pEvent = e.inner;
		using float_sec = std::chrono::duration<float>;
		float_sec event_age = std::chrono::steady_clock::now() - e.insert_time;
		pEvent->eventAgeSeconds = event_age.count();
		return true;
	}
	return false;
}

void
Context::GetRawTrackedDevicePoses(float fPredictedSecondsFromNow,
                                  vr::TrackedDevicePose_t *pTrackedDevicePoseArray,
                                  uint32_t unTrackedDevicePoseArrayCount)
{
	// This is the bare minimum required for SlimeVR's HMD feedback to work
	if (unTrackedDevicePoseArrayCount != 10 || this->hmd == nullptr)
		return;
	const uint64_t time =
	    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
	        .count();
	xrt_space_relation head = {};
	xrt_device_get_tracked_pose(this->hmd, XRT_INPUT_GENERIC_HEAD_POSE, time, &head);
	xrt_matrix_3x3 rot = {};
	math_matrix_3x3_from_quat(&head.pose.orientation, &rot);
	pTrackedDevicePoseArray[0].mDeviceToAbsoluteTracking = {{
	    {rot.v[0], rot.v[3], rot.v[6], head.pose.position.x},
	    {rot.v[1], rot.v[4], rot.v[7], head.pose.position.y},
	    {rot.v[2], rot.v[5], rot.v[8], head.pose.position.z},
	}};
}

void
Context::RequestRestart(const char *pchLocalizedReason,
                        const char *pchExecutableToStart,
                        const char *pchArguments,
                        const char *pchWorkingDirectory)
{}

uint32_t
Context::GetFrameTimings(vr::Compositor_FrameTiming *pTiming, uint32_t nFrames)
{
	return 0;
}

void
Context::SetDisplayEyeToHead(uint32_t unWhichDevice,
                             const vr::HmdMatrix34_t &eyeToHeadLeft,
                             const vr::HmdMatrix34_t &eyeToHeadRight)
{
	hmd->SetDisplayEyeToHead(unWhichDevice, eyeToHeadLeft, eyeToHeadRight);
}

void
Context::SetDisplayProjectionRaw(uint32_t unWhichDevice, const vr::HmdRect2_t &eyeLeft, const vr::HmdRect2_t &eyeRight)
{}

void
Context::SetRecommendedRenderTargetSize(uint32_t unWhichDevice, uint32_t nWidth, uint32_t nHeight)
{}

/***** IVRDriverInput methods *****/


vr::EVRInputError
Context::create_component_common(vr::PropertyContainerHandle_t container,
                                 const char *name,
                                 vr::VRInputComponentHandle_t *pHandle)
{
	*pHandle = vr::k_ulInvalidInputComponentHandle;

	std::lock_guard lk(this->devices_mut);
	Device *device = prop_container_to_device(container);
	if (!device) {
		return vr::VRInputError_InvalidHandle;
	}

	if (xrt_input *input = device->get_input_from_name(name); input) {
		CTX_DEBUG("creating component %s for %p", name, (void *)device);
		vr::VRInputComponentHandle_t handle = device->new_input_handle_locked();
		this->input.handle_to_input[handle] = input;
		this->input.handle_to_device[handle] = device;
		*pHandle = handle;
	}

	return vr::VRInputError_None;
}

xrt_input *
Context::update_component_common_locked(vr::VRInputComponentHandle_t handle,
                                        double offset,
                                        std::chrono::steady_clock::time_point now)
{
	xrt_input *input{nullptr};

	if (handle != vr::k_ulInvalidInputComponentHandle) {
		// A destroyed device's handles are gone from the table, haptic handles map to no input.
		auto it = this->input.handle_to_input.find(handle);
		if (it == this->input.handle_to_input.end() || it->second == nullptr) {
			return nullptr;
		}
		input = it->second;
		std::chrono::duration<double, std::chrono::seconds::period> offset_dur(offset);
		std::chrono::duration offset = (now + offset_dur).time_since_epoch();
		int64_t timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(offset).count();
		input->active = true;
		input->timestamp = timestamp;
	}

	return input;
}

vr::EVRInputError
Context::CreateBooleanComponent(vr::PropertyContainerHandle_t ulContainer,
                                const char *pchName,
                                vr::VRInputComponentHandle_t *pHandle)
{
	return create_component_common(ulContainer, pchName, pHandle);
}

// True if this device's analog trigger is currently pulled (or fully clicked).
static bool
device_trigger_held(const xrt_device *dev)
{
	bool held = false;
	for (uint32_t i = 0; i < dev->input_count; i++) {
		const xrt_input *in = &dev->inputs[i];
		if (in->name == XRT_INPUT_INDEX_TRIGGER_VALUE && in->value.vec1.x >= 0.5f) {
			held = true;
		}
		if (in->name == XRT_INPUT_INDEX_TRIGGER_CLICK && in->value.boolean) {
			held = true;
		}
	}
	return held;
}

vr::EVRInputError
Context::UpdateBooleanComponent(vr::VRInputComponentHandle_t ulComponent, bool bNewValue, double fTimeOffset)
{
	std::lock_guard lk(this->devices_mut);
	xrt_input *input = update_component_common_locked(ulComponent, fTimeOffset);
	if (input) {
		// Screenshot on a deliberate chord: hold the trigger and click the
		// system button, on either hand controller. The trigger requirement
		// keeps it from firing on an accidental system tap - and on the right,
		// where the system button also opens WayVR, the chord makes the
		// screenshot intentional rather than an accident.
		bool is_system = input->name == XRT_INPUT_INDEX_SYSTEM_CLICK || //
		                 input->name == XRT_INPUT_VIVE_SYSTEM_CLICK;
		if (is_system && bNewValue && !input->value.boolean) {
			Device *dev = nullptr;
			if (auto it = this->input.handle_to_device.find(ulComponent);
			    it != this->input.handle_to_device.end()) {
				dev = it->second;
			}
			bool is_hand = dev != nullptr && (dev->device_type == XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER ||
			                                  dev->device_type == XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER);
			if (is_hand && device_trigger_held(dev)) {
				u_screenshot_request();
			}
		}
		input->value.boolean = bNewValue;
	}
	return vr::VRInputError_None;
}

vr::EVRInputError
Context::CreateScalarComponent(vr::PropertyContainerHandle_t ulContainer,
                               const char *pchName,
                               vr::VRInputComponentHandle_t *pHandle,
                               vr::EVRScalarType eType,
                               vr::EVRScalarUnits eUnits)
{
	std::string_view name{pchName};
	// Lighthouse gives thumbsticks/trackpads as x/y components,
	// we need to combine them for Monado
	auto end = name.back();
	auto second_last = name.at(name.size() - 2);
	if (second_last == '/' && (end == 'x' || end == 'y')) {
		std::lock_guard lk(this->devices_mut);

		Device *device = this->prop_container_to_device(ulContainer);
		if (!device) {
			return vr::VRInputError_InvalidHandle;
		}

		bool x = end == 'x';
		name.remove_suffix(2);
		std::string n(name);
		xrt_input *input = device->get_input_from_name(n);
		if (!input) {
			return vr::VRInputError_None;
		}

		// Create the component mapping if it hasn't been created yet
		Vec2Components *components =
		    this->input.vec2_input_to_components.try_emplace(input, new Vec2Components).first->second.get();

		vr::VRInputComponentHandle_t new_handle = device->new_input_handle_locked();
		if (x)
			components->x = new_handle;
		else
			components->y = new_handle;

		this->input.handle_to_input[new_handle] = input;
		*pHandle = new_handle;
		return vr::VRInputError_None;
	}
	return create_component_common(ulContainer, pchName, pHandle);
}

static struct xrt_vec2
applyDeadzone(struct xrt_vec2 input)
{
	static const float deadzone = [] {
		float raw = debug_get_float_option_lh_stick_deadzone();
		// we apply deadzone to the input's absolute value; valid range is 0..1
		float clamped = CLAMP(raw, 0.0f, 0.99f);
		if (raw != clamped) {
			U_LOG_W("LH_STICK_DEADZONE value of %.2f falls outside of expected range 0..1 - clamp to %.2f",
			        raw, clamped);
		}
		return clamped;
	}();

	if (input.x * input.x + input.y * input.y <= deadzone * deadzone) {
		input.x = 0.0f;
		input.y = 0.0f;
	}

	return input;
}

vr::EVRInputError
Context::UpdateScalarComponent(vr::VRInputComponentHandle_t ulComponent, float fNewValue, double fTimeOffset)
{
	std::lock_guard lk(this->devices_mut);

	auto h = this->input.handle_to_input.find(ulComponent);
	if (h != this->input.handle_to_input.end() && h->second) {
		xrt_input *input = update_component_common_locked(ulComponent, fTimeOffset);
		if (XRT_GET_INPUT_TYPE(input->name) == XRT_INPUT_TYPE_VEC2_MINUS_ONE_TO_ONE) {
			std::unique_ptr<Vec2Components> &components = this->input.vec2_input_to_components.at(input);
			if (components->x == ulComponent) {
				input->value.vec2.x = fNewValue;
			} else if (components->y == ulComponent) {
				input->value.vec2.y = fNewValue;
			} else {
				CTX_WARN("Attempted to update component with handle %" PRIu64
				         " but it was neither the x nor y "
				         "component of its associated input",
				         ulComponent);
			}
			input->value.vec2 = applyDeadzone(input->value.vec2);

		} else {
			input->value.vec1.x = fNewValue;
		}
	}

	return vr::VRInputError_None;
}

vr::EVRInputError
Context::CreateHapticComponent(vr::PropertyContainerHandle_t ulContainer,
                               const char *pchName,
                               vr::VRInputComponentHandle_t *pHandle)
{
	*pHandle = vr::k_ulInvalidInputComponentHandle;

	std::lock_guard lk(this->devices_mut);
	Device *d = prop_container_to_device(ulContainer);
	if (!d) {
		return vr::VRInputError_InvalidHandle;
	}

	// Assuming HMDs won't have haptics.
	// Maybe a wrong assumption.
	if (d == hmd) {
		CTX_WARN("Didn't expect HMD with haptics.");
		return vr::VRInputError_InvalidHandle;
	}

	auto *device = static_cast<ControllerDevice *>(d);
	vr::VRInputComponentHandle_t handle = device->new_input_handle_locked();
	this->input.handle_to_input[handle] = nullptr;
	device->set_haptic_handle(handle);
	*pHandle = handle;

	return vr::VRInputError_None;
}

vr::EVRInputError
Context::CreateSkeletonComponent(vr::PropertyContainerHandle_t ulContainer,
                                 const char *pchName,
                                 const char *pchSkeletonPath,
                                 const char *pchBasePosePath,
                                 vr::EVRSkeletalTrackingLevel eSkeletalTrackingLevel,
                                 const vr::VRBoneTransform_t *pGripLimitTransforms,
                                 uint32_t unGripLimitTransformCount,
                                 vr::VRInputComponentHandle_t *pHandle)
{
	std::string_view path(pchSkeletonPath); // should be /skeleton/hand/left or /skeleton/hand/right
	std::string_view skeleton_pfx("/skeleton/hand/");
	if (!path.starts_with(skeleton_pfx)) {
		CTX_ERR("Got invalid skeleton path: %s", std::string(path).c_str());
		return vr::VRInputError_InvalidSkeleton;
	}

	if (auto ret = create_component_common(ulContainer, pchSkeletonPath, pHandle); ret != vr::VRInputError_None) {
		return ret;
	}

	std::lock_guard lk(this->devices_mut);

	auto *device = static_cast<ControllerDevice *>(prop_container_to_device(ulContainer));
	path.remove_prefix(skeleton_pfx.size());
	xrt_hand hand;
	if (path == "left") {
		hand = XRT_HAND_LEFT;
	} else if (path == "right") {
		hand = XRT_HAND_RIGHT;
	} else {
		CTX_ERR("Got invalid skeleton path suffix: %s", std::string(path).c_str());
		return vr::VRInputError_InvalidSkeleton;
	}

	device->set_skeleton(std::span(pGripLimitTransforms, unGripLimitTransformCount), hand,
	                     eSkeletalTrackingLevel == vr::VRSkeletalTracking_Estimated, pchSkeletonPath);
	this->input.skeleton_to_controller[*pHandle] = device;

	return vr::VRInputError_None;
}

vr::EVRInputError
Context::UpdateSkeletonComponent(vr::VRInputComponentHandle_t ulComponent,
                                 vr::EVRSkeletalMotionRange eMotionRange,
                                 const vr::VRBoneTransform_t *pTransforms,
                                 uint32_t unTransformCount)
{
	if (eMotionRange != vr::VRSkeletalMotionRange_WithoutController) {
		return vr::VRInputError_None;
	}

	std::lock_guard lk(this->devices_mut);
	if (!update_component_common_locked(ulComponent, 0)) {
		return vr::VRInputError_InvalidHandle;
	}

	auto *device = this->input.skeleton_to_controller[ulComponent];
	if (!device) {
		CTX_ERR("Got unknown component handle %" PRIu64, ulComponent);
		return vr::VRInputError_InvalidHandle;
	}

	device->update_skeleton_transforms(std::span(pTransforms, unTransformCount));

	return vr::VRInputError_None;
}

vr::EVRInputError
Context::CreatePoseComponent(vr::PropertyContainerHandle_t ulContainer,
                             const char *pchName,
                             vr::VRInputComponentHandle_t *pHandle)
{
	return vr::VRInputError_None;
}

vr::EVRInputError
Context::UpdatePoseComponent(vr::VRInputComponentHandle_t ulComponent,
                             const vr::HmdMatrix34_t *pMatPoseOffset,
                             double fTimeOffset)
{
	return vr::VRInputError_None;
}

vr::EVRInputError
Context::CreateEyeTrackingComponent(vr::PropertyContainerHandle_t ulContainer,
                                    const char *pchName,
                                    vr::VRInputComponentHandle_t *pHandle)
{
	return vr::VRInputError_None;
}

vr::EVRInputError
Context::UpdateEyeTrackingComponent(vr::VRInputComponentHandle_t ulComponent,
                                    const vr::VREyeTrackingData_t *pEyeTrackingData,
                                    double fTimeOffset)
{
	return vr::VRInputError_None;
}

/***** IVRProperties methods *****/

vr::ETrackedPropertyError
Context::ReadPropertyBatch(vr::PropertyContainerHandle_t ulContainerHandle,
                           vr::PropertyRead_t *pBatch,
                           uint32_t unBatchEntryCount)
{
	std::lock_guard lk(this->devices_mut);

	Device *device = prop_container_to_device(ulContainerHandle);
	if (!device)
		return vr::TrackedProp_InvalidContainer;
	if (!pBatch)
		return vr::TrackedProp_InvalidOperation; // not verified vs steamvr
	return device->handle_read_properties(pBatch, unBatchEntryCount);
}

vr::ETrackedPropertyError
Context::WritePropertyBatch(vr::PropertyContainerHandle_t ulContainerHandle,
                            vr::PropertyWrite_t *pBatch,
                            uint32_t unBatchEntryCount)
{
	std::lock_guard lk(this->devices_mut);

	Device *device = prop_container_to_device(ulContainerHandle);
	if (!device)
		return vr::TrackedProp_InvalidContainer;
	if (!pBatch)
		return vr::TrackedProp_InvalidOperation; // not verified vs steamvr
	return device->handle_properties(pBatch, unBatchEntryCount);
}

const char *
Context::GetPropErrorNameFromEnum(vr::ETrackedPropertyError error)
{
	return nullptr;
}

Device *
Context::prop_container_to_device(vr::PropertyContainerHandle_t handle)
{
	switch (handle) {
	case 1: {
		return hmd;
		break;
	}
	default: {
		// If the handle corresponds to a controller
		if (handle >= 2 && handle <= MAX_CONTROLLERS + 1) {
			return controller[handle - 2];
		} else {
			return nullptr;
		}
		break;
	}
	}
}

vr::PropertyContainerHandle_t
Context::TrackedDeviceToPropertyContainer(vr::TrackedDeviceIndex_t nDevice)
{
	size_t container = nDevice + 1;
	if (nDevice == 0 && this->hmd) {
		return container;
	}
	if (nDevice >= 1 && nDevice <= MAX_CONTROLLERS && this->controller[nDevice - 1]) {
		return container;
	}

	return vr::k_ulInvalidPropertyContainer;
}

void
Context::Log(const char *pchLogMessage)
{
	CTX_TRACE("[lighthouse]: %s", pchLogMessage);
}
// NOLINTEND(bugprone-easily-swappable-parameters)

Device *
Context::device_from_xdev_locked(struct xrt_device *xdev)
{
	if (xdev == nullptr) {
		return nullptr;
	}
	if (this->hmd != nullptr && xdev == this->hmd) {
		return this->hmd;
	}
	for (ControllerDevice *controller : this->controller) {
		if (controller != nullptr && xdev == controller) {
			return controller;
		}
	}
	return nullptr;
}

xrt_result_t
Context::get_roles(struct xrt_system_devices *xsysd, struct xrt_system_roles *out_roles)
{
	int head, eyes, face, left, right, gamepad;

	// Devices can be appended concurrently from the frame thread: the count
	// is published after the slot is filled, so load it once with acquire.
	uint32_t count = __atomic_load_n(&xsysd->static_xdev_count, __ATOMIC_ACQUIRE);
	if (count > XRT_SYSTEM_MAX_DEVICES) {
		count = XRT_SYSTEM_MAX_DEVICES;
	}

	/*
	 * Only consider devices this driver owns and that are currently
	 * powered on. The builder may have appended foreign devices (e.g.
	 * udcap gloves) which must not be cast to Device; those get their
	 * roles from the builder's get_roles wrapper instead.
	 */
	struct xrt_device *candidates[XRT_SYSTEM_MAX_DEVICES] = {};
	{
		std::lock_guard lk(this->devices_mut);
		for (uint32_t i = 0; i < count; i++) {
			Device *dev = device_from_xdev_locked(xsysd->static_xdevs[i]);
			if (dev != nullptr && dev->connected.load(std::memory_order_relaxed)) {
				candidates[i] = dev;
			}
		}
	}

	u_device_assign_xdev_roles(candidates, count, &head, &eyes, &face, &left, &right, &gamepad);

	std::lock_guard lk(roles_mutex);

	/*
	 * Sticky roles: a powered-off device keeps its role until some
	 * connected device can actually take it over. In-game hands freeze in
	 * place instead of snapping back to the body, matching what users of
	 * this fork are used to. The builder's glove wrapper checks
	 * connectedness itself so gloves still take over from powered-off
	 * controllers.
	 */
	if (left == XRT_DEVICE_ROLE_UNASSIGNED && cached_roles.left != XRT_DEVICE_ROLE_UNASSIGNED &&
	    cached_roles.left < (int32_t)count) {
		left = cached_roles.left;
	}
	if (right == XRT_DEVICE_ROLE_UNASSIGNED && cached_roles.right != XRT_DEVICE_ROLE_UNASSIGNED &&
	    cached_roles.right < (int32_t)count) {
		right = cached_roles.right;
	}

	if (left != cached_roles.left || right != cached_roles.right || gamepad != cached_roles.gamepad) {
		CTX_INFO("Controller roles changed: left=%d right=%d gamepad=%d", left, right, gamepad);

		cached_roles.generation_id++;

		cached_roles.left = left;
		cached_roles.right = right;
		cached_roles.gamepad = gamepad;

		cached_roles.left_profile =
		    left != XRT_DEVICE_ROLE_UNASSIGNED ? xsysd->static_xdevs[left]->name : XRT_DEVICE_INVALID;
		cached_roles.right_profile =
		    right != XRT_DEVICE_ROLE_UNASSIGNED ? xsysd->static_xdevs[right]->name : XRT_DEVICE_INVALID;
		cached_roles.gamepad_profile =
		    gamepad != XRT_DEVICE_ROLE_UNASSIGNED ? xsysd->static_xdevs[gamepad]->name : XRT_DEVICE_INVALID;

		// Tell the devices which hand they are now, as b_system_devices does for the boot roles.
		if (left != XRT_DEVICE_ROLE_UNASSIGNED && xsysd->static_xdevs[left]->supported.notify_chirality) {
			xrt_device_notify_chirality(xsysd->static_xdevs[left], true, XRT_HAND_LEFT);
		}
		if (right != XRT_DEVICE_ROLE_UNASSIGNED && xsysd->static_xdevs[right]->supported.notify_chirality) {
			xrt_device_notify_chirality(xsysd->static_xdevs[right], true, XRT_HAND_RIGHT);
		}
	}

	*out_roles = cached_roles;

	return XRT_SUCCESS;
}

extern "C" xrt_result_t
steamvr_lh_get_roles(struct xrt_system_devices *xsysd, struct xrt_system_roles *out_roles)
{
	std::shared_ptr<Context> ctx = g_context.lock();
	if (ctx == nullptr) {
		// No system has been created (yet, or any more).
		return XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	return ctx->get_roles(xsysd, out_roles);
}

extern "C" void
steamvr_lh_set_device_added_callback(struct xrt_system_devices *xsysd,
                                     steamvr_lh_device_added_callback callback,
                                     void *userdata)
{
	(void)xsysd; // There is only ever one steamvr_lh system.
	std::shared_ptr<Context> ctx = g_context.lock();
	if (ctx == nullptr) {
		return;
	}

	std::lock_guard lk(ctx->devices_mut);
	ctx->device_added_cb = callback;
	ctx->device_added_ud = userdata;
}

extern "C" bool
steamvr_lh_device_is_connected(struct xrt_device *xdev)
{
	std::shared_ptr<Context> ctx = g_context.lock();
	if (ctx == nullptr) {
		return false;
	}

	std::lock_guard lk(ctx->devices_mut);
	Device *dev = ctx->device_from_xdev_locked(xdev);

	// Not one of ours (e.g. a glove device).
	return dev != nullptr && dev->connected.load(std::memory_order_relaxed);
}

extern "C" enum xrt_result
steamvr_lh_create_devices(struct xrt_prober *xp, struct xrt_system_devices *xsysd)
{
	u_logging_level level = debug_get_log_option_lh_log();
	// The driver likes to create a bunch of transient folders -
	// let's try to make sure they're created where they normally are.
	std::filesystem::path dir = STEAM_INSTALL_DIR + "/config/lighthouse";
	if (!std::filesystem::exists(dir)) {
		U_LOG_IFL_W(level,
		            "Couldn't find lighthouse config folder (%s)- transient folders will be created in current "
		            "working directory (%s)",
		            dir.c_str(), std::filesystem::current_path().c_str());
	} else {
		std::filesystem::current_path(dir);
	}

	std::string steamvr{};
	if (getenv("STEAMVR_PATH") != nullptr) {
		steamvr = getenv("STEAMVR_PATH");
	} else {
		steamvr = find_steamvr_install();
	}

	if (steamvr.empty()) {
		U_LOG_IFL_E(level, "Could not find where SteamVR is installed!");
		return xrt_result::XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	U_LOG_IFL_I(level, "Found SteamVR install: %s", steamvr.c_str());

	std::vector<vr::IServerTrackedDeviceProvider *> drivers = {};
	const auto loadDriver = [&](const std::string &soPath, bool require) {
		// TODO: support windows?
		void *driver_lib = dlopen((steamvr + soPath).c_str(), RTLD_LAZY);
		if (!driver_lib) {
			U_LOG_IFL_E(level, "Couldn't open driver lib: %s", dlerror());
			return !require;
		}

		void *sym = dlsym(driver_lib, "HmdDriverFactory");
		if (!sym) {
			U_LOG_IFL_E(level, "Couldn't find HmdDriverFactory in driver lib: %s", dlerror());
			return false;
		}
		using HmdDriverFactory_t = void *(*)(const char *, int *);
		auto factory = reinterpret_cast<HmdDriverFactory_t>(sym);

		vr::EVRInitError err = vr::VRInitError_None;
		drivers.push_back(static_cast<vr::IServerTrackedDeviceProvider *>(
		    factory(vr::IServerTrackedDeviceProvider_Version, (int *)&err)));
		if (err != vr::VRInitError_None) {
			U_LOG_IFL_E(level, "Couldn't get tracked device driver: error %u", err);
			return false;
		}
		return true;
	};
	if (!loadDriver("/drivers/lighthouse/bin/" OVR_PLAT_SUBDIR "/driver_lighthouse" OVR_PLAT_EXT, true))
		return xrt_result::XRT_ERROR_DEVICE_CREATION_FAILED;
	if (debug_get_bool_option_lh_load_slimevr() &&
	    !loadDriver("/drivers/slimevr/bin/" OVR_PLAT_SUBDIR "/driver_slimevr" OVR_PLAT_EXT, false))
		return xrt_result::XRT_ERROR_DEVICE_CREATION_FAILED;
	auto ctx = Context::create(STEAM_INSTALL_DIR, steamvr, std::move(drivers));
	if (ctx == nullptr)
		return xrt_result::XRT_ERROR_DEVICE_CREATION_FAILED;

	U_LOG_IFL_I(level, "Lighthouse initialization complete, giving time to setup connected devices...");
	// RunFrame needs to be called to detect controllers
	ctx->wait_for_discover();
	U_LOG_IFL_I(level, "Device search time complete.");

	if (xsysd == NULL) {
		U_LOG_IFL_E(level, "Invalid system pointer");
		return xrt_result::XRT_ERROR_DEVICE_CREATION_FAILED;
	}

	std::lock_guard lk(ctx->devices_mut);
	ctx->in_setup = false;

	// Include the HMD
	if (ctx->hmd) {
		if (ctx->hmd->variant == VIVE_VARIANT_PRO2 && !ctx->hmd->init_vive_pro_2(xp)) {
			U_LOG_IFL_W(level, "Found Vive Pro 2, but failed to initialize.");
		}

		xsysd->static_xdevs[xsysd->static_xdev_count++] = ctx->hmd;
	}

	// Include the controllers
	for (size_t i = 0; i < MAX_CONTROLLERS; i++) {
		if (ctx->controller[i]) {
			xsysd->static_xdevs[xsysd->static_xdev_count++] = ctx->controller[i];
		}
	}

	// Give the context a reference to xsysd so the frame thread can publish
	// devices that are hotplugged after boot.
	ctx->xsysd = xsysd;

	// Valid generations start at 1 (clients cache starting from 0).
	ctx->cached_roles.generation_id = 1;

	g_context = ctx;

	return xrt_result::XRT_SUCCESS;
}
