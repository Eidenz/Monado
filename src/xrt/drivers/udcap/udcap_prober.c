// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  UDCAP device creation + tracker pose attachment.
 *
 * Creates a controller device per hand (reading the udcap-server shm) and wraps
 * each in a tracking override so its 6DoF pose follows a Lighthouse tracker
 * mounted on the glove. The tracker<->hand pairing comes from the shm (set by
 * udcap-server via --tracker-left/--tracker-right), falling back to env vars,
 * then to the first unused trackers.
 *
 * If no udcap-server is running (shm absent or stale), no devices are created,
 * so the gloves never take over the controller roles without live data.
 *
 * @ingroup drv_udcap
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "xrt/xrt_device.h"
#include "xrt/xrt_defines.h"

#include "util/u_logging.h"

#include "udcap_interface.h"
#include "udcap_device.h"
#include "udcap_shm.h"

#define UDCAP_PROBER_LOG U_LOGGING_INFO
#define UDCAP_INFO(...) U_LOG_IFL_I(UDCAP_PROBER_LOG, __VA_ARGS__)
#define UDCAP_WARN(...) U_LOG_IFL_W(UDCAP_PROBER_LOG, __VA_ARGS__)

// Read static config from the shm: the configured tracker serial per hand and
// the writing server's pid (0 = no live server / stale segment).
static uint32_t
read_shm_config(char serial_left[32], char serial_right[32])
{
	serial_left[0] = '\0';
	serial_right[0] = '\0';

	int fd = shm_open(UDCAP_SHM_NAME, O_RDONLY, 0);
	if (fd < 0) {
		return 0;
	}
	udcap_shm *shm = mmap(NULL, sizeof(udcap_shm), PROT_READ, MAP_SHARED, fd, 0);
	close(fd);
	if (shm == MAP_FAILED) {
		return 0;
	}

	uint32_t pid = 0;
	if (shm->magic == UDCAP_SHM_MAGIC && shm->version == UDCAP_SHM_VERSION) {
		pid = shm->server_pid;
		// shm tracker_serial is zero-initialised and holds short serials, so it
		// is always NUL-terminated; snprintf copies safely without truncation.
		snprintf(serial_left, 32, "%s", shm->hands[UDCAP_HAND_LEFT].tracker_serial);
		snprintf(serial_right, 32, "%s", shm->hands[UDCAP_HAND_RIGHT].tracker_serial);
	} else {
		UDCAP_WARN("udcap: shm magic/version mismatch (got %08x/%u, want %08x/%u) - reinstall the "
		           "driver (this Monado build) so it matches udcap-server",
		           shm->magic, shm->version, UDCAP_SHM_MAGIC, UDCAP_SHM_VERSION);
	}
	munmap(shm, sizeof(udcap_shm));
	return pid;
}

// Whether the device is a Lighthouse tracker. The GENERIC_TRACKER device_type
// is only set by an asynchronous role-hint property from the proprietary
// driver and is usually not in yet at boot, so also accept the input class
// (set synchronously during Activate).
static bool
is_tracker(struct xrt_device *d)
{
	return d->device_type == XRT_DEVICE_TYPE_GENERIC_TRACKER || d->name == XRT_DEVICE_VIVE_TRACKER;
}

// Pick a tracker from devs: by serial substring if `serial` is set, else the
// first unused tracker. Marks the chosen index in `used`.
static struct xrt_device *
pick_tracker(struct xrt_device *const *devs, size_t dev_count, const char *serial, bool *used)
{
	for (size_t i = 0; i < dev_count; i++) {
		struct xrt_device *d = devs[i];
		if (d == NULL || used[i]) {
			continue;
		}
		if (serial != NULL && serial[0] != '\0') {
			// A serial uniquely identifies the device, no need to
			// also check that it is a tracker.
			if (strstr(d->serial, serial) == NULL) {
				continue;
			}
		} else if (!is_tracker(d)) {
			continue;
		}
		used[i] = true;
		return d;
	}
	return NULL;
}

// Late-attach state: hands created without a tracker wait for one to be
// hotplugged (the steamvr builder forwards new devices here).
static struct xrt_device *late_hands[2] = {NULL, NULL};
static char late_serials[2][32] = {{0}, {0}};
static bool late_attached[2] = {false, false};

void
udcap_notify_device_added(struct xrt_device *xdev)
{
	if (xdev == NULL) {
		return;
	}

	for (int h = 0; h < 2; h++) {
		if (late_hands[h] == NULL || late_attached[h]) {
			continue;
		}
		if (late_serials[h][0] != '\0') {
			if (strstr(xdev->serial, late_serials[h]) == NULL) {
				continue;
			}
		} else if (!is_tracker(xdev)) {
			continue;
		}

		udcap_device_set_tracker(late_hands[h], xdev);
		late_attached[h] = true;
		UDCAP_INFO("udcap: late-attached %s hand to hotplugged tracker '%s'", h == 0 ? "LEFT" : "RIGHT",
		           xdev->serial);
		return; // One tracker only serves one hand.
	}
}

// Attach `hand_dev`'s pose to `tracker` (the device applies the live per-hand
// offset on top). Returns the device (pose stays at origin if no tracker).
static struct xrt_device *
attach_pose(struct xrt_device *hand_dev, struct xrt_device *tracker, const char *hand_name)
{
	if (hand_dev == NULL) {
		return NULL;
	}
	if (tracker == NULL) {
		UDCAP_WARN("udcap: no tracker for %s hand; pose will be at origin. "
		           "Point udcap-server --tracker-%s at a connected tracker serial.",
		           hand_name, hand_name);
		return hand_dev;
	}

	udcap_device_set_tracker(hand_dev, tracker);
	UDCAP_INFO("udcap: attached %s hand to tracker '%s'", hand_name, tracker->serial);
	return hand_dev;
}

void
udcap_create_devices(struct xrt_device *const *devs,
                     size_t dev_count,
                     struct xrt_device **out_left,
                     struct xrt_device **out_right)
{
	char cfg_left[32], cfg_right[32];
	uint32_t server_pid = read_shm_config(cfg_left, cfg_right);
	if (server_pid == 0) {
		UDCAP_INFO("udcap: no running udcap-server (shm absent or stale); not creating glove devices");
		return;
	}

	// Create the controller devices.
	struct xrt_device *left = udcap_device_create(XRT_HAND_LEFT);
	struct xrt_device *right = udcap_device_create(XRT_HAND_RIGHT);
	if (left == NULL && right == NULL) {
		return;
	}

	bool *used = dev_count ? calloc(dev_count, sizeof(bool)) : NULL;

	// Tracker serial: shm config first, then env var.
	const char *ser_left = cfg_left[0] ? cfg_left : getenv("UDCAP_TRACKER_LEFT");
	const char *ser_right = cfg_right[0] ? cfg_right : getenv("UDCAP_TRACKER_RIGHT");
	UDCAP_INFO("udcap: configured tracker serials: left='%s' right='%s' (empty = first available tracker)",
	           ser_left != NULL ? ser_left : "", ser_right != NULL ? ser_right : "");

	struct xrt_device *trk_left = NULL, *trk_right = NULL;
	if (ser_left && ser_left[0]) {
		trk_left = pick_tracker(devs, dev_count, ser_left, used);
	}
	if (ser_right && ser_right[0]) {
		trk_right = pick_tracker(devs, dev_count, ser_right, used);
	}
	if (trk_left == NULL) {
		trk_left = pick_tracker(devs, dev_count, NULL, used);
	}
	if (trk_right == NULL) {
		trk_right = pick_tracker(devs, dev_count, NULL, used);
	}

	if (left != NULL) {
		*out_left = attach_pose(left, trk_left, "LEFT");
	}
	if (right != NULL) {
		*out_right = attach_pose(right, trk_right, "RIGHT");
	}

	// Remember unattached hands so a tracker hotplugged later can be
	// picked up via udcap_notify_device_added().
	late_hands[0] = left;
	late_hands[1] = right;
	late_attached[0] = trk_left != NULL;
	late_attached[1] = trk_right != NULL;
	if (ser_left != NULL) {
		snprintf(late_serials[0], sizeof(late_serials[0]), "%s", ser_left);
	}
	if (ser_right != NULL) {
		snprintf(late_serials[1], sizeof(late_serials[1]), "%s", ser_right);
	}

	free(used);
}
