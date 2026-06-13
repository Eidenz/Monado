// Copyright 2026, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  In-headset screenshot capture (off-thread PNG writer).
 * @ingroup comp_main
 */

#include "main/comp_screenshot.h"

#include "util/u_misc.h"
#include "util/u_logging.h"
#include "util/u_screenshot.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/stat.h>

// Static so this copy of the encoder doesn't collide with the GUI's (gui_stb.c).
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"


/*
 *
 * Trigger (test harness).
 *
 */

/*
 * Fired by `kill -USR1 <monado-service pid>`: a no-hardware way to test capture.
 * Routes into the shared u_screenshot flag, the same channel the controller's
 * system button (and, later, a gesture detector) uses. u_screenshot_request()
 * is async-signal-safe.
 */
static void
screenshot_signal_handler(int sig)
{
	(void)sig;
	u_screenshot_request();
}


/*
 *
 * Worker thread.
 *
 */

/*!
 * Best-effort camera-shutter sound, fired from the worker thread so it never
 * touches the compositor. Double-fork so the player is reparented to init: no
 * zombie, and we never block on playback (a stuck audio server can't wedge the
 * worker). Override the command with MONADO_SCREENSHOT_SOUND_CMD, or silence
 * it entirely with MONADO_SCREENSHOT_NO_SOUND.
 */
static void
play_shutter_sound(void)
{
#if !defined(_WIN32)
	if (getenv("MONADO_SCREENSHOT_NO_SOUND") != NULL) {
		return;
	}

	// Read env in the parent: only async-signal-safe calls are allowed
	// between fork() and exec().
	const char *cmd = getenv("MONADO_SCREENSHOT_SOUND_CMD");
	if (cmd == NULL || cmd[0] == '\0') {
		// Default: play the freedesktop shutter sound file directly, trying the
		// players most likely to be installed. Direct file playback avoids
		// sound-theme event lookups (e.g. canberra's `-i camera-shutter`, which
		// fails when the active theme lacks the event).
		cmd = "paplay /usr/share/sounds/freedesktop/stereo/camera-shutter.oga 2>/dev/null || "
		      "pw-play /usr/share/sounds/freedesktop/stereo/camera-shutter.oga 2>/dev/null || "
		      "canberra-gtk-play -f /usr/share/sounds/freedesktop/stereo/camera-shutter.oga 2>/dev/null";
	}

	pid_t pid = fork();
	if (pid < 0) {
		return;
	}
	if (pid == 0) {
		pid_t pid2 = fork();
		if (pid2 == 0) {
			execl("/bin/sh", "sh", "-c", cmd, (char *)NULL);
			_exit(127); // exec failed
		}
		_exit(0); // intermediate child exits at once; the player reparents to init
	}

	// Parent: reap the intermediate child (returns immediately, it just forked).
	waitpid(pid, NULL, 0);
#endif
}

// Resolve the (optional) crop region into pixel bounds within the frame.
static void
resolve_crop(const struct xrt_frame *frame,
             const struct u_screenshot_request *req,
             uint32_t *out_x,
             uint32_t *out_y,
             uint32_t *out_w,
             uint32_t *out_h)
{
	uint32_t w = frame->width;
	uint32_t h = frame->height;

	*out_x = 0;
	*out_y = 0;
	*out_w = w;
	*out_h = h;

	if (req == NULL || !req->has_region) {
		return;
	}

	float fx0 = req->x0 < 0.0f ? 0.0f : (req->x0 > 1.0f ? 1.0f : req->x0);
	float fy0 = req->y0 < 0.0f ? 0.0f : (req->y0 > 1.0f ? 1.0f : req->y0);
	float fx1 = req->x1 < 0.0f ? 0.0f : (req->x1 > 1.0f ? 1.0f : req->x1);
	float fy1 = req->y1 < 0.0f ? 0.0f : (req->y1 > 1.0f ? 1.0f : req->y1);

	uint32_t px0 = (uint32_t)(fx0 * (float)w);
	uint32_t py0 = (uint32_t)(fy0 * (float)h);
	uint32_t px1 = (uint32_t)(fx1 * (float)w);
	uint32_t py1 = (uint32_t)(fy1 * (float)h);

	// Only honour a sane, non-degenerate rect; otherwise keep the full frame.
	if (px1 > px0 + 1 && py1 > py0 + 1 && px1 <= w && py1 <= h) {
		*out_x = px0;
		*out_y = py0;
		*out_w = px1 - px0;
		*out_h = py1 - py0;
	}
}

static void
write_frame_to_disk(struct comp_screenshot *cs, struct xrt_frame *frame, const struct u_screenshot_request *req)
{
	uint32_t full_w = frame->width;
	uint32_t full_h = frame->height;

	uint32_t cx, cy, cw, ch;
	resolve_crop(frame, req, &cx, &cy, &cw, &ch);

	// Readback frames are XRT_FORMAT_R8G8B8X8: 4 bytes per pixel, and the row
	// stride may exceed width*4. PNG wants tightly packed RGB, so repack the
	// (possibly cropped) region and drop the unused 4th channel. The bytes are
	// already sRGB-encoded, which is exactly what a PNG stores.
	uint8_t *rgb = malloc((size_t)cw * ch * 3);
	if (rgb == NULL) {
		U_LOG_E("screenshot: out of memory packing %ux%u", cw, ch);
		return;
	}

	for (uint32_t y = 0; y < ch; y++) {
		const uint8_t *src_row = frame->data + (size_t)(cy + y) * frame->stride + (size_t)cx * 4;
		uint8_t *dst_row = rgb + (size_t)y * cw * 3;
		for (uint32_t x = 0; x < cw; x++) {
			dst_row[x * 3 + 0] = src_row[x * 4 + 0];
			dst_row[x * 3 + 1] = src_row[x * 4 + 1];
			dst_row[x * 3 + 2] = src_row[x * 4 + 2];
		}
	}

	char path[1024];
	(void)snprintf(path, sizeof(path), "%s/monado_screenshot_%ld_%u.png", cs->dir, (long)time(NULL),
	               cs->counter++);

	int ok = stbi_write_png(path, (int)cw, (int)ch, 3, rgb, (int)(cw * 3));
	free(rgb);

	if (ok) {
		U_LOG_I("screenshot: saved %s (%ux%u of %ux%u)", path, cw, ch, full_w, full_h);
		play_shutter_sound();
	} else {
		U_LOG_E("screenshot: failed to write %s", path);
	}
}

static void *
run_worker(void *ptr)
{
	struct comp_screenshot *cs = (struct comp_screenshot *)ptr;

	os_thread_helper_lock(&cs->oth);

	while (os_thread_helper_is_running_locked(&cs->oth)) {
		if (cs->pending == NULL) {
			os_thread_helper_wait_locked(&cs->oth);
			continue;
		}

		// Take ownership of the pending frame and release the lock for the
		// (slow) encode + write.
		struct xrt_frame *frame = cs->pending;
		struct u_screenshot_request req = cs->pending_req;
		cs->pending = NULL;
		os_thread_helper_unlock(&cs->oth);

		write_frame_to_disk(cs, frame, &req);
		xrt_frame_reference(&frame, NULL); // returns the frame to the readback pool

		os_thread_helper_lock(&cs->oth);
	}

	// Drop anything still queued at shutdown.
	if (cs->pending != NULL) {
		xrt_frame_reference(&cs->pending, NULL);
	}

	os_thread_helper_unlock(&cs->oth);

	return NULL;
}


/*
 *
 * 'Exported' functions.
 *
 */

#if !defined(_WIN32)
// Create `path` and any missing parent directories (best-effort).
static void
make_dirs(const char *path)
{
	char tmp[512];
	size_t len = (size_t)snprintf(tmp, sizeof(tmp), "%s", path);
	if (len == 0 || len >= sizeof(tmp)) {
		return;
	}
	if (tmp[len - 1] == '/') {
		tmp[len - 1] = '\0';
	}
	for (char *p = tmp + 1; *p != '\0'; p++) {
		if (*p == '/') {
			*p = '\0';
			(void)mkdir(tmp, 0755);
			*p = '/';
		}
	}
	(void)mkdir(tmp, 0755);
}
#endif

int
comp_screenshot_init(struct comp_screenshot *cs)
{
	U_ZERO(cs);

	const char *env = getenv("MONADO_SCREENSHOT_DIR");
	const char *home = getenv("HOME");
	if (env != NULL && env[0] != '\0') {
		(void)snprintf(cs->dir, sizeof(cs->dir), "%s", env);
	} else if (home != NULL && home[0] != '\0') {
		(void)snprintf(cs->dir, sizeof(cs->dir), "%s/Pictures/Monado", home);
	} else {
		(void)snprintf(cs->dir, sizeof(cs->dir), "%s", "/tmp");
	}

#if !defined(_WIN32)
	// Best-effort: create the directory (and any missing parents).
	make_dirs(cs->dir);
#endif

	int ret = os_thread_helper_init(&cs->oth);
	if (ret != 0) {
		U_LOG_E("screenshot: os_thread_helper_init failed");
		return ret;
	}

	ret = os_thread_helper_start(&cs->oth, run_worker, cs);
	if (ret != 0) {
		U_LOG_E("screenshot: os_thread_helper_start failed");
		os_thread_helper_destroy(&cs->oth);
		return ret;
	}

#if !defined(_WIN32)
	// Install the test trigger. sigaction (not signal) so flags are explicit;
	// SA_RESTART so we don't interrupt the compositor's syscalls.
	struct sigaction sa;
	U_ZERO(&sa);
	sa.sa_handler = screenshot_signal_handler;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = SA_RESTART;
	if (sigaction(SIGUSR1, &sa, NULL) != 0) {
		U_LOG_W("screenshot: failed to install SIGUSR1 handler");
	}
#endif

	U_LOG_I("screenshot: ready, writing to %s (sound on; MONADO_SCREENSHOT_NO_SOUND to mute)", cs->dir);

	return 0;
}

bool
comp_screenshot_consume_request(struct comp_screenshot *cs, struct u_screenshot_request *out)
{
	(void)cs;
	return u_screenshot_consume_request(out);
}

void
comp_screenshot_submit(struct comp_screenshot *cs, struct xrt_frame *frame, const struct u_screenshot_request *req)
{
	os_thread_helper_lock(&cs->oth);

	// One-slot mailbox: if a write is still pending, drop the older frame
	// rather than let work pile up.
	if (cs->pending != NULL) {
		xrt_frame_reference(&cs->pending, NULL);
	}

	cs->pending = frame; // transfer caller's reference
	cs->pending_req = *req;
	os_thread_helper_signal_locked(&cs->oth);

	os_thread_helper_unlock(&cs->oth);
}

void
comp_screenshot_fini(struct comp_screenshot *cs)
{
	if (!cs->oth.initialized) {
		return;
	}

	// Stops the worker and joins; the worker drops any pending frame on exit.
	os_thread_helper_destroy(&cs->oth);
}
