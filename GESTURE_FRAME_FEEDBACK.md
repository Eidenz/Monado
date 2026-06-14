# Deferred idea: in-headset "frame detected" feedback for the finger-frame gesture

**Status:** scoped, **deferred 2026-06-13** (small UX reward for a fair amount of
compositor work). This doc captures the scouting so it can be picked up later.

## Goal

While the finger-frame gesture is being held (during the ~2 s hold before the
shutter fires), draw a rectangle outline in the headset showing the region about
to be captured — like VRHandsFrame's red frame. Confirms "detected, about to
shoot" and previews the crop.

## Decision: draw it in Monado, not in monado-frame

The overlay app (`monado-frame`) *could* draw it via quad layers, but its world
origin (overlay `LOCAL` space) may not match the server's tracking space, so the
frame could sit offset from the fingers. Drawing inside the compositor avoids
this entirely: the compositor projects with its **own per-eye matrices**, so the
frame is perfectly aligned and depth-correct. It's also self-contained (works
without the overlay running) and lower latency. The cost is custom rendering in
the compositor.

## Scout findings (paths are in this repo, `~/monado`)

- **Default pipeline on Linux is COMPUTE** — `USE_COMPUTE_DEFAULT = true` for
  non-Android, `src/xrt/compositor/main/comp_settings.c:14-18`. (GFX only on
  Android or `XRT_COMPOSITOR_COMPUTE=false`.)
- **Per-eye scratch image** (undistorted, post-layer, pre-distortion) is the
  surface to draw into. It's what the screenshot path already reads back, and
  it's populated during gameplay (the in-game gesture crop proves it). It's
  created **mutable-format with both a `STORAGE` view (compute-writable) and a
  `COLOR_ATTACHMENT` view** — `src/xrt/compositor/util/comp_scratch.c:174-213`.
  So it can be drawn into from either a compute dispatch or a graphics pass.
  - High-level scratch: `src/xrt/compositor/util/comp_high_level_scratch.{c,h}`.
- **Compute render path:** `src/xrt/compositor/util/comp_render_cs.c` — layers
  are composited into the per-eye scratch, then a distortion compute pass reads
  scratch → writes target. Insert our draw between those two.
- **GFX render path (not default):** `src/xrt/compositor/util/comp_render_gfx.c`
  — layers rendered into the scratch via a render pass; inject a draw after the
  last layer, before `render_gfx_end_view()` (~`:865`). Per-eye MVPs available as
  `state->eye_vp_full` / `state->world_vp_full`. Model after the mesh draw in
  `src/xrt/compositor/render/render_gfx.c:1166` (`render_gfx_mesh_draw`).
- **Frame entry point:** `comp_renderer_draw()` at
  `src/xrt/compositor/main/comp_renderer.c:1033`; GFX/compute chosen ~`:1094`.
- **Detector & compositor share the process** (`monado-service`):
  `src/xrt/ipc/server/ipc_server_process.c:693` creates the gesture detector;
  the compositor (`xsysc`) lives in the same `ipc_server` struct.
- **The detector already has what's needed** — `src/xrt/auxiliary/util/u_gesture.c`:
  - 90 Hz tick (`TICK_NS = 1e9/90`, `:36`).
  - Computes the **4 finger corners in tracking space** in
    `request_framed_screenshot()` (~`:296-306`) via
    `math_pose_transform_point(&hand_pose.pose, &local, &world)`.
  - Head/eye poses + FOVs via `xrt_device_get_view_poses` (~`:282`); projects to
    the left eye in `project_to_uv()` (~`:238-260`) → normalized crop rect →
    `u_screenshot_request_rect()`.
  - Arming state machine in `detect_and_maybe_fire()` (~`:344-417`):
    `hold_start_ns` (0 = not held), `hold_ns`, `last_fire_ns`, `cooldown_ns`.
    Progress = `(now - hold_start_ns) / hold_ns`.
- **Channel template:** `src/xrt/auxiliary/util/u_screenshot.{c,h}` — a
  process-global atomic flag + POD data, produced by the detector and consumed
  in the render path (`comp_renderer.c:~1185`). Copy this pattern.

## Implementation plan

**Detector side (small, low risk):**
1. New `src/xrt/auxiliary/util/u_gesture_feedback.{c,h}` mirroring
   `u_screenshot`: process-global `{armed: bool, progress: f32, corners[4]:
   xrt_vec3 (tracking space)}` with set/get.
2. In `u_gesture.c`, compute the 4 corners whenever the pose is *held* (refactor
   so it's not fire-only) and publish armed + progress + corners each tick.
   Clear (armed=false) when not held. Gate behind a new `frame_feedback` key in
   `gestures.json` (config load + hot-reload already exist).

**Compositor side (the bulk):**
3. In `comp_renderer_draw()` near the screenshot consume (`comp_renderer.c:~1185`),
   consume the feedback state.
4. Project the 4 world corners into **each eye** with the per-view matrices the
   render path already computes (per-eye → disparity → correct depth). Either
   the rotated quad (4 projected corners) or each eye's axis-aligned bounding box
   of them (simpler shader; still depth-correct via per-eye disparity).
   NOTE: do NOT draw the same UV in both eyes — that fuses at infinity and is
   uncomfortable over near content. Per-eye projection is required.
5. New compute pass writing the rectangle edges into the scratch `STORAGE` (unorm)
   view, inserted between layer-accum and distortion in `comp_render_cs.c`, per
   view, gated on `armed`:
   - `src/xrt/compositor/shaders/frame_overlay.comp` (~40 lines: edge-distance or
     box-edge test; color/thickness/progress via UBO) + register in
     `src/xrt/compositor/shaders/CMakeLists.txt` (or wherever shaders are listed).
   - Pipeline + descriptor in `src/xrt/compositor/render/render_resources.c` and
     `render_interface.h`; dispatch helper in `render_compute.c` (model after the
     existing layer/distortion compute passes).
   - Correct image **barriers/layout** for the scratch between layer-accum → our
     pass → distortion read.
   (GFX-path alternative: a line-list graphics pipeline into the scratch render
   pass, modeled on `render_gfx_mesh_draw`. Only needed if running GFX.)

## Effort & risks

- Detector + channel: a few hours, low risk (mirrors `u_screenshot`).
- Compositor compute pass: ~1–2 days of build/test iteration — intricate
  plumbing (new compute pipeline + descriptors + dispatch + barriers + shader
  build), but the existing layer/distortion passes are templates.
- Risks to nail:
  1. A compute **fast path** might composite a single projection layer straight
     to the target, skipping the scratch — screenshots suggest the scratch *is*
     used in-game, but confirm it isn't bypassed in some scenes.
  2. Image **barriers/layout** for the scratch between the three passes.
  3. Matching the corner projection to the layer projection exactly.

## Recommended first step (de-risk)

Before wiring the detector, **hard-code a fixed rectangle drawn into the scratch
every frame** (no gesture). Once a rectangle reliably appears in-headset at the
right place/depth, the risky compositor plumbing is proven; the rest (detector
publish + projection + toggle) is straightforward.
