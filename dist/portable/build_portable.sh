#!/usr/bin/env bash
# Build a RELOCATABLE, portable Monado tree from this fork.
#
# Why this exists
# ---------------
# A Monado built against your distro's bleeding-edge glibc only runs on machines
# with an equal-or-newer glibc (glibc is backwards-, not forwards-compatible).
# Two levers make a prebuilt binary portable across distros:
#   1. build on an OLD base  -> low glibc floor  (CI uses Ubuntu 22.04 = glibc 2.35)
#   2. bundle the non-system shared libs with $ORIGIN-relative RPATHs, so the
#      result no longer depends on the target's library *versions*.
# The graphics / glibc / systemd stack is deliberately LEFT to the target: its
# GPU driver and display server must provide Vulkan, GL, X11/Wayland, libdrm,
# libudev, etc. That "leave to the target" set is exactly the AppImage
# excludelist that linuxdeploy uses for Envision's AppImage -- we apply the same
# list directly here, because monado-service is a daemon (no .desktop/GUI that
# linuxdeploy expects).
#
# The CMake flags mirror Envision's Monado recipe (src/builders/build_monado.rs)
# plus the drivers we want shipped.
#
# Output: dist-out/monado-fork-<version>-linux-x86_64.tar.gz
#   top-level layout  monado/{bin,lib,share}  -- the prefix Monadeck downloads,
#   extracts, and points `monado_prefix` at.
#
# Usage: run from anywhere inside the repo.   BUILD_TYPE=RelWithDebInfo for debug.

set -euo pipefail

# Release (-O3, no debug info) for the distributed artifact. The README uses
# RelWithDebInfo, but that's only useful for getting crash backtraces during
# development -- and we strip the artifact anyway, so the symbols would be thrown
# away. Override with BUILD_TYPE=RelWithDebInfo if you need to debug a build.
BUILD_TYPE="${BUILD_TYPE:-Release}"
SRC_DIR="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
BUILD_DIR="$SRC_DIR/build-portable"
APPDIR="$BUILD_DIR/AppDir"
PREFIX="$APPDIR/usr"
OUT_DIR="$SRC_DIR/dist-out"

cd "$SRC_DIR"

VERSION="$(git describe --tags --always 2>/dev/null || true)"
[ -n "$VERSION" ] || VERSION="$(grep -m1 -oE 'VERSION[[:space:]]+[0-9]+\.[0-9]+\.[0-9]+' CMakeLists.txt | awk '{print $2}')"
VERSION="${VERSION:-dev}"

echo "==> Monado portable build  (type=$BUILD_TYPE  version=$VERSION)"
rm -rf "$BUILD_DIR" "$OUT_DIR"
mkdir -p "$PREFIX/lib" "$OUT_DIR"

# --- configure (Envision's Monado flags + relocatable rpath + shipped drivers) ---
cmake -G Ninja -S "$SRC_DIR" -B "$BUILD_DIR" \
	-DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
	-DCMAKE_INSTALL_PREFIX=/usr \
	-DCMAKE_INSTALL_LIBDIR=lib \
	-DCMAKE_INSTALL_RPATH='$ORIGIN/../lib' \
	-DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
	-DXRT_HAVE_SYSTEM_CJSON=NO \
	-DXRT_FEATURE_SERVICE=ON \
	-DXRT_FEATURE_OPENXR=ON \
	-DXRT_FEATURE_SERVICE_SYSTEMD=OFF \
	-DXRT_BUILD_DRIVER_STEAMVR_LIGHTHOUSE=ON \
	-DXRT_BUILD_DRIVER_SURVIVE=OFF \
	-DXRT_BUILD_DRIVER_UDCAP=ON \
	-DXRT_FEATURE_GESTURE_DETECTOR=OFF

# Guard: option_with_deps silently turns a driver OFF when a build dependency is
# missing (it doesn't error). Fail loudly if a fork-critical driver didn't enable.
for opt in XRT_BUILD_DRIVER_STEAMVR_LIGHTHOUSE XRT_BUILD_DRIVER_UDCAP; do
	if ! grep -q "^${opt}:BOOL=ON" "$BUILD_DIR/CMakeCache.txt"; then
		echo "ERROR: $opt did not enable (missing build dependency?)" >&2
		grep "^${opt}:" "$BUILD_DIR/CMakeCache.txt" >&2 || true
		exit 1
	fi
done

cmake --build "$BUILD_DIR"
DESTDIR="$APPDIR" cmake --install "$BUILD_DIR"

echo "==> installed binaries / libs:"
ls -la "$PREFIX/bin" "$PREFIX/lib" 2>/dev/null || true

# --- the "leave to the target" set: the canonical AppImage excludelist (what
#     linuxdeploy uses), with a minimal embedded fallback if it can't be fetched ---
EXCL_FILE="$BUILD_DIR/excludelist"
if curl -SsL --max-time 20 \
	https://raw.githubusercontent.com/AppImage/pkg2appimage/master/excludelist \
	-o "$EXCL_FILE" 2>/dev/null && [ -s "$EXCL_FILE" ]; then
	mapfile -t EXC < <(sed 's/#.*//' "$EXCL_FILE" | awk 'NF{print $1}')
	echo "==> using AppImage excludelist (${#EXC[@]} entries)"
else
	echo "==> excludelist fetch failed; using embedded fallback"
	EXC=(
		ld-linux-x86-64.so.2 libc.so.6 libm.so.6 libdl.so.2 libpthread.so.0 librt.so.1
		libresolv.so.2 libstdc++.so.6 libgcc_s.so.1 libGL.so.1 libEGL.so.1 libGLX.so.0
		libOpenGL.so.0 libGLdispatch.so.0 libdrm.so.2 libvulkan.so.1
		libwayland-client.so.0 libwayland-server.so.0 libwayland-egl.so.1
		libwayland-cursor.so.0 libxkbcommon.so.0 libX11.so.6 libX11-xcb.so.1 libxcb.so.1
		libxcb-randr.so.0 libXau.so.6 libXdmcp.so.6 libXext.so.6 libXrandr.so.2
		libXrender.so.1 libXfixes.so.3 libXi.so.6 libdbus-1.so.3 libudev.so.1
		libsystemd.so.0 libasound.so.2 libz.so.1 libexpat.so.1 libffi.so.8
		libgio-2.0.so.0 libglib-2.0.so.0 libgobject-2.0.so.0
	)
fi
declare -A SKIP
for l in "${EXC[@]}"; do SKIP["$l"]=1; done

# Force-to-system (never bundle): the GPU / kernel-coupled stack -- a bundled
# copy would mismatch the target's GPU userspace driver or kernel DRM. The
# excludelist already leaves libGL/libdrm/libX11/libxcb/libwayland-client to the
# system (the latter two avoid the infamous double-libxcb-with-system-GL crash),
# but it does NOT exclude the Vulkan loader -- and a VR runtime wants the
# target's loader so it matches that machine's own GPU ICDs. The GL/EGL/gbm
# names are listed for clarity and in case the embedded fallback list is used.
for l in libvulkan.so.1 libGL.so.1 libEGL.so.1 libGLX.so.0 libGLdispatch.so.0 \
	libOpenGL.so.0 libgbm.so.1 libdrm.so.2; do
	SKIP["$l"]=1
done

# Force-to-bundle (override the excludelist): SDL2 hard-links the PulseAudio/ALSA
# stack, but libasound isn't guaranteed present (pipewire-only / minimal boxes).
# Monado never uses audio, so a bundled copy just has to satisfy the loader --
# bundling drops the dependency entirely. Leaving it to the system is what failed
# the clean-container check.
unset 'SKIP[libasound.so.2]' 2>/dev/null || true

# Recursively copy a binary's non-excluded shared-lib deps into PREFIX/lib.
bundle_deps() {
	local elf="$1" dep base
	while read -r dep; do
		[ -f "$dep" ] || continue
		base="$(basename "$dep")"
		[ -n "${SKIP[$base]:-}" ] && continue   # belongs to the target system
		[ -f "$PREFIX/lib/$base" ] && continue   # already bundled
		cp -L "$dep" "$PREFIX/lib/$base"
		chmod u+w "$PREFIX/lib/$base"
		bundle_deps "$PREFIX/lib/$base"          # follow this lib's own deps
	done < <(ldd "$elf" 2>/dev/null | awk '/=>/ && $3 ~ /^\// {print $3}')
}

echo "==> bundling non-system libraries"
for b in "$PREFIX"/bin/*;       do [ -f "$b" ] && bundle_deps "$b"; done
for l in "$PREFIX"/lib/*.so*;    do [ -f "$l" ] && bundle_deps "$l"; done

# --- relocatable RPATHs + strip ---
command -v patchelf >/dev/null || { echo "patchelf is required" >&2; exit 1; }
find "$PREFIX/bin" -maxdepth 1 -type f \
	-exec patchelf --set-rpath '$ORIGIN/../lib' {} \; 2>/dev/null || true
find "$PREFIX/lib" -maxdepth 1 -type f -name '*.so*' \
	-exec patchelf --set-rpath '$ORIGIN' {} \; 2>/dev/null || true
find "$PREFIX/bin" "$PREFIX/lib" -type f \( -name '*.so*' -o -perm -u+x \) \
	-exec strip --strip-unneeded {} + 2>/dev/null || true

# --- assemble the tarball: top-level monado/{bin,lib,share} ---
STAGE="$BUILD_DIR/stage"
mkdir -p "$STAGE/monado"
cp -a "$PREFIX/." "$STAGE/monado/"
TARBALL="$OUT_DIR/monado-fork-${VERSION}-linux-x86_64.tar.gz"
tar -C "$STAGE" -czf "$TARBALL" monado
( cd "$OUT_DIR" && sha256sum "$(basename "$TARBALL")" > "$(basename "$TARBALL").sha256" )

echo "==> done: $TARBALL"
du -h "$TARBALL"
echo "==> bundled libs:"; ls "$PREFIX/lib" | sed 's/^/    /'
# NB: `| head` would SIGPIPE tar and trip `set -o pipefail`; sed reads to EOF.
echo "==> tree (first 25 entries):"; tar -tzf "$TARBALL" | sed -n '1,25p'
