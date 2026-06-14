# Copyright 2019-2024, Collabora, Ltd.
# Copyright 2025, NVIDIA CORPORATION.
# SPDX-License-Identifier: BSL-1.0

###
# Shaders headers
#

set(SHADERS
    blit.comp
    clear.comp
    distortion.comp
    frame_overlay.comp
    layer.comp
    mesh.frag
    mesh.vert
    layer_cylinder.frag
    layer_cylinder.vert
    layer_equirect2.frag
    layer_equirect2.vert
    layer_projection.vert
    layer_projection.frag
    layer_quad.vert
    layer_quad.frag
	)

spirv_shaders(
	SHADER_HEADERS
	SPIRV_VERSION
	1.0 # Currently targeting Vulkan 1.0
	SOURCES
	${SHADERS}
	)

# Create a custom target that depends on the shader headers.
# This ensures the headers are built before they're needed.
# Note: Libraries using shaders must also link to comp_includes
# via target_link_libraries() to get the correct include directories.
add_custom_target(comp_shaders_headers DEPENDS ${SHADER_HEADERS})

###
# Shader library
#

add_library(comp_shaders STATIC ${SHADER_HEADERS} render_shaders_interface.h render_shaders.c)

target_link_libraries(
	comp_shaders
	PUBLIC xrt-interfaces aux_vk comp_includes
	PRIVATE aux_util
	)

# Ensure shader headers are built before compiling this library
add_dependencies(comp_shaders comp_shaders_headers)

# Shaders - marked SYSTEM so we get no warnings
target_include_directories(comp_shaders SYSTEM PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
