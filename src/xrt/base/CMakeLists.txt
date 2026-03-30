# Copyright 2026, NVIDIA CORPORATION.
# SPDX-License-Identifier: BSL-1.0

add_library(base STATIC)
target_include_directories(base PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(
	base
	PRIVATE aux_util
	PUBLIC xrt-interfaces
	)

target_sources(
	base
	PRIVATE
		b_documentation.h
		b_stub.c
		b_system_devices.h
		b_system_devices.c
		b_space_overseer.h
		b_space_overseer.c
	)
