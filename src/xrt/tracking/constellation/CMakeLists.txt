# Copyright 2026, Beyley Cardellio
# SPDX-License-Identifier: BSL-1.0

add_library(constellation STATIC t_rift_blobwatch.c t_rift_blobwatch.h)
target_link_libraries(constellation PRIVATE xrt-interfaces aux_os aux_util aux_math)
