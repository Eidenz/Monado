# Copyright 2026, Beyley Cardellio
# SPDX-License-Identifier: BSL-1.0

add_library(
	constellation STATIC
	t_rift_blobwatch.c
	t_rift_blobwatch.h
	lambdatwist/cubic.h
	lambdatwist/eig3x3known0.h
	lambdatwist/lambdatwist_p3p.h
	lambdatwist/mat.h
	lambdatwist/quadratic.h
	lambdatwist/refine_lambda.h
	lambdatwist/vec.h
	)
target_link_libraries(constellation PRIVATE xrt-interfaces aux_os aux_util aux_math)
