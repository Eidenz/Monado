// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  C++ helpers for weak pointers.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 *
 * @ingroup aux_util
 */

#pragma once

#include <memory>


#define U_ASSERT_WEAK_PTR_RET(ptr, msg, ...)                                                                           \
	do {                                                                                                           \
		if (!ptr) {                                                                                            \
			U_LOG_E("Handle " #ptr " has expired unexpectedly! [%s:%d]: " msg, __func__, __LINE__);        \
			assert(false);                                                                                 \
			return __VA_ARGS__;                                                                            \
		}                                                                                                      \
	} while (0)
