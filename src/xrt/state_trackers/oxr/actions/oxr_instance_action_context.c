// Copyright 2018-2024, Collabora, Ltd.
// Copyright 2023-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Holds per instance action context.
 * @ingroup oxr_main
 */

#include "oxr_instance_action_context.h"
#include "oxr_interaction_profile_array.h"


XrResult
oxr_instance_action_context_init(struct oxr_logger *log, struct oxr_instance_action_context *context)
{
	XrResult ret = oxr_pair_hashset_init(log, &context->action_sets);
	if (ret != XR_SUCCESS) {
		return ret;
	}

	// Probably not needed, but done anyway.
	context->suggested_profiles.profiles = NULL;
	context->suggested_profiles.count = 0;

	return XR_SUCCESS;
}

void
oxr_instance_action_context_fini(struct oxr_instance_action_context *context)
{
	oxr_pair_hashset_fini(&context->action_sets);
	oxr_interaction_profile_array_clear(&context->suggested_profiles);
}
