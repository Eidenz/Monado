// Copyright 2018-2024, Collabora, Ltd.
// Copyright 2023-2026, NVIDIA CORPORATION.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Holds per instance action context.
 * @ingroup oxr_main
 */

#include "util/u_hashset.h"

#include "oxr_instance_action_context.h"
#include "oxr_interaction_profile_array.h"
#include "oxr_binding.h"

#include "../oxr_logger.h"


XrResult
oxr_instance_action_context_init(struct oxr_logger *log, struct oxr_instance_action_context *context)
{
	int h_ret;

	h_ret = u_hashset_create(&context->action_sets.name_store);
	if (h_ret != 0) {
		return oxr_error(log, XR_ERROR_RUNTIME_FAILURE, "Failed to create name_store hashset");
	}

	h_ret = u_hashset_create(&context->action_sets.loc_store);
	if (h_ret != 0) {
		u_hashset_destroy(&context->action_sets.name_store);
		return oxr_error(log, XR_ERROR_RUNTIME_FAILURE, "Failed to create loc_store hashset");
	}

	// Probably not needed, but done anyway.
	context->suggested_profiles.profiles = NULL;
	context->suggested_profiles.count = 0;

	return XR_SUCCESS;
}

void
oxr_instance_action_context_fini(struct oxr_instance_action_context *context)
{
	u_hashset_destroy(&context->action_sets.name_store);
	u_hashset_destroy(&context->action_sets.loc_store);
	oxr_interaction_profile_array_clear(&context->suggested_profiles);
}
