// Copyright 2020-2024 Jan Schmidt
// Copyright 2025-2026 Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  LED search model management code
 * @author Jan Schmidt <jan@centricular.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#include "math/m_api.h"

#include "led_search_model.h"


/* Set to 0 to use 3D euclidean distance to sort neighbours,
 * 1 to use orthographic projected (undistorted) 2D distance
 * with the anchor LED is forward-facing */
#define PROJECTED_DISTANCE 0
#define DUMP_FULL_DEBUG 0

struct led_candidate_sort_entry
{
	struct t_constellation_tracker_led *led;
	double distance;
};

#define POW2(_x) ((_x) * (_x))

static double
calc_led_dist(const struct t_constellation_search_led_candidate *c,
              const struct t_constellation_tracker_led *anchor,
              struct xrt_vec3 *out_led_pos)
{
	struct xrt_vec3 led_pos;

#if PROJECTED_DISTANCE
	struct xrt_vec3 tmp;

	tmp = m_vec3_add(anchor->position, c->pose.position);
	math_quat_rotate_vec3(&c->pose.orientation, &tmp, &led_pos);

	if (out_led_pos) {
		*out_led_pos = led_pos;
	}

	if (led_pos.z > 0.0) {
		return sqrt(POW2(led_pos.x / led_pos.z) + POW2(led_pos.y / led_pos.z));
	}

	return sqrt(POW2(led_pos.x) + POW2(led_pos.y));
#else
	led_pos = m_vec3_add(anchor->position, c->pose.position);

	if (out_led_pos) {
		*out_led_pos = led_pos;
	}

	return m_vec3_len(led_pos);
#endif
}

static int
compare_led_distance(const void *elem1, const void *elem2)
{
	const struct led_candidate_sort_entry *l1 = (const struct led_candidate_sort_entry *)elem1;
	const struct led_candidate_sort_entry *l2 = (const struct led_candidate_sort_entry *)elem2;

	if (l1->distance > l2->distance) {
		return 1;
	}
	if (l1->distance < l2->distance) {
		return -1;
	}

	return 0;
}

static struct t_constellation_search_led_candidate *
t_constellation_search_led_candidate_new(struct t_constellation_tracker_led *led,
                                         struct t_constellation_tracker_led_model *led_model)
{
	struct t_constellation_search_led_candidate *c = calloc(1, sizeof(struct t_constellation_search_led_candidate));
	c->led = led;

	/*
	 * Calculate the pose that places this LED forward-facing at 0,0,0
	 * and then calculate the distance for all visible LEDs when (orthographic)
	 * projected in that pose
	 */
	const struct xrt_vec3 fwd = (struct xrt_vec3){0.0, 0.0, 1.0};

	c->pose.position = m_vec3_inverse(led->position);
	math_quat_from_vec_a_to_vec_b(&led->normal, &fwd, &c->pose.orientation);

	struct led_candidate_sort_entry array[256];

	for (size_t i = 0; i < led_model->led_count; i++) {
		struct t_constellation_tracker_led *cur = led_model->leds + i;

		// Don't put the current LED in its own neighbour list
		if (cur == led) {
			continue;
		}

		// Normals are more than 90 degrees apart - these are mutually exclusive LEDs
		if (m_vec3_dot(led->normal, cur->normal) <= 0) {
			continue;
		}

		struct led_candidate_sort_entry *entry = &array[c->num_neighbours];
		entry->led = cur;
		entry->distance = calc_led_dist(c, cur, NULL);

		c->num_neighbours++;
	}

	if (c->num_neighbours > 1) {
		qsort(array, c->num_neighbours, sizeof(struct led_candidate_sort_entry), compare_led_distance);

		c->neighbours = calloc(c->num_neighbours, sizeof(struct t_constellation_tracker_led *));
		for (uint8_t i = 0; i < c->num_neighbours; i++) {
			c->neighbours[i] = array[i].led;
		}
	}

#if DUMP_FULL_DEBUG
	U_LOG_D("Have %u neighbours for LED %u (%f,%f,%f) dir (%f,%f,%f) ", c->num_neighbours, c->led->id,
	        c->led->position.x, c->led->position.y, c->led->position.z, c->led->normal.x, c->led->normal.y,
	        c->led->normal.z);
	U_LOG_D(" front-facing rotation (%f,%f,%f,%f):", c->pose.orientation.x, c->pose.orientation.y,
	        c->pose.orientation.z, c->pose.orientation.w);

	for (uint8_t i = 0; i < c->num_neighbours; i++) {
		struct t_constellation_tracker_led *cur = c->neighbours[i];
		struct xrt_vec3 led_pos;
		double distance = calc_led_dist(c, cur, &led_pos);

		struct xrt_vec3 tmp;
		math_quat_rotate_vec3(&c->pose.orientation, &cur->normal, &tmp);
		float theta = m_vec3_dot(tmp, fwd);

		U_LOG_D(
		    "  LED id %2u @ %10.7f %10.7f %10.7f dir %10.7f %10.7f %10.7f -> %10.7f %10.7f dist %10.7f (dir "
		    "%f,%f,%f) angle %f",
		    cur->id, cur->position.x, cur->position.y, cur->position.z, cur->normal.x, cur->normal.y,
		    cur->normal.z, led_pos.x, led_pos.y, distance, tmp.x, tmp.y, tmp.z, acosf(theta) / M_PI * 180.0);
	}
#endif

	return c;
}

static void
t_constellation_search_led_candidate_free(struct t_constellation_search_led_candidate *candidate)
{
	free(candidate->neighbours);
	free(candidate);
}

/*
 * Exported functions.
 */

struct t_constellation_search_model *
t_constellation_search_model_new(t_constellation_device_id_t device_id,
                                 struct t_constellation_tracker_led_model *led_model)
{
	struct t_constellation_search_model *m = calloc(1, sizeof(struct t_constellation_search_model));

	m->device_id = device_id;
	m->led_model = led_model;

	m->points = calloc(led_model->led_count, sizeof(struct t_constellation_search_led_candidate *));
	m->num_points = led_model->led_count;

	for (size_t i = 0; i < led_model->led_count; i++) {
		m->points[i] = t_constellation_search_led_candidate_new(led_model->leds + i, led_model);
	}

	return m;
}

void
t_constellation_search_model_free(struct t_constellation_search_model *model)
{
	for (uint8_t i = 0; i < model->num_points; i++) {
		t_constellation_search_led_candidate_free(model->points[i]);
	}

	free(model->points);
	free(model);
}
