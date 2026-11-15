#!/usr/bin/env python3
"""Names and exact predicates for the existing ten target-1 rule groups."""

DDC10_RULES = {
    "DDC10_1": ("speed_and_future_distance", "swarm_avg_current_speed >= 1.284 AND triad_future_min_distance <= 1.475"),
    "DDC10_2": ("future_distance_and_offset", "triad_future_min_distance <= 1.475 AND triad_max_trajectory_offset >= 0.0295"),
    "DDC10_3": ("swarm_speed_and_source_ratio", "swarm_avg_current_speed >= 1.284 AND source_speed_over_swarm_avg_current <= 1.136"),
    "DDC10_4": ("speed_and_turn", "swarm_avg_current_speed >= 1.284 AND source_turn_angle_deg >= 7.079"),
    "DDC10_5": ("speed_distance_and_heading", "swarm_avg_current_speed >= 1.284 AND triad_future_min_distance <= 1.363 AND triad_mean_heading_angle_deg >= 15.54"),
    "DDC10_6": ("distance_offset_and_heading", "triad_future_min_distance <= 1.475 AND triad_max_trajectory_offset >= 0.0295 AND triad_mean_heading_angle_deg >= 15.54"),
    "DDC10_7": ("current_crowding_and_future_distance", "triad_max_pair_distance <= 3.387 AND triad_future_min_distance <= 1.475"),
    "DDC10_8": ("speed_and_victim_offset", "swarm_avg_current_speed >= 1.351 AND victim_trajectory_offset >= 0.0067"),
    "DDC10_9": ("planning_activity_and_future_distance", "triad_replan_or_stop_count >= 2 AND triad_new_traj_count >= 2 AND triad_future_min_distance <= 1.475"),
    "DDC10_10": ("closing_and_future_distance", "triad_best_distance_shrink_5s >= 0.3 AND triad_future_min_distance <= 1.475"),
}

DDC10_RULE_IDS = tuple(DDC10_RULES)
DDC10_RULE_NAMES = {rule_id: values[0] for rule_id, values in DDC10_RULES.items()}


def ddc10_or_components(rule_id):
    """Return the ordered DDC10 component rules for a DDC10_OR_<ids> arm."""
    prefix = "DDC10_OR_"
    value = str(rule_id).strip().upper()
    if not value.startswith(prefix):
        return ()
    suffix = value[len(prefix):]
    if not suffix:
        return ()
    try:
        numbers = [int(item) for item in suffix.split("_")]
    except ValueError:
        return ()
    if len(numbers) < 2 or len(set(numbers)) != len(numbers):
        return ()
    components = ["DDC10_{}".format(number) for number in numbers]
    if any(component not in DDC10_RULES for component in components):
        return ()
    return tuple(sorted(components, key=DDC10_RULE_IDS.index))


def ddc10_or_rule_id(rule_ids):
    """Build a stable composite arm ID from two or more DDC10 rule IDs."""
    components = [str(item).strip().upper() for item in rule_ids]
    if len(components) < 2 or len(set(components)) != len(components):
        raise ValueError("a DDC10 OR group needs at least two distinct rule IDs")
    if any(component not in DDC10_RULES for component in components):
        raise ValueError("unknown DDC10 rule in OR group")
    components.sort(key=DDC10_RULE_IDS.index)
    return "DDC10_OR_" + "_".join(component.rsplit("_", 1)[1] for component in components)
