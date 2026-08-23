"""Convert internal global-routing results into RoutePlan messages."""

import hashlib
import math

from autonomy_interfaces.msg import RoutePlan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from global_route_planner.global_segment_ids import (
    route_lane_segment_ids,
)


def stable_map_revision(
    map_name: str,
    opendrive_text: str,
) -> int:
    """Return a stable uint64 fingerprint of the global map."""

    payload = (
        str(map_name)
        + "\0"
        + str(opendrive_text)
    ).encode(
        "utf-8"
    )

    digest = hashlib.sha256(
        payload
    ).digest()

    return int.from_bytes(
        digest[:8],
        byteorder="big",
        signed=False,
    )


def set_yaw_quaternion(
    orientation,
    yaw_deg: float,
) -> None:
    """Match the CARLA bridge's direct yaw convention."""

    yaw_rad = math.radians(
        float(yaw_deg)
    )

    orientation.x = 0.0
    orientation.y = 0.0
    orientation.z = math.sin(
        yaw_rad * 0.5
    )
    orientation.w = math.cos(
        yaw_rad * 0.5
    )


def pose_from_association(
    association,
    stamp,
    frame_id: str,
) -> PoseStamped:
    """Represent a map-associated world pose."""

    pose = PoseStamped()

    pose.header.stamp = stamp
    pose.header.frame_id = (
        frame_id
    )

    pose.pose.position.x = float(
        association.projected_x
    )

    pose.pose.position.y = float(
        association.projected_y
    )

    pose.pose.position.z = float(
        association.projected_z
    )

    set_yaw_quaternion(
        pose.pose.orientation,
        association.projected_yaw_deg,
    )

    return pose


def pose_from_routing_node(
    node,
    stamp,
    frame_id: str,
) -> PoseStamped:
    """Represent one sampled map-routing state."""

    pose = PoseStamped()

    pose.header.stamp = stamp
    pose.header.frame_id = (
        frame_id
    )

    pose.pose.position.x = float(
        node.x
    )

    pose.pose.position.y = float(
        node.y
    )

    pose.pose.position.z = float(
        node.z
    )

    set_yaw_quaternion(
        pose.pose.orientation,
        node.yaw_deg,
    )

    return pose


def association_quality(
    start_association,
    goal_association,
    max_projection_distance_m: float,
    max_sample_s_error_m: float,
) -> float:
    """Return a normalized association-quality heuristic.

    This is not a probability. It measures margin to the
    configured association rejection thresholds.
    """

    if max_projection_distance_m <= 0.0:
        raise ValueError(
            "max_projection_distance_m "
            "must be positive"
        )

    if max_sample_s_error_m <= 0.0:
        raise ValueError(
            "max_sample_s_error_m "
            "must be positive"
        )

    normalized_errors = (
        start_association.projection_distance_m
        / max_projection_distance_m,

        start_association.sample_s_error_m
        / max_sample_s_error_m,

        goal_association.projection_distance_m
        / max_projection_distance_m,

        goal_association.sample_s_error_m
        / max_sample_s_error_m,
    )

    worst_error = max(
        normalized_errors
    )

    return float(
        max(
            0.0,
            min(
                1.0,
                1.0 - worst_error,
            ),
        )
    )


def build_route_plan_message(
    graph,
    route,
    start_association,
    goal_association,
    route_id: int,
    map_revision: int,
    stamp,
    frame_id: str,
    max_projection_distance_m: float,
    max_sample_s_error_m: float,
) -> RoutePlan:
    """Build one valid RoutePlan from an internal route."""

    message = RoutePlan()

    message.header.stamp = stamp
    message.header.frame_id = (
        frame_id
    )

    message.route_id = int(
        route_id
    )

    message.map_revision = int(
        map_revision
    )

    message.start = (
        pose_from_association(
            start_association,
            stamp,
            frame_id,
        )
    )

    message.goal = (
        pose_from_association(
            goal_association,
            stamp,
            frame_id,
        )
    )

    path = Path()

    path.header.stamp = stamp
    path.header.frame_id = (
        frame_id
    )

    for node_key in route.node_path:
        node = graph.nodes[
            node_key
        ]

        path.poses.append(
            pose_from_routing_node(
                node,
                stamp,
                frame_id,
            )
        )

    message.coarse_reference_path = (
        path
    )

    message.lane_segment_ids = (
        route_lane_segment_ids(
            routing_graph=graph,
            route=route,
            map_revision=map_revision,
        )
    )

    message.status = (
        RoutePlan.STATUS_VALID
    )

    message.confidence = (
        association_quality(
            start_association=(
                start_association
            ),
            goal_association=(
                goal_association
            ),
            max_projection_distance_m=(
                max_projection_distance_m
            ),
            max_sample_s_error_m=(
                max_sample_s_error_m
            ),
        )
    )

    return message


def build_invalid_route_plan_message(
    start_association,
    goal_association,
    route_id: int,
    map_revision: int,
    stamp,
    frame_id: str,
) -> RoutePlan:
    """Build a RoutePlan representing a failed route search.

    Start and goal remain the successfully map-associated poses,
    while route geometry and segment provenance are empty.

    Confidence is zero because there is no usable route.
    """

    message = RoutePlan()

    message.header.stamp = stamp
    message.header.frame_id = (
        frame_id
    )

    message.route_id = int(
        route_id
    )

    message.map_revision = int(
        map_revision
    )

    message.start = (
        pose_from_association(
            start_association,
            stamp,
            frame_id,
        )
    )

    message.goal = (
        pose_from_association(
            goal_association,
            stamp,
            frame_id,
        )
    )

    path = Path()

    path.header.stamp = stamp
    path.header.frame_id = (
        frame_id
    )

    message.coarse_reference_path = (
        path
    )

    message.lane_segment_ids = []

    message.status = (
        RoutePlan.STATUS_INVALID
    )

    message.confidence = 0.0

    return message
