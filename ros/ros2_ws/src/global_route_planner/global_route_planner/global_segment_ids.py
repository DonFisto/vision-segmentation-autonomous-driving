"""Stable IDs for global OpenDRIVE routing segments.

These IDs belong to the global CARLA/OpenDRIVE routing namespace.
They are intentionally unrelated to perceived LaneMap segment IDs.
"""

import hashlib

from global_route_planner.routing_graph import (
    RoutingEdgeType,
)


ID_NAMESPACE = (
    "global_route_planner/"
    "opendrive_topology_segment/v1"
)


def topology_segment_id(
    map_revision: int,
    topology_edge_key,
) -> int:
    """Return a deterministic uint64 ID for one topology edge."""

    source = topology_edge_key.source
    target = topology_edge_key.target

    payload = (
        f"{ID_NAMESPACE}|"
        f"{int(map_revision)}|"
        f"{source.road_id},"
        f"{source.section_id},"
        f"{source.lane_id},"
        f"{source.s_cm}|"
        f"{target.road_id},"
        f"{target.section_id},"
        f"{target.lane_id},"
        f"{target.s_cm}"
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


def topology_segment_id_map(
    topology_graph,
    map_revision: int,
):
    """Build and collision-check IDs for a topology graph."""

    ids = {}

    reverse = {}

    for edge_key in topology_graph.edges:
        segment_id = topology_segment_id(
            map_revision=map_revision,
            topology_edge_key=edge_key,
        )

        previous = reverse.get(
            segment_id
        )

        if (
            previous is not None
            and previous != edge_key
        ):
            raise RuntimeError(
                "Global topology segment ID collision: "
                f"id={segment_id} "
                f"first={previous} "
                f"second={edge_key}"
            )

        ids[
            edge_key
        ] = segment_id

        reverse[
            segment_id
        ] = edge_key

    return ids


def route_topology_edge_keys(
    routing_graph,
    route,
):
    """Return ordered topology provenance of a sampled route.

    Consecutive samples belonging to the same coarse topology
    edge collapse to one entry.

    Lane-change routing edges use their source topology
    provenance. The following lane-follow edge naturally moves
    the sequence to the adjacent lane's topology segment.
    """

    ordered = []

    previous = None

    for routing_edge_key in route.edge_path:
        edge = routing_graph.edges[
            routing_edge_key
        ]

        topology_edge_key = (
            edge.source_topology_edge_key
        )

        if topology_edge_key is None:
            raise RuntimeError(
                "Routing edge has no topology provenance: "
                f"{routing_edge_key}"
            )

        # Both longitudinal and lateral transitions carry
        # their source topology provenance. This prevents
        # lane changes from inventing a separate lane-segment
        # namespace.
        if (
            edge.transition_type
            not in (
                RoutingEdgeType.LANE_FOLLOW,
                RoutingEdgeType.LANE_CHANGE_LEFT,
                RoutingEdgeType.LANE_CHANGE_RIGHT,
            )
        ):
            raise RuntimeError(
                "Unsupported routing transition type: "
                f"{edge.transition_type}"
            )

        if topology_edge_key == previous:
            continue

        ordered.append(
            topology_edge_key
        )

        previous = (
            topology_edge_key
        )

    return tuple(
        ordered
    )


def route_lane_segment_ids(
    routing_graph,
    route,
    map_revision: int,
):
    """Return ordered global topology IDs for one route."""

    topology_keys = (
        route_topology_edge_keys(
            routing_graph=routing_graph,
            route=route,
        )
    )

    return [
        topology_segment_id(
            map_revision=map_revision,
            topology_edge_key=edge_key,
        )
        for edge_key in topology_keys
    ]
