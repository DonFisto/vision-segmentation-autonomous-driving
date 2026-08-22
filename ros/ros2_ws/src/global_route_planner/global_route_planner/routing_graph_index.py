"""Indexes for efficient lookup of sampled routing states."""

from bisect import bisect_left
from collections import defaultdict
from dataclasses import dataclass
from typing import Optional

from global_route_planner.routing_graph import (
    DirectedRoutingGraph,
    RoutingNodeKey,
)


@dataclass(frozen=True)
class RoutingNodeMatch:
    """Nearest sampled routing state on one semantic lane."""

    node_key: RoutingNodeKey

    requested_s_m: float
    node_s_m: float
    s_error_m: float


class RoutingGraphIndex:
    """Semantic lane/s index over a DirectedRoutingGraph."""

    def __init__(
        self,
        graph: DirectedRoutingGraph,
    ) -> None:
        self._graph = graph

        lane_nodes = defaultdict(
            list
        )

        for node_key in graph.nodes:
            lane = (
                node_key.road_id,
                node_key.section_id,
                node_key.lane_id,
            )

            lane_nodes[
                lane
            ].append(
                node_key
            )

        self._lane_nodes = {}
        self._lane_s_cm = {}

        for lane, keys in lane_nodes.items():
            ordered = tuple(
                sorted(
                    keys,
                    key=lambda key: (
                        key.s_cm,
                        key,
                    ),
                )
            )

            self._lane_nodes[
                lane
            ] = ordered

            self._lane_s_cm[
                lane
            ] = tuple(
                key.s_cm
                for key in ordered
            )

    def nearest_on_lane(
        self,
        road_id: int,
        section_id: int,
        lane_id: int,
        target_s_m: float,
        max_s_error_m: Optional[float] = None,
    ) -> Optional[RoutingNodeMatch]:
        """Find the nearest sampled state on one semantic lane."""

        if (
            max_s_error_m is not None
            and max_s_error_m < 0.0
        ):
            raise ValueError(
                "max_s_error_m cannot be negative"
            )

        lane = (
            int(road_id),
            int(section_id),
            int(lane_id),
        )

        keys = self._lane_nodes.get(
            lane
        )

        values = self._lane_s_cm.get(
            lane
        )

        if not keys or not values:
            return None

        target_s_m = float(
            target_s_m
        )

        target_s_cm = int(
            round(
                target_s_m
                * 100.0
            )
        )

        index = bisect_left(
            values,
            target_s_cm,
        )

        candidate_indices = []

        if index < len(keys):
            candidate_indices.append(
                index
            )

        if index > 0:
            candidate_indices.append(
                index - 1
            )

        if not candidate_indices:
            return None

        best_index = min(
            candidate_indices,
            key=lambda candidate: (
                abs(
                    self._graph.nodes[
                        keys[candidate]
                    ].s_m
                    - target_s_m
                ),
                keys[candidate],
            ),
        )

        node_key = keys[
            best_index
        ]

        node_s_m = float(
            self._graph.nodes[
                node_key
            ].s_m
        )

        s_error_m = abs(
            node_s_m
            - target_s_m
        )

        if (
            max_s_error_m is not None
            and s_error_m > max_s_error_m
        ):
            return None

        return RoutingNodeMatch(
            node_key=node_key,
            requested_s_m=target_s_m,
            node_s_m=node_s_m,
            s_error_m=s_error_m,
        )
