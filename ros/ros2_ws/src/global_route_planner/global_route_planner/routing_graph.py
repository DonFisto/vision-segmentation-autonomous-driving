"""Sampled lane-level graph used by global route search.

The topology graph remains the coarse OpenDRIVE representation.
This graph expands its geometry into longitudinal samples and
adds explicit legal lateral lane-change transitions.
"""

from collections import defaultdict
from dataclasses import dataclass
from enum import IntEnum
from typing import (
    Dict,
    List,
    Optional,
)

from global_route_planner.topology import (
    TopologyEdgeKey,
)


class RoutingEdgeType(IntEnum):
    """Semantic routing transition type."""

    LANE_FOLLOW = 1
    LANE_CHANGE_LEFT = 2
    LANE_CHANGE_RIGHT = 3


@dataclass(
    frozen=True,
    order=True,
)
class RoutingNodeKey:
    """Semantic identity of one sampled lane position."""

    road_id: int
    section_id: int
    lane_id: int
    s_cm: int


@dataclass(frozen=True)
class RoutingNode:
    """One sampled state on a nominal map lane."""

    key: RoutingNodeKey

    s_m: float

    x: float
    y: float
    z: float
    yaw_deg: float

    is_junction: bool


@dataclass(
    frozen=True,
    order=True,
)
class RoutingEdgeKey:
    """Identity of one directed routing transition."""

    source: RoutingNodeKey
    target: RoutingNodeKey
    transition_type: RoutingEdgeType


@dataclass(frozen=True)
class RoutingEdge:
    """One legal transition in the route-search graph."""

    key: RoutingEdgeKey

    source: RoutingNodeKey
    target: RoutingNodeKey

    transition_type: RoutingEdgeType

    # Geometric displacement represented by this transition.
    length_m: float

    # Search cost. This may include penalties in addition
    # to geometric length.
    cost_m: float

    # Provenance in the coarse topology graph.
    source_topology_edge_key: Optional[
        TopologyEdgeKey
    ]


class DirectedRoutingGraph:
    """Explicit sampled directed graph for route search."""

    NODE_POSITION_TOLERANCE_M = 0.05

    def __init__(self) -> None:
        self.nodes: Dict[
            RoutingNodeKey,
            RoutingNode,
        ] = {}

        self.edges: Dict[
            RoutingEdgeKey,
            RoutingEdge,
        ] = {}

        self.outgoing: Dict[
            RoutingNodeKey,
            List[RoutingEdgeKey],
        ] = defaultdict(list)

        self.incoming: Dict[
            RoutingNodeKey,
            List[RoutingEdgeKey],
        ] = defaultdict(list)

    def add_node(
        self,
        node: RoutingNode,
    ) -> None:
        existing = self.nodes.get(
            node.key
        )

        if existing is None:
            self.nodes[
                node.key
            ] = node
            return

        dx = existing.x - node.x
        dy = existing.y - node.y
        dz = existing.z - node.z

        distance_sq = (
            dx * dx
            + dy * dy
            + dz * dz
        )

        tolerance_sq = (
            self.NODE_POSITION_TOLERANCE_M
            * self.NODE_POSITION_TOLERANCE_M
        )

        if distance_sq > tolerance_sq:
            raise ValueError(
                "Routing node-key collision: "
                f"{node.key} maps to geometrically "
                "different samples."
            )

    def add_edge(
        self,
        edge: RoutingEdge,
    ) -> None:
        if edge.source not in self.nodes:
            raise KeyError(
                "Routing edge source node "
                f"is missing: {edge.source}"
            )

        if edge.target not in self.nodes:
            raise KeyError(
                "Routing edge target node "
                f"is missing: {edge.target}"
            )

        if edge.key.source != edge.source:
            raise ValueError(
                "Routing edge key/source mismatch."
            )

        if edge.key.target != edge.target:
            raise ValueError(
                "Routing edge key/target mismatch."
            )

        if (
            edge.key.transition_type
            != edge.transition_type
        ):
            raise ValueError(
                "Routing edge key/type mismatch."
            )

        if edge.length_m < 0.0:
            raise ValueError(
                "Routing edge length cannot be negative."
            )

        if edge.cost_m < 0.0:
            raise ValueError(
                "Routing edge cost cannot be negative."
            )

        if edge.key in self.edges:
            raise ValueError(
                "Duplicate routing edge: "
                f"{edge.key}"
            )

        self.edges[
            edge.key
        ] = edge

        self.outgoing[
            edge.source
        ].append(
            edge.key
        )

        self.incoming[
            edge.target
        ].append(
            edge.key
        )
