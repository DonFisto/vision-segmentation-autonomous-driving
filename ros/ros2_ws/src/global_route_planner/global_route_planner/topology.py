"""Core directed-topology data structures.

This module intentionally contains no ROS or CARLA dependencies.

CARLA/OpenDRIVE-specific conversion belongs in
carla_topology_adapter.py.
"""

from collections import Counter, defaultdict
from dataclasses import dataclass
from typing import Dict, List


@dataclass(frozen=True, order=True)
class TopologyNodeKey:
    """Stable semantic identity of one topology endpoint."""

    road_id: int
    section_id: int
    lane_id: int
    s_cm: int


@dataclass(frozen=True)
class TopologyNode:
    """One semantic endpoint in the directed road graph."""

    key: TopologyNodeKey

    s_m: float

    x: float
    y: float
    z: float
    yaw_deg: float

    is_junction: bool


@dataclass(frozen=True, order=True)
class TopologyEdgeKey:
    """Identity of one directed CARLA topology connection."""

    source: TopologyNodeKey
    target: TopologyNodeKey


@dataclass(frozen=True)
class TopologyEdge:
    """Directed connection between two topology endpoints."""

    key: TopologyEdgeKey

    source: TopologyNodeKey
    target: TopologyNodeKey

    # Diagnostic only for now.
    #
    # This is straight-line endpoint distance, NOT the final
    # routing cost. Later we will sample the actual centerline
    # geometry and compute its arc length.
    endpoint_distance_m: float

    entry_is_junction: bool
    exit_is_junction: bool

    @property
    def junction_involved(self) -> bool:
        return (
            self.entry_is_junction
            or self.exit_is_junction
        )


class DirectedTopologyGraph:
    """Small explicit directed graph for global routing.

    This deliberately avoids NetworkX so graph semantics remain
    visible and later Dijkstra/A* implementations operate directly
    on project-owned data structures.
    """

    NODE_POSITION_TOLERANCE_M = 0.05

    def __init__(self) -> None:
        self.nodes: Dict[
            TopologyNodeKey,
            TopologyNode,
        ] = {}

        self.edges: Dict[
            TopologyEdgeKey,
            TopologyEdge,
        ] = {}

        self.outgoing: Dict[
            TopologyNodeKey,
            List[TopologyEdgeKey],
        ] = defaultdict(list)

        self.incoming: Dict[
            TopologyNodeKey,
            List[TopologyEdgeKey],
        ] = defaultdict(list)

    def add_node(self, node: TopologyNode) -> None:
        existing = self.nodes.get(node.key)

        if existing is None:
            self.nodes[node.key] = node
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
                "Topology node-key collision: "
                f"{node.key} maps to geometrically "
                "different endpoints."
            )

    def add_edge(
        self,
        source_node: TopologyNode,
        target_node: TopologyNode,
        edge: TopologyEdge,
    ) -> None:
        if edge.source != source_node.key:
            raise ValueError(
                "Edge source does not match source node."
            )

        if edge.target != target_node.key:
            raise ValueError(
                "Edge target does not match target node."
            )

        if edge.key.source != edge.source:
            raise ValueError(
                "Edge key/source mismatch."
            )

        if edge.key.target != edge.target:
            raise ValueError(
                "Edge key/target mismatch."
            )

        if edge.key in self.edges:
            raise ValueError(
                "Duplicate directed topology edge: "
                f"{edge.key}"
            )

        self.add_node(source_node)
        self.add_node(target_node)

        self.edges[edge.key] = edge

        self.outgoing[edge.source].append(edge.key)
        self.incoming[edge.target].append(edge.key)

    def out_degree(
        self,
        node: TopologyNodeKey,
    ) -> int:
        return len(self.outgoing.get(node, []))

    def in_degree(
        self,
        node: TopologyNodeKey,
    ) -> int:
        return len(self.incoming.get(node, []))

    def out_degree_distribution(self) -> Counter:
        return Counter(
            self.out_degree(node)
            for node in self.nodes
        )

    def in_degree_distribution(self) -> Counter:
        return Counter(
            self.in_degree(node)
            for node in self.nodes
        )

    def branching_nodes(self) -> List[TopologyNodeKey]:
        return sorted(
            node
            for node in self.nodes
            if self.out_degree(node) > 1
        )

    def merging_nodes(self) -> List[TopologyNodeKey]:
        return sorted(
            node
            for node in self.nodes
            if self.in_degree(node) > 1
        )

    def source_nodes(self) -> List[TopologyNodeKey]:
        return sorted(
            node
            for node in self.nodes
            if self.in_degree(node) == 0
        )

    def sink_nodes(self) -> List[TopologyNodeKey]:
        return sorted(
            node
            for node in self.nodes
            if self.out_degree(node) == 0
        )
