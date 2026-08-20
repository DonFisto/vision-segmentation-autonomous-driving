"""Shortest-path algorithms for the global topology graph.

This module intentionally contains no ROS or CARLA dependencies.
"""

from dataclasses import dataclass
import heapq
import math
from itertools import count
from typing import Dict, Tuple

from global_route_planner.topology import (
    DirectedTopologyGraph,
    TopologyEdgeKey,
    TopologyNodeKey,
)


class NoPathError(RuntimeError):
    """Raised when no directed path exists between two graph nodes."""


@dataclass(frozen=True)
class ShortestPathResult:
    """Result of one graph shortest-path query."""

    start: TopologyNodeKey
    goal: TopologyNodeKey

    node_path: Tuple[TopologyNodeKey, ...]
    edge_path: Tuple[TopologyEdgeKey, ...]

    total_cost_m: float
    settled_nodes: int


def dijkstra_shortest_path(
    graph: DirectedTopologyGraph,
    start: TopologyNodeKey,
    goal: TopologyNodeKey,
) -> ShortestPathResult:
    """Compute a minimum-distance directed path.

    Edge cost is TopologyEdge.length_m.
    """

    if start not in graph.nodes:
        raise KeyError(
            f"Start node is not present in graph: {start}"
        )

    if goal not in graph.nodes:
        raise KeyError(
            f"Goal node is not present in graph: {goal}"
        )

    if start == goal:
        return ShortestPathResult(
            start=start,
            goal=goal,
            node_path=(start,),
            edge_path=(),
            total_cost_m=0.0,
            settled_nodes=1,
        )

    distances: Dict[
        TopologyNodeKey,
        float,
    ] = {
        start: 0.0,
    }

    predecessor_edge: Dict[
        TopologyNodeKey,
        TopologyEdgeKey,
    ] = {}

    settled = set()

    # Sequence number prevents heap comparisons from depending
    # on node identity when two path costs are equal.
    sequence = count()

    queue = [
        (
            0.0,
            next(sequence),
            start,
        )
    ]

    while queue:
        current_cost, _, current = heapq.heappop(
            queue
        )

        if current in settled:
            continue

        known_cost = distances.get(
            current,
            math.inf,
        )

        if current_cost > known_cost:
            continue

        settled.add(current)

        if current == goal:
            break

        for edge_key in graph.outgoing.get(
            current,
            (),
        ):
            edge = graph.edges[edge_key]

            if edge.length_m < 0.0:
                raise ValueError(
                    "Dijkstra requires non-negative "
                    f"edge costs: {edge_key}"
                )

            candidate_cost = (
                current_cost
                + edge.length_m
            )

            old_cost = distances.get(
                edge.target,
                math.inf,
            )

            if candidate_cost < old_cost:
                distances[edge.target] = (
                    candidate_cost
                )

                predecessor_edge[
                    edge.target
                ] = edge_key

                heapq.heappush(
                    queue,
                    (
                        candidate_cost,
                        next(sequence),
                        edge.target,
                    ),
                )

    if goal not in settled:
        raise NoPathError(
            "No directed path exists from "
            f"{start} to {goal}"
        )

    reversed_edges = []

    current = goal

    while current != start:
        edge_key = predecessor_edge.get(
            current
        )

        if edge_key is None:
            raise RuntimeError(
                "Dijkstra predecessor chain is "
                "incomplete."
            )

        reversed_edges.append(edge_key)

        current = graph.edges[
            edge_key
        ].source

    edge_path = tuple(
        reversed(reversed_edges)
    )

    node_path = [start]

    for edge_key in edge_path:
        edge = graph.edges[edge_key]

        if edge.source != node_path[-1]:
            raise RuntimeError(
                "Reconstructed route is "
                "not topologically continuous."
            )

        node_path.append(
            edge.target
        )

    return ShortestPathResult(
        start=start,
        goal=goal,
        node_path=tuple(node_path),
        edge_path=edge_path,
        total_cost_m=distances[goal],
        settled_nodes=len(settled),
    )
