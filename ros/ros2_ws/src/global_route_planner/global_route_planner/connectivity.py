"""Connectivity analysis for the directed global topology graph."""

from collections import deque
from typing import List, Set

from global_route_planner.topology import (
    DirectedTopologyGraph,
    TopologyNodeKey,
)


def weakly_connected_components(
    graph: DirectedTopologyGraph,
) -> List[Set[TopologyNodeKey]]:
    """Return weak components, ignoring edge direction."""

    unvisited = set(graph.nodes)
    components = []

    while unvisited:
        start = min(unvisited)

        component = {start}
        queue = deque([start])

        unvisited.remove(start)

        while queue:
            current = queue.popleft()

            neighbors = set()

            for edge_key in graph.outgoing.get(
                current,
                (),
            ):
                neighbors.add(
                    graph.edges[edge_key].target
                )

            for edge_key in graph.incoming.get(
                current,
                (),
            ):
                neighbors.add(
                    graph.edges[edge_key].source
                )

            for neighbor in neighbors:
                if neighbor not in unvisited:
                    continue

                unvisited.remove(neighbor)
                component.add(neighbor)
                queue.append(neighbor)

        components.append(component)

    return components


def strongly_connected_components(
    graph: DirectedTopologyGraph,
) -> List[Set[TopologyNodeKey]]:
    """Return SCCs using Kosaraju's algorithm."""

    visited = set()
    finish_order = []

    def dfs_forward(node):
        visited.add(node)

        for edge_key in graph.outgoing.get(
            node,
            (),
        ):
            target = graph.edges[edge_key].target

            if target not in visited:
                dfs_forward(target)

        finish_order.append(node)

    for node in sorted(graph.nodes):
        if node not in visited:
            dfs_forward(node)

    visited.clear()
    components = []

    def dfs_reverse(node, component):
        visited.add(node)
        component.add(node)

        for edge_key in graph.incoming.get(
            node,
            (),
        ):
            source = graph.edges[edge_key].source

            if source not in visited:
                dfs_reverse(
                    source,
                    component,
                )

    for node in reversed(finish_order):
        if node in visited:
            continue

        component = set()

        dfs_reverse(
            node,
            component,
        )

        components.append(component)

    return components
