"""Connectivity analysis for project-owned directed graphs.

The algorithms here are intentionally graph-type agnostic.
"""

from collections import deque


def weakly_connected_components(
    graph,
):
    """Return weak components, ignoring edge direction."""

    unvisited = set(
        graph.nodes
    )

    components = []

    while unvisited:
        start = next(
            iter(unvisited)
        )

        component = {
            start
        }

        queue = deque(
            [start]
        )

        unvisited.remove(
            start
        )

        while queue:
            current = queue.popleft()

            neighbors = set()

            for edge_key in graph.outgoing.get(
                current,
                (),
            ):
                neighbors.add(
                    graph.edges[
                        edge_key
                    ].target
                )

            for edge_key in graph.incoming.get(
                current,
                (),
            ):
                neighbors.add(
                    graph.edges[
                        edge_key
                    ].source
                )

            for neighbor in neighbors:
                if neighbor not in unvisited:
                    continue

                unvisited.remove(
                    neighbor
                )

                component.add(
                    neighbor
                )

                queue.append(
                    neighbor
                )

        components.append(
            component
        )

    return components


def strongly_connected_components(
    graph,
):
    """Return SCCs using iterative Kosaraju search."""

    visited = set()
    finish_order = []

    # First pass: directed graph.
    for start in graph.nodes:
        if start in visited:
            continue

        stack = [
            (
                start,
                False,
            )
        ]

        while stack:
            (
                current,
                expanded,
            ) = stack.pop()

            if expanded:
                finish_order.append(
                    current
                )
                continue

            if current in visited:
                continue

            visited.add(
                current
            )

            stack.append(
                (
                    current,
                    True,
                )
            )

            for edge_key in graph.outgoing.get(
                current,
                (),
            ):
                target = graph.edges[
                    edge_key
                ].target

                if target not in visited:
                    stack.append(
                        (
                            target,
                            False,
                        )
                    )

    # Second pass: reversed graph.
    visited.clear()
    components = []

    for start in reversed(
        finish_order
    ):
        if start in visited:
            continue

        component = set()

        stack = [
            start
        ]

        visited.add(
            start
        )

        while stack:
            current = stack.pop()

            component.add(
                current
            )

            for edge_key in graph.incoming.get(
                current,
                (),
            ):
                source = graph.edges[
                    edge_key
                ].source

                if source in visited:
                    continue

                visited.add(
                    source
                )

                stack.append(
                    source
                )

        components.append(
            component
        )

    return components
