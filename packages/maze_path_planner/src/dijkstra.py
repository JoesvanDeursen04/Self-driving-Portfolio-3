#!/usr/bin/env python3
"""
dijkstra.py – Dijkstra's Shortest Path Algorithm

Finds the shortest weighted path in the Duckietown maze graph.
"""

import heapq
from typing import Dict, List, Optional, Tuple


def dijkstra(
    graph: Dict[str, List[Tuple[str, float]]],
    start: str,
    goal: str,
) -> Tuple[Optional[List[str]], float]:
    """
    Compute the shortest path from *start* to *goal* using Dijkstra's algorithm.

    Parameters
    ----------
    graph : dict
        Adjacency list  { node: [(neighbour, cost), ...] }
    start : str
        Start node name.
    goal : str
        Goal node name.

    Returns
    -------
    path : list[str] or None
        Ordered list of node names (start … goal), or *None* if unreachable.
    cost : float
        Total cost of the path (0.0 if no path found).
    """
    if start not in graph or goal not in graph:
        return None, 0.0

    # Priority queue entries: (cumulative_cost, current_node)
    heap: List[Tuple[float, str]] = [(0.0, start)]
    # Best known cost to each node
    dist: Dict[str, float] = {start: 0.0}
    # Previous node for path reconstruction
    prev: Dict[str, Optional[str]] = {start: None}

    while heap:
        cost, node = heapq.heappop(heap)

        # Skip stale entries
        if cost > dist.get(node, float('inf')):
            continue

        # Goal reached – reconstruct path
        if node == goal:
            return _reconstruct_path(prev, start, goal), cost

        for neighbour, edge_cost in graph.get(node, []):
            new_cost = cost + edge_cost
            if new_cost < dist.get(neighbour, float('inf')):
                dist[neighbour] = new_cost
                prev[neighbour] = node
                heapq.heappush(heap, (new_cost, neighbour))

    return None, 0.0  # Goal unreachable


def _reconstruct_path(
    prev: Dict[str, Optional[str]],
    start: str,
    goal: str,
) -> List[str]:
    """Walk backwards through *prev* to build the ordered path."""
    path: List[str] = []
    node: Optional[str] = goal
    while node is not None:
        path.append(node)
        node = prev.get(node)
    path.reverse()
    return path
