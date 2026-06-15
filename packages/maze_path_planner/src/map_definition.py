#!/usr/bin/env python3
"""
map_definition.py - Duckietown maze graph and maneuver utilities.

The planner can load a maze from an external JSON/YAML file.
If loading fails, it falls back to the defaults defined in this file.
"""

import json
import os
from typing import Optional

try:
    import yaml
    _HAS_YAML = True
except ImportError:
    _HAS_YAML = False

# ---------------------------------------------------------------------------
# Node positions (for visualization / distance computation if needed)
# ---------------------------------------------------------------------------
DEFAULT_NODE_POSITIONS = {
    'A': (0, 0), 'B': (1, 0), 'C': (2, 0), 'D': (3, 0),
    'E': (0, 1), 'F': (1, 1), 'G': (2, 1), 'H': (3, 1),
    'S': (0, 2), 'I': (1, 2), 'J': (2, 2), 'T': (3, 2),
}

# ---------------------------------------------------------------------------
# Adjacency list: { node: [(neighbour, cost_in_tiles), ...] }
# ---------------------------------------------------------------------------
DEFAULT_MAZE_GRAPH = {
    'A': [('B', 1), ('E', 1)],
    'B': [('A', 1), ('C', 1)],
    'C': [('B', 1), ('D', 1)],
    'D': [('C', 1), ('H', 1)],
    'E': [('A', 1), ('S', 1)],
    'F': [('G', 1), ('I', 1)],
    'G': [('F', 1), ('J', 1)],
    'H': [('D', 1), ('T', 1)],
    'S': [('E', 1), ('I', 1)],
    'I': [('S', 1), ('J', 1), ('F', 1)],
    'J': [('I', 1), ('T', 1), ('G', 1)],
    'T': [('J', 1), ('H', 1)],
}

# ---------------------------------------------------------------------------
# Heading changes at each node
# The heading (direction the robot faces when arriving) determines whether
# the next segment is straight, left, or right.
# Headings: 'N'(north/up), 'S'(south/down), 'E'(east/right), 'W'(west/left)
# ---------------------------------------------------------------------------

def _validate_map(node_positions: dict, graph: dict) -> bool:
    """Return True when the map has the required structure."""
    if not isinstance(node_positions, dict) or not isinstance(graph, dict):
        return False

    for node, pos in node_positions.items():
        if not isinstance(node, str):
            return False
        if not isinstance(pos, (list, tuple)) or len(pos) != 2:
            return False

    for node, neighbours in graph.items():
        if not isinstance(node, str) or node not in node_positions:
            return False
        if not isinstance(neighbours, list):
            return False
        for entry in neighbours:
            if not isinstance(entry, (list, tuple)) or len(entry) != 2:
                return False
            neighbour, cost = entry
            if not isinstance(neighbour, str) or neighbour not in node_positions:
                return False
            if not isinstance(cost, (int, float)):
                return False

    return True


def _normalize_graph(raw_graph: dict) -> dict:
    """Accept list-based or dict-based neighbour definitions."""
    graph = {}
    for node, neighbours in raw_graph.items():
        if isinstance(neighbours, dict):
            graph[node] = [[n, c] for n, c in neighbours.items()]
        else:
            graph[node] = neighbours
    return graph


def load_maze_from_file(file_path: Optional[str]):
    """
    Load node positions and graph from JSON/YAML.

    Supported schema:
    - node_positions: {A: [0, 0], B: [1, 0], ...}
    - graph: {A: [[B, 1], [E, 1]], ...}
    """
    if not file_path:
        return DEFAULT_NODE_POSITIONS, DEFAULT_MAZE_GRAPH

    if not os.path.isfile(file_path):
        return DEFAULT_NODE_POSITIONS, DEFAULT_MAZE_GRAPH

    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            raw = f.read()

        if file_path.endswith(('.yaml', '.yml')) and _HAS_YAML:
            data = yaml.safe_load(raw)
        else:
            data = json.loads(raw)

        node_positions = data.get('node_positions', {})
        graph = _normalize_graph(data.get('graph', {}))

        if _validate_map(node_positions, graph):
            return node_positions, graph
    except Exception:
        pass

    return DEFAULT_NODE_POSITIONS, DEFAULT_MAZE_GRAPH


def get_heading(from_node: str, to_node: str, node_positions: dict) -> str:
    """Return the compass heading when travelling from *from_node* to *to_node*."""
    fx, fy = node_positions[from_node]
    tx, ty = node_positions[to_node]
    dx = tx - fx
    dy = ty - fy
    if dx > 0:
        return 'E'
    if dx < 0:
        return 'W'
    if dy > 0:
        return 'S'   # row increases downward
    return 'N'


def get_maneuver(prev_node: str, curr_node: str, next_node: str, node_positions: dict) -> str:
    """
    Determine the maneuver needed at *curr_node*.

    Returns one of: 'straight', 'left', 'right', 'stop'
    """
    if next_node is None:
        return 'stop'

    # At the start node there is no meaningful arrival heading;
    # the robot simply drives forward.
    if prev_node == curr_node:
        return 'straight'

    arrival_heading = get_heading(prev_node, curr_node, node_positions)
    departure_heading = get_heading(curr_node, next_node, node_positions)

    turn_table = {
        # (arrival, departure) -> maneuver
        ('E', 'E'): 'straight',
        ('W', 'W'): 'straight',
        ('N', 'N'): 'straight',
        ('S', 'S'): 'straight',
        ('E', 'N'): 'left',
        ('E', 'S'): 'right',
        ('W', 'N'): 'right',
        ('W', 'S'): 'left',
        ('N', 'E'): 'right',
        ('N', 'W'): 'left',
        ('S', 'E'): 'left',
        ('S', 'W'): 'right',
    }
    return turn_table.get((arrival_heading, departure_heading), 'straight')


# Backwards-compatible names used in other modules.
NODE_POSITIONS = DEFAULT_NODE_POSITIONS
MAZE_GRAPH = DEFAULT_MAZE_GRAPH
