#!/usr/bin/env python3
"""
map_definition.py – Duckietown Maze Map

Defines the graph representation of the Duckietown maze.

Map layout (tiles):

    A ── B ── C ── D
    │              │      │
    E       F ── G     H
    │       │       │     │
    S ── I ── J ── T

Nodes (name -> (col, row) grid position):
  A(0,0)  B(1,0)  C(2,0)  D(3,0)
  E(0,1)  F(1,1)  G(2,1)  H(3,1)
  S(0,2)  I(1,2)  J(2,2)  T(3,2)

Edges (undirected, weight = number of tiles):
  Top row  : A-B, B-C, C-D
  Left col : A-E, E-S
  Right col: D-H, H-T
  Middle   : F-G
  Bot row  : S-I, I-J, J-T
  Verticals: I-F, J-G

Shortest path S→T: S→I→J→T  (cost = 3)
"""

# ---------------------------------------------------------------------------
# Node positions (for visualization / distance computation if needed)
# ---------------------------------------------------------------------------
NODE_POSITIONS = {
    'A': (0, 0), 'B': (1, 0), 'C': (2, 0), 'D': (3, 0),
    'E': (0, 1), 'F': (1, 1), 'G': (2, 1), 'H': (3, 1),
    'S': (0, 2), 'I': (1, 2), 'J': (2, 2), 'T': (3, 2),
}

# ---------------------------------------------------------------------------
# Adjacency list: { node: [(neighbour, cost_in_tiles), ...] }
# ---------------------------------------------------------------------------
MAZE_GRAPH = {
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

def get_heading(from_node: str, to_node: str) -> str:
    """Return the compass heading when travelling from *from_node* to *to_node*."""
    fx, fy = NODE_POSITIONS[from_node]
    tx, ty = NODE_POSITIONS[to_node]
    dx = tx - fx
    dy = ty - fy
    if dx > 0:
        return 'E'
    if dx < 0:
        return 'W'
    if dy > 0:
        return 'S'   # row increases downward
    return 'N'


def get_maneuver(prev_node: str, curr_node: str, next_node: str) -> str:
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

    arrival_heading = get_heading(prev_node, curr_node)
    departure_heading = get_heading(curr_node, next_node)

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
