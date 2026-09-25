"""
astar.py

A* Search implementation for the Warehouse Robot Path Planner.

This file contains ONLY the A* algorithm logic.
No GUI code should be written here.

The warehouse is a grid of (row, col) cells. The robot can move
Up, Down, Left, Right (no diagonal moves). Each move costs 1.

The goal passed in is the open ACCESS POINT beside the target rack, never the
rack itself (racks are blocked cells, so they can never be reached).

f(n) = g(n) + h(n)
    g(n) = actual cost from start to node n
    h(n) = estimated cost from node n to goal (Manhattan distance)
"""

import heapq


def manhattan_distance(cell, goal):
    """
    Heuristic h(n): Manhattan distance between a cell and the goal.

    With only 4-directional moves of cost 1, the real cost can never be less
    than this, so h(n) never overestimates - which is what keeps A* optimal.
    """
    row1, col1 = cell
    row2, col2 = goal
    return abs(row1 - row2) + abs(col1 - col2)


def get_neighbors(cell, grid_size):
    """Return the valid Up/Down/Left/Right neighbors of a cell that lie inside the grid."""
    row, col = cell
    possible_moves = [
        (row - 1, col),  # up
        (row + 1, col),  # down
        (row, col - 1),  # left
        (row, col + 1),  # right
    ]

    neighbors = []
    for r, c in possible_moves:
        if 0 <= r < grid_size and 0 <= c < grid_size:
            neighbors.append((r, c))
    return neighbors


def reconstruct_path(came_from, current):
    """Walk backwards from goal to start using the came_from map and reverse it."""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def find_path(grid_size, obstacles, start, goal):
    """
    Run A* search on the warehouse grid.

    grid_size: size of the grid (grid_size x grid_size)
    obstacles: a set of (row, col) cells that are blocked
    start: (row, col) starting cell
    goal: (row, col) goal cell

    Returns (path, cost, explored):
        path     - list of (row, col) cells from start to goal (inclusive),
                   or None if no path exists
        cost     - number of moves (len(path) - 1), or None if no path exists
        explored - the cells A* expanded, in the exact order it expanded them.
                   This is the search trace the UI replays; it is recorded
                   during the one and only search, never re-computed.
    """

    # open_set is a priority queue of (f_score, h_score, cell). Python's heapq
    # is a min-heap, so the cell with the smallest f = g + h is popped first.
    # h_score only breaks ties, so A* prefers cells that look closer to the goal.
    start_h = manhattan_distance(start, goal)
    open_set = [(start_h, start_h, start)]

    came_from = {}          # cell -> parent cell
    g_score = {start: 0}    # best known cost from start to each discovered cell
    closed = set()          # cells already expanded
    explored = []           # expansion order (the search trace)

    while open_set:
        _, _, current = heapq.heappop(open_set)

        if current in closed:
            # Stale entry. heapq cannot update a queued item, so when a cheaper route to
            # a cell is found we push a NEW entry and leave the old one behind. The
            # cheaper entry always pops first, so any later copy is skipped here.
            continue
        closed.add(current)
        explored.append(current)

        if current == goal:
            path = reconstruct_path(came_from, current)
            cost = len(path) - 1  # number of moves, not number of cells
            return path, cost, explored

        for neighbor in get_neighbors(current, grid_size):
            # Manhattan distance is consistent on a 4-directional unit-cost grid, so a
            # cell's g is already optimal when it is expanded and closed cells can be skipped.
            if neighbor in obstacles or neighbor in closed:
                continue

            tentative_g = g_score[current] + 1  # every move costs 1

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                h = manhattan_distance(neighbor, goal)
                heapq.heappush(open_set, (tentative_g + h, h, neighbor))

    # open_set became empty without reaching the goal
    return None, None, explored
