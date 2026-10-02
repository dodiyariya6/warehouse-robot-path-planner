# Warehouse Robot Path Planner Using A\* Search

A Streamlit web application that simulates a robot planning a route through a small grid-based warehouse. The robot is given a target storage rack, and **A\* Search** with a **Manhattan distance heuristic** finds the shortest obstacle-free route to the open aisle cell beside that rack. The application then replays the search, highlights the final path and animates the robot along it.

The project was built as a B.Tech Artificial Intelligence course project. Its purpose is to make informed (heuristic) search visible on a concrete, easy-to-follow problem.

> **Scope:** this is a software simulation of path planning on a fixed 10 × 10 grid. It does not control a physical robot and does not use sensors, machine learning or real warehouse data.

---

## Contents

- [Problem Statement](#problem-statement)
- [Academic Relevance](#academic-relevance)
- [The Warehouse Model](#the-warehouse-model)
- [A\* in This Project](#a-in-this-project)
- [Features](#features)
- [Warehouse Scenarios](#warehouse-scenarios)
- [Code Organisation](#code-organisation)
- [Installation and Running](#installation-and-running)
- [Using the Application](#using-the-application)
- [Screenshots](#screenshots)
- [Limitations](#limitations)
- [Future Scope](#future-scope)

---

## Problem Statement

A robot in a warehouse has to collect an item from a particular rack. Racks are solid, so the robot can't move through them, and it can't stand inside the rack it is visiting. It has to reach an open aisle cell next to the rack, and it should get there in as few moves as possible.

The project models this as a shortest-path search on a grid and solves it with A\*. A\* fits the problem well:

- The goal location is known in advance, so the remaining distance can be estimated cheaply.
- That estimate (the Manhattan distance) never overestimates the true cost under the movement rules used here, so A\* is guaranteed to return a shortest path.
- Because the search is guided towards the goal, it usually expands fewer cells than an uninformed search such as breadth-first search. The application reports the number of explored nodes so this can be observed directly.

---

## Academic Relevance

The project maps onto **Unit I: Problem Solving by Search** of the AI syllabus, specifically **informed (heuristic) search** and **A\* Search** with a hand-designed **heuristic function**.

Expressed in the standard search-problem formulation:

| Component        | In this project                                                                  |
| ---------------- | -------------------------------------------------------------------------------- |
| State            | The robot's cell on the grid, `(row, col)`                                       |
| Initial state    | The robot's start cell                                                           |
| Actions          | Move Up, Down, Left or Right                                                      |
| Transition model | The move succeeds if the next cell is inside the grid and is not a rack          |
| Goal test        | The current cell is the Access Point of the target rack                          |
| Path cost        | 1 per move, so the cost of a path is its number of moves                         |
| Heuristic `h(n)` | Manhattan distance from the cell to the Access Point                             |

The project does not cover other syllabus topics such as adversarial search, constraint satisfaction or logic. It focuses on heuristic search alone.

---

## The Warehouse Model

The warehouse is a **10 × 10 grid**. Cells are addressed as `(row, col)`, with `(0, 0)` at the top-left.

| Element          | Meaning                                                            | Traversable |
| ---------------- | ------------------------------------------------------------------ | ----------- |
| Aisle (floor)    | Open cell the robot can move through                               | Yes         |
| Rack             | Storage rack, an obstacle                                          | No          |
| Robot / Start    | The robot's starting cell                                          | Yes         |
| **Target Rack**  | The rack the robot has been sent to                                | No          |
| **Access Point** | An open aisle cell directly beside the Target Rack, used as A\*'s goal | Yes     |

### Target Rack vs. Access Point

The Target Rack and the Access Point are kept separate on purpose:

- The **Target Rack** is a blocked cell. A\* never treats it as a reachable goal, because no path can enter it.
- The **Access Point** is the cell A\* actually searches for. The application picks it by checking the rack's four neighbours in the order up, down, left, right and taking the first one that is inside the grid and not a rack.
- If a rack has no open side, it can't be selected as a target.
- If later edits remove the target rack, the target is cleared. If edits block its Access Point, another open side is chosen, and if none remains the target is cleared.

The Access Point is chosen by that fixed neighbour order, not by distance to the robot. A different side of the rack could sometimes be closer (see [Limitations](#limitations)).

---

## A\* in This Project

A\* ranks each discovered cell `n` by

```text
f(n) = g(n) + h(n)
```

- `g(n)`: actual cost of the best known route from the robot's start to `n` (number of moves)
- `h(n)`: estimated cost from `n` to the Access Point
- `f(n)`: estimated total cost of a route through `n`

### Heuristic: Manhattan Distance

```text
h(n) = |row_n − row_goal| + |col_n − col_goal|
```

This is the familiar `|x1 − x2| + |y1 − y2|`, written in grid coordinates. It suits this project because the robot moves only in four directions with a cost of 1 per move. Reaching the goal takes at least that many horizontal plus vertical moves, so the heuristic:

- is **admissible**, because it never overestimates the true remaining cost, and therefore A\* returns a shortest path;
- is **consistent**, because it changes by at most 1 between neighbouring cells, so once a cell is expanded its cost is final and it never needs to be revisited.

Euclidean distance would also be admissible, but it underestimates more on a four-directional grid and therefore guides the search less well.

### Search Procedure (`astar.py`)

1. Put the start cell in a priority queue (a `heapq` min-heap) with `g = 0`.
2. Pop the cell with the lowest `f(n)`. When two cells have equal `f(n)`, the one with the lower `h(n)` comes first.
3. If the cell has already been expanded, skip it. Otherwise mark it expanded and record it in the exploration trace.
4. If the cell is the Access Point, rebuild the path by following parent links back to the start, then stop.
5. Otherwise, for each in-bounds, non-rack, unexpanded neighbour: compute `g = g(current) + 1`. If this beats the neighbour's best known `g`, record the parent and push the neighbour with its new `f(n)`.
6. If the queue empties without reaching the goal, report that no path exists.

`find_path()` returns three things: the **path** (a list of cells, or `None`), its **cost** (number of moves), and the **explored** list (cells in the exact order A\* expanded them). The interface animates this stored result. The search is never re-run during the animation.

### From Click to Result

```text
Find Path using A*
      │
      ▼
astar.find_path() runs once ──► path, cost, explored saved in session state
      │
      ▼
1. Exploration replay   explored cells appear in expansion order (orange = current node)
2. Final path           the shortest route is highlighted
3. Robot movement       the robot steps along the path one cell at a time
4. Result               status, path cost and nodes explored are shown
```

---

## Features

**Search**
- A\* Search with the Manhattan distance heuristic
- Shortest path under four-directional, unit-cost movement
- Path cost (moves) and number of nodes explored
- "No Path Found" feedback when the Access Point can't be reached

**Warehouse setup**
- Generated layouts in four scenarios: Easy, Medium, Dense and Random
- Racks laid out as straight horizontal or vertical sections, like shelving rows, not scattered single blocks
- Generated layouts are checked with A\* so the robot can always reach the Access Point
- Manual editing by clicking cells: set the robot, set the target rack, add or remove racks
- Reset to an empty grid

**Visualisation**
- Colour-coded grid with a legend (robot, target rack, Access Point, racks, explored cells, current node, final path)
- Replay of A\*'s exploration order
- Final path highlight followed by step-by-step robot movement
- Robot Status panel with start, target rack, Access Point and outcome
- An in-app "About A\* Search" section showing the formulas

**Consistency safeguards**
- Any edit clears the previous search result, so a path is never shown on a layout it wasn't computed for
- Controls are disabled while an animation is running
- Invalid actions are rejected with a message, for example placing a rack on the robot or choosing a rack with no open side

---

## Warehouse Scenarios

Each scenario places a random number of rack sections of random length, oriented horizontally or vertically. Sections may overlap.

| Scenario | Rack sections | Cells per section |
| -------- | ------------- | ----------------- |
| Easy     | 2–4           | 3–4               |
| Medium   | 4–6           | 3–5               |
| Dense    | 8–11          | 4–6               |
| Random   | 4–7           | 3–6               |

A target rack with at least one open side is chosen at random, and the robot is placed on a random open cell. The layout is accepted only if A\* finds a path from the robot to the Access Point. Otherwise a new layout is generated, with up to 300 attempts before falling back to a simple layout that is known to be solvable.

---

## Code Organisation

```text
warehouse-robot-path-planner/
├── app.py                  # Streamlit interface and application logic
├── astar.py                # A* search (no UI code)
├── style.css               # Visual styling of the grid, legend and panels
├── .streamlit/
│   └── config.toml         # Streamlit theme (light base, blue primary colour)
├── screenshots/            # Images used in this README
├── requirements.txt        # Python dependencies (Streamlit)
└── README.md
```

**`astar.py`** contains only the search logic:
- `manhattan_distance()`: the heuristic
- `get_neighbors()`: in-bounds up/down/left/right cells
- `reconstruct_path()`: follows parent links from goal back to start
- `find_path()`: the A\* loop, returning `(path, cost, explored)`

**`app.py`** handles everything else:
- Session state (robot, racks, target rack, Access Point, stored A\* result, animation phase)
- Scenario-based warehouse generation and validation
- Cell-click editing and Access Point selection and recomputation
- Grid drawing, legend, status panel and statistics
- The phased animation (exploration → path → robot movement → result)

**`style.css`** defines how each cell state looks: explored, current, path, robot, Access Point, and three cosmetic rack styles. `app.py` only decides which state a cell is in.

---

## Installation and Running

**Requirements:** Python 3.9 or later and pip. The app uses keyed containers and other Streamlit features, so a recent Streamlit release is needed. `pip install` will fetch the latest version.

```bash
git clone <repository-url>
cd warehouse-robot-path-planner
pip install -r requirements.txt
streamlit run app.py
```

The application opens at `http://localhost:8501`.

---

## Using the Application

1. **Step 1: Warehouse.** Choose a scenario and click **Generate Warehouse**, or click **Reset** to start from an empty grid.
2. **Step 2: Edit (optional).** Pick a mode and click grid cells:
   - **Set Robot** moves the robot's start. Choosing a rack cell removes that rack.
   - **Set Target Rack** makes the clicked cell the target rack, turning it into a rack if needed. Its Access Point is chosen automatically.
   - **Add / Remove Rack** toggles a rack on the clicked cell.
3. **Step 3: Plan and move.** Click **Find Path using A\***, then watch the exploration replay, the final path and the robot's movement.
4. Read the **Robot Status** panel: *Target Reached* with the path cost and nodes explored, or *No Path Found* if the layout blocks every route.

On the grid, **R** marks the robot, **T** the target rack and **G** the Access Point (goal).

To compare runs, keep the robot and target fixed and add racks one at a time. Watch how the path cost and the number of explored nodes change.

---

## Screenshots

**Application interface**

![Warehouse Interface](screenshots/warehouse-interface.png)

**Generated warehouse grid**

![Warehouse Grid](screenshots/warehouse-grid.png)

**A\* exploration replay**

![A* Exploration](screenshots/astar-exploration.png)

**Final result and statistics**

![Result](screenshots/result.png)

---

## Limitations

These describe the current scope of the project:

- **Simulation only.** No physical robot, sensors or hardware are involved.
- **Fixed 10 × 10 grid.** The size isn't configurable from the interface.
- **Four-directional, equal-cost movement.** There are no diagonal moves and no cell weights.
- **Static environment.** The layout doesn't change while a path is being planned or followed.
- **One robot, one target** per run.
- **Access Point by fixed order.** The first open side of the target rack is used. It isn't necessarily the side nearest the robot, so the route is shortest to that Access Point but not always to the rack overall.
- **Compressed replay.** Long explorations are replayed in at most about 30 frames, so several cells can appear in one frame. The order is preserved.

---

## Future Scope

The following are **not implemented**. They are possible extensions:

- Adjustable grid size
- Choosing the Access Point closest to the robot, or searching to all sides of the rack at once
- Weighted cells (for example congested aisles) and diagonal movement
- Multiple target racks visited in sequence
- Multiple robots sharing the warehouse
- Obstacles that change during execution, with re-planning
- Pause, step and speed controls for the animation
- Additional algorithms (BFS, Uniform Cost Search, Greedy Best-First Search) with a side-by-side comparison of path cost and nodes explored

---

## License

Developed for educational purposes as part of a B.Tech Artificial Intelligence course project.
