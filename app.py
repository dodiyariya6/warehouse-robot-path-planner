"""
app.py

Streamlit GUI for the Warehouse Robot Path Planner.

This file contains the interface / behaviour: session state, warehouse
generation, grid drawing, and the animations. The A* search lives in astar.py
and is simply imported and called from here. All styling lives in style.css.

Flow when "Find Path using A*" is clicked:
    astar.find_path() runs ONCE  ->  (path, cost, explored), stored in session state
    1. "search" phase : replay the stored `explored` trace (cells A* expanded, in order)
    2. "path"   phase : show the stored final path
    3. "robot"  phase : move the robot along the stored path, one cell per step
    4. "done"         : show the final result
Nothing is searched again during the animations - they only replay the stored result.
"""

import math
import random
import time
from pathlib import Path

import streamlit as st

from astar import find_path

GRID_SIZE = 10
SEARCH_FRAME_DELAY = 0.08     # seconds between search-exploration frames
SEARCH_MAX_FRAMES = 30        # a long search is replayed in at most this many frames
PATH_PAUSE_SECONDS = 0.8      # pause on the finished path before the robot moves
ROBOT_STEP_DELAY = 0.35       # seconds per robot move

# Warehouse scenarios: (min sections, max sections, min length, max length).
# A "section" is one straight run of rack cells, like a real shelving row.
DIFFICULTY_SETTINGS = {
    "Easy": (2, 4, 3, 4),
    "Medium": (4, 6, 3, 5),
    "Dense": (8, 11, 4, 6),
    "Random": (4, 7, 3, 6),
}

st.set_page_config(page_title="Warehouse Robot Path Planner", layout="centered")


def load_css():
    """Inject style.css (blank lines removed so Markdown keeps it as one HTML block)."""
    css = (Path(__file__).parent / "style.css").read_text(encoding="utf-8")
    css = "\n".join(line for line in css.splitlines() if line.strip())
    st.markdown(f"<style>\n{css}\n</style>", unsafe_allow_html=True)


def rack_visual_type(row, col):
    """Purely cosmetic: which of the 3 rack looks (see style.css) a rack cell uses."""
    return ((row // 2) + (col // 2)) % 3


# ---------------------------------------------------------
# Session state
#
# The robot's task is "go pick up an item at a specific rack", not
# "go to an arbitrary grid cell". So the warehouse tracks:
#   - target_rack: the RACK cell the robot has been sent to (blocked)
#   - goal:        the open AISLE cell right next to target_rack (the
#                  "access point") - this is the cell A* searches for,
#                  since the rack itself is physically blocked
# ---------------------------------------------------------
STATE_DEFAULTS = {
    "start": None,          # robot's home cell
    "target_rack": None,    # rack cell the robot must reach
    "goal": None,           # access point next to target_rack (A* goal)
    "racks": set(),         # blocked cells
    "warning": "",
    "difficulty": "Medium",
    # A* result (None until "Find Path" is clicked)
    "astar_result": None,   # dict: path, cost, explored
    # animation
    "phase": "idle",        # idle | search | path | robot | done
    "search_index": 0,      # how many explored cells are shown so far
    "robot_pos": None,
    "animation_index": 0,
}
for key, value in STATE_DEFAULTS.items():
    if key not in st.session_state:
        st.session_state[key] = value


def clear_path():
    """
    Forget the stored A* result (path, trace, stats) and any animation.
    Called on every edit / generate / reset: a stored path only describes the
    warehouse it was computed on, so keeping it after a change would show stale
    or impossible routes.
    """
    st.session_state.astar_result = None
    st.session_state.phase = "idle"
    st.session_state.search_index = 0
    st.session_state.robot_pos = None
    st.session_state.animation_index = 0


def reset_grid():
    """Clear the whole warehouse back to its empty starting state."""
    st.session_state.start = None
    st.session_state.target_rack = None
    st.session_state.goal = None
    st.session_state.racks = set()
    st.session_state.warning = ""
    clear_path()


# ---------------------------------------------------------
# Warehouse generation and editing rules
# ---------------------------------------------------------
def build_rack_sections(difficulty):
    """
    Build blocked cells arranged as horizontal/vertical rack sections (like
    real shelving rows) instead of scattered single obstacles. The scenario
    only changes how many sections there are and how long they are.
    """
    min_sections, max_sections, min_len, max_len = DIFFICULTY_SETTINGS[difficulty]
    racks = set()

    for _ in range(random.randint(min_sections, max_sections)):
        length = random.randint(min_len, max_len)
        if random.choice(["horizontal", "vertical"]) == "horizontal":
            row = random.randint(0, GRID_SIZE - 1)
            col_start = random.randint(0, GRID_SIZE - length)
            racks.update((row, col_start + i) for i in range(length))
        else:
            col = random.randint(0, GRID_SIZE - 1)
            row_start = random.randint(0, GRID_SIZE - length)
            racks.update((row_start + i, col) for i in range(length))

    return racks


def find_access_cell(cell, racks):
    """
    Return an open (non-rack) neighbor of `cell`, or None if `cell` is boxed
    in. The target rack itself is blocked and can never be entered, so the
    robot's real destination is this open AISLE cell beside it (the access
    point) - that cell is what astar.find_path() receives as the goal.
    """
    row, col = cell
    for d_row, d_col in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        neighbor = (row + d_row, col + d_col)
        if 0 <= neighbor[0] < GRID_SIZE and 0 <= neighbor[1] < GRID_SIZE and neighbor not in racks:
            return neighbor
    return None


def is_adjacent(cell_a, cell_b):
    return abs(cell_a[0] - cell_b[0]) + abs(cell_a[1] - cell_b[1]) == 1


def recompute_target_access():
    """
    Keep target_rack/goal consistent after the rack layout changes. If the
    target rack was removed, or its access point got blocked, pick a new
    access cell or drop the target designation entirely.
    """
    target = st.session_state.target_rack
    if target is None:
        return

    if target not in st.session_state.racks:
        st.session_state.target_rack = None
        st.session_state.goal = None
        return

    goal = st.session_state.goal
    if goal is not None and goal not in st.session_state.racks and is_adjacent(goal, target):
        return  # existing access cell is still valid

    new_access = find_access_cell(target, st.session_state.racks)
    st.session_state.goal = new_access
    if new_access is None:
        st.session_state.target_rack = None


def generate_warehouse(difficulty):
    """
    Generate a structured rack layout for the chosen difficulty, pick one rack
    as the TARGET RACK, use the open aisle cell beside it as the access point,
    and place the robot on a random open cell.

    The layout is not accepted blindly: astar.find_path() must confirm the
    robot can reach the access point (random rack sections can wall the robot
    in). Otherwise a new layout is generated.
    """
    all_cells = [(r, c) for r in range(GRID_SIZE) for c in range(GRID_SIZE)]

    for _ in range(300):
        racks = build_rack_sections(difficulty)
        open_cells = [cell for cell in all_cells if cell not in racks]

        target_candidates = [cell for cell in racks if find_access_cell(cell, racks) is not None]
        if not target_candidates or len(open_cells) < 2:
            continue
        target = random.choice(target_candidates)
        access = find_access_cell(target, racks)

        start_candidates = [cell for cell in open_cells if cell != access]
        if not start_candidates:
            continue
        start = random.choice(start_candidates)

        path, _, _ = find_path(GRID_SIZE, racks, start, access)
        if path is not None:
            st.session_state.start = start
            st.session_state.racks = racks
            st.session_state.target_rack = target
            st.session_state.goal = access
            st.session_state.warning = ""
            clear_path()
            return

    # Extremely unlikely fallback: one simple rack, always solvable.
    st.session_state.start = (0, 0)
    st.session_state.racks = {(5, 5)}
    st.session_state.target_rack = (5, 5)
    st.session_state.goal = (5, 6)
    st.session_state.warning = ""
    clear_path()


def handle_cell_click(cell, mode):
    """Update the grid state based on the currently selected editing mode."""
    st.session_state.warning = ""
    clear_path()  # editing the warehouse invalidates every previous result

    if mode == "Set Robot":
        st.session_state.start = cell
        if cell in st.session_state.racks:
            st.session_state.racks.discard(cell)  # robot can't stand inside a rack
            recompute_target_access()

    elif mode == "Set Target Rack":
        if cell == st.session_state.start:
            st.session_state.warning = "Cannot place a rack on the robot."
            return
        previous_racks = set(st.session_state.racks)
        st.session_state.racks.add(cell)
        access = find_access_cell(cell, st.session_state.racks)
        if access is None:
            st.session_state.racks = previous_racks  # boxed in, unusable as a target
            st.session_state.warning = "That rack has no open aisle beside it - choose another cell."
            return
        st.session_state.target_rack = cell
        st.session_state.goal = access

    elif mode == "Add / Remove Rack":
        if cell == st.session_state.start:
            st.session_state.warning = "Cannot place a rack on the robot."
            return
        if cell in st.session_state.racks:
            st.session_state.racks.remove(cell)
        else:
            st.session_state.racks.add(cell)
        recompute_target_access()


# ---------------------------------------------------------
# Running A*
# ---------------------------------------------------------
def run_search():
    """
    Run A* once and start the exploration replay.

    The search finishes here, before any animation. Its path and exploration
    trace are stored in session state because Streamlit re-runs the whole
    script for every animation frame; the frames just replay the stored trace
    and path instead of searching again.
    """
    clear_path()
    if st.session_state.start is None or st.session_state.goal is None:
        st.session_state.warning = "Please set both the Robot and a Target Rack first."
        return
    st.session_state.warning = ""
    path, cost, explored = find_path(
        GRID_SIZE, st.session_state.racks, st.session_state.start, st.session_state.goal
    )
    st.session_state.astar_result = {"path": path, "cost": cost, "explored": explored}
    st.session_state.phase = "search"


# ---------------------------------------------------------
# Drawing
# ---------------------------------------------------------
def cell_state(row, col, explored_cells, current_cell, path_cells):
    """Name the visual state of a cell; style.css defines how each state looks."""
    cell = (row, col)
    robot_pos = st.session_state.robot_pos

    if robot_pos is not None and cell == robot_pos:
        return "robot", "R"      # robot is moving / has arrived
    if robot_pos is None and cell == st.session_state.start:
        return "start", "R"      # robot still at home
    if cell == st.session_state.goal:
        return "access", "G"
    if cell in st.session_state.racks:
        kind = rack_visual_type(row, col)
        if cell == st.session_state.target_rack:
            return f"target{kind}", "T"
        return f"rack{kind}", " "
    if cell == current_cell:
        return "current", " "
    if cell in path_cells:
        return "path", " "
    if cell in explored_cells:
        return "explored", " "
    return ("floora" if (row + col) % 2 == 0 else "floorb"), " "


def visible_search_cells():
    """Which explored cells / current cell / path cells are visible in the current phase."""
    result = st.session_state.astar_result
    if result is None:
        return set(), None, set()

    if st.session_state.phase == "search":
        shown = result["explored"][: st.session_state.search_index]
        current = shown[-1] if shown else None
        return set(shown), current, set()

    path_cells = set(result["path"]) if result["path"] else set()
    return set(result["explored"]), None, path_cells


def draw_grid(mode, busy):
    """Draw the 10x10 warehouse; every cell is a button that edits the grid."""
    explored_cells, current_cell, path_cells = visible_search_cells()

    with st.container(key="warehouse-grid"):
        for row in range(GRID_SIZE):
            cols = st.columns(GRID_SIZE, gap="small")
            for col in range(GRID_SIZE):
                state, label = cell_state(row, col, explored_cells, current_cell, path_cells)
                with cols[col]:
                    # container key -> CSS class "st-key-cell-<state>-<row>-<col>" (see style.css)
                    with st.container(key=f"cell-{state}-{row}-{col}"):
                        if st.button(label, key=f"cell_{row}_{col}", disabled=busy):
                            handle_cell_click((row, col), mode)
                            st.rerun()


# (CSS class from style.css, label)
LEGEND_ITEMS = [
    ("sw-start", "Robot / Start"),
    ("sw-target", "Target Rack"),
    ("sw-access", "Access Point"),
    ("sw-rack", "Rack"),
    ("sw-explored", "Explored by A*"),
    ("sw-current", "Current node"),
    ("sw-path", "Final path"),
]


def draw_legend():
    items = "".join(
        f'<span><span class="swatch {css_class}"></span>{name}</span>'
        for css_class, name in LEGEND_ITEMS
    )
    st.markdown(f'<div class="legend">{items}</div>', unsafe_allow_html=True)


# ---------------------------------------------------------
# Result / statistics
# ---------------------------------------------------------
def stat(key, value):
    return f'<div class="stat"><span class="k">{key}</span><span class="v">{value}</span></div>'


def panel(title, body):
    st.markdown(
        f'<div class="panel"><div class="panel-title">{title}</div>{body}</div>',
        unsafe_allow_html=True,
    )


def fmt(cell):
    return "Not set" if cell is None else f"({cell[0]}, {cell[1]})"


def status_badge():
    """Return (css class, text) describing what the robot / search is doing now."""
    result = st.session_state.astar_result
    phase = st.session_state.phase
    if phase == "search":
        return "busy", "A* is finding a path..."
    if phase == "path":
        return "busy", "Path Found"
    if phase == "robot":
        return "busy", "Robot moving..."
    if phase == "done" and result and result["path"] is None:
        return "error", "No Path Found"
    if phase == "done":
        return "ok", "Target Reached"
    return "", "Ready"


def result_message():
    """One plain sentence explaining the current outcome, or "" while A* is still running."""
    result = st.session_state.astar_result
    if result is None:
        return '<p class="result-message muted">No search run yet. Click <b>Find Path using A*</b> to start.</p>'
    if st.session_state.phase != "done":
        return ""
    if result["path"] is None:
        return (
            '<p class="result-message error">The robot cannot reach the access point with the '
            "current warehouse layout. Try changing the rack layout or robot position.</p>"
        )
    return (
        '<p class="result-message">The robot successfully reached the access point '
        "beside the target rack.</p>"
    )


def draw_results():
    st.markdown("### Result")

    result = st.session_state.astar_result
    css_class, status_text = status_badge()

    stats = (
        stat("Start", fmt(st.session_state.start))
        + stat("Target Rack", fmt(st.session_state.target_rack))
        + stat("Access Point", fmt(st.session_state.goal))
    )
    # Path statistics appear once the exploration replay has finished and a path exists.
    if result is not None and result["path"] is not None and st.session_state.phase != "search":
        stats += stat("Path Cost", f"{result['cost']} moves") + stat(
            "Nodes Explored", len(result["explored"])
        )

    panel(
        "Robot Status",
        f'<span class="status-badge {css_class}">{status_text}</span>'
        + result_message()
        + f'<div class="stats stats-spaced">{stats}</div>',
    )


# ===========================================================
# Page
# ===========================================================
load_css()
busy = st.session_state.phase in ("search", "path", "robot")  # lock controls while animating

st.title("Warehouse Robot Path Planner")
st.caption("A* search for a warehouse robot: reach the access point beside a target rack")

st.markdown('<div class="step-label">Step 1 - Warehouse</div>', unsafe_allow_html=True)
diff_col, gen_col, reset_col = st.columns([2, 2, 1.4], vertical_alignment="bottom")
with diff_col:
    st.selectbox("Scenario", list(DIFFICULTY_SETTINGS), key="difficulty", disabled=busy)
with gen_col:
    if st.button("Generate Warehouse", disabled=busy, use_container_width=True):
        generate_warehouse(st.session_state.difficulty)
        st.rerun()
with reset_col:
    if st.button("Reset", disabled=busy, use_container_width=True):
        reset_grid()
        st.rerun()

st.markdown('<div class="step-label">Step 2 - Set robot, target rack, or edit racks</div>', unsafe_allow_html=True)
mode = st.radio(
    "Edit Mode",
    ["Set Robot", "Set Target Rack", "Add / Remove Rack"],
    horizontal=True,
    disabled=busy,
    label_visibility="collapsed",
)
st.markdown(
    '<p class="flow-hint">Pick a mode, then click a grid cell. Any edit clears the previous search.</p>',
    unsafe_allow_html=True,
)

if st.session_state.warning:
    st.warning(st.session_state.warning)

draw_grid(mode, busy)
draw_legend()
st.caption(
    "The robot's task is to reach the red access point beside the gold-bordered Target Rack. "
    "The rack itself stays blocked, so A* searches for the access point."
)

st.markdown('<div class="step-label">Step 3 - Plan and move</div>', unsafe_allow_html=True)
if st.button("Find Path using A*", type="primary", use_container_width=True, disabled=busy):
    run_search()
    st.rerun()

# One st.empty() slot holds the result area, and the Robot Status panel is a single
# element. The animation reruns end early, so if the number of elements changed
# between frames Streamlit would keep showing leftovers from the previous frame.
with st.empty().container():
    draw_results()

with st.expander("About A* Search"):
    st.write("A* Search evaluates each cell using:")
    st.latex(r"f(n) = g(n) + h(n)")
    st.write(
        "g(n) is the actual cost from the robot's position to the current cell, and "
        "h(n) is the estimated cost from the current cell to the goal. Since the "
        "target rack itself is blocked, the actual A* goal is the open aisle cell "
        "right beside it - the robot stops there rather than trying to enter the rack. "
        "The project uses the Manhattan distance for h(n) because the robot can only "
        "move up, down, left, or right."
    )
    st.latex(r"h(n) = |row_n - row_{goal}| + |col_n - col_{goal}|")

# ===========================================================
# Advance the animation, if one is in progress.
#
# This block runs LAST, after the page above has been drawn for the CURRENT
# frame; only then do we pause and move to the next frame and rerun.
# A* already ran once in run_search(). Here we only REPLAY what it returned.
# ===========================================================
phase = st.session_state.phase
result = st.session_state.astar_result

if phase == "search":
    total = len(result["explored"])
    step = max(1, math.ceil(total / SEARCH_MAX_FRAMES))
    time.sleep(SEARCH_FRAME_DELAY)
    st.session_state.search_index = min(total, st.session_state.search_index + step)
    if st.session_state.search_index >= total:
        # replay finished: show the path, or stop if A* found none
        st.session_state.phase = "path" if result["path"] else "done"
    st.rerun()

elif phase == "path":
    time.sleep(PATH_PAUSE_SECONDS)
    st.session_state.phase = "robot"
    st.session_state.animation_index = 0
    st.session_state.robot_pos = result["path"][0]
    st.rerun()

elif phase == "robot":
    path = result["path"]
    if st.session_state.animation_index < len(path) - 1:
        time.sleep(ROBOT_STEP_DELAY)
        st.session_state.animation_index += 1
        st.session_state.robot_pos = path[st.session_state.animation_index]
    else:
        st.session_state.phase = "done"
    st.rerun()
