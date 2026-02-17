

import pygame
import sys
import heapq
import random
from collections import deque


#  WINDOW / GRID CONSTANTS

ROWS      = 22
COLS      = 36
CELL      = 24
PANEL_W   = 270
BOT_H     = 84

WIN_W = COLS * CELL + PANEL_W
WIN_H = ROWS * CELL + BOT_H

FPS       = 60
DEF_SPEED = 50       # ms between steps
DLS_LIMIT = 14       # default depth limit for DLS


#  CELL STATES

EMPTY    = 0
WALL     = 1
START    = 2
TARGET   = 3
FRONTIER = 4
EXPLORED = 5
PATH     = 6


#  COLOUR PALETTE

PAL = {
    "bg":      (7,   11,  19),
    "empty":   (16,  22,  36),
    "wall":    (42,  48,  68),
    "start":   (0,   224, 112),   # green
    "target":  (255, 55,  75),    # red
    "frontier":(255, 198, 48),    # amber   <- Frontier nodes
    "explored":(28,  98,  175),   # blue    <- Explored nodes
    "path":    (255, 105, 105),   # coral   <- Final path
    "bwd":     (80,  155, 235),   # lt-blue <- Bidirectional back-wave
    "panel":   (11,  17,  29),
    "border":  (38,  58,  98),
    "txt":     (198, 208, 228),
    "txt_hi":  (0,   228, 255),
    "txt_dim": (78,  98,  128),
    "btn_on":  (0,   175, 195),
    "btn_off": (28,  48,  78),
    "barb":    (9,   14,  24),
    "black":  (0, 0, 0),
    "yellow": (255, 221, 0),
}


#  MOVEMENT ORDER — exactly 6 directions as specified

DIRS = [
    (-1,  0),   # 1. Up
    ( 0, +1),   # 2. Right
    (+1,  0),   # 3. Bottom
    (+1, +1),   # 4. Bottom-Right (diagonal)
    ( 0, -1),   # 5. Left
    (-1, -1),   # 6. Top-Left    (diagonal)
]



#  GRID CLASS

class Grid:
    def __init__(self):
        self.rows   = ROWS
        self.cols   = COLS
        self.start  = (1, 1)
        self.target = (ROWS - 2, COLS - 2)
        self.cells  = [[EMPTY] * COLS for _ in range(ROWS)]

    def reset(self, density=0.28):
        rng = random.Random(99)
        for r in range(self.rows):
            for c in range(self.cols):
                border = (r == 0 or r == self.rows - 1 or
                          c == 0 or c == self.cols - 1)
                self.cells[r][c] = (WALL if (border or rng.random() < density)
                                    else EMPTY)
        self._carve_corridor()
        self.cells[self.start[0]][self.start[1]]   = START
        self.cells[self.target[0]][self.target[1]] = TARGET

    def _carve_corridor(self):
        """Guarantee at least one traversable path from start to target."""
        r, c = self.start
        tr, tc = self.target
        while r != tr:
            r += 1 if r < tr else -1
            if self.cells[r][c] == WALL:
                self.cells[r][c] = EMPTY
        while c != tc:
            c += 1 if c < tc else -1
            if self.cells[r][c] == WALL:
                self.cells[r][c] = EMPTY

    def passable(self, r, c):
        return (0 <= r < self.rows and 0 <= c < self.cols
                and self.cells[r][c] != WALL)

    def neighbors(self, r, c):
        """Passable neighbors in the 6 specified directions."""
        return [(r + dr, c + dc) for dr, dc in DIRS
                if self.passable(r + dr, c + dc)]

    def cost(self, r1, c1, r2, c2):
        """Cardinal move = 1.0, diagonal move = √2."""
        return 1.414 if (r1 != r2 and c1 != c2) else 1.0

    def clear_overlay(self):
        """Remove search coloring, keep only walls."""
        for r in range(self.rows):
            for c in range(self.cols):
                if self.cells[r][c] in (FRONTIER, EXPLORED, PATH):
                    self.cells[r][c] = EMPTY
        self.cells[self.start[0]][self.start[1]]   = START
        self.cells[self.target[0]][self.target[1]] = TARGET



#  PATH RECONSTRUCTION

def reconstruct(came_from, node):
    path = []
    while node is not None:
        path.append(node)
        node = came_from.get(node)
    return path[::-1]



#  ALGORITHM GENERATORS
#
#  Every generator yields exactly:
#    (frontier_set, explored_set, bwd_set, path_or_None, status_str)
#
#  bwd_set — only used by Bidirectional; all others yield None
#  path    — non-None only when goal is found


# ─── 1. BFS ──────────────────────────────────────────────────────
def bfs(grid, _=None):
    start, goal = grid.start, grid.target
    queue     = deque([start])
    came_from = {start: None}
    explored  = set()

    while queue:
        node = queue.popleft()
        if node in explored:
            continue
        explored.add(node)

        if node == goal:
            path = reconstruct(came_from, node)
            yield set(queue), explored, None, path, "BFS: Path Found!"
            return

        for nb in grid.neighbors(*node):
            if nb not in came_from:
                came_from[nb] = node
                queue.append(nb)

        yield set(queue), explored, None, None, f"BFS: Exploring {node}"

    yield set(), explored, None, None, "BFS: No path found!"


# ─── 2. DFS ──────────────────────────────────────────────────────
def dfs(grid, _=None):
    start, goal = grid.start, grid.target
    stack     = [start]
    came_from = {start: None}
    explored  = set()

    while stack:
        node = stack.pop()
        if node in explored:
            continue
        explored.add(node)

        if node == goal:
            path = reconstruct(came_from, node)
            yield set(stack), explored, None, path, "DFS: Path Found!"
            return

        # reversed so direction priority order is respected after pop()
        for nb in reversed(grid.neighbors(*node)):
            if nb not in came_from:
                came_from[nb] = node
                stack.append(nb)

        yield set(stack), explored, None, None, f"DFS: Exploring {node}"

    yield set(), explored, None, None, "DFS: No path found!"


# ─── 3. UCS ──────────────────────────────────────────────────────
def ucs(grid, _=None):
    start, goal = grid.start, grid.target
    heap        = [(0.0, 0, start)]
    came_from   = {start: None}
    cost_so_far = {start: 0.0}
    explored    = set()
    counter     = 1

    while heap:
        g, _, node = heapq.heappop(heap)
        if node in explored:
            continue
        explored.add(node)

        if node == goal:
            path = reconstruct(came_from, node)
            yield set(), explored, None, path, f"UCS: Found! Cost={g:.2f}"
            return

        for nb in grid.neighbors(*node):
            new_g = g + grid.cost(*node, *nb)
            if nb not in cost_so_far or new_g < cost_so_far[nb]:
                cost_so_far[nb] = new_g
                came_from[nb]   = node
                heapq.heappush(heap, (new_g, counter, nb))
                counter += 1

        frontier_set = {item[2] for item in heap}
        yield frontier_set, explored, None, None, f"UCS: g={g:.2f} @ {node}"

    yield set(), explored, None, None, "UCS: No path found!"


# ─── 4. DLS ──────────────────────────────────────────────────────
def dls(grid, limit=DLS_LIMIT):
    """Depth-Limited Search — animated with explicit stack."""
    start, goal = grid.start, grid.target
    stack     = [(start, 0)]    # (node, depth)
    came_from = {start: None}
    explored  = set()
    cutoff    = False

    while stack:
        node, depth = stack.pop()
        if node in explored:
            continue
        explored.add(node)

        if node == goal:
            path = reconstruct(came_from, node)
            yield {n for n, _ in stack}, explored, None, path, \
                  f"DLS: Found at depth {depth}! (limit={limit})"
            return

        if depth < limit:
            for nb in reversed(grid.neighbors(*node)):
                if nb not in explored:
                    came_from[nb] = node
                    stack.append((nb, depth + 1))
        else:
            cutoff = True

        yield {n for n, _ in stack}, explored, None, None, \
              f"DLS: depth={depth}/{limit}  node={node}"

    msg = (f"DLS: Cutoff reached (limit={limit})" if cutoff
           else "DLS: No path found!")
    yield set(), explored, None, None, msg


# ─── 5. IDDFS ────────────────────────────────────────────────────
def iddfs(grid, _=None):
    """Iterative Deepening DFS: runs DLS with limit 1, 2, 3, …"""
    start, goal  = grid.start, grid.target
    max_depth    = ROWS + COLS
    total_steps  = 0

    for limit in range(1, max_depth + 1):
        stack     = [(start, 0)]
        came_from = {start: None}
        explored  = set()
        cutoff    = False

        while stack:
            node, depth = stack.pop()
            if node in explored:
                continue
            explored.add(node)
            total_steps += 1

            if node == goal:
                path = reconstruct(came_from, node)
                yield set(), explored, None, path, \
                      f"IDDFS: Found! iter={limit} steps={total_steps}"
                return

            if depth < limit:
                for nb in reversed(grid.neighbors(*node)):
                    if nb not in explored:
                        came_from[nb] = node
                        stack.append((nb, depth + 1))
            else:
                cutoff = True

            yield {n for n, _ in stack}, explored, None, None, \
                  f"IDDFS: iter={limit}  depth={depth}  node={node}"

        if not cutoff:
            yield set(), explored, None, None, "IDDFS: No path found!"
            return

    yield set(), explored, None, None, "IDDFS: Max depth exceeded!"


# ─── 6. BIDIRECTIONAL SEARCH ─────────────────────────────────────
def bidirectional(grid, _=None):
    """
    Bidirectional BFS:
    - Forward wave from START  → shown in amber (frontier)
    - Backward wave from TARGET → shown in light blue (bwd)
    - Terminates when the two waves meet
    """
    start, goal = grid.start, grid.target

    fwd_queue = deque([start])
    bwd_queue = deque([goal])
    fwd_from  = {start: None}
    bwd_from  = {goal:  None}
    fwd_vis   = {start}
    bwd_vis   = {goal}

    def join(meeting):
        fp = reconstruct(fwd_from, meeting)
        bp = reconstruct(bwd_from, meeting)
        return fp + bp[-2::-1]   # start→meeting→goal, no duplicate

    steps = 0
    while fwd_queue or bwd_queue:
        steps += 1

        # Forward step
        if fwd_queue:
            node = fwd_queue.popleft()
            for nb in grid.neighbors(*node):
                if nb not in fwd_vis:
                    fwd_from[nb] = node
                    fwd_vis.add(nb)
                    fwd_queue.append(nb)
                if nb in bwd_vis:
                    path = join(nb)
                    yield fwd_vis, bwd_vis, bwd_vis, path, \
                          f"Bidirectional: Met at {nb}  steps={steps}"
                    return

        # Backward step
        if bwd_queue:
            node = bwd_queue.popleft()
            for nb in grid.neighbors(*node):
                if nb not in bwd_vis:
                    bwd_from[nb] = node
                    bwd_vis.add(nb)
                    bwd_queue.append(nb)
                if nb in fwd_vis:
                    path = join(nb)
                    yield fwd_vis, bwd_vis, bwd_vis, path, \
                          f"Bidirectional: Met at {nb}  steps={steps}"
                    return

        yield fwd_vis, bwd_vis, bwd_vis, None, \
              f"Bidirectional: fwd={len(fwd_vis)} bwd={len(bwd_vis)}"

    yield fwd_vis, bwd_vis, bwd_vis, None, "Bidirectional: No path found!"



#  ALGORITHM REGISTRY

ALGOS = {
    pygame.K_1: ("BFS",                bfs,           None),
    pygame.K_2: ("DFS",                dfs,           None),
    pygame.K_3: ("UCS",                ucs,           None),
    pygame.K_4: (f"DLS (d={DLS_LIMIT})", dls,         DLS_LIMIT),
    pygame.K_5: ("IDDFS",              iddfs,         None),
    pygame.K_6: ("Bidirectional",      bidirectional, None),
}



#  RENDERER

class Renderer:
    def __init__(self, screen, grid):
        self.sc    = screen
        self.grid  = grid
        pygame.font.init()
        self.fsm = pygame.font.SysFont("Consolas,Courier New", 13)
        self.fmd = pygame.font.SysFont("Consolas,Courier New", 15, bold=True)
        self.flg = pygame.font.SysFont("Consolas,Courier New", 20, bold=True)
        self.fxl = pygame.font.SysFont("Consolas,Courier New", 24, bold=True)
        self.gsurf = pygame.Surface((COLS * CELL, ROWS * CELL))

    def _cell_color(self, r, c, frontier, explored, path_set, bwd_vis):
        v = self.grid.cells[r][c]
        p = (r, c)
        if v == WALL:   return PAL["wall"]
        if v == START:  return PAL["start"]
        if v == TARGET: return PAL["target"]
        if path_set and p in path_set: return PAL["path"]
        if bwd_vis  and p in bwd_vis:  return PAL["bwd"]
        if p in frontier:              return PAL["frontier"]
        if p in explored:              return PAL["explored"]
        return PAL["empty"]

    def draw_grid(self, frontier, explored, path, bwd_vis):
        self.gsurf.fill(PAL["bg"])
        path_set = set(path) if path else None
        for r in range(self.grid.rows):
            for c in range(self.grid.cols):
                col  = self._cell_color(r, c, frontier, explored, path_set, bwd_vis)
                rect = pygame.Rect(c * CELL, r * CELL, CELL - 1, CELL - 1)
                pygame.draw.rect(self.gsurf, col, rect, border_radius=3)
                if path_set and (r, c) in path_set:
                    pygame.draw.rect(self.gsurf, (255, 160, 160),
                                     rect, 1, border_radius=3)
        self.sc.blit(self.gsurf, (0, 0))

    def draw_panel(self, algo, status, steps, n_front,
                   n_exp, path_len, speed, paused):
        px = COLS * CELL
        panel = pygame.Surface((PANEL_W, ROWS * CELL))
        panel.fill(PAL["panel"])
        pygame.draw.rect(panel, PAL["border"], panel.get_rect(), 2)
        self.sc.blit(panel, (px, 0))

        x, y = px + 12, 12

        # App name
        self.sc.blit(self.fmd.render("AI PATH FINDER", True, PAL["txt_hi"]), (x, y))
        y += 20
        y += 8

        # Active algorithm name
        self.sc.blit(self.flg.render(
            algo or "── press 1-6 ──", True,
            PAL["btn_on"] if algo else PAL["txt_dim"]), (x, y))
        y += 30

        # Status message (word-wrapped, colour-coded)
        col = (PAL["start"]  if "Found"  in status else
               PAL["target"] if any(x in status for x in
                                    ("No path", "Cutoff", "Max")) else
               PAL["txt"])
        for line in self._wrap(status, 28):
            self.sc.blit(self.fsm.render(line, True, col), (x, y))
            y += 16
        y += 6

        self._hline(px + 8, y, PANEL_W - 16); y += 8

        # Statistics
        for label, val in [
            ("Steps",    str(steps)),
            ("Frontier", str(n_front)),
            ("Explored", str(n_exp)),
            ("Path Len", str(path_len) if path_len else "─"),
            ("Delay ms", str(speed)),
            ("State",    "PAUSED" if paused else "RUNNING"),
        ]:
            self.sc.blit(self.fsm.render(f"{label:<10}", True, PAL["txt_dim"]), (x, y))
            self.sc.blit(self.fmd.render(val, True, PAL["txt"]), (x + 112, y))
            y += 21
        y += 6

        self._hline(px + 8, y, PANEL_W - 16); y += 10

        # Legend
        self.sc.blit(self.fsm.render("LEGEND", True, PAL["txt_hi"]), (x, y))
        y += 18
        for col, label in [
            (PAL["start"],    "Start  (S)"),
            (PAL["target"],   "Target (T)"),
            (PAL["wall"],     "Static Wall"),
            (PAL["frontier"], "Frontier — in queue"),
            (PAL["explored"], "Explored — visited"),
            (PAL["path"],     "Final Path"),
            (PAL["bwd"],      "Bidir back-wave"),
        ]:
            pygame.draw.rect(self.sc, col, (x, y + 2, 13, 13), border_radius=2)
            self.sc.blit(self.fsm.render(label, True, PAL["txt"]), (x + 18, y))
            y += 18
        y += 8

        self._hline(px + 8, y, PANEL_W - 16); y += 10

        # Controls
        self.sc.blit(self.fsm.render("CONTROLS", True, PAL["txt_hi"]), (x, y))
        y += 18
        for hint in ["[1-6] Select Algorithm",
                     "[R]   Reset Grid",
                     "[SPC] Pause/Resume",
                     "[+/-] Change Speed",
                     "[L-Click] Draw/Erase Walls",
                     "[R-Click+Drag] Move S/T"]:
            self.sc.blit(self.fsm.render(hint, True, PAL["txt_dim"]), (x, y))
            y += 16

    def draw_botbar(self, active_algo):
        by  = ROWS * CELL
        bar = pygame.Surface((WIN_W, BOT_H))
        bar.fill(PAL["barb"])
        pygame.draw.rect(bar, PAL["border"], bar.get_rect(), 1)
        self.sc.blit(bar, (0, by))

        bw = (COLS * CELL) // 6
        for i, (key, (name, _, _)) in enumerate(ALGOS.items()):
            bx  = i * bw + 3
            bby = by + 8
            on  = (name == active_algo)
            pygame.draw.rect(self.sc, PAL["btn_on"] if on else PAL["btn_off"],
                             (bx, bby, bw - 6, BOT_H - 16), border_radius=7)
            pygame.draw.rect(self.sc, PAL["border"],
                             (bx, bby, bw - 6, BOT_H - 16), 1, border_radius=7)
            self.sc.blit(self.fsm.render(f"[{i+1}]", True, PAL["txt_hi"]),
                         (bx + 6, bby + 7))
            self.sc.blit(self.fsm.render(name, True,
                         PAL["txt"] if on else PAL["txt_dim"]),
                         (bx + 6, bby + 26))

       
    def _hline(self, x, y, w):
        pygame.draw.rect(self.sc, PAL["border"], (x, y, w, 1))

    @staticmethod
    def _wrap(text, width):
        words, lines, cur = text.split(), [], ""
        for w in words:
            if len(cur) + len(w) + 1 <= width:
                cur = (cur + " " + w) if cur else w
            else:
                if cur:
                    lines.append(cur)
                cur = w
        if cur:
            lines.append(cur)
        return lines or [""]
    
def animated_welcome(screen, grid, flashes=1000, delay=10):
    
        rows = WIN_H // CELL
        cols = WIN_W // CELL
        # Fonts
        font_large = pygame.font.SysFont("Audiowide", 80, bold=True)
        font_small = pygame.font.SysFont("Audiowide", 50)

        clock = pygame.time.Clock()

        for _ in range(flashes):
            # Randomly fill the grid with colors for animation
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        return   # instantly go to game
            for r in range(rows):
                for c in range(cols):
                    choice = random.choice([
                        #PAL["empty"],
                        #PAL["wall"],
                        PAL["start"],
                        PAL["target"]
                    ])
                    
                    rect = pygame.Rect(c*CELL, r*CELL, CELL-1, CELL-1)
                    pygame.draw.rect(screen, choice, rect, border_radius=3)
                    

            # Overlay welcome title and instructions
            title_surf = font_large.render("AI PATH FINDER APP", True, PAL["black"])
            instr_surf = font_small.render("Press SPACE to Start", True, PAL["yellow"])

            screen.blit(title_surf, ((WIN_W - title_surf.get_width()) // 2, WIN_H // 3))
            screen.blit(instr_surf, ((WIN_W - instr_surf.get_width()) // 2, WIN_H // 2))


            pygame.display.flip()
            pygame.time.delay(delay)
            clock.tick(60)



#  MAIN LOOP

def main():
    pygame.init()
    
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    clock  = pygame.time.Clock()

    grid     = Grid()
    grid.reset()
    animated_welcome(screen,grid)
    renderer = Renderer(screen, grid)
    

    # App state
    algo_name  = ""
    gen        = None
    frontier   = set()
    explored   = set()
    bwd_vis    = None
    path       = None
    status     = "Press [1-6] to select an algorithm"
    steps      = 0
    speed      = DEF_SPEED
    paused     = False
    finished   = False
    last_t     = 0
    drawing    = False
    draw_mode  = WALL
    moving_node = None   # "start" or "target" while right-click dragging

    def launch(key):
        nonlocal algo_name, gen, frontier, explored, bwd_vis, path
        nonlocal status, steps, finished, paused
        grid.clear_overlay()
        name, fn, lim = ALGOS[key]
        algo_name = name
        gen       = fn(grid, lim)
        frontier  = set()
        explored  = set()
        bwd_vis   = None
        path      = None
        steps     = 0
        finished  = False
        paused    = False
        status    = f"Running {name}..."

    def reset_search():
        nonlocal gen, algo_name, frontier, explored, bwd_vis, path, steps, finished
        gen = None; algo_name = ""
        frontier = set(); explored = set()
        bwd_vis = None; path = None
        steps = 0; finished = False

    running = True
    while running:
        now = pygame.time.get_ticks()

        # ── Events ───────────────────────────────────────────────
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False

            elif ev.type == pygame.KEYDOWN:
                if ev.key in ALGOS:
                    launch(ev.key)
                elif ev.key == pygame.K_r:
                    grid.reset()
                    reset_search()
                    status = "Grid reset — press [1-6]"
                elif ev.key == pygame.K_SPACE:
                    paused = not paused
                elif ev.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    speed = max(5, speed - 10)
                elif ev.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    speed = min(600, speed + 10)

            elif ev.type == pygame.MOUSEBUTTONDOWN:
                mx, my = ev.pos
                if mx < COLS * CELL and my < ROWS * CELL:
                    r, c = my // CELL, mx // CELL
                    cell_val = grid.cells[r][c]

                    if ev.button == 1:   # LEFT click — draw / erase walls
                        if cell_val == WALL:
                            draw_mode = EMPTY
                        elif cell_val in (START, TARGET):
                            draw_mode = None
                        else:
                            draw_mode = WALL
                        drawing = True

                    elif ev.button == 3:  # RIGHT click — pick up S or T
                        if cell_val == START:
                            moving_node = "start"
                        elif cell_val == TARGET:
                            moving_node = "target"

            elif ev.type == pygame.MOUSEBUTTONUP:
                drawing     = False
                moving_node = None

            elif ev.type == pygame.MOUSEMOTION:
                mx, my = ev.pos
                if mx < COLS * CELL and my < ROWS * CELL:
                    r, c = my // CELL, mx // CELL

                    # Right-drag: move Start or Target
                    if moving_node == "start" and (r, c) != grid.start:
                        if grid.cells[r][c] not in (WALL, TARGET):
                            sr, sc = grid.start
                            grid.cells[sr][sc] = EMPTY
                            grid.start = (r, c)
                            grid.cells[r][c] = START
                            reset_search()
                            status = "Start moved — press [1-6] to search"

                    elif moving_node == "target" and (r, c) != grid.target:
                        if grid.cells[r][c] not in (WALL, START):
                            tr, tc = grid.target
                            grid.cells[tr][tc] = EMPTY
                            grid.target = (r, c)
                            grid.cells[r][c] = TARGET
                            reset_search()
                            status = "Target moved — press [1-6] to search"

                    # Left-drag: draw / erase walls
                    elif drawing and draw_mode is not None:
                        if grid.cells[r][c] not in (START, TARGET):
                            grid.cells[r][c] = draw_mode

        # ── Algorithm step ────────────────────────────────────────
        if gen and not paused and not finished and (now - last_t) >= speed:
            last_t = now
            try:
                frontier, explored, bwd_vis, path, status = next(gen)
                steps += 1
                if path or any(k in status for k in
                               ("No path", "Found", "Cutoff", "Max depth")):
                    finished = True
            except StopIteration:
                finished = True
                if not path:
                    status = f"{algo_name}: No path found!"

        # ── Draw ─────────────────────────────────────────────────
        screen.fill(PAL["bg"])
        renderer.draw_grid(frontier, explored, path, bwd_vis)
        renderer.draw_panel(
            algo_name, status, steps,
            len(frontier), len(explored),
            len(path) if path else 0,
            speed, paused
        )
        renderer.draw_botbar(algo_name)
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()