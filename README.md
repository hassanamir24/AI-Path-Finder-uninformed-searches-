# AI PATH FINDER
### Uninformed Search Algorithms Visualizer

A fully interactive grid-based pathfinding visualizer built in **Python + Pygame**.  
Watch 6 classic uninformed search algorithms explore a maze step-by-step in real time.

---

## 🚀 Installation & Running

### 1. Install Python 3.8+
Download from https://python.org if not already installed.

### 2. Install Pygame
```bash
pip install pygame
```

### 3. Run
```bash
python main.py
```

---

## 🎮 Controls

| Key / Input | Action |
|---|---|
| `1` | Breadth-First Search (BFS) |
| `2` | Depth-First Search (DFS) |
| `3` | Uniform-Cost Search (UCS) |
| `4` | Depth-Limited Search (DLS, depth=14) |
| `5` | Iterative Deepening DFS (IDDFS) |
| `6` | Bidirectional Search |
| `R` | Reset the grid |
| `SPACE` | Pause / Resume animation |
| `+` | Speed up |
| `-` | Slow down |
| `Left-Click + Drag` | Draw or erase walls |
| `Right-Click + Drag` | Move Start (green) or Target (red) |

---

## 🧠 Algorithms

| # | Algorithm | Data Structure | Optimal | Complete |
|---|---|---|---|---|
| 1 | BFS | FIFO Queue | ✅ Yes | ✅ Yes |
| 2 | DFS | LIFO Stack | ❌ No | ✅ Yes |
| 3 | UCS | Min-Heap | ✅ Yes | ✅ Yes |
| 4 | DLS | Stack + depth limit | ❌ No | ❌ No |
| 5 | IDDFS | Stack + iterative limit | ✅ Yes | ✅ Yes |
| 6 | Bidirectional | Two FIFO Queues | ✅ Yes | ✅ Yes |

---

## 🌐 Movement Directions (6 total)

```
1. Up              (-1,  0)
2. Right           ( 0, +1)
3. Bottom          (+1,  0)
4. Bottom-Right    (+1, +1)  ← diagonal
5. Left            ( 0, -1)
6. Top-Left        (-1, -1)  ← diagonal
```

Cardinal moves cost **1.0** · Diagonal moves cost **√2 ≈ 1.414** (UCS only)

---

## 🎨 Colour Legend

| Colour | Meaning |
|---|---|
| 🟢 Green | Start node |
| 🔴 Red | Target node |
| ⬛ Dark navy | Empty cell |
| 🟦 Slate | Wall |
| 🟡 Amber | Frontier — currently in queue/stack |
| 🔵 Blue | Explored — already visited |
| 🟠 Coral | Final path |
| 💙 Light Blue | Bidirectional backward wave |
| black |
| yellow |

---

## 📁 Files

```
├── main.py      ← Complete application
└── README.md    ← This file
```

---

## 📦 Dependencies

```
pygame >= 2.0.0
```
