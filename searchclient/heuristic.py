
from abc import ABC, abstractmethod
import sys
from collections import deque

from searchclient.state import State


class Heuristic(ABC):
    DEBUG_HEURISTIC = False

    def __init__(self, initial_state: State) -> None:
        pass

    @abstractmethod
    def h(self, state: State) -> int: ...

    @abstractmethod
    def f(self, state: State) -> int: ...

    @abstractmethod
    def __repr__(self) -> str: ...


class HeuristicGoalCount(Heuristic):
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)

    def h(self, state: State) -> int:
        unsatisfied = 0
        for row in range(len(State.goals)):
            for col in range(len(State.goals[row])):
                goal = State.goals[row][col]
                if "0" <= goal <= "9":
                    a = ord(goal) - ord("0")
                    if state.agent_rows[a] != row or state.agent_cols[a] != col:
                        unsatisfied += 1
                elif "A" <= goal <= "Z":
                    if state.boxes[row][col] != goal:
                        unsatisfied += 1
        return unsatisfied

class HeuristicAdvanced(Heuristic):
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)
        self.goal_distances = {}
        self.agent_goals = []
        self.box_goals_by_type = {}

        for r in range(len(State.goals)):
            for c in range(len(State.goals[r])):
                goal = State.goals[r][c]
                if goal == "":
                    continue
                self.goal_distances[(r, c)] = self._bfs_distances(r, c)
                if "0" <= goal <= "9":
                    self.agent_goals.append((r, c, ord(goal) - ord("0")))
                elif "A" <= goal <= "Z":
                    letter = goal.upper()
                    if letter not in self.box_goals_by_type:
                        self.box_goals_by_type[letter] = []
                    self.box_goals_by_type[letter].append((r, c))

    def _bfs_distances(self, sr, sc):
        distances = {}
        q = deque([(sr, sc, 0)])
        visited = {(sr, sc)}
        while q:
            r, c, d = q.popleft()
            distances[(r, c)] = d
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < len(State.walls) and 0 <= nc < len(State.walls[0]):
                    if not State.walls[nr][nc] and (nr, nc) not in visited:
                        visited.add((nr, nc))
                        q.append((nr, nc, d + 1))
        return distances

    def _get_dist(self, gr, gc, r, c):
        dists = self.goal_distances.get((gr, gc), {})
        return dists.get((r, c), abs(r - gr) + abs(c - gc))

    def h(self, state: State) -> int:
        total = 0
        max_agent = 0
        for gr, gc, a in self.agent_goals:
            d = self._get_dist(gr, gc, state.agent_rows[a], state.agent_cols[a])
            max_agent = max(max_agent, d)
        total += max_agent

        for letter, goals in self.box_goals_by_type.items():
            boxes = []
            for r in range(len(state.boxes)):
                for c in range(len(state.boxes[r])):
                    if state.boxes[r][c].upper() == letter:
                        boxes.append((r, c))
            remaining_goals = list(range(len(goals)))
            remaining_boxes = list(range(len(boxes)))
            while remaining_goals and remaining_boxes:
                best_d, best_gi, best_bi = float('inf'), -1, -1
                for gi in remaining_goals:
                    gr, gc = goals[gi]
                    for bi in remaining_boxes:
                        br, bc = boxes[bi]
                        d = self._get_dist(gr, gc, br, bc)
                        if d < best_d:
                            best_d, best_gi, best_bi = d, gi, bi
                if best_gi >= 0:
                    total += best_d
                    remaining_goals.remove(best_gi)
                    remaining_boxes.remove(best_bi)
                else:
                    break
            total += len(remaining_goals) * 100
        return total

class HeuristicFullDomain(Heuristic):

    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)

        self.goal_distances = {}
        self.agent_goals = []
        self.box_goals_by_type = {}

        for r in range(len(State.goals)):
            for c in range(len(State.goals[r])):
                goal = State.goals[r][c]
                if goal == "":
                    continue
                self.goal_distances[(r, c)] = self._bfs_distances(r, c)
                if "0" <= goal <= "9":
                    self.agent_goals.append((r, c, ord(goal) - ord("0")))
                elif "A" <= goal <= "Z":
                    letter = goal.upper()
                    if letter not in self.box_goals_by_type:
                        self.box_goals_by_type[letter] = []
                    self.box_goals_by_type[letter].append((r, c))

        self.num_agents = len(initial_state.agent_rows)
        self.agent_can_push = {}
        for a in range(self.num_agents):
            agent_color = State.agent_colors[a]
            pushable = set()
            for box_idx in range(26):
                if State.box_colors[box_idx] == agent_color:
                    pushable.add(chr(ord('A') + box_idx))
            self.agent_can_push[a] = pushable

        self._cell_dist_cache = {}

        self.corner_cells = set()
        self.goal_positions = set()
        for r in range(len(State.goals)):
            for c in range(len(State.goals[r])):
                if State.goals[r][c] != "" and "A" <= State.goals[r][c] <= "Z":
                    self.goal_positions.add((r, c))

        for r in range(len(State.walls)):
            for c in range(len(State.walls[0])):
                if State.walls[r][c]:
                    continue
                if (r, c) in self.goal_positions:
                    continue  
                n = r - 1 < 0 or State.walls[r - 1][c]
                s = r + 1 >= len(State.walls) or State.walls[r + 1][c]
                e = c + 1 >= len(State.walls[0]) or State.walls[r][c + 1]
                w = c - 1 < 0 or State.walls[r][c - 1]
                if (n and e) or (n and w) or (s and e) or (s and w):
                    self.corner_cells.add((r, c))

    def _bfs_distances(self, sr, sc):
        distances = {}
        q = deque([(sr, sc, 0)])
        visited = {(sr, sc)}
        while q:
            r, c, d = q.popleft()
            distances[(r, c)] = d
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < len(State.walls) and 0 <= nc < len(State.walls[0]):
                    if not State.walls[nr][nc] and (nr, nc) not in visited:
                        visited.add((nr, nc))
                        q.append((nr, nc, d + 1))
        return distances

    def _get_cell_distances(self, sr, sc):

        if (sr, sc) not in self._cell_dist_cache:
            self._cell_dist_cache[(sr, sc)] = self._bfs_distances(sr, sc)
        return self._cell_dist_cache[(sr, sc)]

    def _get_dist(self, gr, gc, r, c):
        dists = self.goal_distances.get((gr, gc), {})
        return dists.get((r, c), abs(r - gr) + abs(c - gc))

    def _get_bfs_dist(self, r1, c1, r2, c2):
        dists = self._get_cell_distances(r1, c1)
        return dists.get((r2, c2), abs(r1 - r2) + abs(c1 - c2))

    def h(self, state: State) -> int:
        total = 0
        max_agent = 0
        for gr, gc, a in self.agent_goals:
            d = self._get_dist(gr, gc, state.agent_rows[a], state.agent_cols[a])
            max_agent = max(max_agent, d)
        total += max_agent

        total_box_dist = 0
        total_agent_to_box = 0
        deadlock_penalty = 0

        for letter, goals in self.box_goals_by_type.items():
            boxes = []
            for r in range(len(state.boxes)):
                for c in range(len(state.boxes[r])):
                    if state.boxes[r][c].upper() == letter:
                        boxes.append((r, c))

            if not boxes:
                total_box_dist += len(goals) * 100
                continue

            for (br, bc) in boxes:
                if (br, bc) in self.corner_cells:
                    deadlock_penalty += 1000

            remaining_goals = list(range(len(goals)))
            remaining_boxes = list(range(len(boxes)))
            unsatisfied_boxes = []

            while remaining_goals and remaining_boxes:
                best_d, best_gi, best_bi = float('inf'), -1, -1
                for gi in remaining_goals:
                    gr, gc = goals[gi]
                    for bi in remaining_boxes:
                        br, bc = boxes[bi]
                        d = self._get_dist(gr, gc, br, bc)
                        if d < best_d:
                            best_d, best_gi, best_bi = d, gi, bi
                if best_gi >= 0:
                    total_box_dist += best_d
                    if best_d > 0:
                        unsatisfied_boxes.append(boxes[best_bi])
                    remaining_goals.remove(best_gi)
                    remaining_boxes.remove(best_bi)
                else:
                    break

            total_box_dist += len(remaining_goals) * 100

            for (br, bc) in unsatisfied_boxes:
                min_agent_dist = float('inf')
                for a in range(self.num_agents):
                    if letter in self.agent_can_push[a]:
                        d = self._get_bfs_dist(
                            state.agent_rows[a], state.agent_cols[a],
                            br, bc
                        )
                        d = max(0, d - 1)
                        min_agent_dist = min(min_agent_dist, d)
                if min_agent_dist < float('inf'):
                    total_agent_to_box += min_agent_dist

        total += total_box_dist + total_agent_to_box + deadlock_penalty

        if Heuristic.DEBUG_HEURISTIC:
            print(f"\n=== Full Domain Heuristic (Improved) ===", file=sys.stderr)
            print(f"Agents: {max_agent}, Boxes: {total_box_dist}, "
                  f"Agent→Box: {total_agent_to_box}, Deadlock: {deadlock_penalty}",
                  file=sys.stderr)
            print(f"h(s) = {total}", file=sys.stderr)

        return total


class HeuristicAStar(HeuristicFullDomain):
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)

    def f(self, state: State) -> int:
        return state.g + self.h(state)

    def __repr__(self) -> str:
        return "A* evaluation"


class HeuristicAStarGoalCount(HeuristicGoalCount):
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)

    def f(self, state: State) -> int:
        return state.g + self.h(state)

    def __repr__(self) -> str:
        return "A* with Goal Count evaluation"


class HeuristicWeightedAStar(HeuristicFullDomain):
    def __init__(self, initial_state: State, w: int) -> None:
        super().__init__(initial_state)
        self.w = w

    def f(self, state: State) -> int:
        return state.g + self.w * self.h(state)

    def __repr__(self) -> str:
        return f"WA*({self.w}) evaluation"


class HeuristicGreedy(HeuristicFullDomain):
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)

    def f(self, state: State) -> int:
        return self.h(state)

    def __repr__(self) -> str:
        return "greedy evaluation"


class HeuristicGreedyGoalCount(HeuristicGoalCount):
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)

    def f(self, state: State) -> int:
        return self.h(state)

    def __repr__(self) -> str:
        return "greedy with Goal Count evaluation"