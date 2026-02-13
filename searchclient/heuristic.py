from abc import ABC, abstractmethod
import sys
from collections import deque

from searchclient.state import State


class Heuristic(ABC):
    # Set this to True to enable debug printing of states and h(s) values
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
    """
    Simple Goal Count Heuristic: Returns the number of goals not yet satisfied.
    """
    
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)
    
    def h(self, state: State) -> int:
        unsatisfied_goals = 0
        
        for row in range(len(State.goals)):
            for col in range(len(State.goals[row])):
                goal = State.goals[row][col]
                
                # Check agent goals (0-9)
                if "0" <= goal <= "9":
                    agent_num = ord(goal) - ord("0")
                    if state.agent_rows[agent_num] != row or \
                       state.agent_cols[agent_num] != col:
                        unsatisfied_goals += 1
                        
                # Check box goals (A-Z)
                elif "A" <= goal.upper() <= "Z":
                    # Fix: Case-insensitive matching
                    if state.boxes[row][col].upper() != goal.upper():
                        unsatisfied_goals += 1
        
        return unsatisfied_goals


class HeuristicAdvanced(Heuristic):
    """
    Advanced Heuristic with BFS Preprocessing and proper box-goal matching.
    """
    
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)
        
        # Preprocessing: BFS distances from each goal position
        self.goal_distances = {}
        
        # Collect goals by type for efficient matching
        self.agent_goals = []  # list of (row, col, agent_num)
        self.box_goals_by_type = {}  # letter -> list of (row, col)
        
        for goal_row in range(len(State.goals)):
            for goal_col in range(len(State.goals[goal_row])):
                goal = State.goals[goal_row][goal_col]
                if goal == "":
                    continue
                    
                # Compute BFS distances from this goal
                self.goal_distances[(goal_row, goal_col)] = self._compute_distances(goal_row, goal_col)
                
                if "0" <= goal <= "9":
                    self.agent_goals.append((goal_row, goal_col, ord(goal) - ord("0")))
                elif "A" <= goal.upper() <= "Z":
                    goal_upper = goal.upper()
                    if goal_upper not in self.box_goals_by_type:
                        self.box_goals_by_type[goal_upper] = []
                    self.box_goals_by_type[goal_upper].append((goal_row, goal_col))
    
    def _compute_distances(self, start_row: int, start_col: int) -> dict:
        distances = {}
        queue = deque([(start_row, start_col, 0)])
        visited = set()
        visited.add((start_row, start_col))
        
        while queue:
            row, col, dist = queue.popleft()
            distances[(row, col)] = dist
            
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                new_row, new_col = row + dr, col + dc
                if 0 <= new_row < len(State.walls) and 0 <= new_col < len(State.walls[0]):
                    if not State.walls[new_row][new_col] and (new_row, new_col) not in visited:
                        visited.add((new_row, new_col))
                        queue.append((new_row, new_col, dist + 1))
        
        return distances

    def _get_distance(self, goal_row, goal_col, obj_row, obj_col):
        distances = self.goal_distances.get((goal_row, goal_col), {})
        if (obj_row, obj_col) in distances:
            return distances[(obj_row, obj_col)]
        return abs(obj_row - goal_row) + abs(obj_col - goal_col)

    def h(self, state: State) -> int:
        total = 0
        
        # --- Agent goals: use max (agents move in parallel) ---
        max_agent_dist = 0
        for goal_row, goal_col, agent_num in self.agent_goals:
            ar = state.agent_rows[agent_num]
            ac = state.agent_cols[agent_num]
            d = self._get_distance(goal_row, goal_col, ar, ac)
            max_agent_dist = max(max_agent_dist, d)
        total += max_agent_dist
        
        # --- Box goals: greedy minimum-cost matching per box type ---
        for letter, goals in self.box_goals_by_type.items():
            # Collect all boxes of this type (Case Insensitive)
            boxes = []
            for r in range(len(state.boxes)):
                for c in range(len(state.boxes[r])):
                    # Fix: use .upper() for case insensitive matching
                    if state.boxes[r][c].upper() == letter:
                        boxes.append((r, c))
            
            if not boxes or not goals:
                total += len(goals) * 100
                continue
            
            remaining_goals = list(range(len(goals)))
            remaining_boxes = list(range(len(boxes)))
            
            while remaining_goals and remaining_boxes:
                best_dist = float('inf')
                best_gi = -1
                best_bi = -1
                
                for gi in remaining_goals:
                    gr, gc = goals[gi]
                    for bi in remaining_boxes:
                        br, bc = boxes[bi]
                        d = self._get_distance(gr, gc, br, bc)
                        if d < best_dist:
                            best_dist = d
                            best_gi = gi
                            best_bi = bi
                
                if best_gi >= 0:
                    total += best_dist
                    remaining_goals.remove(best_gi)
                    remaining_boxes.remove(best_bi)
                else:
                    break
            
            total += len(remaining_goals) * 100
        
        if Heuristic.DEBUG_HEURISTIC:
            print(f"\n=== Advanced Heuristic ===", file=sys.stderr)
            print(f"h(s) = {total}", file=sys.stderr)
                        
        return total

# ... (Evaluation strategies unchanged) ...
class HeuristicAStar(HeuristicAdvanced):
    """A* using Advanced (BFS-preprocessed) heuristic"""
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)

    def f(self, state: State) -> int:
        return state.g + self.h(state)

    def __repr__(self) -> str:
        return "A* evaluation"


class HeuristicAStarGoalCount(HeuristicGoalCount):
    """A* using Goal Count heuristic (for comparison)"""
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)

    def f(self, state: State) -> int:
        return state.g + self.h(state)

    def __repr__(self) -> str:
        return "A* with Goal Count evaluation"


class HeuristicWeightedAStar(HeuristicAdvanced):
    """WA* using Advanced heuristic"""
    def __init__(self, initial_state: State, w: int) -> None:
        super().__init__(initial_state)
        self.w = w

    def f(self, state: State) -> int:
        return state.g + self.w * self.h(state)

    def __repr__(self) -> str:
        return f"WA*({self.w}) evaluation"


class HeuristicGreedy(HeuristicAdvanced):
    """Greedy using Advanced heuristic"""
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)

    def f(self, state: State) -> int:
        return self.h(state)

    def __repr__(self) -> str:
        return "greedy evaluation"


class HeuristicGreedyGoalCount(HeuristicGoalCount):
    """Greedy using Goal Count heuristic (for comparison)"""
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)

    def f(self, state: State) -> int:
        return self.h(state)

    def __repr__(self) -> str:
        return "greedy with Goal Count evaluation"
