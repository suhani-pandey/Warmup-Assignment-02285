from abc import ABC, abstractmethod
import sys
from collections import deque

from searchclient.state import State


class Heuristic(ABC):
    # Set this to True to enable debug printing of states and h(s) values
    DEBUG_HEURISTIC = False
    
    def __init__(self, initial_state: State) -> None:
        # Preprocessing: Compute actual shortest distances from each goal to all cells
        # This accounts for walls and provides much more accurate estimates
        self.goal_distances = {}
        
        # For each goal position, run BFS to find shortest path to all reachable cells
        for goal_row in range(len(State.goals)):
            for goal_col in range(len(State.goals[goal_row])):
                goal = State.goals[goal_row][goal_col]
                if goal != "":  # If there's a goal at this position
                    # Compute distances from this goal to all cells using BFS
                    self.goal_distances[(goal_row, goal_col)] = self._compute_distances(goal_row, goal_col)
    
    def _compute_distances(self, start_row: int, start_col: int) -> dict:
        """
        Use BFS to compute actual shortest path distances from (start_row, start_col)
        to all reachable cells, accounting for walls.
        """
        distances = {}
        queue = deque([(start_row, start_col, 0)])
        visited = set()
        visited.add((start_row, start_col))
        
        while queue:
            row, col, dist = queue.popleft()
            distances[(row, col)] = dist
            
            # Explore neighbors (4-directional movement)
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                new_row, new_col = row + dr, col + dc
                
                # Check bounds
                if 0 <= new_row < len(State.walls) and 0 <= new_col < len(State.walls[0]):
                    # Check if not a wall and not visited
                    if not State.walls[new_row][new_col] and (new_row, new_col) not in visited:
                        visited.add((new_row, new_col))
                        queue.append((new_row, new_col, dist + 1))
        
        return distances

    def h(self, state: State) -> int:

                # ============================================================
        # OLD HEURISTIC: Goal Count (Commented Out)
        # ============================================================
        # """
        # Goal Count Heuristic: Returns the number of goals not yet satisfied.
        # """
        # unsatisfied_goals = 0
        # 
        # # Iterate over all cells in the goal grid
        # for row in range(len(State.goals)):
        #     for col in range(len(State.goals[row])):
        #         goal = State.goals[row][col]
        #         
        #         # Check agent goals (0-9)
        #         if "0" <= goal <= "9":
        #             agent_num = ord(goal) - ord("0")
        #             if state.agent_rows[agent_num] != row or \
        #                state.agent_cols[agent_num] != col:
        #                 unsatisfied_goals += 1
        #                 
        #         # Check box goals (A-Z)
        #         elif "A" <= goal <= "Z":
        #             if state.boxes[row][col] != goal:
        #                 unsatisfied_goals += 1
        # 
        # return unsatisfied_goals
        # ============================================================
        """
        Advanced Heuristic with BFS Preprocessing:
        - Uses actual shortest paths (accounts for walls)
        - Optimized for multi-agent coordination
        
        Mathematical Definition:
        h(s) = max(agent_distances) + Σ(box_distances)
        
        where distances are BFS-computed actual shortest paths
        """
        
        # ============================================================
        # EVOLUTION OF HEURISTICS:
        # 1. Goal Count: h(s) = count of unsatisfied goals
        # 2. Manhattan: h(s) = Σ |dx| + |dy| (ignores walls)
        # 3. BFS-based (current): Uses pre-computed actual paths
        # ============================================================
        
        max_agent_distance = 0
        total_box_distance = 0
        
        # Iterate over all goal cells
        for goal_row in range(len(State.goals)):
            for goal_col in range(len(State.goals[goal_row])):
                goal = State.goals[goal_row][goal_col]
                
                if goal == "":
                    continue
                
                # Get pre-computed distances for this goal
                distances = self.goal_distances.get((goal_row, goal_col), {})
                
                # Check agent goals (0-9)
                if "0" <= goal <= "9":
                    agent_num = ord(goal) - ord("0")
                    current_row = state.agent_rows[agent_num]
                    current_col = state.agent_cols[agent_num]
                    
                    # Use actual shortest path distance
                    if (current_row, current_col) in distances:
                        distance = distances[(current_row, current_col)]
                    else:
                        # Fallback to Manhattan if unreachable
                        distance = abs(current_row - goal_row) + abs(current_col - goal_col)
                    
                    # Agents can move in parallel - use max
                    max_agent_distance = max(max_agent_distance, distance)
                        
                # Check box goals (A-Z)
                elif "A" <= goal <= "Z":
                    box_found = False
                    for box_row in range(len(state.boxes)):
                        for box_col in range(len(state.boxes[box_row])):
                            if state.boxes[box_row][box_col] == goal:
                                if (box_row, box_col) in distances:
                                    distance = distances[(box_row, box_col)]
                                else:
                                    distance = abs(box_row - goal_row) + abs(box_col - goal_col)
                                
                                total_box_distance += distance
                                box_found = True
                                break
                        if box_found:
                            break
        
        # Combine: max for agents + sum for boxes
        heuristic_value = max_agent_distance + total_box_distance
        
        # Debug printing if enabled
        if Heuristic.DEBUG_HEURISTIC:
            print(f"\n=== Heuristic Debug ===", file=sys.stderr)
            print(f"Max agent: {max_agent_distance}, Boxes: {total_box_distance}", file=sys.stderr)
            print(f"h(s) = {heuristic_value}", file=sys.stderr)
            print("=" * 23, file=sys.stderr, flush=True)
                        
        return heuristic_value

    @abstractmethod
    def f(self, state: State) -> int: ...

    @abstractmethod
    def __repr__(self) -> str: ...


class HeuristicAStar(Heuristic):
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)

    def f(self, state: State) -> int:
        return state.g + self.h(state)

    def __repr__(self) -> str:
        return "A* evaluation"


class HeuristicWeightedAStar(Heuristic):
    def __init__(self, initial_state: State, w: int) -> None:
        super().__init__(initial_state)
        self.w = w

    def f(self, state: State) -> int:
        return state.g + self.w * self.h(state)

    def __repr__(self) -> str:
        return f"WA*({self.w}) evaluation"


class HeuristicGreedy(Heuristic):
    def __init__(self, initial_state: State) -> None:
        super().__init__(initial_state)

    def f(self, state: State) -> int:
        return self.h(state)

    def __repr__(self) -> str:
        return "greedy evaluation"
    

