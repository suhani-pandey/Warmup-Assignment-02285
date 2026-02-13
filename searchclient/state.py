import random
from typing import ClassVar
from searchclient.action import Action, ActionType
from searchclient.color import Color

class State:
    _RNG = random.Random(1)
    agent_colors: ClassVar[list[Color | None]] = []
    walls: ClassVar[list[list[bool]]] = []
    box_colors: ClassVar[list[Color | None]] = []
    goals: ClassVar[list[list[str]]] = []

    def __init__(self, agent_rows: list[int], agent_cols: list[int], boxes: list[list[str]]) -> None:
        self.agent_rows = agent_rows
        self.agent_cols = agent_cols
        self.boxes = boxes
        self.parent = None
        self.joint_action = None
        self.g = 0
        self._hash = None

    def result(self, joint_action: list[Action]) -> "State":
        new_agent_rows = self.agent_rows[:]
        new_agent_cols = self.agent_cols[:]
        new_boxes = [row[:] for row in self.boxes]

        for agent_idx, action in enumerate(joint_action):
            if action.type is ActionType.NoOp:
                continue

            row, col = new_agent_rows[agent_idx], new_agent_cols[agent_idx]
            
            if action.type is ActionType.Move:
                new_agent_rows[agent_idx] += action.agent_row_delta
                new_agent_cols[agent_idx] += action.agent_col_delta

            elif action.type is ActionType.Push:
                # Agent moves to where box currently is
                new_agent_row = row + action.agent_row_delta
                new_agent_col = col + action.agent_col_delta
                # Box moves from its current location in box delta direction
                new_box_row = new_agent_row + action.box_row_delta
                new_box_col = new_agent_col + action.box_col_delta
                
                box_char = new_boxes[new_agent_row][new_agent_col]
                new_boxes[new_agent_row][new_agent_col] = ''
                new_boxes[new_box_row][new_box_col] = box_char
                
                new_agent_rows[agent_idx] = new_agent_row
                new_agent_cols[agent_idx] = new_agent_col

            elif action.type is ActionType.Pull:
                new_agent_row = row + action.agent_row_delta
                new_agent_col = col + action.agent_col_delta
                
                # Box is at current position + box_delta
                box_row = row + action.box_row_delta
                box_col = col + action.box_col_delta
                
                # Box moves to agent's old position
                box_char = new_boxes[box_row][box_col]
                new_boxes[box_row][box_col] = ''
                new_boxes[row][col] = box_char
                
                new_agent_rows[agent_idx] = new_agent_row
                new_agent_cols[agent_idx] = new_agent_col

        child = State(new_agent_rows, new_agent_cols, new_boxes)
        child.parent = self
        child.joint_action = joint_action[:]
        child.g = self.g + 1
        return child

    def is_goal_state(self) -> bool:
        for r, row in enumerate(self.goals):
            for c, goal in enumerate(row):
                if not goal or goal == ' ': continue
                
                # Check Box Goals (Case Insensitive)
                goal_upper = goal.upper()
                if 'A' <= goal_upper <= 'Z': 
                    current_box = self.boxes[r][c]
                    if not current_box or current_box.upper() != goal_upper:
                        return False
                
                # Check Agent Goals
                elif '0' <= goal <= '9': 
                    agent_idx = ord(goal) - ord('0')
                    if self.agent_rows[agent_idx] != r or self.agent_cols[agent_idx] != c: 
                        return False
        return True

    def get_expanded_states(self) -> list["State"]:
        num_agents = len(self.agent_rows)
        possible_actions = [
            [a for a in Action if self.is_applicable(i, a)] for i in range(num_agents)
        ]

        import itertools
        expanded_states = []
        for joint_action in itertools.product(*possible_actions):
            if not self.is_conflicting(joint_action):
                expanded_states.append(self.result(list(joint_action)))

        State._RNG.shuffle(expanded_states)
        return expanded_states

    def is_applicable(self, agent_idx: int, action: Action) -> bool:
        row, col = self.agent_rows[agent_idx], self.agent_cols[agent_idx]
        agent_color = self.agent_colors[agent_idx]

        if action.type is ActionType.NoOp:
            return True
        
        elif action.type is ActionType.Move:
            dest_r, dest_c = row + action.agent_row_delta, col + action.agent_col_delta
            return self.is_free(dest_r, dest_c)

        elif action.type is ActionType.Push:
            # Check if agent can move into box position
            box_r, box_c = row + action.agent_row_delta, col + action.agent_col_delta
            if not self.is_in_bounds(box_r, box_c) or not self.boxes[box_r][box_c]: return False
            if self.box_colors[ord(self.boxes[box_r][box_c].upper()) - ord('A')] != agent_color: return False
            
            # Check if box can be pushed to new position
            dest_box_r, dest_box_c = box_r + action.box_row_delta, box_c + action.box_col_delta
            return self.is_free(dest_box_r, dest_box_c)

        elif action.type is ActionType.Pull:
            # Check if agent can move to destination
            dest_r, dest_c = row + action.agent_row_delta, col + action.agent_col_delta
            if not self.is_free(dest_r, dest_c): return False
            
            # Check if there is a box to pull at the correct offset
            box_r, box_c = row + action.box_row_delta, col + action.box_col_delta
            if not self.is_in_bounds(box_r, box_c) or not self.boxes[box_r][box_c]: return False
            if self.box_colors[ord(self.boxes[box_r][box_c].upper()) - ord('A')] != agent_color: return False
            return True

        return False

    def is_conflicting(self, joint_action: list[Action]) -> bool:
        agent_dests = []
        box_dests = []
        
        for i, action in enumerate(joint_action):
            r, c = self.agent_rows[i], self.agent_cols[i]
            if action.type == ActionType.Move:
                agent_dests.append((r + action.agent_row_delta, c + action.agent_col_delta))
                box_dests.append(None)
            elif action.type == ActionType.Push:
                ar, ac = r + action.agent_row_delta, c + action.agent_col_delta
                agent_dests.append((ar, ac))
                box_dests.append((ar + action.box_row_delta, ac + action.box_col_delta))
            elif action.type == ActionType.Pull:
                agent_dests.append((r + action.agent_row_delta, c + action.agent_col_delta))
                box_dests.append((r, c))
            else:
                agent_dests.append((r, c))
                box_dests.append(None)

        for i in range(len(joint_action)):
            for j in range(i + 1, len(joint_action)):
                if agent_dests[i] == agent_dests[j]: return True
                if box_dests[i] and box_dests[j] and box_dests[i] == box_dests[j]: return True
                if box_dests[i] and box_dests[i] == agent_dests[j]: return True
                if box_dests[j] and box_dests[j] == agent_dests[i]: return True
        
        return False

    def is_free(self, row, col):
        return self.is_in_bounds(row, col) and not self.walls[row][col] and not self.boxes[row][col] and self.agent_at(row, col) is None

    def is_in_bounds(self, row, col):
        return 0 <= row < len(self.walls) and 0 <= col < len(self.walls[0])

    def agent_at(self, row, col):
        for i in range(len(self.agent_rows)):
            if self.agent_rows[i] == row and self.agent_cols[i] == col:
                return str(i)
        return None

    def extract_plan(self):
        plan = []
        state = self
        while state.parent:
            plan.append(state.joint_action)
            state = state.parent
        plan.reverse()
        return plan

    def __hash__(self):
        if self._hash is None:
            prime = 31
            h = 1
            h = h * prime + hash(tuple(self.agent_rows))
            h = h * prime + hash(tuple(self.agent_cols))
            h = h * prime + hash(tuple(tuple(row) for row in self.boxes))
            self._hash = h
        return self._hash

    def __eq__(self, other):
        return (self.agent_rows == other.agent_rows and 
                self.agent_cols == other.agent_cols and 
                self.boxes == other.boxes)

    def __repr__(self):
        return f"State({self.agent_rows}, {self.agent_cols})"