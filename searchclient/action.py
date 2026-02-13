from enum import Enum, unique
from typing import Literal


@unique
class ActionType(Enum):
    NoOp = 0
    Move = 1
    Push = 2
    Pull = 3


@unique
class Action(Enum):
    #   List of possible actions. Each action has the following parameters,
    #    taken in order from left to right:
    #    1. The name of the action as a string.
    #    2. Action type: NoOp, Move, Push or Pull
    #    3. agentRowDelta: the vertical displacement of the agent (-1,0,+1)
    #    4. agentColDelta: the horizontal displacement of the agent (-1,0,+1)
    #    5. boxRowDelta: the vertical displacement of the box (-1,0,+1)
    #    6. boxColDelta: the horizontal displacement of the box (-1,0,+1)
    #
    #    For Push(dir_agent, dir_box):
    #      agentDelta = direction of dir_agent (agent moves toward box)
    #      boxDelta = direction of dir_box (box is pushed in this direction)
    #
    #    For Pull(dir_agent, dir_box):
    #      agentDelta = direction of dir_agent (agent moves away from box)
    #      boxDelta = OPPOSITE of dir_box (where the box currently is relative to agent)
    
    NoOp = ("NoOp", ActionType.NoOp, 0, 0, 0, 0)

    # Move actions - agent moves in a direction
    MoveN = ("Move(N)", ActionType.Move, -1, 0, 0, 0)
    MoveS = ("Move(S)", ActionType.Move, 1, 0, 0, 0)
    MoveE = ("Move(E)", ActionType.Move, 0, 1, 0, 0)
    MoveW = ("Move(W)", ActionType.Move, 0, -1, 0, 0)

    # Push actions - Push(dir_agent, dir_box)
    # Agent moves in dir_agent (into box's cell), box moves in dir_box
    # Fix: Ensure agent delta moves INTO box, and box delta moves FROM box.
    PushNN = ("Push(N,N)", ActionType.Push, -1, 0, -1, 0)
    PushNE = ("Push(N,E)", ActionType.Push, -1, 0, 0, 1)
    PushNW = ("Push(N,W)", ActionType.Push, -1, 0, 0, -1)
    
    PushSS = ("Push(S,S)", ActionType.Push, 1, 0, 1, 0)
    PushSE = ("Push(S,E)", ActionType.Push, 1, 0, 0, 1)
    PushSW = ("Push(S,W)", ActionType.Push, 1, 0, 0, -1)
    
    PushEE = ("Push(E,E)", ActionType.Push, 0, 1, 0, 1)
    PushEN = ("Push(E,N)", ActionType.Push, 0, 1, -1, 0)
    PushES = ("Push(E,S)", ActionType.Push, 0, 1, 1, 0)
    
    PushWW = ("Push(W,W)", ActionType.Push, 0, -1, 0, -1)
    PushWN = ("Push(W,N)", ActionType.Push, 0, -1, -1, 0)
    PushWS = ("Push(W,S)", ActionType.Push, 0, -1, 1, 0)

    # Pull actions - Pull(dir_agent, dir_box)
    # Agent moves in dir_agent, box is in OPPOSITE of dir_box from agent
    PullNN = ("Pull(N,N)", ActionType.Pull, -1, 0, 1, 0)
    PullNE = ("Pull(N,E)", ActionType.Pull, -1, 0, 0, -1)
    PullNW = ("Pull(N,W)", ActionType.Pull, -1, 0, 0, 1)
    
    PullSS = ("Pull(S,S)", ActionType.Pull, 1, 0, -1, 0)
    PullSE = ("Pull(S,E)", ActionType.Pull, 1, 0, 0, -1)
    PullSW = ("Pull(S,W)", ActionType.Pull, 1, 0, 0, 1)
    
    PullEE = ("Pull(E,E)", ActionType.Pull, 0, 1, 0, -1)
    PullEN = ("Pull(E,N)", ActionType.Pull, 0, 1, 1, 0)
    PullES = ("Pull(E,S)", ActionType.Pull, 0, 1, -1, 0)
    
    PullWW = ("Pull(W,W)", ActionType.Pull, 0, -1, 0, 1)
    PullWN = ("Pull(W,N)", ActionType.Pull, 0, -1, 1, 0)
    PullWS = ("Pull(W,S)", ActionType.Pull, 0, -1, -1, 0)

    def __init__(
        self,
        name: str,
        type: ActionType,
        ard: Literal[-1, 0, 1],
        acd: Literal[-1, 0, 1],
        brd: Literal[-1, 0, 1],
        bcd: Literal[-1, 0, 1],
    ) -> None:
        self.name_ = name
        self.type = type
        self.agent_row_delta = ard
        self.agent_col_delta = acd
        self.box_row_delta = brd
        self.box_col_delta = bcd