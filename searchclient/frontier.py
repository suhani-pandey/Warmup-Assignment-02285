from abc import ABC, abstractmethod
from collections import deque
import heapq

from searchclient.heuristic import Heuristic
from searchclient.state import State


class Frontier(ABC):
    @abstractmethod
    def add(self, state: State) -> None: ...

    @abstractmethod
    def pop(self) -> State: ...

    @abstractmethod
    def is_empty(self) -> bool: ...

    @abstractmethod
    def size(self) -> int: ...

    @abstractmethod
    def contains(self, state: State) -> bool: ...

    @abstractmethod
    def get_name(self) -> str: ...


class FrontierBFS(Frontier):
    """
    Breadth-First Search Frontier using a FIFO queue.
    States are explored in order of their depth (shallowest first).
    """
    def __init__(self) -> None:
        super().__init__()
        self.queue: deque[State] = deque()
        self.set: set[State] = set()

    def add(self, state: State) -> None:
        self.queue.append(state)
        self.set.add(state)

    def pop(self) -> State:
        state = self.queue.popleft()
        self.set.remove(state)
        return state

    def is_empty(self) -> bool:
        return len(self.queue) == 0

    def size(self) -> int:
        return len(self.queue)

    def contains(self, state: State) -> bool:
        return state in self.set

    def get_name(self) -> str:
        return "breadth-first search"


class FrontierDFS(Frontier):
    """
    Depth-First Search Frontier using a LIFO stack.
    States are explored in order of their depth (deepest first).
    
    Implementation: Uses a deque as a stack - add to right, pop from right (LIFO).
    This is the main difference from BFS which pops from left (FIFO).
    """
    def __init__(self) -> None:
        super().__init__()
        self.stack: deque[State] = deque()
        self.set: set[State] = set()

    def add(self, state: State) -> None:
        # Add to the end of the deque (right side)
        self.stack.append(state)
        self.set.add(state)

    def pop(self) -> State:
        # Pop from the end of the deque (right side) - this makes it LIFO
        state = self.stack.pop()
        self.set.remove(state)
        return state

    def is_empty(self) -> bool:
        return len(self.stack) == 0

    def size(self) -> int:
        return len(self.stack)

    def contains(self, state: State) -> bool:
        return state in self.set

    def get_name(self) -> str:
        return "depth-first search"


class FrontierBestFirst(Frontier):
    """
    Best-First Search Frontier using a priority queue.
    States are explored based on their evaluation function f(n).
    
    For A*: f(n) = g(n) + h(n)
    For Greedy: f(n) = h(n)
    
    Uses Python's heapq for efficient priority queue operations.
    """
    def __init__(self, heuristic: Heuristic) -> None:
        super().__init__()
        self.heuristic = heuristic
        self.heap: list[tuple[int, int, State]] = []
        self.set: set[State] = set()
        self.counter = 0  # Tie-breaker for states with same f-value

    def add(self, state: State) -> None:
        # Calculate f-value using the heuristic
        f_value = self.heuristic.f(state)
        
        # Use counter as tie-breaker to maintain FIFO order for equal f-values
        # Heap stores tuples: (f_value, counter, state)
        heapq.heappush(self.heap, (f_value, self.counter, state))
        self.counter += 1
        self.set.add(state)

    def pop(self) -> State:
        # Pop the state with minimum f-value
        _, _, state = heapq.heappop(self.heap)
        self.set.remove(state)
        return state

    def is_empty(self) -> bool:
        return len(self.heap) == 0

    def size(self) -> int:
        return len(self.heap)

    def contains(self, state: State) -> bool:
        return state in self.set

    def get_name(self) -> str:
        return f"best-first search using {self.heuristic}"
