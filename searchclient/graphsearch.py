import sys
import time
from searchclient import memory
from searchclient.action import Action
from searchclient.frontier import Frontier
from searchclient.state import State
start_time = time.perf_counter()
def search(initial_state: State, frontier: Frontier) -> list[list[Action]] | None:
    """
    Implements the Graph-Search algorithm from R&N Figure 3.7
    
    This implementation:
    1. Initializes the frontier with the initial state
    2. Maintains an explored set to avoid revisiting states
    3. Expands states by popping from frontier
    4. Checks if state is goal, returns plan if so
    5. Adds unexplored successors to frontier
    """
    iterations = 0
    
    # Initialize frontier with initial state
    frontier.add(initial_state)
    
    # Initialize explored set to keep track of visited states
    explored: set[State] = set()
    while True:
        iterations += 1
        
        # Print status every 1000 iterations
        if iterations % 1000 == 0:
            print_search_status(explored, frontier)

        # Check memory usage
        if memory.get_usage() > memory.max_usage:
            print_search_status(explored, frontier)
            print("Maximum memory usage exceeded.", file=sys.stderr, flush=True)
            return None

        # Check if frontier is empty (no solution exists)
        if frontier.is_empty():
            print_search_status(explored, frontier)
            print("No solution found.", file=sys.stderr, flush=True)
            return None
        
        # Choose a leaf node and remove it from the frontier
        state = frontier.pop()
        
        # Check if this state is a goal state
        if state.is_goal_state():
            print_search_status(explored, frontier)
            print(f"Solution found after expanding {len(explored)} states.", file=sys.stderr, flush=True)
            return state.extract_plan()
        
        # Add the state to the explored set
        explored.add(state)
        
        # Expand the state and add unexplored children to frontier
        for child_state in state.get_expanded_states():
            # Only add child if it hasn't been explored and isn't already in frontier
            if child_state not in explored and not frontier.contains(child_state):
                frontier.add(child_state)


def print_search_status(explored: set[State], frontier: Frontier) -> None:
    elapsed_time = time.perf_counter() - start_time
    print(
        f"#Expanded: {len(explored):8,}, #Frontier: {frontier.size():8,}, "
        f"#Generated: {len(explored) + frontier.size():8,}, Time: {elapsed_time:3.3f} s\n"
        f"[Alloc: {memory.get_usage():4.2f} MB, MaxAlloc: {memory.max_usage:4.2f} MB]",
        file=sys.stderr,
        flush=True,
    )