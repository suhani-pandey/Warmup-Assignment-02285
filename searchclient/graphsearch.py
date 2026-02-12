import sys
import time

from searchclient import memory
from searchclient.action import Action
from searchclient.frontier import Frontier
from searchclient.state import State

start_time = time.perf_counter()


def search(initial_state: State, frontier: Frontier) -> list[list[Action]] | None:
    output_fixed_solution = False

    if output_fixed_solution:
        return [
            [Action.MoveS],
            [Action.MoveE],
            [Action.MoveE],
            [Action.MoveS],
        ]

    iterations = 0

    frontier.add(initial_state)
    explored: set[State] = set()

    while True:
        iterations += 1
        if iterations % 1000 == 0:
            print_search_status(explored, frontier)

        if memory.get_usage() > memory.max_usage:
            print_search_status(explored, frontier)
            print("Maximum memory usage exceeded.", file=sys.stderr, flush=True)
            return None

        if frontier.is_empty():
            print_search_status(explored, frontier)
            return None

        state = frontier.pop()

        if state.is_goal_state():
            print_search_status(explored, frontier)
            return state.extract_plan()

        explored.add(state)

        for child in state.get_expanded_states():
            if not frontier.contains(child) and child not in explored:
                frontier.add(child)

        # Check if frontier is empty (failure)
        if frontier.is_empty():
            print_search_status(explored, frontier)
            print("No solution found.", file=sys.stderr, flush=True)
            return None

        # Pop the next state from the frontier
        current_state = frontier.pop()

        # Check if this is a goal state
        if current_state.is_goal_state():
            print_search_status(explored, frontier)
            return current_state.extract_plan()

        # Add current state to explored set
        explored.add(current_state)

        # Expand the current state and add children to frontier
        for child_state in current_state.get_expanded_states():
            # Only add if not already explored and not in frontier
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