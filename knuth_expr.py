import argparse
import math
from enum import Enum, auto
from queue import Queue
from typing import List, Union


class State(Enum):
    int = auto()
    float = auto()


class Node:
    def __init__(self, state: State, parent = None, action = None, expression = list):
        self.state = state
        self.parent = parent
        self.action = action
        self.expression = expression


class KnuthExpr:
    def __init__(self, initial, goal):
        self.initial = initial
        self.possible_actions = {math.sqrt, math.factorial, math.floor}
        self.goal = goal
    
    def is_goal(self, state: State):
        return state == self.goal

    def actions(self, state: State) -> set:
        assert isinstance(state, int) or isinstance(state, float)
        if state > 100: # don't want factorial to cause overflow
            return {math.sqrt, math.floor}

        if state <= 2: # no action will result in anything new
            return set()

        if isinstance(state, float):
            return {math.sqrt, math.floor} # can't use factorial on float

        if isinstance(state, int):
            return self.possible_actions

    def result(self, state: State, action) -> State:
        return action(state)


def expand_node(problem: KnuthExpr, node: Node) -> List[Node]:
    state = node.state
    children = []
    for action in problem.actions(state):
        new_state = problem.result(state, action)
        children.append(
            Node(
                 state=new_state,
                 parent=node,
                 action=action,
                 expression=node.expression + [action]
            )
        )
    return children


def BFS(problem: KnuthExpr) -> Union[Node, None]:
    n = Node(state=problem.initial, expression=[3])

    frontier = Queue()
    reached_states = set()
    frontier.put(n)
    reached_states.add(problem.initial)

    while not frontier.empty():
        node = frontier.get()
        if problem.is_goal(node.state):
            return node

        children = expand_node(problem, node)
        for child_node in children:
            state = child_node.state
            if problem.is_goal(state):
                return child_node
            if state not in reached_states:
                frontier.put(child_node)
                reached_states.add(state)
    return None # failed


def main(argv):
    assert argv.initial and argv.goal
    p = KnuthExpr(initial=argv.initial, goal=argv.goal)
    node = BFS(p)
    if node:
        print(node.state)
        print(node.expression)
    if not node:
        print("Looks like we failed")
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Uninformed search algorithm for a Knuth's Expression")
    parser.add_argument(
        "-g",
        "--goal",
        type=int,
        dest="goal",
        required=True,
        help="Goal state for the problem"
    )
    parser.add_argument(
        "-i",
        "--initial",
        type=int,
        dest="initial",
        required=True,
        help="Initial state for the problem"
    )
    ARGV = parser.parse_args()
    main(ARGV)
