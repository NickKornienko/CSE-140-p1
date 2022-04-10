"""
In this file, you will implement generic search algorithms which are called by Pacman agents.
"""

from sre_constants import FAILURE
# from inspect import stack

from pacai.util import stack
from pacai.util import queue
from pacai.util import priorityQueue


def depthFirstSearch(problem):
    # node:
    # [0] = state;
    # [1] = actions from to start to state;
    # [2] = cost (unused)

    if problem.isGoal(problem.startingState()):
        return []

    frontier = stack.Stack()
    reached = []

    node = (problem.startingState(), [], 0)
    frontier.push(node)

    while not frontier.isEmpty():
        node = frontier.pop()

        if node[0] in reached:
            continue

        if problem.isGoal(node[0]):
            return node[1]

        reached.append(node[0])
        expand = problem.successorStates(node[0])

        for child in expand:
            childNode = (child[0], node[1] + [child[1]], 0)
            frontier.push(childNode)

    return FAILURE


def breadthFirstSearch(problem):
    return FAILURE


def uniformCostSearch(problem):
    """
    Search the node of least total cost first.
    """

    # *** Your Code Here ***
    raise NotImplementedError()


def aStarSearch(problem, heuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """

    # *** Your Code Here ***
    raise NotImplementedError()
