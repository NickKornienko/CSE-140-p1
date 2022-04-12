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
    # node:
    # [0] = state;
    # [1] = actions from to start to state;
    # [2] = cost (unused)

    if problem.isGoal(problem.startingState()):
        return []

    frontier = queue.Queue()
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


def uniformCostSearch(problem):
    # node:
    # [0] = state;
    # [1] = actions from to start to state;
    # [2] = cost

    if problem.isGoal(problem.startingState()):
        return []

    frontier = priorityQueue.PriorityQueue()
    reached = {}

    node = (problem.startingState(), [], 0)
    frontier.push(node, 0)

    while not frontier.isEmpty():
        node = frontier.pop()

        if node[0] in reached:
            if not node[2] < reached[node[0]]:
                continue

        if problem.isGoal(node[0]):
            return node[1]

        reached[node[0]] = node[2]
        expand = problem.successorStates(node[0])

        for child in expand:
            cost = node[2] + child[2]
            childNode = (child[0], node[1] + [child[1]], cost)
            frontier.push(childNode, cost)

    return FAILURE


def aStarSearch(problem, heuristic):
    # node:
    # [0] = state;
    # [1] = actions from to start to state;
    # [2] = actual cost
    # [3] = actual cost + next heuristic
    if problem.isGoal(problem.startingState()):
        return []

    frontier = priorityQueue.PriorityQueue()
    reached = {}

    node = (problem.startingState(), [], 0, 0)
    frontier.push(node, 0)

    while not frontier.isEmpty():
        node = frontier.pop()

        if problem.isGoal(node[0]):
            return node[1]

        if node[0] in reached:
            if not node[3] < reached[node[0]] + heuristic(node[0], problem):
                continue

        reached[node[0]] = node[2]
        expand = problem.successorStates(node[0])

        for child in expand:
            actualCost = node[2] + child[2]
            estimatedCost = actualCost + heuristic(child[0], problem)

            childNode = (child[0], node[1] + [child[1]],
                         actualCost, estimatedCost)
            frontier.push(childNode, estimatedCost)

    return FAILURE
