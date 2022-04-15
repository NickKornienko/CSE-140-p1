"""
In this file, you will implement generic search algorithms which are called by Pacman agents.
"""

from pacai.util import stack
from pacai.util import queue
from pacai.util import priorityQueue


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first [p 85].

    Your search algorithm needs to return a list of actions that reaches the goal.
    Make sure to implement a graph search algorithm [Fig. 3.7].

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    ```
    print("Start: %s" % (str(problem.startingState())))
    print("Is the start a goal?: %s" % (problem.isGoal(problem.startingState())))
    print("Start's successors: %s" % (problem.successorStates(problem.startingState())))
    ```
    """

    # node:
    # [0] = state;
    # [1] = actions from start to state;
    # [2] = cost (unused)

    if problem.isGoal(problem.startingState()):
        return []

    frontier = stack.Stack()
    reached = []

    node = (problem.startingState(), [], 0)
    frontier.push(node)

    # pop off from frontier until no possible solution can be found
    while not frontier.isEmpty():
        node = frontier.pop()

        # skip explored paths
        if node[0] in reached:
            continue

        # path found
        if problem.isGoal(node[0]):
            return node[1]

        reached.append(node[0])
        expand = problem.successorStates(node[0])

        # add each successor to frontier with an additional action
        for child in expand:
            childNode = (child[0], node[1] + [child[1]], 0)
            frontier.push(childNode)

    return None


def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first. [p 81]
    Identical to DFS, except a queue is used intstead of a stack
    """

    # node:
    # [0] = state;
    # [1] = actions from start to state;
    # [2] = cost (unused)

    if problem.isGoal(problem.startingState()):
        return []

    frontier = queue.Queue()
    reached = []

    node = (problem.startingState(), [], 0)
    frontier.push(node)

    # pop off from frontier until no possible solution can be found
    while not frontier.isEmpty():
        node = frontier.pop()

        # skip explored paths
        if node[0] in reached:
            continue

        # path found
        if problem.isGoal(node[0]):
            return node[1]

        reached.append(node[0])
        expand = problem.successorStates(node[0])

        # add each successor to frontier with an additional action
        for child in expand:
            childNode = (child[0], node[1] + [child[1]], 0)
            frontier.push(childNode)

    return None


def uniformCostSearch(problem):
    """
    Search the node of least total cost first.
    """

    # node:
    # [0] = state;
    # [1] = actions from start to state;
    # [2] = cost

    if problem.isGoal(problem.startingState()):
        return []

    frontier = priorityQueue.PriorityQueue()
    reached = {}  # dictionary which has nodes as the key and cost as the value
    node = (problem.startingState(), [], 0)
    frontier.push(node, 0)

    # pop off from frontier until no possible solution can be found
    while not frontier.isEmpty():
        node = frontier.pop()

        # skip reached nodes that don't have a lower cost
        if node[0] in reached:
            if not node[2] < reached[node[0]]:
                continue

        # path found
        if problem.isGoal(node[0]):
            return node[1]

        reached[node[0]] = node[2]
        expand = problem.successorStates(node[0])

        # add each successor to frontier with an additional action
        for child in expand:
            cost = node[2] + child[2]
            childNode = (child[0], node[1] + [child[1]], cost)
            frontier.push(childNode, cost)

    return None


def aStarSearch(problem, heuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """

    # node:
    # [0] = state;
    # [1] = actions from start to state;
    # [2] = actual cost
    # [3] = actual cost + next heuristic

    if problem.isGoal(problem.startingState()):
        return []

    frontier = priorityQueue.PriorityQueue()
    reachedList = []  # list used instead of dict due to hashing issues

    node = (problem.startingState(), [], 0, 0)
    frontier.push(node, 0)

    # pop off from frontier until no possible solution can be found
    while not frontier.isEmpty():
        node = frontier.pop()
        state = node[0]

        # path found
        if problem.isGoal(node[0]):
            return node[1]

        # look for specific state in the reachedList
        # dict would better, but cannot hash complex types
        foundState = None
        for s in reachedList:
            if s[0] == state:
                foundState = s

        # skip reached nodes that don't have a lower cost
        if foundState is not None:
            if not node[3] < foundState[1] + heuristic(node[0], problem):
                continue

        reachedList.append((state, node[2]))

        expand = problem.successorStates(node[0])

        # add each successor to frontier with an additional action
        for child in expand:
            actualCost = node[2] + child[2]
            estimatedCost = actualCost + heuristic(child[0], problem)

            childNode = (child[0], node[1] + [child[1]],
                         actualCost, estimatedCost)
            frontier.push(childNode, estimatedCost)

    return None
