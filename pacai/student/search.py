"""
In this file, you will implement generic search algorithms which are called by Pacman agents.
"""

from sre_constants import FAILURE
import util
from inspect import stack


def expand(problem, node):
    s = node.state
    for action in problem.actions(s):
        t = problem.results(s, action)
        yield node(node.pathCost + problem.actionCost(s, action, t))


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

    print("Start: %s" % (str(problem.startingState())))
    print("Is the start a goal?: %s" %
          (problem.isGoal(problem.startingState())))
    print("Start's successors: %s" %
          (problem.successorStates(problem.startingState())))

    # *** Your Code Here ***
    raise NotImplementedError()


def breadthFirstSearch(problem):
    node = problem.startingState()

    if problem.isGoal(node):
        return node

    frontier = util.stack()
    reached = {problem.startingState()}

    while not frontier.isEmpty():
        node = util.stack.pop(frontier)
        for child in expand:
            s = child.state
            if problem.isGoal(node):
                return child
            if not s in reached:
                reached.add(s)
                frontier.push(child)

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
