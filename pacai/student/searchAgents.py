"""
This file contains incomplete versions of some agents that can be selected to control Pacman.
You will complete their implementations.

Good luck and happy searching!
"""

import logging

from pacai.core.actions import Actions
from pacai.core.directions import Directions
from pacai.core.distance import manhattan, maze
from pacai.core.search.position import PositionSearchProblem
from pacai.core.search.problem import SearchProblem
from pacai.agents.base import BaseAgent
from pacai.agents.search.base import SearchAgent
from pacai.student.search import uniformCostSearch


class CornersProblem(SearchProblem):
    """
    This search problem finds paths through all four corners of a layout.

    You must select a suitable state space and successor function.
    See the `pacai.core.search.position.PositionSearchProblem` class for an example of
    a working SearchProblem.

    Additional methods to implement:

    `pacai.core.search.problem.SearchProblem.startingState`:
    Returns the start state (in your search space,
    NOT a `pacai.core.gamestate.AbstractGameState`).

    `pacai.core.search.problem.SearchProblem.isGoal`:
    Returns whether this search state is a goal state of the problem.

    `pacai.core.search.problem.SearchProblem.successorStates`:
    Returns successor states, the actions they require, and a cost of 1.
    The following code snippet may prove useful:
    ```
        successors = []

        for action in Directions.CARDINAL:
            x, y = currentPosition
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            hitsWall = self.walls[nextx][nexty]

            if (not hitsWall):
                # Construct the successor.

        return successors
    ```
    """

    def __init__(self, startingGameState):
        """
        CornersProblem constructor 
        """
        super().__init__()

        self.walls = startingGameState.getWalls()
        self.startingPosition = startingGameState.getPacmanPosition()
        top = self.walls.getHeight() - 2
        right = self.walls.getWidth() - 2

        self.corners = ((1, 1), (1, top), (right, 1), (right, top))
        for corner in self.corners:
            if not startingGameState.hasFood(*corner):
                logging.warning('Warning: no food in corner ' + str(corner))

        # 0 = bottom left
        # 1 = top left
        # 2 = bottom right
        # 3 = top right
        self.cornersReachedDict = {
            self.corners[0]: 0,
            self.corners[1]: 1,
            self.corners[2]: 2,
            self.corners[3]: 3
        }

        # list used due to dicts not supporting ordering
        cornersReached = [False, False, False, False]

        for corner in self.corners:
            if self.startingPosition == cornersReached[self.cornersReachedDict[corner]]:
                cornersReached[self.cornersReachedDict[corner]] = True

        # 0 = position
        # 1 = which corners have been reached
        self.startState = (
            self.startingPosition,
            cornersReached
        )

    def startingState(self):
        """
        return startState
        """
        return self.startState

    def isGoal(self, state):
        """
        return if state is a goal state (all 4 corners reached)
        """
        for corner in self.corners:
            if state[1][self.cornersReachedDict[corner]] is False:
                return False
        return True

    def successorStates(self, state):
        """
        return successor states from the current state
        """
        successors = []

        for action in Directions.CARDINAL:
            x, y = state[0]
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)

            # add all legal moves as successor states
            if (not self.walls[nextx][nexty]):
                nextPositon = (nextx, nexty)
                cornersReached = list(state[1])
                for corner in self.corners:
                    if nextPositon == corner:
                        cornersReached[self.cornersReachedDict[corner]] = True

                nextState = nextPositon, cornersReached

                # 0 = state
                # 1 = action
                # 2 = cost (always 1)
                # 3 = cornersReached
                successors.append((nextState, action, 1, cornersReached))

        self._numExpanded += 1  # track expanded node count for output
        return successors

    def actionsCost(self, actions):
        """
        Returns the cost of a particular sequence of actions.
        If those actions include an illegal move, return 999999.
        This is implemented for you.
        """

        if (actions is None):
            return 999999

        x, y = self.startingPosition
        for action in actions:
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]:
                return 999999

        return len(actions)


def cornersHeuristic(state, problem):
    """
    returns sum of closest and farthest node distances from the current state
    as the heuristic
    """

    # 0 = bottom left
    # 1 = top left
    # 2 = bottom right
    # 3 = top right
    cornersReachedDict = {
        problem.corners[0]: 0,
        problem.corners[1]: 1,
        problem.corners[2]: 2,
        problem.corners[3]: 3
    }

    remainingCorners = []
    for corner in problem.corners:
        if state[1][cornersReachedDict[corner]] is False:
            remainingCorners.append(corner)

    if not remainingCorners:
        return 0

    cornerDistances = []
    for corner in remainingCorners:
        cornerDistances.append(manhattan(state[0], corner))
    cornerDistances.sort()

    return cornerDistances[0] + cornerDistances[remainingCorners.__len__() - 1]


def foodHeuristic(state, problem):
    """
    returns sum of closest node distance and
    farthest node from closest node distance
    as the heuristic
    """
    position, foodGrid = state

    foodDistances = []
    for food in foodGrid.asList():
        if food:
            # mahhattan distance is used here as it is much
            # faster, but provides relatively accruate results.
            # True distance provides slightly fewer nodes expanded,
            # But nearly doubled run-time in the trickySearch layout
            d = manhattan(position, food)
            foodDistances.append((d, food))
    foodDistances.sort()

    if foodDistances.__len__() == 0:
        return 0

    closestFood = foodDistances[0][1]
    closestFoodDistance = foodDistances[0][0]

    foodDistances = []
    for food in foodGrid.asList():
        if food:
            # true distance is used here as manhatten distance is
            # often not accurate use to walls being in the way.
            # This is slightly slower since BFS is used, but much
            # more accurate which leads to significantly fewer
            # nodes expanded
            d = maze(closestFood, food, problem.startingGameState)
            foodDistances.append((d, food))
    foodDistances.sort()

    if foodDistances.__len__() == 0:
        return closestFoodDistance

    farthestFoodfromClosestDist = foodDistances[foodDistances.__len__() - 1][0]

    return closestFoodDistance + farthestFoodfromClosestDist


class ClosestDotSearchAgent(SearchAgent):
    """
    Search for all food using a sequence of searches.
    """

    def __init__(self, index, **kwargs):
        super().__init__(index, **kwargs)

    def registerInitialState(self, state):
        self._actions = []
        self._actionIndex = 0

        currentState = state

        while (currentState.getFood().count() > 0):
            nextPathSegment = self.findPathToClosestDot(
                currentState)  # The missing piece
            self._actions += nextPathSegment

            for action in nextPathSegment:
                legal = currentState.getLegalActions()
                if action not in legal:
                    raise Exception('findPathToClosestDot returned an illegal move: %s!\n%s' %
                                    (str(action), str(currentState)))

                currentState = currentState.generateSuccessor(0, action)

        logging.info('Path found with cost %d.' % len(self._actions))

    def findPathToClosestDot(self, gameState):
        """
        Use UCS to find a solution (fewest nodes out of DFS, BFS, UFS)
        """
        return uniformCostSearch(AnyFoodSearchProblem(gameState))


class AnyFoodSearchProblem(PositionSearchProblem):
    def __init__(self, gameState, start=None):
        """
        AnyFoodSearchProblem constructor
        """
        super().__init__(gameState, goal=None, start=start)

        # Store the food for later reference.
        self.food = gameState.getFood()

    def isGoal(self, state):
        """
        returns true if current state contains food
        """
        return self.food[state[0]][state[1]]


class ApproximateSearchAgent(BaseAgent):
    """
    Implement your contest entry here.

    Additional methods to implement:

    `pacai.agents.base.BaseAgent.getAction`:
    Get a `pacai.bin.pacman.PacmanGameState`
    and return a `pacai.core.directions.Directions`.

    `pacai.agents.base.BaseAgent.registerInitialState`:
    This method is called before any moves are made.
    """

    def __init__(self, index, **kwargs):
        super().__init__(index, **kwargs)
